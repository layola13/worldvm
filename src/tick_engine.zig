//! Tick Engine - Unified Physics World Coordinator
//!
//! P0: Unified PhysicsWorld Skeleton
//! Coordinates discrete voxel physics with continuous physics subsystems
//!
//! Pipeline: pre_step -> broadphase -> narrowphase -> solve -> integrate -> events -> snapshot

const std = @import("std");
const address = @import("address.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const physics = @import("physics.zig");
const bus = @import("bus.zig");
const joint = @import("joint.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const ragdoll = @import("ragdoll.zig");
const ballistics = @import("ballistics.zig");
const destruction = @import("destruction.zig");
const rewind = @import("rewind.zig");
const terrain = @import("terrain.zig");
const material_pairing = @import("material_pairing.zig");
const contact_response = @import("contact_response.zig");
const collision_event = @import("collision_event.zig");
const sleep_response = @import("sleep_response.zig");
const physics_kernel = @import("physics_kernel.zig");
const physics_world = @import("physics_world.zig");
const crash_defense = @import("crash_defense.zig");
const query_types = @import("query_types.zig");

pub const Operator = enum(u8) { NOP = 0, FALL = 6, FLOW = 7, MOVE = 3, PUSH = 4, BREAK = 5 };
pub const MAX_BROADPHASE_PAIRS: usize = physics_kernel.MAX_BROADPHASE_PAIRS;
pub const BroadPhasePair = physics_kernel.BroadPhasePair;
pub const SleepIsland = physics_kernel.SleepIsland;

pub const Intent = struct {
    instance_idx: u8,
    op: Operator,
    dx: i8 = 0,
    dy: i8 = 0,
    dz: i8 = 0,
    priority: u8 = 128,
    target_instance: u8 = 255,
};

pub const DiscreteIntentStageResult = struct {
    applied: u16 = 0,
    intent_count: u8 = 0,
    pair_count: usize = 0,
    event_count: u8 = 0,
};

pub const SLEEP_TIME_THRESHOLD: u8 = sleep_response.SLEEP_TIME_THRESHOLD;
pub const DEFAULT_TIME_SCALE: f32 = 1.0;
pub const MAX_TRACE_EVENTS_PER_TICK: usize = 256;
pub const MAX_CONTACT_TELEMETRY_PER_TICK: usize = 128;
pub const TRACE_ASYNC_QUEUE_CAPACITY: usize = 64;
pub const TRACE_HISTORY_FRAME_CAPACITY: usize = 256;
const TRACE_VALUE_QUANTIZE_SCALE: f32 = 100.0;

pub const PhysicsTraceEventType = enum(u8) {
    collision = 1,
    sound = 2,
    particle = 3,
    deformation = 4,
    breakage = 5,
    joint_breakage = 6,
};

pub const PhysicsTraceEvent = struct {
    tick_id: u32 = 0,
    event_type: PhysicsTraceEventType = .collision,
    subject_id: u16 = 0,
    value_a: f32 = 0.0,
    value_b: f32 = 0.0,
    value_c: f32 = 0.0,
};

pub const CompressedTraceEvent = struct {
    tick_delta: u16 = 0,
    event_type: PhysicsTraceEventType = .collision,
    subject_id: u16 = 0,
    value_a_q: i16 = 0,
    value_b_q: i16 = 0,
    value_c_q: i16 = 0,
};

pub const TraceCompressionResult = struct {
    compressed: u16 = 0,
    skipped: u16 = 0,
};

pub const CompressedTraceFrame = struct {
    base_tick: u32 = 0,
    events: [MAX_TRACE_EVENTS_PER_TICK]CompressedTraceEvent = undefined,
    count: u16 = 0,
};

fn traceEventMask(event_type: PhysicsTraceEventType) u16 {
    return @as(u16, 1) << @as(u4, @intCast(@intFromEnum(event_type)));
}

fn quantizeTraceValue(value: f32) i16 {
    const rounded: i32 = @intFromFloat(@round(value * TRACE_VALUE_QUANTIZE_SCALE));
    const clamped = std.math.clamp(rounded, @as(i32, std.math.minInt(i16)), @as(i32, std.math.maxInt(i16)));
    return @intCast(clamped);
}

fn dequantizeTraceValue(value_q: i16) f32 {
    return @as(f32, @floatFromInt(value_q)) / TRACE_VALUE_QUANTIZE_SCALE;
}

pub fn compressTraceEvent(event: PhysicsTraceEvent, base_tick: u32) ?CompressedTraceEvent {
    if (event.tick_id < base_tick) return null;
    const tick_delta = event.tick_id - base_tick;
    if (tick_delta > std.math.maxInt(u16)) return null;
    return .{
        .tick_delta = @intCast(tick_delta),
        .event_type = event.event_type,
        .subject_id = event.subject_id,
        .value_a_q = quantizeTraceValue(event.value_a),
        .value_b_q = quantizeTraceValue(event.value_b),
        .value_c_q = quantizeTraceValue(event.value_c),
    };
}

pub fn decompressTraceEvent(compressed: CompressedTraceEvent, base_tick: u32) PhysicsTraceEvent {
    return .{
        .tick_id = base_tick + compressed.tick_delta,
        .event_type = compressed.event_type,
        .subject_id = compressed.subject_id,
        .value_a = dequantizeTraceValue(compressed.value_a_q),
        .value_b = dequantizeTraceValue(compressed.value_b_q),
        .value_c = dequantizeTraceValue(compressed.value_c_q),
    };
}

pub fn compressTraceEventsBatch(events: []const PhysicsTraceEvent, base_tick: u32, out: []CompressedTraceEvent) TraceCompressionResult {
    var result: TraceCompressionResult = .{};
    for (events) |event| {
        const compressed = compressTraceEvent(event, base_tick) orelse {
            result.skipped += 1;
            continue;
        };
        if (@as(usize, result.compressed) >= out.len) {
            result.skipped += 1;
            continue;
        }
        out[@as(usize, result.compressed)] = compressed;
        result.compressed += 1;
    }
    return result;
}

pub const TraceEventFilter = struct {
    enabled: bool = false,
    type_mask: u16 = std.math.maxInt(u16),
    subject_id: ?u16 = null,

    pub fn allowAll() TraceEventFilter {
        return .{};
    }

    pub fn onlyType(event_type: PhysicsTraceEventType) TraceEventFilter {
        return .{
            .enabled = true,
            .type_mask = traceEventMask(event_type),
        };
    }

    pub fn allows(self: TraceEventFilter, event_type: PhysicsTraceEventType, subject_id_in: u16) bool {
        if (!self.enabled) return true;
        if ((self.type_mask & traceEventMask(event_type)) == 0) return false;
        if (self.subject_id) |subject_id| {
            if (subject_id != subject_id_in) return false;
        }
        return true;
    }
};

pub const TraceQuery = struct {
    include_pending: bool = false,
    min_tick: ?u32 = null,
    max_tick: ?u32 = null,
    type_mask: u16 = std.math.maxInt(u16),
    subject_id: ?u16 = null,
    limit: u16 = std.math.maxInt(u16),

    pub fn allows(self: TraceQuery, event: PhysicsTraceEvent) bool {
        if (self.min_tick) |min_tick| {
            if (event.tick_id < min_tick) return false;
        }
        if (self.max_tick) |max_tick| {
            if (event.tick_id > max_tick) return false;
        }
        if ((self.type_mask & traceEventMask(event.event_type)) == 0) return false;
        if (self.subject_id) |subject_id| {
            if (event.subject_id != subject_id) return false;
        }
        return true;
    }
};

pub const TraceAsyncStats = struct {
    queued_frames: u16 = 0,
    history_frames: u16 = 0,
    dropped_frames: u32 = 0,
    flushed_frames: u32 = 0,
};

pub const TraceVisualizationLane = enum(u8) {
    collision = 0,
    sound = 1,
    particle = 2,
    deformation = 3,
    breakage = 4,
    joint_breakage = 5,
};

pub const TraceVisualizationEntry = struct {
    tick_id: u32 = 0,
    event_type: PhysicsTraceEventType = .collision,
    lane: TraceVisualizationLane = .collision,
    subject_id: u16 = 0,
    intensity: f32 = 0.0,
    value_a: f32 = 0.0,
    value_b: f32 = 0.0,
    value_c: f32 = 0.0,
};

pub const TraceBenchmarkConfig = struct {
    ticks: u16 = 64,
    events_per_tick: u16 = 64,
    flush_budget_per_tick: u8 = 1,
    query_every: u8 = 4,
    visualize_every: u8 = 8,
};

pub const TraceBenchmarkResult = struct {
    config: TraceBenchmarkConfig,
    generated_events: u32 = 0,
    compressed_events: u32 = 0,
    queued_frames: u32 = 0,
    flushed_frames: u32 = 0,
    queried_events: u32 = 0,
    visualized_events: u32 = 0,
    dropped_frames: u32 = 0,
    history_frames: u16 = 0,
    pending_frames: u16 = 0,
};

pub const TickContactTelemetry = struct {
    tick_id: u32 = 0,
    entity_id: u16 = 0,
    telemetry: query_types.ContactTelemetry = .{},
};

pub const WorldSnapshot = struct {
    tick: u32 = 0,
    instance_count: u16 = 0,
    world_hash: u64 = 0,
};

pub const WorldHash = struct {
    tick: u32 = 0,
    value: u64 = 0,
    determinism_flags: u32 = 0,
};

pub const FixedTickOutput = struct {
    trace_events: [MAX_TRACE_EVENTS_PER_TICK]PhysicsTraceEvent = undefined,
    trace_count: u16 = 0,
    compressed_trace_base_tick: u32 = 0,
    compressed_trace_events: [MAX_TRACE_EVENTS_PER_TICK]CompressedTraceEvent = undefined,
    compressed_trace_count: u16 = 0,
    contact_telemetry: [MAX_CONTACT_TELEMETRY_PER_TICK]TickContactTelemetry = undefined,
    contact_telemetry_count: u16 = 0,
    snapshot: WorldSnapshot = .{},
    hash: WorldHash = .{},
};

pub const TickEngine = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    intents: [64]Intent = undefined,
    intent_count: u8 = 0,
    max_ticks: u32 = 1000,
    stable: bool = false,
    tick_id: u32 = 0,
    world_bus: ?*bus.Bus = null,
    arousal_mod: i16 = 0,
    time_scale: f32 = DEFAULT_TIME_SCALE,

    // Joint constraints managed by TickEngine
    joints: []joint.Joint = &[_]joint.Joint{},
    joint_count: usize = 0,

    // Fixed timestep for continuous physics
    fixed_dt: f32 = 1.0 / 60.0,
    pending_collisions: collision_event.PendingCollisionQueue = .{},
    pending_sounds: collision_event.PendingSoundQueue = .{},
    pending_particles: collision_event.PendingParticleQueue = .{},
    pending_deformations: collision_event.PendingDeformationQueue = .{},
    pending_breaks: collision_event.PendingBreakQueue = .{},
    pending_joint_breaks: collision_event.PendingJointBreakQueue = .{},
    broadphase_pairs: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined,
    broadphase_pair_count: usize = 0,
    last_step_result: physics_world.StepResult = .{},
    last_tick_output: FixedTickOutput = .{},
    trace_event_filter: TraceEventFilter = .{},
    trace_async_queue: [TRACE_ASYNC_QUEUE_CAPACITY]CompressedTraceFrame = undefined,
    trace_async_queue_head: u16 = 0,
    trace_async_queue_count: u16 = 0,
    trace_history: [TRACE_HISTORY_FRAME_CAPACITY]CompressedTraceFrame = undefined,
    trace_history_head: u16 = 0,
    trace_history_count: u16 = 0,
    trace_async_dropped_frames: u32 = 0,
    trace_async_flushed_frames: u32 = 0,
    trace_async_flush_budget: u8 = 1,
};

pub fn init(engine: *TickEngine, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) void {
    engine.* = .{
        .s1024 = s1024,
        .entities = entities,
        .max_ticks = 1000,
        .stable = false,
        .tick_id = 0,
        .world_bus = null,
        .arousal_mod = 0,
        .time_scale = DEFAULT_TIME_SCALE,
    };
    physics_kernel.initCoreSubsystems();
}

pub fn makePhysicsWorldView(engine: *const TickEngine) physics_world.PhysicsWorld {
    return .{
        .s1024 = engine.s1024,
        .entities = engine.entities,
        .joints = engine.joints,
        .joint_count = @min(engine.joint_count, engine.joints.len),
        .tick = engine.tick_id,
        .world_bus = engine.world_bus,
        .pending_collisions = engine.pending_collisions,
        .pending_sounds = engine.pending_sounds,
        .pending_particles = engine.pending_particles,
        .pending_deformations = engine.pending_deformations,
        .pending_breaks = engine.pending_breaks,
        .pending_joint_breaks = engine.pending_joint_breaks,
        .broadphase_pairs = engine.broadphase_pairs,
        .broadphase_pair_count = engine.broadphase_pair_count,
    };
}

pub fn makePhysicsWorldStepConfig(engine: *const TickEngine, apply_continuous_physics: bool) physics_world.StepConfig {
    return .{
        .dt = engine.fixed_dt,
        .time_scale = engine.time_scale,
        .run_pre_motion_constraint = !apply_continuous_physics,
        .apply_continuous_physics = apply_continuous_physics,
        .finish_world_step = false,
        .authority = .tick_engine,
    };
}

pub fn stepViaPhysicsWorldCompat(engine: *TickEngine, apply_continuous_physics: bool) physics_world.StepResult {
    var world = makePhysicsWorldView(engine);
    const cfg = makePhysicsWorldStepConfig(engine, apply_continuous_physics);
    const before_instance_count = engine.s1024.instance_count;
    var before_instances: [scene32.MAX_INSTANCES]scene32.Instance = undefined;
    @memcpy(before_instances[0..before_instance_count], engine.s1024.instances[0..before_instance_count]);
    var result = physics_world.stepPhysicsConfigured(&world, cfg);

    engine.tick_id = world.tick;
    engine.s1024.global_tick = world.s1024.global_tick;
    engine.pending_collisions = world.pending_collisions;
    engine.pending_sounds = world.pending_sounds;
    engine.pending_particles = world.pending_particles;
    engine.pending_deformations = world.pending_deformations;
    engine.pending_breaks = world.pending_breaks;
    engine.pending_joint_breaks = world.pending_joint_breaks;
    engine.broadphase_pairs = world.broadphase_pairs;
    engine.broadphase_pair_count = world.broadphase_pair_count;
    result.changed = apply_continuous_physics and before_instance_count != world.s1024.instance_count;
    var instance_idx: usize = 0;
    while (apply_continuous_physics and !result.changed and instance_idx < before_instance_count) : (instance_idx += 1) {
        const before = before_instances[instance_idx];
        const after = world.s1024.instances[instance_idx];
        if (before.state == .resting and after.state != .broken) continue;
        if (before.state == .falling and before.vel_y < 0 and after.pos_y == before.pos_y and after.state != .broken) continue;
        result.changed = before.pos_x != after.pos_x or before.pos_y != after.pos_y or before.pos_z != after.pos_z or after.state == .broken;
    }
    engine.last_step_result = result;
    engine.stable = !result.changed;
    clearFixedTickOutput(engine);
    collectFixedTickTraceAndTelemetryFromPending(engine);
    physics_kernel.finishWorldStep(
        &engine.pending_collisions,
        &engine.pending_sounds,
        &engine.pending_particles,
        &engine.pending_deformations,
        &engine.pending_breaks,
        &engine.pending_joint_breaks,
        engine.world_bus,
        engine.tick_id,
        engine.s1024,
        engine.entities,
        engine.fixed_dt,
    );
    result.state_hash = worldHashForTick(engine.tick_id);
    result.determinism_flags = worldDeterminismFlagsForTick(engine.tick_id);
    engine.last_step_result = result;
    finalizeFixedTickSnapshotAndHash(engine, result);
    buildCompressedTraceOutput(engine);
    if (engine.last_tick_output.compressed_trace_count > 0) {
        enqueueCompressedTraceFrame(engine);
    }
    if (engine.trace_async_flush_budget > 0) {
        _ = flushTraceAsyncWrites(engine, @as(u16, engine.trace_async_flush_budget));
    }

    return result;
}

/// Compatibility wrapper for existing callers.
pub fn shouldSleep(inst: *scene32.Instance) bool {
    return physics_kernel.shouldSleep(inst);
}

pub fn shouldSleepInstance(inst: *scene32.Instance, entity: *const entity16.Entity16) bool {
    return physics_kernel.shouldSleepInstance(inst, entity);
}

pub fn computeSleepEnergy(inst: *const scene32.Instance, entity: *const entity16.Entity16) f32 {
    return physics_kernel.computeSleepEnergy(inst, entity);
}

pub fn computeSleepStability(inst: *const scene32.Instance, entity: *const entity16.Entity16) u16 {
    return physics_kernel.computeSleepStability(inst, entity);
}

pub fn isSleepStable(inst: *const scene32.Instance, entity: *const entity16.Entity16) bool {
    return physics_kernel.isSleepStable(inst, entity);
}

pub fn detectSleepIslands(engine: *const TickEngine, out_islands: []SleepIsland) usize {
    return physics_kernel.detectSleepIslands(
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        engine.broadphase_pairs[0..engine.broadphase_pair_count],
        out_islands,
    );
}

pub fn shouldWake(inst: *scene32.Instance, moved: bool) bool {
    return physics_kernel.shouldWake(inst, moved);
}

fn settleGroundContact(inst: *scene32.Instance) void {
    contact_response.settleGroundContact(inst);
}

fn applyBreakIntent(engine: *TickEngine, instance_idx: u8) void {
    _ = physics_kernel.applyBreakFromInstanceAndWake(engine.s1024, engine.entities, instance_idx, 0.05);
}

fn broadcastCollisionPair(engine: *TickEngine, inst: *const scene32.Instance, impact_velocity: i16, blocker_id: u8, did_break_self: bool) void {
    _ = did_break_self;
    physics_kernel.enqueueCollisionPairEvent(
        &engine.pending_collisions,
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        inst,
        impact_velocity,
        blocker_id,
    );
    physics_kernel.enqueueSoundPairEvent(
        &engine.pending_sounds,
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        inst,
        impact_velocity,
        blocker_id,
    );
    physics_kernel.enqueueParticlePairEvent(
        &engine.pending_particles,
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        inst,
        impact_velocity,
        blocker_id,
    );
    physics_kernel.enqueueDeformationPairEvent(
        &engine.pending_deformations,
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        inst,
        impact_velocity,
        blocker_id,
    );
    physics_kernel.enqueueBreakPairEvent(
        &engine.pending_breaks,
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        inst,
        impact_velocity,
        blocker_id,
    );
}

fn handleEvents(engine: *TickEngine) void {
    physics_kernel.publishPendingCollisions(&engine.pending_collisions, engine.world_bus, engine.tick_id);
    physics_kernel.publishPendingSounds(&engine.pending_sounds, engine.world_bus, engine.tick_id);
    physics_kernel.publishPendingParticles(&engine.pending_particles, engine.world_bus, engine.tick_id);
    physics_kernel.publishPendingDeformations(&engine.pending_deformations, engine.world_bus, engine.tick_id);
    physics_kernel.publishPendingBreaks(&engine.pending_breaks, engine.world_bus, engine.tick_id);
    physics_kernel.publishPendingJointBreaks(&engine.pending_joint_breaks, engine.world_bus, engine.tick_id);
}

fn broadPhaseWorld(engine: *TickEngine) void {
    engine.broadphase_pair_count = physics_kernel.collectBroadPhasePairs(
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        engine.broadphase_pairs[0..],
    );
}

fn queueBreakIntent(engine: *TickEngine, instance_idx: u8) void {
    _ = physics_kernel.appendUniqueInstanceIntent(
        engine.intents[0..],
        &engine.intent_count,
        instance_idx,
        Operator.BREAK,
        250,
    );
}

fn queueMoveIntent(
    engine: *TickEngine,
    instance_idx: u8,
    dx: i32,
    dy: i32,
    dz: i32,
    priority: i16,
) void {
    _ = physics_kernel.appendClampedTranslationIntent(
        engine.intents[0..],
        &engine.intent_count,
        instance_idx,
        Operator.MOVE,
        dx,
        dy,
        dz,
        priority,
    );
}

fn maybeQueueCollisionBreak(engine: *TickEngine, instance_idx: u8, impact_velocity: i16, blocker_id: u8) void {
    const decision = physics_kernel.evaluateImpactBreakPair(
        engine.s1024,
        engine.entities,
        instance_idx,
        impact_velocity,
        blocker_id,
    );

    if (decision.break_self) {
        queueBreakIntent(engine, instance_idx);
    }
    if (decision.break_blocker) {
        queueBreakIntent(engine, blocker_id);
    }
}

fn processLateralSweep(
    engine: *TickEngine,
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    start_x: i32,
    start_y: i32,
    start_z: i32,
    vel: i16,
    axis: physics.SweepAxis,
) void {
    if (vel == 0 or engine.intent_count >= engine.intents.len) return;

    const sweep = physics_kernel.sweepMotionAlongAxis(engine.s1024, inst, engine.entities, start_x, start_y, start_z, vel, axis);
    switch (axis) {
        .x => queueMoveIntent(engine, instance_idx, sweep.pos_x - inst.pos_x, 0, 0, @as(i16, 170) + engine.arousal_mod),
        .z => queueMoveIntent(engine, instance_idx, 0, 0, sweep.pos_z - inst.pos_z, @as(i16, 170) + engine.arousal_mod),
        else => return,
    }

    if (sweep.blocked) {
        broadcastCollisionPair(engine, inst, vel, sweep.blocker_id, false);
        maybeQueueCollisionBreak(engine, instance_idx, vel, sweep.blocker_id);
        physics_kernel.handleBlockedLateralContact(inst, entity, sweep.blocker_id, if (axis == .x) .x else .z);
    }
}

fn processUpwardSweep(engine: *TickEngine, instance_idx: u8, inst: *scene32.Instance, entity: *const entity16.Entity16) void {
    if (inst.vel_y <= 0 or engine.intent_count >= engine.intents.len) return;

    const sweep_y = physics_kernel.sweepMotionAlongAxis(
        engine.s1024,
        inst,
        engine.entities,
        inst.pos_x,
        inst.pos_y,
        inst.pos_z,
        inst.vel_y,
        .y,
    );
    queueMoveIntent(engine, instance_idx, 0, sweep_y.pos_y - inst.pos_y, 0, @as(i16, 190) + engine.arousal_mod);
    if (sweep_y.blocked) {
        broadcastCollisionPair(engine, inst, inst.vel_y, sweep_y.blocker_id, false);
        maybeQueueCollisionBreak(engine, instance_idx, inst.vel_y, sweep_y.blocker_id);
        physics_kernel.handleBlockedUpwardContact(inst, entity, sweep_y.blocker_id, inst.vel_y);
    }
}

fn processPlanarSweepsAt(
    engine: *TickEngine,
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    start_y: i32,
) void {
    processLateralSweep(engine, instance_idx, inst, entity, inst.pos_x, start_y, inst.pos_z, inst.vel_x, .x);
    processLateralSweep(engine, instance_idx, inst, entity, inst.pos_x, start_y, inst.pos_z, inst.vel_z, .z);
}

fn handleBlockedFallAtCurrentPosition(
    engine: *TickEngine,
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    impact_velocity: i16,
) void {
    inst.state = physics_kernel.handleBlockedFallContact(inst, entity, blocker_id, impact_velocity);
    broadcastCollisionPair(engine, inst, impact_velocity, blocker_id, false);
    maybeQueueCollisionBreak(engine, instance_idx, impact_velocity, blocker_id);
}

fn processNonUpwardMotion(
    engine: *TickEngine,
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
) void {
    const r = if (inst.vel_y < 0)
        physics.checkContinuousFall(engine.s1024, inst, engine.entities)
    else
        physics.checkFall(engine.s1024, inst, engine.entities);

    if (r.can_fall) {
        _ = physics_kernel.appendFallIntent(
            engine.intents[0..],
            &engine.intent_count,
            instance_idx,
            inst.pos_y,
            r.target_y,
            Operator.FALL,
            @as(i16, 200) + engine.arousal_mod,
        );

        processPlanarSweepsAt(engine, instance_idx, inst, entity, r.target_y);
    } else if (r.blocked) {
        const impact_velocity = inst.vel_y;
        handleBlockedFallAtCurrentPosition(engine, instance_idx, inst, entity, r.blocker_id, impact_velocity);
        processPlanarSweepsAt(engine, instance_idx, inst, entity, inst.pos_y);
    } else {
        processPlanarSweepsAt(engine, instance_idx, inst, entity, inst.pos_y);
        _ = physics_kernel.settleLowEnergyMotion(inst, sleep_response.SLEEP_VELOCITY_THRESHOLD);
    }
}

fn processLiquidMotion(
    engine: *TickEngine,
    instance_idx: u8,
    inst: *scene32.Instance,
) void {
    const r = physics.checkFlow(engine.s1024, inst, engine.entities);
    if (!r.flowed) return;

    _ = physics_kernel.appendFlowIntent(
        engine.intents[0..],
        &engine.intent_count,
        instance_idx,
        inst.pos_x,
        inst.pos_y,
        inst.pos_z,
        r.new_x,
        r.new_y,
        r.new_z,
        Operator.FLOW,
        @as(i16, 180) + engine.arousal_mod,
    );
}

fn processDynamicBodyMotion(
    engine: *TickEngine,
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
) void {
    if (inst.vel_y > 0) {
        processUpwardSweep(engine, instance_idx, inst, entity);
    } else {
        processNonUpwardMotion(engine, instance_idx, inst, entity);
    }
}

/// Wake up instance from sleep
pub fn wakeInstance(inst: *scene32.Instance) void {
    physics_kernel.wakeInstance(inst);
}

pub fn wakeInstanceForMotion(inst: *scene32.Instance, moved: bool) bool {
    return physics_kernel.wakeInstanceForMotion(inst, moved);
}

/// Apply force to instance (adds to velocity)
pub fn applyForce(inst: *scene32.Instance, force_x: f32, force_y: f32, force_z: f32, mass: u16) void {
    if (mass == 0) return;
    inst.vel_x = @truncate(@as(i32, inst.vel_x) + @as(i32, @intFromFloat(@round(force_x / @as(f32, @floatFromInt(mass)) * 10.0))));
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(force_y / @as(f32, @floatFromInt(mass)) * 10.0))));
    inst.vel_z = @truncate(@as(i32, inst.vel_z) + @as(i32, @intFromFloat(@round(force_z / @as(f32, @floatFromInt(mass)) * 10.0))));
    _ = wakeInstanceForMotion(inst, false);
}

/// Apply torque to instance (adds to angular velocity)
pub fn applyTorque(inst: *scene32.Instance, torque_x: f32, torque_y: f32, torque_z: f32, inertia: u16) void {
    if (inertia == 0) return;
    inst.ang_x = @truncate(@as(i16, inst.ang_x) + @as(i8, @intFromFloat(@round(torque_x / @as(f32, @floatFromInt(inertia)) * 10.0))));
    inst.ang_y = @truncate(@as(i16, inst.ang_y) + @as(i8, @intFromFloat(@round(torque_y / @as(f32, @floatFromInt(inertia)) * 10.0))));
    inst.ang_z = @truncate(@as(i16, inst.ang_z) + @as(i8, @intFromFloat(@round(torque_z / @as(f32, @floatFromInt(inertia)) * 10.0))));
    _ = wakeInstanceForMotion(inst, false);
}

/// Apply impulse (instant velocity change)
pub fn applyImpulse(inst: *scene32.Instance, impulse_x: f32, impulse_y: f32, impulse_z: f32) void {
    inst.vel_x = @truncate(@as(i32, inst.vel_x) + @as(i32, @intFromFloat(@round(impulse_x))));
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(impulse_y))));
    inst.vel_z = @truncate(@as(i32, inst.vel_z) + @as(i32, @intFromFloat(@round(impulse_z))));
    _ = wakeInstanceForMotion(inst, false);
}

/// Apply buoyancy force for fluids (upward force proportional to displaced volume)
pub fn applyBuoyancy(inst: *scene32.Instance, fluid_density: f32, mass: u16) void {
    if (mass == 0) return;
    // Buoyancy = fluid_density * volume * gravity (simplified)
    const volume: f32 = 16.0 * 16.0 * 16.0; // Full 16^3 entity volume
    const buoyancy_force = fluid_density * volume * 0.01;
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(buoyancy_force))));
    _ = wakeInstanceForMotion(inst, false);
}

/// Force field types
pub const ForceFieldType = enum(u8) {
    none = 0,
    point = 1, // Radial explosion-like force
    directional = 2, // Constant direction force (wind, gravity)
    vortex = 3, // Rotational force
};

pub const ForceField = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    radius: f32,
    strength: f32,
    field_type: ForceFieldType,
    priority: i16 = 0,
    dir_x: f32 = 0.0,
    dir_y: f32 = 1.0,
    dir_z: f32 = 0.0,
};

const ForceFieldRegionHit = struct {
    dx: f32,
    dy: f32,
    dz: f32,
    distance: f32,
};

const ExplosionRadialImpulse = struct {
    x: f32,
    y: f32,
    z: f32,
};

fn detectForceFieldRegionWithRadiusSq(px: f32, py: f32, pz: f32, cx: f32, cy: f32, cz: f32, radius_sq: f32) ?ForceFieldRegionHit {
    if (!std.math.isFinite(px) or !std.math.isFinite(py) or !std.math.isFinite(pz) or
        !std.math.isFinite(cx) or !std.math.isFinite(cy) or !std.math.isFinite(cz) or
        !std.math.isFinite(radius_sq))
    {
        return null;
    }
    if (radius_sq <= 0.0) return null;

    const dx = px - cx;
    const dy = py - cy;
    const dz = pz - cz;
    const dist_sq = dx * dx + dy * dy + dz * dz;
    if (!std.math.isFinite(dist_sq)) return null;
    if (dist_sq >= radius_sq) return null;

    return .{
        .dx = dx,
        .dy = dy,
        .dz = dz,
        .distance = @sqrt(dist_sq),
    };
}

fn detectForceFieldRegion(px: f32, py: f32, pz: f32, cx: f32, cy: f32, cz: f32, radius: f32) ?ForceFieldRegionHit {
    if (!std.math.isFinite(radius) or radius <= 0.0) return null;
    return detectForceFieldRegionWithRadiusSq(px, py, pz, cx, cy, cz, radius * radius);
}

fn wakeJointConstraintNeighborsForInstance(engine: *TickEngine, instance_idx: u8) void {
    if (engine.joint_count == 0 or engine.joints.len == 0) return;

    const instance_idx_u16: u16 = instance_idx;
    const active_joint_count = @min(engine.joint_count, engine.joints.len);
    var joint_idx: usize = 0;
    while (joint_idx < active_joint_count) : (joint_idx += 1) {
        const joint_def = &engine.joints[joint_idx];
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a != instance_idx_u16 and joint_def.entity_b != instance_idx_u16) continue;

        const other_u16: u16 = if (joint_def.entity_a == instance_idx_u16) joint_def.entity_b else joint_def.entity_a;
        if (other_u16 >= engine.s1024.instance_count) continue;

        const other_idx: usize = @intCast(other_u16);
        const other_inst = &engine.s1024.instances[other_idx];
        if (other_inst.state == .broken) continue;
        wakeInstance(other_inst);
    }
}

fn computeExplosionAttenuationWithInvRadius(distance: f32, inv_radius: f32) f32 {
    if (!std.math.isFinite(distance) or !std.math.isFinite(inv_radius)) return 0.0;
    if (inv_radius <= 0.0 or distance < 0.0) return 0.0;
    const normalized = distance * inv_radius;
    if (!std.math.isFinite(normalized) or normalized >= 1.0) return 0.0;
    const falloff = 1.0 - normalized;
    if (falloff <= 0.0) return 0.0;
    return falloff;
}

fn computeExplosionAttenuation(distance: f32, radius: f32) f32 {
    if (!std.math.isFinite(radius) or radius <= 0.0) return 0.0;
    return computeExplosionAttenuationWithInvRadius(distance, 1.0 / radius);
}

fn computePointExplosionImpulseMagnitudeWithInvRadius(distance: f32, inv_radius: f32, strength: f32, mass: u16) f32 {
    if (!std.math.isFinite(strength) or mass == 0) return 0.0;
    if (distance <= 0.1) return 0.0;
    const attenuation = computeExplosionAttenuationWithInvRadius(distance, inv_radius);
    if (attenuation <= 0.0) return 0.0;
    return strength * attenuation / @as(f32, @floatFromInt(mass));
}

fn computePointExplosionImpulseMagnitude(distance: f32, radius: f32, strength: f32, mass: u16) f32 {
    if (!std.math.isFinite(radius) or radius <= 0.0) return 0.0;
    return computePointExplosionImpulseMagnitudeWithInvRadius(distance, 1.0 / radius, strength, mass);
}

fn computeExplosionRadialImpulseWithInvRadius(dx: f32, dy: f32, dz: f32, distance: f32, inv_radius: f32, strength: f32, mass: u16) ?ExplosionRadialImpulse {
    if (!std.math.isFinite(dx) or !std.math.isFinite(dy) or !std.math.isFinite(dz)) return null;
    const impulse = computePointExplosionImpulseMagnitudeWithInvRadius(distance, inv_radius, strength, mass);
    if (impulse == 0.0) return null;
    return .{
        .x = dx / distance * impulse,
        .y = dy / distance * impulse,
        .z = dz / distance * impulse,
    };
}

fn computeExplosionRadialImpulse(dx: f32, dy: f32, dz: f32, distance: f32, radius: f32, strength: f32, mass: u16) ?ExplosionRadialImpulse {
    if (!std.math.isFinite(radius) or radius <= 0.0) return null;
    return computeExplosionRadialImpulseWithInvRadius(dx, dy, dz, distance, 1.0 / radius, strength, mass);
}

/// Apply point explosion field (radial outward impulse) and return affected instance count.
pub fn applyPointExplosionField(engine: *TickEngine, fx: f32, fy: f32, fz: f32, radius: f32, strength: f32) u32 {
    if (!std.math.isFinite(fx) or !std.math.isFinite(fy) or !std.math.isFinite(fz) or !std.math.isFinite(radius) or !std.math.isFinite(strength)) return 0;
    if (radius <= 0.0) return 0;
    const radius_sq = radius * radius;
    const inv_radius = 1.0 / radius;

    var affected_count: u32 = 0;
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue; // Skip static

        const px = @as(f32, @floatFromInt(inst.pos_x));
        const py = @as(f32, @floatFromInt(inst.pos_y));
        const pz = @as(f32, @floatFromInt(inst.pos_z));
        const region = detectForceFieldRegionWithRadiusSq(px, py, pz, fx, fy, fz, radius_sq) orelse continue;
        const radial_impulse = computeExplosionRadialImpulseWithInvRadius(region.dx, region.dy, region.dz, region.distance, inv_radius, strength, entity.physics.mass) orelse continue;
        applyImpulse(inst, radial_impulse.x, radial_impulse.y, radial_impulse.z);
        wakeJointConstraintNeighborsForInstance(engine, i);
        affected_count +|= 1;
    }

    return affected_count;
}

/// Apply directional force field within spherical range and return affected instance count.
pub fn applyDirectionalForceField(engine: *TickEngine, cx: f32, cy: f32, cz: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, strength: f32) u32 {
    if (!std.math.isFinite(cx) or !std.math.isFinite(cy) or !std.math.isFinite(cz) or
        !std.math.isFinite(radius) or !std.math.isFinite(dir_x) or !std.math.isFinite(dir_y) or
        !std.math.isFinite(dir_z) or !std.math.isFinite(strength))
    {
        return 0;
    }
    if (radius <= 0.0) return 0;
    const radius_sq = radius * radius;
    const inv_radius = 1.0 / radius;

    const dir_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (dir_len <= 0.0001) return 0;
    const ndx = dir_x / dir_len;
    const ndy = dir_y / dir_len;
    const ndz = dir_z / dir_len;

    var affected_count: u32 = 0;
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue;

        const px = @as(f32, @floatFromInt(inst.pos_x));
        const py = @as(f32, @floatFromInt(inst.pos_y));
        const pz = @as(f32, @floatFromInt(inst.pos_z));
        const region = detectForceFieldRegionWithRadiusSq(px, py, pz, cx, cy, cz, radius_sq) orelse continue;
        const falloff = computeExplosionAttenuationWithInvRadius(region.distance, inv_radius);
        if (falloff <= 0.0) continue;
        const force_mag = strength * falloff;
        applyForce(inst, ndx * force_mag, ndy * force_mag, ndz * force_mag, entity.physics.mass);
        wakeJointConstraintNeighborsForInstance(engine, i);
        affected_count +|= 1;
    }

    return affected_count;
}

/// Apply vortex force field (tangential swirl around Y axis) and return affected instance count.
pub fn applyVortexForceField(engine: *TickEngine, cx: f32, cy: f32, cz: f32, radius: f32, strength: f32) u32 {
    if (!std.math.isFinite(cx) or !std.math.isFinite(cy) or !std.math.isFinite(cz) or !std.math.isFinite(radius) or !std.math.isFinite(strength)) return 0;
    if (radius <= 0.0) return 0;
    const radius_sq = radius * radius;
    const inv_radius = 1.0 / radius;

    var affected_count: u32 = 0;
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue;

        const px = @as(f32, @floatFromInt(inst.pos_x));
        const py = @as(f32, @floatFromInt(inst.pos_y));
        const pz = @as(f32, @floatFromInt(inst.pos_z));
        const region = detectForceFieldRegionWithRadiusSq(px, py, pz, cx, cy, cz, radius_sq) orelse continue;
        if (region.distance <= 0.1) continue;
        const falloff = computeExplosionAttenuationWithInvRadius(region.distance, inv_radius);
        if (falloff <= 0.0) continue;

        const tangent_force = strength * falloff * 0.1;
        applyImpulse(inst, -region.dz * tangent_force / region.distance, 0.0, region.dx * tangent_force / region.distance);
        wakeJointConstraintNeighborsForInstance(engine, i);
        affected_count +|= 1;
    }

    return affected_count;
}

/// Apply explosion force (point impulse)
pub fn applyExplosion(engine: *TickEngine, fx: f32, fy: f32, fz: f32, radius: f32, force: f32) void {
    _ = applyPointExplosionField(engine, fx, fy, fz, radius, force);
}

/// Apply force field to all instances in range
pub fn applyForceField(engine: *TickEngine, field: ForceField) u32 {
    if (field.field_type == .point) {
        return applyPointExplosionField(engine, field.pos_x, field.pos_y, field.pos_z, field.radius, field.strength);
    }
    if (field.field_type == .directional) {
        return applyDirectionalForceField(engine, field.pos_x, field.pos_y, field.pos_z, field.radius, field.dir_x, field.dir_y, field.dir_z, field.strength);
    }
    if (field.field_type == .vortex) {
        return applyVortexForceField(engine, field.pos_x, field.pos_y, field.pos_z, field.radius, field.strength);
    }
    return 0;
}

fn findNextForceFieldPriority(fields: []const ForceField, upper_exclusive: ?i16) ?i16 {
    var found = false;
    var best_priority: i16 = 0;
    for (fields) |field| {
        if (upper_exclusive) |upper| {
            if (field.priority >= upper) continue;
        }
        if (!found or field.priority > best_priority) {
            best_priority = field.priority;
            found = true;
        }
    }
    if (!found) return null;
    return best_priority;
}

/// Apply multiple force fields by priority (high to low), stacking effects in each layer.
pub fn applyForceFields(engine: *TickEngine, fields: []const ForceField) u32 {
    var total_affected: u32 = 0;
    var upper_exclusive: ?i16 = null;
    while (findNextForceFieldPriority(fields, upper_exclusive)) |priority| {
        for (fields) |field| {
            if (field.priority != priority) continue;
            total_affected +|= applyForceField(engine, field);
        }
        upper_exclusive = priority;
    }
    return total_affected;
}

/// Solve joints for connected bodies (external solver integration point)
pub fn solveJointsForEngine(engine: *TickEngine, joints: []joint.Joint) void {
    physics_kernel.solveJointConstraints(engine.s1024, engine.entities, joints);
}

/// Apply continuous physics: gravity, damping, angular velocity, sleep check
fn applyContinuousPhysics(engine: *TickEngine) void {
    physics_kernel.applyContinuousPhysics(
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        engine.time_scale,
        SLEEP_TIME_THRESHOLD,
    );
}

fn gatherInternal(engine: *TickEngine, apply_continuous_physics: bool) void {
    if (apply_continuous_physics) {
        applyContinuousPhysics(engine);
    }
    physics_kernel.clearPendingCollisions(&engine.pending_collisions);
    physics_kernel.clearPendingSounds(&engine.pending_sounds);
    physics_kernel.clearPendingParticles(&engine.pending_particles);
    physics_kernel.clearPendingDeformations(&engine.pending_deformations);
    physics_kernel.clearPendingBreaks(&engine.pending_breaks);
    physics_kernel.clearPendingJointBreaks(&engine.pending_joint_breaks);

    engine.intent_count = 0;
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        if (engine.intent_count >= 64) break;
        const inst = &engine.s1024.instances[i];
        if (!physics_kernel.canProcessDynamicInstance(inst, engine.entities)) {
            if (physics_kernel.isStaticInstance(inst, engine.entities)) {
                physics_kernel.markStaticResting(inst);
            }
            continue;
        }
        const entity = &engine.entities[inst.entity_id];

        switch (entity.physics.material) {
            .liquid => {
                processLiquidMotion(engine, i, inst);
            },
            else => {
                processDynamicBodyMotion(engine, i, inst, entity);
            },
        }
    }
}

pub fn gather(engine: *TickEngine) void {
    gatherInternal(engine, true);
}

pub fn speculate(engine: *TickEngine) void {
    physics_kernel.sortByDescendingPriority(engine.intents[0..engine.intent_count]);
}

pub fn resolve(engine: *TickEngine) void {
    physics_kernel.invalidateBlockedTranslationIntents(
        engine.intents[0..engine.intent_count],
        engine.s1024,
        engine.s1024.instances[0..engine.s1024.instance_count],
        engine.entities,
        Operator.NOP,
        Operator.BREAK,
    );
}

pub fn commit(engine: *TickEngine) u16 {
    var applied: u16 = 0;
    var i: u8 = 0;
    while (i < engine.intent_count) : (i += 1) {
        const intent = &engine.intents[i];
        if (intent.op == .NOP) continue;
        const inst = &engine.s1024.instances[intent.instance_idx];

        switch (intent.op) {
            .FALL => {
                if (physics_kernel.applyFallDisplacement(inst, intent.dy)) {
                    applied += 1;
                }
            },
            .FLOW => {
                if (physics_kernel.applyFlowDisplacement(inst, intent.dx, intent.dy, intent.dz)) {
                    applied += 1;
                }
            },
            .MOVE, .PUSH => {
                if (physics_kernel.applyMoveDisplacement(inst, intent.dx, intent.dy, intent.dz)) {
                    applied += 1;
                }
            },
            .BREAK => {
                applyBreakIntent(engine, intent.instance_idx);
                applied += 1;
            },
            else => {},
        }
    }
    const constraint_result = physics_kernel.runPostMotionConstraintStage(
        engine.s1024,
        engine.entities,
        engine.joints[0..engine.joint_count],
        engine.broadphase_pairs[0..],
        &engine.pending_joint_breaks,
    );
    engine.broadphase_pair_count = physics_kernel.mergeObservedPairCount(
        engine.broadphase_pair_count,
        constraint_result.pair_count,
    );
    return applied;
}

pub fn runDiscreteIntentStage(engine: *TickEngine, apply_continuous_physics: bool) DiscreteIntentStageResult {
    gatherInternal(engine, apply_continuous_physics);
    speculate(engine);
    resolve(engine);
    const applied = commit(engine);
    return .{
        .applied = applied,
        .intent_count = engine.intent_count,
        .pair_count = engine.broadphase_pair_count,
        .event_count = engine.pending_collisions.count + engine.pending_sounds.count + engine.pending_particles.count + engine.pending_deformations.count + engine.pending_breaks.count + engine.pending_joint_breaks.count,
    };
}

fn buildPhysicsWorld(engine: *TickEngine) physics_world.PhysicsWorld {
    return .{
        .s1024 = engine.s1024,
        .entities = engine.entities,
        .joints = engine.joints,
        .joint_count = engine.joint_count,
        .tick = engine.tick_id,
        .world_bus = engine.world_bus,
        .pending_collisions = engine.pending_collisions,
        .pending_sounds = engine.pending_sounds,
        .pending_particles = engine.pending_particles,
        .pending_deformations = engine.pending_deformations,
        .pending_breaks = engine.pending_breaks,
        .pending_joint_breaks = engine.pending_joint_breaks,
        .broadphase_pairs = engine.broadphase_pairs,
        .broadphase_pair_count = engine.broadphase_pair_count,
    };
}

fn syncFromPhysicsWorld(engine: *TickEngine, world: *const physics_world.PhysicsWorld, changed: bool) bool {
    engine.tick_id = world.tick;
    engine.pending_collisions = world.pending_collisions;
    engine.pending_sounds = world.pending_sounds;
    engine.pending_particles = world.pending_particles;
    engine.pending_deformations = world.pending_deformations;
    engine.pending_breaks = world.pending_breaks;
    engine.pending_joint_breaks = world.pending_joint_breaks;
    engine.broadphase_pairs = world.broadphase_pairs;
    engine.broadphase_pair_count = world.broadphase_pair_count;
    engine.stable = !changed;
    return engine.stable;
}

fn worldHashForTick(tick: u32) u64 {
    const snapshot = rewind.getWorldSnapshotAtTick(tick) orelse return 0;
    return snapshot.world_hash;
}

fn worldDeterminismFlagsForTick(tick: u32) u32 {
    const snapshot = rewind.getWorldSnapshotAtTick(tick) orelse return 0;
    return rewind.computeWorldDeterminismFlags(snapshot);
}

fn clearFixedTickOutput(engine: *TickEngine) void {
    engine.last_tick_output.trace_count = 0;
    engine.last_tick_output.compressed_trace_base_tick = engine.tick_id;
    engine.last_tick_output.compressed_trace_count = 0;
    engine.last_tick_output.contact_telemetry_count = 0;
    engine.last_tick_output.snapshot = .{};
    engine.last_tick_output.hash = .{};
}

fn buildCompressedTraceOutput(engine: *TickEngine) void {
    const events = getLastTickTraceEvents(engine);
    engine.last_tick_output.compressed_trace_base_tick = engine.tick_id;
    const compression = compressTraceEventsBatch(
        events,
        engine.last_tick_output.compressed_trace_base_tick,
        engine.last_tick_output.compressed_trace_events[0..],
    );
    engine.last_tick_output.compressed_trace_count = compression.compressed;
}

fn ringTailIndex(head: u16, count: u16, capacity: usize) usize {
    return (@as(usize, head) + @as(usize, count)) % capacity;
}

fn advanceRingIndex(index: u16, capacity: usize) u16 {
    return @intCast((@as(usize, index) + 1) % capacity);
}

fn enqueueCompressedTraceFrame(engine: *TickEngine) void {
    if (engine.trace_async_queue_count >= TRACE_ASYNC_QUEUE_CAPACITY) {
        engine.trace_async_dropped_frames += 1;
        return;
    }
    const tail_idx = ringTailIndex(engine.trace_async_queue_head, engine.trace_async_queue_count, TRACE_ASYNC_QUEUE_CAPACITY);
    var frame: CompressedTraceFrame = .{
        .base_tick = engine.last_tick_output.compressed_trace_base_tick,
        .count = engine.last_tick_output.compressed_trace_count,
    };
    if (frame.count > 0) {
        @memcpy(
            frame.events[0..@as(usize, frame.count)],
            engine.last_tick_output.compressed_trace_events[0..@as(usize, frame.count)],
        );
    }
    engine.trace_async_queue[tail_idx] = frame;
    engine.trace_async_queue_count += 1;
}

fn appendQueryMatchesFromFrame(frame: *const CompressedTraceFrame, query: TraceQuery, out: []PhysicsTraceEvent, out_count: *usize) void {
    const query_limit = @min(out.len, @as(usize, query.limit));
    var event_idx: usize = 0;
    while (event_idx < @as(usize, frame.count) and out_count.* < query_limit) : (event_idx += 1) {
        const event = decompressTraceEvent(frame.events[event_idx], frame.base_tick);
        if (!query.allows(event)) continue;
        out[out_count.*] = event;
        out_count.* += 1;
    }
}

pub fn flushTraceAsyncWrites(engine: *TickEngine, max_frames: u16) u16 {
    var flushed: u16 = 0;
    while (flushed < max_frames and engine.trace_async_queue_count > 0) : (flushed += 1) {
        const src_idx = @as(usize, engine.trace_async_queue_head);
        const frame = engine.trace_async_queue[src_idx];

        if (engine.trace_history_count >= TRACE_HISTORY_FRAME_CAPACITY) {
            engine.trace_history_head = advanceRingIndex(engine.trace_history_head, TRACE_HISTORY_FRAME_CAPACITY);
            engine.trace_history_count -= 1;
        }

        const history_tail = ringTailIndex(engine.trace_history_head, engine.trace_history_count, TRACE_HISTORY_FRAME_CAPACITY);
        engine.trace_history[history_tail] = frame;
        engine.trace_history_count += 1;

        engine.trace_async_queue_head = advanceRingIndex(engine.trace_async_queue_head, TRACE_ASYNC_QUEUE_CAPACITY);
        engine.trace_async_queue_count -= 1;
    }
    engine.trace_async_flushed_frames += @as(u32, flushed);
    return flushed;
}

pub fn setTraceAsyncFlushBudget(engine: *TickEngine, frames_per_step: u8) void {
    engine.trace_async_flush_budget = frames_per_step;
}

pub fn getTraceAsyncStats(engine: *const TickEngine) TraceAsyncStats {
    return .{
        .queued_frames = engine.trace_async_queue_count,
        .history_frames = engine.trace_history_count,
        .dropped_frames = engine.trace_async_dropped_frames,
        .flushed_frames = engine.trace_async_flushed_frames,
    };
}

pub fn traceTypeMask(event_type: PhysicsTraceEventType) u16 {
    return traceEventMask(event_type);
}

fn traceVisualizationLaneForType(event_type: PhysicsTraceEventType) TraceVisualizationLane {
    return switch (event_type) {
        .collision => .collision,
        .sound => .sound,
        .particle => .particle,
        .deformation => .deformation,
        .breakage => .breakage,
        .joint_breakage => .joint_breakage,
    };
}

pub fn buildTraceVisualizationEntry(event: PhysicsTraceEvent) TraceVisualizationEntry {
    const intensity = @max(@max(@abs(event.value_a), @abs(event.value_b)), @abs(event.value_c));
    return .{
        .tick_id = event.tick_id,
        .event_type = event.event_type,
        .lane = traceVisualizationLaneForType(event.event_type),
        .subject_id = event.subject_id,
        .intensity = intensity,
        .value_a = event.value_a,
        .value_b = event.value_b,
        .value_c = event.value_c,
    };
}

fn benchmarkTraceEventTypeForIndex(index: u16) PhysicsTraceEventType {
    return switch (index % 6) {
        0 => .collision,
        1 => .sound,
        2 => .particle,
        3 => .deformation,
        4 => .breakage,
        else => .joint_breakage,
    };
}

pub fn queryTraceEvents(engine: *const TickEngine, query: TraceQuery, out: []PhysicsTraceEvent) usize {
    if (out.len == 0) return 0;
    var out_count: usize = 0;

    var history_idx: u16 = 0;
    while (history_idx < engine.trace_history_count and out_count < out.len) : (history_idx += 1) {
        const frame_idx = (@as(usize, engine.trace_history_head) + @as(usize, history_idx)) % TRACE_HISTORY_FRAME_CAPACITY;
        appendQueryMatchesFromFrame(&engine.trace_history[frame_idx], query, out, &out_count);
    }

    if (query.include_pending and out_count < out.len) {
        var pending_idx: u16 = 0;
        while (pending_idx < engine.trace_async_queue_count and out_count < out.len) : (pending_idx += 1) {
            const frame_idx = (@as(usize, engine.trace_async_queue_head) + @as(usize, pending_idx)) % TRACE_ASYNC_QUEUE_CAPACITY;
            appendQueryMatchesFromFrame(&engine.trace_async_queue[frame_idx], query, out, &out_count);
        }
    }

    return out_count;
}

pub fn exportTraceVisualization(engine: *const TickEngine, query: TraceQuery, out: []TraceVisualizationEntry) usize {
    if (out.len == 0) return 0;
    const bounded_limit = @min(@min(out.len, MAX_TRACE_EVENTS_PER_TICK), @as(usize, query.limit));
    if (bounded_limit == 0) return 0;

    var bounded_query = query;
    bounded_query.limit = @intCast(bounded_limit);

    var events: [MAX_TRACE_EVENTS_PER_TICK]PhysicsTraceEvent = undefined;
    const count = queryTraceEvents(engine, bounded_query, events[0..bounded_limit]);
    var idx: usize = 0;
    while (idx < count) : (idx += 1) {
        out[idx] = buildTraceVisualizationEntry(events[idx]);
    }
    return count;
}

pub fn runTraceBenchmark(allocator: std.mem.Allocator, config: TraceBenchmarkConfig) !TraceBenchmarkResult {
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entity = entity16.initEntity16();
    entity.physics.mass = 10;
    entity.physics.material = .solid;
    entity16.setVoxel(&entity, 0, 0, 0);
    var entities = [_]entity16.Entity16{entity};

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    setTraceAsyncFlushBudget(&engine, config.flush_budget_per_tick);

    var query_buf: [MAX_TRACE_EVENTS_PER_TICK]PhysicsTraceEvent = undefined;
    var vis_buf: [MAX_TRACE_EVENTS_PER_TICK]TraceVisualizationEntry = undefined;
    const events_per_tick = @min(config.events_per_tick, @as(u16, @intCast(MAX_TRACE_EVENTS_PER_TICK)));

    var result = TraceBenchmarkResult{ .config = config };
    var tick_idx: u16 = 0;
    while (tick_idx < config.ticks) : (tick_idx += 1) {
        engine.tick_id = @as(u32, tick_idx) + 1;
        clearFixedTickOutput(&engine);

        var event_idx: u16 = 0;
        while (event_idx < events_per_tick) : (event_idx += 1) {
            const event_type = benchmarkTraceEventTypeForIndex(event_idx + tick_idx);
            appendTraceEvent(
                &engine,
                event_type,
                @intCast(event_idx % 32),
                @as(f32, @floatFromInt(event_idx)),
                @as(f32, @floatFromInt(tick_idx)),
                @as(f32, @floatFromInt(event_idx + tick_idx)),
            );
        }
        result.generated_events += @as(u32, engine.last_tick_output.trace_count);

        buildCompressedTraceOutput(&engine);
        result.compressed_events += @as(u32, engine.last_tick_output.compressed_trace_count);

        if (engine.last_tick_output.compressed_trace_count > 0) {
            enqueueCompressedTraceFrame(&engine);
            result.queued_frames += 1;
        }

        if (engine.trace_async_flush_budget > 0) {
            result.flushed_frames += @as(u32, flushTraceAsyncWrites(&engine, @as(u16, engine.trace_async_flush_budget)));
        }

        const tick_number = tick_idx + 1;
        if (config.query_every > 0 and (tick_number % config.query_every) == 0) {
            const count = queryTraceEvents(&engine, .{
                .include_pending = true,
                .limit = @intCast(query_buf.len),
            }, query_buf[0..]);
            result.queried_events += @intCast(count);
        }
        if (config.visualize_every > 0 and (tick_number % config.visualize_every) == 0) {
            const count = exportTraceVisualization(&engine, .{
                .include_pending = true,
                .limit = @intCast(vis_buf.len),
            }, vis_buf[0..]);
            result.visualized_events += @intCast(count);
        }
    }

    result.flushed_frames += @as(u32, flushTraceAsyncWrites(&engine, std.math.maxInt(u16)));
    const stats = getTraceAsyncStats(&engine);
    result.dropped_frames = stats.dropped_frames;
    result.history_frames = stats.history_frames;
    result.pending_frames = stats.queued_frames;
    return result;
}

fn appendTraceEvent(engine: *TickEngine, event_type: PhysicsTraceEventType, subject_id: u16, value_a: f32, value_b: f32, value_c: f32) void {
    if (!engine.trace_event_filter.allows(event_type, subject_id)) return;
    if (engine.last_tick_output.trace_count >= engine.last_tick_output.trace_events.len) return;
    const idx = engine.last_tick_output.trace_count;
    engine.last_tick_output.trace_events[idx] = .{
        .tick_id = engine.tick_id,
        .event_type = event_type,
        .subject_id = subject_id,
        .value_a = value_a,
        .value_b = value_b,
        .value_c = value_c,
    };
    engine.last_tick_output.trace_count += 1;
}

fn appendContactTelemetry(engine: *TickEngine, entity_id: u16, telemetry: query_types.ContactTelemetry) void {
    if (engine.last_tick_output.contact_telemetry_count >= engine.last_tick_output.contact_telemetry.len) return;
    const idx = engine.last_tick_output.contact_telemetry_count;
    engine.last_tick_output.contact_telemetry[idx] = .{
        .tick_id = engine.tick_id,
        .entity_id = entity_id,
        .telemetry = telemetry,
    };
    engine.last_tick_output.contact_telemetry_count += 1;
}

fn makeTelemetryFromCollisionPayload(payload: bus.CollisionPayload) query_types.ContactTelemetry {
    return .{
        .damage_modifier = if (payload.did_break) 1.0 else 0.0,
        .penetration_resistance = @as(f32, @floatFromInt(payload.hardness)),
    };
}

fn collectFixedTickTraceAndTelemetryFromPending(engine: *TickEngine) void {
    var collision_idx: u8 = 0;
    while (collision_idx < engine.pending_collisions.count) : (collision_idx += 1) {
        const entity_id = engine.pending_collisions.entity_ids[collision_idx];
        const payload = engine.pending_collisions.events[collision_idx];
        appendTraceEvent(
            engine,
            .collision,
            entity_id,
            @as(f32, @floatFromInt(payload.impact_velocity)),
            @as(f32, @floatFromInt(payload.hardness)),
            if (payload.did_break) 1.0 else 0.0,
        );
        appendContactTelemetry(engine, entity_id, makeTelemetryFromCollisionPayload(payload));
    }

    var sound_idx: u8 = 0;
    while (sound_idx < engine.pending_sounds.count) : (sound_idx += 1) {
        const entity_id = engine.pending_sounds.entity_ids[sound_idx];
        const payload = engine.pending_sounds.events[sound_idx];
        appendTraceEvent(engine, .sound, entity_id, payload.volume, payload.pitch, payload.duration);
    }

    var particle_idx: u8 = 0;
    while (particle_idx < engine.pending_particles.count) : (particle_idx += 1) {
        const entity_id = engine.pending_particles.entity_ids[particle_idx];
        const payload = engine.pending_particles.events[particle_idx];
        appendTraceEvent(engine, .particle, entity_id, payload.intensity, payload.radius, payload.duration);
    }

    var deformation_idx: u8 = 0;
    while (deformation_idx < engine.pending_deformations.count) : (deformation_idx += 1) {
        const entity_id = engine.pending_deformations.entity_ids[deformation_idx];
        const payload = engine.pending_deformations.events[deformation_idx];
        appendTraceEvent(engine, .deformation, entity_id, payload.total_depth, payload.permanent_depth, payload.recovery_fraction);
    }

    var break_idx: u8 = 0;
    while (break_idx < engine.pending_breaks.count) : (break_idx += 1) {
        const entity_id = engine.pending_breaks.entity_ids[break_idx];
        const payload = engine.pending_breaks.events[break_idx];
        appendTraceEvent(
            engine,
            .breakage,
            entity_id,
            @as(f32, @floatFromInt(payload.impact_velocity)),
            @as(f32, @floatFromInt(payload.hardness)),
            @as(f32, @floatFromInt(payload.fragment_count)),
        );
    }

    var joint_break_idx: u8 = 0;
    while (joint_break_idx < engine.pending_joint_breaks.count) : (joint_break_idx += 1) {
        const joint_id = engine.pending_joint_breaks.joint_ids[joint_break_idx];
        const payload = engine.pending_joint_breaks.events[joint_break_idx];
        appendTraceEvent(engine, .joint_breakage, joint_id, payload.break_ratio, @as(f32, @floatFromInt(payload.entity_a)), @as(f32, @floatFromInt(payload.entity_b)));
    }
}

fn finalizeFixedTickSnapshotAndHash(engine: *TickEngine, result: physics_world.StepResult) void {
    if (rewind.getWorldSnapshotAtTick(result.snapshot_tick)) |snapshot| {
        engine.last_tick_output.snapshot = .{
            .tick = snapshot.tick,
            .instance_count = snapshot.instance_count,
            .world_hash = snapshot.world_hash,
        };
    } else {
        engine.last_tick_output.snapshot = .{
            .tick = result.snapshot_tick,
            .instance_count = engine.s1024.instance_count,
            .world_hash = 0,
        };
    }

    engine.last_tick_output.hash = .{
        .tick = result.snapshot_tick,
        .value = result.state_hash,
        .determinism_flags = result.determinism_flags,
    };
}

pub fn getLastTickTraceEvents(engine: *const TickEngine) []const PhysicsTraceEvent {
    return engine.last_tick_output.trace_events[0..engine.last_tick_output.trace_count];
}

pub fn getLastTickCompressedTraceBaseTick(engine: *const TickEngine) u32 {
    return engine.last_tick_output.compressed_trace_base_tick;
}

pub fn getLastTickCompressedTraceEvents(engine: *const TickEngine) []const CompressedTraceEvent {
    return engine.last_tick_output.compressed_trace_events[0..engine.last_tick_output.compressed_trace_count];
}

pub fn setTraceEventFilter(engine: *TickEngine, filter: TraceEventFilter) void {
    engine.trace_event_filter = filter;
}

pub fn clearTraceEventFilter(engine: *TickEngine) void {
    engine.trace_event_filter = TraceEventFilter.allowAll();
}

pub fn getLastTickContactTelemetry(engine: *const TickEngine) []const TickContactTelemetry {
    return engine.last_tick_output.contact_telemetry[0..engine.last_tick_output.contact_telemetry_count];
}

pub fn getLastTickSnapshot(engine: *const TickEngine) WorldSnapshot {
    return engine.last_tick_output.snapshot;
}

pub fn getLastTickHash(engine: *const TickEngine) WorldHash {
    return engine.last_tick_output.hash;
}

pub fn stepTickResult(engine: *TickEngine) physics_world.StepResult {
    engine.tick_id += 1;
    engine.s1024.global_tick = engine.tick_id;
    const stage_result = runDiscreteIntentStage(engine, true);
    engine.stable = (stage_result.applied == 0);
    clearFixedTickOutput(engine);
    collectFixedTickTraceAndTelemetryFromPending(engine);
    physics_kernel.finishWorldStep(
        &engine.pending_collisions,
        &engine.pending_sounds,
        &engine.pending_particles,
        &engine.pending_deformations,
        &engine.pending_breaks,
        &engine.pending_joint_breaks,
        engine.world_bus,
        engine.tick_id,
        engine.s1024,
        engine.entities,
        engine.fixed_dt,
    );
    engine.last_step_result = .{
        .changed = !engine.stable,
        .pair_count = stage_result.pair_count,
        .event_count = stage_result.event_count,
        .snapshot_tick = engine.tick_id,
        .state_hash = worldHashForTick(engine.tick_id),
        .determinism_flags = worldDeterminismFlagsForTick(engine.tick_id),
        .authority = .tick_engine,
    };
    finalizeFixedTickSnapshotAndHash(engine, engine.last_step_result);
    buildCompressedTraceOutput(engine);
    if (engine.last_tick_output.compressed_trace_count > 0) {
        enqueueCompressedTraceFrame(engine);
    }
    if (engine.trace_async_flush_budget > 0) {
        _ = flushTraceAsyncWrites(engine, @as(u16, engine.trace_async_flush_budget));
    }
    return engine.last_step_result;
}

pub fn stepTick(engine: *TickEngine) bool {
    _ = stepTickResult(engine);
    return engine.stable;
}

pub fn runTicks(engine: *TickEngine, max_ticks: u32) u32 {
    var ticks_run: u32 = 0;
    while (ticks_run < max_ticks and !engine.stable) {
        if (stepTick(engine)) break;
        ticks_run += 1;
    }
    return ticks_run;
}

// ============================================================================
// P0: Unified PhysicsWorld Step
// ============================================================================

/// Unified physics step that coordinates all physics subsystems
/// This is the main entry point for the unified physics world
pub fn stepPhysicsWorldResult(engine: *TickEngine, dt: f32) physics_world.StepResult {
    var world = buildPhysicsWorld(engine);
    const result = physics_world.stepPhysicsConfigured(&world, .{
        .dt = dt,
        .time_scale = engine.time_scale,
        .run_pre_motion_constraint = true,
        .apply_continuous_physics = false,
        .authority = .tick_engine,
    });
    _ = syncFromPhysicsWorld(engine, &world, result.changed);
    engine.last_step_result = result;
    clearFixedTickOutput(engine);
    collectFixedTickTraceAndTelemetryFromPending(engine);
    finalizeFixedTickSnapshotAndHash(engine, result);
    buildCompressedTraceOutput(engine);
    if (engine.last_tick_output.compressed_trace_count > 0) {
        enqueueCompressedTraceFrame(engine);
    }
    if (engine.trace_async_flush_budget > 0) {
        _ = flushTraceAsyncWrites(engine, @as(u16, engine.trace_async_flush_budget));
    }
    // Apply crash defense: clamp velocities and validate scene state
    crash_defense.clampScene(engine.s1024);

    return result;
}

pub fn stepPhysicsWorld(engine: *TickEngine, dt: f32) void {
    _ = stepPhysicsWorldResult(engine, dt);
}

/// Record world state snapshot for rewind
fn recordWorldSnapshot(engine: *TickEngine) void {
    physics_kernel.recordWorldSnapshot(engine.tick_id, engine.s1024, engine.entities);
}

/// Get fixed timestep
pub fn getFixedDT(engine: *const TickEngine) f32 {
    return engine.fixed_dt;
}

/// Set fixed timestep
pub fn setFixedDT(engine: *TickEngine, dt: f32) void {
    engine.fixed_dt = dt;
}

test "TickEngine stepPhysicsWorld builds broadphase pairs for swept collision candidates" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var wall = entity16.initEntity16();
    wall.physics.mass = 0;
    wall.physics.material = .solid;
    wall.physics.flags |= 0x01;
    entity16.setVoxel(&wall, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, wall };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 3,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    const result = stepPhysicsWorldResult(&engine, engine.fixed_dt);

    try std.testing.expectEqual(@as(usize, 1), engine.broadphase_pair_count);
    try std.testing.expectEqual(@as(u8, 0), engine.broadphase_pairs[0].a);
    try std.testing.expectEqual(@as(u8, 1), engine.broadphase_pairs[0].b);
    try std.testing.expectEqual(@as(u32, 1), result.snapshot_tick);
    try std.testing.expect(result.state_hash != 0);
    try std.testing.expectEqual(result.state_hash, engine.last_step_result.state_hash);
    try std.testing.expectEqual(physics_world.StepAuthority.tick_engine, result.authority);
}

test "TickEngine PhysicsWorld bridge mirrors runtime context without stepping" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities = [_]entity16.Entity16{entity16.initEntity16()};

    var engine: TickEngine = undefined;
    init(&engine, &s1024, &entities);
    var event_bus = bus.Bus.init();
    engine.world_bus = &event_bus;
    engine.tick_id = 17;
    engine.time_scale = 0.5;
    engine.fixed_dt = 0.25;
    engine.joint_count = 99;
    engine.broadphase_pair_count = 1;
    engine.broadphase_pairs[0] = .{ .a = 1, .b = 2 };
    engine.pending_collisions.count = 1;

    var world = makePhysicsWorldView(&engine);
    const cfg = makePhysicsWorldStepConfig(&engine, true);

    try std.testing.expect(world.s1024 == engine.s1024);
    try std.testing.expectEqual(engine.entities.ptr, world.entities.ptr);
    try std.testing.expectEqual(@as(usize, 0), world.joint_count);
    try std.testing.expectEqual(@as(u32, 17), world.tick);
    try std.testing.expect(world.world_bus.? == &event_bus);
    try std.testing.expectEqual(@as(u8, 1), world.pending_collisions.count);
    try std.testing.expectEqual(@as(usize, 1), world.broadphase_pair_count);
    try std.testing.expectEqual(BroadPhasePair{ .a = 1, .b = 2 }, world.broadphase_pairs[0]);

    try std.testing.expectEqual(@as(f32, 0.25), cfg.dt);
    try std.testing.expectEqual(@as(f32, 0.5), cfg.time_scale);
    try std.testing.expect(!cfg.run_pre_motion_constraint);
    try std.testing.expect(cfg.apply_continuous_physics);
    try std.testing.expect(!cfg.finish_world_step);
    try std.testing.expectEqual(physics_world.StepAuthority.tick_engine, cfg.authority);

    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 0;
    _ = try world.s1024.addInstance(inst);
    try std.testing.expectEqual(@as(u8, 1), engine.s1024.instance_count);
}

fn expectCompatStepResultMatches(tick_result: physics_world.StepResult, compat_result: physics_world.StepResult) !void {
    try std.testing.expectEqual(tick_result.changed, compat_result.changed);
    try std.testing.expectEqual(tick_result.pair_count, compat_result.pair_count);
    try std.testing.expectEqual(tick_result.event_count, compat_result.event_count);
    try std.testing.expectEqual(tick_result.snapshot_tick, compat_result.snapshot_tick);
    try std.testing.expectEqual(tick_result.authority, compat_result.authority);
}

fn expectCompatEngineStableMatches(tick_engine_instance: *const TickEngine, compat_engine: *const TickEngine) !void {
    try std.testing.expectEqual(tick_engine_instance.stable, compat_engine.stable);
}

fn expectCompatInstanceKinematicsMatch(tick_scene: *const scene1024.Scene1024, compat_scene: *const scene1024.Scene1024, instance_idx: usize) !void {
    try std.testing.expectEqual(tick_scene.instances[instance_idx].pos_x, compat_scene.instances[instance_idx].pos_x);
    try std.testing.expectEqual(tick_scene.instances[instance_idx].pos_y, compat_scene.instances[instance_idx].pos_y);
    try std.testing.expectEqual(tick_scene.instances[instance_idx].pos_z, compat_scene.instances[instance_idx].pos_z);
    try std.testing.expectEqual(tick_scene.instances[instance_idx].vel_x, compat_scene.instances[instance_idx].vel_x);
    try std.testing.expectEqual(tick_scene.instances[instance_idx].vel_y, compat_scene.instances[instance_idx].vel_y);
    try std.testing.expectEqual(tick_scene.instances[instance_idx].vel_z, compat_scene.instances[instance_idx].vel_z);
}

fn expectCompatSnapshotAndHashMatch(tick_engine_instance: *const TickEngine, compat_engine: *const TickEngine) !void {
    try std.testing.expectEqual(tick_engine_instance.last_tick_output.snapshot.tick, compat_engine.last_tick_output.snapshot.tick);
    try std.testing.expectEqual(tick_engine_instance.last_tick_output.snapshot.instance_count, compat_engine.last_tick_output.snapshot.instance_count);
    try std.testing.expectEqual(tick_engine_instance.last_tick_output.snapshot.world_hash, compat_engine.last_tick_output.snapshot.world_hash);
    try std.testing.expectEqual(tick_engine_instance.last_tick_output.hash.tick, compat_engine.last_tick_output.hash.tick);
    try std.testing.expectEqual(tick_engine_instance.last_tick_output.hash.value, compat_engine.last_tick_output.hash.value);
    try std.testing.expectEqual(tick_engine_instance.last_tick_output.hash.determinism_flags, compat_engine.last_tick_output.hash.determinism_flags);
}

fn expectCompatTraceAndTelemetryMatch(tick_engine_instance: *const TickEngine, compat_engine: *const TickEngine) !void {
    try std.testing.expectEqual(tick_engine_instance.last_tick_output.trace_count, compat_engine.last_tick_output.trace_count);
    var trace_idx: usize = 0;
    while (trace_idx < tick_engine_instance.last_tick_output.trace_count) : (trace_idx += 1) {
        const tick_trace = tick_engine_instance.last_tick_output.trace_events[trace_idx];
        const compat_trace = compat_engine.last_tick_output.trace_events[trace_idx];
        try std.testing.expectEqual(tick_trace.tick_id, compat_trace.tick_id);
        try std.testing.expectEqual(tick_trace.event_type, compat_trace.event_type);
        try std.testing.expectEqual(tick_trace.subject_id, compat_trace.subject_id);
        try std.testing.expectApproxEqAbs(tick_trace.value_a, compat_trace.value_a, 0.0001);
        try std.testing.expectApproxEqAbs(tick_trace.value_b, compat_trace.value_b, 0.0001);
        try std.testing.expectApproxEqAbs(tick_trace.value_c, compat_trace.value_c, 0.0001);
    }

    try std.testing.expectEqual(tick_engine_instance.last_tick_output.contact_telemetry_count, compat_engine.last_tick_output.contact_telemetry_count);
    var contact_idx: usize = 0;
    while (contact_idx < tick_engine_instance.last_tick_output.contact_telemetry_count) : (contact_idx += 1) {
        const tick_contact = tick_engine_instance.last_tick_output.contact_telemetry[contact_idx];
        const compat_contact = compat_engine.last_tick_output.contact_telemetry[contact_idx];
        try std.testing.expectEqual(tick_contact.tick_id, compat_contact.tick_id);
        try std.testing.expectEqual(tick_contact.entity_id, compat_contact.entity_id);
        try std.testing.expectApproxEqAbs(tick_contact.telemetry.damage_modifier, compat_contact.telemetry.damage_modifier, 0.0001);
        try std.testing.expectApproxEqAbs(tick_contact.telemetry.penetration_resistance, compat_contact.telemetry.penetration_resistance, 0.0001);
    }
}

test "TickEngine PhysicsWorld compat step matches empty-world tick result" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    var compat_entities = [_]entity16.Entity16{entity16.initEntity16()};
    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, false);

    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();
    var tick_entities = [_]entity16.Entity16{entity16.initEntity16()};
    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(@as(u32, 1), compat_engine.tick_id);
    try std.testing.expectEqual(@as(u32, 1), compat_engine.s1024.global_tick);
    try expectCompatStepResultMatches(tick_result, compat_result);
    try expectCompatEngineStableMatches(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}

test "TickEngine PhysicsWorld compat step matches static non-empty tick result" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();

    var static_body = entity16.initEntity16();
    static_body.physics.mass = 0;
    static_body.physics.material = .solid;
    static_body.physics.flags |= 0x01;
    entity16.setVoxel(&static_body, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{static_body};
    var tick_entities = [_]entity16.Entity16{static_body};
    const instance = scene32.Instance{
        .entity_id = 0,
        .pos_x = 4,
        .pos_y = 2,
        .pos_z = 6,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 1;
    compat_scene.instances[0] = instance;
    tick_scene.instance_count = 1;
    tick_scene.instances[0] = instance;

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, false);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_scene.instances[0].pos_x, compat_scene.instances[0].pos_x);
    try std.testing.expectEqual(tick_scene.instances[0].pos_y, compat_scene.instances[0].pos_y);
    try std.testing.expectEqual(tick_scene.instances[0].pos_z, compat_scene.instances[0].pos_z);
    try std.testing.expectEqual(tick_scene.instances[0].vel_x, compat_scene.instances[0].vel_x);
    try std.testing.expectEqual(tick_scene.instances[0].vel_y, compat_scene.instances[0].vel_y);
    try std.testing.expectEqual(tick_scene.instances[0].vel_z, compat_scene.instances[0].vel_z);
    try expectCompatStepResultMatches(tick_result, compat_result);
    try expectCompatEngineStableMatches(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}

test "TickEngine PhysicsWorld compat step matches single lateral mover tick result" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    _ = try compat_scene.getPage(0);
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();
    _ = try tick_scene.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{mover};
    var tick_entities = [_]entity16.Entity16{mover};
    const instance = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = 3,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 1;
    compat_scene.instances[0] = instance;
    tick_scene.instance_count = 1;
    tick_scene.instances[0] = instance;

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    compat_engine.time_scale = 0.0;
    var compat_bus = bus.Bus.init();
    compat_engine.world_bus = &compat_bus;
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, true);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    tick_engine_instance.time_scale = 0.0;
    var tick_bus = bus.Bus.init();
    tick_engine_instance.world_bus = &tick_bus;
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_scene.instances[0].pos_x, compat_scene.instances[0].pos_x);
    try std.testing.expectEqual(tick_scene.instances[0].pos_y, compat_scene.instances[0].pos_y);
    try std.testing.expectEqual(tick_scene.instances[0].pos_z, compat_scene.instances[0].pos_z);
    try std.testing.expectEqual(tick_scene.instances[0].vel_x, compat_scene.instances[0].vel_x);
    try std.testing.expectEqual(tick_scene.instances[0].vel_z, compat_scene.instances[0].vel_z);
    try expectCompatStepResultMatches(tick_result, compat_result);
    try expectCompatEngineStableMatches(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}


test "TickEngine PhysicsWorld compat step matches falling body blocked by floor" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    _ = try compat_scene.getPage(0);
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();
    _ = try tick_scene.getPage(0);

    var falling = entity16.initEntity16();
    falling.physics.mass = 20;
    falling.physics.material = .solid;
    falling.physics.hardness = 40;
    entity16.setVoxel(&falling, 0, 0, 0);

    var floor = entity16.initEntity16();
    floor.physics.flags = 0x01;
    entity16.setVoxel(&floor, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{ falling, floor };
    var tick_entities = [_]entity16.Entity16{ falling, floor };
    const body = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const ground = scene32.Instance{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 2;
    compat_scene.instances[0] = body;
    compat_scene.instances[1] = ground;
    tick_scene.instance_count = 2;
    tick_scene.instances[0] = body;
    tick_scene.instances[1] = ground;

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    compat_engine.time_scale = 0.0;
    var compat_bus = bus.Bus.init();
    compat_engine.world_bus = &compat_bus;
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, true);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    tick_engine_instance.time_scale = 0.0;
    var tick_bus = bus.Bus.init();
    tick_engine_instance.world_bus = &tick_bus;
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_scene.instances[0].pos_x, compat_scene.instances[0].pos_x);
    try std.testing.expectEqual(tick_scene.instances[0].pos_y, compat_scene.instances[0].pos_y);
    try std.testing.expectEqual(tick_scene.instances[0].pos_z, compat_scene.instances[0].pos_z);
    try std.testing.expectEqual(tick_scene.instances[0].vel_x, compat_scene.instances[0].vel_x);
    try std.testing.expectEqual(tick_scene.instances[0].vel_y, compat_scene.instances[0].vel_y);
    try std.testing.expectEqual(tick_scene.instances[0].vel_z, compat_scene.instances[0].vel_z);
    try expectCompatStepResultMatches(tick_result, compat_result);
    try expectCompatEngineStableMatches(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}

test "TickEngine PhysicsWorld compat step matches lateral wall block" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var floor = entity16.initEntity16();
    floor.physics.flags = 0x01;
    entity16.setVoxel(&floor, 0, 0, 0);

    var wall = entity16.initEntity16();
    wall.physics.flags = 0x01;
    entity16.setVoxel(&wall, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{ mover, floor, wall };
    var tick_entities = [_]entity16.Entity16{ mover, floor, wall };
    const body = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 16,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const ground = scene32.Instance{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const barrier = scene32.Instance{
        .entity_id = 2,
        .pos_x = 4,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 3;
    compat_scene.instances[0] = body;
    compat_scene.instances[1] = ground;
    compat_scene.instances[2] = barrier;
    tick_scene.instance_count = 3;
    tick_scene.instances[0] = body;
    tick_scene.instances[1] = ground;
    tick_scene.instances[2] = barrier;

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    compat_engine.time_scale = 0.0;
    var compat_bus = bus.Bus.init();
    compat_engine.world_bus = &compat_bus;
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, true);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    tick_engine_instance.time_scale = 0.0;
    var tick_bus = bus.Bus.init();
    tick_engine_instance.world_bus = &tick_bus;
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_scene.instances[0].pos_x, compat_scene.instances[0].pos_x);
    try std.testing.expectEqual(tick_scene.instances[0].pos_y, compat_scene.instances[0].pos_y);
    try std.testing.expectEqual(tick_scene.instances[0].pos_z, compat_scene.instances[0].pos_z);
    try std.testing.expectEqual(tick_scene.instances[0].vel_x, compat_scene.instances[0].vel_x);
    try std.testing.expectEqual(tick_scene.instances[0].vel_y, compat_scene.instances[0].vel_y);
    try std.testing.expectEqual(tick_scene.instances[0].vel_z, compat_scene.instances[0].vel_z);
    try expectCompatStepResultMatches(tick_result, compat_result);
    try expectCompatEngineStableMatches(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}


test "TickEngine PhysicsWorld compat step matches upward ceiling block" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    try compat_scene.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 6,
        .lz = 0,
    }), true);
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();
    try tick_scene.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 6,
        .lz = 0,
    }), true);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{mover};
    var tick_entities = [_]entity16.Entity16{mover};
    const body = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 16,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 1;
    compat_scene.instances[0] = body;
    tick_scene.instance_count = 1;
    tick_scene.instances[0] = body;

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    compat_engine.time_scale = 0.0;
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, true);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    tick_engine_instance.time_scale = 0.0;
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_scene.instances[0].pos_x, compat_scene.instances[0].pos_x);
    try std.testing.expectEqual(tick_scene.instances[0].pos_y, compat_scene.instances[0].pos_y);
    try std.testing.expectEqual(tick_scene.instances[0].pos_z, compat_scene.instances[0].pos_z);
    try std.testing.expectEqual(tick_scene.instances[0].vel_x, compat_scene.instances[0].vel_x);
    try std.testing.expectEqual(tick_scene.instances[0].vel_y, compat_scene.instances[0].vel_y);
    try std.testing.expectEqual(tick_scene.instances[0].vel_z, compat_scene.instances[0].vel_z);
    try expectCompatStepResultMatches(tick_result, compat_result);
    try expectCompatEngineStableMatches(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}


test "TickEngine PhysicsWorld compat step matches blocked fall collision events" {
    terrain.init();
    defer terrain.init();

    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    _ = try compat_scene.getPage(0);
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();
    _ = try tick_scene.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var target = entity16.initEntity16();
    target.physics.mass = 20;
    target.physics.material = .solid;
    target.physics.hardness = 30;
    entity16.setVoxel(&target, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{ mover, target };
    var tick_entities = [_]entity16.Entity16{ mover, target };
    const body = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .falling,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -10,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const target_inst = scene32.Instance{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 2;
    compat_scene.instances[0] = body;
    compat_scene.instances[1] = target_inst;
    tick_scene.instance_count = 2;
    tick_scene.instances[0] = body;
    tick_scene.instances[1] = target_inst;

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    var compat_bus = bus.Bus.init();
    compat_engine.world_bus = &compat_bus;
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, true);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    var tick_bus = bus.Bus.init();
    tick_engine_instance.world_bus = &tick_bus;
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_scene.instances[0].pos_y, compat_scene.instances[0].pos_y);
    try std.testing.expectEqual(tick_scene.instances[0].vel_y, compat_scene.instances[0].vel_y);
    try std.testing.expectEqual(tick_result.changed, compat_result.changed);
    try std.testing.expectEqual(tick_result.event_count, compat_result.event_count);
    try std.testing.expectEqual(tick_bus.msg_count, compat_bus.msg_count);
    try expectCompatTraceAndTelemetryMatch(&tick_engine_instance, &compat_engine);

    var tick_mover_hits: u8 = 0;
    var tick_target_hits: u8 = 0;
    var compat_mover_hits: u8 = 0;
    var compat_target_hits: u8 = 0;
    var msg_idx: u16 = 0;
    while (msg_idx < tick_bus.msg_count) : (msg_idx += 1) {
        const tick_entity_id = tick_bus.messages[msg_idx].entity_id;
        const compat_entity_id = compat_bus.messages[msg_idx].entity_id;
        if (tick_entity_id == 0) tick_mover_hits += 1;
        if (tick_entity_id == 1) tick_target_hits += 1;
        if (compat_entity_id == 0) compat_mover_hits += 1;
        if (compat_entity_id == 1) compat_target_hits += 1;
        try std.testing.expectEqual(tick_bus.messages[msg_idx].msg_type, compat_bus.messages[msg_idx].msg_type);
    }
    try std.testing.expectEqual(tick_mover_hits, compat_mover_hits);
    try std.testing.expectEqual(tick_target_hits, compat_target_hits);
}


test "TickEngine PhysicsWorld compat step matches fragile lateral break" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    _ = try compat_scene.getPage(0);
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();
    _ = try tick_scene.getPage(0);

    var fragile = entity16.initEntity16();
    fragile.physics.mass = 500;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);

    var wall = entity16.initEntity16();
    wall.physics.mass = 0;
    wall.physics.material = .solid;
    wall.physics.flags |= 0x01;
    entity16.setVoxel(&wall, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{ fragile, wall };
    var tick_entities = [_]entity16.Entity16{ fragile, wall };
    const body = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 16,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const barrier = scene32.Instance{
        .entity_id = 1,
        .pos_x = 8,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 2;
    compat_scene.instances[0] = body;
    compat_scene.instances[1] = barrier;
    tick_scene.instance_count = 2;
    tick_scene.instances[0] = body;
    tick_scene.instances[1] = barrier;

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    compat_engine.time_scale = 0.0;
    var compat_bus = bus.Bus.init();
    compat_engine.world_bus = &compat_bus;
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, true);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    tick_engine_instance.time_scale = 0.0;
    var tick_bus = bus.Bus.init();
    tick_engine_instance.world_bus = &tick_bus;
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_scene.instances[0].state, compat_scene.instances[0].state);
    try std.testing.expectEqual(tick_scene.instances[0].pos_x, compat_scene.instances[0].pos_x);
    try std.testing.expectEqual(tick_scene.instances[0].vel_x, compat_scene.instances[0].vel_x);
    try std.testing.expectEqual(tick_result.changed, compat_result.changed);
    try std.testing.expectEqual(tick_result.event_count, compat_result.event_count);
    try std.testing.expectEqual(tick_bus.msg_count, compat_bus.msg_count);
    try expectCompatTraceAndTelemetryMatch(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}


test "TickEngine PhysicsWorld compat step matches joint break event" {
    var compat_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer compat_scene.deinit();
    _ = try compat_scene.getPage(0);
    var tick_scene = scene1024.Scene1024.init(std.testing.allocator);
    defer tick_scene.deinit();
    _ = try tick_scene.getPage(0);

    var body_a = entity16.initEntity16();
    body_a.physics.mass = 10;
    body_a.physics.material = .solid;
    entity16.setVoxel(&body_a, 0, 0, 0);

    var body_b = entity16.initEntity16();
    body_b.physics.mass = 10;
    body_b.physics.material = .solid;
    entity16.setVoxel(&body_b, 0, 0, 0);

    var compat_entities = [_]entity16.Entity16{ body_a, body_b };
    var tick_entities = [_]entity16.Entity16{ body_a, body_b };
    const inst_a = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const inst_b = scene32.Instance{
        .entity_id = 1,
        .pos_x = 8,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    compat_scene.instance_count = 2;
    compat_scene.instances[0] = inst_a;
    compat_scene.instances[1] = inst_b;
    tick_scene.instance_count = 2;
    tick_scene.instances[0] = inst_a;
    tick_scene.instances[1] = inst_b;

    const weak_joint = joint.Joint{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .breaking_force = 1.0,
        .stiffness = 1000,
        .damping = 100,
    };

    var compat_engine: TickEngine = undefined;
    init(&compat_engine, &compat_scene, &compat_entities);
    var compat_joints = [_]joint.Joint{weak_joint};
    compat_engine.joints = compat_joints[0..];
    compat_engine.joint_count = compat_joints.len;
    var compat_bus = bus.Bus.init();
    compat_engine.world_bus = &compat_bus;
    const compat_result = stepViaPhysicsWorldCompat(&compat_engine, false);

    var tick_engine_instance: TickEngine = undefined;
    init(&tick_engine_instance, &tick_scene, &tick_entities);
    var tick_joints = [_]joint.Joint{weak_joint};
    tick_engine_instance.joints = tick_joints[0..];
    tick_engine_instance.joint_count = tick_joints.len;
    var tick_bus = bus.Bus.init();
    tick_engine_instance.world_bus = &tick_bus;
    const tick_result = stepTickResult(&tick_engine_instance);

    try std.testing.expectEqual(tick_engine_instance.joints[0].enabled, compat_engine.joints[0].enabled);
    try std.testing.expectEqual(tick_engine_instance.pending_joint_breaks.count, compat_engine.pending_joint_breaks.count);
    try std.testing.expectEqual(tick_result.event_count, compat_result.event_count);
    try std.testing.expectEqual(tick_result.changed, compat_result.changed);
    try std.testing.expectEqual(tick_bus.msg_count, compat_bus.msg_count);
    try expectCompatTraceAndTelemetryMatch(&tick_engine_instance, &compat_engine);
    try expectCompatSnapshotAndHashMatch(&tick_engine_instance, &compat_engine);
}

test "ground settle clamps tiny post-contact bounce to rest" {
    const testing = std.testing;

    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .falling,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = -8,
        .vel_z = -5,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    settleGroundContact(&inst);

    try testing.expect(inst.vel_x == 0);
    try testing.expect(inst.vel_y == 0);
    try testing.expect(inst.vel_z == 0);
    try testing.expect(inst.state == .resting);
    try testing.expect(inst.sleep_tick >= SLEEP_TIME_THRESHOLD);
}

test "ground settle preserves meaningful rebound velocity" {
    const testing = std.testing;

    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .falling,
        .sleep_tick = 0,
        .vel_x = 20,
        .vel_y = 30,
        .vel_z = 10,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    settleGroundContact(&inst);

    try testing.expect(inst.vel_x == 20);
    try testing.expect(inst.vel_y == 30);
    try testing.expect(inst.vel_z == 10);
    try testing.expect(inst.state == .falling);
    try testing.expect(inst.sleep_tick == 0);
}

test "BREAK intent applies broken state and spawns debris" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var fragile = entity16.initEntity16();
    fragile.physics.mass = 500;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);

    var entities = [_]entity16.Entity16{fragile};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 10,
        .vel_y = -20,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    engine.intents[0] = .{ .instance_idx = 0, .op = .BREAK };
    engine.intent_count = 1;

    _ = commit(&engine);

    try std.testing.expect(s1024.instances[0].state == .broken);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
    try std.testing.expect(destruction.getDebrisSystem().count > 0);
}

test "stepTick updates debris physics after break" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var fragile = entity16.initEntity16();
    fragile.physics.mass = 500;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);

    var entities = [_]entity16.Entity16{fragile};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 10,
        .vel_y = -20,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    engine.intents[0] = .{ .instance_idx = 0, .op = .BREAK };
    engine.intent_count = 1;

    _ = commit(&engine);

    const debris_sys = destruction.getDebrisSystem();
    try std.testing.expect(debris_sys.count > 0);
    const before_y = debris_sys.debris[0].pos_y;
    const before_vel_y = debris_sys.debris[0].vel_y;

    engine.stable = false;
    _ = stepTick(&engine);

    try std.testing.expect(destruction.getDebrisSystem().debris[0].pos_y != before_y or destruction.getDebrisSystem().debris[0].vel_y != before_vel_y);
}

test "BREAK intent removes broken instance from occupancy after rebuild" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var fragile = entity16.initEntity16();
    fragile.physics.mass = 500;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);

    var entities = [_]entity16.Entity16{fragile};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 10,
        .vel_y = -20,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    engine.intents[0] = .{ .instance_idx = 0, .op = .BREAK };
    engine.intent_count = 1;

    _ = commit(&engine);

    try std.testing.expect(s1024.instances[0].state == .broken);
    try std.testing.expect(!physics.isOccupiedGlobal(&s1024, null, entities[0..], 0, 10, 0, null));
}

test "BREAK intent wakes resting body that was supported by broken instance" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var fragile = entity16.initEntity16();
    fragile.physics.mass = 500;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);

    var crate = entity16.initEntity16();
    crate.physics.mass = 20;
    crate.physics.material = .solid;
    crate.physics.hardness = 40;
    entity16.setVoxel(&crate, 0, 0, 0);

    var entities = [_]entity16.Entity16{ fragile, crate };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 16,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = SLEEP_TIME_THRESHOLD,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    engine.intents[0] = .{ .instance_idx = 0, .op = .BREAK };
    engine.intent_count = 1;

    _ = commit(&engine);

    try std.testing.expect(s1024.instances[0].state == .broken);
    try std.testing.expect(s1024.instances[1].state != .resting);
    try std.testing.expect(s1024.instances[1].sleep_tick == 0);
}

test "blocked fall broadcasts collision events for both participants" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var target = entity16.initEntity16();
    target.physics.mass = 20;
    target.physics.material = .solid;
    target.physics.hardness = 30;
    entity16.setVoxel(&target, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, target };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .falling,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -10,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    var event_bus = bus.Bus.init();
    engine.world_bus = &event_bus;

    _ = stepTick(&engine);

    try std.testing.expect(event_bus.msg_count >= 4);
    var mover_hits: u8 = 0;
    var target_hits: u8 = 0;
    var msg_idx: u16 = 0;
    while (msg_idx < event_bus.msg_count) : (msg_idx += 1) {
        const entity_id = event_bus.messages[msg_idx].entity_id;
        if (entity_id == 0) mover_hits += 1;
        if (entity_id == 1) target_hits += 1;
    }
    try std.testing.expect(mover_hits >= 2);
    try std.testing.expect(target_hits >= 2);
}

test "stepTickResult emits fixed trace telemetry snapshot and hash outputs" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var target = entity16.initEntity16();
    target.physics.mass = 20;
    target.physics.material = .solid;
    target.physics.hardness = 30;
    entity16.setVoxel(&target, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, target };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .falling,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -10,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    var event_bus = bus.Bus.init();
    engine.world_bus = &event_bus;

    rewind.init();
    const result = stepTickResult(&engine);
    try std.testing.expect(result.event_count > 0);

    const trace_events = getLastTickTraceEvents(&engine);
    const compressed_trace_events = getLastTickCompressedTraceEvents(&engine);
    const compressed_base_tick = getLastTickCompressedTraceBaseTick(&engine);
    const telemetry = getLastTickContactTelemetry(&engine);
    const snapshot = getLastTickSnapshot(&engine);
    const hash = getLastTickHash(&engine);

    try std.testing.expect(trace_events.len > 0);
    try std.testing.expect(compressed_trace_events.len > 0);
    try std.testing.expect(telemetry.len > 0);
    try std.testing.expect(snapshot.tick == result.snapshot_tick);
    try std.testing.expect(snapshot.world_hash == result.state_hash);
    try std.testing.expect(hash.tick == result.snapshot_tick);
    try std.testing.expect(hash.value == result.state_hash);
    try std.testing.expectEqual(@as(u32, 0), result.determinism_flags);
    try std.testing.expectEqual(@as(u32, 0), hash.determinism_flags);
    const first_round_trip = decompressTraceEvent(compressed_trace_events[0], compressed_base_tick);
    try std.testing.expectEqual(trace_events[0].event_type, first_round_trip.event_type);
    try std.testing.expectEqual(trace_events[0].subject_id, first_round_trip.subject_id);
}

test "stepTickResult reports float determinism flags when snapshot contains non-finite values" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entity = entity16.initEntity16();
    entity.physics.mass = 1;
    entity.physics.material = .solid;
    entity16.setVoxel(&entity, 0, 0, 0);
    var entities = [_]entity16.Entity16{entity};

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);

    terrain.init();
    defer terrain.init();
    terrain.applyWeather(.{
        .rain_intensity = std.math.inf(f32),
        .fog_density = 0.0,
        .wind_speed = 0.0,
        .wind_direction = 0.0,
        .air_temperature = 0.0,
        .visibility = 1.0,
        .freezing = false,
    });

    const result = stepTickResult(&engine);
    try std.testing.expect((result.determinism_flags & rewind.DETERMINISM_FLAG_FLOAT_NON_FINITE) != 0);

    const hash = getLastTickHash(&engine);
    try std.testing.expectEqual(result.determinism_flags, hash.determinism_flags);
}

test "stepTickResult reports SIMD determinism flags when snapshot reduction order drifts" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entity = entity16.initEntity16();
    entity.physics.mass = 1;
    entity.physics.material = .solid;
    entity16.setVoxel(&entity, 0, 0, 0);
    var entities = [_]entity16.Entity16{entity};

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);

    terrain.init();
    defer terrain.init();
    const weather_inputs = [_]terrain.WeatherCondition{
        .{
            .rain_intensity = 100000000.0,
            .fog_density = 1.0,
            .wind_speed = -100000000.0,
            .wind_direction = 1.0,
            .air_temperature = 0.0,
            .visibility = 1.0,
            .freezing = false,
        },
        .{
            .rain_intensity = 16777216.0,
            .fog_density = 1.0,
            .wind_speed = 1.0,
            .wind_direction = -16777216.0,
            .air_temperature = 0.0,
            .visibility = 1.0,
            .freezing = false,
        },
        .{
            .rain_intensity = 33554432.0,
            .fog_density = 0.5,
            .wind_speed = -33554432.0,
            .wind_direction = 3.0,
            .air_temperature = 0.0,
            .visibility = 1.0,
            .freezing = false,
        },
        .{
            .rain_intensity = 100000000000000000000.0,
            .fog_density = 1.0,
            .wind_speed = -100000000000000000000.0,
            .wind_direction = 1.0,
            .air_temperature = 0.0,
            .visibility = 1.0,
            .freezing = false,
        },
    };

    var found_simd_mismatch = false;
    for (weather_inputs) |weather| {
        terrain.applyWeather(weather);
        const result = stepTickResult(&engine);
        const hash = getLastTickHash(&engine);
        try std.testing.expectEqual(result.determinism_flags, hash.determinism_flags);
        if ((result.determinism_flags & rewind.DETERMINISM_FLAG_SIMD_REDUCTION_MISMATCH) != 0) {
            found_simd_mismatch = true;
            break;
        }
    }

    try std.testing.expect(found_simd_mismatch);
}

test "trace event filter keeps only selected trace type" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var target = entity16.initEntity16();
    target.physics.mass = 20;
    target.physics.material = .solid;
    target.physics.hardness = 30;
    entity16.setVoxel(&target, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, target };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .falling,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -10,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    setTraceEventFilter(&engine, TraceEventFilter.onlyType(.collision));
    defer clearTraceEventFilter(&engine);

    _ = stepTickResult(&engine);
    const trace_events = getLastTickTraceEvents(&engine);
    try std.testing.expect(trace_events.len > 0);
    for (trace_events) |trace_event| {
        try std.testing.expectEqual(PhysicsTraceEventType.collision, trace_event.event_type);
    }
}

test "compressed trace event round-trips with quantized values" {
    const event = PhysicsTraceEvent{
        .tick_id = 1234,
        .event_type = .deformation,
        .subject_id = 77,
        .value_a = 12.34,
        .value_b = -4.56,
        .value_c = 0.78,
    };

    const compressed = compressTraceEvent(event, 1230).?;
    try std.testing.expectEqual(@as(u16, 4), compressed.tick_delta);

    const round_trip = decompressTraceEvent(compressed, 1230);
    try std.testing.expectEqual(event.tick_id, round_trip.tick_id);
    try std.testing.expectEqual(event.event_type, round_trip.event_type);
    try std.testing.expectEqual(event.subject_id, round_trip.subject_id);
    try std.testing.expectApproxEqAbs(event.value_a, round_trip.value_a, 0.01);
    try std.testing.expectApproxEqAbs(event.value_b, round_trip.value_b, 0.01);
    try std.testing.expectApproxEqAbs(event.value_c, round_trip.value_c, 0.01);
}

test "trace async writer queues and flushes compressed frames" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entity = entity16.initEntity16();
    entity16.setVoxel(&entity, 0, 0, 0);
    var entities = [_]entity16.Entity16{entity};

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    setTraceAsyncFlushBudget(&engine, 0);

    engine.tick_id = 41;
    clearFixedTickOutput(&engine);
    appendTraceEvent(&engine, .collision, 2, 10.0, 20.0, 1.0);
    buildCompressedTraceOutput(&engine);
    enqueueCompressedTraceFrame(&engine);

    var stats = getTraceAsyncStats(&engine);
    try std.testing.expectEqual(@as(u16, 1), stats.queued_frames);
    try std.testing.expectEqual(@as(u16, 0), stats.history_frames);

    const flushed = flushTraceAsyncWrites(&engine, 1);
    try std.testing.expectEqual(@as(u16, 1), flushed);

    stats = getTraceAsyncStats(&engine);
    try std.testing.expectEqual(@as(u16, 0), stats.queued_frames);
    try std.testing.expectEqual(@as(u16, 1), stats.history_frames);
}

test "queryTraceEvents filters history and pending frames" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entity = entity16.initEntity16();
    entity16.setVoxel(&entity, 0, 0, 0);
    var entities = [_]entity16.Entity16{entity};

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    setTraceAsyncFlushBudget(&engine, 0);

    engine.tick_id = 10;
    clearFixedTickOutput(&engine);
    appendTraceEvent(&engine, .collision, 1, 1.0, 0.0, 0.0);
    buildCompressedTraceOutput(&engine);
    enqueueCompressedTraceFrame(&engine);
    _ = flushTraceAsyncWrites(&engine, 1);

    engine.tick_id = 11;
    clearFixedTickOutput(&engine);
    appendTraceEvent(&engine, .sound, 2, 0.5, 0.8, 1.2);
    buildCompressedTraceOutput(&engine);
    enqueueCompressedTraceFrame(&engine);
    _ = flushTraceAsyncWrites(&engine, 1);

    engine.tick_id = 12;
    clearFixedTickOutput(&engine);
    appendTraceEvent(&engine, .particle, 3, 0.1, 0.2, 0.3);
    buildCompressedTraceOutput(&engine);
    enqueueCompressedTraceFrame(&engine);

    var out: [8]PhysicsTraceEvent = undefined;
    const history_only_count = queryTraceEvents(&engine, .{
        .include_pending = false,
        .type_mask = traceEventMask(.particle),
        .limit = @intCast(out.len),
    }, out[0..]);
    try std.testing.expectEqual(@as(usize, 0), history_only_count);

    const with_pending_count = queryTraceEvents(&engine, .{
        .include_pending = true,
        .type_mask = traceEventMask(.particle),
        .subject_id = 3,
        .min_tick = 12,
        .max_tick = 12,
        .limit = @intCast(out.len),
    }, out[0..]);
    try std.testing.expectEqual(@as(usize, 1), with_pending_count);
    try std.testing.expectEqual(PhysicsTraceEventType.particle, out[0].event_type);
    try std.testing.expectEqual(@as(u16, 3), out[0].subject_id);
    try std.testing.expectEqual(@as(u32, 12), out[0].tick_id);
}

test "exportTraceVisualization maps lanes and intensity from trace values" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entity = entity16.initEntity16();
    entity16.setVoxel(&entity, 0, 0, 0);
    var entities = [_]entity16.Entity16{entity};

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    setTraceAsyncFlushBudget(&engine, 0);

    engine.tick_id = 80;
    clearFixedTickOutput(&engine);
    appendTraceEvent(&engine, .collision, 7, -1.0, 3.0, -2.0);
    buildCompressedTraceOutput(&engine);
    enqueueCompressedTraceFrame(&engine);

    engine.tick_id = 81;
    clearFixedTickOutput(&engine);
    appendTraceEvent(&engine, .sound, 9, 0.2, 0.5, 0.1);
    buildCompressedTraceOutput(&engine);
    enqueueCompressedTraceFrame(&engine);

    var out: [4]TraceVisualizationEntry = undefined;
    const count = exportTraceVisualization(&engine, .{
        .include_pending = true,
        .type_mask = traceTypeMask(.collision) | traceTypeMask(.sound),
        .limit = @intCast(out.len),
    }, out[0..]);

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(TraceVisualizationLane.collision, out[0].lane);
    try std.testing.expectEqual(PhysicsTraceEventType.collision, out[0].event_type);
    try std.testing.expectEqual(@as(u16, 7), out[0].subject_id);
    try std.testing.expectEqual(@as(u32, 80), out[0].tick_id);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), out[0].intensity, 0.0001);

    try std.testing.expectEqual(TraceVisualizationLane.sound, out[1].lane);
    try std.testing.expectEqual(PhysicsTraceEventType.sound, out[1].event_type);
    try std.testing.expectEqual(@as(u16, 9), out[1].subject_id);
    try std.testing.expectEqual(@as(u32, 81), out[1].tick_id);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), out[1].intensity, 0.0001);
}

test "runTraceBenchmark records deterministic trace workload counters" {
    const config = TraceBenchmarkConfig{
        .ticks = 16,
        .events_per_tick = 20,
        .flush_budget_per_tick = 1,
        .query_every = 4,
        .visualize_every = 8,
    };
    const result = try runTraceBenchmark(std.testing.allocator, config);

    try std.testing.expectEqual(config, result.config);
    try std.testing.expectEqual(@as(u32, 320), result.generated_events);
    try std.testing.expectEqual(@as(u32, 320), result.compressed_events);
    try std.testing.expectEqual(@as(u32, 16), result.queued_frames);
    try std.testing.expectEqual(@as(u32, 16), result.flushed_frames);
    try std.testing.expectEqual(@as(u32, 736), result.queried_events);
    try std.testing.expectEqual(@as(u32, 416), result.visualized_events);
    try std.testing.expectEqual(@as(u32, 0), result.dropped_frames);
    try std.testing.expectEqual(@as(u16, 16), result.history_frames);
    try std.testing.expectEqual(@as(u16, 0), result.pending_frames);
}

test "TickEngine stepTick advances lateral velocity through MOVE intents" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var entities = [_]entity16.Entity16{mover};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = 3,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    engine.time_scale = 0.0;

    _ = stepTick(&engine);

    try std.testing.expect(s1024.instances[0].pos_x > 0);
    try std.testing.expect(s1024.instances[0].pos_z > 0);
}

test "TickEngine stepTick does not tunnel through thin lateral wall" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var floor = entity16.initEntity16();
    floor.physics.flags = 0x01;
    entity16.setVoxel(&floor, 0, 0, 0);

    var wall = entity16.initEntity16();
    wall.physics.flags = 0x01;
    entity16.setVoxel(&wall, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, floor, wall };
    s1024.instance_count = 3;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 16,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 4,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    engine.time_scale = 0.0;

    _ = stepTick(&engine);

    try std.testing.expect(s1024.instances[0].pos_x == 3);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
}

test "TickEngine stepTick blocks upward motion on ceiling" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 6,
        .lz = 0,
    }), true);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.hardness = 40;
    entity16.setVoxel(&mover, 0, 0, 0);

    var entities = [_]entity16.Entity16{mover};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 16,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    engine.time_scale = 0.0;

    _ = stepTick(&engine);

    try std.testing.expect(s1024.instances[0].pos_y == 5);
    try std.testing.expect(s1024.instances[0].vel_y == 0);
}

test "TickEngine gravity clamps downward velocity to terminal speed" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var body = entity16.initEntity16();
    body.physics.mass = 20;
    body.physics.material = .solid;
    entity16.setVoxel(&body, 0, 0, 0);

    var entities = [_]entity16.Entity16{body};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 400,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .falling,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -490,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);
    const result = stepTickResult(&engine);

    try std.testing.expect(s1024.instances[0].vel_y >= -physics.TERMINAL_VELOCITY);
    try std.testing.expectEqual(@as(u32, 1), result.snapshot_tick);
    try std.testing.expect(result.state_hash != 0);
    try std.testing.expectEqual(result.state_hash, engine.last_step_result.state_hash);
    try std.testing.expectEqual(physics_world.StepAuthority.tick_engine, result.authority);
}

test "TickEngine stepPhysicsWorld advances KCC characters through coordinator" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{entity16.Prototypes.apple()};
    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);

    const char = kcc.createCharacter(10, 50, 10, .{
        .gravity = -600.0,
        .stand_height = 6,
        .crouch_height = 4,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const before_y = char.pos_y;

    stepPhysicsWorld(&engine, engine.fixed_dt);

    try std.testing.expect(char.pos_y < before_y);
    try std.testing.expect(char.vel_y < 0);
}

test "TickEngine applyDirectionalForceField pushes in configured direction" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var body = entity16.Prototypes.apple();
    body.physics.mass = 20;
    body.physics.flags = 0;
    var entities = [_]entity16.Entity16{body};

    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 2,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 0,
        .pos_x = 30,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);

    const affected = applyDirectionalForceField(&engine, 0.0, 0.0, 0.0, 8.0, 1.0, 0.0, 0.0, 80.0);
    try std.testing.expectEqual(@as(u32, 1), affected);
    try std.testing.expect(s1024.instances[0].vel_x > 0);
    try std.testing.expectEqual(@as(i16, 0), s1024.instances[1].vel_x);
}

test "TickEngine applyVortexForceField applies tangential swirl in radius" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var body = entity16.Prototypes.apple();
    body.physics.mass = 20;
    body.physics.flags = 0;
    var entities = [_]entity16.Entity16{body};

    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 2,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 0,
        .pos_x = 30,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);

    const affected = applyVortexForceField(&engine, 0.0, 0.0, 0.0, 8.0, 80.0);
    try std.testing.expectEqual(@as(u32, 1), affected);
    try std.testing.expect(s1024.instances[0].vel_z > 0);
    try std.testing.expectEqual(@as(i16, 0), s1024.instances[1].vel_z);
}

test "computeExplosionAttenuation provides linear falloff and clamps out-of-range distances" {
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), computeExplosionAttenuation(0.0, 10.0), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), computeExplosionAttenuation(5.0, 10.0), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), computeExplosionAttenuation(10.0, 10.0), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), computeExplosionAttenuation(12.0, 10.0), 0.0001);
    try std.testing.expectApproxEqAbs(
        computeExplosionAttenuation(2.5, 10.0),
        computeExplosionAttenuationWithInvRadius(2.5, 0.1),
        0.0001,
    );
}

test "computeExplosionRadialImpulse returns outward impulse scaled by attenuation and mass" {
    const impulse = computeExplosionRadialImpulse(3.0, 4.0, 0.0, 5.0, 10.0, 100.0, 10) orelse return error.TestUnexpectedResult;
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), impulse.x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), impulse.y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), impulse.z, 0.0001);
    try std.testing.expect(computeExplosionRadialImpulse(1.0, 0.0, 0.0, 0.05, 10.0, 100.0, 10) == null);
}

test "detectForceFieldRegion accepts points inside radius and rejects boundary or outside" {
    const inside = detectForceFieldRegion(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0) orelse return error.TestUnexpectedResult;
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), inside.distance, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), inside.dx, 0.0001);
    try std.testing.expect(detectForceFieldRegion(3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0) == null);
    try std.testing.expect(detectForceFieldRegion(4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0) == null);
}

test "findNextForceFieldPriority walks priority levels from high to low" {
    const fields = [_]ForceField{
        .{ .pos_x = 0.0, .pos_y = 0.0, .pos_z = 0.0, .radius = 4.0, .strength = 10.0, .field_type = .point, .priority = 5 },
        .{ .pos_x = 0.0, .pos_y = 0.0, .pos_z = 0.0, .radius = 4.0, .strength = 10.0, .field_type = .directional, .priority = 12, .dir_x = 1.0, .dir_y = 0.0, .dir_z = 0.0 },
        .{ .pos_x = 0.0, .pos_y = 0.0, .pos_z = 0.0, .radius = 4.0, .strength = 10.0, .field_type = .vortex, .priority = -3 },
        .{ .pos_x = 0.0, .pos_y = 0.0, .pos_z = 0.0, .radius = 4.0, .strength = 10.0, .field_type = .point, .priority = 5 },
    };

    try std.testing.expectEqual(@as(?i16, 12), findNextForceFieldPriority(fields[0..], null));
    try std.testing.expectEqual(@as(?i16, 5), findNextForceFieldPriority(fields[0..], 12));
    try std.testing.expectEqual(@as(?i16, -3), findNextForceFieldPriority(fields[0..], 5));
    try std.testing.expect(findNextForceFieldPriority(fields[0..], -3) == null);
}

test "applyForceFields stacks overlapping fields after resolving priority layers" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var body = entity16.Prototypes.apple();
    body.physics.mass = 20;
    body.physics.flags = 0;
    var entities = [_]entity16.Entity16{body};

    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 2,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);

    _ = applyDirectionalForceField(&engine, 0.0, 0.0, 0.0, 8.0, 1.0, 0.0, 0.0, 80.0);
    const single_vel_x = s1024.instances[0].vel_x;
    s1024.instances[0].vel_x = 0;
    s1024.instances[0].vel_y = 0;
    s1024.instances[0].vel_z = 0;

    const fields = [_]ForceField{
        .{
            .pos_x = 0.0,
            .pos_y = 0.0,
            .pos_z = 0.0,
            .radius = 8.0,
            .strength = 80.0,
            .field_type = .directional,
            .priority = 20,
            .dir_x = 1.0,
            .dir_y = 0.0,
            .dir_z = 0.0,
        },
        .{
            .pos_x = 0.0,
            .pos_y = 0.0,
            .pos_z = 0.0,
            .radius = 8.0,
            .strength = 80.0,
            .field_type = .directional,
            .priority = -5,
            .dir_x = 1.0,
            .dir_y = 0.0,
            .dir_z = 0.0,
        },
    };

    const total_affected = applyForceFields(&engine, fields[0..]);
    try std.testing.expectEqual(@as(u32, 2), total_affected);
    try std.testing.expect(s1024.instances[0].vel_x > single_vel_x);
}

test "force field interaction wakes resting joint neighbor when connected body is affected" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var body = entity16.Prototypes.apple();
    body.physics.mass = 20;
    body.physics.flags = 0;
    var entities = [_]entity16.Entity16{body};

    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 2,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 0,
        .pos_x = 30,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 5,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var engine: TickEngine = undefined;
    init(&engine, &s1024, entities[0..]);

    var joints = [_]joint.Joint{
        .{
            .joint_type = .fixed,
            .entity_a = 0,
            .entity_b = 1,
            .anchor_a_x = 0,
            .anchor_a_y = 0,
            .anchor_a_z = 0,
            .anchor_b_x = 0,
            .anchor_b_y = 0,
            .anchor_b_z = 0,
        },
    };
    engine.joints = joints[0..];
    engine.joint_count = joints.len;

    const affected = applyDirectionalForceField(&engine, 0.0, 0.0, 0.0, 8.0, 1.0, 0.0, 0.0, 80.0);
    try std.testing.expectEqual(@as(u32, 1), affected);
    try std.testing.expect(s1024.instances[1].state == .idle);
}
