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

pub const Operator = enum(u8) { NOP = 0, FALL = 6, FLOW = 7, MOVE = 3, PUSH = 4, BREAK = 5 };
pub const MAX_BROADPHASE_PAIRS: usize = physics_kernel.MAX_BROADPHASE_PAIRS;
pub const BroadPhasePair = physics_kernel.BroadPhasePair;

pub const Intent = struct {
    instance_idx: u8,
    op: Operator,
    dx: i8 = 0,
    dy: i8 = 0,
    dz: i8 = 0,
    priority: u8 = 128,
    target_instance: u8 = 255,
};

pub const SLEEP_TIME_THRESHOLD: u8 = 30;
pub const DEFAULT_TIME_SCALE: f32 = 1.0;

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
    broadphase_pairs: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined,
    broadphase_pair_count: usize = 0,
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

/// Compatibility wrapper for existing callers.
pub fn shouldSleep(inst: *scene32.Instance) bool {
    return physics_kernel.shouldSleep(inst);
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
}

fn handleEvents(engine: *TickEngine) void {
    physics_kernel.publishPendingCollisions(&engine.pending_collisions, engine.world_bus, engine.tick_id);
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
        _ = physics_kernel.settleLowEnergyMotion(inst, 5);
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

/// Apply force to instance (adds to velocity)
pub fn applyForce(inst: *scene32.Instance, force_x: f32, force_y: f32, force_z: f32, mass: u16) void {
    if (mass == 0) return;
    inst.vel_x = @truncate(@as(i32, inst.vel_x) + @as(i32, @intFromFloat(@round(force_x / @as(f32, @floatFromInt(mass)) * 10.0))));
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(force_y / @as(f32, @floatFromInt(mass)) * 10.0))));
    inst.vel_z = @truncate(@as(i32, inst.vel_z) + @as(i32, @intFromFloat(@round(force_z / @as(f32, @floatFromInt(mass)) * 10.0))));
    wakeInstance(inst);
}

/// Apply torque to instance (adds to angular velocity)
pub fn applyTorque(inst: *scene32.Instance, torque_x: f32, torque_y: f32, torque_z: f32, inertia: u16) void {
    if (inertia == 0) return;
    inst.ang_x = @truncate(@as(i16, inst.ang_x) + @as(i8, @intFromFloat(@round(torque_x / @as(f32, @floatFromInt(inertia)) * 10.0))));
    inst.ang_y = @truncate(@as(i16, inst.ang_y) + @as(i8, @intFromFloat(@round(torque_y / @as(f32, @floatFromInt(inertia)) * 10.0))));
    inst.ang_z = @truncate(@as(i16, inst.ang_z) + @as(i8, @intFromFloat(@round(torque_z / @as(f32, @floatFromInt(inertia)) * 10.0))));
    wakeInstance(inst);
}

/// Apply impulse (instant velocity change)
pub fn applyImpulse(inst: *scene32.Instance, impulse_x: f32, impulse_y: f32, impulse_z: f32) void {
    inst.vel_x = @truncate(@as(i32, inst.vel_x) + @as(i32, @intFromFloat(@round(impulse_x))));
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(impulse_y))));
    inst.vel_z = @truncate(@as(i32, inst.vel_z) + @as(i32, @intFromFloat(@round(impulse_z))));
    wakeInstance(inst);
}

/// Apply buoyancy force for fluids (upward force proportional to displaced volume)
pub fn applyBuoyancy(inst: *scene32.Instance, fluid_density: f32, mass: u16) void {
    if (mass == 0) return;
    // Buoyancy = fluid_density * volume * gravity (simplified)
    const volume: f32 = 16.0 * 16.0 * 16.0; // Full 16^3 entity volume
    const buoyancy_force = fluid_density * volume * 0.01;
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(buoyancy_force))));
    wakeInstance(inst);
}

/// Force field types
pub const ForceFieldType = enum(u8) {
    none = 0,
    point = 1,      // Radial explosion-like force
    directional = 2, // Constant direction force (wind, gravity)
    vortex = 3,     // Rotational force
};

pub const ForceField = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    radius: f32,
    strength: f32,
    field_type: ForceFieldType,
};

/// Apply explosion force (point impulse)
pub fn applyExplosion(engine: *TickEngine, fx: f32, fy: f32, fz: f32, radius: f32, force: f32) void {
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue; // Skip static

        const dx = @as(f32, @floatFromInt(inst.pos_x)) - fx;
        const dy = @as(f32, @floatFromInt(inst.pos_y)) - fy;
        const dz = @as(f32, @floatFromInt(inst.pos_z)) - fz;
        const dist = @sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < radius and dist > 0.1) {
            const falloff = 1.0 - (dist / radius);
            const impulse = force * falloff / @as(f32, @floatFromInt(entity.physics.mass));
            const nx = dx / dist;
            const ny = dy / dist;
            const nz = dz / dist;
            applyImpulse(inst, nx * impulse, ny * impulse, nz * impulse);
        }
    }
}

/// Apply force field to all instances in range
pub fn applyForceField(engine: *TickEngine, field: ForceField) void {
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue;

        switch (field.field_type) {
            .point => {
                const dx = @as(f32, @floatFromInt(inst.pos_x)) - field.pos_x;
                const dy = @as(f32, @floatFromInt(inst.pos_y)) - field.pos_y;
                const dz = @as(f32, @floatFromInt(inst.pos_z)) - field.pos_z;
                const dist = @sqrt(dx * dx + dy * dy + dz * dz);
                if (dist < field.radius and dist > 0.1) {
                    const falloff = 1.0 - (dist / field.radius);
                    const impulse = field.strength * falloff / @as(f32, @floatFromInt(entity.physics.mass));
                    const nx = dx / dist;
                    const ny = dy / dist;
                    const nz = dz / dist;
                    applyImpulse(inst, nx * impulse, ny * impulse, nz * impulse);
                }
            },
            .directional => {
                const dx = @as(f32, @floatFromInt(inst.pos_x)) - field.pos_x;
                const dy = @as(f32, @floatFromInt(inst.pos_y)) - field.pos_y;
                const dz = @as(f32, @floatFromInt(inst.pos_z)) - field.pos_z;
                const dist = @sqrt(dx * dx + dy * dy + dz * dz);
                if (dist < field.radius) {
                    const falloff = 1.0 - (dist / field.radius);
                    const force_scale = field.strength * falloff / @as(f32, @floatFromInt(entity.physics.mass)) * 0.1;
                    applyForce(inst, field.pos_x * force_scale, field.pos_y * force_scale, field.pos_z * force_scale, entity.physics.mass);
                }
            },
            .vortex => {
                const dx = @as(f32, @floatFromInt(inst.pos_x)) - field.pos_x;
                const dy = @as(f32, @floatFromInt(inst.pos_y)) - field.pos_y;
                const dz = @as(f32, @floatFromInt(inst.pos_z)) - field.pos_z;
                const dist = @sqrt(dx * dx + dy * dy + dz * dz);
                if (dist < field.radius and dist > 0.1) {
                    const falloff = 1.0 - (dist / field.radius);
                    const tangent_force = field.strength * falloff * 0.1;
                    // Tangential impulse (perpendicular to radius)
                    applyImpulse(inst, -dz * tangent_force / dist, 0, dx * tangent_force / dist);
                }
            },
            else => {},
        }
    }
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
    );
    engine.broadphase_pair_count = physics_kernel.mergeObservedPairCount(
        engine.broadphase_pair_count,
        constraint_result.pair_count,
    );
    return applied;
}

fn runDiscreteIntentStage(engine: *TickEngine, apply_continuous_physics: bool) u16 {
    gatherInternal(engine, apply_continuous_physics);
    speculate(engine);
    resolve(engine);
    return commit(engine);
}

fn runCoordinatorStep(
    engine: *TickEngine,
    dt: f32,
    run_pre_motion_constraint: bool,
    apply_continuous_physics: bool,
) bool {
    physics_kernel.beginWorldStep(&engine.tick_id, engine.s1024);

    if (run_pre_motion_constraint) {
        const pre_constraint = physics_kernel.runPreMotionConstraintStage(
            engine.s1024,
            engine.entities,
            engine.joints[0..engine.joint_count],
            engine.broadphase_pairs[0..],
            engine.time_scale,
            SLEEP_TIME_THRESHOLD,
            dt,
        );
        engine.broadphase_pair_count = pre_constraint.pair_count;
    }

    const applied = runDiscreteIntentStage(engine, apply_continuous_physics);
    engine.stable = (applied == 0);
    physics_kernel.finishWorldStep(&engine.pending_collisions, engine.world_bus, engine.tick_id, engine.s1024, engine.entities, dt);
    return engine.stable;
}

pub fn stepTick(engine: *TickEngine) bool {
    return runCoordinatorStep(engine, engine.fixed_dt, false, true);
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
pub fn stepPhysicsWorld(engine: *TickEngine, dt: f32) void {
    _ = runCoordinatorStep(engine, dt, true, false);
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
    stepPhysicsWorld(&engine, engine.fixed_dt);

    try std.testing.expectEqual(@as(usize, 1), engine.broadphase_pair_count);
    try std.testing.expectEqual(@as(u8, 0), engine.broadphase_pairs[0].a);
    try std.testing.expectEqual(@as(u8, 1), engine.broadphase_pairs[0].b);
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
    _ = stepTick(&engine);

    try std.testing.expect(s1024.instances[0].vel_y >= -physics.TERMINAL_VELOCITY);
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

