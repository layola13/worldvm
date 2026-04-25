//! Shared physics kernel helpers for world coordinators.

const std = @import("std");
const address = @import("address.zig");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const bus = @import("bus.zig");
const physics = @import("physics.zig");
const terrain = @import("terrain.zig");
const material_pairing = @import("material_pairing.zig");
const contact_response = @import("contact_response.zig");
const collision_event = @import("collision_event.zig");
const collision = @import("collision.zig");
const query_types = @import("query_types.zig");
const query_penetration = @import("query_penetration.zig");
const prediction = @import("prediction.zig");
const rewind = @import("rewind.zig");
const sleep_response = @import("sleep_response.zig");
const break_response = @import("break_response.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const ragdoll = @import("ragdoll.zig");
const ballistics = @import("ballistics.zig");
const joint = @import("joint.zig");
const destruction = @import("destruction.zig");

pub const MAX_BROADPHASE_PAIRS: usize = scene32.MAX_INSTANCES * (scene32.MAX_INSTANCES - 1) / 2;

pub const BroadPhasePair = struct {
    a: u8,
    b: u8,
};

pub const ConstraintStageResult = struct {
    pair_count: usize,
    changed: bool,
};

const ConstraintSubsystem = enum(u8) {
    joint,
    contact,
    environment,
};

const ConstraintRowKind = enum(u8) {
    joint_anchor,
    joint_limit,
    joint_drive,
    contact_normal,
    contact_friction,
    environment,
};

const ConstraintRow = struct {
    kind: ConstraintRowKind,
    index: usize,
    priority: f32,
    base_residual: f32,
    equation: ConstraintRowEquation,
};

const ConstraintRowState = struct {
    kind: ConstraintRowKind,
    index: usize,
    retained_residual: f32,
    accumulated_impulse: f32,
    last_impulse_delta: f32,
    touched_iteration: u8,
};

const ConstraintRowExecResult = struct {
    changed: bool,
    residual_before: f32,
    residual_after: f32,
    applied_impulse: f32,

    fn improved(self: ConstraintRowExecResult) bool {
        return self.residual_after + 0.0001 < self.residual_before;
    }
};

const ConstraintRowEquation = struct {
    effective_mass: f32,
    bias: f32,
    max_impulse: f32,
};

const JointMassData = struct {
    inv_mass_a: f32,
    inv_mass_b: f32,
    ratio_a: f32,
    ratio_b: f32,
};

const JointAxis = enum { x, y, z };

const JointAxisVector = struct {
    x: f32,
    y: f32,
    z: f32,
};

const JointAxisProjection = struct {
    along: f32,
    perp_x: f32,
    perp_y: f32,
    perp_z: f32,
    perp_len: f32,
};

const ConstraintSolveStep = struct {
    changed: bool,
    applied_impulse: f32,
};

const PreparedDirectionalConstraint = struct {
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    depth: f32,
    predicted_depth: f32,
};

const PredictiveConstraintGain = struct {
    residual_hint: f32,
    bias_delta: f32,
    impulse_delta: f32,
};

const DirectionalRowPlan = struct {
    residual: f32,
    equation: ConstraintRowEquation,
};

const KernelJointDriveState = struct {
    position: f32,
    relative_velocity: f32,
};

const KernelJointDrivePlan = struct {
    signed_step: f32,
    desired_velocity: f32,
};

const ContactConstraintMetrics = struct {
    penetration_depth: f32,
    normal_speed: f32,
    tangential_speed: f32,

    fn stress(self: ContactConstraintMetrics) f32 {
        return @max(
            self.penetration_depth,
            @max(self.normal_speed * 0.25, self.tangential_speed * 0.125),
        );
    }
};

const ContactPreparedPair = struct {
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal: PreparedDirectionalConstraint,
    restitution: f32,
    friction: f32,
    tangent: PreparedDirectionalConstraint,
    has_tangent: bool,
    normal_equation: ConstraintRowEquation,
    friction_equation: ConstraintRowEquation,
};

const PreparedEnvironmentConstraint = struct {
    normal: PreparedDirectionalConstraint,
    move_x: i32,
    move_y: i32,
    move_z: i32,
};

const EnvironmentConstraintMetrics = struct {
    penetration_depth: f32,
    normal_speed: f32,
    tangential_speed: f32,

    fn stress(self: EnvironmentConstraintMetrics) f32 {
        return @max(
            self.penetration_depth,
            @max(self.normal_speed * 0.25, self.tangential_speed * 0.125),
        );
    }
};

pub const ImpactBreakDecision = struct {
    break_self: bool,
    break_blocker: bool,
};

pub const SweepMotionResult = struct {
    pos_x: i32,
    pos_y: i32,
    pos_z: i32,
    blocked: bool,
    blocker_id: u8,

    pub fn movedAlongAxis(self: SweepMotionResult, axis: physics.SweepAxis, start_x: i32, start_y: i32, start_z: i32) bool {
        return switch (axis) {
            .x => self.pos_x != start_x,
            .y => self.pos_y != start_y,
            .z => self.pos_z != start_z,
        };
    }
};

pub fn canParticipateInBroadPhase(inst: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (inst.state == .broken) return false;
    if (inst.entity_id >= entities.len) return false;
    return physics.computeEntityWorldAABB(inst, &entities[inst.entity_id]) != null;
}

pub fn canProcessDynamicInstance(inst: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (inst.state == .broken) return false;
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];
    return (entity.physics.flags & 0x01) == 0;
}

pub fn isStaticInstance(inst: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];
    return (entity.physics.flags & 0x01) != 0;
}

pub fn sortByDescendingPriority(items: anytype) void {
    if (items.len < 2) return;

    var i: usize = 0;
    while (i < items.len - 1) : (i += 1) {
        var j: usize = 0;
        while (j < items.len - i - 1) : (j += 1) {
            if (items[j].priority < items[j + 1].priority) {
                const tmp = items[j];
                items[j] = items[j + 1];
                items[j + 1] = tmp;
            }
        }
    }
}

pub fn shouldGenerateBroadPhasePair(a: *const scene32.Instance, b: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (!canParticipateInBroadPhase(a, entities) or !canParticipateInBroadPhase(b, entities)) return false;

    const entity_a = &entities[a.entity_id];
    const entity_b = &entities[b.entity_id];
    const static_a = (entity_a.physics.flags & 0x01) != 0;
    const static_b = (entity_b.physics.flags & 0x01) != 0;
    if (static_a and static_b) return false;

    const swept_a = physics.computeSweptEntityWorldAABB(a, entity_a).?;
    const swept_b = physics.computeSweptEntityWorldAABB(b, entity_b).?;
    return physics.aabbHit(swept_a, swept_b);
}

pub fn collectBroadPhasePairs(instances: []const scene32.Instance, entities: []entity16.Entity16, out_pairs: []BroadPhasePair) usize {
    var pair_count: usize = 0;

    var i: u8 = 0;
    while (i < instances.len) : (i += 1) {
        const inst_a = &instances[i];
        if (!canParticipateInBroadPhase(inst_a, entities)) continue;

        var j: u8 = i + 1;
        while (j < instances.len) : (j += 1) {
            const inst_b = &instances[j];
            if (!shouldGenerateBroadPhasePair(inst_a, inst_b, entities)) continue;
            if (pair_count >= out_pairs.len) return pair_count;
            out_pairs[pair_count] = .{
                .a = @min(i, j),
                .b = @max(i, j),
            };
            pair_count += 1;
        }
    }

    return pair_count;
}

pub fn enqueueCollisionPairEvent(
    queue: *collision_event.PendingCollisionQueue,
    instances: []const scene32.Instance,
    entities: []entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    queue.enqueuePair(instances, entities, inst, impact_velocity, blocker_id);
}

pub fn applyVerticalContactResponse(inst: *scene32.Instance, entity: *const entity16.Entity16, blocker_id: u8, impact_velocity: i16) void {
    if (blocker_id != 255) {
        inst.vel_y = 0;
        contact_response.settleGroundContact(inst);
        return;
    }

    const surface_type = terrain.getSurfaceAt(inst.pos_x, inst.pos_z);
    contact_response.applyVerticalSurfaceContact(inst, entity, surface_type, impact_velocity);
}

pub fn handleBlockedFallContact(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    impact_velocity: i16,
) scene32.InstanceState {
    applyVerticalContactResponse(inst, entity, blocker_id, impact_velocity);
    return if (inst.vel_x == 0 and inst.vel_z == 0) .resting else .moving;
}

pub fn handleBlockedUpwardContact(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    impact_velocity: i16,
) void {
    _ = entity;
    _ = blocker_id;
    _ = impact_velocity;
    inst.vel_y = 0;
}

pub fn applyLateralContactResponse(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    axis: contact_response.ContactAxis,
) void {
    if (blocker_id != 255) {
        if (axis == .x) {
            inst.vel_x = 0;
        } else {
            inst.vel_z = 0;
        }
        return;
    }

    const surface_type = terrain.getSurfaceAt(inst.pos_x, inst.pos_z);
    contact_response.applyLateralSurfaceFriction(inst, entity, surface_type, axis);
}

pub fn handleBlockedLateralContact(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    axis: contact_response.ContactAxis,
) void {
    applyLateralContactResponse(inst, entity, blocker_id, axis);
}

pub fn finalizeMotionState(
    inst: *scene32.Instance,
    next_x: i32,
    next_y: i32,
    next_z: i32,
    next_state: scene32.InstanceState,
    broke_this_tick: bool,
    sleep_time_threshold: u8,
) bool {
    var inst_moved = false;

    if (next_x != inst.pos_x or next_y != inst.pos_y or next_z != inst.pos_z) {
        inst.pos_x = next_x;
        inst.pos_y = next_y;
        inst.pos_z = next_z;
        if (!broke_this_tick) {
            inst.state = next_state;
        }
        inst_moved = true;
    } else if (!broke_this_tick and inst.vel_x == 0 and inst.vel_y == 0 and inst.vel_z == 0 and inst.state != .broken) {
        inst.state = .resting;
    }

    if (broke_this_tick) {
        return inst_moved;
    }

    if (inst_moved or inst.vel_x != 0 or inst.vel_y != 0 or inst.vel_z != 0 or inst.ang_x != 0 or inst.ang_y != 0 or inst.ang_z != 0) {
        wakeInstance(inst);
    } else if (shouldSleep(inst)) {
        if (inst.sleep_tick < sleep_time_threshold) {
            inst.sleep_tick += 1;
        }
        if (inst.sleep_tick >= sleep_time_threshold) {
            inst.state = .resting;
            inst.vel_x = 0;
            inst.vel_y = 0;
            inst.vel_z = 0;
            inst.ang_x = 0;
            inst.ang_y = 0;
            inst.ang_z = 0;
        }
    } else {
        inst.sleep_tick = 0;
    }

    return inst_moved;
}

pub fn sweepMotionAlongAxis(
    s1024: *scene1024.Scene1024,
    inst: *const scene32.Instance,
    entities: []entity16.Entity16,
    start_x: i32,
    start_y: i32,
    start_z: i32,
    vel: i16,
    axis: physics.SweepAxis,
) SweepMotionResult {
    const sweep = physics.sweepAxis(s1024, inst, entities, start_x, start_y, start_z, vel, axis);
    return .{
        .pos_x = sweep.pos_x,
        .pos_y = sweep.pos_y,
        .pos_z = sweep.pos_z,
        .blocked = sweep.blocked,
        .blocker_id = sweep.blocker_id,
    };
}

pub fn canOccupyTranslatedPosition(
    s1024: *scene1024.Scene1024,
    inst: *const scene32.Instance,
    entities: []entity16.Entity16,
    target_x: i32,
    target_y: i32,
    target_z: i32,
) bool {
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];

    for (0..64) |w_idx| {
        const word = entity.topology[w_idx];
        if (word == 0) continue;
        for (0..64) |b_idx| {
            if ((word & (@as(u64, 1) << @as(u6, @truncate(b_idx)))) == 0) continue;
            const idx = (w_idx << 6) | b_idx;
            const ex: i32 = @intCast((idx >> 4) & 0xF);
            const ey: i32 = @intCast(idx >> 8);
            const ez: i32 = @intCast(idx & 0xF);
            if (physics.isOccupiedGlobal(s1024, inst, entities, target_x + ex, target_y + ey, target_z + ez, null)) {
                return false;
            }
        }
    }

    return true;
}

pub fn invalidateBlockedTranslationIntents(
    intents: anytype,
    s1024: *scene1024.Scene1024,
    instances: []scene32.Instance,
    entities: []entity16.Entity16,
    nop_op: anytype,
    break_op: anytype,
) void {
    for (intents) |*intent| {
        if (intent.op == nop_op or intent.op == break_op) continue;
        if (intent.instance_idx >= instances.len) {
            intent.op = nop_op;
            continue;
        }

        const inst = &instances[intent.instance_idx];
        const nx = inst.pos_x + intent.dx;
        const ny = inst.pos_y + intent.dy;
        const nz = inst.pos_z + intent.dz;

        if (!canOccupyTranslatedPosition(s1024, inst, entities, nx, ny, nz)) {
            intent.op = nop_op;
        }
    }
}

pub fn appendClampedTranslationIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    op: anytype,
    dx: i32,
    dy: i32,
    dz: i32,
    priority: i16,
) bool {
    if (intent_count.* >= intents.len) return false;
    if (dx == 0 and dy == 0 and dz == 0) return false;

    intents[intent_count.*] = .{
        .instance_idx = instance_idx,
        .op = op,
        .dx = @intCast(@min(127, @max(-128, dx))),
        .dy = @intCast(@min(127, @max(-128, dy))),
        .dz = @intCast(@min(127, @max(-128, dz))),
        .priority = @intCast(priority),
    };
    intent_count.* += 1;
    return true;
}

pub fn appendUniqueInstanceIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    op: anytype,
    priority: u8,
) bool {
    var i: u8 = 0;
    while (i < intent_count.*) : (i += 1) {
        const existing = intents[i];
        if (existing.op == op and existing.instance_idx == instance_idx) return false;
    }
    if (intent_count.* >= intents.len) return false;

    intents[intent_count.*] = .{
        .instance_idx = instance_idx,
        .op = op,
        .priority = priority,
    };
    intent_count.* += 1;
    return true;
}

pub fn settleLowEnergyMotion(inst: *scene32.Instance, linear_threshold: i16) bool {
    if (@abs(inst.vel_x) >= linear_threshold or @abs(inst.vel_y) >= linear_threshold or @abs(inst.vel_z) >= linear_threshold) {
        return false;
    }

    inst.state = .resting;
    inst.vel_x = 0;
    inst.vel_y = 0;
    inst.vel_z = 0;
    return true;
}

pub fn markStaticResting(inst: *scene32.Instance) void {
    inst.state = .resting;
}

pub fn appendFallIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    current_y: i32,
    target_y: i32,
    op: anytype,
    priority: i16,
) bool {
    return appendClampedTranslationIntent(
        intents,
        intent_count,
        instance_idx,
        op,
        0,
        target_y - current_y,
        0,
        priority,
    );
}

pub fn appendFlowIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    current_x: i32,
    current_y: i32,
    current_z: i32,
    target_x: i32,
    target_y: i32,
    target_z: i32,
    op: anytype,
    priority: i16,
) bool {
    return appendClampedTranslationIntent(
        intents,
        intent_count,
        instance_idx,
        op,
        target_x - current_x,
        target_y - current_y,
        target_z - current_z,
        priority,
    );
}

pub fn applyFallDisplacement(inst: *scene32.Instance, dy: i8) bool {
    if (dy == 0) return false;
    inst.pos_y += dy;
    inst.state = if (inst.vel_y != 0) .falling else .resting;
    return true;
}

pub fn applyFlowDisplacement(inst: *scene32.Instance, dx: i8, dy: i8, dz: i8) bool {
    if (dx == 0 and dy == 0 and dz == 0) return false;
    inst.pos_x += dx;
    inst.pos_y += dy;
    inst.pos_z += dz;
    inst.state = .flowing;
    return true;
}

pub fn applyMoveDisplacement(inst: *scene32.Instance, dx: i8, dy: i8, dz: i8) bool {
    if (dx == 0 and dy == 0 and dz == 0) return false;
    inst.pos_x += dx;
    inst.pos_y += dy;
    inst.pos_z += dz;
    inst.state = .moving;
    return true;
}

pub fn applyContinuousPhysics(instances: []scene32.Instance, entities: []entity16.Entity16, time_scale: f32, sleep_time_threshold: u8) void {
    if (time_scale <= 0.0) return;

    var i: usize = 0;
    while (i < instances.len) : (i += 1) {
        const inst = &instances[i];
        if (inst.entity_id >= entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue;

        if (entity.physics.material != .liquid) {
            if (@abs(time_scale - 1.0) <= 0.0001) {
                physics.applyGravity(&inst.vel_y, entity.physics.mass);
            } else {
                const scaled_gravity = @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(physics.GRAVITY)) * time_scale)));
                inst.vel_y = physics.clampVerticalVelocity(@as(i32, inst.vel_y) + scaled_gravity);
            }
        }

        physics.applyDamping(&inst.vel_x, &inst.vel_y, &inst.vel_z);

        if (inst.ang_x != 0 or inst.ang_y != 0 or inst.ang_z != 0) {
            const yaw_delta: i16 = @intCast(@divTrunc(inst.ang_y, 10));
            const pitch_delta: i16 = @intCast(@divTrunc(inst.ang_x, 10));
            const roll_delta: i16 = @intCast(@divTrunc(inst.ang_z, 10));

            const new_yaw: i16 = @as(i16, @intCast(inst.rot_yaw)) + yaw_delta;
            const new_pitch: i16 = @as(i16, @intCast(inst.rot_pitch)) + pitch_delta;
            const new_roll: i16 = @as(i16, @intCast(inst.rot_roll)) + roll_delta;

            inst.rot_yaw = @intCast(@mod(new_yaw, 256));
            inst.rot_pitch = @intCast(@mod(new_pitch, 256));
            inst.rot_roll = @intCast(@mod(new_roll, 256));

            physics.applyAngularDamping(&inst.ang_x, &inst.ang_y, &inst.ang_z);
        }

        if (shouldSleep(inst)) {
            inst.sleep_tick += 1;
            if (inst.sleep_tick >= sleep_time_threshold) {
                inst.state = .resting;
                inst.vel_x = 0;
                inst.vel_y = 0;
                inst.vel_z = 0;
                inst.ang_x = 0;
                inst.ang_y = 0;
                inst.ang_z = 0;
            }
        } else {
            inst.sleep_tick = 0;
        }
    }
}

pub fn publishPendingCollisions(queue: *collision_event.PendingCollisionQueue, world_bus: ?*bus.Bus, tick: u32) void {
    const event_bus = world_bus orelse {
        queue.clear();
        return;
    };
    queue.publish(event_bus, tick);
}

pub fn clearPendingCollisions(queue: *collision_event.PendingCollisionQueue) void {
    queue.clear();
}

pub fn shouldSleep(inst: *scene32.Instance) bool {
    return sleep_response.shouldSleep(inst);
}

pub fn wakeInstance(inst: *scene32.Instance) void {
    sleep_response.wakeInstance(inst);
}

pub fn recordWorldSnapshot(tick: u32, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) void {
    rewind.recordWorldSnapshot(tick, s1024, entities);
}

pub fn beginWorldStep(tick: *u32, s1024: *scene1024.Scene1024) void {
    tick.* += 1;
    s1024.global_tick = tick.*;
}

pub fn rebuildOccupancyIfNeeded(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, changed: bool) void {
    if (!changed) return;
    s1024.rebuildOccupancy(entities) catch {};
}

pub fn applyBreakStateAndWake(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    impact_velocity: i16,
    damage_scale: f32,
) bool {
    if (instance_idx >= s1024.instance_count) return false;
    const inst = &s1024.instances[instance_idx];
    if (inst.entity_id >= entities.len) return false;

    const entity = &entities[inst.entity_id];
    const changed = break_response.applyBreakState(inst, entity, impact_velocity, damage_scale);
    if (changed) {
        sleep_response.wakeSupportedInstancesAfterBreak(
            s1024.instances[0..s1024.instance_count],
            entities,
            instance_idx,
        );
    }
    return changed;
}

pub fn computeBreakVelocity(inst: *const scene32.Instance) i16 {
    const break_velocity_i32 = @max(@as(i32, @abs(inst.vel_x)), @max(@as(i32, @abs(inst.vel_y)), @as(i32, @abs(inst.vel_z))));
    return @truncate(break_velocity_i32);
}

pub fn applyBreakFromInstanceAndWake(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    damage_scale: f32,
) bool {
    if (instance_idx >= s1024.instance_count) return false;
    const inst = &s1024.instances[instance_idx];
    const impact_velocity = computeBreakVelocity(inst);
    return applyBreakStateAndWake(s1024, entities, instance_idx, impact_velocity, damage_scale);
}

pub fn applyCollisionBreakPairAndWake(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    impact_velocity: i16,
    blocker_id: u8,
    damage_scale: f32,
) bool {
    var changed = applyBreakStateAndWake(s1024, entities, instance_idx, impact_velocity, damage_scale);
    if (blocker_id != 255 and blocker_id < s1024.instance_count) {
        changed = applyBreakStateAndWake(s1024, entities, blocker_id, impact_velocity, damage_scale) or changed;
    }
    return changed;
}

pub fn shouldBreakFromImpact(entities: []entity16.Entity16, inst: *const scene32.Instance, impact_velocity: i16) bool {
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];
    const impact = physics.calcImpact(impact_velocity, entity.physics.mass);
    const break_result = physics.checkBreak(impact, entity.physics.material, entity.physics.hardness);
    return break_result.did_break;
}

pub fn evaluateImpactBreakPair(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    impact_velocity: i16,
    blocker_id: u8,
) ImpactBreakDecision {
    var decision: ImpactBreakDecision = .{
        .break_self = false,
        .break_blocker = false,
    };

    if (instance_idx < s1024.instance_count) {
        const inst = &s1024.instances[instance_idx];
        decision.break_self = shouldBreakFromImpact(entities, inst, impact_velocity);
    }

    if (blocker_id != 255 and blocker_id < s1024.instance_count) {
        const blocker = &s1024.instances[blocker_id];
        decision.break_blocker = shouldBreakFromImpact(entities, blocker, impact_velocity);
    }

    return decision;
}

pub fn updateKCCSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    kcc.updateSystem(s1024, entities, dt);
}

pub fn updateVehicleSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    vehicle.applyPredictiveVehicleAvoidance(dt);
    const vehicle_sys = vehicle.getSystem();
    var i: u8 = 0;
    while (i < vehicle_sys.count) : (i += 1) {
        const v = &vehicle_sys.vehicles[i];
        vehicle.update(v, s1024, entities, dt);
    }
}

pub fn updateRagdollSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    const ragdoll_sys = ragdoll.getSystem();
    var i: u8 = 0;
    while (i < ragdoll_sys.count) : (i += 1) {
        const r = &ragdoll_sys.ragdolls[i];
        ragdoll.update(r, dt);
        ragdoll.solveJoints(r, s1024.instances[0..s1024.instance_count], entities);
    }
}

pub fn updateProjectileSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    ballistics.simulateAll(s1024, entities, dt);
}

pub fn runPreStepSystems(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    time_scale: f32,
    sleep_time_threshold: u8,
    dt: f32,
) void {
    applyContinuousPhysics(s1024.instances[0..s1024.instance_count], entities, time_scale, sleep_time_threshold);
    updateKCCSystems(s1024, entities, dt);
    updateVehicleSystems(s1024, entities, dt);
    updateRagdollSystems(s1024, entities, dt);
    updateProjectileSystems(s1024, entities, dt);
}

pub fn solveJointConstraints(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, joints: []joint.Joint) void {
    if (joints.len == 0) return;
    joint.solveJointsForTick(s1024.instances[0..s1024.instance_count], joints, entities);
}

fn instanceInverseMass(inst: *const scene32.Instance, entities: []entity16.Entity16) f32 {
    if (inst.entity_id >= entities.len) return 0.0;
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return 0.0;
    return 1.0 / @as(f32, @floatFromInt(entity.physics.mass));
}

fn computeJointMassData(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?JointMassData {
    const inv_mass_a = instanceInverseMass(inst_a, entities);
    const inv_mass_b = instanceInverseMass(inst_b, entities);
    const total_inv_mass = inv_mass_a + inv_mass_b;
    if (total_inv_mass <= 0.0001) return null;
    return .{
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .ratio_a = inv_mass_a / total_inv_mass,
        .ratio_b = inv_mass_b / total_inv_mass,
    };
}

fn jointDominantAxis(joint_def: *const joint.Joint) JointAxis {
    const abs_x = @abs(joint_def.axis_x);
    const abs_y = @abs(joint_def.axis_y);
    const abs_z = @abs(joint_def.axis_z);
    if (abs_x >= abs_y and abs_x >= abs_z) return .x;
    if (abs_y >= abs_z) return .y;
    return .z;
}

fn jointRotationByteToRadians(value: u8) f32 {
    const signed: i8 = @bitCast(value);
    return @as(f32, @floatFromInt(signed)) * (std.math.pi / 128.0);
}

fn jointRadiansToRotationByte(value: f32) u8 {
    const scaled = value * (128.0 / std.math.pi);
    const signed = @as(i8, @intFromFloat(@round(@max(-128.0, @min(127.0, scaled)))));
    return @bitCast(signed);
}

fn getKernelJointAngle(inst: *const scene32.Instance, joint_def: *const joint.Joint) f32 {
    return switch (jointDominantAxis(joint_def)) {
        .x => jointRotationByteToRadians(inst.rot_roll),
        .y => jointRotationByteToRadians(inst.rot_yaw),
        .z => jointRotationByteToRadians(inst.rot_pitch),
    };
}

fn setKernelJointAngle(inst: *scene32.Instance, joint_def: *const joint.Joint, angle: f32) void {
    switch (jointDominantAxis(joint_def)) {
        .x => inst.rot_roll = jointRadiansToRotationByte(angle),
        .y => inst.rot_yaw = jointRadiansToRotationByte(angle),
        .z => inst.rot_pitch = jointRadiansToRotationByte(angle),
    }
}

fn getKernelJointAxisVector(joint_def: *const joint.Joint) ?JointAxisVector {
    const ax = @as(f32, @floatFromInt(joint_def.axis_x));
    const ay = @as(f32, @floatFromInt(joint_def.axis_y));
    const az = @as(f32, @floatFromInt(joint_def.axis_z));
    const len = @sqrt(ax * ax + ay * ay + az * az);
    if (len <= 0.0001) return null;
    return .{ .x = ax / len, .y = ay / len, .z = az / len };
}

fn measureKernelJointAnchorDelta(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) JointAxisVector {
    return .{
        .x = @as(f32, @floatFromInt(inst_b.pos_x + joint_def.anchor_b_x - inst_a.pos_x - joint_def.anchor_a_x)),
        .y = @as(f32, @floatFromInt(inst_b.pos_y + joint_def.anchor_b_y - inst_a.pos_y - joint_def.anchor_a_y)),
        .z = @as(f32, @floatFromInt(inst_b.pos_z + joint_def.anchor_b_z - inst_a.pos_z - joint_def.anchor_a_z)),
    };
}

fn applyKernelPairDirectionalDisplacement(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
) bool {
    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        changed = applyKernelSingleDisplacement(inst_a, dir_x, dir_y, dir_z, magnitude * mass_data.ratio_a) or changed;
    }
    if (mass_data.inv_mass_b > 0.0) {
        changed = applyKernelSingleDisplacement(inst_b, -dir_x, -dir_y, -dir_z, magnitude * mass_data.ratio_b) or changed;
    }
    return changed;
}

fn applyKernelPairCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    corr_x: f32,
    corr_y: f32,
    corr_z: f32,
) bool {
    const magnitude = @sqrt(corr_x * corr_x + corr_y * corr_y + corr_z * corr_z);
    if (magnitude <= 0.0001) return false;
    return applyKernelPairDirectionalDisplacement(
        inst_a,
        inst_b,
        mass_data,
        corr_x / magnitude,
        corr_y / magnitude,
        corr_z / magnitude,
        magnitude,
    );
}

fn signedCorrection(value: f32, warm_impulse: f32) f32 {
    return value + if (value >= 0.0) warm_impulse else -warm_impulse;
}

fn cappedWarmRatio(warm_impulse: f32, magnitude: f32) f32 {
    if (warm_impulse <= 0.0) return 0.0;
    return @min(1.0, warm_impulse / @max(@abs(magnitude), 0.0001));
}

fn measureKernelRelativeLinearSpeed(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
) f32 {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    return rel_vel_x * dir_x + rel_vel_y * dir_y + rel_vel_z * dir_z;
}

fn applyKernelDirectionalPairCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
    warm_impulse: f32,
) bool {
    const scaled_magnitude = magnitude * (1.0 + cappedWarmRatio(warm_impulse, magnitude));
    return applyKernelPairDirectionalDisplacement(
        inst_a,
        inst_b,
        mass_data,
        dir_x,
        dir_y,
        dir_z,
        scaled_magnitude,
    );
}

fn applyKernelAxisPairCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis: JointAxisVector,
    correction: f32,
) bool {
    return applyKernelPairDirectionalDisplacement(
        inst_a,
        inst_b,
        mass_data,
        axis.x,
        axis.y,
        axis.z,
        correction,
    );
}

fn applyKernelAngularPairCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    angle_a: f32,
    angle_b: f32,
    correction: f32,
) bool {
    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        setKernelJointAngle(inst_a, joint_def, angle_a + correction * mass_data.ratio_a);
        changed = true;
    }
    if (mass_data.inv_mass_b > 0.0) {
        setKernelJointAngle(inst_b, joint_def, angle_b - correction * mass_data.ratio_b);
        changed = true;
    }
    return changed;
}

fn lengthAndNormal(vector: JointAxisVector) ?struct { len: f32, nx: f32, ny: f32, nz: f32 } {
    const len = @sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
    if (len <= 0.0001) return null;
    return .{
        .len = len,
        .nx = vector.x / len,
        .ny = vector.y / len,
        .nz = vector.z / len,
    };
}

fn projectJointDeltaToAxis(delta: JointAxisVector, axis: JointAxisVector) JointAxisProjection {
    const along = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
    const perp_x = delta.x - axis.x * along;
    const perp_y = delta.y - axis.y * along;
    const perp_z = delta.z - axis.z * along;
    return .{
        .along = along,
        .perp_x = perp_x,
        .perp_y = perp_y,
        .perp_z = perp_z,
        .perp_len = @sqrt(perp_x * perp_x + perp_y * perp_y + perp_z * perp_z),
    };
}

fn clampKernelI8FromF32(value: f32) i8 {
    const clamped = @max(-128.0, @min(127.0, value));
    if (clamped > 0.0 and clamped < 1.0) return 1;
    if (clamped < 0.0 and clamped > -1.0) return -1;
    return @as(i8, @intFromFloat(@round(clamped)));
}

fn clampKernelI16FromF32(value: f32) i16 {
    const clamped = @max(-32768.0, @min(32767.0, value));
    if (clamped > 0.0 and clamped < 1.0) return 1;
    if (clamped < 0.0 and clamped > -1.0) return -1;
    return @as(i16, @intFromFloat(@round(clamped)));
}

fn applyKernelSingleDisplacement(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
) bool {
    if (magnitude == 0.0) return false;
    const dx = @as(i32, @intFromFloat(@round(dir_x * magnitude)));
    const dy = @as(i32, @intFromFloat(@round(dir_y * magnitude)));
    const dz = @as(i32, @intFromFloat(@round(dir_z * magnitude)));
    if (dx == 0 and dy == 0 and dz == 0) return false;
    inst.pos_x += dx;
    inst.pos_y += dy;
    inst.pos_z += dz;
    return true;
}

fn applyKernelLinearVelocityImpulsePair(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    signed_impulse: f32,
) bool {
    var changed = false;

    if (inv_mass_a > 0.0) {
        const delta_ax = clampKernelI16FromF32(-dir_x * signed_impulse * inv_mass_a);
        const delta_ay = clampKernelI16FromF32(-dir_y * signed_impulse * inv_mass_a);
        const delta_az = clampKernelI16FromF32(-dir_z * signed_impulse * inv_mass_a);
        if (delta_ax != 0 or delta_ay != 0 or delta_az != 0) {
            inst_a.vel_x += delta_ax;
            inst_a.vel_y += delta_ay;
            inst_a.vel_z += delta_az;
            changed = true;
        }
    }

    if (inv_mass_b > 0.0) {
        const delta_bx = clampKernelI16FromF32(dir_x * signed_impulse * inv_mass_b);
        const delta_by = clampKernelI16FromF32(dir_y * signed_impulse * inv_mass_b);
        const delta_bz = clampKernelI16FromF32(dir_z * signed_impulse * inv_mass_b);
        if (delta_bx != 0 or delta_by != 0 or delta_bz != 0) {
            inst_b.vel_x += delta_bx;
            inst_b.vel_y += delta_by;
            inst_b.vel_z += delta_bz;
            changed = true;
        }
    }

    return changed;
}

fn applyKernelLinearDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    relative_speed: f32,
    damping_scale: f32,
) bool {
    if (@abs(relative_speed) <= 0.0001 or damping_scale <= 0.0) return false;

    const damping_impulse = relative_speed * damping_scale;
    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        const dvx = clampKernelI16FromF32(dir_x * damping_impulse * mass_data.ratio_a);
        const dvy = clampKernelI16FromF32(dir_y * damping_impulse * mass_data.ratio_a);
        const dvz = clampKernelI16FromF32(dir_z * damping_impulse * mass_data.ratio_a);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_a.vel_x += dvx;
            inst_a.vel_y += dvy;
            inst_a.vel_z += dvz;
            changed = true;
        }
    }
    if (mass_data.inv_mass_b > 0.0) {
        const dvx = clampKernelI16FromF32(dir_x * damping_impulse * mass_data.ratio_b);
        const dvy = clampKernelI16FromF32(dir_y * damping_impulse * mass_data.ratio_b);
        const dvz = clampKernelI16FromF32(dir_z * damping_impulse * mass_data.ratio_b);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_b.vel_x -= dvx;
            inst_b.vel_y -= dvy;
            inst_b.vel_z -= dvz;
            changed = true;
        }
    }
    return changed;
}

fn wakeJointPair(inst_a: *scene32.Instance, inst_b: *scene32.Instance, changed: bool) void {
    if (!changed) return;
    wakeInstance(inst_a);
    wakeInstance(inst_b);
}

fn wakeSingleInstance(inst: *scene32.Instance, changed: bool) void {
    if (!changed) return;
    wakeInstance(inst);
}

fn settleAndWakeContactPair(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_y: f32,
    changed: bool,
) void {
    settleContactRestState(inst_a, inst_b, inv_mass_a, inv_mass_b, normal_y);
    wakeJointPair(inst_a, inst_b, changed);
}

fn finalizeJointRowResult(
    kind: ConstraintRowKind,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    before: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    wakeJointPair(inst_a, inst_b, changed);
    const after = measureJointRowResidual(kind, instances, joints, joint_idx, entities);
    return finalizeConstraintRowResult(before, after, changed, applied_impulse, equation);
}

fn finalizeJointRowNoChange(
    kind: ConstraintRowKind,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    before: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    const after = measureJointRowResidual(kind, instances, joints, joint_idx, entities);
    return finalizeConstraintRowResult(before, after, false, 0.0, equation);
}

fn finalizeContactRowResult(
    kind: ConstraintRowKind,
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    entities: []entity16.Entity16,
    before: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    wakeJointPair(inst_a, inst_b, changed);
    const after = measureContactRowResidual(kind, inst_a, inst_b, entities);
    return finalizeConstraintRowResult(before, after, changed, applied_impulse, equation);
}

fn finalizeEnvironmentRowResult(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    inst: *scene32.Instance,
    before: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    wakeSingleInstance(inst, changed);
    const after = measureEnvironmentRowResidual(s1024, entities, instance_idx);
    return finalizeConstraintRowResult(before, after, changed, applied_impulse, equation);
}

fn applyEnvironmentSurfaceResponse(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    move_x: i32,
    move_y: i32,
    move_z: i32,
    previous_vel_x: i16,
    previous_vel_y: i16,
    previous_vel_z: i16,
) void {
    if (move_y != 0) {
        applyVerticalContactResponse(inst, entity, 255, previous_vel_y);
    } else {
        if (move_x != 0) {
            applyLateralContactResponse(inst, entity, 255, .x);
            inst.vel_x = 0;
            if (previous_vel_x != 0 and inst.vel_z == previous_vel_z) {
                inst.vel_z = previous_vel_z;
            }
        }
        if (move_z != 0) {
            applyLateralContactResponse(inst, entity, 255, .z);
            inst.vel_z = 0;
            if (previous_vel_z != 0 and inst.vel_x == previous_vel_x) {
                inst.vel_x = previous_vel_x;
            }
        }
    }

    if (move_y > 0 and @abs(inst.vel_x) <= 1 and @abs(inst.vel_z) <= 1) {
        inst.state = .resting;
    }
}

fn applyEnvironmentResolvedMotion(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    move_x: i32,
    move_y: i32,
    move_z: i32,
    previous_vel_x: i16,
    previous_vel_y: i16,
    previous_vel_z: i16,
) bool {
    if (move_x == 0 and move_y == 0 and move_z == 0) return false;
    inst.pos_x += move_x;
    inst.pos_y += move_y;
    inst.pos_z += move_z;
    applyEnvironmentSurfaceResponse(
        inst,
        entity,
        move_x,
        move_y,
        move_z,
        previous_vel_x,
        previous_vel_y,
        previous_vel_z,
    );
    return true;
}

fn applyEnvironmentWarmStartDisplacement(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    warm_move: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applySingleDisplacementRowStep(
        inst,
        dir_x,
        dir_y,
        dir_z,
        warm_move,
        equation,
        0.15,
    );
}

fn applySingleDisplacementRowStep(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
) ConstraintSolveStep {
    const solve_magnitude = @max(0.0, constraintRowSolveMagnitude(raw_magnitude, equation, bias_scale));
    return .{
        .changed = applyKernelSingleDisplacement(inst, dir_x, dir_y, dir_z, solve_magnitude),
        .applied_impulse = solve_magnitude,
    };
}

fn applyPairDirectionalDisplacementRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    magnitude_slop: f32,
) ConstraintSolveStep {
    const total_inv_mass = inv_mass_a + inv_mass_b;
    if (total_inv_mass <= 0.0001) return .{ .changed = false, .applied_impulse = 0.0 };

    const solve_magnitude = @max(0.0, constraintRowSolveMagnitude(raw_magnitude, equation, bias_scale));
    if (solve_magnitude <= 0.0 and magnitude_slop <= 0.0) return .{ .changed = false, .applied_impulse = 0.0 };

    return .{
        .changed = applyKernelPairDirectionalDisplacement(
            inst_a,
            inst_b,
            .{
                .inv_mass_a = inv_mass_a,
                .inv_mass_b = inv_mass_b,
                .ratio_a = inv_mass_a / total_inv_mass,
                .ratio_b = inv_mass_b / total_inv_mass,
            },
            dir_x,
            dir_y,
            dir_z,
            solve_magnitude + magnitude_slop,
        ),
        .applied_impulse = solve_magnitude,
    };
}

fn applyEnvironmentSolveDisplacement(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applySingleDisplacementRowStep(
        inst,
        dir_x,
        dir_y,
        dir_z,
        raw_magnitude,
        equation,
        0.35,
    );
}

fn applyJointAngularScalarCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    angle_a: f32,
    angle_b: f32,
    correction: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const solve_correction = constraintRowSolveMagnitude(correction, equation, 0.35);
    return .{
        .changed = if (solve_correction != 0.0)
            applyKernelAngularPairCorrection(inst_a, inst_b, joint_def, mass_data, angle_a, angle_b, solve_correction)
        else
            false,
        .applied_impulse = @abs(solve_correction),
    };
}

fn applyJointAxisScalarCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis: JointAxisVector,
    correction: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const solve_correction = constraintRowSolveMagnitude(correction, equation, 0.35);
    return .{
        .changed = if (solve_correction != 0.0)
            applyKernelAxisPairCorrection(inst_a, inst_b, mass_data, axis, solve_correction)
        else
            false,
        .applied_impulse = @abs(solve_correction),
    };
}

fn applyJointDirectionalConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
    warm_impulse: f32,
    damping: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const solve_magnitude = constraintRowSolveMagnitude(magnitude, equation, 0.25);
    var changed = applyKernelDirectionalPairCorrection(
        inst_a,
        inst_b,
        mass_data,
        dir_x,
        dir_y,
        dir_z,
        solve_magnitude,
        warm_impulse,
    );
    const rel_speed = measureKernelRelativeLinearSpeed(inst_a, inst_b, dir_x, dir_y, dir_z);
    changed = applyKernelLinearDamping(inst_a, inst_b, mass_data, dir_x, dir_y, dir_z, rel_speed, damping) or changed;
    return .{
        .changed = changed,
        .applied_impulse = @abs(solve_magnitude) * (1.0 + cappedWarmRatio(warm_impulse, solve_magnitude)),
    };
}

fn applyJointAnchorDistanceConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    joint_def: *const joint.Joint,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
    warm_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applyJointDirectionalConstraint(
        inst_a,
        inst_b,
        mass_data,
        dir_x,
        dir_y,
        dir_z,
        magnitude,
        warm_impulse,
        @max(0.05, @min(1.0, @max(0.0, joint_def.damping) * 0.001)),
        equation,
    );
}

fn applyKernelAngularLimitVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
) bool {
    const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
    const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
    const angle_tolerance: f32 = 0.05;
    const relative_angle = getKernelJointAngle(inst_b, joint_def) - getKernelJointAngle(inst_a, joint_def);
    const relative_velocity = switch (jointDominantAxis(joint_def)) {
        .x => @as(f32, @floatFromInt(inst_b.ang_x - inst_a.ang_x)),
        .y => @as(f32, @floatFromInt(inst_b.ang_y - inst_a.ang_y)),
        .z => @as(f32, @floatFromInt(inst_b.ang_z - inst_a.ang_z)),
    };

    const pushes_past_min = relative_angle <= min_angle + angle_tolerance and relative_velocity < 0.0;
    const pushes_past_max = relative_angle >= max_angle - angle_tolerance and relative_velocity > 0.0;
    if (!pushes_past_min and !pushes_past_max) return false;

    const angular_bias = clampKernelI8FromF32(@max(-32.0, @min(32.0, -relative_velocity)));
    if (angular_bias == 0) return false;

    var changed = false;
    switch (jointDominantAxis(joint_def)) {
        .x => {
            if (mass_data.inv_mass_a > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
                if (delta != 0) {
                    inst_a.ang_x -= delta;
                    changed = true;
                }
            }
            if (mass_data.inv_mass_b > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
                if (delta != 0) {
                    inst_b.ang_x += delta;
                    changed = true;
                }
            }
        },
        .y => {
            if (mass_data.inv_mass_a > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
                if (delta != 0) {
                    inst_a.ang_y -= delta;
                    changed = true;
                }
            }
            if (mass_data.inv_mass_b > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
                if (delta != 0) {
                    inst_b.ang_y += delta;
                    changed = true;
                }
            }
        },
        .z => {
            if (mass_data.inv_mass_a > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
                if (delta != 0) {
                    inst_a.ang_z -= delta;
                    changed = true;
                }
            }
            if (mass_data.inv_mass_b > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
                if (delta != 0) {
                    inst_b.ang_z += delta;
                    changed = true;
                }
            }
        },
    }

    return changed;
}

fn applyKernelSliderLimitVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    along: f32,
    min_limit: f32,
    max_limit: f32,
) bool {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_speed = rel_vel_x * axis_x + rel_vel_y * axis_y + rel_vel_z * axis_z;

    const pushes_past_min = along <= min_limit + 0.0001 and rel_speed < 0.0;
    const pushes_past_max = along >= max_limit - 0.0001 and rel_speed > 0.0;
    if (!pushes_past_min and !pushes_past_max) return false;

    const vel_bias = @max(-64.0, @min(64.0, -rel_speed));
    if (vel_bias == 0.0) return false;

    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        const dvx = clampKernelI16FromF32(axis_x * vel_bias * mass_data.ratio_a);
        const dvy = clampKernelI16FromF32(axis_y * vel_bias * mass_data.ratio_a);
        const dvz = clampKernelI16FromF32(axis_z * vel_bias * mass_data.ratio_a);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_a.vel_x -= dvx;
            inst_a.vel_y -= dvy;
            inst_a.vel_z -= dvz;
            changed = true;
        }
    }
    if (mass_data.inv_mass_b > 0.0) {
        const dvx = clampKernelI16FromF32(axis_x * vel_bias * mass_data.ratio_b);
        const dvy = clampKernelI16FromF32(axis_y * vel_bias * mass_data.ratio_b);
        const dvz = clampKernelI16FromF32(axis_z * vel_bias * mass_data.ratio_b);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_b.vel_x += dvx;
            inst_b.vel_y += dvy;
            inst_b.vel_z += dvz;
            changed = true;
        }
    }
    return changed;
}

fn measureKernelJointDriveState(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?KernelJointDriveState {
    return switch (joint_def.joint_type) {
        .hinge => .{
            .position = getKernelJointAngle(inst_b, joint_def) - getKernelJointAngle(inst_a, joint_def),
            .relative_velocity = switch (jointDominantAxis(joint_def)) {
                .x => @as(f32, @floatFromInt(inst_b.ang_x - inst_a.ang_x)),
                .y => @as(f32, @floatFromInt(inst_b.ang_y - inst_a.ang_y)),
                .z => @as(f32, @floatFromInt(inst_b.ang_z - inst_a.ang_z)),
            },
        },
        .slider => blk: {
            const axis = getKernelJointAxisVector(joint_def) orelse return null;
            const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
            break :blk .{
                .position = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z,
                .relative_velocity = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x)) * axis.x +
                    @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y)) * axis.y +
                    @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z)) * axis.z,
            };
        },
        else => null,
    };
}

fn computeKernelJointDrivePlan(
    joint_def: *const joint.Joint,
    drive_state: KernelJointDriveState,
    min_step: f32,
) ?KernelJointDrivePlan {
    const prediction_dt = 1.0 / 60.0;
    const max_drive_velocity = @max(1.0, joint_def.motor_speed);
    const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
    const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
    const target = @max(min_limit, @min(max_limit, joint_def.motor_target));
    const position_error = target - drive_state.position;
    if (@abs(position_error) <= 0.0001) {
        if (@abs(drive_state.relative_velocity) <= 0.0001) return null;
        return .{
            .signed_step = 0.0,
            .desired_velocity = 0.0,
        };
    }

    const predicted_position = drive_state.position + drive_state.relative_velocity * prediction_dt;
    const predicted_error = target - predicted_position;
    if (@abs(predicted_error) <= 0.0001) {
        return .{
            .signed_step = 0.0,
            .desired_velocity = 0.0,
        };
    }

    const use_predictive_brake = position_error * predicted_error < 0.0 or @abs(predicted_error) < @abs(position_error);
    const control_error = if (use_predictive_brake) predicted_error else position_error;
    const speed_step = if (joint_def.motor_speed > 0.0) joint_def.motor_speed * prediction_dt else @abs(control_error);
    const torque_step = if (joint_def.motor_max_torque > 0.0) joint_def.motor_max_torque * 0.001 else @abs(control_error);
    const requested_step = @min(@abs(control_error), @min(speed_step, torque_step));
    const max_step = @max(min_step, requested_step);
    const signed_step = if (control_error < 0.0) -max_step else max_step;
    const desired_velocity = if (use_predictive_brake)
        @max(-max_drive_velocity, @min(max_drive_velocity, predicted_error / prediction_dt))
    else
        signed_step * max_drive_velocity;

    return .{
        .signed_step = signed_step,
        .desired_velocity = desired_velocity,
    };
}

fn applyKernelAngularMotorVelocityBias(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    desired_velocity: f32,
) bool {
    const drive_state = measureKernelJointDriveState(inst_a, inst_b, joint_def) orelse return false;
    const current_rel_velocity = drive_state.relative_velocity;
    if (desired_velocity != 0.0 and current_rel_velocity != 0.0) {
        if ((desired_velocity > 0.0 and current_rel_velocity > 0.0 and current_rel_velocity >= desired_velocity) or
            (desired_velocity < 0.0 and current_rel_velocity < 0.0 and current_rel_velocity <= desired_velocity))
        {
            return false;
        }
    }

    const angular_bias = clampKernelI8FromF32(@max(-32.0, @min(32.0, desired_velocity - current_rel_velocity)));
    if (angular_bias == 0) return false;

    var changed = false;
    switch (jointDominantAxis(joint_def)) {
        .x => {
            if (mass_data.inv_mass_a > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
                if (delta != 0) {
                    inst_a.ang_x -= delta;
                    changed = true;
                }
            }
            if (mass_data.inv_mass_b > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
                if (delta != 0) {
                    inst_b.ang_x += delta;
                    changed = true;
                }
            }
        },
        .y => {
            if (mass_data.inv_mass_a > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
                if (delta != 0) {
                    inst_a.ang_y -= delta;
                    changed = true;
                }
            }
            if (mass_data.inv_mass_b > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
                if (delta != 0) {
                    inst_b.ang_y += delta;
                    changed = true;
                }
            }
        },
        .z => {
            if (mass_data.inv_mass_a > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
                if (delta != 0) {
                    inst_a.ang_z -= delta;
                    changed = true;
                }
            }
            if (mass_data.inv_mass_b > 0.0) {
                const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
                if (delta != 0) {
                    inst_b.ang_z += delta;
                    changed = true;
                }
            }
        },
    }

    return changed;
}

fn applyKernelLinearMotorVelocityBias(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    desired_speed: f32,
) bool {
    const drive_state = measureKernelJointDriveState(inst_a, inst_b, joint_def) orelse return false;
    const current_rel_speed = drive_state.relative_velocity;
    if (desired_speed != 0.0 and current_rel_speed != 0.0) {
        if ((desired_speed > 0.0 and current_rel_speed > 0.0 and current_rel_speed >= desired_speed) or
            (desired_speed < 0.0 and current_rel_speed < 0.0 and current_rel_speed <= desired_speed))
        {
            return false;
        }
    }

    const vel_bias = @max(-64.0, @min(64.0, desired_speed - current_rel_speed));
    if (vel_bias == 0.0) return false;

    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        const dvx = clampKernelI16FromF32(axis_x * vel_bias * mass_data.ratio_a);
        const dvy = clampKernelI16FromF32(axis_y * vel_bias * mass_data.ratio_a);
        const dvz = clampKernelI16FromF32(axis_z * vel_bias * mass_data.ratio_a);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_a.vel_x -= dvx;
            inst_a.vel_y -= dvy;
            inst_a.vel_z -= dvz;
            changed = true;
        }
    }
    if (mass_data.inv_mass_b > 0.0) {
        const dvx = clampKernelI16FromF32(axis_x * vel_bias * mass_data.ratio_b);
        const dvy = clampKernelI16FromF32(axis_y * vel_bias * mass_data.ratio_b);
        const dvz = clampKernelI16FromF32(axis_z * vel_bias * mass_data.ratio_b);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_b.vel_x += dvx;
            inst_b.vel_y += dvy;
            inst_b.vel_z += dvz;
            changed = true;
        }
    }

    return changed;
}

fn buildContactNormalEquation(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
) ConstraintRowEquation {
    const total_inv_mass = inv_mass_a + inv_mass_b;
    const effective_mass = if (total_inv_mass > 0.0001) 1.0 / total_inv_mass else 0.0;
    const bias = penetration_depth * 0.2 + @max(0.0, -relative_normal_speed) * 0.05;
    return .{
        .effective_mass = effective_mass,
        .bias = bias,
        .max_impulse = @max(0.5, penetration_depth * 8.0 + @abs(relative_normal_speed) * 0.5),
    };
}

fn buildContactFrictionEquation(
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangential_speed: f32,
    friction_coeff: f32,
    normal_impulse_limit: f32,
) ConstraintRowEquation {
    const total_inv_mass = inv_mass_a + inv_mass_b;
    const effective_mass = if (total_inv_mass > 0.0001) 1.0 / total_inv_mass else 0.0;
    return .{
        .effective_mass = effective_mass,
        .bias = @abs(tangential_speed) * 0.05,
        .max_impulse = @max(0.25, @min(friction_coeff * @max(0.5, normal_impulse_limit), friction_coeff * 8.0)),
    };
}

fn buildEnvironmentEquation(
    inverse_mass: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
) ConstraintRowEquation {
    const effective_mass = if (inverse_mass > 0.0001) 1.0 / inverse_mass else 0.0;
    const bias = penetration_depth * 0.25 + @max(0.0, -relative_normal_speed) * 0.05;
    return .{
        .effective_mass = effective_mass,
        .bias = bias,
        .max_impulse = @max(0.5, penetration_depth * 6.0 + @abs(relative_normal_speed) * 0.5),
    };
}

fn buildJointEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    const priority = joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities);
    if (priority <= 0.0 or joint_idx >= joints.len) return .{
        .effective_mass = 0.0,
        .bias = 0.0,
        .max_impulse = 0.0,
    };

    const joint_def = &joints[joint_idx];
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return .{
        .effective_mass = 0.0,
        .bias = 0.0,
        .max_impulse = 0.0,
    };

    const inv_mass_a = instanceInverseMass(&instances[joint_def.entity_a], entities);
    const inv_mass_b = instanceInverseMass(&instances[joint_def.entity_b], entities);
    const total_inv_mass = inv_mass_a + inv_mass_b;
    const effective_mass = if (total_inv_mass > 0.0001) 1.0 / total_inv_mass else 0.0;

    return .{
        .effective_mass = effective_mass,
        .bias = priority * 0.15,
        .max_impulse = @max(0.5, priority * 4.0),
    };
}

fn buildJointDriveEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    const priority = joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities);
    if (priority <= 0.0 or joint_idx >= joints.len) return .{
        .effective_mass = 0.0,
        .bias = 0.0,
        .max_impulse = 0.0,
    };

    const joint_def = &joints[joint_idx];
    if (!joint_def.motor_enabled or joint_def.motor_speed <= 0.0) return .{
        .effective_mass = 0.0,
        .bias = 0.0,
        .max_impulse = 0.0,
    };
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return .{
        .effective_mass = 0.0,
        .bias = 0.0,
        .max_impulse = 0.0,
    };

    const inv_mass_a = instanceInverseMass(&instances[joint_def.entity_a], entities);
    const inv_mass_b = instanceInverseMass(&instances[joint_def.entity_b], entities);
    const total_inv_mass = inv_mass_a + inv_mass_b;
    const effective_mass = if (total_inv_mass > 0.0001) 1.0 / total_inv_mass else 0.0;

    return .{
        .effective_mass = effective_mass,
        .bias = @max(priority * 0.2, joint_def.motor_speed * 0.05),
        .max_impulse = @max(0.5, @max(priority * 3.0, joint_def.motor_max_torque * 0.01)),
    };
}

fn jointHasAnchorRow(joint_def: *const joint.Joint) bool {
    return switch (joint_def.joint_type) {
        .fixed, .hinge, .slider, .spring, .ball_socket => true,
    };
}

fn jointHasLimitRow(joint_def: *const joint.Joint) bool {
    return switch (joint_def.joint_type) {
        .hinge, .slider => @abs(joint_def.limit_max - joint_def.limit_min) < 6.0,
        .spring => true,
        else => false,
    };
}

fn buildJointAnchorEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    return buildJointEquation(instances, joints, joint_idx, entities);
}

fn buildJointLimitEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    const base = buildJointEquation(instances, joints, joint_idx, entities);
    return .{
        .effective_mass = base.effective_mass,
        .bias = base.bias * 1.1,
        .max_impulse = base.max_impulse * 0.9,
    };
}

fn measureContactConstraintMetrics(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ContactConstraintMetrics {
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return null;

    const aabb_a = physics.computeEntityWorldAABB(inst_a, &entities[inst_a.entity_id]) orelse return null;
    const aabb_b = physics.computeEntityWorldAABB(inst_b, &entities[inst_b.entity_id]) orelse return null;
    if (!physics.aabbHit(aabb_a, aabb_b)) return null;

    const manifold = collision.buildAABBContactManifold(aabb_a, aabb_b);
    if (manifold.point_count == 0 or manifold.penetration_depth <= 0.0) return null;

    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_normal_vel = rel_vel_x * manifold.normal_x +
        rel_vel_y * manifold.normal_y +
        rel_vel_z * manifold.normal_z;
    const tangent_x = rel_vel_x - rel_normal_vel * manifold.normal_x;
    const tangent_y = rel_vel_y - rel_normal_vel * manifold.normal_y;
    const tangent_z = rel_vel_z - rel_normal_vel * manifold.normal_z;

    return .{
        .penetration_depth = manifold.penetration_depth,
        .normal_speed = @abs(rel_normal_vel),
        .tangential_speed = @sqrt(tangent_x * tangent_x + tangent_y * tangent_y + tangent_z * tangent_z),
    };
}

fn prepareContactConstraintPair(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ContactPreparedPair {
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return null;

    const aabb_a = physics.computeEntityWorldAABB(inst_a, &entities[inst_a.entity_id]) orelse return null;
    const aabb_b = physics.computeEntityWorldAABB(inst_b, &entities[inst_b.entity_id]) orelse return null;
    if (!physics.aabbHit(aabb_a, aabb_b)) return null;

    const manifold = collision.buildAABBContactManifold(aabb_a, aabb_b);
    if (manifold.point_count == 0 or manifold.penetration_depth <= 0.0) return null;

    const entity_a = &entities[inst_a.entity_id];
    const entity_b = &entities[inst_b.entity_id];
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(entity_a.physics.material),
        entity_a.physics.material,
        material_pairing.getSurfaceForMaterial(entity_b.physics.material),
        entity_b.physics.material,
    );
    const inv_mass_a = instanceInverseMass(inst_a, entities);
    const inv_mass_b = instanceInverseMass(inst_b, entities);
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_normal_vel = rel_vel_x * manifold.normal_x +
        rel_vel_y * manifold.normal_y +
        rel_vel_z * manifold.normal_z;
    const tangent_x = rel_vel_x - rel_normal_vel * manifold.normal_x;
    const tangent_y = rel_vel_y - rel_normal_vel * manifold.normal_y;
    const tangent_z = rel_vel_z - rel_normal_vel * manifold.normal_z;
    const tangential_speed = @sqrt(tangent_x * tangent_x + tangent_y * tangent_y + tangent_z * tangent_z);
    const has_tangent = tangential_speed > 0.0001;
    const tangent_dir_x = if (has_tangent) tangent_x / tangential_speed else 0.0;
    const tangent_dir_y = if (has_tangent) tangent_y / tangential_speed else 0.0;
    const tangent_dir_z = if (has_tangent) tangent_z / tangential_speed else 0.0;
    const prediction_dt: f32 = 1.0 / 60.0;
    const predicted_pos_a = prediction.predictLinearState(.{
        .pos_x = @as(f32, @floatFromInt(inst_a.pos_x)),
        .pos_y = @as(f32, @floatFromInt(inst_a.pos_y)),
        .pos_z = @as(f32, @floatFromInt(inst_a.pos_z)),
        .vel_x = @as(f32, @floatFromInt(inst_a.vel_x)),
        .vel_y = @as(f32, @floatFromInt(inst_a.vel_y)),
        .vel_z = @as(f32, @floatFromInt(inst_a.vel_z)),
    }, prediction_dt);
    const predicted_pos_b = prediction.predictLinearState(.{
        .pos_x = @as(f32, @floatFromInt(inst_b.pos_x)),
        .pos_y = @as(f32, @floatFromInt(inst_b.pos_y)),
        .pos_z = @as(f32, @floatFromInt(inst_b.pos_z)),
        .vel_x = @as(f32, @floatFromInt(inst_b.vel_x)),
        .vel_y = @as(f32, @floatFromInt(inst_b.vel_y)),
        .vel_z = @as(f32, @floatFromInt(inst_b.vel_z)),
    }, prediction_dt);
    const predicted_min_ax = predicted_pos_a.pos_x + @as(f32, @floatFromInt(aabb_a.min_x - inst_a.pos_x));
    const predicted_max_ax = predicted_pos_a.pos_x + @as(f32, @floatFromInt(aabb_a.max_x - inst_a.pos_x));
    const predicted_min_ay = predicted_pos_a.pos_y + @as(f32, @floatFromInt(aabb_a.min_y - inst_a.pos_y));
    const predicted_max_ay = predicted_pos_a.pos_y + @as(f32, @floatFromInt(aabb_a.max_y - inst_a.pos_y));
    const predicted_min_az = predicted_pos_a.pos_z + @as(f32, @floatFromInt(aabb_a.min_z - inst_a.pos_z));
    const predicted_max_az = predicted_pos_a.pos_z + @as(f32, @floatFromInt(aabb_a.max_z - inst_a.pos_z));
    const predicted_min_bx = predicted_pos_b.pos_x + @as(f32, @floatFromInt(aabb_b.min_x - inst_b.pos_x));
    const predicted_max_bx = predicted_pos_b.pos_x + @as(f32, @floatFromInt(aabb_b.max_x - inst_b.pos_x));
    const predicted_min_by = predicted_pos_b.pos_y + @as(f32, @floatFromInt(aabb_b.min_y - inst_b.pos_y));
    const predicted_max_by = predicted_pos_b.pos_y + @as(f32, @floatFromInt(aabb_b.max_y - inst_b.pos_y));
    const predicted_min_bz = predicted_pos_b.pos_z + @as(f32, @floatFromInt(aabb_b.min_z - inst_b.pos_z));
    const predicted_max_bz = predicted_pos_b.pos_z + @as(f32, @floatFromInt(aabb_b.max_z - inst_b.pos_z));
    const predicted_overlap_x = @min(predicted_max_ax, predicted_max_bx) - @max(predicted_min_ax, predicted_min_bx);
    const predicted_overlap_y = @min(predicted_max_ay, predicted_max_by) - @max(predicted_min_ay, predicted_min_by);
    const predicted_overlap_z = @min(predicted_max_az, predicted_max_bz) - @max(predicted_min_az, predicted_min_bz);
    const predicted_penetration_depth = @max(0.0, @min(predicted_overlap_x, @min(predicted_overlap_y, predicted_overlap_z)));
    const normal_row_plan = buildDirectionalRowPlan(
        buildContactNormalEquation(
            inv_mass_a,
            inv_mass_b,
            manifold.penetration_depth,
            @abs(rel_normal_vel),
        ),
        .{
            .dir_x = manifold.normal_x,
            .dir_y = manifold.normal_y,
            .dir_z = manifold.normal_z,
            .depth = manifold.penetration_depth,
            .predicted_depth = predicted_penetration_depth,
        },
        0.75,
        0.15,
        0.5,
    );

    return .{
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .normal = .{
            .dir_x = manifold.normal_x,
            .dir_y = manifold.normal_y,
            .dir_z = manifold.normal_z,
            .depth = manifold.penetration_depth,
            .predicted_depth = normal_row_plan.residual,
        },
        .restitution = response.restitution,
        .friction = response.friction,
        .tangent = .{
            .dir_x = tangent_dir_x,
            .dir_y = tangent_dir_y,
            .dir_z = tangent_dir_z,
            .depth = tangential_speed,
            .predicted_depth = tangential_speed,
        },
        .has_tangent = has_tangent,
        .normal_equation = normal_row_plan.equation,
        .friction_equation = buildContactFrictionEquation(
            inv_mass_a,
            inv_mass_b,
            tangential_speed,
            response.friction,
            normal_row_plan.equation.max_impulse,
        ),
    };
}

fn computeContactSolvePriorityMagnitude(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) f32 {
    const metrics = measureContactConstraintMetrics(inst_a, inst_b, entities) orelse return 0.0;
    return metrics.stress();
}

fn computeMaxActiveContactConstraintMagnitude(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) f32 {
    var max_magnitude: f32 = 0.0;

    for (broadphase_pairs) |pair| {
        if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;
        const inst_a = &s1024.instances[pair.a];
        const inst_b = &s1024.instances[pair.b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        max_magnitude = @max(
            max_magnitude,
            computeContactSolvePriorityMagnitude(inst_a, inst_b, entities),
        );
    }

    return max_magnitude;
}

fn computeContactIterationBudget(broadphase_pairs: []const BroadPhasePair, max_stress: f32) u8 {
    var budget: u8 = if (broadphase_pairs.len >= 16)
        4
    else if (broadphase_pairs.len >= 8)
        3
    else
        2;

    if (max_stress >= 1.0) budget += 1;
    if (max_stress >= 4.0) budget += 1;
    return @min(@as(u8, 6), budget);
}

fn computeContactSettleThreshold(max_stress: f32) f32 {
    if (max_stress >= 4.0) return 0.01;
    if (max_stress >= 1.0) return 0.025;
    return 0.05;
}

fn measureEnvironmentConstraintMetrics(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) ?EnvironmentConstraintMetrics {
    if (instance_idx >= s1024.instance_count) return null;
    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return null;
    if (inst.entity_id >= entities.len) return null;

    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return null;

    const aabb = physics.computeEntityWorldAABB(inst, entity) orelse return null;
    const world_view: query_types.QueryWorldView = .{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const penetration = query_penetration.computePenetrationAABB(
        &world_view,
        @floatFromInt(aabb.min_x),
        @floatFromInt(aabb.min_y),
        @floatFromInt(aabb.min_z),
        @floatFromInt(aabb.max_x),
        @floatFromInt(aabb.max_y),
        @floatFromInt(aabb.max_z),
        .{
            .include_static = false,
            .include_dynamic = false,
            .include_kinematic = false,
            .include_sensors = false,
            .ignore_environment = false,
            .ignore_instance_idx = instance_idx,
        },
    );
    if (!penetration.overlapping or penetration.depth <= 0.0 or penetration.instance_idx != -1) return null;

    const rel_normal_vel =
        @as(f32, @floatFromInt(inst.vel_x)) * penetration.dir_x +
        @as(f32, @floatFromInt(inst.vel_y)) * penetration.dir_y +
        @as(f32, @floatFromInt(inst.vel_z)) * penetration.dir_z;
    const tangent_x = @as(f32, @floatFromInt(inst.vel_x)) - rel_normal_vel * penetration.dir_x;
    const tangent_y = @as(f32, @floatFromInt(inst.vel_y)) - rel_normal_vel * penetration.dir_y;
    const tangent_z = @as(f32, @floatFromInt(inst.vel_z)) - rel_normal_vel * penetration.dir_z;

    return .{
        .penetration_depth = penetration.depth,
        .normal_speed = @abs(rel_normal_vel),
        .tangential_speed = @sqrt(tangent_x * tangent_x + tangent_y * tangent_y + tangent_z * tangent_z),
    };
}

fn prepareEnvironmentConstraint(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) ?PreparedEnvironmentConstraint {
    if (instance_idx >= s1024.instance_count) return null;
    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return null;
    if (inst.entity_id >= entities.len) return null;

    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return null;

    const aabb = physics.computeEntityWorldAABB(inst, entity) orelse return null;
    const world_view: query_types.QueryWorldView = .{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const penetration = query_penetration.computePenetrationAABB(
        &world_view,
        @floatFromInt(aabb.min_x),
        @floatFromInt(aabb.min_y),
        @floatFromInt(aabb.min_z),
        @floatFromInt(aabb.max_x),
        @floatFromInt(aabb.max_y),
        @floatFromInt(aabb.max_z),
        .{
            .include_static = false,
            .include_dynamic = false,
            .include_kinematic = false,
            .include_sensors = false,
            .ignore_environment = false,
            .ignore_instance_idx = instance_idx,
        },
    );
    if (!penetration.overlapping or penetration.depth <= 0.0 or penetration.instance_idx != -1) return null;

    const move_x = @as(i32, @intFromFloat(@round(penetration.dir_x * penetration.depth)));
    const move_y = @as(i32, @intFromFloat(@round(penetration.dir_y * penetration.depth)));
    const move_z = @as(i32, @intFromFloat(@round(penetration.dir_z * penetration.depth)));
    if (move_x == 0 and move_y == 0 and move_z == 0) return null;

    return .{
        .normal = .{
            .dir_x = penetration.dir_x,
            .dir_y = penetration.dir_y,
            .dir_z = penetration.dir_z,
            .depth = penetration.depth,
            .predicted_depth = measurePredictiveEnvironmentDepth(s1024, entities, instance_idx, 1.0 / 60.0),
        },
        .move_x = move_x,
        .move_y = move_y,
        .move_z = move_z,
    };
}

fn measurePredictiveEnvironmentDepth(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    prediction_dt: f32,
) f32 {
    if (instance_idx >= s1024.instance_count) return 0.0;
    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return 0.0;
    if (inst.entity_id >= entities.len) return 0.0;

    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return 0.0;

    const aabb = physics.computeEntityWorldAABB(inst, entity) orelse return 0.0;
    const predicted_state = prediction.predictLinearState(.{
        .pos_x = @as(f32, @floatFromInt(inst.pos_x)),
        .pos_y = @as(f32, @floatFromInt(inst.pos_y)),
        .pos_z = @as(f32, @floatFromInt(inst.pos_z)),
        .vel_x = @as(f32, @floatFromInt(inst.vel_x)),
        .vel_y = @as(f32, @floatFromInt(inst.vel_y)),
        .vel_z = @as(f32, @floatFromInt(inst.vel_z)),
    }, prediction_dt);
    const dx = @as(i32, @intFromFloat(@round(predicted_state.pos_x))) - inst.pos_x;
    const dy = @as(i32, @intFromFloat(@round(predicted_state.pos_y))) - inst.pos_y;
    const dz = @as(i32, @intFromFloat(@round(predicted_state.pos_z))) - inst.pos_z;
    const world_view: query_types.QueryWorldView = .{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const penetration = query_penetration.computePenetrationAABB(
        &world_view,
        @floatFromInt(aabb.min_x + dx),
        @floatFromInt(aabb.min_y + dy),
        @floatFromInt(aabb.min_z + dz),
        @floatFromInt(aabb.max_x + dx),
        @floatFromInt(aabb.max_y + dy),
        @floatFromInt(aabb.max_z + dz),
        .{
            .include_static = false,
            .include_dynamic = false,
            .include_kinematic = false,
            .include_sensors = false,
            .ignore_environment = false,
            .ignore_instance_idx = instance_idx,
        },
    );
    if (!penetration.overlapping or penetration.depth <= 0.0 or penetration.instance_idx != -1) return 0.0;
    return penetration.depth;
}

fn computeEnvironmentSolvePriorityMagnitude(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) f32 {
    const metrics = measureEnvironmentConstraintMetrics(s1024, entities, instance_idx) orelse return 0.0;
    const predicted_depth = measurePredictiveEnvironmentDepth(s1024, entities, instance_idx, 1.0 / 60.0);
    const predictive_gain = makePredictiveConstraintGain(predicted_depth, metrics.penetration_depth, 0.75, 0.2, 0.5);
    return @max(metrics.stress(), predictive_gain.residual_hint);
}

fn computeMaxActiveEnvironmentConstraintMagnitude(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) f32 {
    var max_magnitude: f32 = 0.0;
    var idx: u8 = 0;
    while (idx < s1024.instance_count) : (idx += 1) {
        max_magnitude = @max(max_magnitude, computeEnvironmentSolvePriorityMagnitude(s1024, entities, idx));
    }
    return max_magnitude;
}

fn computeEnvironmentIterationBudget(instance_count: usize, max_stress: f32) u8 {
    var budget: u8 = if (instance_count >= 16)
        4
    else if (instance_count >= 8)
        3
    else
        2;

    if (max_stress >= 1.0) budget += 1;
    if (max_stress >= 4.0) budget += 1;
    return @min(@as(u8, 6), budget);
}

fn computeEnvironmentSettleThreshold(max_stress: f32) f32 {
    if (max_stress >= 4.0) return 0.01;
    if (max_stress >= 1.0) return 0.025;
    return 0.05;
}

fn buildEnvironmentPriorityOrder(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []u8,
) usize {
    var count: usize = 0;
    var magnitudes: [scene32.MAX_INSTANCES]f32 = undefined;

    var idx: u8 = 0;
    while (idx < s1024.instance_count) : (idx += 1) {
        const magnitude = computeEnvironmentSolvePriorityMagnitude(s1024, entities, idx);
        if (magnitude <= settle_threshold) continue;
        out_indices[count] = idx;
        magnitudes[count] = magnitude;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                out_indices[j] > out_indices[best]
            else
                out_indices[j] < out_indices[best];
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_idx = out_indices[i];
            out_indices[i] = out_indices[best];
            out_indices[best] = tmp_idx;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn buildEnvironmentPriorityOrderForIndices(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_indices: []const u8,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []u8,
) usize {
    var count: usize = 0;
    var magnitudes: [scene32.MAX_INSTANCES]f32 = undefined;

    for (instance_indices) |instance_idx| {
        const magnitude = computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx);
        if (magnitude <= settle_threshold) continue;
        out_indices[count] = instance_idx;
        magnitudes[count] = magnitude;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                out_indices[j] > out_indices[best]
            else
                out_indices[j] < out_indices[best];
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_idx = out_indices[i];
            out_indices[i] = out_indices[best];
            out_indices[best] = tmp_idx;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn computeConstraintBlockIterationBudget(joint_stress: f32, contact_stress: f32, environment_stress: f32) u8 {
    const max_stress = @max(joint_stress, @max(contact_stress, environment_stress));
    var active_count: u8 = 0;
    if (joint_stress > 0.0) active_count += 1;
    if (contact_stress > 0.0) active_count += 1;
    if (environment_stress > 0.0) active_count += 1;

    var budget: u8 = 2;
    if (active_count >= 2) budget += 1;
    if (active_count >= 3) budget += 1;
    if (max_stress >= 1.0) budget += 1;
    if (max_stress >= 4.0) budget += 1;
    return @min(@as(u8, 6), budget);
}

fn computeConstraintBlockSettleThreshold(joint_stress: f32, contact_stress: f32, environment_stress: f32) f32 {
    const max_stress = @max(joint_stress, @max(contact_stress, environment_stress));
    if (max_stress >= 4.0) return 0.01;
    if (max_stress >= 1.0) return 0.025;
    return 0.05;
}

fn findConstraintRowStateIndex(
    row_states: []const ConstraintRowState,
    kind: ConstraintRowKind,
    index: usize,
) ?usize {
    for (row_states, 0..) |state, state_idx| {
        if (state.kind == kind and state.index == index) return state_idx;
    }
    return null;
}

fn getOrCreateConstraintRowState(
    row_states: []ConstraintRowState,
    state_count: *usize,
    kind: ConstraintRowKind,
    index: usize,
) *ConstraintRowState {
    if (findConstraintRowStateIndex(row_states[0..state_count.*], kind, index)) |state_idx| {
        return &row_states[state_idx];
    }

    const new_idx = state_count.*;
    row_states[new_idx] = .{
        .kind = kind,
        .index = index,
        .retained_residual = 0.0,
        .accumulated_impulse = 0.0,
        .last_impulse_delta = 0.0,
        .touched_iteration = 0,
    };
    state_count.* += 1;
    return &row_states[new_idx];
}

fn measureConstraintRowCachedPriority(base_priority: f32, state: ?*const ConstraintRowState) f32 {
    if (state == null) return base_priority;
    const cached = state.?;
    return @max(
        base_priority,
        cached.retained_residual * 0.75 + @abs(cached.accumulated_impulse) * 0.25,
    );
}

fn updateConstraintRowState(
    state: *ConstraintRowState,
    exec_result: ConstraintRowExecResult,
    iteration: u8,
) void {
    const applied = if (exec_result.applied_impulse > 0.0)
        exec_result.applied_impulse
    else
        @max(0.0, exec_result.residual_before - exec_result.residual_after);
    state.retained_residual = exec_result.residual_after;
    state.last_impulse_delta = applied;
    state.accumulated_impulse = if (exec_result.changed)
        state.accumulated_impulse * 0.35 + applied * 0.65
    else
        state.accumulated_impulse * 0.5;
    state.touched_iteration = iteration;
}

fn constraintRowWarmImpulse(
    row_state: ?*const ConstraintRowState,
    retention_scale: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
) f32 {
    const state = row_state orelse return 0.0;
    return @min(
        state.accumulated_impulse * retention_scale + equation.bias * equation.effective_mass * bias_scale,
        equation.max_impulse,
    );
}

fn constraintRowSignedCorrection(
    row_state: ?*const ConstraintRowState,
    retention_scale: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    value: f32,
    threshold: f32,
) f32 {
    if (@abs(value) <= threshold) return 0.0;
    return signedCorrection(
        value,
        constraintRowWarmImpulse(row_state, retention_scale, equation, bias_scale),
    );
}

fn constraintRowPlannedSignedCorrection(
    row_state: ?*const ConstraintRowState,
    retention_scale: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    value: f32,
) f32 {
    if (value == 0.0) return 0.0;
    return signedCorrection(
        value,
        constraintRowWarmImpulse(row_state, retention_scale, equation, bias_scale),
    );
}

fn constraintRowSolveMagnitude(
    raw_magnitude: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
) f32 {
    if (raw_magnitude == 0.0) return 0.0;
    const signed_bias = if (raw_magnitude >= 0.0)
        equation.bias * equation.effective_mass * bias_scale
    else
        -equation.bias * equation.effective_mass * bias_scale;
    return @max(-equation.max_impulse, @min(equation.max_impulse, raw_magnitude + signed_bias));
}

fn makePredictiveConstraintGain(predicted_depth: f32, current_depth: f32, residual_scale: f32, bias_scale: f32, impulse_scale: f32) PredictiveConstraintGain {
    const predictive_excess = @max(0.0, predicted_depth - current_depth);
    return .{
        .residual_hint = @max(current_depth, predicted_depth * residual_scale),
        .bias_delta = predictive_excess * bias_scale,
        .impulse_delta = predicted_depth * impulse_scale,
    };
}

fn applyPredictiveConstraintGain(equation: ConstraintRowEquation, gain: PredictiveConstraintGain) ConstraintRowEquation {
    return .{
        .effective_mass = equation.effective_mass,
        .bias = equation.bias + gain.bias_delta,
        .max_impulse = equation.max_impulse + gain.impulse_delta,
    };
}

fn buildDirectionalRowPlan(base_equation: ConstraintRowEquation, direction: PreparedDirectionalConstraint, residual_scale: f32, bias_scale: f32, impulse_scale: f32) DirectionalRowPlan {
    const gain = makePredictiveConstraintGain(
        direction.predicted_depth,
        direction.depth,
        residual_scale,
        bias_scale,
        impulse_scale,
    );
    const equation = applyPredictiveConstraintGain(base_equation, gain);
    return .{
        .residual = @max(gain.residual_hint, equation.bias * 1.25),
        .equation = equation,
    };
}

fn finalizeConstraintRowResult(
    before: f32,
    after: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return .{
        .changed = changed or after < before,
        .residual_before = before,
        .residual_after = after,
        .applied_impulse = @min(
            equation.max_impulse,
            @max(applied_impulse, @max(0.0, before - after) + equation.bias * equation.effective_mass),
        ),
    };
}

fn inactiveConstraintRowResult() ConstraintRowExecResult {
    return .{
        .changed = false,
        .residual_before = 0.0,
        .residual_after = 0.0,
        .applied_impulse = 0.0,
    };
}

fn stalledConstraintRowResult(before: f32) ConstraintRowExecResult {
    return .{
        .changed = false,
        .residual_before = before,
        .residual_after = before,
        .applied_impulse = 0.0,
    };
}

fn measureJointRowResidual(
    kind: ConstraintRowKind,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) f32 {
    if (joint_idx >= joints.len) return 0.0;
    const joint_def = &joints[joint_idx];
    return switch (kind) {
        .joint_anchor => if (jointHasAnchorRow(joint_def))
            joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities)
        else
            0.0,
        .joint_limit => if (jointHasLimitRow(joint_def))
            joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities) * 0.9
        else
            0.0,
        .joint_drive => if (joint_def.motor_enabled and joint_def.motor_speed > 0.0)
            joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities) * 0.75
        else
            0.0,
        else => 0.0,
    };
}

fn measureContactRowResidual(
    kind: ConstraintRowKind,
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) f32 {
    const metrics = measureContactConstraintMetrics(inst_a, inst_b, entities) orelse return 0.0;
    return switch (kind) {
        .contact_normal => @max(metrics.penetration_depth, metrics.normal_speed * 0.25),
        .contact_friction => metrics.tangential_speed * 0.125,
        else => 0.0,
    };
}

fn measureEnvironmentRowResidual(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) f32 {
    return computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx);
}

fn buildConstraintSubsystemOrder(
    joint_stress: f32,
    contact_stress: f32,
    environment_stress: f32,
    reverse_tiebreak: bool,
    out_order: *[3]ConstraintSubsystem,
) usize {
    var count: usize = 0;
    var magnitudes: [3]f32 = undefined;

    if (joint_stress > 0.0) {
        out_order[count] = .joint;
        magnitudes[count] = joint_stress;
        count += 1;
    }
    if (contact_stress > 0.0) {
        out_order[count] = .contact;
        magnitudes[count] = contact_stress;
        count += 1;
    }
    if (environment_stress > 0.0) {
        out_order[count] = .environment;
        magnitudes[count] = environment_stress;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                @intFromEnum(out_order[j]) > @intFromEnum(out_order[best])
            else
                @intFromEnum(out_order[j]) < @intFromEnum(out_order[best]);
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_order = out_order[i];
            out_order[i] = out_order[best];
            out_order[best] = tmp_order;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn instanceHasActiveConstraint(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    instance_idx: u8,
) bool {
    if (instance_idx >= s1024.instance_count) return false;
    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return false;

    if (computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx) > 0.0) return true;

    for (broadphase_pairs) |pair| {
        if (pair.a == instance_idx or pair.b == instance_idx) {
            if (computeMaxActiveContactConstraintMagnitude(s1024, entities, &.{pair}) > 0.0) return true;
        }
    }

    for (joints) |joint_def| {
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a != instance_idx and joint_def.entity_b != instance_idx) continue;
        return true;
    }

    return false;
}

fn buildConstraintInstanceIsland(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    start_idx: u8,
    assigned: []bool,
    out_indices: []u8,
) usize {
    var queue: [scene32.MAX_INSTANCES]u8 = undefined;
    var queue_head: usize = 0;
    var queue_tail: usize = 0;
    var count: usize = 0;

    assigned[start_idx] = true;
    queue[queue_tail] = start_idx;
    queue_tail += 1;

    while (queue_head < queue_tail) : (queue_head += 1) {
        const current = queue[queue_head];
        out_indices[count] = current;
        count += 1;

        for (broadphase_pairs) |pair| {
            var neighbor: ?u8 = null;
            if (pair.a == current) neighbor = pair.b;
            if (pair.b == current) neighbor = pair.a;
            if (neighbor) |instance_idx| {
                if (instance_idx >= s1024.instance_count) continue;
                if (assigned[instance_idx]) continue;
                if (!instanceHasActiveConstraint(s1024, entities, joints, broadphase_pairs, instance_idx)) continue;
                assigned[instance_idx] = true;
                queue[queue_tail] = instance_idx;
                queue_tail += 1;
            }
        }

        for (joints) |joint_def| {
            if (!joint_def.enabled) continue;
            var neighbor: ?u8 = null;
            if (joint_def.entity_a == current) neighbor = @intCast(joint_def.entity_b);
            if (joint_def.entity_b == current) neighbor = @intCast(joint_def.entity_a);
            if (neighbor) |instance_idx| {
                if (instance_idx >= s1024.instance_count) continue;
                if (assigned[instance_idx]) continue;
                if (!instanceHasActiveConstraint(s1024, entities, joints, broadphase_pairs, instance_idx)) continue;
                assigned[instance_idx] = true;
                queue[queue_tail] = instance_idx;
                queue_tail += 1;
            }
        }
    }

    return count;
}

fn buildConstraintIslands(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    island_storage: [][scene32.MAX_INSTANCES]u8,
    island_lengths: []usize,
) usize {
    var assigned: [scene32.MAX_INSTANCES]bool = [_]bool{false} ** scene32.MAX_INSTANCES;
    var island_count: usize = 0;
    var instance_idx: u8 = 0;
    while (instance_idx < s1024.instance_count) : (instance_idx += 1) {
        if (assigned[instance_idx]) continue;
        if (!instanceHasActiveConstraint(s1024, entities, joints, broadphase_pairs, instance_idx)) continue;
        island_lengths[island_count] = buildConstraintInstanceIsland(
            s1024,
            entities,
            joints,
            broadphase_pairs,
            instance_idx,
            assigned[0..],
            island_storage[island_count][0..],
        );
        island_count += 1;
    }
    return island_count;
}

fn islandContainsInstance(instance_indices: []const u8, instance_idx: u8) bool {
    for (instance_indices) |candidate| {
        if (candidate == instance_idx) return true;
    }
    return false;
}

fn copyJointSubsetForIsland(
    joints: []joint.Joint,
    instance_indices: []const u8,
    out_joints: []joint.Joint,
) []joint.Joint {
    var count: usize = 0;
    for (joints) |joint_def| {
        if (!joint_def.enabled) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_a))) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_b))) continue;
        out_joints[count] = joint_def;
        count += 1;
    }
    return out_joints[0..count];
}

fn copyJointIndexSubsetForIsland(
    joints: []joint.Joint,
    instance_indices: []const u8,
    out_indices: []usize,
) []usize {
    var count: usize = 0;
    for (joints, 0..) |joint_def, joint_idx| {
        if (!joint_def.enabled) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_a))) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_b))) continue;
        out_indices[count] = joint_idx;
        count += 1;
    }
    return out_indices[0..count];
}

fn copyBroadPhaseSubsetForIsland(
    broadphase_pairs: []const BroadPhasePair,
    instance_indices: []const u8,
    out_pairs: []BroadPhasePair,
) []BroadPhasePair {
    var count: usize = 0;
    for (broadphase_pairs) |pair| {
        if (!islandContainsInstance(instance_indices, pair.a)) continue;
        if (!islandContainsInstance(instance_indices, pair.b)) continue;
        out_pairs[count] = pair;
        count += 1;
    }
    return out_pairs[0..count];
}

fn computeMaxActiveEnvironmentConstraintMagnitudeForIndices(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_indices: []const u8,
) f32 {
    var max_magnitude: f32 = 0.0;
    for (instance_indices) |instance_idx| {
        max_magnitude = @max(max_magnitude, computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx));
    }
    return max_magnitude;
}

fn computeConstraintIslandStress(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    instance_indices: []const u8,
) f32 {
    var joint_index_storage: [joint.MAX_JOINTS]usize = undefined;
    const joint_indices = copyJointIndexSubsetForIsland(joints, instance_indices, joint_index_storage[0..]);
    var pair_subset_storage: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined;
    const pair_subset = copyBroadPhaseSubsetForIsland(broadphase_pairs, instance_indices, pair_subset_storage[0..]);

    const joint_stress = if (joint_indices.len != 0)
        joint.measureJointSolveStressForIndices(s1024.instances[0..s1024.instance_count], joints, joint_indices, entities)
    else
        0.0;
    const contact_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, pair_subset);
    const environment_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(s1024, entities, instance_indices);
    return @max(joint_stress, @max(contact_stress, environment_stress));
}

fn buildConstraintRowsForIsland(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    joint_indices: []const usize,
    pair_subset: []const BroadPhasePair,
    instance_indices: []const u8,
    row_states: []ConstraintRowState,
    state_count: usize,
    out_rows: []ConstraintRow,
) usize {
    var count: usize = 0;

    for (joint_indices) |joint_idx| {
        const base_priority = measureJointRowResidual(
            .joint_anchor,
            s1024.instances[0..s1024.instance_count],
            joints,
            joint_idx,
            entities,
        );
        if (joint_idx >= joints.len) continue;
        const joint_def = &joints[joint_idx];

        if (jointHasAnchorRow(joint_def)) {
            const anchor_equation = buildJointAnchorEquation(
                s1024.instances[0..s1024.instance_count],
                joints,
                joint_idx,
                entities,
            );
            const anchor_state = if (findConstraintRowStateIndex(row_states[0..state_count], .joint_anchor, joint_idx)) |state_idx|
                &row_states[state_idx]
            else
                null;
            const anchor_priority = measureConstraintRowCachedPriority(base_priority, anchor_state);
            if (anchor_priority > 0.0) {
                out_rows[count] = .{
                    .kind = .joint_anchor,
                    .index = joint_idx,
                    .priority = anchor_priority,
                    .base_residual = base_priority,
                    .equation = anchor_equation,
                };
                count += 1;
            }
        }

        if (jointHasLimitRow(joint_def)) {
            const limit_equation = buildJointLimitEquation(
                s1024.instances[0..s1024.instance_count],
                joints,
                joint_idx,
                entities,
            );
            const limit_state = if (findConstraintRowStateIndex(row_states[0..state_count], .joint_limit, joint_idx)) |state_idx|
                &row_states[state_idx]
            else
                null;
            const limit_priority = measureConstraintRowCachedPriority(base_priority * 0.9, limit_state);
            if (limit_priority > 0.0) {
                out_rows[count] = .{
                    .kind = .joint_limit,
                    .index = joint_idx,
                    .priority = limit_priority,
                    .base_residual = base_priority * 0.9,
                    .equation = limit_equation,
                };
                count += 1;
            }
        }

        const drive_state = if (findConstraintRowStateIndex(row_states[0..state_count], .joint_drive, joint_idx)) |state_idx|
            &row_states[state_idx]
        else
            null;
        if (!joint_def.motor_enabled or joint_def.motor_speed <= 0.0) continue;
        const drive_equation = buildJointDriveEquation(
            s1024.instances[0..s1024.instance_count],
            joints,
            joint_idx,
            entities,
        );
        const drive_priority = measureConstraintRowCachedPriority(base_priority * 0.75, drive_state);
        if (drive_priority <= 0.0) continue;
        out_rows[count] = .{
            .kind = .joint_drive,
            .index = joint_idx,
            .priority = drive_priority,
            .base_residual = base_priority * 0.75,
            .equation = drive_equation,
        };
        count += 1;
    }

    for (pair_subset, 0..) |pair, pair_idx| {
        if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;
        const inst_a = &s1024.instances[pair.a];
        const inst_b = &s1024.instances[pair.b];
        const prepared = prepareContactConstraintPair(inst_a, inst_b, entities) orelse continue;
        const normal_residual = @max(prepared.normal.predicted_depth, prepared.normal_equation.bias * 1.25);

        const normal_state = if (findConstraintRowStateIndex(row_states[0..state_count], .contact_normal, pair_idx)) |state_idx|
            &row_states[state_idx]
        else
            null;
        const normal_priority = measureConstraintRowCachedPriority(
            normal_residual,
            normal_state,
        );
        if (normal_priority > 0.0) {
            out_rows[count] = .{
                .kind = .contact_normal,
                .index = pair_idx,
                .priority = normal_priority,
                .base_residual = normal_residual,
                .equation = prepared.normal_equation,
            };
            count += 1;
        }

        const friction_residual = prepared.tangent.depth * 0.125;
        const friction_state = if (findConstraintRowStateIndex(row_states[0..state_count], .contact_friction, pair_idx)) |state_idx|
            &row_states[state_idx]
        else
            null;
        const friction_priority = measureConstraintRowCachedPriority(friction_residual, friction_state);
        if (friction_priority <= 0.0) continue;
        out_rows[count] = .{
            .kind = .contact_friction,
            .index = pair_idx,
            .priority = friction_priority,
            .base_residual = friction_residual,
            .equation = prepared.friction_equation,
        };
        count += 1;
    }

    for (instance_indices, 0..) |instance_idx, local_idx| {
        const base_priority = measureEnvironmentRowResidual(s1024, entities, instance_idx);
        const environment_equation = blk: {
            if (instance_idx >= s1024.instance_count) break :blk ConstraintRowEquation{
                .effective_mass = 0.0,
                .bias = 0.0,
                .max_impulse = 0.0,
            };
            const inst = &s1024.instances[instance_idx];
            const prepared = prepareEnvironmentConstraint(s1024, entities, instance_idx);
            const metrics = measureEnvironmentConstraintMetrics(s1024, entities, instance_idx);
            if (prepared) |p| {
                if (metrics) |m| {
                    const row_plan = buildDirectionalRowPlan(
                        buildEnvironmentEquation(
                            instanceInverseMass(inst, entities),
                            m.penetration_depth,
                            m.normal_speed,
                        ),
                        p.normal,
                        0.75,
                        0.2,
                        0.5,
                    );
                    break :blk row_plan.equation;
                }
            }
            break :blk ConstraintRowEquation{
                .effective_mass = 0.0,
                .bias = 0.0,
                .max_impulse = 0.0,
            };
        };
        const state = if (findConstraintRowStateIndex(row_states[0..state_count], .environment, local_idx)) |state_idx|
            &row_states[state_idx]
        else
            null;
        const priority = measureConstraintRowCachedPriority(base_priority, state);
        if (priority <= 0.0) continue;
        out_rows[count] = .{
            .kind = .environment,
            .index = local_idx,
            .priority = priority,
            .base_residual = base_priority,
            .equation = environment_equation,
        };
        count += 1;
    }

    return count;
}

fn sortConstraintRows(rows: []ConstraintRow, reverse_tiebreak: bool) void {
    if (rows.len < 2) return;

    var i: usize = 0;
    while (i < rows.len) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < rows.len) : (j += 1) {
            const better_priority = rows[j].priority > rows[best].priority;
            const equal_priority = @abs(rows[j].priority - rows[best].priority) <= 0.0001;
            const lhs_key = (@as(u32, @intFromEnum(rows[j].kind)) << 16) | @as(u32, @intCast(rows[j].index));
            const rhs_key = (@as(u32, @intFromEnum(rows[best].kind)) << 16) | @as(u32, @intCast(rows[best].index));
            const better_tiebreak = if (reverse_tiebreak) lhs_key > rhs_key else lhs_key < rhs_key;
            if (better_priority or (equal_priority and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp = rows[i];
            rows[i] = rows[best];
            rows[best] = tmp;
        }
    }
}

fn sortConstraintIslandsByStress(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    island_storage: []const [scene32.MAX_INSTANCES]u8,
    island_lengths: []const usize,
    island_count: usize,
    out_order: []usize,
) void {
    var island_stress: [scene32.MAX_INSTANCES]f32 = undefined;

    var idx: usize = 0;
    while (idx < island_count) : (idx += 1) {
        out_order[idx] = idx;
        island_stress[idx] = computeConstraintIslandStress(
            s1024,
            entities,
            joints,
            broadphase_pairs,
            island_storage[idx][0..island_lengths[idx]],
        );
    }

    var i: usize = 0;
    while (i < island_count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < island_count) : (j += 1) {
            const better_stress = island_stress[j] > island_stress[best];
            const equal_stress = @abs(island_stress[j] - island_stress[best]) <= 0.0001;
            const better_tiebreak = out_order[j] < out_order[best];
            if (better_stress or (equal_stress and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_order = out_order[i];
            out_order[i] = out_order[best];
            out_order[best] = tmp_order;

            const tmp_stress = island_stress[i];
            island_stress[i] = island_stress[best];
            island_stress[best] = tmp_stress;
        }
    }
}

fn buildContactPriorityOrder(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []usize,
) usize {
    var count: usize = 0;
    var magnitudes: [MAX_BROADPHASE_PAIRS]f32 = undefined;

    for (broadphase_pairs, 0..) |pair, idx| {
        if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;

        const inst_a = &s1024.instances[pair.a];
        const inst_b = &s1024.instances[pair.b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        const magnitude = computeContactSolvePriorityMagnitude(inst_a, inst_b, entities);
        if (magnitude <= settle_threshold) continue;

        out_indices[count] = idx;
        magnitudes[count] = magnitude;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                out_indices[j] > out_indices[best]
            else
                out_indices[j] < out_indices[best];
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_idx = out_indices[i];
            out_indices[i] = out_indices[best];
            out_indices[best] = tmp_idx;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn applyPairVelocityImpulseRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_signed_impulse: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
) ConstraintSolveStep {
    if (raw_signed_impulse == 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    const solve_impulse = constraintRowSolveMagnitude(raw_signed_impulse, equation, bias_scale);
    if (solve_impulse == 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    return .{
        .changed = applyKernelLinearVelocityImpulsePair(
            inst_a,
            inst_b,
            inv_mass_a,
            inv_mass_b,
            dir_x,
            dir_y,
            dir_z,
            solve_impulse,
        ),
        .applied_impulse = @abs(solve_impulse),
    };
}

fn applyContactWarmStartRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    equation: ConstraintRowEquation,
    accumulated_impulse: f32,
) ConstraintSolveStep {
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        normal_x,
        normal_y,
        normal_z,
        @max(0.0, accumulated_impulse),
        equation,
        0.15,
    );
}

fn applyContactPositionRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    penetration_depth: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applyPairDirectionalDisplacementRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        -normal_x,
        -normal_y,
        -normal_z,
        penetration_depth,
        equation,
        0.2,
        0.01,
    );
}

fn applyContactVelocityRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    restitution: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_normal_vel = rel_vel_x * normal_x + rel_vel_y * normal_y + rel_vel_z * normal_z;
    if (rel_normal_vel >= 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    const raw_impulse = -((1.0 + @max(0.0, @min(1.0, restitution))) * rel_normal_vel) / @max(inv_mass_a + inv_mass_b, 0.0001);
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        normal_x,
        normal_y,
        normal_z,
        raw_impulse,
        equation,
        0.2,
    );
}

fn applyContactFrictionRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    friction_coeff: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_normal_vel = rel_vel_x * normal_x + rel_vel_y * normal_y + rel_vel_z * normal_z;
    const tangent_x = rel_vel_x - rel_normal_vel * normal_x;
    const tangent_y = rel_vel_y - rel_normal_vel * normal_y;
    const tangent_z = rel_vel_z - rel_normal_vel * normal_z;
    const tangent_len = @sqrt(tangent_x * tangent_x + tangent_y * tangent_y + tangent_z * tangent_z);
    if (tangent_len <= 0.0001) return .{ .changed = false, .applied_impulse = 0.0 };
    const tx = tangent_x / tangent_len;
    const ty = tangent_y / tangent_len;
    const tz = tangent_z / tangent_len;
    const tangential_speed = rel_vel_x * tx + rel_vel_y * ty + rel_vel_z * tz;
    if (@abs(tangential_speed) <= 0.0001) return .{ .changed = false, .applied_impulse = 0.0 };
    const raw_impulse = @min(@abs(tangential_speed) / @max(inv_mass_a + inv_mass_b, 0.0001), friction_coeff * 8.0);
    const signed_impulse = if (tangential_speed > 0.0) -raw_impulse else raw_impulse;
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        tx,
        ty,
        tz,
        signed_impulse,
        equation,
        0.1,
    );
}

fn applyContactFrictionWarmStartRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangent_x: f32,
    tangent_y: f32,
    tangent_z: f32,
    equation: ConstraintRowEquation,
    accumulated_impulse: f32,
    tangential_speed: f32,
) ConstraintSolveStep {
    if (@abs(tangential_speed) <= 0.0001) return .{ .changed = false, .applied_impulse = 0.0 };
    const signed_impulse = if (tangential_speed > 0.0)
        -@max(0.0, accumulated_impulse)
    else
        @max(0.0, accumulated_impulse);
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        tangent_x,
        tangent_y,
        tangent_z,
        signed_impulse,
        equation,
        0.1,
    );
}

fn solvePreparedContactNormalSteps(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    prepared: ContactPreparedPair,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintSolveStep {
    var changed = false;
    var applied_impulse: f32 = 0.0;
    if (row_state) |_| {
        const warm_impulse = constraintRowWarmImpulse(row_state, 0.35, equation, 0.0);
        const warm_step = applyContactWarmStartRowStep(
            inst_a,
            inst_b,
            prepared.inv_mass_a,
            prepared.inv_mass_b,
            prepared.normal.dir_x,
            prepared.normal.dir_y,
            prepared.normal.dir_z,
            equation,
            warm_impulse,
        );
        applied_impulse += warm_step.applied_impulse;
        changed = warm_step.changed or changed;
    }
    const position_step = applyContactPositionRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.normal.depth,
        equation,
    );
    changed = position_step.changed or changed;
    applied_impulse = @max(applied_impulse, position_step.applied_impulse);
    const velocity_step = applyContactVelocityRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.restitution,
        equation,
    );
    changed = velocity_step.changed or changed;
    applied_impulse = @max(applied_impulse, velocity_step.applied_impulse);
    return .{
        .changed = changed,
        .applied_impulse = applied_impulse,
    };
}

fn solvePreparedContactFrictionSteps(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    prepared: ContactPreparedPair,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintSolveStep {
    if (!prepared.has_tangent) return .{ .changed = false, .applied_impulse = 0.0 };

    var changed = false;
    var applied_impulse: f32 = 0.0;
    if (row_state) |_| {
        const friction_hint = constraintRowWarmImpulse(row_state, 0.35, equation, 0.0);
        const warm_step = applyContactFrictionWarmStartRowStep(
            inst_a,
            inst_b,
            prepared.inv_mass_a,
            prepared.inv_mass_b,
            prepared.tangent.dir_x,
            prepared.tangent.dir_y,
            prepared.tangent.dir_z,
            equation,
            friction_hint,
            prepared.tangent.depth,
        );
        applied_impulse += warm_step.applied_impulse;
        changed = warm_step.changed or changed;
    }
    const friction_step = applyContactFrictionRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.friction,
        equation,
    );
    changed = friction_step.changed or changed;
    applied_impulse = @max(applied_impulse, friction_step.applied_impulse);
    return .{
        .changed = changed,
        .applied_impulse = applied_impulse,
    };
}

fn settleContactRestState(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_y: f32,
) void {
    if (@abs(normal_y) < 0.5) return;

    if (inv_mass_a > 0.0 and @abs(inst_a.vel_x) <= 1 and @abs(inst_a.vel_y) <= 1 and @abs(inst_a.vel_z) <= 1) {
        inst_a.vel_x = 0;
        inst_a.vel_y = 0;
        inst_a.vel_z = 0;
        inst_a.state = .resting;
    }

    if (inv_mass_b > 0.0 and @abs(inst_b.vel_x) <= 1 and @abs(inst_b.vel_y) <= 1 and @abs(inst_b.vel_z) <= 1) {
        inst_b.vel_x = 0;
        inst_b.vel_y = 0;
        inst_b.vel_z = 0;
        inst_b.state = .resting;
    }
}

fn solveContactConstraintsForPairs(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) bool {
    var changed = false;
    if (broadphase_pairs.len == 0) return changed;

    const initial_max_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, broadphase_pairs);
    const settle_threshold = computeContactSettleThreshold(initial_max_stress);
    const iterations = computeContactIterationBudget(broadphase_pairs, initial_max_stress);
    var priority_order: [MAX_BROADPHASE_PAIRS]usize = undefined;
    var iter: u8 = 0;
    while (iter < iterations) : (iter += 1) {
        const active_count = buildContactPriorityOrder(
            s1024,
            entities,
            broadphase_pairs,
            settle_threshold,
            (iter & 1) != 0,
            priority_order[0..],
        );
        if (active_count == 0) break;

        var order_idx: usize = 0;
        while (order_idx < active_count) : (order_idx += 1) {
            const pair = broadphase_pairs[priority_order[order_idx]];
            if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;

            const inst_a = &s1024.instances[pair.a];
            const inst_b = &s1024.instances[pair.b];
            if (inst_a.state == .broken or inst_b.state == .broken) continue;
            const prepared = prepareContactConstraintPair(inst_a, inst_b, entities) orelse continue;
            const normal_step = solvePreparedContactNormalSteps(
                inst_a,
                inst_b,
                prepared,
                prepared.normal_equation,
                null,
            );
            const friction_step = solvePreparedContactFrictionSteps(
                inst_a,
                inst_b,
                prepared,
                prepared.friction_equation,
                null,
            );
            const pair_changed = normal_step.changed or friction_step.changed;
            changed = pair_changed or changed;
            settleAndWakeContactPair(
                inst_a,
                inst_b,
                prepared.inv_mass_a,
                prepared.inv_mass_b,
                prepared.normal.dir_y,
                pair_changed,
            );
        }

        if (computeMaxActiveContactConstraintMagnitude(s1024, entities, broadphase_pairs) <= settle_threshold) break;
    }

    return changed;
}

fn solveContactNormalRow(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    pair: BroadPhasePair,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) return inactiveConstraintRowResult();

    const inst_a = &s1024.instances[pair.a];
    const inst_b = &s1024.instances[pair.b];
    if (inst_a.state == .broken or inst_b.state == .broken) return inactiveConstraintRowResult();
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return inactiveConstraintRowResult();

    const before = base_residual;
    if (before <= 0.0) return inactiveConstraintRowResult();
    const prepared = prepareContactConstraintPair(inst_a, inst_b, entities) orelse return stalledConstraintRowResult(before);
    const solve_step = solvePreparedContactNormalSteps(inst_a, inst_b, prepared, equation, row_state);
    settleAndWakeContactPair(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        prepared.normal.dir_y,
        solve_step.changed,
    );
    return finalizeContactRowResult(.contact_normal, inst_a, inst_b, entities, before, solve_step.changed, solve_step.applied_impulse, equation);
}

fn solveContactFrictionRow(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    pair: BroadPhasePair,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) return inactiveConstraintRowResult();

    const inst_a = &s1024.instances[pair.a];
    const inst_b = &s1024.instances[pair.b];
    if (inst_a.state == .broken or inst_b.state == .broken) return inactiveConstraintRowResult();
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return inactiveConstraintRowResult();
    const before = base_residual;
    if (before <= 0.0) return inactiveConstraintRowResult();
    const prepared = prepareContactConstraintPair(inst_a, inst_b, entities) orelse return stalledConstraintRowResult(before);
    if (!prepared.has_tangent) return stalledConstraintRowResult(before);
    const solve_step = solvePreparedContactFrictionSteps(inst_a, inst_b, prepared, equation, row_state);
    return finalizeContactRowResult(.contact_friction, inst_a, inst_b, entities, before, solve_step.changed, solve_step.applied_impulse, equation);
}

fn solveJointLimitRow(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    if (joint_idx >= joints.len) return inactiveConstraintRowResult();

    const joint_def = &joints[joint_idx];
    if (!jointHasLimitRow(joint_def)) return inactiveConstraintRowResult();
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return inactiveConstraintRowResult();

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return inactiveConstraintRowResult();

    const before = base_residual;
    if (before <= 0.0) return inactiveConstraintRowResult();

    const mass_data = computeJointMassData(inst_a, inst_b, entities) orelse return stalledConstraintRowResult(before);
    var changed = false;
    var applied_impulse: f32 = 0.0;

    switch (joint_def.joint_type) {
        .hinge => {
            const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
            const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
            const angle_a = getKernelJointAngle(inst_a, joint_def);
            const angle_b = getKernelJointAngle(inst_b, joint_def);
            const relative_angle = angle_b - angle_a;
            const clamped_relative = @max(min_angle, @min(max_angle, relative_angle));
            const angle_error = relative_angle - clamped_relative;
            const correction = constraintRowSignedCorrection(row_state, 0.1, equation, 1.0, angle_error, 0.0001);
            const step = applyJointAngularScalarCorrection(inst_a, inst_b, joint_def, mass_data, angle_a, angle_b, correction, equation);
            changed = step.changed or changed;
            applied_impulse = @max(applied_impulse, step.applied_impulse);
        },
        .slider => {
            const axis = getKernelJointAxisVector(joint_def) orelse {
                return finalizeJointRowNoChange(
                    .joint_limit,
                    instances,
                    joints,
                    joint_idx,
                    entities,
                    before,
                    equation,
                );
            };
            const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
            const projection = projectJointDeltaToAxis(delta, axis);
            const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
            const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
            const clamped_along = @max(min_limit, @min(max_limit, projection.along));
            const axis_error = projection.along - clamped_along;
            const correction = constraintRowSignedCorrection(row_state, 0.1, equation, 1.0, axis_error, 0.0001);
            const step = applyJointAxisScalarCorrection(inst_a, inst_b, mass_data, axis, correction, equation);
            changed = step.changed or changed;
            applied_impulse = @max(applied_impulse, step.applied_impulse);
        },
        .spring, .fixed, .ball_socket => {},
    }

    return finalizeJointRowResult(
        .joint_limit,
        instances,
        joints,
        joint_idx,
        entities,
        inst_a,
        inst_b,
        before,
        changed,
        applied_impulse,
        equation,
    );
}

fn solveJointAnchorRow(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    if (joint_idx >= joints.len) return inactiveConstraintRowResult();

    const joint_def = &joints[joint_idx];
    if (!jointHasAnchorRow(joint_def)) return inactiveConstraintRowResult();
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return inactiveConstraintRowResult();

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return inactiveConstraintRowResult();

    const before = base_residual;
    if (before <= 0.0) return inactiveConstraintRowResult();

    const mass_data = computeJointMassData(inst_a, inst_b, entities) orelse return stalledConstraintRowResult(before);
    const warm_impulse = constraintRowWarmImpulse(row_state, 0.25, equation, 1.0);

    var changed = false;
    var applied_impulse: f32 = warm_impulse;
    const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);

    switch (joint_def.joint_type) {
        .fixed, .ball_socket, .hinge => {
            if (lengthAndNormal(delta)) |distance| {
                const step = applyJointAnchorDistanceConstraint(
                    inst_a,
                    inst_b,
                    mass_data,
                    joint_def,
                    distance.nx,
                    distance.ny,
                    distance.nz,
                    distance.len,
                    warm_impulse,
                    equation,
                );
                changed = step.changed or changed;
                applied_impulse = @max(applied_impulse, step.applied_impulse);
            }
        },
        .slider => {
            const axis = getKernelJointAxisVector(joint_def) orelse return stalledConstraintRowResult(before);
            const projection = projectJointDeltaToAxis(delta, axis);
            if (projection.perp_len > 0.0001) {
                const nx = projection.perp_x / projection.perp_len;
                const ny = projection.perp_y / projection.perp_len;
                const nz = projection.perp_z / projection.perp_len;
                const step = applyJointAnchorDistanceConstraint(
                    inst_a,
                    inst_b,
                    mass_data,
                    joint_def,
                    nx,
                    ny,
                    nz,
                    projection.perp_len,
                    warm_impulse,
                    equation,
                );
                changed = step.changed or changed;
                applied_impulse = @max(applied_impulse, step.applied_impulse);
            }
        },
        .spring => {
            if (lengthAndNormal(delta)) |distance| {
                if (distance.len > 0.001) {
                    const rest_length = @max(0.0, joint_def.limit_max);
                    const extension = distance.len - rest_length;
                    const stiffness = @max(0.0, joint_def.stiffness) * 0.01;
                    const rel_speed = measureKernelRelativeLinearSpeed(inst_a, inst_b, distance.nx, distance.ny, distance.nz);
                    const prediction_dt = 1.0 / 60.0;
                    const predicted_extension = extension + rel_speed * prediction_dt;
                    const control_extension = if (extension * predicted_extension < 0.0)
                        predicted_extension
                    else if (@abs(predicted_extension) < @abs(extension))
                        predicted_extension
                    else
                        extension;
                    const correction_mag = control_extension * stiffness;
                    const step = applyJointDirectionalConstraint(
                        inst_a,
                        inst_b,
                        mass_data,
                        distance.nx,
                        distance.ny,
                        distance.nz,
                        correction_mag,
                        warm_impulse,
                        @max(0.0, joint_def.damping) * 0.001,
                        equation,
                    );
                    changed = step.changed or changed;
                    applied_impulse = @max(applied_impulse, step.applied_impulse);
                }
            }
        },
    }

    if (joint_def.joint_type == .hinge) {
        changed = applyKernelAngularLimitVelocityDamping(inst_a, inst_b, joint_def, mass_data) or changed;
    } else if (joint_def.joint_type == .slider) {
        if (getKernelJointAxisVector(joint_def)) |axis| {
            const current_delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
            const along = current_delta.x * axis.x + current_delta.y * axis.y + current_delta.z * axis.z;
            const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
            const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
            changed = applyKernelSliderLimitVelocityDamping(
                inst_a,
                inst_b,
                mass_data,
                axis.x,
                axis.y,
                axis.z,
                along,
                min_limit,
                max_limit,
            ) or changed;
        }
    }

    return finalizeJointRowResult(
        .joint_anchor,
        instances,
        joints,
        joint_idx,
        entities,
        inst_a,
        inst_b,
        before,
        changed,
        applied_impulse,
        equation,
    );
}

fn solveJointDriveRow(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    if (joint_idx >= joints.len) return inactiveConstraintRowResult();

    const joint_def = &joints[joint_idx];
    if (!joint_def.motor_enabled or joint_def.motor_speed <= 0.0) return inactiveConstraintRowResult();
    if (joint_def.joint_type != .hinge and joint_def.joint_type != .slider) return inactiveConstraintRowResult();
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return inactiveConstraintRowResult();

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return inactiveConstraintRowResult();

    const before = base_residual;
    if (before <= 0.0) return inactiveConstraintRowResult();

    const mass_data = computeJointMassData(inst_a, inst_b, entities) orelse return stalledConstraintRowResult(before);
    const drive_state = measureKernelJointDriveState(inst_a, inst_b, joint_def) orelse return stalledConstraintRowResult(before);
    const min_step: f32 = switch (joint_def.joint_type) {
        .hinge => 0.001,
        .slider => if (@abs(drive_state.position - joint_def.motor_target) >= 1.0) 1.0 else 0.001,
        else => 0.001,
    };
    const drive_plan = computeKernelJointDrivePlan(joint_def, drive_state, min_step) orelse return stalledConstraintRowResult(before);
    const warm_impulse = constraintRowWarmImpulse(row_state, 0.35, equation, 1.0);

    var changed = false;
    const applied_impulse: f32 = @max(warm_impulse, @abs(drive_plan.signed_step));
    const signed_step = constraintRowPlannedSignedCorrection(row_state, 0.35, equation, 1.0, drive_plan.signed_step);

    switch (joint_def.joint_type) {
        .hinge => {
            const current_a = getKernelJointAngle(inst_a, joint_def);
            const current_b = getKernelJointAngle(inst_b, joint_def);
            const step = applyJointAngularScalarCorrection(inst_a, inst_b, joint_def, mass_data, current_a, current_b, -signed_step, equation);
            changed = step.changed or changed;
            changed = applyKernelAngularMotorVelocityBias(
                inst_a,
                inst_b,
                joint_def,
                mass_data,
                drive_plan.desired_velocity,
            ) or changed;
        },
        .slider => {
            const axis = getKernelJointAxisVector(joint_def) orelse return stalledConstraintRowResult(before);
            const step = applyJointAxisScalarCorrection(inst_a, inst_b, mass_data, axis, -signed_step, equation);
            changed = step.changed or changed;
            changed = applyKernelLinearMotorVelocityBias(
                inst_a,
                inst_b,
                joint_def,
                mass_data,
                axis.x,
                axis.y,
                axis.z,
                drive_plan.desired_velocity,
            ) or changed;
        },
        else => {},
    }

    return finalizeJointRowResult(
        .joint_drive,
        instances,
        joints,
        joint_idx,
        entities,
        inst_a,
        inst_b,
        before,
        changed,
        applied_impulse,
        equation,
    );
}

pub fn solveContactConstraints(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) bool {
    return solveContactConstraintsForPairs(s1024, entities, broadphase_pairs);
}

fn solveEnvironmentConstraintsForIndices(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_indices: []const u8,
) bool {
    var changed = false;
    const initial_max_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(s1024, entities, instance_indices);
    const settle_threshold = computeEnvironmentSettleThreshold(initial_max_stress);
    const iterations = computeEnvironmentIterationBudget(instance_indices.len, initial_max_stress);
    var priority_order: [scene32.MAX_INSTANCES]u8 = undefined;
    var iter: u8 = 0;
    while (iter < iterations) : (iter += 1) {
        const active_count = buildEnvironmentPriorityOrder(
            s1024,
            entities,
            settle_threshold,
            (iter & 1) != 0,
            priority_order[0..],
        );
        if (active_count == 0) break;

        const subset_active_count = buildEnvironmentPriorityOrderForIndices(
            s1024,
            entities,
            instance_indices,
            settle_threshold,
            (iter & 1) != 0,
            priority_order[0..],
        );
        if (subset_active_count == 0) break;

        var order_idx: usize = 0;
        while (order_idx < subset_active_count) : (order_idx += 1) {
            const i = priority_order[order_idx];
            const inst = &s1024.instances[i];
            if (inst.state == .broken) continue;
            if (inst.entity_id >= entities.len) continue;

            const entity = &entities[inst.entity_id];
            if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) continue;
            const prepared = prepareEnvironmentConstraint(s1024, entities, i) orelse continue;

            const previous_vel_x = inst.vel_x;
            const previous_vel_y = inst.vel_y;
            const previous_vel_z = inst.vel_z;
            const moved = applyEnvironmentResolvedMotion(
                inst,
                entity,
                prepared.move_x,
                prepared.move_y,
                prepared.move_z,
                previous_vel_x,
                previous_vel_y,
                previous_vel_z,
            );
            if (moved) wakeInstance(inst);
            changed = moved or changed;
        }

        if (computeMaxActiveEnvironmentConstraintMagnitudeForIndices(s1024, entities, instance_indices) <= settle_threshold) break;
    }

    return changed;
}

fn solveEnvironmentRow(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    if (instance_idx >= s1024.instance_count) return inactiveConstraintRowResult();

    const before = base_residual;
    if (before <= 0.0) return inactiveConstraintRowResult();

    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return stalledConstraintRowResult(before);
    if (inst.entity_id >= entities.len) return stalledConstraintRowResult(before);

    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return stalledConstraintRowResult(before);
    const prepared = prepareEnvironmentConstraint(s1024, entities, instance_idx) orelse return stalledConstraintRowResult(before);

    const previous_vel_x = inst.vel_x;
    const previous_vel_y = inst.vel_y;
    const previous_vel_z = inst.vel_z;
    var applied_impulse: f32 = 0.0;

    if (row_state) |_| {
        const warm_move = @min(
            constraintRowWarmImpulse(row_state, 0.25, equation, 1.0),
            @min(prepared.normal.depth * 0.5, equation.max_impulse),
        );
        const warm_step = applyEnvironmentWarmStartDisplacement(
            inst,
            prepared.normal.dir_x,
            prepared.normal.dir_y,
            prepared.normal.dir_z,
            warm_move,
            equation,
        );
        applied_impulse += warm_step.applied_impulse;
    }

    const solve_step = applyEnvironmentSolveDisplacement(
        inst,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.normal.depth,
        equation,
    );
    applied_impulse = @max(applied_impulse, solve_step.applied_impulse);

    const solve_move_x = @as(i32, @intFromFloat(@round(prepared.normal.dir_x * solve_step.applied_impulse)));
    const solve_move_y = @as(i32, @intFromFloat(@round(prepared.normal.dir_y * solve_step.applied_impulse)));
    const solve_move_z = @as(i32, @intFromFloat(@round(prepared.normal.dir_z * solve_step.applied_impulse)));

    _ = applyEnvironmentResolvedMotion(
        inst,
        entity,
        solve_move_x,
        solve_move_y,
        solve_move_z,
        previous_vel_x,
        previous_vel_y,
        previous_vel_z,
    );
    return finalizeEnvironmentRowResult(
        s1024,
        entities,
        instance_idx,
        inst,
        before,
        solve_step.changed,
        applied_impulse,
        equation,
    );
}

pub fn solveEnvironmentConstraints(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    var all_indices: [scene32.MAX_INSTANCES]u8 = undefined;
    var count: usize = 0;
    var idx: u8 = 0;
    while (idx < s1024.instance_count) : (idx += 1) {
        all_indices[count] = idx;
        count += 1;
    }
    return solveEnvironmentConstraintsForIndices(s1024, entities, all_indices[0..count]);
}

pub fn solveConstraintBlock(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
) bool {
    var changed = false;
    var island_storage: [scene32.MAX_INSTANCES][scene32.MAX_INSTANCES]u8 = undefined;
    var island_lengths: [scene32.MAX_INSTANCES]usize = [_]usize{0} ** scene32.MAX_INSTANCES;
    const island_count = buildConstraintIslands(
        s1024,
        entities,
        joints,
        broadphase_pairs,
        island_storage[0..],
        island_lengths[0..],
    );
    if (island_count == 0) return false;

    var island_order: [scene32.MAX_INSTANCES]usize = undefined;
    sortConstraintIslandsByStress(
        s1024,
        entities,
        joints,
        broadphase_pairs,
        island_storage[0..island_count],
        island_lengths[0..island_count],
        island_count,
        island_order[0..island_count],
    );

    var island_order_idx: usize = 0;
    while (island_order_idx < island_count) : (island_order_idx += 1) {
        const island_idx = island_order[island_order_idx];
        const instance_indices = island_storage[island_idx][0..island_lengths[island_idx]];

        var joint_index_storage: [joint.MAX_JOINTS]usize = undefined;
        const joint_indices = copyJointIndexSubsetForIsland(joints, instance_indices, joint_index_storage[0..]);
        var pair_subset_storage: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined;
        const pair_subset = copyBroadPhaseSubsetForIsland(broadphase_pairs, instance_indices, pair_subset_storage[0..]);

        const initial_joint_stress = if (joint_indices.len != 0)
            joint.measureJointSolveStressForIndices(s1024.instances[0..s1024.instance_count], joints, joint_indices, entities)
        else
            0.0;
        const initial_contact_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, pair_subset);
        const initial_environment_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(
            s1024,
            entities,
            instance_indices,
        );
        const iterations = computeConstraintBlockIterationBudget(
            initial_joint_stress,
            initial_contact_stress,
            initial_environment_stress,
        );
        const settle_threshold = computeConstraintBlockSettleThreshold(
            initial_joint_stress,
            initial_contact_stress,
            initial_environment_stress,
        );

        var iter: u8 = 0;
        var row_state_storage: [joint.MAX_JOINTS * 3 + MAX_BROADPHASE_PAIRS * 2 + scene32.MAX_INSTANCES]ConstraintRowState = undefined;
        var row_state_count: usize = 0;
        while (iter < iterations) : (iter += 1) {
            var iter_changed = false;
            var iter_improved = false;

            const joint_stress = if (joint_indices.len != 0)
                joint.measureJointSolveStressForIndices(s1024.instances[0..s1024.instance_count], joints, joint_indices, entities)
            else
                0.0;
            const contact_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, pair_subset);
            const environment_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(
                s1024,
                entities,
                instance_indices,
            );
            const iter_max_stress = @max(joint_stress, @max(contact_stress, environment_stress));
            if (iter_max_stress <= settle_threshold) break;

            var row_storage: [joint.MAX_JOINTS * 3 + MAX_BROADPHASE_PAIRS * 2 + scene32.MAX_INSTANCES]ConstraintRow = undefined;
            const row_count = buildConstraintRowsForIsland(
                s1024,
                entities,
                joints,
                joint_indices,
                pair_subset,
                instance_indices,
                row_state_storage[0..],
                row_state_count,
                row_storage[0..],
            );
            if (row_count == 0) break;
            sortConstraintRows(row_storage[0..row_count], (iter & 1) != 0);

            var iter_max_before: f32 = 0.0;
            var iter_max_after: f32 = 0.0;

            for (row_storage[0..row_count]) |row| {
                const row_state = getOrCreateConstraintRowState(
                    row_state_storage[0..],
                    &row_state_count,
                    row.kind,
                    row.index,
                );
                const exec_result = switch (row.kind) {
                    .joint_anchor => blk: {
                        break :blk solveJointAnchorRow(
                            s1024.instances[0..s1024.instance_count],
                            joints,
                            row.index,
                            entities,
                            row.base_residual,
                            row.equation,
                            row_state,
                        );
                    },
                    .joint_limit => blk: {
                        break :blk solveJointLimitRow(
                            s1024.instances[0..s1024.instance_count],
                            joints,
                            row.index,
                            entities,
                            row.base_residual,
                            row.equation,
                            row_state,
                        );
                    },
                    .joint_drive => blk: {
                        break :blk solveJointDriveRow(
                            s1024.instances[0..s1024.instance_count],
                            joints,
                            row.index,
                            entities,
                            row.base_residual,
                            row.equation,
                            row_state,
                        );
                    },
                    .contact_normal => solveContactNormalRow(s1024, entities, pair_subset[row.index], row.base_residual, row.equation, row_state),
                    .contact_friction => solveContactFrictionRow(s1024, entities, pair_subset[row.index], row.base_residual, row.equation, row_state),
                    .environment => solveEnvironmentRow(
                        s1024,
                        entities,
                        instance_indices[row.index],
                        row.base_residual,
                        row.equation,
                        row_state,
                    ),
                };

                updateConstraintRowState(row_state, exec_result, iter);
                iter_changed = exec_result.changed or iter_changed;
                iter_improved = exec_result.improved() or iter_improved;
                iter_max_before = @max(iter_max_before, exec_result.residual_before);
                iter_max_after = @max(iter_max_after, exec_result.residual_after);
            }

            changed = iter_changed or changed;
            if (!iter_changed) break;
            if (iter_max_after <= settle_threshold) break;
            if (!iter_improved and iter_max_after + 0.0001 >= iter_max_before) break;
        }
    }

    return changed;
}

pub fn runConstraintStage(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    out_pairs: []BroadPhasePair,
) ConstraintStageResult {
    const pair_count = collectBroadPhasePairs(
        s1024.instances[0..s1024.instance_count],
        entities,
        out_pairs,
    );
    const changed = solveConstraintBlock(
        s1024,
        entities,
        joints,
        out_pairs[0..pair_count],
    );
    return .{
        .pair_count = pair_count,
        .changed = changed,
    };
}

pub fn runPreMotionConstraintStage(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    out_pairs: []BroadPhasePair,
    time_scale: f32,
    sleep_time_threshold: u8,
    dt: f32,
) ConstraintStageResult {
    runPreStepSystems(s1024, entities, time_scale, sleep_time_threshold, dt);
    return runConstraintStage(s1024, entities, joints, out_pairs);
}

pub fn runPostMotionConstraintStage(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    out_pairs: []BroadPhasePair,
) ConstraintStageResult {
    rebuildOccupancyIfNeeded(s1024, entities, true);
    const result = runConstraintStage(s1024, entities, joints, out_pairs);
    rebuildOccupancyIfNeeded(s1024, entities, result.changed);
    return result;
}

pub fn mergeObservedPairCount(previous_pair_count: usize, current_pair_count: usize) usize {
    return @max(previous_pair_count, current_pair_count);
}

pub fn finishWorldStep(
    queue: *collision_event.PendingCollisionQueue,
    world_bus: ?*bus.Bus,
    tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    debris_dt: f32,
) void {
    destruction.updateDebris(debris_dt);
    publishPendingCollisions(queue, world_bus, tick);
    recordWorldSnapshot(tick, s1024, entities);
}

pub fn initCoreSubsystems() void {
    kcc.init();
    vehicle.init();
    ragdoll.init();
    ballistics.init();
    destruction.init();
    rewind.init();
}

test "computeContactIterationBudget increases for higher contact stress" {
    try std.testing.expectEqual(@as(u8, 2), computeContactIterationBudget(&.{}, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeContactIterationBudget(&.{.{ .a = 0, .b = 1 }}, 1.5));
    try std.testing.expectEqual(@as(u8, 4), computeContactIterationBudget(&.{.{ .a = 0, .b = 1 }}, 5.0));
}

test "computeEnvironmentIterationBudget increases for higher environment stress" {
    try std.testing.expectEqual(@as(u8, 2), computeEnvironmentIterationBudget(1, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeEnvironmentIterationBudget(1, 1.5));
    try std.testing.expectEqual(@as(u8, 4), computeEnvironmentIterationBudget(1, 5.0));
}

test "computeConstraintBlockIterationBudget increases with active stressed subsystems" {
    try std.testing.expectEqual(@as(u8, 2), computeConstraintBlockIterationBudget(0.0, 0.0, 0.0));
    try std.testing.expectEqual(@as(u8, 2), computeConstraintBlockIterationBudget(0.5, 0.0, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeConstraintBlockIterationBudget(0.5, 0.5, 0.0));
    try std.testing.expectEqual(@as(u8, 6), computeConstraintBlockIterationBudget(5.0, 5.0, 5.0));
}

test "buildContactPriorityOrder uses stable tiebreak when contact stress matches" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var wide = entity16.initEntity16();
    wide.physics.mass = 10;
    wide.physics.material = .solid;
    entity16.setVoxel(&wide, 0, 0, 0);
    entity16.setVoxel(&wide, 1, 0, 0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ wide, wide, voxel, voxel };
    s1024.instance_count = 4;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
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
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 11,
        .pos_y = 10,
        .pos_z = 10,
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
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 20,
        .pos_y = 20,
        .pos_z = 20,
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
    s1024.instances[3] = .{
        .entity_id = 3,
        .pos_x = 20,
        .pos_y = 20,
        .pos_z = 20,
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

    const pairs = [_]BroadPhasePair{
        .{ .a = 2, .b = 3 },
        .{ .a = 0, .b = 1 },
    };
    var order: [2]usize = undefined;

    const count = buildContactPriorityOrder(
        &s1024,
        entities[0..],
        pairs[0..],
        0.0,
        false,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 0), order[0]);
    try std.testing.expectEqual(@as(usize, 1), order[1]);
}

test "buildEnvironmentPriorityOrder uses stable tiebreak when environment stress matches" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 10,
        .lz = 10,
    }), true);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 20,
        .ly = 20,
        .lz = 20,
    }), true);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
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
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 20,
        .pos_y = 20,
        .pos_z = 20,
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

    var order: [scene32.MAX_INSTANCES]u8 = undefined;
    const count = buildEnvironmentPriorityOrder(
        &s1024,
        entities[0..],
        0.0,
        false,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(u8, 0), order[0]);
    try std.testing.expectEqual(@as(u8, 1), order[1]);
}

test "buildConstraintSubsystemOrder prioritizes highest stress subsystem first" {
    var order: [3]ConstraintSubsystem = undefined;

    const count = buildConstraintSubsystemOrder(
        0.5,
        3.0,
        1.0,
        false,
        &order,
    );

    try std.testing.expectEqual(@as(usize, 3), count);
    try std.testing.expectEqual(ConstraintSubsystem.contact, order[0]);
    try std.testing.expectEqual(ConstraintSubsystem.environment, order[1]);
    try std.testing.expectEqual(ConstraintSubsystem.joint, order[2]);
}

test "sortConstraintIslandsByStress prioritizes higher stress island first" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 10,
        .lz = 10,
    }), true);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel, voxel };
    s1024.instance_count = 3;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
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
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 40,
        .pos_y = 40,
        .pos_z = 40,
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
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 40,
        .pos_y = 40,
        .pos_z = 40,
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

    const broadphase_pairs = [_]BroadPhasePair{
        .{ .a = 1, .b = 2 },
    };
    var joints = [_]joint.Joint{};
    const island_storage = [_][scene32.MAX_INSTANCES]u8{
        blk: {
            var row: [scene32.MAX_INSTANCES]u8 = [_]u8{0} ** scene32.MAX_INSTANCES;
            row[0] = 0;
            break :blk row;
        },
        blk: {
            var row: [scene32.MAX_INSTANCES]u8 = [_]u8{0} ** scene32.MAX_INSTANCES;
            row[0] = 1;
            row[1] = 2;
            break :blk row;
        },
    };
    const island_lengths = [_]usize{ 1, 2 };
    var island_order: [2]usize = undefined;

    sortConstraintIslandsByStress(
        &s1024,
        entities[0..],
        joints[0..],
        broadphase_pairs[0..],
        island_storage[0..],
        island_lengths[0..],
        2,
        island_order[0..],
    );

    try std.testing.expectEqual(@as(usize, 0), island_order[0]);
    try std.testing.expectEqual(@as(usize, 1), island_order[1]);
}
