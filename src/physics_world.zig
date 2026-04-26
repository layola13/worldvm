//! Physics World - Unified Coordinator for All Physics Subsystems
//!
//! P0: Unified PhysicsWorld Skeleton
//! Coordinates KCC, Vehicle, Ragdoll, Ballistics, Joints in a single pipeline

const std = @import("std");
const address = @import("address.zig");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const bus = @import("bus.zig");
const terrain = @import("terrain.zig");
const material_pairing = @import("material_pairing.zig");
const destruction = @import("destruction.zig");
const contact_response = @import("contact_response.zig");
const collision_event = @import("collision_event.zig");
const sleep_response = @import("sleep_response.zig");
const physics_kernel = @import("physics_kernel.zig");
const joint = @import("joint.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const ragdoll = @import("ragdoll.zig");
const ballistics = @import("ballistics.zig");
const rewind = @import("rewind.zig");
const query = @import("query.zig");

pub const FIXED_DT: f32 = 1.0 / 60.0;
pub const SLEEP_TIME_THRESHOLD: u8 = 30;
pub const MAX_BROADPHASE_PAIRS: usize = physics_kernel.MAX_BROADPHASE_PAIRS;
pub const BroadPhasePair = physics_kernel.BroadPhasePair;

pub const StepConfig = struct {
    dt: f32 = FIXED_DT,
    time_scale: f32 = 1.0,
    run_pre_motion_constraint: bool = true,
    apply_continuous_physics: bool = false,
};

pub const StepResult = struct {
    changed: bool,
    pair_count: usize,
};

/// PhysicsWorld - bundles all physics subsystem handles
pub const PhysicsWorld = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    joint_count: usize,
    tick: u32,
    world_bus: ?*bus.Bus,
    pending_collisions: collision_event.PendingCollisionQueue,
    broadphase_pairs: [MAX_BROADPHASE_PAIRS]BroadPhasePair,
    broadphase_pair_count: usize,
};

fn queueCollisionPairEvent(world: *PhysicsWorld, inst: *const scene32.Instance, impact_velocity: i16, blocker_id: u8) void {
    physics_kernel.enqueueCollisionPairEvent(
        &world.pending_collisions,
        world.s1024.instances[0..world.s1024.instance_count],
        world.entities,
        inst,
        impact_velocity,
        blocker_id,
    );
}

fn applyBreakState(world: *PhysicsWorld, instance_idx: u8, impact_velocity: i16) bool {
    return physics_kernel.applyBreakStateAndWake(world.s1024, world.entities, instance_idx, impact_velocity, 0.1);
}

fn applyCollisionBreakState(world: *PhysicsWorld, instance_idx: u8, impact_velocity: i16, blocker_id: u8) bool {
    return physics_kernel.applyCollisionBreakPairAndWake(world.s1024, world.entities, instance_idx, impact_velocity, blocker_id, 0.1);
}

fn processBlockedIntegrateSweep(
    world: *PhysicsWorld,
    instance_idx: u8,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
    broke_this_tick: *bool,
    topology_changed: *bool,
) void {
    broke_this_tick.* = applyCollisionBreakState(world, instance_idx, impact_velocity, blocker_id) or broke_this_tick.*;
    topology_changed.* = broke_this_tick.* or topology_changed.*;
    queueCollisionPairEvent(world, inst, impact_velocity, blocker_id);
}

fn processPlanarIntegrateSweep(
    world: *PhysicsWorld,
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    axis: physics.SweepAxis,
    start_x: i32,
    start_y: i32,
    start_z: i32,
    vel: i16,
    next_axis_pos: *i32,
    next_state: *scene32.InstanceState,
    broke_this_tick: *bool,
    topology_changed: *bool,
) void {
    if (vel == 0) return;

    const sweep = physics_kernel.sweepMotionAlongAxis(
        world.s1024,
        inst,
        world.entities,
        start_x,
        start_y,
        start_z,
        vel,
        axis,
    );

    switch (axis) {
        .x => if (sweep.pos_x != inst.pos_x) {
            next_axis_pos.* = sweep.pos_x;
            next_state.* = .moving;
        },
        .z => if (sweep.pos_z != inst.pos_z) {
            next_axis_pos.* = sweep.pos_z;
            next_state.* = .moving;
        },
        else => return,
    }

    if (sweep.blocked) {
        processBlockedIntegrateSweep(world, instance_idx, inst, vel, sweep.blocker_id, broke_this_tick, topology_changed);
        physics_kernel.handleBlockedLateralContact(inst, entity, sweep.blocker_id, if (axis == .x) .x else .z);
    }
}

fn processVerticalIntegrateMotion(
    world: *PhysicsWorld,
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    next_y: *i32,
    next_state: *scene32.InstanceState,
    broke_this_tick: *bool,
    topology_changed: *bool,
) void {
    if (inst.vel_y < 0) {
        const fall = physics.checkContinuousFall(world.s1024, inst, world.entities);
        if (fall.can_fall) {
            next_y.* = fall.target_y;
            next_state.* = .falling;
        } else if (fall.blocked) {
            next_y.* = fall.target_y;
            const impact_velocity = inst.vel_y;
            next_state.* = physics_kernel.handleBlockedFallContact(inst, entity, fall.blocker_id, impact_velocity);
            processBlockedIntegrateSweep(world, instance_idx, inst, impact_velocity, fall.blocker_id, broke_this_tick, topology_changed);
        }
        return;
    }

    if (inst.vel_y > 0) {
        const sweep_y = physics_kernel.sweepMotionAlongAxis(
            world.s1024,
            inst,
            world.entities,
            inst.pos_x,
            inst.pos_y,
            inst.pos_z,
            inst.vel_y,
            .y,
        );
        if (sweep_y.pos_y != inst.pos_y) {
            next_y.* = sweep_y.pos_y;
            next_state.* = .moving;
        }
        if (sweep_y.blocked) {
            processBlockedIntegrateSweep(world, instance_idx, inst, inst.vel_y, sweep_y.blocker_id, broke_this_tick, topology_changed);
            physics_kernel.handleBlockedUpwardContact(inst, entity, sweep_y.blocker_id, inst.vel_y);
        }
    }
}

fn integrateDynamicInstance(
    world: *PhysicsWorld,
    instance_idx: u8,
    moved: *bool,
    topology_changed: *bool,
) void {
    const inst = &world.s1024.instances[instance_idx];
    if (!physics_kernel.canProcessDynamicInstance(inst, world.entities)) {
        if (physics_kernel.isStaticInstance(inst, world.entities)) {
            physics_kernel.markStaticResting(inst);
        }
        return;
    }
    const entity = &world.entities[inst.entity_id];

    var next_x = inst.pos_x;
    var next_z = inst.pos_z;
    var next_y = inst.pos_y;
    var next_state = inst.state;
    var inst_moved = false;
    var broke_this_tick = false;

    processVerticalIntegrateMotion(world, instance_idx, inst, entity, &next_y, &next_state, &broke_this_tick, topology_changed);

    processPlanarIntegrateSweep(world, instance_idx, inst, entity, .x, inst.pos_x, next_y, inst.pos_z, inst.vel_x, &next_x, &next_state, &broke_this_tick, topology_changed);
    processPlanarIntegrateSweep(world, instance_idx, inst, entity, .z, next_x, next_y, inst.pos_z, inst.vel_z, &next_z, &next_state, &broke_this_tick, topology_changed);

    inst_moved = physics_kernel.finalizeMotionState(
        inst,
        next_x,
        next_y,
        next_z,
        next_state,
        broke_this_tick,
        SLEEP_TIME_THRESHOLD,
    );
    if (broke_this_tick) return;
    moved.* = moved.* or inst_moved;
}

/// Initialize physics world with all subsystems
pub fn initWorld(world: *PhysicsWorld, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) void {
    world.* = .{
        .s1024 = s1024,
        .entities = entities,
        .joints = &[_]joint.Joint{},
        .joint_count = 0,
        .tick = 0,
        .world_bus = null,
        .pending_collisions = .{},
        .broadphase_pairs = undefined,
        .broadphase_pair_count = 0,
    };

    physics_kernel.initCoreSubsystems();
}

/// Pre-step phase: apply forces, prepare for integration
pub fn preStep(world: *PhysicsWorld) void {
    physics_kernel.runPreStepSystems(world.s1024, world.entities, 1.0, SLEEP_TIME_THRESHOLD, FIXED_DT);
}

/// Broad phase: spatial queries to find potential collision pairs
pub fn broadPhase(world: *PhysicsWorld) void {
    world.broadphase_pair_count = physics_kernel.collectBroadPhasePairs(
        world.s1024.instances[0..world.s1024.instance_count],
        world.entities,
        world.broadphase_pairs[0..],
    );
}

/// Solve constraints: joints, contacts, springs
pub fn solveConstraints(world: *PhysicsWorld) void {
    _ = physics_kernel.solveConstraintBlock(
        world.s1024,
        world.entities,
        world.joints[0..world.joint_count],
        world.broadphase_pairs[0..world.broadphase_pair_count],
    );
}

/// Integrate: move objects, update positions
pub const IntegrateResult = struct {
    moved: bool,
    topology_changed: bool,
};

pub fn integrate(world: *PhysicsWorld) IntegrateResult {
    physics_kernel.clearPendingCollisions(&world.pending_collisions);
    var moved = false;
    var topology_changed = false;
    var i: u8 = 0;
    while (i < world.s1024.instance_count) : (i += 1) {
        integrateDynamicInstance(world, i, &moved, &topology_changed);
    }

    physics_kernel.rebuildOccupancyIfNeeded(world.s1024, world.entities, moved or topology_changed);
    return .{
        .moved = moved,
        .topology_changed = topology_changed,
    };
}

/// Handle events: collision callbacks, triggers
pub fn handleEvents(world: *PhysicsWorld) void {
    physics_kernel.publishPendingCollisions(&world.pending_collisions, world.world_bus, world.tick);
}

/// Record snapshot for rewind
pub fn recordSnapshot(world: *PhysicsWorld) void {
    physics_kernel.recordWorldSnapshot(world.tick, world.s1024, world.entities);
}

fn runStep(world: *PhysicsWorld, cfg: StepConfig) StepResult {
    physics_kernel.beginWorldStep(&world.tick, world.s1024);

    var pre_changed = false;
    var observed_pair_count: usize = world.broadphase_pair_count;
    if (cfg.run_pre_motion_constraint) {
        const pre_constraint = physics_kernel.runPreMotionConstraintStage(
            world.s1024,
            world.entities,
            world.joints[0..world.joint_count],
            world.broadphase_pairs[0..],
            cfg.time_scale,
            SLEEP_TIME_THRESHOLD,
            cfg.dt,
        );
        world.broadphase_pair_count = pre_constraint.pair_count;
        observed_pair_count = pre_constraint.pair_count;
        pre_changed = pre_constraint.changed;
    } else if (cfg.apply_continuous_physics) {
        physics_kernel.runPreStepSystems(
            world.s1024,
            world.entities,
            cfg.time_scale,
            SLEEP_TIME_THRESHOLD,
            cfg.dt,
        );
    }

    const integrate_result = integrate(world);
    const constraint_result = physics_kernel.runPostMotionConstraintStage(
        world.s1024,
        world.entities,
        world.joints[0..world.joint_count],
        world.broadphase_pairs[0..],
    );
    world.broadphase_pair_count = physics_kernel.mergeObservedPairCount(
        observed_pair_count,
        constraint_result.pair_count,
    );
    physics_kernel.finishWorldStep(&world.pending_collisions, world.world_bus, world.tick, world.s1024, world.entities, cfg.dt);
    return .{
        .changed = pre_changed or integrate_result.moved or integrate_result.topology_changed or constraint_result.changed,
        .pair_count = world.broadphase_pair_count,
    };
}

/// Main physics step - call once per frame
pub fn stepPhysics(world: *PhysicsWorld) void {
    _ = runStep(world, .{});
}

pub fn stepPhysicsConfigured(world: *PhysicsWorld, cfg: StepConfig) StepResult {
    return runStep(world, cfg);
}

// ============================================================================
// Query World View for Unified Query Layer
// ============================================================================

usingnamespace query;

/// Get query world view for unified query layer
pub fn getQueryWorldView(world: *PhysicsWorld) query.QueryWorldView {
    return .{
        .s1024 = world.s1024,
        .instances = world.s1024.instances[0..world.s1024.instance_count],
        .entities = world.entities,
    };
}

test "PhysicsWorld broadPhase collects swept dynamic-static candidate pair" {
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    broadPhase(&world);

    try std.testing.expectEqual(@as(usize, 1), world.broadphase_pair_count);
    try std.testing.expectEqual(@as(u8, 0), world.broadphase_pairs[0].a);
    try std.testing.expectEqual(@as(u8, 1), world.broadphase_pairs[0].b);
}

test "PhysicsWorld broadPhase ignores distant pairs outside swept bounds" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var other = entity16.initEntity16();
    other.physics.mass = 10;
    other.physics.material = .solid;
    entity16.setVoxel(&other, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, other };
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
        .vel_x = 1,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 10,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    broadPhase(&world);

    try std.testing.expectEqual(@as(usize, 0), world.broadphase_pair_count);
}

test "PhysicsWorld broadPhase skips static-static pairs" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var a = entity16.initEntity16();
    a.physics.mass = 0;
    a.physics.material = .solid;
    a.physics.flags |= 0x01;
    entity16.setVoxel(&a, 0, 0, 0);

    var b = entity16.initEntity16();
    b.physics.mass = 0;
    b.physics.material = .solid;
    b.physics.flags |= 0x01;
    entity16.setVoxel(&b, 0, 0, 0);

    var entities = [_]entity16.Entity16{ a, b };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    broadPhase(&world);

    try std.testing.expectEqual(@as(usize, 0), world.broadphase_pair_count);
}

test "PhysicsWorld step resolves overlapping dynamic pair through contact constraints" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var a = entity16.initEntity16();
    a.physics.mass = 10;
    a.physics.material = .solid;
    entity16.setVoxel(&a, 0, 0, 0);

    var b = entity16.initEntity16();
    b.physics.mass = 10;
    b.physics.material = .solid;
    entity16.setVoxel(&b, 0, 0, 0);

    var entities = [_]entity16.Entity16{ a, b };
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    const aabb_a = physics.computeEntityWorldAABB(&s1024.instances[0], &entities[0]).?;
    const aabb_b = physics.computeEntityWorldAABB(&s1024.instances[1], &entities[1]).?;
    try std.testing.expect(!physics.aabbHit(aabb_a, aabb_b));
}

test "PhysicsWorld contact constraints apply tangential friction to overlapping pair" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var floor_like = entity16.initEntity16();
    floor_like.physics.mass = 0;
    floor_like.physics.material = .solid;
    floor_like.physics.flags |= 0x01;
    floor_like.physics.friction = 255;
    entity16.setVoxel(&floor_like, 0, 0, 0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    mover.physics.friction = 255;
    entity16.setVoxel(&mover, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, floor_like };
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
        .vel_x = 8,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    broadPhase(&world);
    _ = physics_kernel.solveContactConstraints(
        world.s1024,
        world.entities,
        world.broadphase_pairs[0..world.broadphase_pair_count],
    );

    try std.testing.expect(@abs(s1024.instances[0].vel_x) < 8);
}

test "PhysicsWorld contact constraints apply restitution to overlapping elastic pair" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var a = entity16.initEntity16();
    a.physics.mass = 10;
    a.physics.material = .elastic;
    a.physics.restitution = 255;
    entity16.setVoxel(&a, 0, 0, 0);

    var b = entity16.initEntity16();
    b.physics.mass = 10;
    b.physics.material = .elastic;
    b.physics.restitution = 255;
    entity16.setVoxel(&b, 0, 0, 0);

    var entities = [_]entity16.Entity16{ a, b };
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
        .vel_y = 8,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -8,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    broadPhase(&world);
    _ = physics_kernel.solveContactConstraints(
        world.s1024,
        world.entities,
        world.broadphase_pairs[0..world.broadphase_pair_count],
    );

    try std.testing.expect(s1024.instances[0].vel_y < 0);
    try std.testing.expect(s1024.instances[1].vel_y > 0);
}

test "PhysicsWorld unified constraint block separates vertical dynamic stack overlap" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var lower = entity16.initEntity16();
    lower.physics.mass = 20;
    lower.physics.material = .solid;
    lower.physics.friction = 200;
    entity16.setVoxel(&lower, 0, 0, 0);

    var upper = entity16.initEntity16();
    upper.physics.mass = 20;
    upper.physics.material = .solid;
    upper.physics.friction = 200;
    entity16.setVoxel(&upper, 0, 0, 0);

    var entities = [_]entity16.Entity16{ lower, upper };
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    broadPhase(&world);
    _ = physics_kernel.solveConstraintBlock(
        world.s1024,
        world.entities,
        world.joints[0..world.joint_count],
        world.broadphase_pairs[0..world.broadphase_pair_count],
    );

    const aabb_lower = physics.computeEntityWorldAABB(&s1024.instances[0], &entities[0]).?;
    const aabb_upper = physics.computeEntityWorldAABB(&s1024.instances[1], &entities[1]).?;
    try std.testing.expect(!physics.aabbHit(aabb_lower, aabb_upper));
}

test "PhysicsWorld unified constraint block depenetrates dynamic body from environment voxel" {
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

    var body = entity16.initEntity16();
    body.physics.mass = 20;
    body.physics.material = .solid;
    entity16.setVoxel(&body, 0, 0, 0);

    var entities = [_]entity16.Entity16{body};
    s1024.instance_count = 1;
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    broadPhase(&world);
    _ = physics_kernel.solveConstraintBlock(
        world.s1024,
        world.entities,
        world.joints[0..world.joint_count],
        world.broadphase_pairs[0..world.broadphase_pair_count],
    );

    try std.testing.expect(
        s1024.instances[0].pos_x != 10 or
            s1024.instances[0].pos_y != 10 or
            s1024.instances[0].pos_z != 10,
    );
}

test "PhysicsWorld step records rewind snapshot" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{entity16.Prototypes.apple()};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(world.tick == 1);
    try std.testing.expect(rewind.getWorldSnapshotAtTick(1) != null);
}

test "PhysicsWorld step advances dynamic instance position" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{entity16.Prototypes.apple()};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 100,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    const before_y = s1024.instances[0].pos_y;
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_y < before_y);
    try std.testing.expect(s1024.instances[0].vel_y < 0);
}

test "PhysicsWorld step blocks falling body on floor" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var falling = entity16.initEntity16();
    falling.physics.mass = 10;
    falling.physics.material = .solid;
    entity16.setVoxel(&falling, 0, 0, 0);

    var floor = entity16.initEntity16();
    floor.physics.mass = 0;
    floor.physics.material = .solid;
    floor.physics.flags |= 0x01;
    entity16.setVoxel(&floor, 0, 0, 0);

    var entities = [_]entity16.Entity16{ falling, floor };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 2,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_y == 1);
    try std.testing.expect(s1024.instances[0].vel_y == 0);
}

test "PhysicsWorld step blocks lateral motion into wall" {
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
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 1,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 1,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_x == 0);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
}

test "PhysicsWorld step settles grounded body into resting state" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var falling = entity16.initEntity16();
    falling.physics.mass = 10;
    falling.physics.material = .solid;
    entity16.setVoxel(&falling, 0, 0, 0);

    var floor = entity16.initEntity16();
    floor.physics.mass = 0;
    floor.physics.material = .solid;
    floor.physics.flags |= 0x01;
    entity16.setVoxel(&floor, 0, 0, 0);

    var entities = [_]entity16.Entity16{ falling, floor };
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
        .vel_x = 2,
        .vel_y = -5,
        .vel_z = 3,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].state == .resting);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
    try std.testing.expect(s1024.instances[0].vel_y == 0);
    try std.testing.expect(s1024.instances[0].vel_z == 0);
    try std.testing.expect(s1024.instances[0].sleep_tick >= 30);
}

test "PhysicsWorld step blocks upward motion on ceiling" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .liquid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var ceiling = entity16.initEntity16();
    ceiling.physics.mass = 0;
    ceiling.physics.material = .solid;
    ceiling.physics.flags |= 0x01;
    entity16.setVoxel(&ceiling, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, ceiling };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 1,
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
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_y == 0);
    try std.testing.expect(s1024.instances[0].vel_y == 0);
}

test "PhysicsWorld step slides along free axis when diagonal path is partially blocked" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .liquid;
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
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = 2,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 1,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_x == 0);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
    try std.testing.expect(s1024.instances[0].pos_z > 0);
}

test "PhysicsWorld wakes resting instance when motion is applied" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .liquid;
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
        .state = .resting,
        .sleep_tick = SLEEP_TIME_THRESHOLD,
        .vel_x = 16,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].state != .resting);
    try std.testing.expect(s1024.instances[0].sleep_tick == 0);
    try std.testing.expect(s1024.instances[0].pos_x > 0);
}

test "PhysicsWorld step does not tunnel through thin wall on large lateral step" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .liquid;
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
        .state = .idle,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_x < s1024.instances[1].pos_x);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
}

test "PhysicsWorld step does not tunnel through thin ceiling on large upward step" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .liquid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var ceiling = entity16.initEntity16();
    ceiling.physics.mass = 0;
    ceiling.physics.material = .solid;
    ceiling.physics.flags |= 0x01;
    entity16.setVoxel(&ceiling, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, ceiling };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 16,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 8,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_y < s1024.instances[1].pos_y);
    try std.testing.expect(s1024.instances[0].vel_y == 0);
}

test "PhysicsWorld handleEvents broadcasts collision to bus" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .liquid;
    mover.physics.hardness = 42;
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
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 1,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);

    var event_bus = bus.Bus.init();
    world.world_bus = &event_bus;
    stepPhysics(&world);

    try std.testing.expect(event_bus.msg_count >= 2);
    try std.testing.expect(event_bus.messages[0].msg_type == .PHYSICS_EVENT);
    try std.testing.expect(event_bus.messages[0].entity_id == 0);
    try std.testing.expect(event_bus.messages[0].payload.collision.hardness == 42);
    try std.testing.expect(event_bus.messages[0].payload.collision.did_break == false);
}

test "PhysicsWorld handleEvents marks breakable impact in bus payload" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 500;
    mover.physics.material = .fragile;
    mover.physics.hardness = 10;
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
        .state = .idle,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);

    var event_bus = bus.Bus.init();
    world.world_bus = &event_bus;
    stepPhysics(&world);

    try std.testing.expect(event_bus.msg_count == 2);
    try std.testing.expect(event_bus.messages[0].priority == .HIGH);
    try std.testing.expect(event_bus.messages[0].payload.collision.did_break);
}

test "PhysicsWorld handleEvents broadcasts both participants for instance collision" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 50;
    mover.physics.material = .solid;
    mover.physics.hardness = 80;
    entity16.setVoxel(&mover, 0, 0, 0);

    var target = entity16.initEntity16();
    target.physics.mass = 20;
    target.physics.material = .solid;
    target.physics.hardness = 25;
    entity16.setVoxel(&target, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, target };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 1,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);

    var event_bus = bus.Bus.init();
    world.world_bus = &event_bus;
    stepPhysics(&world);

    try std.testing.expect(event_bus.msg_count == 4);
    try std.testing.expect(event_bus.messages[0].entity_id == 0);
    try std.testing.expect(event_bus.messages[2].entity_id == 1);
}

test "PhysicsWorld step applies elastic bounce response on rubber terrain" {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 32, .rubber);

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0, .px = 0, .py = 0, .pz = 0, .lx = 0, .ly = 0, .lz = 0,
    }), true);

    var ball = entity16.initEntity16();
    ball.physics.mass = 20;
    ball.physics.material = .elastic;
    ball.physics.restitution = 220;
    entity16.setVoxel(&ball, 0, 0, 0);

    var entities = [_]entity16.Entity16{ball};
    s1024.instance_count = 1;
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
        .vel_y = -40,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].vel_y > 0);
    try std.testing.expect(s1024.instances[0].state != .resting);
}

test "PhysicsWorld step applies soft-ground absorption on mud terrain" {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 32, .mud);

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0, .px = 0, .py = 0, .pz = 0, .lx = 0, .ly = 0, .lz = 0,
    }), true);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .solid;
    mover.physics.restitution = 0;
    entity16.setVoxel(&mover, 0, 0, 0);

    var entities = [_]entity16.Entity16{mover};
    s1024.instance_count = 1;
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
        .vel_x = 10,
        .vel_y = -20,
        .vel_z = 10,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    const before_y = s1024.instances[0].pos_y;
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].vel_y >= 0);
    try std.testing.expect(s1024.instances[0].vel_x < 10);
    try std.testing.expect(s1024.instances[0].vel_z < 10);
    try std.testing.expect(s1024.instances[0].pos_y <= before_y);
}

test "PhysicsWorld collision applies broken state to fragile instance" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

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

    var entities = [_]entity16.Entity16{ fragile, wall };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].state == .broken);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
}

test "PhysicsWorld broken instance is removed from occupancy after rebuild" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

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

    var entities = [_]entity16.Entity16{ fragile, wall };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].state == .broken);
    try std.testing.expect(!physics.isOccupiedGlobal(&s1024, null, entities[0..], 0, 10, 0, null));
}

test "PhysicsWorld break spawns debris record" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

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

    var entities = [_]entity16.Entity16{ fragile, wall };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    const debris_sys = destruction.getDebrisSystem();
    try std.testing.expect(debris_sys.count > 0);
    try std.testing.expect(debris_sys.debris[0].active);
}

test "PhysicsWorld step updates debris physics after break" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

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

    var entities = [_]entity16.Entity16{ fragile, wall };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    const debris_sys = destruction.getDebrisSystem();
    try std.testing.expect(debris_sys.count > 0);
    const before_y = debris_sys.debris[0].pos_y;
    const before_vel_y = debris_sys.debris[0].vel_y;

    stepPhysics(&world);

    try std.testing.expect(destruction.getDebrisSystem().debris[0].pos_y != before_y or destruction.getDebrisSystem().debris[0].vel_y != before_vel_y);
}

test "PhysicsWorld break wakes resting body that was supported by broken instance" {
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

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    try std.testing.expect(s1024.instances[1].state == .resting);

    _ = applyBreakState(&world, 0, 16);

    try std.testing.expect(s1024.instances[0].state == .broken);
    try std.testing.expect(s1024.instances[1].state != .resting);
    try std.testing.expect(s1024.instances[1].sleep_tick == 0);
}

test "PhysicsWorld lateral environment collision applies tangential friction response" {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 32, .mud);

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0, .px = 0, .py = 0, .pz = 0, .lx = 1, .ly = 10, .lz = 0,
    }), true);

    var mover = entity16.initEntity16();
    mover.physics.mass = 20;
    mover.physics.material = .liquid;
    mover.physics.friction = 64;
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
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = 20,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);
    stepPhysics(&world);

    try std.testing.expect(s1024.instances[0].pos_x == 0);
    try std.testing.expect(s1024.instances[0].vel_x == 0);
    try std.testing.expect(s1024.instances[0].vel_z > 0);
    try std.testing.expect(s1024.instances[0].vel_z < 20);
}

test "PhysicsWorld step advances KCC characters through world coordinator" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{entity16.Prototypes.apple()};
    var world: PhysicsWorld = undefined;
    initWorld(&world, &s1024, entities[0..]);

    const char = kcc.createCharacter(10, 50, 10, .{
        .gravity = -600.0,
        .stand_height = 6,
        .crouch_height = 4,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const before_y = char.pos_y;

    stepPhysics(&world);

    try std.testing.expect(char.pos_y < before_y);
    try std.testing.expect(char.vel_y < 0);
}
