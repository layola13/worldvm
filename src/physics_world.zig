//! Physics World - Unified Coordinator for All Physics Subsystems
//!
//! P0: Unified PhysicsWorld Skeleton
//! Coordinates KCC, Vehicle, Ragdoll, Ballistics, Joints in a single pipeline

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const joint = @import("joint.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const ragdoll = @import("ragdoll.zig");
const ballistics = @import("ballistics.zig");
const rewind = @import("rewind.zig");
const query = @import("query.zig");

pub const FIXED_DT: f32 = 1.0 / 60.0;

/// PhysicsWorld - bundles all physics subsystem handles
pub const PhysicsWorld = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    joint_count: usize,
};

/// Initialize physics world with all subsystems
pub fn initWorld(world: *PhysicsWorld, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) void {
    world.* = .{
        .s1024 = s1024,
        .entities = entities,
        .joints = &[_]joint.Joint{},
        .joint_count = 0,
    };

    // Initialize all subsystems
    physics.init();
    kcc.init();
    vehicle.init();
    ragdoll.init();
    ballistics.init();
    rewind.init();
}

/// Pre-step phase: apply forces, prepare for integration
pub fn preStep(world: *PhysicsWorld) void {
    // Apply continuous physics (gravity, damping, angular velocity, sleep)
    applyContinuousPhysics(world);

    // Update KCC characters (movement, jumping, crouching)
    updateKCC(world, FIXED_DT);

    // Update vehicles (throttle, steering, suspension)
    updateVehicles(world, FIXED_DT);

    // Update ragdolls (joint solving, physics)
    updateRagdolls(world, FIXED_DT);

    // Update projectiles (ballistics simulation)
    updateProjectiles(world, FIXED_DT);
}

/// Broad phase: spatial queries to find potential collision pairs
pub fn broadPhase(world: *PhysicsWorld) void {
    // TODO: Implement spatial hashing / broadphase
    // For now, the narrow phase handles collision directly
    _ = world;
}

/// Solve constraints: joints, contacts, springs
pub fn solveConstraints(world: *PhysicsWorld) void {
    // Solve joint constraints
    if (world.joint_count > 0) {
        joint.solveJointsForTick(
            world.s1024.instances[0..world.s1024.instance_count],
            world.joints[0..world.joint_count],
            world.entities,
        );
    }

    // Ragdoll joint solving is handled in updateRagdolls
}

/// Integrate: move objects, update positions
pub fn integrate(world: *PhysicsWorld) void {
    // Projectile integration is handled in updateProjectiles
    _ = world;
}

/// Handle events: collision callbacks, triggers
pub fn handleEvents(world: *PhysicsWorld) void {
    _ = world;
    // TODO: Broadcast collision events via bus
}

/// Record snapshot for rewind
pub fn recordSnapshot(world: *PhysicsWorld) void {
    _ = world;
    // TODO: Implement world state snapshot
}

/// Main physics step - call once per frame
pub fn stepPhysics(world: *PhysicsWorld) void {
    preStep(world);
    broadPhase(world);
    solveConstraints(world);
    integrate(world);
    handleEvents(world);
    recordSnapshot(world);
}

// ============================================================================
// Subsystem Updates
// ============================================================================

fn applyContinuousPhysics(world: *PhysicsWorld) void {
    var i: u8 = 0;
    while (i < world.s1024.instance_count) : (i += 1) {
        const inst = &world.s1024.instances[i];
        if (inst.entity_id >= world.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &world.entities[inst.entity_id];

        // Skip static objects (flag 0x01)
        if ((entity.physics.flags & 0x01) != 0) continue;

        // Apply gravity to velocity
        if (entity.physics.material != .liquid) {
            const grav_vel: i32 = @as(i32, physics.GRAVITY);
            inst.vel_y = @truncate(@as(i32, inst.vel_y) + grav_vel);
        }

        // Apply velocity damping
        physics.applyDamping(&inst.vel_x, &inst.vel_y, &inst.vel_z);

        // Apply angular velocity
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

            // Apply angular damping
            physics.applyAngularDamping(&inst.ang_x, &inst.ang_y, &inst.ang_z);
        }
    }
}

fn updateKCC(world: *PhysicsWorld, dt: f32) void {
    _ = world;
    _ = dt;
    // KCC characters are updated via their own update function
    // which is called by the game logic, not here
    // This is because KCC requires input handling which is external
}

fn updateVehicles(world: *PhysicsWorld, dt: f32) void {
    const vehicle_sys = vehicle.getSystem();
    var i: u8 = 0;
    while (i < vehicle_sys.count) : (i += 1) {
        const v = &vehicle_sys.vehicles[i];
        vehicle.update(v, world.s1024, world.entities, dt);
    }
}

fn updateRagdolls(world: *PhysicsWorld, dt: f32) void {
    const ragdoll_sys = ragdoll.getSystem();
    var i: u8 = 0;
    while (i < ragdoll_sys.count) : (i += 1) {
        const r = &ragdoll_sys.ragdolls[i];
        ragdoll.update(r, dt);
        ragdoll.solveJoints(r, world.s1024, world.entities);
    }
}

fn updateProjectiles(world: *PhysicsWorld, dt: f32) void {
    ballistics.simulateAll(world.s1024, world.entities, dt);
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
