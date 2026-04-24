//! Ragdoll - Hierarchical Joint Chain Physics
//!
//! Phase 10/11: Ragdoll physics with joint chains, limb constraints, and physics state
//! Phase P6: Passive Ragdoll - true body physics with gravity, collision, momentum
//! Phase P7: Active Ragdoll - motor-driven joints, balance control, pose tracking
//!
//! Handles: Body part creation, joint constraints, impulse application, resurrection,
//!          limb collision, momentum inheritance, motor joints, balance recovery

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const joint = @import("joint.zig");
const physics = @import("physics.zig");

pub const RagdollPart = struct {
    joint_idx: u8,
    pos_offset_x: i8,
    pos_offset_y: i8,
    pos_offset_z: i8,
    mass_ratio: f32,
    radius: u8,
    active: bool,

    // Runtime physics state (for passive ragdoll)
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    grounded: bool,
};

// P7: Motor joint for active control
pub const MotorJoint = struct {
    enabled: bool,
    target_angle: f32,
    current_angle: f32,
    proportional_gain: f32,
    derivative_gain: f32,
    max_torque: f32,
    last_error: f32,
};

// P7: Target pose for active ragdoll
pub const RagdollPose = struct {
    part_targets: [MAX_RAGDOLL_PARTS]struct { x: f32, y: f32, z: f32 },
    joint_targets: [MAX_RAGDOLL_JOINTS]f32,
    blend_factor: f32,
};

// P7: Balance state
pub const BalanceState = enum(u8) {
    balanced = 0,
    leaning = 1,
    falling = 2,
    recovery = 3,
};

pub const MAX_RAGDOLL_PARTS: usize = 8;
pub const MAX_RAGDOLL_JOINTS: usize = 7;

pub const Ragdoll = struct {
    parts: [MAX_RAGDOLL_PARTS]RagdollPart,
    joints: [MAX_RAGDOLL_JOINTS]joint.Joint,
    part_count: u8,
    joint_count: u8,
    active: bool,
    resurrection_tick: u16,

    // Base position for all parts
    base_x: i32,
    base_y: i32,
    base_z: i32,

    // Ragdoll-wide physics
    gravity: f32 = -800.0,
    angular_vel_x: f32 = 0,
    angular_vel_y: f32 = 0,
    angular_vel_z: f32 = 0,

    // P7: Active ragdoll control
    motors: [MAX_RAGDOLL_JOINTS]MotorJoint,
    pose: RagdollPose,
    balance_state: BalanceState,
    balance_threshold: f32,
    recovery_strength: f32,
};

pub const MAX_RAGDOLLS: usize = 16;

pub const RagdollSystem = struct {
    ragdolls: [MAX_RAGDOLLS]Ragdoll,
    count: u8,
};

var g_ragdoll_system: RagdollSystem = undefined;

pub fn init() void {
    g_ragdoll_system.count = 0;
    for (0..MAX_RAGDOLLS) |i| {
        g_ragdoll_system.ragdolls[i] = .{
            .parts = [_]RagdollPart{.{.joint_idx=0, .pos_offset_x=0, .pos_offset_y=0, .pos_offset_z=0, .mass_ratio=0, .radius=0, .active=false, .pos_x=0, .pos_y=0, .pos_z=0, .vel_x=0, .vel_y=0, .vel_z=0, .grounded=false}} ** MAX_RAGDOLL_PARTS,
            .joints = undefined,
            .part_count = 0,
            .joint_count = 0,
            .active = false,
            .resurrection_tick = 0,
            .base_x = 0,
            .base_y = 0,
            .base_z = 0,
            .motors = [_]MotorJoint{.{.enabled=false, .target_angle=0, .current_angle=0, .proportional_gain=0, .derivative_gain=0, .max_torque=0, .last_error=0}} ** MAX_RAGDOLL_JOINTS,
            .pose = .{
                .part_targets = undefined,
                .joint_targets = undefined,
                .blend_factor = 0,
            },
            .balance_state = .balanced,
            .balance_threshold = 0.5,
            .recovery_strength = 100.0,
        };
    }
}

/// Standard humanoid ragdoll structure
pub const HumanoidRagdollLayout = struct {
    pub const parts: [8]struct { ox: i8, oy: i8, oz: i8, mass: f32, radius: u8 } = .{
        .{ .ox = 8, .oy = 14, .oz = 8, .mass = 0.15, .radius = 4 },  // Head
        .{ .ox = 8, .oy = 10, .oz = 8, .mass = 0.20, .radius = 5 },  // Torso
        .{ .ox = 4, .oy = 10, .oz = 8, .mass = 0.10, .radius = 3 },  // Left arm
        .{ .ox = 12, .oy = 10, .oz = 8, .mass = 0.10, .radius = 3 }, // Right arm
        .{ .ox = 4, .oy = 6, .oz = 8, .mass = 0.12, .radius = 3 },  // Left forearm
        .{ .ox = 12, .oy = 6, .oz = 8, .mass = 0.12, .radius = 3 },  // Right forearm
        .{ .ox = 6, .oy = 2, .oz = 8, .mass = 0.12, .radius = 3 },  // Left leg
        .{ .ox = 10, .oy = 2, .oz = 8, .mass = 0.12, .radius = 3 },  // Right leg
    };

    pub const joint_defs: [7]struct { a: u8, b: u8, limit_min: f32, limit_max: f32 } = .{
        .{ .a = 0, .b = 1, .limit_min = -0.5, .limit_max = 0.5 },     // Head-torso
        .{ .a = 1, .b = 2, .limit_min = -1.5, .limit_max = 1.5 },     // Left shoulder
        .{ .a = 1, .b = 3, .limit_min = -1.5, .limit_max = 1.5 },    // Right shoulder
        .{ .a = 2, .b = 4, .limit_min = 0, .limit_max = 2.5 },       // Left elbow
        .{ .a = 3, .b = 5, .limit_min = 0, .limit_max = 2.5 },       // Right elbow
        .{ .a = 1, .b = 6, .limit_min = -1.5, .limit_max = 1.5 },    // Left hip
        .{ .a = 1, .b = 7, .limit_min = -1.5, .limit_max = 1.5 },    // Right hip
    };
};

/// Create ragdoll from humanoid template
pub fn createHumanoid(base_x: i32, base_y: i32, base_z: i32) ?*Ragdoll {
    if (g_ragdoll_system.count >= MAX_RAGDOLLS) return null;
    const idx = g_ragdoll_system.count;
    g_ragdoll_system.count += 1;
    var ragdoll = &g_ragdoll_system.ragdolls[idx];

    ragdoll.part_count = 8;
    ragdoll.joint_count = 7;
    ragdoll.active = true;
    ragdoll.resurrection_tick = 0;
    ragdoll.base_x = base_x;
    ragdoll.base_y = base_y;
    ragdoll.base_z = base_z;

    for (HumanoidRagdollLayout.parts, 0..) |p, i| {
        ragdoll.parts[i] = .{
            .joint_idx = 0,
            .pos_offset_x = p.ox,
            .pos_offset_y = p.oy,
            .pos_offset_z = p.oz,
            .mass_ratio = p.mass,
            .radius = p.radius,
            .active = true,
            .pos_x = @as(f32, @floatFromInt(base_x)) + @as(f32, @floatFromInt(p.ox)),
            .pos_y = @as(f32, @floatFromInt(base_y)) + @as(f32, @floatFromInt(p.oy)),
            .pos_z = @as(f32, @floatFromInt(base_z)) + @as(f32, @floatFromInt(p.oz)),
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .grounded = false,
        };
    }

    for (HumanoidRagdollLayout.joint_defs, 0..) |j_def, i| {
        ragdoll.joints[i] = .{
            .joint_type = .hinge,
            .entity_a = @as(u16, j_def.a),
            .entity_b = @as(u16, j_def.b),
            .anchor_a_x = HumanoidRagdollLayout.parts[j_def.a].ox,
            .anchor_a_y = HumanoidRagdollLayout.parts[j_def.a].oy,
            .anchor_a_z = HumanoidRagdollLayout.parts[j_def.a].oz,
            .anchor_b_x = HumanoidRagdollLayout.parts[j_def.b].ox,
            .anchor_b_y = HumanoidRagdollLayout.parts[j_def.b].oy,
            .anchor_b_z = HumanoidRagdollLayout.parts[j_def.b].oz,
            .axis_x = 0,
            .axis_y = 0,
            .axis_z = 1,
            .limit_min = j_def.limit_min,
            .limit_max = j_def.limit_max,
            .breaking_force = 0,
            .stiffness = 1000,
            .damping = 100,
            .enabled = true,
        };
    }

    return ragdoll;
}

/// Create ragdoll from entity
pub fn createFromEntity(entity: *const entity16.Entity16) ?*Ragdoll {
    _ = entity;
    return createHumanoid(0, 0, 0);
}

/// Apply impulse to ragdoll at world point (momentum inheritance)
pub fn applyImpulse(
    ragdoll: *Ragdoll,
    impulse_x: f32,
    impulse_y: f32,
    impulse_z: f32,
    point_x: f32,
    point_y: f32,
    point_z: f32,
) void {
    if (!ragdoll.active) return;

    var i: u8 = 0;
    while (i < ragdoll.part_count) : (i += 1) {
        const part = &ragdoll.parts[i];
        if (!part.active) continue;

        const dx = point_x - part.pos_x;
        const dy = point_y - part.pos_y;
        const dz = point_z - part.pos_z;
        const dist_sq = dx * dx + dy * dy + dz * dz;

        if (dist_sq > 0.1) {
            const dist = @sqrt(dist_sq);
            // Falloff based on distance from impact point
            const falloff = @max(0.2, 1.0 - (dist / 50.0));
            const scale = part.mass_ratio * falloff;

            // Apply impulse scaled by mass ratio and distance falloff
            part.vel_x += impulse_x * scale;
            part.vel_y += impulse_y * scale;
            part.vel_z += impulse_z * scale;
        } else {
            // Direct hit - full impulse
            part.vel_x += impulse_x * part.mass_ratio;
            part.vel_y += impulse_y * part.mass_ratio;
            part.vel_z += impulse_z * part.mass_ratio;
        }
    }

    // Add angular velocity from off-center impact
    // Calculate total momentum for angular calculation
    var total_px: f32 = 0;
    var total_py: f32 = 0;
    var total_pz: f32 = 0;
    var k: u8 = 0;
    while (k < ragdoll.part_count) : (k += 1) {
        const p = &ragdoll.parts[k];
        if (!p.active) continue;
        total_px += p.vel_x * p.mass_ratio;
        total_py += p.vel_y * p.mass_ratio;
        total_pz += p.vel_z * p.mass_ratio;
    }
    // Apply torque based on momentum
    const torque_scale: f32 = 0.001;
    ragdoll.angular_vel_x += total_py * torque_scale;
    ragdoll.angular_vel_y += total_pz * torque_scale;
    ragdoll.angular_vel_z += total_px * torque_scale;
}

/// Break a specific joint (limb)
pub fn breakLimb(ragdoll: *Ragdoll, part_idx: u8) void {
    if (part_idx >= ragdoll.part_count) return;
    ragdoll.parts[part_idx].active = false;

    var i: u8 = 0;
    while (i < ragdoll.joint_count) : (i += 1) {
        const j = &ragdoll.joints[i];
        if (j.entity_a == part_idx or j.entity_b == part_idx) {
            j.enabled = false;
        }
    }
}

/// Check if ragdoll is fully broken
pub fn isFullyBroken(ragdoll: *const Ragdoll) bool {
    var active_count: u8 = 0;
    for (ragdoll.parts[0..ragdoll.part_count]) |part| {
        if (part.active) active_count += 1;
    }
    return active_count < 2;
}

// ============================================================================
// Passive Ragdoll Physics
// ============================================================================

/// Update ragdoll physics - called each tick
pub fn update(ragdoll: *Ragdoll, dt: f32) void {
    if (!ragdoll.active) return;

    if (ragdoll.resurrection_tick > 0) {
        ragdoll.resurrection_tick -= 1;
    }

    // Apply gravity and integrate each part
    var i: u8 = 0;
    while (i < ragdoll.part_count) : (i += 1) {
        const part = &ragdoll.parts[i];
        if (!part.active) continue;

        // Apply gravity
        part.vel_y += ragdoll.gravity * dt;

        // Apply velocity damping
        part.vel_x *= 0.99;
        part.vel_z *= 0.99;

        // Integrate position
        part.pos_x += part.vel_x * dt;
        part.pos_y += part.vel_y * dt;
        part.pos_z += part.vel_z * dt;

        // Ground collision
        if (part.pos_y < @as(f32, @floatFromInt(ragdoll.base_y))) {
            part.pos_y = @as(f32, @floatFromInt(ragdoll.base_y));
            if (part.vel_y < 0) {
                // Bounce with restitution
                part.vel_y = -part.vel_y * 0.3;
                part.grounded = true;
            }
            // Ground friction
            part.vel_x *= 0.8;
            part.vel_z *= 0.8;
        } else {
            part.grounded = false;
        }
    }

    // Apply angular velocity to base position
    ragdoll.angular_vel_x *= 0.95;
    ragdoll.angular_vel_y *= 0.95;
    ragdoll.angular_vel_z *= 0.95;
}

/// Apply angular velocity to ragdoll (for spin/kick effects)
pub fn applyAngularImpulse(ragdoll: *Ragdoll, ang_vel_x: f32, ang_vel_y: f32, ang_vel_z: f32) void {
    ragdoll.angular_vel_x += ang_vel_x;
    ragdoll.angular_vel_y += ang_vel_y;
    ragdoll.angular_vel_z += ang_vel_z;
}

// P7: Enable motor control for a joint
pub fn enableMotor(ragdoll: *Ragdoll, joint_idx: u8, enabled: bool) void {
    if (joint_idx >= ragdoll.joint_count) return;
    ragdoll.motors[joint_idx].enabled = enabled;
}

// P7: Set motor target angle with PD gains
pub fn setMotorTarget(
    ragdoll: *Ragdoll,
    joint_idx: u8,
    target_angle: f32,
    p_gain: f32,
    d_gain: f32,
    max_torque: f32,
) void {
    if (joint_idx >= ragdoll.joint_count) return;
    ragdoll.motors[joint_idx].target_angle = target_angle;
    ragdoll.motors[joint_idx].proportional_gain = p_gain;
    ragdoll.motors[joint_idx].derivative_gain = d_gain;
    ragdoll.motors[joint_idx].max_torque = max_torque;
    ragdoll.motors[joint_idx].enabled = true;
}

// P7: Calculate PD control torque
pub fn calculateMotorTorque(motor: *MotorJoint, current_angle: f32, dt: f32) f32 {
    const err = motor.target_angle - current_angle;
    const p_term = motor.proportional_gain * err;

    const d_term = motor.derivative_gain * (err - motor.last_error) / @max(0.001, dt);
    motor.last_error = err;

    const torque = p_term + d_term;
    return @min(motor.max_torque, @max(-motor.max_torque, torque));
}

// P7: Update active ragdoll motors
pub fn updateActiveMotors(ragdoll: *Ragdoll, dt: f32) void {
    var j: u8 = 0;
    while (j < ragdoll.joint_count) : (j += 1) {
        const motor = &ragdoll.motors[j];
        if (!motor.enabled) continue;

        // Get current joint angle (simplified - using limit diff)
        const joint_def = &ragdoll.joints[j];
        const current_angle = joint_def.limit_max - joint_def.limit_min;

        const torque = calculateMotorTorque(motor, current_angle, dt);

        // Apply torque to connected parts
        const part_a_idx = @as(u8, @intCast(joint_def.entity_a));
        const part_b_idx = @as(u8, @intCast(joint_def.entity_b));

        if (part_a_idx < ragdoll.part_count) {
            const part_a = &ragdoll.parts[part_a_idx];
            const force_scale = torque * part_a.mass_ratio * 0.01;
            part_a.vel_x += force_scale;
            part_a.vel_y += force_scale * 0.5;
        }

        if (part_b_idx < ragdoll.part_count) {
            const part_b = &ragdoll.parts[part_b_idx];
            const force_scale = -torque * part_b.mass_ratio * 0.01;
            part_b.vel_x += force_scale;
            part_b.vel_y += force_scale * 0.5;
        }
    }
}

// P7: Detect balance state from center of mass
pub fn detectBalanceState(ragdoll: *const Ragdoll) BalanceState {
    const com = getCenterOfMass(ragdoll);

    // Calculate COM deviation from base
    const base_x = @as(f32, @floatFromInt(ragdoll.base_x));
    const base_y = @as(f32, @floatFromInt(ragdoll.base_y));
    const base_z = @as(f32, @floatFromInt(ragdoll.base_z));

    const dev_x = com.x - base_x;
    const dev_z = com.z - base_z;
    const lateral_dev = @sqrt(dev_x * dev_x + dev_z * dev_z);

    // Check vertical stability
    const vertical_ratio = (com.y - base_y) / @as(f32, @floatFromInt(ragdoll.base_y));

    if (vertical_ratio < 0.3 or lateral_dev > 10.0) {
        return .falling;
    }
    if (lateral_dev > ragdoll.balance_threshold or vertical_ratio < 0.5) {
        return .leaning;
    }
    return .balanced;
}

// P7: Apply balance recovery forces
pub fn applyBalanceRecovery(ragdoll: *Ragdoll, dt: f32) void {
    ragdoll.balance_state = detectBalanceState(ragdoll);

    if (ragdoll.balance_state == .balanced) return;

    const com = getCenterOfMass(ragdoll);
    const base_x = @as(f32, @floatFromInt(ragdoll.base_x));
    const base_z = @as(f32, @floatFromInt(ragdoll.base_z));

    // Calculate recovery direction
    const recovery_x = base_x - com.x;
    const recovery_z = base_z - com.z;

    // Apply recovery impulse to torso
    if (ragdoll.part_count > 1) {
        const torso = &ragdoll.parts[1]; // Torso is index 1
        const recovery_force = ragdoll.recovery_strength * dt;

        torso.vel_x += recovery_x * recovery_force * 0.1;
        torso.vel_z += recovery_z * recovery_force * 0.1;
    }

    // Apply counter-velocity to legs for leaning
    if (ragdoll.part_count > 6) {
        const left_leg = &ragdoll.parts[6];
        const right_leg = &ragdoll.parts[7];

        left_leg.vel_x -= recovery_x * ragdoll.recovery_strength * dt * 0.05;
        left_leg.vel_z -= recovery_z * ragdoll.recovery_strength * dt * 0.05;
        right_leg.vel_x -= recovery_x * ragdoll.recovery_strength * dt * 0.05;
        right_leg.vel_z -= recovery_z * ragdoll.recovery_strength * dt * 0.05;
    }
}

// P7: Set target pose
pub fn setTargetPose(ragdoll: *Ragdoll, pose: *const RagdollPose) void {
    ragdoll.pose.blend_factor = pose.blend_factor;
    var i: u8 = 0;
    while (i < ragdoll.part_count) : (i += 1) {
        ragdoll.pose.part_targets[i] = pose.part_targets[i];
    }
    i = 0;
    while (i < ragdoll.joint_count) : (i += 1) {
        ragdoll.pose.joint_targets[i] = pose.joint_targets[i];
    }
}

// P7: Blend pose towards target
pub fn blendToPose(ragdoll: *Ragdoll, dt: f32) void {
    if (ragdoll.pose.blend_factor <= 0) return;

    const blend_speed = ragdoll.pose.blend_factor * dt;

    var i: u8 = 0;
    while (i < ragdoll.part_count) : (i += 1) {
        const target = ragdoll.pose.part_targets[i];
        var part = &ragdoll.parts[i];

        part.pos_x += (target.x - part.pos_x) * blend_speed;
        part.pos_y += (target.y - part.pos_y) * blend_speed;
        part.pos_z += (target.z - part.pos_z) * blend_speed;
    }
}

// P7: Full active ragdoll update
pub fn updateActive(ragdoll: *Ragdoll, dt: f32) void {
    if (!ragdoll.active) return;

    // Update passive physics first
    update(ragdoll, dt);

    // Apply motor control
    updateActiveMotors(ragdoll, dt);

    // Apply balance recovery
    applyBalanceRecovery(ragdoll, dt);

    // Blend to target pose
    blendToPose(ragdoll, dt);
}

/// Get part world position
pub fn getPartPosition(ragdoll: *const Ragdoll, part_idx: u8) ?struct { x: f32, y: f32, z: f32 } {
    if (part_idx >= ragdoll.part_count) return null;
    const part = &ragdoll.parts[part_idx];
    return .{ .x = part.pos_x, .y = part.pos_y, .z = part.pos_z };
}

/// Check collision with world for a part
pub fn checkPartCollision(
    ragdoll: *const Ragdoll,
    part_idx: u8,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    if (part_idx >= ragdoll.part_count) return false;
    const part = &ragdoll.parts[part_idx];

    const px = @as(i32, @intFromFloat(@floor(part.pos_x)));
    const py = @as(i32, @intFromFloat(@floor(part.pos_y)));
    const pz = @as(i32, @intFromFloat(@floor(part.pos_z)));
    const radius = @as(i32, @intFromFloat(@floor(@as(f32, @floatFromInt(part.radius)) * 0.5)));

    var dy: i32 = 0;
    while (dy <= radius * 2) : (dy += 2) {
        var dx: i32 = -radius;
        while (dx <= radius) : (dx += 2) {
            var dz: i32 = -radius;
            while (dz <= radius) : (dz += 2) {
                if (physics.isOccupiedGlobal(s1024, null, entities, px + dx, py + dy, pz + dz, null)) {
                    return true;
                }
            }
        }
    }
    return false;
}

/// Resolve collision for a part (push out of solid)
pub fn resolvePartCollision(
    ragdoll: *Ragdoll,
    part_idx: u8,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) void {
    if (part_idx >= ragdoll.part_count) return;
    const part = &ragdoll.parts[part_idx];

    const radius = @as(f32, @floatFromInt(part.radius)) * 0.5;
    var attempts: u8 = 0;

    while (attempts < 4) : (attempts += 1) {
        if (!checkPartCollision(ragdoll, part_idx, s1024, entities)) break;

        // Try to move up
        part.pos_y += 1;
    }

    if (attempts >= 4) {
        // Couldn't resolve - try random directions
        attempts = 0;
        while (attempts < 8) : (attempts += 1) {
            const angle = @as(f32, @floatFromInt(attempts)) * 0.785; // 45 degrees
            const ox = @cos(angle) * radius;
            const oz = @sin(angle) * radius;
            part.pos_x += ox;
            part.pos_z += oz;

            if (!checkPartCollision(ragdoll, part_idx, s1024, entities)) break;

            part.pos_x -= ox;
            part.pos_z -= oz;
        }
    }
}

/// Solve ragdoll joints
pub fn solveJoints(
    ragdoll: *Ragdoll,
    instances: []scene32.Instance,
    entities: []entity16.Entity16,
) void {
    if (!ragdoll.active) return;

    var iter: u8 = 0;
    while (iter < 4) : (iter += 1) {
        var j: u8 = 0;
        while (j < ragdoll.joint_count) : (j += 1) {
            const joint_def = &ragdoll.joints[j];
            if (!joint_def.enabled) continue;

            if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) continue;

            const inst_a = &instances[joint_def.entity_a];
            const inst_b = &instances[joint_def.entity_b];

            if (inst_a.state == .broken or inst_b.state == .broken) continue;

            switch (joint_def.joint_type) {
                .hinge => {
                    joint.solveHingeConstraint(inst_a, inst_b, joint_def, entities);
                },
                .fixed, .ball_socket => {
                    joint.solveDistanceConstraint(inst_a, inst_b, joint_def, entities);
                },
                else => {},
            }
        }
    }
}

/// Get ragdoll center of mass
pub fn getCenterOfMass(ragdoll: *const Ragdoll) struct { x: f32, y: f32, z: f32 } {
    var total_x: f32 = 0;
    var total_y: f32 = 0;
    var total_z: f32 = 0;
    var total_mass: f32 = 0;

    var i: u8 = 0;
    while (i < ragdoll.part_count) : (i += 1) {
        const part = &ragdoll.parts[i];
        if (!part.active) continue;

        total_x += part.pos_x * part.mass_ratio;
        total_y += part.pos_y * part.mass_ratio;
        total_z += part.pos_z * part.mass_ratio;
        total_mass += part.mass_ratio;
    }

    if (total_mass > 0) {
        return .{
            .x = total_x / total_mass,
            .y = total_y / total_mass,
            .z = total_z / total_mass,
        };
    }
    return .{ .x = 0, .y = 0, .z = 0 };
}

/// Trigger resurrection (revive broken ragdoll)
pub fn triggerResurrection(ragdoll: *Ragdoll, delay_ticks: u16) void {
    ragdoll.resurrection_tick = delay_ticks;
    ragdoll.active = true;
}

/// Check if resurrection is ready
pub fn isResurrectionReady(ragdoll: *const Ragdoll) bool {
    return ragdoll.resurrection_tick == 0 and !ragdoll.active;
}

/// Get system for external iteration
pub fn getSystem() *RagdollSystem {
    return &g_ragdoll_system;
}
