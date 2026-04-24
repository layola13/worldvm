//! Joint System - Constraints for Rigid Body Physics
//!
//! Phase 2: Joint constraints (Fixed, Hinge, Slider, Spring, BallSocket)

const std = @import("std");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");

pub const JointType = enum(u8) {
    fixed = 0,
    hinge = 1,
    slider = 2,
    spring = 3,
    ball_socket = 4,
};

pub const Joint = struct {
    joint_type: JointType,
    entity_a: u16,
    entity_b: u16,
    anchor_a_x: i32,
    anchor_a_y: i32,
    anchor_a_z: i32,
    anchor_b_x: i32,
    anchor_b_y: i32,
    anchor_b_z: i32,
    axis_x: i32 = 0,
    axis_y: i32 = 0,
    axis_z: i32 = 1,
    limit_min: f32 = -3.14159,
    limit_max: f32 = 3.14159,
    breaking_force: f32 = 0,
    stiffness: f32 = 1000,
    damping: f32 = 100,
    enabled: bool = true,
};

pub const MAX_JOINTS: usize = 64;

pub const JointSystem = struct {
    joints: [MAX_JOINTS]Joint,
    joint_count: u8 = 0,
};

pub fn init(system: *JointSystem) void {
    system.joint_count = 0;
}

pub fn addJoint(system: *JointSystem, joint: Joint) ?u8 {
    if (system.joint_count >= MAX_JOINTS) return null;
    const idx = system.joint_count;
    system.joints[idx] = joint;
    system.joint_count += 1;
    return idx;
}

pub fn removeJoint(system: *JointSystem, joint_idx: u8) void {
    if (joint_idx >= system.joint_count) return;
    var i = joint_idx;
    while (i < system.joint_count - 1) : (i += 1) {
        system.joints[i] = system.joints[i + 1];
    }
    system.joint_count -= 1;
}

/// Solve distance constraint (Fixed joint, BallSocket)
pub fn solveDistanceConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
) void {
    // Calculate current distance between anchors
    const dx = inst_b.pos_x + joint.anchor_b_x - inst_a.pos_x - joint.anchor_a_x;
    const dy = inst_b.pos_y + joint.anchor_b_y - inst_a.pos_y - joint.anchor_a_y;
    const dz = inst_b.pos_z + joint.anchor_b_z - inst_a.pos_z - joint.anchor_a_z;

    // For fixed joint, anchors should be coincident
    const target_dist: f32 = 0.0;
    const dist_sq = dx * dx + dy * dy + dz * dz;
    const current_dist = @sqrt(@as(f32, @floatFromInt(dist_sq)));

    if (current_dist < 0.001) return;

    const correction = (target_dist - current_dist) / current_dist;

    const ent_a = &entities[inst_a.entity_id];
    const ent_b = &entities[inst_b.entity_id];
    const mass_a: f32 = @floatFromInt(ent_a.physics.mass);
    const mass_b: f32 = @floatFromInt(ent_b.physics.mass);
    const total_mass: f32 = mass_a + mass_b;
    if (total_mass < 0.001) return;

    const ratio_a = mass_b / total_mass;
    const ratio_b = mass_a / total_mass;

    const cx: f32 = @as(f32, @floatFromInt(dx)) * correction * ratio_a;
    const cy: f32 = @as(f32, @floatFromInt(dy)) * correction * ratio_a;
    const cz: f32 = @as(f32, @floatFromInt(dz)) * correction * ratio_a;

    inst_a.pos_x += @as(i32, @intFromFloat(@round(cx)));
    inst_a.pos_y += @as(i32, @intFromFloat(@round(cy)));
    inst_a.pos_z += @as(i32, @intFromFloat(@round(cz)));

    inst_b.pos_x -= @as(i32, @intFromFloat(@round(cx * ratio_b / ratio_a)));
    inst_b.pos_y -= @as(i32, @intFromFloat(@round(cy * ratio_b / ratio_a)));
    inst_b.pos_z -= @as(i32, @intFromFloat(@round(cz * ratio_b / ratio_a)));
}

/// Solve hinge constraint (rotation around axis only)
pub fn solveHingeConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
) void {
    // Hinge: allows rotation around axis, resists translation and rotation off axis
    // Current implementation: maintain anchor point coincidence
    const dx = inst_b.pos_x + joint.anchor_b_x - inst_a.pos_x - joint.anchor_a_x;
    const dy = inst_b.pos_y + joint.anchor_b_y - inst_a.pos_y - joint.anchor_a_y;
    const dz = inst_b.pos_z + joint.anchor_b_z - inst_a.pos_z - joint.anchor_a_z;

    const dist_sq = dx * dx + dy * dy + dz * dz;
    const current_dist = @sqrt(@as(f32, @floatFromInt(dist_sq)));
    if (current_dist < 0.001) return;

    const correction = -current_dist / current_dist;

    const ent_a = &entities[inst_a.entity_id];
    const ent_b = &entities[inst_b.entity_id];
    const mass_a: f32 = @floatFromInt(ent_a.physics.mass);
    const mass_b: f32 = @floatFromInt(ent_b.physics.mass);
    const total_mass: f32 = mass_a + mass_b;
    if (total_mass < 0.001) return;

    const ratio_a = mass_b / total_mass;

    inst_a.pos_x += @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(dx)) * correction * ratio_a)));
    inst_a.pos_y += @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(dy)) * correction * ratio_a)));
    inst_a.pos_z += @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(dz)) * correction * ratio_a)));

    inst_b.pos_x -= @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(dx)) * correction * (1.0 - ratio_a))));
    inst_b.pos_y -= @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(dy)) * correction * (1.0 - ratio_a))));
    inst_b.pos_z -= @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(dz)) * correction * (1.0 - ratio_a))));

    // Apply angular correction based on hinge axis
    if (joint.axis_x != 0) {
        const angular_corr: f32 = @as(f32, @floatFromInt(dz)) * correction * 0.1;
        inst_a.rot_roll = @truncate((@as(u16, inst_a.rot_roll) + @as(u8, @intFromFloat(@round(angular_corr)))) & 0xFF);
    }
    if (joint.axis_y != 0) {
        const angular_corr: f32 = @as(f32, @floatFromInt(dx)) * correction * 0.1;
        inst_a.rot_yaw = @truncate((@as(u16, inst_a.rot_yaw) + @as(u8, @intFromFloat(@round(angular_corr)))) & 0xFF);
    }
}

/// Solve slider constraint (translation along axis only)
pub fn solveSliderConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
) void {
    // Slider: allows translation along axis only
    const dx = inst_b.pos_x + joint.anchor_b_x - inst_a.pos_x - joint.anchor_a_x;
    const dy = inst_b.pos_y + joint.anchor_b_y - inst_a.pos_y - joint.anchor_a_y;
    const dz = inst_b.pos_z + joint.anchor_b_z - inst_a.pos_z - joint.anchor_a_z;

    // Project onto slider axis
    const ax = @as(f32, @floatFromInt(joint.axis_x));
    const ay = @as(f32, @floatFromInt(joint.axis_y));
    const az = @as(f32, @floatFromInt(joint.axis_z));
    const axis_len = @sqrt(ax * ax + ay * ay + az * az);
    if (axis_len < 0.001) return;

    const nx = ax / axis_len;
    const ny = ay / axis_len;
    const nz = az / axis_len;

    const dp: f32 = @as(f32, @floatFromInt(dx)) * nx + @as(f32, @floatFromInt(dy)) * ny + @as(f32, @floatFromInt(dz)) * nz;

    const cx = nx * dp;
    const cy = ny * dp;
    const cz = nz * dp;

    const ent_a = &entities[inst_a.entity_id];
    const ent_b = &entities[inst_b.entity_id];
    const mass_a: f32 = @floatFromInt(ent_a.physics.mass);
    const mass_b: f32 = @floatFromInt(ent_b.physics.mass);
    const total_mass: f32 = mass_a + mass_b;
    if (total_mass < 0.001) return;

    const ratio_a = mass_b / total_mass;

    // Correct along axis only
    inst_a.pos_x += @as(i32, @intFromFloat(@round(cx * ratio_a)));
    inst_a.pos_y += @as(i32, @intFromFloat(@round(cy * ratio_a)));
    inst_a.pos_z += @as(i32, @intFromFloat(@round(cz * ratio_a)));

    inst_b.pos_x -= @as(i32, @intFromFloat(@round(cx * (1.0 - ratio_a))));
    inst_b.pos_y -= @as(i32, @intFromFloat(@round(cy * (1.0 - ratio_a))));
    inst_b.pos_z -= @as(i32, @intFromFloat(@round(cz * (1.0 - ratio_a))));
}

/// Solve spring constraint (elastic connection)
pub fn solveSpringConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
) void {
    const dx = inst_b.pos_x + joint.anchor_b_x - inst_a.pos_x - joint.anchor_a_x;
    const dy = inst_b.pos_y + joint.anchor_b_y - inst_a.pos_y - joint.anchor_a_y;
    const dz = inst_b.pos_z + joint.anchor_b_z - inst_a.pos_z - joint.anchor_a_z;

    const dist_sq = dx * dx + dy * dy + dz * dz;
    const current_dist = @sqrt(@as(f32, @floatFromInt(dist_sq)));
    if (current_dist < 0.001) return;

    const rest_length = joint.stiffness;
    const extension = current_dist - rest_length;

    // Spring force: F = -kx
    const force: f32 = extension * joint.damping * 0.01;

    const cx: f32 = @as(f32, @floatFromInt(dx)) / current_dist * force;
    const cy: f32 = @as(f32, @floatFromInt(dy)) / current_dist * force;
    const cz: f32 = @as(f32, @floatFromInt(dz)) / current_dist * force;

    const ent_a = &entities[inst_a.entity_id];
    const ent_b = &entities[inst_b.entity_id];
    const mass_a: f32 = @floatFromInt(ent_a.physics.mass);
    const mass_b: f32 = @floatFromInt(ent_b.physics.mass);
    const total_mass: f32 = mass_a + mass_b;
    if (total_mass < 0.001) return;

    const ratio_a = mass_b / total_mass;

    inst_a.pos_x += @as(i32, @intFromFloat(@round(cx * ratio_a)));
    inst_a.pos_y += @as(i32, @intFromFloat(@round(cy * ratio_a)));
    inst_a.pos_z += @as(i32, @intFromFloat(@round(cz * ratio_a)));

    inst_b.pos_x -= @as(i32, @intFromFloat(@round(cx * ratio_a)));
    inst_b.pos_y -= @as(i32, @intFromFloat(@round(cy * ratio_a)));
    inst_b.pos_z -= @as(i32, @intFromFloat(@round(cz * ratio_a)));
}

/// Check if joint should break based on force
pub fn shouldBreakJoint(joint: *Joint, force_magnitude: f32) bool {
    if (joint.breaking_force <= 0) return false;
    return force_magnitude >= joint.breaking_force;
}

/// Main solver - iterate through all joints
pub fn solveJoints(
    system: *JointSystem,
    instances: [*]scene32.Instance,
    instance_count: u8,
    entities: []entity16.Entity16,
) void {
    const iterations: u8 = 4;
    var iter: u8 = 0;

    while (iter < iterations) : (iter += 1) {
        var j: u8 = 0;
        while (j < system.joint_count) : (j += 1) {
            const joint = &system.joints[j];
            if (!joint.enabled) continue;

            if (joint.entity_a >= instance_count or joint.entity_b >= instance_count) continue;

            const inst_a = &instances[joint.entity_a];
            const inst_b = &instances[joint.entity_b];

            if (inst_a.state == .broken or inst_b.state == .broken) continue;

            switch (joint.joint_type) {
                .fixed, .ball_socket => {
                    solveDistanceConstraint(inst_a, inst_b, joint, entities);
                },
                .hinge => {
                    solveHingeConstraint(inst_a, inst_b, joint, entities);
                },
                .slider => {
                    solveSliderConstraint(inst_a, inst_b, joint, entities);
                },
                .spring => {
                    solveSpringConstraint(inst_a, inst_b, joint, entities);
                },
            }
        }
    }
}

/// Solver for external use with slices
pub fn solveJointsForTick(
    instances: []scene32.Instance,
    joints: []const Joint,
    entities: []entity16.Entity16,
) void {
    if (joints.len == 0) return;
    const iterations: u8 = 4;
    var iter: u8 = 0;

    while (iter < iterations) : (iter += 1) {
        for (joints) |*joint| {
            if (!joint.enabled) continue;

            if (joint.entity_a >= instances.len or joint.entity_b >= instances.len) continue;

            const inst_a = &instances[joint.entity_a];
            const inst_b = &instances[joint.entity_b];

            if (inst_a.state == .broken or inst_b.state == .broken) continue;

            switch (joint.joint_type) {
                .fixed, .ball_socket => {
                    solveDistanceConstraint(inst_a, inst_b, joint, entities);
                },
                .hinge => {
                    solveHingeConstraint(inst_a, inst_b, joint, entities);
                },
                .slider => {
                    solveSliderConstraint(inst_a, inst_b, joint, entities);
                },
                .spring => {
                    solveSpringConstraint(inst_a, inst_b, joint, entities);
                },
            }
        }
    }
}
