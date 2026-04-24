//! Ragdoll - Hierarchical Joint Chain Physics
//!
//! Phase 10: Ragdoll physics with joint chains, limb constraints, and physics state
//! Handles: Body part creation, joint constraints, impulse application, resurrection

const std = @import("std");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const joint = @import("joint.zig");

pub const RagdollPart = struct {
    joint_idx: u8,
    pos_offset_x: i8,
    pos_offset_y: i8,
    pos_offset_z: i8,
    mass_ratio: f32,
    radius: u8,
    active: bool,
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
            .parts = [_]RagdollPart{.{.joint_idx=0, .pos_offset_x=0, .pos_offset_y=0, .pos_offset_z=0, .mass_ratio=0, .radius=0, .active=false}} ** MAX_RAGDOLL_PARTS,
            .joints = undefined,
            .part_count = 0,
            .joint_count = 0,
            .active = false,
            .resurrection_tick = 0,
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

    for (HumanoidRagdollLayout.parts, 0..) |p, i| {
        ragdoll.parts[i] = .{
            .joint_idx = 0,
            .pos_offset_x = p.ox,
            .pos_offset_y = p.oy,
            .pos_offset_z = p.oz,
            .mass_ratio = p.mass,
            .radius = p.radius,
            .active = true,
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

    _ = base_x; _ = base_y; _ = base_z;
    return ragdoll;
}

/// Create ragdoll from entity
pub fn createFromEntity(entity: *const entity16.Entity16) ?*Ragdoll {
    _ = entity;
    return createHumanoid(0, 0, 0);
}

/// Apply impulse to ragdoll at world point
pub fn applyImpulse(
    ragdoll: *Ragdoll,
    impulse_x: f32,
    impulse_y: f32,
    impulse_z: f32,
    point_x: f32,
    point_y: f32,
    point_z: f32,
    base_x: i32,
    base_y: i32,
    base_z: i32,
) void {
    if (!ragdoll.active) return;

    var i: u8 = 0;
    while (i < ragdoll.part_count) : (i += 1) {
        const part = &ragdoll.parts[i];
        if (!part.active) continue;

        const world_x = @as(f32, @floatFromInt(base_x)) + @as(f32, @floatFromInt(part.pos_offset_x));
        const world_y = @as(f32, @floatFromInt(base_y)) + @as(f32, @floatFromInt(part.pos_offset_y));
        const world_z = @as(f32, @floatFromInt(base_z)) + @as(f32, @floatFromInt(part.pos_offset_z));

        const dx = point_x - world_x;
        const dy = point_y - world_y;
        const dz = point_z - world_z;
        const dist = @sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > 0.1) {
            const falloff = 1.0 - (dist / 50.0);
            const scale = part.mass_ratio * falloff;

            _ = impulse_x; _ = impulse_y; _ = impulse_z;
            _ = scale;
        }
    }
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

/// Update ragdoll physics
pub fn update(ragdoll: *Ragdoll, dt: f32) void {
    _ = dt;
    if (!ragdoll.active) return;

    if (ragdoll.resurrection_tick > 0) {
        ragdoll.resurrection_tick -= 1;
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
