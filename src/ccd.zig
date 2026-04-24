//! CCD - Continuous Collision Detection
//!
//! Phase 3: Continuous Collision Detection for high-speed objects
//! Prevents tunneling through thin objects

const std = @import("std");
const physics = @import("physics.zig");

/// Time of impact result
pub const TOI = struct {
    hit: bool,
    time: f32,        // 0.0 to 1.0, fraction of timestep
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    entity_id: u16,
};

/// Swept AABB for CCD
pub const SweptAABB = struct {
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
};

pub fn makeSweptAABB(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, vel_x: f32, vel_y: f32, vel_z: f32) SweptAABB {
    return .{
        .min_x = min_x, .min_y = min_y, .min_z = min_z,
        .max_x = max_x, .max_y = max_y, .max_z = max_z,
        .vel_x = vel_x, .vel_y = vel_y, .vel_z = vel_z,
    };
}

/// Check if swept AABB collides with static AABB
/// Uses Separating Axis Theorem (SAT)
pub fn sweptAABBvsAABB(swept: SweptAABB, target: physics.AABB) TOI {
    var toi: f32 = 0.0;
    var max_toi: f32 = 1.0;

    // Calculate velocity per axis
    const inv_vel_x = if (swept.vel_x > 0.0001) 1.0 / swept.vel_x else if (swept.vel_x < -0.0001) 1.0 / swept.vel_x else 0.0;
    const inv_vel_y = if (swept.vel_y > 0.0001) 1.0 / swept.vel_y else if (swept.vel_y < -0.0001) 1.0 / swept.vel_y else 0.0;
    const inv_vel_z = if (swept.vel_z > 0.0001) 1.0 / swept.vel_z else if (swept.vel_z < -0.0001) 1.0 / swept.vel_z else 0.0;

    // X-axis
    if (inv_vel_x != 0.0) {
        if (swept.vel_x > 0.0) {
            const t1 = (@as(f32, @floatFromInt(target.min_x)) - swept.max_x) * inv_vel_x;
            const t2 = (@as(f32, @floatFromInt(target.max_x)) - swept.min_x) * inv_vel_x;
            if (t1 > toi) {
                toi = t1;
                max_toi = t2;
            }
        } else if (swept.vel_x < 0.0) {
            const t1 = (@as(f32, @floatFromInt(target.max_x)) - swept.min_x) * inv_vel_x;
            const t2 = (@as(f32, @floatFromInt(target.min_x)) - swept.max_x) * inv_vel_x;
            if (t1 > toi) {
                toi = t1;
                max_toi = t2;
            }
        }
    }

    // Y-axis
    if (inv_vel_y != 0.0) {
        if (swept.vel_y > 0.0) {
            const t1 = (@as(f32, @floatFromInt(target.min_y)) - swept.max_y) * inv_vel_y;
            const t2 = (@as(f32, @floatFromInt(target.max_y)) - swept.min_y) * inv_vel_y;
            if (t1 > toi) {
                toi = t1;
                max_toi = t2;
            }
        } else if (swept.vel_y < 0.0) {
            const t1 = (@as(f32, @floatFromInt(target.max_y)) - swept.min_y) * inv_vel_y;
            const t2 = (@as(f32, @floatFromInt(target.min_y)) - swept.max_y) * inv_vel_y;
            if (t1 > toi) {
                toi = t1;
                max_toi = t2;
            }
        }
    }

    // Z-axis
    if (inv_vel_z != 0.0) {
        if (swept.vel_z > 0.0) {
            const t1 = (@as(f32, @floatFromInt(target.min_z)) - swept.max_z) * inv_vel_z;
            const t2 = (@as(f32, @floatFromInt(target.max_z)) - swept.min_z) * inv_vel_z;
            if (t1 > toi) {
                toi = t1;
                max_toi = t2;
            }
        } else if (swept.vel_z < 0.0) {
            const t1 = (@as(f32, @floatFromInt(target.max_z)) - swept.min_z) * inv_vel_z;
            const t2 = (@as(f32, @floatFromInt(target.min_z)) - swept.max_z) * inv_vel_z;
            if (t1 > toi) {
                toi = t1;
                max_toi = t2;
            }
        }
    }

    if (toi <= max_toi and toi >= 0.0 and toi <= 1.0) {
        return .{
            .hit = true,
            .time = toi,
            .normal_x = if (swept.vel_x > 0.0) -1.0 else 1.0,
            .normal_y = if (swept.vel_y > 0.0) -1.0 else 1.0,
            .normal_z = if (swept.vel_z > 0.0) -1.0 else 1.0,
            .entity_id = 255,
        };
    }

    return .{ .hit = false, .time = 1.0, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 255 };
}

/// Compute time of impact between two moving AABBs
pub fn computeTOI(a_min_x: f32, a_min_y: f32, a_min_z: f32, a_max_x: f32, a_max_y: f32, a_max_z: f32, va_x: f32, va_y: f32, va_z: f32, b_min_x: f32, b_min_y: f32, b_min_z: f32, b_max_x: f32, b_max_y: f32, b_max_z: f32, vb_x: f32, vb_y: f32, vb_z: f32) TOI {
    const rel_vel_x = va_x - vb_x;
    const rel_vel_y = va_y - vb_y;
    const rel_vel_z = va_z - vb_z;

    const swept = makeSweptAABB(a_min_x, a_min_y, a_min_z, a_max_x, a_max_y, a_max_z, rel_vel_x, rel_vel_y, rel_vel_z);
    const target = physics.AABB{ .min_x = @intFromFloat(b_min_x), .min_y = @intFromFloat(b_min_y), .min_z = @intFromFloat(b_min_z), .max_x = @intFromFloat(b_max_x), .max_y = @intFromFloat(b_max_y), .max_z = @intFromFloat(b_max_z) };

    return sweptAABBvsAABB(swept, target);
}

/// Angular sweep for rotating objects
/// Returns AABB that encompasses rotated box
pub fn angularSweepAABB(center_x: f32, center_y: f32, center_z: f32, half_extent_x: f32, half_extent_y: f32, half_extent_z: f32) physics.AABB {
    // Simplified: just expand by sqrt(2) for diagonal rotation
    const expansion = @sqrt(2.0);
    const ex = half_extent_x * expansion;
    const ey = half_extent_y * expansion;
    const ez = half_extent_z * expansion;

    return .{
        .min_x = @intFromFloat(center_x - ex),
        .min_y = @intFromFloat(center_y - ey),
        .min_z = @intFromFloat(center_z - ez),
        .max_x = @intFromFloat(center_x + ex),
        .max_y = @intFromFloat(center_y + ey),
        .max_z = @intFromFloat(center_z + ez),
    };
}

/// Check if a point is inside an AABB
pub fn pointInAABB(px: f32, py: f32, pz: f32, box: physics.AABB) bool {
    return px >= @as(f32, @floatFromInt(box.min_x)) and px <= @as(f32, @floatFromInt(box.max_x)) and
           py >= @as(f32, @floatFromInt(box.min_y)) and py <= @as(f32, @floatFromInt(box.max_y)) and
           pz >= @as(f32, @floatFromInt(box.min_z)) and pz <= @as(f32, @floatFromInt(box.max_z));
}

/// Check CCD for entity falling at high speed
/// Returns true if collision would occur within the timestep
pub fn checkCCDCollision(pos_x: i32, pos_y: i32, pos_z: i32, vel_x: i16, vel_y: i16, vel_z: i16, extent: i32, target: physics.AABB) TOI {
    const swept = makeSweptAABB(
        @as(f32, @floatFromInt(pos_x)) - @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_y)) - @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_z)) - @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_x)) + @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_y)) + @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_z)) + @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(vel_x)),
        @as(f32, @floatFromInt(vel_y)),
        @as(f32, @floatFromInt(vel_z)),
    );
    return sweptAABBvsAABB(swept, target);
}
