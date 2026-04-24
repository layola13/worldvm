//! Query Overlap - L1 Overlap Queries
//!
//! Task A: Unified Query Layer
//! AABB, sphere, capsule overlap queries

const std = @import("std");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");

usingnamespace query_types;

/// Check AABB overlap with world
pub fn overlapAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) OverlapResult {
    var result: OverlapResult = .{};

    const start_x = @floor(min_x);
    const start_y = @floor(min_y);
    const start_z = @floor(min_z);
    const end_x = @ceil(max_x);
    const end_y = @ceil(max_y);
    const end_z = @ceil(max_z);

    var x: f32 = start_x;
    while (x <= end_x) : (x += 1) {
        var y: f32 = start_y;
        while (y <= end_y) : (y += 1) {
            var z: f32 = start_z;
            while (z <= end_z) : (z += 1) {
                const hit = query_world.queryAnyVoxel(world, @intFromFloat(x), @intFromFloat(y), @intFromFloat(z), filter);
                if (hit.hit_environment) {
                    result.hit = true;
                    result.environment_overlap = true;
                    result.count += 1;
                    if (result.first_instance_idx == -1) {
                        result.first_instance_idx = -1; // Environment has no instance idx
                    }
                }
                if (hit.instance_idx >= 0) {
                    result.hit = true;
                    result.count += 1;
                    if (result.first_instance_idx == -1) {
                        result.first_instance_idx = hit.instance_idx;
                    }
                }
            }
        }
    }

    return result;
}

/// Check sphere overlap with world
pub fn overlapSphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, filter: QueryFilter) OverlapResult {
    var result: OverlapResult = .{};

    const step_size: f32 = 1.0; // Voxel resolution
    const radius_sq = radius * radius;

    const start_x = @floor(center_x - radius);
    const start_y = @floor(center_y - radius);
    const start_z = @floor(center_z - radius);
    const end_x = @ceil(center_x + radius);
    const end_y = @ceil(center_y + radius);
    const end_z = @ceil(center_z + radius);

    var x: f32 = start_x;
    while (x <= end_x) : (x += step_size) {
        var y: f32 = start_y;
        while (y <= end_y) : (y += step_size) {
            var z: f32 = start_z;
            while (z <= end_z) : (z += step_size) {
                const dx = x - center_x;
                const dy = y - center_y;
                const dz = z - center_z;
                const dist_sq = dx * dx + dy * dy + dz * dz;

                if (dist_sq <= radius_sq) {
                    const hit = query_world.queryAnyVoxel(world, @intFromFloat(x), @intFromFloat(y), @intFromFloat(z), filter);
                    if (hit.hit_environment) {
                        result.hit = true;
                        result.environment_overlap = true;
                        result.count += 1;
                    }
                    if (hit.instance_idx >= 0) {
                        result.hit = true;
                        result.count += 1;
                        if (result.first_instance_idx == -1) {
                            result.first_instance_idx = hit.instance_idx;
                        }
                    }
                }
            }
        }
    }

    return result;
}

/// Check capsule overlap with world
pub fn overlapCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) OverlapResult {
    var result: OverlapResult = .{};

    const step_size: f32 = 1.0;
    const radius_sq = radius * radius;

    // Capsule spans from (center_y - half_height - radius) to (center_y + half_height + radius)
    const start_y = center_y - half_height - radius;
    const end_y = center_y + half_height + radius;

    var y: f32 = @floor(start_y);
    while (y <= end_y) : (y += step_size) {
        // At each y level, the capsule cross-section is a circle with center at (center_x, center_z)
        const dy = y - center_y;
        const circle_radius_sq = radius_sq - dy * dy;

        if (circle_radius_sq > 0) {
            const circle_radius = @sqrt(circle_radius_sq);
            var x: f32 = @floor(center_x - circle_radius);
            while (x <= center_x + circle_radius) : (x += step_size) {
                var z: f32 = @floor(center_z - circle_radius);
                while (z <= center_z + circle_radius) : (z += step_size) {
                    const dx = x - center_x;
                    const dz = z - center_z;
                    const dist_sq = dx * dx + dz * dz;

                    if (dist_sq <= circle_radius_sq) {
                        const hit = query_world.queryAnyVoxel(world, @intFromFloat(x), @intFromFloat(y), @intFromFloat(z), filter);
                        if (hit.hit_environment) {
                            result.hit = true;
                            result.environment_overlap = true;
                            result.count += 1;
                        }
                        if (hit.instance_idx >= 0) {
                            result.hit = true;
                            result.count += 1;
                            if (result.first_instance_idx == -1) {
                                result.first_instance_idx = hit.instance_idx;
                            }
                        }
                    }
                }
            }
        }
    }

    return result;
}
