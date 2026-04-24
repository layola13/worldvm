//! Query Penetration - L4 Penetration Queries
//!
//! Task A: Unified Query Layer
//! Penetration depth and depenetration direction queries

const std = @import("std");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const query_overlap = @import("query_overlap.zig");

usingnamespace query_types;

/// Compute penetration of AABB into world
pub fn computePenetrationAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) PenetrationResult {
    var result: PenetrationResult = .{};

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
                const gx = @intFromFloat(x);
                const gy = @intFromFloat(y);
                const gz = @intFromFloat(z);

                // Check if this voxel is inside the AABB
                if (x >= min_x and x < max_x and y >= min_y and y < max_y and z >= min_z and z < max_z) {
                    const hit = query_world.queryAnyVoxel(world, gx, gy, gz, filter);
                    if (hit.hit) {
                        result.overlapping = true;
                        result.instance_idx = hit.instance_idx;

                        // Calculate how deep we are in this voxel
                        // Voxel is [gx, gx+1] x [gy, gy+1] x [gz, gz+1]
                        // We compute penetration from each face
                        const depth_from_min_x = x - min_x;
                        const depth_from_max_x = max_x - (x + 1);
                        const depth_from_min_y = y - min_y;
                        const depth_from_max_y = max_y - (y + 1);
                        const depth_from_min_z = z - min_z;
                        const depth_from_max_z = max_z - (z + 1);

                        // Find minimum penetration
                        var min_depth = depth_from_min_x;
                        result.dir_x = -1; result.dir_y = 0; result.dir_z = 0;

                        if (depth_from_max_x < min_depth) {
                            min_depth = depth_from_max_x;
                            result.dir_x = 1; result.dir_y = 0; result.dir_z = 0;
                        }
                        if (depth_from_min_y < min_depth) {
                            min_depth = depth_from_min_y;
                            result.dir_x = 0; result.dir_y = -1; result.dir_z = 0;
                        }
                        if (depth_from_max_y < min_depth) {
                            min_depth = depth_from_max_y;
                            result.dir_x = 0; result.dir_y = 1; result.dir_z = 0;
                        }
                        if (depth_from_min_z < min_depth) {
                            min_depth = depth_from_min_z;
                            result.dir_x = 0; result.dir_y = 0; result.dir_z = -1;
                        }
                        if (depth_from_max_z < min_depth) {
                            min_depth = depth_from_max_z;
                            result.dir_x = 0; result.dir_y = 0; result.dir_z = 1;
                        }

                        result.depth = min_depth;
                        return result;
                    }
                }
            }
        }
    }

    return result;
}

/// Compute penetration of capsule into world
pub fn computePenetrationCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) PenetrationResult {
    var result: PenetrationResult = .{};

    const step_size: f32 = 0.5; // Finer resolution for penetration
    const radius_sq = radius * radius;

    const start_y = center_y - half_height - radius;
    const end_y = center_y + half_height + radius;

    var best_depth: f32 = 0;
    var best_dir_x: f32 = 0;
    var best_dir_y: f32 = 1;
    var best_dir_z: f32 = 0;

    var y: f32 = @floor(start_y);
    while (y <= end_y) : (y += step_size) {
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
                        // Check if this point is inside the capsule
                        const point_dist_sq = (x - center_x) * (x - center_x) +
                                              (y - center_y) * (y - center_y) +
                                              (z - center_z) * (z - center_z);

                        if (point_dist_sq <= radius_sq) {
                            const hit = query_world.queryAnyVoxel(world, @intFromFloat(x), @intFromFloat(y), @intFromFloat(z), filter);
                            if (hit.hit) {
                                result.overlapping = true;
                                result.instance_idx = hit.instance_idx;

                                // Calculate direction from hit point to center
                                const dir_x = center_x - x;
                                const dir_y = center_y - y;
                                const dir_z = center_z - z;
                                const dir_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);

                                if (dir_len > 0.001) {
                                    const depth = radius - dir_len;
                                    if (depth > best_depth) {
                                        best_depth = depth;
                                        best_dir_x = dir_x / dir_len;
                                        best_dir_y = dir_y / dir_len;
                                        best_dir_z = dir_z / dir_len;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    result.depth = best_depth;
    result.dir_x = best_dir_x;
    result.dir_y = best_dir_y;
    result.dir_z = best_dir_z;

    return result;
}
