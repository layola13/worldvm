//! Query Sweep - L3 Sweep Queries
//!
//! Task A: Unified Query Layer
//! Sphere, capsule, and box sweep queries

const std = @import("std");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const query_raycast = @import("query_raycast.zig");

usingnamespace query_types;

/// Sphere sweep against world
pub fn sphereCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    // Simplified sphere cast: use center point raycast with tolerance
    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        // No movement, just check for overlap at current position
        const hit = query_world.queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y)), @intFromFloat(@floor(center_z)), filter);
        if (hit.hit) {
            return .{
                .hit = true,
                .distance = 0,
                .position_x = center_x,
                .position_y = center_y,
                .position_z = center_z,
                .instance_idx = hit.instance_idx,
                .entity_id = hit.entity_id,
                .hit_environment = hit.hit_environment,
                .hit_sensor = hit.hit_sensor,
            };
        }
        return .{};
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;

    const ray = QueryRay{
        .origin_x = center_x,
        .origin_y = center_y,
        .origin_z = center_z,
        .dir_x = ndx,
        .dir_y = ndy,
        .dir_z = ndz,
        .max_distance = max_distance,
    };

    // For sphere, we offset the ray origin by radius in the opposite direction
    const offset_ray = QueryRay{
        .origin_x = center_x - ndx * radius,
        .origin_y = center_y - ndy * radius,
        .origin_z = center_z - ndz * radius,
        .dir_x = ndx,
        .dir_y = ndy,
        .dir_z = ndz,
        .max_distance = max_distance + radius,
    };

    return query_raycast.raycastSingle(world, offset_ray, filter);
}

/// Capsule sweep - capsule is center + radius + half_height (total height = 2*half_height + 2*radius)
pub fn capsuleCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        // No movement, check overlap at current position
        var hit = query_world.queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y)), @intFromFloat(@floor(center_z)), filter);
        if (hit.hit) return hit;
        // Also check top and bottom of capsule
        hit = query_world.queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y + half_height)), @intFromFloat(@floor(center_z)), filter);
        if (hit.hit) return hit;
        hit = query_world.queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y - half_height)), @intFromFloat(@floor(center_z)), filter);
        return hit;
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;

    // Capsule: sample at center, top, and bottom
    // Bottom is at center - half_height, top is at center + half_height
    const total_height = half_height * 2 + radius * 2;
    const sample_count: u32 = @intFromFloat(@max(1, @floor(total_height / 2)));

    var closest_hit: QueryHit = .{};

    var i: u32 = 0;
    while (i < sample_count) : (i += 1) {
        const offset_y = (@as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(sample_count)) - 0.5) * (half_height * 2 + radius);
        const sample_y = center_y + offset_y;

        const offset_ray = QueryRay{
            .origin_x = center_x - ndx * radius,
            .origin_y = sample_y - ndy * radius,
            .origin_z = center_z - ndz * radius,
            .dir_x = ndx,
            .dir_y = ndy,
            .dir_z = ndz,
            .max_distance = max_distance + radius,
        };

        const hit = query_raycast.raycastSingle(world, offset_ray, filter);
        if (hit.hit and (!closest_hit.hit or hit.distance < closest_hit.distance)) {
            closest_hit = hit;
        }
    }

    return closest_hit;
}

/// Box sweep - simplified using center raycast
pub fn boxCast(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    const cx = (min_x + max_x) * 0.5;
    const cy = (min_y + max_y) * 0.5;
    const cz = (min_z + max_z) * 0.5;

    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        // No movement, check overlap at center
        return query_world.queryAnyVoxel(world, @intFromFloat(@floor(cx)), @intFromFloat(@floor(cy)), @intFromFloat(@floor(cz)), filter);
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;

    // Offset origin to box surface in opposite direction of movement
    const offset_x = if (ndx > 0) -((max_x - min_x) * 0.5) else ((max_x - min_x) * 0.5);
    const offset_y = if (ndy > 0) -((max_y - min_y) * 0.5) else ((max_y - min_y) * 0.5);
    const offset_z = if (ndz > 0) -((max_z - min_z) * 0.5) else ((max_z - min_z) * 0.5);

    const ray = QueryRay{
        .origin_x = cx + offset_x,
        .origin_y = cy + offset_y,
        .origin_z = cz + offset_z,
        .dir_x = ndx,
        .dir_y = ndy,
        .dir_z = ndz,
        .max_distance = max_distance + @sqrt(offset_x * offset_x + offset_y * offset_y + offset_z * offset_z),
    };

    return query_raycast.raycastSingle(world, ray, filter);
}
