//! Query Raycast - L2 Ray Queries
//!
//! Task A: Unified Query Layer
//! DDA voxel raycast with proper filtering

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const address = @import("address.zig");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");

usingnamespace query_types;

/// DDA Voxel Raycast with query filter support
/// Returns first voxel hit along ray
pub fn raycastSingle(world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter) QueryHit {
    var px: i32 = @intFromFloat(@floor(ray.origin_x));
    var py: i32 = @intFromFloat(@floor(ray.origin_y));
    var pz: i32 = @intFromFloat(@floor(ray.origin_z));

    const step_x: i32 = if (ray.dir_x >= 0) 1 else -1;
    const step_y: i32 = if (ray.dir_y >= 0) 1 else -1;
    const step_z: i32 = if (ray.dir_z >= 0) 1 else -1;

    const tDelta_x: f32 = if (ray.dir_x != 0) @abs(1.0 / ray.dir_x) else 1e10;
    const tDelta_y: f32 = if (ray.dir_y != 0) @abs(1.0 / ray.dir_y) else 1e10;
    const tDelta_z: f32 = if (ray.dir_z != 0) @abs(1.0 / ray.dir_z) else 1e10;

    var tMax_x: f32 = if (ray.dir_x >= 0) (ray.origin_x - @as(f32, @floatFromInt(px))) * tDelta_x else (@as(f32, @floatFromInt(px)) + 1.0 - ray.origin_x) * tDelta_x;
    var tMax_y: f32 = if (ray.dir_y >= 0) (ray.origin_y - @as(f32, @floatFromInt(py))) * tDelta_y else (@as(f32, @floatFromInt(py)) + 1.0 - ray.origin_y) * tDelta_y;
    var tMax_z: f32 = if (ray.dir_z >= 0) (ray.origin_z - @as(f32, @floatFromInt(pz))) * tDelta_z else (@as(f32, @floatFromInt(pz)) + 1.0 - ray.origin_z) * tDelta_z;

    var t: f32 = 0.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    const max_steps: i32 = 2048;
    var steps: i32 = 0;

    while (t <= ray.max_distance and steps < max_steps) : (steps += 1) {
        // Check environment occupancy at current position
        if (!filter.ignore_environment) {
            const addr = address.encode(.{
                .world = 0,
                .px = @as(u5, @intCast(@divFloor(px, 32))),
                .py = @as(u5, @intCast(@divFloor(py, 32))),
                .pz = @as(u5, @intCast(@divFloor(pz, 32))),
                .lx = @as(u5, @intCast(@mod(px, 32))),
                .ly = @as(u5, @intCast(@mod(py, 32))),
                .lz = @as(u5, @intCast(@mod(pz, 32))),
            });
            if (world.s1024.getVoxelAtGlobal(addr) catch false) {
                return .{
                    .hit = true,
                    .distance = t,
                    .toi = t,
                    .position_x = ray.origin_x + ray.dir_x * t,
                    .position_y = ray.origin_y + ray.dir_y * t,
                    .position_z = ray.origin_z + ray.dir_z * t,
                    .normal_x = normal_x,
                    .normal_y = normal_y,
                    .normal_z = normal_z,
                    .instance_idx = -1,
                    .entity_id = -1,
                    .hit_environment = true,
                    .hit_sensor = false,
                };
            }
        }

        // Check instances at this position
        var i: u8 = 0;
        while (i < world.s1024.instance_count) : (i += 1) {
            if (query_world.shouldIgnoreInstance(world, i, filter)) continue;

            const inst = &world.instances[i];
            const lx = px - inst.pos_x;
            const ly = py - inst.pos_y;
            const lz = pz - inst.pos_z;

            if (lx >= 0 and lx < 16 and ly >= 0 and ly < 16 and lz >= 0 and lz < 16) {
                if (entity16.testVoxel(&world.entities[inst.entity_id], @intCast(lx), @intCast(ly), @intCast(lz))) {
                    const body_type = query_world.getInstanceBodyType(world, i);
                    return .{
                        .hit = true,
                        .distance = t,
                        .toi = t,
                        .position_x = ray.origin_x + ray.dir_x * t,
                        .position_y = ray.origin_y + ray.dir_y * t,
                        .position_z = ray.origin_z + ray.dir_z * t,
                        .normal_x = normal_x,
                        .normal_y = normal_y,
                        .normal_z = normal_z,
                        .instance_idx = @as(i16, @intCast(i)),
                        .entity_id = @as(i16, @intCast(inst.entity_id)),
                        .hit_environment = false,
                        .hit_sensor = (body_type == .sensor),
                    };
                }
            }
        }

        // Advance to next voxel
        if (tMax_x < tMax_y and tMax_x < tMax_z) {
            t = tMax_x;
            tMax_x += tDelta_x;
            px += step_x;
            normal_x = if (step_x > 0) -1.0 else 1.0;
            normal_y = 0.0;
            normal_z = 0.0;
        } else if (tMax_y < tMax_z) {
            t = tMax_y;
            tMax_y += tDelta_y;
            py += step_y;
            normal_x = 0.0;
            normal_y = if (step_y > 0) -1.0 else 1.0;
            normal_z = 0.0;
        } else {
            t = tMax_z;
            tMax_z += tDelta_z;
            pz += step_z;
            normal_x = 0.0;
            normal_y = 0.0;
            normal_z = if (step_z > 0) -1.0 else 1.0;
        }
    }

    return .{};
}

/// Raycast that ignores a specific instance (self)
pub fn raycastIgnoreSelf(world: *const QueryWorldView, ray: QueryRay, ignore_idx: u8) QueryHit {
    return raycastSingle(world, ray, .{
        .ignore_instance_idx = ignore_idx,
        .include_static = true,
        .include_dynamic = true,
        .include_kinematic = true,
        .include_sensors = false,
    });
}

/// Get all hits along a ray
pub fn raycastAll(world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter, out_hits: []QueryHit) u16 {
    var hits: [32]QueryHit = undefined;
    var count: u16 = 0;

    var px: i32 = @intFromFloat(@floor(ray.origin_x));
    var py: i32 = @intFromFloat(@floor(ray.origin_y));
    var pz: i32 = @intFromFloat(@floor(ray.origin_z));

    const step_x: i32 = if (ray.dir_x >= 0) 1 else -1;
    const step_y: i32 = if (ray.dir_y >= 0) 1 else -1;
    const step_z: i32 = if (ray.dir_z >= 0) 1 else -1;

    const tDelta_x: f32 = if (ray.dir_x != 0) @abs(1.0 / ray.dir_x) else 1e10;
    const tDelta_y: f32 = if (ray.dir_y != 0) @abs(1.0 / ray.dir_y) else 1e10;
    const tDelta_z: f32 = if (ray.dir_z != 0) @abs(1.0 / ray.dir_z) else 1e10;

    var tMax_x: f32 = if (ray.dir_x >= 0) (ray.origin_x - @as(f32, @floatFromInt(px))) * tDelta_x else (@as(f32, @floatFromInt(px)) + 1.0 - ray.origin_x) * tDelta_x;
    var tMax_y: f32 = if (ray.dir_y >= 0) (ray.origin_y - @as(f32, @floatFromInt(py))) * tDelta_y else (@as(f32, @floatFromInt(py)) + 1.0 - ray.origin_y) * tDelta_y;
    var tMax_z: f32 = if (ray.dir_z >= 0) (ray.origin_z - @as(f32, @floatFromInt(pz))) * tDelta_z else (@as(f32, @floatFromInt(pz)) + 1.0 - ray.origin_z) * tDelta_z;

    var t: f32 = 0.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    const max_steps: i32 = 2048;
    var steps: i32 = 0;

    while (t <= ray.max_distance and steps < max_steps and count < 32) : (steps += 1) {
        // Check instances at this position
        var i: u8 = 0;
        while (i < world.s1024.instance_count) : (i += 1) {
            if (query_world.shouldIgnoreInstance(world, i, filter)) continue;

            const inst = &world.instances[i];
            const lx = px - inst.pos_x;
            const ly = py - inst.pos_y;
            const lz = pz - inst.pos_z;

            if (lx >= 0 and lx < 16 and ly >= 0 and ly < 16 and lz >= 0 and lz < 16) {
                if (entity16.testVoxel(&world.entities[inst.entity_id], @intCast(lx), @intCast(ly), @intCast(lz))) {
                    const body_type = query_world.getInstanceBodyType(world, i);
                    hits[count] = .{
                        .hit = true,
                        .distance = t,
                        .toi = t,
                        .position_x = ray.origin_x + ray.dir_x * t,
                        .position_y = ray.origin_y + ray.dir_y * t,
                        .position_z = ray.origin_z + ray.dir_z * t,
                        .normal_x = normal_x,
                        .normal_y = normal_y,
                        .normal_z = normal_z,
                        .instance_idx = @as(i16, @intCast(i)),
                        .entity_id = @as(i16, @intCast(inst.entity_id)),
                        .hit_environment = false,
                        .hit_sensor = (body_type == .sensor),
                    };
                    count += 1;
                }
            }
        }

        // Advance to next voxel
        if (tMax_x < tMax_y and tMax_x < tMax_z) {
            t = tMax_x;
            tMax_x += tDelta_x;
            px += step_x;
            normal_x = if (step_x > 0) -1.0 else 1.0;
            normal_y = 0.0;
            normal_z = 0.0;
        } else if (tMax_y < tMax_z) {
            t = tMax_y;
            tMax_y += tDelta_y;
            py += step_y;
            normal_x = 0.0;
            normal_y = if (step_y > 0) -1.0 else 1.0;
            normal_z = 0.0;
        } else {
            t = tMax_z;
            tMax_z += tDelta_z;
            pz += step_z;
            normal_x = 0.0;
            normal_y = 0.0;
            normal_z = if (step_z > 0) -1.0 else 1.0;
        }
    }

    // Copy to output buffer
    var i: u16 = 0;
    while (i < count and i < out_hits.len) : (i += 1) {
        out_hits[i] = hits[i];
    }

    return count;
}
