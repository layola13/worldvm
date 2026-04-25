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

const QueryFilter = query_types.QueryFilter;
const QueryHit = query_types.QueryHit;
const QueryRay = query_types.QueryRay;
const QueryWorldView = query_types.QueryWorldView;

/// DDA Voxel Raycast with query filter support
/// Returns first voxel hit along ray
pub fn raycastSingle(world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter) QueryHit {
    var px: i32 = @intFromFloat(@floor(ray.origin_x));
    var py: i32 = @intFromFloat(@floor(ray.origin_y));
    var pz: i32 = @intFromFloat(@floor(ray.origin_z));

    const step_x: i32 = if (ray.dir_x >= 0) 1 else -1;
    const step_y: i32 = if (ray.dir_y >= 0) 1 else -1;
    const step_z: i32 = if (ray.dir_z >= 0) 1 else -1;

    const tDelta_x: f32 = if (ray.dir_x != 0) @abs(1.0 / ray.dir_x) else std.math.inf(f32);
    const tDelta_y: f32 = if (ray.dir_y != 0) @abs(1.0 / ray.dir_y) else std.math.inf(f32);
    const tDelta_z: f32 = if (ray.dir_z != 0) @abs(1.0 / ray.dir_z) else std.math.inf(f32);

    var tMax_x: f32 = if (ray.dir_x > 0)
        (@as(f32, @floatFromInt(px)) + 1.0 - ray.origin_x) * tDelta_x
    else if (ray.dir_x < 0)
        (ray.origin_x - @as(f32, @floatFromInt(px))) * tDelta_x
    else
        std.math.inf(f32);
    var tMax_y: f32 = if (ray.dir_y > 0)
        (@as(f32, @floatFromInt(py)) + 1.0 - ray.origin_y) * tDelta_y
    else if (ray.dir_y < 0)
        (ray.origin_y - @as(f32, @floatFromInt(py))) * tDelta_y
    else
        std.math.inf(f32);
    var tMax_z: f32 = if (ray.dir_z > 0)
        (@as(f32, @floatFromInt(pz)) + 1.0 - ray.origin_z) * tDelta_z
    else if (ray.dir_z < 0)
        (ray.origin_z - @as(f32, @floatFromInt(pz))) * tDelta_z
    else
        std.math.inf(f32);

    var t: f32 = 0.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    const max_steps: i32 = 2048;
    var steps: i32 = 0;

    while (t <= ray.max_distance and steps < max_steps) : (steps += 1) {
        const base_hit = query_world.queryAnyVoxel(world, px, py, pz, filter);
        if (base_hit.hit) {
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
                .instance_idx = base_hit.instance_idx,
                .entity_id = base_hit.entity_id,
                .hit_environment = base_hit.hit_environment,
                .hit_sensor = base_hit.hit_sensor,
                .classification = base_hit.classification,
                .telemetry = base_hit.telemetry,
            };
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

    const tDelta_x: f32 = if (ray.dir_x != 0) @abs(1.0 / ray.dir_x) else std.math.inf(f32);
    const tDelta_y: f32 = if (ray.dir_y != 0) @abs(1.0 / ray.dir_y) else std.math.inf(f32);
    const tDelta_z: f32 = if (ray.dir_z != 0) @abs(1.0 / ray.dir_z) else std.math.inf(f32);

    var tMax_x: f32 = if (ray.dir_x > 0)
        (@as(f32, @floatFromInt(px)) + 1.0 - ray.origin_x) * tDelta_x
    else if (ray.dir_x < 0)
        (ray.origin_x - @as(f32, @floatFromInt(px))) * tDelta_x
    else
        std.math.inf(f32);
    var tMax_y: f32 = if (ray.dir_y > 0)
        (@as(f32, @floatFromInt(py)) + 1.0 - ray.origin_y) * tDelta_y
    else if (ray.dir_y < 0)
        (ray.origin_y - @as(f32, @floatFromInt(py))) * tDelta_y
    else
        std.math.inf(f32);
    var tMax_z: f32 = if (ray.dir_z > 0)
        (@as(f32, @floatFromInt(pz)) + 1.0 - ray.origin_z) * tDelta_z
    else if (ray.dir_z < 0)
        (ray.origin_z - @as(f32, @floatFromInt(pz))) * tDelta_z
    else
        std.math.inf(f32);

    var t: f32 = 0.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    const max_steps: i32 = 2048;
    var steps: i32 = 0;

    while (t <= ray.max_distance and steps < max_steps and count < 32) : (steps += 1) {
        const hit = query_world.queryAnyVoxel(world, px, py, pz, filter);
        if (hit.hit) {
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
                .instance_idx = hit.instance_idx,
                .entity_id = hit.entity_id,
                .hit_environment = hit.hit_environment,
                .hit_sensor = hit.hit_sensor,
                .classification = hit.classification,
                .telemetry = hit.telemetry,
            };
            count += 1;
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

test "raycastSingle preserves query classification metadata" {
    const testing = std.testing;
    const terrain = @import("terrain.zig");

    terrain.init();
    terrain.addTerrainPatch(0, 0, 8, .ice);

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 0,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = raycastSingle(&world, .{
        .origin_x = 0.25,
        .origin_y = 0.0,
        .origin_z = 0.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 4.0,
    }, .{});

    try testing.expect(hit.hit);
    try testing.expect(hit.classification.surface_type == .ice);
    try testing.expect(hit.classification.surface_condition == .slippery);
    try testing.expect(hit.telemetry.friction < 0.2);
}

test "raycastSingle respects layer mask and can ignore environment" {
    const testing = std.testing;

    var s1024 = scene1024.Scene1024.init(testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 0,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entity = entity16.initEntity16();
    entity.physics.mass = 10;
    entity.physics.material = .solid;
    entity16.setVoxel(&entity, 0, 0, 0);

    var entities = [_]entity16.Entity16{entity};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 1,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        ._reserved = .{0} ** 2,
    };

    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const env_hit = raycastSingle(&world, .{
        .origin_x = 0.1,
        .origin_y = 0.0,
        .origin_z = 0.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 4.0,
    }, .{
        .layer_mask = query_types.QUERY_LAYER_ENVIRONMENT,
        .include_dynamic = false,
    });
    try testing.expect(env_hit.hit_environment);

    const dyn_hit = raycastSingle(&world, .{
        .origin_x = 0.1,
        .origin_y = 0.0,
        .origin_z = 0.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 4.0,
    }, .{
        .layer_mask = query_types.QUERY_LAYER_DYNAMIC,
        .ignore_environment = true,
    });
    try testing.expect(dyn_hit.hit);
    try testing.expect(!dyn_hit.hit_environment);
    try testing.expect(dyn_hit.instance_idx == 0);
}
