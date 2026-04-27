//! Query Sweep - L3 Sweep Queries
//!
//! Task A: Unified Query Layer
//! Sphere, capsule, and box sweep queries

const std = @import("std");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const query_overlap = @import("query_overlap.zig");
const query_raycast = @import("query_raycast.zig");

const QueryFilter = query_types.QueryFilter;
const QueryHit = query_types.QueryHit;
const QueryWorldView = query_types.QueryWorldView;

const SampleOffset = struct {
    x: f32,
    y: f32,
    z: f32,
};

fn appendSampleOffset(out: []SampleOffset, count: *usize, x: f32, y: f32, z: f32) void {
    if (count.* >= out.len) return;
    out[count.*] = .{ .x = x, .y = y, .z = z };
    count.* += 1;
}

fn buildSphereSampleOffsets(out: []SampleOffset, radius: f32) []const SampleOffset {
    var count: usize = 0;
    appendSampleOffset(out, &count, 0, 0, 0);
    appendSampleOffset(out, &count, radius, 0, 0);
    appendSampleOffset(out, &count, -radius, 0, 0);
    appendSampleOffset(out, &count, 0, radius, 0);
    appendSampleOffset(out, &count, 0, -radius, 0);
    appendSampleOffset(out, &count, 0, 0, radius);
    appendSampleOffset(out, &count, 0, 0, -radius);

    const diag2 = radius / @sqrt(2.0);
    appendSampleOffset(out, &count, diag2, diag2, 0);
    appendSampleOffset(out, &count, diag2, -diag2, 0);
    appendSampleOffset(out, &count, -diag2, diag2, 0);
    appendSampleOffset(out, &count, -diag2, -diag2, 0);
    appendSampleOffset(out, &count, diag2, 0, diag2);
    appendSampleOffset(out, &count, diag2, 0, -diag2);
    appendSampleOffset(out, &count, -diag2, 0, diag2);
    appendSampleOffset(out, &count, -diag2, 0, -diag2);
    appendSampleOffset(out, &count, 0, diag2, diag2);
    appendSampleOffset(out, &count, 0, diag2, -diag2);
    appendSampleOffset(out, &count, 0, -diag2, diag2);
    appendSampleOffset(out, &count, 0, -diag2, -diag2);

    const diag3 = radius / @sqrt(3.0);
    appendSampleOffset(out, &count, diag3, diag3, diag3);
    appendSampleOffset(out, &count, diag3, diag3, -diag3);
    appendSampleOffset(out, &count, diag3, -diag3, diag3);
    appendSampleOffset(out, &count, diag3, -diag3, -diag3);
    appendSampleOffset(out, &count, -diag3, diag3, diag3);
    appendSampleOffset(out, &count, -diag3, diag3, -diag3);
    appendSampleOffset(out, &count, -diag3, -diag3, diag3);
    appendSampleOffset(out, &count, -diag3, -diag3, -diag3);

    return out[0..count];
}

fn buildBoxSampleOffsets(out: []SampleOffset, half_x: f32, half_y: f32, half_z: f32) []const SampleOffset {
    var count: usize = 0;
    appendSampleOffset(out, &count, 0, 0, 0);

    const signs = [_]f32{ -1.0, 1.0 };
    for (signs) |sx| {
        for (signs) |sy| {
            for (signs) |sz| {
                appendSampleOffset(out, &count, sx * half_x, sy * half_y, sz * half_z);
            }
        }
    }

    for (signs) |sx| appendSampleOffset(out, &count, sx * half_x, 0, 0);
    for (signs) |sy| appendSampleOffset(out, &count, 0, sy * half_y, 0);
    for (signs) |sz| appendSampleOffset(out, &count, 0, 0, sz * half_z);

    for (signs) |sx| {
        for (signs) |sy| appendSampleOffset(out, &count, sx * half_x, sy * half_y, 0);
        for (signs) |sz| appendSampleOffset(out, &count, sx * half_x, 0, sz * half_z);
    }
    for (signs) |sy| {
        for (signs) |sz| appendSampleOffset(out, &count, 0, sy * half_y, sz * half_z);
    }

    return out[0..count];
}

fn buildCapsuleSampleOffsets(out: []SampleOffset, radius: f32, half_height: f32) []const SampleOffset {
    var count: usize = 0;
    var sphere_offsets_buf: [27]SampleOffset = undefined;
    const sphere_offsets = buildSphereSampleOffsets(sphere_offsets_buf[0..], radius);

    for (sphere_offsets) |offset| {
        appendSampleOffset(out, &count, offset.x, half_height + offset.y, offset.z);
        appendSampleOffset(out, &count, offset.x, -half_height + offset.y, offset.z);
    }

    const segment_height = half_height * 2.0;
    const axial_step = @max(radius, 0.5);
    const segment_steps: usize = @intFromFloat(@max(1.0, @ceil(segment_height / axial_step)));

    var i: usize = 0;
    while (i <= segment_steps) : (i += 1) {
        const t = if (segment_steps == 0) 0.5 else @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(segment_steps));
        const y = -half_height + segment_height * t;
        appendSampleOffset(out, &count, 0, y, 0);
        for (sphere_offsets[1..]) |offset| {
            if (@abs(offset.y) > 0.0001) continue;
            appendSampleOffset(out, &count, offset.x, y, offset.z);
        }
    }

    return out[0..count];
}

fn updateClosestSweepHit(closest_hit: *QueryHit, hit: QueryHit, offset: SampleOffset) void {
    if (!hit.hit) return;
    if (closest_hit.hit and hit.distance >= closest_hit.distance) return;

    var adjusted = hit;
    query_types.setQueryHitPosition(&adjusted, hit.position_x - offset.x, hit.position_y - offset.y, hit.position_z - offset.z);
    closest_hit.* = adjusted;
}

fn hitFromOverlapResult(overlap: query_types.OverlapResult, base_x: f32, base_y: f32, base_z: f32, world: *const QueryWorldView) QueryHit {
    if (!overlap.hit) return .{};

    var hit: QueryHit = if (overlap.first_hit.hit) overlap.first_hit else .{ .hit = true };
    hit.distance = 0.0;
    hit.toi = 0.0;
    hit.instance_idx = overlap.first_instance_idx;
    hit.hit_environment = overlap.environment_overlap;
    query_types.setQueryHitPosition(&hit, base_x, base_y, base_z);

    if (overlap.first_instance_idx >= 0) {
        const idx: usize = @intCast(overlap.first_instance_idx);
        if (idx < world.instances.len) {
            hit.entity_id = @intCast(world.instances[idx].entity_id);
        }
    }
    query_types.normalizeQueryHitNormal(&hit);
    return hit;
}

fn overlapFromSamples(world: *const QueryWorldView, base_x: f32, base_y: f32, base_z: f32, offsets: []const SampleOffset, filter: QueryFilter) QueryHit {
    var cache = query_world.QueryAnyVoxelCache.init(filter);
    for (offsets) |offset| {
        const hit = query_world.queryAnyVoxelCached(
            world,
            &cache,
            @intFromFloat(@floor(base_x + offset.x)),
            @intFromFloat(@floor(base_y + offset.y)),
            @intFromFloat(@floor(base_z + offset.z)),
        );
        if (hit.hit) {
            var adjusted = hit;
            adjusted.distance = 0;
            adjusted.toi = 0;
            query_types.setQueryHitPosition(&adjusted, base_x, base_y, base_z);
            query_types.normalizeQueryHitNormal(&adjusted);
            return adjusted;
        }
    }

    return .{};
}

fn sweepFromSamples(world: *const QueryWorldView, origin_x: f32, origin_y: f32, origin_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, offsets: []const SampleOffset, filter: QueryFilter) QueryHit {
    var closest_hit: QueryHit = .{};

    for (offsets) |offset| {
        const hit = query_raycast.raycastSingle(world, .{
            .origin_x = origin_x + offset.x,
            .origin_y = origin_y + offset.y,
            .origin_z = origin_z + offset.z,
            .dir_x = dir_x,
            .dir_y = dir_y,
            .dir_z = dir_z,
            .max_distance = max_distance,
        }, filter);
        updateClosestSweepHit(&closest_hit, hit, offset);
    }

    return closest_hit;
}

/// Sphere sweep against world
pub fn sphereCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    var offsets_buf: [27]SampleOffset = undefined;
    const offsets = buildSphereSampleOffsets(offsets_buf[0..], radius);

    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        return hitFromOverlapResult(query_overlap.overlapSphere(world, center_x, center_y, center_z, radius, filter), center_x, center_y, center_z, world);
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;
    return sweepFromSamples(world, center_x, center_y, center_z, ndx, ndy, ndz, max_distance, offsets, filter);
}

/// Capsule sweep - capsule is center + radius + half_height (total height = 2*half_height + 2*radius)
pub fn capsuleCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    var offsets_buf: [96]SampleOffset = undefined;
    const offsets = buildCapsuleSampleOffsets(offsets_buf[0..], radius, half_height);
    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        return hitFromOverlapResult(query_overlap.overlapCapsule(world, center_x, center_y, center_z, radius, half_height, filter), center_x, center_y, center_z, world);
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;
    return sweepFromSamples(world, center_x, center_y, center_z, ndx, ndy, ndz, max_distance, offsets, filter);
}

pub fn boxCast(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    const cx = (min_x + max_x) * 0.5;
    const cy = (min_y + max_y) * 0.5;
    const cz = (min_z + max_z) * 0.5;
    const half_x = (max_x - min_x) * 0.5;
    const half_y = (max_y - min_y) * 0.5;
    const half_z = (max_z - min_z) * 0.5;

    var offsets_buf: [27]SampleOffset = undefined;
    const offsets = buildBoxSampleOffsets(offsets_buf[0..], half_x, half_y, half_z);

    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        return hitFromOverlapResult(query_overlap.overlapAABB(world, min_x, min_y, min_z, max_x, max_y, max_z, filter), cx, cy, cz, world);
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;
    return sweepFromSamples(world, cx, cy, cz, ndx, ndy, ndz, max_distance, offsets, filter);
}

test "sphereCast hits offset obstacle within radius even when center ray would miss" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 3,
        .ly = 1,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = sphereCast(&world, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 5.0, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(query_types.queryHitIsConsistent(hit));
    try std.testing.expect(hit.hit_environment);
    try std.testing.expect(hit.distance <= 3.0);
}

test "sphereCast hits diagonal obstacle within radius even when axis samples would miss" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 3,
        .ly = 1,
        .lz = 1,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = sphereCast(&world, 0.0, 0.0, 0.0, 1.5, 1.0, 0.0, 0.0, 5.0, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(hit.hit_environment);
    try std.testing.expect(hit.distance <= 3.0);
}

test "boxCast hits corner obstacle outside center line" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 3,
        .ly = 2,
        .lz = 2,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = boxCast(&world, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 1.0, 0.0, 0.0, 5.0, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(hit.hit_environment);
    try std.testing.expect(hit.distance <= 3.0);
}

test "capsuleCast detects obstacle intersecting top cap" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 4,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = capsuleCast(&world, 0.5, 2.7, 0.5, 0.8, 0.5, 0.0, 0.0, 0.0, 0.0, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(hit.hit_environment);
}

test "sphereCast zero direction reuses real overlap semantics for edge contact" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");
    const terrain = @import("terrain.zig");

    terrain.init();
    terrain.addTerrainPatch(0, 0, 8, .mud);
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 1,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = sphereCast(&world, 0.9, 0.9, 0.5, 0.2, 0.0, 0.0, 0.0, 0.0, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(query_types.queryHitIsConsistent(hit));
    try std.testing.expect(hit.hit_environment);
    try std.testing.expect(hit.distance == 0.0);
    try std.testing.expect(hit.classification.surface_type == .mud);
    try std.testing.expect(hit.material_type == hit.classification.material_type);
    try std.testing.expect(hit.surface_condition == hit.classification.surface_condition);
    try std.testing.expect(hit.medium_type == hit.classification.medium_type);
    try std.testing.expect(hit.body_type == hit.classification.body_type);
}
