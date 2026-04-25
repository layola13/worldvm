//! Query Overlap - L1 Overlap Queries
//!
//! Task A: Unified Query Layer
//! AABB, sphere, capsule overlap queries

const std = @import("std");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const query_penetration = @import("query_penetration.zig");
const scene32 = @import("scene32.zig");

const OverlapResult = query_types.OverlapResult;
const QueryFilter = query_types.QueryFilter;
const QueryWorldView = query_types.QueryWorldView;

const OverlapAccumulator = struct {
    result: OverlapResult = .{},
    seen_environment: bool = false,
    seen_instances: [scene32.MAX_INSTANCES]bool = [_]bool{false} ** scene32.MAX_INSTANCES,
};

fn appendOverlapHit(acc: *OverlapAccumulator, hit: query_types.QueryHit) void {
    if (hit.hit_environment) {
        acc.result.hit = true;
        acc.result.environment_overlap = true;
        if (!acc.seen_environment) {
            acc.seen_environment = true;
            acc.result.count += 1;
        }
    }
    if (hit.instance_idx >= 0) {
        acc.result.hit = true;
        const idx: usize = @intCast(hit.instance_idx);
        if (idx < acc.seen_instances.len and !acc.seen_instances[idx]) {
            acc.seen_instances[idx] = true;
            acc.result.count += 1;
            if (acc.result.first_instance_idx == -1) {
                acc.result.first_instance_idx = hit.instance_idx;
            }
        }
    }
}

/// Check AABB overlap with world
pub fn overlapAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) OverlapResult {
    var acc: OverlapAccumulator = .{};
    const range = query_penetration.computeAABBVoxelRange(min_x, min_y, min_z, max_x, max_y, max_z);

    var x = range.start_x;
    while (x <= range.end_x) : (x += 1) {
        var y = range.start_y;
        while (y <= range.end_y) : (y += 1) {
            var z = range.start_z;
            while (z <= range.end_z) : (z += 1) {
                const hit = query_world.queryAnyVoxel(world, x, y, z, filter);
                appendOverlapHit(&acc, hit);
            }
        }
    }

    return acc.result;
}

/// Check sphere overlap with world
pub fn overlapSphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, filter: QueryFilter) OverlapResult {
    var acc: OverlapAccumulator = .{};
    const radius_sq = radius * radius;
    const range = query_penetration.computeSphereVoxelRange(center_x, center_y, center_z, radius);

    var gx = range.start_x;
    while (gx <= range.end_x) : (gx += 1) {
        var gy = range.start_y;
        while (gy <= range.end_y) : (gy += 1) {
            var gz = range.start_z;
            while (gz <= range.end_z) : (gz += 1) {
                const box_min_x = @as(f32, @floatFromInt(gx));
                const box_min_y = @as(f32, @floatFromInt(gy));
                const box_min_z = @as(f32, @floatFromInt(gz));
                const box_max_x = box_min_x + 1.0;
                const box_max_y = box_min_y + 1.0;
                const box_max_z = box_min_z + 1.0;

                const closest = query_penetration.computeSphereBoxClosestPoint(center_x, center_y, center_z, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                if (closest.dist_sq > radius_sq) continue;

                const hit = query_world.queryAnyVoxel(world, gx, gy, gz, filter);
                appendOverlapHit(&acc, hit);
            }
        }
    }

    return acc.result;
}

/// Check capsule overlap with world
pub fn overlapCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) OverlapResult {
    var acc: OverlapAccumulator = .{};
    const seg_min_y = center_y - half_height;
    const seg_max_y = center_y + half_height;
    const range = query_penetration.computeCapsuleVoxelRange(center_x, center_y, center_z, radius, half_height);
    const radius_sq = radius * radius;

    var gx = range.start_x;
    while (gx <= range.end_x) : (gx += 1) {
        var gy = range.start_y;
        while (gy <= range.end_y) : (gy += 1) {
            var gz = range.start_z;
            while (gz <= range.end_z) : (gz += 1) {
                const box_min_x = @as(f32, @floatFromInt(gx));
                const box_min_y = @as(f32, @floatFromInt(gy));
                const box_min_z = @as(f32, @floatFromInt(gz));
                const box_max_x = box_min_x + 1.0;
                const box_max_y = box_min_y + 1.0;
                const box_max_z = box_min_z + 1.0;

                const closest = query_penetration.computeCapsuleBoxClosestPoint(center_x, center_y, center_z, seg_min_y, seg_max_y, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                if (closest.dist_sq > radius_sq) continue;

                const hit = query_world.queryAnyVoxel(world, gx, gy, gz, filter);
                appendOverlapHit(&acc, hit);
            }
        }
    }

    return acc.result;
}

test "overlapCapsule detects environment voxel intersecting top cap" {
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

    const result = overlapCapsule(&world, 0.5, 2.7, 0.5, 0.8, 0.5, .{});
    try std.testing.expect(result.hit);
    try std.testing.expect(result.environment_overlap);
    try std.testing.expect(result.count > 0);
}

test "overlapCapsule detects side overlap along segment away from capsule center" {
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
        .lx = 1,
        .ly = 3,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const result = overlapCapsule(&world, 0.5, 2.0, 0.5, 0.8, 2.0, .{});
    try std.testing.expect(result.hit);
    try std.testing.expect(result.environment_overlap);
    try std.testing.expect(result.count > 0);
}

test "overlapSphere detects voxel intersecting sphere edge even when voxel center is outside sphere" {
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

    const result = overlapSphere(&world, 0.9, 0.9, 0.5, 0.2, .{});
    try std.testing.expect(result.hit);
    try std.testing.expect(result.environment_overlap);
    try std.testing.expect(result.count > 0);
}

test "overlapAABB counts environment overlap once across multiple voxels" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr_a = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 1,
        .lz = 1,
    });
    const addr_b = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 2,
        .ly = 1,
        .lz = 1,
    });
    try s1024.setVoxelAtGlobal(addr_a, true);
    try s1024.setVoxelAtGlobal(addr_b, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const result = overlapAABB(&world, 1.1, 1.1, 1.1, 2.9, 1.9, 1.9, .{});
    try std.testing.expect(result.hit);
    try std.testing.expect(result.environment_overlap);
    try std.testing.expectEqual(@as(u16, 1), result.count);
}

test "overlapAABB does not count voxel that only touches exact max face" {
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
        .lx = 2,
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

    const result = overlapAABB(&world, 1.1, 1.1, 1.1, 2.0, 1.9, 1.9, .{});
    try std.testing.expect(!result.hit);
    try std.testing.expect(!result.environment_overlap);
    try std.testing.expectEqual(@as(u16, 0), result.count);
}
