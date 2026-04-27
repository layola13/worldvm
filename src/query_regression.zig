//! Query Regression - fixed query protocol coverage suite.

const std = @import("std");
const address = @import("address.zig");
const entity16 = @import("entity16.zig");
const query_benchmark = @import("query_benchmark.zig");
const query_debug = @import("query_debug.zig");
const query_overlap = @import("query_overlap.zig");
const query_penetration = @import("query_penetration.zig");
const query_raycast = @import("query_raycast.zig");
const query_sweep = @import("query_sweep.zig");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const scene1024 = @import("scene1024.zig");
const terrain = @import("terrain.zig");

pub const QueryRegressionResult = struct {
    point_hit: query_types.QueryHit = .{},
    ray_hit_count: u16 = 0,
    sweep_hit: query_types.QueryHit = .{},
    overlap_hit: query_types.QueryHit = .{},
    penetration: query_types.PenetrationResult = .{},
    debug_primitive_count: u16 = 0,
    benchmark: query_benchmark.QueryBenchmarkResult = .{ .config = .{} },
};

fn setVoxel(s1024: *scene1024.Scene1024, x: u5, y: u5, z: u5) !void {
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = x,
        .ly = y,
        .lz = z,
    }), true);
}

fn buildRegressionWorld(allocator: std.mem.Allocator, entities_out: []entity16.Entity16) !struct {
    s1024: scene1024.Scene1024,
} {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 8, .ice);

    var s1024 = scene1024.Scene1024.init(allocator);
    errdefer s1024.deinit();
    try setVoxel(&s1024, 1, 1, 1);
    try setVoxel(&s1024, 3, 1, 1);
    try setVoxel(&s1024, 5, 1, 1);
    try setVoxel(&s1024, 3, 2, 1);

    if (entities_out.len > 0) {
        var entity = entity16.initEntity16();
        entity.physics.material = .elastic;
        entity.physics.mass = 20;
        entity16.setVoxel(&entity, 0, 0, 0);
        entities_out[0] = entity;
        s1024.instance_count = 1;
        s1024.instances[0] = .{
            .entity_id = 0,
            .pos_x = 7,
            .pos_y = 1,
            .pos_z = 1,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .idle,
            .sleep_tick = 0,
            ._reserved = .{0} ** 2,
        };
    }

    return .{ .s1024 = s1024 };
}

pub fn runQueryRegressionSuite(allocator: std.mem.Allocator) !QueryRegressionResult {
    var entities = [_]entity16.Entity16{entity16.initEntity16()};
    var fixture = try buildRegressionWorld(allocator, entities[0..]);
    defer fixture.s1024.deinit();

    const world = query_types.QueryWorldView{
        .s1024 = &fixture.s1024,
        .instances = fixture.s1024.instances[0..fixture.s1024.instance_count],
        .entities = entities[0..],
    };

    var ray_hits: [8]query_types.QueryHit = undefined;
    var debug_storage: [8]query_debug.QueryDebugPrimitive = undefined;
    var debug_buffer = query_debug.QueryDebugBuffer.init(debug_storage[0..]);

    const point_hit = query_world.queryAnyVoxel(&world, 1, 1, 1, .{});
    const ray_hit_count = query_raycast.raycastAll(&world, .{
        .origin_x = 0.25,
        .origin_y = 1.0,
        .origin_z = 1.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 9.0,
    }, .{}, ray_hits[0..]);
    const sweep_hit = query_sweep.sphereCast(&world, 0.0, 2.0, 1.0, 0.75, 1.0, 0.0, 0.0, 5.0, .{});
    const overlap_hit = query_overlap.overlapAABBSingle(&world, 2.9, 0.9, 0.9, 3.25, 1.25, 1.25, .{});
    const penetration = query_penetration.computePenetrationBox(&world, 2.8, 0.8, 0.8, 3.2, 1.2, 1.2, .{});
    _ = query_debug.appendRaycastDebug(&debug_buffer, &world, .{
        .origin_x = 0.25,
        .origin_y = 1.0,
        .origin_z = 1.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 9.0,
    }, .{});

    return .{
        .point_hit = point_hit,
        .ray_hit_count = ray_hit_count,
        .sweep_hit = sweep_hit,
        .overlap_hit = overlap_hit,
        .penetration = penetration,
        .debug_primitive_count = debug_buffer.count,
        .benchmark = try query_benchmark.runQueryBenchmark(allocator, .{
            .point_iterations = 16,
            .batch_iterations = 2,
            .batch_width = 8,
            .raycast_iterations = 2,
            .sort_iterations = 2,
            .sort_width = 4,
        }),
    };
}

test "runQueryRegressionSuite covers fixed query protocol behavior" {
    const result = try runQueryRegressionSuite(std.testing.allocator);

    try std.testing.expect(result.point_hit.hit);
    try std.testing.expect(query_types.queryHitIsConsistent(result.point_hit));
    try std.testing.expect(result.point_hit.classification.surface_type == .ice);
    try std.testing.expect(result.ray_hit_count >= 3);
    try std.testing.expect(result.sweep_hit.hit);
    try std.testing.expect(query_types.queryHitIsConsistent(result.sweep_hit));
    try std.testing.expect(result.overlap_hit.hit);
    try std.testing.expect(query_types.queryHitIsConsistent(result.overlap_hit));
    try std.testing.expect(result.penetration.overlapping);
    try std.testing.expect(result.penetration.depth > 0.0);
    try std.testing.expect(result.debug_primitive_count == 3);
    try std.testing.expect(result.benchmark.stats.point_queries > 0);
    try std.testing.expect(result.benchmark.stats.raycast_all_queries > 0);
}
