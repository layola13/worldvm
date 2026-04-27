//! Query Benchmark - deterministic query performance workload.
//!
//! This module measures query-layer work using operation counters instead of
//! wall-clock time, so the benchmark is stable in tests and CI.

const std = @import("std");
const address = @import("address.zig");
const entity16 = @import("entity16.zig");
const query_raycast = @import("query_raycast.zig");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const scene1024 = @import("scene1024.zig");

const QueryHit = query_types.QueryHit;
const QueryRay = query_types.QueryRay;
const QueryStats = query_types.QueryStats;
const QueryVoxelRequest = query_types.QueryVoxelRequest;
const QueryWorldView = query_types.QueryWorldView;

pub const QueryBenchmarkConfig = struct {
    point_iterations: u16 = 64,
    batch_iterations: u16 = 4,
    batch_width: u16 = 16,
    raycast_iterations: u16 = 4,
    sort_iterations: u16 = 4,
    sort_width: u16 = 8,
};

pub const QueryBenchmarkResult = struct {
    config: QueryBenchmarkConfig,
    point_hits: u32 = 0,
    batch_hits: u32 = 0,
    ray_hits: u32 = 0,
    sort_first_instance_sum: i32 = 0,
    stats: QueryStats = .{},
};

fn setBenchmarkVoxel(s1024: *scene1024.Scene1024, x: u5, y: u5, z: u5) !void {
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

fn buildBenchmarkWorld(s1024: *scene1024.Scene1024, entities_out: []entity16.Entity16) !void {
    try setBenchmarkVoxel(s1024, 1, 1, 1);
    try setBenchmarkVoxel(s1024, 3, 1, 1);
    try setBenchmarkVoxel(s1024, 5, 1, 1);
    try setBenchmarkVoxel(s1024, 7, 2, 1);

    if (entities_out.len > 0) {
        var entity = entity16.initEntity16();
        entity.physics.material = .elastic;
        entity.physics.mass = 10;
        entity16.setVoxel(&entity, 0, 0, 0);
        entities_out[0] = entity;
        s1024.instance_count = 1;
        s1024.instances[0] = .{
            .entity_id = 0,
            .pos_x = 9,
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
}

fn benchmarkPointQueries(world: *const QueryWorldView, iterations: u16) u32 {
    var hits: u32 = 0;
    var i: u16 = 0;
    while (i < iterations) : (i += 1) {
        const x: i32 = @intCast(i % 10);
        const y: i32 = if ((i % 7) == 0) 2 else 1;
        const hit = query_world.queryAnyVoxel(world, x, y, 1, .{});
        if (hit.hit) hits += 1;
    }
    return hits;
}

fn benchmarkBatchQueries(world: *const QueryWorldView, iterations: u16, width: u16) u32 {
    var requests: [64]QueryVoxelRequest = undefined;
    var hits_buf: [64]QueryHit = undefined;
    const clamped_width: u16 = @min(width, @as(u16, @intCast(requests.len)));

    var hits: u32 = 0;
    var iter: u16 = 0;
    while (iter < iterations) : (iter += 1) {
        var i: u16 = 0;
        while (i < clamped_width) : (i += 1) {
            requests[i] = .{
                .gx = @intCast((i + iter) % 10),
                .gy = if (((i + iter) % 7) == 0) 2 else 1,
                .gz = 1,
            };
        }

        const written = query_world.queryAnyVoxelBatch(world, requests[0..clamped_width], .{}, hits_buf[0..clamped_width]);
        i = 0;
        while (i < written) : (i += 1) {
            if (hits_buf[i].hit) hits += 1;
        }
    }
    return hits;
}

fn benchmarkRaycasts(world: *const QueryWorldView, iterations: u16) u32 {
    var out_hits: [32]QueryHit = undefined;
    var hits: u32 = 0;
    var i: u16 = 0;
    while (i < iterations) : (i += 1) {
        const ray = QueryRay{
            .origin_x = 0.25,
            .origin_y = if ((i % 2) == 0) 1.0 else 1.25,
            .origin_z = 1.0,
            .dir_x = 1.0,
            .dir_y = 0.0,
            .dir_z = 0.0,
            .max_distance = 12.0,
        };
        hits += query_raycast.raycastAll(world, ray, .{}, out_hits[0..]);
    }
    return hits;
}

fn benchmarkSorts(iterations: u16, width: u16) i32 {
    var hits: [32]QueryHit = undefined;
    const clamped_width: u16 = @min(width, @as(u16, @intCast(hits.len)));

    var sum: i32 = 0;
    var iter: u16 = 0;
    while (iter < iterations) : (iter += 1) {
        var i: u16 = 0;
        while (i < clamped_width) : (i += 1) {
            hits[i] = .{
                .hit = true,
                .distance = @floatFromInt(clamped_width - i),
                .toi = @floatFromInt(clamped_width - i),
                .instance_idx = @intCast(i),
            };
        }
        query_types.sortQueryHitsByDistanceStable(hits[0..clamped_width]);
        if (clamped_width > 0) sum += hits[0].instance_idx;
    }
    return sum;
}

pub fn runQueryBenchmark(allocator: std.mem.Allocator, config: QueryBenchmarkConfig) !QueryBenchmarkResult {
    var entities = [_]entity16.Entity16{entity16.initEntity16()};
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();
    try buildBenchmarkWorld(&s1024, entities[0..]);
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    query_types.resetQueryStats();
    var result = QueryBenchmarkResult{ .config = config };
    result.point_hits = benchmarkPointQueries(&world, config.point_iterations);
    result.batch_hits = benchmarkBatchQueries(&world, config.batch_iterations, config.batch_width);
    result.ray_hits = benchmarkRaycasts(&world, config.raycast_iterations);
    result.sort_first_instance_sum = benchmarkSorts(config.sort_iterations, config.sort_width);
    result.stats = query_types.getQueryStats();
    return result;
}

test "runQueryBenchmark records deterministic query workload counters" {
    const result = try runQueryBenchmark(std.testing.allocator, .{});

    try std.testing.expect(result.point_hits > 0);
    try std.testing.expect(result.batch_hits > 0);
    try std.testing.expect(result.ray_hits > 0);
    try std.testing.expect(result.stats.point_queries > result.config.point_iterations);
    try std.testing.expectEqual(@as(u64, result.config.batch_iterations), result.stats.batch_queries);
    try std.testing.expectEqual(@as(u64, result.config.raycast_iterations), result.stats.raycast_all_queries);
    try std.testing.expectEqual(@as(u64, result.config.raycast_iterations + result.config.sort_iterations), result.stats.sort_calls);
    try std.testing.expect(result.stats.sorted_hits >= result.config.sort_iterations * result.config.sort_width);
}
