//! bench.zig - Performance Benchmarking Module
//! Per doc 16: establishes reproducible performance evaluation for WVM.

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const tick_engine = @import("tick_engine.zig");
const entity16 = @import("entity16.zig");
const scenarios = @import("scenarios.zig");

pub const BenchmarkResult = struct {
    scenario_name: []const u8,
    run_count: u32,
    tick_avg_us: f64,
    tick_p95_us: u64,
    tick_p99_us: u64,
    tick_min_us: u64,
    tick_max_us: u64,
    total_us: u64,
    stable_count: u32,
    stable_rate: f64,
};

pub const BenchmarkConfig = struct {
    run_count: u32 = 5,
    max_ticks: u32 = 100,
};

fn measureTickRun(allocator: std.mem.Allocator, scenario: scenarios.Scenario, max_ticks: u32) !struct { tick_us: u64, stable: bool } {
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [64]entity16.Entity16 = undefined;
    scenarios.setupScenario(scenario, &s1024, &entities);

    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &s1024, &entities);

    const start = std.time.nanoTimestamp();
    _ = tick_engine.runTicks(&engine, max_ticks);
    const elapsed = @as(u64, @intCast(std.time.nanoTimestamp() - start)) / 1000;

    return .{ .tick_us = elapsed / @max(engine.tick_id, 1), .stable = engine.stable };
}

pub fn runBenchmark(allocator: std.mem.Allocator, scenario_name: []const u8, cfg: BenchmarkConfig) !BenchmarkResult {
    const scenario: scenarios.Scenario = if (std.mem.eql(u8, scenario_name, "hammer_glass")) .hammer_glass
        else if (std.mem.eql(u8, scenario_name, "water_flow")) .water_flow
        else .apple_table;

    var tick_times = try allocator.alloc(u64, cfg.run_count);
    defer allocator.free(tick_times);

    var stable_count: u32 = 0;
    var total_us: u64 = 0;

    for (0..cfg.run_count) |i| {
        const result = try measureTickRun(allocator, scenario, cfg.max_ticks);
        tick_times[i] = result.tick_us;
        total_us += result.tick_us;
        if (result.stable) stable_count += 1;
    }

    std.mem.sort(u64, tick_times, {}, std.sort.asc(u64));

    const p95_idx = (@as(u64, cfg.run_count) * 95) / 100;
    const p99_idx = (@as(u64, cfg.run_count) * 99) / 100;

    return .{
        .scenario_name = scenario_name,
        .run_count = cfg.run_count,
        .tick_avg_us = @as(f64, @floatFromInt(total_us)) / @as(f64, @floatFromInt(cfg.run_count)),
        .tick_p95_us = tick_times[@min(@as(usize, p95_idx), cfg.run_count - 1)],
        .tick_p99_us = tick_times[@min(@as(usize, p99_idx), cfg.run_count - 1)],
        .tick_min_us = tick_times[0],
        .tick_max_us = tick_times[@as(usize, cfg.run_count) - 1],
        .total_us = total_us,
        .stable_count = stable_count,
        .stable_rate = @as(f64, @floatFromInt(stable_count)) / @as(f64, @floatFromInt(cfg.run_count)),
    };
}

pub fn printResult(writer: anytype, result: BenchmarkResult) !void {
    try writer.print("Benchmark: {s}\n", .{result.scenario_name});
    try writer.print("  Runs: {d}\n", .{result.run_count});
    try writer.print("  Tick avg: {d:.1f}us\n", .{result.tick_avg_us});
    try writer.print("  Tick p95: {d}us\n", .{result.tick_p95_us});
    try writer.print("  Tick p99: {d}us\n", .{result.tick_p99_us});
    try writer.print("  Tick min: {d}us\n", .{result.tick_min_us});
    try writer.print("  Tick max: {d}us\n", .{result.tick_max_us});
    try writer.print("  Stable: {d}/{d} ({d:.0f}%)\n", .{
        result.stable_count, result.run_count, result.stable_rate * 100
    });
}

test "BenchmarkResult fields" {
    const result = BenchmarkResult{
        .scenario_name = "test",
        .run_count = 5,
        .tick_avg_us = 100.0,
        .tick_p95_us = 120,
        .tick_p99_us = 130,
        .tick_min_us = 80,
        .tick_max_us = 150,
        .total_us = 500,
        .stable_count = 5,
        .stable_rate = 1.0,
    };
    try std.testing.expect(result.stable_rate == 1.0);
    try std.testing.expect(result.tick_p95_us >= result.tick_min_us);
}

test "Benchmark runs apple_table scenario" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const result = try runBenchmark(gpa.allocator(), "apple_table", .{ .run_count = 3, .max_ticks = 20 });
    try std.testing.expect(result.run_count == 3);
    try std.testing.expect(result.tick_avg_us > 0);
}
