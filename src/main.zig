//! World VM - 3D Voxel Simulation Engine

const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const ai_traffic = @import("ai_traffic.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");
const renderer = @import("renderer.zig");
const Scenarios = @import("scenarios.zig");

const MAX_ENTITIES = 64;

pub fn main() !void {
    realMain() catch |err| switch (err) {
        error.BrokenPipe => return,
        else => return err,
    };
}

fn realMain() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();

    _ = args.next(); // skip argv[0]

    // Collect all remaining args first so we can peek at the first one.
    var arg_list = std.ArrayList([]const u8).init(allocator);
    defer arg_list.deinit();
    while (args.next()) |a| try arg_list.append(a);

    const all_args = arg_list.items;

    // If the first arg is a known subcommand, consume it; otherwise default to "run".
    var offset: usize = 0;
    const subcommand: []const u8 = if (all_args.len > 0 and
        (std.mem.eql(u8, all_args[0], "run") or
            std.mem.eql(u8, all_args[0], "bench") or
            std.mem.eql(u8, all_args[0], "dump")))
    blk: {
        offset = 1;
        break :blk all_args[0];
    } else "run";

    if (std.mem.eql(u8, subcommand, "run")) {
        try cmdRun(allocator, all_args[offset..]);
    } else if (std.mem.eql(u8, subcommand, "bench")) {
        try cmdBench(allocator, all_args[offset..]);
    } else if (std.mem.eql(u8, subcommand, "dump")) {
        try cmdDump(allocator, all_args[offset..]);
    } else {
        try std.io.getStdErr().writer().print("Usage: worldvm <run|bench|dump>\n", .{});
        return error.InvalidCommand;
    }
}

fn getScenario(name: []const u8) Scenarios.Scenario {
    if (std.mem.eql(u8, name, "hammer_glass")) return .hammer_glass;
    if (std.mem.eql(u8, name, "water_flow")) return .water_flow;
    if (std.mem.eql(u8, name, "bounce_test")) return .bounce_test;
    if (std.mem.eql(u8, name, "domino_chain")) return .domino_chain;
    if (std.mem.eql(u8, name, "pyramid_collapse")) return .pyramid_collapse;
    if (std.mem.eql(u8, name, "multi_stack")) return .multi_stack;
    if (std.mem.eql(u8, name, "gas_expand")) return .gas_expand;
    return .apple_table;
}

fn cmdRun(allocator: std.mem.Allocator, args: []const []const u8) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    var ticks: u32 = 50;
    var verbose = false;
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (i + 1 < args.len) {
                i += 1;
                scenario_name = args[i];
            }
        } else if (std.mem.eql(u8, arg, "--ticks") or std.mem.eql(u8, arg, "-t")) {
            if (i + 1 < args.len) {
                i += 1;
                ticks = std.fmt.parseInt(u32, args[i], 10) catch 50;
            }
        } else if (std.mem.eql(u8, arg, "-v") or std.mem.eql(u8, arg, "--verbose")) {
            verbose = true;
        }
    }

    try stdout.print("World VM - Scenario: {s}, Ticks: {d}\n\n", .{ scenario_name, ticks });

    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();

    const entry = try s1024.getPage(0);
    const scene = entry.scene.?;

    var entities: [MAX_ENTITIES]entity16.Entity16 = undefined;
    Scenarios.setupScenario(getScenario(scenario_name), &s1024, &entities);

    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &s1024, &entities);

    var t: u32 = 0;
    while (t < ticks and !engine.stable) : (t += 1) {
        // 1. Step physics (vehicle reads AI target speed, moves)
        tick_engine.stepPhysicsWorld(&engine, tick_engine.getFixedDT(&engine));
        // 2. Sync vehicle poses back to AI traffic
        ai_traffic.syncTrafficVehiclesFromPhysics();
        // 3. AI plans using actual vehicle state
        ai_traffic.updateAI(tick_engine.getFixedDT(&engine));
        if (verbose) {
            try renderer.renderScene(scene, stdout);
            try stdout.print("\n", .{});
        }
    }

    try stdout.print("=== Final State ===\n", .{});
    try renderer.renderScene(scene, stdout);
    try renderer.renderInstances(&s1024, &entities, stdout);
    try stdout.print("\nDone. Ticks: {d}, Stable: {any}\n", .{ t, engine.stable });
}

fn cmdBench(allocator: std.mem.Allocator, args: []const []const u8) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (i + 1 < args.len) {
                i += 1;
                scenario_name = args[i];
            }
        }
    }

    try stdout.print("Benchmark: {s}\n\n", .{scenario_name});

    var stable_count: u32 = 0;
    var total_us: u64 = 0;

    var r: u32 = 0;
    while (r < 5) : (r += 1) {
        var s1024 = scene1024.Scene1024.init(allocator);
        defer s1024.deinit();
        _ = try s1024.getPage(0);

        var entities: [MAX_ENTITIES]entity16.Entity16 = undefined;
        Scenarios.setupScenario(getScenario(scenario_name), &s1024, &entities);

        var engine: tick_engine.TickEngine = undefined;
        tick_engine.init(&engine, &s1024, &entities);

        const start = std.time.nanoTimestamp();
        var ticks_run: u32 = 0;
        while (ticks_run < 100 and !engine.stable) : (ticks_run += 1) {
            ai_traffic.updateAI(tick_engine.getFixedDT(&engine));
            tick_engine.stepPhysicsWorld(&engine, tick_engine.getFixedDT(&engine));
        }
        const elapsed = (@as(u64, @intCast(std.time.nanoTimestamp() - start))) / 1000;
        total_us += elapsed;
        if (engine.stable) stable_count += 1;

        try stdout.print("Run {}: {}us\n", .{ r + 1, elapsed });
    }

    try stdout.print("\nAvg: {}us, {}% stable\n", .{ total_us / 5, stable_count * 20 });
}

fn cmdDump(allocator: std.mem.Allocator, args: []const []const u8) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    var i: usize = 0;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (i + 1 < args.len) {
                i += 1;
                scenario_name = args[i];
            }
        }
    }

    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();
    const entry = try s1024.getPage(0);
    const scene = entry.scene.?;

    var entities: [MAX_ENTITIES]entity16.Entity16 = undefined;
    Scenarios.setupScenario(getScenario(scenario_name), &s1024, &entities);

    try stdout.print("=== {s} Initial State ===\n\n", .{scenario_name});
    try renderer.renderScene(scene, stdout);
    try renderer.renderInstances(&s1024, &entities, stdout);
}
