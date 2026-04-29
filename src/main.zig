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
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();
    
    var args = try std.process.argsWithAllocator(allocator);
    defer args.deinit();
    
    _ = args.next();
    const subcommand = args.next() orelse "run";
    
    if (std.mem.eql(u8, subcommand, "run")) {
        try cmdRun(allocator, &args);
    } else if (std.mem.eql(u8, subcommand, "bench")) {
        try cmdBench(allocator, &args);
    } else if (std.mem.eql(u8, subcommand, "dump")) {
        try cmdDump(allocator, &args);
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

fn cmdRun(allocator: std.mem.Allocator, args: *std.process.ArgIterator) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    var ticks: u32 = 50;
    var verbose = false;
    
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (args.next()) |v| scenario_name = v;
        } else if (std.mem.eql(u8, arg, "--ticks") or std.mem.eql(u8, arg, "-t")) {
            if (args.next()) |v| ticks = std.fmt.parseInt(u32, v, 10) catch 50;
        } else if (std.mem.eql(u8, arg, "-v") or std.mem.eql(u8, arg, "--verbose")) {
            verbose = true;
        }
    }
    
    try stdout.print("World VM - Scenario: {s}, Ticks: {d}\n\n", .{scenario_name, ticks});
    
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
    try stdout.print("\nDone. Ticks: {d}, Stable: {any}\n", .{t, engine.stable});
}

fn cmdBench(allocator: std.mem.Allocator, args: *std.process.ArgIterator) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (args.next()) |v| scenario_name = v;
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
        
        try stdout.print("Run {}: {}us\n", .{r+1, elapsed});
    }
    
    try stdout.print("\nAvg: {}us, {}% stable\n", .{total_us / 5, stable_count * 20});
}

fn cmdDump(allocator: std.mem.Allocator, args: *std.process.ArgIterator) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    
    while (args.next()) |arg| {
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (args.next()) |v| scenario_name = v;
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
