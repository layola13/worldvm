//! World VM - 3D Voxel Simulation Engine

const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
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
    
    // skip program name
    _ = args.next();
    
    const subcommand = args.next() orelse "run";
    
    if (std.mem.eql(u8, subcommand, "run")) {
        try cmdRun(&args);
    } else if (std.mem.eql(u8, subcommand, "bench")) {
        try cmdBench(&args);
    } else if (std.mem.eql(u8, subcommand, "dump")) {
        try cmdDump(&args);
    } else {
        try std.io.getStdErr().writer().print("Usage: worldvm <run|bench|dump>\n", .{});
        return error.InvalidCommand;
    }
}

fn getScenario(name: []const u8) Scenarios.Scenario {
    if (std.mem.eql(u8, name, "hammer_glass")) return .hammer_glass;
    if (std.mem.eql(u8, name, "water_flow")) return .water_flow;
    return .apple_table;
}

fn cmdRun(args: *anyopaque) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    var ticks: u32 = 50;
    var verbose = false;
    
    var iter = @as(*std.process.ArgIterator, @ptrCast(@alignCast(args)));
    while (iter.next()) |arg| {
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (iter.next()) |v| scenario_name = v;
        } else if (std.mem.eql(u8, arg, "--ticks") or std.mem.eql(u8, arg, "-t")) {
            if (iter.next()) |v| ticks = std.fmt.parseInt(u32, v, 10) catch 50;
        } else if (std.mem.eql(u8, arg, "-v") or std.mem.eql(u8, arg, "--verbose")) {
            verbose = true;
        }
    }
    
    try stdout.print("World VM - Scenario: {s}, Ticks: {d}\n\n", .{scenario_name, ticks});
    
    var scene = scene32.initScene();
    var entities: [MAX_ENTITIES]entity16.Entity16 = undefined;
    Scenarios.setupScenario(getScenario(scenario_name), &scene, &entities);
    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &scene, &entities);
    
    var t: u32 = 0;
    while (t < ticks and !engine.stable) : (t += 1) {
        _ = tick_engine.stepTick(&engine);
        if (verbose) {
            try renderer.renderScene(&scene, stdout);
            try stdout.print("\n", .{});
        }
    }
    
    try stdout.print("=== Final State ===\n", .{});
    try renderer.renderScene(&scene, stdout);
    try renderer.renderInstances(&scene, &entities, stdout);
    try stdout.print("\nDone. Ticks: {d}, Stable: {any}\n", .{scene.tick, engine.stable});
}

fn cmdBench(args: *anyopaque) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    
    var iter = @as(*std.process.ArgIterator, @ptrCast(@alignCast(args)));
    while (iter.next()) |arg| {
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (iter.next()) |v| scenario_name = v;
        }
    }
    
    try stdout.print("Benchmark: {s}\n\n", .{scenario_name});
    
    var stable_count: u32 = 0;
    var total_us: u64 = 0;
    
    var r: u32 = 0;
    while (r < 5) : (r += 1) {
        var scene = scene32.initScene();
        var entities: [MAX_ENTITIES]entity16.Entity16 = undefined;
        Scenarios.setupScenario(getScenario(scenario_name), &scene, &entities);
        var engine: tick_engine.TickEngine = undefined;
        tick_engine.init(&engine, &scene, &entities);
        
        const start = std.time.nanoTimestamp();
        _ = tick_engine.runTicks(&engine, 100);
        const elapsed = (@as(u64, @intCast(std.time.nanoTimestamp() - start))) / 1000;
        total_us += elapsed;
        if (engine.stable) stable_count += 1;
        
        try stdout.print("Run {}: {}us\n", .{r+1, elapsed});
    }
    
    try stdout.print("\nAvg: {}us, {}% stable\n", .{total_us / 5, stable_count * 20});
}

fn cmdDump(args: *anyopaque) !void {
    const stdout = std.io.getStdOut().writer();
    var scenario_name: []const u8 = "apple_table";
    
    var iter = @as(*std.process.ArgIterator, @ptrCast(@alignCast(args)));
    while (iter.next()) |arg| {
        if (std.mem.eql(u8, arg, "--scenario") or std.mem.eql(u8, arg, "-s")) {
            if (iter.next()) |v| scenario_name = v;
        }
    }
    
    var scene = scene32.initScene();
    var entities: [MAX_ENTITIES]entity16.Entity16 = undefined;
    Scenarios.setupScenario(getScenario(scenario_name), &scene, &entities);
    
    try stdout.print("=== {s} Initial State ===\n\n", .{scenario_name});
    try renderer.renderScene(&scene, stdout);
    try renderer.renderInstances(&scene, &entities, stdout);
    
    try stdout.print("\nEntity count:\n", .{});
    var i: usize = 0;
    while (i < 6) : (i += 1) {
        const voxels = entity16.countVoxels(&entities[i]);
        try stdout.print("Entity {}: {} voxels, {} mass\n", .{
            i, voxels, entities[i].physics.mass
        });
    }
}

pub const LexiconResponse = struct {
    status: ResponseStatus,
    confidence: u8,
};

pub const ResponseStatus = enum {
    ok,
    not_connected,
    timeout,
};

pub const ExternalLexiconAdapter = struct {
    pub fn query(_: *const ExternalLexiconAdapter, input: []const u8) LexiconResponse {
        _ = input;
        return .{ .status = .not_connected, .confidence = 0 };
    }
};

pub const Verifier = struct {
    pub fn verify(_: *const Verifier, response: *const LexiconResponse) bool {
        return response.status == .ok and response.confidence >= 50;
    }
};
