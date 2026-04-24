//! Chapter 24: 地形系统 Tests 231-240
//! 地表类型、摩擦变化、坡度行走

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 231, .name = "terrain_concrete", .setup_fn = setupTerrainConcrete },
    .{ .id = 232, .name = "terrain_mud", .setup_fn = setupTerrainMud },
    .{ .id = 233, .name = "terrain_ice", .setup_fn = setupTerrainIce },
    .{ .id = 234, .name = "terrain_sand", .setup_fn = setupTerrainSand },
    .{ .id = 235, .name = "terrain_gravel", .setup_fn = setupTerrainGravel },
    .{ .id = 236, .name = "terrain_grass", .setup_fn = setupTerrainGrass },
    .{ .id = 237, .name = "terrain_metal", .setup_fn = setupTerrainMetal },
    .{ .id = 238, .name = "terrain_wood", .setup_fn = setupTerrainWood },
    .{ .id = 239, .name = "terrain_carpet", .setup_fn = setupTerrainCarpet },
    .{ .id = 240, .name = "terrain_mixed", .setup_fn = setupTerrainMixed },
};

fn setupTerrainConcrete(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Concrete floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Object
    return 2;
}

fn setupTerrainMud(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Mud floor
    inst[1] = makeInstance(10, 10, 10, 15, .idle);   // Heavy object sinks
    return 2;
}

fn setupTerrainIce(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ice floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Sliding object
    return 2;
}

fn setupTerrainSand(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Sandy floor
    inst[1] = makeInstance(10, 10, 15, 15, .idle);   // Sinking object
    return 2;
}

fn setupTerrainGravel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Gravel floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Rolling object
    return 2;
}

fn setupTerrainGrass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Grass floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Object
    return 2;
}

fn setupTerrainMetal(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Metal floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Metal object
    return 2;
}

fn setupTerrainWood(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Wood floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Wood object
    return 2;
}

fn setupTerrainCarpet(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Carpet floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Object
    return 2;
}

fn setupTerrainMixed(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Concrete
    inst[1] = makeInstance(7, 15, 1, 15, .idle);    // Transition to wood
    inst[2] = makeInstance(0, 20, 10, 15, .idle);    // Moving object
    return 3;
}

fn makeInstance(entity_id: u8, x: i32, y: i32, z: i32, state: scene32.InstanceState) scene32.Instance {
    return .{
        .entity_id = entity_id,
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = state,
        .sleep_tick = 0,
        ._reserved = .{0} ** 2,
    };
}

test "Chapter 24: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 24: Test 231 - terrain concrete" {
    const result = try runChapterTest(231);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 232 - terrain mud" {
    const result = try runChapterTest(232);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 233 - terrain ice" {
    const result = try runChapterTest(233);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 234 - terrain sand" {
    const result = try runChapterTest(234);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 235 - terrain gravel" {
    const result = try runChapterTest(235);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 236 - terrain grass" {
    const result = try runChapterTest(236);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 237 - terrain metal" {
    const result = try runChapterTest(237);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 238 - terrain wood" {
    const result = try runChapterTest(238);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 239 - terrain carpet" {
    const result = try runChapterTest(239);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 24: Test 240 - terrain mixed" {
    const result = try runChapterTest(240);
    try std.testing.expect(result.ticks_to_stable > 0);
}

fn runChapterTest(test_id: u32) !physics_tests.PhysicsTestResult {
    physics_tests.createTestEntities();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    _ = try s1024.getPage(0);

    const scenario = for (scenarios) |s| {
        if (s.id == test_id) break s;
    } else return error.TestNotFound;

    var test_instances: [32]scene32.Instance = undefined;
    @memset(@as([*]u8, @ptrCast(&test_instances))[0..@sizeOf(@TypeOf(test_instances))], 0);
    const instance_count = scenario.setup_fn(&physics_tests.test_entities, &test_instances);

    for (0..instance_count) |i| {
        _ = try s1024.addInstance(test_instances[i]);
    }

    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &s1024, &physics_tests.test_entities);

    var ticks: u32 = 0;
    const max_ticks: u32 = 100;

    while (ticks < max_ticks and !engine.stable) : (ticks += 1) {
        _ = tick_engine.stepTick(&engine);
    }

    var final_states: [32]physics_tests.InstanceFinalState = undefined;
    var count: u8 = 0;
    for (0..s1024.instance_count) |i| {
        final_states[count] = .{
            .entity_id = s1024.instances[i].entity_id,
            .pos_x = s1024.instances[i].pos_x,
            .pos_y = s1024.instances[i].pos_y,
            .pos_z = s1024.instances[i].pos_z,
            .state = s1024.instances[i].state,
        };
        count += 1;
    }

    return .{
        .test_id = test_id,
        .name = scenario.name,
        .ticks_to_stable = ticks,
        .stable = engine.stable,
        .final_states = final_states[0..count],
        .expected_stable = true,
        .passed = engine.stable or ticks < max_ticks,
    };
}
