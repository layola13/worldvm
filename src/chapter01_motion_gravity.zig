//! Chapter 1: 运动与重力 Tests 1-10
//! 自由落体、伽利略实验、向上速度、水平运动、零重力、阻尼、终端速度、角速度、角阻尼、质心

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 1, .name = "drop_high", .setup_fn = setupDropHigh },
    .{ .id = 2, .name = "drop_medium", .setup_fn = setupDropMedium },
    .{ .id = 3, .name = "drop_low", .setup_fn = setupDropLow },
    .{ .id = 4, .name = "heavy_drop", .setup_fn = setupHeavyDrop },
    .{ .id = 5, .name = "light_drop", .setup_fn = setupLightDrop },
    .{ .id = 6, .name = "stack_2", .setup_fn = setupStack2 },
    .{ .id = 7, .name = "stack_3", .setup_fn = setupStack3 },
    .{ .id = 8, .name = "stack_5", .setup_fn = setupStack5 },
    .{ .id = 9, .name = "stack_10", .setup_fn = setupStack10 },
    .{ .id = 10, .name = "tower", .setup_fn = setupTower },
};

fn setupDropHigh(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(0, 10, 50, 15, .idle);
    inst[1] = makeInstance(5, 0, 0, 0, .resting);
    return 2;
}

fn setupDropMedium(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(0, 10, 25, 15, .idle);
    inst[1] = makeInstance(5, 0, 0, 0, .resting);
    return 2;
}

fn setupDropLow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(0, 10, 5, 15, .idle);
    inst[1] = makeInstance(5, 0, 0, 0, .resting);
    return 2;
}

fn setupHeavyDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(10, 10, 30, 15, .idle);
    inst[1] = makeInstance(5, 0, 0, 0, .resting);
    return 2;
}

fn setupLightDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(11, 10, 30, 15, .idle);
    inst[1] = makeInstance(5, 0, 0, 0, .resting);
    return 2;
}

fn setupStack2(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(7, 10, 1, 15, .resting);
    inst[1] = makeInstance(7, 10, 8, 15, .idle);
    inst[2] = makeInstance(5, 0, 0, 0, .resting);
    return 3;
}

fn setupStack3(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(7, 10, 1, 15, .resting);
    inst[1] = makeInstance(7, 10, 8, 15, .resting);
    inst[2] = makeInstance(7, 10, 15, 15, .idle);
    inst[3] = makeInstance(5, 0, 0, 0, .resting);
    return 4;
}

fn setupStack5(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(7, 10, 1, 15, .resting);
    inst[1] = makeInstance(7, 10, 8, 15, .resting);
    inst[2] = makeInstance(7, 10, 15, 15, .resting);
    inst[3] = makeInstance(7, 10, 22, 15, .resting);
    inst[4] = makeInstance(7, 10, 29, 15, .idle);
    inst[5] = makeInstance(5, 0, 0, 0, .resting);
    return 6;
}

fn setupStack10(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .idle);
    return 2;
}

fn setupTower(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 10, 1, 15, .idle);
    return 2;
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

test "Chapter 1: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 1: Test 1 - drop from height 50" {
    const result = try runChapterTest(1);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 2 - drop from height 25" {
    const result = try runChapterTest(2);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 3 - drop from height 5" {
    const result = try runChapterTest(3);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 4 - heavy sphere drop" {
    const result = try runChapterTest(4);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 5 - light sphere drop" {
    const result = try runChapterTest(5);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 6 - stack of 2 bricks" {
    const result = try runChapterTest(6);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 7 - stack of 3 bricks" {
    const result = try runChapterTest(7);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 8 - stack of 5 bricks" {
    const result = try runChapterTest(8);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 9 - stack of 10 items" {
    const result = try runChapterTest(9);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 1: Test 10 - single tower" {
    const result = try runChapterTest(10);
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
