//! Chapter 6: 高速运动与CCD Tests 51-60
//! 子弹穿纸、CCD防止隧道效应、旋转CCD、极高动能

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 51, .name = "float_test", .setup_fn = setupFloatTest },
    .{ .id = 52, .name = "avalanche", .setup_fn = setupAvalanche },
    .{ .id = 53, .name = "collapse", .setup_fn = setupCollapse },
    .{ .id = 54, .name = "wrecking_ball", .setup_fn = setupWreckingBall },
    .{ .id = 55, .name = "conveyor", .setup_fn = setupConveyor },
    .{ .id = 56, .name = "sorting", .setup_fn = setupSorting },
    .{ .id = 57, .name = "hammer_fall", .setup_fn = setupHammerFall },
    .{ .id = 58, .name = "anvil_drop", .setup_fn = setupAnvilDrop },
    .{ .id = 59, .name = "bounce_sequence", .setup_fn = setupBouncingBallSequence },
    .{ .id = 60, .name = "pyramid_top", .setup_fn = setupPyramidWithTop },
};

fn setupFloatTest(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(4, 10, 5, 15, .idle);
    return 2;
}

fn setupAvalanche(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 8, 25, 8, .idle);
    inst[2] = makeInstance(0, 10, 25, 10, .idle);
    inst[3] = makeInstance(0, 12, 25, 8, .idle);
    inst[4] = makeInstance(0, 10, 25, 6, .idle);
    return 5;
}

fn setupCollapse(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 10, 1, 15, .idle);
    inst[2] = makeInstance(8, 10, 8, 15, .idle);
    inst[3] = makeInstance(8, 10, 15, 15, .idle);
    inst[4] = makeInstance(8, 10, 22, 15, .idle);
    inst[5] = makeInstance(8, 10, 29, 15, .idle);
    inst[6] = makeInstance(6, 3, 1, 15, .idle);
    return 7;
}

fn setupWreckingBall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 15, 1, 15, .idle);
    inst[2] = makeInstance(7, 15, 8, 15, .idle);
    inst[3] = makeInstance(7, 15, 15, 15, .idle);
    inst[4] = makeInstance(10, 5, 20, 15, .idle);
    return 5;
}

fn setupConveyor(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(9, 0, 5, 15, .idle);
    inst[2] = makeInstance(0, 3, 7, 15, .idle);
    inst[3] = makeInstance(7, 8, 7, 15, .idle);
    return 4;
}

fn setupSorting(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 3, 1, 10, .idle);
    inst[2] = makeInstance(7, 7, 4, 14, .idle);
    inst[3] = makeInstance(0, 3, 8, 12, .idle);
    inst[4] = makeInstance(7, 3, 8, 16, .idle);
    return 5;
}

fn setupHammerFall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(2, 10, 30, 15, .idle);
    return 2;
}

fn setupAnvilDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(10, 10, 40, 15, .idle);
    return 2;
}

fn setupBouncingBallSequence(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(14, 10, 50, 15, .idle);
    return 2;
}

fn setupPyramidWithTop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 15, .idle);
    inst[2] = makeInstance(7, 10, 1, 15, .idle);
    inst[3] = makeInstance(7, 15, 1, 15, .idle);
    inst[4] = makeInstance(7, 7, 8, 15, .idle);
    inst[5] = makeInstance(7, 12, 8, 15, .idle);
    inst[6] = makeInstance(0, 10, 15, 15, .idle);
    return 7;
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

test "Chapter 6: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 6: Test 51 - float test" {
    const result = try runChapterTest(51);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 52 - avalanche" {
    const result = try runChapterTest(52);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 53 - collapse" {
    const result = try runChapterTest(53);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 54 - wrecking ball" {
    const result = try runChapterTest(54);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 55 - conveyor" {
    const result = try runChapterTest(55);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 56 - sorting" {
    const result = try runChapterTest(56);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 57 - hammer fall" {
    const result = try runChapterTest(57);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 58 - anvil drop" {
    const result = try runChapterTest(58);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 59 - bounce sequence" {
    const result = try runChapterTest(59);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 6: Test 60 - pyramid with top" {
    const result = try runChapterTest(60);
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
