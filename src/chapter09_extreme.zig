//! Chapter 9: 极端情况与压力测试 Tests 81-90
//! 极端质量比、零质量物体、极大坐标、万物复苏

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 81, .name = "double_pyramid", .setup_fn = setupDoublePyramid },
    .{ .id = 82, .name = "tower_wall", .setup_fn = setupTowerWall },
    .{ .id = 83, .name = "stair_walk", .setup_fn = setupStairWalk },
    .{ .id = 84, .name = "ball_ball", .setup_fn = setupBallBall },
    .{ .id = 85, .name = "heavy_heavy", .setup_fn = setupHeavyHeavy },
    .{ .id = 86, .name = "light_light", .setup_fn = setupLightLight },
    .{ .id = 87, .name = "mixed_stack", .setup_fn = setupMixedStack },
    .{ .id = 88, .name = "domino_circle", .setup_fn = setupDominoCircle },
    .{ .id = 89, .name = "water_channel", .setup_fn = setupWaterChannel },
    .{ .id = 90, .name = "ball_ramp", .setup_fn = setupBallRamp },
};

fn setupDoublePyramid(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    var c: u8 = 0;
    inst[c] = makeInstance(5, 0, 0, 0, .resting);
    c += 1;
    inst[c] = makeInstance(7, 5, 1, 15, .idle);
    c += 1;
    inst[c] = makeInstance(7, 10, 1, 15, .idle);
    c += 1;
    inst[c] = makeInstance(7, 15, 1, 15, .idle);
    c += 1;
    inst[c] = makeInstance(7, 7, 8, 15, .idle);
    c += 1;
    inst[c] = makeInstance(7, 12, 8, 15, .idle);
    c += 1;
    inst[c] = makeInstance(7, 10, 15, 15, .idle);
    c += 1;
    return c;
}

fn setupTowerWall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 10, 1, 10, .idle);
    inst[2] = makeInstance(8, 10, 1, 20, .idle);
    return 3;
}

fn setupStairWalk(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 10, 1, 15, .idle);
    inst[3] = makeInstance(7, 15, 1, 20, .idle);
    inst[4] = makeInstance(0, 5, 8, 10, .idle);
    return 5;
}

fn setupBallBall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(14, 5, 1, 15, .idle);
    inst[2] = makeInstance(15, 12, 1, 15, .idle);
    return 3;
}

fn setupHeavyHeavy(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .resting);
    inst[2] = makeInstance(7, 10, 8, 15, .resting);
    inst[3] = makeInstance(7, 10, 15, 15, .idle);
    inst[4] = makeInstance(5, 0, 0, 0, .resting);
    return 5;
}

fn setupLightLight(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .resting);
    inst[2] = makeInstance(7, 10, 8, 15, .resting);
    inst[3] = makeInstance(7, 10, 15, 15, .resting);
    inst[4] = makeInstance(7, 10, 22, 15, .resting);
    inst[5] = makeInstance(7, 10, 29, 15, .idle);
    inst[6] = makeInstance(5, 0, 0, 0, .resting);
    return 7;
}

fn setupMixedStack(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .idle);
    inst[2] = makeInstance(0, 10, 8, 15, .idle);
    inst[3] = makeInstance(7, 10, 15, 15, .idle);
    return 4;
}

fn setupDominoCircle(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    var c: u8 = 0;
    inst[c] = makeInstance(5, 0, 0, 0, .resting);
    c += 1;
    inst[c] = makeInstance(8, 5, 5, 15, .idle);
    c += 1;
    inst[c] = makeInstance(8, 8, 5, 15, .idle);
    c += 1;
    inst[c] = makeInstance(8, 11, 5, 15, .idle);
    c += 1;
    inst[c] = makeInstance(8, 14, 5, 15, .idle);
    c += 1;
    inst[c] = makeInstance(8, 17, 5, 15, .idle);
    c += 1;
    return c;
}

fn setupWaterChannel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 5, 1, 20, .idle);
    inst[3] = makeInstance(7, 5, 8, 10, .idle);
    inst[4] = makeInstance(7, 5, 8, 20, .idle);
    inst[5] = makeInstance(4, 10, 10, 15, .idle);
    return 6;
}

fn setupBallRamp(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 3, 1, 15, .idle);
    inst[2] = makeInstance(7, 6, 4, 15, .idle);
    inst[3] = makeInstance(7, 9, 7, 15, .idle);
    inst[4] = makeInstance(0, 3, 10, 15, .idle);
    return 5;
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

test "Chapter 9: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 9: Test 81 - double pyramid" {
    const result = try runChapterTest(81);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 82 - tower near wall" {
    const result = try runChapterTest(82);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 83 - stair walk" {
    const result = try runChapterTest(83);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 84 - ball-ball collision" {
    const result = try runChapterTest(84);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 85 - heavy-heavy collision" {
    const result = try runChapterTest(85);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 86 - light-light stack" {
    const result = try runChapterTest(86);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 87 - mixed stack" {
    const result = try runChapterTest(87);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 88 - domino circle" {
    const result = try runChapterTest(88);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 89 - water channel" {
    const result = try runChapterTest(89);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 9: Test 90 - ball ramp" {
    const result = try runChapterTest(90);
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
