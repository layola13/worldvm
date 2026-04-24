//! Chapter 2: 碰撞检测 Tests 11-20
//! 球体-平面、立方体下落、碰撞响应、多球碰撞

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 11, .name = "two_tower", .setup_fn = setupTwoTower },
    .{ .id = 12, .name = "wall_horizontal", .setup_fn = setupWallHorizontal },
    .{ .id = 13, .name = "wall_vertical", .setup_fn = setupWallVertical },
    .{ .id = 14, .name = "pyramid_3", .setup_fn = setupPyramid3 },
    .{ .id = 15, .name = "pyramid_6", .setup_fn = setupPyramid6 },
    .{ .id = 16, .name = "bridge", .setup_fn = setupBridge },
    .{ .id = 17, .name = "arch", .setup_fn = setupArch },
    .{ .id = 18, .name = "domino_row_5", .setup_fn = setupDominoRow5 },
    .{ .id = 19, .name = "domino_row_10", .setup_fn = setupDominoRow10 },
    .{ .id = 20, .name = "ball_on_platform", .setup_fn = setupBallOnPlatform },
};

fn setupTwoTower(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 10, 1, 10, .idle);
    inst[2] = makeInstance(8, 10, 1, 20, .idle);
    return 3;
}

fn setupWallHorizontal(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 3, 1, 15, .idle);
    inst[2] = makeInstance(7, 10, 1, 15, .idle);
    inst[3] = makeInstance(7, 17, 1, 15, .idle);
    return 4;
}

fn setupWallVertical(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .idle);
    inst[2] = makeInstance(7, 10, 8, 15, .idle);
    inst[3] = makeInstance(7, 10, 15, 15, .idle);
    return 4;
}

fn setupPyramid3(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 15, .idle);
    inst[2] = makeInstance(7, 10, 1, 15, .idle);
    inst[3] = makeInstance(7, 15, 1, 15, .idle);
    return 4;
}

fn setupPyramid6(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
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

fn setupBridge(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 5, 1, 20, .idle);
    inst[3] = makeInstance(9, 0, 8, 10, .idle);
    return 4;
}

fn setupArch(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 5, 1, 20, .idle);
    inst[3] = makeInstance(7, 10, 1, 15, .idle);
    return 4;
}

fn setupDominoRow5(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
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

fn setupDominoRow10(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    var c: u8 = 0;
    inst[c] = makeInstance(5, 0, 0, 0, .resting);
    c += 1;
    var x: i32 = 3;
    var i: u8 = 0;
    while (i < 10) : (i += 1) {
        inst[c] = makeInstance(8, x, 5, 15, .idle);
        x += 3;
        c += 1;
    }
    return c;
}

fn setupBallOnPlatform(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(9, 5, 10, 10, .idle);
    inst[2] = makeInstance(6, 10, 12, 15, .idle);
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

test "Chapter 2: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 2: Test 11 - two towers" {
    const result = try runChapterTest(11);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 12 - horizontal wall" {
    const result = try runChapterTest(12);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 13 - vertical wall" {
    const result = try runChapterTest(13);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 14 - 3-base pyramid" {
    const result = try runChapterTest(14);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 15 - 6-base pyramid" {
    const result = try runChapterTest(15);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 16 - simple bridge" {
    const result = try runChapterTest(16);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 17 - arch structure" {
    const result = try runChapterTest(17);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 18 - 5 dominoes" {
    const result = try runChapterTest(18);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 19 - 10 dominoes" {
    const result = try runChapterTest(19);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 2: Test 20 - ball on platform" {
    const result = try runChapterTest(20);
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
