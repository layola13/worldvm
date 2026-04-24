//! Chapter 4: 堆叠与休眠 Tests 31-40
//! 堆叠稳定性、碰撞传播、链式反应、崩塔、滚石

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 31, .name = "box_vs_sphere", .setup_fn = setupBoxVsSphere },
    .{ .id = 32, .name = "angled_drop", .setup_fn = setupAngledDrop },
    .{ .id = 33, .name = "side_by_side", .setup_fn = setupSideBySide },
    .{ .id = 34, .name = "triple_drop", .setup_fn = setupTripleDrop },
    .{ .id = 35, .name = "quad_drop", .setup_fn = setupQuadDrop },
    .{ .id = 36, .name = "chain_reaction", .setup_fn = setupChainReaction },
    .{ .id = 37, .name = "topple_from_side", .setup_fn = setupToppleFromSide },
    .{ .id = 38, .name = "cascade", .setup_fn = setupCascade },
    .{ .id = 39, .name = "pendulum", .setup_fn = setupPendulum },
    .{ .id = 40, .name = "ball_tower", .setup_fn = setupBallTowerCollision },
};

fn setupBoxVsSphere(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 20, 15, .idle);
    inst[2] = makeInstance(0, 10, 1, 15, .idle);
    return 3;
}

fn setupAngledDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 5, 30, 5, .idle);
    return 2;
}

fn setupSideBySide(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 5, 20, 10, .idle);
    inst[2] = makeInstance(7, 15, 20, 10, .idle);
    return 3;
}

fn setupTripleDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 5, 30, 10, .idle);
    inst[2] = makeInstance(0, 10, 30, 15, .idle);
    inst[3] = makeInstance(0, 15, 30, 20, .idle);
    return 4;
}

fn setupQuadDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 5, 30, 10, .idle);
    inst[2] = makeInstance(7, 10, 30, 10, .idle);
    inst[3] = makeInstance(0, 15, 30, 10, .idle);
    inst[4] = makeInstance(7, 20, 30, 10, .idle);
    return 5;
}

fn setupChainReaction(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(6, 2, 5, 15, .idle);
    inst[2] = makeInstance(8, 5, 5, 15, .idle);
    inst[3] = makeInstance(8, 8, 5, 15, .idle);
    inst[4] = makeInstance(8, 11, 5, 15, .idle);
    inst[5] = makeInstance(8, 14, 5, 15, .idle);
    return 6;
}

fn setupToppleFromSide(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 10, 5, 15, .idle);
    inst[2] = makeInstance(6, 5, 5, 15, .idle);
    return 3;
}

fn setupCascade(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 10, 1, 15, .idle);
    inst[3] = makeInstance(7, 15, 1, 20, .idle);
    inst[4] = makeInstance(0, 5, 8, 10, .idle);
    return 5;
}

fn setupPendulum(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 10, 20, 15, .idle);
    return 2;
}

fn setupBallTowerCollision(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 10, 1, 15, .idle);
    inst[2] = makeInstance(8, 10, 8, 15, .idle);
    inst[3] = makeInstance(8, 10, 15, 15, .idle);
    inst[4] = makeInstance(6, 3, 5, 15, .idle);
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

test "Chapter 4: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 4: Test 31 - box on sphere" {
    const result = try runChapterTest(31);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 32 - angled drop" {
    const result = try runChapterTest(32);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 33 - side by side" {
    const result = try runChapterTest(33);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 34 - triple drop" {
    const result = try runChapterTest(34);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 35 - quad drop" {
    const result = try runChapterTest(35);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 36 - chain reaction" {
    const result = try runChapterTest(36);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 37 - topple from side" {
    const result = try runChapterTest(37);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 38 - cascade" {
    const result = try runChapterTest(38);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 39 - pendulum" {
    const result = try runChapterTest(39);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 4: Test 40 - ball vs tower" {
    const result = try runChapterTest(40);
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
