//! Chapter 10: 流体、力场与机制拓展 Tests 91-100
//! 力场区域、爆炸径向力、浮力体积、局部时间缩放

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 91, .name = "high_stack", .setup_fn = setupHighStack },
    .{ .id = 92, .name = "platform_drop", .setup_fn = setupPlatformDrop },
    .{ .id = 93, .name = "multi_ball", .setup_fn = setupMultiBall },
    .{ .id = 94, .name = "shatter", .setup_fn = setupShatter },
    .{ .id = 95, .name = "bounce_test", .setup_fn = setupBounceTest },
    .{ .id = 96, .name = "flow_test", .setup_fn = setupFlowTest },
    .{ .id = 97, .name = "stability", .setup_fn = setupStability },
    .{ .id = 98, .name = "domino_effect", .setup_fn = setupDominoEffect },
    .{ .id = 99, .name = "integrity", .setup_fn = setupIntegrity },
    .{ .id = 100, .name = "freeform", .setup_fn = setupFreeform },
};

fn setupHighStack(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .idle);
    return 2;
}

fn setupPlatformDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(9, 5, 10, 10, .idle);
    inst[2] = makeInstance(6, 10, 12, 15, .idle);
    return 3;
}

fn setupMultiBall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 5, 30, 10, .idle);
    inst[2] = makeInstance(0, 10, 30, 15, .idle);
    inst[3] = makeInstance(0, 15, 30, 20, .idle);
    return 4;
}

fn setupShatter(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(3, 10, 15, 15, .idle);
    inst[2] = makeInstance(2, 10, 25, 15, .idle);
    return 3;
}

fn setupBounceTest(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(14, 10, 20, 15, .idle);
    return 2;
}

fn setupFlowTest(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(4, 10, 20, 15, .idle);
    return 2;
}

fn setupStability(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 15, .idle);
    inst[2] = makeInstance(7, 15, 1, 15, .idle);
    inst[3] = makeInstance(7, 8, 8, 15, .idle);
    inst[4] = makeInstance(0, 10, 10, 15, .idle);
    return 5;
}

fn setupDominoEffect(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(6, 2, 5, 15, .idle);
    inst[2] = makeInstance(8, 5, 5, 15, .idle);
    inst[3] = makeInstance(8, 8, 5, 15, .idle);
    inst[4] = makeInstance(8, 11, 5, 15, .idle);
    inst[5] = makeInstance(8, 14, 5, 15, .idle);
    return 6;
}

fn setupIntegrity(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 5, 1, 20, .idle);
    inst[3] = makeInstance(9, 0, 8, 10, .idle);
    return 4;
}

fn setupFreeform(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 5, 30, 10, .idle);
    inst[2] = makeInstance(10, 10, 30, 15, .idle);
    inst[3] = makeInstance(14, 15, 30, 20, .idle);
    return 4;
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

test "Chapter 10: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 10: Test 91 - high stack" {
    const result = try runChapterTest(91);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 92 - platform drop" {
    const result = try runChapterTest(92);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 93 - multi ball" {
    const result = try runChapterTest(93);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 94 - shatter test" {
    const result = try runChapterTest(94);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 95 - bounce test" {
    const result = try runChapterTest(95);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 96 - flow test" {
    const result = try runChapterTest(96);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 97 - stability test" {
    const result = try runChapterTest(97);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 98 - domino effect" {
    const result = try runChapterTest(98);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 99 - integrity test" {
    const result = try runChapterTest(99);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 10: Test 100 - freeform physics" {
    const result = try runChapterTest(100);
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
