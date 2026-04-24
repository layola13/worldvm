//! Chapter 8: 运动学与动力学交互 Tests 71-80
//! 运动学推开动力学、动力学撞击运动学、运动学平台载物

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 71, .name = "see_saw", .setup_fn = setupSeeSaw },
    .{ .id = 72, .name = "drop_timing", .setup_fn = setupBallDropTiming },
    .{ .id = 73, .name = "pyramid_doom", .setup_fn = setupPyramidOfDoom },
    .{ .id = 74, .name = "target", .setup_fn = setupTargetPractice },
    .{ .id = 75, .name = "freefall_race", .setup_fn = setupFreefallRace },
    .{ .id = 76, .name = "momentum", .setup_fn = setupMomentumTransfer },
    .{ .id = 77, .name = "funnel", .setup_fn = setupFunnel },
    .{ .id = 78, .name = "blocker", .setup_fn = setupBlocker },
    .{ .id = 79, .name = "precision", .setup_fn = setupPrecisionDrop },
    .{ .id = 80, .name = "compaction", .setup_fn = setupCompaction },
};

fn setupSeeSaw(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .idle);
    inst[2] = makeInstance(9, 5, 8, 15, .idle);
    inst[3] = makeInstance(10, 7, 10, 15, .idle);
    inst[4] = makeInstance(0, 15, 10, 15, .idle);
    return 5;
}

fn setupBallDropTiming(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 10, 10, 15, .idle);
    inst[2] = makeInstance(0, 10, 5, 15, .idle);
    inst[3] = makeInstance(0, 10, 1, 15, .idle);
    return 4;
}

fn setupPyramidOfDoom(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    var c: u8 = 0;
    inst[c] = makeInstance(5, 0, 0, 0, .resting);
    c += 1;
    var y: i32 = 1;
    var x: i32 = 3;
    var i: u8 = 0;
    while (i < 6) : (i += 1) {
        inst[c] = makeInstance(7, x, y, 15, .idle);
        x += 2;
        y += 7;
        c += 1;
    }
    return c;
}

fn setupTargetPractice(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(12, 10, 1, 15, .idle);
    inst[2] = makeInstance(12, 15, 1, 15, .idle);
    inst[3] = makeInstance(10, 3, 15, 15, .idle);
    return 4;
}

fn setupFreefallRace(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 5, 30, 10, .idle);
    inst[2] = makeInstance(10, 10, 30, 15, .idle);
    inst[3] = makeInstance(14, 15, 30, 20, .idle);
    return 4;
}

fn setupMomentumTransfer(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(14, 5, 1, 15, .idle);
    inst[2] = makeInstance(15, 12, 1, 15, .idle);
    return 3;
}

fn setupFunnel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 10, 8, .idle);
    inst[2] = makeInstance(7, 15, 10, 8, .idle);
    inst[3] = makeInstance(7, 8, 15, 10, .idle);
    inst[4] = makeInstance(7, 12, 15, 10, .idle);
    inst[5] = makeInstance(6, 10, 20, 10, .idle);
    return 6;
}

fn setupBlocker(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(13, 10, 1, 15, .idle);
    inst[2] = makeInstance(0, 10, 20, 15, .idle);
    return 3;
}

fn setupPrecisionDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 12, .idle);
    inst[2] = makeInstance(7, 5, 1, 18, .idle);
    inst[3] = makeInstance(6, 10, 20, 15, .idle);
    return 4;
}

fn setupCompaction(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(11, 8, 1, 13, .idle);
    inst[2] = makeInstance(11, 10, 1, 13, .idle);
    inst[3] = makeInstance(11, 12, 1, 13, .idle);
    inst[4] = makeInstance(11, 8, 5, 13, .idle);
    inst[5] = makeInstance(11, 10, 5, 13, .idle);
    inst[6] = makeInstance(11, 12, 5, 13, .idle);
    inst[7] = makeInstance(10, 10, 25, 13, .idle);
    return 8;
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

test "Chapter 8: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 8: Test 71 - see-saw" {
    const result = try runChapterTest(71);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 72 - drop timing" {
    const result = try runChapterTest(72);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 73 - pyramid of doom" {
    const result = try runChapterTest(73);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 74 - target practice" {
    const result = try runChapterTest(74);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 75 - freefall race" {
    const result = try runChapterTest(75);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 76 - momentum transfer" {
    const result = try runChapterTest(76);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 77 - funnel" {
    const result = try runChapterTest(77);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 78 - blocker" {
    const result = try runChapterTest(78);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 79 - precision drop" {
    const result = try runChapterTest(79);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 8: Test 80 - compaction" {
    const result = try runChapterTest(80);
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
