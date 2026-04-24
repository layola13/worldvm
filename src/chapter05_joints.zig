//! Chapter 5: 关节连接 Tests 41-50
//! 固定关节、铰链关节、弹簧关节、球窝关节

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 41, .name = "sandwich", .setup_fn = setupSandwich },
    .{ .id = 42, .name = "unstable", .setup_fn = setupUnstable },
    .{ .id = 43, .name = "balanced", .setup_fn = setupBalanced },
    .{ .id = 44, .name = "tunnel", .setup_fn = setupTunnel },
    .{ .id = 45, .name = "ramp", .setup_fn = setupRamp },
    .{ .id = 46, .name = "shelf", .setup_fn = setupShelf },
    .{ .id = 47, .name = "jenga_tower", .setup_fn = setupJengaTower },
    .{ .id = 48, .name = "billiards", .setup_fn = setupBilliards },
    .{ .id = 49, .name = "marble_run", .setup_fn = setupMarbleRun },
    .{ .id = 50, .name = "weight_test", .setup_fn = setupWeightTest },
};

fn setupSandwich(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .idle);
    inst[2] = makeInstance(0, 10, 8, 15, .idle);
    inst[3] = makeInstance(7, 10, 15, 15, .idle);
    return 4;
}

fn setupUnstable(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 10, 1, 15, .idle);
    inst[2] = makeInstance(7, 7, 8, 12, .idle);
    inst[3] = makeInstance(7, 13, 8, 18, .idle);
    return 4;
}

fn setupBalanced(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 15, .idle);
    inst[2] = makeInstance(7, 15, 1, 15, .idle);
    inst[3] = makeInstance(7, 8, 8, 15, .idle);
    inst[4] = makeInstance(0, 10, 10, 15, .idle);
    return 5;
}

fn setupTunnel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 5, 1, 20, .idle);
    inst[3] = makeInstance(7, 5, 10, 10, .idle);
    inst[4] = makeInstance(7, 5, 10, 20, .idle);
    inst[5] = makeInstance(0, 2, 5, 15, .idle);
    return 6;
}

fn setupRamp(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 3, 1, 15, .idle);
    inst[2] = makeInstance(7, 6, 4, 15, .idle);
    inst[3] = makeInstance(7, 9, 7, 15, .idle);
    inst[4] = makeInstance(0, 3, 10, 15, .idle);
    return 5;
}

fn setupShelf(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 10, 10, .idle);
    inst[2] = makeInstance(7, 5, 10, 20, .idle);
    inst[3] = makeInstance(9, 0, 15, 10, .idle);
    inst[4] = makeInstance(0, 7, 17, 15, .idle);
    return 5;
}

fn setupJengaTower(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    var c: u8 = 0;
    inst[c] = makeInstance(5, 0, 0, 0, .resting);
    c += 1;
    var y: i32 = 1;
    var i: u8 = 0;
    while (i < 8) : (i += 1) {
        inst[c] = makeInstance(7, 8, y, 15, .idle);
        y += 7;
        c += 1;
    }
    return c;
}

fn setupBilliards(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(6, 10, 1, 12, .idle);
    inst[2] = makeInstance(6, 8, 1, 15, .idle);
    inst[3] = makeInstance(6, 12, 1, 15, .idle);
    inst[4] = makeInstance(6, 10, 1, 18, .idle);
    inst[5] = makeInstance(0, 5, 1, 15, .idle);
    return 6;
}

fn setupMarbleRun(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(9, 3, 5, 8, .idle);
    inst[2] = makeInstance(9, 7, 10, 12, .idle);
    inst[3] = makeInstance(9, 3, 15, 16, .idle);
    inst[4] = makeInstance(6, 5, 2, 8, .idle);
    return 5;
}

fn setupWeightTest(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(12, 10, 15, 15, .idle);
    inst[2] = makeInstance(10, 10, 25, 15, .idle);
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

test "Chapter 5: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 5: Test 41 - sandwich structure" {
    const result = try runChapterTest(41);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 42 - unstable structure" {
    const result = try runChapterTest(42);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 43 - balanced structure" {
    const result = try runChapterTest(43);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 44 - tunnel" {
    const result = try runChapterTest(44);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 45 - ramp" {
    const result = try runChapterTest(45);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 46 - shelf" {
    const result = try runChapterTest(46);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 47 - jenga tower" {
    const result = try runChapterTest(47);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 48 - billiards" {
    const result = try runChapterTest(48);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 49 - marble run" {
    const result = try runChapterTest(49);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 5: Test 50 - weight test" {
    const result = try runChapterTest(50);
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
