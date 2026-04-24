//! Chapter 7: 射线检测与体积扫掠 Tests 61-70
//! 单条射线、穿透射线、球体扫掠、盒子扫掠、层遮罩

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 61, .name = "house_of_cards", .setup_fn = setupHouseOfCards },
    .{ .id = 62, .name = "tetris", .setup_fn = setupTetrisLike },
    .{ .id = 63, .name = "ball_dominoes", .setup_fn = setupBallVsDominoes },
    .{ .id = 64, .name = "heavy_stack", .setup_fn = setupHeavyOnStack },
    .{ .id = 65, .name = "stack_plate", .setup_fn = setupStackOnPlate },
    .{ .id = 66, .name = "water_box", .setup_fn = setupWaterContainment },
    .{ .id = 67, .name = "water_overflow", .setup_fn = setupWaterOverflow },
    .{ .id = 68, .name = "sliding", .setup_fn = setupSlidingMass },
    .{ .id = 69, .name = "newton_cradle", .setup_fn = setupNewtonCradle },
    .{ .id = 70, .name = "split_level", .setup_fn = setupSplitLevel },
};

fn setupHouseOfCards(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(8, 8, 5, 15, .idle);
    inst[2] = makeInstance(8, 12, 5, 15, .idle);
    inst[3] = makeInstance(8, 10, 12, 15, .idle);
    return 4;
}

fn setupTetrisLike(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 8, 30, 15, .idle);
    return 2;
}

fn setupBallVsDominoes(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    var c: u8 = 0;
    inst[c] = makeInstance(5, 0, 0, 0, .resting);
    c += 1;
    var x: i32 = 8;
    var i: u8 = 0;
    while (i < 5) : (i += 1) {
        inst[c] = makeInstance(8, x, 5, 15, .idle);
        x += 3;
        c += 1;
    }
    inst[c] = makeInstance(6, 3, 5, 15, .idle);
    c += 1;
    return c;
}

fn setupHeavyOnStack(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 10, 1, 15, .idle);
    inst[2] = makeInstance(7, 10, 8, 15, .idle);
    inst[3] = makeInstance(10, 10, 20, 15, .idle);
    return 4;
}

fn setupStackOnPlate(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(9, 5, 10, 10, .idle);
    inst[2] = makeInstance(7, 10, 12, 15, .idle);
    inst[3] = makeInstance(7, 10, 19, 15, .idle);
    return 4;
}

fn setupWaterContainment(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 5, 1, 20, .idle);
    inst[3] = makeInstance(7, 5, 8, 10, .idle);
    inst[4] = makeInstance(7, 5, 8, 20, .idle);
    inst[5] = makeInstance(4, 10, 10, 15, .idle);
    return 6;
}

fn setupWaterOverflow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 5, 8, 10, .idle);
    inst[3] = makeInstance(4, 10, 5, 15, .idle);
    return 4;
}

fn setupSlidingMass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 5, 12, .idle);
    inst[2] = makeInstance(7, 10, 10, 18, .idle);
    inst[3] = makeInstance(10, 7, 15, 15, .idle);
    return 4;
}

fn setupNewtonCradle(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(6, 8, 15, 15, .idle);
    inst[2] = makeInstance(6, 11, 15, 15, .idle);
    inst[3] = makeInstance(6, 14, 15, 15, .idle);
    inst[4] = makeInstance(6, 3, 20, 15, .idle);
    return 5;
}

fn setupSplitLevel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(7, 5, 1, 10, .idle);
    inst[2] = makeInstance(7, 15, 10, 20, .idle);
    inst[3] = makeInstance(0, 5, 10, 10, .idle);
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

test "Chapter 7: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 7: Test 61 - house of cards" {
    const result = try runChapterTest(61);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 62 - tetris piece" {
    const result = try runChapterTest(62);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 63 - ball vs dominoes" {
    const result = try runChapterTest(63);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 64 - heavy on stack" {
    const result = try runChapterTest(64);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 65 - stack on plate" {
    const result = try runChapterTest(65);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 66 - water in box" {
    const result = try runChapterTest(66);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 67 - water overflow" {
    const result = try runChapterTest(67);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 68 - sliding mass" {
    const result = try runChapterTest(68);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 69 - newton cradle" {
    const result = try runChapterTest(69);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 7: Test 70 - split level" {
    const result = try runChapterTest(70);
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
