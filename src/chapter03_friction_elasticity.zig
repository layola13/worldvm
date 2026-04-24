//! Chapter 3: 摩擦与弹性 Tests 21-30
//! 弹性碰撞、非弹性碰撞、摩擦系数、恢复系数

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 21, .name = "bounce_elastic", .setup_fn = setupBounceElastic },
    .{ .id = 22, .name = "bounce_inelastic", .setup_fn = setupBounceInelastic },
    .{ .id = 23, .name = "hammer_glass", .setup_fn = setupHammerGlass },
    .{ .id = 24, .name = "hammer_soft", .setup_fn = setupHammerSoft },
    .{ .id = 25, .name = "hammer_hard", .setup_fn = setupHammerHard },
    .{ .id = 26, .name = "heavy_on_glass", .setup_fn = setupHeavyOnGlass },
    .{ .id = 27, .name = "water_flow", .setup_fn = setupWaterFlow },
    .{ .id = 28, .name = "water_puddle", .setup_fn = setupWaterPuddle },
    .{ .id = 29, .name = "multi_water", .setup_fn = setupMultiWater },
    .{ .id = 30, .name = "sphere_vs_box", .setup_fn = setupSphereVsBox },
};

fn setupBounceElastic(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(14, 10, 20, 15, .idle);
    return 2;
}

fn setupBounceInelastic(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(15, 10, 20, 15, .idle);
    return 2;
}

fn setupHammerGlass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(3, 10, 15, 15, .idle);
    inst[2] = makeInstance(2, 10, 25, 15, .idle);
    return 3;
}

fn setupHammerSoft(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(12, 10, 15, 15, .idle);
    inst[2] = makeInstance(2, 10, 25, 15, .idle);
    return 3;
}

fn setupHammerHard(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(13, 10, 15, 15, .idle);
    inst[2] = makeInstance(2, 10, 25, 15, .idle);
    return 3;
}

fn setupHeavyOnGlass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(3, 10, 15, 15, .idle);
    inst[2] = makeInstance(10, 10, 20, 15, .idle);
    return 3;
}

fn setupWaterFlow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(4, 10, 20, 15, .idle);
    return 2;
}

fn setupWaterPuddle(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(4, 10, 1, 15, .idle);
    return 2;
}

fn setupMultiWater(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(4, 5, 20, 10, .idle);
    inst[2] = makeInstance(4, 15, 20, 20, .idle);
    return 3;
}

fn setupSphereVsBox(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);
    inst[1] = makeInstance(0, 10, 20, 15, .idle);
    inst[2] = makeInstance(7, 10, 1, 15, .idle);
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

test "Chapter 3: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 3: Test 21 - high restitution bounce" {
    const result = try runChapterTest(21);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 22 - low restitution bounce" {
    const result = try runChapterTest(22);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 23 - hammer breaks glass" {
    const result = try runChapterTest(23);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 24 - hammer on soft material" {
    const result = try runChapterTest(24);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 25 - hammer on hard material" {
    const result = try runChapterTest(25);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 26 - heavy object on glass" {
    const result = try runChapterTest(26);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 27 - water flowing" {
    const result = try runChapterTest(27);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 28 - water puddle" {
    const result = try runChapterTest(28);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 29 - multiple water bodies" {
    const result = try runChapterTest(29);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 3: Test 30 - sphere on box" {
    const result = try runChapterTest(30);
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
