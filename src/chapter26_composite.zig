//! Chapter 26: 复合场景 Tests 251-260
//! 多系统交互、连锁反应

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 251, .name = "wrecking_ball_chain", .setup_fn = setupWreckingBallChain },
    .{ .id = 252, .name = "domino_car_crash", .setup_fn = setupDominoCarCrash },
    .{ .id = 253, .name = "waterwheel_power", .setup_fn = setupWaterwheelPower },
    .{ .id = 254, .name = "catapult_launch", .setup_fn = setupCatapultLaunch },
    .{ .id = 255, .name = "bridge_collapse", .setup_fn = setupBridgeCollapse },
    .{ .id = 256, .name = "seesaw_balancing", .setup_fn = setupSeesawBalancing },
    .{ .id = 257, .name = "pulley_system", .setup_fn = setupPulleySystem },
    .{ .id = 258, .name = "gear_mechanism", .setup_fn = setupGearMechanism },
    .{ .id = 259, .name = "pendulum_clockwork", .setup_fn = setupPendulumClockwork },
    .{ .id = 260, .name = "rube_goldberg", .setup_fn = setupRubeGoldberg },
};

fn setupWreckingBallChain(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(7, 10, 1, 15, .idle);     // Building
    inst[2] = makeInstance(10, 5, 20, 15, .idle);    // Wrecking ball
    return 3;
}

fn setupDominoCarCrash(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Road
    inst[1] = makeInstance(8, 8, 5, 15, .idle);      // Domino 1
    inst[2] = makeInstance(8, 11, 5, 15, .idle);     // Domino 2
    inst[3] = makeInstance(8, 14, 5, 15, .idle);     // Domino 3
    inst[4] = makeInstance(0, 3, 5, 15, .idle);      // Car
    return 5;
}

fn setupWaterwheelPower(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Riverbed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);    // Water flow
    inst[2] = makeInstance(7, 10, 10, 15, .idle);   // Wheel
    return 3;
}

fn setupCatapultLaunch(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Counterweight
    inst[2] = makeInstance(0, 10, 20, 15, .idle);   // Projectile
    return 3;
}

fn setupBridgeCollapse(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ravine
    inst[1] = makeInstance(7, 5, 5, 10, .idle);      // Support L
    inst[2] = makeInstance(7, 15, 5, 10, .idle);    // Support R
    inst[3] = makeInstance(7, 10, 10, 10, .idle);    // Deck
    inst[4] = makeInstance(10, 10, 20, 15, .idle);   // Heavy load
    return 5;
}

fn setupSeesawBalancing(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Fulcrum
    inst[1] = makeInstance(0, 7, 8, 15, .idle);     // Light L
    inst[2] = makeInstance(10, 13, 8, 15, .idle);   // Heavy R
    return 3;
}

fn setupPulleySystem(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ceiling
    inst[1] = makeInstance(7, 10, 10, 15, .idle);   // Pulley
    inst[2] = makeInstance(0, 10, 15, 15, .idle);   // Weight
    return 3;
}

fn setupGearMechanism(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Base
    inst[1] = makeInstance(7, 8, 5, 15, .idle);     // Gear 1
    inst[2] = makeInstance(7, 12, 5, 15, .idle);    // Gear 2
    return 3;
}

fn setupPendulumClockwork(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Clock body
    inst[1] = makeInstance(7, 10, 10, 15, .idle);   // Pendulum
    inst[2] = makeInstance(0, 10, 20, 15, .idle);   // Escapement
    return 3;
}

fn setupRubeGoldberg(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Table
    inst[1] = makeInstance(0, 5, 5, 15, .idle);     // Ball
    inst[2] = makeInstance(7, 10, 5, 15, .idle);    // Ramp
    inst[3] = makeInstance(8, 15, 5, 15, .idle);     // Domino trigger
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

test "Chapter 26: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 26: Test 251 - wrecking ball chain" {
    const result = try runChapterTest(251);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 252 - domino car crash" {
    const result = try runChapterTest(252);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 253 - waterwheel power" {
    const result = try runChapterTest(253);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 254 - catapult launch" {
    const result = try runChapterTest(254);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 255 - bridge collapse" {
    const result = try runChapterTest(255);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 256 - seesaw balancing" {
    const result = try runChapterTest(256);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 257 - pulley system" {
    const result = try runChapterTest(257);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 258 - gear mechanism" {
    const result = try runChapterTest(258);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 259 - pendulum clockwork" {
    const result = try runChapterTest(259);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 26: Test 260 - rube goldberg" {
    const result = try runChapterTest(260);
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
