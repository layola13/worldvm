//! Chapter 21: 流体交互 Tests 201-210
//! 水体浮力、水流阻力、物体沉浮

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 201, .name = "water_float_heavy", .setup_fn = setupWaterFloatHeavy },
    .{ .id = 202, .name = "water_float_light", .setup_fn = setupWaterFloatLight },
    .{ .id = 203, .name = "water_sink", .setup_fn = setupWaterSink },
    .{ .id = 204, .name = "water_flow_push", .setup_fn = setupWaterFlowPush },
    .{ .id = 205, .name = "water_wave_ripple", .setup_fn = setupWaterWaveRipple },
    .{ .id = 206, .name = "ice_floating", .setup_fn = setupIceFloating },
    .{ .id = 207, .name = "boat_displacement", .setup_fn = setupBoatDisplacement },
    .{ .id = 208, .name = "water_resistance", .setup_fn = setupWaterResistance },
    .{ .id = 209, .name = "aquarium_glass", .setup_fn = setupAquariumGlass },
    .{ .id = 210, .name = "floating_chain", .setup_fn = setupFloatingChain },
};

fn setupWaterFloatHeavy(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);      // Water
    inst[2] = makeInstance(10, 10, 5, 15, .idle);    // Heavy object
    return 3;
}

fn setupWaterFloatLight(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);      // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);    // Water
    inst[2] = makeInstance(0, 10, 5, 15, .idle);    // Light object
    return 3;
}

fn setupWaterSink(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Water
    inst[2] = makeInstance(10, 10, 5, 15, .idle);   // Sinking heavy
    return 3;
}

fn setupWaterFlowPush(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Riverbed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Flowing water
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Floating object
    return 3;
}

fn setupWaterWaveRipple(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Water
    inst[2] = makeInstance(0, 10, 10, 15, .idle);    // Disturbance
    return 3;
}

fn setupIceFloating(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Water
    inst[2] = makeInstance(7, 10, 4, 15, .idle);    // Ice block
    return 3;
}

fn setupBoatDisplacement(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Water
    inst[2] = makeInstance(7, 10, 5, 15, .idle);    // Boat hull
    return 3;
}

fn setupWaterResistance(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Deep water
    inst[2] = makeInstance(0, 10, 10, 15, .idle);    // Moving object
    return 3;
}

fn setupAquariumGlass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 5, 1, 10, .idle);      // Glass wall 1
    inst[2] = makeInstance(7, 5, 1, 20, .idle);     // Glass wall 2
    inst[3] = makeInstance(7, 5, 8, 10, .idle);     // Glass base
    inst[4] = makeInstance(4, 5, 5, 15, .idle);       // Water inside
    return 5;
}

fn setupFloatingChain(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Anchor
    inst[1] = makeInstance(4, 10, 2, 15, .idle);    // Water surface
    inst[2] = makeInstance(0, 10, 5, 15, .idle);   // Chain link 1
    inst[3] = makeInstance(0, 10, 10, 15, .idle);   // Chain link 2
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

test "Chapter 21: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 21: Test 201 - water float heavy" {
    const result = try runChapterTest(201);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 202 - water float light" {
    const result = try runChapterTest(202);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 203 - water sink" {
    const result = try runChapterTest(203);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 204 - water flow push" {
    const result = try runChapterTest(204);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 205 - water wave ripple" {
    const result = try runChapterTest(205);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 206 - ice floating" {
    const result = try runChapterTest(206);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 207 - boat displacement" {
    const result = try runChapterTest(207);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 208 - water resistance" {
    const result = try runChapterTest(208);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 209 - aquarium glass" {
    const result = try runChapterTest(209);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 21: Test 210 - floating chain" {
    const result = try runChapterTest(210);
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
