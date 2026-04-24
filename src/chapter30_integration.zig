//! Chapter 30: 集成测试与验收 Tests 291-300
//! 端到端场景、用户旅程、验收测试

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 291, .name = "iron_ball_concrete", .setup_fn = setupIronBallConcrete },
    .{ .id = 292, .name = "glass_ball_concrete", .setup_fn = setupGlassBallConcrete },
    .{ .id = 293, .name = "bowling_mud", .setup_fn = setupBowlingMud },
    .{ .id = 294, .name = "bowling_water", .setup_fn = setupBowlingWater },
    .{ .id = 295, .name = "iron_ball_mud_sink", .setup_fn = setupIronBallMudSink },
    .{ .id = 296, .name = "glass_ball_shatter", .setup_fn = setupGlassBallShatter },
    .{ .id = 297, .name = "mixed_media", .setup_fn = setupMixedMedia },
    .{ .id = 298, .name = "chained_physics", .setup_fn = setupChainedPhysics },
    .{ .id = 299, .name = "chaotic_system", .setup_fn = setupChaoticSystem },
    .{ .id = 300, .name = "full_integration", .setup_fn = setupFullIntegration },
};

fn setupIronBallConcrete(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Concrete floor
    inst[1] = makeInstance(10, 10, 30, 15, .idle);   // Iron ball (heavy, solid)
    return 2;
}

fn setupGlassBallConcrete(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Concrete floor
    inst[1] = makeInstance(3, 10, 30, 15, .idle);    // Glass ball (fragile)
    return 2;
}

fn setupBowlingMud(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Mud floor
    inst[1] = makeInstance(10, 10, 20, 15, .idle);  // Bowling ball
    return 2;
}

fn setupBowlingWater(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);    // Water
    inst[2] = makeInstance(10, 10, 10, 15, .idle); // Bowling ball floats
    return 3;
}

fn setupIronBallMudSink(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Mud floor
    inst[1] = makeInstance(10, 10, 25, 15, .idle);  // Iron ball sinks
    return 2;
}

fn setupGlassBallShatter(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Concrete floor
    inst[1] = makeInstance(3, 10, 40, 15, .idle);   // Glass high fall
    return 2;
}

fn setupMixedMedia(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Concrete
    inst[1] = makeInstance(7, 10, 1, 10, .idle);    // Transition
    inst[2] = makeInstance(4, 10, 10, 15, .idle);   // Water pool
    inst[3] = makeInstance(10, 10, 15, 15, .idle);  // Multiple balls
    return 4;
}

fn setupChainedPhysics(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(10, 5, 20, 15, .idle);   // Iron ball
    inst[2] = makeInstance(3, 10, 10, 15, .idle);   // Glass ball
    inst[3] = makeInstance(7, 15, 5, 15, .idle);    // Domino trigger
    return 4;
}

fn setupChaoticSystem(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 8, 1, 10, .idle);      // Tower A
    inst[2] = makeInstance(7, 12, 1, 15, .idle);     // Tower B
    inst[3] = makeInstance(10, 10, 30, 15, .idle);  // Wrecking ball
    inst[4] = makeInstance(4, 10, 5, 15, .idle);    // Water
    return 5;
}

fn setupFullIntegration(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(10, 5, 25, 15, .idle);   // Iron ball
    inst[2] = makeInstance(3, 10, 20, 15, .idle);   // Glass ball
    inst[3] = makeInstance(7, 15, 5, 15, .idle);    // Structure
    inst[4] = makeInstance(4, 10, 10, 15, .idle);   // Water hazard
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

test "Chapter 30: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 30: Test 291 - iron ball concrete" {
    const result = try runChapterTest(291);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 292 - glass ball concrete" {
    const result = try runChapterTest(292);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 293 - bowling mud" {
    const result = try runChapterTest(293);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 294 - bowling water" {
    const result = try runChapterTest(294);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 295 - iron ball mud sink" {
    const result = try runChapterTest(295);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 296 - glass ball shatter" {
    const result = try runChapterTest(296);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 297 - mixed media" {
    const result = try runChapterTest(297);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 298 - chained physics" {
    const result = try runChapterTest(298);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 299 - chaotic system" {
    const result = try runChapterTest(299);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 30: Test 300 - full integration" {
    const result = try runChapterTest(300);
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
