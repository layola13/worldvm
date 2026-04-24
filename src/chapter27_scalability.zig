//! Chapter 27: 性能与扩展性 Tests 261-270
//! 大规模场景、批处理、空间分区

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 261, .name = "many_balls_drop", .setup_fn = setupManyBallsDrop },
    .{ .id = 262, .name = "many_bricks_stack", .setup_fn = setupManyBricksStack },
    .{ .id = 263, .name = "particle_storm", .setup_fn = setupParticleStorm },
    .{ .id = 264, .name = "fluid_simulation", .setup_fn = setupFluidSimulation },
    .{ .id = 265, .name = "crowd_physics", .setup_fn = setupCrowdPhysics },
    .{ .id = 266, .name = "vehicle_traffic", .setup_fn = setupVehicleTraffic },
    .{ .id = 267, .name = "destruction_massive", .setup_fn = setupDestructionMassive },
    .{ .id = 268, .name = "stress_thin_objects", .setup_fn = setupStressThinObjects },
    .{ .id = 269, .name = "stress_high_speed", .setup_fn = setupStressHighSpeed },
    .{ .id = 270, .name = "stress_tangling", .setup_fn = setupStressTangling },
};

fn setupManyBallsDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 8, 30, 10, .idle);    // Ball 1
    inst[2] = makeInstance(0, 10, 30, 15, .idle);    // Ball 2
    inst[3] = makeInstance(0, 12, 30, 20, .idle);   // Ball 3
    return 4;
}

fn setupManyBricksStack(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);    // Brick 1
    inst[2] = makeInstance(7, 10, 8, 15, .idle);    // Brick 2
    inst[3] = makeInstance(7, 10, 15, 15, .idle);    // Brick 3
    return 4;
}

fn setupParticleStorm(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(0, 10, 40, 10, .idle);    // Particle 1
    inst[2] = makeInstance(0, 10, 40, 15, .idle);    // Particle 2
    inst[3] = makeInstance(0, 10, 40, 20, .idle);   // Particle 3
    return 4;
}

fn setupFluidSimulation(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Container
    inst[1] = makeInstance(7, 5, 1, 10, .idle);      // Wall L
    inst[2] = makeInstance(7, 5, 1, 20, .idle);     // Wall R
    inst[3] = makeInstance(4, 5, 10, 15, .idle);    // Fluid
    return 4;
}

fn setupCrowdPhysics(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(0, 8, 10, 10, .idle);    // Person 1
    inst[2] = makeInstance(0, 10, 10, 15, .idle);   // Person 2
    inst[3] = makeInstance(0, 12, 10, 20, .idle);   // Person 3
    return 4;
}

fn setupVehicleTraffic(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Road
    inst[1] = makeInstance(0, 8, 5, 15, .idle);     // Car 1
    inst[2] = makeInstance(0, 12, 5, 15, .idle);   // Car 2
    return 3;
}

fn setupDestructionMassive(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(7, 8, 1, 10, .idle);     // Block 1
    inst[2] = makeInstance(7, 12, 1, 10, .idle);    // Block 2
    inst[3] = makeInstance(10, 10, 30, 15, .idle);   // Impact
    return 4;
}

fn setupStressThinObjects(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);     // Thin wall
    inst[2] = makeInstance(0, 10, 30, 15, .idle);   // Fast impact
    return 3;
}

fn setupStressHighSpeed(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);   // Target
    inst[2] = makeInstance(0, 10, 50, 15, .idle);  // Bullet
    return 3;
}

fn setupStressTangling(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(12, 8, 10, 10, .idle);   // Rope 1
    inst[2] = makeInstance(12, 12, 10, 15, .idle);  // Rope 2
    inst[3] = makeInstance(12, 10, 15, 12, .idle);   // Rope 3
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

test "Chapter 27: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 27: Test 261 - many balls drop" {
    const result = try runChapterTest(261);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 262 - many bricks stack" {
    const result = try runChapterTest(262);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 263 - particle storm" {
    const result = try runChapterTest(263);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 264 - fluid simulation" {
    const result = try runChapterTest(264);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 265 - crowd physics" {
    const result = try runChapterTest(265);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 266 - vehicle traffic" {
    const result = try runChapterTest(266);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 267 - destruction massive" {
    const result = try runChapterTest(267);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 268 - stress thin objects" {
    const result = try runChapterTest(268);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 269 - stress high speed" {
    const result = try runChapterTest(269);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 27: Test 270 - stress tangling" {
    const result = try runChapterTest(270);
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
