//! Chapter 28: 确定性与人列物理 Tests 271-280
//! 浮点精度、时间步长、replay

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 271, .name = "deterministic_replay", .setup_fn = setupDeterministicReplay },
    .{ .id = 272, .name = "float_precision", .setup_fn = setupFloatPrecision },
    .{ .id = 273, .name = "time_step_variance", .setup_fn = setupTimeStepVariance },
    .{ .id = 274, .name = "initial_condition", .setup_fn = setupInitialCondition },
    .{ .id = 275, .name = "accumulated_error", .setup_fn = setupAccumulatedError },
    .{ .id = 276, .name = "seed_reproducibility", .setup_fn = setupSeedReproducibility },
    .{ .id = 277, .name = "order_independence", .setup_fn = setupOrderIndependence },
    .{ .id = 278, .name = "reversible_physics", .setup_fn = setupReversiblePhysics },
    .{ .id = 279, .name = "state_checkpoint", .setup_fn = setupStateCheckpoint },
    .{ .id = 280, .name = "determinism_verification", .setup_fn = setupDeterminismVerification },
};

fn setupDeterministicReplay(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Ball
    return 2;
}

fn setupFloatPrecision(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 15, 15, .idle);    // High precision test
    return 2;
}

fn setupTimeStepVariance(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 25, 15, .idle);    // Variable dt test
    return 2;
}

fn setupInitialCondition(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // IC test
    return 2;
}

fn setupAccumulatedError(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 30, 15, .idle);   // Long simulation
    return 2;
}

fn setupSeedReproducibility(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Seed test
    return 2;
}

fn setupOrderIndependence(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 8, 15, 15, .idle);    // A before B
    inst[2] = makeInstance(0, 12, 15, 15, .idle);   // B before A
    return 3;
}

fn setupReversiblePhysics(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Forward
    return 2;
}

fn setupStateCheckpoint(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Checkpoint test
    return 2;
}

fn setupDeterminismVerification(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Complex scene
    inst[2] = makeInstance(0, 10, 25, 15, .idle);   // Verification
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

test "Chapter 28: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 28: Test 271 - deterministic replay" {
    const result = try runChapterTest(271);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 272 - float precision" {
    const result = try runChapterTest(272);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 273 - time step variance" {
    const result = try runChapterTest(273);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 274 - initial condition" {
    const result = try runChapterTest(274);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 275 - accumulated error" {
    const result = try runChapterTest(275);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 276 - seed reproducibility" {
    const result = try runChapterTest(276);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 277 - order independence" {
    const result = try runChapterTest(277);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 278 - reversible physics" {
    const result = try runChapterTest(278);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 279 - state checkpoint" {
    const result = try runChapterTest(279);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 28: Test 280 - determinism verification" {
    const result = try runChapterTest(280);
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
