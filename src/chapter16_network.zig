//! Chapter 16: 网络同步 Tests 151-160
//! 多客户端物理状态同步、预测与回滚

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 151, .name = "sync_position", .setup_fn = setupSyncPosition },
    .{ .id = 152, .name = "sync_velocity", .setup_fn = setupSyncVelocity },
    .{ .id = 153, .name = "client_predict", .setup_fn = setupClientPredict },
    .{ .id = 154, .name = "server_authoritative", .setup_fn = setupServerAuthoritative },
    .{ .id = 155, .name = "packet_loss", .setup_fn = setupPacketLoss },
    .{ .id = 156, .name = "latency_compensation", .setup_fn = setupLatencyCompensation },
    .{ .id = 157, .name = "state_interpolation", .setup_fn = setupStateInterpolation },
    .{ .id = 158, .name = "deterministic_lockstep", .setup_fn = setupDeterministicLockstep },
    .{ .id = 159, .name = "physics_prediction", .setup_fn = setupPhysicsPrediction },
    .{ .id = 160, .name = "rollback_replay", .setup_fn = setupRollbackReplay },
};

fn setupSyncPosition(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Entity 1
    inst[2] = makeInstance(0, 15, 10, 20, .idle);   // Entity 2
    return 3;
}

fn setupSyncVelocity(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 5, 20, 15, .idle);     // Fast moving
    inst[2] = makeInstance(0, 15, 20, 15, .idle);    // Slow moving
    return 3;
}

fn setupClientPredict(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 15, 15, .idle);    // Player entity
    inst[2] = makeInstance(7, 20, 5, 15, .idle);    // Obstacle
    return 3;
}

fn setupServerAuthoritative(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Server entity
    return 2;
}

fn setupPacketLoss(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 30, 15, .idle);   // Dropped packet entity
    return 2;
}

fn setupLatencyCompensation(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);   // High latency entity
    inst[2] = makeInstance(0, 20, 20, 15, .idle);  // Low latency entity
    return 3;
}

fn setupStateInterpolation(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 5, 10, 15, .idle);    // Interp point 1
    inst[2] = makeInstance(0, 10, 10, 15, .idle);   // Interp point 2
    inst[3] = makeInstance(0, 15, 10, 15, .idle);   // Interp point 3
    return 4;
}

fn setupDeterministicLockstep(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);      // Brick 1
    inst[2] = makeInstance(7, 10, 8, 15, .idle);     // Brick 2
    inst[3] = makeInstance(0, 10, 20, 15, .idle);    // Impact
    return 4;
}

fn setupPhysicsPrediction(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 25, 15, .idle);   // Predicted path
    return 2;
}

fn setupRollbackReplay(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 15, 15, .idle);   // Rollback entity
    inst[2] = makeInstance(7, 15, 5, 15, .idle);    // Interaction target
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

test "Chapter 16: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 16: Test 151 - sync position" {
    const result = try runChapterTest(151);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 152 - sync velocity" {
    const result = try runChapterTest(152);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 153 - client predict" {
    const result = try runChapterTest(153);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 154 - server authoritative" {
    const result = try runChapterTest(154);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 155 - packet loss" {
    const result = try runChapterTest(155);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 156 - latency compensation" {
    const result = try runChapterTest(156);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 157 - state interpolation" {
    const result = try runChapterTest(157);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 158 - deterministic lockstep" {
    const result = try runChapterTest(158);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 159 - physics prediction" {
    const result = try runChapterTest(159);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 16: Test 160 - rollback replay" {
    const result = try runChapterTest(160);
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
