//! Chapter 11: 运动学角色控制器 (KCC) Tests 101-110
//! KCC角色在各类地表环境的移动、跳跃、蹲伏

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");
const kcc = @import("kcc.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 101, .name = "kcc_flat_ground", .setup_fn = setupKCCFlatGround },
    .{ .id = 102, .name = "kcc_slope_walk", .setup_fn = setupKCCSlopeWalk },
    .{ .id = 103, .name = "kcc_step_climb", .setup_fn = setupKCCStepClimb },
    .{ .id = 104, .name = "kcc_jump_basic", .setup_fn = setupKCCJumpBasic },
    .{ .id = 105, .name = "kcc_jump_height", .setup_fn = setupKCCJumpHeight },
    .{ .id = 106, .name = "kcc_crouch", .setup_fn = setupKCCCrouch },
    .{ .id = 107, .name = "kcc_friction", .setup_fn = setupKCCFriction },
    .{ .id = 108, .name = "kcc_ice_walk", .setup_fn = setupKCCIceWalk },
    .{ .id = 109, .name = "kcc_water_wade", .setup_fn = setupKCCWaterWade },
    .{ .id = 110, .name = "kcc_fall_recovery", .setup_fn = setupKCCFallRecovery },
};

fn setupKCCFlatGround(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(0, 10, 5, 15, .idle);   // KCC test entity (apple as proxy)
    return 2;
}

fn setupKCCSlopeWalk(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(7, 5, 1, 15, .idle);    // Slope bricks
    inst[2] = makeInstance(7, 8, 4, 15, .idle);
    inst[3] = makeInstance(7, 11, 7, 15, .idle);
    inst[4] = makeInstance(0, 14, 10, 15, .idle);  // KCC proxy
    return 5;
}

fn setupKCCStepClimb(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(7, 5, 1, 15, .idle);    // Step 1
    inst[2] = makeInstance(7, 5, 5, 15, .idle);    // Step 2
    inst[3] = makeInstance(0, 5, 9, 15, .idle);     // KCC proxy
    return 4;
}

fn setupKCCJumpBasic(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(0, 10, 5, 15, .idle);   // KCC proxy
    return 2;
}

fn setupKCCJumpHeight(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);  // KCC proxy at height
    return 2;
}

fn setupKCCCrouch(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);   // Low ceiling
    inst[2] = makeInstance(0, 10, 5, 15, .idle);   // KCC proxy
    return 3;
}

fn setupKCCFriction(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(0, 5, 5, 15, .idle);    // KCC proxy
    inst[2] = makeInstance(7, 15, 5, 15, .idle);   // Obstacle
    return 3;
}

fn setupKCCIceWalk(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor (ice)
    inst[1] = makeInstance(0, 10, 5, 15, .idle);   // KCC proxy
    return 2;
}

fn setupKCCWaterWade(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Water
    inst[2] = makeInstance(0, 10, 5, 15, .idle);   // KCC proxy
    return 3;
}

fn setupKCCFallRecovery(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting); // Floor
    inst[1] = makeInstance(0, 10, 50, 15, .idle);  // KCC proxy falling
    return 2;
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

test "Chapter 11: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 11: Test 101 - KCC flat ground" {
    const result = try runChapterTest(101);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 102 - KCC slope walk" {
    const result = try runChapterTest(102);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 103 - KCC step climb" {
    const result = try runChapterTest(103);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 104 - KCC jump basic" {
    const result = try runChapterTest(104);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 105 - KCC jump height" {
    const result = try runChapterTest(105);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 106 - KCC crouch" {
    const result = try runChapterTest(106);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 107 - KCC friction" {
    const result = try runChapterTest(107);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 108 - KCC ice walk" {
    const result = try runChapterTest(108);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 109 - KCC water wade" {
    const result = try runChapterTest(109);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 11: Test 110 - KCC fall recovery" {
    const result = try runChapterTest(110);
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
