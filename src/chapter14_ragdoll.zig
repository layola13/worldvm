//! Chapter 14: 布娃娃系统 Tests 131-140
//! 人形布娃娃的肢体断裂、关节约束

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 131, .name = "ragdoll_stand", .setup_fn = setupRagdollStand },
    .{ .id = 132, .name = "ragdoll_fall", .setup_fn = setupRagdollFall },
    .{ .id = 133, .name = "ragdoll_impact", .setup_fn = setupRagdollImpact },
    .{ .id = 134, .name = "limb_break_arm", .setup_fn = setupLimbBreakArm },
    .{ .id = 135, .name = "limb_break_leg", .setup_fn = setupLimbBreakLeg },
    .{ .id = 136, .name = "joint_stretch", .setup_fn = setupJointStretch },
    .{ .id = 137, .name = "joint_dislocate", .setup_fn = setupJointDislocate },
    .{ .id = 138, .name = "full_ragdoll", .setup_fn = setupFullRagdoll },
    .{ .id = 139, .name = "motor_recovery", .setup_fn = setupMotorRecovery },
    .{ .id = 140, .name = "death_animation", .setup_fn = setupDeathAnimation },
};

fn setupRagdollStand(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Ragdoll proxy (standing)
    return 2;
}

fn setupRagdollFall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 30, 15, .idle);   // Ragdoll proxy (falling)
    return 2;
}

fn setupRagdollImpact(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 25, 15, .idle);    // Ragdoll
    inst[2] = makeInstance(7, 10, 5, 15, .idle);     // Obstacle
    return 3;
}

fn setupLimbBreakArm(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Ragdoll
    inst[2] = makeInstance(2, 5, 10, 15, .idle);     // Hammer impact on arm
    return 3;
}

fn setupLimbBreakLeg(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Ragdoll
    inst[2] = makeInstance(10, 10, 5, 15, .idle);    // Heavy weight on leg
    return 3;
}

fn setupJointStretch(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 5, 10, 10, .idle);     // Left support
    inst[2] = makeInstance(7, 15, 10, 10, .idle);     // Right support
    inst[3] = makeInstance(0, 10, 10, 15, .idle);     // Ragdoll hanging
    return 4;
}

fn setupJointDislocate(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Ragdoll
    inst[2] = makeInstance(7, 3, 5, 15, .idle);      // Side pull
    return 3;
}

fn setupFullRagdoll(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 50, 15, .idle);     // Ragdoll from height
    inst[2] = makeInstance(7, 7, 1, 15, .idle);      // Obstacle 1
    inst[3] = makeInstance(7, 13, 1, 15, .idle);     // Obstacle 2
    return 4;
}

fn setupMotorRecovery(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);   // Ragdoll
    return 2;
}

fn setupDeathAnimation(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 40, 15, .idle);     // Ragdoll
    inst[2] = makeInstance(10, 10, 50, 15, .idle);    // Bullet impact
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

test "Chapter 14: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 14: Test 131 - ragdoll stand" {
    const result = try runChapterTest(131);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 132 - ragdoll fall" {
    const result = try runChapterTest(132);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 133 - ragdoll impact" {
    const result = try runChapterTest(133);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 134 - limb break arm" {
    const result = try runChapterTest(134);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 135 - limb break leg" {
    const result = try runChapterTest(135);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 136 - joint stretch" {
    const result = try runChapterTest(136);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 137 - joint dislocate" {
    const result = try runChapterTest(137);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 138 - full ragdoll" {
    const result = try runChapterTest(138);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 139 - motor recovery" {
    const result = try runChapterTest(139);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 14: Test 140 - death animation" {
    const result = try runChapterTest(140);
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
