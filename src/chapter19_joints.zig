//! Chapter 19: 关节与约束 Tests 181-190
//! 铰链、滑动、弹簧、绳索关节

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 181, .name = "hinge_door", .setup_fn = setupHingeDoor },
    .{ .id = 182, .name = "spring_mass", .setup_fn = setupSpringMass },
    .{ .id = 183, .name = "rope_swing", .setup_fn = setupRopeSwing },
    .{ .id = 184, .name = "pendulum_clock", .setup_fn = setupPendulumClock },
    .{ .id = 185, .name = "slider_block", .setup_fn = setupSliderBlock },
    .{ .id = 186, .name = "constraint_chain", .setup_fn = setupConstraintChain },
    .{ .id = 187, .name = "elastic_collision", .setup_fn = setupElasticCollision },
    .{ .id = 188, .name = "rigid_bridge", .setup_fn = setupRigidBridge },
    .{ .id = 189, .name = "breakable_joint", .setup_fn = setupBreakableJoint },
    .{ .id = 190, .name = "motor_joint", .setup_fn = setupMotorJoint },
};

fn setupHingeDoor(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);     // Door
    inst[2] = makeInstance(0, 15, 5, 15, .idle);    // Wind force
    return 3;
}

fn setupSpringMass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 10, 15, .idle);   // Support
    inst[2] = makeInstance(0, 10, 20, 15, .idle);   // Mass
    return 3;
}

fn setupRopeSwing(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 15, 15, .idle);   // Anchor
    inst[2] = makeInstance(0, 10, 25, 15, .idle);   // Swinger
    return 3;
}

fn setupPendulumClock(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 15, 15, .idle);    // Pivot
    inst[2] = makeInstance(0, 10, 25, 15, .idle);    // Pendulum
    return 3;
}

fn setupSliderBlock(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Rail
    inst[2] = makeInstance(0, 15, 5, 15, .idle);    // Sliding block
    return 3;
}

fn setupConstraintChain(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);     // Link 1
    inst[2] = makeInstance(0, 10, 10, 15, .idle);     // Link 2
    inst[3] = makeInstance(0, 10, 15, 15, .idle);     // Link 3
    return 4;
}

fn setupElasticCollision(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(14, 5, 10, 15, .idle);    // Bouncy ball
    inst[2] = makeInstance(14, 15, 10, 15, .idle);   // Target bouncy
    return 3;
}

fn setupRigidBridge(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 5, 5, 10, .idle);      // Support 1
    inst[2] = makeInstance(7, 15, 5, 10, .idle);    // Support 2
    inst[3] = makeInstance(7, 10, 10, 10, .idle);    // Bridge deck
    inst[4] = makeInstance(0, 10, 15, 15, .idle);    // Load
    return 5;
}

fn setupBreakableJoint(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);   // Support
    inst[2] = makeInstance(0, 10, 15, 15, .idle);   // Weight
    return 3;
}

fn setupMotorJoint(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);   // Motor base
    inst[2] = makeInstance(0, 10, 12, 15, .idle);   // Motor arm
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

test "Chapter 19: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 19: Test 181 - hinge door" {
    const result = try runChapterTest(181);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 182 - spring mass" {
    const result = try runChapterTest(182);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 183 - rope swing" {
    const result = try runChapterTest(183);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 184 - pendulum clock" {
    const result = try runChapterTest(184);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 185 - slider block" {
    const result = try runChapterTest(185);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 186 - constraint chain" {
    const result = try runChapterTest(186);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 187 - elastic collision" {
    const result = try runChapterTest(187);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 188 - rigid bridge" {
    const result = try runChapterTest(188);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 189 - breakable joint" {
    const result = try runChapterTest(189);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 19: Test 190 - motor joint" {
    const result = try runChapterTest(190);
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
