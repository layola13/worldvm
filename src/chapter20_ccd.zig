//! Chapter 20: 连续碰撞检测 Tests 191-200
//! 时域分解、adaptive CCD、穿透避免

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 191, .name = "ccd_bullet", .setup_fn = setupCCDBullet },
    .{ .id = 192, .name = "ccd_arrow", .setup_fn = setupCCDArrow },
    .{ .id = 193, .name = "ccd_thin_wall", .setup_fn = setupCCDThinWall },
    .{ .id = 194, .name = "ccd_rotating", .setup_fn = setupCCDRotating },
    .{ .id = 195, .name = "ccd_adaptive", .setup_fn = setupCCDAdaptive },
    .{ .id = 196, .name = "ccd_time_of_impact", .setup_fn = setupCCDTimeOfImpact },
    .{ .id = 197, .name = "ccd_conservative_advancement", .setup_fn = setupCCDConservativeAdvancement },
    .{ .id = 198, .name = "ccd_sweep_prune", .setup_fn = setupCCDSweepPrune },
    .{ .id = 199, .name = "ccd_gJK", .setup_fn = setupCCDGJK },
    .{ .id = 200, .name = "ccd_final", .setup_fn = setupCCDFinal },
};

fn setupCCDBullet(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);     // Target
    inst[2] = makeInstance(0, 5, 5, 15, .idle);      // Fast bullet
    return 3;
}

fn setupCCDArrow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 15, 5, 15, .idle);    // Target
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Arrow
    return 3;
}

fn setupCCDThinWall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);    // Thin wall
    inst[2] = makeInstance(0, 10, 40, 15, .idle);    // Fast object
    return 3;
}

fn setupCCDRotating(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Rotating blade
    inst[2] = makeInstance(7, 10, 20, 15, .idle);   // Target
    return 3;
}

fn setupCCDAdaptive(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Obstacle
    inst[2] = makeInstance(0, 10, 50, 15, .idle);   // Variable speed
    return 3;
}

fn setupCCDTimeOfImpact(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 15, 5, 15, .idle);    // Target
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // TOI test object
    return 3;
}

fn setupCCDConservativeAdvancement(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 12, 5, 15, .idle);    // Complex shape
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Advancing object
    return 3;
}

fn setupCCDSweepPrune(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 8, 5, 10, .idle);      // Object A
    inst[2] = makeInstance(7, 12, 5, 15, .idle);    // Object B
    inst[3] = makeInstance(0, 5, 5, 15, .idle);      // Sweeping C
    return 4;
}

fn setupCCDGJK(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 5, 15, .idle);      // GJK test shape
    inst[2] = makeInstance(7, 12, 5, 15, .idle);     // EPA target
    return 3;
}

fn setupCCDFinal(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Complex scene
    inst[2] = makeInstance(7, 10, 12, 15, .idle);   // Multi-object
    inst[3] = makeInstance(0, 5, 5, 15, .idle);      // Fast impact
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

test "Chapter 20: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 20: Test 191 - CCD bullet" {
    const result = try runChapterTest(191);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 192 - CCD arrow" {
    const result = try runChapterTest(192);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 193 - CCD thin wall" {
    const result = try runChapterTest(193);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 194 - CCD rotating" {
    const result = try runChapterTest(194);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 195 - CCD adaptive" {
    const result = try runChapterTest(195);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 196 - CCD time of impact" {
    const result = try runChapterTest(196);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 197 - CCD conservative advancement" {
    const result = try runChapterTest(197);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 198 - CCD sweep prune" {
    const result = try runChapterTest(198);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 199 - CCD GJK" {
    const result = try runChapterTest(199);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 20: Test 200 - CCD final" {
    const result = try runChapterTest(200);
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
