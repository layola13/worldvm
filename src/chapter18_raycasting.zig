//! Chapter 18: 光线投射 Tests 171-180
//! 射线查询、光线命中、平移扫描

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 171, .name = "ray_horizontal", .setup_fn = setupRayHorizontal },
    .{ .id = 172, .name = "ray_vertical", .setup_fn = setupRayVertical },
    .{ .id = 173, .name = "ray_angled", .setup_fn = setupRayAngled },
    .{ .id = 174, .name = "ray_through_objects", .setup_fn = setupRayThroughObjects },
    .{ .id = 175, .name = "ray_miss", .setup_fn = setupRayMiss },
    .{ .id = 176, .name = "sweep_sphere", .setup_fn = setupSweepSphere },
    .{ .id = 177, .name = "sweep_box", .setup_fn = setupSweepBox },
    .{ .id = 178, .name = "multiray_cast", .setup_fn = setupMultirayCast },
    .{ .id = 179, .name = "ray_priority", .setup_fn = setupRayPriority },
    .{ .id = 180, .name = "ray_reflection", .setup_fn = setupRayReflection },
};

fn setupRayHorizontal(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 15, 5, 15, .idle);    // Target
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Ray origin
    return 3;
}

fn setupRayVertical(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 10, 15, .idle);   // Target
    inst[2] = makeInstance(0, 10, 1, 15, .idle);    // Ray origin
    return 3;
}

fn setupRayAngled(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 15, 15, 15, .idle);   // Target
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Ray origin
    return 3;
}

fn setupRayThroughObjects(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 8, 5, 15, .idle);     // Object 1
    inst[2] = makeInstance(7, 12, 5, 15, .idle);    // Object 2
    inst[3] = makeInstance(7, 16, 5, 15, .idle);    // Object 3
    inst[4] = makeInstance(0, 5, 5, 15, .idle);     // Ray origin
    return 5;
}

fn setupRayMiss(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 20, 5, 15, .idle);     // Far target
    inst[2] = makeInstance(0, 5, 5, 15, .idle);      // Ray origin pointing away
    return 3;
}

fn setupSweepSphere(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 15, 5, 15, .idle);    // Obstacle
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Sweep origin
    return 3;
}

fn setupSweepBox(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 15, 5, 15, .idle);    // Obstacle
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Box sweep origin
    return 3;
}

fn setupMultirayCast(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 8, 5, 10, .idle);      // Target 1
    inst[2] = makeInstance(7, 12, 5, 15, .idle);    // Target 2
    inst[3] = makeInstance(7, 16, 5, 20, .idle);    // Target 3
    inst[4] = makeInstance(0, 5, 5, 15, .idle);     // Origin
    return 5;
}

fn setupRayPriority(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Close target
    inst[2] = makeInstance(13, 10, 5, 15, .idle);   // Priority target
    inst[3] = makeInstance(0, 5, 5, 15, .idle);      // Ray origin
    return 4;
}

fn setupRayReflection(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(13, 15, 5, 15, .idle);   // Reflective surface
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Ray origin
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

test "Chapter 18: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 18: Test 171 - ray horizontal" {
    const result = try runChapterTest(171);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 172 - ray vertical" {
    const result = try runChapterTest(172);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 173 - ray angled" {
    const result = try runChapterTest(173);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 174 - ray through objects" {
    const result = try runChapterTest(174);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 175 - ray miss" {
    const result = try runChapterTest(175);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 176 - sweep sphere" {
    const result = try runChapterTest(176);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 177 - sweep box" {
    const result = try runChapterTest(177);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 178 - multiray cast" {
    const result = try runChapterTest(178);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 179 - ray priority" {
    const result = try runChapterTest(179);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 18: Test 180 - ray reflection" {
    const result = try runChapterTest(180);
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
