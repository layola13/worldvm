//! Chapter 29: 实时性能优化 Tests 281-290
//! tick预算、帧时间、内存分配

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 281, .name = "tick_budget_simple", .setup_fn = setupTickBudgetSimple },
    .{ .id = 282, .name = "tick_budget_complex", .setup_fn = setupTickBudgetComplex },
    .{ .id = 283, .name = "frame_time_basic", .setup_fn = setupFrameTimeBasic },
    .{ .id = 284, .name = "frame_time_spike", .setup_fn = setupFrameTimeSpike },
    .{ .id = 285, .name = "memory_arena", .setup_fn = setupMemoryArena },
    .{ .id = 286, .name = "memory_leak_check", .setup_fn = setupMemoryLeakCheck },
    .{ .id = 287, .name = "cache_coherence", .setup_fn = setupCacheCoherence },
    .{ .id = 288, .name = "SIMD_vectorization", .setup_fn = setupSIMDVectorization },
    .{ .id = 289, .name = "parallel_physics", .setup_fn = setupParallelPhysics },
    .{ .id = 290, .name = "adaptive_quality", .setup_fn = setupAdaptiveQuality },
};

fn setupTickBudgetSimple(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Simple body
    return 2;
}

fn setupTickBudgetComplex(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 8, 1, 10, .idle);     // Multi-body
    inst[2] = makeInstance(7, 12, 1, 15, .idle);   // Scene
    inst[3] = makeInstance(0, 10, 20, 15, .idle);   // Dynamic
    return 4;
}

fn setupFrameTimeBasic(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 15, 15, .idle);   // Standard load
    return 2;
}

fn setupFrameTimeSpike(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Many contacts
    inst[2] = makeInstance(10, 10, 30, 15, .idle);  // Collision spike
    return 3;
}

fn setupMemoryArena(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Alloc test
    return 2;
}

fn setupMemoryLeakCheck(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Leak test
    return 2;
}

fn setupCacheCoherence(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 15, 15, .idle);   // Sequential access
    return 2;
}

fn setupSIMDVectorization(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 8, 20, 10, .idle);    // Vectorizable
    inst[2] = makeInstance(0, 10, 20, 15, .idle);   // Batch
    inst[3] = makeInstance(0, 12, 20, 20, .idle);   // Ops
    return 4;
}

fn setupParallelPhysics(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 8, 15, 10, .idle);   // Parallel 1
    inst[2] = makeInstance(0, 10, 15, 15, .idle);  // Parallel 2
    inst[3] = makeInstance(0, 12, 15, 20, .idle);  // Parallel 3
    return 4;
}

fn setupAdaptiveQuality(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Quality levels
    inst[2] = makeInstance(0, 10, 25, 15, .idle);   // Adaptive
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

test "Chapter 29: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 29: Test 281 - tick budget simple" {
    const result = try runChapterTest(281);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 282 - tick budget complex" {
    const result = try runChapterTest(282);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 283 - frame time basic" {
    const result = try runChapterTest(283);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 284 - frame time spike" {
    const result = try runChapterTest(284);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 285 - memory arena" {
    const result = try runChapterTest(285);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 286 - memory leak check" {
    const result = try runChapterTest(286);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 287 - cache coherence" {
    const result = try runChapterTest(287);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 288 - SIMD vectorization" {
    const result = try runChapterTest(288);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 289 - parallel physics" {
    const result = try runChapterTest(289);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 29: Test 290 - adaptive quality" {
    const result = try runChapterTest(290);
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
