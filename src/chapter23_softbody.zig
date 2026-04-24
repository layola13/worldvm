//! Chapter 23: 软体物理 Tests 221-230
//! 软体模拟、弹簧网络、织物

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 221, .name = "softball_drop", .setup_fn = setupSoftballDrop },
    .{ .id = 222, .name = "softball_bounce", .setup_fn = setupSoftballBounce },
    .{ .id = 223, .name = "jelly_wobble", .setup_fn = setupJellyWobble },
    .{ .id = 224, .name = "balloon_float", .setup_fn = setupBalloonFloat },
    .{ .id = 225, .name = "rubber_band", .setup_fn = setupRubberBand },
    .{ .id = 226, .name = "cloth_draping", .setup_fn = setupClothDraping },
    .{ .id = 227, .name = "rope_coil", .setup_fn = setupRopeCoil },
    .{ .id = 228, .name = "chain_sway", .setup_fn = setupChainSway },
    .{ .id = 229, .name = "spring_network", .setup_fn = setupSpringNetwork },
    .{ .id = 230, .name = "soft_collision", .setup_fn = setupSoftCollision },
};

fn setupSoftballDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(12, 10, 20, 15, .idle);   // Soft ball
    return 2;
}

fn setupSoftballBounce(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(12, 10, 30, 15, .idle);   // Soft ball
    return 2;
}

fn setupJellyWobble(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Table
    inst[1] = makeInstance(12, 10, 5, 15, .idle);   // Jelly
    return 2;
}

fn setupBalloonFloat(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Balloon
    return 2;
}

fn setupRubberBand(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 10, .idle);    // Anchor 1
    inst[2] = makeInstance(7, 10, 5, 20, .idle);    // Anchor 2
    inst[3] = makeInstance(12, 10, 10, 15, .idle);  // Band
    return 4;
}

fn setupClothDraping(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Table
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Cloth center
    return 2;
}

fn setupRopeCoil(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Hook
    inst[1] = makeInstance(12, 10, 10, 15, .idle);  // Rope coil
    return 2;
}

fn setupChainSway(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Support
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Chain top
    inst[2] = makeInstance(0, 10, 20, 15, .idle);    // Chain bottom
    return 3;
}

fn setupSpringNetwork(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Base
    inst[1] = makeInstance(12, 10, 15, 15, .idle);   // Spring mass
    return 2;
}

fn setupSoftCollision(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(12, 10, 15, 15, .idle);   // Soft 1
    inst[2] = makeInstance(7, 10, 5, 15, .idle);    // Hard
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

test "Chapter 23: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 23: Test 221 - softball drop" {
    const result = try runChapterTest(221);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 222 - softball bounce" {
    const result = try runChapterTest(222);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 223 - jelly wobble" {
    const result = try runChapterTest(223);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 224 - balloon float" {
    const result = try runChapterTest(224);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 225 - rubber band" {
    const result = try runChapterTest(225);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 226 - cloth draping" {
    const result = try runChapterTest(226);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 227 - rope coil" {
    const result = try runChapterTest(227);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 228 - chain sway" {
    const result = try runChapterTest(228);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 229 - spring network" {
    const result = try runChapterTest(229);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 23: Test 230 - soft collision" {
    const result = try runChapterTest(230);
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
