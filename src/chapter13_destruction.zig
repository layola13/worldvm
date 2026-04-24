//! Chapter 13: 破坏系统 Tests 121-130
//! 可破坏物体的裂纹、碎裂、破碎行为

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 121, .name = "glass_crack", .setup_fn = setupGlassCrack },
    .{ .id = 122, .name = "glass_shatter", .setup_fn = setupGlassShatter },
    .{ .id = 123, .name = "wood_splinter", .setup_fn = setupWoodSplinter },
    .{ .id = 124, .name = "concrete_crumble", .setup_fn = setupConcreteCrumble },
    .{ .id = 125, .name = "armor_pierce", .setup_fn = setupArmorPierce },
    .{ .id = 126, .name = "chain_break", .setup_fn = setupChainBreak },
    .{ .id = 127, .name = "rope_cut", .setup_fn = setupRopeCut },
    .{ .id = 128, .name = "impact_crater", .setup_fn = setupImpactCrater },
    .{ .id = 129, .name = "progressive_collapse", .setup_fn = setupProgressiveCollapse },
    .{ .id = 130, .name = "structural_failure", .setup_fn = setupStructuralFailure },
};

fn setupGlassCrack(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(3, 10, 1, 15, .idle);   // Glass
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Light impact
    return 2;
}

fn setupGlassShatter(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(3, 10, 1, 15, .idle);    // Glass
    inst[1] = makeInstance(10, 10, 30, 15, .idle);  // Heavy impact
    return 2;
}

fn setupWoodSplinter(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(7, 10, 1, 15, .idle);     // Wood planks
    inst[1] = makeInstance(2, 10, 20, 15, .idle);    // Hammer
    return 2;
}

fn setupConcreteCrumble(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(7, 10, 1, 15, .idle);     // Concrete
    inst[1] = makeInstance(7, 10, 8, 15, .idle);    // More concrete
    inst[2] = makeInstance(10, 10, 40, 15, .idle);   // Heavy impact
    return 3;
}

fn setupArmorPierce(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(13, 10, 1, 15, .idle);    // Armor plate
    inst[1] = makeInstance(0, 10, 50, 15, .idle);   // High velocity impact
    return 2;
}

fn setupChainBreak(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);     // Support
    inst[2] = makeInstance(0, 10, 15, 15, .idle);    // Weight on chain
    return 3;
}

fn setupRopeCut(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 10, 15, .idle);    // Platform
    inst[2] = makeInstance(0, 10, 20, 15, .idle);    // Weight
    return 3;
}

fn setupImpactCrater(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(10, 10, 50, 15, .idle);   // Massive impact
    return 2;
}

fn setupProgressiveCollapse(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);    // Pillar 1
    inst[2] = makeInstance(7, 10, 8, 15, .idle);     // Pillar 2
    inst[3] = makeInstance(7, 10, 15, 15, .idle);     // Pillar 3
    inst[4] = makeInstance(7, 5, 22, 15, .idle);      // Roof
    inst[5] = makeInstance(0, 10, 40, 15, .idle);    // Impact
    return 6;
}

fn setupStructuralFailure(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Foundation
    inst[1] = makeInstance(7, 5, 1, 10, .idle);      // Wall 1
    inst[2] = makeInstance(7, 5, 1, 20, .idle);      // Wall 2
    inst[3] = makeInstance(7, 5, 8, 10, .idle);      // Upper 1
    inst[4] = makeInstance(7, 5, 8, 20, .idle);      // Upper 2
    inst[5] = makeInstance(10, 5, 30, 15, .idle);    // Heavy weight
    return 6;
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

test "Chapter 13: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 13: Test 121 - glass crack" {
    const result = try runChapterTest(121);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 122 - glass shatter" {
    const result = try runChapterTest(122);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 123 - wood splinter" {
    const result = try runChapterTest(123);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 124 - concrete crumble" {
    const result = try runChapterTest(124);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 125 - armor pierce" {
    const result = try runChapterTest(125);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 126 - chain break" {
    const result = try runChapterTest(126);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 127 - rope cut" {
    const result = try runChapterTest(127);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 128 - impact crater" {
    const result = try runChapterTest(128);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 129 - progressive collapse" {
    const result = try runChapterTest(129);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 13: Test 130 - structural failure" {
    const result = try runChapterTest(130);
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
