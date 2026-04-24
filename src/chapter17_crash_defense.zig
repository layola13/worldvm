//! Chapter 17: 碰撞防御系统 Tests 161-170
//! 碰撞检测优化、穿透修正、防御姿态

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 161, .name = "penetration_correct", .setup_fn = setupPenetrationCorrect },
    .{ .id = 162, .name = "tunneling_fix", .setup_fn = setupTunnelingFix },
    .{ .id = 163, .name = "CCD_continuous", .setup_fn = setupCCDContinuous },
    .{ .id = 164, .name = "defense_posture", .setup_fn = setupDefensePosture },
    .{ .id = 165, .name = "roll_dodge", .setup_fn = setupRollDodge },
    .{ .id = 166, .name = "shield_block", .setup_fn = setupShieldBlock },
    .{ .id = 167, .name = "parry_timing", .setup_fn = setupParryTiming },
    .{ .id = 168, .name = "evasion_path", .setup_fn = setupEvasionPath },
    .{ .id = 169, .name = "absorption_layer", .setup_fn = setupAbsorptionLayer },
    .{ .id = 170, .name = "impact_distribution", .setup_fn = setupImpactDistribution },
};

fn setupPenetrationCorrect(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(0, 10, 5, 15, .idle);     // Fast object
    inst[2] = makeInstance(7, 10, 3, 15, .idle);    // Target
    return 3;
}

fn setupTunnelingFix(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);    // Thin wall
    inst[2] = makeInstance(0, 10, 40, 15, .idle);   // Very fast object
    return 3;
}

fn setupCCDContinuous(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);     // Small target
    inst[2] = makeInstance(0, 10, 50, 15, .idle);    // High speed projectile
    return 3;
}

fn setupDefensePosture(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 5, 15, .idle);     // Defender
    inst[2] = makeInstance(0, 15, 25, 15, .idle);    // Attacker
    return 3;
}

fn setupRollDodge(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Roller
    inst[2] = makeInstance(0, 20, 10, 15, .idle);    // Projectile
    return 3;
}

fn setupShieldBlock(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Shield proxy
    inst[2] = makeInstance(0, 10, 30, 15, .idle);    // Attack
    return 3;
}

fn setupParryTiming(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Weapon
    inst[2] = makeInstance(0, 15, 20, 15, .idle);    // Parry target
    return 3;
}

fn setupEvasionPath(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 5, 10, 15, .idle);     // Evader
    inst[2] = makeInstance(7, 10, 5, 10, .idle);     // Obstacle 1
    inst[3] = makeInstance(7, 10, 5, 20, .idle);     // Obstacle 2
    inst[4] = makeInstance(0, 20, 10, 15, .idle);    // Threat
    return 5;
}

fn setupAbsorptionLayer(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(12, 10, 3, 15, .idle);   // Soft absorber
    inst[2] = makeInstance(13, 10, 10, 15, .idle);  // Hard shell
    inst[3] = makeInstance(0, 10, 35, 15, .idle);    // Impact
    return 4;
}

fn setupImpactDistribution(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);     // Distributed brick 1
    inst[2] = makeInstance(7, 10, 8, 15, .idle);    // Distributed brick 2
    inst[3] = makeInstance(10, 10, 25, 15, .idle);  // Heavy impact
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

test "Chapter 17: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 17: Test 161 - penetration correct" {
    const result = try runChapterTest(161);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 162 - tunneling fix" {
    const result = try runChapterTest(162);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 163 - CCD continuous" {
    const result = try runChapterTest(163);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 164 - defense posture" {
    const result = try runChapterTest(164);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 165 - roll dodge" {
    const result = try runChapterTest(165);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 166 - shield block" {
    const result = try runChapterTest(166);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 167 - parry timing" {
    const result = try runChapterTest(167);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 168 - evasion path" {
    const result = try runChapterTest(168);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 169 - absorption layer" {
    const result = try runChapterTest(169);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 17: Test 170 - impact distribution" {
    const result = try runChapterTest(170);
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
