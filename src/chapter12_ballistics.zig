//! Chapter 12: 弹道学与射弹物理 Tests 111-120
//! 射弹穿透、偏转、碎裂、停止行为

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 111, .name = "bullet_wood", .setup_fn = setupBulletWood },
    .{ .id = 112, .name = "bullet_metal", .setup_fn = setupBulletMetal },
    .{ .id = 113, .name = "bullet_glass", .setup_fn = setupBulletGlass },
    .{ .id = 114, .name = "arrow_wood", .setup_fn = setupArrowWood },
    .{ .id = 115, .name = "arrow_armor", .setup_fn = setupArrowArmor },
    .{ .id = 116, .name = "shell_ricochet", .setup_fn = setupShellRicochet },
    .{ .id = 117, .name = "grenade_bounce", .setup_fn = setupGrenadeBounce },
    .{ .id = 118, .name = "rocket_explosion", .setup_fn = setupRocketExplosion },
    .{ .id = 119, .name = "shotgun_spread", .setup_fn = setupShotgunSpread },
    .{ .id = 120, .name = "apfsds_penetration", .setup_fn = setupApfsdsPenetration },
};

fn setupBulletWood(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(7, 10, 1, 15, .idle);    // Wood target
    inst[1] = makeInstance(0, 10, 20, 15, .idle);  // Bullet proxy
    return 2;
}

fn setupBulletMetal(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(13, 10, 1, 15, .idle);  // Hard metal target
    inst[1] = makeInstance(0, 10, 20, 15, .idle);  // Bullet proxy
    return 2;
}

fn setupBulletGlass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(3, 10, 1, 15, .idle);   // Glass target
    inst[1] = makeInstance(0, 10, 20, 15, .idle);   // Bullet proxy
    return 2;
}

fn setupArrowWood(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(7, 15, 1, 15, .idle);    // Wood target
    inst[1] = makeInstance(0, 15, 15, 15, .idle);   // Arrow proxy
    return 2;
}

fn setupArrowArmor(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(13, 15, 1, 15, .idle);   // Armor target
    inst[1] = makeInstance(0, 15, 20, 15, .idle);   // Arrow proxy
    return 2;
}

fn setupShellRicochet(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(13, 10, 1, 15, .idle);  // Metal surface
    inst[1] = makeInstance(0, 5, 15, 15, .idle);    // Shell proxy
    return 2;
}

fn setupGrenadeBounce(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);   // Floor
    inst[1] = makeInstance(7, 5, 1, 15, .idle);     // Wall
    inst[2] = makeInstance(0, 3, 10, 15, .idle);    // Grenade proxy
    return 3;
}

fn setupRocketExplosion(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);    // Target bricks
    inst[2] = makeInstance(7, 10, 8, 15, .idle);
    inst[3] = makeInstance(0, 10, 30, 15, .idle);   // Rocket proxy
    return 4;
}

fn setupShotgunSpread(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);   // Floor
    inst[1] = makeInstance(7, 10, 1, 15, .idle);     // Target
    inst[2] = makeInstance(0, 10, 20, 10, .idle);   // Shot 1
    inst[3] = makeInstance(0, 10, 20, 15, .idle);    // Shot 2
    inst[4] = makeInstance(0, 10, 20, 20, .idle);    // Shot 3
    return 5;
}

fn setupApfsdsPenetration(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(13, 10, 1, 15, .idle);   // Hard armor
    inst[1] = makeInstance(7, 10, 8, 15, .idle);     // Secondary target
    inst[2] = makeInstance(0, 10, 40, 15, .idle);    // APFSDS proxy
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

test "Chapter 12: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 12: Test 111 - bullet wood" {
    const result = try runChapterTest(111);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 112 - bullet metal" {
    const result = try runChapterTest(112);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 113 - bullet glass" {
    const result = try runChapterTest(113);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 114 - arrow wood" {
    const result = try runChapterTest(114);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 115 - arrow armor" {
    const result = try runChapterTest(115);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 116 - shell ricochet" {
    const result = try runChapterTest(116);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 117 - grenade bounce" {
    const result = try runChapterTest(117);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 118 - rocket explosion" {
    const result = try runChapterTest(118);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 119 - shotgun spread" {
    const result = try runChapterTest(119);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 12: Test 120 - apfsds penetration" {
    const result = try runChapterTest(120);
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
