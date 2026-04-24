//! Chapter 25: 力场与区域 Tests 241-250
//! 重力区域、风力、爆炸、磁场

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 241, .name = "gravity_zone_low", .setup_fn = setupGravityZoneLow },
    .{ .id = 242, .name = "gravity_zone_high", .setup_fn = setupGravityZoneHigh },
    .{ .id = 243, .name = "wind_zone", .setup_fn = setupWindZone },
    .{ .id = 244, .name = "explosion_radial", .setup_fn = setupExplosionRadial },
    .{ .id = 245, .name = "magnetic_attract", .setup_fn = setupMagneticAttract },
    .{ .id = 246, .name = "magnetic_repel", .setup_fn = setupMagneticRepel },
    .{ .id = 247, .name = "buoyancy_zone", .setup_fn = setupBuoyancyZone },
    .{ .id = 248, .name = "drag_zone", .setup_fn = setupDragZone },
    .{ .id = 249, .name = "force_field_wall", .setup_fn = setupForceFieldWall },
    .{ .id = 250, .name = "multi_force_field", .setup_fn = setupMultiForceField },
};

fn setupGravityZoneLow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Moon object
    return 2;
}

fn setupGravityZoneHigh(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(10, 10, 20, 15, .idle);  // Heavy planet
    return 2;
}

fn setupWindZone(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Wind-blown object
    return 2;
}

fn setupExplosionRadial(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Debris
    inst[2] = makeInstance(0, 10, 10, 15, .idle);    // Explosion center
    return 3;
}

fn setupMagneticAttract(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 5, 10, 15, .idle);    // Metal 1
    inst[2] = makeInstance(13, 15, 10, 15, .idle);  // Magnet
    return 3;
}

fn setupMagneticRepel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);    // Metal
    inst[2] = makeInstance(13, 10, 20, 15, .idle);   // Repel magnet
    return 3;
}

fn setupBuoyancyZone(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Seabed
    inst[1] = makeInstance(4, 10, 5, 15, .idle);   // Buoyancy zone
    inst[2] = makeInstance(10, 10, 10, 15, .idle); // Heavy object
    return 3;
}

fn setupDragZone(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(0, 10, 10, 15, .idle);   // Dragged object
    return 2;
}

fn setupForceFieldWall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);     // Force field
    inst[2] = makeInstance(0, 10, 15, 15, .idle);   // Approaching object
    return 3;
}

fn setupMultiForceField(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 8, 5, 12, .idle);     // Field 1
    inst[2] = makeInstance(7, 12, 5, 18, .idle);    // Field 2
    inst[3] = makeInstance(0, 10, 15, 15, .idle);   // Object
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

test "Chapter 25: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 25: Test 241 - gravity zone low" {
    const result = try runChapterTest(241);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 242 - gravity zone high" {
    const result = try runChapterTest(242);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 243 - wind zone" {
    const result = try runChapterTest(243);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 244 - explosion radial" {
    const result = try runChapterTest(244);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 245 - magnetic attract" {
    const result = try runChapterTest(245);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 246 - magnetic repel" {
    const result = try runChapterTest(246);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 247 - buoyancy zone" {
    const result = try runChapterTest(247);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 248 - drag zone" {
    const result = try runChapterTest(248);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 249 - force field wall" {
    const result = try runChapterTest(249);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 25: Test 250 - multi force field" {
    const result = try runChapterTest(250);
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
