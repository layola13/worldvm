//! Chapter 15: 载具系统 Tests 141-150
//! 汽车、直升机、船的行驶与碰撞

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 141, .name = "car_flat", .setup_fn = setupCarFlat },
    .{ .id = 142, .name = "car_slope", .setup_fn = setupCarSlope },
    .{ .id = 143, .name = "car_crash", .setup_fn = setupCarCrash },
    .{ .id = 144, .name = "car_recovery", .setup_fn = setupCarRecovery },
    .{ .id = 145, .name = "helicopter_hover", .setup_fn = setupHelicopterHover },
    .{ .id = 146, .name = "helicopter_crash", .setup_fn = setupHelicopterCrash },
    .{ .id = 147, .name = "boat_water", .setup_fn = setupBoatWater },
    .{ .id = 148, .name = "boat_wake", .setup_fn = setupBoatWake },
    .{ .id = 149, .name = "tank_tread", .setup_fn = setupTankTread },
    .{ .id = 150, .name = "vehicle_rollover", .setup_fn = setupVehicleRollover },
};

fn setupCarFlat(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(0, 10, 5, 15, .idle);     // Car proxy
    return 2;
}

fn setupCarSlope(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground base
    inst[1] = makeInstance(7, 5, 1, 15, .idle);     // Slope
    inst[2] = makeInstance(7, 10, 5, 15, .idle);    // Slope continue
    inst[3] = makeInstance(0, 15, 9, 15, .idle);    // Car proxy
    return 4;
}

fn setupCarCrash(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(7, 20, 1, 15, .idle);    // Wall
    inst[2] = makeInstance(0, 5, 5, 15, .idle);      // Car proxy
    return 3;
}

fn setupCarRecovery(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(7, 10, 3, 15, .idle);    // Pit wall
    inst[2] = makeInstance(0, 10, 10, 15, .idle);   // Car in pit
    return 3;
}

fn setupHelicopterHover(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Helicopter proxy
    return 2;
}

fn setupHelicopterCrash(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(7, 10, 1, 15, .idle);      // Building
    inst[2] = makeInstance(0, 10, 30, 15, .idle);    // Helicopter
    return 3;
}

fn setupBoatWater(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);    // Water
    inst[2] = makeInstance(0, 10, 5, 15, .idle);    // Boat proxy
    return 3;
}

fn setupBoatWake(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Seabed
    inst[1] = makeInstance(4, 10, 2, 15, .idle);    // Water
    inst[2] = makeInstance(0, 5, 5, 15, .idle);     // Boat moving
    inst[3] = makeInstance(0, 15, 5, 15, .idle);    // Boat following
    return 4;
}

fn setupTankTread(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(7, 5, 1, 10, .idle);      // Obstacle 1
    inst[2] = makeInstance(7, 15, 1, 15, .idle);    // Obstacle 2
    inst[3] = makeInstance(0, 10, 5, 15, .idle);     // Tank proxy
    return 4;
}

fn setupVehicleRollover(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(7, 8, 1, 15, .idle);       // Ramp 1
    inst[2] = makeInstance(7, 12, 1, 15, .idle);      // Ramp 2
    inst[3] = makeInstance(0, 10, 10, 15, .idle);    // Vehicle proxy
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

test "Chapter 15: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 15: Test 141 - car flat" {
    const result = try runChapterTest(141);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 142 - car slope" {
    const result = try runChapterTest(142);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 143 - car crash" {
    const result = try runChapterTest(143);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 144 - car recovery" {
    const result = try runChapterTest(144);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 145 - helicopter hover" {
    const result = try runChapterTest(145);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 146 - helicopter crash" {
    const result = try runChapterTest(146);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 147 - boat water" {
    const result = try runChapterTest(147);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 148 - boat wake" {
    const result = try runChapterTest(148);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 149 - tank tread" {
    const result = try runChapterTest(149);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 15: Test 150 - vehicle rollover" {
    const result = try runChapterTest(150);
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
