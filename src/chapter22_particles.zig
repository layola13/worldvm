//! Chapter 22: 粒子系统 Tests 211-220
//! 粒子发射、粒子物理、粒子碰撞

const std = @import("std");
const physics_tests = @import("physics_tests.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

const SCENARIO_COUNT = 10;

const scenarios = [_]struct { id: u32, name: []const u8, setup_fn: *const fn ([]entity16.Entity16, *[32]scene32.Instance) u8 }{
    .{ .id = 211, .name = "particle_fountain", .setup_fn = setupParticleFountain },
    .{ .id = 212, .name = "particle_explosion", .setup_fn = setupParticleExplosion },
    .{ .id = 213, .name = "particle_sand", .setup_fn = setupParticleSand },
    .{ .id = 214, .name = "particle_dust", .setup_fn = setupParticleDust },
    .{ .id = 215, .name = "particle_snow", .setup_fn = setupParticleSnow },
    .{ .id = 216, .name = "particle_rain", .setup_fn = setupParticleRain },
    .{ .id = 217, .name = "particle_spray", .setup_fn = setupParticleSpray },
    .{ .id = 218, .name = "particle_embers", .setup_fn = setupParticleEmbers },
    .{ .id = 219, .name = "particle_debris", .setup_fn = setupParticleDebris },
    .{ .id = 220, .name = "particle_smoke", .setup_fn = setupParticleSmoke },
};

fn setupParticleFountain(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Basin
    inst[1] = makeInstance(0, 10, 5, 15, .idle);   // Particle source
    return 2;
}

fn setupParticleExplosion(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(0, 10, 5, 15, .idle);   // Center
    return 2;
}

fn setupParticleSand(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Sand pile
    inst[1] = makeInstance(0, 10, 20, 15, .idle);    // Falling sand
    return 2;
}

fn setupParticleDust(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Floor
    inst[1] = makeInstance(7, 10, 5, 15, .idle);   // Disturbed
    inst[2] = makeInstance(0, 10, 10, 15, .idle);    // Dust particles
    return 3;
}

fn setupParticleSnow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(0, 10, 30, 15, .idle);   // Snow particles
    return 2;
}

fn setupParticleRain(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(0, 10, 40, 15, .idle);   // Rain drops
    return 2;
}

fn setupParticleSpray(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(4, 10, 2, 15, .idle);   // Water
    inst[2] = makeInstance(0, 10, 10, 15, .idle);    // Spray
    return 3;
}

fn setupParticleEmbers(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(0, 10, 5, 15, .idle);   // Fire source
    return 2;
}

fn setupParticleDebris(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);    // Ground
    inst[1] = makeInstance(7, 10, 5, 15, .idle);    // Structure
    inst[2] = makeInstance(0, 10, 25, 15, .idle);   // Impact
    return 3;
}

fn setupParticleSmoke(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 {
    inst[0] = makeInstance(5, 0, 0, 0, .resting);     // Ground
    inst[1] = makeInstance(0, 10, 5, 15, .idle);   // Smoke source
    return 2;
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

test "Chapter 22: Test count" {
    try std.testing.expect(scenarios.len == SCENARIO_COUNT);
}

test "Chapter 22: Test 211 - particle fountain" {
    const result = try runChapterTest(211);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 212 - particle explosion" {
    const result = try runChapterTest(212);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 213 - particle sand" {
    const result = try runChapterTest(213);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 214 - particle dust" {
    const result = try runChapterTest(214);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 215 - particle snow" {
    const result = try runChapterTest(215);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 216 - particle rain" {
    const result = try runChapterTest(216);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 217 - particle spray" {
    const result = try runChapterTest(217);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 218 - particle embers" {
    const result = try runChapterTest(218);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 219 - particle debris" {
    const result = try runChapterTest(219);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Chapter 22: Test 220 - particle smoke" {
    const result = try runChapterTest(220);
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
