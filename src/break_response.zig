//! Shared break/destruction response helpers.

const std = @import("std");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const destruction = @import("destruction.zig");

pub fn calcImpactMagnitude(vel: i16, mass: u16) u16 {
    const abs_vel: u32 = @intCast(@abs(@as(i32, vel)));
    const impact: u32 = (abs_vel * @as(u32, mass)) / 100;
    return @truncate(@min(impact, 65535));
}

pub fn applyBreakState(inst: *scene32.Instance, entity: *const entity16.Entity16, impact_velocity: i16, speed_scale: f32) bool {
    if (inst.state == .broken) return false;

    const impact = calcImpactMagnitude(impact_velocity, entity.physics.mass);
    const break_result = physics.checkBreak(impact, entity.physics.material, entity.physics.hardness);
    if (!break_result.did_break) return false;

    const fragment_count = @max(@as(u8, 1), break_result.fragments);
    const debris_mass = @as(f32, @floatFromInt(entity.physics.mass)) / @as(f32, @floatFromInt(fragment_count));
    const debris_speed = @as(f32, @floatFromInt(@abs(@as(i32, impact_velocity))));
    _ = destruction.spawnDebris(
        @floatFromInt(inst.pos_x),
        @floatFromInt(inst.pos_y),
        @floatFromInt(inst.pos_z),
        0,
        0,
        0,
        debris_speed * speed_scale,
        @max(20.0, debris_speed * speed_scale * 2.0),
        0.0,
        1.0,
        debris_mass,
    );

    inst.state = .broken;
    inst.vel_x = 0;
    inst.vel_y = 0;
    inst.vel_z = 0;
    inst.ang_x = 0;
    inst.ang_y = 0;
    inst.ang_z = 0;
    inst.sleep_tick = 0;
    return true;
}

test "calcImpactMagnitude uses absolute velocity and saturates" {
    try std.testing.expectEqual(@as(u16, 0), calcImpactMagnitude(0, 100));
    try std.testing.expectEqual(@as(u16, 50), calcImpactMagnitude(-50, 100));
    try std.testing.expectEqual(@as(u16, 50), calcImpactMagnitude(50, 100));
    try std.testing.expectEqual(@as(u16, 65535), calcImpactMagnitude(-32768, 65535));
}

test "applyBreakState breaks fragile instance and clears motion" {
    destruction.initDebris();

    var entity = entity16.initEntity16();
    entity.physics.mass = 100;
    entity.physics.material = .fragile;
    entity.physics.hardness = 255;

    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 0;
    inst.pos_x = 1;
    inst.pos_y = 2;
    inst.pos_z = 3;
    inst.state = .falling;
    inst.vel_x = 10;
    inst.vel_y = -60;
    inst.vel_z = 5;
    inst.ang_x = 1;
    inst.ang_y = 2;
    inst.ang_z = 3;
    inst.sleep_tick = 7;

    try std.testing.expect(applyBreakState(&inst, &entity, -60, 1.0));
    try std.testing.expectEqual(scene32.InstanceState.broken, inst.state);
    try std.testing.expectEqual(@as(i16, 0), inst.vel_x);
    try std.testing.expectEqual(@as(i16, 0), inst.vel_y);
    try std.testing.expectEqual(@as(i16, 0), inst.vel_z);
    try std.testing.expectEqual(@as(i8, 0), inst.ang_x);
    try std.testing.expectEqual(@as(i8, 0), inst.ang_y);
    try std.testing.expectEqual(@as(i8, 0), inst.ang_z);
    try std.testing.expectEqual(@as(u8, 0), inst.sleep_tick);
    try std.testing.expectEqual(@as(u16, 1), destruction.getDebrisSystem().count);

    try std.testing.expect(!applyBreakState(&inst, &entity, -60, 1.0));
    try std.testing.expectEqual(@as(u16, 1), destruction.getDebrisSystem().count);
}

test "applyBreakState leaves insufficient impact unchanged" {
    destruction.initDebris();

    var entity = entity16.initEntity16();
    entity.physics.mass = 10;
    entity.physics.material = .solid;
    entity.physics.hardness = 500;

    var inst = std.mem.zeroes(scene32.Instance);
    inst.state = .falling;
    inst.vel_y = -10;

    try std.testing.expect(!applyBreakState(&inst, &entity, -10, 1.0));
    try std.testing.expectEqual(scene32.InstanceState.falling, inst.state);
    try std.testing.expectEqual(@as(i16, -10), inst.vel_y);
    try std.testing.expectEqual(@as(u16, 0), destruction.getDebrisSystem().count);
}
