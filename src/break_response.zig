//! Shared break/destruction response helpers.

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
