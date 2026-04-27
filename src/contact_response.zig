//! Shared contact response helpers for world/tick integration paths.

const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const terrain = @import("terrain.zig");
const material_pairing = @import("material_pairing.zig");
const sleep_response = @import("sleep_response.zig");

pub const ContactAxis = enum { x, z };

pub const GROUND_SETTLE_VERTICAL_THRESHOLD: i16 = sleep_response.GROUND_SETTLE_VERTICAL_THRESHOLD;
pub const GROUND_SETTLE_LATERAL_THRESHOLD: i16 = sleep_response.GROUND_SETTLE_LATERAL_THRESHOLD;
pub const SLEEP_TIME_THRESHOLD: u8 = sleep_response.SLEEP_TIME_THRESHOLD;

pub fn settleGroundContact(inst: *scene32.Instance) void {
    if (@abs(inst.vel_y) <= GROUND_SETTLE_VERTICAL_THRESHOLD) {
        inst.vel_y = 0;
    }
    if (@abs(inst.vel_x) <= GROUND_SETTLE_LATERAL_THRESHOLD) {
        inst.vel_x = 0;
    }
    if (@abs(inst.vel_z) <= GROUND_SETTLE_LATERAL_THRESHOLD) {
        inst.vel_z = 0;
    }

    if (inst.vel_x == 0 and inst.vel_y == 0 and inst.vel_z == 0) {
        inst.state = .resting;
        if (inst.sleep_tick < SLEEP_TIME_THRESHOLD) {
            inst.sleep_tick = SLEEP_TIME_THRESHOLD;
        }
    }
}

pub fn applyVerticalSurfaceContact(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    surface_type: terrain.SurfaceType,
    impact_velocity: i16,
) void {
    const medium_type = material_pairing.getMediumType(surface_type);
    const response = material_pairing.getDefaultResponse(surface_type);
    const combined_restitution = material_pairing.combineRestitution(entity.physics.restitution, surface_type);
    const combined_friction = material_pairing.combineFriction(entity.physics.friction, surface_type);

    if (medium_type == .liquid) {
        const effective_gravity = 1.0 - response.buoyancy;
        inst.vel_y = @divTrunc(impact_velocity * @as(i16, @intFromFloat(effective_gravity)), 10);
        const water_friction = @as(u8, @intFromFloat(@min(1.0, combined_friction * 1.5) * 255.0));
        physics.applyFriction(&inst.vel_x, &inst.vel_z, water_friction);
    } else if (medium_type == .soft) {
        inst.vel_y = @divTrunc(impact_velocity * @as(i16, @intFromFloat(combined_restitution * 0.5)), 10);
        const soft_friction = @as(u8, @intFromFloat(@min(1.0, combined_friction * 1.2) * 255.0));
        physics.applyFriction(&inst.vel_x, &inst.vel_z, soft_friction);
        inst.pos_y -= 1;
    } else {
        const restitution_u8 = @as(u8, @intFromFloat(@min(1.0, combined_restitution) * 255.0));
        inst.vel_y = physics.applyRestitution(impact_velocity, restitution_u8);
        const friction_u8 = @as(u8, @intFromFloat(@min(1.0, combined_friction) * 255.0));
        physics.applyFriction(&inst.vel_x, &inst.vel_z, friction_u8);
    }

    settleGroundContact(inst);
}

pub fn applyLateralSurfaceFriction(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    surface_type: terrain.SurfaceType,
    axis: ContactAxis,
) void {
    const medium_type = material_pairing.getMediumType(surface_type);
    const combined_friction = material_pairing.combineFriction(entity.physics.friction, surface_type);
    const friction_scale = switch (medium_type) {
        .liquid => @min(1.0, combined_friction * 1.5),
        .soft => @min(1.0, combined_friction * 1.2),
        else => @min(1.0, combined_friction),
    };
    const friction_factor: i32 = 256 - @as(i32, @intFromFloat(friction_scale * 255.0));

    if (axis == .x) {
        inst.vel_x = 0;
        inst.vel_z = @truncate(@divTrunc(@as(i32, inst.vel_z) * friction_factor, 256));
    } else {
        inst.vel_z = 0;
        inst.vel_x = @truncate(@divTrunc(@as(i32, inst.vel_x) * friction_factor, 256));
    }
}
