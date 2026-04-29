//! Shared contact response helpers for world/tick integration paths.
const std = @import("std");

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
    } else if (medium_type == .vapor) {
        const vapor_restitution = @as(u8, @intFromFloat(@min(1.0, combined_restitution * 1.5) * 255.0));
        inst.vel_y = physics.applyRestitution(impact_velocity, vapor_restitution);
        const vapor_friction = @as(u8, @intFromFloat(@min(1.0, combined_friction * 0.3) * 255.0));
        physics.applyFriction(&inst.vel_x, &inst.vel_z, vapor_friction);
    } else if (medium_type == .plasma) {
        inst.vel_y = @divTrunc(impact_velocity * @as(i16, @intFromFloat(combined_restitution * 0.2)), 10);
        const plasma_friction = @as(u8, @intFromFloat(@min(1.0, combined_friction * 2.0) * 255.0));
        physics.applyFriction(&inst.vel_x, &inst.vel_z, plasma_friction);
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
        .vapor => @min(1.0, combined_friction * 0.3),
        .plasma => @min(1.0, combined_friction * 2.0),
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



/// Apply rolling resistance to vehicle speed based on ground surface.
/// Uses terrain surface rolling resistance for realistic deceleration.
pub fn applyVehicleRollingResistance(
    speed: f32,
    world_x: i32,
    world_z: i32,
    mass_kg: f32,
    dt: f32,
) f32 {
    if (@abs(speed) < 0.01) return 0.0;
    
    const base_rolling = terrain.getRollingResistanceAt(world_x, world_z);
    // Vehicle mass scale: heavier = more rolling resistance
    const mass_scale = std.math.clamp(mass_kg / 1500.0, 0.5, 2.0);
    // Rolling resistance coefficient per tick at 60fps
    const rolling_coeff = base_rolling * mass_scale;
    
    const ticks = dt * 60.0;
    const damping = std.math.pow(f32, 1.0 - rolling_coeff, ticks);
    const clamped_damping = std.math.clamp(damping, 0.5, 1.0);
    return speed * clamped_damping;
}

/// Compute water depth at vehicle position, returns 0 if dry land
pub fn getVehicleWaterDepth(vehicle_x: f32, vehicle_z: f32) f32 {
    const world_x = @as(i32, @intFromFloat(@floor(vehicle_x)));
    const world_z = @as(i32, @intFromFloat(@floor(vehicle_z)));
    return terrain.getWaterDepthAt(world_x, world_z);
}

/// Compute water resistance coefficient for vehicle speed
/// Higher speed = exponential drag (wave-making resistance)
pub fn computeWaterDragCoefficient(speed: f32, water_depth: f32) f32 {
    if (water_depth <= 0) return 0.0;
    // Drag rises with speed^2 (wave resistance) and inversely with depth (shallow = more drag)
    const depth_factor = std.math.clamp(1.0 / (1.0 + water_depth * 0.1), 0.2, 1.0);
    const speed_factor = std.math.pow(f32, speed / 100.0, 2.0);
    return std.math.clamp(0.08 * depth_factor * (1.0 + speed_factor), 0.02, 0.5);
}

/// Apply water resistance to vehicle speed
pub fn applyVehicleWaterResistance(
    speed: f32,
    vehicle_x: f32,
    vehicle_z: f32,
    dt: f32,
) f32 {
    const water_depth = getVehicleWaterDepth(vehicle_x, vehicle_z);
    if (water_depth <= 0) return speed;
    if (@abs(speed) < 0.1) return 0.0;
    
    const drag_coeff = computeWaterDragCoefficient(@abs(speed), water_depth);
    const ticks = dt * 60.0;
    const damping = std.math.pow(f32, 1.0 - drag_coeff, ticks);
    const clamped_damping = std.math.clamp(damping, 0.5, 1.0);
    return speed * clamped_damping;
}

/// Compute buoyancy force (upward) from water depth
/// Returns fraction of gravity counteracted (0-1)
pub fn computeBuoyancyFraction(water_depth: f32, vehicle_draft: f32) f32 {
    if (water_depth <= 0) return 0.0;
    const submerged = @min(water_depth, vehicle_draft);
    return std.math.clamp(submerged / @max(vehicle_draft, 0.1), 0.0, 0.95);
}

/// Compute vehicle draft (how deep it sits in water) based on mass
pub fn computeVehicleDraft(mass_kg: f32) f32 {
    // Rough draft: heavier vehicles sink deeper
    // ~100kg -> 0.1m draft, ~5000kg -> 0.8m draft
    return std.math.clamp(0.05 + (mass_kg / 1500.0) * 0.15, 0.1, 1.0);
}

/// Compute weather-modified aerodynamic drag coefficient for vehicle
pub fn computeWeatherDragCoefficient(weather_severity: f32, wind_speed: f32) f32 {
    // Base drag (0.5% per tick at calm)
    var base_drag: f32 = 0.005;
    // Weather increases drag (rain, snow, fog)
    base_drag *= 1.0 + weather_severity * 0.8;
    // Wind adds significant drag
    const wind_factor = @min(wind_speed / 60.0, 1.0);
    base_drag *= 1.0 + wind_factor * 0.5;
    return base_drag;
}

/// Apply weather-based aerodynamic drag to vehicle speed
pub fn applyVehicleAerodynamicDrag(speed: f32, weather_severity: f32, wind_speed: f32, dt: f32) f32 {
    const drag_coeff = computeWeatherDragCoefficient(weather_severity, wind_speed);
    const ticks = dt * 60.0;
    const damping = std.math.pow(f32, @max(1.0 - drag_coeff, 0.8), ticks);
    return speed * damping;
}
