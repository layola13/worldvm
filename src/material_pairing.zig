//! Material Pairing System - Contact Material × Material Response
//!
//! P0: Material pairing is completely missing per todo/1.md assessment
//! This system implements Material A × Material B → ContactResponse lookup
//!
//! Without this, we can only express "things fall down" but NOT
//! "what hits what medium with what response"
//!
//! Uses terrain.zig's SurfaceType to avoid duplication

const std = @import("std");
const entity16 = @import("entity16.zig");
const terrain = @import("terrain.zig");

/// Contact response from material pairing
pub const ContactResponse = struct {
    restitution: f32, // Bounciness 0.0-1.0
    friction: f32, // Sliding resistance 0.0-1.0
    damage_modifier: f32, // Damage multiplier on impact
    sound_type: u8, // What sound to play
    dust_type: u8, // What dust/debris particle type
    penetration_resistance: f32, // How hard to penetrate this surface
    buoyancy: f32, // 0.0 = sinks, 1.0 = floats
    medium_type: terrain.MediumType, // solid/soft/liquid/vapor
};

pub const StaticFrictionPolicy = struct {
    static_multiplier: f32 = 1.25,
    low_speed_threshold: f32 = 0.5,
    breakaway_speed: f32 = 2.0,
    max_static_friction: f32 = 2.0,
};

pub const DynamicFrictionPolicy = struct {
    reference_speed: f32 = 2.0,
    saturation_speed: f32 = 16.0,
    high_speed_scale: f32 = 0.75,
    max_dynamic_friction: f32 = 2.0,
};

pub const RollingFrictionPolicy = struct {
    rolling_ratio: f32 = 0.08,
    angular_reference_speed: f32 = 1.0,
    angular_saturation_speed: f32 = 16.0,
    high_speed_multiplier: f32 = 1.5,
    max_rolling_friction: f32 = 0.5,
};

pub const AnisotropicFrictionPolicy = struct {
    isotropic_minor_axis_scale: f32 = 1.0,
    loose_minor_axis_scale: f32 = 0.85,
    fiber_minor_axis_scale: f32 = 0.65,
    groove_minor_axis_scale: f32 = 0.5,
    rutted_minor_axis_scale: f32 = 0.4,
    min_minor_axis_scale: f32 = 0.25,
    max_minor_axis_scale: f32 = 1.0,
};

/// Approximate a material-only object to a representative surface class so
/// query/contact metadata can stay on the same vocabulary as terrain hits.
pub fn getSurfaceForMaterial(material: entity16.MaterialType) terrain.SurfaceType {
    return switch (material) {
        .solid => .concrete,
        .liquid => .water,
        .gas => .plastic,
        .fragile => .plastic,
        .elastic => .rubber,
        .composite => .gravel,
    };
}

/// Get medium type for a surface
pub fn getMediumType(surface: terrain.SurfaceType) terrain.MediumType {
    return switch (surface) {
        .asphalt_dry, .asphalt_wet, .concrete, .gravel, .rumble_strip, .ice, .rubber, .plastic => .solid,
        .grass, .sand, .mud, .mud_ruts, .snow, .cloth, .carpet => .soft,
        .water => .liquid,
    };
}

/// Get default contact response for a single surface (no pairing)
pub fn getDefaultResponse(surface: terrain.SurfaceType) ContactResponse {
    return switch (surface) {
        .asphalt_dry => .{
            .restitution = 0.5,
            .friction = 0.8,
            .damage_modifier = 1.0,
            .sound_type = 1, // Tire screech
            .dust_type = 0, // None
            .penetration_resistance = 0.9,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .asphalt_wet => .{
            .restitution = 0.3,
            .friction = 0.5,
            .damage_modifier = 0.9,
            .sound_type = 1,
            .dust_type = 12, // Water droplets
            .penetration_resistance = 0.85,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .concrete => .{
            .restitution = 0.3,
            .friction = 0.7,
            .damage_modifier = 1.0,
            .sound_type = 1, // Hard thud
            .dust_type = 1, // Concrete dust
            .penetration_resistance = 0.9,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .gravel => .{
            .restitution = 0.25,
            .friction = 0.6,
            .damage_modifier = 0.8,
            .sound_type = 6, // Crunch
            .dust_type = 8, // Gravel
            .penetration_resistance = 0.5,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .grass => .{
            .restitution = 0.2,
            .friction = 0.65,
            .damage_modifier = 0.5,
            .sound_type = 9, // Swish
            .dust_type = 11, // Grass
            .penetration_resistance = 0.1,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        .sand => .{
            .restitution = 0.05,
            .friction = 0.8,
            .damage_modifier = 0.3,
            .sound_type = 8, // Rustle
            .dust_type = 10, // Sand
            .penetration_resistance = 0.15,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        .mud => .{
            .restitution = 0.0,
            .friction = 1.0,
            .damage_modifier = 0.4,
            .sound_type = 7, // Squelch
            .dust_type = 9, // Mud splatter
            .penetration_resistance = 0.2,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        .ice => .{
            .restitution = 0.8,
            .friction = 0.1,
            .damage_modifier = 0.9,
            .sound_type = 4, // Sharp crack
            .dust_type = 6, // Ice shards
            .penetration_resistance = 0.4,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .snow => .{
            .restitution = 0.1,
            .friction = 0.4,
            .damage_modifier = 0.3,
            .sound_type = 15, // Crunch
            .dust_type = 16, // Snow
            .penetration_resistance = 0.1,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        .water => .{
            .restitution = 0.4,
            .friction = 0.3,
            .damage_modifier = 0.7,
            .sound_type = 10, // Splash
            .dust_type = 12, // Water droplets
            .penetration_resistance = 0.05,
            .buoyancy = 1.0,
            .medium_type = .liquid,
        },
        .rumble_strip => .{
            .restitution = 0.5,
            .friction = 0.9,
            .damage_modifier = 1.5,
            .sound_type = 14, // Rumble
            .dust_type = 0,
            .penetration_resistance = 0.95,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .mud_ruts => .{
            .restitution = 0.0,
            .friction = 1.0,
            .damage_modifier = 0.5,
            .sound_type = 7, // Squelch
            .dust_type = 9, // Mud
            .penetration_resistance = 0.25,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        .cloth => .{
            .restitution = 0.1,
            .friction = 0.85,
            .damage_modifier = 0.3,
            .sound_type = 17, // Soft thud
            .dust_type = 0, // None
            .penetration_resistance = 0.05,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        .rubber => .{
            .restitution = 0.95,
            .friction = 0.9,
            .damage_modifier = 0.5,
            .sound_type = 12, // Bounce
            .dust_type = 0, // None
            .penetration_resistance = 0.6,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .plastic => .{
            .restitution = 0.5,
            .friction = 0.5,
            .damage_modifier = 0.7,
            .sound_type = 13, // Click
            .dust_type = 14, // Plastic bits
            .penetration_resistance = 0.3,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        .carpet => .{
            .restitution = 0.15,
            .friction = 0.9,
            .damage_modifier = 0.3,
            .sound_type = 18, // Soft thud
            .dust_type = 0, // None
            .penetration_resistance = 0.08,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
    };
}

/// Combine two materials to get a paired contact response
/// Uses geometric mean for most properties, with special rules
pub fn getPairedResponse(
    surface_a: terrain.SurfaceType,
    material_a: entity16.MaterialType,
    surface_b: terrain.SurfaceType,
    material_b: entity16.MaterialType,
) ContactResponse {
    const resp_a = getDefaultResponse(surface_a);
    const resp_b = getDefaultResponse(surface_b);

    // Base combination: geometric mean for smooth blend
    var result = ContactResponse{
        .restitution = @sqrt(resp_a.restitution * resp_b.restitution),
        .friction = (resp_a.friction + resp_b.friction) / 2.0,
        .damage_modifier = (resp_a.damage_modifier + resp_b.damage_modifier) / 2.0,
        .sound_type = if (resp_a.sound_type != resp_b.sound_type) resp_a.sound_type else resp_b.sound_type,
        .dust_type = resp_a.dust_type,
        .penetration_resistance = (resp_a.penetration_resistance + resp_b.penetration_resistance) / 2.0,
        .buoyancy = (resp_a.buoyancy + resp_b.buoyancy) / 2.0,
        .medium_type = if (@intFromEnum(getMediumType(surface_a)) > @intFromEnum(getMediumType(surface_b)))
            getMediumType(surface_a)
        else
            getMediumType(surface_b),
    };

    // Special case: fragile materials shatter more easily
    if (material_a == .fragile or material_b == .fragile) {
        result.damage_modifier *= 1.5;
    }

    // Special case: elastic materials bounce more
    if (material_a == .elastic or material_b == .elastic) {
        result.restitution = @min(1.0, result.restitution * 1.3);
    }

    // Special case: liquid surfaces have unique friction
    if (getMediumType(surface_a) == .liquid or getMediumType(surface_b) == .liquid) {
        result.friction = @min(0.5, result.friction * 0.5);
        result.restitution = (resp_a.restitution + resp_b.restitution) / 2.0;
    }

    return result;
}

/// Check if a surface is considered "hard" for collision purposes
pub fn isHardSurface(surface: terrain.SurfaceType) bool {
    return switch (surface) {
        .asphalt_dry, .asphalt_wet, .concrete, .gravel, .ice, .rumble_strip, .rubber, .plastic => true,
        else => false,
    };
}

/// Check if a surface causes significant damage on impact
pub fn isHighDamageSurface(surface: terrain.SurfaceType) bool {
    _ = surface;
    return false; // No terrain surface causes high damage currently
}

/// Get the effective restitution combining entity restitution with surface
pub fn combineRestitution(entity_restitution: u8, surface: terrain.SurfaceType) f32 {
    const surface_resp = getDefaultResponse(surface);
    const entity_restitution_f = @as(f32, @floatFromInt(entity_restitution)) / 255.0;
    // Weighted average: entity is 40%, surface is 60%
    return entity_restitution_f * 0.4 + surface_resp.restitution * 0.6;
}

/// Get the effective friction combining entity friction with surface
pub fn combineFriction(entity_friction: u8, surface: terrain.SurfaceType) f32 {
    const surface_resp = getDefaultResponse(surface);
    const entity_friction_f = @as(f32, @floatFromInt(entity_friction)) / 255.0;
    // Weighted average: entity is 40%, surface is 60%
    return entity_friction_f * 0.4 + surface_resp.friction * 0.6;
}

pub fn defaultStaticFrictionPolicy() StaticFrictionPolicy {
    return .{};
}

pub fn defaultDynamicFrictionPolicy() DynamicFrictionPolicy {
    return .{};
}

pub fn defaultRollingFrictionPolicy() RollingFrictionPolicy {
    return .{};
}

pub fn defaultAnisotropicFrictionPolicy() AnisotropicFrictionPolicy {
    return .{};
}

pub fn computeDynamicFrictionCoefficient(
    base_friction: f32,
    tangential_speed: f32,
    policy: DynamicFrictionPolicy,
) f32 {
    const base = @min(policy.max_dynamic_friction, @max(0.0, base_friction));
    const speed = @abs(tangential_speed);
    const reference_speed = @max(0.0, policy.reference_speed);
    const saturation_speed = @max(reference_speed, policy.saturation_speed);
    const high_speed_scale = @max(0.0, @min(1.0, policy.high_speed_scale));

    if (speed <= reference_speed) return base;
    if (saturation_speed <= reference_speed + 0.0001) return base * high_speed_scale;

    const blend = @min(1.0, (speed - reference_speed) / (saturation_speed - reference_speed));
    const scale = 1.0 + (high_speed_scale - 1.0) * blend;
    return base * scale;
}

pub fn computeRollingFrictionCoefficient(
    dynamic_friction: f32,
    angular_speed: f32,
    policy: RollingFrictionPolicy,
) f32 {
    const dynamic = @max(0.0, dynamic_friction);
    const base = dynamic * @max(0.0, policy.rolling_ratio);
    const speed = @abs(angular_speed);
    const reference_speed = @max(0.0, policy.angular_reference_speed);
    const saturation_speed = @max(reference_speed, policy.angular_saturation_speed);
    const high_speed_multiplier = @max(1.0, policy.high_speed_multiplier);

    if (speed <= reference_speed) return @min(policy.max_rolling_friction, base);
    if (saturation_speed <= reference_speed + 0.0001) {
        return @min(policy.max_rolling_friction, base * high_speed_multiplier);
    }

    const blend = @min(1.0, (speed - reference_speed) / (saturation_speed - reference_speed));
    const multiplier = 1.0 + (high_speed_multiplier - 1.0) * blend;
    return @min(policy.max_rolling_friction, base * multiplier);
}

pub fn computeStaticFrictionCoefficient(
    dynamic_friction: f32,
    tangential_speed: f32,
    policy: StaticFrictionPolicy,
) f32 {
    const dynamic = @max(0.0, dynamic_friction);
    const static = @min(policy.max_static_friction, dynamic * @max(1.0, policy.static_multiplier));
    const speed = @abs(tangential_speed);
    const low_speed = @max(0.0, policy.low_speed_threshold);
    const breakaway = @max(low_speed, policy.breakaway_speed);

    if (speed <= low_speed) return static;
    if (speed >= breakaway or breakaway <= low_speed + 0.0001) return dynamic;

    const blend = (speed - low_speed) / (breakaway - low_speed);
    return static + (dynamic - static) * blend;
}

fn clampAnisotropicMinorAxisScale(scale: f32, policy: AnisotropicFrictionPolicy) f32 {
    const min_scale = @max(0.0, policy.min_minor_axis_scale);
    const max_scale = @max(min_scale, policy.max_minor_axis_scale);
    return @max(min_scale, @min(max_scale, scale));
}

pub fn computeSurfaceAnisotropicFrictionMinorAxisScale(
    surface: terrain.SurfaceType,
    policy: AnisotropicFrictionPolicy,
) f32 {
    const raw_scale = switch (surface) {
        .mud_ruts => policy.rutted_minor_axis_scale,
        .rumble_strip => policy.groove_minor_axis_scale,
        .cloth, .carpet => policy.fiber_minor_axis_scale,
        .gravel, .grass, .sand, .snow => policy.loose_minor_axis_scale,
        else => policy.isotropic_minor_axis_scale,
    };
    return clampAnisotropicMinorAxisScale(raw_scale, policy);
}

pub fn computeAnisotropicFrictionMinorAxisScale(
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
    policy: AnisotropicFrictionPolicy,
) f32 {
    return @min(
        computeSurfaceAnisotropicFrictionMinorAxisScale(surface_a, policy),
        computeSurfaceAnisotropicFrictionMinorAxisScale(surface_b, policy),
    );
}

pub fn computeAnisotropicFrictionCoefficient(
    major_axis_friction: f32,
    minor_axis_alignment: f32,
    minor_axis_scale: f32,
) f32 {
    const friction = @max(0.0, major_axis_friction);
    const alignment = @max(0.0, @min(1.0, @abs(minor_axis_alignment)));
    const scale = @max(0.0, @min(1.0, minor_axis_scale));
    return friction * (1.0 + (scale - 1.0) * alignment);
}

/// Calculate impact damage using paired response
pub fn calculateImpactDamage(
    impact_velocity: f32,
    mass: f32,
    surface: terrain.SurfaceType,
    material: entity16.MaterialType,
) f32 {
    const resp = getDefaultResponse(surface);
    const kinetic_energy = 0.5 * mass * impact_velocity * impact_velocity;

    // Base damage from kinetic energy
    var damage = kinetic_energy / 1000.0;

    // Apply material-specific modifier
    const material_mult: f32 = switch (material) {
        .fragile => 3.0,
        .solid => 1.5,
        .elastic => 0.5,
        .composite => 1.0,
        else => 1.0,
    };

    damage *= material_mult;
    damage *= resp.damage_modifier;

    return damage;
}

/// Get surface type at a world position (queries terrain patches)
pub fn getSurfaceAtPosition(world_x: f32, world_z: f32) terrain.SurfaceType {
    return terrain.getSurfaceAt(world_x, world_z);
}

test "computeStaticFrictionCoefficient boosts low-speed contact friction" {
    const policy = defaultStaticFrictionPolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 0.625),
        computeStaticFrictionCoefficient(0.5, 0.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.5),
        computeStaticFrictionCoefficient(0.5, 2.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.5833333),
        computeStaticFrictionCoefficient(0.5, 1.0, policy),
        0.0001,
    );
}

test "computeStaticFrictionCoefficient clamps negative and excessive inputs" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeStaticFrictionCoefficient(-0.25, 0.0, defaultStaticFrictionPolicy()),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 2.0),
        computeStaticFrictionCoefficient(
            4.0,
            0.0,
            .{ .static_multiplier = 2.0, .max_static_friction = 2.0 },
        ),
        0.0001,
    );
}

test "computeDynamicFrictionCoefficient preserves low-speed sliding friction" {
    const policy = defaultDynamicFrictionPolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 0.8),
        computeDynamicFrictionCoefficient(0.8, 0.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.8),
        computeDynamicFrictionCoefficient(0.8, 2.0, policy),
        0.0001,
    );
}

test "computeDynamicFrictionCoefficient decays high-speed sliding friction" {
    const policy = defaultDynamicFrictionPolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 0.7),
        computeDynamicFrictionCoefficient(0.8, 9.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.6),
        computeDynamicFrictionCoefficient(0.8, 16.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeDynamicFrictionCoefficient(-0.8, 16.0, policy),
        0.0001,
    );
}

test "computeRollingFrictionCoefficient derives small angular resistance" {
    const policy = defaultRollingFrictionPolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 0.064),
        computeRollingFrictionCoefficient(0.8, 0.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.096),
        computeRollingFrictionCoefficient(0.8, 16.0, policy),
        0.0001,
    );
}

test "computeRollingFrictionCoefficient clamps invalid and excessive values" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeRollingFrictionCoefficient(-0.8, 16.0, defaultRollingFrictionPolicy()),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.25),
        computeRollingFrictionCoefficient(
            4.0,
            16.0,
            .{ .rolling_ratio = 1.0, .high_speed_multiplier = 2.0, .max_rolling_friction = 0.25 },
        ),
        0.0001,
    );
}

test "computeAnisotropicFrictionMinorAxisScale selects directional surface resistance" {
    const policy = defaultAnisotropicFrictionPolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0),
        computeAnisotropicFrictionMinorAxisScale(.concrete, .rubber, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.5),
        computeAnisotropicFrictionMinorAxisScale(.rumble_strip, .rubber, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.4),
        computeAnisotropicFrictionMinorAxisScale(.carpet, .mud_ruts, policy),
        0.0001,
    );
}

test "computeAnisotropicFrictionCoefficient blends major to minor axis friction" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.8),
        computeAnisotropicFrictionCoefficient(0.8, 0.0, 0.5),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.4),
        computeAnisotropicFrictionCoefficient(0.8, 1.0, 0.5),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.6),
        computeAnisotropicFrictionCoefficient(0.8, 0.5, 0.5),
        0.0001,
    );
}
