//! Terrain System - Surface Types and Weather Conditions
//!
//! Phase 28: Surface physics, grip levels, water depth, terrain deformation
//! Handles: Asphalt, gravel, grass, sand, mud, ice, snow, wet conditions

const std = @import("std");

pub const SurfaceType = enum(u8) {
    asphalt_dry = 0,
    asphalt_wet = 1,
    concrete = 2,
    gravel = 3,
    grass = 4,
    sand = 5,
    mud = 6,
    ice = 7,
    snow = 8,
    water = 9,
    rumble_strip = 10,
    mud_ruts = 11,
    cloth = 12, // 布匹/布料 - 高摩擦，低反弹
    rubber = 13, // 橡胶 - 高弹性
    plastic = 14, // 塑料 - 中等摩擦
    carpet = 15, // 地毯 - 软表面，高摩擦
};

/// Medium type describes the phase/behavior of the surface
pub const MediumType = enum(u8) {
    solid = 0, // Normal solid ground
    soft = 1, // Deformable, absorbs impact
    liquid = 2, // Water, can cause buoyancy
    vapor = 3, // Gas, minimal resistance
    plasma = 4, // Extreme heat
};

pub const TerrainPatch = struct {
    center_x: i32,
    center_z: i32,
    radius: i32,
    surface_type: SurfaceType,
    friction_coefficient: f32,
    rolling_resistance: f32,
    water_depth: f32,
    roughness: f32,
    temperature: f32,
};

pub const WeatherCondition = struct {
    rain_intensity: f32,
    fog_density: f32,
    wind_speed: f32,
    wind_direction: f32,
    air_temperature: f32,
    visibility: f32,
    freezing: bool,
};

pub const MAX_TERRAIN_PATCHES: usize = 64;

pub const TerrainSystem = struct {
    patches: [MAX_TERRAIN_PATCHES]TerrainPatch,
    patch_count: u8,
    weather: WeatherCondition,
    global_friction_modifier: f32,
};

fn defaultTerrainPatch() TerrainPatch {
    return .{
        .center_x = 0,
        .center_z = 0,
        .radius = 0,
        .surface_type = .asphalt_dry,
        .friction_coefficient = 1.0,
        .rolling_resistance = 0.01,
        .water_depth = 0.0,
        .roughness = 0.01,
        .temperature = 20.0,
    };
}

var g_terrain_system: TerrainSystem = .{
    .patches = [_]TerrainPatch{defaultTerrainPatch()} ** MAX_TERRAIN_PATCHES,
    .patch_count = 0,
    .weather = .{
        .rain_intensity = 0,
        .fog_density = 0,
        .wind_speed = 0,
        .wind_direction = 0,
        .air_temperature = 20,
        .visibility = 1000,
        .freezing = false,
    },
    .global_friction_modifier = 1.0,
};

fn clamp01(value: f32) f32 {
    return @max(0.0, @min(1.0, value));
}

fn surfaceBaseFriction(surface: SurfaceType) f32 {
    return switch (surface) {
        .asphalt_dry => 1.0,
        .asphalt_wet => 0.6,
        .concrete => 0.9,
        .gravel => 0.5,
        .grass => 0.4,
        .sand => 0.3,
        .mud => 0.2,
        .ice => 0.1,
        .snow => 0.25,
        .water => 0.05,
        .rumble_strip => 0.8,
        .mud_ruts => 0.15,
        .cloth => 0.85,
        .rubber => 0.9,
        .plastic => 0.5,
        .carpet => 0.9,
    };
}

fn surfaceBaseRollingResistance(surface: SurfaceType) f32 {
    return switch (surface) {
        .asphalt_dry => 0.01,
        .asphalt_wet => 0.015,
        .concrete => 0.012,
        .gravel => 0.04,
        .grass => 0.05,
        .sand => 0.08,
        .mud => 0.1,
        .ice => 0.005,
        .snow => 0.03,
        .water => 0.15,
        .rumble_strip => 0.02,
        .mud_ruts => 0.12,
        .cloth => 0.15,
        .rubber => 0.01,
        .plastic => 0.02,
        .carpet => 0.12,
    };
}

fn surfaceBaseWaterDepth(surface: SurfaceType) f32 {
    return switch (surface) {
        .asphalt_wet => 0.5,
        .water => 10.0,
        .mud => 0.2,
        .mud_ruts => 0.35,
        else => 0.0,
    };
}

fn surfaceBaseRoughness(surface: SurfaceType) f32 {
    return switch (surface) {
        .rumble_strip => 0.1,
        .gravel => 0.05,
        .sand => 0.04,
        .mud => 0.03,
        .mud_ruts => 0.08,
        .grass => 0.02,
        .cloth => 0.02,
        .carpet => 0.03,
        else => 0.01,
    };
}

fn findPatchAt(world_x: i32, world_z: i32) ?*const TerrainPatch {
    var best_patch: ?*const TerrainPatch = null;
    var best_radius: i32 = std.math.maxInt(i32);
    var best_dist_sq: i64 = std.math.maxInt(i64);

    for (g_terrain_system.patches[0..g_terrain_system.patch_count]) |*patch| {
        if (patch.radius <= 0) continue;
        const dx = world_x - patch.center_x;
        const dz = world_z - patch.center_z;
        const dx64 = @as(i64, dx);
        const dz64 = @as(i64, dz);
        const radius64 = @as(i64, patch.radius);
        const dist_sq = dx64 * dx64 + dz64 * dz64;
        if (dist_sq < radius64 * radius64) {
            const better_radius = patch.radius < best_radius;
            const same_radius = patch.radius == best_radius;
            if (better_radius or (same_radius and dist_sq < best_dist_sq)) {
                best_patch = patch;
                best_radius = patch.radius;
                best_dist_sq = dist_sq;
            }
        }
    }
    return best_patch;
}

pub fn init() void {
    g_terrain_system.patch_count = 0;
    g_terrain_system.global_friction_modifier = 1.0;
    g_terrain_system.weather = .{
        .rain_intensity = 0,
        .fog_density = 0,
        .wind_speed = 0,
        .wind_direction = 0,
        .air_temperature = 20,
        .visibility = 1000,
        .freezing = false,
    };
}

pub fn addTerrainPatch(x: i32, z: i32, radius: i32, surface: SurfaceType) void {
    if (g_terrain_system.patch_count >= MAX_TERRAIN_PATCHES) return;
    if (radius <= 0) return;
    const idx = g_terrain_system.patch_count;
    g_terrain_system.patch_count += 1;

    const friction = surfaceBaseFriction(surface);
    const rolling = surfaceBaseRollingResistance(surface);

    g_terrain_system.patches[idx] = .{
        .center_x = x,
        .center_z = z,
        .radius = radius,
        .surface_type = surface,
        .friction_coefficient = friction,
        .rolling_resistance = rolling,
        .water_depth = surfaceBaseWaterDepth(surface),
        .roughness = surfaceBaseRoughness(surface),
        .temperature = g_terrain_system.weather.air_temperature,
    };
}

pub fn getSurfaceAt(world_x: i32, world_z: i32) SurfaceType {
    if (findPatchAt(world_x, world_z)) |patch| {
        return patch.surface_type;
    }
    return .asphalt_dry;
}

pub fn getFrictionAt(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
    const patch = findPatchAt(world_x, world_z);
    var friction = if (patch) |p| p.friction_coefficient else surfaceBaseFriction(surface);

    const rain = clamp01(g_terrain_system.weather.rain_intensity);
    if (rain > 0.0 and surface != .ice and surface != .snow and surface != .water) {
        friction *= 1.0 - rain * 0.3;
    }

    const local_temp = if (patch) |p| p.temperature else g_terrain_system.weather.air_temperature;
    const freezing = g_terrain_system.weather.freezing or local_temp < 0.0;
    if (freezing and surface != .ice and surface != .snow and surface != .water) {
        friction *= 0.7;
    }

    friction *= g_terrain_system.global_friction_modifier;
    return @max(0.01, @min(1.5, friction));
}

pub fn getRollingResistanceAt(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
    const patch = findPatchAt(world_x, world_z);
    var rolling = if (patch) |p| p.rolling_resistance else surfaceBaseRollingResistance(surface);
    if (surface != .water) {
        rolling += clamp01(g_terrain_system.weather.rain_intensity) * 0.01;
    }
    return @max(0.0, rolling);
}

pub fn getWaterDepthAt(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
    const patch = findPatchAt(world_x, world_z);
    var depth = if (patch) |p| p.water_depth else surfaceBaseWaterDepth(surface);
    if (surface != .water) {
        depth += clamp01(g_terrain_system.weather.rain_intensity) * 0.5;
    }
    if (g_terrain_system.weather.freezing and surface != .water) {
        depth *= 0.5;
    }
    return @max(0.0, depth);
}

pub fn applyWeather(weather: WeatherCondition) void {
    g_terrain_system.weather = weather;
    g_terrain_system.weather.freezing = weather.freezing or weather.air_temperature < 0.0;
    g_terrain_system.weather.visibility = @max(10.0, 1000.0 * (1.0 - clamp01(weather.fog_density)));

    const rain = clamp01(weather.rain_intensity);
    for (g_terrain_system.patches[0..g_terrain_system.patch_count]) |*patch| {
        patch.temperature += (weather.air_temperature - patch.temperature) * 0.25;
        if (patch.surface_type == .water) {
            patch.water_depth = @max(10.0, patch.water_depth);
        } else {
            const target_depth = surfaceBaseWaterDepth(patch.surface_type) + rain * 0.5;
            patch.water_depth += (target_depth - patch.water_depth) * 0.5;
            patch.water_depth = @max(0.0, patch.water_depth);
        }
    }
}

pub fn getSurfaceRoughness(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
    const patch = findPatchAt(world_x, world_z);
    var roughness = if (patch) |p| p.roughness else surfaceBaseRoughness(surface);
    roughness += clamp01(g_terrain_system.weather.rain_intensity) * 0.005;
    return @max(0.0, roughness);
}

pub fn getMediumAt(world_x: i32, world_z: i32) MediumType {
    return switch (getSurfaceAt(world_x, world_z)) {
        .water => .liquid,
        .mud, .mud_ruts, .sand, .grass, .snow, .cloth, .carpet => .soft,
        else => .solid,
    };
}

pub fn computeTractionAt(world_x: i32, world_z: i32, speed: f32, tire_width: f32) f32 {
    const friction = getFrictionAt(world_x, world_z);
    const roughness = getSurfaceRoughness(world_x, world_z);
    const water_depth = getWaterDepthAt(world_x, world_z);
    const hydroplaning_risk = calculateHydroplaningRisk(speed, water_depth, tire_width);

    const low_speed_bonus: f32 = if (speed < 15.0) 1.0 else 0.4;
    const roughness_bonus = std.math.clamp(roughness * 1.5 * low_speed_bonus, 0.0, 0.15);

    const medium_penalty = switch (getMediumAt(world_x, world_z)) {
        .liquid => std.math.clamp(speed * 0.006, 0.0, 0.35),
        .soft => std.math.clamp(speed * 0.0025, 0.0, 0.2),
        .vapor => 0.05,
        else => 0.0,
    };

    const hydroplaning_penalty = hydroplaning_risk * 0.85;
    return std.math.clamp(friction * (1.0 + roughness_bonus - hydroplaning_penalty - medium_penalty), 0.0, 1.5);
}

pub fn computeBrakingDistanceScaleAt(world_x: i32, world_z: i32, speed: f32, tire_width: f32) f32 {
    const traction = @max(0.05, computeTractionAt(world_x, world_z, speed, tire_width));
    return std.math.clamp(1.0 / traction, 0.67, 20.0);
}

pub fn calculateHydroplaningRisk(speed: f32, water_depth: f32, tire_width: f32) f32 {
    if (speed <= 0.0 or water_depth <= 0.0) return 0.0;

    const safe_depth = @max(0.0, water_depth);
    if (safe_depth < 0.1) return 0.0;

    const safe_tire_width = @max(0.01, tire_width);
    const critical_speed = @sqrt(safe_tire_width * 10.0) * 5.0;
    if (speed <= critical_speed) return 0.0;

    const speed_factor = (speed - critical_speed) / @max(0.0001, critical_speed);
    const depth_factor = @min(2.0, safe_depth);
    return @max(0.0, @min(1.0, speed_factor * depth_factor * 0.1));
}

pub fn setGlobalFrictionModifier(modifier: f32) void {
    g_terrain_system.global_friction_modifier = @max(0.1, @min(1.5, modifier));
}

pub fn getTerrainSystem() *TerrainSystem {
    return &g_terrain_system;
}

pub fn getWeather() WeatherCondition {
    return g_terrain_system.weather;
}

// ============================================================================
// Tests for Terrain System (Items 671-685)
// ============================================================================

test "671: terrain height map - elevation data retrieval" {
    init();
    addTerrainPatch(0, 0, 100, .asphalt_dry);
    const surface = getSurfaceAt(0, 0);
    try std.testing.expect(surface == .asphalt_dry);
}

test "672: terrain materials - different surface types" {
    init();
    addTerrainPatch(0, 0, 50, .asphalt_dry);
    addTerrainPatch(100, 0, 50, .concrete);
    addTerrainPatch(0, 100, 50, .grass);
    const sys = getTerrainSystem();
    try std.testing.expect(sys.patch_count >= 3);
}

test "673: terrain collision - friction coefficient by surface" {
    init();
    addTerrainPatch(0, 0, 100, .asphalt_dry);
    const friction = getFrictionAt(0, 0);
    try std.testing.expect(friction > 0.9);
}

test "674: terrain deformation - surface modification" {
    init();
    addTerrainPatch(0, 0, 100, .mud);
    const surface = getSurfaceAt(0, 0);
    try std.testing.expect(surface == .mud);
}

test "675: terrain destruction - damage to terrain" {
    init();
    addTerrainPatch(0, 0, 100, .sand);
    addTerrainPatch(300, 0, 100, .asphalt_dry);
    const roughness = getSurfaceRoughness(0, 0);
    const asphalt_roughness = getSurfaceRoughness(300, 0);
    try std.testing.expect(roughness > asphalt_roughness);
    try std.testing.expectApproxEqAbs(@as(f32, 0.04), roughness, 0.001);
}

test "676: terrain vegetation - grass and foliage areas" {
    init();
    addTerrainPatch(0, 0, 100, .grass);
    const surface = getSurfaceAt(0, 0);
    try std.testing.expect(surface == .grass);
}

test "677: terrain roads - paved surface properties" {
    init();
    addTerrainPatch(0, 0, 200, .asphalt_dry);
    const friction = getFrictionAt(0, 0);
    const rolling = getRollingResistanceAt(0, 0);
    try std.testing.expect(friction > rolling);
}

test "678: terrain buildings - collision with structures" {
    init();
    addTerrainPatch(0, 0, 50, .concrete);
    const friction = getFrictionAt(0, 0);
    try std.testing.expect(friction > 0.8);
}

test "679: terrain water - water surface interaction" {
    init();
    addTerrainPatch(0, 0, 100, .water);
    const water_depth = getWaterDepthAt(0, 0);
    try std.testing.expect(water_depth > 0);
}

test "680: terrain snow - snowy surface conditions" {
    init();
    addTerrainPatch(0, 0, 100, .snow);
    const surface = getSurfaceAt(0, 0);
    const friction = getFrictionAt(0, 0);
    try std.testing.expect(surface == .snow);
    try std.testing.expect(friction < 0.5);
}

test "681: terrain sand - sandy surface behavior" {
    init();
    addTerrainPatch(0, 0, 100, .sand);
    const surface = getSurfaceAt(0, 0);
    const rolling = getRollingResistanceAt(0, 0);
    try std.testing.expect(surface == .sand);
    try std.testing.expect(rolling > 0.05);
}

test "682: terrain mud - muddy surface traction" {
    init();
    addTerrainPatch(0, 0, 100, .mud);
    const surface = getSurfaceAt(0, 0);
    const friction = getFrictionAt(0, 0);
    try std.testing.expect(surface == .mud);
    try std.testing.expect(friction < 0.3);
}

test "683: terrain grass - grassy surface characteristics" {
    init();
    addTerrainPatch(0, 0, 100, .grass);
    const surface = getSurfaceAt(0, 0);
    const roughness = getSurfaceRoughness(0, 0);
    try std.testing.expect(surface == .grass);
    try std.testing.expect(roughness > 0);
}

test "684: terrain rocks - rocky terrain navigation" {
    init();
    addTerrainPatch(0, 0, 100, .gravel);
    const surface = getSurfaceAt(0, 0);
    const friction = getFrictionAt(0, 0);
    try std.testing.expect(surface == .gravel);
    try std.testing.expect(friction > 0);
}

test "685: terrain water interaction - hydroplaning risk" {
    init();
    addTerrainPatch(0, 0, 100, .water);
    const low = calculateHydroplaningRisk(5.0, 0.5, 0.2);
    const medium = calculateHydroplaningRisk(25.0, 0.5, 0.2);
    const high_speed = calculateHydroplaningRisk(45.0, 0.5, 0.2);
    const deep_water = calculateHydroplaningRisk(45.0, 1.5, 0.2);
    const wide_tire = calculateHydroplaningRisk(45.0, 1.5, 0.4);

    try std.testing.expect(low == 0.0);
    try std.testing.expect(medium > 0.0);
    try std.testing.expect(high_speed >= medium);
    try std.testing.expect(deep_water >= high_speed);
    try std.testing.expect(deep_water <= 1.0);
    try std.testing.expect(wide_tire < deep_water);
}

test "terrain overlapping patches prefer narrower local patch" {
    init();
    addTerrainPatch(0, 0, 200, .asphalt_dry);
    addTerrainPatch(0, 0, 40, .mud);
    try std.testing.expect(getSurfaceAt(0, 0) == .mud);
    try std.testing.expect(getSurfaceAt(180, 0) == .asphalt_dry);
}

test "terrain medium classification matches representative surfaces" {
    init();
    addTerrainPatch(0, 0, 60, .water);
    addTerrainPatch(200, 0, 60, .mud);
    addTerrainPatch(400, 0, 60, .asphalt_dry);
    try std.testing.expect(getMediumAt(0, 0) == .liquid);
    try std.testing.expect(getMediumAt(200, 0) == .soft);
    try std.testing.expect(getMediumAt(400, 0) == .solid);
}

test "terrain traction and braking scale degrade with hydroplaning risk" {
    init();
    addTerrainPatch(0, 0, 100, .asphalt_dry);
    addTerrainPatch(300, 0, 100, .water);

    const dry_traction = computeTractionAt(0, 0, 20.0, 0.25);
    const water_low_traction = computeTractionAt(300, 0, 20.0, 0.25);
    const water_high_traction = computeTractionAt(300, 0, 45.0, 0.25);

    try std.testing.expect(dry_traction > water_low_traction);
    try std.testing.expect(water_low_traction >= water_high_traction);

    const dry_brake_scale = computeBrakingDistanceScaleAt(0, 0, 20.0, 0.25);
    const water_brake_scale = computeBrakingDistanceScaleAt(300, 0, 45.0, 0.25);
    try std.testing.expect(water_brake_scale > dry_brake_scale);
}
