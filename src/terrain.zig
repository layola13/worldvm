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

var g_terrain_system: TerrainSystem = undefined;

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
    const idx = g_terrain_system.patch_count;
    g_terrain_system.patch_count += 1;

    const friction: f32 = switch (surface) {
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
    };

    const rolling: f32 = switch (surface) {
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
    };

    g_terrain_system.patches[idx] = .{
        .center_x = x,
        .center_z = z,
        .radius = radius,
        .surface_type = surface,
        .friction_coefficient = friction,
        .rolling_resistance = rolling,
        .water_depth = if (surface == .asphalt_wet) 0.5 else 0,
        .roughness = if (surface == .rumble_strip) 0.1 else 0.01,
        .temperature = 20,
    };
}

pub fn getSurfaceAt(world_x: i32, world_z: i32) SurfaceType {
    for (g_terrain_system.patches[0..g_terrain_system.patch_count]) |patch| {
        const dx = world_x - patch.center_x;
        const dz = world_z - patch.center_z;
        const dist_sq = dx * dx + dz * dz;
        if (dist_sq < patch.radius * patch.radius) {
            return patch.surface_type;
        }
    }
    return .asphalt_dry;
}

pub fn getFrictionAt(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
    var friction: f32 = switch (surface) {
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
    };

    if (g_terrain_system.weather.rain_intensity > 0) {
        friction *= 1.0 - g_terrain_system.weather.rain_intensity * 0.3;
    }

    return friction * g_terrain_system.global_friction_modifier;
}

pub fn getRollingResistanceAt(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
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
        else => 0.02,
    };
}

pub fn getWaterDepthAt(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
    if (surface == .asphalt_wet) {
        return 0.5 + g_terrain_system.weather.rain_intensity * 2.0;
    }
    if (surface == .water) {
        return 10.0;
    }
    return g_terrain_system.weather.rain_intensity * 0.5;
}

pub fn applyWeather(weather: WeatherCondition) void {
    g_terrain_system.weather = weather;

    if (weather.air_temperature < 0) {
        g_terrain_system.weather.freezing = true;
    }

    g_terrain_system.visibility = if (weather.fog_density > 0)
        1000.0 * (1.0 - weather.fog_density)
    else
        1000.0;
}

pub fn getSurfaceRoughness(world_x: i32, world_z: i32) f32 {
    const surface = getSurfaceAt(world_x, world_z);
    return switch (surface) {
        .rumble_strip => 0.1,
        .gravel => 0.05,
        .mud => 0.03,
        .mud_ruts => 0.08,
        else => 0.01,
    };
}

pub fn calculateHydroplaningRisk(speed: f32, water_depth: f32, tire_width: f32) f32 {
    if (water_depth < 0.1) return 0;
    const critical_speed = @sqrt(tire_width * 10.0) * 5.0;
    if (speed < critical_speed) return 0;
    return @min(1.0, (speed - critical_speed) / critical_speed * water_depth * 0.1);
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
