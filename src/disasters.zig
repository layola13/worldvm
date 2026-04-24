//! Disaster and Extreme Events System - Earthquakes, Volcanoes, Tsunamis, etc.
//!
//! Phase 71-80: Natural disasters, apocalyptic events, chain reactions
//! Handles: Earthquakes, volcanic activity, tsunamis, hurricanes, meteors, chain disasters

const std = @import("std");

pub const DisasterType = enum(u8) {
    none = 0,
    earthquake = 1,
    volcanic_eruption = 2,
    tsunami = 3,
    hurricane = 4,
    tornado = 5,
    meteor_strike = 6,
    wildfire = 7,
    flood = 8,
    building_collapse = 9,
    avalanche = 10,
};

pub const DisasterEvent = struct {
    disaster_type: DisasterType,
    intensity: f32,
    epicenter_x: f32,
    epicenter_y: f32,
    epicenter_z: f32,
    radius: f32,
    duration: f32,
    elapsed: f32,
    active: bool,
};

pub const SeismicWave = struct {
    amplitude: f32,
    frequency: f32,
    wave_type: u8,
    propagation_speed: f32,
};

pub const AtmosphericEvent = struct {
    pressure_delta: f32,
    wind_speed_max: f32,
    temperature_change: f32,
    humidity: f32,
    visibility_reduction: f32,
};

pub const MAX_ACTIVE_DISASTERS: usize = 4;

pub const DisasterSystem = struct {
    active_disasters: [MAX_ACTIVE_DISASTERS]DisasterEvent,
    disaster_count: u8,
    chain_reaction_enabled: bool,
    apocalypse_mode: bool,
    seismic_wave: SeismicWave,
    atmosphere: AtmosphericEvent,
};

var g_disaster_system: DisasterSystem = undefined;

pub fn init() void {
    g_disaster_system.disaster_count = 0;
    g_disaster_system.chain_reaction_enabled = false;
    g_disaster_system.apocalypse_mode = false;
    g_disaster_system.seismic_wave = .{
        .amplitude = 0,
        .frequency = 0,
        .wave_type = 0,
        .propagation_speed = 3000,
    };
    g_disaster_system.atmosphere = .{
        .pressure_delta = 0,
        .wind_speed_max = 0,
        .temperature_change = 0,
        .humidity = 0.5,
        .visibility_reduction = 0,
    };
}

pub fn triggerDisaster(disaster_type: DisasterType, intensity: f32, x: f32, y: f32, z: f32, radius: f32) void {
    if (g_disaster_system.disaster_count >= MAX_ACTIVE_DISASTERS) return;

    var found_slot = false;
    for (0..MAX_ACTIVE_DISASTERS) |i| {
        if (!g_disaster_system.active_disasters[i].active) {
            g_disaster_system.active_disasters[i] = .{
                .disaster_type = disaster_type,
                .intensity = intensity,
                .epicenter_x = x,
                .epicenter_y = y,
                .epicenter_z = z,
                .radius = radius,
                .duration = switch (disaster_type) {
                    .earthquake => 30,
                    .volcanic_eruption => 120,
                    .tsunami => 60,
                    .hurricane => 300,
                    .tornado => 45,
                    .meteor_strike => 10,
                    .wildfire => 600,
                    .flood => 480,
                    .building_collapse => 30,
                    .avalanche => 60,
                    else => 60,
                },
                .elapsed = 0,
                .active = true,
            };
            found_slot = true;
            break;
        }
    }
    if (found_slot) g_disaster_system.disaster_count += 1;
}

pub fn calculateSeismicIntensity(distance: f32, magnitude: f32) f32 {
    if (distance < 0.1) return magnitude;
    const attenuation = 1.0 / (1.0 + distance * 0.001);
    return magnitude * attenuation * attenuation;
}

pub fn calculateTsunamiWaveHeight(distance: f32, magnitude: f32) f32 {
    const initial_height = magnitude * 10;
    const decay = 1.0 / (1.0 + distance * 0.0005);
    return initial_height * decay;
}

pub fn calculateWindForce(distance_from_center: f32, hurricane_intensity: f32) f32 {
    const max_wind = hurricane_intensity * 100;
    const decay = 1.0 / (1.0 + distance_from_center * 0.01);
    return max_wind * decay * decay;
}

pub fn calculateHeatRadiation(distance: f32, fire_intensity: f32) f32 {
    const radiant_heat = fire_intensity * 1000;
    const decay = 1.0 / (1.0 + distance * 0.1);
    return radiant_heat * decay * decay;
}

pub fn checkChainReaction(x: f32, y: f32, z: f32) bool {
    if (!g_disaster_system.chain_reaction_enabled) return false;

    for (g_disaster_system.active_disasters[0..MAX_ACTIVE_DISASTERS]) |disaster| {
        if (!disaster.active) continue;

        const dx = x - disaster.epicenter_x;
        const dy = y - disaster.epicenter_y;
        const dz = z - disaster.epicenter_z;
        const dist = @sqrt(dx * dx + dy * dy + dz * dz);

        if (dist > disaster.radius) continue;

        switch (disaster.disaster_type) {
            .earthquake => {
                if (disaster.intensity > 7.0 and dist < disaster.radius * 0.5) {
                    return true;
                }
            },
            .volcanic_eruption => {
                if (disaster.elapsed > 10 and dist < disaster.radius * 0.3) {
                    return true;
                }
            },
            .meteor_strike => {
                if (dist < disaster.radius) {
                    return true;
                }
            },
            else => {},
        }
    }
    return false;
}

pub fn updateDisasters(dt: f32) void {
    for (0..MAX_ACTIVE_DISASTERS) |i| {
        var disaster = &g_disaster_system.active_disasters[i];
        if (!disaster.active) continue;

        disaster.elapsed += dt;

        if (disaster.elapsed >= disaster.duration) {
            disaster.active = false;
            g_disaster_system.disaster_count -= 1;
        }

        switch (disaster.disaster_type) {
            .earthquake => {
                const progress = disaster.elapsed / disaster.duration;
                const wave_intensity = disaster.intensity * std.math.sin(progress * 3.14159 * 10);
                g_disaster_system.seismic_wave.amplitude = wave_intensity;
            },
            .hurricane => {
                const progress = disaster.elapsed / disaster.duration;
                g_disaster_system.atmosphere.wind_speed_max = disaster.intensity * 50 * (1.0 - progress * 0.3);
            },
            .wildfire => {
                disaster.radius += dt * disaster.intensity * 0.5;
            },
            else => {},
        }
    }
}

pub fn getGroundDisplacement(x: f32, z: f32, time: f32) struct { x: f32, y: f32, z: f32 } {
    var total_dx: f32 = 0;
    var total_dy: f32 = 0;
    var total_dz: f32 = 0;

    for (g_disaster_system.active_disasters[0..MAX_ACTIVE_DISASTERS]) |disaster| {
        if (!disaster.active or disaster.disaster_type != .earthquake) continue;

        const dx = x - disaster.epicenter_x;
        const dz = z - disaster.epicenter_z;
        const dist = @sqrt(dx * dx + dz * dz);
        const intensity = calculateSeismicIntensity(dist, disaster.intensity);

        const wave = std.math.sin(dist / 100.0 - time * disaster.intensity * 2.0) * intensity * 0.1;
        total_dx += dx / @max(dist, 0.1) * wave;
        total_dz += dz / @max(dist, 0.1) * wave;
        total_dy += std.math.sin(dist / 50.0 - time * disaster.intensity * 3.0) * intensity * 0.05;
    }

    return .{ .x = total_dx, .y = total_dy, .z = total_dz };
}

pub fn getWindVelocity(x: f32, z: f32) struct { x: f32, y: f32, z: f32 } {
    var wind_x: f32 = 0;
    var wind_z: f32 = 0;

    for (g_disaster_system.active_disasters[0..MAX_ACTIVE_DISASTERS]) |disaster| {
        if (!disaster.active) continue;

        const dx = x - disaster.epicenter_x;
        const dz = z - disaster.epicenter_z;
        const dist = @sqrt(dx * dx + dz * dz);

        if (disaster.disaster_type == .hurricane or disaster.disaster_type == .tornado) {
            const force = calculateWindForce(dist, disaster.intensity);
            const angle = std.math.atan2(dz, dx) + disaster.elapsed * 0.1;
            wind_x += std.math.cos(angle) * force;
            wind_z += std.math.sin(angle) * force;
        }
    }

    wind_x += g_disaster_system.atmosphere.wind_speed_max * std.math.sin(g_disaster_system.atmosphere.pressure_delta * 0.1);
    wind_z += g_disaster_system.atmosphere.wind_speed_max * std.math.cos(g_disaster_system.atmosphere.pressure_delta * 0.1);

    return .{ .x = wind_x, .y = 0, .z = wind_z };
}

pub fn enableChainReactions(enable: bool) void {
    g_disaster_system.chain_reaction_enabled = enable;
}

pub fn enableApocalypseMode(enable: bool) void {
    g_disaster_system.apocalypse_mode = enable;
}

pub fn getDisasterSystem() *DisasterSystem {
    return &g_disaster_system;
}
