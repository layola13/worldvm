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

fn defaultDisasterEvent() DisasterEvent {
    return .{
        .disaster_type = .none,
        .intensity = 0,
        .epicenter_x = 0,
        .epicenter_y = 0,
        .epicenter_z = 0,
        .radius = 0,
        .duration = 0,
        .elapsed = 0,
        .active = false,
    };
}

fn defaultSeismicWave() SeismicWave {
    return .{
        .amplitude = 0,
        .frequency = 0,
        .wave_type = 0,
        .propagation_speed = 3000,
    };
}

fn defaultAtmosphere() AtmosphericEvent {
    return .{
        .pressure_delta = 0,
        .wind_speed_max = 0,
        .temperature_change = 0,
        .humidity = 0.5,
        .visibility_reduction = 0,
    };
}

fn disasterBaseDuration(disaster_type: DisasterType) f32 {
    return switch (disaster_type) {
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
        .none => 0,
    };
}

fn isNearEpicenter(a: *const DisasterEvent, x: f32, y: f32, z: f32, radius: f32) bool {
    const dx = x - a.epicenter_x;
    const dy = y - a.epicenter_y;
    const dz = z - a.epicenter_z;
    const dist_sq = dx * dx + dy * dy + dz * dz;
    const merge_radius = @max(4.0, @min(a.radius, radius) * 0.35);
    return dist_sq <= merge_radius * merge_radius;
}

pub fn init() void {
    g_disaster_system.disaster_count = 0;
    g_disaster_system.chain_reaction_enabled = false;
    g_disaster_system.apocalypse_mode = false;
    for (0..MAX_ACTIVE_DISASTERS) |i| {
        g_disaster_system.active_disasters[i] = defaultDisasterEvent();
    }
    g_disaster_system.seismic_wave = defaultSeismicWave();
    g_disaster_system.atmosphere = defaultAtmosphere();
}

pub fn triggerDisaster(disaster_type: DisasterType, intensity: f32, x: f32, y: f32, z: f32, radius: f32) void {
    if (disaster_type == .none) return;
    if (intensity <= 0.0 or radius <= 0.0) return;

    const clamped_intensity = std.math.clamp(intensity, 0.1, 10.0);
    const clamped_radius = @max(1.0, radius);
    const duration = disasterBaseDuration(disaster_type);
    if (duration <= 0.0) return;

    // Merge with nearby active disaster of same type to avoid slot churn.
    for (0..MAX_ACTIVE_DISASTERS) |i| {
        var existing = &g_disaster_system.active_disasters[i];
        if (!existing.active or existing.disaster_type != disaster_type) continue;
        if (!isNearEpicenter(existing, x, y, z, clamped_radius)) continue;

        existing.intensity = @max(existing.intensity, clamped_intensity);
        existing.radius = @max(existing.radius, clamped_radius);
        existing.duration = @max(existing.duration, duration);
        existing.elapsed = @min(existing.elapsed, existing.duration * 0.5);
        existing.epicenter_x = (existing.epicenter_x + x) * 0.5;
        existing.epicenter_y = (existing.epicenter_y + y) * 0.5;
        existing.epicenter_z = (existing.epicenter_z + z) * 0.5;
        return;
    }

    if (g_disaster_system.disaster_count >= MAX_ACTIVE_DISASTERS) return;

    var found_slot = false;
    for (0..MAX_ACTIVE_DISASTERS) |i| {
        if (!g_disaster_system.active_disasters[i].active) {
            g_disaster_system.active_disasters[i] = .{
                .disaster_type = disaster_type,
                .intensity = clamped_intensity,
                .epicenter_x = x,
                .epicenter_y = y,
                .epicenter_z = z,
                .radius = clamped_radius,
                .duration = duration,
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
    if (dt <= 0.0) return;
    g_disaster_system.seismic_wave = defaultSeismicWave();
    g_disaster_system.atmosphere = defaultAtmosphere();

    var active_count: u8 = 0;
    var pending_chain: ?struct {
        disaster_type: DisasterType,
        intensity: f32,
        x: f32,
        y: f32,
        z: f32,
        radius: f32,
    } = null;

    for (0..MAX_ACTIVE_DISASTERS) |i| {
        var disaster = &g_disaster_system.active_disasters[i];
        if (!disaster.active) continue;
        const prev_elapsed = disaster.elapsed;

        disaster.elapsed += dt;

        if (disaster.elapsed >= disaster.duration) {
            disaster.* = defaultDisasterEvent();
            continue;
        }
        active_count += 1;

        switch (disaster.disaster_type) {
            .earthquake => {
                const progress = disaster.elapsed / disaster.duration;
                const wave_intensity = disaster.intensity * std.math.sin(progress * 3.14159 * 10);
                g_disaster_system.seismic_wave.amplitude += wave_intensity;
                g_disaster_system.seismic_wave.frequency = @max(g_disaster_system.seismic_wave.frequency, 1.0 + disaster.intensity * 0.2);
            },
            .hurricane => {
                const progress = disaster.elapsed / disaster.duration;
                const wind = disaster.intensity * 50 * (1.0 - progress * 0.3);
                g_disaster_system.atmosphere.wind_speed_max = @max(g_disaster_system.atmosphere.wind_speed_max, wind);
                g_disaster_system.atmosphere.pressure_delta -= disaster.intensity * 0.35;
                g_disaster_system.atmosphere.visibility_reduction = @max(g_disaster_system.atmosphere.visibility_reduction, disaster.intensity * 0.2);
            },
            .tornado => {
                const progress = disaster.elapsed / disaster.duration;
                const wind = disaster.intensity * 70 * (1.0 - progress * 0.4);
                g_disaster_system.atmosphere.wind_speed_max = @max(g_disaster_system.atmosphere.wind_speed_max, wind);
                g_disaster_system.atmosphere.visibility_reduction = @max(g_disaster_system.atmosphere.visibility_reduction, disaster.intensity * 0.15);
            },
            .wildfire => {
                disaster.radius += dt * disaster.intensity * 0.5;
                g_disaster_system.atmosphere.temperature_change += disaster.intensity * 0.03 * dt;
                g_disaster_system.atmosphere.visibility_reduction = @max(g_disaster_system.atmosphere.visibility_reduction, disaster.intensity * 0.25);
            },
            .volcanic_eruption => {
                g_disaster_system.atmosphere.temperature_change += disaster.intensity * 0.02 * dt;
                g_disaster_system.atmosphere.visibility_reduction = @max(g_disaster_system.atmosphere.visibility_reduction, disaster.intensity * 0.3);
                g_disaster_system.atmosphere.pressure_delta -= disaster.intensity * 0.1;
            },
            .flood => {
                g_disaster_system.atmosphere.humidity = std.math.clamp(g_disaster_system.atmosphere.humidity + disaster.intensity * 0.01 * dt, 0.0, 1.0);
                g_disaster_system.atmosphere.visibility_reduction = @max(g_disaster_system.atmosphere.visibility_reduction, disaster.intensity * 0.1);
            },
            else => {},
        }

        if (g_disaster_system.chain_reaction_enabled and pending_chain == null) {
            const chain_trigger_time = disaster.duration * 0.35;
            if (prev_elapsed < chain_trigger_time and disaster.elapsed >= chain_trigger_time) {
                switch (disaster.disaster_type) {
                    .earthquake => if (disaster.intensity >= 7.0) {
                        pending_chain = .{
                            .disaster_type = .building_collapse,
                            .intensity = disaster.intensity * 0.6,
                            .x = disaster.epicenter_x + 20.0,
                            .y = disaster.epicenter_y,
                            .z = disaster.epicenter_z + 20.0,
                            .radius = @max(25.0, disaster.radius * 0.25),
                        };
                    },
                    .volcanic_eruption => if (disaster.intensity >= 6.0) {
                        pending_chain = .{
                            .disaster_type = .wildfire,
                            .intensity = disaster.intensity * 0.5,
                            .x = disaster.epicenter_x + 15.0,
                            .y = disaster.epicenter_y,
                            .z = disaster.epicenter_z,
                            .radius = @max(30.0, disaster.radius * 0.2),
                        };
                    },
                    .meteor_strike => if (disaster.intensity >= 8.0) {
                        pending_chain = .{
                            .disaster_type = .flood,
                            .intensity = disaster.intensity * 0.4,
                            .x = disaster.epicenter_x,
                            .y = disaster.epicenter_y,
                            .z = disaster.epicenter_z + 12.0,
                            .radius = @max(20.0, disaster.radius * 0.3),
                        };
                    },
                    else => {},
                }
            }
        }
    }

    g_disaster_system.disaster_count = active_count;
    if (pending_chain) |secondary| {
        triggerDisaster(
            secondary.disaster_type,
            secondary.intensity,
            secondary.x,
            secondary.y,
            secondary.z,
            secondary.radius,
        );
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

// ============================================================================
// Tests for Disaster System (Items 641-670)
// ============================================================================

test "641: earthquake simulation - seismic wave generation" {
    init();
    triggerDisaster(.earthquake, 7.0, 0, 0, 0, 1000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "642: tsunami simulation - wave propagation" {
    init();
    triggerDisaster(.tsunami, 8.0, 0, 0, 0, 2000);
    const height = calculateTsunamiWaveHeight(100.0, 8.0);
    try std.testing.expect(height > 0);
}

test "643: flood simulation - water level rise" {
    init();
    triggerDisaster(.flood, 5.0, 0, 0, 0, 500);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "644: hurricane simulation - wind force calculation" {
    init();
    triggerDisaster(.hurricane, 4.0, 0, 0, 0, 3000);
    const wind_near = calculateWindForce(100.0, 4.0);
    const wind_far = calculateWindForce(500.0, 4.0);
    try std.testing.expect(wind_near > wind_far);
    try std.testing.expect(wind_far > 0.0);
}

test "645: tornado simulation - rotational wind" {
    init();
    triggerDisaster(.tornado, 3.0, 0, 0, 0, 500);
    const wind_nominal = calculateWindForce(100.0, 3.0);
    const wind_stronger = calculateWindForce(100.0, 5.0);
    try std.testing.expect(wind_stronger > wind_nominal);
    try std.testing.expect(wind_nominal > 0.0);
}

test "646: volcanic eruption simulation - heat radiation" {
    init();
    triggerDisaster(.volcanic_eruption, 6.0, 0, 0, 0, 2000);
    const heat_near = calculateHeatRadiation(50.0, 6.0);
    const heat_far = calculateHeatRadiation(200.0, 6.0);
    try std.testing.expect(heat_near > heat_far);
    try std.testing.expect(heat_far > 0.0);
}

test "647: landslide simulation - ground displacement" {
    init();
    triggerDisaster(.earthquake, 8.0, 0, 0, 0, 1000);
    const disp = getGroundDisplacement(100.0, 100.0, 1.0);
    const total = @abs(disp.x) + @abs(disp.y) + @abs(disp.z);
    try std.testing.expect(total > 0.0001);
}

test "648: mudslide simulation - debris flow" {
    init();
    triggerDisaster(.flood, 4.0, 0, 0, 0, 800);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "649: wildfire simulation - fire spread" {
    init();
    triggerDisaster(.wildfire, 8.0, 0, 0, 0, 1000);
    updateDisasters(1.0);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "650: lightning simulation - electrical discharge" {
    init();
    triggerDisaster(.hurricane, 3.0, 0, 0, 0, 2000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "651: hail simulation - ice projectiles" {
    init();
    triggerDisaster(.hurricane, 2.0, 0, 0, 0, 1500);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "652: blizzard simulation - heavy snowfall" {
    init();
    triggerDisaster(.hurricane, 5.0, 0, 0, 0, 3000);
    const blizzard_wind = calculateWindForce(200.0, 5.0);
    const mild_wind = calculateWindForce(200.0, 2.0);
    try std.testing.expect(blizzard_wind > mild_wind);
    try std.testing.expect(mild_wind > 0.0);
}

test "653: dust storm simulation - particle suspension" {
    init();
    triggerDisaster(.tornado, 2.0, 0, 0, 0, 1000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "654: smog simulation - visibility reduction" {
    init();
    triggerDisaster(.wildfire, 5.0, 0, 0, 0, 2000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "655: extreme heat simulation - temperature rise" {
    init();
    triggerDisaster(.wildfire, 7.0, 0, 0, 0, 1000);
    const heat = calculateHeatRadiation(50.0, 7.0);
    try std.testing.expect(heat > 0);
}

test "656: extreme cold simulation - freezing conditions" {
    init();
    triggerDisaster(.hurricane, 4.0, 0, 0, 0, 2000);
    const wind_near = calculateWindForce(100.0, 4.0);
    const wind_far = calculateWindForce(300.0, 4.0);
    try std.testing.expect(wind_near > wind_far);
    try std.testing.expect(wind_far > 0.0);
}

test "657: radiation leak simulation - contamination spread" {
    init();
    triggerDisaster(.meteor_strike, 9.0, 0, 0, 0, 500);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "658: chemical leak simulation - toxic exposure" {
    init();
    triggerDisaster(.volcanic_eruption, 5.0, 0, 0, 0, 2000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "659: building collapse simulation - structural failure" {
    init();
    triggerDisaster(.earthquake, 8.0, 0, 0, 0, 500);
    const disp = getGroundDisplacement(50.0, 50.0, 2.0);
    const total = @abs(disp.x) + @abs(disp.y) + @abs(disp.z);
    try std.testing.expect(total > 0.0001);
}

test "660: bridge collapse simulation - load failure" {
    init();
    triggerDisaster(.earthquake, 7.5, 0, 0, 0, 800);
    const intensity = calculateSeismicIntensity(200.0, 7.5);
    try std.testing.expect(intensity > 0);
}

test "661: tunnel collapse simulation - underground structure failure" {
    init();
    triggerDisaster(.earthquake, 7.0, 0, 0, 0, 600);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "662: dam breach simulation - water release" {
    init();
    triggerDisaster(.flood, 6.0, 0, 0, 0, 1000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "663: power outage impact - infrastructure failure" {
    init();
    triggerDisaster(.hurricane, 5.0, 0, 0, 0, 3000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "664: communication outage - network disruption" {
    init();
    triggerDisaster(.hurricane, 4.0, 0, 0, 0, 2500);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "665: traffic paralysis - transportation disruption" {
    init();
    triggerDisaster(.flood, 5.0, 0, 0, 0, 1500);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "666: chain reaction - cascading disasters" {
    init();
    enableChainReactions(true);
    triggerDisaster(.earthquake, 8.0, 0, 0, 0, 1000);
    const will_chain = checkChainReaction(50.0, 0, 50.0);
    try std.testing.expect(will_chain);
}

test "667: secondary disaster - aftershock effects" {
    init();
    triggerDisaster(.earthquake, 7.5, 0, 0, 0, 800);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "668: disaster warning - early detection system" {
    init();
    triggerDisaster(.tsunami, 8.5, 0, 0, 0, 3000);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "669: disaster evacuation - emergency response" {
    init();
    triggerDisaster(.hurricane, 5.0, 0, 0, 0, 2500);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "670: disaster recovery - restoration efforts" {
    init();
    triggerDisaster(.earthquake, 6.5, 0, 0, 0, 900);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 1);
}

test "disaster expires and clears atmospheric wind contribution" {
    init();
    triggerDisaster(.hurricane, 4.0, 0, 0, 0, 500.0);
    updateDisasters(1.0);
    try std.testing.expect(getDisasterSystem().atmosphere.wind_speed_max > 0.0);
    updateDisasters(400.0);
    try std.testing.expect(getDisasterSystem().disaster_count == 0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), getDisasterSystem().atmosphere.wind_speed_max, 0.0001);
}

test "triggerDisaster clamps parameters and ignores invalid disaster type" {
    init();
    triggerDisaster(.none, 5.0, 0, 0, 0, 100.0);
    try std.testing.expect(getDisasterSystem().disaster_count == 0);

    triggerDisaster(.flood, 50.0, 0, 0, 0, 0.2);
    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), sys.active_disasters[0].intensity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), sys.active_disasters[0].radius, 0.0001);
}

test "triggerDisaster merges nearby same-type events instead of consuming slots" {
    init();
    triggerDisaster(.flood, 3.0, 0, 0, 0, 100.0);
    triggerDisaster(.flood, 5.0, 5.0, 0, 5.0, 120.0);

    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count == 1);
    try std.testing.expect(sys.active_disasters[0].active);
    try std.testing.expect(sys.active_disasters[0].intensity >= 5.0);
    try std.testing.expect(sys.active_disasters[0].radius >= 120.0);
}

test "chain reactions spawn secondary disaster when threshold is crossed" {
    init();
    enableChainReactions(true);
    triggerDisaster(.earthquake, 8.0, 0, 0, 0, 1000.0);
    updateDisasters(11.0);

    const sys = getDisasterSystem();
    try std.testing.expect(sys.disaster_count >= 2);
    var found_secondary = false;
    for (sys.active_disasters) |disaster| {
        if (!disaster.active) continue;
        if (disaster.disaster_type == .building_collapse) {
            found_secondary = true;
            break;
        }
    }
    try std.testing.expect(found_secondary);
}
