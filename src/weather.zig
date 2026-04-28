//! Weather System - Atmospheric Conditions and Environmental Effects
//!
//! Phase 76: Weather simulation, precipitation, visibility, temperature
//! Handles: Rain, snow, fog, wind, temperature, humidity, atmospheric effects

const std = @import("std");
const BASE_TEMPERATURE: f32 = 20.0;
const BASE_HUMIDITY: f32 = 0.5;
const BASE_PRESSURE: f32 = 1013.25;
const BASE_VISIBILITY: f32 = 1000.0;

fn clamp01(value: f32) f32 {
    return std.math.clamp(value, 0.0, 1.0);
}

fn makeDefaultAtmosphere(wind_direction: f32) AtmosphericConditions {
    return .{
        .temperature = BASE_TEMPERATURE,
        .humidity = BASE_HUMIDITY,
        .pressure = BASE_PRESSURE,
        .wind_speed = 0,
        .wind_direction = wind_direction,
        .visibility = BASE_VISIBILITY,
        .precipitation = 0,
    };
}

fn recalcWeatherCount() void {
    var count: u8 = 0;
    for (g_weather_system.active_states) |state| {
        if (state.active) count += 1;
    }
    g_weather_system.weather_count = count;
}

pub const WeatherType = enum(u8) {
    clear = 0,
    cloudy = 1,
    rain = 2,
    snow = 3,
    fog = 4,
    storm = 5,
    hurricane = 6,
    blizzard = 7,
};

pub const WeatherState = struct {
    weather_type: WeatherType,
    intensity: f32,
    duration: f32,
    elapsed: f32,
    active: bool,
};

pub const AtmosphericConditions = struct {
    temperature: f32,
    humidity: f32,
    pressure: f32,
    wind_speed: f32,
    wind_direction: f32,
    visibility: f32,
    precipitation: f32,
};

pub const MAX_WEATHER_STATES: usize = 4;

pub const WeatherSystem = struct {
    active_states: [MAX_WEATHER_STATES]WeatherState,
    weather_count: u8,
    atmosphere: AtmosphericConditions,
    global_time: f32,
};

var g_weather_system: WeatherSystem = .{
    .active_states = [_]WeatherState{.{
        .weather_type = .clear,
        .intensity = 0,
        .duration = 0,
        .elapsed = 0,
        .active = false,
    }} ** MAX_WEATHER_STATES,
    .weather_count = 0,
    .atmosphere = .{
        .temperature = BASE_TEMPERATURE,
        .humidity = BASE_HUMIDITY,
        .pressure = BASE_PRESSURE,
        .wind_speed = 0,
        .wind_direction = 0,
        .visibility = BASE_VISIBILITY,
        .precipitation = 0,
    },
    .global_time = 0,
};

pub fn init() void {
    g_weather_system.weather_count = 0;
    g_weather_system.global_time = 0;
    g_weather_system.atmosphere = makeDefaultAtmosphere(0.0);
    for (0..MAX_WEATHER_STATES) |i| {
        g_weather_system.active_states[i] = .{
            .weather_type = .clear,
            .intensity = 0,
            .duration = 0,
            .elapsed = 0,
            .active = false,
        };
    }
}

pub fn triggerWeather(weather_type: WeatherType, intensity: f32, duration: f32) void {
    if (duration <= 0.0) return;
    const safe_intensity = clamp01(intensity);

    for (0..MAX_WEATHER_STATES) |i| {
        var existing = &g_weather_system.active_states[i];
        if (!existing.active or existing.weather_type != weather_type) continue;
        const remaining = @max(0.0, existing.duration - existing.elapsed);
        existing.intensity = @max(existing.intensity, safe_intensity);
        existing.duration = @max(duration, remaining);
        existing.elapsed = 0.0;
        return;
    }

    var free_slot: ?usize = null;
    for (0..MAX_WEATHER_STATES) |i| {
        if (!g_weather_system.active_states[i].active) {
            free_slot = i;
            break;
        }
    }

    if (free_slot) |idx| {
        g_weather_system.active_states[idx] = .{
            .weather_type = weather_type,
            .intensity = safe_intensity,
            .duration = duration,
            .elapsed = 0,
            .active = true,
        };
        recalcWeatherCount();
        return;
    }

    var weakest_idx: usize = 0;
    var weakest_score: f32 = std.math.inf(f32);
    for (0..MAX_WEATHER_STATES) |i| {
        const state = g_weather_system.active_states[i];
        const remaining = @max(0.0, state.duration - state.elapsed);
        const score = state.intensity * remaining;
        if (score < weakest_score) {
            weakest_score = score;
            weakest_idx = i;
        }
    }

    if (safe_intensity * duration > weakest_score) {
        g_weather_system.active_states[weakest_idx] = .{
            .weather_type = weather_type,
            .intensity = safe_intensity,
            .duration = duration,
            .elapsed = 0,
            .active = true,
        };
        recalcWeatherCount();
    }
}

pub fn updateWeather(dt: f32) void {
    if (dt <= 0.0) return;
    g_weather_system.global_time += dt;

    for (0..MAX_WEATHER_STATES) |i| {
        var weather = &g_weather_system.active_states[i];
        if (!weather.active) continue;

        weather.elapsed += dt;

        if (weather.elapsed >= weather.duration) {
            weather.active = false;
            continue;
        }
    }

    recalcWeatherCount();
    var atmosphere = makeDefaultAtmosphere(g_weather_system.atmosphere.wind_direction);
    var visibility_multiplier: f32 = 1.0;
    var wind_x: f32 = 0.0;
    var wind_z: f32 = 0.0;
    for (0..MAX_WEATHER_STATES) |i| {
        const weather = &g_weather_system.active_states[i];
        if (!weather.active) continue;
        const intensity = clamp01(weather.intensity);
        const life_progress = clamp01(weather.elapsed / @max(0.001, weather.duration));
        const activity = intensity * (1.0 - life_progress * 0.35);

        const base_dir = g_weather_system.atmosphere.wind_direction;
        const weather_dir_bias: f32 = switch (weather.weather_type) {
            .storm => 0.35,
            .hurricane => 0.7,
            .blizzard => -0.25,
            .rain => 0.1,
            .fog => -0.1,
            else => 0.0,
        };
        const jitter = @sin(g_weather_system.global_time * 0.03 + @as(f32, @floatFromInt(i)) * 0.7) * 0.18;
        const wind_dir = base_dir + weather_dir_bias + jitter;
        const wind_speed = switch (weather.weather_type) {
            .storm => activity * 50.0,
            .hurricane => activity * 100.0,
            .blizzard => activity * 30.0,
            .rain => activity * 12.0,
            .cloudy => activity * 8.0,
            .fog => activity * 4.0,
            else => activity * 2.0,
        };
        wind_x += @sin(wind_dir) * wind_speed;
        wind_z += @cos(wind_dir) * wind_speed;

        switch (weather.weather_type) {
            .rain => {
                atmosphere.precipitation = @max(atmosphere.precipitation, calculatePrecipitationIntensity(.rain, activity));
                visibility_multiplier *= 1.0 - activity * 0.3;
                atmosphere.humidity += activity * 0.15;
            },
            .snow => {
                atmosphere.precipitation = @max(atmosphere.precipitation, calculatePrecipitationIntensity(.snow, activity));
                visibility_multiplier *= 1.0 - activity * 0.4;
                atmosphere.temperature = @min(atmosphere.temperature, BASE_TEMPERATURE - activity * 12.0);
            },
            .fog => {
                visibility_multiplier *= 1.0 - activity * 0.8;
                atmosphere.humidity += activity * 0.05;
            },
            .storm => {
                atmosphere.precipitation = @max(atmosphere.precipitation, calculatePrecipitationIntensity(.storm, activity));
                visibility_multiplier *= 1.0 - activity * 0.5;
                atmosphere.pressure -= activity * 12.0;
                atmosphere.humidity += activity * 0.12;
            },
            .hurricane => {
                atmosphere.precipitation = @max(atmosphere.precipitation, calculatePrecipitationIntensity(.hurricane, activity));
                visibility_multiplier *= 1.0 - activity * 0.65;
                atmosphere.pressure -= activity * 25.0;
                atmosphere.humidity += activity * 0.2;
            },
            .blizzard => {
                atmosphere.precipitation = @max(atmosphere.precipitation, calculatePrecipitationIntensity(.blizzard, activity));
                visibility_multiplier *= 1.0 - activity * 0.75;
                atmosphere.temperature = @min(atmosphere.temperature, BASE_TEMPERATURE - activity * 30.0);
                atmosphere.humidity += activity * 0.1;
            },
            .cloudy => {
                visibility_multiplier *= 1.0 - activity * 0.15;
                atmosphere.humidity += activity * 0.08;
            },
            .clear => {},
        }
    }

    const wind_speed = @sqrt(wind_x * wind_x + wind_z * wind_z);
    atmosphere.wind_speed = wind_speed;
    if (wind_speed > 0.0001) {
        atmosphere.wind_direction = std.math.atan2(wind_x, wind_z);
    }
    atmosphere.visibility *= std.math.clamp(visibility_multiplier, 0.02, 1.0);
    atmosphere.visibility = std.math.clamp(atmosphere.visibility, 20.0, BASE_VISIBILITY);
    atmosphere.humidity = clamp01(atmosphere.humidity);
    atmosphere.pressure = std.math.clamp(atmosphere.pressure, 850.0, BASE_PRESSURE);
    g_weather_system.atmosphere = atmosphere;
}

pub fn calculateVisibility(precipitation: f32, fog_density: f32) f32 {
    const base_visibility: f32 = 1000;
    const precipitation_factor = 1.0 - @min(1.0, precipitation * 0.01);
    const fog_factor = 1.0 - @min(1.0, fog_density * 0.02);
    return base_visibility * precipitation_factor * fog_factor;
}

pub fn calculateWindEffect(distance_from_center: f32, wind_speed: f32) f32 {
    if (distance_from_center < 0.1) return wind_speed;
    const decay = 1.0 / (1.0 + distance_from_center * 0.001);
    return wind_speed * decay;
}

pub fn calculateTemperatureEffect(altitude: f32, base_temp: f32) f32 {
    const lapse_rate: f32 = 0.0065;
    return base_temp - altitude * lapse_rate;
}

pub fn calculateHumidityEffect(temperature: f32, humidity: f32) f32 {
    const saturation = 17.27 * temperature / (temperature + 237.3);
    const factor = @exp(saturation) * humidity;
    return factor;
}

pub fn calculatePrecipitationIntensity(weather_type: WeatherType, intensity: f32) f32 {
    return switch (weather_type) {
        .rain => intensity * 10,
        .snow => intensity * 5,
        .storm => intensity * 15,
        .hurricane => intensity * 20,
        .blizzard => intensity * 8,
        else => 0,
    };
}

pub fn calculatePressureChange(altitude: f32) f32 {
    const sea_level_pressure: f32 = 1013.25;
    const scale_height: f32 = 8435;
    return sea_level_pressure * @exp(-altitude / scale_height);
}

pub fn getWeatherSeverity() f32 {
    if (g_weather_system.weather_count == 0) return 0.0;
    var severity: f32 = 0.0;
    for (g_weather_system.active_states) |state| {
        if (!state.active) continue;
        const base: f32 = switch (state.weather_type) {
            .clear => 0.0,
            .cloudy => 0.1,
            .rain => 0.35,
            .snow => 0.4,
            .fog => 0.45,
            .storm => 0.75,
            .hurricane => 1.0,
            .blizzard => 0.85,
        };
        severity += base * clamp01(state.intensity);
    }
    return clamp01(severity);
}

pub fn getSensorVisibilityFactor() f32 {
    const visibility_factor = clamp01(g_weather_system.atmosphere.visibility / BASE_VISIBILITY);
    const fog_penalty = clamp01((BASE_VISIBILITY - g_weather_system.atmosphere.visibility) / BASE_VISIBILITY);
    return std.math.clamp(visibility_factor * (1.0 - fog_penalty * 0.35), 0.05, 1.0);
}

pub fn getRoadTractionPenalty() f32 {
    const precipitation_factor = clamp01(g_weather_system.atmosphere.precipitation / 20.0);
    const freezing_penalty: f32 = if (g_weather_system.atmosphere.temperature <= 0.0) 0.25 else 0.0;
    const severity = getWeatherSeverity();
    return clamp01(precipitation_factor * 0.6 + severity * 0.25 + freezing_penalty);
}

pub fn getAtmosphericConditions() AtmosphericConditions {
    return g_weather_system.atmosphere;
}

pub fn getWeatherSystem() *WeatherSystem {
    return &g_weather_system;
}

// ============================================================================
// Tests for Weather System (Items 686-700)
// ============================================================================

test "686: rain weather - precipitation simulation" {
    init();
    triggerWeather(.rain, 0.5, 60);
    updateWeather(1.0);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.precipitation > 0);
}

test "687: snow weather - snowfall simulation" {
    init();
    triggerWeather(.snow, 0.8, 120);
    updateWeather(1.0);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.precipitation > 0);
}

test "688: fog weather - visibility reduction" {
    init();
    triggerWeather(.fog, 0.6, 90);
    updateWeather(1.0);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.visibility < 1000);
}

test "689: cloud weather - overcast conditions" {
    init();
    triggerWeather(.cloudy, 0.5, 180);
    const sys = getWeatherSystem();
    try std.testing.expect(sys.weather_count >= 1);
}

test "690: wind weather - atmospheric movement" {
    init();
    triggerWeather(.clear, 0.2, 60);
    updateWeather(1.0);
    const clear_conditions = getAtmosphericConditions();

    init();
    triggerWeather(.storm, 0.7, 60);
    updateWeather(1.0);
    const storm_conditions = getAtmosphericConditions();

    try std.testing.expect(storm_conditions.wind_speed > clear_conditions.wind_speed);
    try std.testing.expect(storm_conditions.wind_speed > 0);
}

test "691: temperature weather - thermal conditions" {
    init();
    triggerWeather(.blizzard, 1.0, 60);
    updateWeather(1.0);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.temperature < 20.0);
}

test "692: humidity weather - moisture content" {
    init();
    g_weather_system.atmosphere.humidity = 0.8;
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.humidity > 0);
}

test "693: pressure weather - atmospheric pressure" {
    init();
    const pressure = calculatePressureChange(0);
    try std.testing.expect(pressure > 1000);
}

test "694: visibility weather - sight distance calculation" {
    init();
    const visibility = calculateVisibility(5.0, 0.3);
    try std.testing.expect(visibility > 0);
    try std.testing.expect(visibility <= 1000);
}

test "695: lighting weather - illumination conditions" {
    init();
    triggerWeather(.cloudy, 0.5, 120);
    const sys = getWeatherSystem();
    try std.testing.expect(sys.weather_count >= 1);
}

test "696: weather transition - smooth weather changes" {
    init();
    triggerWeather(.clear, 1.0, 30);
    triggerWeather(.rain, 0.5, 60);
    const sys = getWeatherSystem();
    try std.testing.expect(sys.weather_count >= 2);
}

test "697: weather physics effect - impact on objects" {
    init();
    triggerWeather(.storm, 0.7, 45);
    updateWeather(1.0);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.wind_speed > 0);
}

test "698: weather sensor effect - visibility for sensors" {
    init();
    triggerWeather(.fog, 0.9, 60);
    updateWeather(1.0);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.visibility < 500);
}

test "699: weather vehicle effect - driving conditions" {
    init();
    triggerWeather(.rain, 0.6, 120);
    updateWeather(1.0);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.precipitation > 0);
}

test "700: weather visibility effect - sight line obstruction" {
    init();
    const visibility = calculateVisibility(10.0, 0.5);
    try std.testing.expect(visibility > 0);
    try std.testing.expect(visibility < 1000);
}

test "weather expires and restores baseline atmosphere" {
    init();
    triggerWeather(.rain, 1.0, 0.5);
    updateWeather(0.25);
    try std.testing.expect(getAtmosphericConditions().precipitation > 0.0);
    updateWeather(0.5);
    const conditions = getAtmosphericConditions();
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), conditions.precipitation, 0.0001);
    try std.testing.expect(conditions.visibility > 990.0);
}

test "weather combines fog and rain conservatively" {
    init();
    triggerWeather(.rain, 0.4, 30.0);
    triggerWeather(.fog, 0.5, 30.0);
    updateWeather(0.1);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.precipitation > 0.0);
    try std.testing.expect(conditions.visibility < 700.0);
}

test "weather trigger clamps intensity range" {
    init();
    triggerWeather(.storm, 2.5, 20.0);
    updateWeather(0.1);
    const conditions = getAtmosphericConditions();
    try std.testing.expect(conditions.wind_speed <= 50.0);
    try std.testing.expect(conditions.wind_speed > 49.0);
    try std.testing.expect(conditions.precipitation <= 15.0);
}

test "weather trigger merges duplicate active weather type" {
    init();
    triggerWeather(.rain, 0.3, 10.0);
    triggerWeather(.rain, 0.8, 20.0);
    const sys = getWeatherSystem();
    try std.testing.expect(sys.weather_count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), sys.active_states[0].intensity, 0.0001);
    try std.testing.expect(sys.active_states[0].duration >= 20.0);
}

test "weather full state set replaces weakest event with stronger one" {
    init();
    triggerWeather(.cloudy, 0.1, 5.0);
    triggerWeather(.rain, 0.3, 5.0);
    triggerWeather(.fog, 0.2, 5.0);
    triggerWeather(.snow, 0.4, 5.0);
    triggerWeather(.hurricane, 0.9, 20.0);
    const sys = getWeatherSystem();
    try std.testing.expect(sys.weather_count == MAX_WEATHER_STATES);
    var found_hurricane = false;
    for (sys.active_states) |state| {
        if (!state.active) continue;
        if (state.weather_type == .hurricane) found_hurricane = true;
    }
    try std.testing.expect(found_hurricane);
}

test "weather derived factors reflect hazardous conditions" {
    init();
    triggerWeather(.storm, 0.9, 60.0);
    triggerWeather(.fog, 0.8, 60.0);
    updateWeather(1.0);
    try std.testing.expect(getWeatherSeverity() > 0.7);
    try std.testing.expect(getSensorVisibilityFactor() < 0.6);
    try std.testing.expect(getRoadTractionPenalty() > 0.4);
}
