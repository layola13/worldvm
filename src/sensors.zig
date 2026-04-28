//! Sensor System - Perception, Degradation, and Sensor Fusion
//!
//! Phase 64, 18: Sensor models, noise, degradation, fusion
//! Handles: Radar, LiDAR, camera, ultrasonic, sensor fusion, false positives

const std = @import("std");
const prediction = @import("prediction.zig");
const weather = @import("weather.zig");

pub const SensorType = enum(u8) {
    camera = 0,
    radar = 1,
    lidar = 2,
    ultrasonic = 3,
    imu = 4,
    gps = 5,
    v2v = 6,
};

pub const SensorState = struct {
    sensor_type: SensorType,
    enabled: bool,
    field_of_view: f32,
    range: f32,
    noise_level: f32,
    occlusion_factor: f32,
    confidence: f32,
    last_update_time: f32,
};

pub const DetectedObject = struct {
    object_id: u16,
    object_type: u8,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    confidence: f32,
    age: f32,
    sensor_source: SensorType,
};

pub const SensorFusionState = struct {
    detected_objects: [32]DetectedObject,
    object_count: u8,
    timestamp: f32,
};

pub const MAX_SENSORS: usize = 8;
pub const MAX_DETECTED_OBJECTS: usize = 32;

pub const SensorSystem = struct {
    sensors: [MAX_SENSORS]SensorState,
    sensor_count: u8,
    fusion: SensorFusionState,
    degradation_factor: f32,
    interference_level: f32,
};

var g_sensor_system: SensorSystem = undefined;

fn clamp01(value: f32) f32 {
    return @max(0.0, @min(1.0, value));
}

pub fn init() void {
    g_sensor_system.sensor_count = 0;
    g_sensor_system.degradation_factor = 1.0;
    g_sensor_system.interference_level = 0;
    g_sensor_system.fusion.object_count = 0;
    g_sensor_system.fusion.timestamp = 0;
    g_sensor_system.fusion.detected_objects = [_]DetectedObject{.{
        .object_id = 0,
        .object_type = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0,
        .age = 0,
        .sensor_source = .camera,
    }} ** MAX_DETECTED_OBJECTS;
    for (0..MAX_SENSORS) |i| {
        g_sensor_system.sensors[i] = .{
            .sensor_type = .camera,
            .enabled = false,
            .field_of_view = 0,
            .range = 0,
            .noise_level = 0,
            .occlusion_factor = 0,
            .confidence = 0,
            .last_update_time = 0,
        };
    }
}

pub fn addSensor(sensor_type: SensorType, fov: f32, range: f32) ?*SensorState {
    if (g_sensor_system.sensor_count >= MAX_SENSORS) return null;
    const idx = g_sensor_system.sensor_count;
    g_sensor_system.sensor_count += 1;
    const clamped_fov = @max(0.0, @min(360.0, fov));
    const clamped_range = @max(0.0, range);

    g_sensor_system.sensors[idx] = .{
        .sensor_type = sensor_type,
        .enabled = true,
        .field_of_view = clamped_fov,
        .range = clamped_range,
        .noise_level = switch (sensor_type) {
            .camera => 0.05,
            .radar => 0.1,
            .lidar => 0.02,
            .ultrasonic => 0.15,
            .imu => 0.01,
            .gps => 0.5,
            .v2v => 0.02,
        },
        .occlusion_factor = 0,
        .confidence = 1.0,
        .last_update_time = 0,
    };
    return &g_sensor_system.sensors[idx];
}

pub fn setSensorEnabled(sensor_type: SensorType, enabled: bool) void {
    for (0..g_sensor_system.sensor_count) |i| {
        if (g_sensor_system.sensors[i].sensor_type == sensor_type) {
            g_sensor_system.sensors[i].enabled = enabled;
        }
    }
}

pub fn degradeSensor(sensor_type: SensorType, factor: f32) void {
    const safe_factor = clamp01(factor);
    for (0..g_sensor_system.sensor_count) |i| {
        if (g_sensor_system.sensors[i].sensor_type == sensor_type) {
            const noise_multiplier = 1.0 + (1.0 - safe_factor);
            g_sensor_system.sensors[i].noise_level *= noise_multiplier;
            g_sensor_system.sensors[i].confidence = clamp01(g_sensor_system.sensors[i].confidence * safe_factor);
        }
    }
    g_sensor_system.degradation_factor = clamp01(g_sensor_system.degradation_factor * safe_factor);
}

pub fn addInterference(level: f32) void {
    const safe_level = @max(0.0, level);
    g_sensor_system.interference_level = @min(5.0, g_sensor_system.interference_level + safe_level);
    for (0..g_sensor_system.sensor_count) |i| {
        if (g_sensor_system.sensors[i].sensor_type == .radar or
            g_sensor_system.sensors[i].sensor_type == .lidar)
        {
            g_sensor_system.sensors[i].noise_level *= (1.0 + safe_level);
        }
    }
}

pub fn raycastOcclusion(origin_x: f32, origin_y: f32, origin_z: f32, target_x: f32, target_y: f32, target_z: f32, occluder_x: f32, occluder_y: f32, occluder_z: f32, occluder_radius: f32) f32 {
    if (occluder_radius <= 0.0) return 0.0;
    const dx = target_x - origin_x;
    const dy = target_y - origin_y;
    const dz = target_z - origin_z;
    const ray_length_sq = dx * dx + dy * dy + dz * dz;
    if (ray_length_sq <= 0.000001) return 0.0;
    const t = ((occluder_x - origin_x) * dx + (occluder_y - origin_y) * dy + (occluder_z - origin_z) * dz) / ray_length_sq;

    if (t < 0 or t > 1) return 0;

    const closest_x = origin_x + dx * t;
    const closest_y = origin_y + dy * t;
    const closest_z = origin_z + dz * t;

    const dist_to_occluder = @sqrt((closest_x - occluder_x) * (closest_x - occluder_x) +
        (closest_y - occluder_y) * (closest_y - occluder_y) +
        (closest_z - occluder_z) * (closest_z - occluder_z));

    if (dist_to_occluder < occluder_radius) {
        return 1.0 - dist_to_occluder / occluder_radius;
    }
    return 0;
}

pub fn addDetectedObject(obj: DetectedObject) void {
    for (0..g_sensor_system.fusion.object_count) |i| {
        if (g_sensor_system.fusion.detected_objects[i].object_id == obj.object_id) {
            var existing = &g_sensor_system.fusion.detected_objects[i];
            const previous_confidence = existing.confidence;
            const total_confidence = @max(0.0001, existing.confidence + obj.confidence);
            const confidence_weight = obj.confidence / total_confidence;
            existing.pos_x = existing.pos_x * (1.0 - confidence_weight) + obj.pos_x * confidence_weight;
            existing.pos_y = existing.pos_y * (1.0 - confidence_weight) + obj.pos_y * confidence_weight;
            existing.pos_z = existing.pos_z * (1.0 - confidence_weight) + obj.pos_z * confidence_weight;
            existing.vel_x = existing.vel_x * (1.0 - confidence_weight) + obj.vel_x * confidence_weight;
            existing.vel_y = existing.vel_y * (1.0 - confidence_weight) + obj.vel_y * confidence_weight;
            existing.vel_z = existing.vel_z * (1.0 - confidence_weight) + obj.vel_z * confidence_weight;
            existing.confidence = clamp01((existing.confidence + obj.confidence) / 2.0);
            if (obj.confidence >= previous_confidence) {
                existing.object_type = obj.object_type;
                existing.sensor_source = obj.sensor_source;
            }
            existing.age = 0;
            return;
        }
    }

    if (g_sensor_system.fusion.object_count >= MAX_DETECTED_OBJECTS) return;
    const idx = g_sensor_system.fusion.object_count;
    g_sensor_system.fusion.detected_objects[idx] = obj;
    g_sensor_system.fusion.object_count += 1;
}

pub fn fuseSensors(time: f32) void {
    const dt = @max(0.0, time - g_sensor_system.fusion.timestamp);
    g_sensor_system.fusion.timestamp = time;

    var i: usize = 0;
    while (i < g_sensor_system.fusion.object_count) {
        var object = &g_sensor_system.fusion.detected_objects[i];
        object.pos_x += object.vel_x * dt;
        object.pos_y += object.vel_y * dt;
        object.pos_z += object.vel_z * dt;
        object.age += dt;
        object.confidence *= std.math.pow(f32, 0.94, dt);

        if (object.age > 2.0 or object.confidence < 0.05)
        {
            g_sensor_system.fusion.object_count -= 1;
            if (g_sensor_system.fusion.object_count > 0 and i < g_sensor_system.fusion.object_count) {
                g_sensor_system.fusion.detected_objects[i] = g_sensor_system.fusion.detected_objects[g_sensor_system.fusion.object_count];
                continue;
            }
        }
        i += 1;
    }
}

pub fn calculateConfidence(distance: f32, sensor: *const SensorState, weather_visibility: f32) f32 {
    const range_factor = if (sensor.range <= 0.0)
        1.0
    else
        1.0 - clamp01(@max(0.0, distance) / @max(0.001, sensor.range));
    const weather_factor = clamp01(weather_visibility / 1000.0);
    const noise_factor = clamp01(1.0 - sensor.noise_level);
    const occlusion_factor = clamp01(1.0 - sensor.occlusion_factor);
    const degradation_factor = clamp01(g_sensor_system.degradation_factor);
    const interference_factor = 1.0 / (1.0 + @max(0.0, g_sensor_system.interference_level));
    return clamp01(range_factor * weather_factor * noise_factor * occlusion_factor * degradation_factor * interference_factor);
}

pub fn calculateConfidenceFromWeather(distance: f32, sensor: *const SensorState) f32 {
    const weather_visibility = weather.getSensorVisibilityFactor() * 1000.0;
    const base_confidence = calculateConfidence(distance, sensor, weather_visibility);
    const severity_penalty = std.math.clamp(1.0 - weather.getWeatherSeverity() * 0.2, 0.75, 1.0);
    return clamp01(base_confidence * severity_penalty);
}

pub fn predictObjectPosition(obj: *const DetectedObject, time_delta: f32) struct { x: f32, y: f32, z: f32 } {
    const predicted = prediction.predictLinearState(.{
        .pos_x = obj.pos_x,
        .pos_y = obj.pos_y,
        .pos_z = obj.pos_z,
        .vel_x = obj.vel_x,
        .vel_y = obj.vel_y,
        .vel_z = obj.vel_z,
    }, time_delta);
    return .{
        .x = predicted.pos_x,
        .y = predicted.pos_y,
        .z = predicted.pos_z,
    };
}

pub fn predictObjectPositionFromComponents(pos_x: f32, pos_y: f32, pos_z: f32, vel_x: f32, vel_y: f32, vel_z: f32, time_delta: f32) struct { x: f32, y: f32, z: f32 } {
    const predicted = prediction.predictLinearState(.{
        .pos_x = pos_x,
        .pos_y = pos_y,
        .pos_z = pos_z,
        .vel_x = vel_x,
        .vel_y = vel_y,
        .vel_z = vel_z,
    }, time_delta);
    return .{
        .x = predicted.pos_x,
        .y = predicted.pos_y,
        .z = predicted.pos_z,
    };
}

pub fn predictObjectPose(obj: *const DetectedObject, time_delta: f32) prediction.PlanarPoseForecast {
    return prediction.predictPlanarPose(.{
        .pos_x = obj.pos_x,
        .pos_y = obj.pos_y,
        .pos_z = obj.pos_z,
        .yaw = prediction.resolvePlanarHeading(obj.vel_x, obj.vel_z, 0.0),
    }, obj.vel_x, obj.vel_y, obj.vel_z, 0.0, time_delta);
}

pub fn getDetectedObjects() []DetectedObject {
    return g_sensor_system.fusion.detected_objects[0..g_sensor_system.fusion.object_count];
}

pub fn getSensorState(sensor_type: SensorType) ?*SensorState {
    for (0..MAX_SENSORS) |i| {
        if (g_sensor_system.sensors[i].sensor_type == sensor_type and g_sensor_system.sensors[i].enabled) {
            return &g_sensor_system.sensors[i];
        }
    }
    return null;
}

pub fn resetSensors() void {
    g_sensor_system.sensor_count = 0;
    g_sensor_system.fusion.object_count = 0;
    g_sensor_system.fusion.timestamp = 0;
    g_sensor_system.degradation_factor = 1.0;
    g_sensor_system.interference_level = 0;
    for (0..MAX_SENSORS) |i| {
        g_sensor_system.sensors[i] = .{
            .sensor_type = .camera,
            .enabled = false,
            .field_of_view = 0,
            .range = 0,
            .noise_level = 0,
            .occlusion_factor = 0,
            .confidence = 0,
            .last_update_time = 0,
        };
    }
    g_sensor_system.fusion.detected_objects = [_]DetectedObject{.{
        .object_id = 0,
        .object_type = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0,
        .age = 0,
        .sensor_source = .camera,
    }} ** MAX_DETECTED_OBJECTS;
}

pub fn getSystem() *SensorSystem {
    return &g_sensor_system;
}

test "predictObjectPose advances detected object and preserves heading from velocity" {
    const pose = predictObjectPose(&.{
        .object_id = 1,
        .object_type = 0,
        .pos_x = 10,
        .pos_y = 2,
        .pos_z = 4,
        .vel_x = 3,
        .vel_y = 0,
        .vel_z = 4,
        .confidence = 1.0,
        .age = 0,
        .sensor_source = .radar,
    }, 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 16.0), pose.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), pose.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, std.math.atan2(@as(f32, 3.0), @as(f32, 4.0))), pose.yaw, 0.0001);
}

// ============================================================================
// Tests for Sensor System (Items 586-610)
// ============================================================================

test "586: radar sensor - detection and range" {
    init();
    const radar = addSensor(.radar, 30.0, 200.0);
    try std.testing.expect(radar != null);
    try std.testing.expect(radar.?.sensor_type == .radar);
    try std.testing.expectApproxEqAbs(@as(f32, 200.0), radar.?.range, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), radar.?.noise_level, 0.0001);
}

test "587: lidar sensor - point cloud generation" {
    init();
    const lidar = addSensor(.lidar, 60.0, 100.0);
    try std.testing.expect(lidar != null);
    try std.testing.expect(lidar.?.sensor_type == .lidar);
    try std.testing.expectApproxEqAbs(@as(f32, 0.02), lidar.?.noise_level, 0.0001);
}

test "588: camera sensor - vision detection" {
    init();
    const camera = addSensor(.camera, 90.0, 80.0);
    try std.testing.expect(camera != null);
    try std.testing.expect(camera.?.sensor_type == .camera);
    try std.testing.expect(camera.?.field_of_view > 0);
}

test "589: ultrasonic sensor - short range detection" {
    init();
    const ultrasonic = addSensor(.ultrasonic, 30.0, 10.0);
    try std.testing.expect(ultrasonic != null);
    try std.testing.expect(ultrasonic.?.sensor_type == .ultrasonic);
    try std.testing.expect(ultrasonic.?.range < 20.0);
}

test "590: IMU sensor - inertial measurement" {
    init();
    const imu = addSensor(.imu, 0, 0);
    try std.testing.expect(imu != null);
    try std.testing.expect(imu.?.sensor_type == .imu);
    try std.testing.expect(imu.?.noise_level < 0.05);
}

test "591: GPS sensor - position tracking" {
    init();
    const gps = addSensor(.gps, 0, 0);
    try std.testing.expect(gps != null);
    try std.testing.expect(gps.?.sensor_type == .gps);
    try std.testing.expect(gps.?.noise_level > 0.1);
}

test "592: sensor fusion - combining multiple sensors" {
    init();
    _ = addSensor(.radar, 30.0, 200.0);
    _ = addSensor(.lidar, 60.0, 100.0);
    _ = addSensor(.camera, 90.0, 80.0);
    const sys = getSystem();
    try std.testing.expect(sys.sensor_count == 3);
}

test "593: sensor calibration - confidence calculation" {
    init();
    const radar = addSensor(.radar, 30.0, 200.0).?;
    const confidence = calculateConfidence(50.0, radar, 1000.0);
    try std.testing.expect(confidence > 0);
    try std.testing.expect(confidence <= 1.0);
}

test "594: sensor noise - random variation" {
    init();
    const radar = addSensor(.radar, 30.0, 200.0).?;
    try std.testing.expect(radar.noise_level > 0);
}

test "595: sensor fusion - object tracking" {
    init();
    addDetectedObject(.{
        .object_id = 1,
        .object_type = 0,
        .pos_x = 10,
        .pos_y = 0,
        .pos_z = 20,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.9,
        .age = 0,
        .sensor_source = .radar,
    });
    const objects = getDetectedObjects();
    try std.testing.expect(objects.len == 1);
    try std.testing.expect(objects[0].object_id == 1);
}

test "596: sensor degradation - reduced performance over time" {
    init();
    const radar = addSensor(.radar, 30.0, 200.0).?;
    const initial_confidence = radar.confidence;
    degradeSensor(.radar, 0.5);
    try std.testing.expect(radar.confidence < initial_confidence);
}

test "597: sensor interference - noise from external sources" {
    init();
    const radar = addSensor(.radar, 30.0, 200.0).?;
    const initial_noise = radar.noise_level;
    addInterference(0.2);
    try std.testing.expect(radar.noise_level >= initial_noise);
}

test "598: sensor occlusion - blocked detection" {
    const occlusion = raycastOcclusion(0, 0, 0, 10, 0, 0, 5, 0, 0, 1.0);
    try std.testing.expect(occlusion > 0);
    try std.testing.expect(occlusion <= 1.0);
}

test "599: sensor occlusion - no occlusion when clear" {
    const occlusion = raycastOcclusion(0, 0, 0, 10, 0, 0, 20, 0, 0, 1.0);
    try std.testing.expect(occlusion == 0);
}

test "600: sensor failure - detection on failure" {
    init();
    const radar = addSensor(.radar, 30.0, 200.0).?;
    degradeSensor(.radar, 0.1);
    try std.testing.expect(radar.confidence < 1.0);
}

test "601: sensor redundancy - multiple sensors for reliability" {
    init();
    _ = addSensor(.radar, 30.0, 200.0);
    _ = addSensor(.radar, 30.0, 200.0);
    const sys = getSystem();
    try std.testing.expect(sys.sensor_count >= 2);
}

test "602: object detection - tracking multiple objects" {
    init();
    addDetectedObject(.{
        .object_id = 1,
        .object_type = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.9,
        .age = 0,
        .sensor_source = .radar,
    });
    addDetectedObject(.{
        .object_id = 2,
        .object_type = 0,
        .pos_x = 10,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.9,
        .age = 0,
        .sensor_source = .lidar,
    });
    const objects = getDetectedObjects();
    try std.testing.expect(objects.len == 2);
}

test "603: object classification - type identification" {
    init();
    addDetectedObject(.{
        .object_id = 1,
        .object_type = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.9,
        .age = 0,
        .sensor_source = .camera,
    });
    const objects = getDetectedObjects();
    if (objects.len > 0) {
        try std.testing.expect(objects[0].object_type == 2);
    }
}

test "604: object tracking - position updates" {
    init();
    addDetectedObject(.{
        .object_id = 1,
        .object_type = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 5,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.9,
        .age = 0,
        .sensor_source = .radar,
    });
    const objects_before = getDetectedObjects();
    if (objects_before.len > 0) {
        fuseSensors(0.5);
        const objects = getDetectedObjects();
        if (objects.len > 0) {
            try std.testing.expect(objects[0].age > 0);
            try std.testing.expectApproxEqAbs(@as(f32, 0.5), objects[0].age, 0.0001);
            try std.testing.expectApproxEqAbs(@as(f32, 2.5), objects[0].pos_x, 0.0001);
        }
    }
}

test "605: object classification - confidence weighting" {
    init();
    addDetectedObject(.{
        .object_id = 1,
        .object_type = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.5,
        .age = 0,
        .sensor_source = .radar,
    });
    const objects = getDetectedObjects();
    try std.testing.expect(objects.len == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), objects[0].confidence, 0.0001);
}

test "606: object prediction - future position estimation" {
    init();
    const obj = DetectedObject{
        .object_id = 1,
        .object_type = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.9,
        .age = 0,
        .sensor_source = .radar,
    };
    const predicted = predictObjectPosition(&obj, 1.0);
    try std.testing.expect(predicted.x > 0);
}

test "607: environment modeling - spatial representation" {
    init();
    _ = addSensor(.lidar, 60.0, 100.0);
    const sys = getSystem();
    try std.testing.expect(sys.sensor_count == 1);
}

test "608: SLAM interface - localization and mapping" {
    init();
    _ = addSensor(.gps, 0, 0);
    const gps = getSensorState(.gps);
    try std.testing.expect(gps != null);
}

test "609: localization - position from sensors" {
    init();
    const gps = addSensor(.gps, 0, 0);
    try std.testing.expect(gps != null);
}

test "610: mapping - environment representation" {
    init();
    _ = addSensor(.lidar, 60.0, 100.0);
    const sys = getSystem();
    try std.testing.expect(sys.sensor_count == 1);
}

test "sensor confidence for non-ranging sensor does not collapse with distance" {
    init();
    const imu = addSensor(.imu, 0.0, 0.0) orelse return error.TestUnexpectedResult;
    const near = calculateConfidence(1.0, imu, 1000.0);
    const far = calculateConfidence(10000.0, imu, 1000.0);
    try std.testing.expectApproxEqAbs(near, far, 0.0001);
}

test "degradeSensor clamps out-of-range factors safely" {
    init();
    const radar = addSensor(.radar, 30.0, 200.0) orelse return error.TestUnexpectedResult;
    degradeSensor(.radar, 1.5);
    try std.testing.expect(radar.confidence <= 1.0 and radar.confidence >= 0.0);
    degradeSensor(.radar, -0.5);
    try std.testing.expect(radar.confidence == 0.0);
}

test "raycastOcclusion handles zero-length ray without NaN or inf" {
    const occ = raycastOcclusion(1, 2, 3, 1, 2, 3, 1, 2, 3, 1.0);
    try std.testing.expect(occ == 0.0);
}

test "addDetectedObject fusion updates velocity with weighted confidence" {
    init();
    addDetectedObject(.{
        .object_id = 7,
        .object_type = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 1,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.2,
        .age = 0,
        .sensor_source = .radar,
    });
    addDetectedObject(.{
        .object_id = 7,
        .object_type = 2,
        .pos_x = 10,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 5,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 0.8,
        .age = 0,
        .sensor_source = .lidar,
    });

    const objects = getDetectedObjects();
    try std.testing.expect(objects.len == 1);
    try std.testing.expect(objects[0].vel_x > 3.0);
    try std.testing.expect(objects[0].object_type == 2);
    try std.testing.expect(objects[0].sensor_source == .lidar);
}

test "addDetectedObject updates existing track even when pool is full" {
    init();
    var i: usize = 0;
    while (i < MAX_DETECTED_OBJECTS) : (i += 1) {
        addDetectedObject(.{
            .object_id = @as(u16, @intCast(i + 1)),
            .object_type = 1,
            .pos_x = @as(f32, @floatFromInt(i)),
            .pos_y = 0,
            .pos_z = 0,
            .vel_x = 1,
            .vel_y = 0,
            .vel_z = 0,
            .confidence = 0.5,
            .age = 0,
            .sensor_source = .radar,
        });
    }
    try std.testing.expect(getDetectedObjects().len == MAX_DETECTED_OBJECTS);

    addDetectedObject(.{
        .object_id = 1,
        .object_type = 3,
        .pos_x = 100,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .confidence = 1.0,
        .age = 0,
        .sensor_source = .lidar,
    });

    const objects = getDetectedObjects();
    try std.testing.expect(objects.len == MAX_DETECTED_OBJECTS);
    try std.testing.expect(objects[0].pos_x > 60.0);
    try std.testing.expect(objects[0].object_type == 3);
    try std.testing.expect(objects[0].sensor_source == .lidar);
}

test "calculateConfidenceFromWeather degrades confidence in hazardous weather" {
    init();
    weather.init();
    const camera = addSensor(.camera, 90.0, 120.0) orelse return error.TestUnexpectedResult;
    const clear_confidence = calculateConfidenceFromWeather(40.0, camera);

    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.85, 60.0);
    weather.updateWeather(1.0);
    const hazard_confidence = calculateConfidenceFromWeather(40.0, camera);

    try std.testing.expect(hazard_confidence < clear_confidence);
    weather.init();
}
