//! Sensor System - Perception, Degradation, and Sensor Fusion
//!
//! Phase 64, 18: Sensor models, noise, degradation, fusion
//! Handles: Radar, LiDAR, camera, ultrasonic, sensor fusion, false positives

const std = @import("std");

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

var g_sensor_system: struct {
    sensors: [MAX_SENSORS]SensorState,
    sensor_count: u8,
    fusion: SensorFusionState,
    degradation_factor: f32,
    interference_level: f32,
} = undefined;

pub fn init() void {
    g_sensor_system.sensor_count = 0;
    g_sensor_system.degradation_factor = 1.0;
    g_sensor_system.interference_level = 0;
    g_sensor_system.fusion.object_count = 0;
    g_sensor_system.fusion.timestamp = 0;
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

    g_sensor_system.sensors[idx] = .{
        .sensor_type = sensor_type,
        .enabled = true,
        .field_of_view = fov,
        .range = range,
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
    for (0..MAX_SENSORS) |i| {
        if (g_sensor_system.sensors[i].sensor_type == sensor_type) {
            g_sensor_system.sensors[i].enabled = enabled;
        }
    }
}

pub fn degradeSensor(sensor_type: SensorType, factor: f32) void {
    for (0..MAX_SENSORS) |i| {
        if (g_sensor_system.sensors[i].sensor_type == sensor_type) {
            g_sensor_system.sensors[i].noise_level *= (2.0 - factor);
            g_sensor_system.sensors[i].confidence *= factor;
        }
    }
    g_sensor_system.degradation_factor *= factor;
}

pub fn addInterference(level: f32) void {
    g_sensor_system.interference_level += level;
    for (0..MAX_SENSORS) |i| {
        if (g_sensor_system.sensors[i].sensor_type == .radar or
            g_sensor_system.sensors[i].sensor_type == .lidar) {
            g_sensor_system.sensors[i].noise_level *= (1.0 + level);
        }
    }
}

pub fn raycastOcclusion(origin_x: f32, origin_y: f32, origin_z: f32,
                         target_x: f32, target_y: f32, target_z: f32,
                         occluder_x: f32, occluder_y: f32, occluder_z: f32,
                         occluder_radius: f32) f32 {
    const dx = target_x - origin_x;
    const dy = target_y - origin_y;
    const dz = target_z - origin_z;
    const ray_length = @sqrt(dx * dx + dy * dy + dz * dz);

    const t = ((occluder_x - origin_x) * dx + (occluder_y - origin_y) * dy + (occluder_z - origin_z) * dz) / (ray_length * ray_length);

    if (t < 0 or t > 1) return 0;

    const closest_x = origin_x + dx * t;
    const closest_y = origin_y + dy * t;
    const closest_z = origin_z + dz * t;

    const dist_to_occluder = @sqrt(
        (closest_x - occluder_x) * (closest_x - occluder_x) +
        (closest_y - occluder_y) * (closest_y - occluder_y) +
        (closest_z - occluder_z) * (closest_z - occluder_z)
    );

    if (dist_to_occluder < occluder_radius) {
        return 1.0 - dist_to_occluder / occluder_radius;
    }
    return 0;
}

pub fn addDetectedObject(obj: DetectedObject) void {
    if (g_sensor_system.fusion.object_count >= MAX_DETECTED_OBJECTS) return;

    for (0..MAX_DETECTED_OBJECTS) |i| {
        if (g_sensor_system.fusion.detected_objects[i].object_id == obj.object_id) {
            var existing = &g_sensor_system.fusion.detected_objects[i];
            const confidence_weight = obj.confidence / (existing.confidence + obj.confidence);
            existing.pos_x = existing.pos_x * (1.0 - confidence_weight) + obj.pos_x * confidence_weight;
            existing.pos_y = existing.pos_y * (1.0 - confidence_weight) + obj.pos_y * confidence_weight;
            existing.pos_z = existing.pos_z * (1.0 - confidence_weight) + obj.pos_z * confidence_weight;
            existing.confidence = (existing.confidence + obj.confidence) / 2.0;
            existing.age = 0;
            return;
        }
    }

    const idx = g_sensor_system.fusion.object_count;
    g_sensor_system.fusion.detected_objects[idx] = obj;
    g_sensor_system.fusion.object_count += 1;
}

pub fn fuseSensors(time: f32) void {
    g_sensor_system.fusion.timestamp = time;

    for (0..MAX_DETECTED_OBJECTS) |i| {
        if (i >= g_sensor_system.fusion.object_count) break;
        g_sensor_system.fusion.detected_objects[i].age += 0.1;

        if (g_sensor_system.fusion.detected_objects[i].age > 2.0) {
            g_sensor_system.fusion.detected_objects[i] = g_sensor_system.fusion.detected_objects[g_sensor_system.fusion.object_count - 1];
            g_sensor_system.fusion.object_count -= 1;
        }
    }
}

pub fn calculateConfidence(distance: f32, sensor: *const SensorState, weather_visibility: f32) f32 {
    const range_factor = 1.0 - @min(1.0, distance / sensor.range);
    const weather_factor = weather_visibility / 1000.0;
    const noise_factor = 1.0 - sensor.noise_level;
    const occlusion_factor = 1.0 - sensor.occlusion_factor;
    const degradation_factor = g_sensor_system.degradation_factor;

    return range_factor * weather_factor * noise_factor * occlusion_factor * degradation_factor;
}

pub fn predictObjectPosition(obj: *const DetectedObject, time_delta: f32) struct { x: f32, y: f32, z: f32 } {
    return .{
        .x = obj.pos_x + obj.vel_x * time_delta,
        .y = obj.pos_y + obj.vel_y * time_delta,
        .z = obj.pos_z + obj.vel_z * time_delta,
    };
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
    g_sensor_system.degradation_factor = 1.0;
    g_sensor_system.interference_level = 0;
}
