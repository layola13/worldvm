//! Aerodynamics - Air Resistance, Downforce, and Drag Models
//!
//! Phase 25: Aerodynamic forces, drag coefficients, downforce, wind
//! Handles: Drag, lift, downforce, spoilers, wing angle, crosswinds

const std = @import("std");

pub const AeroState = struct {
    drag_force: f32,
    downforce: f32,
    lift_force: f32,
    crosswind_force: f32,
    drag_coefficient: f32,
    frontal_area: f32,
    downforce_coefficient: f32,
    wind_velocity_x: f32,
    wind_velocity_z: f32,
    air_density: f32,
    pressure_front: f32,
    pressure_rear: f32,
};

pub const AeroConfig = struct {
    drag_coefficient: f32 = 0.30,
    frontal_area: f32 = 2.2,
    downforce_coefficient: f32 = 0.5,
    rear_wing_angle: f32 = 0,
    front_splitter_angle: f32 = 0,
    diffuser_angle: f32 = 0,
    wing_surface_area: f32 = 0.5,
};

pub const MAX_AERO_DEVICES: usize = 4;

pub const AeroSystem = struct {
    aero: AeroState,
    devices: [MAX_AERO_DEVICES]struct {
        active: bool,
        drag_delta: f32,
        downforce_delta: f32,
    },
    device_count: u8,
};

var g_aero_system: AeroSystem = undefined;

pub fn init() void {
    g_aero_system.aero = .{
        .drag_force = 0,
        .downforce = 0,
        .lift_force = 0,
        .crosswind_force = 0,
        .drag_coefficient = 0.30,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.5,
        .wind_velocity_x = 0,
        .wind_velocity_z = 0,
        .air_density = 1.225,
        .pressure_front = 0,
        .pressure_rear = 0,
    };
    g_aero_system.device_count = 0;
}

pub fn calculateDragForce(velocity_x: f32, velocity_z: f32, config: AeroConfig) f32 {
    const speed_squared = velocity_x * velocity_x + velocity_z * velocity_z;
    _ = @sqrt(speed_squared);
    _ = 0.5 * g_aero_system.aero.air_density * speed_squared;

    var total_drag_coef = config.drag_coefficient;
    var total_frontal = config.frontal_area;

    for (g_aero_system.devices[0..g_aero_system.device_count]) |device| {
        if (device.active) {
            total_drag_coef += device.drag_delta;
            total_frontal += device.drag_delta * 0.1;
        }
    }

    return 0.5 * g_aero_system.aero.air_density * speed_squared * total_drag_coef * total_frontal;
}

pub fn calculateDownforce(velocity_x: f32, velocity_z: f32, config: AeroConfig) f32 {
    const speed_squared = velocity_x * velocity_x + velocity_z * velocity_z;
    _ = @sqrt(speed_squared);

    var total_df_coef = config.downforce_coefficient;
    var wing_area = config.wing_surface_area;

    const wing_angle_factor = 1.0 + std.math.sin(config.rear_wing_angle) * 2.0;
    const splitter_factor = 1.0 + std.math.sin(config.front_splitter_angle) * 0.5;

    for (g_aero_system.devices[0..g_aero_system.device_count]) |device| {
        if (device.active) {
            total_df_coef += device.downforce_delta;
            wing_area += device.downforce_delta * 0.1;
        }
    }

    return 0.5 * g_aero_system.aero.air_density * speed_squared * total_df_coef * wing_area * wing_angle_factor * splitter_factor;
}

pub fn calculateLiftForce(velocity_x: f32, velocity_z: f32, config: AeroConfig) f32 {
    const speed_squared = velocity_x * velocity_x + velocity_z * velocity_z;
    const base_lift = 0.5 * g_aero_system.aero.air_density * speed_squared * 0.15 * config.frontal_area;
    const downforce = calculateDownforce(velocity_x, velocity_z, config);
    return base_lift - downforce * 0.3;
}

pub fn calculateCrosswindForce(wind_angle: f32, wind_speed: f32, velocity_x: f32, velocity_z: f32) f32 {
    const relative_angle = wind_angle - std.math.atan2(f32, velocity_z, velocity_x);
    const relative_speed = @sqrt(velocity_x * velocity_x + velocity_z * velocity_z) + wind_speed;
    const side_force_coefficient = std.math.sin(relative_angle) * 0.5;
    return 0.5 * g_aero_system.aero.air_density * relative_speed * relative_speed * side_force_coefficient;
}

pub fn updateAero(velocity_x: f32, velocity_z: f32, _: f32) void {
    var aero = &g_aero_system.aero;
    const config = AeroConfig{};

    aero.drag_force = calculateDragForce(velocity_x, velocity_z, config);
    aero.downforce = calculateDownforce(velocity_x, velocity_z, config);
    aero.lift_force = calculateLiftForce(velocity_x, velocity_z, config);
    aero.crosswind_force = calculateCrosswindForce(0, 5.0, velocity_x, velocity_z);

    const pressure_base = 0.5 * aero.air_density * (velocity_x * velocity_x + velocity_z * velocity_z);
    aero.pressure_front = pressure_base * (1.0 + config.front_splitter_angle * 0.5);
    aero.pressure_rear = pressure_base * (1.0 - config.rear_wing_angle * 0.3);
}

pub fn applyWingAngle(angle: f32) void {
    _ = angle;
}

pub fn applySplitterAngle(angle: f32) void {
    _ = angle;
}

pub fn setWindDirection(angle: f32, speed: f32) void {
    g_aero_system.aero.wind_velocity_x = std.math.cos(angle) * speed;
    g_aero_system.aero.wind_velocity_z = std.math.sin(angle) * speed;
}

pub fn addAeroDevice(drag_delta: f32, downforce_delta: f32) void {
    if (g_aero_system.device_count >= MAX_AERO_DEVICES) return;
    const idx = g_aero_system.device_count;
    g_aero_system.device_count += 1;
    g_aero_system.devices[idx] = .{
        .active = true,
        .drag_delta = drag_delta,
        .downforce_delta = downforce_delta,
    };
}

pub fn getDragForce() f32 {
    return g_aero_system.aero.drag_force;
}

pub fn getDownforce() f32 {
    return g_aero_system.aero.downforce;
}

pub fn getAeroState() *AeroState {
    return &g_aero_system.aero;
}
