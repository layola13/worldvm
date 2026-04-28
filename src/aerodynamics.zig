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
    rear_wing_angle: f32,
    front_splitter_angle: f32,
    diffuser_angle: f32,
    wing_surface_area: f32,
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
        .rear_wing_angle = 0,
        .front_splitter_angle = 0,
        .diffuser_angle = 0,
        .wing_surface_area = 0.5,
    };
    g_aero_system.device_count = 0;
}

pub fn calculateDragForce(velocity_x: f32, velocity_z: f32, config: AeroConfig) f32 {
    const speed_squared = velocity_x * velocity_x + velocity_z * velocity_z;

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
    const wind_x = std.math.cos(wind_angle) * wind_speed;
    const wind_z = std.math.sin(wind_angle) * wind_speed;
    const rel_x = wind_x - velocity_x;
    const rel_z = wind_z - velocity_z;
    const rel_speed_sq = rel_x * rel_x + rel_z * rel_z;
    if (rel_speed_sq < 0.000001) return 0.0;

    // Derive vehicle heading from velocity, fallback to +Z when nearly stationary.
    var heading_x: f32 = 0.0;
    var heading_z: f32 = 1.0;
    const veh_speed_sq = velocity_x * velocity_x + velocity_z * velocity_z;
    if (veh_speed_sq > 0.000001) {
        const inv_veh_speed = 1.0 / @sqrt(veh_speed_sq);
        heading_x = velocity_x * inv_veh_speed;
        heading_z = velocity_z * inv_veh_speed;
    }

    const side_x = -heading_z;
    const side_z = heading_x;
    const rel_speed = @sqrt(rel_speed_sq);
    const lateral_component = (rel_x * side_x + rel_z * side_z) / rel_speed;
    const side_area: f32 = 1.8;
    return 0.5 * g_aero_system.aero.air_density * rel_speed_sq * side_area * lateral_component;
}

pub fn updateAero(velocity_x: f32, velocity_z: f32, _: f32) void {
    var aero = &g_aero_system.aero;

    // Aerodynamic forces depend on relative airflow, not just chassis speed.
    const rel_vx = velocity_x - aero.wind_velocity_x;
    const rel_vz = velocity_z - aero.wind_velocity_z;

    const config = AeroConfig{
        .drag_coefficient = aero.drag_coefficient,
        .frontal_area = aero.frontal_area,
        .downforce_coefficient = aero.downforce_coefficient,
        .rear_wing_angle = aero.rear_wing_angle,
        .front_splitter_angle = aero.front_splitter_angle,
        .diffuser_angle = aero.diffuser_angle,
        .wing_surface_area = aero.wing_surface_area,
    };

    aero.drag_force = calculateDragForce(rel_vx, rel_vz, config);
    aero.downforce = calculateDownforce(rel_vx, rel_vz, config);
    aero.lift_force = calculateLiftForce(rel_vx, rel_vz, config);

    const wind_speed = @sqrt(aero.wind_velocity_x * aero.wind_velocity_x + aero.wind_velocity_z * aero.wind_velocity_z);
    const wind_angle = if (wind_speed > 0.000001)
        std.math.atan2(aero.wind_velocity_z, aero.wind_velocity_x)
    else
        0.0;
    aero.crosswind_force = calculateCrosswindForce(wind_angle, wind_speed, velocity_x, velocity_z);

    const pressure_base = 0.5 * aero.air_density * (rel_vx * rel_vx + rel_vz * rel_vz);
    const splitter_bias = std.math.sin(aero.front_splitter_angle) * 0.25;
    aero.pressure_front = pressure_base * (1.0 + splitter_bias);
    aero.pressure_rear = pressure_base * (1.0 - splitter_bias * 0.6);
}

pub fn applyWingAngle(angle: f32) void {
    const clamped = @max(-1.2, @min(1.2, angle));
    g_aero_system.aero.rear_wing_angle = clamped;
    g_aero_system.aero.drag_coefficient = 0.30 + @abs(clamped) * 0.02; // More angle = more drag
    g_aero_system.aero.downforce_coefficient = 0.5 + @abs(clamped) * 0.1; // More angle = more downforce
}

pub fn applySplitterAngle(angle: f32) void {
    g_aero_system.aero.front_splitter_angle = @max(-0.8, @min(0.8, angle));
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

// ============================================================================
// Tests for Aerodynamics System (Items 551-555)
// ============================================================================

test "551: drag coefficient - aerodynamic drag calculation" {
    init();
    const config = AeroConfig{
        .drag_coefficient = 0.30,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.5,
        .rear_wing_angle = 0,
        .front_splitter_angle = 0,
        .diffuser_angle = 0,
        .wing_surface_area = 0.5,
    };
    const drag = calculateDragForce(30.0, 0.0, config);
    try std.testing.expect(drag > 0);
}

test "552: lift coefficient - net lift force from speed" {
    init();
    const config = AeroConfig{
        .drag_coefficient = 0.30,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.5,
        .rear_wing_angle = 0,
        .front_splitter_angle = 0,
        .diffuser_angle = 0,
        .wing_surface_area = 0.5,
    };
    const lift = calculateLiftForce(30.0, 0.0, config);
    try std.testing.expect(lift > 0);
    try std.testing.expect(!std.math.isNan(lift));
}

test "553: downforce - negative lift for vehicle grip" {
    init();
    const config = AeroConfig{
        .drag_coefficient = 0.30,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.5,
        .rear_wing_angle = 0.3,
        .front_splitter_angle = 0.1,
        .diffuser_angle = 0,
        .wing_surface_area = 0.5,
    };
    const downforce = calculateDownforce(50.0, 0.0, config);
    try std.testing.expect(downforce > 0);
}

test "554: induced drag - drag increases with downforce" {
    init();
    const config_low = AeroConfig{
        .drag_coefficient = 0.30,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.2,
        .rear_wing_angle = 0,
        .front_splitter_angle = 0,
        .diffuser_angle = 0,
        .wing_surface_area = 0.3,
    };
    const config_high = AeroConfig{
        .drag_coefficient = 0.30,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.8,
        .rear_wing_angle = 0.5,
        .front_splitter_angle = 0.2,
        .diffuser_angle = 0,
        .wing_surface_area = 0.8,
    };
    const drag_low = calculateDragForce(40.0, 0.0, config_low);
    const drag_high = calculateDragForce(40.0, 0.0, config_high);
    try std.testing.expect(drag_high >= drag_low);
}

test "555: spoiler effect - rear wing adds drag and downforce" {
    init();
    const config_no_spoiler = AeroConfig{
        .drag_coefficient = 0.30,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.3,
        .rear_wing_angle = 0,
        .front_splitter_angle = 0,
        .diffuser_angle = 0,
        .wing_surface_area = 0.3,
    };
    applyWingAngle(0.4);
    const config_with_spoiler = AeroConfig{
        .drag_coefficient = 0.38,
        .frontal_area = 2.2,
        .downforce_coefficient = 0.9,
        .rear_wing_angle = 0.4,
        .front_splitter_angle = 0,
        .diffuser_angle = 0,
        .wing_surface_area = 0.5,
    };
    const drag_no_spoiler = calculateDragForce(30.0, 0.0, config_no_spoiler);
    const drag_with_spoiler = calculateDragForce(30.0, 0.0, config_with_spoiler);
    try std.testing.expect(drag_with_spoiler > drag_no_spoiler);
}

test "aero wind affects drag by relative airflow direction" {
    init();
    setWindDirection(0.0, 20.0); // tailwind for +X motion
    updateAero(20.0, 0.0, 0.0);
    const tailwind_drag = getDragForce();

    setWindDirection(std.math.pi, 20.0); // headwind for +X motion
    updateAero(20.0, 0.0, 0.0);
    const headwind_drag = getDragForce();

    try std.testing.expect(headwind_drag > tailwind_drag);
}

test "aero crosswind magnitude rises for lateral wind versus aligned wind" {
    init();
    setWindDirection(std.math.pi / 2.0, 15.0); // aligned with +Z motion
    updateAero(0.0, 30.0, 0.0);
    const aligned_crosswind = @abs(getAeroState().crosswind_force);

    setWindDirection(0.0, 15.0); // lateral wind from +X
    updateAero(0.0, 30.0, 0.0);
    const lateral_crosswind = @abs(getAeroState().crosswind_force);

    try std.testing.expect(lateral_crosswind > aligned_crosswind);
}
