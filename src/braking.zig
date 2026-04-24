//! Braking System - ABS, Brake Balance, and Thermal Dynamics
//!
//! Phase 26: Brake systems, ABS, brake bias, thermal fade, brake-by-wire
//! Handles: Brake force distribution, ABS cycling, thermal management, brake assist

const std = @import("std");

pub const BrakeState = struct {
    pedal_position: f32,
    front_brake_torque: f32,
    rear_brake_torque: f32,
    front_pressure: f32,
    rear_pressure: f32,
    front_temperature: f32,
    rear_temperature: f32,
    abs_active: bool,
    abs_cycle: u8,
    brake_bias: f32,
    handbrake_active: bool,
};

pub const BrakeConfig = struct {
    max_front_torque: f32,
    max_rear_torque: f32,
    pedal_ratio: f32,
    booster_gain: f32,
    rotor_diameter: f32,
    pad_area: f32,
    cooling_coefficient: f32,
    fade_threshold: f32,
    abs_threshold: f32,
};

pub const MAX_AXLES: usize = 2;

pub const BrakeSystem = struct {
    axle: [MAX_AXLES]BrakeState,
    brake_balance: f32,
    brake_by_wire: bool,
    brake_assist: bool,
};

var g_brake_system: BrakeSystem = undefined;

pub fn init() void {
    g_brake_system.brake_balance = 0.65;
    g_brake_system.brake_by_wire = false;
    g_brake_system.brake_assist = true;
    for (0..MAX_AXLES) |i| {
        g_brake_system.axle[i] = .{
            .pedal_position = 0,
            .front_brake_torque = 0,
            .rear_brake_torque = 0,
            .front_pressure = 0,
            .rear_pressure = 0,
            .front_temperature = 20,
            .rear_temperature = 20,
            .abs_active = false,
            .abs_cycle = 0,
            .brake_bias = if (i == 0) 0.65 else 0.35,
            .handbrake_active = false,
        };
    }
}

pub fn applyBrake(pedal_position: f32) void {
    const clamped_pedal = @max(0, @min(1, pedal_position));

    for (0..MAX_AXLES) |i| {
        g_brake_system.axle[i].pedal_position = clamped_pedal;
    }
}

pub fn applyHandbrake(active: bool) void {
    for (0..MAX_AXLES) |i| {
        g_brake_system.axle[i].handbrake_active = active;
    }
}

pub fn calculateBrakeForce(pedal: f32, max_torque: f32, temperature: f32, config: BrakeConfig) f32 {
    const pressure = pedal * config.pedal_ratio * config.booster_gain;
    const fade_factor = if (temperature > config.fade_threshold)
        1.0 - (temperature - config.fade_threshold) / (400 - config.fade_threshold) * 0.5
    else
        1.0;
    return pressure * max_torque * fade_factor;
}

pub fn updateABS(axle_idx: usize, wheel_slip: f32, config: BrakeConfig) void {
    if (axle_idx >= MAX_AXLES) return;
    var axle = &g_brake_system.axle[axle_idx];

    if (@abs(wheel_slip) > config.abs_threshold) {
        axle.abs_active = true;
        axle.abs_cycle = (axle.abs_cycle + 1) % 4;
        axle.pedal_position *= 0.7;
    } else {
        axle.abs_active = false;
    }
}

pub fn calculateBrakeBalance(front_load: f32, rear_load: f32, handbrake_ratio: f32) f32 {
    const total_load = front_load + rear_load;
    if (total_load < 0.001) return 0.65;

    var bias = front_load / total_load;
    if (handbrake_ratio > 0) {
        bias = bias * (1.0 - handbrake_ratio) + 0.0 * handbrake_ratio;
    }
    return @max(0.5, @min(0.8, bias));
}

pub fn updateBraking(dt: f32) void {
    const config = BrakeConfig{
        .max_front_torque = 3000,
        .max_rear_torque = 2000,
        .pedal_ratio = 4.0,
        .booster_gain = 1.5,
        .rotor_diameter = 0.35,
        .pad_area = 50,
        .cooling_coefficient = 0.1,
        .fade_threshold = 300,
        .abs_threshold = 0.15,
    };

    for (0..MAX_AXLES) |i| {
        var axle = &g_brake_system.axle[i];

        if (axle.handbrake_active) {
            axle.front_brake_torque = calculateBrakeForce(0, config.max_front_torque, axle.front_temperature, config) * 0.3;
            axle.rear_brake_torque = calculateBrakeForce(1.0, config.max_rear_torque, axle.rear_temperature, config);
        } else {
            axle.front_brake_torque = calculateBrakeForce(axle.pedal_position, config.max_front_torque, axle.front_temperature, config) * g_brake_system.brake_balance;
            axle.rear_brake_torque = calculateBrakeForce(axle.pedal_position, config.max_rear_torque, axle.rear_temperature, config) * (1.0 - g_brake_system.brake_balance);
        }

        axle.front_pressure = axle.pedal_position * config.booster_gain * (if (i == 0) g_brake_system.brake_balance else (1.0 - g_brake_system.brake_balance));
        axle.rear_pressure = axle.pedal_position * config.booster_gain * (if (i == 0) (1.0 - g_brake_system.brake_balance) else g_brake_system.brake_balance);

        const heat_generation = (@abs(axle.front_brake_torque) + @abs(axle.rear_brake_torque)) * 0.01 * axle.pedal_position;
        const cooling = (axle.front_temperature - 20) * config.cooling_coefficient * dt;
        axle.front_temperature += heat_generation - cooling;
        axle.rear_temperature += heat_generation * 1.2 - cooling * 1.1;
    }
}

pub fn setBrakeBalance(bias: f32) void {
    g_brake_system.brake_balance = @max(0.5, @min(0.8, bias));
}

pub fn getBrakeTorque(wheel_index: usize) f32 {
    if (wheel_index < 4) {
        return if (wheel_index < 2) g_brake_system.axle[0].front_brake_torque else g_brake_system.axle[1].rear_brake_torque;
    }
    return 0;
}

pub fn isABSActive(wheel_index: usize) bool {
    if (wheel_index < 4) {
        return g_brake_system.axle[if (wheel_index < 2) 0 else 1].abs_active;
    }
    return false;
}

pub fn getBrakeState() *BrakeState {
    return &g_brake_system.axle[0];
}

pub fn enableBrakeAssist(enable: bool) void {
    g_brake_system.brake_assist = enable;
}
