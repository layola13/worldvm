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

fn clamp01(value: f32) f32 {
    return @max(0.0, @min(1.0, value));
}

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
    const clamped_pedal = @max(0.0, @min(1.0, pedal));
    const pressure = clamped_pedal * config.pedal_ratio * config.booster_gain;
    const fade_factor = if (temperature > config.fade_threshold)
        @max(0.25, 1.0 - (temperature - config.fade_threshold) / (400 - config.fade_threshold) * 0.5)
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
        axle.pedal_position = @max(0.1, axle.pedal_position * 0.7);
    } else {
        axle.abs_active = false;
        axle.abs_cycle = 0;
    }
}

pub fn calculateBrakeBalance(front_load: f32, rear_load: f32, handbrake_ratio: f32) f32 {
    const total_load = front_load + rear_load;
    if (total_load < 0.001) return 0.65;

    const base_bias = @max(0.5, @min(0.8, front_load / total_load));
    const handbrake = clamp01(handbrake_ratio);

    // Handbrake intentionally shifts torque away from front axle toward rear lock.
    if (handbrake > 0.0) {
        return base_bias * (1.0 - handbrake);
    }
    return base_bias;
}

pub fn updateBraking(dt: f32) void {
    if (dt <= 0.0) return;
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
        var effective_pedal = axle.pedal_position;

        if (g_brake_system.brake_assist and effective_pedal > 0.7) {
            effective_pedal = @min(1.0, effective_pedal * 1.1);
        }
        if (g_brake_system.brake_by_wire) {
            effective_pedal = effective_pedal * 0.95;
        }

        if (axle.handbrake_active) {
            axle.front_brake_torque = calculateBrakeForce(0, config.max_front_torque, axle.front_temperature, config) * 0.3;
            axle.rear_brake_torque = calculateBrakeForce(1.0, config.max_rear_torque, axle.rear_temperature, config);
        } else {
            axle.front_brake_torque = calculateBrakeForce(effective_pedal, config.max_front_torque, axle.front_temperature, config) * g_brake_system.brake_balance;
            axle.rear_brake_torque = calculateBrakeForce(effective_pedal, config.max_rear_torque, axle.rear_temperature, config) * (1.0 - g_brake_system.brake_balance);
        }

        axle.front_pressure = effective_pedal * config.booster_gain * (if (i == 0) g_brake_system.brake_balance else (1.0 - g_brake_system.brake_balance));
        axle.rear_pressure = effective_pedal * config.booster_gain * (if (i == 0) (1.0 - g_brake_system.brake_balance) else g_brake_system.brake_balance);

        const ambient: f32 = 20.0;
        const heat_generation = (@abs(axle.front_brake_torque) + @abs(axle.rear_brake_torque)) * 0.01 * axle.pedal_position;
        const front_cooling = (axle.front_temperature - ambient) * config.cooling_coefficient * dt;
        const rear_cooling = (axle.rear_temperature - ambient) * config.cooling_coefficient * dt * 1.1;
        axle.front_temperature += heat_generation - front_cooling;
        axle.rear_temperature += heat_generation * 1.2 - rear_cooling;
        axle.front_temperature = @max(ambient, axle.front_temperature);
        axle.rear_temperature = @max(ambient, axle.rear_temperature);
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

pub fn getSystem() *BrakeSystem {
    return &g_brake_system;
}

// ============================================================================
// Tests for Braking System (Items 556-560)
// ============================================================================

test "556: brake pressure - pedal position affects brake force" {
    init();
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
    const force_low = calculateBrakeForce(0.3, config.max_front_torque, 20.0, config);
    const force_high = calculateBrakeForce(0.8, config.max_front_torque, 20.0, config);
    try std.testing.expect(force_high > force_low);
}

test "557: brake force distribution - front/rear bias" {
    init();
    const front_load: f32 = 4000;
    const rear_load: f32 = 2000;
    const bias = calculateBrakeBalance(front_load, rear_load, 0);
    try std.testing.expect(bias >= 0.5);
    try std.testing.expect(bias <= 0.8);
}

test "558: ABS control - prevents wheel lockup" {
    init();
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
    applyBrake(1.0);
    updateABS(0, 0.2, config);
    const state = getBrakeState();
    try std.testing.expect(state.abs_active);
    try std.testing.expect(state.pedal_position < 1.0);
    updateABS(0, 0.05, config);
    try std.testing.expect(!state.abs_active);
}

test "559: brake thermal model - temperature rises under braking" {
    init();
    applyBrake(0.7);
    updateBraking(1.0);
    const state = getBrakeState();
    try std.testing.expect(state.front_temperature > 20.0);
    try std.testing.expect(state.rear_temperature > 20.0);
}

test "560: brake fade - reduced effectiveness at high temperature" {
    init();
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
    const force_normal = calculateBrakeForce(0.8, config.max_front_torque, 100.0, config);
    const force_hot = calculateBrakeForce(0.8, config.max_front_torque, 400.0, config);
    try std.testing.expect(force_hot < force_normal);
}

test "brake balance shifts rearward with handbrake ratio" {
    init();
    const no_handbrake = calculateBrakeBalance(4000.0, 2000.0, 0.0);
    const full_handbrake = calculateBrakeBalance(4000.0, 2000.0, 1.0);
    try std.testing.expect(no_handbrake >= 0.5 and no_handbrake <= 0.8);
    try std.testing.expect(full_handbrake == 0.0);
}

test "brake temperature cools toward ambient without pedal input" {
    init();
    var sys = getSystem();
    sys.axle[0].front_temperature = 200.0;
    sys.axle[0].rear_temperature = 180.0;
    applyBrake(0.0);
    updateBraking(1.0);
    try std.testing.expect(sys.axle[0].front_temperature < 200.0);
    try std.testing.expect(sys.axle[0].rear_temperature < 180.0);
    try std.testing.expect(sys.axle[0].front_temperature >= 20.0);
    try std.testing.expect(sys.axle[0].rear_temperature >= 20.0);
}
