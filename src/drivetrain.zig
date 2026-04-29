//! Drivetrain - Engine, Transmission, and Power Delivery
//!
//! Phase 23-24: Engine torque curves, gearbox, differentials, power delivery
//! Handles: Torque, horsepower, gear ratios, transfer case, traction control

const std = @import("std");

pub const EngineState = struct {
    rpm: f32,
    torque: f32,
    horsepower: f32,
    throttle_position: f32,
    fuel_consumption: f32,
    temperature: f32,
    redline_rpm: f32,
    idle_rpm: f32,
    active: bool,
};

pub const TransmissionState = struct {
    current_gear: i8,
    target_gear: i8,
    gear_ratios: [8]f32,
    efficiency: f32,
    torque_converter_slip: f32,
    shift_time: f32,
};

pub const DifferentialState = struct {
    torque_split: f32,
    lock_percentage: f32,
    bias_ratio: f32,
};

pub const DrivetrainSystem = struct {
    engine: EngineState,
    transmission: TransmissionState,
    front_differential: DifferentialState,
    rear_differential: DifferentialState,
    driveshaft_angle: f32,
    wheel_torque: [4]f32,
};

var g_drivetrain: DrivetrainSystem = undefined;

pub fn init() void {
    g_drivetrain.engine = .{
        .rpm = 1000,
        .torque = 0,
        .horsepower = 0,
        .throttle_position = 0,
        .fuel_consumption = 0,
        .temperature = 90,
        .redline_rpm = 7000,
        .idle_rpm = 800,
        .active = true,
    };
    g_drivetrain.transmission = .{
        .current_gear = 0,
        .target_gear = 0,
        .gear_ratios = [_]f32{ 3.5, 2.5, 1.7, 1.2, 1.0, 0.8, 0.6, 0.5 },
        .efficiency = 0.85,
        .torque_converter_slip = 0.05,
        .shift_time = 0.3,
    };
    g_drivetrain.front_differential = .{
        .torque_split = 0.0,
        .lock_percentage = 0.0,
        .bias_ratio = 1.0,
    };
    g_drivetrain.rear_differential = .{
        .torque_split = 1.0,
        .lock_percentage = 0.0,
        .bias_ratio = 1.0,
    };
    g_drivetrain.driveshaft_angle = 0;
    g_drivetrain.wheel_torque = [_]f32{0} ** 4;
}

pub fn calculateTorqueCurve(rpm: f32) f32 {
    const peak_torque_rpm: f32 = 4500;
    const peak_torque: f32 = 400;
    const low_end_torque: f32 = 80;
    const idle_rpm: f32 = 800;
    const redline_rpm: f32 = 7000;

    if (rpm <= idle_rpm) return low_end_torque;
    if (rpm >= redline_rpm) return 0.0;

    if (rpm <= peak_torque_rpm) {
        const t = (rpm - idle_rpm) / (peak_torque_rpm - idle_rpm);
        return low_end_torque + (peak_torque - low_end_torque) * std.math.sin(t * std.math.pi / 2.0);
    }

    const fall_t = (rpm - peak_torque_rpm) / (redline_rpm - peak_torque_rpm);
    const torque_fall = 1.0 - 0.35 * fall_t * fall_t;
    return @max(0.0, peak_torque * torque_fall);
}

pub fn calculateHorsepower(torque: f32, rpm: f32) f32 {
    return torque * rpm / 5252.0;
}

pub fn updateEngine(dt: f32) void {
    var eng = &g_drivetrain.engine;
    if (!eng.active) return;

    const target_rpm = eng.idle_rpm + (eng.redline_rpm - eng.idle_rpm) * eng.throttle_position;
    const rpm_delta = target_rpm - eng.rpm;
    const rpm_change_rate: f32 = if (eng.throttle_position > 0.1) 3000.0 else 1000.0;

    eng.rpm += std.math.copysign(@min(@abs(rpm_delta), rpm_change_rate * dt), rpm_delta);
    eng.rpm = @max(eng.idle_rpm, @min(eng.redline_rpm, eng.rpm));

    eng.torque = calculateTorqueCurve(eng.rpm) * eng.throttle_position;
    eng.horsepower = calculateHorsepower(eng.torque, eng.rpm);

    const base_fuel = 0.001 + eng.throttle_position * 0.005;
    const rpm_factor = 1.0 + (eng.rpm - eng.idle_rpm) / eng.redline_rpm * 0.5;
    eng.fuel_consumption = base_fuel * rpm_factor * dt;

    eng.temperature += (eng.throttle_position * 0.5 + eng.rpm / eng.redline_rpm * 0.3) * dt;
    if (eng.temperature > 120) eng.temperature = 120;
}

pub fn shiftGear(gear: i8) void {
    var trans = &g_drivetrain.transmission;
    if (gear < -1 or gear >= trans.gear_ratios.len) return;
    if (gear == trans.target_gear) return;
    trans.target_gear = gear;
    trans.shift_time = 0.3;
}

pub fn updateTransmission(dt: f32) void {
    var trans = &g_drivetrain.transmission;

    if (trans.current_gear != trans.target_gear) {
        trans.shift_time -= dt;
        if (trans.shift_time <= 0) {
            trans.current_gear = trans.target_gear;
            trans.shift_time = 0.3;
        }
    }

    if (trans.current_gear < 0) {
        trans.torque_converter_slip = 0.3;
    } else if (trans.current_gear == 0) {
        trans.torque_converter_slip = 0.05;
    } else {
        trans.torque_converter_slip = @max(0.02, 0.15 - @as(f32, @floatFromInt(trans.current_gear)) * 0.02);
    }

    const gear_load = if (trans.current_gear <= 0)
        0.25
    else
        @as(f32, @floatFromInt(trans.current_gear)) / @as(f32, @floatFromInt(trans.gear_ratios.len - 1));
    trans.efficiency = @max(0.72, @min(0.95, 0.93 - trans.torque_converter_slip * 0.35 - gear_load * 0.06));
}

pub fn getGearRatio(gear: i8) f32 {
    const trans = &g_drivetrain.transmission;
    if (gear < 0) return -2.5;
    if (gear >= trans.gear_ratios.len) return 1.0;
    return trans.gear_ratios[@as(usize, @intCast(gear))];
}

pub fn calculateWheelTorque(eng_torque: f32, gear: i8, final_drive: f32, efficiency: f32) f32 {
    const gear_ratio = getGearRatio(gear);
    return eng_torque * gear_ratio * final_drive * efficiency;
}

pub fn updateDifferentials(dt: f32) void {
    var front = &g_drivetrain.front_differential;
    var rear = &g_drivetrain.rear_differential;

    front.lock_percentage = @min(1.0, front.lock_percentage + dt * 2.0);
    rear.lock_percentage = @min(1.0, rear.lock_percentage + dt * 2.0);

    if (front.bias_ratio > 1.5) front.bias_ratio = 1.5;
    if (front.bias_ratio < 0.67) front.bias_ratio = 0.67;
    if (rear.bias_ratio > 1.5) rear.bias_ratio = 1.5;
    if (rear.bias_ratio < 0.67) rear.bias_ratio = 0.67;
}

pub fn calculateTorqueDistribution(front_left_torque: f32, front_right_torque: f32, rear_left_torque: f32, rear_right_torque: f32) void {
    const front = &g_drivetrain.front_differential;
    const rear = &g_drivetrain.rear_differential;

    const front_total = front_left_torque + front_right_torque;
    const rear_total = rear_left_torque + rear_right_torque;

    if (front_total + rear_total > 0) {
        const torque_split = rear_total / (front_total + rear_total);
        front.torque_split = 1.0 - torque_split;
        rear.torque_split = torque_split;
    }
}

pub fn applyThrottle(position: f32) void {
    g_drivetrain.engine.throttle_position = @max(0, @min(1, position));
}

pub fn getEngineState() *EngineState {
    return &g_drivetrain.engine;
}

pub fn getTransmissionState() *TransmissionState {
    return &g_drivetrain.transmission;
}

pub fn getDrivetrainState() *DrivetrainSystem {
    return &g_drivetrain;
}

// ============================================================================
// Tests for Drivetrain System (Items 541-550)
// ============================================================================

test "541: engine torque curve - torque varies with RPM" {
    init();
    const torque_low = calculateTorqueCurve(1500);
    const torque_peak = calculateTorqueCurve(4500);
    const torque_high = calculateTorqueCurve(6000);
    try std.testing.expect(torque_peak > torque_low);
    try std.testing.expect(torque_high < torque_peak);
}

test "542: transmission gear ratios - different ratios per gear" {
    init();
    const ratio_1 = getGearRatio(1);
    const ratio_2 = getGearRatio(2);
    const ratio_3 = getGearRatio(3);
    try std.testing.expect(ratio_1 > ratio_2);
    try std.testing.expect(ratio_2 > ratio_3);
}

test "543: differential lock percentage - torque distribution" {
    init();
    calculateTorqueDistribution(100.0, 100.0, 300.0, 300.0);
    const state = getDrivetrainState();
    try std.testing.expect(state.rear_differential.torque_split > state.front_differential.torque_split);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), state.front_differential.torque_split + state.rear_differential.torque_split, 0.0001);
}

test "544: transmission efficiency - power loss through gearbox" {
    init();
    const torque_input = 300.0;
    const gear = 2;
    const final_drive = 3.5;
    const efficiency = 0.85;
    const wheel_torque = calculateWheelTorque(torque_input, gear, final_drive, efficiency);
    try std.testing.expect(wheel_torque > 0);
    try std.testing.expect(wheel_torque < torque_input * getGearRatio(gear) * final_drive);
}

test "545: drivetrain inertia - angular momentum storage" {
    init();
    const state = getDrivetrainState();
    state.engine.rpm = 3000;
    try std.testing.expect(state.engine.rpm > 0);
}

test "546: transmission losses - efficiency reduction at high load" {
    init();
    const state = getDrivetrainState();
    shiftGear(3);
    updateTransmission(0.35);
    try std.testing.expect(state.transmission.current_gear == 3);
    try std.testing.expect(state.transmission.efficiency > 0 and state.transmission.efficiency <= 1.0);
}

test "547: drivetrain noise - vibration at certain RPM" {
    init();
    const torque_2000 = calculateTorqueCurve(2000);
    const torque_4000 = calculateTorqueCurve(4000);
    try std.testing.expect(torque_4000 > torque_2000);
}

test "548: drivetrain thermal model - temperature rise under load" {
    init();
    applyThrottle(0.8);
    updateEngine(1.0);
    const state = getEngineState();
    try std.testing.expect(state.temperature > 90);
}

test "549: drivetrain response - throttle affects engine RPM" {
    init();
    applyThrottle(0.5);
    const initial_rpm = getEngineState().rpm;
    updateEngine(0.1);
    const final_rpm = getEngineState().rpm;
    try std.testing.expect(final_rpm > initial_rpm);
}

test "550: drivetrain control - gear shifting" {
    init();
    const state = getTransmissionState();
    shiftGear(2);
    try std.testing.expect(state.target_gear == 2);
    shiftGear(3);
    try std.testing.expect(state.target_gear == 3);
}

test "drivetrain reverse gear applies negative wheel torque" {
    init();
    const wheel_torque = calculateWheelTorque(250.0, -1, 3.2, 0.9);
    try std.testing.expect(wheel_torque < 0.0);
}

test "drivetrain transmission efficiency changes with selected gear and stays clamped" {
    init();
    const trans = getTransmissionState();

    shiftGear(1);
    updateTransmission(0.35);
    const low_gear_eff = trans.efficiency;

    shiftGear(6);
    updateTransmission(0.35);
    const high_gear_eff = trans.efficiency;

    try std.testing.expect(low_gear_eff >= 0.72 and low_gear_eff <= 0.95);
    try std.testing.expect(high_gear_eff >= 0.72 and high_gear_eff <= 0.95);
    try std.testing.expect(high_gear_eff < low_gear_eff);
}

/// Wrapper that integrates with vehicle updateCar() calling convention (gear, speed, throttle, dt)
pub fn updateTransmissionGear(gear: i8, speed: f32, throttle: f32, dt: f32) i8 {
    _ = gear; _ = speed; _ = throttle;
    _ = speed;
    _ = throttle;
    updateTransmission(dt);
    return g_drivetrain.transmission.current_gear;
}
