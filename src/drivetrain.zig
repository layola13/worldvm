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
    const idle_rpm: f32 = 800;
    const redline_rpm: f32 = 7000;

    if (rpm < idle_rpm) return 50;
    if (rpm > redline_rpm) return 0;

    const torque_rise = std.math.sin((rpm - idle_rpm) / (peak_torque_rpm - idle_rpm) * 3.14159 / 2.0);
    const torque_fall = 1.0 - (rpm - peak_torque_rpm) / (redline_rpm - peak_torque_rpm) * 0.3;

    return peak_torque * torque_rise * torque_fall;
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
    trans.target_gear = gear;
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
}

pub fn getGearRatio(gear: i8) f32 {
    const trans = &g_drivetrain.transmission;
    if (gear < 0) return 2.5;
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

pub fn calculateTorqueDistribution(front_left_torque: f32, front_right_torque: f32,
                                   rear_left_torque: f32, rear_right_torque: f32) void {
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
