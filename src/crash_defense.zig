//! Crash Defense - Sanity Checks and Recovery
//!
//! Phase 13: Physics state validation, emergency stops, snapshot recovery
//! Handles: NaN detection, bounds checking, energy conservation, state recovery

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");

pub const SanityCheckConfig = struct {
    nan_check_enabled: bool = true,
    bounds_check_enabled: bool = true,
    energy_check_enabled: bool = false,
    velocity_cap: f32 = 10000.0,
    position_min: i32 = -10000,
    position_max: i32 = 10000,
    max_ticks_without_progress: u16 = 1000,
};

pub const SanityCheckResult = struct {
    nan_detected: bool,
    bounds_violated: bool,
    energy_violation: bool,
    velocity_exceeded: bool,
    instance_errors: u8,
};

pub const NaNHandlingPlan = struct {
    valid: bool,
    nan_detected: bool,
    sanitized: bool,
    emergency_stop_required: bool,
    component_mask: u32,
    nan_count: u32,
    sanitized_x: f32,
    sanitized_y: f32,
    sanitized_z: f32,
    fallback_value: f32,
    reason_code: u32,
};

pub const InfinityHandlingPlan = struct {
    valid: bool,
    infinity_detected: bool,
    sanitized: bool,
    emergency_stop_required: bool,
    component_mask: u32,
    infinity_count: u32,
    sanitized_x: f32,
    sanitized_y: f32,
    sanitized_z: f32,
    clamp_abs: f32,
    reason_code: u32,
};

pub const BoundsCorrectionPlan = struct {
    valid: bool,
    bounds_violated: bool,
    corrected: bool,
    emergency_stop_required: bool,
    component_mask: u32,
    corrected_x: f32,
    corrected_y: f32,
    corrected_z: f32,
    correction_distance: f32,
    max_correction_distance: f32,
    reason_code: u32,
};

pub const EnergyLimitPlan = struct {
    valid: bool,
    energy_violation: bool,
    clamped: bool,
    emergency_stop_required: bool,
    current_energy: f32,
    reference_energy: f32,
    allowed_energy: f32,
    excess_energy: f32,
    relative_error: f32,
    safe_scale: f32,
    reason_code: u32,
};

pub const VelocityLimitPlan = struct {
    valid: bool,
    velocity_exceeded: bool,
    clamped: bool,
    emergency_stop_required: bool,
    current_speed: f32,
    allowed_speed: f32,
    excess_speed: f32,
    safe_scale: f32,
    clamped_x: f32,
    clamped_y: f32,
    clamped_z: f32,
    reason_code: u32,
};

pub const PositionRangeLimitPlan = struct {
    valid: bool,
    range_violation: bool,
    clamped: bool,
    emergency_stop_required: bool,
    current_offset: f32,
    allowed_offset: f32,
    excess_offset: f32,
    safe_scale: f32,
    corrected_x: f32,
    corrected_y: f32,
    corrected_z: f32,
    reason_code: u32,
};

pub const TorqueLimitPlan = struct {
    valid: bool,
    torque_exceeded: bool,
    clamped: bool,
    emergency_stop_required: bool,
    current_torque: f32,
    allowed_torque: f32,
    excess_torque: f32,
    safe_scale: f32,
    clamped_x: f32,
    clamped_y: f32,
    clamped_z: f32,
    reason_code: u32,
};

pub const SolverDivergencePlan = struct {
    valid: bool,
    diverging: bool,
    reset_required: bool,
    emergency_stop_required: bool,
    previous_error: f32,
    current_error: f32,
    max_allowed_error: f32,
    growth_ratio: f32,
    allowed_growth_ratio: f32,
    excess_error: f32,
    reason_code: u32,
};

pub const Snapshot = struct {
    tick: u32,
    instance_count: u8,
    instances: [128]scene32.Instance,
    energy_sum: f32,
    timestamp: u64,
};

pub const DefenseSystem = struct {
    config: SanityCheckConfig,
    last_progress_tick: u32,
    emergency_stopped: bool,
    snapshot_count: u8,
    snapshots: [4]Snapshot,
};

var g_defense_system: DefenseSystem = undefined;

pub fn init(config: SanityCheckConfig) void {
    g_defense_system.config = config;
    g_defense_system.last_progress_tick = 0;
    g_defense_system.emergency_stopped = false;
    g_defense_system.snapshot_count = 0;
}

/// Check if float is NaN
pub fn isNaN(value: f32) bool {
    // NaN is the only float that compares unequal to itself
    return value != value;
}

/// Check if float is infinite
pub fn isInfinite(value: f32) bool {
    const inf_bits: u32 = 0x7F800000;
    const inf: f32 = @bitCast(inf_bits);
    return @abs(value) == inf;
}

/// Check if value is valid (not NaN, not infinite)
pub fn isValidFloat(value: f32) bool {
    return !isNaN(value) and !isInfinite(value);
}

fn invalidNaNHandlingPlan() NaNHandlingPlan {
    return .{
        .valid = false,
        .nan_detected = false,
        .sanitized = false,
        .emergency_stop_required = false,
        .component_mask = 0,
        .nan_count = 0,
        .sanitized_x = 0.0,
        .sanitized_y = 0.0,
        .sanitized_z = 0.0,
        .fallback_value = 0.0,
        .reason_code = 0,
    };
}

fn invalidInfinityHandlingPlan() InfinityHandlingPlan {
    return .{
        .valid = false,
        .infinity_detected = false,
        .sanitized = false,
        .emergency_stop_required = false,
        .component_mask = 0,
        .infinity_count = 0,
        .sanitized_x = 0.0,
        .sanitized_y = 0.0,
        .sanitized_z = 0.0,
        .clamp_abs = 0.0,
        .reason_code = 0,
    };
}

fn invalidBoundsCorrectionPlan() BoundsCorrectionPlan {
    return .{
        .valid = false,
        .bounds_violated = false,
        .corrected = false,
        .emergency_stop_required = false,
        .component_mask = 0,
        .corrected_x = 0.0,
        .corrected_y = 0.0,
        .corrected_z = 0.0,
        .correction_distance = 0.0,
        .max_correction_distance = 0.0,
        .reason_code = 0,
    };
}

fn invalidEnergyLimitPlan() EnergyLimitPlan {
    return .{
        .valid = false,
        .energy_violation = false,
        .clamped = false,
        .emergency_stop_required = false,
        .current_energy = 0.0,
        .reference_energy = 0.0,
        .allowed_energy = 0.0,
        .excess_energy = 0.0,
        .relative_error = 0.0,
        .safe_scale = 1.0,
        .reason_code = 0,
    };
}

fn invalidVelocityLimitPlan() VelocityLimitPlan {
    return .{
        .valid = false,
        .velocity_exceeded = false,
        .clamped = false,
        .emergency_stop_required = false,
        .current_speed = 0.0,
        .allowed_speed = 0.0,
        .excess_speed = 0.0,
        .safe_scale = 1.0,
        .clamped_x = 0.0,
        .clamped_y = 0.0,
        .clamped_z = 0.0,
        .reason_code = 0,
    };
}

fn invalidPositionRangeLimitPlan() PositionRangeLimitPlan {
    return .{
        .valid = false,
        .range_violation = false,
        .clamped = false,
        .emergency_stop_required = false,
        .current_offset = 0.0,
        .allowed_offset = 0.0,
        .excess_offset = 0.0,
        .safe_scale = 1.0,
        .corrected_x = 0.0,
        .corrected_y = 0.0,
        .corrected_z = 0.0,
        .reason_code = 0,
    };
}

fn invalidTorqueLimitPlan() TorqueLimitPlan {
    return .{
        .valid = false,
        .torque_exceeded = false,
        .clamped = false,
        .emergency_stop_required = false,
        .current_torque = 0.0,
        .allowed_torque = 0.0,
        .excess_torque = 0.0,
        .safe_scale = 1.0,
        .clamped_x = 0.0,
        .clamped_y = 0.0,
        .clamped_z = 0.0,
        .reason_code = 0,
    };
}

fn invalidSolverDivergencePlan() SolverDivergencePlan {
    return .{
        .valid = false,
        .diverging = false,
        .reset_required = false,
        .emergency_stop_required = false,
        .previous_error = 0.0,
        .current_error = 0.0,
        .max_allowed_error = 0.0,
        .growth_ratio = 0.0,
        .allowed_growth_ratio = 0.0,
        .excess_error = 0.0,
        .reason_code = 0,
    };
}

fn magnitude3(x: f32, y: f32, z: f32) f32 {
    return @sqrt(x * x + y * y + z * z);
}

/// Detect and sanitize NaN components in a 3D float state.
pub fn computeNaNHandlingPlan(
    x: f32,
    y: f32,
    z: f32,
    fallback_value: f32,
    max_nan_components: u32,
    emergency_on_nan: bool,
) NaNHandlingPlan {
    if (isNaN(fallback_value) or isInfinite(fallback_value)) return invalidNaNHandlingPlan();

    const x_nan = isNaN(x);
    const y_nan = isNaN(y);
    const z_nan = isNaN(z);
    const component_mask: u32 = (if (x_nan) @as(u32, 1) else 0) |
        (if (y_nan) @as(u32, 2) else 0) |
        (if (z_nan) @as(u32, 4) else 0);
    const nan_count: u32 = (if (x_nan) @as(u32, 1) else 0) +
        (if (y_nan) @as(u32, 1) else 0) +
        (if (z_nan) @as(u32, 1) else 0);
    const nan_detected = nan_count != 0;
    const emergency_stop_required = nan_detected and (emergency_on_nan or nan_count > max_nan_components);

    return .{
        .valid = true,
        .nan_detected = nan_detected,
        .sanitized = nan_detected,
        .emergency_stop_required = emergency_stop_required,
        .component_mask = component_mask,
        .nan_count = nan_count,
        .sanitized_x = if (x_nan) fallback_value else x,
        .sanitized_y = if (y_nan) fallback_value else y,
        .sanitized_z = if (z_nan) fallback_value else z,
        .fallback_value = fallback_value,
        .reason_code = if (emergency_stop_required) 2 else if (nan_detected) 1 else 0,
    };
}

fn sanitizeInfiniteComponent(value: f32, clamp_abs: f32) f32 {
    if (!isInfinite(value)) return value;
    return if (value < 0.0) -clamp_abs else clamp_abs;
}

fn clampToBounds(value: f32, min_value: f32, max_value: f32) f32 {
    return @max(min_value, @min(max_value, value));
}

/// Detect and clamp infinite components in a 3D float state.
pub fn computeInfinityHandlingPlan(
    x: f32,
    y: f32,
    z: f32,
    clamp_abs: f32,
    max_infinite_components: u32,
    emergency_on_infinity: bool,
) InfinityHandlingPlan {
    if (isNaN(x) or isNaN(y) or isNaN(z) or isNaN(clamp_abs) or isInfinite(clamp_abs) or clamp_abs < 0.0) return invalidInfinityHandlingPlan();

    const x_inf = isInfinite(x);
    const y_inf = isInfinite(y);
    const z_inf = isInfinite(z);
    const component_mask: u32 = (if (x_inf) @as(u32, 1) else 0) |
        (if (y_inf) @as(u32, 2) else 0) |
        (if (z_inf) @as(u32, 4) else 0);
    const infinity_count: u32 = (if (x_inf) @as(u32, 1) else 0) +
        (if (y_inf) @as(u32, 1) else 0) +
        (if (z_inf) @as(u32, 1) else 0);
    const infinity_detected = infinity_count != 0;
    const emergency_stop_required = infinity_detected and (emergency_on_infinity or infinity_count > max_infinite_components);

    return .{
        .valid = true,
        .infinity_detected = infinity_detected,
        .sanitized = infinity_detected,
        .emergency_stop_required = emergency_stop_required,
        .component_mask = component_mask,
        .infinity_count = infinity_count,
        .sanitized_x = sanitizeInfiniteComponent(x, clamp_abs),
        .sanitized_y = sanitizeInfiniteComponent(y, clamp_abs),
        .sanitized_z = sanitizeInfiniteComponent(z, clamp_abs),
        .clamp_abs = clamp_abs,
        .reason_code = if (emergency_stop_required) 2 else if (infinity_detected) 1 else 0,
    };
}

/// Detect and clamp out-of-bounds position components.
pub fn computeBoundsCorrectionPlan(
    x: f32,
    y: f32,
    z: f32,
    position_min: f32,
    position_max: f32,
    max_correction_distance: f32,
    emergency_on_escape: bool,
) BoundsCorrectionPlan {
    if (!std.math.isFinite(x) or !std.math.isFinite(y) or !std.math.isFinite(z) or
        !std.math.isFinite(position_min) or !std.math.isFinite(position_max) or
        !std.math.isFinite(max_correction_distance) or position_min > position_max or max_correction_distance < 0.0) return invalidBoundsCorrectionPlan();

    const x_out = x < position_min or x > position_max;
    const y_out = y < position_min or y > position_max;
    const z_out = z < position_min or z > position_max;
    const component_mask: u32 = (if (x_out) @as(u32, 1) else 0) |
        (if (y_out) @as(u32, 2) else 0) |
        (if (z_out) @as(u32, 4) else 0);
    const corrected_x = clampToBounds(x, position_min, position_max);
    const corrected_y = clampToBounds(y, position_min, position_max);
    const corrected_z = clampToBounds(z, position_min, position_max);
    const correction_distance = @abs(corrected_x - x) + @abs(corrected_y - y) + @abs(corrected_z - z);
    const bounds_violated = component_mask != 0;
    const emergency_stop_required = bounds_violated and (emergency_on_escape or correction_distance > max_correction_distance);

    return .{
        .valid = true,
        .bounds_violated = bounds_violated,
        .corrected = bounds_violated,
        .emergency_stop_required = emergency_stop_required,
        .component_mask = component_mask,
        .corrected_x = corrected_x,
        .corrected_y = corrected_y,
        .corrected_z = corrected_z,
        .correction_distance = correction_distance,
        .max_correction_distance = max_correction_distance,
        .reason_code = if (emergency_stop_required) 2 else if (bounds_violated) 1 else 0,
    };
}

/// Detect energy drift and derive a safe clamp scale.
pub fn computeEnergyLimitPlan(
    current_energy: f32,
    reference_energy: f32,
    allowed_energy: f32,
    relative_tolerance: f32,
    hard_limit_scale: f32,
    emergency_on_violation: bool,
) EnergyLimitPlan {
    if (!std.math.isFinite(current_energy) or !std.math.isFinite(reference_energy) or
        !std.math.isFinite(allowed_energy) or !std.math.isFinite(relative_tolerance) or
        !std.math.isFinite(hard_limit_scale) or current_energy < 0.0 or reference_energy < 0.0 or
        allowed_energy < 0.0 or relative_tolerance < 0.0 or hard_limit_scale < 1.0) return invalidEnergyLimitPlan();

    const baseline = @max(@max(reference_energy, allowed_energy), 1.0);
    const relative_error = @abs(current_energy - reference_energy) / baseline;
    const over_limit = current_energy > allowed_energy;
    const tolerance_violated = relative_error > relative_tolerance;
    const energy_violation = over_limit or tolerance_violated;
    const excess_energy = if (current_energy > allowed_energy) current_energy - allowed_energy else 0.0;
    const clamped = current_energy > 0.0 and current_energy > allowed_energy;
    const safe_scale: f32 = if (clamped and allowed_energy > 0.0)
        @as(f32, @sqrt(allowed_energy / current_energy))
    else if (allowed_energy == 0.0 and current_energy > 0.0)
        @as(f32, 0.0)
    else
        @as(f32, 1.0);
    const emergency_threshold = allowed_energy * hard_limit_scale;
    const emergency_stop_required = energy_violation and (emergency_on_violation or current_energy > emergency_threshold);

    return .{
        .valid = true,
        .energy_violation = energy_violation,
        .clamped = clamped,
        .emergency_stop_required = emergency_stop_required,
        .current_energy = current_energy,
        .reference_energy = reference_energy,
        .allowed_energy = allowed_energy,
        .excess_energy = excess_energy,
        .relative_error = relative_error,
        .safe_scale = safe_scale,
        .reason_code = if (emergency_stop_required) 3 else if (over_limit) 1 else if (tolerance_violated) 2 else 0,
    };
}

/// Detect speed overflow and derive a safe velocity clamp.
pub fn computeVelocityLimitPlan(
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    allowed_speed: f32,
    hard_limit_scale: f32,
    emergency_on_violation: bool,
) VelocityLimitPlan {
    if (!std.math.isFinite(vel_x) or !std.math.isFinite(vel_y) or !std.math.isFinite(vel_z) or
        !std.math.isFinite(allowed_speed) or !std.math.isFinite(hard_limit_scale) or
        allowed_speed < 0.0 or hard_limit_scale < 1.0) return invalidVelocityLimitPlan();

    const current_speed = magnitude3(vel_x, vel_y, vel_z);
    const velocity_exceeded = current_speed > allowed_speed;
    const clamped = velocity_exceeded and current_speed > 0.0;
    const safe_scale: f32 = if (clamped and allowed_speed > 0.0)
        allowed_speed / current_speed
    else if (allowed_speed == 0.0 and current_speed > 0.0)
        0.0
    else
        1.0;
    const excess_speed = if (velocity_exceeded) current_speed - allowed_speed else 0.0;
    const emergency_threshold = allowed_speed * hard_limit_scale;
    const emergency_stop_required = velocity_exceeded and (emergency_on_violation or current_speed > emergency_threshold);

    return .{
        .valid = true,
        .velocity_exceeded = velocity_exceeded,
        .clamped = clamped,
        .emergency_stop_required = emergency_stop_required,
        .current_speed = current_speed,
        .allowed_speed = allowed_speed,
        .excess_speed = excess_speed,
        .safe_scale = safe_scale,
        .clamped_x = vel_x * safe_scale,
        .clamped_y = vel_y * safe_scale,
        .clamped_z = vel_z * safe_scale,
        .reason_code = if (emergency_stop_required) 2 else if (velocity_exceeded) 1 else 0,
    };
}

/// Detect position drift from a reference point and derive a safe correction.
pub fn computePositionRangeLimitPlan(
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    reference_x: f32,
    reference_y: f32,
    reference_z: f32,
    allowed_offset: f32,
    hard_limit_scale: f32,
    emergency_on_violation: bool,
) PositionRangeLimitPlan {
    if (!std.math.isFinite(pos_x) or !std.math.isFinite(pos_y) or !std.math.isFinite(pos_z) or
        !std.math.isFinite(reference_x) or !std.math.isFinite(reference_y) or !std.math.isFinite(reference_z) or
        !std.math.isFinite(allowed_offset) or !std.math.isFinite(hard_limit_scale) or
        allowed_offset < 0.0 or hard_limit_scale < 1.0) return invalidPositionRangeLimitPlan();

    const delta_x = pos_x - reference_x;
    const delta_y = pos_y - reference_y;
    const delta_z = pos_z - reference_z;
    const current_offset = magnitude3(delta_x, delta_y, delta_z);
    const range_violation = current_offset > allowed_offset;
    const clamped = range_violation and current_offset > 0.0;
    const safe_scale: f32 = if (clamped and allowed_offset > 0.0)
        allowed_offset / current_offset
    else if (allowed_offset == 0.0 and current_offset > 0.0)
        0.0
    else
        1.0;
    const excess_offset = if (range_violation) current_offset - allowed_offset else 0.0;
    const emergency_threshold = allowed_offset * hard_limit_scale;
    const emergency_stop_required = range_violation and (emergency_on_violation or current_offset > emergency_threshold);

    return .{
        .valid = true,
        .range_violation = range_violation,
        .clamped = clamped,
        .emergency_stop_required = emergency_stop_required,
        .current_offset = current_offset,
        .allowed_offset = allowed_offset,
        .excess_offset = excess_offset,
        .safe_scale = safe_scale,
        .corrected_x = reference_x + delta_x * safe_scale,
        .corrected_y = reference_y + delta_y * safe_scale,
        .corrected_z = reference_z + delta_z * safe_scale,
        .reason_code = if (emergency_stop_required) 2 else if (range_violation) 1 else 0,
    };
}

/// Detect torque overflow and derive a safe torque clamp.
pub fn computeTorqueLimitPlan(
    torque_x: f32,
    torque_y: f32,
    torque_z: f32,
    allowed_torque: f32,
    hard_limit_scale: f32,
    emergency_on_violation: bool,
) TorqueLimitPlan {
    if (!std.math.isFinite(torque_x) or !std.math.isFinite(torque_y) or !std.math.isFinite(torque_z) or
        !std.math.isFinite(allowed_torque) or !std.math.isFinite(hard_limit_scale) or
        allowed_torque < 0.0 or hard_limit_scale < 1.0) return invalidTorqueLimitPlan();

    const current_torque = magnitude3(torque_x, torque_y, torque_z);
    const torque_exceeded = current_torque > allowed_torque;
    const clamped = torque_exceeded and current_torque > 0.0;
    const safe_scale: f32 = if (clamped and allowed_torque > 0.0)
        allowed_torque / current_torque
    else if (allowed_torque == 0.0 and current_torque > 0.0)
        0.0
    else
        1.0;
    const excess_torque = if (torque_exceeded) current_torque - allowed_torque else 0.0;
    const emergency_threshold = allowed_torque * hard_limit_scale;
    const emergency_stop_required = torque_exceeded and (emergency_on_violation or current_torque > emergency_threshold);

    return .{
        .valid = true,
        .torque_exceeded = torque_exceeded,
        .clamped = clamped,
        .emergency_stop_required = emergency_stop_required,
        .current_torque = current_torque,
        .allowed_torque = allowed_torque,
        .excess_torque = excess_torque,
        .safe_scale = safe_scale,
        .clamped_x = torque_x * safe_scale,
        .clamped_y = torque_y * safe_scale,
        .clamped_z = torque_z * safe_scale,
        .reason_code = if (emergency_stop_required) 2 else if (torque_exceeded) 1 else 0,
    };
}

/// Detect solver divergence from rapidly growing residual error.
pub fn computeSolverDivergencePlan(
    previous_error: f32,
    current_error: f32,
    max_allowed_error: f32,
    allowed_growth_ratio: f32,
    emergency_growth_ratio: f32,
    emergency_on_divergence: bool,
) SolverDivergencePlan {
    if (!std.math.isFinite(previous_error) or !std.math.isFinite(current_error) or
        !std.math.isFinite(max_allowed_error) or !std.math.isFinite(allowed_growth_ratio) or
        !std.math.isFinite(emergency_growth_ratio) or previous_error < 0.0 or current_error < 0.0 or
        max_allowed_error < 0.0 or allowed_growth_ratio < 1.0 or emergency_growth_ratio < allowed_growth_ratio)
    {
        return invalidSolverDivergencePlan();
    }

    const baseline_error = @max(previous_error, 0.0001);
    const growth_ratio = if (current_error == 0.0) 0.0 else current_error / baseline_error;
    const growth_divergence = current_error > previous_error and growth_ratio > allowed_growth_ratio;
    const hard_limit_violation = current_error > max_allowed_error;
    const diverging = growth_divergence or hard_limit_violation;
    const excess_error = if (hard_limit_violation) current_error - max_allowed_error else 0.0;
    const severe_growth = current_error > previous_error and growth_ratio > emergency_growth_ratio;
    const emergency_stop_required = diverging and (emergency_on_divergence or severe_growth);

    return .{
        .valid = true,
        .diverging = diverging,
        .reset_required = diverging,
        .emergency_stop_required = emergency_stop_required,
        .previous_error = previous_error,
        .current_error = current_error,
        .max_allowed_error = max_allowed_error,
        .growth_ratio = growth_ratio,
        .allowed_growth_ratio = allowed_growth_ratio,
        .excess_error = excess_error,
        .reason_code = if (emergency_stop_required) 3 else if (hard_limit_violation) 2 else if (growth_divergence) 1 else 0,
    };
}

/// Validate physics state of single instance
pub fn validateInstance(inst: *const scene32.Instance) SanityCheckResult {
    var result: SanityCheckResult = .{
        .nan_detected = false,
        .bounds_violated = false,
        .energy_violation = false,
        .velocity_exceeded = false,
        .instance_errors = 0,
    };

    if (g_defense_system.config.nan_check_enabled) {
        if (isNaN(@as(f32, @floatFromInt(inst.vel_x))) or
            isNaN(@as(f32, @floatFromInt(inst.vel_y))) or
            isNaN(@as(f32, @floatFromInt(inst.vel_z))))
        {
            result.nan_detected = true;
            result.instance_errors += 1;
        }
    }

    if (g_defense_system.config.bounds_check_enabled) {
        if (inst.pos_x < g_defense_system.config.position_min or
            inst.pos_x > g_defense_system.config.position_max or
            inst.pos_y < g_defense_system.config.position_min or
            inst.pos_y > g_defense_system.config.position_max or
            inst.pos_z < g_defense_system.config.position_min or
            inst.pos_z > g_defense_system.config.position_max)
        {
            result.bounds_violated = true;
            result.instance_errors += 1;
        }
    }

    const vel_magnitude = magnitude3(
        @as(f32, @floatFromInt(inst.vel_x)),
        @as(f32, @floatFromInt(inst.vel_y)),
        @as(f32, @floatFromInt(inst.vel_z)),
    );

    if (vel_magnitude > g_defense_system.config.velocity_cap) {
        result.velocity_exceeded = true;
        result.instance_errors += 1;
    }

    return result;
}

/// Validate all instances in scene
pub fn validateScene(s1024: *const scene1024.Scene1024, entities: []entity16.Entity16) SanityCheckResult {
    var result: SanityCheckResult = .{
        .nan_detected = false,
        .bounds_violated = false,
        .energy_violation = false,
        .velocity_exceeded = false,
        .instance_errors = 0,
    };

    _ = entities;

    for (0..s1024.instance_count) |i| {
        const inst_result = validateInstance(&s1024.instances[i]);
        if (inst_result.nan_detected) result.nan_detected = true;
        if (inst_result.bounds_violated) result.bounds_violated = true;
        if (inst_result.velocity_exceeded) result.velocity_exceeded = true;
        result.instance_errors += inst_result.instance_errors;
    }

    return result;
}

/// Clamp instance state to valid range
pub fn clampInstance(inst: *scene32.Instance) void {
    if (!isValidFloat(@as(f32, @floatFromInt(inst.vel_x)))) {
        inst.vel_x = 0;
    }
    if (!isValidFloat(@as(f32, @floatFromInt(inst.vel_y)))) {
        inst.vel_y = 0;
    }
    if (!isValidFloat(@as(f32, @floatFromInt(inst.vel_z)))) {
        inst.vel_z = 0;
    }

    const velocity_plan = computeVelocityLimitPlan(
        @as(f32, @floatFromInt(inst.vel_x)),
        @as(f32, @floatFromInt(inst.vel_y)),
        @as(f32, @floatFromInt(inst.vel_z)),
        g_defense_system.config.velocity_cap,
        2.0,
        false,
    );
    if (velocity_plan.valid and velocity_plan.clamped) {
        inst.vel_x = @intFromFloat(velocity_plan.clamped_x);
        inst.vel_y = @intFromFloat(velocity_plan.clamped_y);
        inst.vel_z = @intFromFloat(velocity_plan.clamped_z);
    }

    if (inst.pos_x < g_defense_system.config.position_min) inst.pos_x = g_defense_system.config.position_min;
    if (inst.pos_x > g_defense_system.config.position_max) inst.pos_x = g_defense_system.config.position_max;
    if (inst.pos_y < g_defense_system.config.position_min) inst.pos_y = g_defense_system.config.position_min;
    if (inst.pos_y > g_defense_system.config.position_max) inst.pos_y = g_defense_system.config.position_max;
    if (inst.pos_z < g_defense_system.config.position_min) inst.pos_z = g_defense_system.config.position_min;
    if (inst.pos_z > g_defense_system.config.position_max) inst.pos_z = g_defense_system.config.position_max;
}

/// Clamp all instances in scene
pub fn clampScene(s1024: *scene1024.Scene1024) void {
    for (0..s1024.instance_count) |i| {
        clampInstance(&s1024.instances[i]);
    }
}

/// Calculate total kinetic energy
pub fn calculateEnergy(s1024: *const scene1024.Scene1024, entities: []entity16.Entity16) f32 {
    var total_energy: f32 = 0;

    for (0..s1024.instance_count) |i| {
        const inst = &s1024.instances[i];
        if (inst.entity_id >= entities.len) continue;

        const ent = &entities[inst.entity_id];
        const mass = @as(f32, @floatFromInt(ent.physics.mass));

        const vel_sq = @as(f32, @floatFromInt(inst.vel_x * inst.vel_x)) +
            @as(f32, @floatFromInt(inst.vel_y * inst.vel_y)) +
            @as(f32, @floatFromInt(inst.vel_z * inst.vel_z));

        const kinetic = 0.5 * mass * vel_sq;
        const potential = mass * @as(f32, @floatFromInt(inst.pos_y)) * 9.8;

        total_energy += kinetic + potential;
    }

    return total_energy;
}

/// Check energy conservation (for closed systems)
pub fn checkEnergyConservation(
    s1024: *const scene1024.Scene1024,
    entities: []entity16.Entity16,
    initial_energy: f32,
    tolerance: f32,
) bool {
    if (!g_defense_system.config.energy_check_enabled) return true;

    const current_energy = calculateEnergy(s1024, entities);
    const energy_diff = @abs(current_energy - initial_energy);

    return energy_diff < tolerance * @abs(initial_energy);
}

/// Emergency stop all physics
pub fn emergencyStop(s1024: *scene1024.Scene1024) void {
    for (0..s1024.instance_count) |i| {
        const inst = &s1024.instances[i];
        inst.vel_x = 0;
        inst.vel_y = 0;
        inst.vel_z = 0;
        inst.ang_x = 0;
        inst.ang_y = 0;
        inst.ang_z = 0;
        inst.state = .resting;
    }
    g_defense_system.emergency_stopped = true;
}

/// Reset emergency stop flag
pub fn resetEmergencyStop() void {
    g_defense_system.emergency_stopped = false;
}

/// Check if emergency stopped
pub fn isEmergencyStopped() bool {
    return g_defense_system.emergency_stopped;
}

/// Save snapshot for recovery
pub fn saveSnapshot(s1024: *const scene1024.Scene1024, tick: u32) void {
    const idx = g_defense_system.snapshot_count % 4;
    var snap = &g_defense_system.snapshots[idx];

    snap.tick = tick;
    snap.instance_count = s1024.instance_count;
    snap.energy_sum = calculateEnergy(s1024, undefined);
    snap.timestamp = @as(u64, tick);

    for (s1024.instances[0..s1024.instance_count], 0..) |inst, i| {
        snap.instances[i] = inst;
    }

    g_defense_system.snapshot_count += 1;
}

/// Restore from most recent valid snapshot
pub fn restoreSnapshot(s1024: *scene1024.Scene1024) bool {
    if (g_defense_system.snapshot_count == 0) return false;

    const idx = (g_defense_system.snapshot_count - 1) % 4;
    const snap = g_defense_system.snapshots[idx];

    s1024.instance_count = snap.instance_count;

    for (snap.instances[0..snap.instance_count], 0..) |inst, i| {
        s1024.instances[i] = inst;
    }

    return true;
}

/// Find first valid state by checking snapshots
pub fn findValidSnapshot() ?*const Snapshot {
    if (g_defense_system.snapshot_count == 0) return null;

    var i: u8 = g_defense_system.snapshot_count;
    while (i > 0) {
        i -= 1;
        const idx = i % 4;
        const snap = &g_defense_system.snapshots[idx];

        var valid = true;
        for (snap.instances[0..snap.instance_count]) |inst| {
            if (!isValidFloat(@as(f32, @floatFromInt(inst.vel_x))) or
                !isValidFloat(@as(f32, @floatFromInt(inst.vel_y))) or
                !isValidFloat(@as(f32, @floatFromInt(inst.vel_z))) or
                inst.pos_x < g_defense_system.config.position_min or
                inst.pos_x > g_defense_system.config.position_max)
            {
                valid = false;
                break;
            }
        }

        if (valid) return snap;
    }

    return null;
}

/// Update progress tracking
pub fn updateProgress(tick: u32) void {
    g_defense_system.last_progress_tick = tick;
}

/// Check if simulation is stuck
pub fn isStuck(current_tick: u32) bool {
    const ticks_without_progress = current_tick - g_defense_system.last_progress_tick;
    return ticks_without_progress > g_defense_system.config.max_ticks_without_progress;
}

/// Load shedding - reduce simulation load
pub fn shouldReduceLoad(s1024: *const scene1024.Scene1024) LoadReduction {
    if (s1024.instance_count > 100) return .REDUCE_INSTANCES;
    if (isStuck(s1024.global_tick)) return .PAUSE_SIMULATION;
    return .NONE;
}

pub const LoadReduction = enum(u8) {
    NONE = 0,
    REDUCE_INSTANCES = 1,
    REDUCE_ITERATIONS = 2,
    PAUSE_SIMULATION = 3,
};

/// Get system for external access
pub fn getSystem() *DefenseSystem {
    return &g_defense_system;
}

test "computeNaNHandlingPlan sanitizes NaN components" {
    const nan: f32 = @bitCast(@as(u32, 0x7FC00000));
    const plan = computeNaNHandlingPlan(nan, 2.0, nan, 0.0, 2, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.nan_detected);
    try std.testing.expect(plan.sanitized);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectEqual(@as(u32, 5), plan.component_mask);
    try std.testing.expectEqual(@as(u32, 2), plan.nan_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.sanitized_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), plan.sanitized_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.sanitized_z, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeNaNHandlingPlan escalates excessive NaN components" {
    const nan: f32 = @bitCast(@as(u32, 0x7FC00000));
    const plan = computeNaNHandlingPlan(nan, nan, nan, -1.0, 1, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectEqual(@as(u32, 7), plan.component_mask);
    try std.testing.expectEqual(@as(u32, 3), plan.nan_count);
    try std.testing.expectEqual(@as(u32, 2), plan.reason_code);
}

test "computeInfinityHandlingPlan clamps infinite components" {
    const inf: f32 = @bitCast(@as(u32, 0x7F800000));
    const plan = computeInfinityHandlingPlan(inf, 2.0, -inf, 100.0, 2, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.infinity_detected);
    try std.testing.expect(plan.sanitized);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectEqual(@as(u32, 5), plan.component_mask);
    try std.testing.expectEqual(@as(u32, 2), plan.infinity_count);
    try std.testing.expectApproxEqAbs(@as(f32, 100.0), plan.sanitized_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), plan.sanitized_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -100.0), plan.sanitized_z, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeInfinityHandlingPlan escalates excessive infinite components" {
    const inf: f32 = @bitCast(@as(u32, 0x7F800000));
    const plan = computeInfinityHandlingPlan(inf, -inf, inf, 100.0, 1, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectEqual(@as(u32, 7), plan.component_mask);
    try std.testing.expectEqual(@as(u32, 3), plan.infinity_count);
    try std.testing.expectEqual(@as(u32, 2), plan.reason_code);
}

test "computeBoundsCorrectionPlan clamps out-of-range position" {
    const plan = computeBoundsCorrectionPlan(-150.0, 25.0, 120.0, -100.0, 100.0, 80.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.bounds_violated);
    try std.testing.expect(plan.corrected);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectEqual(@as(u32, 5), plan.component_mask);
    try std.testing.expectApproxEqAbs(@as(f32, -100.0), plan.corrected_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 25.0), plan.corrected_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 100.0), plan.corrected_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 70.0), plan.correction_distance, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeBoundsCorrectionPlan escalates large correction distance" {
    const plan = computeBoundsCorrectionPlan(-250.0, 25.0, 220.0, -100.0, 100.0, 80.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectEqual(@as(u32, 5), plan.component_mask);
    try std.testing.expectApproxEqAbs(@as(f32, 270.0), plan.correction_distance, 0.0001);
    try std.testing.expectEqual(@as(u32, 2), plan.reason_code);
}

test "computeEnergyLimitPlan clamps excess energy" {
    const plan = computeEnergyLimitPlan(400.0, 220.0, 250.0, 0.1, 2.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.energy_violation);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 150.0), plan.excess_energy, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.72), plan.relative_error, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.7905694), plan.safe_scale, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeEnergyLimitPlan escalates severe energy spike" {
    const plan = computeEnergyLimitPlan(700.0, 220.0, 250.0, 0.1, 2.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.energy_violation);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 450.0), plan.excess_energy, 0.0001);
    try std.testing.expectEqual(@as(u32, 3), plan.reason_code);
}

test "computeVelocityLimitPlan clamps speed magnitude" {
    const plan = computeVelocityLimitPlan(6.0, 8.0, 0.0, 5.0, 2.5, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.velocity_exceeded);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.current_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), plan.excess_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), plan.safe_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), plan.clamped_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), plan.clamped_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.clamped_z, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeVelocityLimitPlan escalates severe speed spike" {
    const plan = computeVelocityLimitPlan(30.0, 40.0, 0.0, 10.0, 2.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.velocity_exceeded);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 50.0), plan.current_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 40.0), plan.excess_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), plan.safe_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), plan.clamped_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), plan.clamped_y, 0.0001);
    try std.testing.expectEqual(@as(u32, 2), plan.reason_code);
}

test "computePositionRangeLimitPlan clamps drift from reference position" {
    const plan = computePositionRangeLimitPlan(16.0, 18.0, 10.0, 10.0, 10.0, 10.0, 5.0, 3.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.range_violation);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.current_offset, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), plan.excess_offset, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), plan.safe_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 13.0), plan.corrected_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 14.0), plan.corrected_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.corrected_z, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computePositionRangeLimitPlan escalates severe drift" {
    const plan = computePositionRangeLimitPlan(40.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 2.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.range_violation);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 30.0), plan.current_offset, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 20.0), plan.excess_offset, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 20.0), plan.corrected_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.corrected_y, 0.0001);
    try std.testing.expectEqual(@as(u32, 2), plan.reason_code);
}

test "computeTorqueLimitPlan clamps torque magnitude" {
    const plan = computeTorqueLimitPlan(6.0, 8.0, 0.0, 5.0, 2.5, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.torque_exceeded);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.current_torque, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), plan.excess_torque, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), plan.safe_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), plan.clamped_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), plan.clamped_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.clamped_z, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeTorqueLimitPlan escalates severe torque spike" {
    const plan = computeTorqueLimitPlan(30.0, 40.0, 0.0, 10.0, 2.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.torque_exceeded);
    try std.testing.expect(plan.clamped);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 50.0), plan.current_torque, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 40.0), plan.excess_torque, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), plan.clamped_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), plan.clamped_y, 0.0001);
    try std.testing.expectEqual(@as(u32, 2), plan.reason_code);
}

test "computeSolverDivergencePlan detects residual growth divergence" {
    const plan = computeSolverDivergencePlan(2.0, 5.0, 10.0, 2.0, 4.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.diverging);
    try std.testing.expect(plan.reset_required);
    try std.testing.expect(!plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 2.5), plan.growth_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.excess_error, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeSolverDivergencePlan escalates severe divergence" {
    const plan = computeSolverDivergencePlan(2.0, 12.0, 10.0, 2.0, 4.0, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.diverging);
    try std.testing.expect(plan.reset_required);
    try std.testing.expect(plan.emergency_stop_required);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), plan.growth_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), plan.excess_error, 0.0001);
    try std.testing.expectEqual(@as(u32, 3), plan.reason_code);
}
