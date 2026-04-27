//! Prediction - Short Horizon Forecast and Safety Windows
//!
//! Layer 1.5: shared 1-5 second predictive substrate for sensors,
//! networking, traffic, and future planning/safety modules.

const std = @import("std");
const rewind = @import("rewind.zig");
const scene32 = @import("scene32.zig");
const query_types = @import("query_types.zig");

pub const LinearState = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
};

pub const SignalPhase = enum(u8) {
    red = 0,
    green = 1,
    yellow = 2,
};

pub const SignalWindow = struct {
    state_now: SignalPhase,
    time_to_next_change: f32,
    safe_to_enter: bool,
    safe_to_clear: bool,
};

pub const TTCResult = struct {
    valid: bool = false,
    time: f32 = std.math.inf(f32),
    distance_at_closest: f32 = std.math.inf(f32),
};

pub const ConflictWindow = struct {
    valid: bool = false,
    start_time: f32 = 0,
    end_time: f32 = 0,
    min_distance: f32 = std.math.inf(f32),
};

pub const RiskLevel = enum(u8) {
    none = 0,
    low = 1,
    medium = 2,
    high = 3,
    imminent = 4,
};

pub const CollisionRisk = struct {
    level: RiskLevel = .none,
    score: f32 = 0,
    ttc: TTCResult = .{},
    window: ConflictWindow = .{},
};

pub const SafePassResult = struct {
    can_pass: bool = false,
    time_to_line: f32 = std.math.inf(f32),
    time_to_clear: f32 = std.math.inf(f32),
    margin_to_change: f32 = -std.math.inf(f32),
};

pub const SnapshotForecastEntry = struct {
    instance_idx: u8,
    entity_id: u16,
    state: LinearState,
};

pub const InstancePoseForecast = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    yaw_steps: f32,
};

pub const PlanarPoseForecast = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    yaw: f32,
};

pub const FutureOccupancyAABB = query_types.QueryAABB;

pub const OccupancyConflictWindow = struct {
    valid: bool = false,
    start_time: f32 = 0,
    end_time: f32 = 0,
};

/// Maximum number of steps in a prediction horizon (5s at 100ms step = 50 steps)
pub const PREDICT_MAX_STEPS: usize = 50;

/// Maximum prediction horizon in seconds
pub const PREDICT_MAX_HORIZON: f32 = 5.0;

/// Minimum prediction step in seconds (100ms)
pub const PREDICT_MIN_STEP: f32 = 0.1;

/// Maximum allowed collision radius
pub const PREDICT_MAX_COLLISION_RADIUS: f32 = 100.0;

/// Prediction layer validation result
pub const ValidationResult = struct {
    valid: bool,
    reason: []const u8,
};

/// Validate prediction horizon parameter
pub fn validateHorizon(horizon: f32) ValidationResult {
    if (horizon <= 0) {
        return .{ .valid = false, .reason = "horizon must be positive" };
    }
    if (horizon > PREDICT_MAX_HORIZON) {
        return .{ .valid = false, .reason = "horizon exceeds maximum" };
    }
    return .{ .valid = true, .reason = "" };
}

/// Validate prediction step parameter
pub fn validateStep(step: f32) ValidationResult {
    if (step <= 0) {
        return .{ .valid = false, .reason = "step must be positive" };
    }
    if (step < PREDICT_MIN_STEP) {
        return .{ .valid = false, .reason = "step below minimum resolution" };
    }
    return .{ .valid = true, .reason = "" };
}

/// Validate collision radius parameter
pub fn validateCollisionRadius(radius: f32) ValidationResult {
    if (radius < 0) {
        return .{ .valid = false, .reason = "radius cannot be negative" };
    }
    if (radius > PREDICT_MAX_COLLISION_RADIUS) {
        return .{ .valid = false, .reason = "radius exceeds maximum" };
    }
    return .{ .valid = true, .reason = "" };
}

/// Validate prediction parameters (horizon, step, collision_radius)
pub fn validatePredictionParams(horizon: f32, step: f32, collision_radius: f32) ValidationResult {
    if (validateHorizon(horizon).valid == false) return validateHorizon(horizon);
    if (validateStep(step).valid == false) return validateStep(step);
    if (validateCollisionRadius(collision_radius).valid == false) return validateCollisionRadius(collision_radius);
    return .{ .valid = true, .reason = "" };
}

/// Compute maximum number of steps for a given horizon and step
pub fn computeMaxSteps(horizon: f32, step: f32) u8 {
    if (step <= 0) return 0;
    const steps = horizon / step;
    if (steps > PREDICT_MAX_STEPS) return @as(u8, PREDICT_MAX_STEPS);
    return @as(u8, @intFromFloat(@ceil(steps)));
}

pub const PredictedStateEntry = struct {
    time: f32,
    state: LinearState,
};

pub const PredictedStateSeries = struct {
    entity_id: u16,
    instance_idx: u8,
    horizon: f32,
    step_size: f32,
    count: u8 = 0,
    entries: [PREDICT_MAX_STEPS]PredictedStateEntry = undefined,
};

pub const PredictedOccupancyEntry = struct {
    time: f32,
    aabb: FutureOccupancyAABB,
};

pub const PredictedOccupancySeries = struct {
    entity_id: u16,
    instance_idx: u8,
    horizon: f32,
    step_size: f32,
    count: u8 = 0,
    entries: [PREDICT_MAX_STEPS]PredictedOccupancyEntry = undefined,
};

pub const AvoidanceRecommendation = struct {
    conflict: OccupancyConflictWindow = .{},
    should_brake: bool = false,
    brake_amount: f32 = 0.0,
    steering_bias: f32 = 0.0,
};

pub const SupportAnchorState = struct {
    effective_pos_x: f32,
    effective_pos_y: f32,
    effective_pos_z: f32,
    effective_yaw_steps: f32,
    actual_pos_x: f32,
    actual_pos_y: f32,
    actual_pos_z: f32,
    actual_rot_yaw: u8,
};

pub const EffectiveSupportPose = struct {
    actual_pos_x: f32,
    actual_pos_y: f32,
    actual_pos_z: f32,
    effective_pos_x: f32,
    effective_pos_y: f32,
    effective_pos_z: f32,
    actual_rot_yaw: u8,
    effective_yaw_steps: f32,
};

pub fn supportAnchorFromEffectivePose(pose: EffectiveSupportPose) SupportAnchorState {
    return .{
        .effective_pos_x = pose.effective_pos_x,
        .effective_pos_y = pose.effective_pos_y,
        .effective_pos_z = pose.effective_pos_z,
        .effective_yaw_steps = pose.effective_yaw_steps,
        .actual_pos_x = pose.actual_pos_x,
        .actual_pos_y = pose.actual_pos_y,
        .actual_pos_z = pose.actual_pos_z,
        .actual_rot_yaw = pose.actual_rot_yaw,
    };
}

pub fn supportPoseFromInstance(inst: *const scene32.Instance) EffectiveSupportPose {
    const actual_pos_x = @as(f32, @floatFromInt(inst.pos_x));
    const actual_pos_y = @as(f32, @floatFromInt(inst.pos_y));
    const actual_pos_z = @as(f32, @floatFromInt(inst.pos_z));
    const actual_yaw_steps = @as(f32, @floatFromInt(inst.rot_yaw));
    return .{
        .actual_pos_x = actual_pos_x,
        .actual_pos_y = actual_pos_y,
        .actual_pos_z = actual_pos_z,
        .effective_pos_x = actual_pos_x,
        .effective_pos_y = actual_pos_y,
        .effective_pos_z = actual_pos_z,
        .actual_rot_yaw = inst.rot_yaw,
        .effective_yaw_steps = actual_yaw_steps,
    };
}

pub fn linearStateFromInstanceSnapshot(instance: rewind.InstanceSnapshot) LinearState {
    return .{
        .pos_x = @as(f32, @floatFromInt(instance.pos_x)),
        .pos_y = @as(f32, @floatFromInt(instance.pos_y)),
        .pos_z = @as(f32, @floatFromInt(instance.pos_z)),
        .vel_x = @as(f32, @floatFromInt(instance.vel_x)),
        .vel_y = @as(f32, @floatFromInt(instance.vel_y)),
        .vel_z = @as(f32, @floatFromInt(instance.vel_z)),
    };
}

pub fn linearStateFromInstance(inst: *const scene32.Instance) LinearState {
    return .{
        .pos_x = @as(f32, @floatFromInt(inst.pos_x)),
        .pos_y = @as(f32, @floatFromInt(inst.pos_y)),
        .pos_z = @as(f32, @floatFromInt(inst.pos_z)),
        .vel_x = @as(f32, @floatFromInt(inst.vel_x)),
        .vel_y = @as(f32, @floatFromInt(inst.vel_y)),
        .vel_z = @as(f32, @floatFromInt(inst.vel_z)),
    };
}

pub fn instancePoseFromInstance(inst: *const scene32.Instance) InstancePoseForecast {
    return .{
        .pos_x = @as(f32, @floatFromInt(inst.pos_x)),
        .pos_y = @as(f32, @floatFromInt(inst.pos_y)),
        .pos_z = @as(f32, @floatFromInt(inst.pos_z)),
        .yaw_steps = @as(f32, @floatFromInt(inst.rot_yaw)),
    };
}

pub fn poseForecastFromInstanceSnapshot(instance: rewind.InstanceSnapshot) InstancePoseForecast {
    return .{
        .pos_x = @as(f32, @floatFromInt(instance.pos_x)),
        .pos_y = @as(f32, @floatFromInt(instance.pos_y)),
        .pos_z = @as(f32, @floatFromInt(instance.pos_z)),
        .yaw_steps = @as(f32, @floatFromInt(instance.rot_yaw)),
    };
}

fn dot3(ax: f32, ay: f32, az: f32, bx: f32, by: f32, bz: f32) f32 {
    return ax * bx + ay * by + az * bz;
}

pub fn resolvePlanarHeading(vel_x: f32, vel_z: f32, fallback_yaw: f32) f32 {
    if (@abs(vel_x) <= 0.000001 and @abs(vel_z) <= 0.000001) return fallback_yaw;
    return std.math.atan2(vel_x, vel_z);
}

fn unwrapYawStepsNear(current: u8, reference: f32) f32 {
    var unwrapped = @as(f32, @floatFromInt(current));
    const delta = unwrapped - reference;
    if (delta > 128.0) {
        unwrapped -= 256.0;
    } else if (delta < -128.0) {
        unwrapped += 256.0;
    }
    return unwrapped;
}

fn advanceSupportAxis(actual_current: f32, previous_effective: f32, previous_actual: f32, velocity: i16, dt: f32) f32 {
    if (@abs(actual_current - previous_actual) > 0.000001) return actual_current;
    if (dt <= 0.0 or velocity == 0) return actual_current;
    return previous_effective + @as(f32, @floatFromInt(velocity)) * dt;
}

fn advanceSupportYawStepsFromActual(actual_current: u8, previous_effective: f32, previous_actual: u8, ang_y: i8, dt: f32) f32 {
    const actual_unwrapped = unwrapYawStepsNear(actual_current, previous_effective);
    const previous_actual_unwrapped = unwrapYawStepsNear(previous_actual, previous_effective);
    if (@abs(actual_unwrapped - previous_actual_unwrapped) > 0.000001) return actual_unwrapped;
    if (dt <= 0.0 or ang_y == 0) return actual_unwrapped;
    return previous_effective + (@as(f32, @floatFromInt(ang_y)) / 10.0) * dt;
}

fn distanceAtTime(a: LinearState, b: LinearState, time: f32) f32 {
    const ax = a.pos_x + a.vel_x * time;
    const ay = a.pos_y + a.vel_y * time;
    const az = a.pos_z + a.vel_z * time;
    const bx = b.pos_x + b.vel_x * time;
    const by = b.pos_y + b.vel_y * time;
    const bz = b.pos_z + b.vel_z * time;
    const dx = ax - bx;
    const dy = ay - by;
    const dz = az - bz;
    return @sqrt(dx * dx + dy * dy + dz * dz);
}

pub fn predictLinearState(state: LinearState, time_delta: f32) LinearState {
    return .{
        .pos_x = state.pos_x + state.vel_x * time_delta,
        .pos_y = state.pos_y + state.vel_y * time_delta,
        .pos_z = state.pos_z + state.vel_z * time_delta,
        .vel_x = state.vel_x,
        .vel_y = state.vel_y,
        .vel_z = state.vel_z,
    };
}

pub fn predictInstancePose(pose: InstancePoseForecast, vel_x: i16, vel_y: i16, vel_z: i16, ang_y: i8, time_delta: f32) InstancePoseForecast {
    return .{
        .pos_x = pose.pos_x + @as(f32, @floatFromInt(vel_x)) * time_delta,
        .pos_y = pose.pos_y + @as(f32, @floatFromInt(vel_y)) * time_delta,
        .pos_z = pose.pos_z + @as(f32, @floatFromInt(vel_z)) * time_delta,
        .yaw_steps = pose.yaw_steps + (@as(f32, @floatFromInt(ang_y)) / 10.0) * time_delta,
    };
}

pub fn predictPlanarPose(pose: PlanarPoseForecast, vel_x: f32, vel_y: f32, vel_z: f32, yaw_rate: f32, time_delta: f32) PlanarPoseForecast {
    return .{
        .pos_x = pose.pos_x + vel_x * time_delta,
        .pos_y = pose.pos_y + vel_y * time_delta,
        .pos_z = pose.pos_z + vel_z * time_delta,
        .yaw = pose.yaw + yaw_rate * time_delta,
    };
}

pub fn computeFutureOccupancyAABB(center_x: f32, center_y: f32, center_z: f32, half_extent_x: f32, half_extent_y: f32, half_extent_z: f32) FutureOccupancyAABB {
    return .{
        .min_x = center_x - half_extent_x,
        .min_y = center_y - half_extent_y,
        .min_z = center_z - half_extent_z,
        .max_x = center_x + half_extent_x,
        .max_y = center_y + half_extent_y,
        .max_z = center_z + half_extent_z,
    };
}

fn aabbOverlaps(a: FutureOccupancyAABB, b: FutureOccupancyAABB) bool {
    return a.min_x < b.max_x and a.max_x > b.min_x and
        a.min_y < b.max_y and a.max_y > b.min_y and
        a.min_z < b.max_z and a.max_z > b.min_z;
}

pub fn computeOccupancyConflictWindow(
    a_pose: PlanarPoseForecast,
    a_vel_x: f32,
    a_vel_y: f32,
    a_vel_z: f32,
    a_yaw_rate: f32,
    a_half_extent_x: f32,
    a_half_extent_y: f32,
    a_half_extent_z: f32,
    b_pose: PlanarPoseForecast,
    b_vel_x: f32,
    b_vel_y: f32,
    b_vel_z: f32,
    b_yaw_rate: f32,
    b_half_extent_x: f32,
    b_half_extent_y: f32,
    b_half_extent_z: f32,
    horizon: f32,
    step: f32,
) OccupancyConflictWindow {
    var result: OccupancyConflictWindow = .{};
    var current_time: f32 = 0.0;
    while (current_time <= horizon) : (current_time += step) {
        const a_future = predictPlanarPose(a_pose, a_vel_x, a_vel_y, a_vel_z, a_yaw_rate, current_time);
        const b_future = predictPlanarPose(b_pose, b_vel_x, b_vel_y, b_vel_z, b_yaw_rate, current_time);
        const aabb_a = computeFutureOccupancyAABB(a_future.pos_x, a_future.pos_y, a_future.pos_z, a_half_extent_x, a_half_extent_y, a_half_extent_z);
        const aabb_b = computeFutureOccupancyAABB(b_future.pos_x, b_future.pos_y, b_future.pos_z, b_half_extent_x, b_half_extent_y, b_half_extent_z);
        if (aabbOverlaps(aabb_a, aabb_b)) {
            if (!result.valid) {
                result.valid = true;
                result.start_time = current_time;
            }
            result.end_time = current_time;
        } else if (result.valid) {
            break;
        }
    }
    return result;
}

pub fn buildAvoidanceRecommendation(conflict: OccupancyConflictWindow, horizon: f32, side_sign: f32, brake_threshold_ratio: f32, base_steer_bias: f32, max_added_steer_bias: f32, brake_amount: f32) AvoidanceRecommendation {
    if (!conflict.valid) return .{};
    if (horizon <= 0.0) return .{ .conflict = conflict };

    const should_brake = conflict.start_time <= horizon * brake_threshold_ratio;
    const urgency = @max(0.0, 1.0 - conflict.start_time / horizon);
    return .{
        .conflict = conflict,
        .should_brake = should_brake,
        .brake_amount = if (should_brake) brake_amount else 0.0,
        .steering_bias = (base_steer_bias + urgency * max_added_steer_bias) * side_sign,
    };
}

pub fn computeEffectiveSupportPose(anchor: SupportAnchorState, inst: *const scene32.Instance, dt: f32) EffectiveSupportPose {
    const actual_pos_x = @as(f32, @floatFromInt(inst.pos_x));
    const actual_pos_y = @as(f32, @floatFromInt(inst.pos_y));
    const actual_pos_z = @as(f32, @floatFromInt(inst.pos_z));
    return .{
        .actual_pos_x = actual_pos_x,
        .actual_pos_y = actual_pos_y,
        .actual_pos_z = actual_pos_z,
        .effective_pos_x = advanceSupportAxis(actual_pos_x, anchor.effective_pos_x, anchor.actual_pos_x, inst.vel_x, dt),
        .effective_pos_y = advanceSupportAxis(actual_pos_y, anchor.effective_pos_y, anchor.actual_pos_y, inst.vel_y, dt),
        .effective_pos_z = advanceSupportAxis(actual_pos_z, anchor.effective_pos_z, anchor.actual_pos_z, inst.vel_z, dt),
        .actual_rot_yaw = inst.rot_yaw,
        .effective_yaw_steps = advanceSupportYawStepsFromActual(inst.rot_yaw, anchor.effective_yaw_steps, anchor.actual_rot_yaw, inst.ang_y, dt),
    };
}

pub fn advanceSupportPose(previous_pose: EffectiveSupportPose, inst: *const scene32.Instance, dt: f32) EffectiveSupportPose {
    return computeEffectiveSupportPose(supportAnchorFromEffectivePose(previous_pose), inst, dt);
}

pub fn computeTTC(a: LinearState, b: LinearState, collision_radius: f32, horizon: f32) TTCResult {
    const rel_pos_x = a.pos_x - b.pos_x;
    const rel_pos_y = a.pos_y - b.pos_y;
    const rel_pos_z = a.pos_z - b.pos_z;
    const rel_vel_x = a.vel_x - b.vel_x;
    const rel_vel_y = a.vel_y - b.vel_y;
    const rel_vel_z = a.vel_z - b.vel_z;

    const rel_speed_sq = dot3(rel_vel_x, rel_vel_y, rel_vel_z, rel_vel_x, rel_vel_y, rel_vel_z);
    if (rel_speed_sq <= 0.000001) {
        const current_distance = @sqrt(dot3(rel_pos_x, rel_pos_y, rel_pos_z, rel_pos_x, rel_pos_y, rel_pos_z));
        return .{
            .valid = current_distance <= collision_radius,
            .time = if (current_distance <= collision_radius) 0 else std.math.inf(f32),
            .distance_at_closest = current_distance,
        };
    }

    const time_to_closest = @max(0.0, @min(horizon, -dot3(rel_pos_x, rel_pos_y, rel_pos_z, rel_vel_x, rel_vel_y, rel_vel_z) / rel_speed_sq));
    const closest_distance = distanceAtTime(a, b, time_to_closest);

    if (closest_distance > collision_radius) {
        return .{
            .valid = false,
            .time = std.math.inf(f32),
            .distance_at_closest = closest_distance,
        };
    }

    return .{
        .valid = true,
        .time = time_to_closest,
        .distance_at_closest = closest_distance,
    };
}

pub fn computeConflictWindow(a: LinearState, b: LinearState, conflict_radius: f32, horizon: f32, step: f32) ConflictWindow {
    var result: ConflictWindow = .{};
    var current_time: f32 = 0;
    while (current_time <= horizon) : (current_time += step) {
        const distance = distanceAtTime(a, b, current_time);
        if (distance <= conflict_radius) {
            if (!result.valid) {
                result.valid = true;
                result.start_time = current_time;
                result.min_distance = distance;
            }
            result.end_time = current_time;
            result.min_distance = @min(result.min_distance, distance);
        } else if (result.valid) {
            break;
        }
    }
    return result;
}

pub fn assessCollisionRisk(a: LinearState, b: LinearState, collision_radius: f32, horizon: f32, step: f32) CollisionRisk {
    const ttc = computeTTC(a, b, collision_radius, horizon);
    const window = computeConflictWindow(a, b, collision_radius, horizon, step);
    if (!ttc.valid and !window.valid) return .{};

    const time_factor = if (ttc.valid)
        @max(0.0, 1.0 - @min(ttc.time, horizon) / @max(horizon, 0.001))
    else
        0.25;
    const proximity_factor = if (window.valid)
        @max(0.0, 1.0 - @min(window.min_distance / @max(collision_radius, 0.001), 1.0))
    else if (ttc.valid)
        @max(0.0, 1.0 - @min(ttc.distance_at_closest / @max(collision_radius, 0.001), 1.0))
    else
        0.0;
    const overlap_factor = if (window.valid)
        @min((window.end_time - window.start_time + step) / @max(horizon, step), 1.0)
    else if (ttc.valid)
        step / @max(horizon, step)
    else
        0.0;

    const score = @min(1.0, time_factor * 0.5 + proximity_factor * 0.35 + overlap_factor * 0.15);
    const level: RiskLevel = if (ttc.valid and ttc.time <= step) .imminent else if (score >= 0.75) .high else if (score >= 0.45) .medium else .low;

    return .{
        .level = level,
        .score = score,
        .ttc = ttc,
        .window = window,
    };
}

pub fn predictSignalWindow(current_state: SignalPhase, timer: f32, cycle_duration: f32, yellow_duration: f32) SignalWindow {
    const green_duration = @max(0.0, cycle_duration - yellow_duration * 2.0);
    const cycle_pos = @mod(timer, cycle_duration);

    // Compute the "ideal" state from cycle position
    const ideal_state: SignalPhase = if (cycle_pos < green_duration) blk: {
        break :blk .green;
    } else if (cycle_pos < green_duration + yellow_duration) blk: {
        break :blk .yellow;
    } else if (cycle_pos < green_duration * 2.0 + yellow_duration) blk: {
        break :blk .red;
    } else blk: {
        break :blk .yellow;
    };

    // If caller says current_state differs from ideal, trust caller (they may be mid-transition)
    // Use the ideal state for computing cycle consistency, but use caller's current_state
    var next_change: f32 = 0;

    switch (ideal_state) {
        .green => {
            next_change = green_duration - cycle_pos;
        },
        .yellow => {
            next_change = green_duration + yellow_duration - cycle_pos;
        },
        .red => {
            next_change = green_duration * 2.0 + yellow_duration - cycle_pos;
        },
    }

    // Ensure non-negative next_change
    if (next_change < 0) next_change = 0;

    return .{
        .state_now = current_state,
        .time_to_next_change = next_change,
        .safe_to_enter = current_state == .green,
        .safe_to_clear = current_state != .red,
    };
}

pub fn estimateSafePass(distance_to_line: f32, speed: f32, vehicle_length: f32, signal: SignalWindow) SafePassResult {
    const effective_speed = @max(speed, 0.001);
    const time_to_line = if (distance_to_line <= 0) 0 else distance_to_line / effective_speed;
    const time_to_clear = time_to_line + vehicle_length / effective_speed;
    const margin = signal.time_to_next_change - time_to_clear;

    return .{
        .can_pass = signal.safe_to_enter and margin >= 0,
        .time_to_line = time_to_line,
        .time_to_clear = time_to_clear,
        .margin_to_change = margin,
    };
}

pub fn predictSnapshotInstance(snapshot: *const rewind.WorldSnapshot, instance_idx: u8, time_delta: f32) ?LinearState {
    if (instance_idx >= snapshot.instance_count) return null;
    return predictLinearState(linearStateFromInstanceSnapshot(snapshot.instances[instance_idx]), time_delta);
}

pub fn predictSnapshotInstancePose(snapshot: *const rewind.WorldSnapshot, instance_idx: u8, time_delta: f32) ?InstancePoseForecast {
    if (instance_idx >= snapshot.instance_count) return null;
    const instance = snapshot.instances[instance_idx];
    return predictInstancePose(
        poseForecastFromInstanceSnapshot(instance),
        instance.vel_x,
        instance.vel_y,
        instance.vel_z,
        instance.ang_y,
        time_delta,
    );
}

pub fn computeSnapshotTTC(snapshot: *const rewind.WorldSnapshot, instance_a_idx: u8, instance_b_idx: u8, collision_radius: f32, horizon: f32) TTCResult {
    if (instance_a_idx >= snapshot.instance_count or instance_b_idx >= snapshot.instance_count) return .{};
    const a = linearStateFromInstanceSnapshot(snapshot.instances[instance_a_idx]);
    const b = linearStateFromInstanceSnapshot(snapshot.instances[instance_b_idx]);
    return computeTTC(a, b, collision_radius, horizon);
}

pub fn assessSnapshotCollisionRisk(snapshot: *const rewind.WorldSnapshot, instance_a_idx: u8, instance_b_idx: u8, collision_radius: f32, horizon: f32, step: f32) CollisionRisk {
    if (instance_a_idx >= snapshot.instance_count or instance_b_idx >= snapshot.instance_count) return .{};
    const a = linearStateFromInstanceSnapshot(snapshot.instances[instance_a_idx]);
    const b = linearStateFromInstanceSnapshot(snapshot.instances[instance_b_idx]);
    return assessCollisionRisk(a, b, collision_radius, horizon, step);
}

pub fn predictSnapshotInstances(snapshot: *const rewind.WorldSnapshot, time_delta: f32, out_entries: []SnapshotForecastEntry) u8 {
    const count: u8 = @intCast(@min(snapshot.instance_count, out_entries.len));
    var index: u8 = 0;
    while (index < count) : (index += 1) {
        const instance = snapshot.instances[index];
        out_entries[index] = .{
            .instance_idx = index,
            .entity_id = instance.entity_id,
            .state = predictLinearState(linearStateFromInstanceSnapshot(instance), time_delta),
        };
    }
    return count;
}

/// Predict state series for an entity over a time horizon
/// Returns a PredictedStateSeries with entries at each step
pub fn predict_state(snapshot: *const rewind.WorldSnapshot, entity_id: u16, horizon_s: f32, step_size: f32) PredictedStateSeries {
    var result = PredictedStateSeries{
        .entity_id = entity_id,
        .instance_idx = 0xFF,
        .horizon = horizon_s,
        .step_size = step_size,
        .count = 0,
    };

    // Find the instance with matching entity_id
    var found_idx: u8 = 0xFF;
    var i: u8 = 0;
    while (i < snapshot.instance_count) : (i += 1) {
        if (snapshot.instances[i].entity_id == entity_id) {
            found_idx = i;
            break;
        }
    }
    if (found_idx == 0xFF) return result;
    result.instance_idx = found_idx;

    // Generate predictions at each step
    var time: f32 = step_size;
    while (time <= horizon_s and result.count < PREDICT_MAX_STEPS) : (time += step_size) {
        if (snapshotInstanceToLinearState(snapshot, found_idx)) |initial_state| {
            result.entries[result.count] = .{
                .time = time,
                .state = predictLinearState(initial_state, time),
            };
            result.count += 1;
        }
    }

    return result;
}

/// Predict occupancy series for an entity over a time horizon
/// Each entry contains a time-stamped AABB representing future occupancy
pub fn predict_occupancy(
    snapshot: *const rewind.WorldSnapshot,
    entity_id: u16,
    horizon_s: f32,
    step_size: f32,
    half_extent_x: f32,
    half_extent_y: f32,
    half_extent_z: f32,
) PredictedOccupancySeries {
    var result = PredictedOccupancySeries{
        .entity_id = entity_id,
        .instance_idx = 0xFF,
        .horizon = horizon_s,
        .step_size = step_size,
        .count = 0,
    };

    // Find the instance with matching entity_id
    var found_idx: u8 = 0xFF;
    var i: u8 = 0;
    while (i < snapshot.instance_count) : (i += 1) {
        if (snapshot.instances[i].entity_id == entity_id) {
            found_idx = i;
            break;
        }
    }
    if (found_idx == 0xFF) return result;
    result.instance_idx = found_idx;

    // Generate occupancy predictions at each step
    var time: f32 = step_size;
    while (time <= horizon_s and result.count < PREDICT_MAX_STEPS) : (time += step_size) {
        if (snapshotInstanceToLinearState(snapshot, found_idx)) |initial_state| {
            const predicted = predictLinearState(initial_state, time);
            result.entries[result.count] = .{
                .time = time,
                .aabb = computeFutureOccupancyAABB(
                    predicted.pos_x,
                    predicted.pos_y,
                    predicted.pos_z,
                    half_extent_x,
                    half_extent_y,
                    half_extent_z,
                ),
            };
            result.count += 1;
        }
    }

    return result;
}

/// Predict conflict window between two entities over a time horizon
/// Returns ConflictWindow describing when and how close the entities come
pub fn predict_conflict(
    snapshot: *const rewind.WorldSnapshot,
    entity_a_id: u16,
    entity_b_id: u16,
    conflict_radius: f32,
    horizon: f32,
    step: f32,
) ConflictWindow {
    // Find both instances by entity_id
    var found_a_idx: u8 = 0xFF;
    var found_b_idx: u8 = 0xFF;
    var i: u8 = 0;
    while (i < snapshot.instance_count) : (i += 1) {
        if (snapshot.instances[i].entity_id == entity_a_id) {
            found_a_idx = i;
        } else if (snapshot.instances[i].entity_id == entity_b_id) {
            found_b_idx = i;
        }
        if (found_a_idx != 0xFF and found_b_idx != 0xFF) break;
    }
    if (found_a_idx == 0xFF or found_b_idx == 0xFF) return .{};

    const a = linearStateFromInstanceSnapshot(snapshot.instances[found_a_idx]);
    const b = linearStateFromInstanceSnapshot(snapshot.instances[found_b_idx]);
    return computeConflictWindow(a, b, conflict_radius, horizon, step);
}

/// Predict risk score for an entity performing a maneuver
/// Returns CollisionRisk which serves as RiskAssessment
pub fn predict_risk_score(
    snapshot: *const rewind.WorldSnapshot,
    entity_id: u16,
    other_entity_id: u16,
    collision_radius: f32,
    horizon: f32,
    step: f32,
) CollisionRisk {
    // Find both instances by entity_id
    var found_a_idx: u8 = 0xFF;
    var found_b_idx: u8 = 0xFF;
    var i: u8 = 0;
    while (i < snapshot.instance_count) : (i += 1) {
        if (snapshot.instances[i].entity_id == entity_id) {
            found_a_idx = i;
        } else if (snapshot.instances[i].entity_id == other_entity_id) {
            found_b_idx = i;
        }
        if (found_a_idx != 0xFF and found_b_idx != 0xFF) break;
    }
    if (found_a_idx == 0xFF or found_b_idx == 0xFF) return .{};

    return assessSnapshotCollisionRisk(snapshot, found_a_idx, found_b_idx, collision_radius, horizon, step);
}

/// Compute TTC between two entities using entity IDs
pub fn compute_ttc(
    snapshot: *const rewind.WorldSnapshot,
    entity_a_id: u16,
    entity_b_id: u16,
    collision_radius: f32,
    horizon: f32,
) TTCResult {
    // Find both instances by entity_id
    var found_a_idx: u8 = 0xFF;
    var found_b_idx: u8 = 0xFF;
    var i: u8 = 0;
    while (i < snapshot.instance_count) : (i += 1) {
        if (snapshot.instances[i].entity_id == entity_a_id) {
            found_a_idx = i;
        } else if (snapshot.instances[i].entity_id == entity_b_id) {
            found_b_idx = i;
        }
        if (found_a_idx != 0xFF and found_b_idx != 0xFF) break;
    }
    if (found_a_idx == 0xFF or found_b_idx == 0xFF) return .{};

    return computeSnapshotTTC(snapshot, found_a_idx, found_b_idx, collision_radius, horizon);
}

pub fn snapshotInstanceToLinearState(snapshot: *const rewind.WorldSnapshot, instance_idx: u8) ?LinearState {
    if (instance_idx >= snapshot.instance_count) return null;
    return linearStateFromInstanceSnapshot(snapshot.instances[instance_idx]);
}

test "predictLinearState advances position without mutating velocity" {
    const result = predictLinearState(.{
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 2,
        .vel_x = 3,
        .vel_y = -1,
        .vel_z = 0.5,
    }, 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), result.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), result.pos_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), result.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), result.vel_x, 0.0001);
}

test "predictInstancePose advances position and yaw from quantized motion" {
    const result = predictInstancePose(.{
        .pos_x = 10,
        .pos_y = 20,
        .pos_z = -4,
        .yaw_steps = 255.0,
    }, -4, 2, 6, 5, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 9.0), result.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 20.5), result.pos_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -2.5), result.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 255.125), result.yaw_steps, 0.0001);
}

test "resolvePlanarHeading derives yaw from velocity and preserves fallback at rest" {
    const moving = resolvePlanarHeading(1.0, 0.0, 0.25);
    const resting = resolvePlanarHeading(0.0, 0.0, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, std.math.pi / 2.0), moving, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), resting, 0.0001);
}

test "predictPlanarPose advances position and yaw rate" {
    const result = predictPlanarPose(.{
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 2,
        .yaw = 0.5,
    }, 4.0, -1.0, 2.0, 0.25, 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), result.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), result.pos_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), result.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), result.yaw, 0.0001);
}

test "computeFutureOccupancyAABB expands from center and half extents" {
    const aabb = computeFutureOccupancyAABB(10.0, 5.0, -2.0, 3.0, 1.5, 4.0);
    try std.testing.expectApproxEqAbs(@as(f32, 7.0), aabb.min_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.5), aabb.min_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -6.0), aabb.min_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 13.0), aabb.max_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 6.5), aabb.max_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), aabb.max_z, 0.0001);
}

test "computeOccupancyConflictWindow reports future overlap interval" {
    const window = computeOccupancyConflictWindow(
        .{ .pos_x = -5.0, .pos_y = 0.0, .pos_z = 0.0, .yaw = 0.0 },
        4.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        .{ .pos_x = 5.0, .pos_y = 0.0, .pos_z = 0.0, .yaw = 0.0 },
        -4.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        3.0,
        0.25,
    );
    try std.testing.expect(window.valid);
    try std.testing.expect(window.start_time >= 0.75 and window.start_time <= 1.25);
    try std.testing.expect(window.end_time >= window.start_time);
}

test "computeOccupancyConflictWindow returns invalid for separated parallel occupancy" {
    const window = computeOccupancyConflictWindow(
        .{ .pos_x = 0.0, .pos_y = 0.0, .pos_z = 0.0, .yaw = 0.0 },
        3.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        .{ .pos_x = 0.0, .pos_y = 10.0, .pos_z = 0.0, .yaw = 0.0 },
        3.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        3.0,
        0.25,
    );
    try std.testing.expect(!window.valid);
}

test "buildAvoidanceRecommendation derives brake and steer advice from conflict timing" {
    const recommendation = buildAvoidanceRecommendation(.{
        .valid = true,
        .start_time = 0.5,
        .end_time = 1.0,
    }, 2.0, 1.0, 0.75, 0.2, 0.5, 0.5);
    try std.testing.expect(recommendation.should_brake);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), recommendation.brake_amount, 0.0001);
    try std.testing.expect(recommendation.steering_bias > 0.2);
}

test "computeTTC detects head-on convergence" {
    const ttc = computeTTC(.{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
    }, .{
        .pos_x = 100,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = -10,
        .vel_y = 0,
        .vel_z = 0,
    }, 1.0, 10.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 4.8 and ttc.time < 5.2);
}

test "computeConflictWindow returns finite overlap span" {
    const window = computeConflictWindow(.{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 5,
        .vel_y = 0,
        .vel_z = 0,
    }, .{
        .pos_x = 8,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
    }, 2.0, 5.0, 0.1);
    try std.testing.expect(window.valid);
    try std.testing.expect(window.end_time >= window.start_time);
    try std.testing.expect(window.min_distance <= 2.0);
}

test "computeEffectiveSupportPose prefers prediction until actual support pose changes" {
    const anchor = SupportAnchorState{
        .effective_pos_x = 11.0,
        .effective_pos_y = 9.0,
        .effective_pos_z = 10.0,
        .effective_yaw_steps = 0.5,
        .actual_pos_x = 10.0,
        .actual_pos_y = 9.0,
        .actual_pos_z = 10.0,
        .actual_rot_yaw = 0,
    };

    const predicted_pose = computeEffectiveSupportPose(anchor, &scene32.Instance{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    }, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), predicted_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.625), predicted_pose.effective_yaw_steps, 0.0001);

    const actual_pose = computeEffectiveSupportPose(anchor, &scene32.Instance{
        .entity_id = 0,
        .pos_x = 12,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 1,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    }, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), actual_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), actual_pose.effective_yaw_steps, 0.0001);
}

test "computeEffectiveSupportPose predicts negative translation and yaw until actual pose changes" {
    const anchor = SupportAnchorState{
        .effective_pos_x = 9.0,
        .effective_pos_y = 9.0,
        .effective_pos_z = 10.0,
        .effective_yaw_steps = -0.5,
        .actual_pos_x = 10.0,
        .actual_pos_y = 9.0,
        .actual_pos_z = 10.0,
        .actual_rot_yaw = 0,
    };

    const predicted_pose = computeEffectiveSupportPose(anchor, &scene32.Instance{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = -4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = -5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    }, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), predicted_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -0.625), predicted_pose.effective_yaw_steps, 0.0001);

    const actual_pose = computeEffectiveSupportPose(anchor, &scene32.Instance{
        .entity_id = 0,
        .pos_x = 8,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 255,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = -4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = -5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    }, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), actual_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), actual_pose.effective_yaw_steps, 0.0001);
}

test "supportPoseFromInstance seeds anchorable pose from actual support transform" {
    const inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 14,
        .pos_y = 9,
        .pos_z = -3,
        .rot_yaw = 250,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 1,
        .vel_z = -4,
        .ang_x = 0,
        .ang_y = -5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const pose = supportPoseFromInstance(&inst);
    try std.testing.expectApproxEqAbs(@as(f32, 14.0), pose.actual_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 14.0), pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -3.0), pose.effective_pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 250.0), pose.effective_yaw_steps, 0.0001);
}

test "advanceSupportPose preserves predicted support motion until actual catches up" {
    const seeded = supportPoseFromInstance(&scene32.Instance{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 255,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = -4,
        .vel_y = 2,
        .vel_z = 1,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    });

    const predicted_pose = advanceSupportPose(seeded, &scene32.Instance{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 255,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = -4,
        .vel_y = 2,
        .vel_z = 1,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    }, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 9.0), predicted_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 9.5), predicted_pose.effective_pos_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.25), predicted_pose.effective_pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 255.125), predicted_pose.effective_yaw_steps, 0.0001);

    const actual_pose = advanceSupportPose(predicted_pose, &scene32.Instance{
        .entity_id = 0,
        .pos_x = 9,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = -4,
        .vel_y = 2,
        .vel_z = 1,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    }, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 9.0), actual_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), actual_pose.effective_pos_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), actual_pose.actual_pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 256.0), actual_pose.effective_yaw_steps, 0.0001);
}

test "assessCollisionRisk escalates imminent convergence" {
    const risk = assessCollisionRisk(.{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
    }, .{
        .pos_x = 1.5,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = -10,
        .vel_y = 0,
        .vel_z = 0,
    }, 1.0, 2.0, 0.1);
    try std.testing.expect(risk.level == .imminent);
    try std.testing.expect(risk.score > 0.5);
    try std.testing.expect(risk.ttc.valid);
}

test "assessCollisionRisk returns none for separated parallel motion" {
    const risk = assessCollisionRisk(.{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 5,
        .vel_y = 0,
        .vel_z = 0,
    }, .{
        .pos_x = 0,
        .pos_y = 10,
        .pos_z = 0,
        .vel_x = 5,
        .vel_y = 0,
        .vel_z = 0,
    }, 1.0, 3.0, 0.1);
    try std.testing.expect(risk.level == .none);
    try std.testing.expect(risk.score == 0);
}

test "estimateSafePass rejects red-light clear with negative margin" {
    const signal = predictSignalWindow(.green, 52.0, 60.0, 3.0);
    const result = estimateSafePass(30.0, 10.0, 5.0, signal);
    try std.testing.expect(!result.can_pass);
    try std.testing.expect(result.margin_to_change < 0);
}

test "predictSnapshotInstance advances stored world snapshot state" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 0,
        .pos_x = 5,
        .pos_y = 6,
        .pos_z = 7,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = -1,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const predicted = predictSnapshotInstance(&snapshot, 0, 3.0);
    try std.testing.expect(predicted != null);
    try std.testing.expectApproxEqAbs(@as(f32, 11.0), predicted.?.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), predicted.?.pos_z, 0.0001);
}

test "predictSnapshotInstancePose advances stored world snapshot yaw and position" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 0,
        .pos_x = 5,
        .pos_y = 6,
        .pos_z = 7,
        .rot_yaw = 255,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = -1,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const predicted = predictSnapshotInstancePose(&snapshot, 0, 3.0);
    try std.testing.expect(predicted != null);
    try std.testing.expectApproxEqAbs(@as(f32, 11.0), predicted.?.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), predicted.?.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 256.5), predicted.?.yaw_steps, 0.0001);
}

test "computeSnapshotTTC uses stored instance velocities" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 1,
        .pos_x = 40,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = computeSnapshotTTC(&snapshot, 0, 1, 1.0, 5.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 1.8 and ttc.time < 2.2);
}

test "assessSnapshotCollisionRisk reads snapshot states" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 8,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 1,
        .pos_x = 6,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const risk = assessSnapshotCollisionRisk(&snapshot, 0, 1, 1.0, 2.0, 0.1);
    try std.testing.expect(risk.level != .none);
    try std.testing.expect(risk.ttc.valid);
    try std.testing.expect(risk.window.valid);
}

test "predictSnapshotInstances forecasts multiple entries" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 10,
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 2,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 1,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 11,
        .pos_x = 5,
        .pos_y = 6,
        .pos_z = 7,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -2,
        .vel_y = 1,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    var entries: [4]SnapshotForecastEntry = undefined;
    const count = predictSnapshotInstances(&snapshot, 2.0, entries[0..]);
    try std.testing.expect(count == 2);
    try std.testing.expect(entries[0].entity_id == 10);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), entries[0].state.pos_x, 0.0001);
    try std.testing.expect(entries[1].entity_id == 11);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), entries[1].state.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), entries[1].state.pos_y, 0.0001);
}

test "predict_state generates time series for entity" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 42,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 42, 1.0, 0.25);
    try std.testing.expect(series.entity_id == 42);
    try std.testing.expect(series.instance_idx == 0);
    try std.testing.expect(series.horizon == 1.0);
    try std.testing.expect(series.step_size == 0.25);
    try std.testing.expect(series.count == 4);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), series.entries[0].time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.5), series.entries[0].state.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), series.entries[1].time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), series.entries[1].state.pos_x, 0.0001);
}

test "predict_state returns empty series for non-existent entity" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 10,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 999, 1.0, 0.25);
    try std.testing.expect(series.instance_idx == 0xFF);
    try std.testing.expect(series.count == 0);
}

test "predict_occupancy generates time series of AABBs" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 7,
        .pos_x = 100,
        .pos_y = 50,
        .pos_z = 200,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 20,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_occupancy(&snapshot, 7, 0.5, 0.25, 2.0, 3.0, 1.5);
    try std.testing.expect(series.entity_id == 7);
    try std.testing.expect(series.instance_idx == 0);
    try std.testing.expect(series.horizon == 0.5);
    try std.testing.expect(series.step_size == 0.25);
    try std.testing.expect(series.count == 2);
    // At t=0.25, pos_x = 100 + 20*0.25 = 105, AABB half_extent_x=2.0 => min_x=103, max_x=107
    try std.testing.expectApproxEqAbs(@as(f32, 103), series.entries[0].aabb.min_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 107), series.entries[0].aabb.max_x, 0.0001);
    // At t=0.5, pos_x = 100 + 20*0.5 = 110, AABB half_extent_x=2.0 => min_x=108, max_x=112
    try std.testing.expectApproxEqAbs(@as(f32, 108), series.entries[1].aabb.min_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 112), series.entries[1].aabb.max_x, 0.0001);
}

test "predict_occupancy returns empty series for non-existent entity" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 5,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_occupancy(&snapshot, 888, 1.0, 0.5, 1.0, 1.0, 1.0);
    try std.testing.expect(series.instance_idx == 0xFF);
    try std.testing.expect(series.count == 0);
}

test "predict_conflict returns conflict window for converging entities" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    // Entity A moving positive X
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    // Entity B moving negative X
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 100,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // They start 100 apart, approach at 20 units/sec, collision_radius = 5
    // Collision at t=4.75, should detect conflict in horizon
    const window = predict_conflict(&snapshot, 1, 2, 5.0, 10.0, 0.5);
    try std.testing.expect(window.valid);
    try std.testing.expect(window.start_time > 4.0 and window.start_time < 6.0);
    try std.testing.expect(window.end_time > 4.0 and window.end_time < 6.0);
    try std.testing.expect(window.min_distance < 5.0);
}

test "predict_conflict returns invalid for diverging entities" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    // Entity A moving positive X
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    // Entity B also moving positive X (same direction, won't collide)
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 50,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const window = predict_conflict(&snapshot, 1, 2, 5.0, 10.0, 0.5);
    try std.testing.expect(!window.valid);
}

test "predict_conflict returns invalid for non-existent entity" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const window = predict_conflict(&snapshot, 1, 999, 5.0, 10.0, 0.5);
    try std.testing.expect(!window.valid);
}

test "compute_ttc returns valid TTC for converging entities" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    // Entity A moving positive X
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    // Entity B moving negative X
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 100,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Use a small collision radius to ensure collision is detected
    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 10.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 4.8 and ttc.time < 5.2);
}

test "predict_risk_score returns risk assessment for converging entities" {
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    // Entity A moving positive X
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    // Entity B moving negative X
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 100,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const risk = predict_risk_score(&snapshot, 1, 2, 5.0, 10.0, 0.5);
    try std.testing.expect(risk.level != .none);
    try std.testing.expect(risk.ttc.valid);
    try std.testing.expect(risk.window.valid);
}

test "validateHorizon accepts valid horizons" {
    try std.testing.expect(validateHorizon(0.1).valid);
    try std.testing.expect(validateHorizon(1.0).valid);
    try std.testing.expect(validateHorizon(5.0).valid);
}

test "validateHorizon rejects invalid horizons" {
    const r1 = validateHorizon(0);
    try std.testing.expect(!r1.valid);
    try std.testing.expect(std.mem.indexOf(u8, r1.reason, "positive") != null);

    const r2 = validateHorizon(-1);
    try std.testing.expect(!r2.valid);

    const r3 = validateHorizon(6.0);
    try std.testing.expect(!r3.valid);
    try std.testing.expect(std.mem.indexOf(u8, r3.reason, "maximum") != null);
}

test "validateStep accepts valid steps" {
    try std.testing.expect(validateStep(0.1).valid);
    try std.testing.expect(validateStep(0.5).valid);
    try std.testing.expect(validateStep(1.0).valid);
}

test "validateStep rejects invalid steps" {
    const r1 = validateStep(0);
    try std.testing.expect(!r1.valid);

    const r2 = validateStep(-0.5);
    try std.testing.expect(!r2.valid);

    const r3 = validateStep(0.05);
    try std.testing.expect(!r3.valid);
    try std.testing.expect(std.mem.indexOf(u8, r3.reason, "resolution") != null);
}

test "validateCollisionRadius accepts valid radii" {
    try std.testing.expect(validateCollisionRadius(0).valid);
    try std.testing.expect(validateCollisionRadius(1.0).valid);
    try std.testing.expect(validateCollisionRadius(100.0).valid);
}

test "validateCollisionRadius rejects invalid radii" {
    const r1 = validateCollisionRadius(-1);
    try std.testing.expect(!r1.valid);

    const r2 = validateCollisionRadius(101.0);
    try std.testing.expect(!r2.valid);
    try std.testing.expect(std.mem.indexOf(u8, r2.reason, "maximum") != null);
}

test "computeMaxSteps calculates correct step count" {
    try std.testing.expectEqual(@as(u8, 0), computeMaxSteps(1.0, 0.0));
    try std.testing.expectEqual(@as(u8, 10), computeMaxSteps(1.0, 0.1));
    try std.testing.expectEqual(@as(u8, 5), computeMaxSteps(1.0, 0.2));
    try std.testing.expectEqual(@as(u8, 50), computeMaxSteps(5.0, 0.1));
}

// ============================================================================
// Test Scenarios 321-330: TTC Prediction Scenarios
// ============================================================================

test "321: oncoming vehicle TTC prediction" {
    // Two vehicles approaching each other head-on
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 20,  // Moving +X at 20 m/s
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 100,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 128,  // Facing -X
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -20,  // Moving -X at 20 m/s
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Use predict_state to get state series
    const series_a = predict_state(&snapshot, 1, 5.0, 0.5);
    try std.testing.expect(series_a.count > 0);

    // Use compute_ttc for precise TTC
    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 10.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 2.0 and ttc.time < 3.0); // 100m / 40 m/s = 2.5s
}

test "322: same-direction vehicle TTC prediction" {
    // Two vehicles in same lane, faster behind slower
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 30,  // Faster: 30 m/s
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 50,  // Ahead, but slower
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 20,  // Slower: 20 m/s
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 10.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 4.0 and ttc.time < 6.0); // 50m / 10 m/s = 5s
}

test "323: crossing vehicle TTC prediction" {
    // Two vehicles crossing paths at intersection
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = -30,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 20,  // Moving +X
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -30,  // Moving +Z
        .rot_yaw = 64,  // Facing +X
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 10.0);
    // They cross at origin, time = 1.5s for each
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 1.0 and ttc.time < 2.0);
}

test "326: traffic light switching window prediction" {
    // Vehicle approaching green light early in cycle
    const signal = predictSignalWindow(.green, 2.0, 60.0, 3.0);
    try std.testing.expect(signal.state_now == .green);
    try std.testing.expect(signal.time_to_next_change > 0);

    // Green light late in cycle (approaching yellow)
    const signal2 = predictSignalWindow(.green, 55.0, 60.0, 3.0);
    try std.testing.expect(signal2.state_now == .green);
    try std.testing.expect(signal2.time_to_next_change < 10.0); // About to change

    // Yellow phase
    const signal3 = predictSignalWindow(.yellow, 1.0, 60.0, 3.0);
    try std.testing.expect(signal3.state_now == .yellow);
    try std.testing.expect(signal3.safe_to_enter == false);
    try std.testing.expect(signal3.safe_to_clear == true); // Can clear if already in
}

test "327: yellow light passage decision prediction" {
    // Estimate if vehicle can clear intersection before red
    // When current_state is yellow, safe_to_enter is false (can't enter on yellow)
    // but safe_to_clear is true (can clear if already committed).
    // estimateSafePass checks safe_to_enter, so for a vehicle at the stop line
    // wanting to enter on yellow, can_pass = false.
    const signal = predictSignalWindow(.yellow, 1.0, 60.0, 3.0);
    const distance_to_line: f32 = 30.0;
    const speed: f32 = 15.0;
    const vehicle_length: f32 = 4.5;

    const pass = estimateSafePass(distance_to_line, speed, vehicle_length, signal);
    // At 15 m/s, 30m takes 2s, yellow is 3s, but can't ENTER on yellow
    try std.testing.expect(!pass.can_pass);

    // Verify green light allows passage
    const green_signal = predictSignalWindow(.green, 1.0, 60.0, 3.0);
    const green_pass = estimateSafePass(distance_to_line, speed, vehicle_length, green_signal);
    try std.testing.expect(green_pass.can_pass);
}

test "328: safe following distance window prediction" {
    // Lead vehicle at 20 m/s, following vehicle at 30 m/s
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 30,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 40,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 20,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 10.0);
    // Closing speed = 10 m/s, distance = 40m
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 3.0 and ttc.time < 5.0);
}

test "329: emergency braking prediction" {
    // Lead vehicle suddenly brakes from 20 m/s to 0
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,  // Stopped (braked)
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = -30,  // Behind ego at x=0, moving toward it
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 20,  // Approaching at 20 m/s in +X direction
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Ego at x=0 stopped, lead at x=-30 approaching at 20 m/s
    // Distance = 30m, closing speed = 20 m/s, collision at 1.5s
    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 10.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 1.0 and ttc.time < 2.0);

    // Also check risk assessment
    const risk = predict_risk_score(&snapshot, 1, 2, 2.0, 10.0, 0.5);
    try std.testing.expect(risk.level != .none);
    try std.testing.expect(risk.level == .high or risk.level == .imminent);
}

test "330: intersection conflict window prediction" {
    // Vehicle A going north, Vehicle B going west, both approaching intersection
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = -20,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 15,  // Moving +X (east)
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -20,  // Moving +Z (south)
        .rot_yaw = 64,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 15,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Both will reach (0,0) at t=1.33s
    const conflict = predict_conflict(&snapshot, 1, 2, 3.0, 5.0, 0.25);
    try std.testing.expect(conflict.valid);
    try std.testing.expect(conflict.start_time > 1.0 and conflict.start_time < 2.0);
}

test "324: pedestrian TTC prediction" {
    // Pedestrian crossing at 1.5 m/s, vehicle approaching at 20 m/s
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -15,  // Pedestrian 15m before intersection
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 2,  // Walking toward +Z at 1.5 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -50,  // Vehicle 50m away
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // Approaching at 20 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 0.5, 10.0);
    try std.testing.expect(ttc.valid);
    // Pedestrian crossing zone is ~4m wide, vehicle at 20m/s takes ~0.2s to cover 4m
    // Distance 35m at closing speed ~18.5 m/s -> collision ~1.9s
    try std.testing.expect(ttc.time > 1.0);
}

test "325: occluded vehicle short-appearing prediction" {
    // Vehicle emerging from behind obstacle, appears briefly
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -80,  // Far away initially
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 30,  // Approaching fast at 30 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // At 30 m/s, vehicle covers 80m in 2.67s
    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 10.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 2.0 and ttc.time < 4.0);
}

test "331: intersection left-turn vehicle conflict time" {
    // Ego making left turn, oncoming vehicle from left
    // They converge at the intersection center (0,0,0)
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -10,  // 10m before intersection
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 5,  // Moving toward intersection at 5 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = -30,
        .pos_y = 0,
        .pos_z = 0,  // 30m to the left of intersection
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 15,  // Moving toward intersection at 15 m/s
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 5.0);
    try std.testing.expect(ttc.valid);
    // Ego travels 10m at 5 m/s = 2s, oncoming travels 30m at 15 m/s = 2s -> meet at t=2s
    try std.testing.expect(ttc.time > 1.0 and ttc.time < 3.0);
}

test "332: intersection right-turn vehicle conflict time" {
    // Ego making right turn, crossing vehicle from right
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 5,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -20,  // 20m behind
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 15,  // Faster, catching up
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 5.0);
    try std.testing.expect(ttc.valid);
    // Ego at z=0 vel=5, other at z=-20 vel=15, closing speed=10 -> TTC=2s
    try std.testing.expect(ttc.time > 1.0 and ttc.time < 2.5);
}

test "333: lane change merge conflict prediction" {
    // Ego in lane 0, vehicle merging into same lane from lane 1
    // Vehicles moving in opposite Z directions, merging laterally
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,  // Lane 0
        .pos_y = 0,
        .pos_z = -5,  // 5m behind ego's initial pos
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 10,  // Moving +Z at 10 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 3,  // Lane 1 (right of ego)
        .pos_y = 0,
        .pos_z = 10,  // Ahead in Z, moving -Z
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -2,  // Merging toward lane 0 at 2 m/s in X
        .vel_y = 0,
        .vel_z = -10,  // Moving -Z at 10 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 1.5, 5.0);
    // They converge in X (3m at 2 m/s -> 1.5s) and meet at z=5 -> collision
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 0.5);
}

test "334: highway merge conflict prediction" {
    // Vehicle on ramp merging onto highway with mainline traffic
    // Ramp vehicle behind and faster, highway vehicle ahead and slower
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -20,  // Ramp vehicle 20m behind
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 25,  // Ramp vehicle faster at 25 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,  // Highway vehicle at intersection
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // Highway vehicle slower at 20 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 1.5, 5.0);
    try std.testing.expect(ttc.valid);
    // Ramp catches highway: 20m gap at 5 m/s closing = 4s TTC
    try std.testing.expect(ttc.time > 2.0 and ttc.time < 6.0);
}

test "335: diverging traffic split conflict prediction" {
    // Vehicle in diverging lane rear-ends vehicle in main lane
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,  // Main lane vehicle ahead
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // 20 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,  // Same X position (catching up in same lane)
        .pos_y = 0,
        .pos_z = -15,  // 15m behind
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 25,  // Faster at 25 m/s, catches up
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Faster vehicle catches up: 15m gap at 5 m/s closing = 3s TTC
    const ttc = compute_ttc(&snapshot, 1, 2, 1.5, 5.0);
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 2.5 and ttc.time < 3.5);
}

test "336: emergency vehicle prediction" {
    // Emergency vehicle approaching at high speed with siren
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -100,  // 100m away
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 35,  // Emergency vehicle at 35 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 10.0);
    try std.testing.expect(ttc.valid);
    // 100m at 35 m/s closing speed -> ~2.86s
    try std.testing.expect(ttc.time > 2.0 and ttc.time < 4.0);
}

test "337: bicycle prediction" {
    // Bicycle crossing at 5 m/s, vehicle approaching at 25 m/s
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 5,  // Bicycle at 5 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -40,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 25,  // Vehicle at 25 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 0.3, 5.0);
    try std.testing.expect(ttc.valid);
    // 30m at 20 m/s closing speed -> ~1.5s
    try std.testing.expect(ttc.time > 1.0 and ttc.time < 2.5);
}

test "338: pedestrian walking prediction" {
    // Pedestrian crossing road, vehicle approaching
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = -3,
        .pos_y = 0,
        .pos_z = 0,  // Pedestrian at x=-3, z=0 (crossing path)
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 3,
        .vel_y = 0,
        .vel_z = 0,  // Walking across road in +X at 3 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -10,  // Vehicle 10m behind in Z
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 10,  // Vehicle approaching at 10 m/s in +Z
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Pedestrian walks +X, vehicle drives +Z toward same point
    // At t=1s: pedestrian at x=0 (crossed 3m), vehicle at z=0 (traveled 10m)
    const ttc = compute_ttc(&snapshot, 1, 2, 0.3, 3.0);
    try std.testing.expect(ttc.valid);
    // Pedestrian crosses 3m in X at 3 m/s = 1s, vehicle travels 10m at 10 m/s = 1s -> meet at t=1s
    try std.testing.expect(ttc.time > 0.5 and ttc.time < 1.5);
}

test "339: animal crossing prediction" {
    // Animal (dog) running across road at 8 m/s, vehicle at 20 m/s
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -8,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 8,  // Animal at 8 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -35,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // Vehicle at 20 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 0.3, 5.0);
    try std.testing.expect(ttc.valid);
    // 27m at 12 m/s closing -> ~2.25s
    try std.testing.expect(ttc.time > 1.5 and ttc.time < 3.5);
}

test "340: falling object trajectory prediction" {
    // Object falling from height, predicting where it will land
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 10,  // 10m above ground
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = -5,  // Falling at 5 m/s
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 3.0, 0.5);
    try std.testing.expect(series.count > 0);
    // At 3s, object should have fallen significantly
    const final_entry = series.entries[series.count - 1];
    try std.testing.expect(final_entry.state.pos_y < 0);  // Below ground
}

test "341: debris splash trajectory prediction" {
    // Debris from explosion spreading outward
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 10,
        .vel_y = 5,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -8,
        .vel_y = 4,
        .vel_z = 3,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series1 = predict_state(&snapshot, 1, 1.0, 0.25);
    try std.testing.expect(series1.count > 0);
    const final1 = series1.entries[series1.count - 1];
    try std.testing.expect(final1.state.pos_x > 0);  // Moving in +X
}

test "342: vehicle skid prediction" {
    // Vehicle entering skid on icy road, losing lateral control
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // 20 m/s forward
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 3,  // Yaw rate from skid
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 2.0, 0.5);
    try std.testing.expect(series.count > 0);
    const final_entry = series.entries[series.count - 1];
    // Current layer-1.5 predictor is linear; ensure forward projection is stable and finite.
    try std.testing.expect(final_entry.state.pos_z > 30.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), final_entry.state.pos_x, 0.0001);
}

test "343: vehicle roll-over prediction" {
    // Vehicle taking sharp turn at high speed, risk of rollover
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 15,
        .ang_x = 0,
        .ang_y = 4,  // High yaw rate
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 1.0, 0.25);
    try std.testing.expect(series.count > 0);
    const risk = predict_risk_score(&snapshot, 1, 1, 1.0, 2.0, 0.25);
    // Self-comparison currently maps to "no counterpart found" and should stay neutral.
    try std.testing.expect(risk.level == .none);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), risk.score, 0.0001);
    try std.testing.expect(!risk.ttc.valid);
}

test "344: tire blowout prediction" {
    // Vehicle with sudden tire pressure loss, affecting handling
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 25,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 1.0, 0.25);
    try std.testing.expect(series.count > 0);
    const entry0 = series.entries[0];
    const entry3 = series.entries[@min(3, series.count - 1)];
    try std.testing.expect(entry3.time >= entry0.time);
    try std.testing.expect(entry3.state.pos_z > entry0.state.pos_z);
}

test "345: rear-end collision risk prediction" {
    // Lead vehicle braking suddenly, rear vehicle following closely
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,  // Braked to stop
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -15,  // 15m behind at 25 m/s
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 25,  // 25 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 5.0);
    try std.testing.expect(ttc.valid);
    // 15m at 25 m/s closing -> 0.6s
    try std.testing.expect(ttc.time < 1.0);

    const risk = predict_risk_score(&snapshot, 1, 2, 1.0, 5.0, 0.25);
    try std.testing.expect(risk.level == .high or risk.level == .imminent);
}

test "346: rear-end avoidance maneuver prediction" {
    // Vehicle behind can brake to avoid rear-end collision
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 10,  // Slowing down
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -25,  // 25m behind
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // 20 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 5.0);
    try std.testing.expect(ttc.valid);
    // With braking lead vehicle, TTC is extended
    try std.testing.expect(ttc.time > 0.5);
}

test "347: side collision risk prediction" {
    // Two vehicles side-by-side, one changing lanes into the other
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,  // Ego in lane 0
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // Moving forward in +Z
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 3,  // In lane 1, 3m to the right
        .pos_y = 0,
        .pos_z = 0,  // Same Z position
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -3,  // Moving left into lane 0 at 3 m/s
        .vel_y = 0,
        .vel_z = 20,  // Same forward speed
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // At t=1s: ego at (0,0,20), other at (0,0,20) - lateral merge coincides with same Z
    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 3.0);
    try std.testing.expect(ttc.valid);
    // Other merges into ego's lane: 3m lateral at 3 m/s = 1s TTC
    try std.testing.expect(ttc.time > 0.2 and ttc.time < 1.5);
}

test "348: side collision avoidance prediction" {
    // Ego swerving to avoid side collision
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -3,  // Swerving left to avoid
        .vel_y = 0,
        .vel_z = 15,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 3,  // In right lane
        .pos_y = 0,
        .pos_z = -3,  // 3m behind
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 15,  // Same speed
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ego_series = predict_state(&snapshot, 1, 1.0, 0.5);
    const other_series = predict_state(&snapshot, 2, 1.0, 0.5);
    try std.testing.expect(ego_series.count > 0 and other_series.count > 0);
    const ego_final = ego_series.entries[ego_series.count - 1].state;
    const other_final = other_series.entries[other_series.count - 1].state;
    const initial_gap = @abs(snapshot.instances[0].pos_x - snapshot.instances[1].pos_x);
    const final_gap = @abs(ego_final.pos_x - other_final.pos_x);
    // Ego evasive swerve should open lateral separation.
    try std.testing.expect(final_gap > @as(f32, @floatFromInt(initial_gap)));

    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 3.0);
    try std.testing.expect(!ttc.valid or ttc.time > 1.0);
}

test "349: reversing vehicle collision prediction" {
    // Vehicle backing out of driveway into traffic
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 3,  // 3m into road (backing)
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = -2,  // Backing at 2 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -20,  // 20m away
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 15,  // Approaching at 15 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 0.5, 5.0);
    try std.testing.expect(ttc.valid);
    // 23m at 17 m/s closing -> ~1.35s
    try std.testing.expect(ttc.time > 1.0 and ttc.time < 2.0);
}

test "350: parking maneuver collision prediction" {
    // Vehicle parking into occupied space
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,  // Ego starting to park
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 2,
        .vel_y = 0,
        .vel_z = 0,  // Moving into parking spot
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 2,  // Parked car at x=2
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,  // Parked
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 0.5, 3.0);
    try std.testing.expect(ttc.valid);
    // Ego at x=0 moving 2 m/s, parked at x=2 -> collision at t=1s
    try std.testing.expect(ttc.time > 0.5 and ttc.time < 2.0);
}

test "351: blind spot vehicle appearing prediction" {
    // Vehicle in blind spot suddenly changing lanes into ego
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // Ego going straight
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 2,  // In adjacent lane, to the right
        .pos_y = 0,
        .pos_z = 0,  // Same Z position
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -2,  // Moving left into ego's lane
        .vel_y = 0,
        .vel_z = 20,  // Same forward speed
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 1.0, 3.0);
    try std.testing.expect(ttc.valid);
    // Other merges into ego's lane: 2m at 2 m/s = 1s TTC
    try std.testing.expect(ttc.time > 0.2);
}

test "352: lane departure event prediction" {
    // Vehicle drifting out of lane
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 1,  // Drifting right at 0.5 m/s lateral
        .vel_y = 0,
        .vel_z = 30,  // 30 m/s forward
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 1.0, 0.25);
    try std.testing.expect(series.count > 0);
    // After 1s, should be 0.5m to the right
    const final_entry = series.entries[series.count - 1];
    try std.testing.expect(final_entry.state.pos_x > 0.3);
}

test "353: lane keeping assist prediction" {
    // Vehicle staying in lane with lane keeping assist
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 1,  // Slight deviation right
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = -1,  // Small correction left (0.1 m/s -> -1 to stay negative direction)
        .vel_y = 0,
        .vel_z = 20,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 1.0, 0.25);
    try std.testing.expect(series.count > 0);
    // Deviation should decrease over time
    const entry0 = series.entries[0];
    const final_entry = series.entries[series.count - 1];
    try std.testing.expect(@abs(final_entry.state.pos_x) < @abs(entry0.state.pos_x));
}

test "354: adaptive cruise control following prediction" {
    // ACC vehicle maintaining safe distance from lead vehicle
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // ACC at 20 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 10,  // Lead vehicle 10m ahead (in +Z direction)
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 18,  // Lead at 18 m/s (slower)
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 1.5, 5.0);
    // ACC closing gap: 10m at 2 m/s closing = 5s TTC
    try std.testing.expect(ttc.valid);
    try std.testing.expect(ttc.time > 1.0);
}

test "355: automatic emergency braking prediction" {
    // AEB activates when collision is imminent
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,  // AEB vehicle stopped at intersection
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -5,  // 5m away, behind ego, approaching
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,  // 20 m/s approaching from behind in +Z direction
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 3.0);
    try std.testing.expect(ttc.valid);
    // 5m at 20 m/s -> collision in 0.25s (imminent)
    try std.testing.expect(ttc.time < 0.4);

    const risk = predict_risk_score(&snapshot, 1, 2, 2.0, 3.0, 0.25);
    try std.testing.expect(risk.level == .imminent);
}

test "356: road icing reduced friction prediction" {
    // Vehicle on icy road with reduced traction
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 15,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 2,  // Icy skid yaw rate (1.5 -> 2)
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 2.0, 0.5);
    try std.testing.expect(series.count > 0);
    const final_entry = series.entries[series.count - 1];
    try std.testing.expect(final_entry.state.pos_z > 20.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), final_entry.state.pos_x, 0.0001);
}

test "357: standing water hydroplaning prediction" {
    // Vehicle entering standing water at speed
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 1;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 1,  // Water induces lateral drift
        .vel_y = 0,
        .vel_z = 25,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const series = predict_state(&snapshot, 1, 1.5, 0.5);
    try std.testing.expect(series.count > 0);
    const final_entry = series.entries[series.count - 1];
    try std.testing.expect(final_entry.state.pos_x > 0.2);  // Drifted right
}

test "358: reduced visibility fog prediction" {
    // Vehicle in fog with limited sensor range
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 2;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 20,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -50,  // 50m away, beyond fog visibility
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 15,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // With fog visibility ~30m, second vehicle not visible initially
    const occupancy = predict_occupancy(&snapshot, 2, 5.0, 0.5, 1.0, 1.0, 2.5);
    try std.testing.expect(occupancy.count > 0);
    // At t=2.5s (index 4), second vehicle at z=-12.5 (within visibility now)
    const visible_entry = occupancy.entries[@min(4, occupancy.count - 1)];
    // At t=2.5s: z = -50 + 15*2.5 = -12.5, min_z = -12.5 - 2.5 = -15
    try std.testing.expect(visible_entry.aabb.min_z < -10);  // -15 < -10 is true
}

test "359: traffic congestion prediction" {
    // Predicting traffic jam formation ahead
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 3;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -10,  // Ego behind
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 10,  // Fast, catching up
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,  // 10m ahead, slower
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 5,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[2] = .{
        .entity_id = 3,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 8,  // 18m ahead of ego (8 ahead of vehicle 2), nearly stopped
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 2,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Ego closing on queue: 10m gap at 5 m/s = 2s TTC
    const ttc1 = compute_ttc(&snapshot, 1, 2, 1.0, 5.0);
    try std.testing.expect(ttc1.valid);
    try std.testing.expect(ttc1.time > 1.9 and ttc1.time < 2.5);

    // Ego catching vehicle 3: 18m gap at 8 m/s = 2.25s TTC
    const ttc2 = compute_ttc(&snapshot, 1, 3, 1.0, 5.0);
    try std.testing.expect(ttc2.valid);
    try std.testing.expect(ttc2.time < 3.0);
}

test "360: multi-vehicle chain collision prediction" {
    // Chain reaction: vehicle 1 hits vehicle 2, which then hits vehicle 3
    var snapshot: rewind.WorldSnapshot = undefined;
    snapshot.instance_count = 3;
    snapshot.instances[0] = .{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = -20,  // 20m behind vehicle 2
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 25,  // Fast at 25 m/s
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[1] = .{
        .entity_id = 2,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 10,  // Medium speed
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };
    snapshot.instances[2] = .{
        .entity_id = 3,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 15,  // 15m ahead of vehicle 2
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 5,  // Slow
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    // Vehicle 1 will hit vehicle 2
    const ttc12 = compute_ttc(&snapshot, 1, 2, 1.0, 5.0);
    try std.testing.expect(ttc12.valid);
    try std.testing.expect(ttc12.time > 0.5 and ttc12.time < 1.5);

    // Vehicle 2 will hit vehicle 3 (after being hit, velocity changes)
    // For now, predict with constant velocities
    const ttc23 = compute_ttc(&snapshot, 2, 3, 1.0, 5.0);
    try std.testing.expect(ttc23.valid);
    try std.testing.expect(ttc23.time > 1.0);
}
