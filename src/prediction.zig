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

    var next_change: f32 = 0;
    const resolved_state: SignalPhase = if (cycle_pos < green_duration) blk: {
        next_change = green_duration - cycle_pos;
        break :blk SignalPhase.green;
    } else if (cycle_pos < green_duration + yellow_duration) blk: {
        next_change = green_duration + yellow_duration - cycle_pos;
        break :blk SignalPhase.yellow;
    } else if (cycle_pos < green_duration * 2.0 + yellow_duration) blk: {
        next_change = green_duration * 2.0 + yellow_duration - cycle_pos;
        break :blk SignalPhase.red;
    } else blk: {
        next_change = cycle_duration - cycle_pos;
        break :blk SignalPhase.yellow;
    };

    _ = current_state;
    return .{
        .state_now = resolved_state,
        .time_to_next_change = next_change,
        .safe_to_enter = resolved_state == .green,
        .safe_to_clear = resolved_state != .red,
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
    const signal = predictSignalWindow(.yellow, 1.0, 60.0, 3.0);
    const distance_to_line: f32 = 30.0;
    const speed: f32 = 15.0;
    const vehicle_length: f32 = 4.5;

    const pass = estimateSafePass(distance_to_line, speed, vehicle_length, signal);
    // At 15 m/s, 30m takes 2s, yellow is 3s, so can likely pass
    try std.testing.expect(pass.can_pass);
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
        .pos_x = 30,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .vel_x = 20,  // Approaching at 20 m/s
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        .sleep_tick = 0,
    };

    const ttc = compute_ttc(&snapshot, 1, 2, 2.0, 10.0);
    // Closing speed = 20 m/s, distance = 30m, collision at 1.5s
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
