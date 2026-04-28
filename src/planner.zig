//! Planner System - Path Planning, Trajectory, and Decision Making
//!
//! Phase 62-63: Planning systems, behavior trees, cost functions, constraints
//! Handles: Path planning, trajectory optimization, behavior decisions, risk assessment

const std = @import("std");
const terrain = @import("terrain.zig");
const vehicle_dynamics = @import("vehicle.zig");
const weather = @import("weather.zig");

pub const PathPoint = struct {
    x: f32,
    y: f32,
    z: f32,
    yaw: f32,
    speed: f32,
};

pub const Path = struct {
    points: [64]PathPoint,
    point_count: u8,
    total_length: f32,
};

pub const Trajectory = struct {
    positions: [128]struct { x: f32, y: f32, z: f32 },
    velocities: [128]struct { x: f32, y: f32, z: f32 },
    accelerations: [128]struct { x: f32, y: f32, z: f32 },
    point_count: u16,
    duration: f32,
};

pub const BehaviorState = enum(u8) {
    idle = 0,
    following = 1,
    overtaking = 2,
    lane_changing = 3,
    turning = 4,
    stopping = 5,
    emergency = 6,
};

pub const CostFunction = struct {
    time_weight: f32,
    safety_weight: f32,
    comfort_weight: f32,
    efficiency_weight: f32,
};

pub const Constraint = struct {
    max_speed: f32,
    max_acceleration: f32,
    max_jerk: f32,
    min_following_distance: f32,
};

pub const PlannerConfig = struct {
    max_speed: f32 = 50.0,
    max_acceleration: f32 = 5.0,
    max_jerk: f32 = 2.0,
    min_following_distance: f32 = 10.0,
    safety_margin: f32 = 2.0,
    comfort_threshold: f32 = 0.3,
};

pub const PlannerEnvironmentContext = struct {
    enabled: bool,
    vehicle_type: vehicle_dynamics.VehicleType,
    pos_x: f32,
    pos_z: f32,
    current_speed: f32,
};

pub const SpeedGovernanceInput = struct {
    requested_target_speed: f32,
    behavior_speed_cap: f32,
    risk_level: f32,
    environment_context: PlannerEnvironmentContext,
};

pub const SpeedGovernanceOutput = struct {
    constrained_target_speed: f32,
    behavior_cap: f32,
    risk_cap: f32,
    environment_cap: f32,
};

pub const PlannerState = struct {
    current_path: ?*Path,
    current_trajectory: ?*Trajectory,
    behavior: BehaviorState,
    target_speed: f32,
    risk_level: f32,
    constraints: Constraint,
    environment_context: PlannerEnvironmentContext,
};

pub const MAX_WAYPOINTS: usize = 64;
pub const MAX_TRAJECTORY_POINTS: usize = 128;

var g_planner_state: PlannerState = undefined;

fn clamp01(value: f32) f32 {
    return std.math.clamp(value, 0.0, 1.0);
}

fn behaviorSpeedFactor(behavior: BehaviorState) f32 {
    return switch (behavior) {
        .idle => 0.25,
        .following => 0.85,
        .overtaking => 1.0,
        .lane_changing => 0.75,
        .turning => 0.6,
        .stopping => 0.2,
        .emergency => 0.05,
    };
}

fn defaultEnvironmentContext() PlannerEnvironmentContext {
    return .{
        .enabled = false,
        .vehicle_type = .car,
        .pos_x = 0.0,
        .pos_z = 0.0,
        .current_speed = 0.0,
    };
}

fn defaultEnvironmentAuthority() vehicle_dynamics.EnvironmentControlAuthority {
    return .{
        .speed_scale = 1.0,
        .throttle_scale = 1.0,
        .brake_scale = 1.0,
        .steering_scale = 1.0,
    };
}

pub fn measureEnvironmentAuthorityForContext(context: PlannerEnvironmentContext) vehicle_dynamics.EnvironmentControlAuthority {
    if (!context.enabled) return defaultEnvironmentAuthority();
    return vehicle_dynamics.measureEnvironmentControlAuthority(
        context.vehicle_type,
        context.pos_x,
        context.pos_z,
        context.current_speed,
    );
}

fn measurePlannerEnvironmentAuthority() vehicle_dynamics.EnvironmentControlAuthority {
    return measureEnvironmentAuthorityForContext(g_planner_state.environment_context);
}

fn environmentSpeedCap() f32 {
    const max_speed = g_planner_state.constraints.max_speed;
    const authority = measurePlannerEnvironmentAuthority();
    return std.math.clamp(max_speed * authority.speed_scale, 0.0, max_speed);
}

fn riskSpeedFactor(risk_level: f32) f32 {
    return std.math.clamp(1.0 - clamp01(risk_level) * 0.85, 0.1, 1.0);
}

pub fn computeGovernedTargetSpeed(input: SpeedGovernanceInput) SpeedGovernanceOutput {
    const safe_behavior_cap = @max(0.0, input.behavior_speed_cap);
    const requested = std.math.clamp(input.requested_target_speed, 0.0, safe_behavior_cap);
    const risk_cap = safe_behavior_cap * riskSpeedFactor(input.risk_level);
    const authority = measureEnvironmentAuthorityForContext(input.environment_context);
    const environment_cap = safe_behavior_cap * std.math.clamp(authority.speed_scale, 0.0, 1.0);
    const constrained = @min(@min(requested, risk_cap), environment_cap);
    return .{
        .constrained_target_speed = std.math.clamp(constrained, 0.0, safe_behavior_cap),
        .behavior_cap = safe_behavior_cap,
        .risk_cap = std.math.clamp(risk_cap, 0.0, safe_behavior_cap),
        .environment_cap = std.math.clamp(environment_cap, 0.0, safe_behavior_cap),
    };
}

fn behaviorRiskSpeedCap() f32 {
    const max_speed = g_planner_state.constraints.max_speed;
    const governed = computeGovernedTargetSpeed(.{
        .requested_target_speed = max_speed,
        .behavior_speed_cap = max_speed * behaviorSpeedFactor(g_planner_state.behavior),
        .risk_level = g_planner_state.risk_level,
        .environment_context = g_planner_state.environment_context,
    });
    return std.math.clamp(governed.constrained_target_speed, 0.0, max_speed);
}

pub fn init() void {
    g_planner_state = .{
        .current_path = null,
        .current_trajectory = null,
        .behavior = .idle,
        .target_speed = 0,
        .risk_level = 0,
        .constraints = .{
            .max_speed = 50.0,
            .max_acceleration = 5.0,
            .max_jerk = 2.0,
            .min_following_distance = 10.0,
        },
        .environment_context = defaultEnvironmentContext(),
    };
}

pub fn setBehavior(behavior: BehaviorState) void {
    g_planner_state.behavior = behavior;
}

pub fn getBehavior() BehaviorState {
    return g_planner_state.behavior;
}

pub fn setTargetSpeed(speed: f32) void {
    g_planner_state.target_speed = std.math.clamp(speed, 0.0, g_planner_state.constraints.max_speed);
}

pub fn getTargetSpeed() f32 {
    return g_planner_state.target_speed;
}

pub fn setConstraints(constraints: Constraint) void {
    g_planner_state.constraints = .{
        .max_speed = @max(0.1, constraints.max_speed),
        .max_acceleration = @max(0.1, constraints.max_acceleration),
        .max_jerk = @max(0.1, constraints.max_jerk),
        .min_following_distance = @max(0.0, constraints.min_following_distance),
    };
    g_planner_state.target_speed = std.math.clamp(g_planner_state.target_speed, 0.0, g_planner_state.constraints.max_speed);
}

pub fn setRiskLevel(risk_level: f32) void {
    g_planner_state.risk_level = clamp01(risk_level);
}

pub fn setEnvironmentContext(vehicle_type: vehicle_dynamics.VehicleType, pos_x: f32, pos_z: f32, current_speed: f32) void {
    g_planner_state.environment_context = .{
        .enabled = true,
        .vehicle_type = vehicle_type,
        .pos_x = pos_x,
        .pos_z = pos_z,
        .current_speed = @max(0.0, current_speed),
    };
}

pub fn clearEnvironmentContext() void {
    g_planner_state.environment_context = defaultEnvironmentContext();
}

pub fn getEnvironmentContext() PlannerEnvironmentContext {
    return g_planner_state.environment_context;
}

pub fn getRiskLevel() f32 {
    return g_planner_state.risk_level;
}

pub fn getEffectiveTargetSpeed() f32 {
    return @min(g_planner_state.target_speed, behaviorRiskSpeedCap());
}

pub fn ingestRiskEvidence(distance_to_obstacle: f32, relative_speed: f32) f32 {
    const safe_distance = @max(0.0, distance_to_obstacle);
    const ttc = safe_distance / @max(0.1, @abs(relative_speed));
    const observed = riskAssessment(safe_distance, ttc);
    const retained = g_planner_state.risk_level * 0.75;
    g_planner_state.risk_level = clamp01(@max(retained, observed));
    return g_planner_state.risk_level;
}

pub fn computePathLength(path: *const Path) f32 {
    var length: f32 = 0;
    for (1..path.point_count) |i| {
        const dx = path.points[i].x - path.points[i - 1].x;
        const dz = path.points[i].z - path.points[i - 1].z;
        length += @sqrt(dx * dx + dz * dz);
    }
    return length;
}

pub fn generateStraightPath(start_x: f32, start_z: f32, end_x: f32, end_z: f32) Path {
    var path = Path{
        .points = undefined,
        .point_count = 0,
        .total_length = 0,
    };
    const dx = end_x - start_x;
    const dz = end_z - start_z;
    const dist = @sqrt(dx * dx + dz * dz);
    const segment_count_f = @ceil(dist / 5.0);
    const segment_count = @as(u8, @intFromFloat(@min(63.0, @max(1.0, segment_count_f))));
    const num_points: u8 = segment_count + 1;

    for (0..num_points) |i| {
        const t = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_points - 1));
        path.points[i] = .{
            .x = start_x + dx * t,
            .y = 0,
            .z = start_z + dz * t,
            .yaw = std.math.atan2(dx, dz),
            .speed = 20.0,
        };
        path.point_count = @as(u8, @intCast(i + 1));
    }
    path.total_length = dist;
    return path;
}

pub fn smoothPath(path: *Path) void {
    if (path.point_count < 3) return;
    for (1..path.point_count - 1) |i| {
        path.points[i].x = (path.points[i - 1].x + path.points[i].x + path.points[i + 1].x) / 3.0;
        path.points[i].z = (path.points[i - 1].z + path.points[i].z + path.points[i + 1].z) / 3.0;
    }
}

pub fn generateTrajectory(path: *const Path, initial_speed: f32, target_speed: f32, dt: f32) Trajectory {
    var traj = Trajectory{
        .positions = undefined,
        .velocities = undefined,
        .accelerations = undefined,
        .point_count = 0,
        .duration = 0,
    };

    const safe_dt = @max(0.001, dt);
    const speed_cap = environmentSpeedCap();
    var speed = initial_speed;
    var accel: f32 = 0;
    var time: f32 = 0;
    var idx: u16 = 0;

    for (0..path.point_count) |i| {
        if (idx >= MAX_TRAJECTORY_POINTS) break;
        if (i > 0) {
            const dx = path.points[i].x - path.points[i - 1].x;
            const dz = path.points[i].z - path.points[i - 1].z;
            const segment_length = @sqrt(dx * dx + dz * dz);
            const target_segment_speed = std.math.clamp(
                path.points[i].speed * 0.5 + target_speed * 0.5,
                0.0,
                speed_cap,
            );
            const speed_diff = target_segment_speed - speed;
            accel = speed_diff / safe_dt;
            accel = @max(-g_planner_state.constraints.max_acceleration, @min(g_planner_state.constraints.max_acceleration, accel));
            speed += accel * safe_dt;
            speed = @max(0, @min(speed_cap, speed));
            if (segment_length > 0.0001) {
                time += segment_length / @max(0.1, speed);
            } else {
                time += safe_dt;
            }
        }
        traj.positions[idx] = .{ .x = path.points[i].x, .y = path.points[i].y, .z = path.points[i].z };
        traj.velocities[idx] = .{
            .x = @sin(path.points[i].yaw) * speed,
            .y = 0,
            .z = @cos(path.points[i].yaw) * speed,
        };
        traj.accelerations[idx] = .{ .x = @cos(path.points[i].yaw) * accel, .y = 0, .z = @sin(path.points[i].yaw) * accel };
        traj.point_count = @as(u16, @intCast(idx + 1));
        idx += 1;
    }
    traj.duration = time;
    return traj;
}

pub fn computeVelocityProfile(path: *Path, initial_speed: f32, target_speed: f32) void {
    if (path.point_count == 0) return;
    if (path.point_count == 1) {
        path.points[0].speed = initial_speed;
        return;
    }
    for (0..path.point_count) |i| {
        const t = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(path.point_count - 1));
        path.points[i].speed = initial_speed + (target_speed - initial_speed) * t;
    }
}

pub fn computeSafetyCost(distance_to_obstacle: f32, relative_speed: f32) f32 {
    const ttc = distance_to_obstacle / @max(0.1, @abs(relative_speed));
    const time_margin = @max(0, ttc - 2.0);
    return 1.0 / (1.0 + time_margin);
}

pub fn computeTimeCost(current_speed: f32, target_speed: f32) f32 {
    const speed_diff = target_speed - current_speed;
    return @abs(speed_diff) * 0.1;
}

pub fn computeComfortCost(jerk: f32) f32 {
    return @min(1.0, @abs(jerk) / g_planner_state.constraints.max_jerk);
}

pub fn computeEfficiencyCost(current_speed: f32, target_speed: f32) f32 {
    const max_speed = @max(0.1, g_planner_state.constraints.max_speed);
    const clamped_current = std.math.clamp(current_speed, 0.0, max_speed);
    const clamped_target = std.math.clamp(target_speed, 0.0, max_speed);
    const utilization_loss = 1.0 - clamped_current / max_speed;
    const tracking_loss = @abs(clamped_target - clamped_current) / max_speed;
    return std.math.clamp(utilization_loss * 0.4 + tracking_loss * 0.6, 0.0, 1.0);
}

pub fn computeCost(distance_to_obstacle: f32, relative_speed: f32, jerk: f32, cost_fn: CostFunction) f32 {
    const effective_target = getEffectiveTargetSpeed();
    const estimated_current_speed = std.math.clamp(
        effective_target - relative_speed * 0.4,
        0.0,
        g_planner_state.constraints.max_speed,
    );
    const safety = computeSafetyCost(distance_to_obstacle, relative_speed);
    const time = computeTimeCost(estimated_current_speed, effective_target);
    const comfort = computeComfortCost(jerk);
    const efficiency = computeEfficiencyCost(estimated_current_speed, effective_target);
    const risk_penalty = g_planner_state.risk_level * safety * 0.2;
    return safety * cost_fn.safety_weight +
        time * cost_fn.time_weight +
        comfort * cost_fn.comfort_weight +
        efficiency * cost_fn.efficiency_weight +
        risk_penalty;
}

pub fn optimizeTrajectory(traj: *Trajectory, cost_fn: CostFunction) void {
    if (traj.point_count == 0) return;
    const behavior_risk_cap = behaviorRiskSpeedCap();
    const safety_scale = std.math.clamp(1.0 - cost_fn.safety_weight * 0.6 - g_planner_state.risk_level * 0.5, 0.25, 1.0);
    const comfort_scale = std.math.clamp(1.0 - cost_fn.comfort_weight * 0.35, 0.45, 1.0);
    const bonus = std.math.clamp(cost_fn.time_weight * 0.2 + cost_fn.efficiency_weight * 0.2, 0.0, 0.35);
    var optimized_cap = behavior_risk_cap * safety_scale * comfort_scale;
    optimized_cap += (behavior_risk_cap - optimized_cap) * bonus;
    optimized_cap = std.math.clamp(optimized_cap, 0.0, behavior_risk_cap);
    const point_count = @as(f32, @floatFromInt(@max(traj.point_count - 1, 1)));
    const dt_est = if (traj.duration > 0.0)
        @max(0.001, traj.duration / point_count)
    else
        0.1;
    const max_speed_step = @max(0.01, g_planner_state.constraints.max_acceleration * dt_est);
    var prev_speed: f32 = 0.0;

    for (0..traj.point_count) |i| {
        const vel_x = traj.velocities[i].x;
        const vel_z = traj.velocities[i].z;
        const raw_speed = @sqrt(vel_x * vel_x + vel_z * vel_z);
        var speed = @min(raw_speed, optimized_cap);
        if (i > 0) {
            if (speed > prev_speed + max_speed_step) {
                speed = prev_speed + max_speed_step;
            } else if (speed + max_speed_step < prev_speed) {
                speed = @max(0.0, prev_speed - max_speed_step);
            }
        }

        if (raw_speed > 0.0001) {
            const scale = speed / raw_speed;
            traj.velocities[i].x = vel_x * scale;
            traj.velocities[i].z = vel_z * scale;
        } else {
            traj.velocities[i].x = 0.0;
            traj.velocities[i].z = 0.0;
        }

        if (i > 0) {
            const accel_mag = (speed - prev_speed) / dt_est;
            if (speed > 0.0001) {
                const inv_speed = 1.0 / speed;
                traj.accelerations[i].x = traj.velocities[i].x * inv_speed * accel_mag;
                traj.accelerations[i].z = traj.velocities[i].z * inv_speed * accel_mag;
            } else {
                traj.accelerations[i].x = 0.0;
                traj.accelerations[i].z = 0.0;
            }
            traj.accelerations[i].y = 0.0;
        }

        prev_speed = speed;
    }
}

pub fn checkConstraintViolation(traj: *const Trajectory) bool {
    for (0..traj.point_count) |i| {
        const speed = @sqrt(traj.velocities[i].x * traj.velocities[i].x + traj.velocities[i].z * traj.velocities[i].z);
        if (speed > g_planner_state.constraints.max_speed) return true;
        const accel = @sqrt(traj.accelerations[i].x * traj.accelerations[i].x + traj.accelerations[i].z * traj.accelerations[i].z);
        if (accel > g_planner_state.constraints.max_acceleration) return true;
    }
    return false;
}

fn trajectoryMaxSpeed(traj: *const Trajectory) f32 {
    var max_speed: f32 = 0.0;
    for (0..traj.point_count) |i| {
        const speed = @sqrt(traj.velocities[i].x * traj.velocities[i].x + traj.velocities[i].z * traj.velocities[i].z);
        if (speed > max_speed) max_speed = speed;
    }
    return max_speed;
}

pub fn mpcPredict(traj: *const Trajectory, horizon: u16) struct { x: f32, z: f32 } {
    if (traj.point_count == 0) return .{ .x = 0.0, .z = 0.0 };
    const look_idx = @min(horizon, traj.point_count - 1);
    return .{ .x = traj.positions[look_idx].x, .z = traj.positions[look_idx].z };
}

pub fn riskAssessment(distance_to_obstacle: f32, time_to_collision: f32) f32 {
    if (distance_to_obstacle < 5.0) return 1.0;
    if (time_to_collision < 1.0) return 0.9;
    if (time_to_collision < 2.0) return 0.6;
    if (time_to_collision < 4.0) return 0.3;
    return 0.0;
}

pub fn updatePlanner(dt: f32) void {
    const safe_dt = @max(0.0, dt);
    const decay = std.math.clamp(safe_dt * 0.5, 0.0, 1.0);
    g_planner_state.risk_level = clamp01(g_planner_state.risk_level * (1.0 - decay));

    const authority = measurePlannerEnvironmentAuthority();
    const accel_scale = std.math.clamp(authority.throttle_scale, 0.1, 1.0);
    const braking_scale = std.math.clamp(1.0 / @max(0.5, authority.brake_scale), 0.35, 1.0);
    const constrained_target = getEffectiveTargetSpeed();
    const max_up_step = @max(0.0, g_planner_state.constraints.max_acceleration * safe_dt * accel_scale);
    const max_down_step = @max(0.0, g_planner_state.constraints.max_acceleration * safe_dt * braking_scale);
    if (g_planner_state.target_speed > constrained_target + max_down_step) {
        g_planner_state.target_speed -= max_down_step;
    } else if (g_planner_state.target_speed + max_up_step < constrained_target) {
        g_planner_state.target_speed += max_up_step;
    } else {
        g_planner_state.target_speed = constrained_target;
    }
    g_planner_state.target_speed = std.math.clamp(g_planner_state.target_speed, 0.0, g_planner_state.constraints.max_speed);
}

pub fn getPlannerState() *PlannerState {
    return &g_planner_state;
}

// ============================================================================
// Tests for Planner System (Items 611-625)
// ============================================================================

test "611: path planning - straight path generation" {
    init();
    const path = generateStraightPath(0, 0, 100, 0);
    try std.testing.expect(path.point_count >= 2);
    try std.testing.expect(path.total_length > 0);
}

test "612: trajectory planning - velocity profile" {
    init();
    var path = generateStraightPath(0, 0, 100, 0);
    computeVelocityProfile(&path, 0, 30);
    try std.testing.expect(path.points[0].speed == 0);
    try std.testing.expect(path.points[path.point_count - 1].speed == 30);
}

test "613: speed planning - target speed profile" {
    init();
    setTargetSpeed(30.0);
    try std.testing.expect(getTargetSpeed() == 30.0);
}

test "614: behavior planning - state transitions" {
    init();
    setBehavior(.following);
    try std.testing.expect(getBehavior() == .following);
    setBehavior(.lane_changing);
    try std.testing.expect(getBehavior() == .lane_changing);
}

test "615: decision planning - cost evaluation" {
    init();
    const cost = computeSafetyCost(20.0, 5.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 3.0), cost, 0.0001);
}

test "616: risk assessment - collision risk scoring" {
    init();
    const risk = riskAssessment(10.0, 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.3), risk, 0.0001);
}

test "617: cost function - weighted cost computation" {
    init();
    setBehavior(.following);
    setTargetSpeed(30.0);
    setRiskLevel(0.1);
    const cost_fn = CostFunction{
        .time_weight = 0.3,
        .safety_weight = 0.5,
        .comfort_weight = 0.2,
        .efficiency_weight = 0.1,
    };
    const baseline = computeCost(30.0, 1.0, 0.2, cost_fn);
    const closing_fast = computeCost(30.0, 10.0, 0.2, cost_fn);
    setRiskLevel(0.8);
    const high_risk = computeCost(30.0, 1.0, 0.2, cost_fn);
    try std.testing.expect(closing_fast > baseline);
    try std.testing.expect(high_risk > baseline);
}

test "618: constraint satisfaction - speed limit checking" {
    init();
    const path = generateStraightPath(0, 0, 100, 0);
    const traj = generateTrajectory(&path, 0, 50, 0.1);
    const violated = checkConstraintViolation(&traj);
    try std.testing.expect(!violated);
}

test "619: optimization solver - trajectory optimization" {
    init();
    setBehavior(.following);
    setTargetSpeed(45.0);
    setRiskLevel(0.7);
    const path = generateStraightPath(0, 0, 100, 0);
    var traj = generateTrajectory(&path, 0, 45, 0.1);
    const speed_before = trajectoryMaxSpeed(&traj);
    const cost_fn = CostFunction{
        .time_weight = 0.1,
        .safety_weight = 0.8,
        .comfort_weight = 0.4,
        .efficiency_weight = 0.1,
    };
    optimizeTrajectory(&traj, cost_fn);
    const speed_after = trajectoryMaxSpeed(&traj);
    try std.testing.expect(speed_after <= speed_before);
    try std.testing.expect(speed_after <= getEffectiveTargetSpeed() + 0.001);
    try std.testing.expect(!checkConstraintViolation(&traj));
}

test "620: model predictive control - horizon prediction" {
    init();
    const path = generateStraightPath(0, 0, 100, 0);
    const traj = generateTrajectory(&path, 0, 30, 0.1);
    const mid_pred = mpcPredict(&traj, 2);
    try std.testing.expectApproxEqAbs(traj.positions[2].x, mid_pred.x, 0.0001);
    try std.testing.expectApproxEqAbs(traj.positions[2].z, mid_pred.z, 0.0001);
    const last_pred = mpcPredict(&traj, 1000);
    const last_idx = traj.point_count - 1;
    try std.testing.expectApproxEqAbs(traj.positions[last_idx].x, last_pred.x, 0.0001);
    try std.testing.expectApproxEqAbs(traj.positions[last_idx].z, last_pred.z, 0.0001);
}

test "621: state machine - behavior state tracking" {
    init();
    setBehavior(.idle);
    try std.testing.expect(getBehavior() == .idle);
    setBehavior(.emergency);
    try std.testing.expect(getBehavior() == .emergency);
}

test "622: behavior tree - hierarchical decisions" {
    init();
    setBehavior(.following);
    setTargetSpeed(20.0);
    try std.testing.expect(getBehavior() == .following);
}

test "623: rule engine - constraint-based decisions" {
    init();
    const constraints = Constraint{
        .max_speed = 30.0,
        .max_acceleration = 3.0,
        .max_jerk = 1.5,
        .min_following_distance = 15.0,
    };
    setConstraints(constraints);
    try std.testing.expect(getPlannerState().constraints.max_speed == 30.0);
}

test "624: learning interface - planner state access" {
    init();
    const state = getPlannerState();
    try std.testing.expect(state.behavior == .idle);
    try std.testing.expect(state.constraints.max_speed > 0);
}

test "625: debug interface - trajectory visualization data" {
    init();
    setBehavior(.following);
    setTargetSpeed(35.0);
    const path = generateStraightPath(0, 0, 50, 0);
    var traj = generateTrajectory(&path, 0, 35, 0.1);
    const speed_before = trajectoryMaxSpeed(&traj);
    setRiskLevel(0.85);
    optimizeTrajectory(&traj, .{
        .time_weight = 0.1,
        .safety_weight = 0.9,
        .comfort_weight = 0.4,
        .efficiency_weight = 0.1,
    });
    try std.testing.expect(traj.point_count > 0);
    try std.testing.expect(traj.duration > 0);
    try std.testing.expect(trajectoryMaxSpeed(&traj) <= speed_before);
    for (1..traj.point_count) |i| {
        try std.testing.expect(traj.positions[i].x >= traj.positions[i - 1].x);
    }
}

test "planner ingestRiskEvidence and updatePlanner clamp risk and speed" {
    init();
    setBehavior(.following);
    setTargetSpeed(45.0);
    const risk = ingestRiskEvidence(2.0, 15.0);
    try std.testing.expect(risk >= 0.9);
    const speed_before_update = getTargetSpeed();
    updatePlanner(0.5);
    try std.testing.expect(getRiskLevel() < risk);
    try std.testing.expect(getTargetSpeed() < speed_before_update);
    try std.testing.expect(getTargetSpeed() >= getEffectiveTargetSpeed());
}

test "planner generateTrajectory aligns velocity with path heading" {
    init();
    var path = generateStraightPath(0, 0, 0, 20);
    computeVelocityProfile(&path, 5, 5);
    const traj = generateTrajectory(&path, 5, 5, 0.1);
    try std.testing.expect(traj.point_count >= 2);
    try std.testing.expect(@abs(traj.velocities[1].x) < 0.0001);
    try std.testing.expect(traj.velocities[1].z > 0.0);
}

test "planner computeVelocityProfile handles single-point path" {
    init();
    var path = Path{
        .points = undefined,
        .point_count = 1,
        .total_length = 0,
    };
    path.points[0] = .{
        .x = 0,
        .y = 0,
        .z = 0,
        .yaw = 0,
        .speed = 0,
    };
    computeVelocityProfile(&path, 7.0, 30.0);
    try std.testing.expectApproxEqAbs(@as(f32, 7.0), path.points[0].speed, 0.0001);
}

test "planner mpcPredict handles empty trajectory safely" {
    init();
    const traj = Trajectory{
        .positions = undefined,
        .velocities = undefined,
        .accelerations = undefined,
        .point_count = 0,
        .duration = 0,
    };
    const pred = mpcPredict(&traj, 5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), pred.x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), pred.z, 0.0001);
}

test "planner zero-distance path keeps two valid waypoints" {
    init();
    const path = generateStraightPath(10, 20, 10, 20);
    try std.testing.expect(path.point_count == 2);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), path.points[0].x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), path.points[1].x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 20.0), path.points[0].z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 20.0), path.points[1].z, 0.0001);
}

test "planner governed speed clamps by risk without environment context" {
    init();
    const result = computeGovernedTargetSpeed(.{
        .requested_target_speed = 40.0,
        .behavior_speed_cap = 50.0,
        .risk_level = 0.8,
        .environment_context = .{
            .enabled = false,
            .vehicle_type = .car,
            .pos_x = 0.0,
            .pos_z = 0.0,
            .current_speed = 20.0,
        },
    });
    try std.testing.expect(result.behavior_cap == 50.0);
    try std.testing.expect(result.environment_cap == 50.0);
    try std.testing.expect(result.risk_cap < 20.0);
    try std.testing.expect(result.constrained_target_speed <= result.risk_cap);
}

test "planner governed speed applies environment authority cap when enabled" {
    init();
    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.9, 60.0);
    weather.updateWeather(1.0);

    const result = computeGovernedTargetSpeed(.{
        .requested_target_speed = 50.0,
        .behavior_speed_cap = 50.0,
        .risk_level = 0.0,
        .environment_context = .{
            .enabled = true,
            .vehicle_type = .car,
            .pos_x = 0.0,
            .pos_z = 0.0,
            .current_speed = 25.0,
        },
    });
    try std.testing.expect(result.environment_cap < result.behavior_cap);
    try std.testing.expect(result.constrained_target_speed <= result.environment_cap);

    weather.init();
    terrain.init();
}

test "planner environment context reduces effective target speed on hazardous surface and weather" {
    terrain.init();
    weather.init();
    init();

    setBehavior(.overtaking);
    setTargetSpeed(45.0);
    setEnvironmentContext(.car, 0.0, 0.0, 20.0);
    const clear_effective = getEffectiveTargetSpeed();

    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.9, 60.0);
    weather.updateWeather(1.0);

    const hazard_effective = getEffectiveTargetSpeed();
    try std.testing.expect(hazard_effective < clear_effective);

    clearEnvironmentContext();
    weather.init();
    terrain.init();
}

test "planner environment context remains neutral for aircraft authority model" {
    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 1.0, 60.0);
    weather.triggerWeather(.fog, 1.0, 60.0);
    weather.updateWeather(1.0);

    init();
    setBehavior(.overtaking);
    setTargetSpeed(42.0);
    setEnvironmentContext(.aircraft, 0.0, 0.0, 42.0);
    try std.testing.expectApproxEqAbs(@as(f32, 42.0), getEffectiveTargetSpeed(), 0.0001);
    clearEnvironmentContext();

    weather.init();
    terrain.init();
}
