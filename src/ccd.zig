//! CCD - Continuous Collision Detection
//!
//! Phase 3: Continuous Collision Detection for high-speed objects
//! Prevents tunneling through thin objects

const std = @import("std");
const physics = @import("physics.zig");

/// Time of impact result
pub const TOI = struct {
    hit: bool,
    time: f32, // 0.0 to 1.0, fraction of timestep
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    entity_id: u16,
};

/// Time of entry result for a swept interval test.
pub const TOE = struct {
    hit: bool,
    entry_time: f32,
    exit_time: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
};

/// Iterative time-of-impact result.
pub const IterativeTOI = struct {
    hit: bool,
    time: f32,
    iterations: u32,
    converged: bool,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
};

/// Conservative timestep plan for CCD advancement.
pub const ConservativeStep = struct {
    valid: bool,
    step_fraction: f32,
    substep_count: u32,
    linear_fraction: f32,
    angular_fraction: f32,
    limited: bool,
};

/// Deterministic normalized substep window for CCD advancement.
pub const CCDSubstepPlan = struct {
    valid: bool,
    start_fraction: f32,
    end_fraction: f32,
    step_fraction: f32,
    substep_index: u32,
    substep_count: u32,
    remaining_fraction: f32,
    complete: bool,
    limited: bool,
};

/// Iteration budget cap for iterative CCD solvers.
pub const CCDIterationLimit = struct {
    valid: bool,
    requested_iterations: u32,
    effective_iterations: u32,
    hard_limit: u32,
    estimated_iterations: u32,
    capped: bool,
    tolerance_reachable: bool,
};

/// No-progress watchdog state for iterative CCD advancement.
pub const CCDProgressWatchdog = struct {
    valid: bool,
    progress_delta: f32,
    stagnant_iterations: u32,
    no_progress: bool,
    abort: bool,
    reason_code: u32,
    iteration: u32,
    effective_iterations: u32,
};

/// Non-blocking trigger volume CCD result.
pub const CCDTriggerHit = struct {
    valid: bool,
    triggered: bool,
    entry_time: f32,
    exit_time: f32,
    duration: f32,
    starts_inside: bool,
    ends_inside: bool,
    non_blocking: bool,
    trigger_id: u32,
};

/// Thin-wall tunneling risk detected by continuous CCD but missed by discrete endpoints.
pub const CCDThinWallHit = struct {
    valid: bool,
    risk: bool,
    entry_time: f32,
    exit_time: f32,
    wall_thickness: f32,
    motion_distance: f32,
    starts_overlapping: bool,
    ends_overlapping: bool,
    ccd_required: bool,
};

/// Motion clamp plan that suppresses tunneling before thin-wall impact.
pub const CCDTunnelSuppressionPlan = struct {
    valid: bool,
    suppress: bool,
    safe_time: f32,
    remaining_time: f32,
    entry_time: f32,
    wall_thickness: f32,
    motion_distance: f32,
    clamped_motion_x: f32,
    clamped_motion_y: f32,
    clamped_motion_z: f32,
    ccd_required: bool,
};

/// Deterministic CCD performance gate and work budget.
pub const CCDPerformancePlan = struct {
    valid: bool,
    use_ccd: bool,
    skip_ccd: bool,
    discrete_ok: bool,
    candidate_limit: u32,
    iteration_limit: u32,
    estimated_pair_work: u32,
    motion_ratio: f32,
    angular_ratio: f32,
    reason_code: u32,
};

/// Precision tuning knobs for high-risk CCD sweeps.
pub const CCDPrecisionPlan = struct {
    valid: bool,
    tolerance: f32,
    contact_slop: f32,
    min_progress: f32,
    iteration_limit: u32,
    substep_count: u32,
    precision_tier: u32,
    conservative_step_fraction: f32,
    motion_ratio: f32,
    angular_ratio: f32,
    reason_code: u32,
};

/// Deterministic validation summary for CCD solver stability.
pub const CCDStabilityValidation = struct {
    valid: bool,
    stable: bool,
    bracket_valid: bool,
    precision_valid: bool,
    progress_safe: bool,
    substeps_safe: bool,
    time_error: f32,
    tolerance: f32,
    iteration_limit: u32,
    substep_count: u32,
    reason_code: u32,
};

/// Deterministic scheduling plan for parallel CCD islands.
pub const CCDIslandParallelPlan = struct {
    valid: bool,
    parallel_enabled: bool,
    serial_fallback: bool,
    scheduled_islands: u32,
    worker_count: u32,
    batch_count: u32,
    candidate_limit_per_island: u32,
    iteration_limit: u32,
    estimated_pair_work: u32,
    islands_per_worker: f32,
    reason_code: u32,
};

/// Sleep/wake decision plan for CCD-sensitive bodies.
pub const CCDSleepInteractionPlan = struct {
    valid: bool,
    wake_required: bool,
    sleep_allowed: bool,
    keep_awake: bool,
    ccd_required: bool,
    reset_sleep_tick: bool,
    motion_ratio: f32,
    angular_ratio: f32,
    time_to_impact: f32,
    sleep_progress: f32,
    reason_code: u32,
};

pub const MAX_POLYGON_VERTICES: usize = 16;
pub const DEFAULT_TOI_ITERATION_LIMIT: u32 = 64;

/// Swept AABB for CCD
pub const SweptAABB = struct {
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
};

pub fn makeSweptAABB(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, vel_x: f32, vel_y: f32, vel_z: f32) SweptAABB {
    return .{
        .min_x = min_x,
        .min_y = min_y,
        .min_z = min_z,
        .max_x = max_x,
        .max_y = max_y,
        .max_z = max_z,
        .vel_x = vel_x,
        .vel_y = vel_y,
        .vel_z = vel_z,
    };
}

fn missTOE() TOE {
    return .{ .hit = false, .entry_time = 1.0, .exit_time = 1.0, .normal_x = 0, .normal_y = 0, .normal_z = 0 };
}

fn missIterativeTOI() IterativeTOI {
    return .{ .hit = false, .time = 1.0, .iterations = 0, .converged = false, .normal_x = 0, .normal_y = 0, .normal_z = 0 };
}

fn invalidConservativeStep() ConservativeStep {
    return .{ .valid = false, .step_fraction = 0.0, .substep_count = 0, .linear_fraction = 0.0, .angular_fraction = 0.0, .limited = false };
}

fn invalidCCDSubstepPlan() CCDSubstepPlan {
    return .{ .valid = false, .start_fraction = 0.0, .end_fraction = 0.0, .step_fraction = 0.0, .substep_index = 0, .substep_count = 0, .remaining_fraction = 0.0, .complete = false, .limited = false };
}

fn invalidCCDIterationLimit() CCDIterationLimit {
    return .{ .valid = false, .requested_iterations = 0, .effective_iterations = 0, .hard_limit = 0, .estimated_iterations = 0, .capped = false, .tolerance_reachable = false };
}

fn invalidCCDProgressWatchdog() CCDProgressWatchdog {
    return .{ .valid = false, .progress_delta = 0.0, .stagnant_iterations = 0, .no_progress = false, .abort = false, .reason_code = 0, .iteration = 0, .effective_iterations = 0 };
}

fn invalidCCDTriggerHit() CCDTriggerHit {
    return .{ .valid = false, .triggered = false, .entry_time = 1.0, .exit_time = 1.0, .duration = 0.0, .starts_inside = false, .ends_inside = false, .non_blocking = true, .trigger_id = 0 };
}

fn invalidCCDThinWallHit() CCDThinWallHit {
    return .{ .valid = false, .risk = false, .entry_time = 1.0, .exit_time = 1.0, .wall_thickness = 0.0, .motion_distance = 0.0, .starts_overlapping = false, .ends_overlapping = false, .ccd_required = false };
}

fn invalidCCDTunnelSuppressionPlan() CCDTunnelSuppressionPlan {
    return .{ .valid = false, .suppress = false, .safe_time = 1.0, .remaining_time = 0.0, .entry_time = 1.0, .wall_thickness = 0.0, .motion_distance = 0.0, .clamped_motion_x = 0.0, .clamped_motion_y = 0.0, .clamped_motion_z = 0.0, .ccd_required = false };
}

fn invalidCCDPerformancePlan() CCDPerformancePlan {
    return .{ .valid = false, .use_ccd = false, .skip_ccd = false, .discrete_ok = false, .candidate_limit = 0, .iteration_limit = 0, .estimated_pair_work = 0, .motion_ratio = 0.0, .angular_ratio = 0.0, .reason_code = 0 };
}

fn invalidCCDPrecisionPlan() CCDPrecisionPlan {
    return .{ .valid = false, .tolerance = 0.0, .contact_slop = 0.0, .min_progress = 0.0, .iteration_limit = 0, .substep_count = 0, .precision_tier = 0, .conservative_step_fraction = 0.0, .motion_ratio = 0.0, .angular_ratio = 0.0, .reason_code = 0 };
}

fn invalidCCDStabilityValidation() CCDStabilityValidation {
    return .{ .valid = false, .stable = false, .bracket_valid = false, .precision_valid = false, .progress_safe = false, .substeps_safe = false, .time_error = 0.0, .tolerance = 0.0, .iteration_limit = 0, .substep_count = 0, .reason_code = 0 };
}

fn invalidCCDIslandParallelPlan() CCDIslandParallelPlan {
    return .{ .valid = false, .parallel_enabled = false, .serial_fallback = true, .scheduled_islands = 0, .worker_count = 0, .batch_count = 0, .candidate_limit_per_island = 0, .iteration_limit = 0, .estimated_pair_work = 0, .islands_per_worker = 0.0, .reason_code = 0 };
}

fn invalidCCDSleepInteractionPlan() CCDSleepInteractionPlan {
    return .{ .valid = false, .wake_required = false, .sleep_allowed = false, .keep_awake = false, .ccd_required = false, .reset_sleep_tick = false, .motion_ratio = 0.0, .angular_ratio = 0.0, .time_to_impact = 1.0, .sleep_progress = 0.0, .reason_code = 0 };
}

fn finite3(x: f32, y: f32, z: f32) bool {
    return std.math.isFinite(x) and std.math.isFinite(y) and std.math.isFinite(z);
}

fn dot3(ax: f32, ay: f32, az: f32, bx: f32, by: f32, bz: f32) f32 {
    return ax * bx + ay * by + az * bz;
}

fn normalFromVector(x: f32, y: f32, z: f32, fallback_x: f32, fallback_y: f32, fallback_z: f32) struct { x: f32, y: f32, z: f32 } {
    const len_sq = dot3(x, y, z, x, y, z);
    if (len_sq > 0.000001) {
        const inv_len = 1.0 / @sqrt(len_sq);
        return .{ .x = x * inv_len, .y = y * inv_len, .z = z * inv_len };
    }

    const fallback_len_sq = dot3(fallback_x, fallback_y, fallback_z, fallback_x, fallback_y, fallback_z);
    if (fallback_len_sq > 0.000001) {
        const inv_len = 1.0 / @sqrt(fallback_len_sq);
        return .{ .x = fallback_x * inv_len, .y = fallback_y * inv_len, .z = fallback_z * inv_len };
    }

    return .{ .x = 0.0, .y = 0.0, .z = 0.0 };
}

fn clipTimeWindow(start: *f32, end: *f32, clip_start: f32, clip_end: f32) bool {
    start.* = @max(start.*, clip_start);
    end.* = @min(end.*, clip_end);
    return start.* <= end.*;
}

fn clipLinearRange(base: f32, velocity: f32, min_value: f32, max_value: f32, start: *f32, end: *f32) bool {
    if (@abs(velocity) <= 0.000001) {
        return base >= min_value and base <= max_value;
    }

    const t_min = (min_value - base) / velocity;
    const t_max = (max_value - base) / velocity;
    return clipTimeWindow(start, end, @min(t_min, t_max), @max(t_min, t_max));
}

fn clipQuadraticLTE(a: f32, b: f32, c: f32, start: *f32, end: *f32) bool {
    if (@abs(a) <= 0.000001) {
        if (@abs(b) <= 0.000001) return c <= 0.0;
        const root = -c / b;
        if (b > 0.0) {
            return clipTimeWindow(start, end, -std.math.inf(f32), root);
        }
        return clipTimeWindow(start, end, root, std.math.inf(f32));
    }

    const discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) return false;
    const sqrt_discriminant = @sqrt(discriminant);
    const inv_denom = 1.0 / (2.0 * a);
    const t0 = (-b - sqrt_discriminant) * inv_denom;
    const t1 = (-b + sqrt_discriminant) * inv_denom;
    return clipTimeWindow(start, end, @min(t0, t1), @max(t0, t1));
}

fn updateCapsuleCandidate(
    candidate_start: f32,
    candidate_end: f32,
    rel_x: f32,
    rel_y: f32,
    rel_z: f32,
    rel_vel_x: f32,
    rel_vel_y: f32,
    rel_vel_z: f32,
    half_height_sum: f32,
    entry_time: *f32,
    exit_time: *f32,
    normal_x: *f32,
    normal_y: *f32,
    normal_z: *f32,
    has_hit: *bool,
) void {
    if (candidate_start > candidate_end) return;

    if (!has_hit.* or candidate_start < entry_time.*) {
        entry_time.* = candidate_start;
        const impact_rel_x = rel_x + rel_vel_x * candidate_start;
        const impact_rel_y = rel_y + rel_vel_y * candidate_start;
        const impact_rel_z = rel_z + rel_vel_z * candidate_start;
        const closest_y = if (impact_rel_y > half_height_sum)
            impact_rel_y - half_height_sum
        else if (impact_rel_y < -half_height_sum)
            impact_rel_y + half_height_sum
        else
            0.0;
        const normal = normalFromVector(impact_rel_x, closest_y, impact_rel_z, -rel_vel_x, -rel_vel_y, -rel_vel_z);
        normal_x.* = normal.x;
        normal_y.* = normal.y;
        normal_z.* = normal.z;
    }
    exit_time.* = if (!has_hit.*) candidate_end else @max(exit_time.*, candidate_end);
    has_hit.* = true;
}

fn polygonPointX(points: []const f32, idx: usize) f32 {
    return points[idx * 2];
}

fn polygonPointY(points: []const f32, idx: usize) f32 {
    return points[idx * 2 + 1];
}

fn projectPolygonAxis(points: []const f32, count: usize, axis_x: f32, axis_y: f32) struct { min: f32, max: f32 } {
    var min_projection = polygonPointX(points, 0) * axis_x + polygonPointY(points, 0) * axis_y;
    var max_projection = min_projection;
    var idx: usize = 1;
    while (idx < count) : (idx += 1) {
        const projection = polygonPointX(points, idx) * axis_x + polygonPointY(points, idx) * axis_y;
        min_projection = @min(min_projection, projection);
        max_projection = @max(max_projection, projection);
    }
    return .{ .min = min_projection, .max = max_projection };
}

fn polygonInputsValid(points: []const f32, count: usize) bool {
    if (count < 3 or count > MAX_POLYGON_VERTICES or points.len < count * 2) return false;
    for (points[0 .. count * 2]) |value| {
        if (!std.math.isFinite(value)) return false;
    }
    return true;
}

fn clipPolygonSweepAxis(
    moving_points: []const f32,
    moving_count: usize,
    target_points: []const f32,
    target_count: usize,
    rel_vel_x: f32,
    rel_vel_y: f32,
    axis_x: f32,
    axis_y: f32,
    entry_time: *f32,
    exit_time: *f32,
    normal_x: *f32,
    normal_y: *f32,
) bool {
    const axis_len = @sqrt(axis_x * axis_x + axis_y * axis_y);
    if (axis_len <= 0.0001) return true;
    const nx = axis_x / axis_len;
    const ny = axis_y / axis_len;
    const moving = projectPolygonAxis(moving_points, moving_count, nx, ny);
    const target = projectPolygonAxis(target_points, target_count, nx, ny);
    const velocity = rel_vel_x * nx + rel_vel_y * ny;

    if (@abs(velocity) <= 0.0001) {
        return moving.max >= target.min and moving.min <= target.max;
    }

    const inv_velocity = 1.0 / velocity;
    const axis_entry = if (velocity > 0.0)
        (target.min - moving.max) * inv_velocity
    else
        (target.max - moving.min) * inv_velocity;
    const axis_exit = if (velocity > 0.0)
        (target.max - moving.min) * inv_velocity
    else
        (target.min - moving.max) * inv_velocity;

    if (axis_entry > entry_time.*) {
        entry_time.* = axis_entry;
        if (velocity > 0.0) {
            normal_x.* = -nx;
            normal_y.* = -ny;
        } else {
            normal_x.* = nx;
            normal_y.* = ny;
        }
    }
    exit_time.* = @min(exit_time.*, axis_exit);
    return entry_time.* <= exit_time.*;
}

fn clipPolygonEdgeAxes(
    source_points: []const f32,
    source_count: usize,
    moving_points: []const f32,
    moving_count: usize,
    target_points: []const f32,
    target_count: usize,
    rel_vel_x: f32,
    rel_vel_y: f32,
    entry_time: *f32,
    exit_time: *f32,
    normal_x: *f32,
    normal_y: *f32,
) bool {
    var idx: usize = 0;
    while (idx < source_count) : (idx += 1) {
        const next_idx = (idx + 1) % source_count;
        const edge_x = polygonPointX(source_points, next_idx) - polygonPointX(source_points, idx);
        const edge_y = polygonPointY(source_points, next_idx) - polygonPointY(source_points, idx);
        if (!clipPolygonSweepAxis(
            moving_points,
            moving_count,
            target_points,
            target_count,
            rel_vel_x,
            rel_vel_y,
            -edge_y,
            edge_x,
            entry_time,
            exit_time,
            normal_x,
            normal_y,
        )) return false;
    }
    return true;
}

fn clipEntryAxis(
    moving_min: f32,
    moving_max: f32,
    velocity: f32,
    target_min: f32,
    target_max: f32,
    entry_time: *f32,
    exit_time: *f32,
    normal_x: *f32,
    normal_y: *f32,
    normal_z: *f32,
    axis_normal_x: f32,
    axis_normal_y: f32,
    axis_normal_z: f32,
) bool {
    if (@abs(velocity) <= 0.0001) {
        return moving_max >= target_min and moving_min <= target_max;
    }

    const inv_velocity = 1.0 / velocity;
    const axis_entry = if (velocity > 0.0)
        (target_min - moving_max) * inv_velocity
    else
        (target_max - moving_min) * inv_velocity;
    const axis_exit = if (velocity > 0.0)
        (target_max - moving_min) * inv_velocity
    else
        (target_min - moving_max) * inv_velocity;

    if (axis_entry > entry_time.*) {
        entry_time.* = axis_entry;
        normal_x.* = axis_normal_x;
        normal_y.* = axis_normal_y;
        normal_z.* = axis_normal_z;
        if (velocity < 0.0) {
            normal_x.* = -normal_x.*;
            normal_y.* = -normal_y.*;
            normal_z.* = -normal_z.*;
        }
    }
    exit_time.* = @min(exit_time.*, axis_exit);
    return entry_time.* <= exit_time.*;
}

fn boxInputsValid(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, vel_x: f32, vel_y: f32, vel_z: f32) bool {
    return finite3(min_x, min_y, min_z) and finite3(max_x, max_y, max_z) and finite3(vel_x, vel_y, vel_z) and
        min_x <= max_x and min_y <= max_y and min_z <= max_z;
}

fn aabbOverlapsF32(a_min_x: f32, a_min_y: f32, a_min_z: f32, a_max_x: f32, a_max_y: f32, a_max_z: f32, b_min_x: f32, b_min_y: f32, b_min_z: f32, b_max_x: f32, b_max_y: f32, b_max_z: f32) bool {
    return a_max_x >= b_min_x and a_min_x <= b_max_x and
        a_max_y >= b_min_y and a_min_y <= b_max_y and
        a_max_z >= b_min_z and a_min_z <= b_max_z;
}

fn min3(a: f32, b: f32, c: f32) f32 {
    return @min(a, @min(b, c));
}

fn rotatingBoxInputsValid(center_x: f32, center_y: f32, center_z: f32, half_x: f32, half_y: f32, half_z: f32, yaw_start: f32, yaw_end: f32, vel_x: f32, vel_y: f32, vel_z: f32) bool {
    return finite3(center_x, center_y, center_z) and finite3(half_x, half_y, half_z) and finite3(vel_x, vel_y, vel_z) and
        std.math.isFinite(yaw_start) and std.math.isFinite(yaw_end) and half_x >= 0.0 and half_y >= 0.0 and half_z >= 0.0;
}

fn yawHalfExtentX(half_x: f32, half_z: f32, yaw: f32) f32 {
    return @abs(half_x * @cos(yaw)) + @abs(half_z * @sin(yaw));
}

fn yawHalfExtentZ(half_x: f32, half_z: f32, yaw: f32) f32 {
    return @abs(half_x * @sin(yaw)) + @abs(half_z * @cos(yaw));
}

fn angleInRangeWithPeriod(base_angle: f32, range_min: f32, range_max: f32) bool {
    const tau = std.math.tau;
    var k: i32 = @intFromFloat(@floor((range_min - base_angle) / tau));
    while (k <= @as(i32, @intFromFloat(@ceil((range_max - base_angle) / tau)))) : (k += 1) {
        const candidate = base_angle + @as(f32, @floatFromInt(k)) * tau;
        if (candidate >= range_min and candidate <= range_max) return true;
    }
    return false;
}

fn yawSweepHalfExtents(half_x: f32, half_z: f32, yaw_start: f32, yaw_end: f32) struct { x: f32, z: f32 } {
    const delta = @abs(yaw_end - yaw_start);
    const full_extent = @sqrt(half_x * half_x + half_z * half_z);
    if (delta >= std.math.tau) return .{ .x = full_extent, .z = full_extent };

    const range_min = @min(yaw_start, yaw_end);
    const range_max = @max(yaw_start, yaw_end);
    var extent_x = @max(yawHalfExtentX(half_x, half_z, yaw_start), yawHalfExtentX(half_x, half_z, yaw_end));
    var extent_z = @max(yawHalfExtentZ(half_x, half_z, yaw_start), yawHalfExtentZ(half_x, half_z, yaw_end));

    const critical_x = std.math.atan2(half_z, half_x);
    const critical_z = std.math.atan2(half_x, half_z);
    const bases_x = [_]f32{ critical_x, std.math.pi - critical_x, std.math.pi + critical_x, std.math.tau - critical_x };
    const bases_z = [_]f32{ critical_z, std.math.pi - critical_z, std.math.pi + critical_z, std.math.tau - critical_z };
    for (bases_x) |base| {
        if (angleInRangeWithPeriod(base, range_min, range_max)) extent_x = full_extent;
    }
    for (bases_z) |base| {
        if (angleInRangeWithPeriod(base, range_min, range_max)) extent_z = full_extent;
    }

    return .{ .x = extent_x, .z = extent_z };
}

/// Compute the time of entry/exit for a swept AABB against a static AABB.
pub fn computeTimeOfEntry(swept: SweptAABB, target: physics.AABB) TOE {
    var entry_time: f32 = 0.0;
    var exit_time: f32 = 1.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    if (!clipEntryAxis(swept.min_x, swept.max_x, swept.vel_x, @floatFromInt(target.min_x), @floatFromInt(target.max_x), &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, -1.0, 0.0, 0.0)) return missTOE();
    if (!clipEntryAxis(swept.min_y, swept.max_y, swept.vel_y, @floatFromInt(target.min_y), @floatFromInt(target.max_y), &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, 0.0, -1.0, 0.0)) return missTOE();
    if (!clipEntryAxis(swept.min_z, swept.max_z, swept.vel_z, @floatFromInt(target.min_z), @floatFromInt(target.max_z), &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, 0.0, 0.0, -1.0)) return missTOE();

    if (exit_time < 0.0 or entry_time > 1.0) return missTOE();
    return .{
        .hit = true,
        .entry_time = @max(0.0, entry_time),
        .exit_time = @min(1.0, @max(0.0, exit_time)),
        .normal_x = normal_x,
        .normal_y = normal_y,
        .normal_z = normal_z,
    };
}

fn sweptAABBTouchesTargetAtTime(swept: SweptAABB, target: physics.AABB, time: f32) bool {
    const min_x = swept.min_x + swept.vel_x * time;
    const max_x = swept.max_x + swept.vel_x * time;
    const min_y = swept.min_y + swept.vel_y * time;
    const max_y = swept.max_y + swept.vel_y * time;
    const min_z = swept.min_z + swept.vel_z * time;
    const max_z = swept.max_z + swept.vel_z * time;

    return max_x >= @as(f32, @floatFromInt(target.min_x)) and min_x <= @as(f32, @floatFromInt(target.max_x)) and
        max_y >= @as(f32, @floatFromInt(target.min_y)) and min_y <= @as(f32, @floatFromInt(target.max_y)) and
        max_z >= @as(f32, @floatFromInt(target.min_z)) and min_z <= @as(f32, @floatFromInt(target.max_z));
}

/// Compute a deterministic hard iteration budget for iterative CCD solvers.
pub fn computeCCDIterationLimit(requested_iterations: u32, tolerance: f32, initial_interval: f32, hard_limit: u32) CCDIterationLimit {
    if (!std.math.isFinite(tolerance) or !std.math.isFinite(initial_interval) or tolerance < 0.0 or initial_interval < 0.0 or hard_limit == 0) return invalidCCDIterationLimit();

    var estimated_iterations: u32 = 0;
    if (initial_interval > tolerance and tolerance > 0.0) {
        var interval = initial_interval;
        while (estimated_iterations < hard_limit and interval > tolerance) : (estimated_iterations += 1) {
            interval *= 0.5;
        }
    } else if (initial_interval > tolerance and tolerance == 0.0) {
        estimated_iterations = hard_limit;
    }

    const effective_iterations = @min(requested_iterations, hard_limit);

    return .{
        .valid = true,
        .requested_iterations = requested_iterations,
        .effective_iterations = effective_iterations,
        .hard_limit = hard_limit,
        .estimated_iterations = estimated_iterations,
        .capped = requested_iterations > hard_limit,
        .tolerance_reachable = estimated_iterations <= effective_iterations,
    };
}

/// Detect CCD solver stalls before they consume the whole iteration budget.
pub fn computeCCDProgressWatchdog(
    previous_time: f32,
    current_time: f32,
    min_progress: f32,
    stagnant_iterations: u32,
    max_stagnant_iterations: u32,
    iteration: u32,
    effective_iterations: u32,
) CCDProgressWatchdog {
    if (!std.math.isFinite(previous_time) or !std.math.isFinite(current_time) or !std.math.isFinite(min_progress) or
        previous_time < 0.0 or current_time < 0.0 or min_progress < 0.0 or max_stagnant_iterations == 0 or effective_iterations == 0) return invalidCCDProgressWatchdog();

    const progress_delta = @abs(current_time - previous_time);
    const no_progress = progress_delta <= min_progress;
    const next_stagnant = if (no_progress) stagnant_iterations +| 1 else 0;
    const hit_iteration_limit = iteration >= effective_iterations;
    const hit_stagnant_limit = next_stagnant >= max_stagnant_iterations;

    return .{
        .valid = true,
        .progress_delta = progress_delta,
        .stagnant_iterations = next_stagnant,
        .no_progress = no_progress,
        .abort = hit_stagnant_limit or hit_iteration_limit,
        .reason_code = if (hit_stagnant_limit) 1 else if (hit_iteration_limit) 2 else 0,
        .iteration = iteration,
        .effective_iterations = effective_iterations,
    };
}

/// Iteratively refine TOI from a TOE-provided bracket.
pub fn solveTOIIterative(swept: SweptAABB, target: physics.AABB, max_iterations: u32, tolerance: f32) IterativeTOI {
    const toe = computeTimeOfEntry(swept, target);
    if (!toe.hit) return missIterativeTOI();

    const safe_tolerance = @max(0.0, tolerance);
    if (toe.entry_time <= safe_tolerance) {
        return .{
            .hit = true,
            .time = 0.0,
            .iterations = 0,
            .converged = true,
            .normal_x = toe.normal_x,
            .normal_y = toe.normal_y,
            .normal_z = toe.normal_z,
        };
    }

    var low: f32 = 0.0;
    var high: f32 = toe.entry_time;
    var iterations: u32 = 0;
    const limit = computeCCDIterationLimit(max_iterations, safe_tolerance, high - low, DEFAULT_TOI_ITERATION_LIMIT);
    const effective_iterations = if (limit.valid) limit.effective_iterations else 0;
    while (iterations < effective_iterations and high - low > safe_tolerance) : (iterations += 1) {
        const mid = (low + high) * 0.5;
        if (sweptAABBTouchesTargetAtTime(swept, target, mid)) {
            high = mid;
        } else {
            low = mid;
        }
    }

    return .{
        .hit = true,
        .time = high,
        .iterations = iterations,
        .converged = high - low <= safe_tolerance,
        .normal_x = toe.normal_x,
        .normal_y = toe.normal_y,
        .normal_z = toe.normal_z,
    };
}

/// Check if swept AABB collides with static AABB
pub fn sweptAABBvsAABB(swept: SweptAABB, target: physics.AABB) TOI {
    const toe = computeTimeOfEntry(swept, target);
    if (!toe.hit) return .{ .hit = false, .time = 1.0, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 255 };
    return .{
        .hit = true,
        .time = toe.entry_time,
        .normal_x = toe.normal_x,
        .normal_y = toe.normal_y,
        .normal_z = toe.normal_z,
        .entity_id = 255,
    };
}

/// Compute time of impact between two moving AABBs
pub fn computeTOI(a_min_x: f32, a_min_y: f32, a_min_z: f32, a_max_x: f32, a_max_y: f32, a_max_z: f32, va_x: f32, va_y: f32, va_z: f32, b_min_x: f32, b_min_y: f32, b_min_z: f32, b_max_x: f32, b_max_y: f32, b_max_z: f32, vb_x: f32, vb_y: f32, vb_z: f32) TOI {
    const rel_vel_x = va_x - vb_x;
    const rel_vel_y = va_y - vb_y;
    const rel_vel_z = va_z - vb_z;

    const swept = makeSweptAABB(a_min_x, a_min_y, a_min_z, a_max_x, a_max_y, a_max_z, rel_vel_x, rel_vel_y, rel_vel_z);
    const target = physics.AABB{ .min_x = @intFromFloat(b_min_x), .min_y = @intFromFloat(b_min_y), .min_z = @intFromFloat(b_min_z), .max_x = @intFromFloat(b_max_x), .max_y = @intFromFloat(b_max_y), .max_z = @intFromFloat(b_max_z) };

    return sweptAABBvsAABB(swept, target);
}

/// Compute time of entry between two moving AABBs.
pub fn computeTOE(a_min_x: f32, a_min_y: f32, a_min_z: f32, a_max_x: f32, a_max_y: f32, a_max_z: f32, va_x: f32, va_y: f32, va_z: f32, b_min_x: f32, b_min_y: f32, b_min_z: f32, b_max_x: f32, b_max_y: f32, b_max_z: f32, vb_x: f32, vb_y: f32, vb_z: f32) TOE {
    const rel_vel_x = va_x - vb_x;
    const rel_vel_y = va_y - vb_y;
    const rel_vel_z = va_z - vb_z;

    const swept = makeSweptAABB(a_min_x, a_min_y, a_min_z, a_max_x, a_max_y, a_max_z, rel_vel_x, rel_vel_y, rel_vel_z);
    const target = physics.AABB{ .min_x = @intFromFloat(b_min_x), .min_y = @intFromFloat(b_min_y), .min_z = @intFromFloat(b_min_z), .max_x = @intFromFloat(b_max_x), .max_y = @intFromFloat(b_max_y), .max_z = @intFromFloat(b_max_z) };

    return computeTimeOfEntry(swept, target);
}

/// Iteratively solve time of impact between two moving AABBs.
pub fn computeTOIIterative(a_min_x: f32, a_min_y: f32, a_min_z: f32, a_max_x: f32, a_max_y: f32, a_max_z: f32, va_x: f32, va_y: f32, va_z: f32, b_min_x: f32, b_min_y: f32, b_min_z: f32, b_max_x: f32, b_max_y: f32, b_max_z: f32, vb_x: f32, vb_y: f32, vb_z: f32, max_iterations: u32, tolerance: f32) IterativeTOI {
    const rel_vel_x = va_x - vb_x;
    const rel_vel_y = va_y - vb_y;
    const rel_vel_z = va_z - vb_z;

    const swept = makeSweptAABB(a_min_x, a_min_y, a_min_z, a_max_x, a_max_y, a_max_z, rel_vel_x, rel_vel_y, rel_vel_z);
    const target = physics.AABB{ .min_x = @intFromFloat(b_min_x), .min_y = @intFromFloat(b_min_y), .min_z = @intFromFloat(b_min_z), .max_x = @intFromFloat(b_max_x), .max_y = @intFromFloat(b_max_y), .max_z = @intFromFloat(b_max_z) };

    return solveTOIIterative(swept, target, max_iterations, tolerance);
}

/// Continuous collision detection for two moving axis-aligned boxes.
pub fn computeBoxCCD(a_min_x: f32, a_min_y: f32, a_min_z: f32, a_max_x: f32, a_max_y: f32, a_max_z: f32, va_x: f32, va_y: f32, va_z: f32, b_min_x: f32, b_min_y: f32, b_min_z: f32, b_max_x: f32, b_max_y: f32, b_max_z: f32, vb_x: f32, vb_y: f32, vb_z: f32) TOE {
    if (!boxInputsValid(a_min_x, a_min_y, a_min_z, a_max_x, a_max_y, a_max_z, va_x, va_y, va_z) or
        !boxInputsValid(b_min_x, b_min_y, b_min_z, b_max_x, b_max_y, b_max_z, vb_x, vb_y, vb_z)) return missTOE();

    const rel_vel_x = va_x - vb_x;
    const rel_vel_y = va_y - vb_y;
    const rel_vel_z = va_z - vb_z;

    var entry_time: f32 = 0.0;
    var exit_time: f32 = 1.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    if (!clipEntryAxis(a_min_x, a_max_x, rel_vel_x, b_min_x, b_max_x, &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, -1.0, 0.0, 0.0)) return missTOE();
    if (!clipEntryAxis(a_min_y, a_max_y, rel_vel_y, b_min_y, b_max_y, &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, 0.0, -1.0, 0.0)) return missTOE();
    if (!clipEntryAxis(a_min_z, a_max_z, rel_vel_z, b_min_z, b_max_z, &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, 0.0, 0.0, -1.0)) return missTOE();
    if (exit_time < 0.0 or entry_time > 1.0) return missTOE();

    return .{
        .hit = true,
        .entry_time = @max(0.0, entry_time),
        .exit_time = @min(1.0, @max(0.0, exit_time)),
        .normal_x = normal_x,
        .normal_y = normal_y,
        .normal_z = normal_z,
    };
}

/// Continuous non-blocking trigger detection for a moving AABB against a trigger AABB.
pub fn computeCCDTriggerAABB(
    a_min_x: f32,
    a_min_y: f32,
    a_min_z: f32,
    a_max_x: f32,
    a_max_y: f32,
    a_max_z: f32,
    va_x: f32,
    va_y: f32,
    va_z: f32,
    trigger_min_x: f32,
    trigger_min_y: f32,
    trigger_min_z: f32,
    trigger_max_x: f32,
    trigger_max_y: f32,
    trigger_max_z: f32,
    trigger_vel_x: f32,
    trigger_vel_y: f32,
    trigger_vel_z: f32,
    trigger_id: u32,
) CCDTriggerHit {
    if (!boxInputsValid(a_min_x, a_min_y, a_min_z, a_max_x, a_max_y, a_max_z, va_x, va_y, va_z) or
        !boxInputsValid(trigger_min_x, trigger_min_y, trigger_min_z, trigger_max_x, trigger_max_y, trigger_max_z, trigger_vel_x, trigger_vel_y, trigger_vel_z)) return invalidCCDTriggerHit();

    const hit = computeBoxCCD(
        a_min_x,
        a_min_y,
        a_min_z,
        a_max_x,
        a_max_y,
        a_max_z,
        va_x,
        va_y,
        va_z,
        trigger_min_x,
        trigger_min_y,
        trigger_min_z,
        trigger_max_x,
        trigger_max_y,
        trigger_max_z,
        trigger_vel_x,
        trigger_vel_y,
        trigger_vel_z,
    );
    if (!hit.hit) return .{
        .valid = true,
        .triggered = false,
        .entry_time = 1.0,
        .exit_time = 1.0,
        .duration = 0.0,
        .starts_inside = false,
        .ends_inside = false,
        .non_blocking = true,
        .trigger_id = trigger_id,
    };

    const starts_inside = aabbOverlapsF32(
        a_min_x,
        a_min_y,
        a_min_z,
        a_max_x,
        a_max_y,
        a_max_z,
        trigger_min_x,
        trigger_min_y,
        trigger_min_z,
        trigger_max_x,
        trigger_max_y,
        trigger_max_z,
    );
    const end_min_x = a_min_x + va_x;
    const end_min_y = a_min_y + va_y;
    const end_min_z = a_min_z + va_z;
    const end_max_x = a_max_x + va_x;
    const end_max_y = a_max_y + va_y;
    const end_max_z = a_max_z + va_z;
    const trigger_end_min_x = trigger_min_x + trigger_vel_x;
    const trigger_end_min_y = trigger_min_y + trigger_vel_y;
    const trigger_end_min_z = trigger_min_z + trigger_vel_z;
    const trigger_end_max_x = trigger_max_x + trigger_vel_x;
    const trigger_end_max_y = trigger_max_y + trigger_vel_y;
    const trigger_end_max_z = trigger_max_z + trigger_vel_z;
    const ends_inside = aabbOverlapsF32(
        end_min_x,
        end_min_y,
        end_min_z,
        end_max_x,
        end_max_y,
        end_max_z,
        trigger_end_min_x,
        trigger_end_min_y,
        trigger_end_min_z,
        trigger_end_max_x,
        trigger_end_max_y,
        trigger_end_max_z,
    );

    return .{
        .valid = true,
        .triggered = true,
        .entry_time = hit.entry_time,
        .exit_time = hit.exit_time,
        .duration = @max(0.0, hit.exit_time - hit.entry_time),
        .starts_inside = starts_inside,
        .ends_inside = ends_inside,
        .non_blocking = true,
        .trigger_id = trigger_id,
    };
}

/// Detect a thin wall that would be missed by start/end discrete overlap tests.
pub fn computeCCDThinWallPenetration(
    a_min_x: f32,
    a_min_y: f32,
    a_min_z: f32,
    a_max_x: f32,
    a_max_y: f32,
    a_max_z: f32,
    va_x: f32,
    va_y: f32,
    va_z: f32,
    wall_min_x: f32,
    wall_min_y: f32,
    wall_min_z: f32,
    wall_max_x: f32,
    wall_max_y: f32,
    wall_max_z: f32,
    wall_vel_x: f32,
    wall_vel_y: f32,
    wall_vel_z: f32,
    max_thin_thickness: f32,
) CCDThinWallHit {
    if (!boxInputsValid(a_min_x, a_min_y, a_min_z, a_max_x, a_max_y, a_max_z, va_x, va_y, va_z) or
        !boxInputsValid(wall_min_x, wall_min_y, wall_min_z, wall_max_x, wall_max_y, wall_max_z, wall_vel_x, wall_vel_y, wall_vel_z) or
        !std.math.isFinite(max_thin_thickness) or max_thin_thickness < 0.0) return invalidCCDThinWallHit();

    const hit = computeBoxCCD(
        a_min_x,
        a_min_y,
        a_min_z,
        a_max_x,
        a_max_y,
        a_max_z,
        va_x,
        va_y,
        va_z,
        wall_min_x,
        wall_min_y,
        wall_min_z,
        wall_max_x,
        wall_max_y,
        wall_max_z,
        wall_vel_x,
        wall_vel_y,
        wall_vel_z,
    );

    const wall_thickness = min3(wall_max_x - wall_min_x, wall_max_y - wall_min_y, wall_max_z - wall_min_z);
    const rel_vel_x = va_x - wall_vel_x;
    const rel_vel_y = va_y - wall_vel_y;
    const rel_vel_z = va_z - wall_vel_z;
    const motion_distance = @sqrt(dot3(rel_vel_x, rel_vel_y, rel_vel_z, rel_vel_x, rel_vel_y, rel_vel_z));
    const starts_overlapping = aabbOverlapsF32(
        a_min_x,
        a_min_y,
        a_min_z,
        a_max_x,
        a_max_y,
        a_max_z,
        wall_min_x,
        wall_min_y,
        wall_min_z,
        wall_max_x,
        wall_max_y,
        wall_max_z,
    );
    const ends_overlapping = aabbOverlapsF32(
        a_min_x + va_x,
        a_min_y + va_y,
        a_min_z + va_z,
        a_max_x + va_x,
        a_max_y + va_y,
        a_max_z + va_z,
        wall_min_x + wall_vel_x,
        wall_min_y + wall_vel_y,
        wall_min_z + wall_vel_z,
        wall_max_x + wall_vel_x,
        wall_max_y + wall_vel_y,
        wall_max_z + wall_vel_z,
    );
    const discrete_missed = !starts_overlapping and !ends_overlapping;
    const thin_enough = wall_thickness <= max_thin_thickness;
    const risk = hit.hit and discrete_missed and thin_enough;

    return .{
        .valid = true,
        .risk = risk,
        .entry_time = if (hit.hit) hit.entry_time else 1.0,
        .exit_time = if (hit.hit) hit.exit_time else 1.0,
        .wall_thickness = wall_thickness,
        .motion_distance = motion_distance,
        .starts_overlapping = starts_overlapping,
        .ends_overlapping = ends_overlapping,
        .ccd_required = risk,
    };
}

/// Build a safe motion plan that stops just before a thin-wall tunneling impact.
pub fn computeCCDTunnelSuppression(
    a_min_x: f32,
    a_min_y: f32,
    a_min_z: f32,
    a_max_x: f32,
    a_max_y: f32,
    a_max_z: f32,
    va_x: f32,
    va_y: f32,
    va_z: f32,
    wall_min_x: f32,
    wall_min_y: f32,
    wall_min_z: f32,
    wall_max_x: f32,
    wall_max_y: f32,
    wall_max_z: f32,
    wall_vel_x: f32,
    wall_vel_y: f32,
    wall_vel_z: f32,
    max_thin_thickness: f32,
    safety_fraction: f32,
) CCDTunnelSuppressionPlan {
    if (!std.math.isFinite(safety_fraction) or safety_fraction < 0.0) return invalidCCDTunnelSuppressionPlan();

    const risk = computeCCDThinWallPenetration(
        a_min_x,
        a_min_y,
        a_min_z,
        a_max_x,
        a_max_y,
        a_max_z,
        va_x,
        va_y,
        va_z,
        wall_min_x,
        wall_min_y,
        wall_min_z,
        wall_max_x,
        wall_max_y,
        wall_max_z,
        wall_vel_x,
        wall_vel_y,
        wall_vel_z,
        max_thin_thickness,
    );
    if (!risk.valid) return invalidCCDTunnelSuppressionPlan();

    if (!risk.risk) {
        return .{
            .valid = true,
            .suppress = false,
            .safe_time = 1.0,
            .remaining_time = 0.0,
            .entry_time = risk.entry_time,
            .wall_thickness = risk.wall_thickness,
            .motion_distance = risk.motion_distance,
            .clamped_motion_x = va_x,
            .clamped_motion_y = va_y,
            .clamped_motion_z = va_z,
            .ccd_required = false,
        };
    }

    const safe_time = @max(0.0, risk.entry_time - safety_fraction);
    return .{
        .valid = true,
        .suppress = true,
        .safe_time = safe_time,
        .remaining_time = 1.0 - safe_time,
        .entry_time = risk.entry_time,
        .wall_thickness = risk.wall_thickness,
        .motion_distance = risk.motion_distance,
        .clamped_motion_x = va_x * safe_time,
        .clamped_motion_y = va_y * safe_time,
        .clamped_motion_z = va_z * safe_time,
        .ccd_required = true,
    };
}

/// Conservative CCD for two yaw-rotating boxes using rotation-swept AABBs.
pub fn computeRotatingBoxCCD(
    center_ax: f32,
    center_ay: f32,
    center_az: f32,
    half_ax: f32,
    half_ay: f32,
    half_az: f32,
    yaw_start_a: f32,
    yaw_end_a: f32,
    vel_ax: f32,
    vel_ay: f32,
    vel_az: f32,
    center_bx: f32,
    center_by: f32,
    center_bz: f32,
    half_bx: f32,
    half_by: f32,
    half_bz: f32,
    yaw_start_b: f32,
    yaw_end_b: f32,
    vel_bx: f32,
    vel_by: f32,
    vel_bz: f32,
) TOE {
    if (!rotatingBoxInputsValid(center_ax, center_ay, center_az, half_ax, half_ay, half_az, yaw_start_a, yaw_end_a, vel_ax, vel_ay, vel_az) or
        !rotatingBoxInputsValid(center_bx, center_by, center_bz, half_bx, half_by, half_bz, yaw_start_b, yaw_end_b, vel_bx, vel_by, vel_bz)) return missTOE();

    const a_extents = yawSweepHalfExtents(half_ax, half_az, yaw_start_a, yaw_end_a);
    const b_extents = yawSweepHalfExtents(half_bx, half_bz, yaw_start_b, yaw_end_b);
    return computeBoxCCD(
        center_ax - a_extents.x,
        center_ay - half_ay,
        center_az - a_extents.z,
        center_ax + a_extents.x,
        center_ay + half_ay,
        center_az + a_extents.z,
        vel_ax,
        vel_ay,
        vel_az,
        center_bx - b_extents.x,
        center_by - half_by,
        center_bz - b_extents.z,
        center_bx + b_extents.x,
        center_by + half_by,
        center_bz + b_extents.z,
        vel_bx,
        vel_by,
        vel_bz,
    );
}

/// CCD for yaw-angular-velocity boxes. Linear velocities are units per second.
pub fn computeAngularVelocityCCD(
    center_ax: f32,
    center_ay: f32,
    center_az: f32,
    half_ax: f32,
    half_ay: f32,
    half_az: f32,
    yaw_a: f32,
    angular_velocity_a: f32,
    vel_ax: f32,
    vel_ay: f32,
    vel_az: f32,
    center_bx: f32,
    center_by: f32,
    center_bz: f32,
    half_bx: f32,
    half_by: f32,
    half_bz: f32,
    yaw_b: f32,
    angular_velocity_b: f32,
    vel_bx: f32,
    vel_by: f32,
    vel_bz: f32,
    time_delta: f32,
) TOE {
    if (!std.math.isFinite(angular_velocity_a) or !std.math.isFinite(angular_velocity_b) or !std.math.isFinite(time_delta) or time_delta < 0.0) return missTOE();

    const yaw_end_a = yaw_a + angular_velocity_a * time_delta;
    const yaw_end_b = yaw_b + angular_velocity_b * time_delta;
    return computeRotatingBoxCCD(
        center_ax,
        center_ay,
        center_az,
        half_ax,
        half_ay,
        half_az,
        yaw_a,
        yaw_end_a,
        vel_ax * time_delta,
        vel_ay * time_delta,
        vel_az * time_delta,
        center_bx,
        center_by,
        center_bz,
        half_bx,
        half_by,
        half_bz,
        yaw_b,
        yaw_end_b,
        vel_bx * time_delta,
        vel_by * time_delta,
        vel_bz * time_delta,
    );
}

/// Compute a normalized conservative advancement step for CCD.
pub fn computeConservativeStep(
    motion_x: f32,
    motion_y: f32,
    motion_z: f32,
    angular_motion: f32,
    min_feature_size: f32,
    sweep_radius: f32,
    safety_factor: f32,
    max_step_fraction: f32,
    max_substeps: u32,
) ConservativeStep {
    if (!finite3(motion_x, motion_y, motion_z) or !std.math.isFinite(angular_motion) or
        !std.math.isFinite(min_feature_size) or !std.math.isFinite(sweep_radius) or
        !std.math.isFinite(safety_factor) or !std.math.isFinite(max_step_fraction) or
        min_feature_size <= 0.0 or sweep_radius < 0.0 or safety_factor <= 0.0 or
        max_step_fraction <= 0.0 or max_substeps == 0) return invalidConservativeStep();

    const linear_distance = @sqrt(dot3(motion_x, motion_y, motion_z, motion_x, motion_y, motion_z));
    const angular_distance = @abs(angular_motion) * sweep_radius;
    const safe_distance = min_feature_size * safety_factor;
    const max_step = @min(1.0, max_step_fraction);

    const linear_fraction = if (linear_distance > safe_distance) safe_distance / linear_distance else 1.0;
    const angular_fraction = if (angular_distance > safe_distance) safe_distance / angular_distance else 1.0;
    const requested_step = @max(0.000001, @min(max_step, @min(linear_fraction, angular_fraction)));
    const requested_substeps: u32 = @intFromFloat(@ceil(1.0 / requested_step));
    const substep_count = @min(max_substeps, @max(@as(u32, 1), requested_substeps));
    const step_fraction = 1.0 / @as(f32, @floatFromInt(substep_count));

    return .{
        .valid = true,
        .step_fraction = step_fraction,
        .substep_count = substep_count,
        .linear_fraction = @min(1.0, linear_fraction),
        .angular_fraction = @min(1.0, angular_fraction),
        .limited = substep_count < requested_substeps,
    };
}

/// Decide whether CCD work can be skipped and clamp candidate/iteration budgets.
pub fn computeCCDPerformancePlan(
    motion_x: f32,
    motion_y: f32,
    motion_z: f32,
    angular_motion: f32,
    min_feature_size: f32,
    sweep_radius: f32,
    candidate_count: u32,
    max_candidates: u32,
    requested_iterations: u32,
    hard_iteration_limit: u32,
    skip_motion_ratio: f32,
) CCDPerformancePlan {
    if (!finite3(motion_x, motion_y, motion_z) or !std.math.isFinite(angular_motion) or
        !std.math.isFinite(min_feature_size) or !std.math.isFinite(sweep_radius) or !std.math.isFinite(skip_motion_ratio) or
        min_feature_size <= 0.0 or sweep_radius < 0.0 or max_candidates == 0 or hard_iteration_limit == 0 or skip_motion_ratio < 0.0) return invalidCCDPerformancePlan();

    const linear_distance = @sqrt(dot3(motion_x, motion_y, motion_z, motion_x, motion_y, motion_z));
    const angular_distance = @abs(angular_motion) * sweep_radius;
    const motion_ratio = linear_distance / min_feature_size;
    const angular_ratio = angular_distance / min_feature_size;
    const discrete_ok = motion_ratio <= skip_motion_ratio and angular_ratio <= skip_motion_ratio;
    const candidate_limit = @min(candidate_count, max_candidates);
    const iteration_limit = @min(requested_iterations, hard_iteration_limit);
    const use_ccd = !discrete_ok and candidate_limit > 0 and iteration_limit > 0;
    const estimated_pair_work = candidate_limit *| iteration_limit;

    return .{
        .valid = true,
        .use_ccd = use_ccd,
        .skip_ccd = !use_ccd,
        .discrete_ok = discrete_ok,
        .candidate_limit = candidate_limit,
        .iteration_limit = iteration_limit,
        .estimated_pair_work = estimated_pair_work,
        .motion_ratio = motion_ratio,
        .angular_ratio = angular_ratio,
        .reason_code = if (discrete_ok) 1 else if (candidate_limit == 0) 2 else if (iteration_limit == 0) 3 else 0,
    };
}

/// Tune solver tolerance, slop and advancement granularity from CCD sweep risk.
pub fn computeCCDPrecisionPlan(
    motion_x: f32,
    motion_y: f32,
    motion_z: f32,
    angular_motion: f32,
    min_feature_size: f32,
    sweep_radius: f32,
    base_tolerance: f32,
    base_contact_slop: f32,
    requested_iterations: u32,
    hard_iteration_limit: u32,
) CCDPrecisionPlan {
    if (!finite3(motion_x, motion_y, motion_z) or !std.math.isFinite(angular_motion) or
        !std.math.isFinite(min_feature_size) or !std.math.isFinite(sweep_radius) or
        !std.math.isFinite(base_tolerance) or !std.math.isFinite(base_contact_slop) or
        min_feature_size <= 0.0 or sweep_radius < 0.0 or base_tolerance <= 0.0 or
        base_contact_slop < 0.0 or requested_iterations == 0 or hard_iteration_limit == 0) return invalidCCDPrecisionPlan();

    const linear_distance = @sqrt(dot3(motion_x, motion_y, motion_z, motion_x, motion_y, motion_z));
    const angular_distance = @abs(angular_motion) * sweep_radius;
    const motion_ratio = linear_distance / min_feature_size;
    const angular_ratio = angular_distance / min_feature_size;
    const risk_ratio = @max(motion_ratio, angular_ratio);

    const precision_tier: u32 = if (risk_ratio <= 0.5)
        0
    else if (risk_ratio <= 2.0)
        1
    else if (risk_ratio <= 8.0)
        2
    else
        3;
    const divisor: f32 = switch (precision_tier) {
        0 => 1.0,
        1 => 2.0,
        2 => 4.0,
        else => 8.0,
    };
    const iteration_multiplier: u32 = precision_tier + 1;
    const tuned_requested = if (requested_iterations > std.math.maxInt(u32) / iteration_multiplier)
        std.math.maxInt(u32)
    else
        requested_iterations * iteration_multiplier;
    const tolerance_floor = min_feature_size * 0.000001;
    const tolerance = @max(tolerance_floor, base_tolerance / divisor);
    const contact_slop = @max(tolerance * 2.0, base_contact_slop / divisor);
    const min_progress = tolerance * 0.25;
    const safety_factor: f32 = switch (precision_tier) {
        0 => 0.5,
        1 => 0.25,
        2 => 0.125,
        else => 0.0625,
    };
    const conservative = computeConservativeStep(
        motion_x,
        motion_y,
        motion_z,
        angular_motion,
        min_feature_size,
        sweep_radius,
        safety_factor,
        1.0,
        256,
    );
    const limit = computeCCDIterationLimit(tuned_requested, tolerance, 1.0, hard_iteration_limit);

    return .{
        .valid = conservative.valid and limit.valid,
        .tolerance = tolerance,
        .contact_slop = contact_slop,
        .min_progress = min_progress,
        .iteration_limit = limit.effective_iterations,
        .substep_count = conservative.substep_count,
        .precision_tier = precision_tier,
        .conservative_step_fraction = conservative.step_fraction,
        .motion_ratio = motion_ratio,
        .angular_ratio = angular_ratio,
        .reason_code = if (motion_ratio > angular_ratio) 1 else if (angular_ratio > motion_ratio) 2 else if (risk_ratio > 0.5) 3 else 0,
    };
}

/// Validate a CCD solve against bracket, precision and progress invariants.
pub fn computeCCDStabilityValidation(
    entry_time: f32,
    exit_time: f32,
    solved_time: f32,
    previous_time: f32,
    current_time: f32,
    motion_x: f32,
    motion_y: f32,
    motion_z: f32,
    angular_motion: f32,
    min_feature_size: f32,
    sweep_radius: f32,
    base_tolerance: f32,
    base_contact_slop: f32,
    requested_iterations: u32,
    hard_iteration_limit: u32,
    max_substeps: u32,
) CCDStabilityValidation {
    if (!std.math.isFinite(entry_time) or !std.math.isFinite(exit_time) or !std.math.isFinite(solved_time) or
        !std.math.isFinite(previous_time) or !std.math.isFinite(current_time) or max_substeps == 0) return invalidCCDStabilityValidation();

    const precision = computeCCDPrecisionPlan(
        motion_x,
        motion_y,
        motion_z,
        angular_motion,
        min_feature_size,
        sweep_radius,
        base_tolerance,
        base_contact_slop,
        requested_iterations,
        hard_iteration_limit,
    );
    if (!precision.valid) return invalidCCDStabilityValidation();

    const bracket_valid = entry_time >= 0.0 and exit_time <= 1.0 and entry_time <= exit_time;
    const time_error = if (solved_time < entry_time)
        entry_time - solved_time
    else if (solved_time > exit_time)
        solved_time - exit_time
    else
        0.0;
    const bracket_safe = bracket_valid and time_error <= precision.tolerance;
    const watchdog = computeCCDProgressWatchdog(previous_time, current_time, precision.min_progress, 0, 3, 1, precision.iteration_limit);
    const progress_safe = watchdog.valid and (!watchdog.no_progress or time_error <= precision.tolerance);
    const substeps_safe = precision.substep_count <= max_substeps;
    const stable = bracket_safe and progress_safe and substeps_safe;

    return .{
        .valid = true,
        .stable = stable,
        .bracket_valid = bracket_valid,
        .precision_valid = precision.valid,
        .progress_safe = progress_safe,
        .substeps_safe = substeps_safe,
        .time_error = time_error,
        .tolerance = precision.tolerance,
        .iteration_limit = precision.iteration_limit,
        .substep_count = precision.substep_count,
        .reason_code = if (!bracket_valid) 1 else if (!bracket_safe) 2 else if (!progress_safe) 3 else if (!substeps_safe) 4 else 0,
    };
}

/// Plan deterministic parallel execution for independent CCD islands.
pub fn computeCCDIslandParallelPlan(
    island_count: u32,
    ccd_island_count: u32,
    candidate_count_per_island: u32,
    max_candidates_per_island: u32,
    requested_iterations: u32,
    hard_iteration_limit: u32,
    requested_workers: u32,
    max_parallel_islands: u32,
    cross_island_pair_count: u32,
) CCDIslandParallelPlan {
    if (island_count == 0 or ccd_island_count > island_count or max_candidates_per_island == 0 or
        hard_iteration_limit == 0 or requested_workers == 0 or max_parallel_islands == 0) return invalidCCDIslandParallelPlan();

    const scheduled_islands = @min(ccd_island_count, max_parallel_islands);
    const effective_workers = if (scheduled_islands == 0) 0 else @min(requested_workers, scheduled_islands);
    const candidate_limit = @min(candidate_count_per_island, max_candidates_per_island);
    const iteration_limit = @min(requested_iterations, hard_iteration_limit);
    const batch_count = if (effective_workers == 0) 0 else (scheduled_islands + effective_workers - 1) / effective_workers;
    const estimated_pair_work = scheduled_islands *| candidate_limit *| iteration_limit;
    const has_parallel_capacity = scheduled_islands > 1 and effective_workers > 1;
    const has_work = candidate_limit > 0 and iteration_limit > 0;
    const isolated = cross_island_pair_count == 0;
    const parallel_enabled = has_parallel_capacity and has_work and isolated;

    return .{
        .valid = true,
        .parallel_enabled = parallel_enabled,
        .serial_fallback = !parallel_enabled,
        .scheduled_islands = scheduled_islands,
        .worker_count = effective_workers,
        .batch_count = batch_count,
        .candidate_limit_per_island = candidate_limit,
        .iteration_limit = iteration_limit,
        .estimated_pair_work = estimated_pair_work,
        .islands_per_worker = if (effective_workers == 0) 0.0 else @as(f32, @floatFromInt(scheduled_islands)) / @as(f32, @floatFromInt(effective_workers)),
        .reason_code = if (ccd_island_count == 0) 1 else if (!isolated) 2 else if (!has_parallel_capacity) 3 else if (!has_work) 4 else if (ccd_island_count > max_parallel_islands) 5 else 0,
    };
}

/// Decide how CCD activity should interact with a body's sleep state.
pub fn computeCCDSleepInteraction(
    is_sleeping: bool,
    sleep_tick: u32,
    sleep_tick_threshold: u32,
    motion_x: f32,
    motion_y: f32,
    motion_z: f32,
    angular_motion: f32,
    min_feature_size: f32,
    sweep_radius: f32,
    time_to_impact: f32,
    ccd_required: bool,
    trigger_only: bool,
) CCDSleepInteractionPlan {
    if (!finite3(motion_x, motion_y, motion_z) or !std.math.isFinite(angular_motion) or
        !std.math.isFinite(min_feature_size) or !std.math.isFinite(sweep_radius) or !std.math.isFinite(time_to_impact) or
        min_feature_size <= 0.0 or sweep_radius < 0.0 or sleep_tick_threshold == 0 or time_to_impact < 0.0) return invalidCCDSleepInteractionPlan();

    const linear_distance = @sqrt(dot3(motion_x, motion_y, motion_z, motion_x, motion_y, motion_z));
    const angular_distance = @abs(angular_motion) * sweep_radius;
    const motion_ratio = linear_distance / min_feature_size;
    const angular_ratio = angular_distance / min_feature_size;
    const high_risk_motion = motion_ratio > 0.5 or angular_ratio > 0.5;
    const blocking_ccd = ccd_required and !trigger_only;
    const impending_impact = blocking_ccd and time_to_impact <= 1.0 and (time_to_impact < 1.0 or high_risk_motion);
    const wake_required = is_sleeping and impending_impact;
    const keep_awake = blocking_ccd and (wake_required or high_risk_motion);
    const sleep_mature = sleep_tick >= sleep_tick_threshold;
    const sleep_allowed = sleep_mature and !wake_required and !keep_awake and (!ccd_required or trigger_only);

    return .{
        .valid = true,
        .wake_required = wake_required,
        .sleep_allowed = sleep_allowed,
        .keep_awake = keep_awake,
        .ccd_required = ccd_required,
        .reset_sleep_tick = wake_required or keep_awake,
        .motion_ratio = motion_ratio,
        .angular_ratio = angular_ratio,
        .time_to_impact = @min(1.0, time_to_impact),
        .sleep_progress = @min(1.0, @as(f32, @floatFromInt(sleep_tick)) / @as(f32, @floatFromInt(sleep_tick_threshold))),
        .reason_code = if (wake_required) 1 else if (keep_awake) 2 else if (sleep_allowed) 3 else if (trigger_only and ccd_required) 4 else if (!ccd_required) 5 else 0,
    };
}

/// Compute the next deterministic CCD substep window from a conservative plan.
pub fn computeCCDSubstepPlan(
    motion_x: f32,
    motion_y: f32,
    motion_z: f32,
    angular_motion: f32,
    min_feature_size: f32,
    sweep_radius: f32,
    safety_factor: f32,
    max_step_fraction: f32,
    max_substeps: u32,
    current_substep: u32,
) CCDSubstepPlan {
    const step = computeConservativeStep(
        motion_x,
        motion_y,
        motion_z,
        angular_motion,
        min_feature_size,
        sweep_radius,
        safety_factor,
        max_step_fraction,
        max_substeps,
    );
    if (!step.valid) return invalidCCDSubstepPlan();

    if (current_substep >= step.substep_count) {
        return .{
            .valid = true,
            .start_fraction = 1.0,
            .end_fraction = 1.0,
            .step_fraction = 0.0,
            .substep_index = step.substep_count,
            .substep_count = step.substep_count,
            .remaining_fraction = 0.0,
            .complete = true,
            .limited = step.limited,
        };
    }

    const count_f: f32 = @floatFromInt(step.substep_count);
    const start_fraction = @as(f32, @floatFromInt(current_substep)) / count_f;
    const end_fraction = @as(f32, @floatFromInt(current_substep + 1)) / count_f;

    return .{
        .valid = true,
        .start_fraction = start_fraction,
        .end_fraction = end_fraction,
        .step_fraction = end_fraction - start_fraction,
        .substep_index = current_substep,
        .substep_count = step.substep_count,
        .remaining_fraction = 1.0 - end_fraction,
        .complete = current_substep + 1 >= step.substep_count,
        .limited = step.limited,
    };
}

/// Swept SAT for two convex 2D polygons. Points are packed as x,y pairs.
pub fn computePolygonCCD(
    moving_points: []const f32,
    moving_count: usize,
    vel_x: f32,
    vel_y: f32,
    target_points: []const f32,
    target_count: usize,
) TOE {
    if (!polygonInputsValid(moving_points, moving_count) or !polygonInputsValid(target_points, target_count)) return missTOE();
    if (!std.math.isFinite(vel_x) or !std.math.isFinite(vel_y)) return missTOE();

    var entry_time: f32 = 0.0;
    var exit_time: f32 = 1.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;

    if (!clipPolygonEdgeAxes(moving_points, moving_count, moving_points, moving_count, target_points, target_count, vel_x, vel_y, &entry_time, &exit_time, &normal_x, &normal_y)) return missTOE();
    if (!clipPolygonEdgeAxes(target_points, target_count, moving_points, moving_count, target_points, target_count, vel_x, vel_y, &entry_time, &exit_time, &normal_x, &normal_y)) return missTOE();
    if (exit_time < 0.0 or entry_time > 1.0) return missTOE();

    return .{
        .hit = true,
        .entry_time = @max(0.0, entry_time),
        .exit_time = @min(1.0, @max(0.0, exit_time)),
        .normal_x = normal_x,
        .normal_y = normal_y,
        .normal_z = 0.0,
    };
}

/// Analytic continuous collision detection for two moving spheres.
pub fn computeSphereCCD(
    center_ax: f32,
    center_ay: f32,
    center_az: f32,
    radius_a: f32,
    vel_ax: f32,
    vel_ay: f32,
    vel_az: f32,
    center_bx: f32,
    center_by: f32,
    center_bz: f32,
    radius_b: f32,
    vel_bx: f32,
    vel_by: f32,
    vel_bz: f32,
) TOE {
    if (!finite3(center_ax, center_ay, center_az) or !finite3(vel_ax, vel_ay, vel_az) or
        !finite3(center_bx, center_by, center_bz) or !finite3(vel_bx, vel_by, vel_bz) or
        !std.math.isFinite(radius_a) or !std.math.isFinite(radius_b) or radius_a < 0.0 or radius_b < 0.0) return missTOE();

    const rel_pos_x = center_ax - center_bx;
    const rel_pos_y = center_ay - center_by;
    const rel_pos_z = center_az - center_bz;
    const rel_vel_x = vel_ax - vel_bx;
    const rel_vel_y = vel_ay - vel_by;
    const rel_vel_z = vel_az - vel_bz;
    const radius_sum = radius_a + radius_b;
    const radius_sq = radius_sum * radius_sum;
    const pos_len_sq = dot3(rel_pos_x, rel_pos_y, rel_pos_z, rel_pos_x, rel_pos_y, rel_pos_z);
    const vel_len_sq = dot3(rel_vel_x, rel_vel_y, rel_vel_z, rel_vel_x, rel_vel_y, rel_vel_z);

    if (pos_len_sq <= radius_sq) {
        const normal = normalFromVector(rel_pos_x, rel_pos_y, rel_pos_z, -rel_vel_x, -rel_vel_y, -rel_vel_z);
        if (vel_len_sq <= 0.000001) {
            return .{ .hit = true, .entry_time = 0.0, .exit_time = 1.0, .normal_x = normal.x, .normal_y = normal.y, .normal_z = normal.z };
        }

        const b = 2.0 * dot3(rel_pos_x, rel_pos_y, rel_pos_z, rel_vel_x, rel_vel_y, rel_vel_z);
        const c = pos_len_sq - radius_sq;
        const discriminant = b * b - 4.0 * vel_len_sq * c;
        const exit_time = if (discriminant >= 0.0) (-b + @sqrt(discriminant)) / (2.0 * vel_len_sq) else 1.0;
        return .{
            .hit = true,
            .entry_time = 0.0,
            .exit_time = @min(1.0, @max(0.0, exit_time)),
            .normal_x = normal.x,
            .normal_y = normal.y,
            .normal_z = normal.z,
        };
    }

    if (vel_len_sq <= 0.000001) return missTOE();

    const b = 2.0 * dot3(rel_pos_x, rel_pos_y, rel_pos_z, rel_vel_x, rel_vel_y, rel_vel_z);
    const c = pos_len_sq - radius_sq;
    const discriminant = b * b - 4.0 * vel_len_sq * c;
    if (discriminant < 0.0) return missTOE();

    const sqrt_discriminant = @sqrt(discriminant);
    const inv_denom = 1.0 / (2.0 * vel_len_sq);
    const entry_time = (-b - sqrt_discriminant) * inv_denom;
    const exit_time = (-b + sqrt_discriminant) * inv_denom;
    if (exit_time < 0.0 or entry_time > 1.0) return missTOE();

    const clamped_entry = @max(0.0, entry_time);
    const normal = normalFromVector(
        rel_pos_x + rel_vel_x * clamped_entry,
        rel_pos_y + rel_vel_y * clamped_entry,
        rel_pos_z + rel_vel_z * clamped_entry,
        -rel_vel_x,
        -rel_vel_y,
        -rel_vel_z,
    );
    return .{
        .hit = true,
        .entry_time = clamped_entry,
        .exit_time = @min(1.0, @max(0.0, exit_time)),
        .normal_x = normal.x,
        .normal_y = normal.y,
        .normal_z = normal.z,
    };
}

/// Continuous collision detection for two moving Y-axis capsules.
pub fn computeCapsuleCCD(
    center_ax: f32,
    center_ay: f32,
    center_az: f32,
    radius_a: f32,
    half_height_a: f32,
    vel_ax: f32,
    vel_ay: f32,
    vel_az: f32,
    center_bx: f32,
    center_by: f32,
    center_bz: f32,
    radius_b: f32,
    half_height_b: f32,
    vel_bx: f32,
    vel_by: f32,
    vel_bz: f32,
) TOE {
    if (!finite3(center_ax, center_ay, center_az) or !finite3(vel_ax, vel_ay, vel_az) or
        !finite3(center_bx, center_by, center_bz) or !finite3(vel_bx, vel_by, vel_bz) or
        !std.math.isFinite(radius_a) or !std.math.isFinite(radius_b) or
        !std.math.isFinite(half_height_a) or !std.math.isFinite(half_height_b) or
        radius_a < 0.0 or radius_b < 0.0 or half_height_a < 0.0 or half_height_b < 0.0) return missTOE();

    const rel_x = center_ax - center_bx;
    const rel_y = center_ay - center_by;
    const rel_z = center_az - center_bz;
    const rel_vel_x = vel_ax - vel_bx;
    const rel_vel_y = vel_ay - vel_by;
    const rel_vel_z = vel_az - vel_bz;
    const radius_sum = radius_a + radius_b;
    const radius_sq = radius_sum * radius_sum;
    const half_height_sum = half_height_a + half_height_b;

    var entry_time: f32 = 1.0;
    var exit_time: f32 = 1.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;
    var has_hit = false;

    const horizontal_a = rel_vel_x * rel_vel_x + rel_vel_z * rel_vel_z;
    const horizontal_b = 2.0 * (rel_x * rel_vel_x + rel_z * rel_vel_z);
    const horizontal_c = rel_x * rel_x + rel_z * rel_z - radius_sq;
    var side_start: f32 = 0.0;
    var side_end: f32 = 1.0;
    if (clipLinearRange(rel_y, rel_vel_y, -half_height_sum, half_height_sum, &side_start, &side_end) and
        clipQuadraticLTE(horizontal_a, horizontal_b, horizontal_c, &side_start, &side_end))
    {
        updateCapsuleCandidate(side_start, side_end, rel_x, rel_y, rel_z, rel_vel_x, rel_vel_y, rel_vel_z, half_height_sum, &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, &has_hit);
    }

    const full_a = rel_vel_x * rel_vel_x + rel_vel_y * rel_vel_y + rel_vel_z * rel_vel_z;
    const above_base_y = rel_y - half_height_sum;
    const above_b = 2.0 * (rel_x * rel_vel_x + above_base_y * rel_vel_y + rel_z * rel_vel_z);
    const above_c = rel_x * rel_x + above_base_y * above_base_y + rel_z * rel_z - radius_sq;
    var above_start: f32 = 0.0;
    var above_end: f32 = 1.0;
    if (clipLinearRange(rel_y, rel_vel_y, half_height_sum, std.math.inf(f32), &above_start, &above_end) and
        clipQuadraticLTE(full_a, above_b, above_c, &above_start, &above_end))
    {
        updateCapsuleCandidate(above_start, above_end, rel_x, rel_y, rel_z, rel_vel_x, rel_vel_y, rel_vel_z, half_height_sum, &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, &has_hit);
    }

    const below_base_y = rel_y + half_height_sum;
    const below_b = 2.0 * (rel_x * rel_vel_x + below_base_y * rel_vel_y + rel_z * rel_vel_z);
    const below_c = rel_x * rel_x + below_base_y * below_base_y + rel_z * rel_z - radius_sq;
    var below_start: f32 = 0.0;
    var below_end: f32 = 1.0;
    if (clipLinearRange(rel_y, rel_vel_y, -std.math.inf(f32), -half_height_sum, &below_start, &below_end) and
        clipQuadraticLTE(full_a, below_b, below_c, &below_start, &below_end))
    {
        updateCapsuleCandidate(below_start, below_end, rel_x, rel_y, rel_z, rel_vel_x, rel_vel_y, rel_vel_z, half_height_sum, &entry_time, &exit_time, &normal_x, &normal_y, &normal_z, &has_hit);
    }

    if (!has_hit or exit_time < 0.0 or entry_time > 1.0) return missTOE();
    return .{
        .hit = true,
        .entry_time = @max(0.0, entry_time),
        .exit_time = @min(1.0, @max(0.0, exit_time)),
        .normal_x = normal_x,
        .normal_y = normal_y,
        .normal_z = normal_z,
    };
}

/// Angular sweep for rotating objects
/// Returns AABB that encompasses rotated box
pub fn angularSweepAABB(center_x: f32, center_y: f32, center_z: f32, half_extent_x: f32, half_extent_y: f32, half_extent_z: f32) physics.AABB {
    // Simplified: just expand by sqrt(2) for diagonal rotation
    const expansion = @sqrt(2.0);
    const ex = half_extent_x * expansion;
    const ey = half_extent_y * expansion;
    const ez = half_extent_z * expansion;

    return .{
        .min_x = @intFromFloat(center_x - ex),
        .min_y = @intFromFloat(center_y - ey),
        .min_z = @intFromFloat(center_z - ez),
        .max_x = @intFromFloat(center_x + ex),
        .max_y = @intFromFloat(center_y + ey),
        .max_z = @intFromFloat(center_z + ez),
    };
}

/// Check if a point is inside an AABB
pub fn pointInAABB(px: f32, py: f32, pz: f32, box: physics.AABB) bool {
    return px >= @as(f32, @floatFromInt(box.min_x)) and px <= @as(f32, @floatFromInt(box.max_x)) and
        py >= @as(f32, @floatFromInt(box.min_y)) and py <= @as(f32, @floatFromInt(box.max_y)) and
        pz >= @as(f32, @floatFromInt(box.min_z)) and pz <= @as(f32, @floatFromInt(box.max_z));
}

/// Check CCD for entity falling at high speed
/// Returns true if collision would occur within the timestep
pub fn checkCCDCollision(pos_x: i32, pos_y: i32, pos_z: i32, vel_x: i16, vel_y: i16, vel_z: i16, extent: i32, target: physics.AABB) TOI {
    const swept = makeSweptAABB(
        @as(f32, @floatFromInt(pos_x)) - @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_y)) - @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_z)) - @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_x)) + @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_y)) + @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(pos_z)) + @as(f32, @floatFromInt(extent)),
        @as(f32, @floatFromInt(vel_x)),
        @as(f32, @floatFromInt(vel_y)),
        @as(f32, @floatFromInt(vel_z)),
    );
    return sweptAABBvsAABB(swept, target);
}

test "computeTimeOfEntry reports entry exit and impact normal" {
    const swept = makeSweptAABB(0, 0, 0, 1, 1, 1, 10, 0, 0);
    const target = physics.AABB.init(3, 0, 0, 4, 1, 1);

    const toe = computeTimeOfEntry(swept, target);
    try std.testing.expect(toe.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), toe.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), toe.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), toe.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), toe.normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), toe.normal_z, 0.0001);
}

test "computeTimeOfEntry rejects parallel separated sweep" {
    const swept = makeSweptAABB(0, 0, 0, 1, 1, 1, 10, 0, 0);
    const target = physics.AABB.init(3, 2, 0, 4, 3, 1);

    const toe = computeTimeOfEntry(swept, target);
    try std.testing.expect(!toe.hit);
}

test "solveTOIIterative refines bracketed impact time" {
    const swept = makeSweptAABB(0, 0, 0, 1, 1, 1, 10, 0, 0);
    const target = physics.AABB.init(3, 0, 0, 4, 1, 1);

    const toi = solveTOIIterative(swept, target, 16, 0.001);
    try std.testing.expect(toi.hit);
    try std.testing.expect(toi.converged);
    try std.testing.expect(toi.iterations > 0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), toi.time, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), toi.normal_x, 0.0001);
}

test "computeCCDIterationLimit clamps requested iterations" {
    const limit = computeCCDIterationLimit(128, 0.001, 1.0, 32);
    try std.testing.expect(limit.valid);
    try std.testing.expect(limit.capped);
    try std.testing.expect(limit.tolerance_reachable);
    try std.testing.expectEqual(@as(u32, 128), limit.requested_iterations);
    try std.testing.expectEqual(@as(u32, 32), limit.effective_iterations);
    try std.testing.expectEqual(@as(u32, 32), limit.hard_limit);
    try std.testing.expectEqual(@as(u32, 10), limit.estimated_iterations);
}

test "solveTOIIterative enforces default hard iteration limit" {
    const swept = makeSweptAABB(0, 0, 0, 1, 1, 1, 10, 0, 0);
    const target = physics.AABB.init(3, 0, 0, 4, 1, 1);

    const toi = solveTOIIterative(swept, target, 1000, 0.0);
    try std.testing.expect(toi.hit);
    try std.testing.expect(!toi.converged);
    try std.testing.expectEqual(DEFAULT_TOI_ITERATION_LIMIT, toi.iterations);
}

test "computeCCDProgressWatchdog aborts on repeated no-progress" {
    const watchdog = computeCCDProgressWatchdog(0.25, 0.2500001, 0.0001, 2, 3, 12, 64);
    try std.testing.expect(watchdog.valid);
    try std.testing.expect(watchdog.no_progress);
    try std.testing.expect(watchdog.abort);
    try std.testing.expectEqual(@as(u32, 3), watchdog.stagnant_iterations);
    try std.testing.expectEqual(@as(u32, 1), watchdog.reason_code);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0000001), watchdog.progress_delta, 0.000001);
}

test "computeCCDProgressWatchdog resets on progress" {
    const watchdog = computeCCDProgressWatchdog(0.25, 0.3, 0.0001, 2, 3, 12, 64);
    try std.testing.expect(watchdog.valid);
    try std.testing.expect(!watchdog.no_progress);
    try std.testing.expect(!watchdog.abort);
    try std.testing.expectEqual(@as(u32, 0), watchdog.stagnant_iterations);
    try std.testing.expectEqual(@as(u32, 0), watchdog.reason_code);
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), watchdog.progress_delta, 0.0001);
}

test "computePolygonCCD reports convex polygon time of impact" {
    const moving = [_]f32{
        0.0, 0.0,
        1.0, 0.0,
        1.0, 1.0,
        0.0, 1.0,
    };
    const target = [_]f32{
        3.0, 0.0,
        4.0, 0.0,
        4.0, 1.0,
        3.0, 1.0,
    };

    const hit = computePolygonCCD(moving[0..], 4, 10.0, 0.0, target[0..], 4);
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), hit.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_y, 0.0001);
}

test "computeBoxCCD reports fractional box time of impact" {
    const hit = computeBoxCCD(
        0.25,
        0.0,
        0.0,
        1.25,
        1.0,
        1.0,
        10.0,
        0.0,
        0.0,
        3.75,
        0.0,
        0.0,
        4.75,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.45), hit.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_z, 0.0001);
}

test "computeBoxCCD rejects separated parallel box sweep" {
    const miss = computeBoxCCD(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        10.0,
        0.0,
        0.0,
        3.0,
        2.0,
        0.0,
        4.0,
        3.0,
        1.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(!miss.hit);
}

test "computeCCDTriggerAABB reports non-blocking trigger window" {
    const trigger = computeCCDTriggerAABB(
        0.25,
        0.0,
        0.0,
        1.25,
        1.0,
        1.0,
        10.0,
        0.0,
        0.0,
        3.75,
        0.0,
        0.0,
        4.75,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        42,
    );
    try std.testing.expect(trigger.valid);
    try std.testing.expect(trigger.triggered);
    try std.testing.expect(trigger.non_blocking);
    try std.testing.expect(!trigger.starts_inside);
    try std.testing.expect(!trigger.ends_inside);
    try std.testing.expectEqual(@as(u32, 42), trigger.trigger_id);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), trigger.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.45), trigger.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), trigger.duration, 0.0001);
}

test "computeCCDTriggerAABB reports initial overlap" {
    const trigger = computeCCDTriggerAABB(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        1.0,
        0.0,
        0.0,
        0.5,
        0.0,
        0.0,
        1.5,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        7,
    );
    try std.testing.expect(trigger.valid);
    try std.testing.expect(trigger.triggered);
    try std.testing.expect(trigger.starts_inside);
    try std.testing.expect(trigger.ends_inside);
    try std.testing.expectEqual(@as(u32, 7), trigger.trigger_id);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), trigger.entry_time, 0.0001);
}

test "computeCCDThinWallPenetration detects discrete endpoint miss" {
    const risk = computeCCDThinWallPenetration(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        10.0,
        0.0,
        0.0,
        5.0,
        0.0,
        0.0,
        5.1,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.25,
    );
    try std.testing.expect(risk.valid);
    try std.testing.expect(risk.risk);
    try std.testing.expect(risk.ccd_required);
    try std.testing.expect(!risk.starts_overlapping);
    try std.testing.expect(!risk.ends_overlapping);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), risk.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.51), risk.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), risk.wall_thickness, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), risk.motion_distance, 0.0001);
}

test "computeCCDThinWallPenetration ignores thick walls" {
    const risk = computeCCDThinWallPenetration(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        10.0,
        0.0,
        0.0,
        5.0,
        0.0,
        0.0,
        5.5,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.25,
    );
    try std.testing.expect(risk.valid);
    try std.testing.expect(!risk.risk);
    try std.testing.expect(!risk.ccd_required);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), risk.wall_thickness, 0.0001);
}

test "computeCCDTunnelSuppression clamps motion before thin wall" {
    const plan = computeCCDTunnelSuppression(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        10.0,
        0.0,
        0.0,
        5.0,
        0.0,
        0.0,
        5.1,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.25,
        0.01,
    );
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.suppress);
    try std.testing.expect(plan.ccd_required);
    try std.testing.expectApproxEqAbs(@as(f32, 0.39), plan.safe_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.61), plan.remaining_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), plan.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), plan.wall_thickness, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.motion_distance, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.9), plan.clamped_motion_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.clamped_motion_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.clamped_motion_z, 0.0001);
}

test "computeCCDTunnelSuppression keeps full motion without risk" {
    const plan = computeCCDTunnelSuppression(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        1.0,
        10.0,
        0.0,
        0.0,
        5.0,
        0.0,
        0.0,
        5.5,
        1.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.25,
        0.01,
    );
    try std.testing.expect(plan.valid);
    try std.testing.expect(!plan.suppress);
    try std.testing.expect(!plan.ccd_required);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), plan.safe_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.remaining_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.clamped_motion_x, 0.0001);
}

test "computeRotatingBoxCCD reports conservative rotating box time of impact" {
    const hit = computeRotatingBoxCCD(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        3.0,
        0.0,
        std.math.pi / 2.0,
        10.0,
        0.0,
        0.0,
        5.0,
        0.0,
        0.0,
        0.5,
        1.0,
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1338), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8662), hit.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);
}

test "computeRotatingBoxCCD rejects separated rotating box sweep" {
    const miss = computeRotatingBoxCCD(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        3.0,
        0.0,
        std.math.pi / 2.0,
        10.0,
        0.0,
        0.0,
        5.0,
        5.0,
        0.0,
        0.5,
        1.0,
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(!miss.hit);
}

test "computeAngularVelocityCCD converts angular velocity into rotating CCD" {
    const hit = computeAngularVelocityCCD(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        3.0,
        0.0,
        std.math.pi / 2.0,
        10.0,
        0.0,
        0.0,
        5.0,
        0.0,
        0.0,
        0.5,
        1.0,
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    );
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1338), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8662), hit.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);
}

test "computeAngularVelocityCCD scales linear velocity by timestep" {
    const hit = computeAngularVelocityCCD(
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        3.0,
        0.0,
        std.math.pi,
        20.0,
        0.0,
        0.0,
        5.0,
        0.0,
        0.0,
        0.5,
        1.0,
        0.5,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.5,
    );
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1338), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8662), hit.exit_time, 0.0001);
}

test "computeConservativeStep limits linear and angular advancement" {
    const step = computeConservativeStep(10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.25, 1.0, 100);
    try std.testing.expect(step.valid);
    try std.testing.expect(!step.limited);
    try std.testing.expectEqual(@as(u32, 40), step.substep_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.025), step.step_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.025), step.linear_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0625), step.angular_fraction, 0.0001);
}

test "computeConservativeStep reports capped substeps" {
    const step = computeConservativeStep(10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.25, 1.0, 8);
    try std.testing.expect(step.valid);
    try std.testing.expect(step.limited);
    try std.testing.expectEqual(@as(u32, 8), step.substep_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.125), step.step_fraction, 0.0001);
}

test "computeCCDPerformancePlan skips low-risk motion" {
    const plan = computeCCDPerformancePlan(0.05, 0.0, 0.0, 0.0, 1.0, 2.0, 128, 32, 16, 64, 0.1);
    try std.testing.expect(plan.valid);
    try std.testing.expect(!plan.use_ccd);
    try std.testing.expect(plan.skip_ccd);
    try std.testing.expect(plan.discrete_ok);
    try std.testing.expectEqual(@as(u32, 32), plan.candidate_limit);
    try std.testing.expectEqual(@as(u32, 16), plan.iteration_limit);
    try std.testing.expectEqual(@as(u32, 512), plan.estimated_pair_work);
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), plan.motion_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.angular_ratio, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeCCDPerformancePlan clamps high-risk work" {
    const plan = computeCCDPerformancePlan(10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 128, 32, 128, 64, 0.1);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.use_ccd);
    try std.testing.expect(!plan.skip_ccd);
    try std.testing.expect(!plan.discrete_ok);
    try std.testing.expectEqual(@as(u32, 32), plan.candidate_limit);
    try std.testing.expectEqual(@as(u32, 64), plan.iteration_limit);
    try std.testing.expectEqual(@as(u32, 2048), plan.estimated_pair_work);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.motion_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), plan.angular_ratio, 0.0001);
    try std.testing.expectEqual(@as(u32, 0), plan.reason_code);
}

test "computeCCDPrecisionPlan tightens high-risk sweep tolerances" {
    const plan = computeCCDPrecisionPlan(10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.001, 0.02, 16, 64);
    try std.testing.expect(plan.valid);
    try std.testing.expectEqual(@as(u32, 3), plan.precision_tier);
    try std.testing.expectEqual(@as(u32, 64), plan.iteration_limit);
    try std.testing.expectEqual(@as(u32, 160), plan.substep_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.000125), plan.tolerance, 0.000001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0025), plan.contact_slop, 0.000001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.00003125), plan.min_progress, 0.000001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.00625), plan.conservative_step_fraction, 0.00001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.motion_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), plan.angular_ratio, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeCCDPrecisionPlan preserves low-risk sweep settings" {
    const plan = computeCCDPrecisionPlan(0.2, 0.0, 0.0, 0.0, 1.0, 2.0, 0.001, 0.02, 16, 64);
    try std.testing.expect(plan.valid);
    try std.testing.expectEqual(@as(u32, 0), plan.precision_tier);
    try std.testing.expectEqual(@as(u32, 16), plan.iteration_limit);
    try std.testing.expectEqual(@as(u32, 1), plan.substep_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.001), plan.tolerance, 0.000001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.02), plan.contact_slop, 0.000001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.00025), plan.min_progress, 0.000001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), plan.conservative_step_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), plan.motion_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.angular_ratio, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeCCDStabilityValidation accepts stable high-risk solve" {
    const validation = computeCCDStabilityValidation(0.2, 0.3, 0.20005, 0.19, 0.20005, 10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.001, 0.02, 16, 64, 256);
    try std.testing.expect(validation.valid);
    try std.testing.expect(validation.stable);
    try std.testing.expect(validation.bracket_valid);
    try std.testing.expect(validation.precision_valid);
    try std.testing.expect(validation.progress_safe);
    try std.testing.expect(validation.substeps_safe);
    try std.testing.expectEqual(@as(u32, 64), validation.iteration_limit);
    try std.testing.expectEqual(@as(u32, 160), validation.substep_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), validation.time_error, 0.000001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.000125), validation.tolerance, 0.000001);
    try std.testing.expectEqual(@as(u32, 0), validation.reason_code);
}

test "computeCCDStabilityValidation rejects solves outside tolerance" {
    const validation = computeCCDStabilityValidation(0.2, 0.3, 0.31, 0.19, 0.31, 10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.001, 0.02, 16, 64, 256);
    try std.testing.expect(validation.valid);
    try std.testing.expect(!validation.stable);
    try std.testing.expect(validation.bracket_valid);
    try std.testing.expect(validation.progress_safe);
    try std.testing.expect(validation.substeps_safe);
    try std.testing.expectApproxEqAbs(@as(f32, 0.01), validation.time_error, 0.00001);
    try std.testing.expectEqual(@as(u32, 2), validation.reason_code);
}

test "computeCCDIslandParallelPlan schedules independent islands" {
    const plan = computeCCDIslandParallelPlan(8, 6, 40, 32, 128, 64, 4, 6, 0);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.parallel_enabled);
    try std.testing.expect(!plan.serial_fallback);
    try std.testing.expectEqual(@as(u32, 6), plan.scheduled_islands);
    try std.testing.expectEqual(@as(u32, 4), plan.worker_count);
    try std.testing.expectEqual(@as(u32, 2), plan.batch_count);
    try std.testing.expectEqual(@as(u32, 32), plan.candidate_limit_per_island);
    try std.testing.expectEqual(@as(u32, 64), plan.iteration_limit);
    try std.testing.expectEqual(@as(u32, 12288), plan.estimated_pair_work);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), plan.islands_per_worker, 0.0001);
    try std.testing.expectEqual(@as(u32, 0), plan.reason_code);
}

test "computeCCDIslandParallelPlan falls back on cross-island pairs" {
    const plan = computeCCDIslandParallelPlan(8, 6, 40, 32, 128, 64, 4, 6, 1);
    try std.testing.expect(plan.valid);
    try std.testing.expect(!plan.parallel_enabled);
    try std.testing.expect(plan.serial_fallback);
    try std.testing.expectEqual(@as(u32, 2), plan.reason_code);
}

test "computeCCDSleepInteraction wakes sleeping body for blocking CCD" {
    const plan = computeCCDSleepInteraction(true, 30, 30, 10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.2, true, false);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.wake_required);
    try std.testing.expect(!plan.sleep_allowed);
    try std.testing.expect(plan.keep_awake);
    try std.testing.expect(plan.ccd_required);
    try std.testing.expect(plan.reset_sleep_tick);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), plan.motion_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), plan.angular_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), plan.time_to_impact, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), plan.sleep_progress, 0.0001);
    try std.testing.expectEqual(@as(u32, 1), plan.reason_code);
}

test "computeCCDSleepInteraction allows mature sleep for trigger-only CCD" {
    const plan = computeCCDSleepInteraction(true, 30, 30, 10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.2, true, true);
    try std.testing.expect(plan.valid);
    try std.testing.expect(!plan.wake_required);
    try std.testing.expect(plan.sleep_allowed);
    try std.testing.expect(!plan.keep_awake);
    try std.testing.expect(plan.ccd_required);
    try std.testing.expect(!plan.reset_sleep_tick);
    try std.testing.expectEqual(@as(u32, 3), plan.reason_code);
}

test "computeCCDSubstepPlan returns next substep window" {
    const plan = computeCCDSubstepPlan(10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.25, 1.0, 100, 2);
    try std.testing.expect(plan.valid);
    try std.testing.expect(!plan.complete);
    try std.testing.expect(!plan.limited);
    try std.testing.expectEqual(@as(u32, 2), plan.substep_index);
    try std.testing.expectEqual(@as(u32, 40), plan.substep_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), plan.start_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.075), plan.end_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.025), plan.step_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.925), plan.remaining_fraction, 0.0001);
}

test "computeCCDSubstepPlan clamps completed advancement" {
    const plan = computeCCDSubstepPlan(10.0, 0.0, 0.0, 2.0, 1.0, 2.0, 0.25, 1.0, 8, 8);
    try std.testing.expect(plan.valid);
    try std.testing.expect(plan.complete);
    try std.testing.expect(plan.limited);
    try std.testing.expectEqual(@as(u32, 8), plan.substep_index);
    try std.testing.expectEqual(@as(u32, 8), plan.substep_count);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), plan.start_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), plan.end_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.step_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.remaining_fraction, 0.0001);
}

test "computePolygonCCD rejects separated parallel polygons" {
    const moving = [_]f32{
        0.0, 0.0,
        1.0, 0.0,
        1.0, 1.0,
        0.0, 1.0,
    };
    const target = [_]f32{
        3.0, 2.0,
        4.0, 2.0,
        4.0, 3.0,
        3.0, 3.0,
    };

    const miss = computePolygonCCD(moving[0..], 4, 10.0, 0.0, target[0..], 4);
    try std.testing.expect(!miss.hit);
}

test "computeSphereCCD reports sphere time of impact" {
    const hit = computeSphereCCD(
        0.0,
        0.0,
        0.0,
        1.0,
        10.0,
        0.0,
        0.0,
        4.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6), hit.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_z, 0.0001);
}

test "computeSphereCCD rejects sphere miss" {
    const miss = computeSphereCCD(
        0.0,
        0.0,
        0.0,
        1.0,
        10.0,
        0.0,
        0.0,
        4.0,
        4.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(!miss.hit);
}

test "computeCapsuleCCD reports side time of impact" {
    const hit = computeCapsuleCCD(
        0.0,
        0.0,
        0.0,
        0.5,
        1.0,
        10.0,
        0.0,
        0.0,
        3.0,
        0.0,
        0.0,
        0.5,
        1.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), hit.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_z, 0.0001);
}

test "computeCapsuleCCD reports cap time of impact" {
    const hit = computeCapsuleCCD(
        0.0,
        0.0,
        0.0,
        0.5,
        1.0,
        0.0,
        10.0,
        0.0,
        0.0,
        4.0,
        0.0,
        0.5,
        1.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), hit.entry_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.7), hit.exit_time, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_z, 0.0001);
}

test "computeCapsuleCCD rejects capsule miss" {
    const miss = computeCapsuleCCD(
        0.0,
        0.0,
        0.0,
        0.5,
        1.0,
        10.0,
        0.0,
        0.0,
        3.0,
        4.0,
        0.0,
        0.5,
        1.0,
        0.0,
        0.0,
        0.0,
    );
    try std.testing.expect(!miss.hit);
}
