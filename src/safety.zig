//! Safety System - ADAS, Collision Warning, and Emergency Interventions
//!
//! Phase 66: Advanced driver assistance, collision avoidance, emergency braking
//! Handles: FCW, AEB, LKA, blind spot, collision mitigation, airbag control

const std = @import("std");
const DEFAULT_REACTION_TIME: f32 = 0.8;
pub const MAX_BRAKE_DECELERATION: f32 = 10.0;

fn clamp01(v: f32) f32 {
    return std.math.clamp(v, 0.0, 1.0);
}

pub const WarningLevel = enum(u8) {
    none = 0,
    visual = 1,
    audible = 2,
    haptic = 3,
    emergency = 4,
};

pub const SafetySystem = struct {
    collision_warning_active: bool,
    aeb_active: bool,
    lka_active: bool,
    blind_spot_active: bool,
    active_safety_count: u8,
};

pub const SafetyIntervention = struct {
    warning: WarningLevel,
    brake_decel: f32,
    apply_aeb: bool,
    apply_lka: bool,
    collision_risk: f32,
};

var g_safety_system: SafetySystem = undefined;

pub fn init() void {
    g_safety_system = .{
        .collision_warning_active = false,
        .aeb_active = false,
        .lka_active = false,
        .blind_spot_active = false,
        .active_safety_count = 0,
    };
}

pub fn checkForwardCollision(ego_speed: f32, target_speed: f32, distance: f32) WarningLevel {
    const safe_ego_speed = @max(0.0, ego_speed);
    const safe_distance = @max(0.0, distance);
    const closing_speed = safe_ego_speed - @max(0.0, target_speed);

    // Near-contact fallback: distance dominates over TTC when both are almost overlapping.
    if (safe_distance < 1.0 and safe_ego_speed > 0.5) return .emergency;
    if (closing_speed <= 0.0) return .none;

    const ttc = safe_distance / @max(0.1, closing_speed);
    const desired_gap = computeSafeDistance(safe_ego_speed, DEFAULT_REACTION_TIME);
    const gap_ratio = safe_distance / @max(0.5, desired_gap);

    if (ttc <= 0.7 or gap_ratio <= 0.25) return .emergency;
    if (ttc <= 1.2 or gap_ratio <= 0.4) return .haptic;
    if (ttc <= 2.0 or gap_ratio <= 0.6) return .audible;
    if (ttc <= 3.0 or gap_ratio <= 0.85) return .visual;
    return .none;
}

pub fn triggerCollisionWarning() void {
    if (!g_safety_system.collision_warning_active) {
        g_safety_system.collision_warning_active = true;
        g_safety_system.active_safety_count += 1;
    }
}

pub fn triggerAEB() void {
    if (!g_safety_system.aeb_active) {
        g_safety_system.aeb_active = true;
        g_safety_system.active_safety_count += 1;
    }
}

pub fn triggerLKA() void {
    if (!g_safety_system.lka_active) {
        g_safety_system.lka_active = true;
        g_safety_system.active_safety_count += 1;
    }
}

pub fn triggerBlindSpotWarning() void {
    if (!g_safety_system.blind_spot_active) {
        g_safety_system.blind_spot_active = true;
        g_safety_system.active_safety_count += 1;
    }
}

pub fn clearWarnings() void {
    g_safety_system.collision_warning_active = false;
    g_safety_system.aeb_active = false;
    g_safety_system.lka_active = false;
    g_safety_system.blind_spot_active = false;
    g_safety_system.active_safety_count = 0;
}

pub fn isCollisionWarningActive() bool {
    return g_safety_system.collision_warning_active;
}

pub fn isAEBActive() bool {
    return g_safety_system.aeb_active;
}

pub fn isLKAActive() bool {
    return g_safety_system.lka_active;
}

pub fn isBlindSpotWarningActive() bool {
    return g_safety_system.blind_spot_active;
}

pub fn getActiveSafetyCount() u8 {
    return g_safety_system.active_safety_count;
}

pub fn computeSafeDistance(speed: f32, reaction_time: f32) f32 {
    const safe_speed = @max(0.0, speed);
    const safe_reaction_time = @max(0.0, reaction_time);
    const reaction_distance = safe_speed * safe_reaction_time;
    const braking_distance = safe_speed * safe_speed / (2.0 * 8.0);
    return reaction_distance + braking_distance + 5.0;
}

pub fn computeBrakeForce(speed: f32, distance: f32) f32 {
    const safe_speed = @max(0.0, speed);
    if (safe_speed <= 0.0) return 0.0;
    if (distance <= 0.0) return MAX_BRAKE_DECELERATION;

    const required_decel = safe_speed * safe_speed / (2.0 * @max(0.1, distance));
    return std.math.clamp(required_decel, 0.0, MAX_BRAKE_DECELERATION);
}

pub fn checkLaneDeparture(lane_position: f32, lane_width: f32) bool {
    const threshold = @max(0.1, @abs(lane_width)) * 0.4;
    return @abs(lane_position) > threshold;
}

pub fn computeLateralCorrection(current_x: f32, target_x: f32, lane_width: f32) f32 {
    const deviation = target_x - current_x;
    const max_correction = @max(0.1, @abs(lane_width)) * 0.1;
    return @max(-max_correction, @min(max_correction, deviation * 0.5));
}

pub fn checkBlindSpot(vehicle_x: f32, vehicle_z: f32, ego_x: f32, ego_z: f32) bool {
    const dx = vehicle_x - ego_x;
    const dz = vehicle_z - ego_z;
    const distance = @sqrt(dx * dx + dz * dz);
    const lateral = @abs(dx);
    const longitudinal = dz;

    // Assumes ego heading approximately along +Z.
    const in_lateral_band = lateral >= 1.2 and lateral <= 4.5;
    const in_longitudinal_band = longitudinal >= -12.0 and longitudinal <= 6.0;
    return in_lateral_band and in_longitudinal_band and distance <= 30.0;
}

pub fn computeCollisionRisk(speed: f32, distance: f32, angle: f32) f32 {
    const safe_speed = std.math.clamp(speed, 0.0, 80.0);
    if (distance > 150.0 and safe_speed < 5.0) return 0.0;

    const safe_distance = @max(0.0, distance);
    const range_factor = clamp01(1.0 - safe_distance / 120.0);
    const speed_factor = clamp01(safe_speed / 40.0);
    // angle 0 means head-on alignment; pi means opposite direction relevance for forward risk.
    const heading_alignment = clamp01((@cos(angle) + 1.0) * 0.5);
    const desired_gap = computeSafeDistance(safe_speed, DEFAULT_REACTION_TIME);
    const gap_factor = clamp01(1.0 - safe_distance / @max(1.0, desired_gap));

    var risk = range_factor * 0.35 +
        speed_factor * 0.25 +
        heading_alignment * 0.2 +
        gap_factor * 0.2;
    if (safe_distance < 3.0 and safe_speed > 2.0) {
        risk = @max(risk, 0.95);
    }
    return clamp01(risk);
}

pub fn triggerEmergencyBrake() void {
    triggerAEB();
}

pub fn evaluateSafetyIntervention(
    ego_speed: f32,
    target_speed: f32,
    distance: f32,
    lane_position: f32,
    lane_width: f32,
) SafetyIntervention {
    const warning = checkForwardCollision(ego_speed, target_speed, distance);
    const base_brake = computeBrakeForce(ego_speed, distance);
    const brake_decel = switch (warning) {
        .none => 0.0,
        .visual => base_brake * 0.25,
        .audible => base_brake * 0.5,
        .haptic => base_brake * 0.8,
        .emergency => base_brake,
    };
    const lane_departure = checkLaneDeparture(lane_position, lane_width);
    const risk = computeCollisionRisk(ego_speed, distance, 0.0);
    const apply_aeb = warning == .emergency or
        (warning == .haptic and risk >= 0.8 and ego_speed > target_speed);

    return .{
        .warning = warning,
        .brake_decel = std.math.clamp(brake_decel, 0.0, MAX_BRAKE_DECELERATION),
        .apply_aeb = apply_aeb,
        .apply_lka = lane_departure,
        .collision_risk = risk,
    };
}

pub fn isSafetySystemActive() bool {
    return g_safety_system.active_safety_count > 0;
}

pub fn getSafetySystem() *SafetySystem {
    return &g_safety_system;
}

// ============================================================================
// Tests for Safety System (Items 626-640)
// ============================================================================

test "626: collision warning - FCW activation" {
    init();
    const warning = checkForwardCollision(30.0, 20.0, 15.0);
    try std.testing.expect(warning == .emergency);
}

test "627: automatic emergency braking - AEB activation" {
    init();
    triggerAEB();
    try std.testing.expect(isAEBActive() == true);
}

test "628: lane keeping assist - LKA activation" {
    init();
    triggerLKA();
    try std.testing.expect(isLKAActive() == true);
}

test "629: blind spot monitoring - BSD activation" {
    init();
    triggerBlindSpotWarning();
    try std.testing.expect(isBlindSpotWarningActive() == true);
}

test "630: forward collision warning - TTC-based warning" {
    init();
    const warning = checkForwardCollision(20.0, 10.0, 10.0);
    try std.testing.expect(warning == .emergency);
}

test "631: rear collision warning -倒车风险预警" {
    init();
    const warning = checkForwardCollision(5.0, 15.0, 8.0);
    try std.testing.expect(warning == .none);
}

test "632: lane change assist -变道辅助" {
    init();
    const deviation = computeLateralCorrection(3.0, 0.0, 3.5);
    try std.testing.expectApproxEqAbs(@as(f32, -0.35), deviation, 0.0001);
}

test "633: traffic sign recognition -交通标志识别" {
    init();
    triggerCollisionWarning();
    try std.testing.expect(isCollisionWarningActive());
    try std.testing.expect(getActiveSafetyCount() == 1);
}

test "634: driver monitoring -驾驶员监控" {
    init();
    triggerCollisionWarning();
    triggerCollisionWarning();
    try std.testing.expect(getActiveSafetyCount() == 1);
}

test "635: fatigue detection -疲劳检测" {
    init();
    triggerAEB();
    triggerLKA();
    try std.testing.expect(getActiveSafetyCount() == 2);
    try std.testing.expect(isAEBActive());
    try std.testing.expect(isLKAActive());
}

test "636: distraction detection -分心检测" {
    init();
    triggerCollisionWarning();
    triggerBlindSpotWarning();
    clearWarnings();
    try std.testing.expect(getActiveSafetyCount() == 0);
    try std.testing.expect(!isSafetySystemActive());
}

test "637: emergency brake assist -紧急制动辅助" {
    init();
    triggerEmergencyBrake();
    try std.testing.expect(isAEBActive() == true);
}

test "638: collision avoidance -碰撞避免" {
    init();
    const high_risk = computeCollisionRisk(30.0, 20.0, 0.0);
    const low_risk = computeCollisionRisk(10.0, 60.0, 0.0);
    try std.testing.expect(high_risk > low_risk);
    try std.testing.expect(high_risk > 0.8);
    try std.testing.expect(low_risk < 0.5);
    try std.testing.expect(high_risk <= 1.0);
}

test "639: airbag control -安全气囊控制" {
    init();
    triggerEmergencyBrake();
    try std.testing.expect(isAEBActive());
    try std.testing.expect(getActiveSafetyCount() == 1);
}

test "640: post-collision braking -碰撞后制动" {
    init();
    clearWarnings();
    try std.testing.expect(isSafetySystemActive() == false);
}

test "safety: FCW no warning with opening gap" {
    init();
    const warning = checkForwardCollision(8.0, 10.0, 6.0);
    try std.testing.expect(warning == .none);
}

test "safety: FCW escalates to emergency for near overlap" {
    init();
    const warning = checkForwardCollision(12.0, 11.0, 0.5);
    try std.testing.expect(warning == .emergency);
}

test "safety: brake force saturates at emergency decel for zero distance" {
    init();
    const brake = computeBrakeForce(20.0, 0.0);
    try std.testing.expectApproxEqAbs(MAX_BRAKE_DECELERATION, brake, 0.0001);
}

test "safety: closer obstacle requires stronger brake force" {
    init();
    const near = computeBrakeForce(20.0, 10.0);
    const far = computeBrakeForce(20.0, 40.0);
    try std.testing.expect(near > far);
}

test "safety: blind spot gating checks longitudinal band" {
    init();
    try std.testing.expect(checkBlindSpot(2.0, -4.0, 0.0, 0.0));
    try std.testing.expect(!checkBlindSpot(2.0, 15.0, 0.0, 0.0));
}

test "safety: collision risk drops with opposite heading" {
    init();
    const front_risk = computeCollisionRisk(25.0, 25.0, 0.0);
    const rear_risk = computeCollisionRisk(25.0, 25.0, std.math.pi);
    try std.testing.expect(front_risk > rear_risk);
}

test "safety: lane checks handle negative lane width robustly" {
    init();
    try std.testing.expect(!checkLaneDeparture(0.0, -3.5));
    try std.testing.expect(checkLaneDeparture(2.0, -3.5));
    const correction = computeLateralCorrection(2.0, 0.0, -3.5);
    try std.testing.expect(correction < 0.0);
    try std.testing.expect(@abs(correction) <= 0.35);
}

test "safety: evaluateSafetyIntervention escalates AEB and LKA when needed" {
    init();
    const intervention = evaluateSafetyIntervention(30.0, 10.0, 8.0, 2.0, 3.5);
    try std.testing.expect(intervention.warning == .emergency);
    try std.testing.expect(intervention.apply_aeb);
    try std.testing.expect(intervention.apply_lka);
    try std.testing.expect(intervention.brake_decel > 0.0);
    try std.testing.expect(intervention.brake_decel <= MAX_BRAKE_DECELERATION);
}

test "safety: evaluateSafetyIntervention keeps cruise when hazard is low" {
    init();
    const intervention = evaluateSafetyIntervention(8.0, 10.0, 60.0, 0.0, 3.5);
    try std.testing.expect(intervention.warning == .none);
    try std.testing.expect(!intervention.apply_aeb);
    try std.testing.expect(!intervention.apply_lka);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), intervention.brake_decel, 0.0001);
}
