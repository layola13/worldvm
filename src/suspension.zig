//! Suspension System - Springs, Dampers, and Ride Dynamics
//!
//! Phase 22: Suspension physics, spring rates, damping, bump/rebound asymmetry
//! Handles: Wheel articulation, anti-roll bars, camber gain, ride frequency

const std = @import("std");

pub const SuspensionState = struct {
    rest_length: f32,
    current_length: f32,
    velocity: f32,
    compression: f32,
    force: f32,
    damper_force: f32,
    spring_force: f32,
    bump_threshold: f32,
    rebound_threshold: f32,
    active: bool,
};

pub const SuspensionConfig = struct {
    spring_rate: f32,
    damping_ratio: f32,
    bump_damping: f32,
    rebound_damping: f32,
    preloaded: f32,
    max_length: f32,
    min_length: f32,
    anti_roll_rate: f32,
};

pub const MAX_WHEELS: usize = 8;

pub const SuspensionSystem = struct {
    suspensions: [MAX_WHEELS]SuspensionState,
    count: u8,
};

var g_suspension_system: SuspensionSystem = undefined;

pub fn init() void {
    g_suspension_system.count = 0;
}

pub fn createSuspension(_: SuspensionConfig) ?*SuspensionState {
    if (g_suspension_system.count >= MAX_WHEELS) return null;
    const idx = g_suspension_system.count;
    g_suspension_system.count += 1;
    const susp = &g_suspension_system.suspensions[idx];

    susp.* = .{
        .rest_length = 0.3,
        .current_length = 0.3,
        .velocity = 0,
        .compression = 0,
        .force = 0,
        .damper_force = 0,
        .spring_force = 0,
        .bump_threshold = 0.1,
        .rebound_threshold = 0.1,
        .active = true,
    };
    return susp;
}

pub fn calculateSpringForce(susp: *const SuspensionState, config: SuspensionConfig) f32 {
    const displacement = susp.rest_length - susp.current_length;
    const force = displacement * config.spring_rate + config.preloaded;
    return force;
}

pub fn calculateDampingForce(susp: *const SuspensionState, config: SuspensionConfig) f32 {
    const damping = if (susp.velocity < 0)
        config.bump_damping
    else
        config.rebound_damping;
    return -susp.velocity * damping;
}

pub fn updateSuspension(susp: *SuspensionState, config: SuspensionConfig, dt: f32) void {
    if (!susp.active) return;

    _ = susp.current_length;
    susp.spring_force = calculateSpringForce(susp, config);
    susp.damper_force = calculateDampingForce(susp, config);
    susp.force = susp.spring_force + susp.damper_force;
    susp.compression = (susp.rest_length - susp.current_length) / susp.rest_length;

    const acceleration = susp.force / 50.0;
    susp.velocity += acceleration * dt;
    susp.current_length += susp.velocity * dt;

    susp.current_length = @max(config.min_length, @min(config.max_length, susp.current_length));

    if (@abs(susp.velocity) < 0.001) {
        susp.velocity = 0;
    }
}

pub fn applyAntiRoll(front_left: *SuspensionState, front_right: *SuspensionState,
                      rear_left: *SuspensionState, rear_right: *SuspensionState,
                      config: SuspensionConfig) void {
    const front_diff = front_left.compression - front_right.compression;
    const rear_diff = rear_left.compression - rear_right.compression;

    const front_roll_force = front_diff * config.anti_roll_rate;
    const rear_roll_force = rear_diff * config.anti_roll_rate;

    front_left.force += front_roll_force;
    front_right.force -= front_roll_force;
    rear_left.force += rear_roll_force;
    rear_right.force -= rear_roll_force;
}

pub fn calculateNaturalFrequency(mass: f32, spring_rate: f32) f32 {
    return @sqrt(spring_rate / mass) / (2.0 * 3.14159);
}

pub fn calculateDampingCoefficient(mass: f32, damping_ratio: f32, natural_freq: f32) f32 {
    return 2.0 * damping_ratio * natural_freq * mass;
}

pub fn checkGroundContact(susp: *const SuspensionState) bool {
    return susp.current_length < susp.rest_length - 0.01;
}

pub fn getSuspensionTravel(susp: *const SuspensionState) f32 {
    return susp.rest_length - susp.current_length;
}

pub fn isBottomedOut(susp: *const SuspensionState, config: SuspensionConfig) bool {
    return susp.current_length <= config.min_length;
}

pub fn isFullyExtended(susp: *const SuspensionState, config: SuspensionConfig) bool {
    return susp.current_length >= config.max_length;
}

pub fn getSystem() *SuspensionSystem {
    return &g_suspension_system;
}
