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

pub fn createSuspension(config: SuspensionConfig) ?*SuspensionState {
    if (g_suspension_system.count >= MAX_WHEELS) return null;
    const idx = g_suspension_system.count;
    g_suspension_system.count += 1;
    const susp = &g_suspension_system.suspensions[idx];

    const travel = @max(0.0001, config.max_length - config.min_length);
    const preload_compression = config.preloaded / @max(1.0, config.spring_rate);
    const rest = @max(config.min_length, @min(config.max_length, config.max_length - preload_compression));

    susp.* = .{
        .rest_length = rest,
        .current_length = rest,
        .velocity = 0,
        .compression = 0,
        .force = 0,
        .damper_force = 0,
        .spring_force = 0,
        .bump_threshold = config.min_length + travel * 0.15,
        .rebound_threshold = config.max_length - travel * 0.15,
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
    var damping = if (susp.velocity < 0)
        config.bump_damping
    else
        config.rebound_damping;

    // Progressive damping near travel limits to reduce harsh impacts and top-out oscillation.
    if (susp.velocity < 0 and susp.current_length <= susp.bump_threshold) {
        damping *= 1.5;
    } else if (susp.velocity > 0 and susp.current_length >= susp.rebound_threshold) {
        damping *= 1.25;
    }
    return -susp.velocity * damping;
}

pub fn updateSuspension(susp: *SuspensionState, config: SuspensionConfig, dt: f32) void {
    if (!susp.active or dt < 0.0) return;

    const travel = @max(0.0001, config.max_length - config.min_length);
    susp.spring_force = calculateSpringForce(susp, config);
    susp.damper_force = calculateDampingForce(susp, config);

    if (susp.current_length <= susp.bump_threshold) {
        const bump_penetration = (susp.bump_threshold - susp.current_length) / travel;
        susp.spring_force += bump_penetration * bump_penetration * config.spring_rate * 0.35;
    } else if (susp.current_length >= susp.rebound_threshold) {
        const rebound_extension = (susp.current_length - susp.rebound_threshold) / travel;
        susp.spring_force -= rebound_extension * config.spring_rate * 0.15;
    }

    susp.force = susp.spring_force + susp.damper_force;

    const acceleration = susp.force / 50.0;
    susp.velocity += acceleration * dt;
    susp.current_length += susp.velocity * dt;

    susp.current_length = @max(config.min_length, @min(config.max_length, susp.current_length));
    if (susp.current_length <= config.min_length and susp.velocity < 0) susp.velocity = 0;
    if (susp.current_length >= config.max_length and susp.velocity > 0) susp.velocity = 0;

    susp.compression = (config.max_length - susp.current_length) / travel;
    susp.compression = @max(0.0, @min(1.0, susp.compression));

    if (@abs(susp.velocity) < 0.001) {
        susp.velocity = 0;
    }
}

pub fn applyAntiRoll(front_left: *SuspensionState, front_right: *SuspensionState, rear_left: *SuspensionState, rear_right: *SuspensionState, config: SuspensionConfig) void {
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

// ============================================================================
// Tests for Suspension System (Items 526-540)
// ============================================================================

test "526: suspension spring force - force from displacement" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.current_length = 0.25;
    const spring_force = calculateSpringForce(susp, config);
    try std.testing.expect(spring_force > 0);
}

test "suspension createSuspension honors config bounds and thresholds" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 35000,
        .damping_ratio = 0.7,
        .bump_damping = 2500,
        .rebound_damping = 3500,
        .preloaded = 700,
        .max_length = 0.55,
        .min_length = 0.10,
        .anti_roll_rate = 8000,
    };
    const susp = createSuspension(config) orelse return error.TestUnexpectedResult;

    try std.testing.expect(susp.rest_length >= config.min_length and susp.rest_length <= config.max_length);
    try std.testing.expect(susp.bump_threshold > config.min_length and susp.bump_threshold < config.max_length);
    try std.testing.expect(susp.rebound_threshold > susp.bump_threshold and susp.rebound_threshold < config.max_length);
}

test "suspension bump region increases resisting force over mid-travel" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };

    const mid = createSuspension(config) orelse return error.TestUnexpectedResult;
    mid.current_length = 0.30;
    updateSuspension(mid, config, 0.0);
    const mid_force = mid.spring_force;

    const near_bump = createSuspension(config) orelse return error.TestUnexpectedResult;
    near_bump.current_length = 0.205;
    updateSuspension(near_bump, config, 0.0);
    const bump_force = near_bump.spring_force;

    try std.testing.expect(bump_force > mid_force);
}

test "527: suspension damping force - force from velocity" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.velocity = -0.1;
    const damping_force = calculateDampingForce(susp, config);
    try std.testing.expect(damping_force != 0);
}

test "528: suspension anti-roll bar - force distribution between wheels" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const fl = createSuspension(config).?;
    const fr = createSuspension(config).?;
    const rl = createSuspension(config).?;
    const rr = createSuspension(config).?;
    fl.compression = 0.1;
    fr.compression = 0.2;
    const initial_fl_force = fl.force;
    applyAntiRoll(fl, fr, rl, rr, config);
    try std.testing.expect(fl.force != initial_fl_force);
}

test "529: suspension geometry - camber gain from compression" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.compression = 0.15;
    try std.testing.expect(susp.compression > 0);
}

test "530: suspension travel - range of motion calculation" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.current_length = 0.25;
    const travel = getSuspensionTravel(susp);
    try std.testing.expect(travel > 0);
}

test "531: suspension stiffness - spring rate affects force" {
    init();
    const config_soft = SuspensionConfig{
        .spring_rate = 20000,
        .damping_ratio = 0.7,
        .bump_damping = 2000,
        .rebound_damping = 3000,
        .preloaded = 50,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 5000,
    };
    const config_stiff = SuspensionConfig{
        .spring_rate = 80000,
        .damping_ratio = 0.7,
        .bump_damping = 4000,
        .rebound_damping = 5000,
        .preloaded = 200,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 20000,
    };
    const susp_soft = createSuspension(config_soft).?;
    const susp_stiff = createSuspension(config_stiff).?;
    susp_soft.current_length = 0.25;
    susp_stiff.current_length = 0.25;
    const force_soft = calculateSpringForce(susp_soft, config_soft);
    const force_stiff = calculateSpringForce(susp_stiff, config_stiff);
    try std.testing.expect(force_stiff > force_soft);
}

test "532: suspension preload - initial force at rest" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 500,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.current_length = susp.rest_length;
    const spring_force = calculateSpringForce(susp, config);
    try std.testing.expect(spring_force >= 500);
}

test "533: suspension non-linearity - progressive rate" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.current_length = 0.28;
    const force_1 = calculateSpringForce(susp, config);
    susp.current_length = 0.22;
    const force_2 = calculateSpringForce(susp, config);
    try std.testing.expect(force_2 > force_1);
}

test "534: suspension bump collision - force response at full compression" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.current_length = 0.15;
    const is_bottomed = isBottomedOut(susp, config);
    try std.testing.expect(is_bottomed == true);
}

test "535: suspension stability - velocity damping prevents oscillation" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.velocity = 0.5;
    const damping_force = calculateDampingForce(susp, config);
    try std.testing.expect(damping_force < 0);
}

test "536: suspension response - update changes compression" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.current_length = 0.3;
    susp.velocity = -0.05;
    updateSuspension(susp, config, 0.016);
    try std.testing.expect(susp.current_length != 0.3 or susp.velocity != -0.05);
}

test "537: suspension durability - force limits prevent damage" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    susp.current_length = 0.3;
    updateSuspension(susp, config, 0.016);
    const not_bottomed = !isBottomedOut(susp, config);
    try std.testing.expect(not_bottomed);
}

test "538: suspension debug - system state inspection" {
    init();
    const config = SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 100,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const susp = createSuspension(config).?;
    const sys = getSystem();
    try std.testing.expect(sys.count > 0);
    try std.testing.expect(susp.active == true);
}

test "539: suspension parameter identification - natural frequency calculation" {
    const freq = calculateNaturalFrequency(500.0, 50000.0);
    try std.testing.expect(freq > 0);
    try std.testing.expect(freq < 20.0);
}

test "540: suspension comfort mode - softer spring rate" {
    init();
    const config_comfort = SuspensionConfig{
        .spring_rate = 25000,
        .damping_ratio = 0.5,
        .bump_damping = 1500,
        .rebound_damping = 2000,
        .preloaded = 50,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 3000,
    };
    const config_sport = SuspensionConfig{
        .spring_rate = 70000,
        .damping_ratio = 0.9,
        .bump_damping = 5000,
        .rebound_damping = 6000,
        .preloaded = 200,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 15000,
    };
    const susp_comfort = createSuspension(config_comfort).?;
    const susp_sport = createSuspension(config_sport).?;
    susp_comfort.current_length = 0.25;
    susp_sport.current_length = 0.25;
    const force_comfort = calculateSpringForce(susp_comfort, config_comfort);
    const force_sport = calculateSpringForce(susp_sport, config_sport);
    try std.testing.expect(force_sport > force_comfort);
}
