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

    const vel_magnitude = @sqrt(
        @as(f32, @floatFromInt(inst.vel_x * inst.vel_x)) +
        @as(f32, @floatFromInt(inst.vel_y * inst.vel_y)) +
        @as(f32, @floatFromInt(inst.vel_z * inst.vel_z))
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

    if (@abs(inst.vel_x) > @as(i16, @intFromFloat(g_defense_system.config.velocity_cap))) {
        inst.vel_x = if (inst.vel_x > 0) @as(i16, @intFromFloat(g_defense_system.config.velocity_cap)) else @as(i16, @intFromFloat(-g_defense_system.config.velocity_cap));
    }
    if (@abs(inst.vel_y) > @as(i16, @intFromFloat(g_defense_system.config.velocity_cap))) {
        inst.vel_y = if (inst.vel_y > 0) @as(i16, @intFromFloat(g_defense_system.config.velocity_cap)) else @as(i16, @intFromFloat(-g_defense_system.config.velocity_cap));
    }
    if (@abs(inst.vel_z) > @as(i16, @intFromFloat(g_defense_system.config.velocity_cap))) {
        inst.vel_z = if (inst.vel_z > 0) @as(i16, @intFromFloat(g_defense_system.config.velocity_cap)) else @as(i16, @intFromFloat(-g_defense_system.config.velocity_cap));
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
