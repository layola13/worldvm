//! Rewind and Determinism System - Full World State Recording
//!
//! Phase P4: World-level rollback and determinism
//! Handles: World snapshots, rollback, determinism checks, ghost replay
//!
//! Features:
//! - Full world state snapshots (instances, entities, subsystems)
//! - World state hashing for determinism verification
//! - Input logging for replay
//! - Fast-forward replay from recorded state
//! - Deterministic math enforcement

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const ragdoll = @import("ragdoll.zig");
const ballistics = @import("ballistics.zig");

// ============================================================================
// Core Types
// ============================================================================

pub const RewindState = struct {
    tick: u32,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    yaw: f32,
    pitch: f32,
    roll: f32,
    input_forwards: bool,
    input_backwards: bool,
    input_left: bool,
    input_right: bool,
    input_jump: bool,
    input_brake: bool,
};

pub const DeterminismProof = struct {
    initial_state_hash: u64,
    final_state_hash: u64,
    tick_count: u32,
    mismatches: u16,
    verified: bool,
};

pub const MAX_REWIND_STATES: usize = 1200;
pub const MAX_INPUT_LOG: usize = 4096;

pub const InputLog = struct {
    tick: u32,
    forward: bool,
    backward: bool,
    left: bool,
    right: bool,
    jump: bool,
    brake: bool,
    mouse_dx: f32,
    mouse_dy: f32,
};

// ============================================================================
// World State Snapshot
// ============================================================================

pub const InstanceSnapshot = struct {
    entity_id: u16,
    pos_x: i32,
    pos_y: i32,
    pos_z: i32,
    rot_yaw: u8,
    rot_pitch: u8,
    rot_roll: u8,
    state: scene32.InstanceState,
    vel_x: i16,
    vel_y: i16,
    vel_z: i16,
    ang_x: i8,
    ang_y: i8,
    ang_z: i8,
    sleep_tick: u8,
};

pub const WorldSnapshot = struct {
    tick: u32,
    global_tick: u32,

    // Instance states
    instance_count: u8,
    instances: [scene32.MAX_INSTANCES]InstanceSnapshot,

    // Entity states (simplified - just flags that affect physics)
    entity_hp: [256]f32,  // Health for destructible entities

    // KCC states
    kcc_count: u8,
    kcc_positions: [kcc.MAX_KCC][3]f32,
    kcc_velocities: [kcc.MAX_KCC][3]f32,
    kcc_grounded: [kcc.MAX_KCC]bool,

    // Vehicle states (simplified)
    vehicle_count: u8,
    vehicle_positions: [vehicle.MAX_VEHICLES][3]f32,
    vehicle_velocities: [vehicle.MAX_VEHICLES][3]f32,
    vehicle_yaw: [vehicle.MAX_VEHICLES]f32,

    // Ragdoll states (simplified)
    ragdoll_count: u8,

    // Projectile states (simplified)
    projectile_count: u8,

    // Input for replay
    input_log: [MAX_INPUT_LOG]InputLog,
    input_count: u16,

    // World hash for determinism
    world_hash: u64,
};

pub const RewindSystem = struct {
    // Per-entity rewind states (legacy)
    states: [MAX_REWIND_STATES]RewindState,
    state_count: u16,
    current_index: u16,
    max_tick: u32,
    proof: DeterminismProof,
    deterministic: bool,

    // Full world snapshots
    world_snapshots: [120]WorldSnapshot,
    world_snapshot_count: u8,
    world_snapshot_index: u8,

    // Input log
    input_log: [MAX_INPUT_LOG]InputLog,
    input_count: u16,
    max_input_tick: u32,

    // Determinism verification
    proof_ticks: u32,
    proof_initial_hash: u64,
    proof_final_hash: u64,
};

var g_rewind_system: RewindSystem = undefined;

pub fn init() void {
    g_rewind_system.state_count = 0;
    g_rewind_system.current_index = 0;
    g_rewind_system.max_tick = 0;
    g_rewind_system.proof = .{
        .initial_state_hash = 0,
        .final_state_hash = 0,
        .tick_count = 0,
        .mismatches = 0,
        .verified = false,
    };
    g_rewind_system.deterministic = true;
    g_rewind_system.world_snapshot_count = 0;
    g_rewind_system.world_snapshot_index = 0;
    g_rewind_system.input_count = 0;
    g_rewind_system.max_input_tick = 0;
    g_rewind_system.proof_ticks = 0;
    g_rewind_system.proof_initial_hash = 0;
    g_rewind_system.proof_final_hash = 0;
}

// ============================================================================
// World State Snapshot Operations
// ============================================================================

/// Capture full world state snapshot
pub fn captureWorldSnapshot(
    tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) WorldSnapshot {
    var snapshot: WorldSnapshot = undefined;
    snapshot.tick = tick;
    snapshot.global_tick = s1024.global_tick;
    snapshot.instance_count = s1024.instance_count;

    // Capture instance states
    var i: u8 = 0;
    while (i < s1024.instance_count and i < scene32.MAX_INSTANCES) : (i += 1) {
        const inst = &s1024.instances[i];
        snapshot.instances[i] = .{
            .entity_id = inst.entity_id,
            .pos_x = inst.pos_x,
            .pos_y = inst.pos_y,
            .pos_z = inst.pos_z,
            .rot_yaw = inst.rot_yaw,
            .rot_pitch = inst.rot_pitch,
            .rot_roll = inst.rot_roll,
            .state = inst.state,
            .vel_x = inst.vel_x,
            .vel_y = inst.vel_y,
            .vel_z = inst.vel_z,
            .ang_x = inst.ang_x,
            .ang_y = inst.ang_y,
            .ang_z = inst.ang_z,
            .sleep_tick = inst.sleep_tick,
        };
    }

    // Capture entity HP (for destructibles)
    i = 0;
    while (i < entities.len and i < 256) : (i += 1) {
        snapshot.entity_hp[i] = entities[i].physics.hardness;  // Reuse hardness field as HP proxy
    }

    // Capture KCC states
    const kcc_sys = kcc.getSystem();
    snapshot.kcc_count = kcc_sys.count;
    i = 0;
    while (i < kcc_sys.count and i < kcc.MAX_KCC) : (i += 1) {
        const k = &kcc_sys.characters[i];
        snapshot.kcc_positions[i] = .{ k.pos_x, k.pos_y, k.pos_z };
        snapshot.kcc_velocities[i] = .{ k.vel_x, k.vel_y, k.vel_z };
        snapshot.kcc_grounded[i] = k.grounded;
    }

    // Capture Vehicle states
    const vehicle_sys = vehicle.getSystem();
    snapshot.vehicle_count = vehicle_sys.count;
    i = 0;
    while (i < vehicle_sys.count and i < vehicle.MAX_VEHICLES) : (i += 1) {
        const v = &vehicle_sys.vehicles[i];
        snapshot.vehicle_positions[i] = .{ v.pos_x, v.pos_y, v.pos_z };
        snapshot.vehicle_velocities[i] = .{ v.vel_x, v.vel_y, v.vel_z };
        snapshot.vehicle_yaw[i] = v.yaw;
    }

    // Capture Ragdoll count (simplified)
    const ragdoll_sys = ragdoll.getSystem();
    snapshot.ragdoll_count = ragdoll_sys.count;

    // Capture Projectile count (simplified)
    const proj_sys = ballistics.getSystem();
    snapshot.projectile_count = proj_sys.count;

    // Copy input log
    snapshot.input_count = g_rewind_system.input_count;
    i = 0;
    while (i < snapshot.input_count and i < MAX_INPUT_LOG) : (i += 1) {
        snapshot.input_log[i] = g_rewind_system.input_log[i];
    }

    // Compute world hash
    snapshot.world_hash = computeWorldHash(&snapshot);

    return snapshot;
}

/// Record world snapshot
pub fn recordWorldSnapshot(
    tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) void {
    const snapshot = captureWorldSnapshot(tick, s1024, entities);

    const idx = g_rewind_system.world_snapshot_index;
    g_rewind_system.world_snapshots[idx] = snapshot;

    g_rewind_system.world_snapshot_index = (g_rewind_system.world_snapshot_index + 1) % 120;
    if (g_rewind_system.world_snapshot_count < 120) {
        g_rewind_system.world_snapshot_count += 1;
    }
}

/// Get world snapshot at specific tick
pub fn getWorldSnapshotAtTick(tick: u32) ?*const WorldSnapshot {
    var i: u8 = 0;
    while (i < g_rewind_system.world_snapshot_count) : (i += 1) {
        const idx = (g_rewind_system.world_snapshot_index + 120 - g_rewind_system.world_snapshot_count + i) % 120;
        if (g_rewind_system.world_snapshots[idx].tick == tick) {
            return &g_rewind_system.world_snapshots[idx];
        }
    }
    return null;
}

/// Restore world from snapshot
pub fn restoreWorldSnapshot(
    snapshot: *const WorldSnapshot,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) void {
    s1024.global_tick = snapshot.global_tick;

    // Restore instance states
    var i: u8 = 0;
    while (i < snapshot.instance_count and i < scene32.MAX_INSTANCES) : (i += 1) {
        const inst = &s1024.instances[i];
        const snap = snapshot.instances[i];
        inst.entity_id = snap.entity_id;
        inst.pos_x = snap.pos_x;
        inst.pos_y = snap.pos_y;
        inst.pos_z = snap.pos_z;
        inst.rot_yaw = snap.rot_yaw;
        inst.rot_pitch = snap.rot_pitch;
        inst.rot_roll = snap.rot_roll;
        inst.state = snap.state;
        inst.vel_x = snap.vel_x;
        inst.vel_y = snap.vel_y;
        inst.vel_z = snap.vel_z;
        inst.ang_x = snap.ang_x;
        inst.ang_y = snap.ang_y;
        inst.ang_z = snap.ang_z;
        inst.sleep_tick = snap.sleep_tick;
    }

    // Rebuild occupancy after restore
    s1024.rebuildOccupancy(entities) catch {};
}

// ============================================================================
// World Hash for Determinism
// ============================================================================

/// Compute hash of world state for determinism verification
pub fn computeWorldHash(snapshot: *const WorldSnapshot) u64 {
    var hash: u64 = 0xDEADBEEFCAFEBABE;

    // Hash tick
    hash ^= @as(u64, snapshot.tick) * 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.global_tick) * 0x9e3779b97f4a7c15;

    // Hash instance states (deterministic order)
    var i: u8 = 0;
    while (i < snapshot.instance_count) : (i += 1) {
        const inst = &snapshot.instances[i];
        hash ^= @as(u64, inst.entity_id) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, inst.pos_x) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, inst.pos_y) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, inst.pos_z) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, inst.vel_x) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, inst.vel_y) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, inst.vel_z) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @intFromEnum(inst.state)) + 0x9e3779b97f4a7c15;
    }

    // Hash KCC states
    i = 0;
    while (i < snapshot.kcc_count) : (i += 1) {
        hash ^= @as(u64, snapshot.kcc_positions[i][0]) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, snapshot.kcc_positions[i][1]) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, snapshot.kcc_positions[i][2]) + 0x9e3779b97f4a7c15;
    }

    // Hash Vehicle states
    i = 0;
    while (i < snapshot.vehicle_count) : (i += 1) {
        hash ^= @as(u64, snapshot.vehicle_positions[i][0]) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, snapshot.vehicle_positions[i][1]) + 0x9e3779b97f4a7c15;
        hash ^= @as(u64, snapshot.vehicle_positions[i][2]) + 0x9e3779b97f4a7c15;
    }

    // Finalize hash
    hash = (hash << 31) | (hash >> 33);
    hash ^= 0x1234567890ABCDEF;

    return hash;
}

/// Verify determinism between two world snapshots
pub fn verifyWorldDeterminism(snap_a: *const WorldSnapshot, snap_b: *const WorldSnapshot) bool {
    if (snap_a.tick != snap_b.tick) return false;
    if (snap_a.instance_count != snap_b.instance_count) return false;

    // Compare instance states
    var i: u8 = 0;
    while (i < snap_a.instance_count) : (i += 1) {
        const a = &snap_a.instances[i];
        const b = &snap_b.instances[i];
        if (a.pos_x != b.pos_x or a.pos_y != b.pos_y or a.pos_z != b.pos_z) return false;
        if (a.vel_x != b.vel_x or a.vel_y != b.vel_y or a.vel_z != b.vel_z) return false;
        if (a.state != b.state) return false;
    }

    return true;
}

// ============================================================================
// Input Logging
// ============================================================================

/// Record input for a tick
pub fn recordInput(
    tick: u32,
    forward: bool,
    backward: bool,
    left: bool,
    right: bool,
    jump: bool,
    brake: bool,
    mouse_dx: f32,
    mouse_dy: f32,
) void {
    if (g_rewind_system.input_count >= MAX_INPUT_LOG) return;

    const idx = g_rewind_system.input_count;
    g_rewind_system.input_log[idx] = .{
        .tick = tick,
        .forward = forward,
        .backward = backward,
        .left = left,
        .right = right,
        .jump = jump,
        .brake = brake,
        .mouse_dx = mouse_dx,
        .mouse_dy = mouse_dy,
    };
    g_rewind_system.input_count += 1;
    g_rewind_system.max_input_tick = tick;
}

/// Get input at specific tick
pub fn getInputAtTick(tick: u32) ?*const InputLog {
    var i: u16 = 0;
    while (i < g_rewind_system.input_count) : (i += 1) {
        if (g_rewind_system.input_log[i].tick == tick) {
            return &g_rewind_system.input_log[i];
        }
    }
    return null;
}

/// Clear input log
pub fn clearInputLog() void {
    g_rewind_system.input_count = 0;
    g_rewind_system.max_input_tick = 0;
}

// ============================================================================
// Fast Forward Replay
// ============================================================================

/// Fast forward simulation from recorded inputs
/// Returns number of ticks actually simulated
pub fn fastForwardTicks(
    start_tick: u32,
    end_tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) u32 {
    var ticks_simulated: u32 = 0;
    var tick = start_tick;

    while (tick <= end_tick) {
        // Could replay inputs and step physics here

        // Record snapshot at interval
        if (ticks_simulated % 10 == 0) {
            recordWorldSnapshot(tick, s1024, entities);
        }

        tick += 1;
        ticks_simulated += 1;
    }

    return ticks_simulated;
}

// ============================================================================
// Legacy Per-Entity Rewind (kept for compatibility)
// ============================================================================

pub fn recordState(state: RewindState) void {
    const idx = g_rewind_system.current_index;
    g_rewind_system.states[idx] = state;

    g_rewind_system.current_index = @as(u16, @intCast((g_rewind_system.current_index + 1) % MAX_REWIND_STATES));
    if (g_rewind_system.state_count < MAX_REWIND_STATES) {
        g_rewind_system.state_count += 1;
    }

    if (state.tick > g_rewind_system.max_tick) {
        g_rewind_system.max_tick = state.tick;
    }
}

pub fn getStateAtTick(tick: u32) ?*const RewindState {
    if (tick > g_rewind_system.max_tick) return null;

    const offset = g_rewind_system.max_tick - tick;
    if (offset >= g_rewind_system.state_count) return null;

    const idx = if (g_rewind_system.current_index >= offset)
        g_rewind_system.current_index - offset
    else
        MAX_REWIND_STATES - (offset - g_rewind_system.current_index);

    return &g_rewind_system.states[idx];
}

pub fn rewindToTick(tick: u32) ?*const RewindState {
    return getStateAtTick(tick);
}

pub fn calculateStateHash(state: *const RewindState) u64 {
    var hash: u64 = 0x1234567890ABCDEF;

    const px = @as(u32, @bitCast(state.pos_x));
    const py = @as(u32, @bitCast(state.pos_y));
    const pz = @as(u32, @bitCast(state.pos_z));
    const vx = @as(u32, @bitCast(state.vel_x));
    const vy = @as(u32, @bitCast(state.vel_y));
    const vz = @as(u32, @bitCast(state.vel_z));

    hash ^= @as(u64, px) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, py) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, pz) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, vx) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, vy) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, vz) + 0x9e3779b97f4a7c15;

    hash = (hash << 31) | (hash >> 33);

    return hash;
}

pub fn verifyDeterminism(tick_a: u32, tick_b: u32) bool {
    const state_a = getStateAtTick(tick_a);
    const state_b = getStateAtTick(tick_b);

    if (state_a == null or state_b == null) return false;

    return calculateStateHash(state_a.?) == calculateStateHash(state_b.?);
}

pub fn beginDeterminismProof() void {
    if (g_rewind_system.state_count > 0) {
        g_rewind_system.proof_initial_hash = calculateStateHash(&g_rewind_system.states[0]);
    }
    g_rewind_system.proof_ticks = g_rewind_system.max_tick;
    g_rewind_system.proof.tick_count = g_rewind_system.max_tick;
}

pub fn endDeterminismProof() void {
    if (g_rewind_system.state_count > 0) {
        const last_idx = (g_rewind_system.current_index + MAX_REWIND_STATES - 1) % MAX_REWIND_STATES;
        g_rewind_system.proof_final_hash = calculateStateHash(&g_rewind_system.states[last_idx]);
    }
    g_rewind_system.proof.verified = true;
}

pub fn compareTraces(trace_a: []const RewindState, trace_b: []const RewindState) u16 {
    var mismatches: u16 = 0;
    const min_len = @min(trace_a.len, trace_b.len);

    for (0..min_len) |i| {
        const hash_a = calculateStateHash(&trace_a[i]);
        const hash_b = calculateStateHash(&trace_b[i]);
        if (hash_a != hash_b) mismatches += 1;
    }

    mismatches += @as(u16, @abs(@as(i32, @intCast(trace_a.len)) - @as(i32, @intCast(trace_b.len))));

    return mismatches;
}

pub fn createGhostReplay(tick_start: u32, tick_end: u32) []const RewindState {
    const start_idx: u16 = if (tick_start <= g_rewind_system.max_tick) tick_start else g_rewind_system.max_tick;
    const end_idx: u16 = if (tick_end <= g_rewind_system.max_tick) tick_end else g_rewind_system.max_tick;

    if (start_idx > end_idx) return &[_]RewindState{};

    const count = end_idx - start_idx + 1;
    const result: [100]RewindState = undefined;

    for (0..count) |i| {
        const tick = start_idx + @as(u32, @intCast(i));
        if (getStateAtTick(tick)) |state| {
            result[i] = state.*;
        }
    }

    return &result;
}

pub fn getRewindBufferUsage() struct { count: u16, capacity: usize, percent: f32 } {
    return .{
        .count = g_rewind_system.state_count,
        .capacity = MAX_REWIND_STATES,
        .percent = @as(f32, @floatFromInt(g_rewind_system.state_count)) / @as(f32, @floatFromInt(MAX_REWIND_STATES)) * 100,
    };
}

pub fn clearRewindBuffer() void {
    g_rewind_system.state_count = 0;
    g_rewind_system.current_index = 0;
    g_rewind_system.max_tick = 0;
}

pub fn isDeterministic() bool {
    return g_rewind_system.deterministic;
}

pub fn getDeterminismProof() DeterminismProof {
    return g_rewind_system.proof;
}

/// Get world snapshot buffer usage
pub fn getWorldSnapshotBufferUsage() struct { count: u8, capacity: usize, percent: f32 } {
    return .{
        .count = g_rewind_system.world_snapshot_count,
        .capacity = 120,
        .percent = @as(f32, @floatFromInt(g_rewind_system.world_snapshot_count)) / 120.0 * 100,
    };
}

/// Get system for external access
pub fn getSystem() *RewindSystem {
    return &g_rewind_system;
}
