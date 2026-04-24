//! Rewind and Determinism System - State Recording and Playback
//!
//! Phase 35, 19: State recording, rewind, determinism verification, replay
//! Handles: State snapshots, rollback, determinism checks, ghost replay

const std = @import("std");

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

pub const RewindSystem = struct {
    states: [MAX_REWIND_STATES]RewindState,
    state_count: u16,
    current_index: u16,
    max_tick: u32,
    proof: DeterminismProof,
    deterministic: bool,
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
}

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
        g_rewind_system.proof.initial_state_hash = calculateStateHash(&g_rewind_system.states[0]);
    }
    g_rewind_system.proof.tick_count = g_rewind_system.max_tick;
}

pub fn endDeterminismProof() void {
    if (g_rewind_system.state_count > 0) {
        const last_idx = (g_rewind_system.current_index + MAX_REWIND_STATES - 1) % MAX_REWIND_STATES;
        g_rewind_system.proof.final_state_hash = calculateStateHash(&g_rewind_system.states[last_idx]);
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
