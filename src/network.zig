//! Network - Deterministic Lockstep with Rollback
//!
//! Phase 12: Network synchronization, prediction, rollback, CRC validation
//! Handles: State replication, lag compensation, determinism verification

const std = @import("std");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");

pub const ReplicaState = struct {
    entity_id: u16,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    yaw: u8,
    tick: u32,
    timestamp: u32,
    checksum: u32,
};

pub const InputState = struct {
    tick: u32,
    forward: bool,
    backward: bool,
    left: bool,
    right: bool,
    jump: bool,
    crouch: bool,
    fire: bool,
    aim_x: f32,
    aim_y: f32,
};

pub const SyncConfig = struct {
    send_rate_hz: u16 = 20,
    timeout_ms: u32 = 500,
    max_rollback_ticks: u8 = 10,
    crc_check_enabled: bool = true,
    prediction_window: u8 = 5,
};

pub const MAX_REPLICAS: usize = 64;
pub const MAX_INPUTS: usize = 128;

pub const NetworkSystem = struct {
    replicas: [MAX_REPLICAS]ReplicaState,
    replica_count: u8,
    inputs: [MAX_INPUTS]InputState,
    input_count: u16,
    local_tick: u32,
    remote_tick: u32,
    config: SyncConfig,
    last_sync_tick: u32,
    crc_errors: u16,
};

var g_network_system: NetworkSystem = undefined;

pub fn init(config: SyncConfig) void {
    g_network_system.replica_count = 0;
    g_network_system.input_count = 0;
    g_network_system.local_tick = 0;
    g_network_system.remote_tick = 0;
    g_network_system.config = config;
    g_network_system.last_sync_tick = 0;
    g_network_system.crc_errors = 0;
}

/// Calculate CRC32 checksum for state
pub fn calculateCRC(state: *const ReplicaState) u32 {
    var crc: u32 = 0xFFFFFFFF;

    crc = crc32Step(crc, @as(u32, state.entity_id));
    crc = crc32Step(crc, @as(u32, @bitCast(state.pos_x)));
    crc = crc32Step(crc, @as(u32, @bitCast(state.pos_y)));
    crc = crc32Step(crc, @as(u32, @bitCast(state.pos_z)));
    crc = crc32Step(crc, @as(u32, @bitCast(state.vel_x)));
    crc = crc32Step(crc, @as(u32, @bitCast(state.vel_y)));
    crc = crc32Step(crc, @as(u32, @bitCast(state.vel_z)));
    crc = crc32Step(crc, @as(u32, state.yaw));
    crc = crc32Step(crc, state.tick);

    return ~crc;
}

fn crc32Step(crc: u32, value: u32) u32 {
    var v = crc ^ value;
    var i: u8 = 0;
    while (i < 32) : (i += 8) {
        v = (v >> 8) ^ 0xEDB88320;
    }
    return v;
}

/// Create replica for entity
pub fn createReplica(entity_id: u16) ?*ReplicaState {
    if (g_network_system.replica_count >= MAX_REPLICAS) return null;
    const idx = g_network_system.replica_count;
    g_network_system.replica_count += 1;
    const replica = &g_network_system.replicas[idx];
    replica.* = .{
        .entity_id = entity_id,
        .pos_x = 0, .pos_y = 0, .pos_z = 0,
        .vel_x = 0, .vel_y = 0, .vel_z = 0,
        .yaw = 0, .tick = 0, .timestamp = 0, .checksum = 0,
    };
    return replica;
}

/// Update replica state
pub fn updateReplica(
    replica: *ReplicaState,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    yaw: u8,
    tick: u32,
) void {
    replica.pos_x = pos_x;
    replica.pos_y = pos_y;
    replica.pos_z = pos_z;
    replica.vel_x = vel_x;
    replica.vel_y = vel_y;
    replica.vel_z = vel_z;
    replica.yaw = yaw;
    replica.tick = tick;
    replica.checksum = calculateCRC(replica);
}

/// Store input
pub fn storeInput(input: InputState) void {
    if (g_network_system.input_count >= MAX_INPUTS) {
        for (0..MAX_INPUTS - 1) |i| {
            g_network_system.inputs[i] = g_network_system.inputs[i + 1];
        }
        g_network_system.input_count = MAX_INPUTS - 1;
    }
    g_network_system.inputs[g_network_system.input_count] = input;
    g_network_system.input_count += 1;
}

/// Get input for specific tick
pub fn getInput(tick: u32) ?*const InputState {
    for (g_network_system.inputs[0..g_network_system.input_count]) |input| {
        if (input.tick == tick) return &input;
    }
    return null;
}

/// Predict state forward
pub fn predict(
    replica: *ReplicaState,
    input: *const InputState,
    dt: f32,
) void {
    const move_speed: f32 = 200.0;
    const jump_force: f32 = 350.0;
    const gravity: f32 = -800.0;

    if (input.forward) replica.pos_z += move_speed * dt;
    if (input.backward) replica.pos_z -= move_speed * dt;
    if (input.left) replica.pos_x -= move_speed * dt;
    if (input.right) replica.pos_x += move_speed * dt;

    if (input.jump) replica.vel_y = jump_force;

    replica.vel_y += gravity * dt;
    replica.pos_y += replica.vel_y * dt;

    if (input.crouch) {
        replica.pos_y -= 2.0 * dt;
    }

    if (input.aim_x != 0 or input.aim_y != 0) {
        replica.yaw = @as(u8, @intFromFloat(@mod(@as(f32, @floatFromInt(replica.yaw)) + input.aim_x * 180.0, 360.0)));
    }

    replica.tick += 1;
    replica.checksum = calculateCRC(replica);
}

/// Reconcile local with remote state
pub fn reconcile(
    local: *ReplicaState,
    remote: *const ReplicaState,
) ReconciliationResult {
    if (local.tick != remote.tick) {
        return .TICK_MISMATCH;
    }

    if (local.pos_x != remote.pos_x or
        local.pos_y != remote.pos_y or
        local.pos_z != remote.pos_z)
    {
        const pos_error = @sqrt(
            (local.pos_x - remote.pos_x) * (local.pos_x - remote.pos_x) +
            (local.pos_y - remote.pos_y) * (local.pos_y - remote.pos_y) +
            (local.pos_z - remote.pos_z) * (local.pos_z - remote.pos_z)
        );

        if (pos_error > 10.0) {
            return .POSITION_DIVERGED;
        }
    }

    if (g_network_system.config.crc_check_enabled) {
        if (local.checksum != remote.checksum) {
            g_network_system.crc_errors += 1;
            return .CRC_MISMATCH;
        }
    }

    return .OK;
}

pub const ReconciliationResult = enum(u8) {
    OK = 0,
    TICK_MISMATCH = 1,
    POSITION_DIVERGED = 2,
    CRC_MISMATCH = 3,
    TIMEOUT = 4,
};

/// Rollback to previous state
pub fn rollback(replica: *ReplicaState, target_tick: u32) void {
    if (target_tick >= replica.tick) return;

    replica.tick = target_tick;
}

/// Check if rollback is needed
pub fn shouldRollback(replica: *const ReplicaState, remote: *const ReplicaState) bool {
    if (replica.tick < remote.tick) return true;

    const tick_diff = remote.tick - replica.tick;
    if (tick_diff > g_network_system.config.max_rollback_ticks) return true;

    return false;
}

/// Get system for external iteration
pub fn getSystem() *NetworkSystem {
    return &g_network_system;
}

/// Snapshot for save/restore
pub const Snapshot = struct {
    tick: u32,
    replicas: [MAX_REPLICAS]ReplicaState,
    replica_count: u8,
    crc_errors: u16,
};

/// Save current state
pub fn saveSnapshot() Snapshot {
    var snap: Snapshot = undefined;
    snap.tick = g_network_system.local_tick;
    snap.replica_count = g_network_system.replica_count;
    snap.crc_errors = g_network_system.crc_errors;

    for (g_network_system.replicas[0..g_network_system.replica_count], 0..) |replica, i| {
        snap.replicas[i] = replica;
    }

    return snap;
}

/// Restore from snapshot
pub fn restoreSnapshot(snap: *const Snapshot) void {
    g_network_system.local_tick = snap.tick;
    g_network_system.replica_count = snap.replica_count;
    g_network_system.crc_errors = snap.crc_errors;

    for (snap.replicas[0..snap.replica_count], 0..) |replica, i| {
        g_network_system.replicas[i] = replica;
    }
}

/// Advance local tick
pub fn advanceTick() void {
    g_network_system.local_tick += 1;
}

/// Get current tick
pub fn getTick() u32 {
    return g_network_system.local_tick;
}

/// Validate deterministic behavior
pub fn validateDeterminism(initial: *const Snapshot, final: *const Snapshot) bool {
    if (initial.replica_count != final.replica_count) return false;

    for (initial.replicas[0..initial.replica_count], 0..) |rep, i| {
        if (rep.checksum != final.replicas[i].checksum) return false;
    }

    return true;
}
