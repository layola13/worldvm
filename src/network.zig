//! Network - Deterministic Lockstep with Rollback
//!
//! Phase 12: Network synchronization, prediction, rollback, CRC validation
//! Handles: State replication, lag compensation, determinism verification

const std = @import("std");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const prediction = @import("prediction.zig");

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
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .tick = 0,
        .timestamp = 0,
        .checksum = 0,
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
    for (0..g_network_system.input_count) |i| {
        const input = &g_network_system.inputs[i];
        if (input.tick == tick) return input;
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

    var vel_x = replica.vel_x;
    var vel_z = replica.vel_z;

    if (input.forward) vel_z = move_speed;
    if (input.backward) vel_z = -move_speed;
    if (input.left) vel_x = -move_speed;
    if (input.right) vel_x = move_speed;

    if (input.jump) replica.vel_y = jump_force;

    replica.vel_x = vel_x;
    replica.vel_z = vel_z;
    replica.vel_y += gravity * dt;

    advanceReplicaLinearPosition(replica, dt);

    if (input.crouch) {
        replica.pos_y -= 2.0 * dt;
    }

    if (input.aim_x != 0 or input.aim_y != 0) {
        replica.yaw = @as(u8, @intFromFloat(@mod(@as(f32, @floatFromInt(replica.yaw)) + input.aim_x * 180.0, 360.0)));
    }

    replica.tick += 1;
    replica.checksum = calculateCRC(replica);
}

pub fn predictLinearReplica(replica: *const ReplicaState, dt: f32) ReplicaState {
    const predicted = prediction.predictLinearState(.{
        .pos_x = replica.pos_x,
        .pos_y = replica.pos_y,
        .pos_z = replica.pos_z,
        .vel_x = replica.vel_x,
        .vel_y = replica.vel_y,
        .vel_z = replica.vel_z,
    }, dt);

    var result = replica.*;
    result.pos_x = predicted.pos_x;
    result.pos_y = predicted.pos_y;
    result.pos_z = predicted.pos_z;
    return result;
}

fn advanceReplicaLinearPosition(replica: *ReplicaState, dt: f32) void {
    const predicted = predictLinearReplica(replica, dt);
    replica.pos_x = predicted.pos_x;
    replica.pos_y = predicted.pos_y;
    replica.pos_z = predicted.pos_z;
}

/// Predict replica state over a multi-step horizon for sync window planning.
/// Uses the unified prediction layer for consistency with sensors/ai_traffic/safety.
pub fn predictReplicaHorizon(replica: *const ReplicaState, dt: f32, steps: u8) prediction.PredictedStateSeries {
    var series = prediction.PredictedStateSeries{};
    var current = prediction.LinearState{
        .pos_x = replica.pos_x,
        .pos_y = replica.pos_y,
        .pos_z = replica.pos_z,
        .vel_x = replica.vel_x,
        .vel_y = replica.vel_y,
        .vel_z = replica.vel_z,
    };
    series.states[0] = current;
    series.valid_count = 1;

    var step: u8 = 1;
    while (step < steps and step < prediction.MAX_PREDICTION_STATES) : (step += 1) {
        const predicted = prediction.predictLinearState(current, dt);
        series.states[step] = predicted;
        series.valid_count += 1;
        current = predicted;
    }
    return series;
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
        const pos_error = @sqrt((local.pos_x - remote.pos_x) * (local.pos_x - remote.pos_x) +
            (local.pos_y - remote.pos_y) * (local.pos_y - remote.pos_y) +
            (local.pos_z - remote.pos_z) * (local.pos_z - remote.pos_z));

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
    if (replica.tick <= remote.tick) return false;
    const tick_diff = replica.tick - remote.tick;
    return tick_diff <= g_network_system.config.max_rollback_ticks;
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

// ============================================================================
// Sync Protocol (Item 287)
// ============================================================================
// The sync protocol defines how local and remote replicas are kept in sync:
// 1. Client predicts local replicas forward every tick using predict().
// 2. Server sends authoritative replica states at sync_rate_hz.
// 3. On receiving a server update, the client reconciles via reconcile().
// 4. If reconciliation fails, rollback to server tick and re-predict forward.

/// Apply a remote authoritative state with smooth correction or hard rollback.
pub fn applyRemoteState(
    local: *ReplicaState,
    remote: *const ReplicaState,
    input_history: []const InputState,
    dt: f32,
) ReconciliationResult {
    const result = reconcile(local, remote);
    switch (result) {
        .OK => {
            const snap_factor: f32 = 0.3;
            local.pos_x += (remote.pos_x - local.pos_x) * snap_factor;
            local.pos_y += (remote.pos_y - local.pos_y) * snap_factor;
            local.pos_z += (remote.pos_z - local.pos_z) * snap_factor;
            local.vel_x += (remote.vel_x - local.vel_x) * snap_factor;
            local.vel_y += (remote.vel_y - local.vel_y) * snap_factor;
            local.vel_z += (remote.vel_z - local.vel_z) * snap_factor;
        },
        .POSITION_DIVERGED, .CRC_MISMATCH, .TICK_MISMATCH => {
            rollbackWithPrediction(local, remote, input_history, dt);
        },
        .TIMEOUT => {},
    }
    return result;
}

// ============================================================================
// Rollback Semantics (Item 288)
// ============================================================================
// Rollback resets to an authoritative remote snapshot and re-predicts forward
// through input history using the unified prediction layer. This guarantees
// deterministic reconciliation.

/// Full rollback: reset to remote state and re-predict through input history.
pub fn rollbackWithPrediction(
    replica: *ReplicaState,
    remote: *const ReplicaState,
    input_history: []const InputState,
    dt: f32,
) void {
    replica.* = remote.*;
    for (input_history) |input| {
        if (input.tick > remote.tick) {
            predict(replica, &input, dt);
        }
    }
}

/// Predict the result of a hypothetical rollback without modifying state.
pub fn predictRollbackResult(
    target_remote: *const ReplicaState,
    input_history: []const InputState,
    dt: f32,
) ReplicaState {
    var hypothetical = target_remote.*;
    for (input_history) |input| {
        if (input.tick > target_remote.tick) {
            predict(&hypothetical, &input, dt);
        }
    }
    return hypothetical;
}

/// Validate deterministic behavior
pub fn validateDeterminism(initial: *const Snapshot, final: *const Snapshot) bool {
    if (initial.replica_count != final.replica_count) return false;

    for (initial.replicas[0..initial.replica_count], 0..) |rep, i| {
        if (rep.checksum != final.replicas[i].checksum) return false;
    }

    return true;
}

test "network getInput returns stable backing pointer" {
    init(.{});
    storeInput(.{
        .tick = 42,
        .forward = true,
        .backward = false,
        .left = false,
        .right = false,
        .jump = false,
        .crouch = false,
        .fire = false,
        .aim_x = 0,
        .aim_y = 0,
    });

    const got = getInput(42);
    try std.testing.expect(got != null);
    const expected_ptr = &getSystem().inputs[0];
    try std.testing.expectEqual(@intFromPtr(expected_ptr), @intFromPtr(got.?));
}

test "network shouldRollback only for bounded local-ahead rollback window" {
    init(.{ .max_rollback_ticks = 5 });

    var local = ReplicaState{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .tick = 10,
        .timestamp = 0,
        .checksum = 0,
    };
    var remote = local;

    remote.tick = 8;
    try std.testing.expect(shouldRollback(&local, &remote));

    remote.tick = 4;
    try std.testing.expect(!shouldRollback(&local, &remote));

    remote.tick = 12;
    try std.testing.expect(!shouldRollback(&local, &remote));
}

test "network rollbackWithPrediction replays only inputs newer than remote tick" {
    init(.{});

    var local = ReplicaState{
        .entity_id = 7,
        .pos_x = 100,
        .pos_y = 10,
        .pos_z = -50,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .tick = 99,
        .timestamp = 0,
        .checksum = 0,
    };
    var remote = ReplicaState{
        .entity_id = 7,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .tick = 5,
        .timestamp = 0,
        .checksum = 0,
    };
    remote.checksum = calculateCRC(&remote);

    const history = [_]InputState{
        .{ .tick = 4, .forward = true, .backward = false, .left = false, .right = false, .jump = false, .crouch = false, .fire = false, .aim_x = 0, .aim_y = 0 },
        .{ .tick = 6, .forward = true, .backward = false, .left = false, .right = false, .jump = false, .crouch = false, .fire = false, .aim_x = 0, .aim_y = 0 },
        .{ .tick = 7, .forward = false, .backward = false, .left = false, .right = true, .jump = false, .crouch = false, .fire = false, .aim_x = 0, .aim_y = 0 },
    };

    rollbackWithPrediction(&local, &remote, history[0..], 0.1);
    try std.testing.expectEqual(@as(u32, 7), local.tick);
    try std.testing.expect(local.pos_z > 30.0);
    try std.testing.expect(local.pos_x > 15.0);
}

test "network applyRemoteState smooths minor delta and rollbacks diverged state" {
    init(.{ .crc_check_enabled = false });

    var local_ok = ReplicaState{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .tick = 10,
        .timestamp = 0,
        .checksum = 0,
    };
    var remote_ok = local_ok;
    remote_ok.pos_x = 10.0; // <= divergence threshold, should smooth

    const empty_history = [_]InputState{};
    const ok_result = applyRemoteState(&local_ok, &remote_ok, empty_history[0..], 0.016);
    try std.testing.expect(ok_result == .OK);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), local_ok.pos_x, 0.0001);

    var local_diverged = local_ok;
    local_diverged.tick = 8;
    local_diverged.pos_x = 0;
    var remote_diverged = local_diverged;
    remote_diverged.pos_x = 50.0;

    const replay_history = [_]InputState{
        .{ .tick = 9, .forward = true, .backward = false, .left = false, .right = false, .jump = false, .crouch = false, .fire = false, .aim_x = 0, .aim_y = 0 },
    };
    const diverged_result = applyRemoteState(&local_diverged, &remote_diverged, replay_history[0..], 0.1);
    try std.testing.expect(diverged_result == .POSITION_DIVERGED);
    try std.testing.expectEqual(@as(u32, 9), local_diverged.tick);
    try std.testing.expectApproxEqAbs(@as(f32, 50.0), local_diverged.pos_x, 0.0001);
    try std.testing.expect(local_diverged.pos_z > 15.0);
}
