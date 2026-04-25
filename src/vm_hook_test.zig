const std = @import("std");
const vm_hook = @import("vm_hook.zig");

test "HookResultCode values match C int" {
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.PASS) == 0);
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.FAIL) == 1);
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.UNKNOWN) == 2);
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.TIMEOUT) == 3);
}

test "reset_context returns -1 when kernel not initialized" {
    _ = vm_hook.shutdown_kernel();
    const rc = vm_hook.reset_context();
    try std.testing.expect(rc == -1);
}

test "init_kernel returns 0 on first call" {
    _ = vm_hook.shutdown_kernel();
    const r = vm_hook.init_kernel();
    try std.testing.expect(r == 0);
    _ = vm_hook.shutdown_kernel();
}

test "init_kernel returns 1 when already initialized" {
    _ = vm_hook.shutdown_kernel();
    _ = vm_hook.init_kernel();
    const r = vm_hook.init_kernel();
    try std.testing.expect(r == 1);
    _ = vm_hook.shutdown_kernel();
}

test "run_logic_check returns UNKNOWN when kernel not initialized" {
    _ = vm_hook.shutdown_kernel();
    const name_with_null: [*:0]const u8 = "apple_table";
    const r = vm_hook.run_logic_check(name_with_null, 50);
    try std.testing.expect(r == .UNKNOWN);
}

test "run_logic_check returns PASS or FAIL when kernel initialized" {
    _ = vm_hook.shutdown_kernel();
    _ = vm_hook.init_kernel();
    const name_with_null: [*:0]const u8 = "apple_table";
    const r = vm_hook.run_logic_check(name_with_null, 50);
    try std.testing.expect(r == .PASS or r == .FAIL);
    _ = vm_hook.shutdown_kernel();
}

test "get_trace_summary returns UNKNOWN when kernel not initialized" {
    _ = vm_hook.shutdown_kernel();
    const summary = vm_hook.get_trace_summary();
    try std.testing.expect(summary.final_status == .UNKNOWN);
    try std.testing.expect(summary.entry_count == 0);
}

test "get_trace_summary returns valid data after init" {
    _ = vm_hook.shutdown_kernel();
    _ = vm_hook.init_kernel();
    const summary = vm_hook.get_trace_summary();
    try std.testing.expect(summary.final_status == .PASS or summary.final_status == .FAIL);
    _ = vm_hook.shutdown_kernel();
}

test "shutdown_kernel returns 0 after init" {
    _ = vm_hook.init_kernel();
    const r = vm_hook.shutdown_kernel();
    try std.testing.expect(r == 0);
}

test "shutdown_kernel returns -1 when not initialized" {
    _ = vm_hook.shutdown_kernel();
    const r = vm_hook.shutdown_kernel();
    try std.testing.expect(r == -1);
}

test "world snapshot buffer is visible through hook after stepping" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 10, 0) == 0);
    _ = vm_hook.run_ticks(4);
    try std.testing.expect(vm_hook.rewind_get_world_snapshot_count() > 0);
    _ = vm_hook.shutdown_kernel();
}

test "snapshot prediction is visible through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 10, 0) == 0);
    _ = vm_hook.run_ticks(2);

    var predicted: [3]f32 = .{0} ** 3;
    try std.testing.expect(vm_hook.rewind_predict_instance_position(1, 0, 0.0, &predicted) == 0);
    try std.testing.expect(std.math.isFinite(predicted[0]));
    try std.testing.expect(std.math.isFinite(predicted[1]));
    try std.testing.expect(std.math.isFinite(predicted[2]));
    _ = vm_hook.shutdown_kernel();
}

test "snapshot pose prediction exports yaw through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 10, 0) == 0);
    try std.testing.expect(vm_hook.apply_torque(0, 0, 50, 0) == 0);
    _ = vm_hook.run_ticks(1);

    var predicted: [4]f32 = .{0} ** 4;
    try std.testing.expect(vm_hook.rewind_predict_instance_pose(1, 0, 1.0, &predicted) == 0);
    try std.testing.expect(std.math.isFinite(predicted[0]));
    try std.testing.expect(std.math.isFinite(predicted[1]));
    try std.testing.expect(std.math.isFinite(predicted[2]));
    try std.testing.expect(std.math.isFinite(predicted[3]));
    try std.testing.expect(predicted[3] != 0.0);
    _ = vm_hook.shutdown_kernel();
}

test "snapshot forecast exports batch positions through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 10, 0) == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 5, 12, 0) == 0);
    _ = vm_hook.run_ticks(2);

    var buffer: [16]f32 = .{0} ** 16;
    const count = vm_hook.rewind_forecast_snapshot_positions(1, 0.0, &buffer, 4);
    try std.testing.expect(count > 0);
    try std.testing.expect(std.math.isFinite(buffer[1]));
    try std.testing.expect(std.math.isFinite(buffer[2]));
    try std.testing.expect(std.math.isFinite(buffer[3]));
    _ = vm_hook.shutdown_kernel();
}

test "snapshot simulated preview advances isolated future position through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 400, 0) == 0);
    _ = vm_hook.run_ticks(1);

    var snapshot_pos: [3]f32 = .{0} ** 3;
    var simulated_pos: [3]f32 = .{0} ** 3;

    try std.testing.expect(vm_hook.rewind_predict_instance_position(1, 0, 0.0, &snapshot_pos) == 0);
    try std.testing.expect(vm_hook.rewind_simulate_snapshot_instance_position(1, 0, 1, &simulated_pos) == 0);
    try std.testing.expect(simulated_pos[1] < snapshot_pos[1]);
    _ = vm_hook.shutdown_kernel();
}

test "snapshot simulated hash changes after forward preview" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 400, 0) == 0);
    _ = vm_hook.run_ticks(1);

    const current_hash = vm_hook.rewind_get_world_snapshot_hash(1);
    const future_hash = vm_hook.rewind_simulate_snapshot_hash(1, 1);
    try std.testing.expect(current_hash != 0);
    try std.testing.expect(future_hash != 0);
    try std.testing.expect(current_hash != future_hash);
    _ = vm_hook.shutdown_kernel();
}

test "snapshot simulated batch forecast exports future positions through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 400, 0) == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 5, 420, 0) == 0);
    _ = vm_hook.run_ticks(1);

    var current: [16]f32 = .{0} ** 16;
    var simulated: [16]f32 = .{0} ** 16;
    const current_count = vm_hook.rewind_forecast_snapshot_positions(1, 0.0, &current, 4);
    const simulated_count = vm_hook.rewind_simulate_snapshot_positions(1, 1, &simulated, 4);

    try std.testing.expect(current_count == simulated_count);
    try std.testing.expect(simulated_count >= 2);
    try std.testing.expect(std.math.isFinite(simulated[1]));
    try std.testing.expect(std.math.isFinite(simulated[2]));
    try std.testing.expect(std.math.isFinite(simulated[3]));
    try std.testing.expect(simulated[2] < current[2] or simulated[6] < current[6]);
    _ = vm_hook.shutdown_kernel();
}

test "snapshot simulated diff exports structural change summary through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 0, 400, 0) == 0);
    _ = vm_hook.run_ticks(1);

    var diff: [31]u32 = .{0} ** 31;
    const written = vm_hook.rewind_diff_simulated_snapshot(1, 1, &diff, diff.len);
    try std.testing.expect(written == 31);
    try std.testing.expect(diff[0] == 1);
    try std.testing.expect(diff[1] == 2);
    try std.testing.expect(diff[2] == 1);
    try std.testing.expect(diff[4] > 0);
    try std.testing.expect(diff[20] == 0 or diff[20] == 1);
    try std.testing.expect(diff[21] == 0 or diff[21] == 1);
    try std.testing.expect(diff[22] == 0 or diff[22] == 1);
    try std.testing.expect(diff[23] == 0 or diff[23] == 1);
    try std.testing.expect(diff[24] == 0 or diff[24] == 1);
    try std.testing.expect(diff[25] == 0 or diff[25] == 1);
    try std.testing.expect(diff[26] == 0 or diff[26] == 1);
    try std.testing.expect(diff[27] == 0 or diff[27] == 1);
    try std.testing.expect(diff[28] == 0 or diff[28] == 1);
    try std.testing.expect(diff[29] == 0 or diff[29] == 1);
    try std.testing.expect(diff[30] == 0 or diff[30] == 1);
    _ = vm_hook.shutdown_kernel();
}

test "prediction collision risk is visible through hook" {
    var result: [7]f32 = .{0} ** 7;
    const rc = vm_hook.prediction_assess_collision_risk(
        0, 0, 0, 10, 0, 0,
        1.5, 0, 0, -10, 0, 0,
        1.0, 2.0, 0.1,
        &result,
    );

    try std.testing.expect(rc == 1);
    try std.testing.expect(result[0] >= 3);
    try std.testing.expect(result[1] > 0.5);
    try std.testing.expect(result[2] >= 0);
}

test "prediction collision risk hook returns zero for safe parallel motion" {
    var result: [7]f32 = .{0} ** 7;
    const rc = vm_hook.prediction_assess_collision_risk(
        0, 0, 0, 5, 0, 0,
        0, 10, 0, 5, 0, 0,
        1.0, 3.0, 0.1,
        &result,
    );

    try std.testing.expect(rc == 0);
    try std.testing.expect(result[0] == 0);
    try std.testing.expect(result[1] == 0);
}

test "prediction occupancy conflict hook returns future overlap window" {
    var result: [2]f32 = .{0} ** 2;
    const rc = vm_hook.prediction_compute_occupancy_conflict(
        -5, 0, 0, 4, 0, 0, 0, 0, 1, 1, 1,
        5, 0, 0, -4, 0, 0, 0, 0, 1, 1, 1,
        3.0, 0.25,
        &result,
    );
    try std.testing.expect(rc == 1);
    try std.testing.expect(result[0] >= 0.75 and result[0] <= 1.25);
    try std.testing.expect(result[1] >= result[0]);
}

test "kcc hook refuses uncrouch when ceiling blocks standing volume" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(1, 0, 6, 0) == 0);

    const config = vm_hook.KCCConfigFFI{
        .move_speed = 200,
        .jump_force = 350,
        .gravity = -800,
        .crouch_speed_mult = 0.5,
        .push_force = 100,
        .step_height = 2,
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
        .max_slope_angle = 45,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
    };
    try std.testing.expect(vm_hook.kcc_create_character(0, 0, 0, config) == 1);
    try std.testing.expect(vm_hook.kcc_try_set_crouch(0, true) == 1);
    try std.testing.expect(vm_hook.kcc_get_height(0) == 4);
    try std.testing.expect(vm_hook.kcc_try_set_crouch(0, false) == 0);
    try std.testing.expect(vm_hook.kcc_get_height(0) == 4);
    _ = vm_hook.shutdown_kernel();
}

test "penetration manifold is visible through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);

    var result: [17]f32 = .{0} ** 17;
    const rc = vm_hook.query_compute_penetration_aabb_with_manifold(0.2, 0.2, 0.2, 0.8, 0.8, 0.8, &result, result.len);
    try std.testing.expect(rc == 0 or rc == 1);

    try std.testing.expect(vm_hook.spawn_instance(1, 0, 0, 0) == 0);
    const overlap_rc = vm_hook.query_compute_penetration_aabb_with_manifold(0.2, 0.2, 0.2, 0.8, 0.8, 0.8, &result, result.len);
    try std.testing.expect(overlap_rc == 1);
    try std.testing.expect(result[0] > 0);
    try std.testing.expect(result[4] > 0);
    try std.testing.expect(std.math.isFinite(result[5]));
    try std.testing.expect(std.math.isFinite(result[6]));
    try std.testing.expect(std.math.isFinite(result[7]));
    _ = vm_hook.shutdown_kernel();
}

test "capsule penetration is visible through hook" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(1, 0, 2, 0) == 0);

    var result: [5]f32 = .{0} ** 5;
    const rc = vm_hook.query_compute_penetration_capsule(0.6, 2.5, 0.5, 0.8, 0.5, &result, result.len);
    try std.testing.expect(rc == 1);
    try std.testing.expect(result[0] > 0);
    try std.testing.expect(result[1] != 0 or result[2] != 0 or result[3] != 0);
    _ = vm_hook.shutdown_kernel();
}

test "layer-aware raycast hook can filter environment and dynamic hits" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 1, 0, 0) == 0);

    var hit: [8]f32 = .{0} ** 8;

    const dyn_rc = vm_hook.raycast_single_with_layer_mask(0.1, 8.0, 8.0, 1.0, 0.0, 0.0, 20.0, 1 << 2, &hit);
    try std.testing.expect(dyn_rc == 1);
    try std.testing.expect(hit[4] >= 0);

    const env_rc = vm_hook.raycast_single_with_layer_mask(0.1, 8.0, 8.0, 1.0, 0.0, 0.0, 20.0, 1 << 0, &hit);
    try std.testing.expect(env_rc == 0);
    _ = vm_hook.shutdown_kernel();
}

test "layer-aware overlap hook can separate dynamic and environment overlaps" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(0, 1, 0, 0) == 0);

    var result: [4]f32 = .{0} ** 4;

    const dyn_rc = vm_hook.query_overlap_aabb_with_layer_mask(8.2, 8.2, 8.2, 8.8, 8.8, 8.8, 1 << 2, &result, result.len);
    try std.testing.expect(dyn_rc == 1);
    try std.testing.expect(result[1] > 0);
    try std.testing.expect(result[2] >= 0);
    try std.testing.expect(result[3] == 0);

    const env_rc = vm_hook.query_overlap_aabb_with_layer_mask(-0.4, 0.2, 0.2, 0.2, 0.8, 0.8, 1 << 0, &result, result.len);
    try std.testing.expect(env_rc == 1);
    try std.testing.expect(result[3] == 1);
    _ = vm_hook.shutdown_kernel();
}

test "layer-aware capsule overlap hook can isolate environment hits" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);

    var result: [4]f32 = .{0} ** 4;
    const env_rc = vm_hook.query_overlap_capsule_with_layer_mask(-0.1, 0.5, 0.5, 0.8, 0.5, 1 << 0, &result, result.len);
    try std.testing.expect(env_rc == 1);
    try std.testing.expect(result[3] == 1);
    try std.testing.expect(result[2] < 0);

    const dyn_rc = vm_hook.query_overlap_capsule_with_layer_mask(-0.1, 0.5, 0.5, 0.8, 0.5, 1 << 2, &result, result.len);
    try std.testing.expect(dyn_rc == 0);
    _ = vm_hook.shutdown_kernel();
}

test "layer-aware penetration hook can separate dynamic and environment manifolds" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(1, 0, 0, 0) == 0);

    var result: [18]f32 = .{0} ** 18;

    const dyn_rc = vm_hook.query_compute_penetration_aabb_with_layer_mask(0.2, 0.2, 0.2, 0.8, 0.8, 0.8, 1 << 1, &result, result.len);
    try std.testing.expect(dyn_rc == 1);
    try std.testing.expect(result[0] > 0);
    try std.testing.expect(result[4] > 0);
    try std.testing.expect(result[5] >= 0);

    const env_rc = vm_hook.query_compute_penetration_aabb_with_layer_mask(-0.4, 0.2, 0.2, 0.2, 0.8, 0.8, 1 << 0, &result, result.len);
    try std.testing.expect(env_rc == 1);
    try std.testing.expect(result[5] < 0);
    _ = vm_hook.shutdown_kernel();
}

test "layer-aware capsule penetration hook can isolate static instance hits" {
    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    try std.testing.expect(vm_hook.spawn_instance(1, 0, 2, 0) == 0);

    var result: [5]f32 = .{0} ** 5;
    const static_rc = vm_hook.query_compute_penetration_capsule_with_layer_mask(0.6, 2.5, 0.5, 0.8, 0.5, 1 << 1, &result, result.len);
    try std.testing.expect(static_rc == 1);
    try std.testing.expect(result[0] > 0);
    try std.testing.expect(result[4] >= 0);

    const env_rc = vm_hook.query_compute_penetration_capsule_with_layer_mask(-0.1, 0.5, 0.5, 0.8, 0.5, 1 << 0, &result, result.len);
    try std.testing.expect(env_rc == 1);
    _ = vm_hook.shutdown_kernel();
}

test "vehicle predictive conflict response hook exports brake and steering advice" {
    const vehicle = @import("vehicle.zig");

    _ = vm_hook.shutdown_kernel();
    try std.testing.expect(vm_hook.init_kernel() == 0);
    vm_hook.vehicle_init();
    try std.testing.expect(vm_hook.vehicle_create_car(-12.0, 0.0, 0.0, 0.0) == 1);
    try std.testing.expect(vm_hook.vehicle_create_car(-12.0, 0.0, 20.0, std.math.pi) == 1);

    const vsys = vehicle.getSystem();
    vsys.vehicles[0].speed = 10.0;
    vsys.vehicles[1].speed = 10.0;
    vsys.vehicles[0].steering = 0.0;
    vsys.vehicles[1].steering = 0.0;

    var result: [5]f32 = .{0} ** 5;
    const rc = vm_hook.vehicle_predictive_conflict_response(0, 1, 0.25, &result);
    try std.testing.expect(rc == 1);
    try std.testing.expect(result[0] == 1.0);
    try std.testing.expect(result[1] >= 0.0);
    try std.testing.expect(result[3] >= 0.5);
    try std.testing.expect(@abs(result[4]) > 0.01);
    _ = vm_hook.shutdown_kernel();
}
