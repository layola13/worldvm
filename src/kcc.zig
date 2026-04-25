//! KCC - Kinematic Character Controller
//!
//! Phase 7: Character movement for player/AI controlled characters
//! Handles: ground detection, collision response, jumping, crouching, rigid body pushing

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const prediction = @import("prediction.zig");
const query = @import("query.zig");
const query_types = @import("query_types.zig");
const query_penetration = @import("query_penetration.zig");

pub const KCCState = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    yaw: f32,
    grounded: bool,
    crouching: bool,
    jumping: bool,
    stand_height: i32,
    crouch_height: i32,
    radius: i32,
    mass: u16,
    move_speed: f32,
    jump_force: f32,
    gravity: f32,
    crouch_speed_mult: f32,
    push_force: f32,
    step_height: i32,
    max_slope_angle: f32,
    step_offset: f32,
    prevent_fall_off_ledges: bool,

    // Internal state for KCC
    was_grounded: bool,
    ground_normal_x: f32,
    ground_normal_y: f32,
    ground_normal_z: f32,
    support_instance_idx: i16,
    support_pos_x: f32,
    support_pos_y: f32,
    support_pos_z: f32,
    support_rot_yaw: u8,
    support_yaw_steps: f32,
    support_actual_pos_x: f32,
    support_actual_pos_y: f32,
    support_actual_pos_z: f32,
    support_actual_rot_yaw: u8,
};

pub const KCCConfig = struct {
    move_speed: f32 = 200.0,
    jump_force: f32 = 350.0,
    gravity: f32 = -800.0,
    crouch_speed_mult: f32 = 0.5,
    push_force: f32 = 100.0,
    step_height: i32 = 2,
    stand_height: i32 = 14,
    crouch_height: i32 = 8,
    radius: i32 = 4,
    max_slope_angle: f32 = 45.0,
    step_offset: f32 = 0.25,
    prevent_fall_off_ledges: bool = false,
};

pub const MAX_KCC: usize = 8;

pub const KCCSystem = struct {
    characters: [MAX_KCC]KCCState,
    count: u8,
};

var g_kcc_system: KCCSystem = undefined;

pub fn init() void {
    g_kcc_system.count = 0;
    for (0..MAX_KCC) |i| {
        g_kcc_system.characters[i] = .{
            .pos_x = 0, .pos_y = 0, .pos_z = 0,
            .vel_x = 0, .vel_y = 0, .vel_z = 0,
            .yaw = 0, .grounded = false, .crouching = false, .jumping = false,
            .stand_height = 14, .crouch_height = 8, .radius = 4, .mass = 80,
            .move_speed = 200.0, .jump_force = 350.0, .gravity = -800.0,
            .crouch_speed_mult = 0.5, .push_force = 100.0, .step_height = 2,
            .max_slope_angle = 45.0, .step_offset = 0.25, .prevent_fall_off_ledges = false,
            .was_grounded = false,
            .ground_normal_x = 0, .ground_normal_y = 1, .ground_normal_z = 0,
            .support_instance_idx = -1,
            .support_pos_x = 0,
            .support_pos_y = 0,
            .support_pos_z = 0,
            .support_rot_yaw = 0,
            .support_yaw_steps = 0,
            .support_actual_pos_x = 0,
            .support_actual_pos_y = 0,
            .support_actual_pos_z = 0,
            .support_actual_rot_yaw = 0,
        };
    }
}

pub fn createCharacter(x: f32, y: f32, z: f32, config: KCCConfig) ?*KCCState {
    if (g_kcc_system.count >= MAX_KCC) return null;
    const idx = g_kcc_system.count;
    g_kcc_system.count += 1;
    const char = &g_kcc_system.characters[idx];
    char.* = .{
        .pos_x = x, .pos_y = y, .pos_z = z,
        .vel_x = 0, .vel_y = 0, .vel_z = 0,
        .yaw = 0, .grounded = false, .crouching = false, .jumping = false,
        .stand_height = config.stand_height,
        .crouch_height = config.crouch_height,
        .radius = config.radius,
        .mass = 80,
        .move_speed = config.move_speed,
        .jump_force = config.jump_force,
        .gravity = config.gravity,
        .crouch_speed_mult = config.crouch_speed_mult,
        .push_force = config.push_force,
        .step_height = config.step_height,
        .max_slope_angle = config.max_slope_angle,
        .step_offset = config.step_offset,
        .prevent_fall_off_ledges = config.prevent_fall_off_ledges,
        .was_grounded = false,
        .ground_normal_x = 0, .ground_normal_y = 1, .ground_normal_z = 0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };
    return char;
}

pub fn removeCharacter(char: *KCCState) void {
    _ = char;
    if (g_kcc_system.count > 0) g_kcc_system.count -= 1;
}

pub fn move(state: *KCCState, input_x: f32, _: f32, input_z: f32, _: f32, config: KCCConfig) void {
    const speed = if (state.crouching) config.move_speed * config.crouch_speed_mult else config.move_speed;
    state.vel_x = input_x * speed;
    state.vel_z = input_z * speed;
}

pub fn jump(state: *KCCState, config: KCCConfig) void {
    if (state.grounded and !state.crouching) {
        state.vel_y = config.jump_force;
        state.grounded = false;
        state.jumping = true;
    }
}

pub fn crouch(state: *KCCState, active: bool) void {
    if (state.grounded) {
        state.crouching = active;
    }
}

pub fn trySetCrouch(
    state: *KCCState,
    active: bool,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    if (active) {
        state.crouching = true;
        return true;
    }

    if (!state.crouching) return true;
    if (!canFitThrough(state.pos_x, state.pos_y, state.pos_z, state.stand_height, state.radius, s1024, entities)) {
        return false;
    }

    state.crouching = false;
    return true;
}

pub fn setYaw(state: *KCCState, yaw: f32) void {
    state.yaw = yaw;
}

pub fn getHeight(state: *KCCState) i32 {
    return if (state.crouching) state.crouch_height else state.stand_height;
}

fn getHeightConst(state: *const KCCState) i32 {
    return if (state.crouching) state.crouch_height else state.stand_height;
}

/// Check if position is grounded using voxel sampling
pub fn checkGrounded(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    const check_y = @as(i32, @intFromFloat(@floor(state.pos_y))) - 1;

    const radius = state.radius;
    const half_radius = @divTrunc(radius, 2);

    const world_view = query.QueryWorldView{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };

    const filter = query.QueryFilter{
        .include_static = true,
        .include_dynamic = true,
        .include_kinematic = true,
        .include_sensors = false,
    };

    const base_x = @as(i32, @intFromFloat(@floor(state.pos_x)));
    const base_z = @as(i32, @intFromFloat(@floor(state.pos_z)));

    if (query.queryAnyVoxel(&world_view, base_x, check_y, base_z, filter).hit) {
        return true;
    }

    var dx: i32 = -half_radius;
    while (dx <= half_radius) : (dx += 2) {
        var dz: i32 = -half_radius;
        while (dz <= half_radius) : (dz += 2) {
            if (query.queryAnyVoxel(&world_view, base_x + dx, check_y, base_z + dz, filter).hit) {
                return true;
            }
        }
    }
    return false;
}

fn findSupportingKinematicInstance(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) i16 {
    const check_y = @as(i32, @intFromFloat(@floor(state.pos_y))) - 1;
    const radius = state.radius;
    const half_radius = @divTrunc(radius, 2);

    const world_view = query.QueryWorldView{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const filter = query.QueryFilter{
        .include_static = true,
        .include_dynamic = true,
        .include_kinematic = true,
        .include_sensors = false,
    };

    const base_x = @as(i32, @intFromFloat(@floor(state.pos_x)));
    const base_z = @as(i32, @intFromFloat(@floor(state.pos_z)));

    const center_hit = query.queryAnyVoxel(&world_view, base_x, check_y, base_z, filter);
    if (center_hit.instance_idx >= 0 and center_hit.classification.body_type == .kinematic) {
        return center_hit.instance_idx;
    }

    var dx: i32 = -half_radius;
    while (dx <= half_radius) : (dx += 2) {
        var dz: i32 = -half_radius;
        while (dz <= half_radius) : (dz += 2) {
            const hit = query.queryAnyVoxel(&world_view, base_x + dx, check_y, base_z + dz, filter);
            if (hit.instance_idx >= 0 and hit.classification.body_type == .kinematic) {
                return hit.instance_idx;
            }
        }
    }

    return -1;
}

/// Get ground normal for slope handling
pub fn getGroundNormal(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) struct { x: f32, y: f32, z: f32 } {
    const px = @as(i32, @intFromFloat(@floor(state.pos_x)));
    const pz = @as(i32, @intFromFloat(@floor(state.pos_z)));
    const below_y = @as(i32, @intFromFloat(@floor(state.pos_y))) - 1;

    const world_view = query.QueryWorldView{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const filter = query.QueryFilter{
        .include_static = true,
        .include_dynamic = true,
        .include_kinematic = true,
        .include_sensors = false,
    };

    const left: f32 = if (query.queryAnyVoxel(&world_view, px - 1, below_y, pz, filter).hit) 1.0 else 0.0;
    const right: f32 = if (query.queryAnyVoxel(&world_view, px + 1, below_y, pz, filter).hit) 1.0 else 0.0;
    const back: f32 = if (query.queryAnyVoxel(&world_view, px, below_y, pz - 1, filter).hit) 1.0 else 0.0;
    const front: f32 = if (query.queryAnyVoxel(&world_view, px, below_y, pz + 1, filter).hit) 1.0 else 0.0;

    var nx = left - right;
    var ny: f32 = 1.0;
    var nz = back - front;
    const len_sq = nx * nx + ny * ny + nz * nz;
    if (len_sq <= 0.000001) {
        return .{ .x = 0, .y = 1, .z = 0 };
    }

    const inv_len = 1.0 / @sqrt(len_sq);
    nx *= inv_len;
    ny *= inv_len;
    nz *= inv_len;
    return .{ .x = nx, .y = ny, .z = nz };
}

fn trySnapToGround(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    config: KCCConfig,
) void {
    if (state.grounded or state.vel_y > 0) return;

    const snap_distance = @max(config.step_offset, 0.0);
    if (snap_distance <= 0.0) return;

    const original_y = state.pos_y;
    state.pos_y -= snap_distance;

    if (!checkCollision(state.pos_x, state.pos_y, state.pos_z, getHeight(state), state.radius, s1024, entities) and
        checkGrounded(state, s1024, entities))
    {
        state.grounded = true;
        state.vel_y = 0;
        state.jumping = false;
        return;
    }

    state.pos_y = original_y;
}

fn getSupportPose(state: *const KCCState) prediction.EffectiveSupportPose {
    return .{
        .actual_pos_x = state.support_actual_pos_x,
        .actual_pos_y = state.support_actual_pos_y,
        .actual_pos_z = state.support_actual_pos_z,
        .effective_pos_x = state.support_pos_x,
        .effective_pos_y = state.support_pos_y,
        .effective_pos_z = state.support_pos_z,
        .actual_rot_yaw = state.support_actual_rot_yaw,
        .effective_yaw_steps = state.support_yaw_steps,
    };
}

fn carryAlongSupportingKinematic(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    dt: f32,
) bool {
    if (!state.grounded or state.jumping) {
        state.support_instance_idx = -1;
        return false;
    }

    var carried = false;
    if (state.support_instance_idx >= 0 and state.support_instance_idx < s1024.instance_count) {
        const remembered_inst = &s1024.instances[@as(u8, @intCast(state.support_instance_idx))];
        const pose = prediction.advanceSupportPose(getSupportPose(state), remembered_inst, dt);
        const delta_x = pose.effective_pos_x - state.support_pos_x;
        const delta_y = pose.effective_pos_y - state.support_pos_y;
        const delta_z = pose.effective_pos_z - state.support_pos_z;
        const yaw_delta_steps = pose.effective_yaw_steps - state.support_yaw_steps;
        const yaw_delta = yaw_delta_steps * (std.math.tau / 256.0);

        if (@abs(yaw_delta) > 0.000001) {
            const previous_center = getSupportRotationCenter(&scene32.Instance{
                .entity_id = remembered_inst.entity_id,
                .pos_x = @as(i32, @intFromFloat(state.support_pos_x)),
                .pos_y = @as(i32, @intFromFloat(state.support_pos_y)),
                .pos_z = @as(i32, @intFromFloat(state.support_pos_z)),
                .rot_yaw = @intCast(@mod(@as(i32, @intFromFloat(@round(state.support_yaw_steps))), 256)),
                .rot_pitch = remembered_inst.rot_pitch,
                .rot_roll = remembered_inst.rot_roll,
                .state = remembered_inst.state,
                .sleep_tick = remembered_inst.sleep_tick,
                .vel_x = remembered_inst.vel_x,
                .vel_y = remembered_inst.vel_y,
                .vel_z = remembered_inst.vel_z,
                .ang_x = remembered_inst.ang_x,
                .ang_y = remembered_inst.ang_y,
                .ang_z = remembered_inst.ang_z,
                ._reserved = remembered_inst._reserved,
            }, entities);
            const current_center = getSupportRotationCenter(remembered_inst, entities);
            const rel_x = state.pos_x - previous_center.x;
            const rel_z = state.pos_z - previous_center.z;
            const sin_yaw = @sin(yaw_delta);
            const cos_yaw = @cos(yaw_delta);
            state.pos_x = current_center.x + (pose.effective_pos_x - pose.actual_pos_x) + rel_x * cos_yaw - rel_z * sin_yaw;
            state.pos_z = current_center.z + (pose.effective_pos_z - pose.actual_pos_z) + rel_x * sin_yaw + rel_z * cos_yaw;
        } else {
            state.pos_x += delta_x;
            state.pos_z += delta_z;
        }

        state.pos_y += delta_y;
        state.support_pos_x = pose.effective_pos_x;
        state.support_pos_y = pose.effective_pos_y;
        state.support_pos_z = pose.effective_pos_z;
        state.support_yaw_steps = pose.effective_yaw_steps;
        state.support_rot_yaw = pose.actual_rot_yaw;
        state.support_actual_pos_x = pose.actual_pos_x;
        state.support_actual_pos_y = pose.actual_pos_y;
        state.support_actual_pos_z = pose.actual_pos_z;
        state.support_actual_rot_yaw = pose.actual_rot_yaw;
        carried = true;
    }

    const support_idx = findSupportingKinematicInstance(state, s1024, entities);
    if (support_idx < 0) {
        if (carried and state.support_instance_idx >= 0 and state.support_instance_idx < s1024.instance_count) {
            const remembered_inst = &s1024.instances[@as(u8, @intCast(state.support_instance_idx))];
            state.support_rot_yaw = remembered_inst.rot_yaw;
            state.support_actual_pos_x = @as(f32, @floatFromInt(remembered_inst.pos_x));
            state.support_actual_pos_y = @as(f32, @floatFromInt(remembered_inst.pos_y));
            state.support_actual_pos_z = @as(f32, @floatFromInt(remembered_inst.pos_z));
            state.support_actual_rot_yaw = remembered_inst.rot_yaw;
            return true;
        }
        state.support_instance_idx = -1;
        return false;
    }

    const inst = &s1024.instances[@as(u8, @intCast(support_idx))];
    const inst_pos_x = @as(f32, @floatFromInt(inst.pos_x));
    const inst_pos_y = @as(f32, @floatFromInt(inst.pos_y));
    const inst_pos_z = @as(f32, @floatFromInt(inst.pos_z));
    const pose = if (state.support_instance_idx == support_idx)
        prediction.advanceSupportPose(getSupportPose(state), inst, dt)
    else
        prediction.supportPoseFromInstance(inst);

    if (!carried and state.support_instance_idx == support_idx) {
        state.pos_x += pose.effective_pos_x - state.support_pos_x;
        state.pos_y += pose.effective_pos_y - state.support_pos_y;
        state.pos_z += pose.effective_pos_z - state.support_pos_z;
    } else if (!carried) {
        state.pos_x += if (dt > 0.0) pose.effective_pos_x - inst_pos_x else @as(f32, @floatFromInt(inst.vel_x));
        state.pos_y += if (dt > 0.0) pose.effective_pos_y - inst_pos_y else @as(f32, @floatFromInt(inst.vel_y));
        state.pos_z += if (dt > 0.0) pose.effective_pos_z - inst_pos_z else @as(f32, @floatFromInt(inst.vel_z));
    }

    state.support_instance_idx = support_idx;
    state.support_pos_x = pose.effective_pos_x;
    state.support_pos_y = pose.effective_pos_y;
    state.support_pos_z = pose.effective_pos_z;
    state.support_rot_yaw = pose.actual_rot_yaw;
    state.support_yaw_steps = pose.effective_yaw_steps;
    state.support_actual_pos_x = pose.actual_pos_x;
    state.support_actual_pos_y = pose.actual_pos_y;
    state.support_actual_pos_z = pose.actual_pos_z;
    state.support_actual_rot_yaw = pose.actual_rot_yaw;
    return true;
}

fn getSupportRotationCenter(inst: *const scene32.Instance, entities: []entity16.Entity16) struct { x: f32, z: f32 } {
    if (inst.entity_id >= entities.len) {
        return .{
            .x = @as(f32, @floatFromInt(inst.pos_x)),
            .z = @as(f32, @floatFromInt(inst.pos_z)),
        };
    }

    const bounds = physics.computeEntityLocalAABB(&entities[inst.entity_id]) orelse {
        return .{
            .x = @as(f32, @floatFromInt(inst.pos_x)),
            .z = @as(f32, @floatFromInt(inst.pos_z)),
        };
    };

    return .{
        .x = @as(f32, @floatFromInt(inst.pos_x)) + (@as(f32, @floatFromInt(bounds.min_x + bounds.max_x)) * 0.5),
        .z = @as(f32, @floatFromInt(inst.pos_z)) + (@as(f32, @floatFromInt(bounds.min_z + bounds.max_z)) * 0.5),
    };
}

fn shouldPreventLedgeFall(
    state: *const KCCState,
    old_x: f32,
    old_z: f32,
    carried_by_support: bool,
    config: KCCConfig,
) bool {
    if (!config.prevent_fall_off_ledges) return false;
    if (!state.was_grounded or state.jumping or carried_by_support) return false;
    if (state.grounded) return false;
    if (@abs(state.pos_x - old_x) <= 0.0001 and @abs(state.pos_z - old_z) <= 0.0001) return false;
    return true;
}

fn rangesOverlap(min_a: f32, max_a: f32, min_b: f32, max_b: f32) bool {
    return min_a < max_b and min_b < max_a;
}

fn getHorizontalAvoidancePriority(state: *const KCCState) f32 {
    const speed = @sqrt(state.vel_x * state.vel_x + state.vel_z * state.vel_z);
    const mass_bias = @as(f32, @floatFromInt(state.mass)) * 0.01;
    const grounded_bonus: f32 = if (state.grounded) 0.25 else 0.0;
    const jumping_bonus: f32 = if (state.jumping) -0.15 else 0.0;
    return mass_bias + speed * 0.05 + grounded_bonus + jumping_bonus;
}

fn getCharacterHalfExtents(state: *const KCCState) struct { x: f32, y: f32, z: f32 } {
    const radius = @as(f32, @floatFromInt(state.radius));
    return .{
        .x = radius,
        .y = @as(f32, @floatFromInt(getHeightConst(state))) * 0.5,
        .z = radius,
    };
}

fn applyPredictiveAvoidancePair(a: *KCCState, b: *KCCState, dt: f32) void {
    const a_half_height = @as(f32, @floatFromInt(getHeight(a))) * 0.5;
    const b_half_height = @as(f32, @floatFromInt(getHeight(b))) * 0.5;
    const a_min_y = a.pos_y;
    const a_max_y = a.pos_y + a_half_height * 2.0;
    const b_min_y = b.pos_y;
    const b_max_y = b.pos_y + b_half_height * 2.0;
    if (!rangesOverlap(a_min_y, a_max_y, b_min_y, b_max_y)) return;

    const a_state = prediction.LinearState{
        .pos_x = a.pos_x,
        .pos_y = a.pos_y,
        .pos_z = a.pos_z,
        .vel_x = a.vel_x,
        .vel_y = a.vel_y,
        .vel_z = a.vel_z,
    };
    const b_state = prediction.LinearState{
        .pos_x = b.pos_x,
        .pos_y = b.pos_y,
        .pos_z = b.pos_z,
        .vel_x = b.vel_x,
        .vel_y = b.vel_y,
        .vel_z = b.vel_z,
    };
    const combined_radius = @as(f32, @floatFromInt(a.radius + b.radius));
    const horizon = @max(0.15, dt * 4.0);
    const ttc = prediction.computeTTC(a_state, b_state, combined_radius + 0.25, horizon);
    const a_extents = getCharacterHalfExtents(a);
    const b_extents = getCharacterHalfExtents(b);
    const occupancy_window = prediction.computeOccupancyConflictWindow(
        .{
            .pos_x = a.pos_x,
            .pos_y = a.pos_y + a_extents.y,
            .pos_z = a.pos_z,
            .yaw = a.yaw,
        },
        a.vel_x,
        a.vel_y,
        a.vel_z,
        0.0,
        a_extents.x,
        a_extents.y,
        a_extents.z,
        .{
            .pos_x = b.pos_x,
            .pos_y = b.pos_y + b_extents.y,
            .pos_z = b.pos_z,
            .yaw = b.yaw,
        },
        b.vel_x,
        b.vel_y,
        b.vel_z,
        0.0,
        b_extents.x,
        b_extents.y,
        b_extents.z,
        horizon,
        @max(0.05, dt),
    );
    if ((!ttc.valid or ttc.time > horizon) and (!occupancy_window.valid or occupancy_window.start_time > horizon)) return;

    const dx = b.pos_x - a.pos_x;
    const dz = b.pos_z - a.pos_z;
    const dist_sq = dx * dx + dz * dz;
    const rel_hvel_x = b.vel_x - a.vel_x;
    const rel_hvel_z = b.vel_z - a.vel_z;
    const rel_hspeed = @sqrt(rel_hvel_x * rel_hvel_x + rel_hvel_z * rel_hvel_z);
    const anticipation_radius = combined_radius + @max(1.0, rel_hspeed * horizon);
    if (dist_sq > anticipation_radius * anticipation_radius) return;

    var normal_x: f32 = 1.0;
    var normal_z: f32 = 0.0;
    if (dist_sq > 0.000001) {
        const inv_dist = 1.0 / @sqrt(dist_sq);
        normal_x = dx * inv_dist;
        normal_z = dz * inv_dist;
    }
    const tangent_x = -normal_z;
    const tangent_z = normal_x;
    const side_sign: f32 = if (@intFromPtr(a) < @intFromPtr(b)) 1.0 else -1.0;

    const priority_a = getHorizontalAvoidancePriority(a);
    const priority_b = getHorizontalAvoidancePriority(b);
    const priority_sum = priority_a + priority_b;
    const yield_ratio_a = if (priority_sum > 0.0001) @max(0.15, @min(0.85, priority_b / priority_sum)) else 0.5;
    const yield_ratio_b = if (priority_sum > 0.0001) @max(0.15, @min(0.85, priority_a / priority_sum)) else 0.5;
    const urgency_ttc = if (ttc.valid) @max(0.0, 1.0 - ttc.time / horizon) else 0.0;
    const urgency_occupancy = if (occupancy_window.valid) @max(0.0, 1.0 - occupancy_window.start_time / horizon) else 0.0;
    const urgency = @max(urgency_ttc, urgency_occupancy);
    const side_bias = (0.2 + urgency * 0.6) * side_sign;
    const brake = 0.25 + urgency * 0.5;

    a.vel_x -= tangent_x * side_bias * yield_ratio_a;
    a.vel_z -= tangent_z * side_bias * yield_ratio_a;
    b.vel_x += tangent_x * side_bias * yield_ratio_b;
    b.vel_z += tangent_z * side_bias * yield_ratio_b;

    const closing_speed = (b.vel_x - a.vel_x) * normal_x + (b.vel_z - a.vel_z) * normal_z;
    if (closing_speed < 0.0) {
        a.vel_x += closing_speed * normal_x * brake * yield_ratio_a;
        a.vel_z += closing_speed * normal_z * brake * yield_ratio_a;
        b.vel_x -= closing_speed * normal_x * brake * yield_ratio_b;
        b.vel_z -= closing_speed * normal_z * brake * yield_ratio_b;
    }
}

fn applyPredictiveCharacterAvoidance(sys: *KCCSystem, dt: f32) void {
    var i: usize = 0;
    while (i < sys.count) : (i += 1) {
        var j: usize = i + 1;
        while (j < sys.count) : (j += 1) {
            applyPredictiveAvoidancePair(&sys.characters[i], &sys.characters[j], dt);
        }
    }
}

fn resolveCharacterPair(a: *KCCState, b: *KCCState) void {
    const a_half_height = @as(f32, @floatFromInt(getHeight(a))) * 0.5;
    const b_half_height = @as(f32, @floatFromInt(getHeight(b))) * 0.5;
    const a_min_y = a.pos_y;
    const a_max_y = a.pos_y + a_half_height * 2.0;
    const b_min_y = b.pos_y;
    const b_max_y = b.pos_y + b_half_height * 2.0;
    if (!rangesOverlap(a_min_y, a_max_y, b_min_y, b_max_y)) return;

    const dx = b.pos_x - a.pos_x;
    const dz = b.pos_z - a.pos_z;
    const combined_radius = @as(f32, @floatFromInt(a.radius + b.radius));
    const dist_sq = dx * dx + dz * dz;
    if (dist_sq >= combined_radius * combined_radius) return;

    var normal_x: f32 = 1.0;
    var normal_z: f32 = 0.0;
    if (dist_sq > 0.000001) {
        const inv_dist = 1.0 / @sqrt(dist_sq);
        normal_x = dx * inv_dist;
        normal_z = dz * inv_dist;
    }

    const dist = if (dist_sq > 0.000001) @sqrt(dist_sq) else 0.0;
    const overlap = combined_radius - dist;
    if (overlap <= 0.0) return;

    const inv_mass_a = if (a.mass == 0) 0.0 else 1.0 / @as(f32, @floatFromInt(a.mass));
    const inv_mass_b = if (b.mass == 0) 0.0 else 1.0 / @as(f32, @floatFromInt(b.mass));
    const inv_mass_sum = inv_mass_a + inv_mass_b;
    const move_ratio_a = if (inv_mass_sum > 0.0) inv_mass_a / inv_mass_sum else 0.5;
    const move_ratio_b = if (inv_mass_sum > 0.0) inv_mass_b / inv_mass_sum else 0.5;
    const separation = overlap + 0.01;
    const tangent_x = -normal_z;
    const tangent_z = normal_x;

    a.pos_x -= normal_x * separation * move_ratio_a;
    a.pos_z -= normal_z * separation * move_ratio_a;
    b.pos_x += normal_x * separation * move_ratio_b;
    b.pos_z += normal_z * separation * move_ratio_b;

    const tangential_offset = dx * tangent_x + dz * tangent_z;
    if (@abs(tangential_offset) < 0.01) {
        const side_sign: f32 = if (@intFromPtr(a) < @intFromPtr(b)) 1.0 else -1.0;
        const side_sep = @min(0.125, overlap * 0.25);
        a.pos_x -= tangent_x * side_sep * side_sign * move_ratio_a;
        a.pos_z -= tangent_z * side_sep * side_sign * move_ratio_a;
        b.pos_x += tangent_x * side_sep * side_sign * move_ratio_b;
        b.pos_z += tangent_z * side_sep * side_sign * move_ratio_b;
    }

    const relative_vel = (b.vel_x - a.vel_x) * normal_x + (b.vel_z - a.vel_z) * normal_z;
    if (relative_vel < 0.0) {
        const correction = relative_vel;
        a.vel_x += correction * normal_x * move_ratio_a;
        a.vel_z += correction * normal_z * move_ratio_a;
        b.vel_x -= correction * normal_x * move_ratio_b;
        b.vel_z -= correction * normal_z * move_ratio_b;

        const tangential_vel = (b.vel_x - a.vel_x) * tangent_x + (b.vel_z - a.vel_z) * tangent_z;
        if (@abs(tangential_vel) < 0.05 and @abs(relative_vel) > 0.1) {
            const side_sign: f32 = if (@intFromPtr(a) < @intFromPtr(b)) 1.0 else -1.0;
            const side_bias = @min(0.5, @abs(relative_vel) * 0.25);
            a.vel_x -= tangent_x * side_bias * side_sign * move_ratio_a;
            a.vel_z -= tangent_z * side_bias * side_sign * move_ratio_a;
            b.vel_x += tangent_x * side_bias * side_sign * move_ratio_b;
            b.vel_z += tangent_z * side_bias * side_sign * move_ratio_b;
        }
    }
}

fn resolveCharacterCharacterCollisions(sys: *KCCSystem) void {
    var iteration: u8 = 0;
    while (iteration < 4) : (iteration += 1) {
        var i: usize = 0;
        while (i < sys.count) : (i += 1) {
            var j: usize = i + 1;
            while (j < sys.count) : (j += 1) {
                resolveCharacterPair(&sys.characters[i], &sys.characters[j]);
            }
        }
    }
}

/// Check collision at position with character volume
pub fn checkCollision(
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    height: i32,
    radius: i32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    const px = @as(i32, @intFromFloat(@floor(pos_x)));
    const py = @as(i32, @intFromFloat(@floor(pos_y)));
    const pz = @as(i32, @intFromFloat(@floor(pos_z)));

    const world_view = query.QueryWorldView{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };

    const filter = query.QueryFilter{
        .include_static = true,
        .include_dynamic = true,
        .include_kinematic = true,
        .include_sensors = false,
    };

    var y: i32 = 0;
    while (y < height) : (y += 2) {
        if (query.queryAnyVoxel(&world_view, px, py + y, pz, filter).hit) {
            return true;
        }
        var x: i32 = -radius;
        while (x <= radius) : (x += 2) {
            var z: i32 = -radius;
            while (z <= radius) : (z += 2) {
                if (query.queryAnyVoxel(&world_view, px + x, py + y, pz + z, filter).hit) {
                    return true;
                }
            }
        }
    }
    return false;
}

/// Slide along walls
pub fn slideAlongWall(
    vel_x: f32,
    vel_z: f32,
    normal_x: f32,
    normal_z: f32,
) struct { x: f32, z: f32 } {
    const dot = vel_x * normal_x + vel_z * normal_z;
    return .{
        .x = vel_x - dot * normal_x,
        .z = vel_z - dot * normal_z,
    };
}

fn slideVelocityAlongSurface3D(
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
) struct { x: f32, y: f32, z: f32 } {
    const dot = vel_x * normal_x + vel_y * normal_y + vel_z * normal_z;
    if (dot >= 0.0) {
        return .{
            .x = vel_x,
            .y = vel_y,
            .z = vel_z,
        };
    }
    return .{
        .x = vel_x - dot * normal_x,
        .y = vel_y - dot * normal_y,
        .z = vel_z - dot * normal_z,
    };
}

fn computeCharacterCapsulePenetration(
    state: *const KCCState,
    height: i32,
    radius: i32,
    world_view: *const query_types.QueryWorldView,
    filter: query_types.QueryFilter,
) query_types.PenetrationResult {
    const radius_f = @as(f32, @floatFromInt(radius));
    const height_f = @as(f32, @floatFromInt(height));
    const cylinder_half_height = @max(0.0, height_f * 0.5 - radius_f);

    return query_penetration.computePenetrationCapsule(
        world_view,
        state.pos_x,
        state.pos_y + height_f * 0.5,
        state.pos_z,
        radius_f,
        cylinder_half_height,
        filter,
    );
}

fn enforceSlopeLimit(state: *KCCState, config: KCCConfig) void {
    if (!state.grounded) return;

    const max_angle_radians = config.max_slope_angle * std.math.pi / 180.0;
    const min_ground_y = @cos(max_angle_radians);
    if (state.ground_normal_y >= min_ground_y) return;

    const horizontal_len_sq = state.ground_normal_x * state.ground_normal_x +
        state.ground_normal_z * state.ground_normal_z;
    if (horizontal_len_sq <= 0.000001) return;

    const inv_horizontal_len = 1.0 / @sqrt(horizontal_len_sq);
    const downhill_x = state.ground_normal_x * inv_horizontal_len;
    const downhill_z = state.ground_normal_z * inv_horizontal_len;

    const downhill_component = state.vel_x * downhill_x + state.vel_z * downhill_z;
    if (downhill_component < 0) {
        state.vel_x -= downhill_component * downhill_x;
        state.vel_z -= downhill_component * downhill_z;
    } else if (@abs(state.vel_x) + @abs(state.vel_z) < 0.001) {
        const slope_factor = @min(1.0, (min_ground_y - state.ground_normal_y) / @max(min_ground_y, 0.0001));
        const slide_speed = @max(1.0, config.move_speed * 0.05) * slope_factor;
        state.vel_x += downhill_x * slide_speed;
        state.vel_z += downhill_z * slide_speed;
    }
}

fn clampUphillDisplacementOnSteepSlope(
    state: *KCCState,
    prev_x: f32,
    prev_z: f32,
    config: KCCConfig,
) void {
    if (!state.grounded) return;

    const max_angle_radians = config.max_slope_angle * std.math.pi / 180.0;
    const min_ground_y = @cos(max_angle_radians);
    if (state.ground_normal_y >= min_ground_y) return;

    const horizontal_len_sq = state.ground_normal_x * state.ground_normal_x +
        state.ground_normal_z * state.ground_normal_z;
    if (horizontal_len_sq <= 0.000001) return;

    const inv_horizontal_len = 1.0 / @sqrt(horizontal_len_sq);
    const downhill_x = state.ground_normal_x * inv_horizontal_len;
    const downhill_z = state.ground_normal_z * inv_horizontal_len;
    const uphill_x = -downhill_x;
    const uphill_z = -downhill_z;

    const move_x = state.pos_x - prev_x;
    const move_z = state.pos_z - prev_z;
    const uphill_displacement = move_x * uphill_x + move_z * uphill_z;
    if (uphill_displacement <= 0.0) return;

    state.pos_x -= uphill_x * uphill_displacement;
    state.pos_z -= uphill_z * uphill_displacement;
}

fn tryStepUpFromHorizontalBlockage(
    state: *KCCState,
    height: i32,
    radius: i32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    config: KCCConfig,
    penetration: query_types.PenetrationResult,
) bool {
    if (config.step_height <= 0) return false;
    if (!state.grounded) return false;
    if (penetration.dir_x == 0 and penetration.dir_z == 0) return false;

    var step: i32 = 1;
    while (step <= config.step_height) : (step += 1) {
        if (!checkCollision(state.pos_x, state.pos_y + @as(f32, @floatFromInt(step)), state.pos_z, height, radius, s1024, entities)) {
            state.pos_y += @as(f32, @floatFromInt(step));
            state.vel_y = 0;
            state.grounded = true;
            state.jumping = false;
            return true;
        }
    }

    return false;
}

/// Apply gravity and update velocity
pub fn applyGravity(state: *KCCState, dt: f32, config: KCCConfig) void {
    if (!state.grounded) {
        state.vel_y += config.gravity * dt;
        if (state.vel_y < config.gravity * 2.0) {
            state.vel_y = config.gravity * 2.0;
        }
    }
}

/// Resolve collision and update position
pub fn resolveCollision(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    config: KCCConfig,
) void {
    const height = getHeight(state);
    const radius = state.radius;
    const old_x = state.pos_x;
    const old_y = state.pos_y;
    const old_z = state.pos_z;
    const world_view = query_types.QueryWorldView{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const filter = query_types.QueryFilter{
        .include_static = true,
        .include_dynamic = true,
        .include_kinematic = true,
        .include_sensors = false,
    };

    const overlapping_aabb = checkCollision(state.pos_x, state.pos_y, state.pos_z, height, radius, s1024, entities);
    const capsule_penetration = computeCharacterCapsulePenetration(state, height, radius, &world_view, filter);
    const ceiling_hit = state.vel_y > 0 and capsule_penetration.overlapping and capsule_penetration.dir_y < -0.0001;

    if (!overlapping_aabb and !ceiling_hit) return;

    const penetration = if (overlapping_aabb)
        query_penetration.computePenetrationAABB(
            &world_view,
            state.pos_x - @as(f32, @floatFromInt(radius)),
            state.pos_y,
            state.pos_z - @as(f32, @floatFromInt(radius)),
            state.pos_x + @as(f32, @floatFromInt(radius)),
            state.pos_y + @as(f32, @floatFromInt(height)),
            state.pos_z + @as(f32, @floatFromInt(radius)),
            filter,
        )
    else
        query_types.PenetrationResult{};

    if (penetration.overlapping and penetration.depth > 0) {
        if (tryStepUpFromHorizontalBlockage(state, height, radius, s1024, entities, config, penetration)) {
            return;
        }
    }

    if (ceiling_hit) {
        const depenetration = capsule_penetration.depth + 0.05;
        state.pos_x += capsule_penetration.dir_x * depenetration;
        state.pos_y += capsule_penetration.dir_y * depenetration;
        state.pos_z += capsule_penetration.dir_z * depenetration;

        const slid = slideVelocityAlongSurface3D(
            state.vel_x,
            state.vel_y,
            state.vel_z,
            capsule_penetration.dir_x,
            capsule_penetration.dir_y,
            capsule_penetration.dir_z,
        );
        state.vel_x = slid.x;
        state.vel_y = @min(slid.y, 0.0);
        state.vel_z = slid.z;
    } else if (penetration.overlapping and penetration.depth > 0) {
        const depenetration = penetration.depth + 0.05;
        state.pos_x += penetration.dir_x * depenetration;
        state.pos_y += penetration.dir_y * depenetration;
        state.pos_z += penetration.dir_z * depenetration;

        if (penetration.dir_y > 0) {
            state.vel_y = 0;
            state.grounded = true;
            state.jumping = false;
        } else if (penetration.dir_y < 0 and state.vel_y > 0) {
            state.vel_y = 0;
        }

        if (penetration.dir_x != 0 or penetration.dir_z != 0) {
            const slid = slideAlongWall(state.vel_x, state.vel_z, penetration.dir_x, penetration.dir_z);
            state.vel_x = slid.x;
            state.vel_z = slid.z;
        }
    }

    if (checkCollision(state.pos_x, state.pos_y, state.pos_z, height, radius, s1024, entities)) {
        var attempts: u8 = 0;
        const max_step_attempts: u8 = @intCast(@max(config.step_height, 1));
        while (attempts < max_step_attempts) : (attempts += 1) {
            state.pos_y += 1;
            if (!checkCollision(state.pos_x, state.pos_y, state.pos_z, height, radius, s1024, entities)) {
                break;
            }
        }

        if (checkCollision(state.pos_x, state.pos_y, state.pos_z, height, radius, s1024, entities)) {
            state.pos_x = old_x;
            state.pos_y = old_y;
            state.pos_z = old_z;
            state.vel_x = 0;
            state.vel_z = 0;
            state.vel_y = 0;
        }
    }

    if (checkCollision(state.pos_x, state.pos_y, state.pos_z, height, radius, s1024, entities)) {
        state.pos_x = old_x;
        state.vel_y = 0;
    }
}

/// Push rigid bodies
pub fn pushNearbyBodies(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    config: KCCConfig,
) void {
    const px = @as(i32, @intFromFloat(@floor(state.pos_x)));
    const py = @as(i32, @intFromFloat(@floor(state.pos_y)));
    const pz = @as(i32, @intFromFloat(@floor(state.pos_z)));
    const height = getHeight(state);
    const radius = state.radius;

    var i: u8 = 0;
    while (i < s1024.instance_count) : (i += 1) {
        const inst = &s1024.instances[i];
        if (inst.entity_id >= entities.len) continue;

        const ent = &entities[inst.entity_id];
        if ((ent.physics.flags & 0x01) != 0) continue;
        if (ent.physics.mass == 0) continue;

        const dx = @as(i32, inst.pos_x) - px;
        const dy = @as(i32, inst.pos_y) - py;
        const dz = @as(i32, inst.pos_z) - pz;

        if (dx >= -radius - 16 and dx <= radius + 16 and
            dy >= -height and dy <= height and
            dz >= -radius - 16 and dz <= radius + 16)
        {
            const dist_sq = dx * dx + dy * dy + dz * dz;
            const dist_sq_f = @as(f32, @floatFromInt(dist_sq));
            const push_dist = @as(f32, @floatFromInt(radius + 8));
            if (dist_sq_f < push_dist * push_dist and dist_sq > 0) {
                const dist = @sqrt(dist_sq_f);
                const nx = @as(f32, @floatFromInt(dx)) / dist;
                const nz = @as(f32, @floatFromInt(dz)) / dist;
                const push_impulse = config.push_force / @as(f32, @floatFromInt(ent.physics.mass));

                inst.vel_x = @truncate(@as(i32, @intFromFloat(@round(nx * push_impulse))));
                inst.vel_z = @truncate(@as(i32, @intFromFloat(@round(nz * push_impulse))));
            }
        }
    }
}

/// Main update function - call each tick
pub fn update(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    config: KCCConfig,
    dt: f32,
) void {
    state.was_grounded = state.grounded;
    const carried_by_support = carryAlongSupportingKinematic(state, s1024, entities, dt);
    const old_x = state.pos_x;
    const old_z = state.pos_z;
    var prevented_ledge_fall = false;

    applyGravity(state, dt, config);

    state.pos_x += state.vel_x * dt;
    state.pos_z += state.vel_z * dt;
    state.pos_y += state.vel_y * dt;

    state.grounded = if (carried_by_support) true else checkGrounded(state, s1024, entities);
    if (state.grounded and state.vel_y < 0) {
        state.vel_y = 0;
        state.jumping = false;
    } else if (!carried_by_support) {
        trySnapToGround(state, s1024, entities, config);
    }
    if (!state.grounded and !carried_by_support) {
        state.support_instance_idx = -1;
    }

    if (shouldPreventLedgeFall(state, old_x, old_z, carried_by_support, config)) {
        state.pos_x = old_x;
        state.pos_z = old_z;
        state.vel_x = 0;
        state.vel_z = 0;
        state.grounded = true;
        prevented_ledge_fall = true;
    }

    resolveCollision(state, s1024, entities, config);
    pushNearbyBodies(state, s1024, entities, config);

    // Update ground normal
    if (state.grounded and !prevented_ledge_fall) {
        const gn = getGroundNormal(state, s1024, entities);
        state.ground_normal_x = gn.x;
        state.ground_normal_y = gn.y;
        state.ground_normal_z = gn.z;
        clampUphillDisplacementOnSteepSlope(state, old_x, old_z, config);
        enforceSlopeLimit(state, config);
    } else if (prevented_ledge_fall) {
        state.ground_normal_x = 0;
        state.ground_normal_y = 1;
        state.ground_normal_z = 0;
    }
}

/// Check if character can fit through a gap
pub fn canFitThrough(
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    height: i32,
    radius: i32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    return !checkCollision(pos_x, pos_y, pos_z, height, radius, s1024, entities);
}

/// Get system for external iteration
pub fn getSystem() *KCCSystem {
    return &g_kcc_system;
}

pub fn getConfig(state: *const KCCState) KCCConfig {
    return .{
        .move_speed = state.move_speed,
        .jump_force = state.jump_force,
        .gravity = state.gravity,
        .crouch_speed_mult = state.crouch_speed_mult,
        .push_force = state.push_force,
        .step_height = state.step_height,
        .stand_height = state.stand_height,
        .crouch_height = state.crouch_height,
        .radius = state.radius,
        .max_slope_angle = state.max_slope_angle,
        .step_offset = state.step_offset,
        .prevent_fall_off_ledges = state.prevent_fall_off_ledges,
    };
}

pub fn updateSystem(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    dt: f32,
) void {
    const sys = getSystem();
    applyPredictiveCharacterAvoidance(sys, dt);
    var i: u8 = 0;
    while (i < sys.count) : (i += 1) {
        const state = &sys.characters[i];
        update(state, s1024, entities, getConfig(state), dt);
    }
    resolveCharacterCharacterCollisions(sys);
}

test "KCC checkGrounded samples full support area instead of one diagonal" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var floor = entity16.initEntity16();
    floor.physics.flags |= 0x01;
    entity16.setVoxel(&floor, 0, 0, 0);

    var entities = [_]entity16.Entity16{floor};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 9,
        .pos_y = 9,
        .pos_z = 11,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(10.0, 10.0, 10.0, .{
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 2,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    try std.testing.expect(checkGrounded(char, &s1024, entities[0..]));
}

test "KCC getGroundNormal tilts away from supported uphill side" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};

    const addr = @import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 11,
        .ly = 9,
        .lz = 10,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    const char = createCharacter(10.0, 10.0, 10.0, .{
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    const normal = getGroundNormal(char, &s1024, entities[0..]);
    try std.testing.expect(normal.x < 0);
    try std.testing.expect(normal.y > 0.5);
}

test "KCC update snaps small downhill gaps back to grounded state" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    try s1024.setVoxelAtGlobal(@import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 9,
        .lz = 10,
    }), true);

    const char = createCharacter(10.0, 11.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
        .step_offset = 0.25,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = false;
    char.vel_y = -1.0;
    const before_y = char.pos_y;

    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    try std.testing.expect(char.grounded);
    try std.testing.expect(char.pos_y < before_y);
    try std.testing.expect(char.pos_y >= before_y - 0.25 - 0.0001);
}

test "KCC update carries grounded character with kinematic support platform" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 11.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    const before_x = char.pos_x;

    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    try std.testing.expect(char.grounded);
    try std.testing.expect(char.pos_x > before_x);
    try std.testing.expectApproxEqAbs(@as(f32, before_x + 10.0), char.pos_x, 0.0001);
}

test "KCC update preserves relative offset on moved support platform across frames" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 11.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const initial_offset_x = char.pos_x - @as(f32, @floatFromInt(s1024.instances[0].pos_x));
    const initial_offset_z = char.pos_z - @as(f32, @floatFromInt(s1024.instances[0].pos_z));

    s1024.instances[0].pos_x += 3;
    s1024.instances[0].pos_z += 2;

    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(initial_offset_x, char.pos_x - @as(f32, @floatFromInt(s1024.instances[0].pos_x)), 0.0001);
    try std.testing.expectApproxEqAbs(initial_offset_z, char.pos_z - @as(f32, @floatFromInt(s1024.instances[0].pos_z)), 0.0001);
}

test "KCC update rotates relative offset with supporting platform yaw" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    s1024.instances[0].rot_yaw = 64;

    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(@as(f32, 14.0), char.pos_x, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, 11.0), char.pos_z, 0.05);
}

test "KCC update rotates support across yaw wrap without large reverse jump" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 255,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const center_before = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    const rel_x_before = char.pos_x - center_before.x;
    const rel_z_before = char.pos_z - center_before.z;

    s1024.instances[0].rot_yaw = 1;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const expected_yaw_delta = @as(f32, 2.0) * (std.math.tau / 256.0);
    const center_after = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(center_after.x + rel_x_before * @cos(expected_yaw_delta) - rel_z_before * @sin(expected_yaw_delta), char.pos_x, 0.05);
    try std.testing.expectApproxEqAbs(center_after.z + rel_x_before * @sin(expected_yaw_delta) + rel_z_before * @cos(expected_yaw_delta), char.pos_z, 0.05);
}

test "KCC update follows support angular velocity before yaw step quantizes" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const center_before = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    const rel_x_before = char.pos_x - center_before.x;
    const rel_z_before = char.pos_z - center_before.z;
    const dt: f32 = 0.5;
    const expected_yaw_delta = (@as(f32, @floatFromInt(s1024.instances[0].ang_y)) / 10.0) * dt * (std.math.tau / 256.0);

    update(char, &s1024, entities[0..], getConfig(char), dt);

    const center_after = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(center_after.x + rel_x_before * @cos(expected_yaw_delta) - rel_z_before * @sin(expected_yaw_delta), char.pos_x, 0.02);
    try std.testing.expectApproxEqAbs(center_after.z + rel_x_before * @sin(expected_yaw_delta) + rel_z_before * @cos(expected_yaw_delta), char.pos_z, 0.02);
}

test "KCC update does not predict support angular carry when dt is zero" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const before_x = char.pos_x;
    const before_z = char.pos_z;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    try std.testing.expectApproxEqAbs(before_x, char.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(before_z, char.pos_z, 0.0001);
}

test "KCC update follows support linear velocity before position step quantizes" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = -2,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 11.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const before_x = char.pos_x;
    const before_z = char.pos_z;
    const dt: f32 = 0.25;
    update(char, &s1024, entities[0..], getConfig(char), dt);

    try std.testing.expectApproxEqAbs(before_x + 1.0, char.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(before_z - 0.5, char.pos_z, 0.0001);
}

test "KCC update does not double-apply support velocity when platform position already advanced" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = -2,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 11.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const offset_x = char.pos_x - @as(f32, @floatFromInt(s1024.instances[0].pos_x));
    const offset_z = char.pos_z - @as(f32, @floatFromInt(s1024.instances[0].pos_z));
    s1024.instances[0].pos_x += 1;
    s1024.instances[0].pos_z -= 1;

    update(char, &s1024, entities[0..], getConfig(char), 0.25);

    try std.testing.expectApproxEqAbs(offset_x, char.pos_x - @as(f32, @floatFromInt(s1024.instances[0].pos_x)), 0.0001);
    try std.testing.expectApproxEqAbs(offset_z, char.pos_z - @as(f32, @floatFromInt(s1024.instances[0].pos_z)), 0.0001);
}

test "KCC update follows support vertical velocity before position step quantizes" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 4,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 11.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const before_y = char.pos_y;
    update(char, &s1024, entities[0..], getConfig(char), 0.25);

    try std.testing.expectApproxEqAbs(before_y + 1.0, char.pos_y, 0.0001);
    try std.testing.expect(char.grounded);
}

test "KCC update combines predicted support translation and rotation before quantization" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 2,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const center_before = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    const rel_x_before = char.pos_x - center_before.x;
    const rel_z_before = char.pos_z - center_before.z;
    const dt: f32 = 0.25;
    const predicted_dx = @as(f32, @floatFromInt(s1024.instances[0].vel_x)) * dt;
    const predicted_dz = @as(f32, @floatFromInt(s1024.instances[0].vel_z)) * dt;
    const predicted_yaw_delta = (@as(f32, @floatFromInt(s1024.instances[0].ang_y)) / 10.0) * dt * (std.math.tau / 256.0);

    update(char, &s1024, entities[0..], getConfig(char), dt);

    const center_after = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(center_after.x + predicted_dx + rel_x_before * @cos(predicted_yaw_delta) - rel_z_before * @sin(predicted_yaw_delta), char.pos_x, 0.02);
    try std.testing.expectApproxEqAbs(center_after.z + predicted_dz + rel_x_before * @sin(predicted_yaw_delta) + rel_z_before * @cos(predicted_yaw_delta), char.pos_z, 0.02);
}

test "KCC update does not overshoot when real support position catches up after prediction" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 11.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const initial_offset_x = char.pos_x - @as(f32, @floatFromInt(s1024.instances[0].pos_x));
    const initial_offset_z = char.pos_z - @as(f32, @floatFromInt(s1024.instances[0].pos_z));

    update(char, &s1024, entities[0..], getConfig(char), 0.25);
    update(char, &s1024, entities[0..], getConfig(char), 0.25);

    s1024.instances[0].pos_x = 12;
    update(char, &s1024, entities[0..], getConfig(char), 0.25);

    try std.testing.expectApproxEqAbs(initial_offset_x, char.pos_x - @as(f32, @floatFromInt(s1024.instances[0].pos_x)), 0.0001);
    try std.testing.expectApproxEqAbs(initial_offset_z, char.pos_z - @as(f32, @floatFromInt(s1024.instances[0].pos_z)), 0.0001);
}

test "KCC update does not overshoot when real support yaw catches up after prediction" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const center_initial = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    const initial_rel_x = char.pos_x - center_initial.x;
    const initial_rel_z = char.pos_z - center_initial.z;

    update(char, &s1024, entities[0..], getConfig(char), 0.5);
    update(char, &s1024, entities[0..], getConfig(char), 0.5);

    s1024.instances[0].rot_yaw = 1;
    update(char, &s1024, entities[0..], getConfig(char), 0.25);

    const center_after = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    const expected_yaw_delta = @as(f32, 1.0) * (std.math.tau / 256.0);
    try std.testing.expectApproxEqAbs(center_after.x + initial_rel_x * @cos(expected_yaw_delta) - initial_rel_z * @sin(expected_yaw_delta), char.pos_x, 0.02);
    try std.testing.expectApproxEqAbs(center_after.z + initial_rel_x * @sin(expected_yaw_delta) + initial_rel_z * @cos(expected_yaw_delta), char.pos_z, 0.02);
}

test "KCC update follows negative support translation and yaw prediction" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = -4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = -5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    const center_before = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    const rel_x_before = char.pos_x - center_before.x;
    const rel_z_before = char.pos_z - center_before.z;
    const dt: f32 = 0.25;
    const predicted_dx = @as(f32, @floatFromInt(s1024.instances[0].vel_x)) * dt;
    const predicted_yaw_delta = (@as(f32, @floatFromInt(s1024.instances[0].ang_y)) / 10.0) * dt * (std.math.tau / 256.0);

    update(char, &s1024, entities[0..], getConfig(char), dt);

    const center_after = getSupportRotationCenter(&s1024.instances[0], entities[0..]);
    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(center_after.x + predicted_dx + rel_x_before * @cos(predicted_yaw_delta) - rel_z_before * @sin(predicted_yaw_delta), char.pos_x, 0.02);
    try std.testing.expectApproxEqAbs(center_after.z + rel_x_before * @sin(predicted_yaw_delta) + rel_z_before * @cos(predicted_yaw_delta), char.pos_z, 0.02);
}

test "KCC update combines support translation and yaw without double-applying translation" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var platform = entity16.initEntity16();
    platform.physics.flags |= 0x02;
    entity16.fillBox(&platform, 0, 0, 0, 3, 0, 3);

    var entities = [_]entity16.Entity16{platform};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const char = createCharacter(11.0, 10.0, 10.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    s1024.instances[0].pos_x += 3;
    s1024.instances[0].pos_z += 2;
    s1024.instances[0].rot_yaw = 64;

    update(char, &s1024, entities[0..], getConfig(char), 0.0);

    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(@as(f32, 17.0), char.pos_x, 0.05);
    try std.testing.expectApproxEqAbs(@as(f32, 13.0), char.pos_z, 0.05);
}

test "KCC support prediction helper prefers prediction until actual support pose changes" {
    var state = KCCState{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .grounded = true,
        .crouching = false,
        .jumping = false,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
        .mass = 80,
        .move_speed = 200,
        .jump_force = 350,
        .gravity = -800,
        .crouch_speed_mult = 0.5,
        .push_force = 100,
        .step_height = 1,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = true,
        .ground_normal_x = 0,
        .ground_normal_y = 1,
        .ground_normal_z = 0,
        .support_instance_idx = 0,
        .support_pos_x = 11.0,
        .support_pos_y = 9.0,
        .support_pos_z = 10.0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0.5,
        .support_actual_pos_x = 10.0,
        .support_actual_pos_y = 9.0,
        .support_actual_pos_z = 10.0,
        .support_actual_rot_yaw = 0,
    };

    const inst_predicted = scene32.Instance{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const predicted_pose = prediction.advanceSupportPose(getSupportPose(&state), &inst_predicted, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), predicted_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.625), predicted_pose.effective_yaw_steps, 0.0001);

    const inst_actual = scene32.Instance{
        .entity_id = 0,
        .pos_x = 12,
        .pos_y = 9,
        .pos_z = 10,
        .rot_yaw = 1,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 5,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const actual_pose = prediction.advanceSupportPose(getSupportPose(&state), &inst_actual, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), actual_pose.effective_pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), actual_pose.effective_yaw_steps, 0.0001);
}

test "KCC enforceSlopeLimit cancels uphill motion on steep slope" {
    var state = KCCState{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .grounded = true,
        .crouching = false,
        .jumping = false,
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
        .mass = 80,
        .move_speed = 200,
        .jump_force = 350,
        .gravity = -800,
        .crouch_speed_mult = 0.5,
        .push_force = 100,
        .step_height = 2,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = true,
        .ground_normal_x = -0.8,
        .ground_normal_y = 0.6,
        .ground_normal_z = 0.0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };

    enforceSlopeLimit(&state, getConfig(&state));

    try std.testing.expectApproxEqAbs(@as(f32, 0.0), state.vel_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), state.vel_z, 0.0001);
}

test "KCC enforceSlopeLimit adds downhill slide on steep slope when idle" {
    var state = KCCState{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .grounded = true,
        .crouching = false,
        .jumping = false,
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
        .mass = 80,
        .move_speed = 200,
        .jump_force = 350,
        .gravity = -800,
        .crouch_speed_mult = 0.5,
        .push_force = 100,
        .step_height = 2,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = true,
        .ground_normal_x = -0.8,
        .ground_normal_y = 0.6,
        .ground_normal_z = 0.0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };

    enforceSlopeLimit(&state, getConfig(&state));

    try std.testing.expect(state.vel_x < 0);
}

test "KCC clampUphillDisplacementOnSteepSlope removes uphill movement on steep ground" {
    var state = KCCState{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 10,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .grounded = true,
        .crouching = false,
        .jumping = false,
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
        .mass = 80,
        .move_speed = 200,
        .jump_force = 350,
        .gravity = -800,
        .crouch_speed_mult = 0.5,
        .push_force = 100,
        .step_height = 2,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = true,
        .ground_normal_x = -0.8,
        .ground_normal_y = 0.6,
        .ground_normal_z = 0.0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };

    state.pos_x = 2.0;
    clampUphillDisplacementOnSteepSlope(&state, 0.0, 0.0, getConfig(&state));

    try std.testing.expectApproxEqAbs(@as(f32, 0.0), state.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), state.pos_z, 0.0001);
}

test "KCC update blocks uphill displacement on sampled steep support" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    try s1024.setVoxelAtGlobal(@import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 9,
        .lz = 10,
    }), true);
    try s1024.setVoxelAtGlobal(@import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 9,
        .ly = 9,
        .lz = 10,
    }), true);
    try s1024.setVoxelAtGlobal(@import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 9,
        .lz = 9,
    }), true);

    const char = createCharacter(10.9, 10.0, 10.9, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 2,
        .max_slope_angle = 45.0,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    char.vel_x = -10.0;
    char.vel_z = -10.0;
    const before_x = char.pos_x;
    const before_z = char.pos_z;

    update(char, &s1024, entities[0..], getConfig(char), 0.04);

    try std.testing.expect(char.grounded);
    try std.testing.expect(char.pos_x >= before_x - 0.05);
    try std.testing.expect(char.pos_z >= before_z - 0.05);
}

test "KCC trySetCrouch refuses uncrouch when ceiling blocks standing height" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    try s1024.setVoxelAtGlobal(@import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 6,
        .lz = 0,
    }), true);

    var entities = [_]entity16.Entity16{};
    const char = createCharacter(0.0, 0.0, 0.0, .{
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    try std.testing.expect(trySetCrouch(char, true, &s1024, entities[0..]));
    try std.testing.expect(char.crouching);
    try std.testing.expect(!trySetCrouch(char, false, &s1024, entities[0..]));
    try std.testing.expect(char.crouching);
}

test "KCC trySetCrouch allows uncrouch when standing volume is clear" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const char = createCharacter(10.0, 10.0, 10.0, .{
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    try std.testing.expect(trySetCrouch(char, true, &s1024, entities[0..]));
    try std.testing.expect(trySetCrouch(char, false, &s1024, entities[0..]));
    try std.testing.expect(!char.crouching);
}

test "KCC resolveCollision can step up obstacle within step height" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var step = entity16.initEntity16();
    step.physics.flags = 0x01;
    entity16.fillBox(&step, 0, 0, 0, 0, 0, 3);

    var entities = [_]entity16.Entity16{step};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var state = KCCState{
        .pos_x = 1.2,
        .pos_y = 0.0,
        .pos_z = 1.0,
        .vel_x = -10.0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .grounded = true,
        .crouching = false,
        .jumping = false,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
        .mass = 80,
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 1,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = true,
        .ground_normal_x = 0,
        .ground_normal_y = 1,
        .ground_normal_z = 0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };

    resolveCollision(&state, &s1024, entities[0..], getConfig(&state));

    try std.testing.expect(state.pos_y > 0.5);
}

test "KCC resolveCollision blocks obstacle taller than step height" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var step = entity16.initEntity16();
    step.physics.flags = 0x01;
    entity16.fillBox(&step, 0, 0, 0, 0, 2, 3);

    var entities = [_]entity16.Entity16{step};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var state = KCCState{
        .pos_x = 1.2,
        .pos_y = 0.0,
        .pos_z = 1.0,
        .vel_x = -10.0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .grounded = true,
        .crouching = false,
        .jumping = false,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
        .mass = 80,
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 1,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = true,
        .ground_normal_x = 0,
        .ground_normal_y = 1,
        .ground_normal_z = 0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };

    resolveCollision(&state, &s1024, entities[0..], getConfig(&state));

    try std.testing.expectApproxEqAbs(@as(f32, 0.0), state.pos_y, 0.0001);
    try std.testing.expect(state.pos_x > 1.2);
}

test "KCC resolveCollision depenetrates sideways instead of only lifting" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var wall = entity16.initEntity16();
    wall.physics.flags = 0x01;
    entity16.fillBox(&wall, 0, 0, 0, 0, 15, 15);

    var entities = [_]entity16.Entity16{wall};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var state = KCCState{
        .pos_x = 1.2,
        .pos_y = 1.0,
        .pos_z = 8.0,
        .vel_x = -10.0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .grounded = false,
        .crouching = false,
        .jumping = false,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
        .mass = 80,
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 1,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = false,
        .ground_normal_x = 0,
        .ground_normal_y = 1,
        .ground_normal_z = 0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };

    resolveCollision(&state, &s1024, entities[0..], .{ .stand_height = 4, .crouch_height = 2, .radius = 1, .step_height = 1 });

    try std.testing.expect(state.pos_x > 1.2);
}

test "KCC resolveCollision slides along sloped ceiling instead of killing tangent motion" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    try s1024.setVoxelAtGlobal(@import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 2,
        .ly = 6,
        .lz = 1,
    }), true);

    var entities = [_]entity16.Entity16{};
    var state = KCCState{
        .pos_x = 1.35,
        .pos_y = 2.6,
        .pos_z = 1.5,
        .vel_x = 8.0,
        .vel_y = 6.0,
        .vel_z = 0.0,
        .yaw = 0,
        .grounded = false,
        .crouching = false,
        .jumping = true,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
        .mass = 80,
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 1,
        .max_slope_angle = 45.0,
        .step_offset = 0.25,
        .prevent_fall_off_ledges = false,
        .was_grounded = false,
        .ground_normal_x = 0,
        .ground_normal_y = 1,
        .ground_normal_z = 0,
        .support_instance_idx = -1,
        .support_pos_x = 0,
        .support_pos_y = 0,
        .support_pos_z = 0,
        .support_rot_yaw = 0,
        .support_yaw_steps = 0,
        .support_actual_pos_x = 0,
        .support_actual_pos_y = 0,
        .support_actual_pos_z = 0,
        .support_actual_rot_yaw = 0,
    };

    resolveCollision(&state, &s1024, entities[0..], getConfig(&state));

    try std.testing.expect(state.vel_y <= 0.0001);
    try std.testing.expect(@abs(state.vel_x) > 0.25 or @abs(state.vel_z) > 0.25);
    try std.testing.expect(state.pos_y < 2.6);
    try std.testing.expect(!checkCollision(state.pos_x, state.pos_y, state.pos_z, getHeight(&state), state.radius, &s1024, entities[0..]));
}

test "KCC update prevents walking off ledge when ledge constraint is enabled" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    try s1024.setVoxelAtGlobal(@import("address.zig").encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 9,
        .lz = 0,
    }), true);

    var entities = [_]entity16.Entity16{};
    const char = createCharacter(0.0, 10.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
        .prevent_fall_off_ledges = true,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    char.grounded = true;
    char.vel_x = 4.0;
    const before_x = char.pos_x;

    update(char, &s1024, entities[0..], getConfig(char), 0.5);

    try std.testing.expect(char.grounded);
    try std.testing.expectApproxEqAbs(before_x, char.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), char.vel_x, 0.0001);
}

test "KCC updateSystem prevents characters from overlapping head-on" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const a = createCharacter(0.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const b = createCharacter(1.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    a.grounded = true;
    b.grounded = true;
    a.vel_x = 2.0;
    b.vel_x = -2.0;

    updateSystem(&s1024, entities[0..], 0.0);

    const dx = b.pos_x - a.pos_x;
    const dz = b.pos_z - a.pos_z;
    const dist_sq = dx * dx + dz * dz;

    try std.testing.expect(dist_sq >= 4.0 - 0.05);
    try std.testing.expect(a.vel_x <= b.vel_x or a.vel_x <= 0.0001);
}

test "KCC updateSystem separates chained overlapping characters" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const a = createCharacter(0.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const b = createCharacter(1.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const c = createCharacter(2.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    a.grounded = true;
    b.grounded = true;
    c.grounded = true;

    updateSystem(&s1024, entities[0..], 0.0);

    const ab_dx = b.pos_x - a.pos_x;
    const ab_dz = b.pos_z - a.pos_z;
    const bc_dx = c.pos_x - b.pos_x;
    const bc_dz = c.pos_z - b.pos_z;
    try std.testing.expect(ab_dx * ab_dx + ab_dz * ab_dz >= 3.8);
    try std.testing.expect(bc_dx * bc_dx + bc_dz * bc_dz >= 3.8);
}

test "KCC updateSystem adds stable lateral bypass for head-on characters" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const a = createCharacter(0.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const b = createCharacter(1.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    a.grounded = true;
    b.grounded = true;
    a.vel_x = 2.0;
    b.vel_x = -2.0;

    updateSystem(&s1024, entities[0..], 0.0);

    try std.testing.expect(@abs(a.pos_z) > 0.01 or @abs(b.pos_z) > 0.01);
    try std.testing.expect(@abs(a.pos_z - b.pos_z) > 0.01);
}

test "KCC updateSystem separates overlapping characters proportionally to mass" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const light = createCharacter(0.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const heavy = createCharacter(1.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    light.grounded = true;
    heavy.grounded = true;
    light.mass = 40;
    heavy.mass = 160;

    const light_before = light.pos_x;
    const heavy_before = heavy.pos_x;
    updateSystem(&s1024, entities[0..], 0.0);

    try std.testing.expect(@abs(light.pos_x - light_before) > @abs(heavy.pos_x - heavy_before));
}

test "KCC updateSystem adds stable lateral bypass for z-axis head-on characters" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const a = createCharacter(0.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const b = createCharacter(0.0, 0.0, 1.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    a.grounded = true;
    b.grounded = true;
    a.vel_z = 2.0;
    b.vel_z = -2.0;

    updateSystem(&s1024, entities[0..], 0.0);

    try std.testing.expect(@abs(a.pos_x) > 0.01 or @abs(b.pos_x) > 0.01);
    try std.testing.expect(@abs(a.pos_x - b.pos_x) > 0.01);
}

test "KCC updateSystem applies predictive avoidance before overlap occurs" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const a = createCharacter(0.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const b = createCharacter(3.5, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    a.grounded = true;
    b.grounded = true;
    a.vel_x = 2.0;
    b.vel_x = -2.0;

    const before_dist = b.pos_x - a.pos_x;
    updateSystem(&s1024, entities[0..], 0.25);

    try std.testing.expect(before_dist > 2.0);
    try std.testing.expect(@abs(a.pos_z) > 0.01 or @abs(b.pos_z) > 0.01);
    try std.testing.expect(a.vel_x < 2.0);
    try std.testing.expect(b.vel_x > -2.0);
}

test "KCC updateSystem predictive avoidance yields lighter character more" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const light = createCharacter(0.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const heavy = createCharacter(3.5, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    light.grounded = true;
    heavy.grounded = true;
    light.mass = 40;
    heavy.mass = 160;
    light.vel_x = 2.0;
    heavy.vel_x = -2.0;

    updateSystem(&s1024, entities[0..], 0.25);

    try std.testing.expect(@abs(light.pos_z) >= @abs(heavy.pos_z));
    try std.testing.expect(@abs(light.vel_z) >= @abs(heavy.vel_z));
    try std.testing.expect((2.0 - light.vel_x) >= (heavy.vel_x + 2.0));
}

test "KCC updateSystem predictive avoidance evaluates every pair in three-character approach" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const left = createCharacter(-2.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const mid = createCharacter(100.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    const right = createCharacter(2.0, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 4,
        .crouch_height = 2,
        .radius = 1,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    left.grounded = true;
    mid.grounded = true;
    right.grounded = true;
    left.vel_x = 2.0;
    mid.vel_x = 0.0;
    right.vel_x = -2.0;

    updateSystem(&s1024, entities[0..], 0.25);

    try std.testing.expectApproxEqAbs(@as(f32, 100.0), mid.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), mid.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), mid.vel_z, 0.0001);
    try std.testing.expect(@abs(left.pos_z) > 0.01);
    try std.testing.expect(@abs(right.pos_z) > 0.01);
}

test "KCC updateSystem predictive avoidance reacts to upcoming occupancy overlap" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{};
    const a = createCharacter(-2.5, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
    }) orelse return error.TestUnexpectedResult;
    const b = createCharacter(2.5, 0.0, 0.0, .{
        .gravity = 0.0,
        .stand_height = 8,
        .crouch_height = 4,
        .radius = 2,
    }) orelse return error.TestUnexpectedResult;
    defer init();

    a.grounded = true;
    b.grounded = true;
    a.vel_x = 1.0;
    b.vel_x = -1.0;

    updateSystem(&s1024, entities[0..], 0.25);

    try std.testing.expect(@abs(a.pos_z) > 0.01 or @abs(b.pos_z) > 0.01);
    try std.testing.expect(a.vel_x < 1.0 or b.vel_x > -1.0);
}
