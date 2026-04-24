//! KCC - Kinematic Character Controller
//!
//! Phase 7: Character movement for player/AI controlled characters
//! Handles: ground detection, collision response, jumping, crouching, rigid body pushing

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");

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

pub fn setYaw(state: *KCCState, yaw: f32) void {
    state.yaw = yaw;
}

pub fn getHeight(state: *KCCState) i32 {
    return if (state.crouching) state.crouch_height else state.stand_height;
}

/// Check if position is grounded using voxel sampling
pub fn checkGrounded(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    _ = getHeight(state);
    const check_y = @as(i32, @intFromFloat(@floor(state.pos_y))) - 1;

    const radius = state.radius;
    const half_radius = @divTrunc(radius, 2);

    var x: i32 = -half_radius;
    while (x <= half_radius) : (x += 2) {
        const wx = @as(i32, @intFromFloat(@floor(state.pos_x))) + x;
        const wz = @as(i32, @intFromFloat(@floor(state.pos_z))) + x;
        if (physics.isOccupiedGlobal(s1024, undefined, entities, wx, check_y, wz, null)) {
            return true;
        }
    }
    return false;
}

/// Get ground normal for slope handling
pub fn getGroundNormal(
    state: *KCCState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) struct { x: f32, y: f32, z: f32 } {
    _ = state;
    _ = s1024;
    _ = entities;
    return .{ .x = 0, .y = 1, .z = 0 };
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

    var y: i32 = 0;
    while (y < height) : (y += 2) {
        var x: i32 = -radius;
        while (x <= radius) : (x += 2) {
            var z: i32 = -radius;
            while (z <= radius) : (z += 2) {
                if (physics.isOccupiedGlobal(s1024, undefined, entities, px + x, py + y, pz + z, null)) {
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
    _: KCCConfig,
) void {
    const height = getHeight(state);
    const radius = state.radius;
    const old_y = state.pos_y;

    if (checkCollision(state.pos_x, state.pos_y, state.pos_z, height, radius, s1024, entities)) {
        var attempts: u8 = 0;
        while (attempts < 4) : (attempts += 1) {
            state.pos_y += 1;
            if (!checkCollision(state.pos_x, state.pos_y, state.pos_z, height, radius, s1024, entities)) {
                break;
            }
        }

        if (attempts >= 4) {
            state.pos_y = old_y;
            state.vel_y = 0;
        }
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

        const dx = @as(i32, inst.pos_x) - px;
        const dy = @as(i32, inst.pos_y) - py;
        const dz = @as(i32, inst.pos_z) - pz;

        if (dx >= -radius - 16 and dx <= radius + 16 and
            dy >= -height and dy <= height and
            dz >= -radius - 16 and dz <= radius + 16)
        {
            const dist_sq = dx * dx + dy * dy + dz * dz;
            const push_dist = @as(f32, @floatFromInt(radius + 8));
            if (dist_sq < push_dist * push_dist and dist_sq > 0) {
                const dist = @sqrt(@as(f32, @floatFromInt(dist_sq)));
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
    applyGravity(state, dt, config);

    state.pos_x += state.vel_x * dt;
    state.pos_z += state.vel_z * dt;
    state.pos_y += state.vel_y * dt;

    state.grounded = checkGrounded(state, s1024, entities);
    if (state.grounded and state.vel_y < 0) {
        state.vel_y = 0;
        state.jumping = false;
    }

    resolveCollision(state, s1024, entities, config);
    pushNearbyBodies(state, s1024, entities, config);
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
