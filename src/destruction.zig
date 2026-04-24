//! Destruction - Destructible Entity System
//!
//! Phase 9: Damage models, fracture patterns, shattering, structural collapse
//! Handles: HP systems, crack propagation, fragment generation, avalanches

const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");

pub const DamageEntry = struct {
    damage_type: u8,
    damage_amount: f32,
    armor_reduction: f32,
};

pub const DamageModel = struct {
    max_hp: f32,
    current_hp: f32,
    damage_table: [8]DamageEntry,
    invulnerable: bool = false,
};

pub const CrackLine = struct {
    start_x: i8,
    start_y: i8,
    start_z: i8,
    end_x: i8,
    end_y: i8,
    end_z: i8,
    severity: f32,
};

pub const Fragment = struct {
    local_x: i8,
    local_y: i8,
    local_z: i8,
    size: u8,
    velocity_x: f32,
    velocity_y: f32,
    velocity_z: f32,
    rotation_x: f32,
    rotation_y: f32,
    rotation_z: f32,
    active: bool,
};

pub const FracturePattern = struct {
    seed: u32,
    crack_count: u8,
    cracks: [16]CrackLine,
    fragment_count: u8,
    fragments: [16]Fragment,
};

pub const MAX_DESTROYABLE: usize = 32;

pub const DestroyableState = struct {
    entity_id: u16,
    damage_model: DamageModel,
    fracture_pattern: ?FracturePattern,
    broken: bool,
};

pub const DestroyableSystem = struct {
    destroyables: [MAX_DESTROYABLE]DestroyableState,
    count: u8,
};

var g_destroyable_system: DestroyableSystem = undefined;

pub fn init() void {
    g_destroyable_system.count = 0;
    for (0..MAX_DESTROYABLE) |i| {
        g_destroyable_system.destroyables[i] = .{
            .entity_id = 0,
            .damage_model = .{
                .max_hp = 100.0,
                .current_hp = 100.0,
                .damage_table = [_]DamageEntry{.{.damage_type=0, .damage_amount=0, .armor_reduction=0}} ** 8,
                .invulnerable = false,
            },
            .fracture_pattern = null,
            .broken = false,
        };
    }
}

/// Create a destroyable entity
pub fn createDestroyable(entity_id: u16, max_hp: f32) ?*DestroyableState {
    if (g_destroyable_system.count >= MAX_DESTROYABLE) return null;
    const idx = g_destroyable_system.count;
    g_destroyable_system.count += 1;
    const state = &g_destroyable_system.destroyables[idx];
    state.* = .{
        .entity_id = entity_id,
        .damage_model = .{
            .max_hp = max_hp,
            .current_hp = max_hp,
            .damage_table = [_]DamageEntry{.{.damage_type=0, .damage_amount=0, .armor_reduction=0}} ** 8,
            .invulnerable = false,
        },
        .fracture_pattern = null,
        .broken = false,
    };
    return state;
}

/// Calculate damage based on impact
pub fn calculateDamage(
    impact: f32,
    material: entity16.MaterialType,
    hardness: u16,
) f32 {
    const base_damage = impact;

    const material_mult: f32 = switch (material) {
        .fragile => 3.0,
        .solid => 1.5,
        .elastic => 0.8,
        .composite => 1.0,
        else => 1.0,
    };

    const hardness_factor = 1.0 - (@as(f32, @floatFromInt(hardness)) / 255.0);
    return base_damage * material_mult * hardness_factor;
}

/// Apply damage to entity
pub fn applyDamage(state: *DestroyableState, damage: f32) void {
    if (state.damage_model.invulnerable or state.broken) return;

    state.damage_model.current_hp -= damage;
    if (state.damage_model.current_hp <= 0) {
        state.damage_model.current_hp = 0;
        state.broken = true;
    }
}

/// Generate fracture pattern from impact point
pub fn generateFracture(
    _: *const entity16.Entity16,
    point_x: i8,
    point_y: i8,
    point_z: i8,
    energy: f32,
    seed: u32,
) FracturePattern {
    var pattern: FracturePattern = undefined;
    pattern.seed = seed;
    pattern.crack_count = 0;
    pattern.fragment_count = 0;

    const base_angle: f32 = @as(f32, @floatFromInt(seed % 360)) * 3.14159 / 180.0;
    const crack_count = @as(u8, @min(16, @as(u8, @intFromFloat(@sqrt(energy / 10.0)))));

    var i: u8 = 0;
    while (i < crack_count) : (i += 1) {
        const angle = base_angle + @as(f32, @floatFromInt(i)) * 3.14159 * 2.0 / @as(f32, @floatFromInt(crack_count));
        const length = @as(f32, @floatFromInt(3 + (seed % 10)));

        pattern.cracks[i] = .{
            .start_x = point_x,
            .start_y = point_y,
            .start_z = point_z,
            .end_x = @as(i8, @intFromFloat(@round(@as(f32, @floatFromInt(point_x)) + @cos(angle) * length))),
            .end_y = @as(i8, @intFromFloat(@round(@as(f32, @floatFromInt(point_y)) + @sin(angle) * length))),
            .end_z = point_z,
            .severity = energy / 100.0,
        };
        pattern.crack_count = i + 1;
    }

    return pattern;
}

/// Check if entity should shatter based on damage
pub fn shouldShatter(state: *const DestroyableState) bool {
    if (state.damage_model.current_hp <= 0) return true;

    const hp_ratio = state.damage_model.current_hp / state.damage_model.max_hp;
    return hp_ratio < 0.25;
}

/// Generate fragments from broken entity
pub fn generateFragments(
    entity: *const entity16.Entity16,
    impact_x: i8,
    impact_y: i8,
    impact_z: i8,
    fragment_energy: f32,
) [16]Fragment {
    var fragments: [16]Fragment = undefined;
    var count: u8 = 0;

    var x: i8 = 0;
    while (x < 16) : (x += 4) {
        var y: i8 = 0;
        while (y < 16) : (y += 4) {
            var z: i8 = 0;
            while (z < 16) : (z += 4) {
                if (entity16.testVoxel(entity, @intCast(x), @intCast(y), @intCast(z))) {
                    const dx = @as(f32, @floatFromInt(x)) - @as(f32, @floatFromInt(impact_x));
                    const dy = @as(f32, @floatFromInt(y)) - @as(f32, @floatFromInt(impact_y));
                    const dz = @as(f32, @floatFromInt(z)) - @as(f32, @floatFromInt(impact_z));
                    const dist = @sqrt(dx * dx + dy * dy + dz * dz);

                    if (count < 16) {
                        fragments[count] = .{
                            .local_x = x,
                            .local_y = y,
                            .local_z = z,
                            .size = 4,
                            .velocity_x = dx / @max(0.1, dist) * fragment_energy,
                            .velocity_y = dy / @max(0.1, dist) * fragment_energy + 50.0,
                            .velocity_z = dz / @max(0.1, dist) * fragment_energy,
                            .rotation_x = 0,
                            .rotation_y = 0,
                            .rotation_z = 0,
                            .active = true,
                        };
                        count += 1;
                    }
                }
            }
        }
    }

    while (count < 16) : (count += 1) {
        fragments[count] = .{
            .local_x = 0, .local_y = 0, .local_z = 0,
            .size = 0, .velocity_x = 0, .velocity_y = 0, .velocity_z = 0,
            .rotation_x = 0, .rotation_y = 0, .rotation_z = 0,
            .active = false,
        };
    }

    return fragments;
}

/// Simulate structural collapse (domino effect)
pub fn checkStructuralCollapse(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    inst_idx: u8,
    direction_x: f32,
    direction_z: f32,
) bool {
    if (inst_idx >= s1024.instance_count) return false;

    const inst = &s1024.instances[inst_idx];
    if (inst.entity_id >= entities.len) return false;

    _ = &entities; // entities parameter unused in simplified version

    const check_x = inst.pos_x + @as(i32, @intFromFloat(@round(direction_x * 20.0)));
    const check_z = inst.pos_z + @as(i32, @intFromFloat(@round(direction_z * 20.0)));

    for (0..s1024.instance_count) |i| {
        if (i == inst_idx) continue;
        const other = &s1024.instances[i];

        const dx = @as(i32, @intFromFloat(@floor(@as(f32, @floatFromInt(other.pos_x)) - @as(f32, @floatFromInt(check_x)))));
        const dz = @as(i32, @intFromFloat(@floor(@as(f32, @floatFromInt(other.pos_z)) - @as(f32, @floatFromInt(check_z)))));

        if (@abs(dx) < 20 and @abs(dz) < 20) {
            const other_ent = &entities[other.entity_id];
            if ((other_ent.physics.flags & 0x01) == 0) {
                const dist = @sqrt(@as(f32, @floatFromInt(dx * dx + dz * dz)));
                if (dist < 20.0) {
                    return true;
                }
            }
        }
    }

    return false;
}

/// Trigger avalanche of damaged entities
pub fn triggerAvalanche(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    center_x: i32,
    center_y: i32,
    center_z: i32,
    radius: i32,
) void {
    var i: u8 = 0;
    while (i < s1024.instance_count) : (i += 1) {
        const inst = &s1024.instances[i];
        if (inst.entity_id >= entities.len) continue;

        const ent = &entities[inst.entity_id];
        if ((ent.physics.flags & 0x01) != 0) continue;

        const dx = inst.pos_x - center_x;
        const dy = inst.pos_y - center_y;
        const dz = inst.pos_z - center_z;
        const dist_sq = dx * dx + dy * dy + dz * dz;

        if (dist_sq < radius * radius) {
            inst.vel_y = @truncate(@as(i32, @intFromFloat(@sqrt(@as(f32, @floatFromInt(radius * radius - dist_sq))) * 10.0)));
        }
    }
}

/// Get system for external iteration
pub fn getSystem() *DestroyableSystem {
    return &g_destroyable_system;
}
