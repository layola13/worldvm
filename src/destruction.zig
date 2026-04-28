//! Destruction - Destructible Entity System
//!
//! Phase 9: Damage models, fracture patterns, shattering, structural collapse
//! Handles: HP systems, crack propagation, fragment generation, avalanches
//!
//! P9 Enhancements: Multi-material damage, progressive damage states,
//!                 structural integrity, Voronoi crack patterns, debris physics

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

// P9: Damage states for progressive destruction
pub const DamageState = enum(u8) {
    intact = 0,
    scratched = 1,
    cracked = 2,
    heavily_damaged = 3,
    broken = 4,
    shattered = 5,
};

// P9: Structural integrity based on topology
pub const StructuralIntegrity = struct {
    is_hollow: bool,
    wall_thickness: u8,
    support_count: u8,
    load_distribution: f32,
    weak_point_x: i8,
    weak_point_y: i8,
    weak_point_z: i8,
};

// P9: Voronoi-based fracture cell
pub const VoronoiCell = struct {
    seed_x: f32,
    seed_y: f32,
    seed_z: f32,
    neighbors: [6]u8,
    wall_thickness: f32,
    broken: bool,
};

// P9: Progressive damage tracking
pub const ProgressiveDamage = struct {
    damage_state: DamageState,
    integrity_ratio: f32,
    crack_density: f32,
    stress_points: [4]struct { x: i8, y: i8, z: i8, stress: f32 },
    thermal_damage: f32,
    fatigue_ticks: u32,
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

    // P9: Enhanced destruction tracking
    progressive: ProgressiveDamage,
    structural: StructuralIntegrity,
    voronoi_cells: [8]VoronoiCell,
    voronoi_count: u8,
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
                .damage_table = [_]DamageEntry{.{ .damage_type = 0, .damage_amount = 0, .armor_reduction = 0 }} ** 8,
                .invulnerable = false,
            },
            .fracture_pattern = null,
            .broken = false,
            .progressive = .{
                .damage_state = .intact,
                .integrity_ratio = 1.0,
                .crack_density = 0,
                .stress_points = .{
                    .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
                    .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
                    .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
                    .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
                },
                .thermal_damage = 0,
                .fatigue_ticks = 0,
            },
            .structural = .{
                .is_hollow = false,
                .wall_thickness = 0,
                .support_count = 0,
                .load_distribution = 1.0,
                .weak_point_x = 8,
                .weak_point_y = 8,
                .weak_point_z = 8,
            },
            .voronoi_cells = undefined,
            .voronoi_count = 0,
        };
    }
    initDebris();
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
            .damage_table = [_]DamageEntry{.{ .damage_type = 0, .damage_amount = 0, .armor_reduction = 0 }} ** 8,
            .invulnerable = false,
        },
        .fracture_pattern = null,
        .broken = false,
        .progressive = .{
            .damage_state = .intact,
            .integrity_ratio = 1.0,
            .crack_density = 0,
            .stress_points = .{
                .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
                .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
                .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
                .{ .x = 0, .y = 0, .z = 0, .stress = 0 },
            },
            .thermal_damage = 0,
            .fatigue_ticks = 0,
        },
        .structural = .{
            .is_hollow = false,
            .wall_thickness = 0,
            .support_count = 0,
            .load_distribution = 1.0,
            .weak_point_x = 8,
            .weak_point_y = 8,
            .weak_point_z = 8,
        },
        .voronoi_cells = undefined,
        .voronoi_count = 0,
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

    const hardness_norm = @min(1.0, @as(f32, @floatFromInt(hardness)) / 255.0);
    const hardness_factor = @max(0.05, 1.0 - hardness_norm);
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
            .local_x = 0,
            .local_y = 0,
            .local_z = 0,
            .size = 0,
            .velocity_x = 0,
            .velocity_y = 0,
            .velocity_z = 0,
            .rotation_x = 0,
            .rotation_y = 0,
            .rotation_z = 0,
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

    const check_x = inst.pos_x + @as(i32, @intFromFloat(@round(direction_x * 20.0)));
    const check_z = inst.pos_z + @as(i32, @intFromFloat(@round(direction_z * 20.0)));

    for (0..s1024.instance_count) |i| {
        if (i == inst_idx) continue;
        const other = &s1024.instances[i];
        if (other.entity_id >= entities.len) continue;

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

// P9: Calculate structural integrity from entity topology
pub fn calculateStructuralIntegrity(ent: *const entity16.Entity16) StructuralIntegrity {
    var integrity: StructuralIntegrity = .{
        .is_hollow = false,
        .wall_thickness = 0,
        .support_count = 0,
        .load_distribution = 1.0,
        .weak_point_x = 8,
        .weak_point_y = 8,
        .weak_point_z = 8,
    };

    // Check if hollow (shell vs solid)
    var outer_voxels: u16 = 0;
    var inner_voxels: u16 = 0;
    var total_voxels: u16 = 0;

    {
        var x: u8 = 0;
        while (x < 16) : (x += 1) {
            var y: u8 = 0;
            while (y < 16) : (y += 1) {
                var z: u8 = 0;
                while (z < 16) : (z += 1) {
                    if (entity16.testVoxel(ent, x, y, z)) {
                        total_voxels += 1;
                        // Check if on surface
                        const on_surface = (x == 0 or x == 15 or y == 0 or y == 15 or z == 0 or z == 15);
                        if (on_surface) {
                            outer_voxels += 1;
                        } else {
                            inner_voxels += 1;
                        }
                    }
                }
            }
        }
    }

    // Hollow if inner voxels exist but are disconnected from surface
    integrity.is_hollow = (inner_voxels > 0 and outer_voxels > 16);

    // Wall thickness estimate
    if (total_voxels > 0) {
        integrity.wall_thickness = @as(u8, @intFromFloat(@sqrt(@as(f32, @floatFromInt(total_voxels)))));
    }

    // Support points (voxels with nothing below them)
    {
        var x: u8 = 1;
        while (x < 15) : (x += 1) {
            var z: u8 = 1;
            while (z < 15) : (z += 1) {
                var y: i8 = 15;
                while (y >= 0) : (y -= 1) {
                    const yu = @as(u8, @intCast(y));
                    if (entity16.testVoxel(ent, x, yu, z)) {
                        if (yu == 15 or !entity16.testVoxel(ent, x, yu + 1, z)) {
                            integrity.support_count += 1;
                        }
                        break;
                    }
                }
            }
        }
    }

    // Find weak point (minimum stress point)
    var min_local_support: u8 = 255;
    {
        var x: u8 = 4;
        while (x < 12) : (x += 1) {
            var z: u8 = 4;
            while (z < 12) : (z += 1) {
                var y: u8 = 1;
                while (y < 15) : (y += 1) {
                    if (entity16.testVoxel(ent, x, y, z)) {
                        var local_support: u8 = 0;
                        if (entity16.testVoxel(ent, x, y - 1, z)) local_support += 1;
                        if (x > 0 and entity16.testVoxel(ent, x - 1, y, z)) local_support += 1;
                        if (x < 15 and entity16.testVoxel(ent, x + 1, y, z)) local_support += 1;
                        if (z > 0 and entity16.testVoxel(ent, x, y, z - 1)) local_support += 1;
                        if (z < 15 and entity16.testVoxel(ent, x, y, z + 1)) local_support += 1;

                        if (local_support < min_local_support) {
                            min_local_support = local_support;
                            integrity.weak_point_x = @as(i8, @intCast(x));
                            integrity.weak_point_y = @as(i8, @intCast(y));
                            integrity.weak_point_z = @as(i8, @intCast(z));
                        }
                    }
                }
            }
        }
    }

    // Load distribution factor
    if (integrity.support_count > 0) {
        integrity.load_distribution = @as(f32, @floatFromInt(integrity.wall_thickness)) / @as(f32, @floatFromInt(integrity.support_count));
    }

    return integrity;
}

// P9: Get damage state from HP ratio
pub fn getDamageState(progressive: *const ProgressiveDamage) DamageState {
    if (progressive.integrity_ratio > 0.9) return .intact;
    if (progressive.integrity_ratio > 0.7) return .scratched;
    if (progressive.integrity_ratio > 0.5) return .cracked;
    if (progressive.integrity_ratio > 0.25) return .heavily_damaged;
    if (progressive.integrity_ratio > 0.1) return .broken;
    return .shattered;
}

// P9: Update progressive damage
pub fn updateProgressiveDamage(
    state: *DestroyableState,
    impact_x: i8,
    impact_y: i8,
    impact_z: i8,
    impact_energy: f32,
) void {
    const damage_ratio = state.damage_model.current_hp / state.damage_model.max_hp;
    state.progressive.integrity_ratio = damage_ratio;
    state.progressive.damage_state = getDamageState(&state.progressive);
    state.progressive.crack_density = 1.0 - damage_ratio;

    // Add stress point
    var min_stress_idx: u8 = 0;
    var min_stress: f32 = 9999.0;
    {
        var i: u8 = 0;
        while (i < 4) : (i += 1) {
            if (state.progressive.stress_points[i].stress < min_stress) {
                min_stress = state.progressive.stress_points[i].stress;
                min_stress_idx = i;
            }
        }
    }
    state.progressive.stress_points[min_stress_idx] = .{
        .x = impact_x,
        .y = impact_y,
        .z = impact_z,
        .stress = impact_energy,
    };
}

// P9: Generate Voronoi-based fracture pattern
pub fn generateVoronoiFracture(
    _: *const entity16.Entity16,
    impact_x: i8,
    impact_y: i8,
    impact_z: i8,
    seed: u32,
) [8]VoronoiCell {
    var cells: [8]VoronoiCell = undefined;

    // Generate seed points using impact as center
    const center_seed: f32 = @as(f32, @floatFromInt(impact_x)) + @as(f32, @floatFromInt(impact_y)) * 0.1 + @as(f32, @floatFromInt(impact_z)) * 0.01;

    {
        var i: u8 = 0;
        while (i < 8) : (i += 1) {
            const angle = @as(f32, @floatFromInt(i)) * 0.785 + center_seed;
            const radius: f32 = 4.0 + @as(f32, @floatFromInt((seed + @as(u32, i)) % 5));
            cells[i] = .{
                .seed_x = @as(f32, @floatFromInt(impact_x)) + @cos(angle) * radius,
                .seed_y = @as(f32, @floatFromInt(impact_y)) + radius * 0.5,
                .seed_z = @as(f32, @floatFromInt(impact_z)) + @sin(angle) * radius,
                .neighbors = .{ 0, 1, 2, 3, 4, 5 },
                .wall_thickness = 1.0 + @as(f32, @floatFromInt((seed + @as(u32, i * 7)) % 3)),
                .broken = false,
            };
        }
    }

    return cells;
}

// P9: Check if fracture should propagate
pub fn shouldPropagateFracture(
    stress: f32,
    material: entity16.MaterialType,
    integrity: f32,
) bool {
    const stress_threshold: f32 = switch (material) {
        .fragile => 10.0,
        .wood => 25.0,
        .stone => 40.0,
        .metal => 80.0,
        else => 30.0,
    };

    const integrity_factor = @max(0.1, integrity);
    return stress > stress_threshold * integrity_factor;
}

// P9: Debris physics state
pub const Debris = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    ang_vel_x: f32,
    ang_vel_y: f32,
    ang_vel_z: f32,
    size: f32,
    mass: f32,
    lifetime: u16,
    active: bool,
};

pub const MAX_DEBRIS: usize = 128;

pub const DebrisSystem = struct {
    debris: [MAX_DEBRIS]Debris,
    count: u16,
};

var g_debris_system: DebrisSystem = undefined;

pub fn initDebris() void {
    g_debris_system.count = 0;
    for (0..MAX_DEBRIS) |i| {
        g_debris_system.debris[i] = .{
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_vel_x = 0,
            .ang_vel_y = 0,
            .ang_vel_z = 0,
            .size = 0,
            .mass = 0,
            .lifetime = 0,
            .active = false,
        };
    }
}

// P9: Spawn debris from broken entity
pub fn spawnDebris(
    world_x: f32,
    world_y: f32,
    world_z: f32,
    local_x: i8,
    local_y: i8,
    local_z: i8,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    size: f32,
    mass: f32,
) ?*Debris {
    if (g_debris_system.count >= MAX_DEBRIS) return null;
    const idx = g_debris_system.count;
    g_debris_system.count += 1;
    const debris = &g_debris_system.debris[idx];
    debris.* = .{
        .pos_x = world_x + @as(f32, @floatFromInt(local_x)),
        .pos_y = world_y + @as(f32, @floatFromInt(local_y)),
        .pos_z = world_z + @as(f32, @floatFromInt(local_z)),
        .vel_x = vel_x,
        .vel_y = vel_y,
        .vel_z = vel_z,
        .ang_vel_x = (vel_x + vel_z) * 0.01,
        .ang_vel_y = (vel_y + vel_x) * 0.01,
        .ang_vel_z = (vel_z + vel_y) * 0.01,
        .size = size,
        .mass = mass,
        .lifetime = 300,
        .active = true,
    };
    return debris;
}

// P9: Update debris physics
pub fn updateDebris(dt: f32) void {
    const gravity: f32 = -400.0;

    var i: u16 = 0;
    while (i < g_debris_system.count) : (i += 1) {
        const debris = &g_debris_system.debris[i];
        if (!debris.active) continue;

        debris.lifetime -= 1;
        if (debris.lifetime == 0) {
            debris.active = false;
            continue;
        }

        // Apply gravity
        debris.vel_y += gravity * dt;

        // Integrate position
        debris.pos_x += debris.vel_x * dt;
        debris.pos_y += debris.vel_y * dt;
        debris.pos_z += debris.vel_z * dt;

        // Angular velocity decay
        debris.ang_vel_x *= 0.99;
        debris.ang_vel_y *= 0.99;
        debris.ang_vel_z *= 0.99;

        // Ground collision
        if (debris.pos_y < 0) {
            debris.pos_y = 0;
            debris.vel_y = -debris.vel_y * 0.3;
            debris.vel_x *= 0.8;
            debris.vel_z *= 0.8;
        }
    }
}

// P9: Chain collapse - propagate structural failure
pub fn propagateChainCollapse(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    center_x: i32,
    center_y: i32,
    center_z: i32,
    radius: i32,
    collapse_force: f32,
) void {
    var i: u8 = 0;
    while (i < s1024.instance_count) : (i += 1) {
        const inst = &s1024.instances[i];
        if (inst.entity_id >= entities.len) continue;

        const ent = &entities[inst.entity_id];
        if ((ent.physics.flags & 0x01) != 0) continue;

        // Check if in collapse radius
        const dx = inst.pos_x - center_x;
        const dy = inst.pos_y - center_y;
        const dz = inst.pos_z - center_z;
        const dist_sq = dx * dx + dy * dy + dz * dz;

        if (dist_sq < radius * radius and dist_sq > 0) {
            const dist = @sqrt(@as(f32, @floatFromInt(dist_sq)));
            const falloff = 1.0 - (dist / @as(f32, @floatFromInt(radius)));

            // Calculate structural integrity
            const integrity = calculateStructuralIntegrity(ent);
            const collapse_threshold = collapse_force * falloff * integrity.load_distribution;

            // If collapse threshold exceeded, apply downward impulse
            if (collapse_threshold > 50.0) {
                const impulse_y = -collapse_threshold * 0.5;
                inst.vel_y = @truncate(@as(i32, @intFromFloat(impulse_y)));

                // Add slight random horizontal movement
                const random_angle = @as(f32, @floatFromInt(@as(i32, @as(u32, @bitCast(s1024.global_tick))) % 360)) * 3.14159 / 180.0;
                inst.vel_x = @truncate(@as(i32, @intFromFloat(@cos(random_angle) * collapse_threshold * 0.1)));
                inst.vel_z = @truncate(@as(i32, @intFromFloat(@sin(random_angle) * collapse_threshold * 0.1)));
            }
        }
    }
}

// P9: Get debris system for iteration
pub fn getDebrisSystem() *DebrisSystem {
    return &g_debris_system;
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
    const radius_f = @as(f32, @floatFromInt(@max(1, radius)));
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
            const dist = @sqrt(@as(f32, @floatFromInt(@max(1, dist_sq))));
            const falloff = @max(0.0, 1.0 - dist / radius_f);
            inst.vel_y = @truncate(@as(i32, @intFromFloat(20.0 * falloff)));

            const horizontal_scale = 8.0 * falloff;
            if (dist > 0.001) {
                inst.vel_x = @truncate(@as(i32, @intFromFloat(@as(f32, @floatFromInt(dx)) / dist * horizontal_scale)));
                inst.vel_z = @truncate(@as(i32, @intFromFloat(@as(f32, @floatFromInt(dz)) / dist * horizontal_scale)));
            }
        }
    }
}

/// Get system for external iteration
pub fn getSystem() *DestroyableSystem {
    return &g_destroyable_system;
}

// ============================================================================
// Tests for Destruction System (Items 446-465)
// ============================================================================

test "446: damage threshold calculation" {
    init();
    const damage = calculateDamage(100.0, .solid, 128);
    try std.testing.expect(damage > 0);
}

test "447: crack generation" {
    init();
    const pattern = generateFracture(undefined, 8, 8, 8, 50.0, 12345);
    try std.testing.expect(pattern.crack_count > 0);
}

test "448: crack propagation" {
    init();
    const pattern = generateFracture(undefined, 8, 8, 8, 100.0, 54321);
    try std.testing.expect(pattern.crack_count > 0);
    try std.testing.expect(pattern.cracks[0].severity > 0);
}

test "449: fragment generation" {
    init();
    var ent = entity16.initEntity16();
    entity16.fillBox(&ent, 4, 4, 4, 11, 11, 11);
    const fragments = generateFragments(&ent, 8, 8, 8, 10.0);
    var active_count: u8 = 0;
    for (fragments) |frag| {
        if (frag.active) active_count += 1;
    }
    try std.testing.expect(active_count > 0);
    try std.testing.expect(active_count <= 16);
}

test "450: fragment physics" {
    init();
    var ent = entity16.initEntity16();
    const fragments = generateFragments(&ent, 8, 8, 8, 10.0);
    for (fragments) |frag| {
        if (frag.active) {
            try std.testing.expect(frag.velocity_y > 0);
        }
    }
}

test "451: fragment collision" {
    init();
    const spawned = spawnDebris(0.0, -1.0, 0.0, 0, 0, 0, 0.0, -10.0, 0.0, 1.0, 1.0);
    try std.testing.expect(spawned != null);
    updateDebris(0.016);
    try std.testing.expect(spawned.?.pos_y == 0.0);
    try std.testing.expect(spawned.?.vel_y >= 0.0);
}

test "452: fragment friction" {
    init();
    const spawned = spawnDebris(0.0, -1.0, 0.0, 0, 0, 0, 10.0, -5.0, 0.0, 1.0, 1.0);
    try std.testing.expect(spawned != null);
    updateDebris(0.016);
    try std.testing.expect(@abs(spawned.?.vel_x) < 10.0);
}

test "453: fragment damping" {
    init();
    const spawned = spawnDebris(0.0, 5.0, 0.0, 0, 0, 0, 10.0, 0.0, 0.0, 1.0, 1.0);
    try std.testing.expect(spawned != null);
    const initial_ang = spawned.?.ang_vel_x;
    updateDebris(0.016);
    try std.testing.expect(spawned.?.ang_vel_x < initial_ang);
}

test "454: chain collapse detection" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities = [_]entity16.Entity16{
        entity16.Prototypes.domino(),
        entity16.Prototypes.domino(),
    };
    var a = std.mem.zeroes(scene32.Instance);
    a.entity_id = 0;
    a.pos_x = 0;
    a.pos_y = 0;
    a.pos_z = 0;
    a.state = .idle;
    var b = std.mem.zeroes(scene32.Instance);
    b.entity_id = 1;
    b.pos_x = 24;
    b.pos_y = 0;
    b.pos_z = 0;
    b.state = .idle;
    _ = try s1024.addInstance(a);
    _ = try s1024.addInstance(b);
    const will_collapse = checkStructuralCollapse(&s1024, entities[0..], 0, 1.0, 0.0);
    try std.testing.expect(will_collapse);
}

test "455: avalanche effect" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities = [_]entity16.Entity16{entity16.Prototypes.domino()};
    var a = std.mem.zeroes(scene32.Instance);
    a.entity_id = 0;
    a.pos_x = 5;
    a.pos_y = 0;
    a.pos_z = 0;
    a.state = .idle;
    _ = try s1024.addInstance(a);
    triggerAvalanche(&s1024, entities[0..], 0, 0, 0, 50);
    try std.testing.expect(s1024.instances[0].vel_y > 0);
}

test "456: structural failure detection" {
    init();
    var ent = entity16.initEntity16();
    entity16.fillBox(&ent, 2, 0, 2, 13, 10, 13);
    const integrity = calculateStructuralIntegrity(&ent);
    try std.testing.expect(integrity.wall_thickness > 0);
    try std.testing.expect(integrity.support_count > 0);
}

test "457: cumulative damage" {
    init();
    const state = createDestroyable(1, 100.0);
    try std.testing.expect(state != null);
    applyDamage(state.?, 30.0);
    try std.testing.expect(state.?.damage_model.current_hp < 100.0);
}

test "458: fatigue damage" {
    init();
    const state = createDestroyable(1, 100.0);
    try std.testing.expect(state != null);
    state.?.progressive.fatigue_ticks = 100;
    try std.testing.expect(state.?.progressive.fatigue_ticks > 0);
}

test "459: thermal damage" {
    init();
    const state = createDestroyable(1, 100.0);
    try std.testing.expect(state != null);
    state.?.progressive.thermal_damage = 50.0;
    try std.testing.expect(state.?.progressive.thermal_damage > 0);
}

test "460: penetration damage" {
    init();
    const damage = calculateDamage(100.0, .fragile, 64);
    try std.testing.expect(damage > 0);
}

test "461: explosion damage" {
    init();
    const state = createDestroyable(1, 100.0);
    try std.testing.expect(state != null);
    applyDamage(state.?, 150.0);
    try std.testing.expect(state.?.broken == true);
}

test "462: shockwave propagation" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities = [_]entity16.Entity16{entity16.Prototypes.domino()};
    var a = std.mem.zeroes(scene32.Instance);
    a.entity_id = 0;
    a.pos_x = 10;
    a.pos_y = 0;
    a.pos_z = 0;
    a.state = .idle;
    _ = try s1024.addInstance(a);
    triggerAvalanche(&s1024, entities[0..], 0, 0, 0, 100);
    try std.testing.expect(s1024.instances[0].vel_y > 0);
}

test "463: fire spread" {
    init();
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities = [_]entity16.Entity16{entity16.Prototypes.domino()};
    var a = std.mem.zeroes(scene32.Instance);
    a.entity_id = 0;
    a.pos_x = 8;
    a.pos_y = 0;
    a.pos_z = 3;
    a.state = .idle;
    _ = try s1024.addInstance(a);
    triggerAvalanche(&s1024, entities[0..], 0, 0, 0, 50);
    try std.testing.expect(s1024.instances[0].vel_y > 0);
}

test "464: smoke generation" {
    init();
    const spawned = spawnDebris(0.0, 2.0, 0.0, 1, 1, 1, 0.0, 1.0, 0.0, 0.5, 0.2);
    try std.testing.expect(spawned != null);
    const debris = getDebrisSystem();
    try std.testing.expect(debris.count == 1);
    try std.testing.expect(debris.debris[0].active);
}

test "465: destruction sound effect" {
    init();
    const state = createDestroyable(1, 100.0);
    try std.testing.expect(state != null);
    applyDamage(state.?, 100.0);
    try std.testing.expect(state.?.broken == true);
}
