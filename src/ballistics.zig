//! Ballistics - Projectile Simulation System
//!
//! Phase 8: Projectile physics, penetration, deflection, fragmentation
//! Handles: bullet simulation, armor penetration, ricochet, explosion shockwaves
//!
//! P8 Enhancements: Material penetration physics, layered hits, topology splitting,
//!                  mass/COM recalculation, fragment generation

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");

pub const ProjectileState = enum(u8) {
    active = 0,
    penetrated = 1,
    deflected = 2,
    stopped = 3,
    fragmented = 4,
    multi_penetrated = 5,  // P8: Penetrated through multiple layers
};

// P8: Material properties for penetration calculation
pub const MaterialProps = struct {
    density: f32,        // kg/m^3 density
    thickness: f32,     // mm effective thickness
    porosity: f32,       // 0-1 porosity factor
    hardness: f32,       // Brinell hardness
    toughness: f32,      // Fracture toughness
};

pub const MaterialDensity = struct {
    pub const steel: f32 = 7850.0;
    pub const aluminum: f32 = 2700.0;
    pub const concrete: f32 = 2400.0;
    pub const wood: f32 = 600.0;
    pub const flesh: f32 = 1050.0;
    pub const rubber: f32 = 1100.0;
    pub const glass: f32 = 2500.0;
    pub const ceramic: f32 = 3900.0;
    pub const composite: f32 = 1600.0;
};

// P8: Layered hit result for multi-layer penetration
pub const LayeredHit = struct {
    layer_count: u8,
    hits: [8]struct {
        entity_id: u16,
        instance_idx: u8,
        t_enter: f32,
        t_exit: f32,
        penetration_depth: f32,
        energy_loss: f32,
        velocity_after: f32,
    },
    final_t: f32,
    total_penetration: f32,
    total_energy_loss: f32,
};

pub const Projectile = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    mass: f32,
    caliber: f32,
    entity_id: u16,
    state: ProjectileState,
    penetration_depth: f32,
    restitution: f32,
    lifetime: u16,

    // P8: Enhanced penetration physics
    remaining_energy: f32,     // Energy after each penetration
    penetration_distance: f32,  // Total distance penetrated through materials
    layer_count: u8,           // Number of layers penetrated
};

// P8: Fragment instance for spawning into world
pub const FragmentInstance = struct {
    pos_x: f32, pos_y: f32, pos_z: f32,
    vel_x: f32, vel_y: f32, vel_z: f32,
    size: f32,    // Fragment size
    mass: f32,    // Fragment mass
    entity_id: u16,
};

pub const RayHit = struct {
    hit: bool,
    t: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    entity_id: u16,
};

pub const MAX_PROJECTILES: usize = 64;

pub const ProjectileSystem = struct {
    projectiles: [MAX_PROJECTILES]Projectile,
    count: u8,
};

var g_projectile_system: ProjectileSystem = undefined;

pub fn init() void {
    g_projectile_system.count = 0;
    for (0..MAX_PROJECTILES) |i| {
        g_projectile_system.projectiles[i] = .{
            .pos_x = 0, .pos_y = 0, .pos_z = 0,
            .vel_x = 0, .vel_y = 0, .vel_z = 0,
            .mass = 1.0, .caliber = 5.56, .entity_id = 0,
            .state = .stopped, .penetration_depth = 0, .restitution = 0.3,
            .lifetime = 0,
            .remaining_energy = 0,
            .penetration_distance = 0,
            .layer_count = 0,
        };
    }
}

pub fn spawnProjectile(
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    mass: f32,
    caliber: f32,
) ?*Projectile {
    if (g_projectile_system.count >= MAX_PROJECTILES) return null;
    const idx = g_projectile_system.count;
    g_projectile_system.count += 1;
    const proj = &g_projectile_system.projectiles[idx];
    const speed = @sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z);
    const initial_energy = 0.5 * mass * speed * speed;
    proj.* = .{
        .pos_x = pos_x, .pos_y = pos_y, .pos_z = pos_z,
        .vel_x = vel_x, .vel_y = vel_y, .vel_z = vel_z,
        .mass = mass, .caliber = caliber, .entity_id = 0,
        .state = .active, .penetration_depth = 0, .restitution = 0.3,
        .lifetime = 0,
        .remaining_energy = initial_energy,
        .penetration_distance = 0,
        .layer_count = 0,
    };
    return proj;
}

pub fn removeProjectile(proj: *Projectile) void {
    _ = proj;
    if (g_projectile_system.count > 0) g_projectile_system.count -= 1;
}

/// Calculate speed from velocity
pub fn getSpeed(proj: *const Projectile) f32 {
    return @sqrt(proj.vel_x * proj.vel_x + proj.vel_y * proj.vel_y + proj.vel_z * proj.vel_z);
}

/// Calculate kinetic energy
pub fn getKineticEnergy(proj: *const Projectile) f32 {
    const speed = getSpeed(proj);
    return 0.5 * proj.mass * speed * speed;
}

// P8: Get material properties from entity physics
pub fn getMaterialProps(entity: *const entity16.Entity16) MaterialProps {
    const hardness = @as(f32, @floatFromInt(entity.physics.hardness));
    const mat = entity.physics.material;

    return switch (mat) {
        .metal => .{
            .density = MaterialDensity.steel,
            .thickness = 2.0,
            .porosity = 0.0,
            .hardness = hardness,
            .toughness = hardness * 0.5,
        },
        .stone => .{
            .density = MaterialDensity.concrete,
            .thickness = 4.0,
            .porosity = 0.1,
            .hardness = hardness,
            .toughness = hardness * 0.3,
        },
        .wood => .{
            .density = MaterialDensity.wood,
            .thickness = 3.0,
            .porosity = 0.4,
            .hardness = hardness,
            .toughness = hardness * 0.2,
        },
        .fragile => .{
            .density = MaterialDensity.glass,
            .thickness = 1.0,
            .porosity = 0.0,
            .hardness = hardness,
            .toughness = hardness * 0.1,
        },
        else => .{
            .density = MaterialDensity.concrete,
            .thickness = 2.0,
            .porosity = 0.2,
            .hardness = hardness,
            .toughness = hardness * 0.25,
        },
    };
}

// P8: Calculate penetration depth through a material layer
pub fn calculatePenetrationDepth(
    proj: *const Projectile,
    material: *const MaterialProps,
) f32 {
    const energy = proj.remaining_energy;
    if (energy <= 0) return 0;

    // Ballistic penetration formula: t = (E / (K * A))^(1/n)
    // Simplified: depth proportional to energy / (hardness * thickness * (1 - porosity))
    const resistance = material.hardness * material.thickness * (1.0 - material.porosity);
    const effective_energy = energy * (1.0 - material.porosity * 0.5);

    return @max(0, effective_energy / @max(0.001, resistance));
}

// P8: Apply energy decay after penetrating a layer
pub fn applyPenetrationEnergyLoss(proj: *Projectile, energy_loss: f32) void {
    proj.remaining_energy = @max(0, proj.remaining_energy - energy_loss);
    proj.penetration_distance += proj.penetration_depth;
    proj.layer_count += 1;

    // Scale velocity based on remaining energy
    const new_speed_sq = 2.0 * proj.remaining_energy / proj.mass;
    const current_speed = getSpeed(proj);
    if (current_speed > 0.001) {
        const scale = @sqrt(@max(0, new_speed_sq)) / current_speed;
        proj.vel_x *= scale;
        proj.vel_y *= scale;
        proj.vel_z *= scale;
    }
}

// P8: Ray-voxel intersection for precise penetration tracking
pub fn rayVoxelIntersect(
    orig_x: f32, orig_y: f32, orig_z: f32,
    dir_x: f32, dir_y: f32, dir_z: f32,
    vox_x: i32, vox_y: i32, vox_z: i32,
) ?struct { t_enter: f32, t_exit: f32 } {
    const min_x = @as(f32, @floatFromInt(vox_x));
    const min_y = @as(f32, @floatFromInt(vox_y));
    const min_z = @as(f32, @floatFromInt(vox_z));
    const max_x = min_x + 1.0;
    const max_y = min_y + 1.0;
    const max_z = min_z + 1.0;

    var tmin: f32 = 0.0;
    var tmax: f32 = 1.0;

    const axes = [_]struct { min: f32, max: f32, o: f32, d: f32 }{
        .{ .min = min_x, .max = max_x, .o = orig_x, .d = dir_x },
        .{ .min = min_y, .max = max_y, .o = orig_y, .d = dir_y },
        .{ .min = min_z, .max = max_z, .o = orig_z, .d = dir_z },
    };

    for (axes) |axis| {
        if (@abs(axis.d) < 0.0001) {
            if (axis.o < axis.min or axis.o > axis.max) return null;
        } else {
            const inv_d = 1.0 / axis.d;
            var t1 = (axis.min - axis.o) * inv_d;
            var t2 = (axis.max - axis.o) * inv_d;
            if (t1 > t2) {
                const tmp = t1; t1 = t2; t2 = tmp;
            }
            tmin = @max(tmin, t1);
            tmax = @min(tmax, t2);
            if (tmin > tmax) return null;
        }
    }

    if (tmin < 0) return null;
    return .{ .t_enter = tmin, .t_exit = tmax };
}

// P8: Calculate effective thickness at angle (for angled hits)
pub fn calculateEffectiveThickness(
    thickness: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
) f32 {
    const dot = normal_x * dir_x + normal_y * dir_y + normal_z * dir_z;
    const cos_angle = @abs(dot);
    if (cos_angle < 0.001) return thickness * 100.0; // Nearly parallel - very thick
    return thickness / cos_angle;
}

// P8: Perform layered raycast through entity voxels
pub fn layeredRaycast(
    proj: *const Projectile,
    ent: *const entity16.Entity16,
    entity_id: u16,
    inst_x: i32, inst_y: i32, inst_z: i32,
) LayeredHit {
    var result: LayeredHit = .{ .layer_count = 0, .final_t = 0, .total_penetration = 0, .total_energy_loss = 0 };

    const props = getMaterialProps(ent);
    var energy = proj.remaining_energy;
    var t_current: f32 = 0.0;

    // DDA-style voxel traversal through the entity volume
    const steps: i32 = 32; // Max voxels to traverse
    var step_i: i32 = 0;

    while (step_i < steps and energy > 0) : (step_i += 1) {
        const px = proj.pos_x + proj.vel_x * t_current;
        const py = proj.pos_y + proj.vel_y * t_current;
        const pz = proj.pos_z + proj.vel_z * t_current;

        const vx = @as(i32, @intFromFloat(@floor(px))) - inst_x;
        const vy = @as(i32, @intFromFloat(@floor(py))) - inst_y;
        const vz = @as(i32, @intFromFloat(@floor(pz))) - inst_z;

        if (vx < 0 or vx >= 16 or vy < 0 or vy >= 16 or vz < 0 or vz >= 16) break;

        if (entity16.testVoxel(ent, @intCast(vx), @intCast(vy), @intCast(vz))) {
            if (result.layer_count >= 8) break;

            const hit_idx = result.layer_count;
            result.hits[hit_idx].entity_id = entity_id;
            result.hits[hit_idx].instance_idx = 0;
            result.hits[hit_idx].t_enter = t_current;

            // Calculate penetration through this voxel
            const effective_t = calculateEffectiveThickness(props.thickness, 0, 1, 0, proj.vel_x, proj.vel_y, proj.vel_z);
            const pen_depth = @min(effective_t, energy / @max(0.001, props.hardness));
            const energy_loss = pen_depth * props.hardness * 0.1;

            result.hits[hit_idx].penetration_depth = pen_depth;
            result.hits[hit_idx].energy_loss = energy_loss;
            result.total_energy_loss += energy_loss;
            result.total_penetration += pen_depth;

            energy -= energy_loss;
            result.hits[hit_idx].velocity_after = @sqrt(@max(0, 2.0 * energy / proj.mass));

            t_current += 0.1; // Advance through voxel
            result.layer_count += 1;
        } else {
            t_current += 0.05;
        }

        if (t_current > 32.0) break; // Max trace distance
    }

    result.final_t = t_current;
    return result;
}

// P8: Runtime topology splitting - find connected components after damage
pub const TopologyFragment = struct {
    voxel_count: u16,
    voxels: [128]struct { x: u8, y: u8, z: u8 },
    center_x: f32,
    center_y: f32,
    center_z: f32,
    mass: f32,
};

pub fn splitTopology(
    ent: *entity16.Entity16,
    damage_x: i32, damage_y: i32, damage_z: i32,
    radius: i32,
) [8]TopologyFragment {
    var fragments: [8]TopologyFragment = undefined;
    for (0..8) |i| {
        fragments[i] = .{
            .voxel_count = 0,
            .voxels = undefined,
            .center_x = 0, .center_y = 0, .center_z = 0,
            .mass = 0,
        };
    }

    // Mark damaged voxels
    var damaged: [16][16][16]bool = undefined;
    {
        var x: u8 = 0;
        while (x < 16) : (x += 1) {
            var y: u8 = 0;
            while (y < 16) : (y += 1) {
                var z: u8 = 0;
                while (z < 16) : (z += 1) {
                    damaged[x][y][z] = false;
                }
            }
        }
    }

    var dx: i32 = -radius;
    while (dx <= radius) : (dx += 1) {
        var dy: i32 = -radius;
        while (dy <= radius) : (dy += 1) {
            var dz: i32 = -radius;
            while (dz <= radius) : (dz += 1) {
                const vx = damage_x + dx;
                const vy = damage_y + dy;
                const vz = damage_z + dz;
                if (vx >= 0 and vx < 16 and vy >= 0 and vy < 16 and vz >= 0 and vz < 16) {
                    if (entity16.testVoxel(ent, @intCast(vx), @intCast(vy), @intCast(vz))) {
                        damaged[@intCast(vx)][@intCast(vy)][@intCast(vz)] = true;
                    }
                }
            }
        }
    }

    // BFS to find connected components (simplified - just split by octant)
    var fragment_idx: u8 = 0;
    var temp_voxels: [128]struct { x: u8, y: u8, z: u8 } = undefined;
    var temp_count: u16 = 0;

    {
        var x: u8 = 0;
        while (x < 16) : (x += 1) {
            var y: u8 = 0;
            while (y < 16) : (y += 1) {
                var z: u8 = 0;
                while (z < 16) : (z += 1) {
                    if (damaged[x][y][z] and fragment_idx < 8) {
                        damaged[x][y][z] = false; // Mark as used
                        temp_voxels[temp_count] = .{ .x = x, .y = y, .z = z };
                        temp_count += 1;

                        if (temp_count >= 128 or temp_count >= 64) {
                            // Save current fragment
                            var fx: f32 = 0;
                            var fy: f32 = 0;
                            var fz: f32 = 0;
                            for (0..temp_count) |vi| {
                                fx += @as(f32, @floatFromInt(temp_voxels[vi].x));
                                fy += @as(f32, @floatFromInt(temp_voxels[vi].y));
                                fz += @as(f32, @floatFromInt(temp_voxels[vi].z));
                                fragments[fragment_idx].voxels[vi] = temp_voxels[vi];
                            }
                            fragments[fragment_idx].voxel_count = temp_count;
                            fragments[fragment_idx].center_x = fx / @as(f32, @floatFromInt(temp_count));
                            fragments[fragment_idx].center_y = fy / @as(f32, @floatFromInt(temp_count));
                            fragments[fragment_idx].center_z = fz / @as(f32, @floatFromInt(temp_count));
                            fragments[fragment_idx].mass = @as(f32, @floatFromInt(temp_count)) * 0.1;
                            fragment_idx += 1;
                            temp_count = 0;
                        }
                    }
                }
            }
        }
    }

    // Save remaining voxels
    if (temp_count > 0 and fragment_idx < 8) {
        var fx: f32 = 0;
        var fy: f32 = 0;
        var fz: f32 = 0;
        for (0..temp_count) |vi| {
            fx += @as(f32, @floatFromInt(temp_voxels[vi].x));
            fy += @as(f32, @floatFromInt(temp_voxels[vi].y));
            fz += @as(f32, @floatFromInt(temp_voxels[vi].z));
            fragments[fragment_idx].voxels[vi] = temp_voxels[vi];
        }
        fragments[fragment_idx].voxel_count = temp_count;
        fragments[fragment_idx].center_x = fx / @as(f32, @floatFromInt(temp_count));
        fragments[fragment_idx].center_y = fy / @as(f32, @floatFromInt(temp_count));
        fragments[fragment_idx].center_z = fz / @as(f32, @floatFromInt(temp_count));
        fragments[fragment_idx].mass = @as(f32, @floatFromInt(temp_count)) * 0.1;
    }

    // Clear unused fragments
    while (fragment_idx < 8) : (fragment_idx += 1) {
        fragments[fragment_idx].voxel_count = 0;
        fragments[fragment_idx].mass = 0;
    }

    return fragments;
}

// P8: Recalculate mass and center of mass for damaged entity
pub fn recalculateMassAndCOM(ent: *entity16.Entity16) struct { mass: f32, com_x: f32, com_y: f32, com_z: f32 } {
    var total_mass: f32 = 0;
    var com_x: f32 = 0;
    var com_y: f32 = 0;
    var com_z: f32 = 0;

    {
        var x: u8 = 0;
        while (x < 16) : (x += 1) {
            var y: u8 = 0;
            while (y < 16) : (y += 1) {
                var z: u8 = 0;
                while (z < 16) : (z += 1) {
                    if (entity16.testVoxel(ent, x, y, z)) {
                        const density: f32 = switch (ent.physics.material) {
                            .metal => MaterialDensity.steel,
                            .wood => MaterialDensity.wood,
                            .stone => MaterialDensity.concrete,
                            .fragile => MaterialDensity.glass,
                            else => MaterialDensity.concrete,
                        };
                        const voxel_mass = density * 0.001; // Normalize
                        total_mass += voxel_mass;
                        com_x += @as(f32, @floatFromInt(x)) * voxel_mass;
                        com_y += @as(f32, @floatFromInt(y)) * voxel_mass;
                        com_z += @as(f32, @floatFromInt(z)) * voxel_mass;
                    }
                }
            }
        }
    }

    if (total_mass > 0.001) {
        com_x /= total_mass;
        com_y /= total_mass;
        com_z /= total_mass;
    }

    return .{ .mass = total_mass, .com_x = com_x, .com_y = com_y, .com_z = com_z };
}

// P8: Generate fragment instances from topology fragments
pub fn generateFragmentInstances(
    topology: *const TopologyFragment,
    base_x: i32, base_y: i32, base_z: i32,
    explosion_force: f32,
) [8]FragmentInstance {
    var instances: [8]FragmentInstance = undefined;

    const count = @min(8, topology.voxel_count);
    for (0..count) |i| {
        const v = topology.voxels[i];
        const world_x = @as(f32, @floatFromInt(base_x)) + @as(f32, @floatFromInt(v.x));
        const world_y = @as(f32, @floatFromInt(base_y)) + @as(f32, @floatFromInt(v.y));
        const world_z = @as(f32, @floatFromInt(base_z)) + @as(f32, @floatFromInt(v.z));

        // Random velocity based on explosion
        const angle = @as(f32, @floatFromInt(i)) * 0.785 + 0.1;
        const speed = explosion_force * (0.5 + @as(f32, @floatFromInt((i * 7) % 10)) * 0.1);

        instances[i] = .{
            .pos_x = world_x,
            .pos_y = world_y,
            .pos_z = world_z,
            .vel_x = @cos(angle) * speed,
            .vel_y = speed * 0.7,
            .vel_z = @sin(angle) * speed,
            .size = 0.5 + @as(f32, @floatFromInt(i % 3)) * 0.2,
            .mass = topology.mass / @as(f32, @floatFromInt(count)),
            .entity_id = 0,
        };
    }

    // Clear remaining
    for (count..8) |i| {
        instances[i] = .{
            .pos_x = 0, .pos_y = 0, .pos_z = 0,
            .vel_x = 0, .vel_y = 0, .vel_z = 0,
            .size = 0, .mass = 0, .entity_id = 0,
        };
    }

    return instances;
}

/// Check if projectile hits entity based on AABB
pub fn checkEntityHit(
    proj: *const Projectile,
    ent_pos_x: i32,
    ent_pos_y: i32,
    ent_pos_z: i32,
    _: *const entity16.Entity16,
) RayHit {
    var t_min: f32 = 0.0;
    var t_max: f32 = 1.0;

    const px = proj.pos_x;
    const py = proj.pos_y;
    const pz = proj.pos_z;
    const dx = proj.vel_x;
    const dy = proj.vel_y;
    const dz = proj.vel_z;

    const min_x = @as(f32, @floatFromInt(ent_pos_x));
    const min_y = @as(f32, @floatFromInt(ent_pos_y));
    const min_z = @as(f32, @floatFromInt(ent_pos_z));
    const max_x = min_x + 16.0;
    const max_y = min_y + 16.0;
    const max_z = min_z + 16.0;

    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    const axes = [_]struct { min: f32, max: f32, pos: f32, vel: f32, norm: f32 }{
        .{ .min = min_x, .max = max_x, .pos = px, .vel = dx, .norm = -1.0 },
        .{ .min = min_y, .max = max_y, .pos = py, .vel = dy, .norm = -1.0 },
        .{ .min = min_z, .max = max_z, .pos = pz, .vel = dz, .norm = -1.0 },
    };

    for (axes) |axis| {
        if (@abs(axis.vel) < 0.0001) {
            if (axis.pos < axis.min or axis.pos > axis.max) {
                return .{ .hit = false, .t = 0, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 0 };
            }
        } else {
            const inv_d = 1.0 / axis.vel;
            var t1 = (axis.min - axis.pos) * inv_d;
            var t2 = (axis.max - axis.pos) * inv_d;
            var norm = axis.norm;

            if (t1 > t2) {
                const tmp = t1; t1 = t2; t2 = tmp;
                norm = -norm;
            }

            if (t1 > t_min) {
                t_min = t1;
                normal_x = if (axis.norm < 0) -1.0 else 0.0;
                normal_y = if (axis.norm < 0) -1.0 else 0.0;
                normal_z = if (axis.norm < 0) -1.0 else 0.0;
                if (axis.norm < 0) {
                    normal_x = 0; normal_y = 0; normal_z = 0;
                    if (axis.norm == -1.0) normal_x = 1.0;
                }
            }

            t_max = @min(t_max, t2);
            if (t_min > t_max) return .{ .hit = false, .t = 0, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 0 };
        }
    }

    if (t_min < 0) return .{ .hit = false, .t = 0, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 0 };

    return .{
        .hit = true,
        .t = t_min,
        .normal_x = normal_x,
        .normal_y = normal_y,
        .normal_z = normal_z,
        .entity_id = 0,
    };
}

/// Check penetration capability
pub fn checkPenetration(
    proj: *const Projectile,
    entity: *const entity16.Entity16,
    thickness: f32,
) bool {
    const energy = getKineticEnergy(proj);
    const hardness = @as(f32, @floatFromInt(entity.physics.hardness));
    const material_factor: f32 = switch (entity.physics.material) {
        .metal => 2.0,
        .stone => 1.5,
        .wood => 0.8,
        .fragile => 0.3,
        else => 1.0,
    };

    const penetration_threshold = hardness * thickness * material_factor * 10.0;
    return energy > penetration_threshold;
}

/// Calculate deflection angle based on surface normal
pub fn calculateDeflection(
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    restitution: f32,
) struct { x: f32, y: f32, z: f32 } {
    const dot = vel_x * normal_x + vel_y * normal_y + vel_z * normal_z;
    return .{
        .x = (vel_x - 2.0 * dot * normal_x) * restitution,
        .y = (vel_y - 2.0 * dot * normal_y) * restitution,
        .z = (vel_z - 2.0 * dot * normal_z) * restitution,
    };
}

/// Apply drag to projectile
pub fn applyDrag(proj: *Projectile, drag_coefficient: f32, dt: f32) void {
    const speed = getSpeed(proj);
    if (speed < 0.001) return;

    const drag = drag_coefficient * speed * speed * dt;
    const scale = @max(0.0, 1.0 - drag / speed);

    proj.vel_x *= scale;
    proj.vel_y *= scale;
    proj.vel_z *= scale;
}

/// Apply gravity drop
pub fn applyGravityDrop(proj: *Projectile, gravity: f32, dt: f32) void {
    proj.vel_y += gravity * dt;
}

/// Simulate projectile for one tick (P8 enhanced)
pub fn simulate(
    proj: *Projectile,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    dt: f32,
) void {
    if (proj.state != .active) return;

    proj.lifetime += 1;
    if (proj.lifetime > 1000) {
        proj.state = .stopped;
        return;
    }

    applyDrag(proj, 0.001, dt);
    applyGravityDrop(proj, -100.0, dt);

    // Update remaining energy from current velocity
    const current_speed = getSpeed(proj);
    proj.remaining_energy = 0.5 * proj.mass * current_speed * current_speed;

    const steps: u8 = 4;
    const step_dt = dt / @as(f32, @floatFromInt(steps));
    var i: u8 = 0;

    while (i < steps) : (i += 1) {
        proj.pos_x += proj.vel_x * step_dt;
        proj.pos_y += proj.vel_y * step_dt;
        proj.pos_z += proj.vel_z * step_dt;

        for (0..s1024.instance_count) |j| {
            const inst = &s1024.instances[j];
            if (inst.entity_id >= entities.len) continue;

            const hit = checkEntityHit(proj, inst.pos_x, inst.pos_y, inst.pos_z, &entities[inst.entity_id]);
            if (hit.hit) {
                const entity = &entities[inst.entity_id];
                const props = getMaterialProps(entity);

                // P8: Use layered raycast for detailed penetration
                const layered = layeredRaycast(proj, entity, inst.entity_id, inst.pos_x, inst.pos_y, inst.pos_z);

                if (layered.layer_count > 0 and proj.remaining_energy > layered.total_energy_loss) {
                    // P8: Penetrated - apply energy decay
                    proj.remaining_energy -= layered.total_energy_loss;
                    proj.penetration_depth += layered.total_penetration;
                    proj.layer_count += layered.layer_count;

                    // Scale velocity based on remaining energy
                    const new_speed_sq = 2.0 * proj.remaining_energy / proj.mass;
                    const scale = @sqrt(@max(0, new_speed_sq)) / @max(0.001, current_speed);
                    proj.vel_x *= scale;
                    proj.vel_y *= scale;
                    proj.vel_z *= scale;

                    if (layered.layer_count > 1) {
                        proj.state = .multi_penetrated;
                    } else {
                        proj.state = .penetrated;
                    }

                    // Check if entity should be damaged/destroyed
                    if (layered.total_penetration > 4.0) {
                        // P8: Trigger topology split for enough damage
                        const damage_x = @as(i32, @intFromFloat(@floor(proj.pos_x))) - inst.pos_x;
                        const damage_y = @as(i32, @intFromFloat(@floor(proj.pos_y))) - inst.pos_y;
                        const damage_z = @as(i32, @intFromFloat(@floor(proj.pos_z))) - inst.pos_z;
                        _ = splitTopology(entity, damage_x, damage_y, damage_z, 2);
                    }
                    return;
                } else {
                    // P8: Deflected or stopped
                    const def = calculateDeflection(
                        proj.vel_x, proj.vel_y, proj.vel_z,
                        hit.normal_x, hit.normal_y, hit.normal_z,
                        proj.restitution,
                    );
                    proj.vel_x = def.x;
                    proj.vel_y = def.y;
                    proj.vel_z = def.z;

                    if (proj.remaining_energy < props.hardness * 0.5) {
                        proj.state = .stopped;
                    } else {
                        proj.state = .deflected;
                    }
                    return;
                }
            }
        }
    }

    if (getSpeed(proj) < 1.0) {
        proj.state = .stopped;
    }
}

/// Simulate all active projectiles
pub fn simulateAll(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    dt: f32,
) void {
    for (0..g_projectile_system.count) |i| {
        simulate(&g_projectile_system.projectiles[i], s1024, entities, dt);
    }
}

/// Apply explosion shockwave to nearby entities
pub fn applyExplosionShockwave(
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    radius: f32,
    force: f32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) void {
    for (0..s1024.instance_count) |i| {
        const inst = &s1024.instances[i];
        if (inst.entity_id >= entities.len) continue;

        const ent = &entities[inst.entity_id];
        if ((ent.physics.flags & 0x01) != 0) continue;

        const dx = @as(f32, @floatFromInt(inst.pos_x)) - pos_x;
        const dy = @as(f32, @floatFromInt(inst.pos_y)) - pos_y;
        const dz = @as(f32, @floatFromInt(inst.pos_z)) - pos_z;
        const dist = @sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < radius and dist > 0.1) {
            const falloff = 1.0 - (dist / radius);
            const impulse = force * falloff / @as(f32, @floatFromInt(ent.physics.mass));
            const nx = dx / dist;
            const ny = dy / dist;
            const nz = dz / dist;

            inst.vel_x = @truncate(@as(i32, @intFromFloat(@round(nx * impulse))));
            inst.vel_y = @truncate(@as(i32, @intFromFloat(@round(ny * impulse))));
            inst.vel_z = @truncate(@as(i32, @intFromFloat(@round(nz * impulse))));
        }
    }
}

/// Generate fragments from explosion
pub const Fragment = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    lifetime: u8,
};

pub const MAX_FRAGMENTS: usize = 32;

pub fn generateFragments(
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    count: u8,
    force: f32,
) [MAX_FRAGMENTS]Fragment {
    var fragments: [MAX_FRAGMENTS]Fragment = undefined;
    var i: u8 = 0;

    while (i < count) : (i += 1) {
        const angle = @as(f32, @floatFromInt(i)) * 3.14159 * 2.0 / @as(f32, @floatFromInt(count));
        const spread = @as(f32, @floatFromInt((i * 17) % 10)) * 0.1 + 0.5;

        fragments[i] = .{
            .pos_x = pos_x,
            .pos_y = pos_y,
            .pos_z = pos_z,
            .vel_x = @cos(angle) * force * spread,
            .vel_y = force * 0.5 * spread,
            .vel_z = @sin(angle) * force * spread,
            .lifetime = 30,
        };
    }

    while (i < MAX_FRAGMENTS) : (i += 1) {
        fragments[i] = .{
            .pos_x = 0, .pos_y = 0, .pos_z = 0,
            .vel_x = 0, .vel_y = 0, .vel_z = 0,
            .lifetime = 0,
        };
    }

    return fragments;
}

/// Update fragments
pub fn updateFragments(fragments: *[MAX_FRAGMENTS]Fragment, dt: f32) void {
    var i: usize = 0;
    while (i < MAX_FRAGMENTS) : (i += 1) {
        if (fragments[i].lifetime > 0) {
            fragments[i].lifetime -= 1;
            fragments[i].pos_x += fragments[i].vel_x * dt;
            fragments[i].pos_y += fragments[i].vel_y * dt;
            fragments[i].pos_z += fragments[i].vel_z * dt;
            fragments[i].vel_y -= 200.0 * dt;
        }
    }
}

/// Get system for external iteration
pub fn getSystem() *ProjectileSystem {
    return &g_projectile_system;
}
