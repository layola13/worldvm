//! Ballistics - Projectile Simulation System
//!
//! Phase 8: Projectile physics, penetration, deflection, fragmentation
//! Handles: bullet simulation, armor penetration, ricochet, explosion shockwaves

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
    proj.* = .{
        .pos_x = pos_x, .pos_y = pos_y, .pos_z = pos_z,
        .vel_x = vel_x, .vel_y = vel_y, .vel_z = vel_z,
        .mass = mass, .caliber = caliber, .entity_id = 0,
        .state = .active, .penetration_depth = 0, .restitution = 0.3,
        .lifetime = 0,
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

/// Simulate projectile for one tick
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
                const thickness: f32 = 8.0;

                if (checkPenetration(proj, entity, thickness)) {
                    proj.state = .penetrated;
                    proj.penetration_depth += thickness;
                    return;
                } else {
                    const def = calculateDeflection(
                        proj.vel_x, proj.vel_y, proj.vel_z,
                        hit.normal_x, hit.normal_y, hit.normal_z,
                        proj.restitution,
                    );
                    proj.vel_x = def.x;
                    proj.vel_y = def.y;
                    proj.vel_z = def.z;
                    proj.state = .deflected;
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
