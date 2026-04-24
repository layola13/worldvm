//! Collision and Damage System - Impact, Deformation, and Structural Integrity
//!
//! Phase 30: Collision response, damage models, deformation, crumple zones
//! Handles: Impact forces, energy absorption, structural failure, collision detection

const std = @import("std");

pub const CollisionResult = struct {
    collided: bool,
    penetration_depth: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    impact_speed: f32,
    impact_energy: f32,
    contact_x: f32,
    contact_y: f32,
    contact_z: f32,
};

pub const DamageState = struct {
    hood_damage: f32,
    front_left_damage: f32,
    front_right_damage: f32,
    rear_left_damage: f32,
    rear_right_damage: f32,
    roof_damage: f32,
    structural_integrity: f32,
    deployed_airbags: u8,
    engine_damage: f32,
    transmission_damage: f32,
};

pub const CollisionConfig = struct {
    crumple_zones_front: f32 = 0.3,
    crumple_zones_rear: f32 = 0.25,
    crumple_zones_side: f32 = 0.15,
    structural_strength: f32 = 1.0,
    energy_absorption_rate: f32 = 0.8,
    min_impact_for_damage: f32 = 2.0,
};

pub const CollisionSystem = struct {
    damage: DamageState,
    last_collision: CollisionResult,
    collision_count: u16,
};

var g_collision_system: CollisionSystem = undefined;

pub fn init() void {
    g_collision_system.damage = .{
        .hood_damage = 0,
        .front_left_damage = 0,
        .front_right_damage = 0,
        .rear_left_damage = 0,
        .rear_right_damage = 0,
        .roof_damage = 0,
        .structural_integrity = 100,
        .deployed_airbags = 0,
        .engine_damage = 0,
        .transmission_damage = 0,
    };
    g_collision_system.last_collision = .{
        .collided = false,
        .penetration_depth = 0,
        .normal_x = 0,
        .normal_y = 0,
        .normal_z = 0,
        .impact_speed = 0,
        .impact_energy = 0,
        .contact_x = 0,
        .contact_y = 0,
        .contact_z = 0,
    };
    g_collision_system.collision_count = 0;
}

pub fn calculateImpactEnergy(mass1: f32, mass2: f32, relative_velocity: f32) f32 {
    const reduced_mass = (mass1 * mass2) / (mass1 + mass2 + 0.001);
    return 0.5 * reduced_mass * relative_velocity * relative_velocity;
}

pub fn calculateDamage(impact_energy: f32, config: CollisionConfig) f32 {
    if (impact_energy < config.min_impact_for_damage) return 0;
    const absorbed_energy = impact_energy * config.energy_absorption_rate;
    return absorbed_energy / 100.0;
}

pub fn applyCollision(
    pos_x: f32, pos_y: f32, pos_z: f32,
    vel_x: f32, vel_y: f32, vel_z: f32,
    mass: f32,
    config: CollisionConfig
) CollisionResult {
    var result: CollisionResult = .{
        .collided = false,
        .penetration_depth = 0,
        .normal_x = 0,
        .normal_y = 1,
        .normal_z = 0,
        .impact_speed = @sqrt(vel_x * vel_x + vel_y * vel_y + vel_z * vel_z),
        .impact_energy = 0,
        .contact_x = pos_x,
        .contact_y = pos_y,
        .contact_z = pos_z,
    };

    if (result.impact_speed < 0.1) return result;

    const impact_energy = calculateImpactEnergy(mass, 1500, result.impact_speed);
    result.impact_energy = impact_energy;
    result.collided = true;

    if (impact_energy > config.min_impact_for_damage) {
        const damage = calculateDamage(impact_energy, config);
        var dmg = &g_collision_system.damage;

        const front_factor = if (vel_z < 0) @abs(vel_z) / result.impact_speed else 0;
        const rear_factor = if (vel_z > 0) @abs(vel_z) / result.impact_speed else 0;

        dmg.hood_damage = @min(100, dmg.hood_damage + damage * front_factor * 100);
        dmg.front_left_damage = @min(100, dmg.front_left_damage + damage * front_factor * 80);
        dmg.front_right_damage = @min(100, dmg.front_right_damage + damage * front_factor * 80);
        dmg.rear_left_damage = @min(100, dmg.rear_left_damage + damage * rear_factor * 80);
        dmg.rear_right_damage = @min(100, dmg.rear_right_damage + damage * rear_factor * 80);

        const total_damage = (dmg.hood_damage + dmg.front_left_damage + dmg.front_right_damage +
                           dmg.rear_left_damage + dmg.rear_right_damage + dmg.roof_damage) / 6.0;
        dmg.structural_integrity = 100.0 - total_damage;

        dmg.engine_damage = @min(100, dmg.engine_damage + damage * front_factor * 50);
        dmg.transmission_damage = @min(100, dmg.transmission_damage + damage * (front_factor + rear_factor) * 30);

        if (impact_energy > 50 and dmg.deployed_airbags < 6) {
            dmg.deployed_airbags = 6;
        } else if (impact_energy > 30 and dmg.deployed_airbags < 4) {
            dmg.deployed_airbags = 4;
        } else if (impact_energy > 15 and dmg.deployed_airbags < 2) {
            dmg.deployed_airbags = 2;
        }

        g_collision_system.collision_count += 1;
    }

    g_collision_system.last_collision = result;
    return result;
}

pub fn checkStructuralFailure() bool {
    return g_collision_system.damage.structural_integrity < 20;
}

pub fn getCollisionResult() CollisionResult {
    return g_collision_system.last_collision;
}

pub fn getDamageState() *DamageState {
    return &g_collision_system.damage;
}

pub fn repairDamage(amount: f32) void {
    var dmg = &g_collision_system.damage;
    dmg.hood_damage = @max(0, dmg.hood_damage - amount);
    dmg.front_left_damage = @max(0, dmg.front_left_damage - amount);
    dmg.front_right_damage = @max(0, dmg.front_right_damage - amount);
    dmg.rear_left_damage = @max(0, dmg.rear_left_damage - amount);
    dmg.rear_right_damage = @max(0, dmg.rear_right_damage - amount);
    dmg.roof_damage = @max(0, dmg.roof_damage - amount);
    dmg.engine_damage = @max(0, dmg.engine_damage - amount * 0.5);
    dmg.transmission_damage = @max(0, dmg.transmission_damage - amount * 0.5);

    const total_damage = (dmg.hood_damage + dmg.front_left_damage + dmg.front_right_damage +
                         dmg.rear_left_damage + dmg.rear_right_damage + dmg.roof_damage) / 6.0;
    dmg.structural_integrity = 100.0 - total_damage;
}

pub fn getCollisionCount() u16 {
    return g_collision_system.collision_count;
}
