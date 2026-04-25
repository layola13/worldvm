//! Rewind and Determinism System - Full World State Recording
//!
//! Phase P4: World-level rollback and determinism
//! Handles: World snapshots, rollback, determinism checks, ghost replay
//!
//! Features:
//! - Full world state snapshots (instances, entities, subsystems)
//! - World state hashing for determinism verification
//! - Input logging for replay
//! - Fast-forward replay from recorded state
//! - Deterministic math enforcement

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const tick_engine = @import("tick_engine.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const joint = @import("joint.zig");
const ragdoll = @import("ragdoll.zig");
const ballistics = @import("ballistics.zig");
const destruction = @import("destruction.zig");
const terrain = @import("terrain.zig");
const disasters = @import("disasters.zig");
const collision = @import("collision.zig");
const crash_defense = @import("crash_defense.zig");
const sensors = @import("sensors.zig");
const network = @import("network.zig");
const tire = @import("tire.zig");
const suspension = @import("suspension.zig");
const drivetrain = @import("drivetrain.zig");
const aerodynamics = @import("aerodynamics.zig");
const braking = @import("braking.zig");
const ai_traffic = @import("ai_traffic.zig");

// ============================================================================
// Core Types
// ============================================================================

pub const RewindState = struct {
    tick: u32,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_y: f32,
    vel_z: f32,
    yaw: f32,
    pitch: f32,
    roll: f32,
    input_forwards: bool,
    input_backwards: bool,
    input_left: bool,
    input_right: bool,
    input_jump: bool,
    input_brake: bool,
};

pub const DeterminismProof = struct {
    initial_state_hash: u64,
    final_state_hash: u64,
    tick_count: u32,
    mismatches: u16,
    verified: bool,
};

pub const WorldSnapshotDiff = struct {
    tick_from: u32,
    tick_to: u32,
    hash_changed: bool,
    instance_count_changed: bool,
    instances_moved: u16,
    kcc_count_changed: bool,
    kcc_changed: u8,
    vehicle_count_changed: bool,
    vehicles_changed: u8,
    joint_count_changed: bool,
    joints_changed: u8,
    ragdoll_count_changed: bool,
    ragdolls_changed: u8,
    projectile_count_changed: bool,
    projectiles_changed: u8,
    destroyable_count_changed: bool,
    destroyables_changed: u8,
    terrain_patch_count_changed: bool,
    terrain_changed: bool,
    disaster_count_changed: bool,
    disasters_changed: bool,
    collision_changed: bool,
    defense_changed: bool,
    sensor_changed: bool,
    network_changed: bool,
    tire_changed: bool,
    suspension_changed: bool,
    drivetrain_changed: bool,
    aero_changed: bool,
    braking_changed: bool,
    debris_changed: bool,
    ai_traffic_changed: bool,
};

pub const MAX_REWIND_STATES: usize = 1200;
pub const MAX_INPUT_LOG: usize = 4096;

pub const InputLog = struct {
    tick: u32,
    forward: bool,
    backward: bool,
    left: bool,
    right: bool,
    jump: bool,
    brake: bool,
    mouse_dx: f32,
    mouse_dy: f32,
};

// ============================================================================
// World State Snapshot
// ============================================================================

pub const InstanceSnapshot = struct {
    entity_id: u16,
    pos_x: i32,
    pos_y: i32,
    pos_z: i32,
    rot_yaw: u8,
    rot_pitch: u8,
    rot_roll: u8,
    state: scene32.InstanceState,
    vel_x: i16,
    vel_y: i16,
    vel_z: i16,
    ang_x: i8,
    ang_y: i8,
    ang_z: i8,
    sleep_tick: u8,
};

pub const WorldSnapshot = struct {
    tick: u32,
    global_tick: u32,

    // Instance states
    instance_count: u8,
    instances: [scene32.MAX_INSTANCES]InstanceSnapshot,

    // Entity states (simplified - just flags that affect physics)
    entity_hp: [256]f32,  // Health for destructible entities

    // KCC states
    kcc_count: u8,
    kcc_positions: [kcc.MAX_KCC][3]f32,
    kcc_velocities: [kcc.MAX_KCC][3]f32,
    kcc_grounded: [kcc.MAX_KCC]bool,
    kcc_crouching: [kcc.MAX_KCC]bool,
    kcc_jumping: [kcc.MAX_KCC]bool,
    kcc_yaw: [kcc.MAX_KCC]f32,
    kcc_ground_normals: [kcc.MAX_KCC][3]f32,
    kcc_stand_height: [kcc.MAX_KCC]i32,
    kcc_crouch_height: [kcc.MAX_KCC]i32,
    kcc_radius: [kcc.MAX_KCC]i32,
    kcc_move_speed: [kcc.MAX_KCC]f32,
    kcc_jump_force: [kcc.MAX_KCC]f32,
    kcc_gravity: [kcc.MAX_KCC]f32,
    kcc_crouch_speed_mult: [kcc.MAX_KCC]f32,
    kcc_push_force: [kcc.MAX_KCC]f32,
    kcc_step_height: [kcc.MAX_KCC]i32,
    kcc_max_slope_angle: [kcc.MAX_KCC]f32,
    kcc_step_offset: [kcc.MAX_KCC]f32,

    // Vehicle states (simplified)
    vehicle_count: u8,
    vehicle_positions: [vehicle.MAX_VEHICLES][3]f32,
    vehicle_velocities: [vehicle.MAX_VEHICLES][3]f32,
    vehicle_yaw: [vehicle.MAX_VEHICLES]f32,

    // Joint states
    joint_count: u8,
    joints: [joint.MAX_JOINTS]joint.Joint,

    // Ragdoll states (minimal runtime state for preview/restore)
    ragdoll_count: u8,
    ragdoll_part_count: [ragdoll.MAX_RAGDOLLS]u8,
    ragdoll_base_positions: [ragdoll.MAX_RAGDOLLS][3]i32,
    ragdoll_active: [ragdoll.MAX_RAGDOLLS]bool,
    ragdoll_resurrection_tick: [ragdoll.MAX_RAGDOLLS]u16,
    ragdoll_part_positions: [ragdoll.MAX_RAGDOLLS][ragdoll.MAX_RAGDOLL_PARTS][3]f32,
    ragdoll_part_velocities: [ragdoll.MAX_RAGDOLLS][ragdoll.MAX_RAGDOLL_PARTS][3]f32,

    // Projectile states (minimal runtime state for preview/restore)
    projectile_count: u8,
    projectile_positions: [ballistics.MAX_PROJECTILES][3]f32,
    projectile_velocities: [ballistics.MAX_PROJECTILES][3]f32,
    projectile_mass: [ballistics.MAX_PROJECTILES]f32,
    projectile_caliber: [ballistics.MAX_PROJECTILES]f32,
    projectile_state: [ballistics.MAX_PROJECTILES]ballistics.ProjectileState,
    projectile_lifetime: [ballistics.MAX_PROJECTILES]u16,
    projectile_remaining_energy: [ballistics.MAX_PROJECTILES]f32,
    projectile_penetration_distance: [ballistics.MAX_PROJECTILES]f32,
    projectile_layer_count: [ballistics.MAX_PROJECTILES]u8,

    // Destruction states (minimal persistent damage state)
    destroyable_count: u8,
    destroyable_entity_id: [destruction.MAX_DESTROYABLE]u16,
    destroyable_hp: [destruction.MAX_DESTROYABLE]f32,
    destroyable_broken: [destruction.MAX_DESTROYABLE]bool,
    destroyable_damage_state: [destruction.MAX_DESTROYABLE]destruction.DamageState,
    destroyable_integrity_ratio: [destruction.MAX_DESTROYABLE]f32,
    destroyable_crack_density: [destruction.MAX_DESTROYABLE]f32,

    // Terrain/environment state
    terrain_patch_count: u8,
    terrain_patches: [terrain.MAX_TERRAIN_PATCHES]terrain.TerrainPatch,
    terrain_weather: terrain.WeatherCondition,
    terrain_global_friction_modifier: f32,

    // Disaster/environment state
    disaster_count: u8,
    active_disasters: [disasters.MAX_ACTIVE_DISASTERS]disasters.DisasterEvent,
    disaster_chain_reaction_enabled: bool,
    disaster_apocalypse_mode: bool,
    disaster_seismic_wave: disasters.SeismicWave,
    disaster_atmosphere: disasters.AtmosphericEvent,

    // Collision state
    collision_damage: collision.DamageState,
    collision_last_result: collision.CollisionResult,
    collision_count: u16,

    // Crash defense state
    defense_config: crash_defense.SanityCheckConfig,
    defense_last_progress_tick: u32,
    defense_emergency_stopped: bool,
    defense_snapshot_count: u8,

    // Sensor state
    sensor_count: u8,
    sensors: [sensors.MAX_SENSORS]sensors.SensorState,
    sensor_fusion: sensors.SensorFusionState,
    sensor_degradation_factor: f32,
    sensor_interference_level: f32,

    // Network state
    network_replica_count: u8,
    network_replicas: [network.MAX_REPLICAS]network.ReplicaState,
    network_input_count: u16,
    network_inputs: [network.MAX_INPUTS]network.InputState,
    network_local_tick: u32,
    network_remote_tick: u32,
    network_config: network.SyncConfig,
    network_last_sync_tick: u32,
    network_crc_errors: u16,

    // Tire state
    tire_count: u8,
    tires: [tire.MAX_TIRES]tire.TireState,

    // Suspension state
    suspension_count: u8,
    suspensions: [suspension.MAX_WHEELS]suspension.SuspensionState,

    // Drivetrain state
    drivetrain_state: drivetrain.DrivetrainSystem,

    // Aerodynamics state
    aero_state: aerodynamics.AeroState,
    aero_devices: [aerodynamics.MAX_AERO_DEVICES]struct {
        active: bool,
        drag_delta: f32,
        downforce_delta: f32,
    },
    aero_device_count: u8,

    // Braking state
    brake_system: braking.BrakeSystem,

    // Debris state
    debris_count: u16,
    debris: [destruction.MAX_DEBRIS]destruction.Debris,

    // AI traffic state
    ai_traffic_vehicle_count: u8,
    ai_traffic_vehicles: [ai_traffic.MAX_AI_VEHICLES]ai_traffic.TrafficVehicle,
    ai_traffic_light_count: u8,
    ai_traffic_lights: [ai_traffic.MAX_TRAFFIC_LIGHTS]ai_traffic.TrafficLight,
    ai_traffic_global_time: f32,

    // Input for replay
    input_log: [MAX_INPUT_LOG]InputLog,
    input_count: u16,

    // World hash for determinism
    world_hash: u64,
};

pub const RewindSystem = struct {
    // Per-entity rewind states (legacy)
    states: [MAX_REWIND_STATES]RewindState,
    state_count: u16,
    current_index: u16,
    max_tick: u32,
    proof: DeterminismProof,
    deterministic: bool,

    // Full world snapshots
    world_snapshots: [120]WorldSnapshot,
    world_snapshot_count: u8,
    world_snapshot_index: u8,

    // Input log
    input_log: [MAX_INPUT_LOG]InputLog,
    input_count: u16,
    max_input_tick: u32,

    // Determinism verification
    proof_ticks: u32,
    proof_initial_hash: u64,
    proof_final_hash: u64,
};

var g_rewind_system: RewindSystem = undefined;

pub fn init() void {
    g_rewind_system.state_count = 0;
    g_rewind_system.current_index = 0;
    g_rewind_system.max_tick = 0;
    g_rewind_system.proof = .{
        .initial_state_hash = 0,
        .final_state_hash = 0,
        .tick_count = 0,
        .mismatches = 0,
        .verified = false,
    };
    g_rewind_system.deterministic = true;
    g_rewind_system.world_snapshot_count = 0;
    g_rewind_system.world_snapshot_index = 0;
    g_rewind_system.input_count = 0;
    g_rewind_system.max_input_tick = 0;
    g_rewind_system.proof_ticks = 0;
    g_rewind_system.proof_initial_hash = 0;
    g_rewind_system.proof_final_hash = 0;
}

// ============================================================================
// World State Snapshot Operations
// ============================================================================

/// Capture full world state snapshot
pub fn captureWorldSnapshot(
    tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) WorldSnapshot {
    var snapshot: WorldSnapshot = undefined;
    snapshot.tick = tick;
    snapshot.global_tick = s1024.global_tick;
    snapshot.instance_count = s1024.instance_count;

    // Capture instance states
    var i: u8 = 0;
    while (i < s1024.instance_count and i < scene32.MAX_INSTANCES) : (i += 1) {
        const inst = &s1024.instances[i];
        snapshot.instances[i] = .{
            .entity_id = inst.entity_id,
            .pos_x = inst.pos_x,
            .pos_y = inst.pos_y,
            .pos_z = inst.pos_z,
            .rot_yaw = inst.rot_yaw,
            .rot_pitch = inst.rot_pitch,
            .rot_roll = inst.rot_roll,
            .state = inst.state,
            .vel_x = inst.vel_x,
            .vel_y = inst.vel_y,
            .vel_z = inst.vel_z,
            .ang_x = inst.ang_x,
            .ang_y = inst.ang_y,
            .ang_z = inst.ang_z,
            .sleep_tick = inst.sleep_tick,
        };
    }

    // Capture entity HP (for destructibles)
    i = 0;
    while (i < entities.len and i < 256) : (i += 1) {
        snapshot.entity_hp[i] = @as(f32, @floatFromInt(entities[i].physics.hardness));  // Reuse hardness field as HP proxy
    }

    // Capture KCC states
    const kcc_sys = kcc.getSystem();
    snapshot.kcc_count = kcc_sys.count;
    i = 0;
    while (i < kcc_sys.count and i < kcc.MAX_KCC) : (i += 1) {
        const k = &kcc_sys.characters[i];
        snapshot.kcc_positions[i] = .{ k.pos_x, k.pos_y, k.pos_z };
        snapshot.kcc_velocities[i] = .{ k.vel_x, k.vel_y, k.vel_z };
        snapshot.kcc_grounded[i] = k.grounded;
        snapshot.kcc_crouching[i] = k.crouching;
        snapshot.kcc_jumping[i] = k.jumping;
        snapshot.kcc_yaw[i] = k.yaw;
        snapshot.kcc_ground_normals[i] = .{ k.ground_normal_x, k.ground_normal_y, k.ground_normal_z };
        snapshot.kcc_stand_height[i] = k.stand_height;
        snapshot.kcc_crouch_height[i] = k.crouch_height;
        snapshot.kcc_radius[i] = k.radius;
        snapshot.kcc_move_speed[i] = k.move_speed;
        snapshot.kcc_jump_force[i] = k.jump_force;
        snapshot.kcc_gravity[i] = k.gravity;
        snapshot.kcc_crouch_speed_mult[i] = k.crouch_speed_mult;
        snapshot.kcc_push_force[i] = k.push_force;
        snapshot.kcc_step_height[i] = k.step_height;
        snapshot.kcc_max_slope_angle[i] = k.max_slope_angle;
        snapshot.kcc_step_offset[i] = k.step_offset;
    }

    // Capture Vehicle states
    const vehicle_sys = vehicle.getSystem();
    snapshot.vehicle_count = vehicle_sys.count;
    i = 0;
    while (i < vehicle_sys.count and i < vehicle.MAX_VEHICLES) : (i += 1) {
        const v = &vehicle_sys.vehicles[i];
        snapshot.vehicle_positions[i] = .{ v.pos_x, v.pos_y, v.pos_z };
        snapshot.vehicle_velocities[i] = .{ v.speed, v.angular_velocity, 0.0 };
        snapshot.vehicle_yaw[i] = v.yaw;
    }

    // Capture joint states
    const joint_sys = joint.getSystem();
    snapshot.joint_count = joint_sys.joint_count;
    i = 0;
    while (i < joint_sys.joint_count and i < joint.MAX_JOINTS) : (i += 1) {
        snapshot.joints[i] = joint_sys.joints[i];
    }

    // Capture Ragdoll count (simplified)
    const ragdoll_sys = ragdoll.getSystem();
    snapshot.ragdoll_count = ragdoll_sys.count;
    i = 0;
    while (i < ragdoll_sys.count and i < ragdoll.MAX_RAGDOLLS) : (i += 1) {
        const r = &ragdoll_sys.ragdolls[i];
        snapshot.ragdoll_part_count[i] = r.part_count;
        snapshot.ragdoll_base_positions[i] = .{ r.base_x, r.base_y, r.base_z };
        snapshot.ragdoll_active[i] = r.active;
        snapshot.ragdoll_resurrection_tick[i] = r.resurrection_tick;

        var part_idx: u8 = 0;
        while (part_idx < r.part_count and part_idx < ragdoll.MAX_RAGDOLL_PARTS) : (part_idx += 1) {
            const part = r.parts[part_idx];
            snapshot.ragdoll_part_positions[i][part_idx] = .{ part.pos_x, part.pos_y, part.pos_z };
            snapshot.ragdoll_part_velocities[i][part_idx] = .{ part.vel_x, part.vel_y, part.vel_z };
        }
    }

    // Capture Projectile runtime state
    const proj_sys = ballistics.getSystem();
    snapshot.projectile_count = proj_sys.count;
    i = 0;
    while (i < proj_sys.count and i < ballistics.MAX_PROJECTILES) : (i += 1) {
        const p = &proj_sys.projectiles[i];
        snapshot.projectile_positions[i] = .{ p.pos_x, p.pos_y, p.pos_z };
        snapshot.projectile_velocities[i] = .{ p.vel_x, p.vel_y, p.vel_z };
        snapshot.projectile_mass[i] = p.mass;
        snapshot.projectile_caliber[i] = p.caliber;
        snapshot.projectile_state[i] = p.state;
        snapshot.projectile_lifetime[i] = p.lifetime;
        snapshot.projectile_remaining_energy[i] = p.remaining_energy;
        snapshot.projectile_penetration_distance[i] = p.penetration_distance;
        snapshot.projectile_layer_count[i] = p.layer_count;
    }

    // Capture destruction runtime state
    const destruction_sys = destruction.getSystem();
    snapshot.destroyable_count = destruction_sys.count;
    i = 0;
    while (i < destruction_sys.count and i < destruction.MAX_DESTROYABLE) : (i += 1) {
        const d = &destruction_sys.destroyables[i];
        snapshot.destroyable_entity_id[i] = d.entity_id;
        snapshot.destroyable_hp[i] = d.damage_model.current_hp;
        snapshot.destroyable_broken[i] = d.broken;
        snapshot.destroyable_damage_state[i] = d.progressive.damage_state;
        snapshot.destroyable_integrity_ratio[i] = d.progressive.integrity_ratio;
        snapshot.destroyable_crack_density[i] = d.progressive.crack_density;
    }

    // Capture terrain/environment state
    const terrain_sys = terrain.getTerrainSystem();
    snapshot.terrain_patch_count = terrain_sys.patch_count;
    i = 0;
    while (i < terrain_sys.patch_count and i < terrain.MAX_TERRAIN_PATCHES) : (i += 1) {
        snapshot.terrain_patches[i] = terrain_sys.patches[i];
    }
    snapshot.terrain_weather = terrain_sys.weather;
    snapshot.terrain_global_friction_modifier = terrain_sys.global_friction_modifier;

    // Capture disaster/environment state
    const disaster_sys = disasters.getDisasterSystem();
    snapshot.disaster_count = disaster_sys.disaster_count;
    i = 0;
    while (i < disasters.MAX_ACTIVE_DISASTERS) : (i += 1) {
        snapshot.active_disasters[i] = disaster_sys.active_disasters[i];
    }
    snapshot.disaster_chain_reaction_enabled = disaster_sys.chain_reaction_enabled;
    snapshot.disaster_apocalypse_mode = disaster_sys.apocalypse_mode;
    snapshot.disaster_seismic_wave = disaster_sys.seismic_wave;
    snapshot.disaster_atmosphere = disaster_sys.atmosphere;

    // Capture collision state
    const collision_sys = collision.getSystem();
    snapshot.collision_damage = collision_sys.damage;
    snapshot.collision_last_result = collision_sys.last_collision;
    snapshot.collision_count = collision_sys.collision_count;

    // Capture crash defense state
    const defense_sys = crash_defense.getSystem();
    snapshot.defense_config = defense_sys.config;
    snapshot.defense_last_progress_tick = defense_sys.last_progress_tick;
    snapshot.defense_emergency_stopped = defense_sys.emergency_stopped;
    snapshot.defense_snapshot_count = defense_sys.snapshot_count;

    // Capture sensor state
    const sensor_sys = sensors.getSystem();
    snapshot.sensor_count = sensor_sys.sensor_count;
    i = 0;
    while (i < sensors.MAX_SENSORS) : (i += 1) {
        snapshot.sensors[i] = sensor_sys.sensors[i];
    }
    snapshot.sensor_fusion = sensor_sys.fusion;
    snapshot.sensor_degradation_factor = sensor_sys.degradation_factor;
    snapshot.sensor_interference_level = sensor_sys.interference_level;

    // Capture network state
    const network_sys = network.getSystem();
    snapshot.network_replica_count = network_sys.replica_count;
    i = 0;
    while (i < network_sys.replica_count and i < network.MAX_REPLICAS) : (i += 1) {
        snapshot.network_replicas[i] = network_sys.replicas[i];
    }
    snapshot.network_input_count = network_sys.input_count;
    var input_idx: u16 = 0;
    while (input_idx < network_sys.input_count and input_idx < network.MAX_INPUTS) : (input_idx += 1) {
        snapshot.network_inputs[input_idx] = network_sys.inputs[input_idx];
    }
    snapshot.network_local_tick = network_sys.local_tick;
    snapshot.network_remote_tick = network_sys.remote_tick;
    snapshot.network_config = network_sys.config;
    snapshot.network_last_sync_tick = network_sys.last_sync_tick;
    snapshot.network_crc_errors = network_sys.crc_errors;

    // Capture tire state
    const tire_sys = tire.getSystem();
    snapshot.tire_count = tire_sys.count;
    i = 0;
    while (i < tire_sys.count and i < tire.MAX_TIRES) : (i += 1) {
        snapshot.tires[i] = tire_sys.tires[i];
    }

    // Capture suspension state
    const suspension_sys = suspension.getSystem();
    snapshot.suspension_count = suspension_sys.count;
    i = 0;
    while (i < suspension_sys.count and i < suspension.MAX_WHEELS) : (i += 1) {
        snapshot.suspensions[i] = suspension_sys.suspensions[i];
    }

    // Capture drivetrain state
    snapshot.drivetrain_state = drivetrain.getDrivetrainState().*;

    // Capture aerodynamics state
    const aero_sys = aerodynamics.getAeroState();
    snapshot.aero_state = aero_sys.*;

    // Reconstruct device slice from global aero system by reading internal state through config carrier
    // Current module only exposes aero state directly, so snapshot stores the active aero state plus device count 0.
    snapshot.aero_device_count = 0;

    // Capture braking state
    snapshot.brake_system = braking.getSystem().*;

    // Capture debris state
    const debris_sys = destruction.getDebrisSystem();
    snapshot.debris_count = debris_sys.count;
    var debris_idx: u16 = 0;
    while (debris_idx < debris_sys.count and debris_idx < destruction.MAX_DEBRIS) : (debris_idx += 1) {
        snapshot.debris[debris_idx] = debris_sys.debris[debris_idx];
    }

    // Capture AI traffic state
    snapshot.ai_traffic_vehicle_count = ai_traffic.getVehicleCount();
    i = 0;
    while (i < snapshot.ai_traffic_vehicle_count and i < ai_traffic.MAX_AI_VEHICLES) : (i += 1) {
        snapshot.ai_traffic_vehicles[i] = ai_traffic.getTrafficVehicles()[i];
    }
    snapshot.ai_traffic_light_count = ai_traffic.getTrafficLightCount();
    i = 0;
    while (i < snapshot.ai_traffic_light_count and i < ai_traffic.MAX_TRAFFIC_LIGHTS) : (i += 1) {
        snapshot.ai_traffic_lights[i] = ai_traffic.getTrafficLight(i).?.*;
    }
    snapshot.ai_traffic_global_time = ai_traffic.getSystem().global_time;

    // Copy input log
    snapshot.input_count = g_rewind_system.input_count;
    i = 0;
    while (i < snapshot.input_count and i < MAX_INPUT_LOG) : (i += 1) {
        snapshot.input_log[i] = g_rewind_system.input_log[i];
    }

    // Compute world hash
    snapshot.world_hash = computeWorldHash(&snapshot);

    return snapshot;
}

/// Record world snapshot
pub fn recordWorldSnapshot(
    tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) void {
    const snapshot = captureWorldSnapshot(tick, s1024, entities);

    const idx = g_rewind_system.world_snapshot_index;
    g_rewind_system.world_snapshots[idx] = snapshot;

    g_rewind_system.world_snapshot_index = (g_rewind_system.world_snapshot_index + 1) % 120;
    if (g_rewind_system.world_snapshot_count < 120) {
        g_rewind_system.world_snapshot_count += 1;
    }
}

/// Get world snapshot at specific tick
pub fn getWorldSnapshotAtTick(tick: u32) ?*const WorldSnapshot {
    var i: u8 = 0;
    while (i < g_rewind_system.world_snapshot_count) : (i += 1) {
        const idx = (g_rewind_system.world_snapshot_index + 120 - g_rewind_system.world_snapshot_count + i) % 120;
        if (g_rewind_system.world_snapshots[idx].tick == tick) {
            return &g_rewind_system.world_snapshots[idx];
        }
    }
    return null;
}

/// Restore world from snapshot
pub fn restoreWorldSnapshot(
    snapshot: *const WorldSnapshot,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) void {
    s1024.global_tick = snapshot.global_tick;
    s1024.instance_count = snapshot.instance_count;

    // Restore instance states
    var i: u8 = 0;
    while (i < snapshot.instance_count and i < scene32.MAX_INSTANCES) : (i += 1) {
        const inst = &s1024.instances[i];
        const snap = snapshot.instances[i];
        inst.entity_id = snap.entity_id;
        inst.pos_x = snap.pos_x;
        inst.pos_y = snap.pos_y;
        inst.pos_z = snap.pos_z;
        inst.rot_yaw = snap.rot_yaw;
        inst.rot_pitch = snap.rot_pitch;
        inst.rot_roll = snap.rot_roll;
        inst.state = snap.state;
        inst.vel_x = snap.vel_x;
        inst.vel_y = snap.vel_y;
        inst.vel_z = snap.vel_z;
        inst.ang_x = snap.ang_x;
        inst.ang_y = snap.ang_y;
        inst.ang_z = snap.ang_z;
        inst.sleep_tick = snap.sleep_tick;
    }

    // Restore KCC states
    const kcc_sys = kcc.getSystem();
    kcc_sys.count = snapshot.kcc_count;
    i = 0;
    while (i < snapshot.kcc_count and i < kcc.MAX_KCC) : (i += 1) {
        const state = &kcc_sys.characters[i];
        state.pos_x = snapshot.kcc_positions[i][0];
        state.pos_y = snapshot.kcc_positions[i][1];
        state.pos_z = snapshot.kcc_positions[i][2];
        state.vel_x = snapshot.kcc_velocities[i][0];
        state.vel_y = snapshot.kcc_velocities[i][1];
        state.vel_z = snapshot.kcc_velocities[i][2];
        state.grounded = snapshot.kcc_grounded[i];
        state.was_grounded = snapshot.kcc_grounded[i];
        state.crouching = snapshot.kcc_crouching[i];
        state.jumping = snapshot.kcc_jumping[i];
        state.yaw = snapshot.kcc_yaw[i];
        state.ground_normal_x = snapshot.kcc_ground_normals[i][0];
        state.ground_normal_y = snapshot.kcc_ground_normals[i][1];
        state.ground_normal_z = snapshot.kcc_ground_normals[i][2];
        state.stand_height = snapshot.kcc_stand_height[i];
        state.crouch_height = snapshot.kcc_crouch_height[i];
        state.radius = snapshot.kcc_radius[i];
        state.move_speed = snapshot.kcc_move_speed[i];
        state.jump_force = snapshot.kcc_jump_force[i];
        state.gravity = snapshot.kcc_gravity[i];
        state.crouch_speed_mult = snapshot.kcc_crouch_speed_mult[i];
        state.push_force = snapshot.kcc_push_force[i];
        state.step_height = snapshot.kcc_step_height[i];
        state.max_slope_angle = snapshot.kcc_max_slope_angle[i];
        state.step_offset = snapshot.kcc_step_offset[i];
    }

    // Restore vehicle states
    const vehicle_sys = vehicle.getSystem();
    vehicle_sys.count = snapshot.vehicle_count;
    i = 0;
    while (i < snapshot.vehicle_count and i < vehicle.MAX_VEHICLES) : (i += 1) {
        const state = &vehicle_sys.vehicles[i];
        state.pos_x = snapshot.vehicle_positions[i][0];
        state.pos_y = snapshot.vehicle_positions[i][1];
        state.pos_z = snapshot.vehicle_positions[i][2];
        state.speed = snapshot.vehicle_velocities[i][0];
        state.angular_velocity = snapshot.vehicle_velocities[i][1];
        state.yaw = snapshot.vehicle_yaw[i];
    }

    // Restore joint states
    const joint_sys = joint.getSystem();
    joint_sys.joint_count = snapshot.joint_count;
    i = 0;
    while (i < snapshot.joint_count and i < joint.MAX_JOINTS) : (i += 1) {
        joint_sys.joints[i] = snapshot.joints[i];
    }

    // Restore ragdoll states
    const ragdoll_sys = ragdoll.getSystem();
    ragdoll_sys.count = snapshot.ragdoll_count;
    i = 0;
    while (i < snapshot.ragdoll_count and i < ragdoll.MAX_RAGDOLLS) : (i += 1) {
        const state = &ragdoll_sys.ragdolls[i];
        state.part_count = snapshot.ragdoll_part_count[i];
        state.base_x = snapshot.ragdoll_base_positions[i][0];
        state.base_y = snapshot.ragdoll_base_positions[i][1];
        state.base_z = snapshot.ragdoll_base_positions[i][2];
        state.active = snapshot.ragdoll_active[i];
        state.resurrection_tick = snapshot.ragdoll_resurrection_tick[i];

        var part_idx: u8 = 0;
        while (part_idx < state.part_count and part_idx < ragdoll.MAX_RAGDOLL_PARTS) : (part_idx += 1) {
            state.parts[part_idx].pos_x = snapshot.ragdoll_part_positions[i][part_idx][0];
            state.parts[part_idx].pos_y = snapshot.ragdoll_part_positions[i][part_idx][1];
            state.parts[part_idx].pos_z = snapshot.ragdoll_part_positions[i][part_idx][2];
            state.parts[part_idx].vel_x = snapshot.ragdoll_part_velocities[i][part_idx][0];
            state.parts[part_idx].vel_y = snapshot.ragdoll_part_velocities[i][part_idx][1];
            state.parts[part_idx].vel_z = snapshot.ragdoll_part_velocities[i][part_idx][2];
        }
    }

    // Restore projectile states
    const projectile_sys = ballistics.getSystem();
    projectile_sys.count = snapshot.projectile_count;
    i = 0;
    while (i < snapshot.projectile_count and i < ballistics.MAX_PROJECTILES) : (i += 1) {
        const state = &projectile_sys.projectiles[i];
        state.pos_x = snapshot.projectile_positions[i][0];
        state.pos_y = snapshot.projectile_positions[i][1];
        state.pos_z = snapshot.projectile_positions[i][2];
        state.vel_x = snapshot.projectile_velocities[i][0];
        state.vel_y = snapshot.projectile_velocities[i][1];
        state.vel_z = snapshot.projectile_velocities[i][2];
        state.mass = snapshot.projectile_mass[i];
        state.caliber = snapshot.projectile_caliber[i];
        state.state = snapshot.projectile_state[i];
        state.lifetime = snapshot.projectile_lifetime[i];
        state.remaining_energy = snapshot.projectile_remaining_energy[i];
        state.penetration_distance = snapshot.projectile_penetration_distance[i];
        state.layer_count = snapshot.projectile_layer_count[i];
    }

    // Restore destruction states
    const destruction_sys = destruction.getSystem();
    destruction_sys.count = snapshot.destroyable_count;
    i = 0;
    while (i < snapshot.destroyable_count and i < destruction.MAX_DESTROYABLE) : (i += 1) {
        const state = &destruction_sys.destroyables[i];
        state.entity_id = snapshot.destroyable_entity_id[i];
        state.damage_model.current_hp = snapshot.destroyable_hp[i];
        state.broken = snapshot.destroyable_broken[i];
        state.progressive.damage_state = snapshot.destroyable_damage_state[i];
        state.progressive.integrity_ratio = snapshot.destroyable_integrity_ratio[i];
        state.progressive.crack_density = snapshot.destroyable_crack_density[i];
    }

    // Restore terrain/environment state
    const terrain_sys = terrain.getTerrainSystem();
    terrain_sys.patch_count = snapshot.terrain_patch_count;
    i = 0;
    while (i < snapshot.terrain_patch_count and i < terrain.MAX_TERRAIN_PATCHES) : (i += 1) {
        terrain_sys.patches[i] = snapshot.terrain_patches[i];
    }
    terrain_sys.weather = snapshot.terrain_weather;
    terrain_sys.global_friction_modifier = snapshot.terrain_global_friction_modifier;

    // Restore disaster/environment state
    const disaster_sys = disasters.getDisasterSystem();
    disaster_sys.disaster_count = snapshot.disaster_count;
    i = 0;
    while (i < disasters.MAX_ACTIVE_DISASTERS) : (i += 1) {
        disaster_sys.active_disasters[i] = snapshot.active_disasters[i];
    }
    disaster_sys.chain_reaction_enabled = snapshot.disaster_chain_reaction_enabled;
    disaster_sys.apocalypse_mode = snapshot.disaster_apocalypse_mode;
    disaster_sys.seismic_wave = snapshot.disaster_seismic_wave;
    disaster_sys.atmosphere = snapshot.disaster_atmosphere;

    // Restore collision state
    const collision_sys = collision.getSystem();
    collision_sys.damage = snapshot.collision_damage;
    collision_sys.last_collision = snapshot.collision_last_result;
    collision_sys.collision_count = snapshot.collision_count;

    // Restore crash defense state
    const defense_sys = crash_defense.getSystem();
    defense_sys.config = snapshot.defense_config;
    defense_sys.last_progress_tick = snapshot.defense_last_progress_tick;
    defense_sys.emergency_stopped = snapshot.defense_emergency_stopped;
    defense_sys.snapshot_count = snapshot.defense_snapshot_count;

    // Restore sensor state
    const sensor_sys = sensors.getSystem();
    sensor_sys.sensor_count = snapshot.sensor_count;
    i = 0;
    while (i < sensors.MAX_SENSORS) : (i += 1) {
        sensor_sys.sensors[i] = snapshot.sensors[i];
    }
    sensor_sys.fusion = snapshot.sensor_fusion;
    sensor_sys.degradation_factor = snapshot.sensor_degradation_factor;
    sensor_sys.interference_level = snapshot.sensor_interference_level;

    // Restore network state
    const network_sys = network.getSystem();
    network_sys.replica_count = snapshot.network_replica_count;
    i = 0;
    while (i < snapshot.network_replica_count and i < network.MAX_REPLICAS) : (i += 1) {
        network_sys.replicas[i] = snapshot.network_replicas[i];
    }
    network_sys.input_count = snapshot.network_input_count;
    var restore_input_idx: u16 = 0;
    while (restore_input_idx < snapshot.network_input_count and restore_input_idx < network.MAX_INPUTS) : (restore_input_idx += 1) {
        network_sys.inputs[restore_input_idx] = snapshot.network_inputs[restore_input_idx];
    }
    network_sys.local_tick = snapshot.network_local_tick;
    network_sys.remote_tick = snapshot.network_remote_tick;
    network_sys.config = snapshot.network_config;
    network_sys.last_sync_tick = snapshot.network_last_sync_tick;
    network_sys.crc_errors = snapshot.network_crc_errors;

    // Restore tire state
    const tire_sys = tire.getSystem();
    tire_sys.count = snapshot.tire_count;
    i = 0;
    while (i < snapshot.tire_count and i < tire.MAX_TIRES) : (i += 1) {
        tire_sys.tires[i] = snapshot.tires[i];
    }

    // Restore suspension state
    const suspension_sys = suspension.getSystem();
    suspension_sys.count = snapshot.suspension_count;
    i = 0;
    while (i < snapshot.suspension_count and i < suspension.MAX_WHEELS) : (i += 1) {
        suspension_sys.suspensions[i] = snapshot.suspensions[i];
    }

    // Restore drivetrain state
    drivetrain.getDrivetrainState().* = snapshot.drivetrain_state;

    // Restore aerodynamics state
    aerodynamics.getAeroState().* = snapshot.aero_state;

    // Restore braking state
    braking.getSystem().* = snapshot.brake_system;

    // Restore debris state
    const debris_sys = destruction.getDebrisSystem();
    debris_sys.count = snapshot.debris_count;
    var restore_debris_idx: u16 = 0;
    while (restore_debris_idx < snapshot.debris_count and restore_debris_idx < destruction.MAX_DEBRIS) : (restore_debris_idx += 1) {
        debris_sys.debris[restore_debris_idx] = snapshot.debris[restore_debris_idx];
    }

    // Restore AI traffic state
    const traffic_sys = ai_traffic.getSystem();
    traffic_sys.vehicle_count = snapshot.ai_traffic_vehicle_count;
    i = 0;
    while (i < snapshot.ai_traffic_vehicle_count and i < ai_traffic.MAX_AI_VEHICLES) : (i += 1) {
        traffic_sys.vehicles[i] = snapshot.ai_traffic_vehicles[i];
    }
    traffic_sys.light_count = snapshot.ai_traffic_light_count;
    i = 0;
    while (i < snapshot.ai_traffic_light_count and i < ai_traffic.MAX_TRAFFIC_LIGHTS) : (i += 1) {
        traffic_sys.lights[i] = snapshot.ai_traffic_lights[i];
    }
    traffic_sys.global_time = snapshot.ai_traffic_global_time;

    // Rebuild occupancy after restore
    s1024.rebuildOccupancy(entities) catch {};
}

fn stepPreviewTick(engine: *tick_engine.TickEngine) void {
    engine.tick_id += 1;
    engine.s1024.global_tick = engine.tick_id;
    tick_engine.gather(engine);
    tick_engine.speculate(engine);
    tick_engine.resolve(engine);
    const applied = tick_engine.commit(engine);
    engine.stable = (applied == 0);
}

/// Simulate a stored snapshot forward in an isolated temporary world.
/// This does not mutate the live world or the rewind snapshot ring buffer.
pub fn simulateWorldSnapshotForward(
    snapshot: *const WorldSnapshot,
    ticks: u32,
    allocator: std.mem.Allocator,
    entities: []entity16.Entity16,
) !WorldSnapshot {
    var temp_scene = scene1024.Scene1024.init(allocator);
    defer temp_scene.deinit();

    _ = try temp_scene.getPage(0);
    restoreWorldSnapshot(snapshot, &temp_scene, entities);

    var temp_engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&temp_engine, &temp_scene, entities);
    temp_engine.tick_id = snapshot.tick;
    temp_engine.stable = false;

    var tick_index: u32 = 0;
    while (tick_index < ticks) : (tick_index += 1) {
        stepPreviewTick(&temp_engine);
    }

    return captureWorldSnapshot(temp_engine.tick_id, &temp_scene, entities);
}

pub fn forecastSnapshotInstancesSimulated(
    snapshot: *const WorldSnapshot,
    ticks: u32,
    allocator: std.mem.Allocator,
    entities: []entity16.Entity16,
    out_snapshot: *WorldSnapshot,
) !u8 {
    out_snapshot.* = try simulateWorldSnapshotForward(snapshot, ticks, allocator, entities);
    return out_snapshot.instance_count;
}

pub fn diffWorldSnapshots(a: *const WorldSnapshot, b: *const WorldSnapshot) WorldSnapshotDiff {
    var diff = WorldSnapshotDiff{
        .tick_from = a.tick,
        .tick_to = b.tick,
        .hash_changed = a.world_hash != b.world_hash,
        .instance_count_changed = a.instance_count != b.instance_count,
        .instances_moved = 0,
        .kcc_count_changed = a.kcc_count != b.kcc_count,
        .kcc_changed = 0,
        .vehicle_count_changed = a.vehicle_count != b.vehicle_count,
        .vehicles_changed = 0,
        .joint_count_changed = a.joint_count != b.joint_count,
        .joints_changed = 0,
        .ragdoll_count_changed = a.ragdoll_count != b.ragdoll_count,
        .ragdolls_changed = 0,
        .projectile_count_changed = a.projectile_count != b.projectile_count,
        .projectiles_changed = 0,
        .destroyable_count_changed = a.destroyable_count != b.destroyable_count,
        .destroyables_changed = 0,
        .terrain_patch_count_changed = a.terrain_patch_count != b.terrain_patch_count,
        .terrain_changed = false,
        .disaster_count_changed = a.disaster_count != b.disaster_count,
        .disasters_changed = false,
        .collision_changed = false,
        .defense_changed = false,
        .sensor_changed = false,
        .network_changed = false,
        .tire_changed = false,
        .suspension_changed = false,
        .drivetrain_changed = false,
        .aero_changed = false,
        .braking_changed = false,
        .debris_changed = false,
        .ai_traffic_changed = false,
    };

    var i: u8 = 0;
    const instance_count = @min(a.instance_count, b.instance_count);
    while (i < instance_count) : (i += 1) {
        const lhs = a.instances[i];
        const rhs = b.instances[i];
        if (lhs.pos_x != rhs.pos_x or lhs.pos_y != rhs.pos_y or lhs.pos_z != rhs.pos_z or
            lhs.vel_x != rhs.vel_x or lhs.vel_y != rhs.vel_y or lhs.vel_z != rhs.vel_z or
            lhs.state != rhs.state)
        {
            diff.instances_moved += 1;
        }
    }

    i = 0;
    const kcc_count = @min(a.kcc_count, b.kcc_count);
    while (i < kcc_count) : (i += 1) {
        if (a.kcc_positions[i][0] != b.kcc_positions[i][0] or
            a.kcc_positions[i][1] != b.kcc_positions[i][1] or
            a.kcc_positions[i][2] != b.kcc_positions[i][2] or
            a.kcc_velocities[i][0] != b.kcc_velocities[i][0] or
            a.kcc_velocities[i][1] != b.kcc_velocities[i][1] or
            a.kcc_velocities[i][2] != b.kcc_velocities[i][2] or
            a.kcc_grounded[i] != b.kcc_grounded[i] or
            a.kcc_crouching[i] != b.kcc_crouching[i] or
            a.kcc_jumping[i] != b.kcc_jumping[i] or
            a.kcc_yaw[i] != b.kcc_yaw[i] or
            a.kcc_ground_normals[i][0] != b.kcc_ground_normals[i][0] or
            a.kcc_ground_normals[i][1] != b.kcc_ground_normals[i][1] or
            a.kcc_ground_normals[i][2] != b.kcc_ground_normals[i][2] or
            a.kcc_stand_height[i] != b.kcc_stand_height[i] or
            a.kcc_crouch_height[i] != b.kcc_crouch_height[i] or
            a.kcc_radius[i] != b.kcc_radius[i] or
            a.kcc_move_speed[i] != b.kcc_move_speed[i] or
            a.kcc_jump_force[i] != b.kcc_jump_force[i] or
            a.kcc_gravity[i] != b.kcc_gravity[i] or
            a.kcc_crouch_speed_mult[i] != b.kcc_crouch_speed_mult[i] or
            a.kcc_push_force[i] != b.kcc_push_force[i] or
            a.kcc_step_height[i] != b.kcc_step_height[i] or
            a.kcc_max_slope_angle[i] != b.kcc_max_slope_angle[i] or
            a.kcc_step_offset[i] != b.kcc_step_offset[i])
        {
            diff.kcc_changed += 1;
        }
    }

    i = 0;
    const vehicle_count = @min(a.vehicle_count, b.vehicle_count);
    while (i < vehicle_count) : (i += 1) {
        if (a.vehicle_positions[i][0] != b.vehicle_positions[i][0] or
            a.vehicle_positions[i][1] != b.vehicle_positions[i][1] or
            a.vehicle_positions[i][2] != b.vehicle_positions[i][2] or
            a.vehicle_velocities[i][0] != b.vehicle_velocities[i][0] or
            a.vehicle_velocities[i][1] != b.vehicle_velocities[i][1] or
            a.vehicle_yaw[i] != b.vehicle_yaw[i])
        {
            diff.vehicles_changed += 1;
        }
    }

    i = 0;
    const joint_count = @min(a.joint_count, b.joint_count);
    while (i < joint_count) : (i += 1) {
        const lhs = a.joints[i];
        const rhs = b.joints[i];
        if (lhs.joint_type != rhs.joint_type or
            lhs.entity_a != rhs.entity_a or
            lhs.entity_b != rhs.entity_b or
            lhs.anchor_a_x != rhs.anchor_a_x or
            lhs.anchor_a_y != rhs.anchor_a_y or
            lhs.anchor_a_z != rhs.anchor_a_z or
            lhs.anchor_b_x != rhs.anchor_b_x or
            lhs.anchor_b_y != rhs.anchor_b_y or
            lhs.anchor_b_z != rhs.anchor_b_z or
            lhs.axis_x != rhs.axis_x or
            lhs.axis_y != rhs.axis_y or
            lhs.axis_z != rhs.axis_z or
            lhs.limit_min != rhs.limit_min or
            lhs.limit_max != rhs.limit_max or
            lhs.breaking_force != rhs.breaking_force or
            lhs.stiffness != rhs.stiffness or
            lhs.damping != rhs.damping or
            lhs.motor_enabled != rhs.motor_enabled or
            lhs.motor_target != rhs.motor_target or
            lhs.motor_speed != rhs.motor_speed or
            lhs.motor_max_torque != rhs.motor_max_torque or
            lhs.enabled != rhs.enabled or
            lhs.break_accum != rhs.break_accum)
        {
            diff.joints_changed += 1;
        }
    }

    i = 0;
    const ragdoll_count = @min(a.ragdoll_count, b.ragdoll_count);
    while (i < ragdoll_count) : (i += 1) {
        var changed = a.ragdoll_active[i] != b.ragdoll_active[i] or
            a.ragdoll_base_positions[i][0] != b.ragdoll_base_positions[i][0] or
            a.ragdoll_base_positions[i][1] != b.ragdoll_base_positions[i][1] or
            a.ragdoll_base_positions[i][2] != b.ragdoll_base_positions[i][2] or
            a.ragdoll_resurrection_tick[i] != b.ragdoll_resurrection_tick[i] or
            a.ragdoll_part_count[i] != b.ragdoll_part_count[i];
        var part_idx: u8 = 0;
        while (!changed and part_idx < @min(a.ragdoll_part_count[i], b.ragdoll_part_count[i])) : (part_idx += 1) {
            changed = a.ragdoll_part_positions[i][part_idx][0] != b.ragdoll_part_positions[i][part_idx][0] or
                a.ragdoll_part_positions[i][part_idx][1] != b.ragdoll_part_positions[i][part_idx][1] or
                a.ragdoll_part_positions[i][part_idx][2] != b.ragdoll_part_positions[i][part_idx][2] or
                a.ragdoll_part_velocities[i][part_idx][0] != b.ragdoll_part_velocities[i][part_idx][0] or
                a.ragdoll_part_velocities[i][part_idx][1] != b.ragdoll_part_velocities[i][part_idx][1] or
                a.ragdoll_part_velocities[i][part_idx][2] != b.ragdoll_part_velocities[i][part_idx][2];
        }
        if (changed) diff.ragdolls_changed += 1;
    }

    i = 0;
    const projectile_count = @min(a.projectile_count, b.projectile_count);
    while (i < projectile_count) : (i += 1) {
        if (a.projectile_positions[i][0] != b.projectile_positions[i][0] or
            a.projectile_positions[i][1] != b.projectile_positions[i][1] or
            a.projectile_positions[i][2] != b.projectile_positions[i][2] or
            a.projectile_velocities[i][0] != b.projectile_velocities[i][0] or
            a.projectile_velocities[i][1] != b.projectile_velocities[i][1] or
            a.projectile_velocities[i][2] != b.projectile_velocities[i][2] or
            a.projectile_state[i] != b.projectile_state[i] or
            a.projectile_lifetime[i] != b.projectile_lifetime[i] or
            a.projectile_remaining_energy[i] != b.projectile_remaining_energy[i] or
            a.projectile_penetration_distance[i] != b.projectile_penetration_distance[i] or
            a.projectile_layer_count[i] != b.projectile_layer_count[i])
        {
            diff.projectiles_changed += 1;
        }
    }

    i = 0;
    const destroyable_count = @min(a.destroyable_count, b.destroyable_count);
    while (i < destroyable_count) : (i += 1) {
        if (a.destroyable_entity_id[i] != b.destroyable_entity_id[i] or
            a.destroyable_hp[i] != b.destroyable_hp[i] or
            a.destroyable_broken[i] != b.destroyable_broken[i] or
            a.destroyable_damage_state[i] != b.destroyable_damage_state[i] or
            a.destroyable_integrity_ratio[i] != b.destroyable_integrity_ratio[i] or
            a.destroyable_crack_density[i] != b.destroyable_crack_density[i])
        {
            diff.destroyables_changed += 1;
        }
    }

    diff.terrain_changed = diff.terrain_patch_count_changed or
        a.terrain_weather.rain_intensity != b.terrain_weather.rain_intensity or
        a.terrain_weather.fog_density != b.terrain_weather.fog_density or
        a.terrain_weather.wind_speed != b.terrain_weather.wind_speed or
        a.terrain_weather.wind_direction != b.terrain_weather.wind_direction or
        a.terrain_weather.air_temperature != b.terrain_weather.air_temperature or
        a.terrain_weather.visibility != b.terrain_weather.visibility or
        a.terrain_weather.freezing != b.terrain_weather.freezing or
        a.terrain_global_friction_modifier != b.terrain_global_friction_modifier;

    if (!diff.terrain_changed) {
        i = 0;
        const terrain_patch_count = @min(a.terrain_patch_count, b.terrain_patch_count);
        while (i < terrain_patch_count) : (i += 1) {
            const lhs = a.terrain_patches[i];
            const rhs = b.terrain_patches[i];
            if (lhs.center_x != rhs.center_x or lhs.center_z != rhs.center_z or lhs.radius != rhs.radius or
                lhs.surface_type != rhs.surface_type or lhs.friction_coefficient != rhs.friction_coefficient or
                lhs.rolling_resistance != rhs.rolling_resistance or lhs.water_depth != rhs.water_depth or
                lhs.roughness != rhs.roughness or lhs.temperature != rhs.temperature)
            {
                diff.terrain_changed = true;
                break;
            }
        }
    }

    diff.disasters_changed = diff.disaster_count_changed or
        a.disaster_chain_reaction_enabled != b.disaster_chain_reaction_enabled or
        a.disaster_apocalypse_mode != b.disaster_apocalypse_mode or
        a.disaster_seismic_wave.amplitude != b.disaster_seismic_wave.amplitude or
        a.disaster_seismic_wave.frequency != b.disaster_seismic_wave.frequency or
        a.disaster_seismic_wave.wave_type != b.disaster_seismic_wave.wave_type or
        a.disaster_seismic_wave.propagation_speed != b.disaster_seismic_wave.propagation_speed or
        a.disaster_atmosphere.pressure_delta != b.disaster_atmosphere.pressure_delta or
        a.disaster_atmosphere.wind_speed_max != b.disaster_atmosphere.wind_speed_max or
        a.disaster_atmosphere.temperature_change != b.disaster_atmosphere.temperature_change or
        a.disaster_atmosphere.humidity != b.disaster_atmosphere.humidity or
        a.disaster_atmosphere.visibility_reduction != b.disaster_atmosphere.visibility_reduction;

    if (!diff.disasters_changed) {
        i = 0;
        while (i < disasters.MAX_ACTIVE_DISASTERS) : (i += 1) {
            const lhs = a.active_disasters[i];
            const rhs = b.active_disasters[i];
            if (lhs.disaster_type != rhs.disaster_type or lhs.intensity != rhs.intensity or
                lhs.epicenter_x != rhs.epicenter_x or lhs.epicenter_y != rhs.epicenter_y or lhs.epicenter_z != rhs.epicenter_z or
                lhs.radius != rhs.radius or lhs.duration != rhs.duration or lhs.elapsed != rhs.elapsed or lhs.active != rhs.active)
            {
                diff.disasters_changed = true;
                break;
            }
        }
    }

    diff.collision_changed =
        a.collision_count != b.collision_count or
        a.collision_damage.structural_integrity != b.collision_damage.structural_integrity or
        a.collision_damage.engine_damage != b.collision_damage.engine_damage or
        a.collision_damage.transmission_damage != b.collision_damage.transmission_damage or
        a.collision_last_result.collided != b.collision_last_result.collided or
        a.collision_last_result.impact_energy != b.collision_last_result.impact_energy;

    diff.defense_changed =
        a.defense_config.velocity_cap != b.defense_config.velocity_cap or
        a.defense_config.position_min != b.defense_config.position_min or
        a.defense_config.position_max != b.defense_config.position_max or
        a.defense_last_progress_tick != b.defense_last_progress_tick or
        a.defense_emergency_stopped != b.defense_emergency_stopped or
        a.defense_snapshot_count != b.defense_snapshot_count;

    diff.sensor_changed =
        a.sensor_count != b.sensor_count or
        a.sensor_fusion.object_count != b.sensor_fusion.object_count or
        a.sensor_fusion.timestamp != b.sensor_fusion.timestamp or
        a.sensor_degradation_factor != b.sensor_degradation_factor or
        a.sensor_interference_level != b.sensor_interference_level;

    if (!diff.sensor_changed) {
        i = 0;
        while (i < sensors.MAX_SENSORS) : (i += 1) {
            const lhs = a.sensors[i];
            const rhs = b.sensors[i];
            if (lhs.sensor_type != rhs.sensor_type or lhs.enabled != rhs.enabled or lhs.field_of_view != rhs.field_of_view or
                lhs.range != rhs.range or lhs.noise_level != rhs.noise_level or lhs.occlusion_factor != rhs.occlusion_factor or
                lhs.confidence != rhs.confidence or lhs.last_update_time != rhs.last_update_time)
            {
                diff.sensor_changed = true;
                break;
            }
        }
    }

    diff.network_changed =
        a.network_replica_count != b.network_replica_count or
        a.network_input_count != b.network_input_count or
        a.network_local_tick != b.network_local_tick or
        a.network_remote_tick != b.network_remote_tick or
        a.network_last_sync_tick != b.network_last_sync_tick or
        a.network_crc_errors != b.network_crc_errors or
        a.network_config.send_rate_hz != b.network_config.send_rate_hz or
        a.network_config.timeout_ms != b.network_config.timeout_ms or
        a.network_config.max_rollback_ticks != b.network_config.max_rollback_ticks or
        a.network_config.crc_check_enabled != b.network_config.crc_check_enabled or
        a.network_config.prediction_window != b.network_config.prediction_window;

    diff.tire_changed = a.tire_count != b.tire_count;
    if (!diff.tire_changed) {
        i = 0;
        while (i < a.tire_count) : (i += 1) {
            if (a.tires[i].pos_x != b.tires[i].pos_x or
                a.tires[i].angular_velocity != b.tires[i].angular_velocity or
                a.tires[i].surface_temperature != b.tires[i].surface_temperature or
                a.tires[i].grip_level != b.tires[i].grip_level or
                a.tires[i].hydroplaning != b.tires[i].hydroplaning)
            {
                diff.tire_changed = true;
                break;
            }
        }
    }

    diff.suspension_changed = a.suspension_count != b.suspension_count;
    if (!diff.suspension_changed) {
        i = 0;
        while (i < a.suspension_count) : (i += 1) {
            if (a.suspensions[i].current_length != b.suspensions[i].current_length or
                a.suspensions[i].velocity != b.suspensions[i].velocity or
                a.suspensions[i].force != b.suspensions[i].force or
                a.suspensions[i].active != b.suspensions[i].active)
            {
                diff.suspension_changed = true;
                break;
            }
        }
    }

    diff.drivetrain_changed =
        a.drivetrain_state.engine.rpm != b.drivetrain_state.engine.rpm or
        a.drivetrain_state.engine.throttle_position != b.drivetrain_state.engine.throttle_position or
        a.drivetrain_state.transmission.current_gear != b.drivetrain_state.transmission.current_gear or
        a.drivetrain_state.driveshaft_angle != b.drivetrain_state.driveshaft_angle;

    diff.aero_changed =
        a.aero_state.drag_force != b.aero_state.drag_force or
        a.aero_state.downforce != b.aero_state.downforce or
        a.aero_state.drag_coefficient != b.aero_state.drag_coefficient or
        a.aero_state.wind_velocity_x != b.aero_state.wind_velocity_x or
        a.aero_state.wind_velocity_z != b.aero_state.wind_velocity_z;

    diff.braking_changed =
        a.brake_system.brake_balance != b.brake_system.brake_balance or
        a.brake_system.brake_by_wire != b.brake_system.brake_by_wire or
        a.brake_system.brake_assist != b.brake_system.brake_assist or
        a.brake_system.axle[0].pedal_position != b.brake_system.axle[0].pedal_position or
        a.brake_system.axle[0].abs_active != b.brake_system.axle[0].abs_active;

    diff.debris_changed = a.debris_count != b.debris_count;
    if (!diff.debris_changed) {
        var debris_i: u16 = 0;
        while (debris_i < a.debris_count) : (debris_i += 1) {
            if (a.debris[debris_i].pos_x != b.debris[debris_i].pos_x or
                a.debris[debris_i].pos_y != b.debris[debris_i].pos_y or
                a.debris[debris_i].vel_y != b.debris[debris_i].vel_y or
                a.debris[debris_i].lifetime != b.debris[debris_i].lifetime or
                a.debris[debris_i].active != b.debris[debris_i].active)
            {
                diff.debris_changed = true;
                break;
            }
        }
    }

    diff.ai_traffic_changed =
        a.ai_traffic_vehicle_count != b.ai_traffic_vehicle_count or
        a.ai_traffic_light_count != b.ai_traffic_light_count or
        a.ai_traffic_global_time != b.ai_traffic_global_time;

    if (!diff.ai_traffic_changed) {
        i = 0;
        while (i < a.ai_traffic_vehicle_count) : (i += 1) {
            if (a.ai_traffic_vehicles[i].pos_x != b.ai_traffic_vehicles[i].pos_x or
                a.ai_traffic_vehicles[i].pos_z != b.ai_traffic_vehicles[i].pos_z or
                a.ai_traffic_vehicles[i].target_vel != b.ai_traffic_vehicles[i].target_vel or
                a.ai_traffic_vehicles[i].behavior != b.ai_traffic_vehicles[i].behavior or
                a.ai_traffic_vehicles[i].active != b.ai_traffic_vehicles[i].active)
            {
                diff.ai_traffic_changed = true;
                break;
            }
        }
    }

    return diff;
}

// ============================================================================
// World Hash for Determinism
// ============================================================================

/// Compute hash of world state for determinism verification
pub fn computeWorldHash(snapshot: *const WorldSnapshot) u64 {
    var hash: u64 = 0xDEADBEEFCAFEBABE;

    // Hash tick
    hash ^= @as(u64, snapshot.tick) *% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.global_tick) *% 0x9e3779b97f4a7c15;

    // Hash instance states (deterministic order)
    var i: u8 = 0;
    while (i < snapshot.instance_count) : (i += 1) {
        const inst = &snapshot.instances[i];
        hash ^= @as(u64, inst.entity_id) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(inst.pos_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(inst.pos_y))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(inst.pos_z))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u16, @bitCast(inst.vel_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u16, @bitCast(inst.vel_y))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u16, @bitCast(inst.vel_z))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @intFromEnum(inst.state)) +% 0x9e3779b97f4a7c15;
    }

    // Hash KCC states
    i = 0;
    while (i < snapshot.kcc_count) : (i += 1) {
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_positions[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_positions[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_positions[i][2]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_velocities[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_velocities[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_velocities[i][2]))) +% 0x9e3779b97f4a7c15;
        hash ^= if (snapshot.kcc_grounded[i]) 0xA5A5A5A5A5A5A5A5 else 0x5A5A5A5A5A5A5A5A;
        hash ^= if (snapshot.kcc_crouching[i]) 0x1111111111111111 else 0x2222222222222222;
        hash ^= if (snapshot.kcc_jumping[i]) 0x3333333333333333 else 0x4444444444444444;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_yaw[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_ground_normals[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_ground_normals[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_ground_normals[i][2]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_stand_height[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_crouch_height[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_radius[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_move_speed[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_jump_force[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_gravity[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_crouch_speed_mult[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_push_force[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_step_height[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_max_slope_angle[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.kcc_step_offset[i]))) +% 0x9e3779b97f4a7c15;
    }

    // Hash Vehicle states
    i = 0;
    while (i < snapshot.vehicle_count) : (i += 1) {
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.vehicle_positions[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.vehicle_positions[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.vehicle_positions[i][2]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.vehicle_velocities[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.vehicle_velocities[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.vehicle_yaw[i]))) +% 0x9e3779b97f4a7c15;
    }

    // Hash joint states
    i = 0;
    while (i < snapshot.joint_count) : (i += 1) {
        const j = snapshot.joints[i];
        hash ^= @as(u64, @intFromEnum(j.joint_type)) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, j.entity_a) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, j.entity_b) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.anchor_a_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.anchor_a_y))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.anchor_a_z))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.anchor_b_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.anchor_b_y))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.anchor_b_z))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.axis_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.axis_y))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.axis_z))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.limit_min))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.limit_max))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.breaking_force))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.stiffness))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.damping))) +% 0x9e3779b97f4a7c15;
        hash ^= if (j.motor_enabled) 0xA5A5A5A5A5A5A5A5 else 0x5A5A5A5A5A5A5A5A;
        hash ^= @as(u64, @as(u32, @bitCast(j.motor_target))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.motor_speed))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(j.motor_max_torque))) +% 0x9e3779b97f4a7c15;
        hash ^= if (j.enabled) 0xE7E7E7E7E7E7E7E7 else 0x7E7E7E7E7E7E7E7E;
        hash ^= @as(u64, @as(u32, @bitCast(j.break_accum))) +% 0x9e3779b97f4a7c15;
    }

    // Hash ragdoll runtime state
    i = 0;
    while (i < snapshot.ragdoll_count) : (i += 1) {
        hash ^= @as(u64, snapshot.ragdoll_part_count[i]) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_base_positions[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_base_positions[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_base_positions[i][2]))) +% 0x9e3779b97f4a7c15;
        hash ^= if (snapshot.ragdoll_active[i]) 0xC3C3C3C3C3C3C3C3 else 0x3C3C3C3C3C3C3C3C;
        hash ^= @as(u64, snapshot.ragdoll_resurrection_tick[i]) +% 0x9e3779b97f4a7c15;

        var part_idx: u8 = 0;
        while (part_idx < ragdoll.MAX_RAGDOLL_PARTS) : (part_idx += 1) {
            hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_part_positions[i][part_idx][0]))) +% 0x9e3779b97f4a7c15;
            hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_part_positions[i][part_idx][1]))) +% 0x9e3779b97f4a7c15;
            hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_part_positions[i][part_idx][2]))) +% 0x9e3779b97f4a7c15;
            hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_part_velocities[i][part_idx][0]))) +% 0x9e3779b97f4a7c15;
            hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_part_velocities[i][part_idx][1]))) +% 0x9e3779b97f4a7c15;
            hash ^= @as(u64, @as(u32, @bitCast(snapshot.ragdoll_part_velocities[i][part_idx][2]))) +% 0x9e3779b97f4a7c15;
        }
    }

    // Hash projectile runtime state
    i = 0;
    while (i < snapshot.projectile_count) : (i += 1) {
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_positions[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_positions[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_positions[i][2]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_velocities[i][0]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_velocities[i][1]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_velocities[i][2]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_mass[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_caliber[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @intFromEnum(snapshot.projectile_state[i])) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, snapshot.projectile_lifetime[i]) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_remaining_energy[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.projectile_penetration_distance[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, snapshot.projectile_layer_count[i]) +% 0x9e3779b97f4a7c15;
    }

    // Hash destruction runtime state
    i = 0;
    while (i < snapshot.destroyable_count) : (i += 1) {
        hash ^= @as(u64, snapshot.destroyable_entity_id[i]) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.destroyable_hp[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= if (snapshot.destroyable_broken[i]) 0xD1D1D1D1D1D1D1D1 else 0x1D1D1D1D1D1D1D1D;
        hash ^= @as(u64, @intFromEnum(snapshot.destroyable_damage_state[i])) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.destroyable_integrity_ratio[i]))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.destroyable_crack_density[i]))) +% 0x9e3779b97f4a7c15;
    }

    // Hash terrain/environment state
    hash ^= @as(u64, snapshot.terrain_patch_count) +% 0x9e3779b97f4a7c15;
    i = 0;
    while (i < snapshot.terrain_patch_count) : (i += 1) {
        const patch = snapshot.terrain_patches[i];
        hash ^= @as(u64, @as(u32, @bitCast(patch.center_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(patch.center_z))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(patch.radius))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @intFromEnum(patch.surface_type)) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(patch.friction_coefficient))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(patch.rolling_resistance))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(patch.water_depth))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(patch.roughness))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(patch.temperature))) +% 0x9e3779b97f4a7c15;
    }
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.terrain_weather.rain_intensity))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.terrain_weather.fog_density))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.terrain_weather.wind_speed))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.terrain_weather.wind_direction))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.terrain_weather.air_temperature))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.terrain_weather.visibility))) +% 0x9e3779b97f4a7c15;
    hash ^= if (snapshot.terrain_weather.freezing) 0xAAAA5555AAAA5555 else 0x5555AAAA5555AAAA;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.terrain_global_friction_modifier))) +% 0x9e3779b97f4a7c15;

    // Hash disaster/environment state
    hash ^= @as(u64, snapshot.disaster_count) +% 0x9e3779b97f4a7c15;
    i = 0;
    while (i < disasters.MAX_ACTIVE_DISASTERS) : (i += 1) {
        const event = snapshot.active_disasters[i];
        hash ^= @as(u64, @intFromEnum(event.disaster_type)) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(event.intensity))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(event.epicenter_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(event.epicenter_y))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(event.epicenter_z))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(event.radius))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(event.duration))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(event.elapsed))) +% 0x9e3779b97f4a7c15;
        hash ^= if (event.active) 0xB4B4B4B4B4B4B4B4 else 0x4B4B4B4B4B4B4B4B;
    }
    hash ^= if (snapshot.disaster_chain_reaction_enabled) 0x9999000099990000 else 0x0000999900009999;
    hash ^= if (snapshot.disaster_apocalypse_mode) 0xCCCC1111CCCC1111 else 0x1111CCCC1111CCCC;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_seismic_wave.amplitude))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_seismic_wave.frequency))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.disaster_seismic_wave.wave_type) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_seismic_wave.propagation_speed))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_atmosphere.pressure_delta))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_atmosphere.wind_speed_max))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_atmosphere.temperature_change))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_atmosphere.humidity))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.disaster_atmosphere.visibility_reduction))) +% 0x9e3779b97f4a7c15;

    // Hash collision state
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.collision_damage.structural_integrity))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.collision_damage.engine_damage))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.collision_damage.transmission_damage))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.collision_damage.deployed_airbags) +% 0x9e3779b97f4a7c15;
    hash ^= if (snapshot.collision_last_result.collided) 0xABABABABABABABAB else 0xBABABABABABABABA;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.collision_last_result.impact_energy))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.collision_count) +% 0x9e3779b97f4a7c15;

    // Hash crash defense state
    hash ^= if (snapshot.defense_config.nan_check_enabled) 0x1212121212121212 else 0x2121212121212121;
    hash ^= if (snapshot.defense_config.bounds_check_enabled) 0x3434343434343434 else 0x4343434343434343;
    hash ^= if (snapshot.defense_config.energy_check_enabled) 0x5656565656565656 else 0x6565656565656565;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.defense_config.velocity_cap))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.defense_config.position_min))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.defense_config.position_max))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.defense_last_progress_tick) +% 0x9e3779b97f4a7c15;
    hash ^= if (snapshot.defense_emergency_stopped) 0x7878787878787878 else 0x8787878787878787;
    hash ^= @as(u64, snapshot.defense_snapshot_count) +% 0x9e3779b97f4a7c15;

    // Hash sensor state
    hash ^= @as(u64, snapshot.sensor_count) +% 0x9e3779b97f4a7c15;
    i = 0;
    while (i < sensors.MAX_SENSORS) : (i += 1) {
        const sensor = snapshot.sensors[i];
        hash ^= @as(u64, @intFromEnum(sensor.sensor_type)) +% 0x9e3779b97f4a7c15;
        hash ^= if (sensor.enabled) 0x9191919191919191 else 0x1919191919191919;
        hash ^= @as(u64, @as(u32, @bitCast(sensor.field_of_view))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(sensor.range))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(sensor.noise_level))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(sensor.confidence))) +% 0x9e3779b97f4a7c15;
    }
    hash ^= @as(u64, snapshot.sensor_fusion.object_count) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.sensor_fusion.timestamp))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.sensor_degradation_factor))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.sensor_interference_level))) +% 0x9e3779b97f4a7c15;

    // Hash network state
    hash ^= @as(u64, snapshot.network_replica_count) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_input_count) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_local_tick) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_remote_tick) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_last_sync_tick) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_crc_errors) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_config.send_rate_hz) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_config.timeout_ms) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.network_config.max_rollback_ticks) +% 0x9e3779b97f4a7c15;
    hash ^= if (snapshot.network_config.crc_check_enabled) 0xCDCDCDCDCDCDCDCD else 0xDCDCDCDCDCDCDCDC;
    hash ^= @as(u64, snapshot.network_config.prediction_window) +% 0x9e3779b97f4a7c15;

    // Hash vehicle sub-systems and debris
    hash ^= @as(u64, snapshot.tire_count) +% 0x9e3779b97f4a7c15;
    i = 0;
    while (i < snapshot.tire_count) : (i += 1) {
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.tires[i].pos_x))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.tires[i].angular_velocity))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.tires[i].surface_temperature))) +% 0x9e3779b97f4a7c15;
        hash ^= if (snapshot.tires[i].hydroplaning) 0xFAFAFAFAFAFAFAFA else 0xAFAFAFAFAFAFAFAF;
    }
    hash ^= @as(u64, snapshot.suspension_count) +% 0x9e3779b97f4a7c15;
    i = 0;
    while (i < snapshot.suspension_count) : (i += 1) {
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.suspensions[i].current_length))) +% 0x9e3779b97f4a7c15;
        hash ^= @as(u64, @as(u32, @bitCast(snapshot.suspensions[i].force))) +% 0x9e3779b97f4a7c15;
    }
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.drivetrain_state.engine.rpm))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u8, @bitCast(snapshot.drivetrain_state.transmission.current_gear))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.aero_state.drag_force))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.aero_state.downforce))) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.brake_system.brake_balance))) +% 0x9e3779b97f4a7c15;
    hash ^= if (snapshot.brake_system.axle[0].abs_active) 0xEFEFEFEFEFEFEFEF else 0xFEFEFEFEFEFEFEFE;
    hash ^= @as(u64, snapshot.debris_count) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.ai_traffic_vehicle_count) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, snapshot.ai_traffic_light_count) +% 0x9e3779b97f4a7c15;
    hash ^= @as(u64, @as(u32, @bitCast(snapshot.ai_traffic_global_time))) +% 0x9e3779b97f4a7c15;

    // Finalize hash
    hash = (hash << 31) | (hash >> 33);
    hash ^= 0x1234567890ABCDEF;

    return hash;
}

/// Verify determinism between two world snapshots
pub fn verifyWorldDeterminism(snap_a: *const WorldSnapshot, snap_b: *const WorldSnapshot) bool {
    if (snap_a.tick != snap_b.tick) return false;
    if (snap_a.instance_count != snap_b.instance_count) return false;

    // Compare instance states
    var i: u8 = 0;
    while (i < snap_a.instance_count) : (i += 1) {
        const a = &snap_a.instances[i];
        const b = &snap_b.instances[i];
        if (a.pos_x != b.pos_x or a.pos_y != b.pos_y or a.pos_z != b.pos_z) return false;
        if (a.vel_x != b.vel_x or a.vel_y != b.vel_y or a.vel_z != b.vel_z) return false;
        if (a.state != b.state) return false;
    }

    return true;
}

// ============================================================================
// Input Logging
// ============================================================================

/// Record input for a tick
pub fn recordInput(
    tick: u32,
    forward: bool,
    backward: bool,
    left: bool,
    right: bool,
    jump: bool,
    brake: bool,
    mouse_dx: f32,
    mouse_dy: f32,
) void {
    if (g_rewind_system.input_count >= MAX_INPUT_LOG) return;

    const idx = g_rewind_system.input_count;
    g_rewind_system.input_log[idx] = .{
        .tick = tick,
        .forward = forward,
        .backward = backward,
        .left = left,
        .right = right,
        .jump = jump,
        .brake = brake,
        .mouse_dx = mouse_dx,
        .mouse_dy = mouse_dy,
    };
    g_rewind_system.input_count += 1;
    g_rewind_system.max_input_tick = tick;
}

/// Get input at specific tick
pub fn getInputAtTick(tick: u32) ?*const InputLog {
    var i: u16 = 0;
    while (i < g_rewind_system.input_count) : (i += 1) {
        if (g_rewind_system.input_log[i].tick == tick) {
            return &g_rewind_system.input_log[i];
        }
    }
    return null;
}

/// Clear input log
pub fn clearInputLog() void {
    g_rewind_system.input_count = 0;
    g_rewind_system.max_input_tick = 0;
}

// ============================================================================
// Fast Forward Replay
// ============================================================================

/// Fast forward simulation from recorded inputs
/// Returns number of ticks actually simulated
pub fn fastForwardTicks(
    start_tick: u32,
    end_tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) u32 {
    var ticks_simulated: u32 = 0;
    var tick = start_tick;

    while (tick <= end_tick) {
        // Could replay inputs and step physics here

        // Record snapshot at interval
        if (ticks_simulated % 10 == 0) {
            recordWorldSnapshot(tick, s1024, entities);
        }

        tick += 1;
        ticks_simulated += 1;
    }

    return ticks_simulated;
}

// ============================================================================
// Legacy Per-Entity Rewind (kept for compatibility)
// ============================================================================

pub fn recordState(state: RewindState) void {
    const idx = g_rewind_system.current_index;
    g_rewind_system.states[idx] = state;

    g_rewind_system.current_index = @as(u16, @intCast((g_rewind_system.current_index + 1) % MAX_REWIND_STATES));
    if (g_rewind_system.state_count < MAX_REWIND_STATES) {
        g_rewind_system.state_count += 1;
    }

    if (state.tick > g_rewind_system.max_tick) {
        g_rewind_system.max_tick = state.tick;
    }
}

pub fn getStateAtTick(tick: u32) ?*const RewindState {
    if (tick > g_rewind_system.max_tick) return null;

    const offset = g_rewind_system.max_tick - tick;
    if (offset >= g_rewind_system.state_count) return null;

    const idx = if (g_rewind_system.current_index >= offset)
        g_rewind_system.current_index - offset
    else
        MAX_REWIND_STATES - (offset - g_rewind_system.current_index);

    return &g_rewind_system.states[idx];
}

pub fn rewindToTick(tick: u32) ?*const RewindState {
    return getStateAtTick(tick);
}

pub fn calculateStateHash(state: *const RewindState) u64 {
    var hash: u64 = 0x1234567890ABCDEF;

    const px = @as(u32, @bitCast(state.pos_x));
    const py = @as(u32, @bitCast(state.pos_y));
    const pz = @as(u32, @bitCast(state.pos_z));
    const vx = @as(u32, @bitCast(state.vel_x));
    const vy = @as(u32, @bitCast(state.vel_y));
    const vz = @as(u32, @bitCast(state.vel_z));

    hash ^= @as(u64, px) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, py) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, pz) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, vx) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, vy) + 0x9e3779b97f4a7c15;
    hash ^= @as(u64, vz) + 0x9e3779b97f4a7c15;

    hash = (hash << 31) | (hash >> 33);

    return hash;
}

pub fn verifyDeterminism(tick_a: u32, tick_b: u32) bool {
    const state_a = getStateAtTick(tick_a);
    const state_b = getStateAtTick(tick_b);

    if (state_a == null or state_b == null) return false;

    return calculateStateHash(state_a.?) == calculateStateHash(state_b.?);
}

pub fn beginDeterminismProof() void {
    if (g_rewind_system.state_count > 0) {
        g_rewind_system.proof_initial_hash = calculateStateHash(&g_rewind_system.states[0]);
    }
    g_rewind_system.proof_ticks = g_rewind_system.max_tick;
    g_rewind_system.proof.tick_count = g_rewind_system.max_tick;
}

pub fn endDeterminismProof() void {
    if (g_rewind_system.state_count > 0) {
        const last_idx = (g_rewind_system.current_index + MAX_REWIND_STATES - 1) % MAX_REWIND_STATES;
        g_rewind_system.proof_final_hash = calculateStateHash(&g_rewind_system.states[last_idx]);
    }
    g_rewind_system.proof.verified = true;
}

pub fn compareTraces(trace_a: []const RewindState, trace_b: []const RewindState) u16 {
    var mismatches: u16 = 0;
    const min_len = @min(trace_a.len, trace_b.len);

    for (0..min_len) |i| {
        const hash_a = calculateStateHash(&trace_a[i]);
        const hash_b = calculateStateHash(&trace_b[i]);
        if (hash_a != hash_b) mismatches += 1;
    }

    mismatches += @as(u16, @abs(@as(i32, @intCast(trace_a.len)) - @as(i32, @intCast(trace_b.len))));

    return mismatches;
}

pub fn createGhostReplay(tick_start: u32, tick_end: u32) []const RewindState {
    const start_idx: u16 = if (tick_start <= g_rewind_system.max_tick) tick_start else g_rewind_system.max_tick;
    const end_idx: u16 = if (tick_end <= g_rewind_system.max_tick) tick_end else g_rewind_system.max_tick;

    if (start_idx > end_idx) return &[_]RewindState{};

    const count = end_idx - start_idx + 1;
    const result: [100]RewindState = undefined;

    for (0..count) |i| {
        const tick = start_idx + @as(u32, @intCast(i));
        if (getStateAtTick(tick)) |state| {
            result[i] = state.*;
        }
    }

    return &result;
}

pub fn getRewindBufferUsage() struct { count: u16, capacity: usize, percent: f32 } {
    return .{
        .count = g_rewind_system.state_count,
        .capacity = MAX_REWIND_STATES,
        .percent = @as(f32, @floatFromInt(g_rewind_system.state_count)) / @as(f32, @floatFromInt(MAX_REWIND_STATES)) * 100,
    };
}

pub fn clearRewindBuffer() void {
    g_rewind_system.state_count = 0;
    g_rewind_system.current_index = 0;
    g_rewind_system.max_tick = 0;
}

pub fn isDeterministic() bool {
    return g_rewind_system.deterministic;
}

pub fn getDeterminismProof() DeterminismProof {
    return g_rewind_system.proof;
}

/// Get world snapshot buffer usage
pub fn getWorldSnapshotBufferUsage() struct { count: u8, capacity: usize, percent: f32 } {
    return .{
        .count = g_rewind_system.world_snapshot_count,
        .capacity = 120,
        .percent = @as(f32, @floatFromInt(g_rewind_system.world_snapshot_count)) / 120.0 * 100,
    };
}

/// Get system for external access
pub fn getSystem() *RewindSystem {
    return &g_rewind_system;
}

test "World snapshot capture and lookup" {
    init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 20,
        .pos_z = 30,
        .rot_yaw = 1,
        .rot_pitch = 2,
        .rot_roll = 3,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 5,
        .vel_z = 6,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    recordWorldSnapshot(42, &s1024, entities[0..]);
    const snapshot = getWorldSnapshotAtTick(42);
    try std.testing.expect(snapshot != null);
    try std.testing.expect(snapshot.?.instance_count == 1);
    try std.testing.expect(snapshot.?.instances[0].pos_x == 10);
    try std.testing.expect(snapshot.?.instances[0].vel_z == 6);
    try std.testing.expect(snapshot.?.world_hash != 0);
}

test "World snapshot restore rebuilds instance state" {
    init();
    kcc.init();
    vehicle.init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 1,
        .pos_y = 2,
        .pos_z = 3,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const snapshot = captureWorldSnapshot(7, &s1024, entities[0..]);

    s1024.instances[0].pos_x = 99;
    s1024.instances[0].vel_y = 77;
    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    try std.testing.expect(s1024.instances[0].pos_x == 1);
    try std.testing.expect(s1024.instances[0].vel_y == 0);
    try std.testing.expect(s1024.global_tick == snapshot.global_tick);
}

test "World snapshot restore rebuilds subsystem state" {
    init();
    kcc.init();
    vehicle.init();
    ragdoll.init();
    ballistics.init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    const kcc_state = kcc.createCharacter(10, 20, 30, .{}) orelse return error.TestUnexpectedResult;
    kcc_state.vel_x = 1;
    kcc_state.vel_y = -2;
    kcc_state.vel_z = 3;
    kcc_state.grounded = true;
    kcc_state.crouching = true;
    kcc_state.jumping = true;
    kcc_state.yaw = 1.25;
    kcc_state.ground_normal_x = -0.2;
    kcc_state.ground_normal_y = 0.98;
    kcc_state.ground_normal_z = 0.1;
    kcc_state.gravity = -420.0;
    kcc_state.push_force = 77.0;
    kcc_state.step_height = 4;

    const vehicle_state = vehicle.createCar(40, 50, 60, 0.25) orelse return error.TestUnexpectedResult;
    vehicle_state.speed = 12.5;
    vehicle_state.angular_velocity = 0.75;

    const snapshot = captureWorldSnapshot(9, &s1024, entities[0..]);

    kcc_state.pos_x = 100;
    kcc_state.vel_y = 99;
    kcc_state.grounded = false;
    kcc_state.crouching = false;
    kcc_state.jumping = false;
    kcc_state.yaw = 0.0;
    kcc_state.ground_normal_x = 0;
    kcc_state.ground_normal_y = 1;
    kcc_state.ground_normal_z = 0;
    kcc_state.gravity = -800.0;
    kcc_state.push_force = 5.0;
    kcc_state.step_height = 1;
    vehicle_state.pos_x = 200;
    vehicle_state.speed = 0;
    vehicle_state.yaw = 1.5;

    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    const restored_kcc = kcc.getSystem();
    const restored_vehicle = vehicle.getSystem();
    try std.testing.expect(restored_kcc.count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 10), restored_kcc.characters[0].pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -2), restored_kcc.characters[0].vel_y, 0.0001);
    try std.testing.expect(restored_kcc.characters[0].grounded);
    try std.testing.expect(restored_kcc.characters[0].crouching);
    try std.testing.expect(restored_kcc.characters[0].jumping);
    try std.testing.expectApproxEqAbs(@as(f32, 1.25), restored_kcc.characters[0].yaw, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -0.2), restored_kcc.characters[0].ground_normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.98), restored_kcc.characters[0].ground_normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), restored_kcc.characters[0].ground_normal_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -420.0), restored_kcc.characters[0].gravity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 77.0), restored_kcc.characters[0].push_force, 0.0001);
    try std.testing.expect(restored_kcc.characters[0].step_height == 4);
    try std.testing.expect(restored_vehicle.count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 40), restored_vehicle.vehicles[0].pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 12.5), restored_vehicle.vehicles[0].speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), restored_vehicle.vehicles[0].yaw, 0.0001);
}

test "World snapshot restore rebuilds ragdoll and projectile state" {
    init();
    ragdoll.init();
    ballistics.init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    const rag = ragdoll.createHumanoid(10, 20, 30) orelse return error.TestUnexpectedResult;
    rag.parts[0].vel_y = -5;
    rag.resurrection_tick = 42;

    const proj = ballistics.spawnProjectile(1, 2, 3, 10, 20, 30, 2, 7.62) orelse return error.TestUnexpectedResult;
    proj.lifetime = 11;
    proj.penetration_distance = 3.5;
    proj.layer_count = 2;

    const snapshot = captureWorldSnapshot(15, &s1024, entities[0..]);

    rag.base_x = 99;
    rag.parts[0].vel_y = 0;
    rag.resurrection_tick = 0;
    proj.pos_x = 100;
    proj.vel_z = 0;
    proj.lifetime = 0;

    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    const ragdoll_sys = ragdoll.getSystem();
    const projectile_sys = ballistics.getSystem();
    try std.testing.expect(ragdoll_sys.count == 1);
    try std.testing.expect(ragdoll_sys.ragdolls[0].base_x == 10);
    try std.testing.expectApproxEqAbs(@as(f32, -5), ragdoll_sys.ragdolls[0].parts[0].vel_y, 0.0001);
    try std.testing.expect(ragdoll_sys.ragdolls[0].resurrection_tick == 42);
    try std.testing.expect(projectile_sys.count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 1), projectile_sys.projectiles[0].pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 30), projectile_sys.projectiles[0].vel_z, 0.0001);
    try std.testing.expect(projectile_sys.projectiles[0].lifetime == 11);
    try std.testing.expectApproxEqAbs(@as(f32, 3.5), projectile_sys.projectiles[0].penetration_distance, 0.0001);
    try std.testing.expect(projectile_sys.projectiles[0].layer_count == 2);
}

test "World snapshot restore rebuilds destruction state" {
    init();
    destruction.init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.glass();

    const destroyable = destruction.createDestroyable(0, 100) orelse return error.TestUnexpectedResult;
    destroyable.damage_model.current_hp = 25;
    destroyable.broken = true;
    destroyable.progressive.damage_state = .broken;
    destroyable.progressive.integrity_ratio = 0.2;
    destroyable.progressive.crack_density = 0.8;

    const snapshot = captureWorldSnapshot(18, &s1024, entities[0..]);

    destroyable.damage_model.current_hp = 100;
    destroyable.broken = false;
    destroyable.progressive.damage_state = .intact;
    destroyable.progressive.integrity_ratio = 1.0;
    destroyable.progressive.crack_density = 0.0;

    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    const destruction_sys = destruction.getSystem();
    try std.testing.expect(destruction_sys.count == 1);
    try std.testing.expect(destruction_sys.destroyables[0].entity_id == 0);
    try std.testing.expectApproxEqAbs(@as(f32, 25), destruction_sys.destroyables[0].damage_model.current_hp, 0.0001);
    try std.testing.expect(destruction_sys.destroyables[0].broken);
    try std.testing.expect(destruction_sys.destroyables[0].progressive.damage_state == .broken);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), destruction_sys.destroyables[0].progressive.integrity_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), destruction_sys.destroyables[0].progressive.crack_density, 0.0001);
}

test "World snapshot restore rebuilds joint state" {
    init();
    joint.initGlobal();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    const joint_idx = joint.addGlobalJoint(.{
        .joint_type = .hinge,
        .entity_a = 1,
        .entity_b = 2,
        .anchor_a_x = 3,
        .anchor_a_y = 4,
        .anchor_a_z = 5,
        .anchor_b_x = 6,
        .anchor_b_y = 7,
        .anchor_b_z = 8,
        .axis_x = 0,
        .axis_y = 1,
        .axis_z = 0,
        .limit_min = -1.0,
        .limit_max = 1.0,
        .breaking_force = 50.0,
        .stiffness = 10.0,
        .damping = 2.0,
        .enabled = true,
    }) orelse return error.TestUnexpectedResult;
    try std.testing.expect(joint_idx == 0);

    const snapshot = captureWorldSnapshot(19, &s1024, entities[0..]);
    joint.clearGlobalJoints();
    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    const system = joint.getSystem();
    try std.testing.expect(system.joint_count == 1);
    try std.testing.expect(system.joints[0].joint_type == .hinge);
    try std.testing.expect(system.joints[0].entity_a == 1);
    try std.testing.expect(system.joints[0].entity_b == 2);
    try std.testing.expect(system.joints[0].anchor_b_z == 8);
    try std.testing.expectApproxEqAbs(@as(f32, 50.0), system.joints[0].breaking_force, 0.0001);
}

test "World snapshot restore rebuilds terrain and disaster state" {
    init();
    terrain.init();
    disasters.init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    terrain.addTerrainPatch(10, 20, 8, .mud);
    terrain.applyWeather(.{
        .rain_intensity = 0.7,
        .fog_density = 0.2,
        .wind_speed = 5.0,
        .wind_direction = 1.2,
        .air_temperature = -3.0,
        .visibility = 300.0,
        .freezing = true,
    });
    terrain.setGlobalFrictionModifier(0.8);

    disasters.enableChainReactions(true);
    disasters.enableApocalypseMode(true);
    disasters.triggerDisaster(.earthquake, 8.0, 1, 2, 3, 50);

    const snapshot = captureWorldSnapshot(23, &s1024, entities[0..]);

    terrain.init();
    disasters.init();
    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    const terrain_sys = terrain.getTerrainSystem();
    const disaster_sys = disasters.getDisasterSystem();
    try std.testing.expect(terrain_sys.patch_count == 1);
    try std.testing.expect(terrain_sys.patches[0].surface_type == .mud);
    try std.testing.expectApproxEqAbs(@as(f32, 0.7), terrain_sys.weather.rain_intensity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), terrain_sys.global_friction_modifier, 0.0001);
    try std.testing.expect(disaster_sys.disaster_count == 1);
    try std.testing.expect(disaster_sys.chain_reaction_enabled);
    try std.testing.expect(disaster_sys.apocalypse_mode);
    try std.testing.expect(disaster_sys.active_disasters[0].active);
    try std.testing.expect(disaster_sys.active_disasters[0].disaster_type == .earthquake);
}

test "World snapshot restore rebuilds collision defense sensor and network state" {
    init();
    collision.init();
    crash_defense.init(.{
        .velocity_cap = 321.0,
        .position_min = -50,
        .position_max = 500,
        .max_ticks_without_progress = 25,
    });
    sensors.init();
    network.init(.{
        .send_rate_hz = 30,
        .timeout_ms = 750,
        .max_rollback_ticks = 12,
        .crc_check_enabled = true,
        .prediction_window = 7,
    });

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    _ = collision.applyCollision(0, 0, 0, 0, 0, -20, 1000, .{});
    crash_defense.updateProgress(77);
    crash_defense.emergencyStop(&s1024);
    _ = sensors.addSensor(.radar, 120, 300);
    sensors.addDetectedObject(.{
        .object_id = 9,
        .object_type = 1,
        .pos_x = 1,
        .pos_y = 2,
        .pos_z = 3,
        .vel_x = 4,
        .vel_y = 5,
        .vel_z = 6,
        .confidence = 0.9,
        .age = 0.1,
        .sensor_source = .radar,
    });
    _ = network.createReplica(3);
    network.updateReplica(&network.getSystem().replicas[0], 10, 20, 30, 1, 2, 3, 45, 5);
    network.storeInput(.{
        .tick = 5,
        .forward = true,
        .backward = false,
        .left = false,
        .right = true,
        .jump = false,
        .crouch = false,
        .fire = false,
        .aim_x = 0.5,
        .aim_y = 0.0,
    });

    const snapshot = captureWorldSnapshot(29, &s1024, entities[0..]);

    collision.init();
    crash_defense.init(.{});
    sensors.init();
    network.init(.{});
    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    try std.testing.expectApproxEqAbs(@as(f32, 321.0), crash_defense.getSystem().config.velocity_cap, 0.0001);
    try std.testing.expect(crash_defense.getSystem().emergency_stopped);
    try std.testing.expect(crash_defense.getSystem().last_progress_tick == 77);
    try std.testing.expect(collision.getSystem().collision_count > 0);
    try std.testing.expect(collision.getSystem().last_collision.collided);
    try std.testing.expect(sensors.getSystem().sensor_count == 1);
    try std.testing.expect(sensors.getSystem().fusion.object_count == 1);
    try std.testing.expect(network.getSystem().replica_count == 1);
    try std.testing.expect(network.getSystem().input_count == 1);
    try std.testing.expect(network.getSystem().config.send_rate_hz == 30);
    try std.testing.expect(network.getSystem().replicas[0].entity_id == 3);
}

test "World snapshot restore rebuilds tire suspension drivetrain aero brake and debris state" {
    init();
    tire.init();
    suspension.init();
    drivetrain.init();
    aerodynamics.init();
    braking.init();
    destruction.initDebris();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    const tire_state = tire.createTire(1, 2, 3, .{
        .radius = 0.3,
        .width = 0.2,
        .mass = 20,
        .lateral_stiffness = 1,
        .longitudinal_stiffness = 1,
        .camber_thrust_coefficient = 1,
        .peak_slip_ratio = 1,
        .peak_slip_angle = 1,
        .friction_coefficient = 1,
        .rolling_resistance_coefficient = 0.02,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 80,
        .max_temperature = 120,
    }) orelse return error.TestUnexpectedResult;
    tire_state.angular_velocity = 12;
    tire_state.surface_temperature = 55;

    const susp_state = suspension.createSuspension(.{
        .spring_rate = 1,
        .damping_ratio = 1,
        .bump_damping = 1,
        .rebound_damping = 1,
        .preloaded = 0,
        .max_length = 0.5,
        .min_length = 0.1,
        .anti_roll_rate = 1,
    }) orelse return error.TestUnexpectedResult;
    susp_state.current_length = 0.2;
    susp_state.force = 99;

    drivetrain.getDrivetrainState().engine.rpm = 3456;
    drivetrain.getDrivetrainState().transmission.current_gear = 3;
    aerodynamics.getAeroState().drag_force = 123;
    aerodynamics.getAeroState().downforce = 456;
    braking.getSystem().brake_balance = 0.7;
    braking.getSystem().axle[0].abs_active = true;
    _ = destruction.spawnDebris(0, 0, 0, 1, 2, 3, 4, 5, 6, 0.5, 1.5);

    const snapshot = captureWorldSnapshot(31, &s1024, entities[0..]);

    tire.init();
    suspension.init();
    drivetrain.init();
    aerodynamics.init();
    braking.init();
    destruction.initDebris();
    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    try std.testing.expect(tire.getSystem().count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 12), tire.getSystem().tires[0].angular_velocity, 0.0001);
    try std.testing.expect(suspension.getSystem().count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 99), suspension.getSystem().suspensions[0].force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3456), drivetrain.getDrivetrainState().engine.rpm, 0.0001);
    try std.testing.expect(drivetrain.getDrivetrainState().transmission.current_gear == 3);
    try std.testing.expectApproxEqAbs(@as(f32, 123), aerodynamics.getAeroState().drag_force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.7), braking.getSystem().brake_balance, 0.0001);
    try std.testing.expect(braking.getSystem().axle[0].abs_active);
    try std.testing.expect(destruction.getDebrisSystem().count == 1);
}

test "World snapshot restore rebuilds ai traffic state" {
    init();
    ai_traffic.init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    const traffic_vehicle = ai_traffic.spawnAIVehicle(10, 0, 20, .aggressive) orelse return error.TestUnexpectedResult;
    traffic_vehicle.target_vel = 44;
    const traffic_light = ai_traffic.addTrafficLight(30, 40, 60) orelse return error.TestUnexpectedResult;
    traffic_light.state = .yellow;
    traffic_light.timer = 12.5;
    ai_traffic.getSystem().global_time = 99;

    const snapshot = captureWorldSnapshot(41, &s1024, entities[0..]);

    ai_traffic.init();
    restoreWorldSnapshot(&snapshot, &s1024, entities[0..]);

    const traffic_sys = ai_traffic.getSystem();
    try std.testing.expect(traffic_sys.vehicle_count == 1);
    try std.testing.expect(traffic_sys.light_count == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 44), traffic_sys.vehicles[0].target_vel, 0.0001);
    try std.testing.expect(traffic_sys.vehicles[0].behavior == .aggressive);
    try std.testing.expect(traffic_sys.lights[0].state == .yellow);
    try std.testing.expectApproxEqAbs(@as(f32, 12.5), traffic_sys.lights[0].timer, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 99), traffic_sys.global_time, 0.0001);
}

test "World snapshot simulation advances in isolated temporary world" {
    init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 200,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const snapshot = captureWorldSnapshot(12, &s1024, entities[0..]);
    const advanced = try simulateWorldSnapshotForward(&snapshot, 1, std.testing.allocator, entities[0..]);

    try std.testing.expect(advanced.tick == 13);
    try std.testing.expect(advanced.instances[0].pos_y < snapshot.instances[0].pos_y);
    try std.testing.expect(advanced.instances[0].vel_y < snapshot.instances[0].vel_y);
    try std.testing.expect(snapshot.instances[0].pos_y == 200);
    try std.testing.expect(snapshot.instances[0].vel_y == 0);
}

test "World snapshot diff reports instance and hash changes" {
    init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();

    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 200,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const before = captureWorldSnapshot(21, &s1024, entities[0..]);
    const after = try simulateWorldSnapshotForward(&before, 1, std.testing.allocator, entities[0..]);
    const diff = diffWorldSnapshots(&before, &after);

    try std.testing.expect(diff.tick_from == 21);
    try std.testing.expect(diff.tick_to == 22);
    try std.testing.expect(diff.hash_changed);
    try std.testing.expect(diff.instances_moved > 0);
}
