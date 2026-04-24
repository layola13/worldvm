//! Physics Systems Comprehensive Tests
//!
//! Tests for: KCC, Ballistics, Destruction, Ragdoll, Vehicle, Network, Crash Defense
//! Covers phases 7-13 of the physics engine

const std = @import("std");
const testing = std.testing;
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const physics = @import("physics.zig");
const kcc = @import("kcc.zig");
const ballistics = @import("ballistics.zig");
const destruction = @import("destruction.zig");
const ragdoll = @import("ragdoll.zig");
const vehicle = @import("vehicle.zig");
const network = @import("network.zig");
const crash_defense = @import("crash_defense.zig");

// ============================================================================
// KCC Tests (Phase 7)
// ============================================================================

test "KCC init and create character" {
    kcc.init();
    const config = kcc.KCCConfig{
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 2,
        .stand_height = 14,
        .crouch_height = 8,
        .radius = 4,
    };
    const char = kcc.createCharacter(100, 50, 100, config);
    try testing.expect(char != null);
    try testing.expect(char.?.pos_x == 100);
    try testing.expect(char.?.pos_y == 50);
    try testing.expect(char.?.pos_z == 100);
    try testing.expect(char.?.grounded == false);
    try testing.expect(char.?.crouching == false);
    try testing.expect(char.?.jumping == false);
}

test "KCC move and jump" {
    kcc.init();
    const config = kcc.KCCConfig{
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 2,
        .stand_height = 14,
        .crouch_height = 8,
        .radius = 4,
    };
    const char = kcc.createCharacter(100, 50, 100, config);
    try testing.expect(char != null);

    kcc.move(char.?, 1, 0, 0, 0.016, config);
    try testing.expect(char.?.vel_x > 0);

    kcc.jump(char.?, config);
}

test "KCC crouch" {
    kcc.init();
    const config = kcc.KCCConfig{
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 2,
        .stand_height = 14,
        .crouch_height = 8,
        .radius = 4,
    };
    const char = kcc.createCharacter(100, 50, 100, config);
    try testing.expect(char != null);

    char.?.grounded = true;
    kcc.crouch(char.?, true);
    try testing.expect(char.?.crouching == true);

    kcc.crouch(char.?, false);
    try testing.expect(char.?.crouching == false);
}

test "KCC getHeight" {
    kcc.init();
    const config = kcc.KCCConfig{
        .move_speed = 200.0,
        .jump_force = 350.0,
        .gravity = -800.0,
        .crouch_speed_mult = 0.5,
        .push_force = 100.0,
        .step_height = 2,
        .stand_height = 14,
        .crouch_height = 8,
        .radius = 4,
    };
    const char = kcc.createCharacter(100, 50, 100, config);
    try testing.expect(char != null);

    try testing.expect(kcc.getHeight(char.?) == 14);
    char.?.crouching = true;
    try testing.expect(kcc.getHeight(char.?) == 8);
}

test "KCC slide along wall" {
    const result = kcc.slideAlongWall(10, 0, 1, 0);
    try testing.expect(result.x == 0);
    try testing.expect(result.z == 0);
}

// ============================================================================
// Ballistics Tests (Phase 8)
// ============================================================================

test "Ballistics init and spawn projectile" {
    ballistics.init();
    const proj = ballistics.spawnProjectile(0, 0, 0, 100, 0, 0, 1.0, 5.56);
    try testing.expect(proj != null);
    try testing.expect(proj.?.state == .active);
    try testing.expect(proj.?.pos_x == 0);
    try testing.expect(proj.?.vel_x == 100);
}

test "Ballistics get speed" {
    ballistics.init();
    const proj = ballistics.spawnProjectile(0, 0, 0, 3, 4, 0, 1.0, 5.56);
    try testing.expect(proj != null);
    const speed = ballistics.getSpeed(proj.?);
    try testing.expect(@abs(speed - 5.0) < 0.001);
}

test "Ballistics get kinetic energy" {
    ballistics.init();
    const proj = ballistics.spawnProjectile(0, 0, 0, 3, 4, 0, 2.0, 5.56);
    try testing.expect(proj != null);
    const energy = ballistics.getKineticEnergy(proj.?);
    try testing.expect(@abs(energy - 25.0) < 0.001);
}

test "Ballistics calculate deflection" {
    const def = ballistics.calculateDeflection(10, 0, 0, -1, 0, 0, 0.5);
    try testing.expect(@abs(def.x - (-5.0)) < 0.001);
    try testing.expect(def.y == 0);
    try testing.expect(def.z == 0);
}

test "Ballistics apply drag" {
    ballistics.init();
    const proj = ballistics.spawnProjectile(0, 0, 0, 100, 0, 0, 1.0, 5.56);
    try testing.expect(proj != null);
    const initial_speed = ballistics.getSpeed(proj.?);
    ballistics.applyDrag(proj.?, 0.001, 0.016);
    const new_speed = ballistics.getSpeed(proj.?);
    try testing.expect(new_speed < initial_speed);
}

test "Ballistics generate fragments" {
    const fragments = ballistics.generateFragments(0, 0, 0, 8, 100);
    var count: u8 = 0;
    for (fragments) |frag| {
        if (frag.lifetime > 0) count += 1;
    }
    try testing.expect(count == 8);
}

// ============================================================================
// Destruction Tests (Phase 9)
// ============================================================================

test "Destruction init and create destroyable" {
    destruction.init();
    const destroyable = destruction.createDestroyable(1, 100.0);
    try testing.expect(destroyable != null);
    try testing.expect(destroyable.?.damage_model.max_hp == 100.0);
    try testing.expect(destroyable.?.damage_model.current_hp == 100.0);
    try testing.expect(destroyable.?.broken == false);
}

test "Destruction calculate damage" {
    const damage = destruction.calculateDamage(50.0, .fragile, 30);
    try testing.expect(damage > 50.0);

    const elastic_damage = destruction.calculateDamage(50.0, .elastic, 100);
    try testing.expect(elastic_damage < damage);
}

test "Destruction apply damage" {
    destruction.init();
    const destroyable = destruction.createDestroyable(1, 100.0);
    try testing.expect(destroyable != null);

    destruction.applyDamage(destroyable.?, 30.0);
    try testing.expect(destroyable.?.damage_model.current_hp == 70.0);

    destruction.applyDamage(destroyable.?, 80.0);
    try testing.expect(destroyable.?.damage_model.current_hp == 0.0);
    try testing.expect(destroyable.?.broken == true);
}

test "Destruction generate fracture" {
    destruction.init();
    const entity = entity16.Prototypes.apple();
    const fracture = destruction.generateFracture(&entity, 8, 8, 8, 100.0, 42);
    try testing.expect(fracture.crack_count > 0);
    try testing.expect(fracture.seed == 42);
}

test "Destruction should shatter" {
    destruction.init();
    const destroyable = destruction.createDestroyable(1, 100.0);
    try testing.expect(destroyable != null);
    try testing.expect(destruction.shouldShatter(destroyable.?) == false);

    destruction.applyDamage(destroyable.?, 80.0);
    try testing.expect(destruction.shouldShatter(destroyable.?) == true);
}

test "Destruction invulnerability" {
    destruction.init();
    const destroyable = destruction.createDestroyable(1, 100.0);
    try testing.expect(destroyable != null);

    destroyable.?.damage_model.invulnerable = true;
    destruction.applyDamage(destroyable.?, 100.0);
    try testing.expect(destroyable.?.damage_model.current_hp == 100.0);
}

// ============================================================================
// Ragdoll Tests (Phase 10)
// ============================================================================

test "Ragdoll init and create humanoid" {
    ragdoll.init();
    const ragdoll_ptr = ragdoll.createHumanoid(100, 50, 100);
    try testing.expect(ragdoll_ptr != null);
    try testing.expect(ragdoll_ptr.?.active == true);
    try testing.expect(ragdoll_ptr.?.part_count == 8);
    try testing.expect(ragdoll_ptr.?.joint_count == 7);
}

test "Ragdoll break limb" {
    ragdoll.init();
    const ragdoll_ptr = ragdoll.createHumanoid(100, 50, 100);
    try testing.expect(ragdoll_ptr != null);

    try testing.expect(ragdoll_ptr.?.parts[0].active == true);
    ragdoll.breakLimb(ragdoll_ptr.?, 0);
    try testing.expect(ragdoll_ptr.?.parts[0].active == false);
}

test "Ragdoll is fully broken" {
    ragdoll.init();
    const ragdoll_ptr = ragdoll.createHumanoid(100, 50, 100);
    try testing.expect(ragdoll_ptr != null);

    try testing.expect(ragdoll.isFullyBroken(ragdoll_ptr.?) == false);

    ragdoll.breakLimb(ragdoll_ptr.?, 0);
    ragdoll.breakLimb(ragdoll_ptr.?, 1);
    ragdoll.breakLimb(ragdoll_ptr.?, 2);
    ragdoll.breakLimb(ragdoll_ptr.?, 3);
    ragdoll.breakLimb(ragdoll_ptr.?, 4);
    ragdoll.breakLimb(ragdoll_ptr.?, 5);
    ragdoll.breakLimb(ragdoll_ptr.?, 6);
    ragdoll.breakLimb(ragdoll_ptr.?, 7);

    try testing.expect(ragdoll.isFullyBroken(ragdoll_ptr.?) == true);
}

test "Ragdoll resurrection" {
    ragdoll.init();
    const ragdoll_ptr = ragdoll.createHumanoid(100, 50, 100);
    try testing.expect(ragdoll_ptr != null);

    ragdoll_ptr.?.active = false;
    ragdoll.triggerResurrection(ragdoll_ptr.?, 60);
    try testing.expect(ragdoll_ptr.?.resurrection_tick == 60);
    try testing.expect(ragdoll.isResurrectionReady(ragdoll_ptr.?) == false);

    ragdoll_ptr.?.active = false;
    ragdoll_ptr.?.resurrection_tick = 0;
    try testing.expect(ragdoll.isResurrectionReady(ragdoll_ptr.?) == true);
}

test "Ragdoll update" {
    ragdoll.init();
    const ragdoll_ptr = ragdoll.createHumanoid(100, 50, 100);
    try testing.expect(ragdoll_ptr != null);

    ragdoll.update(ragdoll_ptr.?, 0.016);
    try testing.expect(ragdoll_ptr.?.active == true);
}

// ============================================================================
// Vehicle Tests (Phase 11)
// ============================================================================

test "Vehicle init and create car" {
    vehicle.init();
    const car = vehicle.createCar(100, 50, 100, 0);
    try testing.expect(car != null);
    try testing.expect(car.?.vehicle_type == .car);
    try testing.expect(car.?.mass == 1500);
    try testing.expect(car.?.speed == 0);
}

test "Vehicle create aircraft" {
    vehicle.init();
    const aircraft = vehicle.createAircraft(100, 50, 100);
    try testing.expect(aircraft != null);
    try testing.expect(aircraft.?.vehicle_type == .aircraft);
}

test "Vehicle create boat" {
    vehicle.init();
    const boat = vehicle.createBoat(100, 0, 100, 0);
    try testing.expect(boat != null);
    try testing.expect(boat.?.vehicle_type == .boat);
}

test "Vehicle create hovercraft" {
    vehicle.init();
    const hover = vehicle.createHovercraft(100, 50, 100, 0);
    try testing.expect(hover != null);
    try testing.expect(hover.?.vehicle_type == .hovercraft);
}

test "Vehicle throttle and steering" {
    vehicle.init();
    const car = vehicle.createCar(100, 50, 100, 0);
    try testing.expect(car != null);

    vehicle.applyThrottle(car.?, 0.5);
    try testing.expect(car.?.throttle == 0.5);

    vehicle.applySteering(car.?, -0.3);
    try testing.expect(car.?.steering == -0.3);

    vehicle.applyBrake(car.?, 0.8);
    try testing.expect(car.?.brake == 0.8);
}

test "Vehicle get forward direction" {
    vehicle.init();
    const car = vehicle.createCar(100, 50, 100, 0);
    try testing.expect(car != null);

    const fwd = vehicle.getForwardDir(car.?);
    try testing.expect(@abs(fwd.x - 0.0) < 0.001);
    try testing.expect(@abs(fwd.z - 1.0) < 0.001);

    car.?.yaw = 90.0 * 3.14159 / 180.0;
    const fwd2 = vehicle.getForwardDir(car.?);
    try testing.expect(@abs(fwd2.x - 1.0) < 0.01);
}

test "Vehicle check flipped" {
    vehicle.init();
    const car = vehicle.createCar(100, 50, 100, 0);
    try testing.expect(car != null);

    try testing.expect(vehicle.checkFlipped(car.?) == false);

    car.?.pitch = 2.0;
    try testing.expect(vehicle.checkFlipped(car.?) == true);
}

test "Vehicle handbrake" {
    vehicle.init();
    const car = vehicle.createCar(100, 50, 100, 0);
    try testing.expect(car != null);

    try testing.expect(car.?.handbrake == false);
    vehicle.setHandbrake(car.?, true);
    try testing.expect(car.?.handbrake == true);
}

// ============================================================================
// Network Tests (Phase 12)
// ============================================================================

test "Network init and create replica" {
    const config = network.SyncConfig{
        .send_rate_hz = 20,
        .timeout_ms = 500,
        .max_rollback_ticks = 10,
        .crc_check_enabled = true,
        .prediction_window = 5,
    };
    network.init(config);
    const replica = network.createReplica(1);
    try testing.expect(replica != null);
    try testing.expect(replica.?.entity_id == 1);
}

test "Network update and checksum" {
    const config = network.SyncConfig{
        .send_rate_hz = 20,
        .timeout_ms = 500,
        .max_rollback_ticks = 10,
        .crc_check_enabled = true,
        .prediction_window = 5,
    };
    network.init(config);
    const replica = network.createReplica(1);
    try testing.expect(replica != null);

    network.updateReplica(replica.?, 100, 50, 100, 0, 0, 0, 0, 1);
    try testing.expect(replica.?.pos_x == 100);
    try testing.expect(replica.?.tick == 1);
    try testing.expect(replica.?.checksum != 0);
}

test "Network store and get input" {
    const config = network.SyncConfig{
        .send_rate_hz = 20,
        .timeout_ms = 500,
        .max_rollback_ticks = 10,
        .crc_check_enabled = true,
        .prediction_window = 5,
    };
    network.init(config);

    const input = network.InputState{
        .tick = 10,
        .forward = true,
        .backward = false,
        .left = false,
        .right = true,
        .jump = true,
        .crouch = false,
        .fire = false,
        .aim_x = 0,
        .aim_y = 0,
    };
    network.storeInput(input);

    const retrieved = network.getInput(10);
    try testing.expect(retrieved != null);
    try testing.expect(retrieved.?.forward == true);
    try testing.expect(retrieved.?.jump == true);
}

test "Network predict" {
    const config = network.SyncConfig{
        .send_rate_hz = 20,
        .timeout_ms = 500,
        .max_rollback_ticks = 10,
        .crc_check_enabled = true,
        .prediction_window = 5,
    };
    network.init(config);
    const replica = network.createReplica(1);
    try testing.expect(replica != null);

    network.updateReplica(replica.?, 100, 50, 100, 0, 0, 0, 0, 0);

    const input = network.InputState{
        .tick = 1,
        .forward = true,
        .backward = false,
        .left = false,
        .right = false,
        .jump = false,
        .crouch = false,
        .fire = false,
        .aim_x = 0,
        .aim_y = 0,
    };

    const old_z = replica.?.pos_z;
    network.predict(replica.?, &input, 0.016);
    try testing.expect(replica.?.pos_z > old_z);
}

test "Network snapshot and restore" {
    const config = network.SyncConfig{
        .send_rate_hz = 20,
        .timeout_ms = 500,
        .max_rollback_ticks = 10,
        .crc_check_enabled = true,
        .prediction_window = 5,
    };
    network.init(config);
    const replica = network.createReplica(1);
    try testing.expect(replica != null);

    network.updateReplica(replica.?, 100, 50, 100, 0, 0, 0, 0, 0);
    const snapshot = network.saveSnapshot();

    network.updateReplica(replica.?, 200, 60, 110, 0, 0, 0, 0, 1);

    network.restoreSnapshot(&snapshot);
    try testing.expect(replica.?.pos_x == 100);
    try testing.expect(replica.?.pos_y == 50);
}

test "Network CRC calculation" {
    const config = network.SyncConfig{
        .send_rate_hz = 20,
        .timeout_ms = 500,
        .max_rollback_ticks = 10,
        .crc_check_enabled = true,
        .prediction_window = 5,
    };
    network.init(config);
    const replica = network.createReplica(1);
    try testing.expect(replica != null);

    network.updateReplica(replica.?, 100, 50, 100, 0, 0, 0, 0, 1);
    try testing.expect(replica.?.checksum != 0);

    const checksum1 = replica.?.checksum;
    network.updateReplica(replica.?, 100, 50, 100, 0, 0, 0, 0, 1);
    try testing.expect(replica.?.checksum == checksum1);
}

// ============================================================================
// Crash Defense Tests (Phase 13)
// ============================================================================

test "Crash defense init" {
    const config = crash_defense.SanityCheckConfig{
        .nan_check_enabled = true,
        .bounds_check_enabled = true,
        .energy_check_enabled = true,
        .velocity_cap = 10000.0,
        .position_min = -10000,
        .position_max = 10000,
        .max_ticks_without_progress = 1000,
    };
    crash_defense.init(config);
    try testing.expect(crash_defense.isEmergencyStopped() == false);
}

test "Crash defense isNaN and isValidFloat" {
    // Create NaN via bit pattern (Zig 0.14 doesn't produce NaN from 0.0/0.0)
    const nan_bits: u32 = 0x7FC00000;
    const nan: f32 = @bitCast(nan_bits);
    try testing.expect(crash_defense.isNaN(nan));
    try testing.expect(!crash_defense.isNaN(1.0));
    // Create Infinity via bit pattern
    const inf_bits: u32 = 0x7F800000;
    const inf: f32 = @bitCast(inf_bits);
    try testing.expect(crash_defense.isInfinite(inf));
    try testing.expect(!crash_defense.isInfinite(1.0));
    try testing.expect(crash_defense.isValidFloat(1.0));
    try testing.expect(!crash_defense.isValidFloat(nan));
}

test "Crash defense validate instance" {
    const config = crash_defense.SanityCheckConfig{
        .nan_check_enabled = true,
        .bounds_check_enabled = true,
        .energy_check_enabled = false,
        .velocity_cap = 10000.0,
        .position_min = -10000,
        .position_max = 10000,
        .max_ticks_without_progress = 1000,
    };
    crash_defense.init(config);

    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 100,
        .pos_y = 50,
        .pos_z = 100,
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
        ._reserved = undefined,
    };

    const result = crash_defense.validateInstance(&inst);
    try testing.expect(result.nan_detected == false);
    try testing.expect(result.bounds_violated == false);
}

test "Crash defense clamp instance" {
    const config = crash_defense.SanityCheckConfig{
        .nan_check_enabled = true,
        .bounds_check_enabled = true,
        .energy_check_enabled = false,
        .velocity_cap = 10000.0,
        .position_min = -10000,
        .position_max = 10000,
        .max_ticks_without_progress = 1000,
    };
    crash_defense.init(config);

    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 20000,
        .pos_y = 50,
        .pos_z = 100,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 20000,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = undefined,
    };

    crash_defense.clampInstance(&inst);
    try testing.expect(inst.pos_x <= 10000);
    try testing.expect(@abs(inst.vel_x) <= 10000);
}

test "Crash defense emergency stop" {
    const config = crash_defense.SanityCheckConfig{
        .nan_check_enabled = true,
        .bounds_check_enabled = true,
        .energy_check_enabled = false,
        .velocity_cap = 10000.0,
        .position_min = -10000,
        .position_max = 10000,
        .max_ticks_without_progress = 1000,
    };
    crash_defense.init(config);

    crash_defense.emergencyStop(undefined);
    try testing.expect(crash_defense.isEmergencyStopped() == true);

    crash_defense.resetEmergencyStop();
    try testing.expect(crash_defense.isEmergencyStopped() == false);
}

test "Crash defense progress tracking" {
    const config = crash_defense.SanityCheckConfig{
        .nan_check_enabled = true,
        .bounds_check_enabled = true,
        .energy_check_enabled = false,
        .velocity_cap = 10000.0,
        .position_min = -10000,
        .position_max = 10000,
        .max_ticks_without_progress = 1000,
    };
    crash_defense.init(config);

    crash_defense.updateProgress(100);
    try testing.expect(crash_defense.isStuck(1050) == false);
    try testing.expect(crash_defense.isStuck(1101) == true);
}

test "Crash defense calculate energy" {
    const config = crash_defense.SanityCheckConfig{
        .nan_check_enabled = true,
        .bounds_check_enabled = true,
        .energy_check_enabled = false,
        .velocity_cap = 10000.0,
        .position_min = -10000,
        .position_max = 10000,
        .max_ticks_without_progress = 1000,
    };
    crash_defense.init(config);

    const allocator = std.testing.allocator;
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();

    var entities: [1]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();
    entities[0].physics.mass = 10;

    const inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 10,
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
        ._reserved = undefined,
    };
    _ = try s1024.addInstance(inst);

    const energy = crash_defense.calculateEnergy(&s1024, &entities);
    try testing.expect(energy > 0);
}
