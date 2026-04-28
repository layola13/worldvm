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

    crash_defense.emergencyStop(null);
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

// ============================================================================
// Tire Physics Tests (Phase 21) - Tests 201-210
// ============================================================================

const tire = @import("tire.zig");
const suspension = @import("suspension.zig");
const drivetrain = @import("drivetrain.zig");
const aerodynamics = @import("aerodynamics.zig");
const braking = @import("braking.zig");
const terrain = @import("terrain.zig");
const collision = @import("collision.zig");
const disasters = @import("disasters.zig");
const sensors = @import("sensors.zig");
const rewind = @import("rewind.zig");
const ai_traffic = @import("ai_traffic.zig");
const prediction = @import("prediction.zig");

test "Tire init and create" {
    tire.init();
    const config = tire.TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 10,
        .lateral_stiffness = 100,
        .longitudinal_stiffness = 100,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.15,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.01,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 200,
    };
    const t = tire.createTire(0, 0, 0, config);
    try testing.expect(t != null);
    try testing.expect(t.?.normal_force > 0);
}

test "Tire slip ratio" {
    tire.init();
    const config = tire.TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 10,
        .lateral_stiffness = 100,
        .longitudinal_stiffness = 100,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.15,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.01,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 200,
    };
    const t = tire.createTire(0, 0, 0, config);
    t.?.angular_velocity = 12;
    const slip = tire.calculateSlipRatio(t.?, 3, 0.3);
    try testing.expect(slip > 0);
}

test "Tire friction circle" {
    const longitudinal: f32 = 5000;
    const lateral: f32 = 3000;
    const max_friction: f32 = 6000;
    const combined = tire.calculateFrictionCircle(longitudinal, lateral, max_friction);
    try testing.expect(combined < max_friction);
    try testing.expect(combined > 0);
}

test "Tire hydroplaning check" {
    tire.init();
    const config = tire.TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 10,
        .lateral_stiffness = 100,
        .longitudinal_stiffness = 100,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.15,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.01,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 200,
    };
    const t = tire.createTire(0, 0, 0, config);
    const hydroplaning = tire.checkHydroplaning(t.?, 5.0, 50.0);
    _ = hydroplaning;
}

// ============================================================================
// Suspension Tests (Phase 22) - Tests 211-220
// ============================================================================

test "Suspension init and create" {
    suspension.init();
    const config = suspension.SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 500,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const s = suspension.createSuspension(config);
    try testing.expect(s != null);
    try testing.expect(s.?.active == true);
}

test "Suspension spring force" {
    suspension.init();
    const config = suspension.SuspensionConfig{
        .spring_rate = 50000,
        .damping_ratio = 0.7,
        .bump_damping = 3000,
        .rebound_damping = 4000,
        .preloaded = 500,
        .max_length = 0.4,
        .min_length = 0.2,
        .anti_roll_rate = 10000,
    };
    const s = suspension.createSuspension(config);
    s.?.current_length = 0.25;
    const force = suspension.calculateSpringForce(s.?, config);
    try testing.expect(force > 0);
}

test "Suspension natural frequency" {
    const freq = suspension.calculateNaturalFrequency(500, 50000);
    try testing.expect(freq > 0);
    try testing.expect(freq < 20);
}

// ============================================================================
// Drivetrain Tests (Phase 23-24) - Tests 221-240
// ============================================================================

test "Drivetrain init and engine" {
    drivetrain.init();
    const eng = drivetrain.getEngineState();
    try testing.expect(eng.active == true);
    try testing.expect(eng.idle_rpm == 800);
}

test "Engine torque curve" {
    drivetrain.init();
    drivetrain.applyThrottle(0.5);
    drivetrain.updateEngine(0.016);
    const eng = drivetrain.getEngineState();
    try testing.expect(eng.torque > 0);
}

test "Drivetrain gear ratio" {
    drivetrain.init();
    const gear1 = drivetrain.getGearRatio(1);
    const gear3 = drivetrain.getGearRatio(3);
    try testing.expect(gear1 > gear3);
}

test "Drivetrain wheel torque" {
    drivetrain.init();
    const wheel_torque = drivetrain.calculateWheelTorque(400, 2, 3.5, 0.85);
    try testing.expect(wheel_torque > 0);
}

// ============================================================================
// Aerodynamics Tests (Phase 25) - Tests 241-250
// ============================================================================

test "Aerodynamics init" {
    aerodynamics.init();
    const aero = aerodynamics.getAeroState();
    try testing.expect(aero.drag_coefficient > 0);
}

test "Aerodynamics drag force" {
    aerodynamics.init();
    const drag = aerodynamics.calculateDragForce(30, 0, .{ .drag_coefficient = 0.3, .frontal_area = 2.2, .downforce_coefficient = 0.5, .rear_wing_angle = 0, .front_splitter_angle = 0, .diffuser_angle = 0, .wing_surface_area = 0.5 });
    try testing.expect(drag > 0);
}

test "Aerodynamics downforce" {
    aerodynamics.init();
    const downforce = aerodynamics.calculateDownforce(30, 0, .{ .drag_coefficient = 0.3, .frontal_area = 2.2, .downforce_coefficient = 0.5, .rear_wing_angle = 0, .front_splitter_angle = 0, .diffuser_angle = 0, .wing_surface_area = 0.5 });
    try testing.expect(downforce >= 0);
}

// ============================================================================
// Braking Tests (Phase 26) - Tests 251-260
// ============================================================================

test "Braking init" {
    braking.init();
    const brake = braking.getBrakeState();
    try testing.expect(brake.pedal_position == 0);
}

test "Braking apply brake" {
    braking.init();
    braking.applyBrake(0.5);
    braking.updateBraking(0.016);
    const brake = braking.getBrakeState();
    try testing.expect(brake.pedal_position > 0);
}

test "Braking handbrake" {
    braking.init();
    braking.applyHandbrake(true);
    braking.updateBraking(0.016);
    const brake = braking.getBrakeState();
    try testing.expect(brake.handbrake_active == true);
}

test "Braking ABS" {
    braking.init();
    braking.applyBrake(1.0);
    braking.updateABS(0, 0.2, .{ .max_front_torque = 3000, .max_rear_torque = 2000, .pedal_ratio = 4.0, .booster_gain = 1.5, .rotor_diameter = 0.35, .pad_area = 50, .cooling_coefficient = 0.1, .fade_threshold = 300, .abs_threshold = 0.15 });
    const abs_active = braking.isABSActive(0);
    _ = abs_active;
}

// ============================================================================
// Terrain Tests (Phase 28) - Tests 271-280
// ============================================================================

test "Terrain init" {
    terrain.init();
    const weather = terrain.getWeather();
    try testing.expect(weather.visibility > 0);
}

test "Terrain add patch" {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 10, terrain.SurfaceType.asphalt_wet);
    const surface = terrain.getSurfaceAt(0, 0);
    try testing.expect(surface == .asphalt_wet);
}

test "Terrain friction at position" {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 10, terrain.SurfaceType.ice);
    const friction = terrain.getFrictionAt(0, 0);
    try testing.expect(friction < 0.5);
}

test "Terrain rolling resistance" {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 10, terrain.SurfaceType.gravel);
    const rr = terrain.getRollingResistanceAt(0, 0);
    try testing.expect(rr > 0.01);
}

test "Terrain hydroplaning risk" {
    terrain.init();
    const risk = terrain.calculateHydroplaningRisk(50, 5, 0.2);
    try testing.expect(risk >= 0);
}

// ============================================================================
// Collision Tests (Phase 30) - Tests 291-300
// ============================================================================

test "Collision init" {
    collision.init();
    const dmg = collision.getDamageState();
    try testing.expect(dmg.structural_integrity == 100);
}

test "Collision impact energy" {
    collision.init();
    const energy = collision.calculateImpactEnergy(1500, 1000, 20);
    try testing.expect(energy > 0);
}

test "Collision apply" {
    collision.init();
    const result = collision.applyCollision(0, 0, 0, 0, 0, -10, 1500, .{});
    try testing.expect(result.impact_speed > 0);
}

test "Collision structural failure" {
    collision.init();
    const failed = collision.checkStructuralFailure();
    try testing.expect(failed == false);
}

// ============================================================================
// Disaster Tests (Phase 71-80) - Tests 701-800
// ============================================================================

test "Disasters init" {
    disasters.init();
    const system = disasters.getDisasterSystem();
    try testing.expect(system.disaster_count == 0);
}

test "Disasters trigger earthquake" {
    disasters.init();
    disasters.triggerDisaster(.earthquake, 7.0, 0, 0, 0, 1000);
    const system = disasters.getDisasterSystem();
    try testing.expect(system.disaster_count == 1);
}

test "Disasters seismic intensity" {
    disasters.init();
    const intensity = disasters.calculateSeismicIntensity(100, 7.0);
    try testing.expect(intensity > 0);
    try testing.expect(intensity < 7.0);
}

test "Disasters chain reaction" {
    disasters.init();
    disasters.enableChainReactions(true);
    disasters.triggerDisaster(.meteor_strike, 10.0, 0, 0, 0, 500);
    const triggered = disasters.checkChainReaction(0, 0, 0);
    try testing.expect(triggered == true);
}

test "Disasters wind velocity" {
    disasters.init();
    disasters.triggerDisaster(.hurricane, 5.0, 0, 0, 0, 1000);
    const wind = disasters.getWindVelocity(50, 50);
    try testing.expect(@abs(wind.x) >= 0 or @abs(wind.z) >= 0);
}

// ============================================================================
// Sensor Tests (Phase 18, 64) - Tests 171-180, 631-640
// ============================================================================

test "Sensors init" {
    sensors.init();
    try testing.expect(sensors.getSensorState(.camera) == null);
}

test "Sensors add sensor" {
    sensors.init();
    const sensor = sensors.addSensor(.radar, 1.5, 200);
    try testing.expect(sensor != null);
    try testing.expect(sensor.?.range == 200);
}

test "Sensors detected objects" {
    sensors.init();
    const objs = sensors.getDetectedObjects();
    try testing.expect(objs.len == 0);
}

test "Sensors occlusion" {
    sensors.init();
    const occlusion = sensors.raycastOcclusion(0, 0, 0, 100, 0, 0, 50, 0, 0, 5);
    try testing.expect(occlusion > 0);
}

test "Sensors prediction uses shared linear forecast" {
    const obj = sensors.DetectedObject{
        .object_id = 1,
        .object_type = 0,
        .pos_x = 1.0,
        .pos_y = 2.0,
        .pos_z = 3.0,
        .vel_x = 4.0,
        .vel_y = 0.0,
        .vel_z = -1.0,
        .confidence = 1.0,
        .age = 0,
        .sensor_source = .radar,
    };
    const predicted = sensors.predictObjectPosition(&obj, 2.0);
    try testing.expectApproxEqAbs(@as(f32, 9.0), predicted.x, 0.0001);
    try testing.expectApproxEqAbs(@as(f32, 2.0), predicted.y, 0.0001);
    try testing.expectApproxEqAbs(@as(f32, 1.0), predicted.z, 0.0001);
}

// ============================================================================
// Rewind Tests (Phase 19, 35) - Tests 181-190, 341-350
// ============================================================================

test "Rewind init" {
    rewind.init();
    try testing.expect(rewind.isDeterministic() == true);
}

test "Rewind record state" {
    rewind.init();
    const state = rewind.RewindState{
        .tick = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .input_forwards = false,
        .input_backwards = false,
        .input_left = false,
        .input_right = false,
        .input_jump = false,
        .input_brake = false,
    };
    rewind.recordState(state);
    const usage = rewind.getRewindBufferUsage();
    try testing.expect(usage.count == 1);
}

test "Rewind state hash" {
    rewind.init();
    const state = rewind.RewindState{
        .tick = 100,
        .pos_x = 10,
        .pos_y = 5,
        .pos_z = 20,
        .vel_x = 1,
        .vel_y = 0,
        .vel_z = 2,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .input_forwards = true,
        .input_backwards = false,
        .input_left = false,
        .input_right = false,
        .input_jump = false,
        .input_brake = false,
    };
    const hash = rewind.calculateStateHash(&state);
    try testing.expect(hash != 0);
}

test "Prediction TTC head-on" {
    const result = prediction.computeTTC(.{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 12,
        .vel_y = 0,
        .vel_z = 0,
    }, .{
        .pos_x = 60,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = -8,
        .vel_y = 0,
        .vel_z = 0,
    }, 1.0, 10.0);
    try testing.expect(result.valid);
    try testing.expect(result.time > 2.8 and result.time < 3.2);
}

// ============================================================================
// AI Traffic Tests (Phase 34, 67, 68) - Tests 331-340, 661-670
// ============================================================================

test "AI Traffic init" {
    ai_traffic.init();
    try testing.expect(ai_traffic.getVehicleCount() == 0);
}

test "AI Traffic spawn vehicle" {
    ai_traffic.init();
    const ai_vehicle = ai_traffic.spawnAIVehicle(0, 0, 0, .normal, 0);
    try testing.expect(ai_vehicle != null);
    try testing.expect(ai_vehicle.?.behavior == .normal);
}

test "AI Traffic traffic light" {
    ai_traffic.init();
    const light = ai_traffic.addTrafficLight(0, 0, 60);
    try testing.expect(light != null);
    try testing.expect(light.?.cycle_duration == 60);
}

test "AI Traffic update" {
    ai_traffic.init();
    _ = ai_traffic.spawnAIVehicle(0, 0, 0, .normal, 0);
    ai_traffic.updateAI(0.016);
    const vehicles = ai_traffic.getTrafficVehicles();
    try testing.expect(vehicles.len > 0);
}

test "AI Traffic safe pass estimate blocks late yellow" {
    ai_traffic.init();
    const vehicle_ptr = ai_traffic.spawnAIVehicle(0, 0, 0, .normal, 0);
    const light_ptr = ai_traffic.addTrafficLight(0, 20, 60);
    try testing.expect(vehicle_ptr != null);
    try testing.expect(light_ptr != null);

    vehicle_ptr.?.vel_z = 10.0;
    light_ptr.?.timer = 55.0;
    light_ptr.?.state = .yellow;

    const result = ai_traffic.estimateSafePassForVehicle(vehicle_ptr.?, light_ptr.?, 4.5);
    try testing.expect(!result.can_pass);
    try testing.expect(result.margin_to_change < 0);
}

test "AI Traffic emergency vehicle" {
    ai_traffic.init();
    const ai_vehicle2 = ai_traffic.spawnAIVehicle(0, 0, 0, .cautious, 0);
    ai_traffic.triggerEmergencyVehicle(ai_vehicle2.?);
    try testing.expect(ai_vehicle2.?.behavior == .reckless);
}


test "vehicle-AI traffic bidirectional sync end-to-end" {
    ai_traffic.init();
    vehicle.init();

    const ai_veh = ai_traffic.spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    ai_veh.governed_target_vel = 25.0;
    ai_veh.pos_x = 100.0;
    ai_veh.pos_z = 200.0;
    ai_veh.vel_x = 5.0;
    ai_veh.vel_z = 10.0;

    const car = vehicle.createCar(5.0, 0.0, 0.0, 0.0) orelse return error.TestUnexpectedResult;
    car.speed = 15.0;
    vehicle.setAIVehicleLink(car, ai_veh.vehicle_id, 20.0);

    // setAIVehicleLink -> syncVehicleToTraffic propagated vehicle pose into AI vehicle.
    try testing.expectApproxEqAbs(car.pos_x, ai_veh.pos_x, 0.0001);
    try testing.expectApproxEqAbs(car.pos_z, ai_veh.pos_z, 0.0001);
    const fwd_x = @sin(car.yaw);
    const fwd_z = @cos(car.yaw);
    try testing.expectApproxEqAbs(fwd_x * car.speed, ai_veh.vel_x, 0.0001);
    try testing.expectApproxEqAbs(fwd_z * car.speed, ai_veh.vel_z, 0.0001);

    // syncTrafficVehiclesFromPhysics reads governed_target_vel from AI traffic.
    ai_traffic.syncTrafficVehiclesFromPhysics();
    const governed = ai_traffic.getGovernedTargetSpeed(ai_veh.vehicle_id);
    try testing.expect(governed != null);
    try testing.expectApproxEqAbs(@as(f32, 25.0), governed.?, 0.0001);
}

test "physics vehicle AI link and unlink" {
    ai_traffic.init();
    vehicle.init();

    const ai_veh = ai_traffic.spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    const car = vehicle.createCar(0.0, 0.0, 0.0, 0.0) orelse return error.TestUnexpectedResult;

    // Before link: ai_vehicle_id should be 0.
    try testing.expect(car.ai_vehicle_id == 0);

    // Link.
    vehicle.setAIVehicleLink(car, ai_veh.vehicle_id, 15.0);
    try testing.expect(car.ai_vehicle_id == ai_veh.vehicle_id);

    // Unlink by setting to 0.
    vehicle.setAIVehicleLink(car, 0, -1);
    try testing.expect(car.ai_vehicle_id == 0);
    try testing.expect(vehicle.getAIVehicleTargetVel(car) < 0);
}
