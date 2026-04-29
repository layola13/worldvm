//! Tire Physics - Pacejka/Magic Formula and Friction Models
//!
//! Phase 21: Tire physics, slip ratios, slip angles, friction circles, thermal dynamics
//! Handles: Longitudinal/lateral forces, camber thrust, hydroplaning, rolling resistance

const std = @import("std");

pub const TireState = struct {
    pos_x: f32 = 0,
    pos_y: f32 = 0,
    pos_z: f32 = 0,
    vel_x: f32 = 0,
    vel_y: f32 = 0,
    vel_z: f32 = 0,
    yaw: f32 = 0,
    pitch: f32 = 0,
    roll: f32 = 0,
    angular_velocity: f32 = 0,
    steering_angle: f32 = 0,
    camber_angle: f32 = 0,
    normal_force: f32 = 0,
    load_sensitivity: f32 = 1.0,
    surface_temperature: f32 = 20.0,
    core_temperature: f32 = 20.0,
    grip_level: f32 = 1.0,
    rolling_resistance: f32 = 0,
    hydroplaning: bool = false,
    heat_transfer_coefficient: f32 = 0.1,
};

pub const TireConfig = struct {
    radius: f32,
    width: f32,
    mass: f32,
    lateral_stiffness: f32,
    longitudinal_stiffness: f32,
    camber_thrust_coefficient: f32,
    peak_slip_ratio: f32,
    peak_slip_angle: f32,
    friction_coefficient: f32,
    rolling_resistance_coefficient: f32,
    heat_transfer_coefficient: f32,
    optimal_temperature: f32,
    max_temperature: f32,
};

pub const MAX_TIRES: usize = 16;

pub const TireSystem = struct {
    tires: [MAX_TIRES]TireState,
    count: u8,
};

var g_tire_system: TireSystem = undefined;

pub fn init() void {
    g_tire_system.count = 0;
}

pub fn createTire(x: f32, y: f32, z: f32, config: TireConfig) ?*TireState {
    if (g_tire_system.count >= MAX_TIRES) return null;
    const idx = g_tire_system.count;
    g_tire_system.count += 1;
    const tire = &g_tire_system.tires[idx];

    tire.* = .{
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .angular_velocity = 0,
        .steering_angle = 0,
        .camber_angle = 0,
        .normal_force = config.mass * 9.81,
        .load_sensitivity = 1.0,
        .surface_temperature = 20.0,
        .core_temperature = 20.0,
        .grip_level = 1.0,
        .rolling_resistance = config.rolling_resistance_coefficient,
        .hydroplaning = false,
        .heat_transfer_coefficient = config.heat_transfer_coefficient,
    };
    return tire;
}

pub fn calculateSlipRatio(tire: *const TireState, vehicle_speed: f32, tire_radius: f32) f32 {
    if (@abs(vehicle_speed) < 0.1) return 0;
    const tire_velocity = tire.angular_velocity * tire_radius;
    return (tire_velocity - vehicle_speed) / @max(@abs(vehicle_speed), 0.1);
}

pub fn calculateSlipAngle(tire: *const TireState, vehicle_speed: f32, yaw_rate: f32) f32 {
    if (@abs(vehicle_speed) < 0.1) return 0;
    const slip_angle = std.math.atan(yaw_rate * 2.0 / @max(vehicle_speed, 0.1));
    return slip_angle - tire.steering_angle;
}

pub fn pacejkaMagicFormula(slip: f32, peak_slip: f32, friction_coef: f32, load: f32) f32 {
    const B = 10.0;
    const C = 1.9;
    const D = friction_coef * load;
    const E = 0.97;
    const x = B * slip / peak_slip;
    return D * std.math.sin(C * std.math.atan(x - E * (x - std.math.atan(x))));
}

pub fn calculateLongitudinalForce(tire: *const TireState, slip_ratio: f32, config: TireConfig) f32 {
    const effective_slip = @max(-3.0, @min(3.0, slip_ratio));
    const base_force = pacejkaMagicFormula(effective_slip, config.peak_slip_ratio, config.friction_coefficient, tire.normal_force);
    const temp_factor = if (tire.surface_temperature > config.optimal_temperature)
        1.0 - (tire.surface_temperature - config.optimal_temperature) / (config.max_temperature - config.optimal_temperature) * 0.5
    else
        1.0;
    const grip = base_force * tire.grip_level * temp_factor;
    return @max(-tire.normal_force * config.friction_coefficient, @min(tire.normal_force * config.friction_coefficient, grip));
}

pub fn calculateLateralForce(tire: *const TireState, slip_angle: f32, config: TireConfig) f32 {
    const effective_angle = @max(-1.2, @min(1.2, slip_angle));
    const base_force = pacejkaMagicFormula(effective_angle, config.peak_slip_angle, config.friction_coefficient, tire.normal_force);
    const camber_force = tire.camber_angle * config.camber_thrust_coefficient * tire.normal_force;
    const temp_factor = if (tire.surface_temperature > config.optimal_temperature)
        1.0 - (tire.surface_temperature - config.optimal_temperature) / (config.max_temperature - config.optimal_temperature) * 0.5
    else
        1.0;
    const grip = (base_force + camber_force) * tire.grip_level * temp_factor;
    return @max(-tire.normal_force * config.friction_coefficient, @min(tire.normal_force * config.friction_coefficient, grip));
}

pub fn calculateFrictionCircle(longitudinal: f32, lateral: f32, max_friction: f32) f32 {
    const combined = @sqrt(longitudinal * longitudinal + lateral * lateral);
    if (combined > max_friction) {
        return max_friction;
    }
    return combined;
}

pub fn applyLoadSensitivity(tire: *TireState, vertical_load: f32) void {
    tire.normal_force = vertical_load;
    const base_sensitivity: f32 = 0.02;
    tire.load_sensitivity = 1.0 / (1.0 + base_sensitivity * vertical_load / 1000.0);
}

pub fn updateTemperature(tire: *TireState, friction_work: f32, dt: f32, air_temp: f32) void {
    const heat_input = friction_work * 0.3;
    const surface_cooling = (tire.surface_temperature - air_temp) * tire.heat_transfer_coefficient * dt;
    const core_cooling = (tire.core_temperature - tire.surface_temperature) * tire.heat_transfer_coefficient * 0.5 * dt;

    tire.surface_temperature += heat_input - surface_cooling;
    tire.core_temperature += core_cooling;

    if (tire.surface_temperature < air_temp) tire.surface_temperature = air_temp;
    if (tire.core_temperature < air_temp) tire.core_temperature = air_temp;
}

pub fn checkHydroplaning(tire: *const TireState, water_depth: f32, speed: f32) bool {
    if (water_depth < 0.001) return false;
    const critical_speed = @sqrt(tire.normal_force / (water_depth * 0.1)) * 0.1;
    return speed > critical_speed;
}

pub fn calculateRollingResistance(tire: *const TireState, speed: f32, config: TireConfig) f32 {
    const base_rr = config.rolling_resistance_coefficient;
    const speed_factor = 1.0 + @abs(speed) / 100.0;
    const temp_factor = if (tire.surface_temperature > config.optimal_temperature)
        1.0 + (tire.surface_temperature - config.optimal_temperature) / 100.0
    else
        1.0;
    return base_rr * speed_factor * temp_factor * tire.normal_force;
}

pub fn applySteering(tire: *TireState, angle: f32) void {
    tire.steering_angle = @max(-1.2, @min(1.2, angle));
}

pub fn applyCamber(tire: *TireState, angle: f32) void {
    tire.camber_angle = @max(-0.3, @min(0.3, angle));
}

pub fn getWheelPositions(tire: *const TireState) struct { x: f32, y: f32, z: f32 } {
    return .{ .x = tire.pos_x, .y = tire.pos_y, .z = tire.pos_z };
}

pub fn getSystem() *TireSystem {
    return &g_tire_system;
}

// ============================================================================
// Tests for Tire System (Items 511-525)
// ============================================================================

test "511: tire physics model - tire creation and basic properties" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config);
    try std.testing.expect(tire != null);
    try std.testing.expect(tire.?.normal_force > 0);
    try std.testing.expect(tire.?.grip_level == 1.0);
    try std.testing.expect(tire.?.hydroplaning == false);
}

test "512: tire friction circle - combined force limiting" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    const max_friction = tire.normal_force * config.friction_coefficient;
    const combined = calculateFrictionCircle(5000, 3000, max_friction);
    try std.testing.expect(combined <= max_friction);
}

test "513: tire slip ratio - calculation from velocity difference" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    tire.angular_velocity = 20.0;
    const slip_ratio = calculateSlipRatio(tire, 10.0, config.radius);
    try std.testing.expect(slip_ratio != 0);
}

test "514: tire slip angle - calculation from yaw rate" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    tire.steering_angle = 0.1;
    const slip_angle = calculateSlipAngle(tire, 20.0, 0.5);
    try std.testing.expect(slip_angle != 0);
}

test "515: tire longitudinal force - force calculation from slip ratio" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    const long_force = calculateLongitudinalForce(tire, 0.1, config);
    try std.testing.expect(long_force != 0);
}

test "516: tire lateral force - force calculation from slip angle" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    tire.camber_angle = 0.05;
    const lat_force = calculateLateralForce(tire, 0.08, config);
    try std.testing.expect(lat_force != 0);
}

test "517: tire combined slip - interaction of longitudinal and lateral forces" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    tire.normal_force = 4000;
    const long_force = calculateLongitudinalForce(tire, 0.05, config);
    const lat_force = calculateLateralForce(tire, 0.05, config);
    const max_friction = tire.normal_force * config.friction_coefficient;
    const combined = calculateFrictionCircle(long_force, lat_force, max_friction);
    const uncapped = @sqrt(long_force * long_force + lat_force * lat_force);
    try std.testing.expect(combined >= 0.0);
    try std.testing.expect(combined <= max_friction);
    try std.testing.expectApproxEqAbs(@min(uncapped, max_friction), combined, 0.001);
}

test "518: tire thermal model - temperature update from friction work" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    const initial_temp = tire.surface_temperature;
    updateTemperature(tire, 100.0, 0.1, 20.0);
    try std.testing.expect(tire.surface_temperature != initial_temp);
}

test "519: tire wear model - grip level affects force output" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    tire.grip_level = 1.0;
    const force_full_grip = calculateLongitudinalForce(tire, 0.1, config);
    tire.grip_level = 0.5;
    const force_worn = calculateLongitudinalForce(tire, 0.1, config);
    try std.testing.expect(@abs(force_worn) < @abs(force_full_grip));
}

test "520: tire pressure model - load sensitivity affects force distribution" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    applyLoadSensitivity(tire, 1000);
    try std.testing.expect(tire.normal_force == 1000);
    try std.testing.expect(tire.load_sensitivity < 1.0);
}

test "521: tire hydroplaning - detection at high speed in water" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    tire.normal_force = 4000;
    const no_hydro = checkHydroplaning(tire, 0.0005, 30.0);
    const hydro = checkHydroplaning(tire, 0.1, 80.0);
    try std.testing.expect(!no_hydro);
    try std.testing.expect(hydro);
}

test "522: tire snow conditions - reduced friction coefficient" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 0.4,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    const snow_force = calculateLongitudinalForce(tire, 0.1, config);
    try std.testing.expect(@abs(snow_force) < tire.normal_force);
}

test "523: tire ice conditions - very low friction" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 0.15,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    const ice_force = calculateLongitudinalForce(tire, 0.05, config);
    try std.testing.expect(@abs(ice_force) < 500);
}

test "524: tire off-road - higher slip threshold for mud" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.3,
        .peak_slip_angle = 0.2,
        .friction_coefficient = 0.6,
        .rolling_resistance_coefficient = 0.02,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(0, 0, 0, config).?;
    tire.normal_force = 3000;
    const offroad_force = calculateLongitudinalForce(tire, 0.2, config);
    try std.testing.expect(offroad_force != 0);
}

test "525: tire raycast suspension - position tracking" {
    init();
    const config = TireConfig{
        .radius = 0.3,
        .width = 0.2,
        .mass = 5.0,
        .lateral_stiffness = 10000,
        .longitudinal_stiffness = 10000,
        .camber_thrust_coefficient = 0.5,
        .peak_slip_ratio = 0.15,
        .peak_slip_angle = 0.1,
        .friction_coefficient = 1.0,
        .rolling_resistance_coefficient = 0.015,
        .heat_transfer_coefficient = 0.1,
        .optimal_temperature = 90,
        .max_temperature = 120,
    };
    const tire = createTire(10.0, 5.0, 20.0, config).?;
    const pos = getWheelPositions(tire);
    try std.testing.expect(pos.x == 10.0);
    try std.testing.expect(pos.y == 5.0);
    try std.testing.expect(pos.z == 20.0);
}
