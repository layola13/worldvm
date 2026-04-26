//! Vehicle - Vehicle Physics System
//!
//! Phase 11: Ground vehicles, aircraft, watercraft, hovercraft
//! Handles: Driving, steering, drift, collision response, flipping

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const prediction = @import("prediction.zig");
const query = @import("query.zig");

const WheelWorldPosition = struct {
    x: f32,
    y: f32,
    z: f32,
};

pub const VehicleType = enum(u8) {
    car = 0,
    truck = 1,
    motorcycle = 2,
    aircraft = 3,
    boat = 4,
    hovercraft = 5,
};

pub const WheelConfig = struct {
    offset_x: i8,
    offset_y: i8,
    offset_z: i8,
    radius: u8,
    steering_angle: f32,
    driven: bool,
    braked: bool,
};

pub const VehicleState = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    yaw: f32,
    pitch: f32,
    roll: f32,
    speed: f32,
    angular_velocity: f32,
    throttle: f32,
    steering: f32,
    brake: f32,
    handbrake: bool,
    grounded: bool,
    flipped: bool,
    vehicle_type: VehicleType,
    wheels: [4]WheelConfig,
    mass: u16,
};

pub const MAX_VEHICLES: usize = 16;

pub const VehicleSystem = struct {
    vehicles: [MAX_VEHICLES]VehicleState,
    count: u8,
};

var g_vehicle_system: VehicleSystem = undefined;

pub fn init() void {
    g_vehicle_system.count = 0;
}

/// Default car configuration
pub fn createCar(x: f32, y: f32, z: f32, yaw: f32) ?*VehicleState {
    if (g_vehicle_system.count >= MAX_VEHICLES) return null;
    const idx = g_vehicle_system.count;
    g_vehicle_system.count += 1;
    const vehicle = &g_vehicle_system.vehicles[idx];

    vehicle.* = .{
        .pos_x = x, .pos_y = y, .pos_z = z,
        .yaw = yaw, .pitch = 0, .roll = 0,
        .speed = 0, .angular_velocity = 0,
        .throttle = 0, .steering = 0, .brake = 0,
        .handbrake = false, .grounded = false, .flipped = false,
        .vehicle_type = .car,
        .wheels = [_]WheelConfig{
            .{ .offset_x = -6, .offset_y = 0, .offset_z = -8, .radius = 4, .steering_angle = 0, .driven = true, .braked = true },
            .{ .offset_x = 6, .offset_y = 0, .offset_z = -8, .radius = 4, .steering_angle = 0, .driven = true, .braked = true },
            .{ .offset_x = -6, .offset_y = 0, .offset_z = 8, .radius = 4, .steering_angle = 0, .driven = false, .braked = true },
            .{ .offset_x = 6, .offset_y = 0, .offset_z = 8, .radius = 4, .steering_angle = 0, .driven = false, .braked = true },
        },
        .mass = 1500,
    };
    return vehicle;
}

/// Default aircraft configuration
pub fn createAircraft(x: f32, y: f32, z: f32) ?*VehicleState {
    if (g_vehicle_system.count >= MAX_VEHICLES) return null;
    const idx = g_vehicle_system.count;
    g_vehicle_system.count += 1;
    const vehicle = &g_vehicle_system.vehicles[idx];

    vehicle.* = .{
        .pos_x = x, .pos_y = y, .pos_z = z,
        .yaw = 0, .pitch = 0, .roll = 0,
        .speed = 0, .angular_velocity = 0,
        .throttle = 0, .steering = 0, .brake = 0,
        .handbrake = false, .grounded = false, .flipped = false,
        .vehicle_type = .aircraft,
        .wheels = undefined,
        .mass = 5000,
    };
    return vehicle;
}

/// Default boat configuration
pub fn createBoat(x: f32, y: f32, z: f32, yaw: f32) ?*VehicleState {
    if (g_vehicle_system.count >= MAX_VEHICLES) return null;
    const idx = g_vehicle_system.count;
    g_vehicle_system.count += 1;
    const vehicle = &g_vehicle_system.vehicles[idx];

    vehicle.* = .{
        .pos_x = x, .pos_y = y, .pos_z = z,
        .yaw = yaw, .pitch = 0, .roll = 0,
        .speed = 0, .angular_velocity = 0,
        .throttle = 0, .steering = 0, .brake = 0,
        .handbrake = false, .grounded = false, .flipped = false,
        .vehicle_type = .boat,
        .wheels = undefined,
        .mass = 2000,
    };
    return vehicle;
}

/// Default hovercraft configuration
pub fn createHovercraft(x: f32, y: f32, z: f32, yaw: f32) ?*VehicleState {
    if (g_vehicle_system.count >= MAX_VEHICLES) return null;
    const idx = g_vehicle_system.count;
    g_vehicle_system.count += 1;
    const vehicle = &g_vehicle_system.vehicles[idx];

    vehicle.* = .{
        .pos_x = x, .pos_y = y, .pos_z = z,
        .yaw = yaw, .pitch = 0, .roll = 0,
        .speed = 0, .angular_velocity = 0,
        .throttle = 0, .steering = 0, .brake = 0,
        .handbrake = false, .grounded = true, .flipped = false,
        .vehicle_type = .hovercraft,
        .wheels = undefined,
        .mass = 1000,
    };
    return vehicle;
}

pub fn removeVehicle(vehicle: *VehicleState) void {
    _ = vehicle;
    if (g_vehicle_system.count > 0) g_vehicle_system.count -= 1;
}

/// Apply throttle input (-1 to 1)
pub fn applyThrottle(vehicle: *VehicleState, amount: f32) void {
    vehicle.throttle = @max(-1.0, @min(1.0, amount));
}

/// Apply steering input (-1 to 1)
pub fn applySteering(vehicle: *VehicleState, amount: f32) void {
    vehicle.steering = @max(-1.0, @min(1.0, amount));
}

/// Apply brake input (0 to 1)
pub fn applyBrake(vehicle: *VehicleState, amount: f32) void {
    vehicle.brake = @max(0.0, @min(1.0, amount));
}

/// Apply handbrake
pub fn setHandbrake(vehicle: *VehicleState, active: bool) void {
    vehicle.handbrake = active;
}

/// Get forward direction vector
pub fn getForwardDir(vehicle: *const VehicleState) struct { x: f32, z: f32 } {
    return .{
        .x = @sin(vehicle.yaw),
        .z = @cos(vehicle.yaw),
    };
}

/// Get right direction vector
pub fn getRightDir(vehicle: *const VehicleState) struct { x: f32, z: f32 } {
    return .{
        .x = @cos(vehicle.yaw),
        .z = -@sin(vehicle.yaw),
    };
}

/// Get wheel world positions
pub fn getWheelPositions(vehicle: *const VehicleState) [4]WheelWorldPosition {
    var positions: [4]WheelWorldPosition = undefined;
    const fwd = getForwardDir(vehicle);
    const right = getRightDir(vehicle);

    for (vehicle.wheels, 0..) |wheel, i| {
        const offset_x = @as(f32, @floatFromInt(wheel.offset_x));
        const offset_y = @as(f32, @floatFromInt(wheel.offset_y));
        const offset_z = @as(f32, @floatFromInt(wheel.offset_z));

        positions[i] = .{
            .x = vehicle.pos_x + fwd.x * offset_z + right.x * offset_x,
            .y = vehicle.pos_y + offset_y,
            .z = vehicle.pos_z + fwd.z * offset_z + right.z * offset_x,
        };
    }

    return positions;
}

fn getVehicleHalfExtents(vehicle: *const VehicleState) struct { x: f32, y: f32, z: f32 } {
    return switch (vehicle.vehicle_type) {
        .car, .truck, .motorcycle => .{ .x = 6.0, .y = 2.0, .z = 8.0 },
        .aircraft => .{ .x = 12.0, .y = 4.0, .z = 12.0 },
        .boat => .{ .x = 8.0, .y = 3.0, .z = 12.0 },
        .hovercraft => .{ .x = 7.0, .y = 2.0, .z = 7.0 },
    };
}

pub fn predictVehiclePose(vehicle: *const VehicleState, time_delta: f32) prediction.PlanarPoseForecast {
    const forward = getForwardDir(vehicle);
    const vel_x = forward.x * vehicle.speed;
    const vel_z = forward.z * vehicle.speed;
    return prediction.predictPlanarPose(.{
        .pos_x = vehicle.pos_x,
        .pos_y = vehicle.pos_y,
        .pos_z = vehicle.pos_z,
        .yaw = vehicle.yaw,
    }, vel_x, 0.0, vel_z, vehicle.angular_velocity, time_delta);
}

pub fn predictVehicleOccupancy(vehicle: *const VehicleState, time_delta: f32) prediction.FutureOccupancyAABB {
    const pose = predictVehiclePose(vehicle, time_delta);
    const extents = getVehicleHalfExtents(vehicle);
    return prediction.computeFutureOccupancyAABB(
        pose.pos_x,
        pose.pos_y + extents.y,
        pose.pos_z,
        extents.x,
        extents.y,
        extents.z,
    );
}

pub fn computeVehicleOccupancyConflict(a: *const VehicleState, b: *const VehicleState, horizon: f32, step: f32) prediction.OccupancyConflictWindow {
    const a_extents = getVehicleHalfExtents(a);
    const b_extents = getVehicleHalfExtents(b);
    return prediction.computeOccupancyConflictWindow(
        .{
            .pos_x = a.pos_x,
            .pos_y = a.pos_y + a_extents.y,
            .pos_z = a.pos_z,
            .yaw = a.yaw,
        },
        getForwardDir(a).x * a.speed,
        0.0,
        getForwardDir(a).z * a.speed,
        a.angular_velocity,
        a_extents.x,
        a_extents.y,
        a_extents.z,
        .{
            .pos_x = b.pos_x,
            .pos_y = b.pos_y + b_extents.y,
            .pos_z = b.pos_z,
            .yaw = b.yaw,
        },
        getForwardDir(b).x * b.speed,
        0.0,
        getForwardDir(b).z * b.speed,
        b.angular_velocity,
        b_extents.x,
        b_extents.y,
        b_extents.z,
        horizon,
        step,
    );
}

pub fn buildVehicleAvoidanceRecommendationFromConflict(conflict: prediction.OccupancyConflictWindow, horizon: f32, side_sign: f32) prediction.AvoidanceRecommendation {
    return prediction.buildAvoidanceRecommendation(conflict, horizon, side_sign, 0.75, 0.2, 0.5, 0.5);
}

pub fn buildVehicleAvoidanceRecommendation(a: *const VehicleState, b: *const VehicleState, horizon: f32, step: f32, side_sign: f32) prediction.AvoidanceRecommendation {
    const conflict = computeVehicleOccupancyConflict(a, b, horizon, step);
    return buildVehicleAvoidanceRecommendationFromConflict(conflict, horizon, side_sign);
}

pub fn applyVehicleSteeringBias(self: *const VehicleState, other: *const VehicleState, steering_bias: f32) f32 {
    if (@abs(steering_bias) <= 0.000001) return self.steering;

    const dx = other.pos_x - self.pos_x;
    const dz = other.pos_z - self.pos_z;
    const dist_sq = dx * dx + dz * dz;
    var normal_x: f32 = 1.0;
    var normal_z: f32 = 0.0;
    if (dist_sq > 0.000001) {
        const inv_dist = 1.0 / @sqrt(dist_sq);
        normal_x = dx * inv_dist;
        normal_z = dz * inv_dist;
    }
    const tangent_x = -normal_z;
    const tangent_z = normal_x;
    const right = getRightDir(self);
    const tangent_dot = tangent_x * right.x + tangent_z * right.z;
    return @max(-1.0, @min(1.0, self.steering + tangent_dot * steering_bias));
}

pub fn applyPredictiveVehicleAvoidance(dt: f32) void {
    const sys = getSystem();
    const horizon = @max(0.2, dt * 4.0);
    const step = @max(0.05, dt);
    var i: usize = 0;
    while (i < sys.count) : (i += 1) {
        var j: usize = i + 1;
        while (j < sys.count) : (j += 1) {
            const a = &sys.vehicles[i];
            const b = &sys.vehicles[j];
            const window = computeVehicleOccupancyConflict(a, b, horizon, step);
            const side_sign: f32 = if (@intFromPtr(a) < @intFromPtr(b)) 1.0 else -1.0;
            const recommendation = buildVehicleAvoidanceRecommendationFromConflict(window, horizon, side_sign);
            if (recommendation.conflict.valid) {
                a.steering = applyVehicleSteeringBias(a, b, recommendation.steering_bias);
                b.steering = applyVehicleSteeringBias(b, a, recommendation.steering_bias);
                if (recommendation.should_brake) {
                    a.brake = @max(a.brake, recommendation.brake_amount);
                    b.brake = @max(b.brake, recommendation.brake_amount);
                }
                a.throttle = @min(a.throttle, 0.0);
                b.throttle = @min(b.throttle, 0.0);
            }
        }
    }
}

/// Check if vehicle is grounded
pub fn checkGrounded(
    vehicle: *const VehicleState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    const check_y = @as(i32, @intFromFloat(@floor(vehicle.pos_y - 1)));
    const wheel_pos = getWheelPositions(vehicle);
    const world_view = query.QueryWorldView{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const filter = query.QueryFilter{
        .include_static = true,
        .include_dynamic = true,
        .include_kinematic = true,
        .include_sensors = false,
    };

    for (wheel_pos) |pos| {
        const wx = @as(i32, @intFromFloat(@floor(pos.x)));
        const wz = @as(i32, @intFromFloat(@floor(pos.z)));
        if (query.queryAnyVoxel(&world_view, wx, check_y, wz, filter).hit) {
            return true;
        }
    }
    return false;
}

/// Check if vehicle is flipped
pub fn checkFlipped(vehicle: *const VehicleState) bool {
    return @abs(vehicle.pitch) > 1.0 or @abs(vehicle.roll) > 1.0;
}

/// Update car physics
fn updateCar(vehicle: *VehicleState, dt: f32) void {
    const max_speed: f32 = 500.0;
    const acceleration: f32 = 200.0;
    const braking: f32 = 400.0;
    const max_steering: f32 = 0.05;
    const steering_speed: f32 = 2.0;
    const friction: f32 = 0.98;

    if (vehicle.grounded) {
        vehicle.speed *= friction;

        vehicle.speed += vehicle.throttle * acceleration * dt;

        if (vehicle.brake > 0) {
            if (vehicle.speed > 0) {
                vehicle.speed -= vehicle.brake * braking * dt;
                if (vehicle.speed < 0) vehicle.speed = 0;
            } else if (vehicle.speed < 0) {
                vehicle.speed += vehicle.brake * braking * dt;
                if (vehicle.speed > 0) vehicle.speed = 0;
            }
        }

        if (vehicle.handbrake) {
            vehicle.speed *= 0.9;
        }

        vehicle.speed = @max(-max_speed / 2, @min(max_speed, vehicle.speed));

        const steering_input = vehicle.steering * max_steering;
        const speed_factor = 1.0 - (@abs(vehicle.speed) / max_speed) * 0.5;
        vehicle.angular_velocity = steering_input * steering_speed * speed_factor;

        if (vehicle.speed < 0) {
            vehicle.angular_velocity = -vehicle.angular_velocity;
        }

        vehicle.yaw += vehicle.angular_velocity;

        if (vehicle.handbrake and @abs(vehicle.speed) > 10) {
            vehicle.roll += vehicle.steering * dt * 2.0;
        }
    } else {
        vehicle.speed *= 0.995;
    }
}

/// Update aircraft physics
fn updateAircraft(vehicle: *VehicleState, dt: f32) void {
    const drag: f32 = 0.01;
    const thrust: f32 = 1000.0;

    vehicle.speed += vehicle.throttle * thrust * dt;
    vehicle.speed *= (1.0 - drag);

    const fwd = getForwardDir(vehicle);
    vehicle.pos_x += fwd.x * vehicle.speed * dt;
    vehicle.pos_z += fwd.z * vehicle.speed * dt;

    vehicle.pitch += vehicle.steering * dt * 0.5;
    vehicle.pos_y += @sin(vehicle.pitch) * vehicle.speed * dt;

    if (vehicle.pos_y < 10) {
        vehicle.pos_y = 10;
        vehicle.grounded = true;
    } else {
        vehicle.grounded = false;
    }
}

/// Update boat physics
fn updateBoat(vehicle: *VehicleState, dt: f32) void {
    const water_drag: f32 = 0.98;
    const acceleration: f32 = 300.0;
    const turn_rate: f32 = 1.5;

    vehicle.speed *= water_drag;
    vehicle.speed += vehicle.throttle * acceleration * dt;
    vehicle.speed = @max(0, @min(300, vehicle.speed));

    vehicle.yaw += vehicle.steering * turn_rate * @abs(vehicle.speed) / 100.0 * dt;

    const fwd = getForwardDir(vehicle);
    vehicle.pos_x += fwd.x * vehicle.speed * dt;
    vehicle.pos_z += fwd.z * vehicle.speed * dt;
}

/// Update hovercraft physics
fn updateHovercraft(vehicle: *VehicleState, dt: f32) void {
    const acceleration: f32 = 400.0;
    const friction: f32 = 0.96;
    const turn_rate: f32 = 2.5;

    vehicle.speed *= friction;
    vehicle.speed += vehicle.throttle * acceleration * dt;
    vehicle.speed = @max(-200, @min(200, vehicle.speed));

    vehicle.yaw += vehicle.steering * turn_rate * dt;

    const fwd = getForwardDir(vehicle);
    vehicle.pos_x += fwd.x * vehicle.speed * dt;
    vehicle.pos_z += fwd.z * vehicle.speed * dt;

    vehicle.pos_y = 1;
    vehicle.grounded = true;
}

/// Main update function
pub fn update(
    vehicle: *VehicleState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    dt: f32,
) void {
    vehicle.grounded = checkGrounded(vehicle, s1024, entities);
    vehicle.flipped = checkFlipped(vehicle);

    switch (vehicle.vehicle_type) {
        .car, .truck, .motorcycle => updateCar(vehicle, dt),
        .aircraft => updateAircraft(vehicle, dt),
        .boat => updateBoat(vehicle, dt),
        .hovercraft => updateHovercraft(vehicle, dt),
    }
}

/// Apply collision response (centrifugal force)
pub fn applyCollisionResponse(
    vehicle: *VehicleState,
    normal_x: f32,
    normal_z: f32,
    impulse: f32,
) void {
    const speed_sq = vehicle.speed * vehicle.speed;
    const centrifugal = speed_sq * 0.001 * impulse;

    vehicle.pos_x += normal_x * impulse * 0.1;
    vehicle.pos_z += normal_z * impulse * 0.1;
    vehicle.speed *= 0.5;
    vehicle.angular_velocity += centrifugal;
}

/// Attempt to flip vehicle back upright
pub fn attemptFlip(vehicle: *VehicleState) bool {
    if (!vehicle.flipped) return true;

    vehicle.pitch *= 0.9;
    vehicle.roll *= 0.9;

    if (@abs(vehicle.pitch) < 0.1 and @abs(vehicle.roll) < 0.1) {
        vehicle.flipped = false;
        vehicle.pitch = 0;
        vehicle.roll = 0;
        return true;
    }
    return false;
}

/// Get system for external iteration
pub fn getSystem() *VehicleSystem {
    return &g_vehicle_system;
}

test "predictVehiclePose advances vehicle along forward speed and yaw rate" {
    const pose = predictVehiclePose(&.{
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 10,
        .yaw = std.math.pi / 2.0,
        .pitch = 0,
        .roll = 0,
        .speed = 5,
        .angular_velocity = 0.25,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = true,
        .flipped = false,
        .vehicle_type = .car,
        .wheels = undefined,
        .mass = 1500,
    }, 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), pose.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), pose.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, std.math.pi / 2.0 + 0.5), pose.yaw, 0.0001);
}

test "predictVehicleOccupancy builds future AABB around predicted pose" {
    const occupancy = predictVehicleOccupancy(&.{
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 10,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .speed = 5,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = true,
        .flipped = false,
        .vehicle_type = .car,
        .wheels = undefined,
        .mass = 1500,
    }, 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, -6.0), occupancy.min_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), occupancy.min_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), occupancy.min_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), occupancy.max_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), occupancy.max_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 28.0), occupancy.max_z, 0.0001);
}

test "computeVehicleOccupancyConflict detects future overlap window" {
    const window = computeVehicleOccupancyConflict(&.{
        .pos_x = -20,
        .pos_y = 0,
        .pos_z = 0,
        .yaw = std.math.pi / 2.0,
        .pitch = 0,
        .roll = 0,
        .speed = 10,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = true,
        .flipped = false,
        .vehicle_type = .car,
        .wheels = undefined,
        .mass = 1500,
    }, &.{
        .pos_x = 20,
        .pos_y = 0,
        .pos_z = 0,
        .yaw = -std.math.pi / 2.0,
        .pitch = 0,
        .roll = 0,
        .speed = 10,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = true,
        .flipped = false,
        .vehicle_type = .car,
        .wheels = undefined,
        .mass = 1500,
    }, 3.0, 0.25);
    try std.testing.expect(window.valid);
    try std.testing.expect(window.start_time >= 0.5 and window.start_time <= 1.5);
}

test "applyPredictiveVehicleAvoidance pre-brakes conflicting vehicles" {
    init();
    const a = createCar(-12.0, 0.0, 0.0, std.math.pi / 2.0) orelse return error.TestUnexpectedResult;
    const b = createCar(12.0, 0.0, 0.0, -std.math.pi / 2.0) orelse return error.TestUnexpectedResult;

    a.speed = 10.0;
    b.speed = 10.0;
    a.throttle = 1.0;
    b.throttle = 1.0;
    a.brake = 0.0;
    b.brake = 0.0;

    applyPredictiveVehicleAvoidance(0.25);

    try std.testing.expect(a.brake >= 0.5);
    try std.testing.expect(b.brake >= 0.5);
    try std.testing.expect(a.throttle == 0.0);
    try std.testing.expect(b.throttle == 0.0);
}

test "applyPredictiveVehicleAvoidance also adds steering bias for conflicting vehicles" {
    init();
    const a = createCar(-12.0, 0.0, 0.0, 0.0) orelse return error.TestUnexpectedResult;
    const b = createCar(-12.0, 0.0, 20.0, std.math.pi) orelse return error.TestUnexpectedResult;

    a.speed = 10.0;
    b.speed = 10.0;
    a.throttle = 1.0;
    b.throttle = 1.0;
    a.steering = 0.0;
    b.steering = 0.0;

    applyPredictiveVehicleAvoidance(0.25);

    try std.testing.expect(@abs(a.steering) > 0.01);
    try std.testing.expect(@abs(b.steering) > 0.01);
}
