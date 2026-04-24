//! Vehicle - Vehicle Physics System
//!
//! Phase 11: Ground vehicles, aircraft, watercraft, hovercraft
//! Handles: Driving, steering, drift, collision response, flipping

const std = @import("std");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");

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
pub fn getWheelPositions(vehicle: *const VehicleState) [4]struct { x: f32, y: f32, z: f32 } {
    var positions: [4]struct { x: f32, y: f32, z: f32 } = undefined;
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

/// Check if vehicle is grounded
pub fn checkGrounded(
    vehicle: *const VehicleState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    const check_y = vehicle.pos_y - 1;
    const wheel_pos = getWheelPositions(vehicle);

    for (wheel_pos) |pos| {
        const wx = @as(i32, @intFromFloat(@floor(pos.x)));
        const wz = @as(i32, @intFromFloat(@floor(pos.z)));
        if (physics.isOccupiedGlobal(s1024, undefined, entities, wx, check_y, wz, null)) {
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
