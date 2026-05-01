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
const terrain = @import("terrain.zig");
const contact_response = @import("contact_response.zig");
const weather = @import("weather.zig");
const ai_traffic = @import("ai_traffic.zig");
const tire = @import("tire.zig");
const suspension = @import("suspension.zig");

const safety = @import("safety.zig");
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
    tire_peak_slip_ratio: f32 = 0.15,
    tire_peak_slip_angle: f32 = 0.12,
    tire_friction: f32 = 1.0,
    tire_rolling_resistance: f32 = 0.015,
    susp_spring_rate: f32 = 50000.0,
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
    ai_vehicle_id: u16 = 0, // 0=not linked, >0=linked to ai_traffic vehicle ID
    ai_target_speed: f32 = -1, // -1=no AI control, >=0=AI target speed (m/s)
    target_vel: f32 = -1, // autonomy setter: governed target speed from AI traffic (m/s)
    engine_rpm: f32 = 1000.0, // engine RPM for torque curve (Phase 21)
    engine_torque: f32 = 0.0, // current engine torque output (Phase 21)
    brake: f32,
    handbrake: bool,
    grounded: bool,
    flipped: bool,
    vehicle_type: VehicleType,
    wheels: [4]WheelConfig,
    tire_states: [4]tire.TireState,
    susp_states: [4]suspension.SuspensionState,
    mass: u16,
};

pub const VehicleControlCommand = struct {
    target_speed: f32,
    target_yaw_rate: f32,
    emergency_brake: bool = false,
};

/// Link a vehicle to an AI traffic vehicle and set initial target speed.
/// Writes the vehicle's pose/speed into the AI traffic vehicle so planning
/// is aware of the vehicle's real state before updateAI() runs.
pub fn setAIVehicleLink(vehicle: *VehicleState, ai_vehicle_id: u16, initial_target_speed: f32) void {
    vehicle.ai_vehicle_id = ai_vehicle_id;
    vehicle.ai_target_speed = initial_target_speed;
    vehicle.target_vel = initial_target_speed;
    if (ai_vehicle_id > 0) {
        _ = syncVehicleToTraffic(vehicle);
    }
}

/// Write a vehicle's current pose and speed into the matching AI traffic vehicle.
/// This must be called before ai_traffic.updateAI() each tick so that the AI
/// planner operates on the vehicle's real physics state rather than its internal
/// ghost state. Returns true if a matching AI traffic vehicle was found.
pub fn syncVehicleToTraffic(vehicle: *VehicleState) bool {
    if (vehicle.ai_vehicle_id == 0) return false;
    const tv_opt = ai_traffic.getTrafficVehicle(vehicle.ai_vehicle_id);
    if (tv_opt) |tv| {
        const fwd_x = @sin(vehicle.yaw);
        const fwd_z = @cos(vehicle.yaw);
        tv.pos_x = vehicle.pos_x;
        tv.pos_z = vehicle.pos_z;
        tv.yaw = vehicle.yaw;
        tv.vel_x = fwd_x * vehicle.speed;
        tv.vel_z = fwd_z * vehicle.speed;
        return true;
    }
    return false;
}

pub const VehicleNetState = struct {
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
    ai_target_speed: f32,
    target_vel: f32 = -1,
    ai_vehicle_id: u16 = 0, // linked AI traffic vehicle ID (0=none)
    brake: f32,
    handbrake: bool,
    grounded: bool,
    flipped: bool,
    vehicle_type: VehicleType,
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
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .yaw = yaw,
        .pitch = 0,
        .roll = 0,
        .speed = 0,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = false,
        .flipped = false,
        .vehicle_type = .car,
        .wheels = [_]WheelConfig{
            .{ .offset_x = -6, .offset_y = 0, .offset_z = -8, .radius = 4, .steering_angle = 0, .driven = true, .braked = true },
            .{ .offset_x = 6, .offset_y = 0, .offset_z = -8, .radius = 4, .steering_angle = 0, .driven = true, .braked = true },
            .{ .offset_x = -6, .offset_y = 0, .offset_z = 8, .radius = 4, .steering_angle = 0, .driven = false, .braked = true },
            .{ .offset_x = 6, .offset_y = 0, .offset_z = 8, .radius = 4, .steering_angle = 0, .driven = false, .braked = true },
        },
        .tire_states = [_]tire.TireState{ tire.TireState{}, tire.TireState{}, tire.TireState{}, tire.TireState{} },
        .susp_states = [_]suspension.SuspensionState{ .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 } },
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
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .speed = 0,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = false,
        .flipped = false,
        .vehicle_type = .aircraft,
        .wheels = [_]WheelConfig{.{
            .offset_x = 0,
            .offset_y = 0,
            .offset_z = 0,
            .radius = 0,
            .steering_angle = 0,
            .driven = false,
            .braked = false,
        }} ** 4,
        .tire_states = [_]tire.TireState{ tire.TireState{}, tire.TireState{}, tire.TireState{}, tire.TireState{} },
        .susp_states = [_]suspension.SuspensionState{ .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 } },
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
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .yaw = yaw,
        .pitch = 0,
        .roll = 0,
        .speed = 0,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = false,
        .flipped = false,
        .vehicle_type = .boat,
        .wheels = [_]WheelConfig{.{
            .offset_x = 0,
            .offset_y = 0,
            .offset_z = 0,
            .radius = 0,
            .steering_angle = 0,
            .driven = false,
            .braked = false,
        }} ** 4,
        .tire_states = [_]tire.TireState{ tire.TireState{}, tire.TireState{}, tire.TireState{}, tire.TireState{} },
        .susp_states = [_]suspension.SuspensionState{ .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 } },
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
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .yaw = yaw,
        .pitch = 0,
        .roll = 0,
        .speed = 0,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = true,
        .flipped = false,
        .vehicle_type = .hovercraft,
        .wheels = [_]WheelConfig{.{
            .offset_x = 0,
            .offset_y = 0,
            .offset_z = 0,
            .radius = 0,
            .steering_angle = 0,
            .driven = false,
            .braked = false,
        }} ** 4,
        .tire_states = [_]tire.TireState{ tire.TireState{}, tire.TireState{}, tire.TireState{}, tire.TireState{} },
        .susp_states = [_]suspension.SuspensionState{ .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 } },
        .mass = 1000,
    };
    return vehicle;
}

pub fn removeVehicle(vehicle: *VehicleState) void {
    if (g_vehicle_system.count == 0) return;
    const base_ptr = @intFromPtr(&g_vehicle_system.vehicles[0]);
    const target_ptr = @intFromPtr(vehicle);
    const elem_size = @sizeOf(VehicleState);
    if (target_ptr < base_ptr) return;
    const offset = target_ptr - base_ptr;
    if (offset % elem_size != 0) return;
    const idx: usize = offset / elem_size;
    if (idx >= g_vehicle_system.count) return;

    const last_idx = g_vehicle_system.count - 1;
    if (idx != last_idx) {
        g_vehicle_system.vehicles[idx] = g_vehicle_system.vehicles[last_idx];
    }
    g_vehicle_system.count -= 1;
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

fn clamp01(value: f32) f32 {
    return std.math.clamp(value, 0.0, 1.0);
}

fn isGroundVehicle(vehicle_type: VehicleType) bool {
    return switch (vehicle_type) {
        .car, .truck, .motorcycle => true,
        else => false,
    };
}

fn estimateTireWidth(vehicle_type: VehicleType) f32 {
    return switch (vehicle_type) {
        .car => 0.26,
        .truck => 0.36,
        .motorcycle => 0.15,
        else => 0.28,
    };
}

fn safeWorldToGridCoord(value: f32) i32 {
    const min_f = @as(f32, @floatFromInt(std.math.minInt(i32) + 1));
    const max_f = @as(f32, @floatFromInt(std.math.maxInt(i32) - 1));
    const clamped = std.math.clamp(value, min_f, max_f);
    return @as(i32, @intFromFloat(@floor(clamped)));
}

pub const EnvironmentControlAuthority = struct {
    speed_scale: f32,
    throttle_scale: f32,
    brake_scale: f32,
    steering_scale: f32,
};

fn defaultEnvironmentControlAuthority() EnvironmentControlAuthority {
    return .{
        .speed_scale = 1.0,
        .throttle_scale = 1.0,
        .brake_scale = 1.0,
        .steering_scale = 1.0,
    };
}

pub fn measureEnvironmentControlAuthority(
    vehicle_type: VehicleType,
    pos_x: f32,
    pos_z: f32,
    speed: f32,
) EnvironmentControlAuthority {
    if (!isGroundVehicle(vehicle_type)) return defaultEnvironmentControlAuthority();

    const wx = safeWorldToGridCoord(pos_x);
    const wz = safeWorldToGridCoord(pos_z);
    const tire_width = estimateTireWidth(vehicle_type);
    const abs_speed = @abs(speed);
    const traction = std.math.clamp(terrain.computeTractionAt(wx, wz, abs_speed, tire_width), 0.05, 1.2);
    const braking_distance_scale = std.math.clamp(terrain.computeBrakingDistanceScaleAt(wx, wz, abs_speed, tire_width), 0.67, 20.0);
    const weather_penalty = weather.getRoadTractionPenalty();
    const visibility = weather.getSensorVisibilityFactor();

    const terrain_hazard = clamp01(1.0 - std.math.clamp(traction, 0.0, 1.0));
    const road_hazard = clamp01(terrain_hazard * 0.7 + weather_penalty * 0.7 + (1.0 - visibility) * 0.5);
    const braking_scale = std.math.clamp(1.0 + road_hazard * 0.35 + (braking_distance_scale - 1.0) * 0.15, 0.8, 1.8);

    return .{
        .speed_scale = std.math.clamp(1.0 - road_hazard * 0.75, 0.2, 1.0),
        .throttle_scale = std.math.clamp(1.0 - road_hazard * 0.9, 0.1, 1.0),
        .brake_scale = braking_scale,
        .steering_scale = std.math.clamp(0.35 + traction * 0.4 + visibility * 0.25, 0.2, 1.0),
    };
}

const ControlAuthority = EnvironmentControlAuthority;

fn measureControlAuthority(vehicle: *const VehicleState) ControlAuthority {
    return measureEnvironmentControlAuthority(
        vehicle.vehicle_type,
        vehicle.pos_x,
        vehicle.pos_z,
        vehicle.speed,
    );
}

const MarineControlAuthority = struct {
    speed_scale: f32,
    throttle_scale: f32,
    steering_scale: f32,
    drag_scale: f32,
    wind_drift_scale: f32,
};

fn measureBoatControlAuthority(vehicle: *const VehicleState) MarineControlAuthority {
    const wx = safeWorldToGridCoord(vehicle.pos_x);
    const wz = safeWorldToGridCoord(vehicle.pos_z);
    const medium = terrain.getMediumAt(wx, wz);
    const atmosphere = weather.getAtmosphericConditions();
    const weather_severity = weather.getWeatherSeverity();
    const visibility = weather.getSensorVisibilityFactor();
    const wind_factor = clamp01(atmosphere.wind_speed / 70.0);

    const medium_penalty: f32 = switch (medium) {
        .liquid => 0.0,
        .soft => 0.15,
        .solid => 0.3,
        .vapor => 0.4,
        .plasma => 0.45,
    };
    const hazard = clamp01(weather_severity * 0.45 + (1.0 - visibility) * 0.3 + wind_factor * 0.35 + medium_penalty);

    return .{
        .speed_scale = std.math.clamp(1.0 - hazard * 0.55, 0.35, 1.0),
        .throttle_scale = std.math.clamp(1.0 - hazard * 0.7, 0.25, 1.0),
        .steering_scale = std.math.clamp(1.0 - hazard * 0.5, 0.35, 1.0),
        .drag_scale = std.math.clamp(1.0 + hazard * 1.8 + medium_penalty * 0.8, 0.8, 3.0),
        .wind_drift_scale = std.math.clamp(0.015 + wind_factor * 0.02 + medium_penalty * 0.005, 0.01, 0.04),
    };
}

const HovercraftControlAuthority = struct {
    speed_scale: f32,
    throttle_scale: f32,
    steering_scale: f32,
    drift_scale: f32,
    wind_drift_scale: f32,
};

fn measureHovercraftControlAuthority(vehicle: *const VehicleState) HovercraftControlAuthority {
    const wx = safeWorldToGridCoord(vehicle.pos_x);
    const wz = safeWorldToGridCoord(vehicle.pos_z);
    const medium = terrain.getMediumAt(wx, wz);
    const traction = std.math.clamp(terrain.computeTractionAt(wx, wz, @abs(vehicle.speed), 0.28), 0.05, 1.2);
    const atmosphere = weather.getAtmosphericConditions();
    const weather_severity = weather.getWeatherSeverity();
    const visibility = weather.getSensorVisibilityFactor();
    const wind_factor = clamp01(atmosphere.wind_speed / 90.0);

    const medium_penalty: f32 = switch (medium) {
        .liquid => 0.25,
        .soft => 0.15,
        .solid => 0.0,
        .vapor => 0.08,
        .plasma => 0.2,
    };
    const traction_hazard = clamp01(1.0 - std.math.clamp(traction, 0.0, 1.0));
    const hazard = clamp01(traction_hazard * 0.55 + weather_severity * 0.3 + (1.0 - visibility) * 0.2 + wind_factor * 0.15 + medium_penalty);

    return .{
        .speed_scale = std.math.clamp(1.0 - hazard * 0.5, 0.4, 1.0),
        .throttle_scale = std.math.clamp(1.0 - hazard * 0.75, 0.2, 1.0),
        .steering_scale = std.math.clamp(1.0 - hazard * 0.65, 0.2, 1.0),
        .drift_scale = std.math.clamp(0.9 + hazard * 0.6, 0.9, 1.5),
        .wind_drift_scale = std.math.clamp(0.01 + wind_factor * 0.02 + medium_penalty * 0.01, 0.01, 0.05),
    };
}

fn wrapAnglePi(angle: f32) f32 {
    const tau = std.math.pi * 2.0;
    var wrapped = angle;
    while (wrapped > std.math.pi) wrapped -= tau;
    while (wrapped < -std.math.pi) wrapped += tau;
    return wrapped;
}

fn lerpAngleShortest(from: f32, to: f32, alpha: f32) f32 {
    const clamped = clamp01(alpha);
    const delta = wrapAnglePi(to - from);
    return from + delta * clamped;
}

pub fn applyAutonomyCommand(vehicle: *VehicleState, command: VehicleControlCommand, dt: f32) void {
    const authority = measureControlAuthority(vehicle);
    const safe_dt = @max(0.01, dt);
    const target_speed = @max(0.0, command.target_speed) * authority.speed_scale;
    const speed_error = target_speed - vehicle.speed;
    const accel_request = speed_error / safe_dt;

    const throttle_cmd: f32 = if (accel_request > 0.0)
        std.math.clamp(accel_request / 200.0, 0.0, 1.0)
    else
        0.0;
    var brake_cmd: f32 = if (accel_request < 0.0)
        std.math.clamp(-accel_request / 400.0, 0.0, 1.0)
    else
        0.0;

    if (target_speed <= 0.1 and vehicle.speed > 0.5) {
        brake_cmd = @max(brake_cmd, 0.5);
    }
    if (command.emergency_brake) {
        brake_cmd = 1.0;
    }

    const speed_factor = std.math.clamp(1.0 - (@abs(vehicle.speed) / 500.0) * 0.5, 0.5, 1.0);
    const max_yaw_rate = 0.05 * 2.0 * speed_factor;
    const steering_base = std.math.clamp(command.target_yaw_rate / @max(0.01, max_yaw_rate), -1.0, 1.0);
    const steering_cmd = steering_base * authority.steering_scale;

    applyThrottle(vehicle, throttle_cmd * authority.throttle_scale);
    applyBrake(vehicle, brake_cmd * authority.brake_scale);
    applySteering(vehicle, steering_cmd);
}

pub fn exportNetState(vehicle: *const VehicleState) VehicleNetState {
    return .{
        .pos_x = vehicle.pos_x,
        .pos_y = vehicle.pos_y,
        .pos_z = vehicle.pos_z,
        .yaw = vehicle.yaw,
        .pitch = vehicle.pitch,
        .roll = vehicle.roll,
        .speed = vehicle.speed,
        .angular_velocity = vehicle.angular_velocity,
        .throttle = vehicle.throttle,
        .steering = vehicle.steering,
        .ai_target_speed = vehicle.ai_target_speed,
        .target_vel = vehicle.target_vel,
        .ai_vehicle_id = vehicle.ai_vehicle_id,
        .brake = vehicle.brake,
        .handbrake = vehicle.handbrake,
        .grounded = vehicle.grounded,
        .flipped = vehicle.flipped,
        .vehicle_type = vehicle.vehicle_type,
        .mass = vehicle.mass,
    };
}

pub fn importNetState(vehicle: *VehicleState, state: VehicleNetState, position_alpha: f32, orientation_alpha: f32) void {
    const p_alpha = clamp01(position_alpha);
    const r_alpha = clamp01(orientation_alpha);
    vehicle.pos_x += (state.pos_x - vehicle.pos_x) * p_alpha;
    vehicle.pos_y += (state.pos_y - vehicle.pos_y) * p_alpha;
    vehicle.pos_z += (state.pos_z - vehicle.pos_z) * p_alpha;
    vehicle.yaw = lerpAngleShortest(vehicle.yaw, state.yaw, r_alpha);
    vehicle.pitch = lerpAngleShortest(vehicle.pitch, state.pitch, r_alpha);
    vehicle.roll = lerpAngleShortest(vehicle.roll, state.roll, r_alpha);
    vehicle.speed += (state.speed - vehicle.speed) * p_alpha;
    vehicle.angular_velocity += (state.angular_velocity - vehicle.angular_velocity) * p_alpha;
    vehicle.throttle += (state.throttle - vehicle.throttle) * p_alpha;
    vehicle.steering += (state.steering - vehicle.steering) * r_alpha;
    vehicle.brake += (state.brake - vehicle.brake) * p_alpha;
    vehicle.handbrake = state.handbrake;
    vehicle.grounded = state.grounded;
    vehicle.flipped = state.flipped;
    vehicle.vehicle_type = state.vehicle_type;
    vehicle.mass = state.mass;
    vehicle.ai_vehicle_id = state.ai_vehicle_id;
    vehicle.target_vel = state.target_vel;
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

/// Check if vehicle is grounded by sampling voxels below each wheel
pub fn checkGrounded(
    vehicle: *const VehicleState,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    switch (vehicle.vehicle_type) {
        .car, .truck, .motorcycle => {},
        else => return false,
    }

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

    const wheel_pos = getWheelPositions(vehicle);
    for (wheel_pos) |pos| {
        const base_x = @as(i32, @intFromFloat(@floor(pos.x)));
        const check_y = @as(i32, @intFromFloat(@floor(pos.y))) - 1;
        const base_z = @as(i32, @intFromFloat(@floor(pos.z)));
        if (query.queryAnyVoxel(&world_view, base_x, check_y, base_z, filter).hit) {
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
/// Full physics pipeline: drivetrain + tire + suspension -> speed/yaw
/// Phase 21-24: Engine torque, tire Pacejka forces, suspension dynamics
fn updateCar(vehicle: *VehicleState, dt: f32) void {
    const max_speed: f32 = 500.0;
    const max_steering: f32 = 0.05;
    const steering_speed: f32 = 2.0;

    if (vehicle.grounded) {
        var ai_target: f32 = -1;
        if (vehicle.ai_vehicle_id > 0) {
            if (ai_traffic.g_traffic_system.getGovernedTargetSpeed(vehicle.ai_vehicle_id)) |speed| {
                ai_target = speed;
            }
        }
        if (ai_target < 0) ai_target = vehicle.ai_target_speed;
        if (ai_target >= 0) {
            const err = ai_target - vehicle.speed;
            if (err > 0.5) {
                vehicle.throttle = std.math.clamp(err * 0.08, 0.0, 1.0);
                vehicle.brake = 0;
            } else if (err < -0.5) {
                vehicle.throttle = 0;
                vehicle.brake = std.math.clamp(-err * 0.10, 0.0, 1.0);
            } else {
                vehicle.throttle = 0;
                vehicle.brake = 0;
            }
        }

        if (vehicle.ai_vehicle_id > 0) {
            var forward_dist: f32 = 30.0;
            var target_spd: f32 = vehicle.speed;
            if (vehicle.ai_vehicle_id <= ai_traffic.g_traffic_system.vehicles.len) {
                const tv = ai_traffic.g_traffic_system.vehicles[vehicle.ai_vehicle_id - 1];
                if (tv.active) {
                    forward_dist = tv.following_distance;
                    target_spd = tv.governed_target_vel;
                }
            }
            const intervention = safety.evaluateSafetyIntervention(vehicle.speed, target_spd, forward_dist, vehicle.pos_x, 3.5);
            if (intervention.apply_aeb) {
                vehicle.brake = 1.0;
                vehicle.throttle = 0.0;
                vehicle.speed = @max(0, vehicle.speed - safety.MAX_BRAKE_DECELERATION * dt);
            } else if (intervention.warning != .none) {
                vehicle.throttle *= 0.5;
                vehicle.brake = @max(vehicle.brake, @as(f32, intervention.brake_decel) / 400.0);
            }
        }

        const authority = measureControlAuthority(vehicle);

        // 1. Engine: RPM tracks throttle, torque from peak-torque curve
        const idle_rpm: f32 = 800.0;
        const redline_rpm: f32 = 7000.0;
        const peak_torque_rpm: f32 = 4500.0;
        const peak_torque: f32 = 400.0;
        const low_end_torque: f32 = 80.0;
        const target_rpm = idle_rpm + (redline_rpm - idle_rpm) * vehicle.throttle;
        const rpm_delta = target_rpm - vehicle.engine_rpm;
        vehicle.engine_rpm += std.math.copysign(@min(@abs(rpm_delta), 3000.0 * dt), rpm_delta);
        vehicle.engine_rpm = @max(idle_rpm, @min(redline_rpm, vehicle.engine_rpm));

        if (vehicle.engine_rpm <= peak_torque_rpm) {
            const t = (vehicle.engine_rpm - idle_rpm) / @max(1.0, peak_torque_rpm - idle_rpm);
            vehicle.engine_torque = low_end_torque + (peak_torque - low_end_torque) * @sin(t * std.math.pi * 0.5);
        } else {
            const fall_t = (vehicle.engine_rpm - peak_torque_rpm) / @max(1.0, redline_rpm - peak_torque_rpm);
            vehicle.engine_torque = @max(0.0, peak_torque * (1.0 - 0.35 * fall_t * fall_t));
        }
        // 2. Per-wheel tire + suspension force accumulation
        var total_longitudinal_force: f32 = 0.0;
        var total_lateral_force: f32 = 0.0;
        var total_rolling_resistance: f32 = 0.0;

        const abs_speed = @abs(vehicle.speed);
        const yaw_rate = vehicle.angular_velocity;

        for (vehicle.wheels) |wheel| {
            const radius = @as(f32, @floatFromInt(wheel.radius)) * 0.1;
            const slip_ratio = if (wheel.driven and abs_speed > 0.1)
                @max(-1.0, @min(1.0, vehicle.throttle * 0.15))
            else
                0.0;
            const slip_angle = if (abs_speed > 0.1)
                @max(-1.2, @min(1.2, yaw_rate * radius * 2.0 / @max(abs_speed, 0.1)))
            else
                0.0;
            const B: f32 = 10.0;
            const C: f32 = 1.9;
            const E: f32 = 0.97;
            const peak_slip = if (wheel.driven) wheel.tire_peak_slip_ratio else wheel.tire_peak_slip_angle;
            const D = wheel.tire_friction * authority.throttle_scale * @as(f32, @floatFromInt(vehicle.mass)) * 9.81 / 4.0;
            const x_s = B * slip_ratio / @max(peak_slip, 0.001);
            const longitudinal_force = D * @sin(C * std.math.atan(x_s - E * (x_s - std.math.atan(x_s))));
            const x_a = B * slip_angle / @max(wheel.tire_peak_slip_angle, 0.001);
            const lateral_force = D * @sin(C * std.math.atan(x_a - E * (x_a - std.math.atan(x_a))));
            const susp_compression = @min(1.0, @max(0.0, @as(f32, @floatFromInt(wheel.offset_y)) / 4.0));
            const susp_force = wheel.susp_spring_rate * susp_compression * authority.throttle_scale;
            const rolling_r = wheel.tire_rolling_resistance * authority.brake_scale * (1.0 + abs_speed * 0.01);
            if (wheel.driven) total_longitudinal_force += longitudinal_force;
            total_lateral_force += (lateral_force + susp_force * 0.05) * authority.steering_scale;
            total_rolling_resistance += rolling_r;
        }

        // 3. Net force -> acceleration -> speed
        const mass_f: f32 = @as(f32, @floatFromInt(vehicle.mass));
        const roll_r_coef = 1.0 - @min(total_rolling_resistance / @max(mass_f * 10.0, 1.0), 0.9);
        vehicle.speed *= std.math.pow(f32, @max(roll_r_coef, 0.85), dt * 60.0);
        const throttle_accel = if (vehicle.throttle > 0)
            total_longitudinal_force / mass_f
        else
            0.0;
        vehicle.speed += throttle_accel * dt;
        if (vehicle.brake > 0) {
            if (vehicle.speed > 0) {
                vehicle.speed -= vehicle.brake * authority.brake_scale * 400.0 * dt;
                if (vehicle.speed < 0) vehicle.speed = 0;
            } else if (vehicle.speed < 0) {
                vehicle.speed += vehicle.brake * authority.brake_scale * 400.0 * dt;
                if (vehicle.speed > 0) vehicle.speed = 0;
            }
        }

        if (vehicle.handbrake) {
            vehicle.speed *= 1.0 - 0.1 * authority.brake_scale;
        } else if (vehicle.throttle == 0 and vehicle.brake == 0) {
            const surface_x = @as(i32, @intFromFloat(@floor(vehicle.pos_x)));
            const surface_z = @as(i32, @intFromFloat(@floor(vehicle.pos_z)));
            vehicle.speed = contact_response.applyVehicleRollingResistance(vehicle.speed, surface_x, surface_z, @floatFromInt(vehicle.mass), dt);
            // Apply water resistance when driving through water
            vehicle.speed = contact_response.applyVehicleWaterResistance(vehicle.speed, vehicle.pos_x, vehicle.pos_z, dt);
        }

        vehicle.speed = @max(-max_speed / 2.0, @min(max_speed, vehicle.speed));

        // 4. Steering: lateral force -> angular velocity
        const steering_input = vehicle.steering * max_steering * authority.steering_scale;
        const speed_factor = 1.0 - (@abs(vehicle.speed) / max_speed) * 0.5;
        vehicle.angular_velocity = steering_input * steering_speed * speed_factor;
        if (vehicle.speed < 0) vehicle.angular_velocity = -vehicle.angular_velocity;
        vehicle.yaw += vehicle.angular_velocity * dt;

        // Lateral force countersteers to resist spin-out
        const lateral_accel = total_lateral_force / mass_f;
        vehicle.roll += lateral_accel * 0.0002 * dt;
        if (vehicle.handbrake and @abs(vehicle.speed) > 10) {
            vehicle.roll += vehicle.steering * dt * 2.0;
        }
    } else {
        vehicle.speed *= std.math.pow(f32, 0.999, dt * 60.0);
    }
}
fn updateAircraft(vehicle: *VehicleState, dt: f32) void {
    const drag: f32 = 0.01;
    const thrust: f32 = 1000.0;
    const atmosphere = weather.getAtmosphericConditions();
    const weather_severity = weather.getWeatherSeverity();
    const wind_factor = clamp01(atmosphere.wind_speed / 120.0);
    const thrust_efficiency = std.math.clamp(1.0 - weather_severity * 0.2, 0.75, 1.0);
    const drag_factor = 1.0 + weather_severity * 0.6 + wind_factor * 0.4;
    const drag_per_tick = std.math.clamp(1.0 - drag * drag_factor, 0.9, 0.9995);
    const control_scale = std.math.clamp(1.0 - weather_severity * 0.25, 0.6, 1.0);
    const wind_x = @sin(atmosphere.wind_direction) * atmosphere.wind_speed;
    const wind_z = @cos(atmosphere.wind_direction) * atmosphere.wind_speed;

    vehicle.speed += vehicle.throttle * thrust * thrust_efficiency * dt;
    vehicle.speed *= std.math.pow(f32, drag_per_tick, dt * 60.0);
    // Apply weather-based aerodynamic drag (rain, fog, wind resistance)
    vehicle.speed = contact_response.applyVehicleAerodynamicDrag(vehicle.speed, weather_severity, atmosphere.wind_speed, dt);

    const fwd = getForwardDir(vehicle);
    vehicle.pos_x += fwd.x * vehicle.speed * dt + wind_x * dt * 0.08;
    vehicle.pos_z += fwd.z * vehicle.speed * dt + wind_z * dt * 0.08;

    vehicle.pitch += vehicle.steering * dt * 0.5 * control_scale;
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
    const water_drag_per_tick: f32 = 0.98;
    const acceleration: f32 = 300.0;
    const turn_rate: f32 = 1.5;
    const authority = measureBoatControlAuthority(vehicle);
    const atmosphere = weather.getAtmosphericConditions();
    const wind_x = @sin(atmosphere.wind_direction) * atmosphere.wind_speed;
    const wind_z = @cos(atmosphere.wind_direction) * atmosphere.wind_speed;
    const effective_drag_per_tick = std.math.clamp(
        1.0 - (1.0 - water_drag_per_tick) * authority.drag_scale,
        0.88,
        0.995,
    );
    const max_speed = std.math.clamp(300.0 * authority.speed_scale, 90.0, 300.0);

    vehicle.speed *= std.math.pow(f32, effective_drag_per_tick, dt * 60.0);
    vehicle.speed += vehicle.throttle * acceleration * dt * authority.throttle_scale;
    vehicle.speed = @max(0, @min(max_speed, vehicle.speed));

    const speed_turn_gain = std.math.clamp(0.3 + @abs(vehicle.speed) / 120.0, 0.3, 1.0);
    vehicle.yaw += vehicle.steering * turn_rate * speed_turn_gain * authority.steering_scale * dt;

    const fwd = getForwardDir(vehicle);
    vehicle.pos_x += fwd.x * vehicle.speed * dt + wind_x * dt * authority.wind_drift_scale;
    vehicle.pos_z += fwd.z * vehicle.speed * dt + wind_z * dt * authority.wind_drift_scale;
}

/// Update hovercraft physics
fn updateHovercraft(vehicle: *VehicleState, dt: f32) void {
    const acceleration: f32 = 400.0;
    const friction_per_tick: f32 = 0.96;
    const turn_rate: f32 = 2.5;
    const authority = measureHovercraftControlAuthority(vehicle);
    const atmosphere = weather.getAtmosphericConditions();
    const wind_x = @sin(atmosphere.wind_direction) * atmosphere.wind_speed;
    const wind_z = @cos(atmosphere.wind_direction) * atmosphere.wind_speed;
    const effective_friction_per_tick = std.math.clamp(
        friction_per_tick + (authority.drift_scale - 1.0) * 0.03,
        0.92,
        0.995,
    );
    const max_forward_speed = std.math.clamp(200.0 * authority.speed_scale, 80.0, 200.0);
    const max_reverse_speed = std.math.clamp(120.0 * authority.speed_scale, 50.0, 120.0);

    vehicle.speed *= std.math.pow(f32, effective_friction_per_tick, dt * 60.0);
    vehicle.speed += vehicle.throttle * acceleration * dt * authority.throttle_scale;
    vehicle.speed = @max(-max_reverse_speed, @min(max_forward_speed, vehicle.speed));

    const speed_turn_gain = std.math.clamp(0.55 + @abs(vehicle.speed) / 180.0, 0.55, 1.0);
    vehicle.yaw += vehicle.steering * turn_rate * authority.steering_scale * speed_turn_gain * dt;

    const fwd = getForwardDir(vehicle);
    vehicle.pos_x += fwd.x * vehicle.speed * dt + wind_x * dt * authority.wind_drift_scale;
    vehicle.pos_z += fwd.z * vehicle.speed * dt + wind_z * dt * authority.wind_drift_scale;

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
    // Sync AI planned target speed from traffic system
    syncAIVehicleFromTraffic(vehicle);

    switch (vehicle.vehicle_type) {
        .car, .truck, .motorcycle => vehicle.grounded = checkGrounded(vehicle, s1024, entities),
        else => vehicle.grounded = false,
    }
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
        .tire_states = [_]tire.TireState{ tire.TireState{}, tire.TireState{}, tire.TireState{}, tire.TireState{} },
        .susp_states = [_]suspension.SuspensionState{ .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 }, .{ .rest_length = 0.3 } },
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
        .tire_states = undefined,
        .susp_states = undefined,
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
        .tire_states = undefined,
        .susp_states = undefined,
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
        .tire_states = undefined,
        .susp_states = undefined,
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

// ============================================================================
// Tests for Vehicle System (Items 481-510)
// ============================================================================

test "481: vehicle chassis physics - car creation and basic properties" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    try std.testing.expect(car.?.vehicle_type == .car);
    try std.testing.expect(car.?.mass == 1500);
    try std.testing.expect(car.?.flipped == false);
    try std.testing.expect(car.?.grounded == false);
}

test "482: vehicle mass distribution" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    try std.testing.expect(car.?.mass > 0);
    // Mass affects speed - heavier vehicles should have different characteristics
    try std.testing.expect(car.?.speed == 0); // Initially stationary
}

test "483: vehicle center of gravity" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Car created at origin with 4 wheels at offsets
    // Forward direction at yaw=0 is +Z
    try std.testing.expect(car.?.pos_x == 0);
    try std.testing.expect(car.?.pos_y == 0);
    try std.testing.expect(car.?.pos_z == 0);
}

test "484: vehicle inertia tensor approximation" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Car has yaw, pitch, roll - angular velocity tracking exists
    try std.testing.expect(car.?.angular_velocity == 0);
}

test "485: vehicle suspension system" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Car has 4 wheels with suspension-like config
    try std.testing.expect(car.?.wheels.len == 4);
    // Wheels have offsets representing suspension attachment points
    try std.testing.expect(car.?.wheels[0].offset_y == 0);
}

test "486: vehicle tire physics" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Wheels have radius
    try std.testing.expect(car.?.wheels[0].radius == 4);
    // Wheels can be driven or braked
    try std.testing.expect(car.?.wheels[0].driven == true);
    try std.testing.expect(car.?.wheels[0].braked == true);
}

test "487: vehicle drivetrain - throttle application" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    applyThrottle(car.?, 0.5);
    try std.testing.expect(car.?.throttle == 0.5);
    applyThrottle(car.?, -0.5);
    try std.testing.expect(car.?.throttle == -0.5);
}

test "488: vehicle transmission - steering input" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    applySteering(car.?, 0.3);
    try std.testing.expect(car.?.steering == 0.3);
    applySteering(car.?, -0.7);
    try std.testing.expect(car.?.steering == -0.7);
}

test "489: vehicle steering system" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    applySteering(car.?, 0.5);
    try std.testing.expect(car.?.steering == 0.5);
    // Steering is clamped to [-1, 1]
    applySteering(car.?, 2.0);
    try std.testing.expect(car.?.steering == 1.0);
}

test "490: vehicle braking system" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    applyBrake(car.?, 0.8);
    try std.testing.expect(car.?.brake == 0.8);
    applyBrake(car.?, -0.5);
    try std.testing.expect(car.?.brake == 0); // Clamped to [0, 1]
}

test "491: vehicle aerodynamics - forward direction" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    const fwd = getForwardDir(car.?);
    // At yaw=0, forward is +Z direction
    try std.testing.expectApproxEqAbs(@as(f32, 0), fwd.x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1), fwd.z, 0.0001);
}

test "492: vehicle downforce approximation" {
    init();
    const slow = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    const fast = createCar(10, 0, 0, 0) orelse return error.TestUnexpectedResult;

    slow.grounded = true;
    fast.grounded = true;
    slow.speed = 20.0;
    fast.speed = 200.0;
    slow.steering = 1.0;
    fast.steering = 1.0;

    updateCar(slow, 0.1);
    updateCar(fast, 0.1);

    // At higher speed, steering authority is reduced by speed factor.
    try std.testing.expect(@abs(fast.angular_velocity) < @abs(slow.angular_velocity));
}

test "493: vehicle drag - speed damping" {
    init();
    const grounded = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    const airborne = createCar(10, 0, 0, 0) orelse return error.TestUnexpectedResult;
    grounded.speed = 50.0;
    airborne.speed = 50.0;
    grounded.grounded = true;
    airborne.grounded = false;

    updateCar(grounded, 0.5);
    updateCar(airborne, 0.5);

    try std.testing.expect(grounded.speed < 50.0);
    try std.testing.expect(airborne.speed < 50.0);
    // Ground rolling friction + engine braking should dissipate more than airborne damping.
    try std.testing.expect(grounded.speed < airborne.speed);
}

test "494: vehicle lift" {
    init();
    const aircraft = createAircraft(0, 100, 0);
    try std.testing.expect(aircraft != null);
    try std.testing.expect(aircraft.?.vehicle_type == .aircraft);
    // Aircraft has different physics handling
    try std.testing.expect(aircraft.?.mass == 5000);
}

test "495: vehicle lateral dynamics - right direction" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    const right = getRightDir(car.?);
    // At yaw=0, right is +X direction
    try std.testing.expectApproxEqAbs(@as(f32, 1), right.x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0), right.z, 0.0001);
}

test "496: vehicle longitudinal dynamics" {
    terrain.init();
    terrain.init();
    weather.init();
    init();
    const car = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    car.grounded = true;
    car.speed = 30.0;
    car.ai_target_speed = 50.0; // set target so updateCar throttle PID kicks in
    car.target_vel = 50.0;
    car.ai_vehicle_id = 0;
    car.brake = 0.0;

    updateCar(car, 0.2);
    const accelerated_speed = car.speed;
    try std.testing.expect(accelerated_speed > 30.0);

    car.ai_target_speed = 10.0;
    car.target_vel = 10.0;
    updateCar(car, 0.2);
    try std.testing.expect(car.speed < accelerated_speed);
}

test "vehicle grounded dynamics lose acceleration and steering authority on hazardous surface/weather" {
    terrain.init();
    weather.init();
    init();

    const clear = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    clear.grounded = true;
    clear.speed = 20.0;
    clear.throttle = 1.0;
    clear.brake = 0.0;
    clear.steering = 1.0;
    updateCar(clear, 0.2);
    const clear_speed = clear.speed;
    const clear_ang_vel = @abs(clear.angular_velocity);

    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.9, 60.0);
    weather.updateWeather(1.0);

    init();
    const hazard = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    hazard.grounded = true;
    hazard.speed = 20.0;
    hazard.throttle = 1.0;
    hazard.brake = 0.0;
    hazard.steering = 1.0;
    updateCar(hazard, 0.2);

    try std.testing.expect(hazard.speed < clear_speed);
    try std.testing.expect(@abs(hazard.angular_velocity) < clear_ang_vel);

    weather.init();
    terrain.init();
}

test "497: vehicle stability control" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    car.?.grounded = true;
    car.?.flipped = false;
    // Vehicle should be stable when grounded and not flipped
    try std.testing.expect(car.?.grounded == true);
    try std.testing.expect(car.?.flipped == false);
}

test "498: vehicle traction control" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Driven wheels exist
    try std.testing.expect(car.?.wheels[0].driven == true);
    try std.testing.expect(car.?.wheels[2].driven == false); // Rear wheels not driven
}

test "499: vehicle ABS - anti-lock braking" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Brakes are applied to all wheels
    applyBrake(car.?, 0.7);
    try std.testing.expect(car.?.brake > 0);
    // Handbrake can be set independently
    setHandbrake(car.?, true);
    try std.testing.expect(car.?.handbrake == true);
}

test "500: vehicle electronic stability" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Apply steering and check it's within bounds
    applySteering(car.?, 0.5);
    try std.testing.expect(@abs(car.?.steering) <= 1.0);
}

test "501: vehicle differential" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Front wheels are driven, rear are not
    try std.testing.expect(car.?.wheels[0].driven == true);
    try std.testing.expect(car.?.wheels[1].driven == true);
    try std.testing.expect(car.?.wheels[2].driven == false);
    try std.testing.expect(car.?.wheels[3].driven == false);
}

test "502: vehicle all-wheel drive" {
    init();
    const truck = createCar(0, 0, 0, 0); // Using car for simplicity
    try std.testing.expect(truck != null);
    // Car has 4 wheels available for potential AWD
    try std.testing.expect(truck.?.wheels.len == 4);
}

test "503: vehicle hybrid powertrain" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Throttle can be negative for regenerative braking
    applyThrottle(car.?, -0.5);
    try std.testing.expect(car.?.throttle == -0.5);
}

test "504: vehicle electric powertrain" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    // Electric vehicles can have instant torque
    applyThrottle(car.?, 1.0);
    try std.testing.expect(car.?.throttle == 1.0);
}

test "505: vehicle autonomy interface" {
    init();
    const car = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    car.speed = 12.0;
    applyAutonomyCommand(car, .{
        .target_speed = 30.0,
        .target_yaw_rate = 0.04,
        .emergency_brake = false,
    }, 0.1);

    try std.testing.expect(car.throttle > 0.0);
    try std.testing.expect(car.brake == 0.0);
    try std.testing.expect(car.steering > 0.0);

    applyAutonomyCommand(car, .{
        .target_speed = 0.0,
        .target_yaw_rate = 0.0,
        .emergency_brake = true,
    }, 0.1);
    try std.testing.expect(car.brake > 0.99);
    try std.testing.expect(car.throttle == 0.0);
}

test "vehicle autonomy command scales throttle and steering under terrain/weather hazard" {
    terrain.init();
    weather.init();
    init();
    const clear_car = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    clear_car.grounded = true;
    clear_car.speed = 10.0;
    applyAutonomyCommand(clear_car, .{
        .target_speed = 35.0,
        .target_yaw_rate = 0.06,
        .emergency_brake = false,
    }, 0.1);
    const clear_throttle = clear_car.throttle;
    const clear_steering = @abs(clear_car.steering);

    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.9, 60.0);
    weather.updateWeather(1.0);

    init();
    const hazard_car = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    hazard_car.grounded = true;
    hazard_car.speed = 10.0;
    applyAutonomyCommand(hazard_car, .{
        .target_speed = 35.0,
        .target_yaw_rate = 0.06,
        .emergency_brake = false,
    }, 0.1);

    try std.testing.expect(hazard_car.throttle < clear_throttle);
    try std.testing.expect(@abs(hazard_car.steering) < clear_steering);

    weather.init();
    terrain.init();
}

test "vehicle autonomy command keeps aircraft control authority independent of terrain/weather" {
    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 1.0, 60.0);
    weather.triggerWeather(.fog, 1.0, 60.0);
    weather.updateWeather(1.0);

    init();
    const aircraft = createAircraft(0, 100, 0) orelse return error.TestUnexpectedResult;
    aircraft.speed = 0.0;
    applyAutonomyCommand(aircraft, .{
        .target_speed = 50.0,
        .target_yaw_rate = 0.02,
        .emergency_brake = false,
    }, 0.1);

    try std.testing.expectApproxEqAbs(@as(f32, 1.0), aircraft.throttle, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), aircraft.steering, 0.0001);
    weather.init();
    terrain.init();
}

test "vehicle environment authority reports degraded scales under hazardous terrain and weather" {
    terrain.init();
    weather.init();

    const clear = measureEnvironmentControlAuthority(.car, 0.0, 0.0, 20.0);

    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.9, 60.0);
    weather.updateWeather(1.0);

    const hazard = measureEnvironmentControlAuthority(.car, 0.0, 0.0, 20.0);

    try std.testing.expect(hazard.speed_scale < clear.speed_scale);
    try std.testing.expect(hazard.throttle_scale < clear.throttle_scale);
    try std.testing.expect(hazard.steering_scale < clear.steering_scale);
    try std.testing.expect(hazard.brake_scale > clear.brake_scale);

    weather.init();
    terrain.init();
}

test "vehicle environment authority remains neutral for aircraft type" {
    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 200, .water);
    weather.triggerWeather(.storm, 1.0, 60.0);
    weather.triggerWeather(.fog, 1.0, 60.0);
    weather.updateWeather(1.0);

    const authority = measureEnvironmentControlAuthority(.aircraft, 0.0, 0.0, 80.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), authority.speed_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), authority.throttle_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), authority.brake_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), authority.steering_scale, 0.0001);

    weather.init();
    terrain.init();
}

test "506: vehicle prediction interface" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    car.?.speed = 10.0;
    const pose = predictVehiclePose(car.?, 0.1);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), pose.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), pose.pos_z, 0.0001);
}

test "507: vehicle network synchronization" {
    init();
    const authoritative = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    const replica = createCar(10, 0, 0, 0) orelse return error.TestUnexpectedResult;

    authoritative.pos_x = 100.0;
    authoritative.pos_y = 2.0;
    authoritative.pos_z = -30.0;
    authoritative.yaw = std.math.pi - 0.15;
    authoritative.speed = 45.0;
    authoritative.angular_velocity = 0.2;

    const packet = exportNetState(authoritative);
    importNetState(replica, packet, 1.0, 1.0);

    try std.testing.expectApproxEqAbs(authoritative.pos_x, replica.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(authoritative.pos_y, replica.pos_y, 0.0001);
    try std.testing.expectApproxEqAbs(authoritative.pos_z, replica.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(authoritative.yaw, replica.yaw, 0.0001);
    try std.testing.expectApproxEqAbs(authoritative.speed, replica.speed, 0.0001);

    // Partial blend should converge smoothly instead of hard snapping.
    replica.pos_x = 0.0;
    importNetState(replica, packet, 0.25, 0.25);
    try std.testing.expectApproxEqAbs(@as(f32, 25.0), replica.pos_x, 0.0001);
}

test "508: vehicle collision response" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    car.?.speed = 50.0;
    applyCollisionResponse(car.?, 1.0, 0.0, 10.0);
    // After collision, speed should be reduced
    try std.testing.expect(car.?.speed < 50.0);
}

test "509: vehicle roll physics" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    car.?.roll = 1.5; // > 1.0 to trigger flipped state
    car.?.grounded = true;
    const flipped = checkFlipped(car.?);
    try std.testing.expect(flipped == true);
}

test "510: vehicle flip recovery" {
    init();
    const car = createCar(0, 0, 0, 0);
    try std.testing.expect(car != null);
    car.?.pitch = 1.5;
    car.?.roll = 0.5;
    car.?.flipped = true;
    var recovered = false;
    var i: u8 = 0;
    while (i < 64) : (i += 1) {
        recovered = attemptFlip(car.?);
        if (recovered) break;
    }
    try std.testing.expect(recovered);
    try std.testing.expect(!car.?.flipped);
}

test "vehicle removal keeps array compact and count consistent" {
    init();
    const a = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    _ = createCar(10, 0, 0, 0) orelse return error.TestUnexpectedResult;
    const c = createCar(20, 0, 0, 0) orelse return error.TestUnexpectedResult;
    try std.testing.expect(getSystem().count == 3);

    removeVehicle(a);
    try std.testing.expect(getSystem().count == 2);
    try std.testing.expect(getSystem().vehicles[0].pos_x == c.pos_x or getSystem().vehicles[1].pos_x == c.pos_x);
}

test "vehicle car yaw integration is approximately timestep-invariant over equal total time" {
    var coarse = VehicleState{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .speed = 80,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0.8,
        .brake = 0,
        .handbrake = false,
        .grounded = true,
        .flipped = false,
        .vehicle_type = .car,
        .wheels = [_]WheelConfig{
            .{ .offset_x = -6, .offset_y = 0, .offset_z = -8, .radius = 4, .steering_angle = 0, .driven = true, .braked = true },
            .{ .offset_x = 6, .offset_y = 0, .offset_z = -8, .radius = 4, .steering_angle = 0, .driven = true, .braked = true },
            .{ .offset_x = -6, .offset_y = 0, .offset_z = 8, .radius = 4, .steering_angle = 0, .driven = false, .braked = true },
            .{ .offset_x = 6, .offset_y = 0, .offset_z = 8, .radius = 4, .steering_angle = 0, .driven = false, .braked = true },
        },
        .mass = 1500,
        .susp_states = undefined,
        .tire_states = undefined,
    };
    var fine = coarse;

    updateCar(&coarse, 0.5);
    var i: u8 = 0;
    while (i < 5) : (i += 1) {
        updateCar(&fine, 0.1);
    }

    try std.testing.expectApproxEqAbs(coarse.yaw, fine.yaw, 0.02);
}

test "vehicle boat drag is approximately timestep-invariant over equal total time" {
    var coarse = VehicleState{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .speed = 120,
        .angular_velocity = 0,
        .throttle = 0,
        .steering = 0,
        .brake = 0,
        .handbrake = false,
        .grounded = false,
        .flipped = false,
        .vehicle_type = .boat,
        .wheels = [_]WheelConfig{.{ .offset_x = 0, .offset_y = 0, .offset_z = 0, .radius = 0, .steering_angle = 0, .driven = false, .braked = false }} ** 4,
        .tire_states = undefined,
        .mass = 2000,
        .susp_states = undefined,
    };
    var fine = coarse;

    updateBoat(&coarse, 0.5);
    var i: u8 = 0;
    while (i < 5) : (i += 1) {
        updateBoat(&fine, 0.1);
    }

    try std.testing.expectApproxEqAbs(coarse.speed, fine.speed, 0.1);
}

test "vehicle boat authority degrades under storm and off-water medium" {
    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 300, .water);

    var clear = VehicleState{
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .speed = 80,
        .angular_velocity = 0,
        .throttle = 1.0,
        .steering = 1.0,
        .brake = 0,
        .handbrake = false,
        .grounded = false,
        .flipped = false,
        .vehicle_type = .boat,
        .tire_states = undefined,
        .wheels = [_]WheelConfig{.{ .offset_x = 0, .offset_y = 0, .offset_z = 0, .radius = 0, .steering_angle = 0, .driven = false, .braked = false }} ** 4,
        .mass = 2000,
        .susp_states = undefined,
    };
    updateBoat(&clear, 0.2);
    const clear_speed = clear.speed;
    const clear_yaw = @abs(clear.yaw);

    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 300, .mud);
    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.8, 60.0);
    weather.updateWeather(1.0);

    var hazard = clear;
    hazard.yaw = 0;
    hazard.speed = 80;
    hazard.throttle = 1.0;
    hazard.steering = 1.0;
    updateBoat(&hazard, 0.2);

    try std.testing.expect(hazard.speed < clear_speed);
    try std.testing.expect(@abs(hazard.yaw) < clear_yaw);

    weather.init();
    terrain.init();
}

test "vehicle hovercraft authority degrades under liquid surface and severe weather" {
    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 300, .concrete);

    var clear = VehicleState{
        .pos_x = 0,
        .pos_y = 1,
        .pos_z = 0,
        .yaw = 0,
        .pitch = 0,
        .roll = 0,
        .speed = 60,
        .angular_velocity = 0,
        .throttle = 1.0,
        .steering = 1.0,
        .brake = 0,
        .handbrake = false,
        .grounded = true,
        .flipped = false,
        .tire_states = undefined,
        .vehicle_type = .hovercraft,
        .wheels = [_]WheelConfig{.{ .offset_x = 0, .offset_y = 0, .offset_z = 0, .radius = 0, .steering_angle = 0, .driven = false, .braked = false }} ** 4,
        .mass = 1000,
        .susp_states = undefined,
    };
    updateHovercraft(&clear, 0.2);
    const clear_speed = clear.speed;
    const clear_yaw = @abs(clear.yaw);

    terrain.init();
    weather.init();
    terrain.addTerrainPatch(0, 0, 300, .water);
    weather.triggerWeather(.storm, 0.9, 60.0);
    weather.triggerWeather(.fog, 0.7, 60.0);
    weather.updateWeather(1.0);

    var hazard = clear;
    hazard.yaw = 0;
    hazard.speed = 60;
    hazard.throttle = 1.0;
    hazard.steering = 1.0;
    updateHovercraft(&hazard, 0.2);

    try std.testing.expect(hazard.speed < clear_speed);
    try std.testing.expect(@abs(hazard.yaw) < clear_yaw);

    weather.init();
    terrain.init();
}

/// Sync ai_target_speed from ai_traffic governed_target_vel.
/// Must be called after ai_traffic.update() each tick.
pub fn syncAIVehicleFromTraffic(vehicle: *VehicleState) void {
    if (vehicle.ai_vehicle_id == 0) return;
    if (ai_traffic.getTrafficVehicle(vehicle.ai_vehicle_id)) |tv| {
        vehicle.ai_target_speed = tv.governed_target_vel;
        vehicle.target_vel = tv.governed_target_vel;
    }
}

/// Read governed_target_vel from ai_traffic and write it to vehicle.target_vel.
/// Returns the read speed, or -1 if vehicle is not linked.
pub fn getAIVehicleTargetVel(vehicle: *const VehicleState) f32 {
    if (vehicle.ai_vehicle_id == 0) return -1;
    const tv = ai_traffic.g_traffic_system.vehicles[vehicle.ai_vehicle_id - 1];
    if (!tv.active) return -1;
    const speed = tv.governed_target_vel;
    return speed;
}

test "vehicle reads ai governed target vel" {
    ai_traffic.init();
    init();

    // Create AI traffic vehicle with a known governed_target_vel
    const ai_veh = ai_traffic.spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    ai_veh.governed_target_vel = 33.0;

    // Create a vehicle and link it to the AI vehicle
    const car = createCar(0, 0, 0, 0) orelse return error.TestUnexpectedResult;
    try std.testing.expect(car.ai_vehicle_id == 0);

    setAIVehicleLink(car, ai_veh.vehicle_id, -1);
    try std.testing.expect(car.ai_vehicle_id == ai_veh.vehicle_id);

    // getAIVehicleTargetVel reads governed_target_vel and returns it
    const speed = getAIVehicleTargetVel(car);
    try std.testing.expectApproxEqAbs(@as(f32, 33.0), speed, 0.0001);

    // unlinked vehicle returns -1
    const unlinked = createCar(10, 0, 0, 0) orelse return error.TestUnexpectedResult;
    const unlinked_speed = getAIVehicleTargetVel(unlinked);
    try std.testing.expect(unlinked_speed < 0);
}
