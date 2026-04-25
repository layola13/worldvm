//! AI Traffic System - NPC Vehicles and Behavior
//!
//! Phase 34, 67, 68: AI-controlled vehicles, traffic behavior, aggressive driving
//! Handles: Path following, collision avoidance, lane changing, aggressive maneuvers

const std = @import("std");
const prediction = @import("prediction.zig");

pub const TrafficLightState = enum(u8) {
    red = 0,
    green = 1,
    yellow = 2,
};

pub const AIBehavior = enum(u8) {
    cautious = 0,
    normal = 1,
    aggressive = 2,
    reckless = 3,
};

pub const TrafficVehicle = struct {
    vehicle_id: u16,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    vel_x: f32,
    vel_z: f32,
    yaw: f32,
    target_vel: f32,
    current_lane: i8,
    target_lane: i8,
    behavior: AIBehavior,
    following_distance: f32,
    reaction_time: f32,
    throttle_input: f32,
    brake_input: f32,
    steering_input: f32,
    active: bool,
};

pub const TrafficLight = struct {
    pos_x: f32,
    pos_z: f32,
    state: TrafficLightState,
    timer: f32,
    yellow_duration: f32,
    cycle_duration: f32,
};

pub const MAX_AI_VEHICLES: usize = 32;
pub const MAX_TRAFFIC_LIGHTS: usize = 16;

pub const TrafficSystem = struct {
    vehicles: [MAX_AI_VEHICLES]TrafficVehicle,
    vehicle_count: u8,
    lights: [MAX_TRAFFIC_LIGHTS]TrafficLight,
    light_count: u8,
    global_time: f32,
};

var g_traffic_system: TrafficSystem = undefined;

pub fn init() void {
    g_traffic_system.vehicle_count = 0;
    g_traffic_system.light_count = 0;
    g_traffic_system.global_time = 0;
}

pub fn spawnAIVehicle(x: f32, y: f32, z: f32, behavior: AIBehavior) ?*TrafficVehicle {
    if (g_traffic_system.vehicle_count >= MAX_AI_VEHICLES) return null;
    const idx = g_traffic_system.vehicle_count;
    g_traffic_system.vehicle_count += 1;

    g_traffic_system.vehicles[idx] = .{
        .vehicle_id = @as(u16, @intCast(idx)) + 1,
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .vel_x = 0,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 30,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = behavior,
        .following_distance = switch (behavior) {
            .cautious => 50,
            .normal => 30,
            .aggressive => 15,
            .reckless => 5,
        },
        .reaction_time = switch (behavior) {
            .cautious => 0.5,
            .normal => 0.3,
            .aggressive => 0.15,
            .reckless => 0.05,
        },
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
    };
    return &g_traffic_system.vehicles[idx];
}

pub fn addTrafficLight(x: f32, z: f32, cycle_duration: f32) ?*TrafficLight {
    if (g_traffic_system.light_count >= MAX_TRAFFIC_LIGHTS) return null;
    const idx = g_traffic_system.light_count;
    g_traffic_system.light_count += 1;

    g_traffic_system.lights[idx] = .{
        .pos_x = x,
        .pos_z = z,
        .state = .red,
        .timer = 0,
        .yellow_duration = 3.0,
        .cycle_duration = cycle_duration,
    };
    return &g_traffic_system.lights[idx];
}

pub fn calculateFollowingDistance(self: *const TrafficVehicle, speed: f32) f32 {
    const base_distance = self.following_distance;
    const speed_factor = speed / 30.0;
    return base_distance * (1.0 + speed_factor * 0.5);
}

pub fn checkVehicleAhead(self: *const TrafficVehicle, _: f32) ?*const TrafficVehicle {
    var closest_dist: f32 = 99999;
    var closest: ?*const TrafficVehicle = null;

    for (g_traffic_system.vehicles[0..g_traffic_system.vehicle_count]) |*vehicle| {
        if (!vehicle.active or vehicle.vehicle_id == self.vehicle_id) continue;
        if (@abs(vehicle.pos_z - self.pos_z) > 100) continue;

        const dz = vehicle.pos_z - self.pos_z;
        if (dz < 0) continue;

        const dx = vehicle.pos_x - self.pos_x;
        const dist = @sqrt(dx * dx + dz * dz);

        if (dist < closest_dist and dist < 200) {
            closest_dist = dist;
            closest = vehicle;
        }
    }

    return closest;
}

pub fn checkRedLight(self: *const TrafficVehicle, _: f32) bool {
    for (g_traffic_system.lights[0..g_traffic_system.light_count]) |light| {
        const dx = light.pos_x - self.pos_x;
        const dz = light.pos_z - self.pos_z;
        const dist = @sqrt(dx * dx + dz * dz);

        if (dist < 100 and light.state == .red) {
            return true;
        }
    }
    return false;
}

pub fn getTrafficLightState(light: *const TrafficLight) prediction.SignalWindow {
    return prediction.predictSignalWindow(
        switch (light.state) {
            .red => .red,
            .green => .green,
            .yellow => .yellow,
        },
        light.timer,
        light.cycle_duration,
        light.yellow_duration,
    );
}

pub fn estimateSafePassForVehicle(self: *const TrafficVehicle, light: *const TrafficLight, vehicle_length: f32) prediction.SafePassResult {
    const signal = getTrafficLightState(light);
    const dx = light.pos_x - self.pos_x;
    const dz = light.pos_z - self.pos_z;
    const distance = @sqrt(dx * dx + dz * dz);
    const speed = @max(0.001, @sqrt(self.vel_x * self.vel_x + self.vel_z * self.vel_z));
    return prediction.estimateSafePass(distance, speed, vehicle_length, signal);
}

pub fn computeVehicleConflict(self: *const TrafficVehicle, other: *const TrafficVehicle, horizon: f32) prediction.TTCResult {
    return prediction.computeTTC(.{
        .pos_x = self.pos_x,
        .pos_y = self.pos_y,
        .pos_z = self.pos_z,
        .vel_x = self.vel_x,
        .vel_y = 0,
        .vel_z = self.vel_z,
    }, .{
        .pos_x = other.pos_x,
        .pos_y = other.pos_y,
        .pos_z = other.pos_z,
        .vel_x = other.vel_x,
        .vel_y = 0,
        .vel_z = other.vel_z,
    }, 2.5, horizon);
}

pub fn predictVehiclePose(self: *const TrafficVehicle, time_delta: f32) prediction.PlanarPoseForecast {
    return prediction.predictPlanarPose(.{
        .pos_x = self.pos_x,
        .pos_y = self.pos_y,
        .pos_z = self.pos_z,
        .yaw = self.yaw,
    }, self.vel_x, 0.0, self.vel_z, self.steering_input, time_delta);
}

fn getVehicleHalfExtents(_: *const TrafficVehicle) struct { x: f32, y: f32, z: f32 } {
    return .{ .x = 1.25, .y = 1.0, .z = 2.5 };
}

pub fn computeVehicleOccupancyConflict(self: *const TrafficVehicle, other: *const TrafficVehicle, horizon: f32, step: f32) prediction.OccupancyConflictWindow {
    const self_extents = getVehicleHalfExtents(self);
    const other_extents = getVehicleHalfExtents(other);
    return prediction.computeOccupancyConflictWindow(
        .{
            .pos_x = self.pos_x,
            .pos_y = self.pos_y,
            .pos_z = self.pos_z,
            .yaw = self.yaw,
        },
        self.vel_x,
        0.0,
        self.vel_z,
        self.steering_input,
        self_extents.x,
        self_extents.y,
        self_extents.z,
        .{
            .pos_x = other.pos_x,
            .pos_y = other.pos_y,
            .pos_z = other.pos_z,
            .yaw = other.yaw,
        },
        other.vel_x,
        0.0,
        other.vel_z,
        other.steering_input,
        other_extents.x,
        other_extents.y,
        other_extents.z,
        horizon,
        step,
    );
}

pub fn buildTrafficVehicleAvoidanceRecommendation(self: *const TrafficVehicle, other: *const TrafficVehicle, horizon: f32, step: f32) prediction.AvoidanceRecommendation {
    const conflict = computeVehicleOccupancyConflict(self, other, horizon, step);
    const brake_threshold_ratio = @min(1.0, (self.reaction_time + 0.75) / @max(horizon, 0.001));
    return prediction.buildAvoidanceRecommendation(conflict, horizon, 0.0, brake_threshold_ratio, 0.0, 0.0, 1.0);
}

fn shouldBrakeForVehicleConflict(self: *const TrafficVehicle, other: *const TrafficVehicle) bool {
    const dz = other.pos_z - self.pos_z;
    const required_distance = calculateFollowingDistance(self, self.target_vel);
    const ttc = computeVehicleConflict(self, other, 5.0);
    const recommendation = buildTrafficVehicleAvoidanceRecommendation(self, other, 5.0, 0.25);

    if (dz < required_distance) return true;
    if (ttc.valid and ttc.time < self.reaction_time + 0.5) return true;
    if (recommendation.should_brake) return true;
    return false;
}

pub fn calculateLaneChange(target_lane: i8, current_lane: i8, dt: f32) f32 {
    const diff = @as(f32, @floatFromInt(target_lane - current_lane));
    if (@abs(diff) < 0.01) return 0;
    const sign: f32 = if (diff > 0) 1.0 else -1.0;
    return sign * dt * 2.0;
}

pub fn updateAI(dt: f32) void {
    g_traffic_system.global_time += dt;

    for (0..MAX_TRAFFIC_LIGHTS) |i| {
        var light = &g_traffic_system.lights[i];
        light.timer += dt;

        const cycle_pos = @mod(light.timer, light.cycle_duration);
        const green_duration = light.cycle_duration - light.yellow_duration * 2;

        if (cycle_pos < green_duration) {
            light.state = .green;
        } else if (cycle_pos < green_duration + light.yellow_duration) {
            light.state = .yellow;
        } else if (cycle_pos < green_duration * 2 + light.yellow_duration) {
            light.state = .red;
        } else {
            light.state = .yellow;
        }
    }

    for (0..MAX_AI_VEHICLES) |i| {
        var vehicle = &g_traffic_system.vehicles[i];
        if (!vehicle.active) continue;

        var should_brake_for_signal = false;
        for (g_traffic_system.lights[0..g_traffic_system.light_count]) |*light| {
            const signal_decision = estimateSafePassForVehicle(vehicle, light, 4.5);
            const dx = light.pos_x - vehicle.pos_x;
            const dz = light.pos_z - vehicle.pos_z;
            const distance = @sqrt(dx * dx + dz * dz);
            if (distance < 100 and light.state == .red) {
                should_brake_for_signal = true;
                break;
            }
            if (distance < 40 and light.state == .yellow and !signal_decision.can_pass) {
                should_brake_for_signal = true;
                break;
            }
        }

        if (should_brake_for_signal or checkRedLight(vehicle, dt)) {
            vehicle.brake_input = 1.0;
            vehicle.throttle_input = 0;
        } else if (checkVehicleAhead(vehicle, dt)) |ahead| {
            if (shouldBrakeForVehicleConflict(vehicle, ahead)) {
                vehicle.brake_input = 1.0;
                vehicle.throttle_input = 0;
            } else {
                vehicle.throttle_input = 1.0;
                vehicle.brake_input = 0;
            }
        } else {
            vehicle.throttle_input = 1.0;
            vehicle.brake_input = 0;
        }

        if (vehicle.target_lane != vehicle.current_lane) {
            vehicle.steering_input = calculateLaneChange(vehicle.target_lane, vehicle.current_lane, dt);
            vehicle.current_lane = @addWithOverflow(vehicle.current_lane, @as(i8, @intFromFloat(vehicle.steering_input * dt * 10)))[0];
        }

        _ = @sqrt(vehicle.vel_x * vehicle.vel_x + vehicle.vel_z * vehicle.vel_z);
        vehicle.vel_z += vehicle.throttle_input * 20 * dt;
        vehicle.vel_z -= vehicle.brake_input * 30 * dt;
        vehicle.vel_z = @max(0, vehicle.vel_z);

        vehicle.pos_x += vehicle.vel_x * dt;
        vehicle.pos_z += vehicle.vel_z * dt;
    }
}

pub fn setBehavior(vehicle_id: u16, behavior: AIBehavior) void {
    for (0..MAX_AI_VEHICLES) |i| {
        if (g_traffic_system.vehicles[i].vehicle_id == vehicle_id) {
            g_traffic_system.vehicles[i].behavior = behavior;
            g_traffic_system.vehicles[i].following_distance = switch (behavior) {
                .cautious => 50,
                .normal => 30,
                .aggressive => 15,
                .reckless => 5,
            };
            break;
        }
    }
}

pub fn triggerEmergencyVehicle(vehicle: *TrafficVehicle) void {
    vehicle.behavior = .reckless;
    vehicle.target_vel = 60;
    vehicle.following_distance = 2;
}

pub fn getTrafficVehicles() []TrafficVehicle {
    return g_traffic_system.vehicles[0..g_traffic_system.vehicle_count];
}

pub fn getVehicleCount() u8 {
    return g_traffic_system.vehicle_count;
}

pub fn getTrafficLightCount() u8 {
    return g_traffic_system.light_count;
}

pub fn getTrafficLight(idx: u8) ?*const TrafficLight {
    if (idx >= g_traffic_system.light_count) return null;
    return &g_traffic_system.lights[idx];
}

pub fn getSystem() *TrafficSystem {
    return &g_traffic_system;
}

test "predictVehiclePose advances traffic vehicle pose with steering yaw rate" {
    const pose = predictVehiclePose(&.{
        .vehicle_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 10,
        .vel_x = 2,
        .vel_z = 8,
        .yaw = 0.5,
        .target_vel = 30,
        .current_lane = 0,
        .target_lane = 1,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0.25,
        .active = true,
    }, 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), pose.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 26.0), pose.pos_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), pose.yaw, 0.0001);
}

test "computeVehicleOccupancyConflict detects future overlap window" {
    const window = computeVehicleOccupancyConflict(&.{
        .vehicle_id = 1,
        .pos_x = -6,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 4,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 0,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
    }, &.{
        .vehicle_id = 2,
        .pos_x = 6,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = -4,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 0,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
    }, 3.0, 0.25);
    try std.testing.expect(window.valid);
    try std.testing.expect(window.start_time >= 0.75 and window.start_time <= 1.25);
}

test "shouldBrakeForVehicleConflict triggers on upcoming occupancy overlap" {
    const should_brake = shouldBrakeForVehicleConflict(&.{
        .vehicle_id = 1,
        .pos_x = -6,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 4,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 10,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 1,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
    }, &.{
        .vehicle_id = 2,
        .pos_x = 6,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = -4,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 10,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 1,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
    });
    try std.testing.expect(should_brake);
}

test "updateAI brakes for upcoming occupancy conflict even before close spacing" {
    init();
    const self = spawnAIVehicle(-6.0, 0.0, 0.0, .normal) orelse return error.TestUnexpectedResult;
    const ahead = spawnAIVehicle(6.0, 0.0, 0.0, .normal) orelse return error.TestUnexpectedResult;

    self.vel_x = 4.0;
    self.vel_z = 0.0;
    self.target_vel = 10.0;
    self.following_distance = 1.0;
    self.reaction_time = 0.3;

    ahead.vel_x = -4.0;
    ahead.vel_z = 0.0;
    ahead.target_vel = 10.0;
    ahead.following_distance = 1.0;
    ahead.reaction_time = 0.3;

    updateAI(0.1);

    try std.testing.expect(self.brake_input > 0.99);
    try std.testing.expect(self.throttle_input == 0.0);
}
