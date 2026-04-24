//! AI Traffic System - NPC Vehicles and Behavior
//!
//! Phase 34, 67, 68: AI-controlled vehicles, traffic behavior, aggressive driving
//! Handles: Path following, collision avoidance, lane changing, aggressive maneuvers

const std = @import("std");

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
    state: u8,
    timer: f32,
    yellow_duration: f32,
    cycle_duration: f32,
};

pub const MAX_AI_VEHICLES: usize = 32;
pub const MAX_TRAFFIC_LIGHTS: usize = 16;

var g_traffic_system: struct {
    vehicles: [MAX_AI_VEHICLES]TrafficVehicle,
    vehicle_count: u8,
    lights: [MAX_TRAFFIC_LIGHTS]TrafficLight,
    light_count: u8,
    global_time: f32,
} = undefined;

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
        .state = 0,
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

        if (dist < 100 and light.state == 0) {
            return true;
        }
    }
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
            light.state = 1;
        } else if (cycle_pos < green_duration + light.yellow_duration) {
            light.state = 2;
        } else if (cycle_pos < green_duration * 2 + light.yellow_duration) {
            light.state = 0;
        } else {
            light.state = 2;
        }
    }

    for (0..MAX_AI_VEHICLES) |i| {
        var vehicle = &g_traffic_system.vehicles[i];
        if (!vehicle.active) continue;

        if (checkRedLight(vehicle, dt)) {
            vehicle.brake_input = 1.0;
            vehicle.throttle_input = 0;
        } else if (checkVehicleAhead(vehicle, dt)) |ahead| {
            const dz = ahead.pos_z - vehicle.pos_z;
            const required_distance = calculateFollowingDistance(vehicle, vehicle.target_vel);

            if (dz < required_distance) {
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
    var active: [MAX_AI_VEHICLES]TrafficVehicle = undefined;
    var count: u8 = 0;

    for (g_traffic_system.vehicles[0..g_traffic_system.vehicle_count]) |vehicle| {
        if (vehicle.active) {
            active[count] = vehicle;
            count += 1;
        }
    }

    return active[0..count];
}

pub fn getVehicleCount() u8 {
    return g_traffic_system.vehicle_count;
}
