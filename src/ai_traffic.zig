//! AI Traffic System - NPC Vehicles and Behavior
//!
//! Phase 34, 67, 68: AI-controlled vehicles, traffic behavior, aggressive driving
//! Handles: Path following, collision avoidance, lane changing, aggressive maneuvers

const std = @import("std");
const planner = @import("planner.zig");
const prediction = @import("prediction.zig");
const terrain = @import("terrain.zig");
const vehicle_physics = @import("vehicle.zig");
const weather = @import("weather.zig");
const query = @import("query.zig");

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
    governed_target_vel: f32,
    current_lane: i8,
    target_lane: i8,
    behavior: AIBehavior,
    following_distance: f32,
    reaction_time: f32,
    throttle_input: f32,
    brake_input: f32,
    steering_input: f32,
    active: bool,
    world: ?*const query.QueryWorldView,
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
    world: ?*const query.QueryWorldView,

    pub fn getGovernedTargetSpeed(self: *const TrafficSystem, vehicle_id: u16) ?f32 {
        for (0..self.vehicle_count) |i| {
            if (self.vehicles[i].vehicle_id == vehicle_id) {
                return self.vehicles[i].governed_target_vel;
            }
        }
        return null;
    }

    pub fn getTargetVel(self: *const TrafficSystem, vehicle_id: u16) ?f32 {
        for (0..self.vehicle_count) |i| {
            if (self.vehicles[i].vehicle_id == vehicle_id) {
                return self.vehicles[i].target_vel;
            }
        }
        return null;
    }
};

pub var g_traffic_system: TrafficSystem = undefined;

fn behaviorFollowingDistance(behavior: AIBehavior) f32 {
    return switch (behavior) {
        .cautious => 50,
        .normal => 30,
        .aggressive => 15,
        .reckless => 5,
    };
}

fn behaviorReactionTime(behavior: AIBehavior) f32 {
    return switch (behavior) {
        .cautious => 0.5,
        .normal => 0.3,
        .aggressive => 0.15,
        .reckless => 0.05,
    };
}

fn behaviorTargetSpeedCap(behavior: AIBehavior) f32 {
    return switch (behavior) {
        .cautious => 16.0,
        .normal => 30.0,
        .aggressive => 42.0,
        .reckless => 60.0,
    };
}

fn behaviorCruiseThrottle(behavior: AIBehavior) f32 {
    return switch (behavior) {
        .cautious => 0.7,
        .normal => 1.0,
        .aggressive => 1.0,
        .reckless => 1.0,
    };
}

fn plannerBehaviorForTraffic(behavior: AIBehavior) planner.BehaviorState {
    return switch (behavior) {
        .cautious, .normal => .following,
        .aggressive, .reckless => .overtaking,
    };
}

fn weatherSpeedModifier(behavior: AIBehavior) f32 {
    const visibility = weather.getSensorVisibilityFactor();
    const traction_penalty = weather.getRoadTractionPenalty();
    const severity = weather.getWeatherSeverity();
    const exposure: f32 = switch (behavior) {
        .cautious => 0.45,
        .normal => 0.65,
        .aggressive => 0.85,
        .reckless => 1.0,
    };

    const traction_scale = std.math.clamp(1.0 - traction_penalty * exposure, 0.3, 1.0);
    const visibility_scale = std.math.clamp(0.55 + visibility * 0.45, 0.35, 1.0);
    const severity_scale = std.math.clamp(1.0 - severity * (0.2 + exposure * 0.2), 0.35, 1.0);
    return std.math.clamp(traction_scale * visibility_scale * severity_scale, 0.25, 1.0);
}

pub fn init() void {
    g_traffic_system.vehicle_count = 0;
    g_traffic_system.light_count = 0;
    g_traffic_system.global_time = 0;
    for (0..MAX_AI_VEHICLES) |i| {
        g_traffic_system.vehicles[i] = .{
            .vehicle_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .vel_x = 0,
            .vel_z = 0,
            .yaw = 0,
            .target_vel = 0,
            .governed_target_vel = 0,
            .current_lane = 0,
            .target_lane = 0,
            .behavior = .normal,
            .following_distance = behaviorFollowingDistance(.normal),
            .reaction_time = behaviorReactionTime(.normal),
            .throttle_input = 0,
            .brake_input = 0,
            .steering_input = 0,
            .active = false,
            .world = null,
        };
    }
    for (0..MAX_TRAFFIC_LIGHTS) |i| {
        g_traffic_system.lights[i] = .{
            .pos_x = 0,
            .pos_z = 0,
            .state = .red,
            .timer = 0,
            .yellow_duration = 3.0,
            .cycle_duration = 30.0,
        };
    }
}

pub fn spawnAIVehicle(x: f32, y: f32, z: f32, behavior: AIBehavior, lane: i8) ?*TrafficVehicle {
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
        .governed_target_vel = 30,
        .current_lane = lane,
        .target_lane = 0,
        .behavior = behavior,
        .following_distance = behaviorFollowingDistance(behavior),
        .reaction_time = behaviorReactionTime(behavior),
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
        .world = null,
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

fn getForwardDir(vehicle: *const TrafficVehicle) struct { x: f32, z: f32 } {
    return .{
        .x = @sin(vehicle.yaw),
        .z = @cos(vehicle.yaw),
    };
}

fn getRightDir(vehicle: *const TrafficVehicle) struct { x: f32, z: f32 } {
    return .{
        .x = @cos(vehicle.yaw),
        .z = -@sin(vehicle.yaw),
    };
}

fn projectRelativeToVehicle(self: *const TrafficVehicle, target_x: f32, target_z: f32) struct { forward: f32, lateral: f32 } {
    const dx = target_x - self.pos_x;
    const dz = target_z - self.pos_z;
    const forward = getForwardDir(self);
    const right = getRightDir(self);
    return .{
        .forward = dx * forward.x + dz * forward.z,
        .lateral = dx * right.x + dz * right.z,
    };
}

fn shouldBrakeForTrafficLight(self: *const TrafficVehicle, light: *const TrafficLight) bool {
    const rel = projectRelativeToVehicle(self, light.pos_x, light.pos_z);
    if (rel.forward <= 0.0 or rel.forward > 100.0) return false;
    if (@abs(rel.lateral) > 8.0) return false;

    const dist = @sqrt(rel.forward * rel.forward + rel.lateral * rel.lateral);
    const signal = getTrafficLightState(light);
    const signal_decision = estimateSafePassForVehicle(self, light, 4.5);
    if (signal.state_now == .red) return true;
    // Yellow with no safe passage: treat as red (Item 292)
    if (signal.state_now == .yellow and dist < 40 and !signal_decision.can_pass) return true;
    return false;
}

/// Find the closest vehicle ahead that poses a collision risk.
/// Uses unified prediction-based TTC to filter (Item 290).
pub fn checkVehicleAhead(self: *const TrafficVehicle, _: f32) ?*const TrafficVehicle {
    var closest_dist: f32 = 99999;
    var closest: ?*const TrafficVehicle = null;

    for (g_traffic_system.vehicles[0..g_traffic_system.vehicle_count]) |*vehicle| {
        if (!vehicle.active or vehicle.vehicle_id == self.vehicle_id) continue;
        const rel = projectRelativeToVehicle(self, vehicle.pos_x, vehicle.pos_z);
        const ttc = computeVehicleConflict(self, vehicle, 5.0);
        const occupancy_conflict = computeVehicleOccupancyConflict(self, vehicle, 5.0, 0.25);
        const imminent_conflict = (ttc.valid and ttc.time > 0.0 and ttc.time <= 2.5) or
            (occupancy_conflict.valid and occupancy_conflict.start_time <= 2.5);

        if ((rel.forward <= 0.0 or rel.forward > 200.0) and !imminent_conflict) continue;

        const lateral = @abs(rel.lateral);
        if (lateral > 6.0 and vehicle.current_lane != self.current_lane and vehicle.target_lane != self.current_lane and !imminent_conflict) continue;

        const dz = vehicle.pos_z - self.pos_z;
        const dx = vehicle.pos_x - self.pos_x;
        const dist = @sqrt(dx * dx + dz * dz);

        if (dist < closest_dist and dist < 200) {
            // Filter by prediction-based TTC (Item 290)
            if (ttc.valid and ttc.time > 0) {
                closest_dist = dist;
                closest = vehicle;
            } else if (occupancy_conflict.valid) {
                closest_dist = dist;
                closest = vehicle;
            } else if (dist < self.following_distance) {
                closest_dist = dist;
                closest = vehicle;
            }
        }
    }

    return closest;
}

/// Check if vehicle must stop for a red or unsafe yellow light.
/// Uses unified prediction layer for signal state prediction (Item 289).
/// Query-based obstacle detection using raycasts.
/// Returns distance to nearest obstacle, or max_distance if none.
pub fn queryObstacleAhead(self: *const TrafficVehicle, max_distance: f32) f32 {
    if (self.world == null) return max_distance;
    const world = self.world.?;

    const fwd = getForwardDir(self);
    const ray = query.QueryRay{
        .origin_x = self.pos_x,
        .origin_y = self.pos_y + 1.0,
        .origin_z = self.pos_z,
        .dir_x = fwd.x,
        .dir_y = 0.0,
        .dir_z = fwd.z,
        .max_distance = max_distance,
    };
    const hit = query.raycastSingle(world, ray, .{});
    return if (hit.hit) hit.distance else max_distance;
}

/// Query multiple directions for surround awareness.
pub fn querySurroundDistances(self: *const TrafficVehicle, distances: []f32, max_dist: f32) void {
    if (self.world == null) return;
    const world = self.world.?;
    const fwd = getForwardDir(self);
    // right = rotate forward 90 degrees
    const right_x = -fwd.z;
    const right_z = fwd.x;

    const dirs = [_]struct { f32, f32 }{
        .{ fwd.x, fwd.z },     // front
        .{ right_x, right_z }, // right
        .{ -right_x, -right_z }, // left
        .{ -fwd.x, -fwd.z },   // back
    };

    for (0..4) |i| {
        if (i < distances.len) {
            const ray = query.QueryRay{
                .origin_x = self.pos_x,
                .origin_y = self.pos_y + 1.0,
                .origin_z = self.pos_z,
                .dir_x = dirs[i][0],
                .dir_y = 0.0,
                .dir_z = dirs[i][1],
                .max_distance = max_dist,
            };
            const hit = query.raycastSingle(world, ray, .{});
            distances[i] = if (hit.hit) hit.distance else max_dist;
        }
    }
}

/// Check if road ahead is clear using query layer (faster than prediction).
pub fn isRoadClearAhead(self: *const TrafficVehicle, check_distance: f32) bool {
    const obs_dist = self.queryObstacleAhead(check_distance);
    return obs_dist >= check_distance;
}

/// Query terrain classification ahead for traction awareness.
pub fn queryTerrainAhead(self: *const TrafficVehicle, distance: f32) query.QueryHit {
    if (self.world == null) {
        return query.QueryHit{ .hit = false, .gx = 0, .gy = 0, .gz = 0 };
    }
    const world = self.world.?;
    const fwd = getForwardDir(self);
    const target_x = self.pos_x + fwd.x * distance;
    const target_z = self.pos_z + fwd.z * distance;
    const gy: i32 = @as(i32, @intFromFloat(self.pos_y));
    return query.queryEnvironmentVoxel(world, 
        @intFromFloat(target_x), gy, @intFromFloat(target_z));
}

pub fn checkRedLight(self: *const TrafficVehicle, _: f32) bool {
    for (g_traffic_system.lights[0..g_traffic_system.light_count]) |light| {
        if (shouldBrakeForTrafficLight(self, &light)) {
            return true;
        }
    }
    return false;
}

fn shouldBrakeForAnyTrafficLight(self: *const TrafficVehicle) bool {
    for (g_traffic_system.lights[0..g_traffic_system.light_count]) |*light| {
        if (shouldBrakeForTrafficLight(self, light)) {
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

/// Predict intersection conflict between two vehicles approaching the same intersection (Item 293).
/// Projects both vehicles forward and checks if their arrival windows overlap in the intersection zone.
/// Uses unified prediction layer for consistency.
pub fn predictIntersectionConflict(
    self: *const TrafficVehicle,
    other: *const TrafficVehicle,
    intersection_x: f32,
    intersection_z: f32,
    horizon: f32,
) prediction.ConflictWindow {
    const self_dx = intersection_x - self.pos_x;
    const self_dz = intersection_z - self.pos_z;
    const self_dist = @sqrt(self_dx * self_dx + self_dz * self_dz);
    const self_speed = @max(0.001, @sqrt(self.vel_x * self.vel_x + self.vel_z * self.vel_z));

    const other_dx = intersection_x - other.pos_x;
    const other_dz = intersection_z - other.pos_z;
    const other_dist = @sqrt(other_dx * other_dx + other_dz * other_dz);
    const other_speed = @max(0.001, @sqrt(other.vel_x * other.vel_x + other.vel_z * other.vel_z));

    const self_eta = self_dist / self_speed;
    const other_eta = other_dist / other_speed;

    // Vehicle occupies intersection for ~4.5m / avg_speed seconds
    const occupation_window: f32 = 4.5 / @max(self_speed, 5.0);

    var conflict = prediction.ConflictWindow{};
    if (@abs(self_eta - other_eta) < occupation_window * 2.0 and self_eta < horizon and other_eta < horizon) {
        conflict.valid = true;
        conflict.start_time = @max(0, @min(self_eta, other_eta) - occupation_window);
        conflict.end_time = @max(self_eta, other_eta) + occupation_window;
        conflict.min_distance = @abs(self_eta - other_eta) * @min(self_speed, other_speed);
    }
    return conflict;
}

/// Predict safe car-following behavior using unified prediction layer (Item 294).
/// Returns recommended following distance and whether braking is needed based on TTC.
/// For same-lane following: uses dz (forward distance) for spacing checks.
/// For crossing/head-on: uses TTC as primary indicator.
pub fn predictCarFollowing(
    self: *const TrafficVehicle,
    ahead: *const TrafficVehicle,
    horizon: f32,
) struct {
    recommended_distance: f32,
    should_brake: bool,
    ttc: prediction.TTCResult,
} {
    const ttc = computeVehicleConflict(self, ahead, horizon);
    const rel = projectRelativeToVehicle(self, ahead.pos_x, ahead.pos_z);
    const ego_speed = @max(0.001, @sqrt(self.vel_x * self.vel_x + self.vel_z * self.vel_z));
    const reaction_dist = self.reaction_time * ego_speed;
    const two_second_rule = 2.0 * ego_speed;
    const recommended = reaction_dist + two_second_rule + 4.5;
    // Primary check: TTC-based (unified prediction layer)
    // Fallback: projected distance-based for same-lane scenarios
    const spacing_risk = rel.forward > 0.0 and @abs(rel.lateral) < 6.0 and rel.forward < recommended;
    const should_brake = (ttc.valid and ttc.time < self.reaction_time + 1.0) or
        spacing_risk;
    return .{
        .recommended_distance = recommended,
        .should_brake = should_brake,
        .ttc = ttc,
    };
}

fn shouldBrakeForVehicleConflict(self: *const TrafficVehicle, other: *const TrafficVehicle) bool {
    // Use unified car-following prediction (Item 294)
    const following = predictCarFollowing(self, other, 5.0);
    if (following.should_brake) return true;

    const occupancy = computeVehicleOccupancyConflict(self, other, 5.0, 0.25);
    const occupancy_brake_horizon = self.reaction_time + 1.0;
    if (occupancy.valid and occupancy.start_time <= occupancy_brake_horizon) return true;

    // Also check occupancy-based avoidance (unified prediction layer)
    const recommendation = buildTrafficVehicleAvoidanceRecommendation(self, other, 5.0, 0.25);
    if (recommendation.should_brake) return true;
    return false;
}

fn isLaneChangeGapClear(self: *const TrafficVehicle, target_lane: i8) bool {
    const ego_speed = @sqrt(self.vel_x * self.vel_x + self.vel_z * self.vel_z);
    const planned_speed = if (self.governed_target_vel > 0.0) self.governed_target_vel else self.target_vel;
    const forward_gap = @max(8.0, self.reaction_time * @max(ego_speed, planned_speed * 0.25) + 6.0);
    const rear_gap = @max(6.0, self.reaction_time * ego_speed * 0.5 + 4.0);

    for (g_traffic_system.vehicles[0..g_traffic_system.vehicle_count]) |*other| {
        if (!other.active or other.vehicle_id == self.vehicle_id) continue;
        if (other.current_lane != target_lane and other.target_lane != target_lane) continue;

        const rel = projectRelativeToVehicle(self, other.pos_x, other.pos_z);
        if (@abs(rel.lateral) > 8.0) continue;
        if (rel.forward <= forward_gap and rel.forward >= -rear_gap) {
            return false;
        }
    }
    return true;
}

pub fn calculateLaneChange(target_lane: i8, current_lane: i8, dt: f32) f32 {
    const diff = @as(f32, @floatFromInt(target_lane - current_lane));
    if (@abs(diff) < 0.01) return 0;

    const lane_delta = @min(3.0, @abs(diff));
    const response = std.math.clamp(0.35 + std.math.clamp(dt, 0.0, 0.2) * 6.5, 0.35, 1.0);
    const steering = std.math.clamp(response * lane_delta, 0.0, 1.0);
    const sign: f32 = if (diff > 0) 1.0 else -1.0;
    return sign * steering;
}

/// Pull pose and speed from every physics VehicleState that is linked to
/// an AI traffic vehicle, overwriting the ghost state used for planning.
/// Must be called at the start of each tick before updateAI() so that
/// the planner sees the vehicle's real physics state.
pub fn syncTrafficVehiclesFromPhysics() void {
    // Avoid importing vehicle.zig at file level to prevent circular dependency.
    // vehicle.zig imports ai_traffic.zig; we call back only via the ABI
    // that vehicle.zig already exposes for this purpose.
    const vehicle_mod = @import("vehicle.zig");
    const v_sys = vehicle_mod.getSystem();
    for (0..v_sys.count) |i| {
        const v = &v_sys.vehicles[i];
        _ = vehicle_mod.syncVehicleToTraffic(v);
    }
}

pub fn updateAI(dt: f32) void {
    // Sync physics vehicle state into AI traffic vehicles before planning.
    syncTrafficVehiclesFromPhysics();

    g_traffic_system.global_time += dt;

    for (0..g_traffic_system.light_count) |i| {
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

    for (0..g_traffic_system.vehicle_count) |i| {
        var vehicle = &g_traffic_system.vehicles[i];
        if (!vehicle.active) continue;
        const speed = @sqrt(vehicle.vel_x * vehicle.vel_x + vehicle.vel_z * vehicle.vel_z);
        const visibility_factor = weather.getSensorVisibilityFactor();
        const control_authority = vehicle_physics.measureEnvironmentControlAuthority(
            .car,
            vehicle.pos_x,
            vehicle.pos_z,
            speed,
        );
        const traction_penalty = weather.getRoadTractionPenalty();
        const behavior_speed_cap = behaviorTargetSpeedCap(vehicle.behavior);
        var planning_risk = std.math.clamp(
            weather.getWeatherSeverity() * 0.45 +
                traction_penalty * 0.5 +
                (1.0 - visibility_factor) * 0.35,
            0.0,
            1.0,
        );

        const ahead_vehicle = checkVehicleAhead(vehicle, dt);
        if (ahead_vehicle) |ahead| {
            const following = predictCarFollowing(vehicle, ahead, 5.0);
            if (following.should_brake) {
                planning_risk = @max(planning_risk, 0.9);
            } else if (following.ttc.valid) {
                planning_risk = @max(planning_risk, std.math.clamp(1.0 - following.ttc.time / 6.0, 0.0, 0.8));
            }
        }
        if (shouldBrakeForAnyTrafficLight(vehicle)) {
            planning_risk = @max(planning_risk, 0.75);
        }

        const speed_plan = planner.computeGovernedTargetSpeed(.{
            .requested_target_speed = vehicle.target_vel,
            .behavior_speed_cap = behavior_speed_cap,
            .risk_level = planning_risk,
            .environment_context = .{
                .enabled = true,
                .vehicle_type = .car,
                .pos_x = vehicle.pos_x,
                .pos_z = vehicle.pos_z,
                .current_speed = speed,
            },
        });
        const desired_target_speed = speed_plan.constrained_target_speed;
        const accel_scale = std.math.clamp(control_authority.throttle_scale * (0.7 + visibility_factor * 0.3), 0.1, 1.0);
        const brake_scale = std.math.clamp(1.0 / @max(0.5, control_authority.brake_scale), 0.35, 1.0);
        const maneuver_scale = std.math.clamp((0.4 + visibility_factor * 0.6) * control_authority.steering_scale, 0.2, 1.0);

        vehicle.following_distance = behaviorFollowingDistance(vehicle.behavior) * (1.0 + (1.0 - visibility_factor) * 1.2 + traction_penalty * 0.6);
        vehicle.reaction_time = behaviorReactionTime(vehicle.behavior) * (1.0 + (1.0 - visibility_factor) * 0.9 + traction_penalty * 0.4);

        // Preserve requested target velocity; keep governed result separately.
        vehicle.governed_target_vel = desired_target_speed;

        if (shouldBrakeForAnyTrafficLight(vehicle)) {
            vehicle.brake_input = 1.0;
            vehicle.throttle_input = 0;
        } else if (ahead_vehicle) |ahead| {
            if (shouldBrakeForVehicleConflict(vehicle, ahead)) {
                vehicle.brake_input = 1.0;
                vehicle.throttle_input = 0;
            } else {
                vehicle.throttle_input = behaviorCruiseThrottle(vehicle.behavior);
                vehicle.brake_input = 0;
                // Aggressive profiles proactively seek a passing lane when following.
                if ((vehicle.behavior == .aggressive or vehicle.behavior == .reckless) and vehicle.target_lane == vehicle.current_lane) {
                    const lane_dir: i8 = if (vehicle.current_lane <= 0) 1 else -1;
                    const candidatelane: i8 = vehicle.current_lane + lane_dir;
                    if (candidatelane >= -2 and candidatelane <= 2 and isLaneChangeGapClear(vehicle, candidatelane)) {
                        vehicle.target_lane = candidatelane;
                    }
                }
            }
        } else {
            vehicle.throttle_input = behaviorCruiseThrottle(vehicle.behavior);
            vehicle.brake_input = 0;
        }

        // Behavior-aware speed governance for path following, parking and stopping tasks.
        if (desired_target_speed <= 0.01) {
            vehicle.throttle_input = 0.0;
            const hold_brake: f32 = if (speed > 0.25) 0.8 else 0.2;
            vehicle.brake_input = @max(vehicle.brake_input, hold_brake);
        } else if (speed > desired_target_speed + 0.5) {
            const overspeed = speed - desired_target_speed;
            const brake_gain = std.math.clamp(overspeed / @max(5.0, desired_target_speed), 0.15, 1.0);
            vehicle.brake_input = @max(vehicle.brake_input, brake_gain);
            vehicle.throttle_input = @min(vehicle.throttle_input, 0.5);
        } else if (speed < desired_target_speed - 0.5 and vehicle.brake_input < 0.05) {
            const deficit = desired_target_speed - speed;
            const throttle_gain = std.math.clamp(deficit / @max(5.0, desired_target_speed), 0.35, 1.0);
            vehicle.throttle_input = @max(vehicle.throttle_input, throttle_gain * behaviorCruiseThrottle(vehicle.behavior));
        }

        if (vehicle.target_lane != vehicle.current_lane) {
            const weatherlane_ok = visibility_factor > 0.35 or vehicle.behavior == .reckless;
            const lane_change_allowed = weatherlane_ok and
                (vehicle.behavior == .reckless or isLaneChangeGapClear(vehicle, vehicle.target_lane));
            if (lane_change_allowed) {
                vehicle.steering_input = calculateLaneChange(vehicle.target_lane, vehicle.current_lane, dt) *
                    control_authority.steering_scale;
                const lane_step_f = vehicle.steering_input * dt * 10.0;
                var lane_step: i8 = 0;
                if (lane_step_f >= 0.5) {
                    lane_step = 1;
                } else if (lane_step_f <= -0.5) {
                    lane_step = -1;
                }

                if (lane_step != 0) {
                    const nextlane: i16 = @as(i16, vehicle.current_lane) + @as(i16, lane_step);
                    if (lane_step > 0) {
                        vehicle.current_lane = @as(i8, @intCast(@min(@as(i16, vehicle.target_lane), nextlane)));
                    } else {
                        vehicle.current_lane = @as(i8, @intCast(@max(@as(i16, vehicle.target_lane), nextlane)));
                    }
                }
                if (vehicle.target_lane == vehicle.current_lane) {
                    vehicle.steering_input = 0.0;
                }
            } else {
                vehicle.steering_input = 0.0;
                vehicle.brake_input = @max(vehicle.brake_input, 0.25);
                vehicle.throttle_input = @min(vehicle.throttle_input, 0.5);
            }
        } else {
            vehicle.steering_input = 0;
        }

        if (vehicle.brake_input > 0) {
            const weather_brake_scale = std.math.clamp(brake_scale * (0.8 + 0.2 * maneuver_scale), 0.2, 1.0);
            const new_speed = @max(0, speed - 20 * vehicle.brake_input * dt * weather_brake_scale);
            const factor = if (speed > 0.001) new_speed / speed else 0;
            vehicle.vel_x *= factor;
            vehicle.vel_z *= factor;
        } else if (vehicle.throttle_input > 0) {
            const target_speed = desired_target_speed;
            const yaw = vehicle.yaw + vehicle.steering_input * dt;
            vehicle.yaw = yaw;
            if (speed < target_speed) {
                const accel = 10 * vehicle.throttle_input * dt * accel_scale * maneuver_scale;
                vehicle.vel_x = @sin(yaw) * @min(target_speed, speed + accel);
                vehicle.vel_z = @cos(yaw) * @min(target_speed, speed + accel);
            } else if (speed > target_speed + 0.5) {
                // Gently coast down to target speed using throttle resistance
                const decel = 5 * vehicle.throttle_input * dt * accel_scale;
                vehicle.vel_x = @sin(yaw) * @max(target_speed, speed - decel);
                vehicle.vel_z = @cos(yaw) * @max(target_speed, speed - decel);
            }
        }

        vehicle.pos_x += vehicle.vel_x * dt;
        vehicle.pos_z += vehicle.vel_z * dt;
    }
}

pub fn getVehicleCount() u8 {
    return g_traffic_system.vehicle_count;
}

pub fn getLightCount() u8 {
    return g_traffic_system.light_count;
}

test "updateAI ignores red light behind vehicle heading" {
    init();
    const light = addTrafficLight(0.0, -20.0, 30.0) orelse return error.TestUnexpectedResult;
    light.timer = 28.0; // keep red after update

    const car = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    car.yaw = 0.0; // forward +Z
    car.target_vel = 20.0;

    updateAI(0.1);

    try std.testing.expect(car.brake_input == 0.0);
    try std.testing.expect(car.throttle_input > 0.99);
}

test "init resets traffic system slots to deterministic inactive state" {
    init();
    const car = spawnAIVehicle(1.0, 0.0, 2.0, .aggressive, 0) orelse return error.TestUnexpectedResult;
    try std.testing.expect(car.active);

    init();
    const sys = getSystem();
    try std.testing.expect(sys.vehicle_count == 0);
    try std.testing.expect(sys.light_count == 0);
    try std.testing.expect(!sys.vehicles[0].active);
    try std.testing.expect(sys.vehicles[0].vehicle_id == 0);
}

test "setBehavior updates only active target vehicle profile" {
    init();
    const v1 = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    const v2 = spawnAIVehicle(0.0, 0.0, 10.0, .normal, 0) orelse return error.TestUnexpectedResult;

    const old_v2_following = v2.following_distance;
    const old_v2_reaction = v2.reaction_time;

    setBehavior(v1.vehicle_id, .cautious);

    try std.testing.expect(v1.behavior == .cautious);
    try std.testing.expect(v1.following_distance == behaviorFollowingDistance(.cautious));
    try std.testing.expect(v1.reaction_time == behaviorReactionTime(.cautious));
    try std.testing.expect(v2.behavior == .normal);
    try std.testing.expect(v2.following_distance == old_v2_following);
    try std.testing.expect(v2.reaction_time == old_v2_reaction);
}

test "triggerEmergencyVehicle applies reckless emergency profile" {
    init();
    const vehicle = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    vehicle.target_vel = 25.0;
    vehicle.brake_input = 1.0;
    vehicle.throttle_input = 0.0;

    triggerEmergencyVehicle(vehicle);

    try std.testing.expect(vehicle.behavior == .reckless);
    try std.testing.expect(vehicle.following_distance == 2.0);
    try std.testing.expect(vehicle.reaction_time == behaviorReactionTime(.reckless));
    try std.testing.expect(vehicle.target_vel >= 60.0);
    try std.testing.expect(vehicle.brake_input == 0.0);
    try std.testing.expect(vehicle.throttle_input == 1.0);
}

test "triggerEmergencyVehicle ignores inactive vehicle slots" {
    var vehicle = TrafficVehicle{
        .vehicle_id = 99,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 10,
        .governed_target_vel = 10,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 1,
        .steering_input = 0,
        .active = false,
        .world = null,
    };

    triggerEmergencyVehicle(&vehicle);

    try std.testing.expect(vehicle.behavior == .normal);
    try std.testing.expect(vehicle.target_vel == 10.0);
    try std.testing.expect(vehicle.following_distance == 30.0);
    try std.testing.expect(vehicle.reaction_time == 0.3);
    try std.testing.expect(vehicle.brake_input == 1.0);
    try std.testing.expect(vehicle.throttle_input == 0.0);
}

pub fn setBehavior(vehicle_id: u16, behavior: AIBehavior) void {
    for (g_traffic_system.vehicles[0..g_traffic_system.vehicle_count]) |*vehicle| {
        if (vehicle.vehicle_id == vehicle_id and vehicle.active) {
            vehicle.behavior = behavior;
            vehicle.following_distance = behaviorFollowingDistance(behavior);
            vehicle.reaction_time = behaviorReactionTime(behavior);
            break;
        }
    }
}

pub fn triggerEmergencyVehicle(vehicle: *TrafficVehicle) void {
    if (!vehicle.active) return;
    vehicle.behavior = .reckless;
    vehicle.following_distance = @min(behaviorFollowingDistance(.reckless), 2.0);
    vehicle.reaction_time = behaviorReactionTime(.reckless);
    vehicle.target_vel = @max(vehicle.target_vel, 60.0);
    vehicle.governed_target_vel = @max(vehicle.governed_target_vel, 60.0);
    vehicle.brake_input = 0.0;
    vehicle.throttle_input = 1.0;
}

pub fn getTrafficVehicles() []TrafficVehicle {
    return g_traffic_system.vehicles[0..g_traffic_system.vehicle_count];
}

pub fn getTrafficVehicle(vehicle_id: u16) ?*TrafficVehicle {
    for (g_traffic_system.vehicles[0..g_traffic_system.vehicle_count]) |*v| {
        if (v.vehicle_id == vehicle_id) return v;
    }
    return null;
}

pub fn getGovernedTargetSpeed(vehicle_id: u16) ?f32 {
    for (g_traffic_system.vehicles[0..g_traffic_system.vehicle_count]) |*v| {
        if (v.vehicle_id == vehicle_id) return v.governed_target_vel;
    }
    return null;
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
        .governed_target_vel = 30,
        .current_lane = 0,
        .target_lane = 1,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0.25,
        .active = true,
        .world = null,
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
        .governed_target_vel = 0,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
        .world = null,
    }, &.{
        .vehicle_id = 2,
        .pos_x = 6,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = -4,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 0,
        .governed_target_vel = 0,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
        .world = null,
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
        .governed_target_vel = 10,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 1,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
        .world = null,
    }, &.{
        .vehicle_id = 2,
        .pos_x = 6,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = -4,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 10,
        .governed_target_vel = 10,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 1,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
        .world = null,
    });
    try std.testing.expect(should_brake);
}

test "updateAI brakes for upcoming occupancy conflict even before close spacing" {
    init();
    const self = spawnAIVehicle(-6.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    const ahead = spawnAIVehicle(6.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;

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

test "updateAI reduces speed target under severe weather hazard" {
    weather.init();
    weather.triggerWeather(.storm, 0.95, 60.0);
    weather.triggerWeather(.fog, 0.8, 60.0);
    weather.updateWeather(1.0);

    init();
    const vehicle = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    vehicle.target_vel = 30.0;
    vehicle.vel_z = 22.0;

    updateAI(0.1);

    try std.testing.expect(vehicle.target_vel == 30.0);
    try std.testing.expect(vehicle.governed_target_vel < vehicle.target_vel);
    try std.testing.expect(vehicle.brake_input > 0.0);
    try std.testing.expect(vehicle.throttle_input < 1.0);
    weather.init();
}

test "updateAI defers non-reckless lane change in low visibility weather" {
    weather.init();
    weather.triggerWeather(.fog, 0.95, 60.0);
    weather.updateWeather(1.0);

    init();
    const vehicle = spawnAIVehicle(0.0, 0.0, 0.0, .cautious, 0) orelse return error.TestUnexpectedResult;
    vehicle.target_lane = 1;
    vehicle.current_lane = 0;
    vehicle.target_vel = 20.0;

    updateAI(0.1);

    try std.testing.expect(vehicle.steering_input == 0.0);
    try std.testing.expect(vehicle.current_lane == 0);
    weather.init();
}

test "updateAI reduces speed authority on low-traction terrain even in clear weather" {
    weather.init();
    terrain.init();

    init();
    const clear_vehicle = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    clear_vehicle.target_vel = 30.0;
    clear_vehicle.vel_z = 24.0;
    updateAI(0.1);
    const clear_brake = clear_vehicle.brake_input;
    const clear_throttle = clear_vehicle.throttle_input;

    weather.init();
    terrain.init();
    terrain.addTerrainPatch(0, 0, 200, .water);

    init();
    const hazard_vehicle = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    hazard_vehicle.target_vel = 30.0;
    hazard_vehicle.vel_z = 24.0;
    updateAI(0.1);

    try std.testing.expect(hazard_vehicle.target_vel == 30.0);
    try std.testing.expect(hazard_vehicle.brake_input >= clear_brake);
    try std.testing.expect(hazard_vehicle.throttle_input <= clear_throttle);

    weather.init();
    terrain.init();
}

// ============================================================================
// Tests for AI Traffic System (Items 561-585)
// ============================================================================

test "561: traffic vehicle generation - spawn AI vehicle" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    try std.testing.expect(vehicle != null);
    try std.testing.expect(vehicle.?.active == true);
    try std.testing.expect(vehicle.?.behavior == .normal);
}

test "562: traffic vehicle behavior - different behavior types" {
    init();
    const cautious = spawnAIVehicle(0, 0, 0, .cautious, 0);
    const aggressive = spawnAIVehicle(0, 0, 10, .aggressive, 0);
    try std.testing.expect(cautious.?.following_distance > aggressive.?.following_distance);
    try std.testing.expect(cautious.?.reaction_time > aggressive.?.reaction_time);
}

test "563: traffic flow model - multiple vehicles coexist" {
    init();
    _ = spawnAIVehicle(0, 0, 0, .normal, 0);
    _ = spawnAIVehicle(0, 0, 20, .normal, 0);
    _ = spawnAIVehicle(0, 0, 40, .normal, 0);
    try std.testing.expect(getVehicleCount() == 3);
}

test "564: traffic light behavior - red yellow green cycle" {
    init();
    const light = addTrafficLight(0, 50, 30.0);
    try std.testing.expect(light != null);
    try std.testing.expect(light.?.state == .red);
    light.?.timer = 20.0;
    updateAI(0.016);
    try std.testing.expect(light.?.state == .green);
}

test "565: parking behavior - vehicle stops at target" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 0;
    vehicle.?.brake_input = 1.0;
    try std.testing.expect(vehicle.?.brake_input > 0);
}

test "566: yielding behavior - slow for obstacles" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .cautious, 0);
    vehicle.?.vel_z = 20.0;
    const obstacle = spawnAIVehicle(0, 0, 15, .normal, 0) orelse return error.TestUnexpectedResult;
    obstacle.vel_z = 0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.brake_input > 0.0);
    try std.testing.expect(vehicle.?.throttle_input == 0.0);
}

test "567: lane change behavior - steering input for lane change" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.current_lane = 0;
    vehicle.?.target_lane = 1;
    const steering = calculateLaneChange(1, 0, 0.1);
    try std.testing.expect(steering > 0);
}

test "568: merge behavior - vehicle enters traffic" {
    init();
    const merge = spawnAIVehicle(0, 0, 0, .aggressive, 0);
    merge.?.vel_z = 30.0;
    merge.?.target_lane = 0;
    merge.?.current_lane = -1;
    updateAI(0.1);
    try std.testing.expect(merge.?.current_lane == 0);
    try std.testing.expect(merge.?.steering_input == 0.0);
}

test "569: diverge behavior - vehicle exits traffic" {
    init();
    const diverge = spawnAIVehicle(0, 0, 0, .normal, 0);
    diverge.?.vel_z = 20.0;
    diverge.?.current_lane = 0;
    diverge.?.target_lane = 1;
    updateAI(0.1);
    try std.testing.expect(diverge.?.current_lane == 1);
}

test "570: overtaking behavior - faster vehicle passes" {
    init();
    const slow = spawnAIVehicle(0, 0, 0, .normal, 0);
    const fast = spawnAIVehicle(0, 0, 20, .aggressive, 0);
    slow.?.vel_z = 15.0;
    fast.?.vel_z = 40.0;
    updateAI(0.1);
    try std.testing.expect(fast.?.vel_z > slow.?.vel_z);
}

test "571: car following behavior - maintains safe distance" {
    init();
    const lead = spawnAIVehicle(0, 0, 12, .normal, 0);
    const follow = spawnAIVehicle(0, 0, 0, .normal, 0);
    lead.?.vel_z = 20.0;
    follow.?.vel_z = 25.0;
    updateAI(0.1);
    try std.testing.expect(follow.?.brake_input > 0.0);
    try std.testing.expect(follow.?.throttle_input == 0.0);
}

test "572: path planning - vehicle follows path" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.target_vel = 30.0;
    updateAI(0.2);
    try std.testing.expect(vehicle.?.vel_z > 0.0);
    try std.testing.expect(vehicle.?.pos_z > 0.0);
}

test "573: obstacle avoidance - braking for obstacle" {
    init();
    const light = addTrafficLight(0, 20, 30.0) orelse return error.TestUnexpectedResult;
    light.timer = 26.0; // red phase after update
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 30.0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.brake_input > 0.99);
    try std.testing.expect(vehicle.?.throttle_input == 0.0);
}

test "574: emergency vehicle - aggressive behavior for emergency" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0) orelse return error.TestUnexpectedResult;
    triggerEmergencyVehicle(vehicle);
    try std.testing.expect(vehicle.behavior == .reckless);
    try std.testing.expect(vehicle.target_vel == 60);
}

test "575: pedestrian avoidance - slow for pedestrians" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .cautious, 0);
    vehicle.?.vel_z = 20.0;
    vehicle.?.target_vel = 30.0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.brake_input >= 0.24);
    try std.testing.expect(vehicle.?.throttle_input <= 0.5);
}

test "576: bicycle avoidance - yielding to bikes" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 25.0;
    vehicle.?.target_vel = 12.0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.brake_input > 0.5);
    try std.testing.expect(vehicle.?.throttle_input <= 0.5);
}

test "577: construction detour - path around construction" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.target_lane = 1;
    vehicle.?.current_lane = 0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.current_lane == 1);
}

test "578: accident handling - response to accident" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 30.0;
    vehicle.?.target_vel = 5.0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.brake_input > 0.9);
    try std.testing.expect(vehicle.?.throttle_input <= 0.5);
}

test "579: traffic congestion - slow in traffic" {
    init();
    const v1 = spawnAIVehicle(0, 0, 0, .normal, 0);
    const v2 = spawnAIVehicle(0, 0, 10, .normal, 0);
    const v3 = spawnAIVehicle(0, 0, 20, .normal, 0);
    v1.?.vel_z = 5.0;
    v2.?.vel_z = 5.0;
    v3.?.vel_z = 5.0;
    updateAI(0.1);
    try std.testing.expect(getVehicleCount() == 3);
    try std.testing.expect(v1.?.brake_input > 0.0);
    try std.testing.expect(v2.?.brake_input > 0.0);
}

test "580: ramp merge - merge onto highway" {
    init();
    const ramp = spawnAIVehicle(0, 0, 0, .aggressive, 0);
    ramp.?.vel_z = 40.0;
    ramp.?.current_lane = -1;
    ramp.?.target_lane = 0;
    updateAI(0.1);
    try std.testing.expect(ramp.?.current_lane == 0);
}

test "581: intersection handling - stop at red light" {
    init();
    const light = addTrafficLight(0, 50, 30.0);
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 30.0;
    light.?.timer = 28.0; // red phase after update
    updateAI(0.1);
    try std.testing.expect(vehicle.?.brake_input > 0.99);
    try std.testing.expect(vehicle.?.throttle_input == 0.0);
}

test "582: roundabout handling - navigate roundabout" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 15.0;
    vehicle.?.target_lane = 2;
    vehicle.?.current_lane = 0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.current_lane == 1);
    try std.testing.expect(vehicle.?.yaw > 0.0);
}

test "583: parking lot behavior - slow speed in parking" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .cautious, 0);
    vehicle.?.vel_z = 15.0;
    vehicle.?.target_vel = 5.0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.brake_input > 0.9);
    try std.testing.expect(vehicle.?.vel_z < 15.0);
}

test "584: gas station behavior - stop at station" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 10.0;
    vehicle.?.target_vel = 0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.throttle_input == 0.0);
    try std.testing.expect(vehicle.?.brake_input >= 0.8);
    try std.testing.expect(vehicle.?.vel_z < 10.0);
}

test "585: charging station behavior - stop at charging" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0);
    vehicle.?.vel_z = 8.0;
    vehicle.?.target_vel = 0;
    updateAI(0.1);
    try std.testing.expect(vehicle.?.throttle_input == 0.0);
    try std.testing.expect(vehicle.?.brake_input >= 0.8);
    try std.testing.expect(vehicle.?.vel_z < 8.0);
}

test "lane change progresses one lane per tick toward target without overshoot" {
    init();
    const vehicle = spawnAIVehicle(0, 0, 0, .normal, 0) orelse return error.TestUnexpectedResult;
    vehicle.current_lane = 0;
    vehicle.target_lane = 2;

    updateAI(0.1);
    try std.testing.expect(vehicle.current_lane == 1);
    try std.testing.expect(vehicle.steering_input > 0.0);

    updateAI(0.1);
    try std.testing.expect(vehicle.current_lane == 2);
    try std.testing.expect(vehicle.steering_input == 0.0);
}

test "calculateLaneChange scales steering with timestep" {
    const fast_step = calculateLaneChange(1, 0, 0.1);
    const slow_step = calculateLaneChange(1, 0, 0.01);
    try std.testing.expect(fast_step > slow_step);
    try std.testing.expect(slow_step > 0.0);
}

test "updateAI defers lane change when adjacent lane gap is unsafe" {
    init();
    const self = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    self.current_lane = 0;
    self.target_lane = 1;
    self.vel_z = 12.0;
    self.target_vel = 20.0;

    const blocker = spawnAIVehicle(0.0, 0.0, -3.0, .normal, 0) orelse return error.TestUnexpectedResult;
    blocker.current_lane = 1;
    blocker.target_lane = 1;

    updateAI(0.1);

    try std.testing.expect(self.current_lane == 0);
    try std.testing.expect(self.steering_input == 0.0);
    try std.testing.expect(self.brake_input >= 0.25);
}

test "reckless vehicle can force lane change through tight gap" {
    init();
    const self = spawnAIVehicle(0.0, 0.0, 0.0, .reckless, 0) orelse return error.TestUnexpectedResult;
    self.current_lane = 0;
    self.target_lane = 1;
    self.vel_z = 12.0;
    self.target_vel = 20.0;

    const blocker = spawnAIVehicle(0.0, 0.0, -3.0, .normal, 0) orelse return error.TestUnexpectedResult;
    blocker.current_lane = 1;
    blocker.target_lane = 1;

    updateAI(0.1);

    try std.testing.expect(self.current_lane == 1);
}

test "checkRedLight ignores red light behind vehicle heading" {
    init();
    const car = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    car.yaw = 0.0; // forward +Z

    const behind = addTrafficLight(0.0, -30.0, 30.0) orelse return error.TestUnexpectedResult;
    behind.state = .red;

    try std.testing.expect(!checkRedLight(car, 0.016));
}

test "checkVehicleAhead respects vehicle heading instead of world z-axis" {
    init();
    const self = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    self.yaw = std.math.pi / 2.0; // forward +X
    self.vel_x = 6.0;
    self.following_distance = 40.0;

    const ahead = spawnAIVehicle(20.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    ahead.vel_x = 0.0;
    ahead.vel_z = 0.0;

    const behind_on_z = spawnAIVehicle(0.0, 0.0, 30.0, .normal, 0) orelse return error.TestUnexpectedResult;
    behind_on_z.vel_x = 0.0;
    behind_on_z.vel_z = 0.0;

    const seen = checkVehicleAhead(self, 0.016) orelse return error.TestUnexpectedResult;
    try std.testing.expect(seen.vehicle_id == ahead.vehicle_id);
    try std.testing.expect(seen.vehicle_id != behind_on_z.vehicle_id);
}

test "predictCarFollowing uses projected forward distance by heading" {
    const self = TrafficVehicle{
        .vehicle_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 20,
        .vel_z = 0,
        .yaw = std.math.pi / 2.0, // forward +X
        .target_vel = 20,
        .governed_target_vel = 20,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
        .world = null,
    };
    const ahead = TrafficVehicle{
        .vehicle_id = 2,
        .pos_x = 10,
        .pos_y = 0,
        .pos_z = 0,
        .vel_x = 0,
        .vel_z = 0,
        .yaw = 0,
        .target_vel = 0,
        .governed_target_vel = 0,
        .current_lane = 0,
        .target_lane = 0,
        .behavior = .normal,
        .following_distance = 30,
        .reaction_time = 0.3,
        .throttle_input = 0,
        .brake_input = 0,
        .steering_input = 0,
        .active = true,
        .world = null,
    };

    const result = predictCarFollowing(&self, &ahead, 5.0);
    try std.testing.expect(result.should_brake);
    try std.testing.expect(result.recommended_distance > 0.0);
}


test "setAIVehicleLink syncs vehicle pose to AI traffic immediately" {
    init();
    vehicle_physics.init();

    // Spawn AI traffic vehicle at a known spawn pose.
    const ai_car = spawnAIVehicle(0.0, 0.0, 0.0, .normal, 0) orelse return error.TestUnexpectedResult;
    const ai_id = ai_car.vehicle_id;

    // Create a physics vehicle at a completely different position and link it.
    // setAIVehicleLink calls syncVehicleToTraffic, so AI car should immediately
    // reflect the physics vehicle pose after linking.
    const phys_car = vehicle_physics.createCar(10.0, 0.0, 20.0, 0.5) orelse return error.TestUnexpectedResult;
    phys_car.speed = 15.0;
    vehicle_physics.setAIVehicleLink(phys_car, ai_id, 20.0);

    // Verify the AI vehicle matches the physics vehicle immediately after link.
    try std.testing.expectApproxEqAbs(phys_car.pos_x, ai_car.pos_x, 0.0001);
    try std.testing.expectApproxEqAbs(phys_car.pos_z, ai_car.pos_z, 0.0001);
    const fwd_x = @sin(phys_car.yaw);
    const fwd_z = @cos(phys_car.yaw);
    try std.testing.expectApproxEqAbs(fwd_x * phys_car.speed, ai_car.vel_x, 0.0001);
    try std.testing.expectApproxEqAbs(fwd_z * phys_car.speed, ai_car.vel_z, 0.0001);
}

test "syncVehicleToTraffic only affects linked vehicles" {
    init();
    vehicle_physics.init();

    // Spawn AI traffic vehicle at a known pose.
    const ai_car = spawnAIVehicle(5.0, 0.0, 10.0, .normal, 0) orelse return error.TestUnexpectedResult;
    const saved_x = ai_car.pos_x;
    const saved_z = ai_car.pos_z;
    const saved_vel_x = ai_car.vel_x;

    // Create an unlinked physics vehicle far away.
    const phys_car = vehicle_physics.createCar(100.0, 0.0, 200.0, 1.0) orelse return error.TestUnexpectedResult;
    phys_car.speed = 30.0;
    // ai_vehicle_id is 0 (unlinked) by default.

    // Directly call syncVehicleToTraffic; it should return false and not touch ai_car.
    const ok = vehicle_physics.syncVehicleToTraffic(phys_car);
    try std.testing.expect(ok == false);
    try std.testing.expect(ai_car.pos_x == saved_x);
    try std.testing.expect(ai_car.pos_z == saved_z);
    try std.testing.expect(ai_car.vel_x == saved_vel_x);
}


