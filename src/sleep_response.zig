//! Shared sleep / wake helpers.

const std = @import("std");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");

pub const SleepThresholds = struct {
    linear_velocity: i16 = 5,
    angular_velocity: i16 = 0,
    wake_linear_velocity: i16 = 5,
    wake_angular_velocity: i16 = 1,
    ground_vertical_velocity: i16 = 12,
    ground_lateral_velocity: i16 = 6,
    sleep_energy: f32 = 1250.0,
    sleep_stability: u16 = 95,
    sleep_ticks: u8 = 30,
};

pub const DEFAULT_SLEEP_THRESHOLDS = SleepThresholds{};
pub const SLEEP_VELOCITY_THRESHOLD: i16 = DEFAULT_SLEEP_THRESHOLDS.linear_velocity;
pub const SLEEP_ANGULAR_THRESHOLD: i16 = DEFAULT_SLEEP_THRESHOLDS.angular_velocity;
pub const WAKE_VELOCITY_THRESHOLD: i16 = DEFAULT_SLEEP_THRESHOLDS.wake_linear_velocity;
pub const WAKE_ANGULAR_THRESHOLD: i16 = DEFAULT_SLEEP_THRESHOLDS.wake_angular_velocity;
pub const GROUND_SETTLE_VERTICAL_THRESHOLD: i16 = DEFAULT_SLEEP_THRESHOLDS.ground_vertical_velocity;
pub const GROUND_SETTLE_LATERAL_THRESHOLD: i16 = DEFAULT_SLEEP_THRESHOLDS.ground_lateral_velocity;
pub const SLEEP_ENERGY_THRESHOLD: f32 = DEFAULT_SLEEP_THRESHOLDS.sleep_energy;
pub const SLEEP_STABILITY_THRESHOLD: u16 = DEFAULT_SLEEP_THRESHOLDS.sleep_stability;
pub const SLEEP_TIME_THRESHOLD: u8 = DEFAULT_SLEEP_THRESHOLDS.sleep_ticks;

pub const SleepIsland = struct {
    members: [scene32.MAX_INSTANCES]u8 = [_]u8{0} ** scene32.MAX_INSTANCES,
    member_count: usize = 0,
    pair_count: usize = 0,
    total_energy: f32 = 0.0,
    min_stability: u16 = std.math.maxInt(u16),
    can_sleep: bool = true,
};

pub fn shouldSleepWithThresholds(inst: *const scene32.Instance, thresholds: SleepThresholds) bool {
    const speed = @abs(inst.vel_x) + @abs(inst.vel_y) + @abs(inst.vel_z);
    const ang_speed = @abs(inst.ang_x) + @abs(inst.ang_y) + @abs(inst.ang_z);
    return speed < thresholds.linear_velocity and ang_speed <= thresholds.angular_velocity and inst.state != .broken;
}

pub fn shouldSleep(inst: *const scene32.Instance) bool {
    return shouldSleepWithThresholds(inst, DEFAULT_SLEEP_THRESHOLDS);
}

pub fn computeSleepEnergy(inst: *const scene32.Instance, entity: *const entity16.Entity16) f32 {
    if (entity.physics.mass == 0 or (entity.physics.flags & 0x01) != 0) return 0.0;

    const mass: f32 = @floatFromInt(entity.physics.mass);
    const vx: f32 = @floatFromInt(inst.vel_x);
    const vy: f32 = @floatFromInt(inst.vel_y);
    const vz: f32 = @floatFromInt(inst.vel_z);
    const ax: f32 = @floatFromInt(inst.ang_x);
    const ay: f32 = @floatFromInt(inst.ang_y);
    const az: f32 = @floatFromInt(inst.ang_z);
    const linear_energy = 0.5 * mass * (vx * vx + vy * vy + vz * vz);
    const angular_energy = 0.5 * @max(1.0, mass / 16.0) * (ax * ax + ay * ay + az * az);
    return linear_energy + angular_energy;
}

pub fn computeSleepStability(inst: *const scene32.Instance, entity: *const entity16.Entity16) u16 {
    if (entity.physics.mass == 0 or (entity.physics.flags & 0x01) != 0) return std.math.maxInt(u16);
    if (inst.state == .broken) return 0;

    const speed_penalty: u32 = @intCast(@abs(inst.vel_x) + @abs(inst.vel_y) + @abs(inst.vel_z));
    const angular_penalty: u32 = @intCast(@abs(inst.ang_x) + @abs(inst.ang_y) + @abs(inst.ang_z));
    const total_penalty = speed_penalty + angular_penalty * 2;
    const base: u32 = entity.physics.stability;
    if (total_penalty >= base) return 0;
    return @intCast(base - total_penalty);
}

pub fn isSleepStableWithThresholds(
    inst: *const scene32.Instance,
    entity: *const entity16.Entity16,
    thresholds: SleepThresholds,
) bool {
    return computeSleepStability(inst, entity) >= thresholds.sleep_stability;
}

pub fn isSleepStable(inst: *const scene32.Instance, entity: *const entity16.Entity16) bool {
    return isSleepStableWithThresholds(inst, entity, DEFAULT_SLEEP_THRESHOLDS);
}

pub fn shouldSleepInstanceWithThresholds(
    inst: *const scene32.Instance,
    entity: *const entity16.Entity16,
    thresholds: SleepThresholds,
) bool {
    return shouldSleepWithThresholds(inst, thresholds) and
        computeSleepEnergy(inst, entity) <= thresholds.sleep_energy and
        isSleepStableWithThresholds(inst, entity, thresholds);
}

pub fn shouldSleepInstance(inst: *const scene32.Instance, entity: *const entity16.Entity16) bool {
    return shouldSleepInstanceWithThresholds(inst, entity, DEFAULT_SLEEP_THRESHOLDS);
}

fn isValidSleepIslandInstance(
    instances: []const scene32.Instance,
    entities: []const entity16.Entity16,
    instance_idx: u8,
) bool {
    if (instance_idx >= instances.len) return false;
    const inst = &instances[instance_idx];
    if (inst.state == .broken) return false;
    return inst.entity_id < entities.len;
}

fn sleepIslandContains(island: *const SleepIsland, instance_idx: u8) bool {
    for (island.members[0..island.member_count]) |member| {
        if (member == instance_idx) return true;
    }
    return false;
}

fn summarizeSleepIsland(
    island: *SleepIsland,
    instances: []const scene32.Instance,
    entities: []const entity16.Entity16,
    pairs: anytype,
    thresholds: SleepThresholds,
) void {
    island.pair_count = 0;
    island.total_energy = 0.0;
    island.min_stability = std.math.maxInt(u16);
    island.can_sleep = true;

    for (island.members[0..island.member_count]) |member| {
        const inst = &instances[member];
        const entity = &entities[inst.entity_id];
        const stability = computeSleepStability(inst, entity);
        island.total_energy += computeSleepEnergy(inst, entity);
        island.min_stability = @min(island.min_stability, stability);
        island.can_sleep = island.can_sleep and shouldSleepInstanceWithThresholds(inst, entity, thresholds);
    }

    for (pairs) |pair| {
        if (sleepIslandContains(island, pair.a) and sleepIslandContains(island, pair.b)) {
            island.pair_count += 1;
        }
    }
}

fn buildSleepIslandFrom(
    instances: []const scene32.Instance,
    entities: []const entity16.Entity16,
    pairs: anytype,
    start_idx: u8,
    assigned: []bool,
    out_island: *SleepIsland,
    thresholds: SleepThresholds,
) void {
    var queue: [scene32.MAX_INSTANCES]u8 = undefined;
    var queue_head: usize = 0;
    var queue_tail: usize = 0;

    out_island.* = .{};
    assigned[start_idx] = true;
    queue[queue_tail] = start_idx;
    queue_tail += 1;

    while (queue_head < queue_tail) : (queue_head += 1) {
        const current = queue[queue_head];
        out_island.members[out_island.member_count] = current;
        out_island.member_count += 1;

        for (pairs) |pair| {
            var neighbor: ?u8 = null;
            if (pair.a == current) neighbor = pair.b;
            if (pair.b == current) neighbor = pair.a;
            if (neighbor) |next_idx| {
                if (!isValidSleepIslandInstance(instances, entities, next_idx)) continue;
                if (assigned[next_idx]) continue;
                assigned[next_idx] = true;
                queue[queue_tail] = next_idx;
                queue_tail += 1;
            }
        }
    }

    summarizeSleepIsland(out_island, instances, entities, pairs, thresholds);
}

pub fn detectSleepIslandsWithThresholds(
    instances: []const scene32.Instance,
    entities: []const entity16.Entity16,
    pairs: anytype,
    out_islands: []SleepIsland,
    thresholds: SleepThresholds,
) usize {
    var assigned: [scene32.MAX_INSTANCES]bool = [_]bool{false} ** scene32.MAX_INSTANCES;
    var island_count: usize = 0;
    const instance_count = @min(instances.len, scene32.MAX_INSTANCES);

    var idx: usize = 0;
    while (idx < instance_count) : (idx += 1) {
        const instance_idx: u8 = @intCast(idx);
        if (assigned[instance_idx]) continue;
        if (!isValidSleepIslandInstance(instances, entities, instance_idx)) continue;
        if (island_count >= out_islands.len) break;
        buildSleepIslandFrom(
            instances,
            entities,
            pairs,
            instance_idx,
            assigned[0..],
            &out_islands[island_count],
            thresholds,
        );
        island_count += 1;
    }

    return island_count;
}

pub fn detectSleepIslands(
    instances: []const scene32.Instance,
    entities: []const entity16.Entity16,
    pairs: anytype,
    out_islands: []SleepIsland,
) usize {
    return detectSleepIslandsWithThresholds(instances, entities, pairs, out_islands, DEFAULT_SLEEP_THRESHOLDS);
}

pub fn shouldWakeWithThresholds(inst: *const scene32.Instance, moved: bool, thresholds: SleepThresholds) bool {
    if (inst.state == .broken) return false;
    if (moved) return true;

    const speed = @abs(inst.vel_x) + @abs(inst.vel_y) + @abs(inst.vel_z);
    const ang_speed = @abs(inst.ang_x) + @abs(inst.ang_y) + @abs(inst.ang_z);
    return speed >= thresholds.wake_linear_velocity or ang_speed >= thresholds.wake_angular_velocity;
}

pub fn shouldWake(inst: *const scene32.Instance, moved: bool) bool {
    return shouldWakeWithThresholds(inst, moved, DEFAULT_SLEEP_THRESHOLDS);
}

pub fn wakeInstance(inst: *scene32.Instance) void {
    inst.sleep_tick = 0;
    if (inst.state == .resting) {
        inst.state = .idle;
    }
}

pub fn wakeInstanceForMotion(inst: *scene32.Instance, moved: bool) bool {
    if (!shouldWake(inst, moved)) return false;
    wakeInstance(inst);
    return true;
}

pub fn wakeSupportedInstancesAfterBreak(
    instances: []scene32.Instance,
    entities: []const entity16.Entity16,
    broken_idx: u8,
) void {
    if (broken_idx >= instances.len) return;
    const broken_inst = &instances[broken_idx];
    if (broken_inst.entity_id >= entities.len) return;
    const broken_entity = &entities[broken_inst.entity_id];

    var other_idx: usize = 0;
    while (other_idx < instances.len) : (other_idx += 1) {
        if (other_idx == broken_idx) continue;
        const other = &instances[other_idx];
        if (other.state != .resting) continue;
        if (other.entity_id >= entities.len) continue;
        const other_entity = &entities[other.entity_id];

        var supported = false;
        for (0..64) |broken_w_idx| {
            const broken_word = broken_entity.topology[broken_w_idx];
            if (broken_word == 0) continue;
            for (0..64) |broken_b_idx| {
                if ((broken_word & (@as(u64, 1) << @as(u6, @truncate(broken_b_idx)))) == 0) continue;
                const broken_local = (broken_w_idx << 6) | broken_b_idx;
                const broken_x: i32 = @intCast((broken_local >> 4) & 0xF);
                const broken_y: i32 = @intCast(broken_local >> 8);
                const broken_z: i32 = @intCast(broken_local & 0xF);
                const support_x = broken_inst.pos_x + broken_x;
                const support_y = broken_inst.pos_y + broken_y + 1;
                const support_z = broken_inst.pos_z + broken_z;

                for (0..64) |other_w_idx| {
                    const other_word = other_entity.topology[other_w_idx];
                    if (other_word == 0) continue;
                    for (0..64) |other_b_idx| {
                        if ((other_word & (@as(u64, 1) << @as(u6, @truncate(other_b_idx)))) == 0) continue;
                        const other_local = (other_w_idx << 6) | other_b_idx;
                        const other_x: i32 = @intCast((other_local >> 4) & 0xF);
                        const other_y: i32 = @intCast(other_local >> 8);
                        const other_z: i32 = @intCast(other_local & 0xF);
                        if (other.pos_x + other_x == support_x and
                            other.pos_y + other_y == support_y and
                            other.pos_z + other_z == support_z)
                        {
                            supported = true;
                            break;
                        }
                    }
                    if (supported) break;
                }
                if (supported) break;
            }
            if (supported) break;
        }

        if (supported) {
            wakeInstance(other);
        }
    }
}

test "sleep thresholds expose shared defaults" {
    try std.testing.expectEqual(@as(i16, 5), SLEEP_VELOCITY_THRESHOLD);
    try std.testing.expectEqual(@as(i16, 0), SLEEP_ANGULAR_THRESHOLD);
    try std.testing.expectEqual(@as(i16, 5), WAKE_VELOCITY_THRESHOLD);
    try std.testing.expectEqual(@as(i16, 1), WAKE_ANGULAR_THRESHOLD);
    try std.testing.expectEqual(@as(i16, 12), GROUND_SETTLE_VERTICAL_THRESHOLD);
    try std.testing.expectEqual(@as(i16, 6), GROUND_SETTLE_LATERAL_THRESHOLD);
    try std.testing.expectApproxEqAbs(@as(f32, 1250.0), SLEEP_ENERGY_THRESHOLD, 0.0001);
    try std.testing.expectEqual(@as(u16, 95), SLEEP_STABILITY_THRESHOLD);
    try std.testing.expectEqual(@as(u8, 30), SLEEP_TIME_THRESHOLD);
}

test "shouldSleepWithThresholds applies linear and angular gates" {
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 2,
        .vel_y = 1,
        .vel_z = 1,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expect(shouldSleep(&inst));

    inst.vel_x = 5;
    try std.testing.expect(!shouldSleep(&inst));

    inst.vel_x = 0;
    inst.ang_y = 1;
    try std.testing.expect(!shouldSleep(&inst));
    try std.testing.expect(shouldSleepWithThresholds(&inst, .{ .angular_velocity = 1 }));

    inst.state = .broken;
    try std.testing.expect(!shouldSleepWithThresholds(&inst, .{ .angular_velocity = 1 }));
}

test "shouldWakeWithThresholds applies motion and velocity gates" {
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = SLEEP_TIME_THRESHOLD,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expect(!shouldWake(&inst, false));
    try std.testing.expect(shouldWake(&inst, true));

    inst.vel_x = 5;
    try std.testing.expect(shouldWake(&inst, false));

    inst.vel_x = 0;
    inst.ang_y = 1;
    try std.testing.expect(shouldWake(&inst, false));

    inst.state = .broken;
    try std.testing.expect(!shouldWake(&inst, true));
}

test "wakeInstanceForMotion preserves resting state below wake threshold" {
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = SLEEP_TIME_THRESHOLD,
        .vel_x = 1,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expect(!wakeInstanceForMotion(&inst, false));
    try std.testing.expectEqual(scene32.InstanceState.resting, inst.state);
    try std.testing.expectEqual(SLEEP_TIME_THRESHOLD, inst.sleep_tick);

    inst.vel_x = WAKE_VELOCITY_THRESHOLD;
    try std.testing.expect(wakeInstanceForMotion(&inst, false));
    try std.testing.expectEqual(scene32.InstanceState.idle, inst.state);
    try std.testing.expectEqual(@as(u8, 0), inst.sleep_tick);
}

test "computeSleepEnergy includes mass-weighted linear and angular energy" {
    var entity = entity16.initEntity16();
    entity.physics.mass = 20;
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 3,
        .vel_y = 4,
        .vel_z = 0,
        .ang_x = 2,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expectApproxEqAbs(@as(f32, 252.5), computeSleepEnergy(&inst, &entity), 0.0001);
}

test "shouldSleepInstance rejects low-speed heavy bodies above energy threshold" {
    var heavy = entity16.initEntity16();
    heavy.physics.mass = 1000;
    var light = entity16.initEntity16();
    light.physics.mass = 10;
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 4,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expect(shouldSleepWithThresholds(&inst, DEFAULT_SLEEP_THRESHOLDS));
    try std.testing.expect(shouldSleepInstance(&inst, &light));
    try std.testing.expect(!shouldSleepInstance(&inst, &heavy));
}

test "computeSleepStability applies entity stability and residual motion penalties" {
    var entity = entity16.initEntity16();
    entity.physics.mass = 10;
    entity.physics.stability = 100;
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        .vel_x = 3,
        .vel_y = 1,
        .vel_z = 0,
        .ang_x = 2,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expectEqual(@as(u16, 92), computeSleepStability(&inst, &entity));
    try std.testing.expect(!isSleepStableWithThresholds(&inst, &entity, .{ .sleep_stability = 93 }));

    inst.state = .broken;
    try std.testing.expectEqual(@as(u16, 0), computeSleepStability(&inst, &entity));
}

test "shouldSleepInstance rejects unstable dynamic bodies" {
    var entity = entity16.initEntity16();
    entity.physics.mass = 10;
    entity.physics.stability = 94;
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
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

    try std.testing.expect(!shouldSleepInstance(&inst, &entity));

    entity.physics.stability = 95;
    try std.testing.expect(shouldSleepInstance(&inst, &entity));
}

test "static bodies are sleep-stable regardless of stability score" {
    var entity = entity16.initEntity16();
    entity.physics.mass = 0;
    entity.physics.stability = 0;
    var inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
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

    try std.testing.expectEqual(std.math.maxInt(u16), computeSleepStability(&inst, &entity));
    try std.testing.expect(shouldSleepInstance(&inst, &entity));
}

test "detectSleepIslands builds connected sleep candidates from pairs" {
    const Pair = struct {
        a: u8,
        b: u8,
    };

    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;
    entities[2].physics.mass = 10;

    var instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
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
        },
        .{
            .entity_id = 1,
            .pos_x = 1,
            .pos_y = 0,
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
        },
        .{
            .entity_id = 2,
            .pos_x = 8,
            .pos_y = 0,
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
        },
    };
    const pairs = [_]Pair{.{ .a = 0, .b = 1 }};
    var islands: [3]SleepIsland = undefined;

    const count = detectSleepIslands(instances[0..], entities[0..], pairs[0..], islands[0..]);
    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 2), islands[0].member_count);
    try std.testing.expectEqual(@as(usize, 1), islands[0].pair_count);
    try std.testing.expect(islands[0].can_sleep);
    try std.testing.expectEqual(@as(usize, 1), islands[1].member_count);
    try std.testing.expectEqual(@as(usize, 0), islands[1].pair_count);
    try std.testing.expect(islands[1].can_sleep);
}

test "detectSleepIslands marks whole island awake when any member cannot sleep" {
    const Pair = struct {
        a: u8,
        b: u8,
    };

    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 1000;

    var instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
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
        },
        .{
            .entity_id = 1,
            .pos_x = 1,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .idle,
            .sleep_tick = 0,
            .vel_x = 4,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    const pairs = [_]Pair{.{ .a = 0, .b = 1 }};
    var islands: [2]SleepIsland = undefined;

    const count = detectSleepIslands(instances[0..], entities[0..], pairs[0..], islands[0..]);
    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(usize, 2), islands[0].member_count);
    try std.testing.expectEqual(@as(usize, 1), islands[0].pair_count);
    try std.testing.expect(islands[0].total_energy > SLEEP_ENERGY_THRESHOLD);
    try std.testing.expect(!islands[0].can_sleep);
}
