//! Shared collision event helpers.

const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const bus = @import("bus.zig");
const break_response = @import("break_response.zig");
const std = @import("std");

pub const PendingCollisionQueue = struct {
    events: [64]bus.CollisionPayload = undefined,
    entity_ids: [64]u16 = undefined,
    count: u8 = 0,

    pub fn clear(self: *PendingCollisionQueue) void {
        self.count = 0;
    }

    pub fn enqueueEntity(self: *PendingCollisionQueue, entity_id: u16, impact_velocity: i16, entity: *const entity16.Entity16) void {
        const payload = makeCollisionPayload(impact_velocity, entity);
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            if (self.entity_ids[i] != entity_id) continue;
            const existing = self.events[i];
            if (existing.impact_velocity == payload.impact_velocity and
                existing.hardness == payload.hardness and
                existing.did_break == payload.did_break)
            {
                return;
            }
        }

        if (self.count >= self.events.len) return;
        const idx = self.count;
        self.entity_ids[idx] = entity_id;
        self.events[idx] = payload;
        self.count += 1;
    }

    pub fn enqueuePair(
        self: *PendingCollisionQueue,
        instances: []const scene32.Instance,
        entities: []const entity16.Entity16,
        source_inst: *const scene32.Instance,
        impact_velocity: i16,
        blocker_id: u8,
    ) void {
        if (source_inst.entity_id >= entities.len) return;
        self.enqueueEntity(source_inst.entity_id, impact_velocity, &entities[source_inst.entity_id]);

        if (blocker_id == 255 or blocker_id >= instances.len) return;
        const target_inst = &instances[blocker_id];
        if (target_inst.entity_id >= entities.len) return;
        self.enqueueEntity(target_inst.entity_id, impact_velocity, &entities[target_inst.entity_id]);
    }

    pub fn publish(self: *PendingCollisionQueue, event_bus: *bus.Bus, tick_id: u32) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            event_bus.broadcastPhysicsEvent(self.entity_ids[i], tick_id, self.events[i]);
        }
        self.count = 0;
    }
};

pub const PendingSoundQueue = struct {
    events: [64]bus.SoundPayload = undefined,
    entity_ids: [64]u16 = undefined,
    count: u8 = 0,

    pub fn clear(self: *PendingSoundQueue) void {
        self.count = 0;
    }

    pub fn enqueueEntity(self: *PendingSoundQueue, entity_id: u16, payload: bus.SoundPayload) void {
        if (payload.sound_type == 0 or payload.volume <= 0.0 or payload.duration <= 0.0) return;

        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            if (self.entity_ids[i] != entity_id) continue;
            const existing = self.events[i];
            if (existing.sound_type == payload.sound_type and
                existing.volume == payload.volume and
                existing.pitch == payload.pitch and
                existing.duration == payload.duration)
            {
                return;
            }
        }

        if (self.count >= self.events.len) return;
        const idx = self.count;
        self.entity_ids[idx] = entity_id;
        self.events[idx] = payload;
        self.count += 1;
    }

    pub fn publish(self: *PendingSoundQueue, event_bus: *bus.Bus, tick_id: u32) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            event_bus.broadcastSoundEvent(self.entity_ids[i], tick_id, self.events[i]);
        }
        self.count = 0;
    }
};

pub const PendingParticleQueue = struct {
    events: [64]bus.ParticlePayload = undefined,
    entity_ids: [64]u16 = undefined,
    count: u8 = 0,

    pub fn clear(self: *PendingParticleQueue) void {
        self.count = 0;
    }

    pub fn enqueueEntity(self: *PendingParticleQueue, entity_id: u16, payload: bus.ParticlePayload) void {
        if (payload.particle_type == 0 or payload.intensity <= 0.0 or payload.duration <= 0.0) return;

        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            if (self.entity_ids[i] != entity_id) continue;
            const existing = self.events[i];
            if (existing.particle_type == payload.particle_type and
                existing.intensity == payload.intensity and
                existing.radius == payload.radius and
                existing.duration == payload.duration)
            {
                return;
            }
        }

        if (self.count >= self.events.len) return;
        const idx = self.count;
        self.entity_ids[idx] = entity_id;
        self.events[idx] = payload;
        self.count += 1;
    }

    pub fn publish(self: *PendingParticleQueue, event_bus: *bus.Bus, tick_id: u32) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            event_bus.broadcastParticleEvent(self.entity_ids[i], tick_id, self.events[i]);
        }
        self.count = 0;
    }
};

pub const PendingDeformationQueue = struct {
    events: [64]bus.DeformationPayload = undefined,
    entity_ids: [64]u16 = undefined,
    count: u8 = 0,

    pub fn clear(self: *PendingDeformationQueue) void {
        self.count = 0;
    }

    pub fn enqueueEntity(self: *PendingDeformationQueue, entity_id: u16, payload: bus.DeformationPayload) void {
        if (payload.total_depth <= 0.0) return;

        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            if (self.entity_ids[i] != entity_id) continue;
            const existing = self.events[i];
            if (existing.total_depth == payload.total_depth and
                existing.permanent_depth == payload.permanent_depth and
                existing.recovery_fraction == payload.recovery_fraction and
                existing.severe == payload.severe)
            {
                return;
            }
        }

        if (self.count >= self.events.len) return;
        const idx = self.count;
        self.entity_ids[idx] = entity_id;
        self.events[idx] = payload;
        self.count += 1;
    }

    pub fn publish(self: *PendingDeformationQueue, event_bus: *bus.Bus, tick_id: u32) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            event_bus.broadcastDeformationEvent(self.entity_ids[i], tick_id, self.events[i]);
        }
        self.count = 0;
    }
};

pub const PendingBreakQueue = struct {
    events: [64]bus.BreakPayload = undefined,
    entity_ids: [64]u16 = undefined,
    count: u8 = 0,

    pub fn clear(self: *PendingBreakQueue) void {
        self.count = 0;
    }

    pub fn enqueueEntity(self: *PendingBreakQueue, entity_id: u16, payload: bus.BreakPayload) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            if (self.entity_ids[i] != entity_id) continue;
            const existing = self.events[i];
            if (existing.impact_velocity == payload.impact_velocity and
                existing.hardness == payload.hardness and
                existing.fragment_count == payload.fragment_count)
            {
                return;
            }
        }

        if (self.count >= self.events.len) return;
        const idx = self.count;
        self.entity_ids[idx] = entity_id;
        self.events[idx] = payload;
        self.count += 1;
    }

    pub fn publish(self: *PendingBreakQueue, event_bus: *bus.Bus, tick_id: u32) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            event_bus.broadcastBreakEvent(self.entity_ids[i], tick_id, self.events[i]);
        }
        self.count = 0;
    }
};

pub const PendingJointBreakQueue = struct {
    events: [64]bus.JointBreakPayload = undefined,
    joint_ids: [64]u16 = undefined,
    count: u8 = 0,

    pub fn clear(self: *PendingJointBreakQueue) void {
        self.count = 0;
    }

    pub fn enqueueJoint(self: *PendingJointBreakQueue, joint_idx: u16, payload: bus.JointBreakPayload) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            if (self.joint_ids[i] == joint_idx) return;
        }

        if (self.count >= self.events.len) return;
        const idx = self.count;
        self.joint_ids[idx] = joint_idx;
        self.events[idx] = payload;
        self.count += 1;
    }

    pub fn publish(self: *PendingJointBreakQueue, event_bus: *bus.Bus, tick_id: u32) void {
        var i: u8 = 0;
        while (i < self.count) : (i += 1) {
            event_bus.broadcastJointBreakEvent(self.joint_ids[i], tick_id, self.events[i]);
        }
        self.count = 0;
    }
};

pub fn makeCollisionPayload(impact_velocity: i16, entity: *const entity16.Entity16) bus.CollisionPayload {
    const impact = break_response.calcImpactMagnitude(impact_velocity, entity.physics.mass);
    const break_result = physics.checkBreak(impact, entity.physics.material, entity.physics.hardness);
    return .{
        .impact_velocity = impact_velocity,
        .hardness = entity.physics.hardness,
        .did_break = break_result.did_break,
    };
}

pub fn broadcastCollisionPair(
    event_bus: *bus.Bus,
    tick_id: u32,
    instances: []const scene32.Instance,
    entities: []const entity16.Entity16,
    source_inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    var pending: PendingCollisionQueue = .{};
    pending.enqueuePair(instances, entities, source_inst, impact_velocity, blocker_id);
    pending.publish(event_bus, tick_id);
}

test "makeCollisionPayload mirrors break classification" {
    var fragile = entity16.initEntity16();
    fragile.physics.mass = 100;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 255;

    const payload = makeCollisionPayload(-60, &fragile);
    try std.testing.expectEqual(@as(i16, -60), payload.impact_velocity);
    try std.testing.expectEqual(@as(u16, 255), payload.hardness);
    try std.testing.expect(payload.did_break);
}

test "PendingCollisionQueue deduplicates identical entity payloads and publishes" {
    var entity = entity16.initEntity16();
    entity.physics.mass = 100;
    entity.physics.material = .solid;
    entity.physics.hardness = 200;

    var pending = PendingCollisionQueue{};
    pending.enqueueEntity(7, -10, &entity);
    pending.enqueueEntity(7, -10, &entity);
    pending.enqueueEntity(8, -10, &entity);
    try std.testing.expectEqual(@as(u8, 2), pending.count);

    var event_bus = bus.Bus.init();
    pending.publish(&event_bus, 99);
    try std.testing.expectEqual(@as(u8, 0), pending.count);
    try std.testing.expectEqual(@as(u16, 4), event_bus.msg_count);
    try std.testing.expectEqual(@as(u16, 7), event_bus.messages[0].entity_id);
    try std.testing.expectEqual(@as(u32, 99), event_bus.messages[0].tick_id);
}

test "PendingCollisionQueue enqueuePair ignores invalid blockers" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 100;
    entities[1].physics.mass = 100;

    const instances = [_]scene32.Instance{
        .{ .entity_id = 0, .pos_x = 0, .pos_y = 0, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .moving, .sleep_tick = 0, ._reserved = .{0} ** 2 },
        .{ .entity_id = 1, .pos_x = 1, .pos_y = 0, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 2 },
    };

    var pending = PendingCollisionQueue{};
    pending.enqueuePair(&instances, &entities, &instances[0], -20, 255);
    try std.testing.expectEqual(@as(u8, 1), pending.count);

    pending.clear();
    pending.enqueuePair(&instances, &entities, &instances[0], -20, 1);
    try std.testing.expectEqual(@as(u8, 2), pending.count);
}

test "pending effect queues filter invalid payloads and deduplicate" {
    var sound_queue = PendingSoundQueue{};
    sound_queue.enqueueEntity(1, .{ .sound_type = 0, .volume = 1.0, .pitch = 1.0, .duration = 1.0 });
    sound_queue.enqueueEntity(1, .{ .sound_type = 2, .volume = 1.0, .pitch = 1.0, .duration = 1.0 });
    sound_queue.enqueueEntity(1, .{ .sound_type = 2, .volume = 1.0, .pitch = 1.0, .duration = 1.0 });
    try std.testing.expectEqual(@as(u8, 1), sound_queue.count);

    var particle_queue = PendingParticleQueue{};
    particle_queue.enqueueEntity(2, .{ .particle_type = 0, .intensity = 1.0, .radius = 1.0, .duration = 1.0 });
    particle_queue.enqueueEntity(2, .{ .particle_type = 3, .intensity = 0.5, .radius = 1.0, .duration = 1.0 });
    particle_queue.enqueueEntity(2, .{ .particle_type = 3, .intensity = 0.5, .radius = 1.0, .duration = 1.0 });
    try std.testing.expectEqual(@as(u8, 1), particle_queue.count);

    var deformation_queue = PendingDeformationQueue{};
    deformation_queue.enqueueEntity(3, .{ .total_depth = 0.0, .permanent_depth = 0.0, .recovery_fraction = 1.0, .severe = false });
    deformation_queue.enqueueEntity(3, .{ .total_depth = 1.0, .permanent_depth = 0.5, .recovery_fraction = 0.5, .severe = true });
    deformation_queue.enqueueEntity(3, .{ .total_depth = 1.0, .permanent_depth = 0.5, .recovery_fraction = 0.5, .severe = true });
    try std.testing.expectEqual(@as(u8, 1), deformation_queue.count);
}

test "break queues deduplicate by entity or joint id" {
    var break_queue = PendingBreakQueue{};
    break_queue.enqueueEntity(4, .{ .impact_velocity = -100, .hardness = 10, .fragment_count = 3 });
    break_queue.enqueueEntity(4, .{ .impact_velocity = -100, .hardness = 10, .fragment_count = 3 });
    break_queue.enqueueEntity(4, .{ .impact_velocity = -101, .hardness = 10, .fragment_count = 3 });
    try std.testing.expectEqual(@as(u8, 2), break_queue.count);

    var joint_queue = PendingJointBreakQueue{};
    joint_queue.enqueueJoint(5, .{ .joint_idx = 5, .entity_a = 1, .entity_b = 2, .break_ratio = 1.0 });
    joint_queue.enqueueJoint(5, .{ .joint_idx = 5, .entity_a = 1, .entity_b = 2, .break_ratio = 0.5 });
    try std.testing.expectEqual(@as(u8, 1), joint_queue.count);
}
