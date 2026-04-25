//! Shared collision event helpers.

const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const physics = @import("physics.zig");
const bus = @import("bus.zig");
const break_response = @import("break_response.zig");

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
