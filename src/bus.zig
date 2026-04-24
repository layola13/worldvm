//! bus.zig - Tri-World Bus Synchronization
//! Physical World <-> Psychological World <-> Programming World
//! Follows doc 28: Three-World Paging and Bus Sync

const std = @import("std");
const mind = @import("mind.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");

pub const WorldType = mind.WorldType;

pub const BusMessageType = enum(u8) {
    PHYSICS_EVENT = 0,
    AFFECT_EVENT = 1,
    RULE_UPDATE = 2,
};

pub const BusPriority = enum(u8) {
    LOW = 0,
    NORMAL = 128,
    HIGH = 200,
    CRITICAL = 250,
};

pub const BusMessage = struct {
    msg_type: BusMessageType,
    source_world: WorldType,
    target_world: WorldType,
    entity_id: u16,
    tick_id: u32,
    priority: BusPriority,
    payload: Payload,
    handled: bool = false,

    pub const Payload = union(enum(u8)) {
        none: void,
        collision: CollisionPayload,
        affect_change: AffectPayload,
        rule_change: RulePayload,
    };
};

pub const CollisionPayload = struct {
    impact_velocity: i16,
    hardness: u16,
    did_break: bool,
};

pub const AffectPayload = struct {
    valence_delta: i8,
    arousal_delta: u8,
    certainty_delta: u8,
};

pub const RulePayload = struct {
    rule_id: u16,
    new_threshold: u16,
};

pub const Bus = struct {
    messages: [256]BusMessage,
    msg_count: u16,
    total_dispatched: u32,

    pub fn init() Bus {
        return .{
            .messages = undefined,
            .msg_count = 0,
            .total_dispatched = 0,
        };
    }

    pub fn dispatch(self: *Bus, msg: BusMessage) void {
        self.total_dispatched += 1;
        if (self.msg_count > 255) return; // u8 would overflow on +=1 past 255
        self.messages[self.msg_count] = msg;
        self.msg_count += 1;
    }

    pub fn broadcastPhysicsEvent(self: *Bus, entity_id: u16, tick_id: u32, collision: CollisionPayload) void {
        self.dispatch(.{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (collision.did_break) .HIGH else .NORMAL,
            .payload = .{ .collision = collision },
        });
        self.dispatch(.{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (collision.did_break) .HIGH else .NORMAL,
            .payload = .{ .collision = collision },
        });
    }

    pub fn broadcastAffectEvent(self: *Bus, entity_id: u16, tick_id: u32, affect: AffectPayload) void {
        self.dispatch(.{
            .msg_type = .AFFECT_EVENT,
            .source_world = .psychological,
            .target_world = .physical,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = .NORMAL,
            .payload = .{ .affect_change = affect },
        });
    }

    pub fn processQueue(self: *Bus) u8 {
        var handled: u8 = 0;
        for (0..self.msg_count) |i| {
            if (self.messages[i].handled) continue;
            self.messages[i].handled = true;
            handled += 1;
        }
        self.msg_count = 0;
        return handled;
    }

    pub fn clear(self: *Bus) void {
        self.msg_count = 0;
    }
};

pub const TriWorldState = struct {
    bus: Bus,
    physical_tick: u32 = 0,
    psych_valence: i16 = 0, // Global mood
    
    pub fn init() TriWorldState {
        return .{ .bus = Bus.init() };
    }
};

test "Bus init and dispatch" {
    const b = Bus.init();
    try std.testing.expect(b.msg_count == 0);
    try std.testing.expect(b.total_dispatched == 0);
}

test "Bus dispatch stores message" {
    var b = Bus.init();
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 5,
        .tick_id = 10,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });
    try std.testing.expect(b.msg_count == 1);
    try std.testing.expect(b.total_dispatched == 1);
    try std.testing.expect(b.messages[0].entity_id == 5);
}

test "Bus broadcast physics event to two worlds" {
    var b = Bus.init();
    b.broadcastPhysicsEvent(1, 5, .{ .impact_velocity = -100, .hardness = 30, .did_break = true });
    try std.testing.expect(b.msg_count == 2);
    try std.testing.expect(b.messages[0].target_world == .psychological);
    try std.testing.expect(b.messages[1].target_world == .programming);
    try std.testing.expect(b.messages[0].priority == .HIGH);
}

test "Bus processQueue clears and returns count" {
    var b = Bus.init();
    b.dispatch(.{
        .msg_type = .AFFECT_EVENT,
        .source_world = .psychological,
        .target_world = .physical,
        .entity_id = 2,
        .tick_id = 3,
        .priority = .NORMAL,
        .payload = .{ .none = {} },
    });
    const handled = b.processQueue();
    try std.testing.expect(handled == 1);
    try std.testing.expect(b.msg_count == 0);
}

test "Bus does not overflow" {
    var b = Bus.init();
    var i: u16 = 0;
    while (i < 260) : (i += 1) {
        b.dispatch(.{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = @intCast(i),
            .tick_id = 0,
            .priority = .LOW,
            .payload = .{ .none = {} },
        });
    }
    try std.testing.expect(b.msg_count == 256);
    try std.testing.expect(b.total_dispatched == 260);
}
