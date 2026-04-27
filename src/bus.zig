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
    SOUND_EVENT = 3,
    PARTICLE_EVENT = 4,
    DEFORMATION_EVENT = 5,
    BREAK_EVENT = 6,
};

const ALL_EVENT_TYPE_MASK: u16 =
    eventTypeMask(.PHYSICS_EVENT) |
    eventTypeMask(.AFFECT_EVENT) |
    eventTypeMask(.RULE_UPDATE) |
    eventTypeMask(.SOUND_EVENT) |
    eventTypeMask(.PARTICLE_EVENT) |
    eventTypeMask(.DEFORMATION_EVENT) |
    eventTypeMask(.BREAK_EVENT);

pub const BusPriority = enum(u8) {
    LOW = 0,
    NORMAL = 128,
    HIGH = 200,
    CRITICAL = 250,
};

const ALL_WORLD_TYPE_MASK: u16 =
    worldTypeMask(.physical) |
    worldTypeMask(.psychological) |
    worldTypeMask(.programming);

fn eventTypeMask(msg_type: BusMessageType) u16 {
    return @as(u16, 1) << @as(u4, @intCast(@intFromEnum(msg_type)));
}

fn worldTypeMask(world_type: WorldType) u16 {
    return @as(u16, 1) << @as(u4, @intCast(@intFromEnum(world_type)));
}

pub const BusEventFilter = struct {
    enabled: bool = false,
    type_mask: u16 = ALL_EVENT_TYPE_MASK,
    source_world_mask: u16 = ALL_WORLD_TYPE_MASK,
    target_world_mask: u16 = ALL_WORLD_TYPE_MASK,
    min_priority: BusPriority = .LOW,
    entity_id: ?u16 = null,

    pub fn allowAll() BusEventFilter {
        return .{};
    }

    pub fn onlyType(msg_type: BusMessageType) BusEventFilter {
        return .{
            .enabled = true,
            .type_mask = eventTypeMask(msg_type),
        };
    }

    pub fn onlyTarget(target_world: WorldType) BusEventFilter {
        return .{
            .enabled = true,
            .target_world_mask = worldTypeMask(target_world),
        };
    }

    pub fn allows(self: BusEventFilter, msg: BusMessage) bool {
        if (!self.enabled) return true;
        if ((self.type_mask & eventTypeMask(msg.msg_type)) == 0) return false;
        if ((self.source_world_mask & worldTypeMask(msg.source_world)) == 0) return false;
        if ((self.target_world_mask & worldTypeMask(msg.target_world)) == 0) return false;
        if (@intFromEnum(msg.priority) < @intFromEnum(self.min_priority)) return false;
        if (self.entity_id) |allowed_entity_id| {
            if (msg.entity_id != allowed_entity_id) return false;
        }
        return true;
    }
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
        sound: SoundPayload,
        particle: ParticlePayload,
        deformation: DeformationPayload,
        breakage: BreakPayload,
        joint_breakage: JointBreakPayload,
        affect_change: AffectPayload,
        rule_change: RulePayload,
    };
};

pub const CollisionPayload = struct {
    impact_velocity: i16,
    hardness: u16,
    did_break: bool,
};

pub const SoundPayload = struct {
    sound_type: u8,
    volume: f32,
    pitch: f32,
    duration: f32,
};

pub const ParticlePayload = struct {
    particle_type: u8,
    intensity: f32,
    radius: f32,
    duration: f32,
};

pub const DeformationPayload = struct {
    total_depth: f32,
    permanent_depth: f32,
    recovery_fraction: f32,
    severe: bool,
};

pub const BreakPayload = struct {
    impact_velocity: i16,
    hardness: u16,
    fragment_count: u8,
};

pub const JointBreakPayload = struct {
    joint_idx: u8,
    entity_a: u16,
    entity_b: u16,
    break_ratio: f32,
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

pub const BusBatchResult = struct {
    submitted: u16 = 0,
    accepted: u16 = 0,
    filtered: u16 = 0,
    dropped: u16 = 0,
};

pub const BusCompressedPayload = [16]u8;
pub const BUS_COMPRESSED_PACKET_CAPACITY = 64;
pub const BUS_COMPRESSED_WIRE_MESSAGE_SIZE = 26;
pub const BusCompressedWireMessage = [BUS_COMPRESSED_WIRE_MESSAGE_SIZE]u8;

pub const BusCompressedMessage = struct {
    msg_type: BusMessageType,
    route: u8,
    entity_id: u16,
    tick_delta: u16,
    priority: BusPriority,
    payload_tag: u8,
    payload_len: u8,
    payload: BusCompressedPayload,
    handled: bool = false,
};

pub const BusCompressedPacket = struct {
    base_tick: u32,
    messages: [BUS_COMPRESSED_PACKET_CAPACITY]BusCompressedMessage = undefined,
    count: u8 = 0,
};

pub const BusCompressionResult = struct {
    compressed: u16 = 0,
    skipped: u16 = 0,
};

const BusDispatchResult = struct {
    accepted: bool = false,
    filtered: bool = false,
    dropped: bool = false,
};

const PayloadEncoding = struct {
    tag: u8,
    len: u8,
};

const PAYLOAD_TAG_NONE: u8 = 0;
const PAYLOAD_TAG_COLLISION: u8 = 1;
const PAYLOAD_TAG_SOUND: u8 = 2;
const PAYLOAD_TAG_PARTICLE: u8 = 3;
const PAYLOAD_TAG_DEFORMATION: u8 = 4;
const PAYLOAD_TAG_BREAKAGE: u8 = 5;
const PAYLOAD_TAG_AFFECT_CHANGE: u8 = 6;
const PAYLOAD_TAG_RULE_CHANGE: u8 = 7;
const PAYLOAD_TAG_JOINT_BREAKAGE: u8 = 8;

fn makeCompressedRoute(source_world: WorldType, target_world: WorldType) u8 {
    return (@as(u8, @intCast(@intFromEnum(source_world))) << 4) |
        @as(u8, @intCast(@intFromEnum(target_world)));
}

fn routeSourceWorld(route: u8) WorldType {
    return @enumFromInt((route >> 4) & 0x0f);
}

fn routeTargetWorld(route: u8) WorldType {
    return @enumFromInt(route & 0x0f);
}

fn writeI16Le(out: *BusCompressedPayload, offset: usize, value: i16) void {
    std.mem.writeInt(i16, out[offset..][0..2], value, .little);
}

fn writeU16Le(out: *BusCompressedPayload, offset: usize, value: u16) void {
    std.mem.writeInt(u16, out[offset..][0..2], value, .little);
}

fn writeU32Le(out: *BusCompressedPayload, offset: usize, value: u32) void {
    std.mem.writeInt(u32, out[offset..][0..4], value, .little);
}

fn writeF32Le(out: *BusCompressedPayload, offset: usize, value: f32) void {
    writeU32Le(out, offset, @as(u32, @bitCast(value)));
}

fn readI16Le(in: BusCompressedPayload, offset: usize) i16 {
    return std.mem.readInt(i16, in[offset..][0..2], .little);
}

fn readU16Le(in: BusCompressedPayload, offset: usize) u16 {
    return std.mem.readInt(u16, in[offset..][0..2], .little);
}

fn readU32Le(in: BusCompressedPayload, offset: usize) u32 {
    return std.mem.readInt(u32, in[offset..][0..4], .little);
}

fn readF32Le(in: BusCompressedPayload, offset: usize) f32 {
    return @as(f32, @bitCast(readU32Le(in, offset)));
}

fn writeWireU16Le(out: *BusCompressedWireMessage, offset: usize, value: u16) void {
    std.mem.writeInt(u16, out[offset..][0..2], value, .little);
}

fn readWireU16Le(in: BusCompressedWireMessage, offset: usize) u16 {
    return std.mem.readInt(u16, in[offset..][0..2], .little);
}

fn encodePayload(payload: BusMessage.Payload, out: *BusCompressedPayload) PayloadEncoding {
    out.* = [_]u8{0} ** out.len;
    switch (payload) {
        .none => {
            return .{ .tag = PAYLOAD_TAG_NONE, .len = 0 };
        },
        .collision => |collision| {
            writeI16Le(out, 0, collision.impact_velocity);
            writeU16Le(out, 2, collision.hardness);
            out[4] = @intFromBool(collision.did_break);
            return .{ .tag = PAYLOAD_TAG_COLLISION, .len = 5 };
        },
        .sound => |sound| {
            out[0] = sound.sound_type;
            writeF32Le(out, 1, sound.volume);
            writeF32Le(out, 5, sound.pitch);
            writeF32Le(out, 9, sound.duration);
            return .{ .tag = PAYLOAD_TAG_SOUND, .len = 13 };
        },
        .particle => |particle| {
            out[0] = particle.particle_type;
            writeF32Le(out, 1, particle.intensity);
            writeF32Le(out, 5, particle.radius);
            writeF32Le(out, 9, particle.duration);
            return .{ .tag = PAYLOAD_TAG_PARTICLE, .len = 13 };
        },
        .deformation => |deformation| {
            writeF32Le(out, 0, deformation.total_depth);
            writeF32Le(out, 4, deformation.permanent_depth);
            writeF32Le(out, 8, deformation.recovery_fraction);
            out[12] = @intFromBool(deformation.severe);
            return .{ .tag = PAYLOAD_TAG_DEFORMATION, .len = 13 };
        },
        .breakage => |breakage| {
            writeI16Le(out, 0, breakage.impact_velocity);
            writeU16Le(out, 2, breakage.hardness);
            out[4] = breakage.fragment_count;
            return .{ .tag = PAYLOAD_TAG_BREAKAGE, .len = 5 };
        },
        .joint_breakage => |joint_breakage| {
            out[0] = joint_breakage.joint_idx;
            writeU16Le(out, 1, joint_breakage.entity_a);
            writeU16Le(out, 3, joint_breakage.entity_b);
            writeF32Le(out, 5, joint_breakage.break_ratio);
            return .{ .tag = PAYLOAD_TAG_JOINT_BREAKAGE, .len = 9 };
        },
        .affect_change => |affect| {
            out[0] = @bitCast(affect.valence_delta);
            out[1] = affect.arousal_delta;
            out[2] = affect.certainty_delta;
            return .{ .tag = PAYLOAD_TAG_AFFECT_CHANGE, .len = 3 };
        },
        .rule_change => |rule| {
            writeU16Le(out, 0, rule.rule_id);
            writeU16Le(out, 2, rule.new_threshold);
            return .{ .tag = PAYLOAD_TAG_RULE_CHANGE, .len = 4 };
        },
    }
}

fn decodePayload(tag: u8, payload: BusCompressedPayload) BusMessage.Payload {
    return switch (tag) {
        PAYLOAD_TAG_COLLISION => .{ .collision = .{
            .impact_velocity = readI16Le(payload, 0),
            .hardness = readU16Le(payload, 2),
            .did_break = payload[4] != 0,
        } },
        PAYLOAD_TAG_SOUND => .{ .sound = .{
            .sound_type = payload[0],
            .volume = readF32Le(payload, 1),
            .pitch = readF32Le(payload, 5),
            .duration = readF32Le(payload, 9),
        } },
        PAYLOAD_TAG_PARTICLE => .{ .particle = .{
            .particle_type = payload[0],
            .intensity = readF32Le(payload, 1),
            .radius = readF32Le(payload, 5),
            .duration = readF32Le(payload, 9),
        } },
        PAYLOAD_TAG_DEFORMATION => .{ .deformation = .{
            .total_depth = readF32Le(payload, 0),
            .permanent_depth = readF32Le(payload, 4),
            .recovery_fraction = readF32Le(payload, 8),
            .severe = payload[12] != 0,
        } },
        PAYLOAD_TAG_BREAKAGE => .{ .breakage = .{
            .impact_velocity = readI16Le(payload, 0),
            .hardness = readU16Le(payload, 2),
            .fragment_count = payload[4],
        } },
        PAYLOAD_TAG_JOINT_BREAKAGE => .{ .joint_breakage = .{
            .joint_idx = payload[0],
            .entity_a = readU16Le(payload, 1),
            .entity_b = readU16Le(payload, 3),
            .break_ratio = readF32Le(payload, 5),
        } },
        PAYLOAD_TAG_AFFECT_CHANGE => .{ .affect_change = .{
            .valence_delta = @bitCast(payload[0]),
            .arousal_delta = payload[1],
            .certainty_delta = payload[2],
        } },
        PAYLOAD_TAG_RULE_CHANGE => .{ .rule_change = .{
            .rule_id = readU16Le(payload, 0),
            .new_threshold = readU16Le(payload, 2),
        } },
        else => .{ .none = {} },
    };
}

pub fn compressBusMessage(msg: BusMessage, base_tick: u32) ?BusCompressedMessage {
    if (msg.tick_id < base_tick) return null;
    const tick_delta = msg.tick_id - base_tick;
    if (tick_delta > std.math.maxInt(u16)) return null;

    var payload: BusCompressedPayload = undefined;
    const encoded = encodePayload(msg.payload, &payload);
    return .{
        .msg_type = msg.msg_type,
        .route = makeCompressedRoute(msg.source_world, msg.target_world),
        .entity_id = msg.entity_id,
        .tick_delta = @intCast(tick_delta),
        .priority = msg.priority,
        .payload_tag = encoded.tag,
        .payload_len = encoded.len,
        .payload = payload,
        .handled = msg.handled,
    };
}

pub fn decompressBusMessage(compressed: BusCompressedMessage, base_tick: u32) BusMessage {
    return .{
        .msg_type = compressed.msg_type,
        .source_world = routeSourceWorld(compressed.route),
        .target_world = routeTargetWorld(compressed.route),
        .entity_id = compressed.entity_id,
        .tick_id = base_tick + compressed.tick_delta,
        .priority = compressed.priority,
        .payload = decodePayload(compressed.payload_tag, compressed.payload),
        .handled = compressed.handled,
    };
}

pub fn encodeCompressedMessageWire(compressed: BusCompressedMessage) BusCompressedWireMessage {
    var wire: BusCompressedWireMessage = [_]u8{0} ** BUS_COMPRESSED_WIRE_MESSAGE_SIZE;
    wire[0] = @intFromEnum(compressed.msg_type);
    wire[1] = compressed.route;
    writeWireU16Le(&wire, 2, compressed.entity_id);
    writeWireU16Le(&wire, 4, compressed.tick_delta);
    wire[6] = @intFromEnum(compressed.priority);
    wire[7] = compressed.payload_tag;
    wire[8] = compressed.payload_len;
    wire[9] = @intFromBool(compressed.handled);
    @memcpy(wire[10..26], compressed.payload[0..16]);
    return wire;
}

pub fn decodeCompressedMessageWire(wire: BusCompressedWireMessage) BusCompressedMessage {
    var payload: BusCompressedPayload = undefined;
    @memcpy(payload[0..16], wire[10..26]);
    return .{
        .msg_type = @enumFromInt(wire[0]),
        .route = wire[1],
        .entity_id = readWireU16Le(wire, 2),
        .tick_delta = readWireU16Le(wire, 4),
        .priority = @enumFromInt(wire[6]),
        .payload_tag = wire[7],
        .payload_len = wire[8],
        .payload = payload,
        .handled = wire[9] != 0,
    };
}

pub fn compressBatch(messages: []const BusMessage, base_tick: u32, out: []BusCompressedMessage) BusCompressionResult {
    var result: BusCompressionResult = .{};
    for (messages) |msg| {
        const compressed = compressBusMessage(msg, base_tick) orelse {
            result.skipped += 1;
            continue;
        };
        if (result.compressed >= out.len) {
            result.skipped += 1;
            continue;
        }
        out[result.compressed] = compressed;
        result.compressed += 1;
    }
    return result;
}

pub const Bus = struct {
    messages: [256]BusMessage,
    msg_count: u16,
    total_dispatched: u32,
    total_filtered: u32,
    total_dropped: u32,
    filter: BusEventFilter,

    pub fn init() Bus {
        return .{
            .messages = undefined,
            .msg_count = 0,
            .total_dispatched = 0,
            .total_filtered = 0,
            .total_dropped = 0,
            .filter = BusEventFilter.allowAll(),
        };
    }

    pub fn setEventFilter(self: *Bus, filter: BusEventFilter) void {
        self.filter = filter;
    }

    pub fn clearEventFilter(self: *Bus) void {
        self.filter = BusEventFilter.allowAll();
    }

    pub fn dispatch(self: *Bus, msg: BusMessage) void {
        _ = self.dispatchTracked(msg);
    }

    pub fn dispatchBatch(self: *Bus, batch: []const BusMessage) BusBatchResult {
        var result: BusBatchResult = .{};
        for (batch) |msg| {
            result.submitted += 1;
            const dispatch_result = self.dispatchTracked(msg);
            if (dispatch_result.accepted) result.accepted += 1;
            if (dispatch_result.filtered) result.filtered += 1;
            if (dispatch_result.dropped) result.dropped += 1;
        }
        return result;
    }

    pub fn dispatchCompressedBatch(self: *Bus, compressed: []const BusCompressedMessage, base_tick: u32) BusBatchResult {
        var result: BusBatchResult = .{};
        for (compressed) |compressed_msg| {
            result.submitted += 1;
            const dispatch_result = self.dispatchTracked(decompressBusMessage(compressed_msg, base_tick));
            if (dispatch_result.accepted) result.accepted += 1;
            if (dispatch_result.filtered) result.filtered += 1;
            if (dispatch_result.dropped) result.dropped += 1;
        }
        return result;
    }

    pub fn compressQueue(self: *const Bus, base_tick: u32, out: []BusCompressedMessage) BusCompressionResult {
        return compressBatch(self.messages[0..self.msg_count], base_tick, out);
    }

    fn dispatchTracked(self: *Bus, msg: BusMessage) BusDispatchResult {
        self.total_dispatched += 1;
        if (!self.filter.allows(msg)) {
            self.total_filtered += 1;
            return .{ .filtered = true };
        }
        return self.insertByPriority(msg);
    }

    fn priorityAtLeast(existing: BusPriority, incoming: BusPriority) bool {
        return @intFromEnum(existing) >= @intFromEnum(incoming);
    }

    fn priorityGreater(incoming: BusPriority, existing: BusPriority) bool {
        return @intFromEnum(incoming) > @intFromEnum(existing);
    }

    fn priorityInsertIndex(self: *const Bus, msg: BusMessage) u16 {
        var idx: u16 = 0;
        while (idx < self.msg_count and priorityAtLeast(self.messages[idx].priority, msg.priority)) : (idx += 1) {}
        return idx;
    }

    fn insertByPriority(self: *Bus, msg: BusMessage) BusDispatchResult {
        const capacity: u16 = @intCast(self.messages.len);
        if (self.msg_count == capacity) {
            if (!priorityGreater(msg.priority, self.messages[capacity - 1].priority)) {
                self.total_dropped += 1;
                return .{ .dropped = true };
            }
            const insert_idx = self.priorityInsertIndex(msg);
            var shift_idx: u16 = capacity - 1;
            while (shift_idx > insert_idx) : (shift_idx -= 1) {
                self.messages[shift_idx] = self.messages[shift_idx - 1];
            }
            self.messages[insert_idx] = msg;
            self.total_dropped += 1;
            return .{ .accepted = true, .dropped = true };
        }

        const insert_idx = self.priorityInsertIndex(msg);
        var shift_idx = self.msg_count;
        while (shift_idx > insert_idx) : (shift_idx -= 1) {
            self.messages[shift_idx] = self.messages[shift_idx - 1];
        }
        self.messages[insert_idx] = msg;
        self.msg_count += 1;
        return .{ .accepted = true };
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

    pub fn broadcastSoundEvent(self: *Bus, entity_id: u16, tick_id: u32, sound: SoundPayload) void {
        self.dispatch(.{
            .msg_type = .SOUND_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (sound.volume >= 0.85) .HIGH else .NORMAL,
            .payload = .{ .sound = sound },
        });
        self.dispatch(.{
            .msg_type = .SOUND_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (sound.volume >= 0.85) .HIGH else .NORMAL,
            .payload = .{ .sound = sound },
        });
    }

    pub fn broadcastParticleEvent(self: *Bus, entity_id: u16, tick_id: u32, particle: ParticlePayload) void {
        self.dispatch(.{
            .msg_type = .PARTICLE_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (particle.intensity >= 0.85) .HIGH else .NORMAL,
            .payload = .{ .particle = particle },
        });
        self.dispatch(.{
            .msg_type = .PARTICLE_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (particle.intensity >= 0.85) .HIGH else .NORMAL,
            .payload = .{ .particle = particle },
        });
    }

    pub fn broadcastDeformationEvent(self: *Bus, entity_id: u16, tick_id: u32, deformation: DeformationPayload) void {
        self.dispatch(.{
            .msg_type = .DEFORMATION_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (deformation.severe) .HIGH else .NORMAL,
            .payload = .{ .deformation = deformation },
        });
        self.dispatch(.{
            .msg_type = .DEFORMATION_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = if (deformation.severe) .HIGH else .NORMAL,
            .payload = .{ .deformation = deformation },
        });
    }

    pub fn broadcastBreakEvent(self: *Bus, entity_id: u16, tick_id: u32, breakage: BreakPayload) void {
        self.dispatch(.{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = .HIGH,
            .payload = .{ .breakage = breakage },
        });
        self.dispatch(.{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = entity_id,
            .tick_id = tick_id,
            .priority = .HIGH,
            .payload = .{ .breakage = breakage },
        });
    }

    pub fn broadcastJointBreakEvent(self: *Bus, joint_idx: u16, tick_id: u32, joint_breakage: JointBreakPayload) void {
        self.dispatch(.{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = joint_idx,
            .tick_id = tick_id,
            .priority = .HIGH,
            .payload = .{ .joint_breakage = joint_breakage },
        });
        self.dispatch(.{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = joint_idx,
            .tick_id = tick_id,
            .priority = .HIGH,
            .payload = .{ .joint_breakage = joint_breakage },
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
        const handled = self.processQueueBatch(255);
        self.msg_count = 0;
        return handled;
    }

    pub fn processQueueBatch(self: *Bus, max_messages: u8) u8 {
        var handled: u8 = 0;
        while (handled < max_messages and handled < self.msg_count) : (handled += 1) {
            self.messages[handled].handled = true;
        }

        const remaining_count = self.msg_count - handled;
        var remaining_idx: u16 = 0;
        while (remaining_idx < remaining_count) : (remaining_idx += 1) {
            self.messages[remaining_idx] = self.messages[handled + remaining_idx];
        }
        self.msg_count = remaining_count;
        return handled;
    }

    pub fn processQueueBatchInto(self: *Bus, out_messages: []BusMessage) u8 {
        const max_messages: u8 = @intCast(@min(out_messages.len, 255));
        var handled: u8 = 0;
        while (handled < max_messages and handled < self.msg_count) : (handled += 1) {
            out_messages[handled] = self.messages[handled];
            out_messages[handled].handled = true;
        }

        const remaining_count = self.msg_count - handled;
        var remaining_idx: u16 = 0;
        while (remaining_idx < remaining_count) : (remaining_idx += 1) {
            self.messages[remaining_idx] = self.messages[handled + remaining_idx];
        }
        self.msg_count = remaining_count;
        return handled;
    }

    pub fn peekBatch(self: *const Bus, out_messages: []BusMessage) u8 {
        const count: u8 = @intCast(@min(out_messages.len, self.msg_count, 255));
        var i: u8 = 0;
        while (i < count) : (i += 1) {
            out_messages[i] = self.messages[i];
        }
        return count;
    }

    pub fn clear(self: *Bus) void {
        self.msg_count = 0;
    }
};

pub const BusAsyncDispatchJob = struct {
    batch: []const BusMessage,
    cursor: usize = 0,
    result: BusBatchResult = .{},

    pub fn begin(batch: []const BusMessage) BusAsyncDispatchJob {
        return .{ .batch = batch };
    }

    pub fn step(self: *BusAsyncDispatchJob, event_bus: *Bus, budget: u8) BusBatchResult {
        var step_result: BusBatchResult = .{};
        var processed: u8 = 0;
        while (processed < budget and self.cursor < self.batch.len) : (processed += 1) {
            const dispatch_result = event_bus.dispatchTracked(self.batch[self.cursor]);
            self.cursor += 1;

            step_result.submitted += 1;
            self.result.submitted += 1;
            if (dispatch_result.accepted) {
                step_result.accepted += 1;
                self.result.accepted += 1;
            }
            if (dispatch_result.filtered) {
                step_result.filtered += 1;
                self.result.filtered += 1;
            }
            if (dispatch_result.dropped) {
                step_result.dropped += 1;
                self.result.dropped += 1;
            }
        }
        return step_result;
    }

    pub fn done(self: *const BusAsyncDispatchJob) bool {
        return self.cursor >= self.batch.len;
    }
};

pub const BusAsyncProcessResult = struct {
    processed: u8 = 0,
    total_processed: u16 = 0,
    completed: bool = false,
};

pub const BusAsyncProcessJob = struct {
    target_count: u16,
    processed_count: u16 = 0,
    completed: bool = false,

    pub fn begin(event_bus: *const Bus, max_messages: u16) BusAsyncProcessJob {
        const target_count = if (max_messages == 0)
            event_bus.msg_count
        else
            @min(event_bus.msg_count, max_messages);
        return .{ .target_count = target_count };
    }

    pub fn step(self: *BusAsyncProcessJob, event_bus: *Bus, budget: u8) BusAsyncProcessResult {
        if (self.completed) {
            return .{
                .processed = 0,
                .total_processed = self.processed_count,
                .completed = true,
            };
        }
        if (budget == 0) {
            return .{
                .processed = 0,
                .total_processed = self.processed_count,
                .completed = false,
            };
        }

        const remaining = self.target_count - self.processed_count;
        const step_budget: u8 = @intCast(@min(@as(u16, budget), remaining, @as(u16, 255)));
        const processed = event_bus.processQueueBatch(step_budget);
        self.processed_count += processed;
        self.completed = self.processed_count >= self.target_count or processed == 0;
        return .{
            .processed = processed,
            .total_processed = self.processed_count,
            .completed = self.completed,
        };
    }

    pub fn done(self: *const BusAsyncProcessJob) bool {
        return self.completed;
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
    try std.testing.expect(b.total_filtered == 0);
    try std.testing.expect(b.total_dropped == 0);
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

test "Bus broadcast sound event to two worlds" {
    var b = Bus.init();
    b.broadcastSoundEvent(1, 5, .{ .sound_type = 14, .volume = 0.9, .pitch = 1.1, .duration = 0.2 });
    try std.testing.expect(b.msg_count == 2);
    try std.testing.expect(b.messages[0].msg_type == .SOUND_EVENT);
    try std.testing.expect(b.messages[0].target_world == .psychological);
    try std.testing.expect(b.messages[1].target_world == .programming);
    try std.testing.expect(b.messages[0].priority == .HIGH);
    try std.testing.expect(b.messages[0].payload.sound.sound_type == 14);
}

test "Bus broadcast particle event to two worlds" {
    var b = Bus.init();
    b.broadcastParticleEvent(1, 5, .{ .particle_type = 10, .intensity = 0.9, .radius = 1.5, .duration = 0.75 });
    try std.testing.expect(b.msg_count == 2);
    try std.testing.expect(b.messages[0].msg_type == .PARTICLE_EVENT);
    try std.testing.expect(b.messages[0].target_world == .psychological);
    try std.testing.expect(b.messages[1].target_world == .programming);
    try std.testing.expect(b.messages[0].priority == .HIGH);
    try std.testing.expect(b.messages[0].payload.particle.particle_type == 10);
}

test "Bus broadcast deformation event to two worlds" {
    var b = Bus.init();
    b.broadcastDeformationEvent(1, 5, .{ .total_depth = 2.0, .permanent_depth = 1.0, .recovery_fraction = 0.5, .severe = true });
    try std.testing.expect(b.msg_count == 2);
    try std.testing.expect(b.messages[0].msg_type == .DEFORMATION_EVENT);
    try std.testing.expect(b.messages[0].target_world == .psychological);
    try std.testing.expect(b.messages[1].target_world == .programming);
    try std.testing.expect(b.messages[0].priority == .HIGH);
    try std.testing.expect(b.messages[0].payload.deformation.severe);
}

test "Bus broadcast break event to two worlds" {
    var b = Bus.init();
    b.broadcastBreakEvent(1, 5, .{ .impact_velocity = -120, .hardness = 10, .fragment_count = 3 });
    try std.testing.expect(b.msg_count == 2);
    try std.testing.expect(b.messages[0].msg_type == .BREAK_EVENT);
    try std.testing.expect(b.messages[0].target_world == .psychological);
    try std.testing.expect(b.messages[1].target_world == .programming);
    try std.testing.expect(b.messages[0].priority == .HIGH);
    try std.testing.expect(b.messages[0].payload.breakage.fragment_count == 3);
}

test "Bus broadcast joint break event to two worlds" {
    var b = Bus.init();
    b.broadcastJointBreakEvent(7, 5, .{ .joint_idx = 7, .entity_a = 2, .entity_b = 3, .break_ratio = 1.0 });

    try std.testing.expectEqual(@as(u16, 2), b.msg_count);
    try std.testing.expect(b.messages[0].msg_type == .BREAK_EVENT);
    try std.testing.expect(b.messages[1].msg_type == .BREAK_EVENT);
    try std.testing.expectEqual(@as(u16, 7), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u8, 7), b.messages[0].payload.joint_breakage.joint_idx);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].payload.joint_breakage.entity_a);
    try std.testing.expectEqual(@as(u16, 3), b.messages[0].payload.joint_breakage.entity_b);
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

test "Bus dispatchBatch submits messages with priority ordering" {
    var b = Bus.init();
    const batch = [_]BusMessage{
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 1,
            .tick_id = 0,
            .priority = .LOW,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 2,
            .tick_id = 0,
            .priority = .HIGH,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .AFFECT_EVENT,
            .source_world = .psychological,
            .target_world = .physical,
            .entity_id = 3,
            .tick_id = 0,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        },
    };

    const result = b.dispatchBatch(batch[0..]);

    try std.testing.expectEqual(@as(u16, 3), result.submitted);
    try std.testing.expectEqual(@as(u16, 3), result.accepted);
    try std.testing.expectEqual(@as(u16, 0), result.filtered);
    try std.testing.expectEqual(@as(u16, 0), result.dropped);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u16, 3), b.messages[1].entity_id);
    try std.testing.expectEqual(@as(u16, 1), b.messages[2].entity_id);
}

test "Bus dispatchBatch reports filtered messages" {
    var b = Bus.init();
    b.setEventFilter(BusEventFilter.onlyType(.BREAK_EVENT));
    const batch = [_]BusMessage{
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 1,
            .tick_id = 0,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 2,
            .tick_id = 0,
            .priority = .HIGH,
            .payload = .{ .none = {} },
        },
    };

    const result = b.dispatchBatch(batch[0..]);

    try std.testing.expectEqual(@as(u16, 2), result.submitted);
    try std.testing.expectEqual(@as(u16, 1), result.accepted);
    try std.testing.expectEqual(@as(u16, 1), result.filtered);
    try std.testing.expectEqual(@as(u16, 0), result.dropped);
    try std.testing.expectEqual(@as(u16, 1), b.msg_count);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].entity_id);
}

test "Bus processQueueBatch handles prefix and keeps remaining messages" {
    var b = Bus.init();
    var i: u16 = 0;
    while (i < 5) : (i += 1) {
        b.dispatch(.{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = i,
            .tick_id = 0,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        });
    }

    const handled = b.processQueueBatch(2);

    try std.testing.expectEqual(@as(u8, 2), handled);
    try std.testing.expectEqual(@as(u16, 3), b.msg_count);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u16, 3), b.messages[1].entity_id);
    try std.testing.expectEqual(@as(u16, 4), b.messages[2].entity_id);
}

test "Bus processQueueBatchInto copies handled messages" {
    var b = Bus.init();
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 1,
        .tick_id = 0,
        .priority = .NORMAL,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .BREAK_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 2,
        .tick_id = 0,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .AFFECT_EVENT,
        .source_world = .psychological,
        .target_world = .physical,
        .entity_id = 3,
        .tick_id = 0,
        .priority = .LOW,
        .payload = .{ .none = {} },
    });
    var out: [2]BusMessage = undefined;

    const handled = b.processQueueBatchInto(out[0..]);

    try std.testing.expectEqual(@as(u8, 2), handled);
    try std.testing.expectEqual(@as(u16, 1), b.msg_count);
    try std.testing.expectEqual(@as(u16, 2), out[0].entity_id);
    try std.testing.expect(out[0].handled);
    try std.testing.expectEqual(@as(u16, 1), out[1].entity_id);
    try std.testing.expect(out[1].handled);
    try std.testing.expectEqual(@as(u16, 3), b.messages[0].entity_id);
}

test "Bus peekBatch copies without removing messages" {
    var b = Bus.init();
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 1,
        .tick_id = 0,
        .priority = .NORMAL,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .BREAK_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 2,
        .tick_id = 0,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });
    var out: [4]BusMessage = undefined;

    const copied = b.peekBatch(out[0..]);

    try std.testing.expectEqual(@as(u8, 2), copied);
    try std.testing.expectEqual(@as(u16, 2), b.msg_count);
    try std.testing.expectEqual(@as(u16, 2), out[0].entity_id);
    try std.testing.expectEqual(@as(u16, 1), out[1].entity_id);
}

test "BusAsyncDispatchJob advances dispatch in budgeted steps" {
    var b = Bus.init();
    const batch = [_]BusMessage{
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 1,
            .tick_id = 0,
            .priority = .LOW,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 2,
            .tick_id = 0,
            .priority = .HIGH,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .AFFECT_EVENT,
            .source_world = .psychological,
            .target_world = .physical,
            .entity_id = 3,
            .tick_id = 0,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        },
    };
    var job = BusAsyncDispatchJob.begin(batch[0..]);

    const first = job.step(&b, 2);
    try std.testing.expectEqual(@as(u16, 2), first.submitted);
    try std.testing.expectEqual(@as(u16, 2), first.accepted);
    try std.testing.expect(!job.done());
    try std.testing.expectEqual(@as(u16, 2), b.msg_count);

    const second = job.step(&b, 2);
    try std.testing.expectEqual(@as(u16, 1), second.submitted);
    try std.testing.expectEqual(@as(u16, 1), second.accepted);
    try std.testing.expect(job.done());
    try std.testing.expectEqual(@as(u16, 3), job.result.submitted);
    try std.testing.expectEqual(@as(u16, 3), job.result.accepted);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u16, 3), b.messages[1].entity_id);
    try std.testing.expectEqual(@as(u16, 1), b.messages[2].entity_id);
}

test "BusAsyncDispatchJob reports filtered events across steps" {
    var b = Bus.init();
    b.setEventFilter(BusEventFilter.onlyType(.BREAK_EVENT));
    const batch = [_]BusMessage{
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 1,
            .tick_id = 0,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 2,
            .tick_id = 0,
            .priority = .HIGH,
            .payload = .{ .none = {} },
        },
    };
    var job = BusAsyncDispatchJob.begin(batch[0..]);

    _ = job.step(&b, 1);
    _ = job.step(&b, 1);

    try std.testing.expect(job.done());
    try std.testing.expectEqual(@as(u16, 2), job.result.submitted);
    try std.testing.expectEqual(@as(u16, 1), job.result.accepted);
    try std.testing.expectEqual(@as(u16, 1), job.result.filtered);
    try std.testing.expectEqual(@as(u16, 1), b.msg_count);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].entity_id);
}

test "BusAsyncProcessJob consumes queue over multiple steps" {
    var b = Bus.init();
    var i: u16 = 0;
    while (i < 5) : (i += 1) {
        b.dispatch(.{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = i,
            .tick_id = 0,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        });
    }
    var job = BusAsyncProcessJob.begin(&b, 0);

    const none = job.step(&b, 0);
    try std.testing.expectEqual(@as(u8, 0), none.processed);
    try std.testing.expect(!none.completed);
    try std.testing.expectEqual(@as(u16, 5), b.msg_count);

    const first = job.step(&b, 2);
    try std.testing.expectEqual(@as(u8, 2), first.processed);
    try std.testing.expectEqual(@as(u16, 2), first.total_processed);
    try std.testing.expect(!first.completed);
    try std.testing.expectEqual(@as(u16, 3), b.msg_count);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].entity_id);

    const second = job.step(&b, 3);
    try std.testing.expectEqual(@as(u8, 3), second.processed);
    try std.testing.expectEqual(@as(u16, 5), second.total_processed);
    try std.testing.expect(second.completed);
    try std.testing.expect(job.done());
    try std.testing.expectEqual(@as(u16, 0), b.msg_count);
}

test "BusAsyncProcessJob can process a bounded target count" {
    var b = Bus.init();
    var i: u16 = 0;
    while (i < 5) : (i += 1) {
        b.dispatch(.{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = i,
            .tick_id = 0,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        });
    }
    var job = BusAsyncProcessJob.begin(&b, 3);

    _ = job.step(&b, 2);
    const final = job.step(&b, 2);

    try std.testing.expect(final.completed);
    try std.testing.expectEqual(@as(u16, 3), final.total_processed);
    try std.testing.expectEqual(@as(u16, 2), b.msg_count);
    try std.testing.expectEqual(@as(u16, 3), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u16, 4), b.messages[1].entity_id);
}

test "Bus compressed message round-trips collision payload" {
    const msg = BusMessage{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 42,
        .tick_id = 1005,
        .priority = .HIGH,
        .payload = .{ .collision = .{
            .impact_velocity = -321,
            .hardness = 77,
            .did_break = true,
        } },
        .handled = true,
    };

    const compressed = compressBusMessage(msg, 1000).?;
    const round_trip = decompressBusMessage(compressed, 1000);

    try std.testing.expectEqual(BusMessageType.PHYSICS_EVENT, round_trip.msg_type);
    try std.testing.expectEqual(WorldType.physical, round_trip.source_world);
    try std.testing.expectEqual(WorldType.psychological, round_trip.target_world);
    try std.testing.expectEqual(@as(u16, 42), round_trip.entity_id);
    try std.testing.expectEqual(@as(u32, 1005), round_trip.tick_id);
    try std.testing.expectEqual(BusPriority.HIGH, round_trip.priority);
    try std.testing.expect(round_trip.handled);
    try std.testing.expectEqual(@as(i16, -321), round_trip.payload.collision.impact_velocity);
    try std.testing.expectEqual(@as(u16, 77), round_trip.payload.collision.hardness);
    try std.testing.expect(round_trip.payload.collision.did_break);
}

test "Bus compressed message round-trips float payload bits" {
    const sound_msg = BusMessage{
        .msg_type = .SOUND_EVENT,
        .source_world = .physical,
        .target_world = .programming,
        .entity_id = 7,
        .tick_id = 55,
        .priority = .NORMAL,
        .payload = .{ .sound = .{
            .sound_type = 3,
            .volume = 0.75,
            .pitch = 1.25,
            .duration = 0.5,
        } },
    };
    const particle_msg = BusMessage{
        .msg_type = .PARTICLE_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 8,
        .tick_id = 56,
        .priority = .LOW,
        .payload = .{ .particle = .{
            .particle_type = 9,
            .intensity = 0.625,
            .radius = 2.5,
            .duration = 1.75,
        } },
    };

    const sound = decompressBusMessage(compressBusMessage(sound_msg, 50).?, 50);
    const particle = decompressBusMessage(compressBusMessage(particle_msg, 50).?, 50);

    try std.testing.expectEqual(@as(u8, 3), sound.payload.sound.sound_type);
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 0.75))), @as(u32, @bitCast(sound.payload.sound.volume)));
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 1.25))), @as(u32, @bitCast(sound.payload.sound.pitch)));
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 0.5))), @as(u32, @bitCast(sound.payload.sound.duration)));
    try std.testing.expectEqual(@as(u8, 9), particle.payload.particle.particle_type);
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 0.625))), @as(u32, @bitCast(particle.payload.particle.intensity)));
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 2.5))), @as(u32, @bitCast(particle.payload.particle.radius)));
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 1.75))), @as(u32, @bitCast(particle.payload.particle.duration)));
}

test "Bus compressed message round-trips joint break payload" {
    const msg = BusMessage{
        .msg_type = .BREAK_EVENT,
        .source_world = .physical,
        .target_world = .programming,
        .entity_id = 9,
        .tick_id = 105,
        .priority = .HIGH,
        .payload = .{ .joint_breakage = .{
            .joint_idx = 9,
            .entity_a = 1,
            .entity_b = 2,
            .break_ratio = 1.25,
        } },
    };

    const round_trip = decompressBusMessage(compressBusMessage(msg, 100).?, 100);

    try std.testing.expectEqual(BusMessageType.BREAK_EVENT, round_trip.msg_type);
    try std.testing.expectEqual(@as(u16, 9), round_trip.entity_id);
    try std.testing.expectEqual(@as(u8, 9), round_trip.payload.joint_breakage.joint_idx);
    try std.testing.expectEqual(@as(u16, 1), round_trip.payload.joint_breakage.entity_a);
    try std.testing.expectEqual(@as(u16, 2), round_trip.payload.joint_breakage.entity_b);
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 1.25))), @as(u32, @bitCast(round_trip.payload.joint_breakage.break_ratio)));
}

test "Bus compressBatch preserves order and tick deltas" {
    const batch = [_]BusMessage{
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 1,
            .tick_id = 10,
            .priority = .LOW,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = 2,
            .tick_id = 12,
            .priority = .HIGH,
            .payload = .{ .breakage = .{
                .impact_velocity = -100,
                .hardness = 20,
                .fragment_count = 4,
            } },
        },
    };
    var out: [4]BusCompressedMessage = undefined;

    const result = compressBatch(batch[0..], 10, out[0..]);

    try std.testing.expectEqual(@as(u16, 2), result.compressed);
    try std.testing.expectEqual(@as(u16, 0), result.skipped);
    try std.testing.expectEqual(@as(u16, 0), out[0].tick_delta);
    try std.testing.expectEqual(@as(u16, 2), out[1].tick_delta);
    try std.testing.expectEqual(@as(u16, 1), decompressBusMessage(out[0], 10).entity_id);
    try std.testing.expectEqual(@as(u16, 2), decompressBusMessage(out[1], 10).entity_id);
}

test "Bus dispatchCompressedBatch restores messages with priority ordering" {
    const batch = [_]BusMessage{
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 1,
            .tick_id = 20,
            .priority = .LOW,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .BREAK_EVENT,
            .source_world = .physical,
            .target_world = .programming,
            .entity_id = 2,
            .tick_id = 21,
            .priority = .CRITICAL,
            .payload = .{ .none = {} },
        },
    };
    var compressed: [2]BusCompressedMessage = undefined;
    _ = compressBatch(batch[0..], 20, compressed[0..]);
    var b = Bus.init();

    const result = b.dispatchCompressedBatch(compressed[0..], 20);

    try std.testing.expectEqual(@as(u16, 2), result.submitted);
    try std.testing.expectEqual(@as(u16, 2), result.accepted);
    try std.testing.expectEqual(@as(u16, 2), b.msg_count);
    try std.testing.expectEqual(@as(u16, 2), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u16, 1), b.messages[1].entity_id);
    try std.testing.expectEqual(@as(u32, 21), b.messages[0].tick_id);
}

test "Bus compressed wire encoding restores message from bytes" {
    const msg = BusMessage{
        .msg_type = .DEFORMATION_EVENT,
        .source_world = .physical,
        .target_world = .programming,
        .entity_id = 33,
        .tick_id = 75,
        .priority = .HIGH,
        .payload = .{ .deformation = .{
            .total_depth = 1.5,
            .permanent_depth = 0.25,
            .recovery_fraction = 0.75,
            .severe = true,
        } },
    };

    const compressed = compressBusMessage(msg, 70).?;
    const wire = encodeCompressedMessageWire(compressed);
    const decoded = decodeCompressedMessageWire(wire);
    const round_trip = decompressBusMessage(decoded, 70);

    try std.testing.expectEqual(@as(usize, 26), wire.len);
    try std.testing.expectEqual(BusMessageType.DEFORMATION_EVENT, round_trip.msg_type);
    try std.testing.expectEqual(WorldType.physical, round_trip.source_world);
    try std.testing.expectEqual(WorldType.programming, round_trip.target_world);
    try std.testing.expectEqual(@as(u16, 33), round_trip.entity_id);
    try std.testing.expectEqual(@as(u32, 75), round_trip.tick_id);
    try std.testing.expectEqual(BusPriority.HIGH, round_trip.priority);
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 1.5))), @as(u32, @bitCast(round_trip.payload.deformation.total_depth)));
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 0.25))), @as(u32, @bitCast(round_trip.payload.deformation.permanent_depth)));
    try std.testing.expectEqual(@as(u32, @bitCast(@as(f32, 0.75))), @as(u32, @bitCast(round_trip.payload.deformation.recovery_fraction)));
    try std.testing.expect(round_trip.payload.deformation.severe);
}

test "Bus compression skips tick deltas outside u16 range" {
    const batch = [_]BusMessage{
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 1,
            .tick_id = 9,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 2,
            .tick_id = 10 + @as(u32, std.math.maxInt(u16)) + 1,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        },
        .{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = 3,
            .tick_id = 11,
            .priority = .NORMAL,
            .payload = .{ .none = {} },
        },
    };
    var out: [3]BusCompressedMessage = undefined;

    const result = compressBatch(batch[0..], 10, out[0..]);

    try std.testing.expectEqual(@as(u16, 1), result.compressed);
    try std.testing.expectEqual(@as(u16, 2), result.skipped);
    try std.testing.expectEqual(@as(u16, 3), decompressBusMessage(out[0], 10).entity_id);
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
    try std.testing.expect(b.total_dropped == 4);
}

test "Bus priority order sorts higher priority before lower priority" {
    var b = Bus.init();
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 1,
        .tick_id = 0,
        .priority = .LOW,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 2,
        .tick_id = 0,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 3,
        .tick_id = 0,
        .priority = .CRITICAL,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 4,
        .tick_id = 0,
        .priority = .NORMAL,
        .payload = .{ .none = {} },
    });

    try std.testing.expectEqual(@as(u16, 4), b.msg_count);
    try std.testing.expectEqual(@as(u16, 3), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u16, 2), b.messages[1].entity_id);
    try std.testing.expectEqual(@as(u16, 4), b.messages[2].entity_id);
    try std.testing.expectEqual(@as(u16, 1), b.messages[3].entity_id);
}

test "Bus priority order preserves FIFO for equal priority" {
    var b = Bus.init();
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 1,
        .tick_id = 0,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 2,
        .tick_id = 0,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });
    b.dispatch(.{
        .msg_type = .PHYSICS_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 3,
        .tick_id = 0,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });

    try std.testing.expectEqual(@as(u16, 1), b.messages[0].entity_id);
    try std.testing.expectEqual(@as(u16, 2), b.messages[1].entity_id);
    try std.testing.expectEqual(@as(u16, 3), b.messages[2].entity_id);
}

test "Bus priority queue keeps high priority when full" {
    var b = Bus.init();
    var i: u16 = 0;
    while (i < 256) : (i += 1) {
        b.dispatch(.{
            .msg_type = .PHYSICS_EVENT,
            .source_world = .physical,
            .target_world = .psychological,
            .entity_id = i,
            .tick_id = 0,
            .priority = .LOW,
            .payload = .{ .none = {} },
        });
    }

    b.dispatch(.{
        .msg_type = .BREAK_EVENT,
        .source_world = .physical,
        .target_world = .psychological,
        .entity_id = 999,
        .tick_id = 0,
        .priority = .HIGH,
        .payload = .{ .none = {} },
    });

    try std.testing.expectEqual(@as(u16, 256), b.msg_count);
    try std.testing.expectEqual(@as(u32, 1), b.total_dropped);
    try std.testing.expectEqual(@as(u16, 999), b.messages[0].entity_id);
    try std.testing.expect(b.messages[0].priority == .HIGH);
    try std.testing.expectEqual(@as(u16, 254), b.messages[255].entity_id);
}

test "Bus event filter keeps only selected event type" {
    var b = Bus.init();
    b.setEventFilter(BusEventFilter.onlyType(.BREAK_EVENT));

    b.broadcastSoundEvent(1, 5, .{ .sound_type = 14, .volume = 0.9, .pitch = 1.1, .duration = 0.2 });
    b.broadcastBreakEvent(2, 5, .{ .impact_velocity = -120, .hardness = 10, .fragment_count = 3 });

    try std.testing.expectEqual(@as(u16, 2), b.msg_count);
    try std.testing.expectEqual(@as(u32, 4), b.total_dispatched);
    try std.testing.expectEqual(@as(u32, 2), b.total_filtered);
    try std.testing.expect(b.messages[0].msg_type == .BREAK_EVENT);
    try std.testing.expect(b.messages[1].msg_type == .BREAK_EVENT);
}

test "Bus event filter can target one world" {
    var b = Bus.init();
    b.setEventFilter(BusEventFilter.onlyTarget(.programming));

    b.broadcastPhysicsEvent(1, 5, .{ .impact_velocity = -100, .hardness = 30, .did_break = false });

    try std.testing.expectEqual(@as(u16, 1), b.msg_count);
    try std.testing.expectEqual(@as(u32, 1), b.total_filtered);
    try std.testing.expect(b.messages[0].target_world == .programming);
}

test "Bus event filter applies priority and entity gates" {
    var b = Bus.init();
    b.setEventFilter(.{
        .enabled = true,
        .min_priority = .HIGH,
        .entity_id = 7,
    });

    b.broadcastPhysicsEvent(7, 5, .{ .impact_velocity = -10, .hardness = 30, .did_break = false });
    b.broadcastPhysicsEvent(8, 5, .{ .impact_velocity = -100, .hardness = 30, .did_break = true });
    b.broadcastPhysicsEvent(7, 5, .{ .impact_velocity = -100, .hardness = 30, .did_break = true });

    try std.testing.expectEqual(@as(u16, 2), b.msg_count);
    try std.testing.expectEqual(@as(u32, 4), b.total_filtered);
    try std.testing.expectEqual(@as(u16, 7), b.messages[0].entity_id);
    try std.testing.expect(b.messages[0].priority == .HIGH);
    try std.testing.expectEqual(@as(u16, 7), b.messages[1].entity_id);
}
