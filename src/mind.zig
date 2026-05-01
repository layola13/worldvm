const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const tick_engine = @import("tick_engine.zig");
const entity16 = @import("entity16.zig");
const bus = @import("bus.zig");

pub const WorldType = enum(u4) {
    physical = 0,
    psychological = 1,
    programming = 2,
};

pub const AffectBlock = struct {
    valence: i8,
    arousal: u8,
    certainty: u8,
    control: u8,

    pub fn init(v: i8, a: u8, c: u8, ctrl: u8) AffectBlock {
        return .{ .valence = v, .arousal = a, .certainty = c, .control = ctrl };
    }
};

pub const HookStatus = enum {
    PASS,
    FAIL,
};

pub const HookResult = struct {
    status: HookStatus,
    reason_code: u8,
    break_frame: u32,
    repair_hint: []const u8,
};

// Tri-World Bus integration per doc 28
// Physical <-> Psychological <-> Programming via message bus
pub const TriWorldBus = struct {
    inner: bus.Bus,

    pub fn init() TriWorldBus {
        return .{ .inner = bus.Bus.init() };
    }

    // Broadcast a physics collision event to psychological and programming worlds
    pub fn onCollision(self: *TriWorldBus, entity_id: u16, tick_id: u32, impact_vel: i16, hardness: u16, did_break: bool) void {
        self.inner.broadcastPhysicsEvent(entity_id, tick_id, .{
            .impact_velocity = impact_vel,
            .hardness = hardness,
            .did_break = did_break,
        });
    }

    // Broadcast affect change from psychological world
    pub fn onAffectChange(self: *TriWorldBus, entity_id: u16, tick_id: u32, valence_delta: i8, arousal_delta: u8, certainty_delta: u8) void {
        self.inner.broadcastAffectEvent(entity_id, tick_id, .{
            .valence_delta = valence_delta,
            .arousal_delta = arousal_delta,
            .certainty_delta = certainty_delta,
        });
    }

    pub fn sync(self: *TriWorldBus) u8 {
        return self.inner.processQueue();
    }
};

pub const AffectSystem = struct {
    registers: AffectBlock,

    pub fn init() AffectSystem {
        return .{ .registers = AffectBlock.init(0, 0, 128, 128) };
    }

    pub fn update(self: *AffectSystem, tri_bus: *TriWorldBus) void {
        var i: u16 = 0;
        while (i < tri_bus.inner.msg_count) : (i += 1) {
            const msg = &tri_bus.inner.messages[i];
            if (msg.handled) continue;

            switch (msg.payload) {
                .collision => |c| {
                    if (c.did_break) {
                        // Breaking things causes arousal and negative valence
                        self.registers.valence = @intCast(@max(@as(i16, self.registers.valence) - 20, -128));
                        self.registers.arousal = @intCast(@min(@as(u16, self.registers.arousal) + 50, 255));
                    }
                },
                .affect_change => |a| {
                    self.registers.valence = @intCast(@max(@min(@as(i16, self.registers.valence) + a.valence_delta, 127), -128));
                },
                else => {},
            }
        }
    }

    pub fn getPriorityMod(self: AffectSystem) i16 {
        // High arousal increases urgency (priority)
        return @divTrunc(@as(i16, self.registers.arousal), 10);
    }
};

pub const ShadowSandbox = struct {
    allocator: std.mem.Allocator,
    base_world: *scene1024.Scene1024,
    shadow_world: scene1024.Scene1024,
    entities: []entity16.Entity16,
    tri_bus: TriWorldBus,

    pub fn init(allocator: std.mem.Allocator, base: *scene1024.Scene1024, entities: []entity16.Entity16) !ShadowSandbox {
        var shadow = scene1024.Scene1024.init(allocator);

        for (0..scene1024.MAX_ACTIVE_PAGES) |i| {
            if (base.pages[i].resident and base.pages[i].scene != null) {
                const new_scene = try allocator.create(scene32.Scene32);
                new_scene.* = base.pages[i].scene.?.*;

                shadow.pages[i] = base.pages[i];
                shadow.pages[i].scene = new_scene;
            }
        }
        shadow.active_count = base.active_count;
        shadow.global_tick = base.global_tick;
        shadow.instances = base.instances;
        shadow.instance_count = base.instance_count;

        return ShadowSandbox{
            .allocator = allocator,
            .base_world = base,
            .shadow_world = shadow,
            .entities = entities,
            .tri_bus = TriWorldBus.init(),
        };
    }

    pub fn deinit(self: *ShadowSandbox) void {
        self.shadow_world.deinit();
    }

    pub fn simulate(self: *ShadowSandbox, ticks: u32) HookResult {
        var engine: tick_engine.TickEngine = undefined;
        var active_scene: ?*scene32.Scene32 = null;
        for (0..scene1024.MAX_ACTIVE_PAGES) |i| {
            if (self.shadow_world.pages[i].resident and self.shadow_world.pages[i].page_id == 0) {
                active_scene = self.shadow_world.pages[i].scene;
                break;
            }
        }

        if (active_scene == null) {
            return .{ .status = .FAIL, .reason_code = 1, .break_frame = 0, .repair_hint = "No active page" };
        }

        tick_engine.init(&engine, &self.shadow_world, self.entities);

        var t: u32 = 0;
        while (t < ticks and !engine.stable) : (t += 1) {
            _ = tick_engine.stepTick(&engine);

            // Sync tri-world bus after each tick (Physical -> Psychological/Programming)
            _ = self.tri_bus.sync();

            for (0..self.shadow_world.instance_count) |i| {
                const inst = &self.shadow_world.instances[i];
                if (inst.state == .broken) {
                    // Broadcast break event through tri-world bus
                    self.tri_bus.onCollision(inst.entity_id, t, -100, 30, true);
                    return .{
                        .status = .FAIL,
                        .reason_code = 2,
                        .break_frame = t,
                        .repair_hint = "Entity broken during simulation",
                    };
                }
            }
        }

        return .{
            .status = .PASS,
            .reason_code = 0,
            .break_frame = t,
            .repair_hint = "Simulation successful",
        };
    }
};

test "TriWorldBus collision and affect events enqueue expected messages" {
    var tri_bus = TriWorldBus.init();

    tri_bus.onCollision(7, 11, -100, 30, true);
    try std.testing.expectEqual(@as(u16, 2), tri_bus.inner.msg_count);
    try std.testing.expectEqual(bus.BusMessageType.PHYSICS_EVENT, tri_bus.inner.messages[0].msg_type);
    try std.testing.expectEqual(WorldType.physical, tri_bus.inner.messages[0].source_world);
    try std.testing.expectEqual(@as(u16, 7), tri_bus.inner.messages[0].entity_id);

    tri_bus.onAffectChange(7, 12, 5, 6, 7);
    try std.testing.expectEqual(@as(u16, 3), tri_bus.inner.msg_count);
    try std.testing.expectEqual(bus.BusMessageType.AFFECT_EVENT, tri_bus.inner.messages[2].msg_type);

    try std.testing.expectEqual(@as(u8, 3), tri_bus.sync());
    try std.testing.expectEqual(@as(u16, 0), tri_bus.inner.msg_count);
}

test "AffectSystem applies break and affect deltas with clamping" {
    var affect_system = AffectSystem.init();
    var tri_bus = TriWorldBus.init();

    tri_bus.onCollision(1, 1, -100, 30, true);
    tri_bus.onAffectChange(1, 2, 127, 0, 0);
    affect_system.update(&tri_bus);

    try std.testing.expectEqual(@as(i8, 87), affect_system.registers.valence);
    try std.testing.expectEqual(@as(u8, 100), affect_system.registers.arousal);
    try std.testing.expectEqual(@as(i16, 10), affect_system.getPriorityMod());

    tri_bus.inner.msg_count = 0;
    tri_bus.onAffectChange(1, 3, 127, 0, 0);
    affect_system.update(&tri_bus);
    try std.testing.expectEqual(@as(i8, 127), affect_system.registers.valence);
}

test "ShadowSandbox copies base world pages and instances" {
    var base_world = scene1024.Scene1024.init(std.testing.allocator);
    defer base_world.deinit();
    const page = try base_world.getPage(0);
    scene32.setOccupied(page.scene.?, 1, 2, 3);

    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 0;
    inst.pos_x = 10;
    inst.pos_y = 20;
    inst.pos_z = 30;
    _ = try base_world.addInstance(inst);

    var entities = [_]entity16.Entity16{entity16.initEntity16()};
    var sandbox = try ShadowSandbox.init(std.testing.allocator, &base_world, &entities);
    defer sandbox.deinit();

    try std.testing.expectEqual(base_world.active_count, sandbox.shadow_world.active_count);
    try std.testing.expectEqual(base_world.instance_count, sandbox.shadow_world.instance_count);
    try std.testing.expectEqual(base_world.instances[0], sandbox.shadow_world.instances[0]);
    try std.testing.expect(scene32.isOccupied(sandbox.shadow_world.pages[0].scene.?, 1, 2, 3));

    scene32.clearOccupied(sandbox.shadow_world.pages[0].scene.?, 1, 2, 3);
    try std.testing.expect(scene32.isOccupied(base_world.pages[0].scene.?, 1, 2, 3));
}

test "ShadowSandbox simulate fails cleanly without active page" {
    var base_world = scene1024.Scene1024.init(std.testing.allocator);
    defer base_world.deinit();
    var entities = [_]entity16.Entity16{entity16.initEntity16()};
    var sandbox = try ShadowSandbox.init(std.testing.allocator, &base_world, &entities);
    defer sandbox.deinit();

    const result = sandbox.simulate(1);
    try std.testing.expectEqual(HookStatus.FAIL, result.status);
    try std.testing.expectEqual(@as(u8, 1), result.reason_code);
}
