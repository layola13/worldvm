//! Tick Engine - 4-Step Pipeline
const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const physics = @import("physics.zig");

pub const Operator = enum(u8) { NOP = 0, FALL = 6, FLOW = 7, MOVE = 3, PUSH = 4, BREAK = 5 };

pub const Intent = struct { instance_idx: u8, op: Operator, dx: i8 = 0, dy: i8 = 0, dz: i8 = 0, priority: u8 = 128 };

pub const TickEngine = struct {
    scene: *scene32.Scene32,
    entities: []entity16.Entity16,
    intents: [64]Intent = undefined,
    intent_count: u8 = 0,
    max_ticks: u32 = 1000,
    stable: bool = false,
    tick_id: u32 = 0,
    reason_code: u8 = 0,
};

pub fn init(engine: *TickEngine, scene: *scene32.Scene32, entities: []entity16.Entity16) void {
    engine.* = .{ .scene = scene, .entities = entities, .max_ticks = 1000, .stable = false, .tick_id = 0, .reason_code = 0 };
}

pub fn gather(engine: *TickEngine) void {
    engine.intent_count = 0;
    var i: u8 = 0;
    while (i < engine.scene.*.instance_count) : (i += 1) {
        if (engine.intent_count >= 64) break;
        const inst = &engine.scene.*.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        const entity = &engine.entities[inst.entity_id];
        switch (entity.physics.material) {
            .liquid => {
                const r = physics.checkFlow(engine.scene, inst, engine.entities);
                if (r.flowed) {
                    engine.intents[engine.intent_count] = .{ .instance_idx = i, .op = .FLOW, .dx = r.new_x - inst.pos_x, .dy = r.new_y - inst.pos_y, .dz = r.new_z - inst.pos_z, .priority = 180 };
                    engine.intent_count += 1;
                }
            },
            else => {
                const r = physics.checkFall(engine.scene, inst, engine.entities);
                if (r.can_fall) {
                    engine.intents[engine.intent_count] = .{ .instance_idx = i, .op = .FALL, .dy = r.target_y - inst.pos_y, .priority = 200 };
                    engine.intent_count += 1;
                } else if (r.blocked) {
                    engine.scene.*.instances[i].state = .resting;
                }
            },
        }
    }
}

pub fn speculate(engine: *TickEngine) void { _ = engine; }
pub fn resolve(engine: *TickEngine) void { _ = engine; }

pub fn commit(engine: *TickEngine) u16 {
    var applied: u16 = 0;
    var i: u8 = 0;
    while (i < engine.intent_count) : (i += 1) {
        const intent = &engine.intents[i];
        if (intent.op == .NOP) continue;
        const inst = &engine.scene.*.instances[intent.instance_idx];
        switch (intent.op) {
            .FALL => { inst.pos_y += intent.dy; inst.state = .falling; applied += 1; },
            .FLOW => { inst.pos_x += intent.dx; inst.pos_y += intent.dy; inst.pos_z += intent.dz; inst.state = .flowing; applied += 1; },
            .MOVE, .PUSH => { inst.pos_x += intent.dx; inst.pos_y += intent.dy; inst.pos_z += intent.dz; inst.state = .moving; applied += 1; },
            else => {},
        }
    }
    scene32.rebuildOccupancy(engine.scene, engine.entities);
    return applied;
}

pub fn stepTick(engine: *TickEngine) bool {
    engine.tick_id += 1;
    engine.scene.*.tick = engine.tick_id;
    gather(engine);
    speculate(engine);
    resolve(engine);
    _ = commit(engine);
    engine.stable = (engine.intent_count == 0);
    return engine.stable;
}

pub fn runTicks(engine: *TickEngine, max_ticks: u32) u32 {
    var ticks_run: u32 = 0;
    while (ticks_run < max_ticks and !engine.stable) {
        if (stepTick(engine)) { engine.reason_code = 1; break; }
        ticks_run += 1;
    }
    return ticks_run;
}
