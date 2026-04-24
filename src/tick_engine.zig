//! Tick Engine - 4-Step Pipeline for 1024^3 World
const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const physics = @import("physics.zig");
const bus = @import("bus.zig");

pub const Operator = enum(u8) { NOP = 0, FALL = 6, FLOW = 7, MOVE = 3, PUSH = 4, BREAK = 5 };

pub const Intent = struct { 
    instance_idx: u8, 
    op: Operator, 
    dx: i8 = 0, 
    dy: i8 = 0, 
    dz: i8 = 0, 
    priority: u8 = 128,
    target_instance: u8 = 255,
};

pub const TickEngine = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    intents: [64]Intent = undefined,
    intent_count: u8 = 0,
    max_ticks: u32 = 1000,
    stable: bool = false,
    tick_id: u32 = 0,
    world_bus: ?*bus.Bus = null,
    arousal_mod: i16 = 0,
};

pub fn init(engine: *TickEngine, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) void {
    engine.* = .{ 
        .s1024 = s1024,
        .entities = entities, 
        .max_ticks = 1000, 
        .stable = false, 
        .tick_id = 0,
        .world_bus = null,
        .arousal_mod = 0,
    };
}

pub fn gather(engine: *TickEngine) void {
    engine.intent_count = 0;
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        if (engine.intent_count >= 64) break;
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        const entity = &engine.entities[inst.entity_id];
        
        switch (entity.physics.material) {
            .liquid => {
                const r = physics.checkFlow(engine.s1024, inst, engine.entities);
                if (r.flowed) {
                    engine.intents[engine.intent_count] = .{ 
                        .instance_idx = i, 
                        .op = .FLOW, 
                        .dx = @intCast(r.new_x - inst.pos_x), 
                        .dy = @intCast(r.new_y - inst.pos_y), 
                        .dz = @intCast(r.new_z - inst.pos_z), 
                        .priority = @intCast(@as(i16, 180) + engine.arousal_mod), 
                    };
                    engine.intent_count += 1;
                }
            },
            else => {
                const r = physics.checkFall(engine.s1024, inst, engine.entities);
                if (r.can_fall) {
                    engine.intents[engine.intent_count] = .{ 
                        .instance_idx = i, 
                        .op = .FALL, 
                        .dy = @intCast(r.target_y - inst.pos_y), 
                        .priority = @intCast(@as(i16, 200) + engine.arousal_mod), 
                    };
                    engine.intent_count += 1;
                } else if (r.blocked) {
                    const was_moving = (inst.state == .falling or inst.state == .moving or inst.state == .idle);
                    engine.s1024.instances[i].state = .resting;
                    if (was_moving) {
                        const impact = physics.calcImpact(-100, entity.physics.mass);
                        
                        const b_self = physics.checkBreak(impact, entity.physics.material, entity.physics.hardness);
                        if (b_self.did_break) {
                            engine.intents[engine.intent_count] = .{
                                .instance_idx = i, .op = .BREAK, .priority = 250
                            };
                            engine.intent_count += 1;
                            if (engine.world_bus) |b| {
                                b.broadcastPhysicsEvent(inst.entity_id, engine.tick_id, .{ .impact_velocity = -100, .hardness = entity.physics.hardness, .did_break = true });
                            }
                        }
                        
                        if (r.blocker_id != 255) {
                            const target_inst = &engine.s1024.instances[r.blocker_id];
                            const target_ent = &engine.entities[target_inst.entity_id];
                            const b_target = physics.checkBreak(impact, target_ent.physics.material, target_ent.physics.hardness);
                            if (b_target.did_break) {
                                engine.intents[engine.intent_count] = .{
                                    .instance_idx = r.blocker_id, .op = .BREAK, .priority = 250
                                };
                                engine.intent_count += 1;
                                if (engine.world_bus) |b| {
                                    b.broadcastPhysicsEvent(target_inst.entity_id, engine.tick_id, .{ .impact_velocity = -100, .hardness = target_ent.physics.hardness, .did_break = true });
                                }
                            }
                        }
                    }
                }
            },
        }
    }
}

pub fn speculate(engine: *TickEngine) void {
    if (engine.intent_count < 2) return;
    var i: u8 = 0;
    while (i < engine.intent_count - 1) : (i += 1) {
        var j: u8 = 0;
        while (j < engine.intent_count - i - 1) : (j += 1) {
            if (engine.intents[j].priority < engine.intents[j+1].priority) {
                const tmp = engine.intents[j];
                engine.intents[j] = engine.intents[j+1];
                engine.intents[j+1] = tmp;
            }
        }
    }
}

pub fn resolve(engine: *TickEngine) void {
    var i: u8 = 0;
    while (i < engine.intent_count) : (i += 1) {
        const intent = &engine.intents[i];
        if (intent.op == .NOP or intent.op == .BREAK) continue;
        const inst = &engine.s1024.instances[intent.instance_idx];
        const entity = &engine.entities[inst.entity_id];
        
        const nx = inst.pos_x + intent.dx;
        const ny = inst.pos_y + intent.dy;
        const nz = inst.pos_z + intent.dz;
        
        var valid = true;
        for (0..64) |w_idx| {
            const word = entity.topology[w_idx];
            if (word == 0) continue;
            for (0..64) |b_idx| {
                if ((word & (@as(u64, 1) << @as(u6, @truncate(b_idx)))) != 0) {
                    const idx = (w_idx << 6) | b_idx;
                    const ex: i32 = @intCast((idx >> 4) & 0xF);
                    const ey: i32 = @intCast(idx >> 8);
                    const ez: i32 = @intCast(idx & 0xF);
                    if (physics.isOccupiedGlobal(engine.s1024, inst, engine.entities, nx + ex, ny + ey, nz + ez, null)) {
                        valid = false; break;
                    }
                }
            }
            if (!valid) break;
        }
        if (!valid) intent.op = .NOP;
    }
}

pub fn commit(engine: *TickEngine) u16 {
    var applied: u16 = 0;
    var i: u8 = 0;
    while (i < engine.intent_count) : (i += 1) {
        const intent = &engine.intents[i];
        if (intent.op == .NOP) continue;
        const inst = &engine.s1024.instances[intent.instance_idx];
        
        switch (intent.op) {
            .FALL => { inst.pos_y += intent.dy; inst.state = .falling; applied += 1; },
            .FLOW => { inst.pos_x += intent.dx; inst.pos_y += intent.dy; inst.pos_z += intent.dz; inst.state = .flowing; applied += 1; },
            .MOVE, .PUSH => { inst.pos_x += intent.dx; inst.pos_y += intent.dy; inst.pos_z += intent.dz; inst.state = .moving; applied += 1; },
            .BREAK => { inst.state = .broken; applied += 1; },
            else => {},
        }
    }
    engine.s1024.rebuildOccupancy(engine.entities) catch {};
    return applied;
}

pub fn stepTick(engine: *TickEngine) bool {
    engine.tick_id += 1;
    engine.s1024.global_tick = engine.tick_id;
    gather(engine);
    speculate(engine);
    resolve(engine);
    const applied = commit(engine);
    engine.stable = (applied == 0);
    return engine.stable;
}

pub fn runTicks(engine: *TickEngine, max_ticks: u32) u32 {
    var ticks_run: u32 = 0;
    while (ticks_run < max_ticks and !engine.stable) {
        if (stepTick(engine)) break;
        ticks_run += 1;
    }
    return ticks_run;
}
