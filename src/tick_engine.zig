//! Tick Engine - 4-Step Pipeline for 1024^3 World
//!
//! Phase 1 Enhancement: Continuous Physics
//! Phase 2 Enhancement: Joint Constraints (via joint.zig)
//! Phase 5: Force/Torque & Sleep System
//! Phase 6: Time Scale & Advanced Physics

const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const physics = @import("physics.zig");
const bus = @import("bus.zig");
const joint = @import("joint.zig");

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

pub const SLEEP_VELOCITY_THRESHOLD: i16 = 5;
pub const SLEEP_TIME_THRESHOLD: u8 = 30;
pub const DEFAULT_TIME_SCALE: f32 = 1.0;

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
    time_scale: f32 = DEFAULT_TIME_SCALE,
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
        .time_scale = DEFAULT_TIME_SCALE,
    };
}

/// Check if instance should enter sleep state
pub fn shouldSleep(inst: *scene32.Instance) bool {
    const speed = @abs(inst.vel_x) + @abs(inst.vel_y) + @abs(inst.vel_z);
    const ang_speed = @abs(inst.ang_x) + @abs(inst.ang_y) + @abs(inst.ang_z);
    return speed < SLEEP_VELOCITY_THRESHOLD and ang_speed == 0 and inst.state != .broken;
}

/// Wake up instance from sleep
pub fn wakeInstance(inst: *scene32.Instance) void {
    inst.sleep_tick = 0;
    if (inst.state == .resting) {
        inst.state = .idle;
    }
}

/// Apply force to instance (adds to velocity)
pub fn applyForce(inst: *scene32.Instance, force_x: f32, force_y: f32, force_z: f32, mass: u16) void {
    if (mass == 0) return;
    inst.vel_x = @truncate(@as(i32, inst.vel_x) + @as(i32, @intFromFloat(@round(force_x / @as(f32, @floatFromInt(mass)) * 10.0))));
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(force_y / @as(f32, @floatFromInt(mass)) * 10.0))));
    inst.vel_z = @truncate(@as(i32, inst.vel_z) + @as(i32, @intFromFloat(@round(force_z / @as(f32, @floatFromInt(mass)) * 10.0))));
    wakeInstance(inst);
}

/// Apply torque to instance (adds to angular velocity)
pub fn applyTorque(inst: *scene32.Instance, torque_x: f32, torque_y: f32, torque_z: f32, inertia: u16) void {
    if (inertia == 0) return;
    inst.ang_x = @truncate(@as(i16, inst.ang_x) + @as(i8, @intFromFloat(@round(torque_x / @as(f32, @floatFromInt(inertia)) * 10.0))));
    inst.ang_y = @truncate(@as(i16, inst.ang_y) + @as(i8, @intFromFloat(@round(torque_y / @as(f32, @floatFromInt(inertia)) * 10.0))));
    inst.ang_z = @truncate(@as(i16, inst.ang_z) + @as(i8, @intFromFloat(@round(torque_z / @as(f32, @floatFromInt(inertia)) * 10.0))));
    wakeInstance(inst);
}

/// Apply impulse (instant velocity change)
pub fn applyImpulse(inst: *scene32.Instance, impulse_x: f32, impulse_y: f32, impulse_z: f32) void {
    inst.vel_x = @truncate(@as(i32, inst.vel_x) + @as(i32, @intFromFloat(@round(impulse_x))));
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(impulse_y))));
    inst.vel_z = @truncate(@as(i32, inst.vel_z) + @as(i32, @intFromFloat(@round(impulse_z))));
    wakeInstance(inst);
}

/// Apply buoyancy force for fluids (upward force proportional to displaced volume)
pub fn applyBuoyancy(inst: *scene32.Instance, fluid_density: f32, mass: u16) void {
    if (mass == 0) return;
    // Buoyancy = fluid_density * volume * gravity (simplified)
    const volume: f32 = 16.0 * 16.0 * 16.0; // Full 16^3 entity volume
    const buoyancy_force = fluid_density * volume * 0.01;
    inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(buoyancy_force))));
    wakeInstance(inst);
}

/// Force field types
pub const ForceFieldType = enum(u8) {
    none = 0,
    point = 1,      // Radial explosion-like force
    directional = 2, // Constant direction force (wind, gravity)
    vortex = 3,     // Rotational force
};

pub const ForceField = struct {
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
    radius: f32,
    strength: f32,
    field_type: ForceFieldType,
};

/// Apply explosion force (point impulse)
pub fn applyExplosion(engine: *TickEngine, fx: f32, fy: f32, fz: f32, radius: f32, force: f32) void {
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue; // Skip static

        const dx = @as(f32, @floatFromInt(inst.pos_x)) - fx;
        const dy = @as(f32, @floatFromInt(inst.pos_y)) - fy;
        const dz = @as(f32, @floatFromInt(inst.pos_z)) - fz;
        const dist = @sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < radius and dist > 0.1) {
            const falloff = 1.0 - (dist / radius);
            const impulse = force * falloff / @as(f32, @floatFromInt(entity.physics.mass));
            const nx = dx / dist;
            const ny = dy / dist;
            const nz = dz / dist;
            applyImpulse(inst, nx * impulse, ny * impulse, nz * impulse);
        }
    }
}

/// Apply force field to all instances in range
pub fn applyForceField(engine: *TickEngine, field: ForceField) void {
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue;

        switch (field.field_type) {
            .point => {
                const dx = @as(f32, @floatFromInt(inst.pos_x)) - field.pos_x;
                const dy = @as(f32, @floatFromInt(inst.pos_y)) - field.pos_y;
                const dz = @as(f32, @floatFromInt(inst.pos_z)) - field.pos_z;
                const dist = @sqrt(dx * dx + dy * dy + dz * dz);
                if (dist < field.radius and dist > 0.1) {
                    const falloff = 1.0 - (dist / field.radius);
                    const impulse = field.strength * falloff / @as(f32, @floatFromInt(entity.physics.mass));
                    const nx = dx / dist;
                    const ny = dy / dist;
                    const nz = dz / dist;
                    applyImpulse(inst, nx * impulse, ny * impulse, nz * impulse);
                }
            },
            .directional => {
                const dx = @as(f32, @floatFromInt(inst.pos_x)) - field.pos_x;
                const dy = @as(f32, @floatFromInt(inst.pos_y)) - field.pos_y;
                const dz = @as(f32, @floatFromInt(inst.pos_z)) - field.pos_z;
                const dist = @sqrt(dx * dx + dy * dy + dz * dz);
                if (dist < field.radius) {
                    const falloff = 1.0 - (dist / field.radius);
                    const force_scale = field.strength * falloff / @as(f32, @floatFromInt(entity.physics.mass)) * 0.1;
                    applyForce(inst, field.pos_x * force_scale, field.pos_y * force_scale, field.pos_z * force_scale, entity.physics.mass);
                }
            },
            .vortex => {
                const dx = @as(f32, @floatFromInt(inst.pos_x)) - field.pos_x;
                const dy = @as(f32, @floatFromInt(inst.pos_y)) - field.pos_y;
                const dz = @as(f32, @floatFromInt(inst.pos_z)) - field.pos_z;
                const dist = @sqrt(dx * dx + dy * dy + dz * dz);
                if (dist < field.radius and dist > 0.1) {
                    const falloff = 1.0 - (dist / field.radius);
                    const tangent_force = field.strength * falloff * 0.1;
                    // Tangential impulse (perpendicular to radius)
                    applyImpulse(inst, -dz * tangent_force / dist, 0, dx * tangent_force / dist);
                }
            },
            else => {},
        }
    }
}

/// Solve joints for connected bodies (external solver integration point)
pub fn solveJointsForEngine(engine: *TickEngine, joints: []const joint.Joint) void {
    joint.solveJointsForTick(engine.s1024.instances[0..engine.s1024.instance_count], joints, engine.entities);
}

/// Apply continuous physics: gravity, damping, angular velocity, sleep check
fn applyContinuousPhysics(engine: *TickEngine) void {
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &engine.entities[inst.entity_id];

        // Skip static objects (flag 0x01)
        if ((entity.physics.flags & 0x01) != 0) continue;

        // Apply time scale to physics
        const dt_scale = engine.time_scale;
        if (dt_scale <= 0.0) continue;

        // Apply gravity to velocity (scaled by time_scale)
        if (entity.physics.material != .liquid) {
            const grav_vel: i32 = @as(i32, physics.GRAVITY);
            inst.vel_y = @truncate(@as(i32, inst.vel_y) + @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(grav_vel)) * dt_scale))));
        }

        // Apply velocity damping
        physics.applyDamping(&inst.vel_x, &inst.vel_y, &inst.vel_z);

        // Apply angular velocity and damping
        if (inst.ang_x != 0 or inst.ang_y != 0 or inst.ang_z != 0) {
            // Update rotation (ang is in radians per tick scaled by 10)
            inst.rot_yaw = @truncate((@as(u16, inst.rot_yaw) + @as(u8, @intCast(@divTrunc(inst.ang_y, 10))) & 0xFF));
            inst.rot_pitch = @truncate((@as(u16, inst.rot_pitch) + @as(u8, @intCast(@divTrunc(inst.ang_x, 10))) & 0xFF));
            inst.rot_roll = @truncate((@as(u16, inst.rot_roll) + @as(u8, @intCast(@divTrunc(inst.ang_z, 10))) & 0xFF));

            // Apply angular damping
            physics.applyAngularDamping(&inst.ang_x, &inst.ang_y, &inst.ang_z);
        }

        // Sleep check
        if (shouldSleep(inst)) {
            inst.sleep_tick += 1;
            if (inst.sleep_tick >= SLEEP_TIME_THRESHOLD) {
                inst.state = .resting;
                inst.vel_x = 0;
                inst.vel_y = 0;
                inst.vel_z = 0;
                inst.ang_x = 0;
                inst.ang_y = 0;
                inst.ang_z = 0;
            }
        } else {
            inst.sleep_tick = 0;
        }
    }
}

pub fn gather(engine: *TickEngine) void {
    // First apply continuous physics (gravity, velocity updates)
    applyContinuousPhysics(engine);

    engine.intent_count = 0;
    var i: u8 = 0;
    while (i < engine.s1024.instance_count) : (i += 1) {
        if (engine.intent_count >= 64) break;
        const inst = &engine.s1024.instances[i];
        if (inst.entity_id >= engine.entities.len) continue;
        if (inst.state == .broken) continue;
        const entity = &engine.entities[inst.entity_id];

        // Skip static objects
        if ((entity.physics.flags & 0x01) != 0) {
            inst.state = .resting;
            continue;
        }

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
                // Use continuous fall check if we have velocity
                const r = if (inst.vel_y < 0)
                    physics.checkContinuousFall(engine.s1024, inst, engine.entities)
                else
                    physics.checkFall(engine.s1024, inst, engine.entities);

                if (r.can_fall) {
                    // Move by velocity amount
                    const dy: i8 = @intCast(@min(127, @max(-128, r.target_y - inst.pos_y)));
                    engine.intents[engine.intent_count] = .{
                        .instance_idx = i,
                        .op = .FALL,
                        .dy = dy,
                        .priority = @intCast(@as(i16, 200) + engine.arousal_mod),
                    };
                    engine.intent_count += 1;
                } else if (r.blocked) {
                    // Collision - apply restitution and friction
                    const was_moving = (inst.state == .falling or inst.state == .moving or inst.state == .idle);
                    inst.state = .resting;

                    if (was_moving and inst.vel_y < 0) {
                        // Apply bounce (restitution)
                        inst.vel_y = physics.applyRestitution(inst.vel_y, entity.physics.restitution);

                        // Apply friction to horizontal velocity
                        physics.applyFriction(&inst.vel_x, &inst.vel_z, entity.physics.friction);

                        // Compute impact for breaking
                        const impact = physics.calcImpact(inst.vel_y, entity.physics.mass);

                        const b_self = physics.checkBreak(impact, entity.physics.material, entity.physics.hardness);
                        if (b_self.did_break) {
                            engine.intents[engine.intent_count] = .{
                                .instance_idx = i, .op = .BREAK, .priority = 250
                            };
                            engine.intent_count += 1;
                            if (engine.world_bus) |b| {
                                b.broadcastPhysicsEvent(inst.entity_id, engine.tick_id, .{ .impact_velocity = inst.vel_y, .hardness = entity.physics.hardness, .did_break = true });
                            }
                        }

                        if (r.blocker_id != 255 and r.blocker_id != 0) {
                            const target_inst = &engine.s1024.instances[r.blocker_id];
                            const target_ent = &engine.entities[target_inst.entity_id];
                            const b_target = physics.checkBreak(impact, target_ent.physics.material, target_ent.physics.hardness);
                            if (b_target.did_break) {
                                engine.intents[engine.intent_count] = .{
                                    .instance_idx = r.blocker_id, .op = .BREAK, .priority = 250
                                };
                                engine.intent_count += 1;
                                if (engine.world_bus) |b| {
                                    b.broadcastPhysicsEvent(target_inst.entity_id, engine.tick_id, .{ .impact_velocity = inst.vel_y, .hardness = target_ent.physics.hardness, .did_break = true });
                                }
                            }
                        }
                    }

                    // If still has significant velocity, keep moving
                    if (@abs(inst.vel_x) > 10 or @abs(inst.vel_z) > 10) {
                        // Could add slide logic here
                    }
                } else {
                    // Not blocked, not falling - check if should rest
                    if (@abs(inst.vel_x) < 5 and @abs(inst.vel_y) < 5 and @abs(inst.vel_z) < 5) {
                        inst.state = .resting;
                        inst.vel_x = 0;
                        inst.vel_y = 0;
                        inst.vel_z = 0;
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
            .FALL => {
                inst.pos_y += intent.dy;
                inst.state = if (inst.vel_y != 0) .falling else .resting;
                applied += 1;
            },
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
