//! Deterministic particle simulation core.

const std = @import("std");

pub const MAX_PARTICLES = 512;
pub const MAX_EMITTERS = 32;

pub const ParticleKind = enum(u8) {
    generic,
    fountain,
    explosion,
    sand,
    dust,
    snow,
    rain,
    spray,
    ember,
    debris,
    smoke,
};

pub const EmitterShape = enum(u8) {
    point,
    cone,
    sphere,
    disc,
};

pub const MediumPolicy = enum(u8) {
    air,
    water,
    granular,
    smoke,
    ember,
};

pub const Particle = struct {
    active: bool = false,
    kind: ParticleKind = .generic,
    x: f32 = 0.0,
    y: f32 = 0.0,
    z: f32 = 0.0,
    vx: f32 = 0.0,
    vy: f32 = 0.0,
    vz: f32 = 0.0,
    radius: f32 = 0.05,
    mass: f32 = 1.0,
    age: f32 = 0.0,
    lifetime: f32 = 1.0,
    drag: f32 = 0.0,
    restitution: f32 = 0.0,
    friction: f32 = 0.0,
    temperature: f32 = 0.0,
    density: f32 = 1.0,
    seed: u32 = 0,

    pub fn alpha(self: Particle) f32 {
        if (!self.active or self.lifetime <= 0.0) return 0.0;
        return std.math.clamp(1.0 - self.age / self.lifetime, 0.0, 1.0);
    }
};

pub const Emitter = struct {
    active: bool = false,
    kind: ParticleKind = .generic,
    shape: EmitterShape = .point,
    medium: MediumPolicy = .air,
    x: f32 = 0.0,
    y: f32 = 0.0,
    z: f32 = 0.0,
    direction_x: f32 = 0.0,
    direction_y: f32 = 1.0,
    direction_z: f32 = 0.0,
    rate_per_second: f32 = 0.0,
    burst_count: u16 = 0,
    speed: f32 = 1.0,
    spread: f32 = 0.0,
    particle_lifetime: f32 = 1.0,
    particle_radius: f32 = 0.05,
    particle_mass: f32 = 1.0,
    drag: f32 = 0.0,
    restitution: f32 = 0.0,
    friction: f32 = 0.0,
    temperature: f32 = 0.0,
    density: f32 = 1.0,
    accumulator: f32 = 0.0,
    seed: u32 = 1,
};

pub const StepConfig = struct {
    dt: f32 = 1.0 / 60.0,
    gravity: f32 = -9.8,
    floor_y: f32 = 0.0,
    collide_floor: bool = true,
    wind_x: f32 = 0.0,
    wind_y: f32 = 0.0,
    wind_z: f32 = 0.0,
};

pub const Stats = struct {
    active_particles: u16 = 0,
    emitted_particles: u16 = 0,
    expired_particles: u16 = 0,
    floor_collisions: u16 = 0,
    average_height: f32 = 0.0,
    kinetic_energy: f32 = 0.0,
};

pub const Snapshot = struct {
    particles: [MAX_PARTICLES]Particle = [_]Particle{.{}} ** MAX_PARTICLES,
    emitters: [MAX_EMITTERS]Emitter = [_]Emitter{.{}} ** MAX_EMITTERS,
    next_particle: u16 = 0,
    stats: Stats = .{},
};

pub const ParticleSystem = struct {
    particles: [MAX_PARTICLES]Particle = [_]Particle{.{}} ** MAX_PARTICLES,
    emitters: [MAX_EMITTERS]Emitter = [_]Emitter{.{}} ** MAX_EMITTERS,
    next_particle: u16 = 0,
    stats: Stats = .{},

    pub fn init() ParticleSystem {
        return .{};
    }

    pub fn reset(self: *ParticleSystem) void {
        self.* = .{};
    }

    pub fn addEmitter(self: *ParticleSystem, emitter: Emitter) ?u8 {
        for (&self.emitters, 0..) |*slot, idx| {
            if (!slot.active) {
                slot.* = emitter;
                slot.active = true;
                if (slot.seed == 0) slot.seed = @as(u32, @intCast(idx + 1));
                return @as(u8, @intCast(idx));
            }
        }
        return null;
    }

    pub fn emitBurst(self: *ParticleSystem, emitter: Emitter, count: u16) u16 {
        var emitted: u16 = 0;
        var local = emitter;
        var remaining = count;
        while (remaining > 0) : (remaining -= 1) {
            if (self.spawnFromEmitter(&local)) emitted += 1;
        }
        self.stats.emitted_particles += emitted;
        return emitted;
    }

    pub fn step(self: *ParticleSystem, config: StepConfig) Stats {
        self.stats = .{};
        self.sanitizeState();
        const dt = @max(0.0, finiteOr(config.dt, 0.0));
        self.stepEmitters(dt);
        self.integrateParticles(.{
            .dt = dt,
            .gravity = finiteOr(config.gravity, -9.8),
            .floor_y = finiteOr(config.floor_y, 0.0),
            .collide_floor = config.collide_floor,
            .wind_x = finiteOr(config.wind_x, 0.0),
            .wind_y = finiteOr(config.wind_y, 0.0),
            .wind_z = finiteOr(config.wind_z, 0.0),
        });
        self.sanitizeState();
        self.recomputeStats();
        return self.stats;
    }

    pub fn sanitizeState(self: *ParticleSystem) void {
        for (&self.particles) |*particle| {
            if (!particle.active) continue;
            particle.x = finiteOr(particle.x, 0.0);
            particle.y = finiteOr(particle.y, 0.0);
            particle.z = finiteOr(particle.z, 0.0);
            particle.vx = finiteOr(particle.vx, 0.0);
            particle.vy = finiteOr(particle.vy, 0.0);
            particle.vz = finiteOr(particle.vz, 0.0);
            particle.radius = @max(0.001, finiteOr(particle.radius, 0.05));
            particle.mass = @max(0.001, finiteOr(particle.mass, 1.0));
            particle.age = @max(0.0, finiteOr(particle.age, 0.0));
            particle.lifetime = @max(0.001, finiteOr(particle.lifetime, 1.0));
            particle.drag = @max(0.0, finiteOr(particle.drag, 0.0));
            particle.restitution = finiteOr(particle.restitution, 0.0);
            particle.friction = finiteOr(particle.friction, 0.0);
            particle.temperature = @max(0.0, finiteOr(particle.temperature, 0.0));
            particle.density = @max(0.001, finiteOr(particle.density, 1.0));
        }
        for (&self.emitters) |*emitter| {
            if (!emitter.active) continue;
            emitter.x = finiteOr(emitter.x, 0.0);
            emitter.y = finiteOr(emitter.y, 0.0);
            emitter.z = finiteOr(emitter.z, 0.0);
            emitter.direction_x = finiteOr(emitter.direction_x, 0.0);
            emitter.direction_y = finiteOr(emitter.direction_y, 1.0);
            emitter.direction_z = finiteOr(emitter.direction_z, 0.0);
            emitter.rate_per_second = @max(0.0, finiteOr(emitter.rate_per_second, 0.0));
            emitter.speed = @max(0.0, finiteOr(emitter.speed, 0.0));
            emitter.spread = @max(0.0, finiteOr(emitter.spread, 0.0));
            emitter.particle_lifetime = @max(0.001, finiteOr(emitter.particle_lifetime, 1.0));
            emitter.particle_radius = @max(0.001, finiteOr(emitter.particle_radius, 0.05));
            emitter.particle_mass = @max(0.001, finiteOr(emitter.particle_mass, 1.0));
            emitter.drag = @max(0.0, finiteOr(emitter.drag, 0.0));
            emitter.accumulator = @max(0.0, finiteOr(emitter.accumulator, 0.0));
            emitter.density = @max(0.001, finiteOr(emitter.density, 1.0));
        }
    }

    pub fn snapshot(self: *const ParticleSystem) Snapshot {
        return .{
            .particles = self.particles,
            .emitters = self.emitters,
            .next_particle = self.next_particle,
            .stats = self.stats,
        };
    }

    pub fn restore(self: *ParticleSystem, snap: Snapshot) void {
        self.particles = snap.particles;
        self.emitters = snap.emitters;
        self.next_particle = if (snap.next_particle < MAX_PARTICLES) snap.next_particle else 0;
        self.stats = sanitizeStats(snap.stats);
        self.sanitizeState();
    }

    pub fn hash(self: *const ParticleSystem) u64 {
        var h = std.hash.Wyhash.init(0x7061727469636c65);
        h.update(std.mem.asBytes(&self.next_particle));
        for (self.particles) |particle| {
            if (!particle.active) continue;
            h.update(std.mem.asBytes(&particle.kind));
            h.update(std.mem.asBytes(&quantize(particle.x)));
            h.update(std.mem.asBytes(&quantize(particle.y)));
            h.update(std.mem.asBytes(&quantize(particle.z)));
            h.update(std.mem.asBytes(&quantize(particle.vx)));
            h.update(std.mem.asBytes(&quantize(particle.vy)));
            h.update(std.mem.asBytes(&quantize(particle.vz)));
            h.update(std.mem.asBytes(&quantize(particle.age)));
            h.update(std.mem.asBytes(&quantize(particle.lifetime)));
        }
        return h.final();
    }

    fn stepEmitters(self: *ParticleSystem, dt: f32) void {
        for (&self.emitters) |*emitter| {
            if (!emitter.active) continue;
            if (emitter.burst_count > 0) {
                _ = self.emitBurst(emitter.*, emitter.burst_count);
                emitter.burst_count = 0;
            }
            if (emitter.rate_per_second <= 0.0) continue;
            emitter.accumulator += emitter.rate_per_second * dt;
            while (emitter.accumulator >= 1.0) {
                if (self.spawnFromEmitter(emitter)) self.stats.emitted_particles += 1;
                emitter.accumulator -= 1.0;
            }
        }
    }

    fn integrateParticles(self: *ParticleSystem, config: StepConfig) void {
        const dt = if (config.dt > 0.0) config.dt else 0.0;
        for (&self.particles) |*particle| {
            if (!particle.active) continue;
            particle.age += dt;
            if (particle.age >= particle.lifetime) {
                particle.active = false;
                self.stats.expired_particles += 1;
                continue;
            }

            const drag = std.math.clamp(particle.drag, 0.0, 64.0);
            const drag_factor = @max(0.0, 1.0 - drag * dt);
            particle.vx = (particle.vx + config.wind_x * dt) * drag_factor;
            particle.vy = (particle.vy + (config.gravity + config.wind_y) * dt) * drag_factor;
            particle.vz = (particle.vz + config.wind_z * dt) * drag_factor;

            if (particle.temperature > 0.0) {
                const buoyancy = @min(12.0, particle.temperature * 0.06 / @max(0.1, particle.density));
                particle.vy += buoyancy * dt;
                particle.temperature = @max(0.0, particle.temperature - 8.0 * dt);
            }

            particle.x += particle.vx * dt;
            particle.y += particle.vy * dt;
            particle.z += particle.vz * dt;

            if (config.collide_floor and particle.y - particle.radius < config.floor_y) {
                particle.y = config.floor_y + particle.radius;
                if (particle.vy < 0.0) {
                    particle.vy = -particle.vy * std.math.clamp(particle.restitution, 0.0, 1.0);
                    particle.vx *= @max(0.0, 1.0 - particle.friction);
                    particle.vz *= @max(0.0, 1.0 - particle.friction);
                    self.stats.floor_collisions += 1;
                    if (@abs(particle.vy) < 0.05) particle.vy = 0.0;
                }
            }
        }
    }

    fn recomputeStats(self: *ParticleSystem) void {
        var height_sum: f32 = 0.0;
        for (self.particles) |particle| {
            if (!particle.active) continue;
            self.stats.active_particles += 1;
            height_sum += particle.y;
            self.stats.kinetic_energy += 0.5 * particle.mass * (particle.vx * particle.vx + particle.vy * particle.vy + particle.vz * particle.vz);
        }
        if (self.stats.active_particles > 0) {
            self.stats.average_height = height_sum / @as(f32, @floatFromInt(self.stats.active_particles));
        }
    }

    fn spawnFromEmitter(self: *ParticleSystem, emitter: *Emitter) bool {
        const slot = self.allocateParticleSlot() orelse return false;
        var seed = nextSeed(&emitter.seed);
        const jitter_a = signedUnit(&seed);
        const jitter_b = signedUnit(&seed);
        const jitter_c = signedUnit(&seed);
        const direction = normalized(.{
            emitter.direction_x + jitter_a * emitter.spread,
            emitter.direction_y + jitter_b * emitter.spread,
            emitter.direction_z + jitter_c * emitter.spread,
        });
        const offset = shapeOffset(emitter.shape, emitter.particle_radius, &seed);
        self.particles[slot] = .{
            .active = true,
            .kind = emitter.kind,
            .x = emitter.x + offset[0],
            .y = emitter.y + offset[1],
            .z = emitter.z + offset[2],
            .vx = direction[0] * emitter.speed,
            .vy = direction[1] * emitter.speed,
            .vz = direction[2] * emitter.speed,
            .radius = emitter.particle_radius,
            .mass = @max(0.001, emitter.particle_mass),
            .lifetime = @max(0.001, emitter.particle_lifetime),
            .drag = mediumDrag(emitter.medium, emitter.drag),
            .restitution = emitter.restitution,
            .friction = emitter.friction,
            .temperature = emitter.temperature,
            .density = @max(0.001, emitter.density),
            .seed = seed,
        };
        return true;
    }

    fn allocateParticleSlot(self: *ParticleSystem) ?usize {
        var scanned: u16 = 0;
        while (scanned < MAX_PARTICLES) : (scanned += 1) {
            const idx = self.next_particle;
            self.next_particle = (self.next_particle + 1) % MAX_PARTICLES;
            if (!self.particles[idx].active) return idx;
        }
        return null;
    }
};

fn mediumDrag(medium: MediumPolicy, base_drag: f32) f32 {
    return switch (medium) {
        .air => base_drag,
        .water => base_drag + 2.5,
        .granular => base_drag + 5.0,
        .smoke => base_drag + 1.2,
        .ember => base_drag + 0.6,
    };
}

fn shapeOffset(shape: EmitterShape, radius: f32, seed: *u32) [3]f32 {
    return switch (shape) {
        .point => .{ 0.0, 0.0, 0.0 },
        .cone, .disc => .{ signedUnit(seed) * radius, 0.0, signedUnit(seed) * radius },
        .sphere => .{ signedUnit(seed) * radius, signedUnit(seed) * radius, signedUnit(seed) * radius },
    };
}

fn normalized(v: [3]f32) [3]f32 {
    const len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    if (len_sq <= 0.000001) return .{ 0.0, 1.0, 0.0 };
    const inv_len = 1.0 / @sqrt(len_sq);
    return .{ v[0] * inv_len, v[1] * inv_len, v[2] * inv_len };
}

fn nextSeed(seed: *u32) u32 {
    seed.* = seed.* *% 1664525 +% 1013904223;
    return seed.*;
}

fn signedUnit(seed: *u32) f32 {
    const value = nextSeed(seed);
    const normalized_value = @as(f32, @floatFromInt(value & 0xffff)) / 32767.5 - 1.0;
    return std.math.clamp(normalized_value, -1.0, 1.0);
}

fn finiteOr(value: f32, fallback: f32) f32 {
    return if (std.math.isFinite(value)) value else fallback;
}

fn sanitizeStats(stats: Stats) Stats {
    return .{
        .active_particles = stats.active_particles,
        .emitted_particles = stats.emitted_particles,
        .expired_particles = stats.expired_particles,
        .floor_collisions = stats.floor_collisions,
        .average_height = finiteOr(stats.average_height, 0.0),
        .kinetic_energy = finiteOr(stats.kinetic_energy, 0.0),
    };
}

fn quantize(value: f32) i32 {
    if (!std.math.isFinite(value)) return 0;
    return @as(i32, @intFromFloat(@round(value * 10000.0)));
}

test "particle system emits deterministic burst" {
    var system = ParticleSystem.init();
    const emitted = system.emitBurst(.{
        .kind = .dust,
        .medium = .air,
        .x = 1.0,
        .y = 2.0,
        .z = 3.0,
        .direction_y = 1.0,
        .burst_count = 4,
        .speed = 2.0,
        .particle_lifetime = 1.0,
        .seed = 42,
    }, 4);
    try std.testing.expectEqual(@as(u16, 4), emitted);
    const stats = system.step(.{ .dt = 0.1, .gravity = 0.0 });
    try std.testing.expectEqual(@as(u16, 4), stats.active_particles);
    try std.testing.expect(stats.average_height > 2.0);
}

test "particle system collides with floor and applies friction" {
    var system = ParticleSystem.init();
    _ = system.emitBurst(.{
        .kind = .sand,
        .medium = .air,
        .y = 0.2,
        .direction_y = -1.0,
        .speed = 2.0,
        .particle_lifetime = 2.0,
        .particle_radius = 0.1,
        .restitution = 0.2,
        .friction = 0.5,
        .seed = 7,
    }, 1);
    const stats = system.step(.{ .dt = 0.2, .gravity = -9.8, .floor_y = 0.0 });
    try std.testing.expectEqual(@as(u16, 1), stats.active_particles);
    try std.testing.expect(stats.floor_collisions >= 1);
    try std.testing.expect(system.particles[0].y >= 0.1);
}

test "particle snapshot restore preserves hash" {
    var system = ParticleSystem.init();
    _ = system.emitBurst(.{ .kind = .smoke, .medium = .smoke, .temperature = 120.0, .particle_lifetime = 3.0, .seed = 9 }, 3);
    _ = system.step(.{ .dt = 0.05 });
    const snap = system.snapshot();
    const before = system.hash();
    _ = system.step(.{ .dt = 0.25 });
    system.restore(snap);
    try std.testing.expectEqual(before, system.hash());
}

test "particle system sanitizes non-finite particle and emitter state" {
    var system = ParticleSystem.init();
    system.particles[0] = .{ .active = true, .x = std.math.nan(f32), .y = std.math.inf(f32), .z = 1.0, .radius = std.math.nan(f32), .mass = -1.0, .lifetime = std.math.nan(f32) };
    _ = system.addEmitter(.{ .active = true, .rate_per_second = std.math.inf(f32), .x = std.math.nan(f32), .particle_lifetime = std.math.nan(f32), .particle_radius = -1.0, .particle_mass = -1.0 });
    const stats = system.step(.{ .dt = std.math.nan(f32), .gravity = std.math.inf(f32) });
    try std.testing.expect(stats.active_particles >= 1);
    try std.testing.expect(std.math.isFinite(system.particles[0].x));
    try std.testing.expect(std.math.isFinite(system.particles[0].y));
    try std.testing.expect(system.particles[0].radius > 0.0);
    try std.testing.expect(system.particles[0].mass > 0.0);
    try std.testing.expect(system.particles[0].lifetime > 0.0);
}

test "particle system clamps negative dt to zero" {
    var system = ParticleSystem.init();
    _ = system.emitBurst(.{
        .kind = .dust,
        .medium = .air,
        .x = 1.0,
        .y = 2.0,
        .z = 3.0,
        .direction_y = 1.0,
        .speed = 2.0,
        .particle_lifetime = 1.0,
        .seed = 42,
    }, 1);

    const before_y = system.particles[0].y;
    const before_age = system.particles[0].age;
    const stats = system.step(.{ .dt = -0.25, .gravity = -9.8 });

    try std.testing.expectEqual(@as(u16, 1), stats.active_particles);
    try std.testing.expectApproxEqAbs(before_y, system.particles[0].y, 0.0001);
    try std.testing.expectApproxEqAbs(before_age, system.particles[0].age, 0.0001);
}

test "particle restore clamps invalid snapshot cursor" {
    var system = ParticleSystem.init();
    var snap = system.snapshot();
    snap.next_particle = MAX_PARTICLES + 1;
    snap.particles[0] = .{ .active = true, .x = std.math.nan(f32), .radius = -1.0, .mass = -1.0, .lifetime = -1.0 };
    snap.stats.average_height = std.math.inf(f32);
    snap.stats.kinetic_energy = std.math.nan(f32);

    system.restore(snap);

    try std.testing.expectEqual(@as(u16, 0), system.next_particle);
    try std.testing.expect(std.math.isFinite(system.particles[0].x));
    try std.testing.expect(system.particles[0].radius > 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), system.stats.average_height, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), system.stats.kinetic_energy, 0.0001);
    _ = system.hash();
}

test "particle system handles negative dt gracefully" {
    var system = ParticleSystem.init();
    system.particles[0] = .{
        .active = true,
        .x = 1.0,
        .y = 2.0,
        .z = 3.0,
        .vx = 1.0,
        .vy = 1.0,
        .vz = 1.0,
        .radius = 0.5,
        .mass = 1.0,
        .lifetime = 2.0,
        .age = 0.5,
    };
    const before = system.particles[0];
    _ = system.step(.{ .dt = -1.0 });
    try std.testing.expectEqual(@as(u16, 1), system.stats.active_particles);
    try std.testing.expectApproxEqAbs(before.x, system.particles[0].x, 0.0001);
    try std.testing.expectApproxEqAbs(before.y, system.particles[0].y, 0.0001);
    try std.testing.expectApproxEqAbs(before.z, system.particles[0].z, 0.0001);
    try std.testing.expectApproxEqAbs(before.age, system.particles[0].age, 0.0001);
    try std.testing.expect(!std.math.isNan(system.stats.average_height));
    _ = system.hash();
}
