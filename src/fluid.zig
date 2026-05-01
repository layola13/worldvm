//! Deterministic height-field fluid simulation core.

const std = @import("std");

pub const MAX_WIDTH = 16;
pub const MAX_HEIGHT = 16;
pub const MAX_FLOATERS = 32;

pub const FluidKind = enum(u8) {
    water,
    oil,
    mud,
    foam,
};

pub const Cell = struct {
    height: f32 = 0.0,
    velocity_x: f32 = 0.0,
    velocity_z: f32 = 0.0,
    wave_velocity: f32 = 0.0,
    depth: f32 = 0.0,
    blocked: bool = false,
};

pub const Floater = struct {
    active: bool = false,
    x: f32 = 0.0,
    y: f32 = 0.0,
    z: f32 = 0.0,
    vx: f32 = 0.0,
    vy: f32 = 0.0,
    vz: f32 = 0.0,
    radius: f32 = 0.5,
    mass: f32 = 1.0,
    volume: f32 = 1.0,
    drag_area: f32 = 1.0,
    density: f32 = 0.8,
};

pub const StepConfig = struct {
    dt: f32 = 1.0 / 60.0,
    gravity: f32 = 9.8,
    fluid_density: f32 = 1.0,
    viscosity: f32 = 0.15,
    wave_speed: f32 = 3.0,
    wave_damping: f32 = 0.12,
    flow_decay: f32 = 0.03,
    surface_y: f32 = 0.0,
};

pub const Stats = struct {
    active_floaters: u8 = 0,
    submerged_floaters: u8 = 0,
    floor_contacts: u8 = 0,
    displaced_volume: f32 = 0.0,
    buoyancy_force: f32 = 0.0,
    drag_force: f32 = 0.0,
    max_wave_height: f32 = 0.0,
    average_height: f32 = 0.0,
    flow_energy: f32 = 0.0,
};

pub const Snapshot = struct {
    kind: FluidKind = .water,
    width: u8 = 0,
    height: u8 = 0,
    cells: [MAX_WIDTH * MAX_HEIGHT]Cell = [_]Cell{.{}} ** (MAX_WIDTH * MAX_HEIGHT),
    floaters: [MAX_FLOATERS]Floater = [_]Floater{.{}} ** MAX_FLOATERS,
    stats: Stats = .{},
};

pub const FluidWorld = struct {
    kind: FluidKind = .water,
    width: u8 = 0,
    height: u8 = 0,
    cells: [MAX_WIDTH * MAX_HEIGHT]Cell = [_]Cell{.{}} ** (MAX_WIDTH * MAX_HEIGHT),
    floaters: [MAX_FLOATERS]Floater = [_]Floater{.{}} ** MAX_FLOATERS,
    stats: Stats = .{},

    pub fn init(kind: FluidKind, width: u8, height: u8, base_depth: f32) FluidWorld {
        var world: FluidWorld = .{
            .kind = kind,
            .width = @min(width, MAX_WIDTH),
            .height = @min(height, MAX_HEIGHT),
        };
        for (world.cells[0..world.cellCount()]) |*cell| {
            cell.depth = @max(0.0, base_depth);
            cell.height = 0.0;
        }
        return world;
    }

    pub fn addFloater(self: *FluidWorld, floater: Floater) ?u8 {
        for (&self.floaters, 0..) |*slot, idx| {
            if (!slot.active) {
                slot.* = floater;
                slot.active = true;
                return @as(u8, @intCast(idx));
            }
        }
        return null;
    }

    pub fn disturb(self: *FluidWorld, x: f32, z: f32, radius: f32, strength: f32) void {
        var row: u8 = 0;
        while (row < self.height) : (row += 1) {
            var col: u8 = 0;
            while (col < self.width) : (col += 1) {
                const dx = @as(f32, @floatFromInt(col)) - x;
                const dz = @as(f32, @floatFromInt(row)) - z;
                const dist = @sqrt(dx * dx + dz * dz);
                if (dist > radius) continue;
                const falloff = 1.0 - dist / @max(0.001, radius);
                const cell = self.cellAt(col, row).?;
                cell.wave_velocity += strength * falloff;
            }
        }
    }

    pub fn setFlow(self: *FluidWorld, velocity_x: f32, velocity_z: f32) void {
        for (self.cells[0..self.cellCount()]) |*cell| {
            if (cell.blocked) continue;
            cell.velocity_x = velocity_x;
            cell.velocity_z = velocity_z;
        }
    }

    pub fn setBlocked(self: *FluidWorld, col: u8, row: u8, blocked: bool) void {
        if (self.cellAt(col, row)) |cell| cell.blocked = blocked;
    }

    pub fn step(self: *FluidWorld, config: StepConfig) Stats {
        self.stats = .{};
        self.sanitizeState();
        const dt = @max(0.0, finiteOr(config.dt, 0.0));
        self.stepWaves(config, dt);
        self.stepFloaters(config, dt);
        self.sanitizeState();
        self.recomputeStats();
        return self.stats;
    }

    pub fn sanitizeState(self: *FluidWorld) void {
        for (self.cells[0..self.cellCount()]) |*cell| {
            cell.height = finiteOr(cell.height, 0.0);
            cell.velocity_x = finiteOr(cell.velocity_x, 0.0);
            cell.velocity_z = finiteOr(cell.velocity_z, 0.0);
            cell.wave_velocity = finiteOr(cell.wave_velocity, 0.0);
            cell.depth = @max(0.0, finiteOr(cell.depth, 0.0));
        }
        for (&self.floaters) |*floater| {
            if (!floater.active) continue;
            floater.x = finiteOr(floater.x, 0.0);
            floater.y = finiteOr(floater.y, 0.0);
            floater.z = finiteOr(floater.z, 0.0);
            floater.vx = finiteOr(floater.vx, 0.0);
            floater.vy = finiteOr(floater.vy, 0.0);
            floater.vz = finiteOr(floater.vz, 0.0);
            floater.radius = @max(0.001, finiteOr(floater.radius, 0.5));
            floater.mass = @max(0.001, finiteOr(floater.mass, 1.0));
            floater.volume = @max(0.0, finiteOr(floater.volume, 0.0));
            floater.drag_area = @max(0.0, finiteOr(floater.drag_area, 0.0));
            floater.density = @max(0.001, finiteOr(floater.density, 1.0));
        }
    }

    pub fn snapshot(self: *const FluidWorld) Snapshot {
        return .{
            .kind = self.kind,
            .width = self.width,
            .height = self.height,
            .cells = self.cells,
            .floaters = self.floaters,
            .stats = self.stats,
        };
    }

    pub fn restore(self: *FluidWorld, snap: Snapshot) void {
        self.kind = snap.kind;
        self.width = @min(snap.width, MAX_WIDTH);
        self.height = @min(snap.height, MAX_HEIGHT);
        self.cells = snap.cells;
        self.floaters = snap.floaters;
        self.stats = sanitizeStats(snap.stats);
        self.sanitizeState();
    }

    pub fn hash(self: *const FluidWorld) u64 {
        var h = std.hash.Wyhash.init(0x666c756964);
        h.update(std.mem.asBytes(&self.kind));
        h.update(std.mem.asBytes(&self.width));
        h.update(std.mem.asBytes(&self.height));
        for (self.cells[0..self.cellCount()]) |cell| {
            h.update(std.mem.asBytes(&quantize(cell.height)));
            h.update(std.mem.asBytes(&quantize(cell.velocity_x)));
            h.update(std.mem.asBytes(&quantize(cell.velocity_z)));
            h.update(std.mem.asBytes(&quantize(cell.wave_velocity)));
            h.update(std.mem.asBytes(&quantize(cell.depth)));
            h.update(std.mem.asBytes(&cell.blocked));
        }
        for (self.floaters) |floater| {
            if (!floater.active) continue;
            h.update(std.mem.asBytes(&quantize(floater.x)));
            h.update(std.mem.asBytes(&quantize(floater.y)));
            h.update(std.mem.asBytes(&quantize(floater.z)));
            h.update(std.mem.asBytes(&quantize(floater.vx)));
            h.update(std.mem.asBytes(&quantize(floater.vy)));
            h.update(std.mem.asBytes(&quantize(floater.vz)));
        }
        return h.final();
    }

    pub fn cellCount(self: *const FluidWorld) usize {
        return @as(usize, self.width) * @as(usize, self.height);
    }

    fn cellAt(self: *FluidWorld, col: u8, row: u8) ?*Cell {
        if (col >= self.width or row >= self.height) return null;
        return &self.cells[@as(usize, row) * self.width + col];
    }

    fn sampleCell(self: *const FluidWorld, x: f32, z: f32) Cell {
        if (self.width == 0 or self.height == 0) return .{};
        const safe_x = finiteOr(x, 0.0);
        const safe_z = finiteOr(z, 0.0);
        const col = std.math.clamp(@as(i32, @intFromFloat(@round(safe_x))), 0, @as(i32, self.width) - 1);
        const row = std.math.clamp(@as(i32, @intFromFloat(@round(safe_z))), 0, @as(i32, self.height) - 1);
        return self.cells[@as(usize, @intCast(row)) * self.width + @as(usize, @intCast(col))];
    }

    fn stepWaves(self: *FluidWorld, config: StepConfig, dt: f32) void {
        var next_heights = [_]f32{0.0} ** (MAX_WIDTH * MAX_HEIGHT);
        var row: u8 = 0;
        while (row < self.height) : (row += 1) {
            var col: u8 = 0;
            while (col < self.width) : (col += 1) {
                const idx = @as(usize, row) * self.width + col;
                var cell = &self.cells[idx];
                if (cell.blocked) {
                    next_heights[idx] = cell.height;
                    continue;
                }
                var neighbor_sum: f32 = 0.0;
                var neighbor_count: f32 = 0.0;
                if (col > 0) {
                    neighbor_sum += self.cells[idx - 1].height;
                    neighbor_count += 1.0;
                }
                if (col + 1 < self.width) {
                    neighbor_sum += self.cells[idx + 1].height;
                    neighbor_count += 1.0;
                }
                if (row > 0) {
                    neighbor_sum += self.cells[idx - self.width].height;
                    neighbor_count += 1.0;
                }
                if (row + 1 < self.height) {
                    neighbor_sum += self.cells[idx + self.width].height;
                    neighbor_count += 1.0;
                }
                const laplacian = if (neighbor_count > 0.0) neighbor_sum / neighbor_count - cell.height else 0.0;
                cell.wave_velocity += laplacian * config.wave_speed * dt;
                cell.wave_velocity *= @max(0.0, 1.0 - config.wave_damping * dt);
                next_heights[idx] = cell.height + cell.wave_velocity * dt;
                cell.velocity_x *= @max(0.0, 1.0 - config.flow_decay * dt);
                cell.velocity_z *= @max(0.0, 1.0 - config.flow_decay * dt);
            }
        }
        for (self.cells[0..self.cellCount()], 0..) |*cell, idx| {
            cell.height = next_heights[idx];
        }
    }

    fn stepFloaters(self: *FluidWorld, config: StepConfig, dt: f32) void {
        for (&self.floaters) |*floater| {
            if (!floater.active) continue;
            const cell = self.sampleCell(floater.x, floater.z);
            const surface = config.surface_y + cell.height;
            const submerged = submergedFraction(floater.*, surface);
            const displaced = floater.volume * submerged;
            const buoyancy = displaced * config.fluid_density * config.gravity;
            const weight = floater.mass * config.gravity;
            const relative_vx = floater.vx - cell.velocity_x;
            const relative_vz = floater.vz - cell.velocity_z;
            const drag_scale = config.viscosity * floater.drag_area * submerged;
            const drag_x = -relative_vx * drag_scale;
            const drag_z = -relative_vz * drag_scale;

            floater.vx += (drag_x / @max(0.001, floater.mass) + cell.velocity_x * submerged * 0.1) * dt;
            floater.vy += ((buoyancy - weight) / @max(0.001, floater.mass)) * dt;
            floater.vz += (drag_z / @max(0.001, floater.mass) + cell.velocity_z * submerged * 0.1) * dt;
            floater.x += floater.vx * dt;
            floater.y += floater.vy * dt;
            floater.z += floater.vz * dt;

            const floor_y = config.surface_y - cell.depth;
            if (floater.y - floater.radius < floor_y) {
                floater.y = floor_y + floater.radius;
                if (floater.vy < 0.0) floater.vy = 0.0;
                self.stats.floor_contacts += 1;
            }

            self.stats.active_floaters += 1;
            if (submerged > 0.0) self.stats.submerged_floaters += 1;
            self.stats.displaced_volume += displaced;
            self.stats.buoyancy_force += buoyancy;
            self.stats.drag_force += @sqrt(drag_x * drag_x + drag_z * drag_z);
        }
    }

    fn recomputeStats(self: *FluidWorld) void {
        var height_sum: f32 = 0.0;
        for (self.cells[0..self.cellCount()]) |cell| {
            const abs_height = @abs(cell.height);
            self.stats.max_wave_height = @max(self.stats.max_wave_height, abs_height);
            height_sum += cell.height;
            self.stats.flow_energy += cell.velocity_x * cell.velocity_x + cell.velocity_z * cell.velocity_z + cell.wave_velocity * cell.wave_velocity;
        }
        if (self.cellCount() > 0) {
            self.stats.average_height = height_sum / @as(f32, @floatFromInt(self.cellCount()));
        }
    }
};

pub fn submergedFraction(floater: Floater, surface_y: f32) f32 {
    const bottom = floater.y - floater.radius;
    const top = floater.y + floater.radius;
    if (surface_y <= bottom) return 0.0;
    if (surface_y >= top) return 1.0;
    return std.math.clamp((surface_y - bottom) / @max(0.001, top - bottom), 0.0, 1.0);
}

pub fn makePool(width: u8, height: u8, depth: f32) FluidWorld {
    return FluidWorld.init(.water, width, height, depth);
}

fn finiteOr(value: f32, fallback: f32) f32 {
    return if (std.math.isFinite(value)) value else fallback;
}

fn sanitizeStats(stats: Stats) Stats {
    return .{
        .active_floaters = stats.active_floaters,
        .submerged_floaters = stats.submerged_floaters,
        .floor_contacts = stats.floor_contacts,
        .displaced_volume = finiteOr(stats.displaced_volume, 0.0),
        .buoyancy_force = finiteOr(stats.buoyancy_force, 0.0),
        .drag_force = finiteOr(stats.drag_force, 0.0),
        .max_wave_height = finiteOr(stats.max_wave_height, 0.0),
        .average_height = finiteOr(stats.average_height, 0.0),
        .flow_energy = finiteOr(stats.flow_energy, 0.0),
    };
}

fn quantize(value: f32) i32 {
    if (!std.math.isFinite(value)) return 0;
    return @as(i32, @intFromFloat(@round(value * 10000.0)));
}

test "fluid buoyancy floats light body and sinks dense body" {
    var world = makePool(6, 6, 3.0);
    _ = world.addFloater(.{ .x = 2.0, .y = -0.2, .z = 2.0, .radius = 0.5, .mass = 0.5, .volume = 1.0 });
    _ = world.addFloater(.{ .x = 3.0, .y = -0.2, .z = 3.0, .radius = 0.5, .mass = 3.0, .volume = 1.0 });
    var stats: Stats = .{};
    var tick: u8 = 0;
    while (tick < 20) : (tick += 1) stats = world.step(.{ .dt = 1.0 / 30.0 });
    try std.testing.expectEqual(@as(u8, 2), stats.active_floaters);
    try std.testing.expect(stats.buoyancy_force > 0.0);
    try std.testing.expect(world.floaters[0].y > world.floaters[1].y);
}

test "fluid flow pushes submerged floater" {
    var world = makePool(6, 6, 2.0);
    world.setFlow(2.0, 0.0);
    _ = world.addFloater(.{ .x = 1.0, .y = -0.2, .z = 1.0, .radius = 0.4, .mass = 1.0, .volume = 1.0, .drag_area = 2.0 });
    var stats: Stats = .{};
    var tick: u8 = 0;
    while (tick < 20) : (tick += 1) stats = world.step(.{ .dt = 1.0 / 30.0, .viscosity = 0.8 });
    try std.testing.expect(stats.drag_force > 0.0);
    try std.testing.expect(world.floaters[0].x > 1.0);
}

test "fluid snapshot restore preserves hash" {
    var world = makePool(4, 4, 1.0);
    world.disturb(2.0, 2.0, 2.0, 1.5);
    _ = world.step(.{ .dt = 1.0 / 60.0 });
    const snap = world.snapshot();
    const before = world.hash();
    _ = world.step(.{ .dt = 0.5 });
    world.restore(snap);
    try std.testing.expectEqual(before, world.hash());
}

test "fluid sanitizes non-finite cell and floater state before stepping" {
    var world = makePool(4, 4, 2.0);
    world.cells[0].height = std.math.nan(f32);
    world.cells[0].velocity_x = std.math.inf(f32);
    _ = world.addFloater(.{ .active = true, .x = std.math.nan(f32), .y = -0.2, .z = std.math.inf(f32), .radius = std.math.nan(f32), .mass = -1.0, .volume = std.math.inf(f32) });
    const stats = world.step(.{ .dt = std.math.nan(f32) });
    try std.testing.expectEqual(@as(u8, 1), stats.active_floaters);
    try std.testing.expect(std.math.isFinite(world.cells[0].height));
    try std.testing.expect(std.math.isFinite(world.floaters[0].x));
    try std.testing.expect(std.math.isFinite(world.floaters[0].z));
    try std.testing.expect(world.floaters[0].radius > 0.0);
    try std.testing.expect(world.floaters[0].mass > 0.0);
}

test "fluid restore clamps invalid snapshot dimensions" {
    var world = makePool(2, 2, 1.0);
    var snap = world.snapshot();
    snap.width = MAX_WIDTH + 1;
    snap.height = MAX_HEIGHT + 1;
    snap.cells[0].height = std.math.nan(f32);
    snap.stats.average_height = std.math.nan(f32);
    snap.stats.flow_energy = std.math.inf(f32);

    world.restore(snap);

    try std.testing.expectEqual(@as(u8, MAX_WIDTH), world.width);
    try std.testing.expectEqual(@as(u8, MAX_HEIGHT), world.height);
    try std.testing.expect(std.math.isFinite(world.cells[0].height));
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), world.stats.average_height, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), world.stats.flow_energy, 0.0001);
    _ = world.hash();
}
