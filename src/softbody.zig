//! Deterministic mass-spring soft body simulation core.

const std = @import("std");

pub const MAX_NODES = 64;
pub const MAX_SPRINGS = 128;

pub const SoftBodyKind = enum(u8) {
    generic,
    ball,
    jelly,
    balloon,
    band,
    cloth,
    rope,
    chain,
};

pub const Node = struct {
    active: bool = false,
    pinned: bool = false,
    x: f32 = 0.0,
    y: f32 = 0.0,
    z: f32 = 0.0,
    vx: f32 = 0.0,
    vy: f32 = 0.0,
    vz: f32 = 0.0,
    inv_mass: f32 = 1.0,
    radius: f32 = 0.1,
};

pub const Spring = struct {
    active: bool = false,
    a: u8 = 0,
    b: u8 = 0,
    rest_length: f32 = 1.0,
    stiffness: f32 = 80.0,
    damping: f32 = 1.0,
    strain_limit: f32 = 0.75,
    broken: bool = false,
};

pub const StepConfig = struct {
    dt: f32 = 1.0 / 60.0,
    gravity: f32 = -9.8,
    floor_y: f32 = 0.0,
    collide_floor: bool = true,
    floor_restitution: f32 = 0.1,
    floor_friction: f32 = 0.45,
    air_drag: f32 = 0.02,
    pressure: f32 = 0.0,
    solver_iterations: u8 = 2,
};

pub const Stats = struct {
    active_nodes: u8 = 0,
    active_springs: u8 = 0,
    broken_springs: u8 = 0,
    floor_contacts: u8 = 0,
    max_strain: f32 = 0.0,
    center_x: f32 = 0.0,
    center_y: f32 = 0.0,
    center_z: f32 = 0.0,
    kinetic_energy: f32 = 0.0,
    potential_energy: f32 = 0.0,
};

pub const Snapshot = struct {
    kind: SoftBodyKind = .generic,
    nodes: [MAX_NODES]Node = [_]Node{.{}} ** MAX_NODES,
    springs: [MAX_SPRINGS]Spring = [_]Spring{.{}} ** MAX_SPRINGS,
    node_count: u8 = 0,
    spring_count: u8 = 0,
    stats: Stats = .{},
};

pub const SoftBody = struct {
    kind: SoftBodyKind = .generic,
    nodes: [MAX_NODES]Node = [_]Node{.{}} ** MAX_NODES,
    springs: [MAX_SPRINGS]Spring = [_]Spring{.{}} ** MAX_SPRINGS,
    node_count: u8 = 0,
    spring_count: u8 = 0,
    stats: Stats = .{},

    pub fn init(kind: SoftBodyKind) SoftBody {
        return .{ .kind = kind };
    }

    pub fn reset(self: *SoftBody, kind: SoftBodyKind) void {
        self.* = init(kind);
    }

    pub fn addNode(self: *SoftBody, node: Node) ?u8 {
        if (self.node_count >= MAX_NODES) return null;
        const idx = self.node_count;
        self.nodes[idx] = node;
        self.nodes[idx].active = true;
        if (self.nodes[idx].pinned) self.nodes[idx].inv_mass = 0.0;
        self.node_count += 1;
        return idx;
    }

    pub fn addSpring(self: *SoftBody, a: u8, b: u8, stiffness: f32, damping: f32, strain_limit: f32) ?u8 {
        if (self.spring_count >= MAX_SPRINGS or a >= self.node_count or b >= self.node_count or a == b) return null;
        const rest_length = distance(self.nodes[a], self.nodes[b]);
        const idx = self.spring_count;
        self.springs[idx] = .{
            .active = true,
            .a = a,
            .b = b,
            .rest_length = @max(0.0001, rest_length),
            .stiffness = stiffness,
            .damping = damping,
            .strain_limit = strain_limit,
        };
        self.spring_count += 1;
        return idx;
    }

    pub fn step(self: *SoftBody, config: StepConfig) Stats {
        self.stats = .{};
        self.sanitizeState();
        const dt = @max(0.0, finiteOr(config.dt, 0.0));
        const safe_config = StepConfig{
            .dt = dt,
            .gravity = finiteOr(config.gravity, -9.8),
            .floor_y = finiteOr(config.floor_y, 0.0),
            .collide_floor = config.collide_floor,
            .floor_restitution = finiteOr(config.floor_restitution, 0.1),
            .floor_friction = finiteOr(config.floor_friction, 0.45),
            .air_drag = finiteOr(config.air_drag, 0.02),
            .pressure = finiteOr(config.pressure, 0.0),
            .solver_iterations = config.solver_iterations,
        };
        self.applyForces(safe_config, dt);
        self.integrate(safe_config, dt);
        var iteration: u8 = 0;
        while (iteration < config.solver_iterations) : (iteration += 1) {
            self.solveSprings(dt);
            self.solveFloor(safe_config);
        }
        self.sanitizeState();
        self.recomputeStats();
        return self.stats;
    }

    pub fn sanitizeState(self: *SoftBody) void {
        for (self.nodes[0..self.node_count]) |*node| {
            if (!node.active) continue;
            node.x = finiteOr(node.x, 0.0);
            node.y = finiteOr(node.y, 0.0);
            node.z = finiteOr(node.z, 0.0);
            node.vx = finiteOr(node.vx, 0.0);
            node.vy = finiteOr(node.vy, 0.0);
            node.vz = finiteOr(node.vz, 0.0);
            node.inv_mass = @max(0.0, finiteOr(node.inv_mass, 1.0));
            node.radius = @max(0.001, finiteOr(node.radius, 0.1));
            if (node.pinned) node.inv_mass = 0.0;
        }
        for (self.springs[0..self.spring_count]) |*spring| {
            if (!spring.active) continue;
            spring.rest_length = @max(0.0001, finiteOr(spring.rest_length, 1.0));
            spring.stiffness = @max(0.0, finiteOr(spring.stiffness, 80.0));
            spring.damping = @max(0.0, finiteOr(spring.damping, 1.0));
            spring.strain_limit = @max(0.0001, finiteOr(spring.strain_limit, 0.75));
            if (spring.a >= self.node_count or spring.b >= self.node_count or spring.a == spring.b) spring.broken = true;
        }
    }

    pub fn snapshot(self: *const SoftBody) Snapshot {
        return .{
            .kind = self.kind,
            .nodes = self.nodes,
            .springs = self.springs,
            .node_count = self.node_count,
            .spring_count = self.spring_count,
            .stats = self.stats,
        };
    }

    pub fn restore(self: *SoftBody, snap: Snapshot) void {
        self.kind = snap.kind;
        self.nodes = snap.nodes;
        self.springs = snap.springs;
        self.node_count = @min(snap.node_count, MAX_NODES);
        self.spring_count = @min(snap.spring_count, MAX_SPRINGS);
        self.stats = sanitizeStats(snap.stats);
        self.sanitizeState();
    }

    pub fn hash(self: *const SoftBody) u64 {
        var h = std.hash.Wyhash.init(0x736f6674626f6479);
        h.update(std.mem.asBytes(&self.kind));
        h.update(std.mem.asBytes(&self.node_count));
        h.update(std.mem.asBytes(&self.spring_count));
        for (self.nodes[0..self.node_count]) |node| {
            h.update(std.mem.asBytes(&node.active));
            h.update(std.mem.asBytes(&node.pinned));
            h.update(std.mem.asBytes(&quantize(node.x)));
            h.update(std.mem.asBytes(&quantize(node.y)));
            h.update(std.mem.asBytes(&quantize(node.z)));
            h.update(std.mem.asBytes(&quantize(node.vx)));
            h.update(std.mem.asBytes(&quantize(node.vy)));
            h.update(std.mem.asBytes(&quantize(node.vz)));
        }
        for (self.springs[0..self.spring_count]) |spring| {
            h.update(std.mem.asBytes(&spring.active));
            h.update(std.mem.asBytes(&spring.broken));
            h.update(std.mem.asBytes(&spring.a));
            h.update(std.mem.asBytes(&spring.b));
            h.update(std.mem.asBytes(&quantize(spring.rest_length)));
        }
        return h.final();
    }

    fn applyForces(self: *SoftBody, config: StepConfig, dt: f32) void {
        const drag_factor = @max(0.0, 1.0 - @max(0.0, config.air_drag) * dt);
        for (self.nodes[0..self.node_count]) |*node| {
            if (!node.active or node.pinned or node.inv_mass <= 0.0) continue;
            node.vy += config.gravity * dt;
            node.vx *= drag_factor;
            node.vy *= drag_factor;
            node.vz *= drag_factor;
        }
        if (config.pressure != 0.0) self.applyPressure(config.pressure, dt);
    }

    fn applyPressure(self: *SoftBody, pressure: f32, dt: f32) void {
        const body_center = self.center();
        for (self.nodes[0..self.node_count]) |*node| {
            if (!node.active or node.pinned or node.inv_mass <= 0.0) continue;
            const dx = node.x - body_center[0];
            const dy = node.y - body_center[1];
            const dz = node.z - body_center[2];
            const len_sq = dx * dx + dy * dy + dz * dz;
            if (len_sq <= 0.000001) continue;
            const inv_len = 1.0 / @sqrt(len_sq);
            node.vx += dx * inv_len * pressure * dt * node.inv_mass;
            node.vy += dy * inv_len * pressure * dt * node.inv_mass;
            node.vz += dz * inv_len * pressure * dt * node.inv_mass;
        }
    }

    fn integrate(self: *SoftBody, config: StepConfig, dt: f32) void {
        _ = config;
        for (self.nodes[0..self.node_count]) |*node| {
            if (!node.active or node.pinned) continue;
            node.x += node.vx * dt;
            node.y += node.vy * dt;
            node.z += node.vz * dt;
        }
    }

    fn solveSprings(self: *SoftBody, dt: f32) void {
        for (self.springs[0..self.spring_count]) |*spring| {
            if (!spring.active or spring.broken) continue;
            var a = &self.nodes[spring.a];
            var b = &self.nodes[spring.b];
            if (!a.active or !b.active) continue;
            const dx = b.x - a.x;
            const dy = b.y - a.y;
            const dz = b.z - a.z;
            const len = @sqrt(dx * dx + dy * dy + dz * dz);
            if (len <= 0.000001) continue;
            const strain = @abs(len - spring.rest_length) / spring.rest_length;
            if (strain > spring.strain_limit) {
                spring.broken = true;
                continue;
            }
            const nx = dx / len;
            const ny = dy / len;
            const nz = dz / len;
            const inv_mass_sum = a.inv_mass + b.inv_mass;
            if (inv_mass_sum <= 0.0) continue;
            const positional_error = len - spring.rest_length;
            const compliance = 1.0 / @max(1.0, spring.stiffness);
            const correction = positional_error / (inv_mass_sum + compliance / @max(0.000001, dt * dt));
            if (!a.pinned) {
                a.x += nx * correction * a.inv_mass;
                a.y += ny * correction * a.inv_mass;
                a.z += nz * correction * a.inv_mass;
            }
            if (!b.pinned) {
                b.x -= nx * correction * b.inv_mass;
                b.y -= ny * correction * b.inv_mass;
                b.z -= nz * correction * b.inv_mass;
            }
            const rel_vx = b.vx - a.vx;
            const rel_vy = b.vy - a.vy;
            const rel_vz = b.vz - a.vz;
            const rel_normal_velocity = rel_vx * nx + rel_vy * ny + rel_vz * nz;
            const impulse = rel_normal_velocity * spring.damping / inv_mass_sum;
            if (!a.pinned) {
                a.vx += nx * impulse * a.inv_mass;
                a.vy += ny * impulse * a.inv_mass;
                a.vz += nz * impulse * a.inv_mass;
            }
            if (!b.pinned) {
                b.vx -= nx * impulse * b.inv_mass;
                b.vy -= ny * impulse * b.inv_mass;
                b.vz -= nz * impulse * b.inv_mass;
            }
        }
    }

    fn solveFloor(self: *SoftBody, config: StepConfig) void {
        if (!config.collide_floor) return;
        for (self.nodes[0..self.node_count]) |*node| {
            if (!node.active or node.pinned) continue;
            const min_y = config.floor_y + node.radius;
            if (node.y < min_y) {
                node.y = min_y;
                if (node.vy < 0.0) node.vy = -node.vy * std.math.clamp(config.floor_restitution, 0.0, 1.0);
                node.vx *= @max(0.0, 1.0 - std.math.clamp(config.floor_friction, 0.0, 1.0));
                node.vz *= @max(0.0, 1.0 - std.math.clamp(config.floor_friction, 0.0, 1.0));
                self.stats.floor_contacts += 1;
            }
        }
    }

    fn recomputeStats(self: *SoftBody) void {
        var center_sum = [3]f32{ 0.0, 0.0, 0.0 };
        for (self.nodes[0..self.node_count]) |node| {
            if (!node.active) continue;
            self.stats.active_nodes += 1;
            center_sum[0] += node.x;
            center_sum[1] += node.y;
            center_sum[2] += node.z;
            const mass = if (node.inv_mass > 0.0) 1.0 / node.inv_mass else 0.0;
            self.stats.kinetic_energy += 0.5 * mass * (node.vx * node.vx + node.vy * node.vy + node.vz * node.vz);
        }
        if (self.stats.active_nodes > 0) {
            const inv_count = 1.0 / @as(f32, @floatFromInt(self.stats.active_nodes));
            self.stats.center_x = center_sum[0] * inv_count;
            self.stats.center_y = center_sum[1] * inv_count;
            self.stats.center_z = center_sum[2] * inv_count;
        }
        for (self.springs[0..self.spring_count]) |spring| {
            if (!spring.active) continue;
            if (spring.broken) {
                self.stats.broken_springs += 1;
                continue;
            }
            self.stats.active_springs += 1;
            const len = distance(self.nodes[spring.a], self.nodes[spring.b]);
            const strain = @abs(len - spring.rest_length) / spring.rest_length;
            self.stats.max_strain = @max(self.stats.max_strain, strain);
            self.stats.potential_energy += 0.5 * spring.stiffness * (len - spring.rest_length) * (len - spring.rest_length);
        }
    }

    fn center(self: *const SoftBody) [3]f32 {
        var sum = [3]f32{ 0.0, 0.0, 0.0 };
        var count: u8 = 0;
        for (self.nodes[0..self.node_count]) |node| {
            if (!node.active) continue;
            sum[0] += node.x;
            sum[1] += node.y;
            sum[2] += node.z;
            count += 1;
        }
        if (count == 0) return sum;
        const inv_count = 1.0 / @as(f32, @floatFromInt(count));
        return .{ sum[0] * inv_count, sum[1] * inv_count, sum[2] * inv_count };
    }
};

pub fn makeRope(origin_x: f32, origin_y: f32, origin_z: f32, segments: u8, spacing: f32, pinned_first: bool) SoftBody {
    var body = SoftBody.init(.rope);
    var idx: u8 = 0;
    while (idx < segments and idx < MAX_NODES) : (idx += 1) {
        _ = body.addNode(.{
            .pinned = pinned_first and idx == 0,
            .x = origin_x,
            .y = origin_y - @as(f32, @floatFromInt(idx)) * spacing,
            .z = origin_z,
            .inv_mass = if (pinned_first and idx == 0) 0.0 else 1.0,
            .radius = spacing * 0.2,
        });
        if (idx > 0) _ = body.addSpring(idx - 1, idx, 120.0, 0.8, 0.8);
    }
    return body;
}

pub fn makeCloth(origin_x: f32, origin_y: f32, origin_z: f32, width: u8, height: u8, spacing: f32) SoftBody {
    var body = SoftBody.init(.cloth);
    var row: u8 = 0;
    while (row < height) : (row += 1) {
        var col: u8 = 0;
        while (col < width) : (col += 1) {
            const pinned = row == 0 and (col == 0 or col + 1 == width);
            _ = body.addNode(.{
                .pinned = pinned,
                .x = origin_x + @as(f32, @floatFromInt(col)) * spacing,
                .y = origin_y,
                .z = origin_z + @as(f32, @floatFromInt(row)) * spacing,
                .inv_mass = if (pinned) 0.0 else 1.0,
                .radius = spacing * 0.12,
            });
        }
    }
    row = 0;
    while (row < height) : (row += 1) {
        var col: u8 = 0;
        while (col < width) : (col += 1) {
            const current: u8 = row * width + col;
            if (col + 1 < width) _ = body.addSpring(current, current + 1, 90.0, 0.7, 0.7);
            if (row + 1 < height) _ = body.addSpring(current, current + width, 90.0, 0.7, 0.7);
        }
    }
    return body;
}

pub fn makeSoftBall(center_x: f32, center_y: f32, center_z: f32, radius: f32, pinned: bool) SoftBody {
    var body = SoftBody.init(.ball);
    const points = [_][3]f32{
        .{ -1.0, 0.0, 0.0 }, .{ 1.0, 0.0, 0.0 },
        .{ 0.0, -1.0, 0.0 }, .{ 0.0, 1.0, 0.0 },
        .{ 0.0, 0.0, -1.0 }, .{ 0.0, 0.0, 1.0 },
    };
    for (points, 0..) |point, idx| {
        _ = body.addNode(.{
            .pinned = pinned,
            .x = center_x + point[0] * radius,
            .y = center_y + point[1] * radius,
            .z = center_z + point[2] * radius,
            .inv_mass = if (pinned) 0.0 else 1.0,
            .radius = radius * 0.2,
            .vy = if (idx == 3) 0.5 else 0.0,
        });
    }
    var a: u8 = 0;
    while (a < body.node_count) : (a += 1) {
        var b: u8 = a + 1;
        while (b < body.node_count) : (b += 1) {
            _ = body.addSpring(a, b, 75.0, 0.6, 1.0);
        }
    }
    return body;
}

fn distance(a: Node, b: Node) f32 {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const dz = b.z - a.z;
    return @sqrt(dx * dx + dy * dy + dz * dz);
}

fn finiteOr(value: f32, fallback: f32) f32 {
    return if (std.math.isFinite(value)) value else fallback;
}

fn sanitizeStats(stats: Stats) Stats {
    return .{
        .active_nodes = stats.active_nodes,
        .active_springs = stats.active_springs,
        .broken_springs = stats.broken_springs,
        .floor_contacts = stats.floor_contacts,
        .max_strain = finiteOr(stats.max_strain, 0.0),
        .center_x = finiteOr(stats.center_x, 0.0),
        .center_y = finiteOr(stats.center_y, 0.0),
        .center_z = finiteOr(stats.center_z, 0.0),
        .kinetic_energy = finiteOr(stats.kinetic_energy, 0.0),
        .potential_energy = finiteOr(stats.potential_energy, 0.0),
    };
}

fn quantize(value: f32) i32 {
    if (!std.math.isFinite(value)) return 0;
    return @as(i32, @intFromFloat(@round(value * 10000.0)));
}

test "soft body rope keeps pinned node and stretches under gravity" {
    var body = makeRope(0.0, 2.0, 0.0, 4, 0.5, true);
    const pinned_y = body.nodes[0].y;
    var stats: Stats = .{};
    var tick: u8 = 0;
    while (tick < 20) : (tick += 1) {
        stats = body.step(.{ .dt = 1.0 / 60.0, .solver_iterations = 4 });
    }
    try std.testing.expectEqual(pinned_y, body.nodes[0].y);
    try std.testing.expect(stats.center_y < pinned_y);
    try std.testing.expect(stats.active_springs > 0);
}

test "soft body floor collision prevents node penetration" {
    var body = makeSoftBall(0.0, 0.2, 0.0, 0.25, false);
    var stats: Stats = .{};
    var tick: u8 = 0;
    while (tick < 12) : (tick += 1) {
        stats = body.step(.{ .dt = 1.0 / 30.0, .floor_y = 0.0, .floor_restitution = 0.05, .solver_iterations = 4 });
    }
    try std.testing.expect(stats.floor_contacts > 0);
    for (body.nodes[0..body.node_count]) |node| {
        try std.testing.expect(node.y + 0.0001 >= node.radius);
    }
}

test "soft body snapshot restore preserves hash" {
    var body = makeCloth(0.0, 1.0, 0.0, 3, 3, 0.4);
    _ = body.step(.{ .dt = 1.0 / 60.0, .solver_iterations = 3 });
    const snap = body.snapshot();
    const before = body.hash();
    _ = body.step(.{ .dt = 1.0 / 10.0, .solver_iterations = 3 });
    body.restore(snap);
    try std.testing.expectEqual(before, body.hash());
}

test "soft body sanitizes non-finite node and spring state" {
    var body = makeRope(0.0, 1.0, 0.0, 3, 0.25, true);
    body.nodes[1].x = std.math.nan(f32);
    body.nodes[1].vy = std.math.inf(f32);
    body.nodes[1].radius = std.math.nan(f32);
    body.springs[0].rest_length = std.math.nan(f32);
    body.springs[0].stiffness = -1.0;
    const stats = body.step(.{ .dt = std.math.nan(f32), .gravity = std.math.inf(f32) });
    try std.testing.expect(stats.active_nodes > 0);
    try std.testing.expect(std.math.isFinite(body.nodes[1].x));
    try std.testing.expect(std.math.isFinite(body.nodes[1].vy));
    try std.testing.expect(body.nodes[1].radius > 0.0);
    try std.testing.expect(body.springs[0].rest_length > 0.0);
    try std.testing.expect(body.springs[0].stiffness >= 0.0);
}

test "soft body restore clamps invalid snapshot counts" {
    var body = SoftBody.init(.cloth);
    var snap = body.snapshot();
    snap.node_count = MAX_NODES + 1;
    snap.spring_count = MAX_SPRINGS + 1;
    snap.nodes[0] = .{ .active = true, .x = std.math.nan(f32), .radius = -1.0, .inv_mass = -1.0 };
    snap.springs[0] = .{ .active = true, .a = MAX_NODES, .b = MAX_NODES, .rest_length = -1.0 };
    snap.stats.center_x = std.math.nan(f32);
    snap.stats.kinetic_energy = std.math.inf(f32);

    body.restore(snap);

    try std.testing.expectEqual(@as(u8, MAX_NODES), body.node_count);
    try std.testing.expectEqual(@as(u8, MAX_SPRINGS), body.spring_count);
    try std.testing.expect(std.math.isFinite(body.nodes[0].x));
    try std.testing.expect(body.nodes[0].radius > 0.0);
    try std.testing.expect(body.springs[0].broken);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), body.stats.center_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), body.stats.kinetic_energy, 0.0001);
    _ = body.hash();
}
