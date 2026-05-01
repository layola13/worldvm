//! SDF (Signed Distance Fields) Primitives and Operations
const std = @import("std");

pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub fn init(x: f32, y: f32, z: f32) Vec3 {
        return .{ .x = x, .y = y, .z = z };
    }

    pub fn add(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
    }

    pub fn sub(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
    }

    pub fn length(v: Vec3) f32 {
        return std.math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    pub fn abs(v: Vec3) Vec3 {
        return .{ .x = @abs(v.x), .y = @abs(v.y), .z = @abs(v.z) };
    }

    pub fn max(v: Vec3, s: f32) Vec3 {
        return .{ .x = @max(v.x, s), .y = @max(v.y, s), .z = @max(v.z, s) };
    }
};

pub const SDFOp = enum {
    sphere,
    box,
    torus,
    cylinder,
    union_op,
    intersect_op,
    subtract_op,
};

pub const SDFNode = union(SDFOp) {
    sphere: struct { radius: f32 },
    box: struct { size: Vec3 },
    torus: struct { r1: f32, r2: f32 },
    cylinder: struct { radius: f32, height: f32 },
    union_op: struct { left: *const SDFNode, right: *const SDFNode },
    intersect_op: struct { left: *const SDFNode, right: *const SDFNode },
    subtract_op: struct { left: *const SDFNode, right: *const SDFNode },

    pub fn evaluate(self: SDFNode, p: Vec3) f32 {
        switch (self) {
            .sphere => |s| {
                return Vec3.length(p) - s.radius;
            },
            .box => |b| {
                const q = Vec3.sub(Vec3.abs(p), b.size);
                const outside = Vec3.length(Vec3.max(q, 0.0));
                const inside = @min(@max(q.x, @max(q.y, q.z)), 0.0);
                return outside + inside;
            },
            .torus => |t| {
                const qx = std.math.sqrt(p.x * p.x + p.z * p.z) - t.r1;
                return std.math.sqrt(qx * qx + p.y * p.y) - t.r2;
            },
            .cylinder => |c| {
                const dx = @abs(std.math.sqrt(p.x * p.x + p.z * p.z)) - c.radius;
                const dy = @abs(p.y) - c.height;
                const outside = @max(dx, 0.0) * @max(dx, 0.0) + @max(dy, 0.0) * @max(dy, 0.0);
                return @min(@max(dx, dy), 0.0) + std.math.sqrt(outside);
            },
            .union_op => |op| {
                return @min(op.left.evaluate(p), op.right.evaluate(p));
            },
            .intersect_op => |op| {
                return @max(op.left.evaluate(p), op.right.evaluate(p));
            },
            .subtract_op => |op| {
                return @max(op.left.evaluate(p), -op.right.evaluate(p));
            },
        }
    }
};

test "Vec3 helpers compute component operations and length" {
    const a = Vec3.init(3.0, -4.0, 12.0);
    const b = Vec3.init(1.0, 2.0, 3.0);

    try std.testing.expectEqual(Vec3.init(4.0, -2.0, 15.0), Vec3.add(a, b));
    try std.testing.expectEqual(Vec3.init(2.0, -6.0, 9.0), Vec3.sub(a, b));
    try std.testing.expectEqual(Vec3.init(3.0, 4.0, 12.0), Vec3.abs(a));
    try std.testing.expectEqual(Vec3.init(3.0, 0.0, 12.0), Vec3.max(a, 0.0));
    try std.testing.expectApproxEqAbs(@as(f32, 13.0), Vec3.length(a), 0.0001);
}

test "SDF primitives report signed distances" {
    const sphere = SDFNode{ .sphere = .{ .radius = 1.0 } };
    const box = SDFNode{ .box = .{ .size = Vec3.init(1.0, 1.0, 1.0) } };
    const cylinder = SDFNode{ .cylinder = .{ .radius = 1.0, .height = 2.0 } };

    try std.testing.expect(sphere.evaluate(Vec3.init(0.0, 0.0, 0.0)) < 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), sphere.evaluate(Vec3.init(1.0, 0.0, 0.0)), 0.0001);
    try std.testing.expect(sphere.evaluate(Vec3.init(2.0, 0.0, 0.0)) > 0.0);
    try std.testing.expect(box.evaluate(Vec3.init(0.0, 0.0, 0.0)) < 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), box.evaluate(Vec3.init(1.0, 0.0, 0.0)), 0.0001);
    try std.testing.expect(cylinder.evaluate(Vec3.init(2.0, 0.0, 0.0)) > 0.0);
}

test "SDF boolean operations combine child distances" {
    const sphere = SDFNode{ .sphere = .{ .radius = 1.0 } };
    const box = SDFNode{ .box = .{ .size = Vec3.init(0.5, 0.5, 0.5) } };
    const union_node = SDFNode{ .union_op = .{ .left = &sphere, .right = &box } };
    const intersect_node = SDFNode{ .intersect_op = .{ .left = &sphere, .right = &box } };
    const subtract_node = SDFNode{ .subtract_op = .{ .left = &sphere, .right = &box } };
    const pnt = Vec3.init(0.75, 0.0, 0.0);

    try std.testing.expectApproxEqAbs(@min(sphere.evaluate(pnt), box.evaluate(pnt)), union_node.evaluate(pnt), 0.0001);
    try std.testing.expectApproxEqAbs(@max(sphere.evaluate(pnt), box.evaluate(pnt)), intersect_node.evaluate(pnt), 0.0001);
    try std.testing.expectApproxEqAbs(@max(sphere.evaluate(pnt), -box.evaluate(pnt)), subtract_node.evaluate(pnt), 0.0001);
}
