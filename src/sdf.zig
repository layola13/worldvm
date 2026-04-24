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
