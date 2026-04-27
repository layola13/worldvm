//! Query Debug - renderer-agnostic query visualization data
//!
//! Produces fixed-buffer debug primitives that callers can feed into any
//! renderer, telemetry stream, or editor overlay without coupling query code
//! to a graphics backend.

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const address = @import("address.zig");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const query_raycast = @import("query_raycast.zig");
const query_overlap = @import("query_overlap.zig");

const OverlapResult = query_types.OverlapResult;
const QueryAABB = query_types.QueryAABB;
const QueryFilter = query_types.QueryFilter;
const QueryHit = query_types.QueryHit;
const QueryRay = query_types.QueryRay;
const QueryWorldView = query_types.QueryWorldView;

pub const QueryDebugPrimitiveKind = enum(u8) {
    point = 0,
    line = 1,
    aabb = 2,
    normal = 3,
};

pub const QueryDebugColor = struct {
    r: f32 = 1.0,
    g: f32 = 1.0,
    b: f32 = 1.0,
    a: f32 = 1.0,
};

pub const QueryDebugPrimitive = struct {
    kind: QueryDebugPrimitiveKind = .point,
    x0: f32 = 0,
    y0: f32 = 0,
    z0: f32 = 0,
    x1: f32 = 0,
    y1: f32 = 0,
    z1: f32 = 0,
    color: QueryDebugColor = .{},
    hit: bool = false,
    hit_environment: bool = false,
    instance_idx: i16 = -1,
    entity_id: i16 = -1,
};

pub const QueryDebugBuffer = struct {
    primitives: []QueryDebugPrimitive,
    count: u16 = 0,
    dropped: u16 = 0,

    pub fn init(primitives: []QueryDebugPrimitive) QueryDebugBuffer {
        return .{ .primitives = primitives };
    }

    pub fn clear(self: *QueryDebugBuffer) void {
        self.count = 0;
        self.dropped = 0;
    }

    pub fn push(self: *QueryDebugBuffer, primitive: QueryDebugPrimitive) bool {
        if (self.count >= self.primitives.len) {
            self.dropped +|= 1;
            return false;
        }
        self.primitives[self.count] = primitive;
        self.count += 1;
        return true;
    }

    pub fn items(self: *QueryDebugBuffer) []QueryDebugPrimitive {
        return self.primitives[0..self.count];
    }

    pub fn constItems(self: *const QueryDebugBuffer) []const QueryDebugPrimitive {
        return self.primitives[0..self.count];
    }
};

const color_miss: QueryDebugColor = .{ .r = 0.25, .g = 0.55, .b = 1.0, .a = 0.65 };
const color_hit: QueryDebugColor = .{ .r = 1.0, .g = 0.18, .b = 0.08, .a = 0.95 };
const color_overlap_hit: QueryDebugColor = .{ .r = 1.0, .g = 0.58, .b = 0.1, .a = 0.85 };
const color_overlap_miss: QueryDebugColor = .{ .r = 0.15, .g = 0.85, .b = 0.35, .a = 0.5 };
const color_normal: QueryDebugColor = .{ .r = 0.1, .g = 1.0, .b = 0.9, .a = 0.95 };

fn primitiveFromHit(kind: QueryDebugPrimitiveKind, hit: QueryHit) QueryDebugPrimitive {
    return .{
        .kind = kind,
        .x0 = hit.position_x,
        .y0 = hit.position_y,
        .z0 = hit.position_z,
        .x1 = hit.position_x,
        .y1 = hit.position_y,
        .z1 = hit.position_z,
        .color = if (hit.hit) color_hit else color_miss,
        .hit = hit.hit,
        .hit_environment = hit.hit_environment,
        .instance_idx = hit.instance_idx,
        .entity_id = hit.entity_id,
    };
}

pub fn appendPointQueryDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, gx: i32, gy: i32, gz: i32, filter: QueryFilter) QueryHit {
    const hit = query_world.queryAnyVoxel(world, gx, gy, gz, filter);
    var primitive = primitiveFromHit(.point, hit);
    primitive.x0 = @as(f32, @floatFromInt(gx)) + 0.5;
    primitive.y0 = @as(f32, @floatFromInt(gy)) + 0.5;
    primitive.z0 = @as(f32, @floatFromInt(gz)) + 0.5;
    primitive.x1 = primitive.x0;
    primitive.y1 = primitive.y0;
    primitive.z1 = primitive.z0;
    _ = buffer.push(primitive);
    return hit;
}

pub fn appendRaycastDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter) QueryHit {
    const hit = query_raycast.raycastSingle(world, ray, filter);
    const end_x = if (hit.hit) hit.position_x else ray.origin_x + ray.dir_x * ray.max_distance;
    const end_y = if (hit.hit) hit.position_y else ray.origin_y + ray.dir_y * ray.max_distance;
    const end_z = if (hit.hit) hit.position_z else ray.origin_z + ray.dir_z * ray.max_distance;

    _ = buffer.push(.{
        .kind = .line,
        .x0 = ray.origin_x,
        .y0 = ray.origin_y,
        .z0 = ray.origin_z,
        .x1 = end_x,
        .y1 = end_y,
        .z1 = end_z,
        .color = if (hit.hit) color_hit else color_miss,
        .hit = hit.hit,
        .hit_environment = hit.hit_environment,
        .instance_idx = hit.instance_idx,
        .entity_id = hit.entity_id,
    });

    if (hit.hit) {
        _ = buffer.push(primitiveFromHit(.point, hit));
        _ = buffer.push(.{
            .kind = .normal,
            .x0 = hit.position_x,
            .y0 = hit.position_y,
            .z0 = hit.position_z,
            .x1 = hit.position_x + hit.normal_x,
            .y1 = hit.position_y + hit.normal_y,
            .z1 = hit.position_z + hit.normal_z,
            .color = color_normal,
            .hit = true,
            .hit_environment = hit.hit_environment,
            .instance_idx = hit.instance_idx,
            .entity_id = hit.entity_id,
        });
    }

    return hit;
}

pub fn appendAABBDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) OverlapResult {
    return appendQueryAABBDebug(buffer, world, QueryAABB.init(min_x, min_y, min_z, max_x, max_y, max_z), filter);
}

pub fn appendQueryAABBDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, aabb: QueryAABB, filter: QueryFilter) OverlapResult {
    const box = aabb.normalized();
    const result = query_overlap.overlapQueryAABB(world, box, filter);
    _ = buffer.push(.{
        .kind = .aabb,
        .x0 = box.min_x,
        .y0 = box.min_y,
        .z0 = box.min_z,
        .x1 = box.max_x,
        .y1 = box.max_y,
        .z1 = box.max_z,
        .color = if (result.hit) color_overlap_hit else color_overlap_miss,
        .hit = result.hit,
        .hit_environment = result.environment_overlap,
        .instance_idx = result.first_instance_idx,
    });
    return result;
}

test "QueryDebugBuffer tracks capacity and dropped primitives" {
    var storage: [1]QueryDebugPrimitive = undefined;
    var buffer = QueryDebugBuffer.init(storage[0..]);

    try std.testing.expect(buffer.push(.{ .kind = .point }));
    try std.testing.expect(!buffer.push(.{ .kind = .line }));
    try std.testing.expectEqual(@as(u16, 1), buffer.count);
    try std.testing.expectEqual(@as(u16, 1), buffer.dropped);

    buffer.clear();
    try std.testing.expectEqual(@as(u16, 0), buffer.count);
    try std.testing.expectEqual(@as(u16, 0), buffer.dropped);
}

test "appendRaycastDebug emits ray hit point and normal primitives" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 0,
        .lz = 0,
    }), true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    var storage: [4]QueryDebugPrimitive = undefined;
    var buffer = QueryDebugBuffer.init(storage[0..]);
    const hit = appendRaycastDebug(&buffer, &world, .{
        .origin_x = 0.25,
        .origin_y = 0.0,
        .origin_z = 0.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 4.0,
    }, .{});

    try std.testing.expect(hit.hit);
    try std.testing.expectEqual(@as(u16, 3), buffer.count);
    try std.testing.expectEqual(QueryDebugPrimitiveKind.line, buffer.primitives[0].kind);
    try std.testing.expectEqual(QueryDebugPrimitiveKind.point, buffer.primitives[1].kind);
    try std.testing.expectEqual(QueryDebugPrimitiveKind.normal, buffer.primitives[2].kind);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), buffer.primitives[0].x1, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buffer.primitives[2].x1, 0.0001);
}

test "appendPointQueryDebug and appendAABBDebug preserve hit metadata" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 2,
        .ly = 2,
        .lz = 2,
    }), true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    var storage: [3]QueryDebugPrimitive = undefined;
    var buffer = QueryDebugBuffer.init(storage[0..]);
    const point_hit = appendPointQueryDebug(&buffer, &world, 2, 2, 2, .{});
    const overlap = appendAABBDebug(&buffer, &world, 1.5, 1.5, 1.5, 2.5, 2.5, 2.5, .{});

    try std.testing.expect(point_hit.hit);
    try std.testing.expect(point_hit.hit_environment);
    try std.testing.expect(overlap.hit);
    try std.testing.expect(overlap.environment_overlap);
    try std.testing.expectEqual(@as(u16, 2), buffer.count);
    try std.testing.expectEqual(QueryDebugPrimitiveKind.point, buffer.primitives[0].kind);
    try std.testing.expectEqual(QueryDebugPrimitiveKind.aabb, buffer.primitives[1].kind);
    try std.testing.expect(buffer.primitives[0].hit_environment);
    try std.testing.expect(buffer.primitives[1].hit_environment);
}
