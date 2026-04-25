//! Raycast - Ray and Shape Casting System
//!
//! Phase 4: Raycasting for visibility, AI sensing, and physics queries
//! Implements DDA voxel raycast and sphere/box sweeps

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const address = @import("address.zig");
const query = @import("query.zig");

pub const Vec3 = struct {
    x: f32,
    y: f32,
    z: f32,

    pub fn init(x: f32, y: f32, z: f32) Vec3 { return .{ .x = x, .y = y, .z = z }; }
    pub fn add(a: Vec3, b: Vec3) Vec3 { return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z }; }
    pub fn sub(a: Vec3, b: Vec3) Vec3 { return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z }; }
    pub fn scale(v: Vec3, s: f32) Vec3 { return .{ .x = v.x * s, .y = v.y * s, .z = v.z * s }; }
    pub fn dot(a: Vec3, b: Vec3) f32 { return a.x * b.x + a.y * b.y + a.z * b.z; }
    pub fn length(v: Vec3) f32 { return @sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
    pub fn normalize(v: Vec3) Vec3 {
        const len = length(v);
        if (len < 0.0001) return v;
        return scale(v, 1.0 / len);
    }
};

pub const Ray = struct {
    origin_x: f32,
    origin_y: f32,
    origin_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    max_t: f32 = 1024.0,

    pub fn init(ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32) Ray {
        const len = @sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 0.0001) return .{ .origin_x = ox, .origin_y = oy, .origin_z = oz, .dir_x = 0, .dir_y = 0, .dir_z = 1, .max_t = 1024.0 };
        return .{ .origin_x = ox, .origin_y = oy, .origin_z = oz, .dir_x = dx / len, .dir_y = dy / len, .dir_z = dz / len, .max_t = 1024.0 };
    }

    pub fn at(r: Ray, t: f32) Vec3 {
        return .{ .x = r.origin_x + r.dir_x * t, .y = r.origin_y + r.dir_y * t, .z = r.origin_z + r.dir_z * t };
    }
};

pub const RayHit = struct {
    hit: bool,
    t: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    entity_id: u16,
    point_x: f32,
    point_y: f32,
    point_z: f32,
};

pub const SphereCast = struct {
    center_x: f32,
    center_y: f32,
    center_z: f32,
    radius: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
};

pub const BoxCast = struct {
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
};

const SampleOffset = struct {
    x: f32,
    y: f32,
    z: f32,
};

fn buildQueryWorldView(s1024: *scene1024.Scene1024, entities: []entity16.Entity16) query.QueryWorldView {
    return .{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
}

fn toLegacyRayHit(hit: query.QueryHit, max_distance: f32) RayHit {
    return .{
        .hit = hit.hit,
        .t = if (hit.hit) hit.distance else max_distance,
        .normal_x = hit.normal_x,
        .normal_y = hit.normal_y,
        .normal_z = hit.normal_z,
        .entity_id = if (hit.entity_id >= 0) @intCast(hit.entity_id) else 255,
        .point_x = hit.position_x,
        .point_y = hit.position_y,
        .point_z = hit.position_z,
    };
}

fn updateClosestHit(closest: *RayHit, hit: RayHit, offset: SampleOffset) void {
    if (!hit.hit) return;
    if (closest.hit and hit.t >= closest.t) return;

    var adjusted = hit;
    adjusted.point_x -= offset.x;
    adjusted.point_y -= offset.y;
    adjusted.point_z -= offset.z;
    closest.* = adjusted;
}

fn sweepSamples(origin_x: f32, origin_y: f32, origin_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_dist: f32, offsets: []const SampleOffset, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) RayHit {
    var closest: RayHit = .{ .hit = false, .t = max_dist, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 255, .point_x = 0, .point_y = 0, .point_z = 0 };

    for (offsets) |offset| {
        var ray = Ray.init(origin_x + offset.x, origin_y + offset.y, origin_z + offset.z, dir_x, dir_y, dir_z);
        ray.max_t = max_dist;
        const hit = voxelRaycast(ray, s1024, entities, 0xFFFFFFFF);
        updateClosestHit(&closest, hit, offset);
    }

    return closest;
}

/// DDA Voxel Raycast - efficient 3D grid traversal
/// Returns first voxel hit along ray
pub fn voxelRaycast(ray: Ray, s1024: *scene1024.Scene1024, entities: []entity16.Entity16, layer_mask: u32) RayHit {
    const world = buildQueryWorldView(s1024, entities);
    const hit = query.raycastSingle(&world, .{
        .origin_x = ray.origin_x,
        .origin_y = ray.origin_y,
        .origin_z = ray.origin_z,
        .dir_x = ray.dir_x,
        .dir_y = ray.dir_y,
        .dir_z = ray.dir_z,
        .max_distance = ray.max_t,
    }, .{
        .layer_mask = layer_mask,
    });
    return toLegacyRayHit(hit, ray.max_t);
}

/// Raycast against all instances
pub fn raycast(ray: Ray, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) RayHit {
    return voxelRaycast(ray, s1024, entities, 0xFFFFFFFF);
}

/// Sphere cast against scene
pub fn sphereCast(center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_dist: f32, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) RayHit {
    const world = buildQueryWorldView(s1024, entities);
    return toLegacyRayHit(query.sphereCast(&world, center_x, center_y, center_z, radius, dir_x, dir_y, dir_z, max_dist, .{}), max_dist);
}

/// Box cast against scene
pub fn boxCast(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_dist: f32, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) RayHit {
    const world = buildQueryWorldView(s1024, entities);
    return toLegacyRayHit(query.boxCast(&world, min_x, min_y, min_z, max_x, max_y, max_z, dir_x, dir_y, dir_z, max_dist, .{}), max_dist);
}

/// Get all hits along a ray (穿透射线)
pub fn raycastAll(ray: Ray, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) [32]RayHit {
    var hits: [32]RayHit = [_]RayHit{.{ .hit = false, .t = ray.max_t, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 255, .point_x = 0, .point_y = 0, .point_z = 0 }} ** 32;
    var query_hits: [32]query.QueryHit = undefined;
    const world = buildQueryWorldView(s1024, entities);
    const count = query.raycastAll(&world, .{
        .origin_x = ray.origin_x,
        .origin_y = ray.origin_y,
        .origin_z = ray.origin_z,
        .dir_x = ray.dir_x,
        .dir_y = ray.dir_y,
        .dir_z = ray.dir_z,
        .max_distance = ray.max_t,
    }, .{}, query_hits[0..]);

    var i: usize = 0;
    while (i < count and i < hits.len) : (i += 1) {
        hits[i] = toLegacyRayHit(query_hits[i], ray.max_t);
    }
    return hits;
}

test "sphereCast hits offset environment voxel within radius" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 3,
        .ly = 1,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const hit = sphereCast(0, 0, 0, 1.0, 1.0, 0.0, 0.0, 5.0, &s1024, entities[0..]);
    try std.testing.expect(hit.hit);
    try std.testing.expect(hit.t <= 3.0);
}

test "boxCast hits corner environment voxel outside center line" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 3,
        .ly = 2,
        .lz = 2,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const hit = boxCast(0, 0, 0, 2, 2, 2, 1.0, 0.0, 0.0, 5.0, &s1024, entities[0..]);
    try std.testing.expect(hit.hit);
    try std.testing.expect(hit.t <= 3.0);
}
