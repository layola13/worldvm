//! Raycast - Ray and Shape Casting System
//!
//! Phase 4: Raycasting for visibility, AI sensing, and physics queries
//! Implements DDA voxel raycast and sphere/box sweeps

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const address = @import("address.zig");

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

/// DDA Voxel Raycast - efficient 3D grid traversal
/// Returns first voxel hit along ray
pub fn voxelRaycast(ray: Ray, s1024: *scene1024.Scene1024, entities: []entity16.Entity16, layer_mask: u32) RayHit {
    // Convert ray to world voxel coordinates
    var px: i32 = @intFromFloat(@floor(ray.origin_x));
    var py: i32 = @intFromFloat(@floor(ray.origin_y));
    var pz: i32 = @intFromFloat(@floor(ray.origin_z));

    const step_x: i32 = if (ray.dir_x >= 0) 1 else -1;
    const step_y: i32 = if (ray.dir_y >= 0) 1 else -1;
    const step_z: i32 = if (ray.dir_z >= 0) 1 else -1;

    const tDelta_x: f32 = if (ray.dir_x != 0) @abs(1.0 / ray.dir_x) else 1e10;
    const tDelta_y: f32 = if (ray.dir_y != 0) @abs(1.0 / ray.dir_y) else 1e10;
    const tDelta_z: f32 = if (ray.dir_z != 0) @abs(1.0 / ray.dir_z) else 1e10;

    var tMax_x: f32 = if (ray.dir_x >= 0) (ray.origin_x - @as(f32, @floatFromInt(px))) * tDelta_x else (@as(f32, @floatFromInt(px)) + 1.0 - ray.origin_x) * tDelta_x;
    var tMax_y: f32 = if (ray.dir_y >= 0) (ray.origin_y - @as(f32, @floatFromInt(py))) * tDelta_y else (@as(f32, @floatFromInt(py)) + 1.0 - ray.origin_y) * tDelta_y;
    var tMax_z: f32 = if (ray.dir_z >= 0) (ray.origin_z - @as(f32, @floatFromInt(pz))) * tDelta_z else (@as(f32, @floatFromInt(pz)) + 1.0 - ray.origin_z) * tDelta_z;

    var t: f32 = 0.0;
    const hit_entity: u16 = 255;
    _ = layer_mask; // Reserved for future layer-based filtering
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    const max_steps: i32 = 2048;
    var steps: i32 = 0;

    while (t <= ray.max_t and steps < max_steps) : (steps += 1) {
        // Check voxel occupancy at current position
        const addr = address.encode(.{
            .world = 0,
            .px = @as(u5, @intCast(@divFloor(px, 32))),
            .py = @as(u5, @intCast(@divFloor(py, 32))),
            .pz = @as(u5, @intCast(@divFloor(pz, 32))),
            .lx = @as(u5, @intCast(@mod(px, 32))),
            .ly = @as(u5, @intCast(@mod(py, 32))),
            .lz = @as(u5, @intCast(@mod(pz, 32))),
        });

        const occupied = s1024.getVoxelAtGlobal(addr) catch false;
        if (occupied) {
            return .{
                .hit = true,
                .t = t,
                .normal_x = normal_x,
                .normal_y = normal_y,
                .normal_z = normal_z,
                .entity_id = hit_entity,
                .point_x = ray.origin_x + ray.dir_x * t,
                .point_y = ray.origin_y + ray.dir_y * t,
                .point_z = ray.origin_z + ray.dir_z * t,
            };
        }

        // Check instances at this position
        for (0..s1024.instance_count) |i| {
            const inst = &s1024.instances[i];
            const lx = px - inst.pos_x;
            const ly = py - inst.pos_y;
            const lz = pz - inst.pos_z;
            if (lx >= 0 and lx < 16 and ly >= 0 and ly < 16 and lz >= 0 and lz < 16) {
                if (entity16.testVoxel(&entities[inst.entity_id], @intCast(lx), @intCast(ly), @intCast(lz))) {
                    return .{
                        .hit = true,
                        .t = t,
                        .normal_x = normal_x,
                        .normal_y = normal_y,
                        .normal_z = normal_z,
                        .entity_id = inst.entity_id,
                        .point_x = ray.origin_x + ray.dir_x * t,
                        .point_y = ray.origin_y + ray.dir_y * t,
                        .point_z = ray.origin_z + ray.dir_z * t,
                    };
                }
            }
        }

        // Advance to next voxel
        if (tMax_x < tMax_y and tMax_x < tMax_z) {
            t = tMax_x;
            tMax_x += tDelta_x;
            px += step_x;
            normal_x = if (step_x > 0) -1.0 else 1.0;
            normal_y = 0.0;
            normal_z = 0.0;
        } else if (tMax_y < tMax_z) {
            t = tMax_y;
            tMax_y += tDelta_y;
            py += step_y;
            normal_x = 0.0;
            normal_y = if (step_y > 0) -1.0 else 1.0;
            normal_z = 0.0;
        } else {
            t = tMax_z;
            tMax_z += tDelta_z;
            pz += step_z;
            normal_x = 0.0;
            normal_y = 0.0;
            normal_z = if (step_z > 0) -1.0 else 1.0;
        }
    }

    return .{ .hit = false, .t = ray.max_t, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 255, .point_x = 0, .point_y = 0, .point_z = 0 };
}

/// Raycast against all instances
pub fn raycast(ray: Ray, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) RayHit {
    return voxelRaycast(ray, s1024, entities, 0xFFFFFFFF);
}

/// Sphere cast against scene
pub fn sphereCast(center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_dist: f32, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) RayHit {
    // Simplified sphere cast: use center point raycast
    const ray = Ray.init(center_x, center_y, center_z, dir_x * max_dist, dir_y * max_dist, dir_z * max_dist);
    const hit = voxelRaycast(ray, s1024, entities, 0xFFFFFFFF);

    if (!hit.hit) return hit;

    // Check if hit point is within sphere radius
    const dx = hit.point_x - center_x;
    const dy = hit.point_y - center_y;
    const dz = hit.point_z - center_z;
    const dist = @sqrt(dx * dx + dy * dy + dz * dz);

    if (dist <= radius) {
        return hit;
    }

    return .{ .hit = false, .t = max_dist, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 255, .point_x = 0, .point_y = 0, .point_z = 0 };
}

/// Box cast against scene
pub fn boxCast(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_dist: f32, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) RayHit {
    // Cast ray from box center
    const cx = (min_x + max_x) * 0.5;
    const cy = (min_y + max_y) * 0.5;
    const cz = (min_z + max_z) * 0.5;
    const ray = Ray.init(cx, cy, cz, dir_x * max_dist, dir_y * max_dist, dir_z * max_dist);
    return voxelRaycast(ray, s1024, entities, 0xFFFFFFFF);
}

/// Get all hits along a ray (穿透射线)
pub fn raycastAll(ray: Ray, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) [32]RayHit {
    var hits: [32]RayHit = undefined;
    var count: u8 = 0;

    var px: i32 = @intFromFloat(@floor(ray.origin_x));
    var py: i32 = @intFromFloat(@floor(ray.origin_y));
    var pz: i32 = @intFromFloat(@floor(ray.origin_z));

    const step_x: i32 = if (ray.dir_x >= 0) 1 else -1;
    const step_y: i32 = if (ray.dir_y >= 0) 1 else -1;
    const step_z: i32 = if (ray.dir_z >= 0) 1 else -1;

    const tDelta_x: f32 = if (ray.dir_x != 0) @abs(1.0 / ray.dir_x) else 1e10;
    const tDelta_y: f32 = if (ray.dir_y != 0) @abs(1.0 / ray.dir_y) else 1e10;
    const tDelta_z: f32 = if (ray.dir_z != 0) @abs(1.0 / ray.dir_z) else 1e10;

    var tMax_x: f32 = if (ray.dir_x >= 0) (ray.origin_x - @as(f32, @floatFromInt(px))) * tDelta_x else (@as(f32, @floatFromInt(px)) + 1.0 - ray.origin_x) * tDelta_x;
    var tMax_y: f32 = if (ray.dir_y >= 0) (ray.origin_y - @as(f32, @floatFromInt(py))) * tDelta_y else (@as(f32, @floatFromInt(py)) + 1.0 - ray.origin_y) * tDelta_y;
    var tMax_z: f32 = if (ray.dir_z >= 0) (ray.origin_z - @as(f32, @floatFromInt(pz))) * tDelta_z else (@as(f32, @floatFromInt(pz)) + 1.0 - ray.origin_z) * tDelta_z;

    var t: f32 = 0.0;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    const max_steps: i32 = 2048;
    var steps: i32 = 0;

    while (t <= ray.max_t and steps < max_steps and count < 32) : (steps += 1) {
        // Check instances at this position
        for (0..s1024.instance_count) |i| {
            const inst = &s1024.instances[i];
            const lx = px - inst.pos_x;
            const ly = py - inst.pos_y;
            const lz = pz - inst.pos_z;
            if (lx >= 0 and lx < 16 and ly >= 0 and ly < 16 and lz >= 0 and lz < 16) {
                if (entity16.testVoxel(&entities[inst.entity_id], @intCast(lx), @intCast(ly), @intCast(lz))) {
                    hits[count] = .{
                        .hit = true,
                        .t = t,
                        .normal_x = normal_x,
                        .normal_y = normal_y,
                        .normal_z = normal_z,
                        .entity_id = inst.entity_id,
                        .point_x = ray.origin_x + ray.dir_x * t,
                        .point_y = ray.origin_y + ray.dir_y * t,
                        .point_z = ray.origin_z + ray.dir_z * t,
                    };
                    count += 1;
                }
            }
        }

        // Advance to next voxel
        if (tMax_x < tMax_y and tMax_x < tMax_z) {
            t = tMax_x;
            tMax_x += tDelta_x;
            px += step_x;
            normal_x = if (step_x > 0) -1.0 else 1.0;
            normal_y = 0.0;
            normal_z = 0.0;
        } else if (tMax_y < tMax_z) {
            t = tMax_y;
            tMax_y += tDelta_y;
            py += step_y;
            normal_x = 0.0;
            normal_y = if (step_y > 0) -1.0 else 1.0;
            normal_z = 0.0;
        } else {
            t = tMax_z;
            tMax_z += tDelta_z;
            pz += step_z;
            normal_x = 0.0;
            normal_y = 0.0;
            normal_z = if (step_z > 0) -1.0 else 1.0;
        }
    }

    // Fill remaining with no-hits
    while (count < 32) : (count += 1) {
        hits[count] = .{ .hit = false, .t = ray.max_t, .normal_x = 0, .normal_y = 0, .normal_z = 0, .entity_id = 255, .point_x = 0, .point_y = 0, .point_z = 0 };
    }

    return hits;
}
