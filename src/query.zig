//! Query - Unified Query Layer
//!
//! Task A: Unified Query Layer
//! Consolidated query functions for world interaction

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const address = @import("address.zig");

// ============================================================================
// Query Types
// ============================================================================

pub const BodyType = enum(u8) {
    static = 0,
    dynamic = 1,
    kinematic = 2,
    sensor = 3,
};

pub const QueryLayerMask = u32;

pub const QueryFilter = struct {
    layer_mask: QueryLayerMask = 0xFFFFFFFF,
    include_static: bool = true,
    include_dynamic: bool = true,
    include_kinematic: bool = true,
    include_sensors: bool = false,
    ignore_environment: bool = false,
    ignore_instance_idx: ?u8 = null,
    ignore_entity_id: ?u16 = null,
};

pub const QueryHit = struct {
    hit: bool = false,
    distance: f32 = 0,
    toi: f32 = 0,
    position_x: f32 = 0,
    position_y: f32 = 0,
    position_z: f32 = 0,
    normal_x: f32 = 0,
    normal_y: f32 = 1,
    normal_z: f32 = 0,
    instance_idx: i16 = -1,
    entity_id: i16 = -1,
    hit_environment: bool = false,
    hit_sensor: bool = false,
};

pub const OverlapResult = struct {
    hit: bool = false,
    count: u16 = 0,
    first_instance_idx: i16 = -1,
    environment_overlap: bool = false,
};

pub const PenetrationResult = struct {
    overlapping: bool = false,
    depth: f32 = 0,
    dir_x: f32 = 0,
    dir_y: f32 = 1,
    dir_z: f32 = 0,
    instance_idx: i16 = -1,
};

pub const QueryWorldView = struct {
    s1024: *scene1024.Scene1024,
    instances: []scene32.Instance,
    entities: []entity16.Entity16,
};

pub const QueryRay = struct {
    origin_x: f32,
    origin_y: f32,
    origin_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    max_distance: f32 = 1024.0,
};

// ============================================================================
// L0: World Access - Single Point Voxel Queries
// ============================================================================

/// Check if world voxel at global position is occupied by environment
pub fn queryEnvironmentVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool {
    const addr = address.encode(.{
        .world = 0,
        .px = @as(u5, @intCast(@divFloor(gx, 32))),
        .py = @as(u5, @intCast(@divFloor(gy, 32))),
        .pz = @as(u5, @intCast(@divFloor(gz, 32))),
        .lx = @as(u5, @intCast(@mod(gx, 32))),
        .ly = @as(u5, @intCast(@mod(gy, 32))),
        .lz = @as(u5, @intCast(@mod(gz, 32))),
    });
    return world.s1024.getVoxelAtGlobal(addr) catch false;
}

/// Check if instance voxel at local position is occupied
pub fn queryInstanceVoxel(world: *const QueryWorldView, inst_idx: u8, gx: i32, gy: i32, gz: i32) bool {
    if (inst_idx >= world.s1024.instance_count) return false;
    const inst = &world.s1024.instances[inst_idx];
    const lx = gx - inst.pos_x;
    const ly = gy - inst.pos_y;
    const lz = gz - inst.pos_z;
    if (lx < 0 or lx >= 16 or ly < 0 or ly >= 16 or lz < 0 or lz >= 16) return false;
    if (inst.entity_id >= world.entities.len) return false;
    return entity16.testVoxel(&world.entities[inst.entity_id], @intCast(lx), @intCast(ly), @intCast(lz));
}

/// Get body type for an instance
pub fn getInstanceBodyType(world: *const QueryWorldView, inst_idx: u8) BodyType {
    if (inst_idx >= world.s1024.instance_count) return .static;
    const inst = &world.s1024.instances[inst_idx];
    if (inst.entity_id >= world.entities.len) return .static;
    const entity = &world.entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0) return .static;
    if ((entity.physics.flags & 0x02) != 0) return .kinematic;
    return .dynamic;
}

/// Check if instance should be ignored for collision
fn shouldIgnoreInstance(world: *const QueryWorldView, inst_idx: u8, filter: QueryFilter) bool {
    const inst = &world.s1024.instances[inst_idx];
    if (inst.entity_id >= world.entities.len) return true;

    // Check ignore_instance_idx
    if (filter.ignore_instance_idx) |ignore_idx| {
        if (inst_idx == ignore_idx) return true;
    }

    // Check entity_id
    if (filter.ignore_entity_id) |ignore_eid| {
        if (inst.entity_id == ignore_eid) return true;
    }

    // Check body type filtering
    const body_type = getInstanceBodyType(world, inst_idx);
    switch (body_type) {
        .static => if (!filter.include_static) return true,
        .dynamic => if (!filter.include_dynamic) return true,
        .kinematic => if (!filter.include_kinematic) return true,
        .sensor => if (!filter.include_sensors) return true,
    }

    return false;
}

/// Query any voxel (environment or instance) at position with filtering
pub fn queryAnyVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32, filter: QueryFilter) QueryHit {
    var result: QueryHit = .{};

    // Check environment
    if (!filter.ignore_environment) {
        if (queryEnvironmentVoxel(world, gx, gy, gz)) {
            result.hit = true;
            result.hit_environment = true;
            result.position_x = @as(f32, @floatFromInt(gx));
            result.position_y = @as(f32, @floatFromInt(gy));
            result.position_z = @as(f32, @floatFromInt(gz));
            return result;
        }
    }

    // Check instances
    var i: u8 = 0;
    while (i < world.s1024.instance_count) : (i += 1) {
        if (shouldIgnoreInstance(world, i, filter)) continue;

        if (queryInstanceVoxel(world, i, gx, gy, gz)) {
            const inst = &world.s1024.instances[i];
            result.hit = true;
            result.instance_idx = @as(i16, @intCast(i));
            result.entity_id = @as(i16, @intCast(inst.entity_id));
            result.position_x = @as(f32, @floatFromInt(gx));
            result.position_y = @as(f32, @floatFromInt(gy));
            result.position_z = @as(f32, @floatFromInt(gz));

            const body_type = getInstanceBodyType(world, i);
            result.hit_sensor = (body_type == .sensor);
            return result;
        }
    }

    return result;
}

// ============================================================================
// L1: Overlap Queries
// ============================================================================

/// Check AABB overlap with world
pub fn overlapAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) OverlapResult {
    var result: OverlapResult = .{};

    const start_x = @floor(min_x);
    const start_y = @floor(min_y);
    const start_z = @floor(min_z);
    const end_x = @ceil(max_x);
    const end_y = @ceil(max_y);
    const end_z = @ceil(max_z);

    var x: f32 = start_x;
    while (x <= end_x) : (x += 1) {
        var y: f32 = start_y;
        while (y <= end_y) : (y += 1) {
            var z: f32 = start_z;
            while (z <= end_z) : (z += 1) {
                const hit = queryAnyVoxel(world, @intFromFloat(x), @intFromFloat(y), @intFromFloat(z), filter);
                if (hit.hit_environment) {
                    result.hit = true;
                    result.environment_overlap = true;
                    result.count += 1;
                }
                if (hit.instance_idx >= 0) {
                    result.hit = true;
                    result.count += 1;
                    if (result.first_instance_idx == -1) {
                        result.first_instance_idx = hit.instance_idx;
                    }
                }
            }
        }
    }

    return result;
}

// ============================================================================
// L2: Ray Queries (DDA Voxel Raycast)
// ============================================================================

/// DDA Voxel Raycast with query filter support
pub fn raycastSingle(world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter) QueryHit {
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

    while (t <= ray.max_distance and steps < max_steps) : (steps += 1) {
        // Check environment
        if (!filter.ignore_environment) {
            const addr = address.encode(.{
                .world = 0,
                .px = @as(u5, @intCast(@divFloor(px, 32))),
                .py = @as(u5, @intCast(@divFloor(py, 32))),
                .pz = @as(u5, @intCast(@divFloor(pz, 32))),
                .lx = @as(u5, @intCast(@mod(px, 32))),
                .ly = @as(u5, @intCast(@mod(py, 32))),
                .lz = @as(u5, @intCast(@mod(pz, 32))),
            });
            if (world.s1024.getVoxelAtGlobal(addr) catch false) {
                return .{
                    .hit = true,
                    .distance = t,
                    .toi = t,
                    .position_x = ray.origin_x + ray.dir_x * t,
                    .position_y = ray.origin_y + ray.dir_y * t,
                    .position_z = ray.origin_z + ray.dir_z * t,
                    .normal_x = normal_x,
                    .normal_y = normal_y,
                    .normal_z = normal_z,
                    .instance_idx = -1,
                    .entity_id = -1,
                    .hit_environment = true,
                    .hit_sensor = false,
                };
            }
        }

        // Check instances
        var i: u8 = 0;
        while (i < world.s1024.instance_count) : (i += 1) {
            if (shouldIgnoreInstance(world, i, filter)) continue;

            const inst = &world.s1024.instances[i];
            const lx = px - inst.pos_x;
            const ly = py - inst.pos_y;
            const lz = pz - inst.pos_z;

            if (lx >= 0 and lx < 16 and ly >= 0 and ly < 16 and lz >= 0 and lz < 16) {
                if (entity16.testVoxel(&world.entities[inst.entity_id], @intCast(lx), @intCast(ly), @intCast(lz))) {
                    const body_type = getInstanceBodyType(world, i);
                    return .{
                        .hit = true,
                        .distance = t,
                        .toi = t,
                        .position_x = ray.origin_x + ray.dir_x * t,
                        .position_y = ray.origin_y + ray.dir_y * t,
                        .position_z = ray.origin_z + ray.dir_z * t,
                        .normal_x = normal_x,
                        .normal_y = normal_y,
                        .normal_z = normal_z,
                        .instance_idx = @as(i16, @intCast(i)),
                        .entity_id = @as(i16, @intCast(inst.entity_id)),
                        .hit_environment = false,
                        .hit_sensor = (body_type == .sensor),
                    };
                }
            }
        }

        // Advance DDA
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

    return .{};
}

// ============================================================================
// L3: Sweep Queries
// ============================================================================

/// Sphere sweep - simplified using center raycast
pub fn sphereCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        const hit = queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y)), @intFromFloat(@floor(center_z)), filter);
        if (hit.hit) {
            return .{
                .hit = true,
                .distance = 0,
                .position_x = center_x,
                .position_y = center_y,
                .position_z = center_z,
                .instance_idx = hit.instance_idx,
                .entity_id = hit.entity_id,
                .hit_environment = hit.hit_environment,
                .hit_sensor = hit.hit_sensor,
            };
        }
        return .{};
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;

    // Offset ray origin by radius in opposite direction
    const offset_ray = QueryRay{
        .origin_x = center_x - ndx * radius,
        .origin_y = center_y - ndy * radius,
        .origin_z = center_z - ndz * radius,
        .dir_x = ndx,
        .dir_y = ndy,
        .dir_z = ndz,
        .max_distance = max_distance + radius,
    };

    return raycastSingle(world, offset_ray, filter);
}

/// Capsule sweep - capsule is center + radius + half_height
pub fn capsuleCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    const ray_len = @sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (ray_len < 0.0001) {
        var hit = queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y)), @intFromFloat(@floor(center_z)), filter);
        if (hit.hit) return hit;
        hit = queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y + half_height)), @intFromFloat(@floor(center_z)), filter);
        if (hit.hit) return hit;
        hit = queryAnyVoxel(world, @intFromFloat(@floor(center_x)), @intFromFloat(@floor(center_y - half_height)), @intFromFloat(@floor(center_z)), filter);
        return hit;
    }

    const ndx = dir_x / ray_len;
    const ndy = dir_y / ray_len;
    const ndz = dir_z / ray_len;

    const total_height = half_height * 2 + radius * 2;
    const sample_count: u32 = @intFromFloat(@max(1, @floor(total_height / 2)));

    var closest_hit: QueryHit = .{};

    var i: u32 = 0;
    while (i < sample_count) : (i += 1) {
        const offset_y = (@as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(sample_count)) - 0.5) * (half_height * 2 + radius);
        const sample_y = center_y + offset_y;

        const offset_ray = QueryRay{
            .origin_x = center_x - ndx * radius,
            .origin_y = sample_y - ndy * radius,
            .origin_z = center_z - ndz * radius,
            .dir_x = ndx,
            .dir_y = ndy,
            .dir_z = ndz,
            .max_distance = max_distance + radius,
        };

        const hit = raycastSingle(world, offset_ray, filter);
        if (hit.hit and (!closest_hit.hit or hit.distance < closest_hit.distance)) {
            closest_hit = hit;
        }
    }

    return closest_hit;
}
