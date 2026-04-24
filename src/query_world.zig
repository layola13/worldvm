//! Query World - L0 Single-Point Occupancy Queries
//!
//! Task A: Unified Query Layer
//! Provides pure world occupancy queries without side effects

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const address = @import("address.zig");
const query_types = @import("query_types.zig");

usingnamespace query_types;

/// Check if world voxel at global position is occupied by environment (static voxel)
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

/// Check if instance voxel at global position is occupied (instance's own voxels)
pub fn queryInstanceVoxel(world: *const QueryWorldView, inst_idx: u8, gx: i32, gy: i32, gz: i32) bool {
    if (inst_idx >= world.s1024.instance_count) return false;
    const inst = &world.instances[inst_idx];
    if (inst.entity_id >= world.entities.len) return false;

    const entity = &world.entities[inst.entity_id];
    const lx = gx - inst.pos_x;
    const ly = gy - inst.pos_y;
    const lz = gz - inst.pos_z;

    if (lx < 0 or lx >= 16 or ly < 0 or ly >= 16 or lz < 0 or lz >= 16) return false;
    return entity16.testVoxel(entity, @intCast(lx), @intCast(ly), @intCast(lz));
}

/// Get body type for an instance
pub fn getInstanceBodyType(world: *const QueryWorldView, inst_idx: u8) BodyType {
    if (inst_idx >= world.s1024.instance_count) return .static;
    const inst = &world.instances[inst_idx];
    if (inst.entity_id >= world.entities.len) return .static;
    const entity = &world.entities[inst.entity_id];

    // Check static flag (0x01)
    if ((entity.physics.flags & 0x01) != 0) return .static;
    return .dynamic;
}

/// Check if instance should be ignored based on filter
pub fn shouldIgnoreInstance(world: *const QueryWorldView, inst_idx: u8, filter: QueryFilter) bool {
    if (filter.ignore_instance_idx) |ignore_idx| {
        if (inst_idx == ignore_idx) return true;
    }
    if (filter.ignore_entity_id) |ignore_eid| {
        if (world.instances[inst_idx].entity_id == ignore_eid) return true;
    }

    const body_type = getInstanceBodyType(world, inst_idx);
    switch (body_type) {
        .static => if (!filter.include_static) return true,
        .dynamic => if (!filter.include_dynamic) return true,
        .kinematic => if (!filter.include_kinematic) return true,
        .sensor => if (!filter.include_sensors) return true,
    }
    return false;
}

/// Query any occupancy at a point with filtering
pub fn queryAnyVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32, filter: QueryFilter) QueryHit {
    var hit: QueryHit = .{};

    // Check environment first
    if (!filter.ignore_environment and queryEnvironmentVoxel(world, gx, gy, gz)) {
        hit.hit = true;
        hit.hit_environment = true;
        hit.position_x = @as(f32, @floatFromInt(gx));
        hit.position_y = @as(f32, @floatFromInt(gy));
        hit.position_z = @as(f32, @floatFromInt(gz));
        // Normal points up for environment
        hit.normal_y = 1.0;
        return hit;
    }

    // Check instances
    var i: u8 = 0;
    while (i < world.s1024.instance_count) : (i += 1) {
        if (shouldIgnoreInstance(world, i, filter)) continue;

        if (queryInstanceVoxel(world, i, gx, gy, gz)) {
            hit.hit = true;
            hit.instance_idx = @as(i16, @intCast(i));
            hit.entity_id = @as(i16, @intCast(world.instances[i].entity_id));
            hit.position_x = @as(f32, @floatFromInt(gx));
            hit.position_y = @as(f32, @floatFromInt(gy));
            hit.position_z = @as(f32, @floatFromInt(gz));

            // Check if it's a sensor
            const body_type = getInstanceBodyType(world, i);
            hit.hit_sensor = (body_type == .sensor);
            return hit;
        }
    }

    return hit;
}

/// Simple voxel query - returns true if any solid voxel at position
pub fn queryVoxelSimple(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool {
    return queryAnyVoxel(world, gx, gy, gz, .{}).hit;
}
