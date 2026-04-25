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
const terrain = @import("terrain.zig");
const material_pairing = @import("material_pairing.zig");

const BodyType = query_types.BodyType;
const ContactClassification = query_types.ContactClassification;
const ContactTelemetry = query_types.ContactTelemetry;
const QueryFilter = query_types.QueryFilter;
const QueryHit = query_types.QueryHit;
const QueryLayerMask = query_types.QueryLayerMask;
const QueryWorldView = query_types.QueryWorldView;
const SurfaceCondition = query_types.SurfaceCondition;

fn matchesLayerMask(mask: QueryLayerMask, body_type: BodyType) bool {
    return (mask & query_types.layerMaskForBodyType(body_type)) != 0;
}

fn includesEnvironment(mask: QueryLayerMask) bool {
    return (mask & query_types.QUERY_LAYER_ENVIRONMENT) != 0;
}

fn classifySurfaceCondition(surface: terrain.SurfaceType, medium: terrain.MediumType) SurfaceCondition {
    return switch (surface) {
        .asphalt_wet => .wet,
        .gravel, .sand => .loose,
        .grass, .mud, .mud_ruts, .snow, .cloth, .carpet => .deformable,
        .ice, .rubber => .slippery,
        .water => .submerged,
        else => switch (medium) {
            .liquid => .submerged,
            .soft => .deformable,
            else => .dry,
        },
    };
}

fn buildEnvironmentClassification(gx: i32, gz: i32) ContactClassification {
    const surface_type = terrain.getSurfaceAt(gx, gz);
    const medium_type = material_pairing.getMediumType(surface_type);
    return .{
        .body_type = .static,
        .surface_type = surface_type,
        .medium_type = medium_type,
        .material_type = .solid,
        .surface_condition = classifySurfaceCondition(surface_type, medium_type),
        .hard_surface = material_pairing.isHardSurface(surface_type),
    };
}

fn buildEnvironmentTelemetry(surface_type: terrain.SurfaceType) ContactTelemetry {
    const response = material_pairing.getDefaultResponse(surface_type);
    return .{
        .friction = response.friction,
        .restitution = response.restitution,
        .damage_modifier = response.damage_modifier,
        .penetration_resistance = response.penetration_resistance,
        .buoyancy = response.buoyancy,
    };
}

fn buildInstanceClassification(world: *const QueryWorldView, inst_idx: u8) ContactClassification {
    const inst = &world.instances[inst_idx];
    const entity = &world.entities[inst.entity_id];
    const surface_type = material_pairing.getSurfaceForMaterial(entity.physics.material);
    const medium_type = material_pairing.getMediumType(surface_type);
    const body_type = getInstanceBodyType(world, inst_idx);
    return .{
        .body_type = body_type,
        .surface_type = surface_type,
        .medium_type = medium_type,
        .material_type = entity.physics.material,
        .surface_condition = classifySurfaceCondition(surface_type, medium_type),
        .hard_surface = material_pairing.isHardSurface(surface_type),
    };
}

fn buildInstanceTelemetry(world: *const QueryWorldView, inst_idx: u8, surface_type: terrain.SurfaceType) ContactTelemetry {
    const entity = &world.entities[world.instances[inst_idx].entity_id];
    const response = material_pairing.getDefaultResponse(surface_type);
    return .{
        .friction = material_pairing.combineFriction(entity.physics.friction, surface_type),
        .restitution = material_pairing.combineRestitution(entity.physics.restitution, surface_type),
        .damage_modifier = response.damage_modifier,
        .penetration_resistance = response.penetration_resistance,
        .buoyancy = response.buoyancy,
    };
}

/// Check if world voxel at global position is occupied by environment (static voxel)
pub fn queryEnvironmentVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool {
    if (gx < 0 or gx >= 1024 or gy < 0 or gy >= 1024 or gz < 0 or gz >= 1024) return true;
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

    if ((entity.physics.flags & 0x01) != 0) return .static;
    if ((entity.physics.flags & 0x08) != 0) return .sensor;
    if ((entity.physics.flags & 0x02) != 0) return .kinematic;
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
    if (!matchesLayerMask(filter.layer_mask, body_type)) return true;
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
    if (!filter.ignore_environment and includesEnvironment(filter.layer_mask) and queryEnvironmentVoxel(world, gx, gy, gz)) {
        const classification = buildEnvironmentClassification(gx, gz);
        hit.hit = true;
        hit.hit_environment = true;
        hit.position_x = @as(f32, @floatFromInt(gx));
        hit.position_y = @as(f32, @floatFromInt(gy));
        hit.position_z = @as(f32, @floatFromInt(gz));
        hit.normal_y = 1.0;
        hit.classification = classification;
        hit.telemetry = buildEnvironmentTelemetry(classification.surface_type);
        return hit;
    }

    // Check instances
    var i: u8 = 0;
    while (i < world.s1024.instance_count) : (i += 1) {
        if (shouldIgnoreInstance(world, i, filter)) continue;

        if (queryInstanceVoxel(world, i, gx, gy, gz)) {
            const classification = buildInstanceClassification(world, i);
            hit.hit = true;
            hit.instance_idx = @as(i16, @intCast(i));
            hit.entity_id = @as(i16, @intCast(world.instances[i].entity_id));
            hit.position_x = @as(f32, @floatFromInt(gx));
            hit.position_y = @as(f32, @floatFromInt(gy));
            hit.position_z = @as(f32, @floatFromInt(gz));
            hit.hit_sensor = (classification.body_type == .sensor);
            hit.classification = classification;
            hit.telemetry = buildInstanceTelemetry(world, i, classification.surface_type);
            return hit;
        }
    }

    return hit;
}

/// Simple voxel query - returns true if any solid voxel at position
pub fn queryVoxelSimple(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool {
    return queryAnyVoxel(world, gx, gy, gz, .{}).hit;
}

test "queryAnyVoxel exposes environment classification and telemetry" {
    terrain.init();
    terrain.addTerrainPatch(0, 0, 8, .water);

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 0,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = queryAnyVoxel(&world, 0, 0, 0, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(hit.hit_environment);
    try std.testing.expect(hit.classification.surface_type == .water);
    try std.testing.expect(hit.classification.medium_type == .liquid);
    try std.testing.expect(hit.classification.surface_condition == .submerged);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), hit.telemetry.buoyancy, 0.0001);
}

test "queryAnyVoxel exposes instance classification and combined telemetry" {
    terrain.init();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var entity = entity16.initEntity16();
    entity.physics.material = .elastic;
    entity.physics.friction = 200;
    entity.physics.restitution = 220;
    entity16.setVoxel(&entity, 0, 0, 0);

    var entities = [_]entity16.Entity16{entity};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 4,
        .pos_y = 2,
        .pos_z = 6,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        ._reserved = .{0} ** 2,
    };

    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = queryAnyVoxel(&world, 4, 2, 6, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(!hit.hit_environment);
    try std.testing.expect(hit.classification.material_type == .elastic);
    try std.testing.expect(hit.classification.surface_type == .rubber);
    try std.testing.expect(hit.classification.surface_condition == .slippery);
    try std.testing.expect(hit.telemetry.restitution > 0.8);
    try std.testing.expect(hit.telemetry.friction > 0.7);
}

test "queryAnyVoxel treats negative coordinates as blocking environment without panic" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const hit = queryAnyVoxel(&world, -1, 0, 0, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(hit.hit_environment);
}

test "queryAnyVoxel respects layer mask for environment and dynamic instances" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 0,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entity = entity16.initEntity16();
    entity.physics.mass = 10;
    entity.physics.material = .solid;
    entity16.setVoxel(&entity, 0, 0, 0);

    var entities = [_]entity16.Entity16{entity};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 1,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        ._reserved = .{0} ** 2,
    };

    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const env_only = queryAnyVoxel(&world, 0, 0, 0, .{
        .layer_mask = query_types.QUERY_LAYER_ENVIRONMENT,
        .include_dynamic = false,
    });
    try std.testing.expect(env_only.hit);
    try std.testing.expect(env_only.hit_environment);

    const dynamic_only = queryAnyVoxel(&world, 1, 0, 0, .{
        .layer_mask = query_types.QUERY_LAYER_DYNAMIC,
        .ignore_environment = true,
    });
    try std.testing.expect(dynamic_only.hit);
    try std.testing.expect(dynamic_only.instance_idx == 0);

    const masked_out_env = queryAnyVoxel(&world, 0, 0, 0, .{
        .layer_mask = query_types.QUERY_LAYER_DYNAMIC,
    });
    try std.testing.expect(!masked_out_env.hit);
}
