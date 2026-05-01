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
const QueryVoxelRequest = query_types.QueryVoxelRequest;
const QueryWorldView = query_types.QueryWorldView;
const SurfaceCondition = query_types.SurfaceCondition;
const QUERY_CACHE_CAPACITY: usize = 64;
const QUERY_CACHE_EMPTY_STAMP: u64 = 0;
pub const QUERY_SCRATCH_REQUEST_CAPACITY: usize = 256;
pub const QUERY_SCRATCH_HIT_CAPACITY: usize = 256;
pub const QUERY_SCRATCH_BOOL_CAPACITY: usize = 256;

pub const QueryAnyVoxelCacheEntry = struct {
    stamp: u64 = QUERY_CACHE_EMPTY_STAMP,
    gx: i32 = 0,
    gy: i32 = 0,
    gz: i32 = 0,
    hit: QueryHit = .{},
};

pub const QueryAnyVoxelCache = struct {
    filter: QueryFilter,
    entries: [QUERY_CACHE_CAPACITY]QueryAnyVoxelCacheEntry = [_]QueryAnyVoxelCacheEntry{.{}} ** QUERY_CACHE_CAPACITY,
    generation: u64 = 1,

    pub fn init(filter: QueryFilter) QueryAnyVoxelCache {
        return .{ .filter = filter };
    }

    pub fn reset(self: *QueryAnyVoxelCache, filter: QueryFilter) void {
        self.filter = filter;
        self.generation +%= 1;
        if (self.generation == QUERY_CACHE_EMPTY_STAMP) {
            self.generation = 1;
            @memset(&self.entries, .{});
        }
    }
};

pub const QueryScratchPool = struct {
    requests: [QUERY_SCRATCH_REQUEST_CAPACITY]QueryVoxelRequest = [_]QueryVoxelRequest{.{ .gx = 0, .gy = 0, .gz = 0 }} ** QUERY_SCRATCH_REQUEST_CAPACITY,
    hits: [QUERY_SCRATCH_HIT_CAPACITY]QueryHit = [_]QueryHit{.{}} ** QUERY_SCRATCH_HIT_CAPACITY,
    bools: [QUERY_SCRATCH_BOOL_CAPACITY]bool = [_]bool{false} ** QUERY_SCRATCH_BOOL_CAPACITY,
    cache: QueryAnyVoxelCache = QueryAnyVoxelCache.init(.{}),
    request_cursor: usize = 0,
    hit_cursor: usize = 0,
    bool_cursor: usize = 0,

    pub fn init() QueryScratchPool {
        return .{};
    }

    pub fn reset(self: *QueryScratchPool, filter: QueryFilter) void {
        self.request_cursor = 0;
        self.hit_cursor = 0;
        self.bool_cursor = 0;
        self.cache.reset(filter);
    }

    pub fn allocRequests(self: *QueryScratchPool, count: usize) ?[]QueryVoxelRequest {
        if (count > self.requests.len - self.request_cursor) return null;
        const start = self.request_cursor;
        self.request_cursor += count;
        return self.requests[start..self.request_cursor];
    }

    pub fn allocHits(self: *QueryScratchPool, count: usize) ?[]QueryHit {
        if (count > self.hits.len - self.hit_cursor) return null;
        const start = self.hit_cursor;
        self.hit_cursor += count;
        return self.hits[start..self.hit_cursor];
    }

    pub fn allocBools(self: *QueryScratchPool, count: usize) ?[]bool {
        if (count > self.bools.len - self.bool_cursor) return null;
        const start = self.bool_cursor;
        self.bool_cursor += count;
        return self.bools[start..self.bool_cursor];
    }
};

fn hashPoint(gx: i32, gy: i32, gz: i32) u64 {
    var h: u64 = 0x9E3779B97F4A7C15;
    h ^= @as(u64, @bitCast(@as(i64, gx))) *% 0xBF58476D1CE4E5B9;
    h = (h << 27) | (h >> 37);
    h ^= @as(u64, @bitCast(@as(i64, gy))) *% 0x94D049BB133111EB;
    h = (h << 31) | (h >> 33);
    h ^= @as(u64, @bitCast(@as(i64, gz))) *% 0xD6E8FEB86659FD93;
    return h;
}

fn cacheSlotIndex(gx: i32, gy: i32, gz: i32) usize {
    return @as(usize, @intCast(hashPoint(gx, gy, gz) % QUERY_CACHE_CAPACITY));
}

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

fn pointInsideInstanceBroadphase(world: *const QueryWorldView, inst_idx: u8, gx: i32, gy: i32, gz: i32) bool {
    if (inst_idx >= world.instances.len) return false;
    const inst = &world.instances[inst_idx];

    // Instance-local topology is fixed 16^3, so this AABB is a cheap broadphase.
    if (gx < inst.pos_x or gx >= inst.pos_x + 16) return false;
    if (gy < inst.pos_y or gy >= inst.pos_y + 16) return false;
    if (gz < inst.pos_z or gz >= inst.pos_z + 16) return false;
    return true;
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

    const entity = &world.entities[world.instances[inst_idx].entity_id];
    const group_bit = @as(u32, 1) << @as(u5, @truncate(entity.physics.group_id));
    if ((filter.group_mask & group_bit) == 0) return true;

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
    query_types.recordPointQuery();
    var hit: QueryHit = .{};

    // Check environment first
    if (!filter.ignore_environment and includesEnvironment(filter.layer_mask) and (filter.group_mask & 1) != 0 and queryEnvironmentVoxel(world, gx, gy, gz)) {
        const classification = buildEnvironmentClassification(gx, gz);
        hit.hit = true;
        hit.hit_environment = true;
        query_types.setQueryHitVoxelCenterPosition(&hit, gx, gy, gz);
        hit.normal_y = 1.0;
        query_types.setQueryHitClassification(&hit, classification);
        hit.telemetry = buildEnvironmentTelemetry(classification.surface_type);
        return hit;
    }

    // Check instances
    var i: u8 = 0;
    const instance_count = @as(u8, @intCast(@min(world.instances.len, world.s1024.instance_count)));
    while (i < instance_count) : (i += 1) {
        if (!pointInsideInstanceBroadphase(world, i, gx, gy, gz)) continue;
        if (shouldIgnoreInstance(world, i, filter)) continue;

        if (queryInstanceVoxel(world, i, gx, gy, gz)) {
            const classification = buildInstanceClassification(world, i);
            hit.hit = true;
            hit.instance_idx = @as(i16, @intCast(i));
            hit.entity_id = @as(i16, @intCast(world.instances[i].entity_id));
            query_types.setQueryHitVoxelCenterPosition(&hit, gx, gy, gz);
            hit.hit_sensor = (classification.body_type == .sensor);
            query_types.setQueryHitClassification(&hit, classification);
            hit.telemetry = buildInstanceTelemetry(world, i, classification.surface_type);
            return hit;
        }
    }

    return hit;
}

/// Query any occupancy with a small local cache to avoid duplicate point work
/// inside a single high-level query call.
pub fn queryAnyVoxelCached(world: *const QueryWorldView, cache: *QueryAnyVoxelCache, gx: i32, gy: i32, gz: i32) QueryHit {
    const slot = cacheSlotIndex(gx, gy, gz);
    const entry = &cache.entries[slot];
    if (entry.stamp == cache.generation and entry.gx == gx and entry.gy == gy and entry.gz == gz) {
        query_types.recordCacheHit();
        return entry.hit;
    }

    query_types.recordCacheMiss();
    const hit = queryAnyVoxel(world, gx, gy, gz, cache.filter);
    entry.* = .{
        .stamp = cache.generation,
        .gx = gx,
        .gy = gy,
        .gz = gz,
        .hit = hit,
    };
    return hit;
}

/// Alias for queryAnyVoxel
pub fn queryVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32, filter: QueryFilter) QueryHit {
    return queryAnyVoxel(world, gx, gy, gz, filter);
}

/// Simple voxel query - returns true if any solid voxel at position
pub fn queryVoxelSimple(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool {
    return queryAnyVoxel(world, gx, gy, gz, .{}).hit;
}

/// Batch query for point occupancy hits.
/// Returns number of results written into `out_hits` (up to min(requests.len, out_hits.len)).
pub fn queryAnyVoxelBatch(world: *const QueryWorldView, requests: []const QueryVoxelRequest, filter: QueryFilter, out_hits: []QueryHit) u16 {
    query_types.recordBatchQuery();
    const limit = @min(requests.len, out_hits.len);
    var cache = QueryAnyVoxelCache.init(filter);
    var i: usize = 0;
    while (i < limit) : (i += 1) {
        const request = requests[i];
        out_hits[i] = queryAnyVoxelCached(world, &cache, request.gx, request.gy, request.gz);
    }
    return @as(u16, @intCast(limit));
}

/// Batch query that returns just occupancy booleans.
pub fn queryVoxelSimpleBatch(world: *const QueryWorldView, requests: []const QueryVoxelRequest, out_hits: []bool) u16 {
    query_types.recordBatchQuery();
    const limit = @min(requests.len, out_hits.len);
    var cache = QueryAnyVoxelCache.init(.{});
    var i: usize = 0;
    while (i < limit) : (i += 1) {
        const request = requests[i];
        out_hits[i] = queryAnyVoxelCached(world, &cache, request.gx, request.gy, request.gz).hit;
    }
    return @as(u16, @intCast(limit));
}

/// Incremental batch query state for cooperative/asynchronous execution.
pub const QueryAnyVoxelBatchJob = struct {
    requests: []const QueryVoxelRequest,
    out_hits: []QueryHit,
    filter: QueryFilter,
    cache: QueryAnyVoxelCache,
    limit: usize,
    cursor: usize = 0,
    written: usize = 0,

    pub fn begin(requests: []const QueryVoxelRequest, filter: QueryFilter, out_hits: []QueryHit) QueryAnyVoxelBatchJob {
        return .{
            .requests = requests,
            .out_hits = out_hits,
            .filter = filter,
            .cache = QueryAnyVoxelCache.init(filter),
            .limit = @min(requests.len, out_hits.len),
        };
    }

    pub fn step(self: *QueryAnyVoxelBatchJob, world: *const QueryWorldView, budget: usize) u16 {
        if (budget == 0 or self.cursor >= self.limit) return 0;
        query_types.recordAsyncStep();
        const end = @min(self.limit, self.cursor + budget);
        var i = self.cursor;
        while (i < end) : (i += 1) {
            const request = self.requests[i];
            self.out_hits[i] = queryAnyVoxelCached(world, &self.cache, request.gx, request.gy, request.gz);
        }
        const processed = end - self.cursor;
        self.cursor = end;
        self.written = end;
        return @as(u16, @intCast(processed));
    }

    pub fn done(self: *const QueryAnyVoxelBatchJob) bool {
        return self.cursor >= self.limit;
    }

    pub fn resultCount(self: *const QueryAnyVoxelBatchJob) u16 {
        return @as(u16, @intCast(self.written));
    }
};

/// Incremental boolean batch query state for cooperative/asynchronous execution.
pub const QueryVoxelSimpleBatchJob = struct {
    requests: []const QueryVoxelRequest,
    out_hits: []bool,
    cache: QueryAnyVoxelCache,
    limit: usize,
    cursor: usize = 0,
    written: usize = 0,

    pub fn begin(requests: []const QueryVoxelRequest, out_hits: []bool) QueryVoxelSimpleBatchJob {
        return .{
            .requests = requests,
            .out_hits = out_hits,
            .cache = QueryAnyVoxelCache.init(.{}),
            .limit = @min(requests.len, out_hits.len),
        };
    }

    pub fn step(self: *QueryVoxelSimpleBatchJob, world: *const QueryWorldView, budget: usize) u16 {
        if (budget == 0 or self.cursor >= self.limit) return 0;
        query_types.recordAsyncStep();
        const end = @min(self.limit, self.cursor + budget);
        var i = self.cursor;
        while (i < end) : (i += 1) {
            const request = self.requests[i];
            self.out_hits[i] = queryAnyVoxelCached(world, &self.cache, request.gx, request.gy, request.gz).hit;
        }
        const processed = end - self.cursor;
        self.cursor = end;
        self.written = end;
        return @as(u16, @intCast(processed));
    }

    pub fn done(self: *const QueryVoxelSimpleBatchJob) bool {
        return self.cursor >= self.limit;
    }

    pub fn resultCount(self: *const QueryVoxelSimpleBatchJob) u16 {
        return @as(u16, @intCast(self.written));
    }
};

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
    try std.testing.expect(query_types.queryHitIsConsistent(hit));
    try std.testing.expect(hit.hit_environment);
    try std.testing.expect(hit.classification.surface_type == .water);
    try std.testing.expect(hit.classification.medium_type == .liquid);
    try std.testing.expect(hit.classification.surface_condition == .submerged);
    try std.testing.expect(hit.material_type == hit.classification.material_type);
    try std.testing.expect(hit.surface_condition == hit.classification.surface_condition);
    try std.testing.expect(hit.medium_type == hit.classification.medium_type);
    try std.testing.expect(hit.body_type == hit.classification.body_type);
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
    try std.testing.expect(query_types.queryHitIsConsistent(hit));
    try std.testing.expect(!hit.hit_environment);
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), hit.position_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.5), hit.position_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 6.5), hit.position_z, 0.0001);
    try std.testing.expect(hit.classification.material_type == .elastic);
    try std.testing.expect(hit.classification.surface_type == .rubber);
    try std.testing.expect(hit.classification.surface_condition == .slippery);
    try std.testing.expect(hit.material_type == hit.classification.material_type);
    try std.testing.expect(hit.surface_condition == hit.classification.surface_condition);
    try std.testing.expect(hit.medium_type == hit.classification.medium_type);
    try std.testing.expect(hit.body_type == hit.classification.body_type);
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
    try std.testing.expectApproxEqAbs(@as(f32, -0.5), hit.position_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), hit.position_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), hit.position_z, 0.0001);
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

test "queryAnyVoxel broadphase culls far instance and still finds nearby one" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var far_entity = entity16.initEntity16();
    entity16.setVoxel(&far_entity, 0, 0, 0);
    var near_entity = entity16.initEntity16();
    entity16.setVoxel(&near_entity, 0, 0, 0);

    var entities = [_]entity16.Entity16{ far_entity, near_entity };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 200,
        .pos_y = 200,
        .pos_z = 200,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .idle,
        .sleep_tick = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 5,
        .pos_y = 7,
        .pos_z = 9,
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

    const hit = queryAnyVoxel(&world, 5, 7, 9, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(!hit.hit_environment);
    try std.testing.expectEqual(@as(i16, 1), hit.instance_idx);
}

test "queryAnyVoxel uses instance origin plus Entity16 local voxel coordinates" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var entity = entity16.initEntity16();
    entity16.setVoxel(&entity, 5, 5, 5);
    var entities = [_]entity16.Entity16{entity};

    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 160,
        .pos_y = 224,
        .pos_z = 96,
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

    const local_like_global = queryAnyVoxel(&world, 5, 5, 5, .{});
    try std.testing.expect(!local_like_global.hit);

    const hit = queryAnyVoxel(&world, 165, 229, 101, .{});
    try std.testing.expect(hit.hit);
    try std.testing.expect(!hit.hit_environment);
    try std.testing.expectEqual(@as(i16, 0), hit.instance_idx);
    try std.testing.expectEqual(@as(i16, 0), hit.entity_id);
}

test "queryAnyVoxelBatch returns per-point results in request order" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 1,
        .lz = 1,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var dynamic_entity = entity16.initEntity16();
    entity16.setVoxel(&dynamic_entity, 0, 0, 0);
    var entities = [_]entity16.Entity16{dynamic_entity};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 3,
        .pos_y = 3,
        .pos_z = 3,
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

    const requests = [_]QueryVoxelRequest{
        .{ .gx = 1, .gy = 1, .gz = 1 }, // environment
        .{ .gx = 3, .gy = 3, .gz = 3 }, // dynamic instance
        .{ .gx = 10, .gy = 10, .gz = 10 }, // empty
    };
    var hits: [3]QueryHit = undefined;
    const written = queryAnyVoxelBatch(&world, requests[0..], .{}, hits[0..]);
    try std.testing.expectEqual(@as(u16, 3), written);

    try std.testing.expect(hits[0].hit);
    try std.testing.expect(hits[0].hit_environment);

    try std.testing.expect(hits[1].hit);
    try std.testing.expect(!hits[1].hit_environment);
    try std.testing.expectEqual(@as(i16, 0), hits[1].instance_idx);

    try std.testing.expect(!hits[2].hit);
}

test "queryAnyVoxelBatch respects output buffer length clamp" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const requests = [_]QueryVoxelRequest{
        .{ .gx = 0, .gy = 0, .gz = 0 },
        .{ .gx = 1, .gy = 1, .gz = 1 },
        .{ .gx = 2, .gy = 2, .gz = 2 },
    };
    var hits: [2]QueryHit = undefined;
    const written = queryAnyVoxelBatch(&world, requests[0..], .{}, hits[0..]);
    try std.testing.expectEqual(@as(u16, 2), written);
}

test "queryAnyVoxelBatch async job advances incrementally with budget" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 2,
        .ly = 2,
        .lz = 2,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const requests = [_]QueryVoxelRequest{
        .{ .gx = 2, .gy = 2, .gz = 2 },
        .{ .gx = 10, .gy = 10, .gz = 10 },
        .{ .gx = -1, .gy = 0, .gz = 0 },
    };
    var hits: [3]QueryHit = undefined;
    var job = QueryAnyVoxelBatchJob.begin(requests[0..], .{}, hits[0..]);

    try std.testing.expect(!job.done());
    try std.testing.expectEqual(@as(u16, 0), job.step(&world, 0));
    try std.testing.expectEqual(@as(u16, 1), job.step(&world, 1));
    try std.testing.expect(!job.done());
    try std.testing.expect(hits[0].hit and hits[0].hit_environment);

    try std.testing.expectEqual(@as(u16, 2), job.step(&world, 8));
    try std.testing.expect(job.done());
    try std.testing.expectEqual(@as(u16, 3), job.resultCount());
    try std.testing.expect(!hits[1].hit);
    try std.testing.expect(hits[2].hit and hits[2].hit_environment);
}

test "queryVoxelSimpleBatch async job respects output clamp" {
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

    const requests = [_]QueryVoxelRequest{
        .{ .gx = 0, .gy = 0, .gz = 0 },
        .{ .gx = 1, .gy = 1, .gz = 1 },
        .{ .gx = 2, .gy = 2, .gz = 2 },
    };
    var results: [2]bool = undefined;
    var job = QueryVoxelSimpleBatchJob.begin(requests[0..], results[0..]);
    try std.testing.expectEqual(@as(u16, 2), job.step(&world, 8));
    try std.testing.expect(job.done());
    try std.testing.expectEqual(@as(u16, 2), job.resultCount());
    try std.testing.expect(results[0]);
}

test "queryAnyVoxelCached returns same hit result as direct query" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 1,
        .lz = 1,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    query_types.resetQueryStats();
    var cache = QueryAnyVoxelCache.init(.{});
    const direct = queryAnyVoxel(&world, 1, 1, 1, .{});
    const cached_a = queryAnyVoxelCached(&world, &cache, 1, 1, 1);
    const cached_b = queryAnyVoxelCached(&world, &cache, 1, 1, 1);

    try std.testing.expect(direct.hit);
    try std.testing.expect(cached_a.hit);
    try std.testing.expect(cached_b.hit);
    try std.testing.expectEqual(direct.hit_environment, cached_a.hit_environment);
    try std.testing.expectEqual(cached_a.hit_environment, cached_b.hit_environment);

    const stats = query_types.getQueryStats();
    try std.testing.expectEqual(@as(u64, 2), stats.point_queries);
    try std.testing.expectEqual(@as(u64, 1), stats.cache_hits);
    try std.testing.expectEqual(@as(u64, 1), stats.cache_misses);
}

test "QueryScratchPool allocates fixed scratch slices and resets without heap allocation" {
    var pool = QueryScratchPool.init();

    const requests = pool.allocRequests(3) orelse return error.TestUnexpectedResult;
    const hits = pool.allocHits(3) orelse return error.TestUnexpectedResult;
    const bools = pool.allocBools(3) orelse return error.TestUnexpectedResult;
    requests[0] = .{ .gx = 1, .gy = 2, .gz = 3 };
    hits[0] = .{ .hit = true };
    bools[0] = true;

    try std.testing.expect(pool.allocRequests(QUERY_SCRATCH_REQUEST_CAPACITY) == null);
    pool.reset(.{ .ignore_environment = true });

    const all_requests = pool.allocRequests(QUERY_SCRATCH_REQUEST_CAPACITY) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(@as(usize, QUERY_SCRATCH_REQUEST_CAPACITY), all_requests.len);
    try std.testing.expect(pool.allocRequests(1) == null);
}

test "QueryScratchPool scratch output can back batch query" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 4,
        .ly = 4,
        .lz = 4,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    var pool = QueryScratchPool.init();
    const requests = pool.allocRequests(2) orelse return error.TestUnexpectedResult;
    const hits = pool.allocHits(2) orelse return error.TestUnexpectedResult;
    requests[0] = .{ .gx = 4, .gy = 4, .gz = 4 };
    requests[1] = .{ .gx = 8, .gy = 8, .gz = 8 };

    query_types.resetQueryStats();
    const written = queryAnyVoxelBatch(&world, requests, .{}, hits);
    try std.testing.expectEqual(@as(u16, 2), written);
    try std.testing.expect(hits[0].hit);
    try std.testing.expect(!hits[1].hit);

    const stats = query_types.getQueryStats();
    try std.testing.expectEqual(@as(u64, 1), stats.batch_queries);
    try std.testing.expectEqual(@as(u64, 2), stats.cache_misses);
    try std.testing.expectEqual(@as(u64, 2), stats.point_queries);
}

test "query stats count async batch steps" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const requests = [_]QueryVoxelRequest{
        .{ .gx = 1, .gy = 1, .gz = 1 },
        .{ .gx = 2, .gy = 2, .gz = 2 },
    };
    var hits: [2]QueryHit = undefined;
    var job = QueryAnyVoxelBatchJob.begin(requests[0..], .{}, hits[0..]);

    query_types.resetQueryStats();
    try std.testing.expectEqual(@as(u16, 1), job.step(&world, 1));
    try std.testing.expectEqual(@as(u16, 1), job.step(&world, 1));

    const stats = query_types.getQueryStats();
    try std.testing.expectEqual(@as(u64, 2), stats.async_steps);
    try std.testing.expectEqual(@as(u64, 2), stats.point_queries);
}
