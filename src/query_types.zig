//! Query Types - Unified Query Layer Core Types
//!
//! Task A: Unified Query Layer
//! Provides consistent data structures for all physics queries

const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const terrain = @import("terrain.zig");

pub const QUERY_CONTRACT_VERSION: u32 = 2;
pub const QUERY_CONTRACT_DISTANCE_WORLD_UNITS: u32 = 1 << 0;
pub const QUERY_CONTRACT_TOI_RAY_PARAMETER: u32 = 1 << 1;
pub const QUERY_CONTRACT_POSITION_WORLD_SPACE: u32 = 1 << 2;
pub const QUERY_CONTRACT_POINT_QUERY_VOXEL_CENTER: u32 = 1 << 3;
pub const QUERY_CONTRACT_NORMAL_UNIT_OR_UP: u32 = 1 << 4;
pub const QUERY_CONTRACT_CLASSIFICATION_MIRRORED: u32 = 1 << 5;
pub const QUERY_CONTRACT_STABLE_DISTANCE_SORT: u32 = 1 << 6;
pub const QUERY_CONTRACT_FLAGS: u32 =
    QUERY_CONTRACT_DISTANCE_WORLD_UNITS |
    QUERY_CONTRACT_TOI_RAY_PARAMETER |
    QUERY_CONTRACT_POSITION_WORLD_SPACE |
    QUERY_CONTRACT_POINT_QUERY_VOXEL_CENTER |
    QUERY_CONTRACT_NORMAL_UNIT_OR_UP |
    QUERY_CONTRACT_CLASSIFICATION_MIRRORED |
    QUERY_CONTRACT_STABLE_DISTANCE_SORT;

pub const QueryContract = struct {
    version: u32,
    flags: u32,
};

pub fn getQueryContract() QueryContract {
    return .{
        .version = QUERY_CONTRACT_VERSION,
        .flags = QUERY_CONTRACT_FLAGS,
    };
}

pub const QueryStats = struct {
    point_queries: u64 = 0,
    cache_hits: u64 = 0,
    cache_misses: u64 = 0,
    batch_queries: u64 = 0,
    async_steps: u64 = 0,
    raycast_all_queries: u64 = 0,
    sort_calls: u64 = 0,
    sorted_hits: u64 = 0,
};

var query_stats: QueryStats = .{};

pub fn resetQueryStats() void {
    query_stats = .{};
}

pub fn getQueryStats() QueryStats {
    return query_stats;
}

pub fn recordPointQuery() void {
    query_stats.point_queries += 1;
}

pub fn recordCacheHit() void {
    query_stats.cache_hits += 1;
}

pub fn recordCacheMiss() void {
    query_stats.cache_misses += 1;
}

pub fn recordBatchQuery() void {
    query_stats.batch_queries += 1;
}

pub fn recordAsyncStep() void {
    query_stats.async_steps += 1;
}

pub fn recordRaycastAllQuery() void {
    query_stats.raycast_all_queries += 1;
}

pub fn recordSortCall(hit_count: usize) void {
    query_stats.sort_calls += 1;
    query_stats.sorted_hits += hit_count;
}

/// Body type for collision filtering
pub const BodyType = enum(u8) {
    static = 0,
    dynamic = 1,
    kinematic = 2,
    sensor = 3,
};

/// Query layer mask - 32 bits for collision layers
pub const QueryLayerMask = u32;
pub const QUERY_LAYER_ENVIRONMENT: QueryLayerMask = 1 << 0;
pub const QUERY_LAYER_STATIC: QueryLayerMask = 1 << 1;
pub const QUERY_LAYER_DYNAMIC: QueryLayerMask = 1 << 2;
pub const QUERY_LAYER_KINEMATIC: QueryLayerMask = 1 << 3;
pub const QUERY_LAYER_SENSOR: QueryLayerMask = 1 << 4;
pub const QUERY_LAYER_ALL: QueryLayerMask =
    QUERY_LAYER_ENVIRONMENT |
    QUERY_LAYER_STATIC |
    QUERY_LAYER_DYNAMIC |
    QUERY_LAYER_KINEMATIC |
    QUERY_LAYER_SENSOR;

pub fn layerMaskForBodyType(body_type: BodyType) QueryLayerMask {
    return switch (body_type) {
        .static => QUERY_LAYER_STATIC,
        .dynamic => QUERY_LAYER_DYNAMIC,
        .kinematic => QUERY_LAYER_KINEMATIC,
        .sensor => QUERY_LAYER_SENSOR,
    };
}

/// Query filter - controls what objects are hit by queries
pub const QueryFilter = struct {
    layer_mask: QueryLayerMask = QUERY_LAYER_ALL,
    // Legacy collision group mask expected by query_world and older callers.
    group_mask: u32 = 0xFFFF_FFFF,
    include_static: bool = true,
    include_dynamic: bool = true,
    include_kinematic: bool = true,
    include_sensors: bool = false,
    ignore_environment: bool = false,
    ignore_instance_idx: ?u8 = null,
    ignore_entity_id: ?u16 = null,
};

pub const SurfaceCondition = enum(u8) {
    unknown = 0,
    dry = 1,
    wet = 2,
    loose = 3,
    deformable = 4,
    slippery = 5,
    submerged = 6,
};

pub const ContactClassification = struct {
    body_type: BodyType = .static,
    surface_type: terrain.SurfaceType = .asphalt_dry,
    medium_type: terrain.MediumType = .solid,
    material_type: entity16.MaterialType = .solid,
    surface_condition: SurfaceCondition = .dry,
    hard_surface: bool = true,
};

pub const ContactTelemetry = struct {
    friction: f32 = 0,
    restitution: f32 = 0,
    damage_modifier: f32 = 0,
    penetration_resistance: f32 = 0,
    buoyancy: f32 = 0,
};

pub const ContactPoint = struct {
    x: f32 = 0,
    y: f32 = 0,
    z: f32 = 0,
};

/// Standard hit result for all raycast and sweep queries
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
    // Legacy flattened classification fields; mirrored from `classification`.
    body_type: BodyType = .static,
    medium_type: terrain.MediumType = .solid,
    material_type: entity16.MaterialType = .solid,
    surface_condition: SurfaceCondition = .dry,
    classification: ContactClassification = .{},
    telemetry: ContactTelemetry = .{},
};

pub fn queryRayDirectionLength(ray: QueryRay) f32 {
    return @sqrt(ray.dir_x * ray.dir_x + ray.dir_y * ray.dir_y + ray.dir_z * ray.dir_z);
}

pub fn queryRayParameterToDistance(ray: QueryRay, t: f32) f32 {
    return t * queryRayDirectionLength(ray);
}

pub fn queryRayDistanceToParameter(ray: QueryRay, distance: f32) f32 {
    const len = queryRayDirectionLength(ray);
    if (len <= 0.000001) return 0.0;
    return distance / len;
}

pub fn setQueryHitDistance(hit: *QueryHit, distance: f32) void {
    hit.distance = @max(distance, 0.0);
}

pub fn setQueryHitPosition(hit: *QueryHit, x: f32, y: f32, z: f32) void {
    hit.position_x = x;
    hit.position_y = y;
    hit.position_z = z;
}

pub fn setQueryHitVoxelCenterPosition(hit: *QueryHit, gx: i32, gy: i32, gz: i32) void {
    setQueryHitPosition(
        hit,
        @as(f32, @floatFromInt(gx)) + 0.5,
        @as(f32, @floatFromInt(gy)) + 0.5,
        @as(f32, @floatFromInt(gz)) + 0.5,
    );
}

pub fn setQueryHitClassification(hit: *QueryHit, classification: ContactClassification) void {
    hit.classification = classification;
    hit.body_type = classification.body_type;
    hit.medium_type = classification.medium_type;
    hit.material_type = classification.material_type;
    hit.surface_condition = classification.surface_condition;
}

pub fn setQueryHitNormal(hit: *QueryHit, normal_x: f32, normal_y: f32, normal_z: f32) void {
    const len_sq = normal_x * normal_x + normal_y * normal_y + normal_z * normal_z;
    if (len_sq <= 0.000001) {
        hit.normal_x = 0.0;
        hit.normal_y = 1.0;
        hit.normal_z = 0.0;
        return;
    }

    const inv_len = 1.0 / @sqrt(len_sq);
    hit.normal_x = normal_x * inv_len;
    hit.normal_y = normal_y * inv_len;
    hit.normal_z = normal_z * inv_len;
}

pub fn normalizeQueryHitNormal(hit: *QueryHit) void {
    setQueryHitNormal(hit, hit.normal_x, hit.normal_y, hit.normal_z);
}

fn queryFiniteF32(value: f32) bool {
    const std = @import("std");
    return value == value and value != std.math.inf(f32) and value != -std.math.inf(f32);
}

pub fn queryHitIsConsistent(hit: QueryHit) bool {
    if (!queryFiniteF32(hit.distance) or hit.distance < 0.0) return false;
    if (!queryFiniteF32(hit.toi) or hit.toi < 0.0) return false;
    if (!queryFiniteF32(hit.position_x) or !queryFiniteF32(hit.position_y) or !queryFiniteF32(hit.position_z)) return false;
    if (!queryFiniteF32(hit.normal_x) or !queryFiniteF32(hit.normal_y) or !queryFiniteF32(hit.normal_z)) return false;
    if (hit.material_type != hit.classification.material_type) return false;
    if (hit.surface_condition != hit.classification.surface_condition) return false;
    if (hit.medium_type != hit.classification.medium_type) return false;
    if (hit.body_type != hit.classification.body_type) return false;
    if (!hit.hit) return true;

    const normal_len_sq = hit.normal_x * hit.normal_x + hit.normal_y * hit.normal_y + hit.normal_z * hit.normal_z;
    return normal_len_sq >= 0.999 and normal_len_sq <= 1.001;
}

pub fn sortQueryHitsByDistanceStable(hits: []QueryHit) void {
    recordSortCall(hits.len);
    var i: usize = 1;
    while (i < hits.len) : (i += 1) {
        const value = hits[i];
        var j = i;
        while (j > 0 and queryHitDistanceLess(value, hits[j - 1])) : (j -= 1) {
            hits[j] = hits[j - 1];
        }
        hits[j] = value;
    }
}

fn queryHitDistanceLess(a: QueryHit, b: QueryHit) bool {
    if (a.hit != b.hit) return a.hit;
    if (!a.hit) return false;
    if (a.distance < b.distance) return true;
    if (a.distance > b.distance) return false;
    if (a.toi < b.toi) return true;
    return false;
}

/// Overlap query result
pub const OverlapResult = struct {
    hit: bool = false,
    count: u16 = 0,
    first_instance_idx: i16 = -1,
    environment_overlap: bool = false,
    first_hit: QueryHit = .{},
};

/// Penetration query result - for depenetration and collision resolution
pub const PenetrationResult = struct {
    overlapping: bool = false,
    depth: f32 = 0,
    dir_x: f32 = 0,
    dir_y: f32 = 1,
    dir_z: f32 = 0,
    instance_idx: i16 = -1,
    entity_id: i16 = -1,
    hit_environment: bool = false,
    classification: ContactClassification = .{},
    telemetry: ContactTelemetry = .{},
    manifold_point_count: u8 = 0,
    manifold_points: [4]ContactPoint = [_]ContactPoint{.{}} ** 4,
};

/// World view for query operations - bundles all world data
pub const QueryWorldView = struct {
    s1024: *scene1024.Scene1024,
    instances: []scene32.Instance,
    entities: []entity16.Entity16,
};

/// Ray for raycast queries
pub const QueryRay = struct {
    origin_x: f32,
    origin_y: f32,
    origin_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    max_distance: f32 = 1024.0,
};

/// Point query request used by batch voxel queries.
pub const QueryVoxelRequest = struct {
    gx: i32,
    gy: i32,
    gz: i32,
};

/// Float AABB for query operations.
/// Bounds are canonicalized to min <= max and use half-open overlap semantics:
/// min is inclusive, max is exclusive.
pub const QueryAABB = struct {
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,

    pub fn init(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32) QueryAABB {
        return .{
            .min_x = @min(min_x, max_x),
            .min_y = @min(min_y, max_y),
            .min_z = @min(min_z, max_z),
            .max_x = @max(min_x, max_x),
            .max_y = @max(min_y, max_y),
            .max_z = @max(min_z, max_z),
        };
    }

    pub fn fromCenterHalfExtents(center_x: f32, center_y: f32, center_z: f32, half_x: f32, half_y: f32, half_z: f32) QueryAABB {
        const hx = @max(half_x, 0.0);
        const hy = @max(half_y, 0.0);
        const hz = @max(half_z, 0.0);
        return .{
            .min_x = center_x - hx,
            .min_y = center_y - hy,
            .min_z = center_z - hz,
            .max_x = center_x + hx,
            .max_y = center_y + hy,
            .max_z = center_z + hz,
        };
    }

    pub fn normalized(self: QueryAABB) QueryAABB {
        return init(self.min_x, self.min_y, self.min_z, self.max_x, self.max_y, self.max_z);
    }

    pub fn isEmpty(self: QueryAABB) bool {
        return self.max_x <= self.min_x or self.max_y <= self.min_y or self.max_z <= self.min_z;
    }

    pub fn overlaps(self: QueryAABB, other: QueryAABB) bool {
        const a = self.normalized();
        const b = other.normalized();
        if (a.isEmpty() or b.isEmpty()) return false;
        return a.min_x < b.max_x and a.max_x > b.min_x and
            a.min_y < b.max_y and a.max_y > b.min_y and
            a.min_z < b.max_z and a.max_z > b.min_z;
    }

    pub fn containsPoint(self: QueryAABB, x: f32, y: f32, z: f32) bool {
        const a = self.normalized();
        if (a.isEmpty()) return false;
        return x >= a.min_x and x < a.max_x and
            y >= a.min_y and y < a.max_y and
            z >= a.min_z and z < a.max_z;
    }

    pub fn raycast(self: QueryAABB, ray: QueryRay) QueryAABBRaycastResult {
        return raycastQueryAABB(self, ray);
    }

    pub fn sweep(self: QueryAABB, target: QueryAABB, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32) QueryAABBSweepResult {
        return sweepQueryAABB(self, target, dir_x, dir_y, dir_z, max_distance);
    }
};

pub const QueryAABBRaycastResult = struct {
    hit: bool = false,
    distance: f32 = 0,
    normal_x: f32 = 0,
    normal_y: f32 = 0,
    normal_z: f32 = 0,
};

pub const QueryAABBSweepResult = struct {
    hit: bool = false,
    toi: f32 = 0,
    distance: f32 = 0,
    normal_x: f32 = 0,
    normal_y: f32 = 0,
    normal_z: f32 = 0,
};

pub fn makeQueryAABB(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32) QueryAABB {
    return QueryAABB.init(min_x, min_y, min_z, max_x, max_y, max_z);
}

pub fn queryAABBOverlaps(a: QueryAABB, b: QueryAABB) bool {
    return a.overlaps(b);
}

pub fn queryAABBContainsPoint(aabb: QueryAABB, x: f32, y: f32, z: f32) bool {
    return aabb.containsPoint(x, y, z);
}

pub fn raycastQueryAABB(aabb: QueryAABB, ray: QueryRay) QueryAABBRaycastResult {
    const box = aabb.normalized();
    if (box.isEmpty() or ray.max_distance < 0.0) return .{};

    var t_min: f32 = 0.0;
    var t_max: f32 = ray.max_distance;
    var normal_x: f32 = 0.0;
    var normal_y: f32 = 0.0;
    var normal_z: f32 = 0.0;

    if (!clipRayAABBAxis(ray.origin_x, ray.dir_x, box.min_x, box.max_x, &t_min, &t_max, &normal_x, &normal_y, &normal_z, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0)) return .{};
    if (!clipRayAABBAxis(ray.origin_y, ray.dir_y, box.min_y, box.max_y, &t_min, &t_max, &normal_x, &normal_y, &normal_z, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0)) return .{};
    if (!clipRayAABBAxis(ray.origin_z, ray.dir_z, box.min_z, box.max_z, &t_min, &t_max, &normal_x, &normal_y, &normal_z, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0)) return .{};

    return .{
        .hit = true,
        .distance = t_min,
        .normal_x = normal_x,
        .normal_y = normal_y,
        .normal_z = normal_z,
    };
}

pub fn sweepQueryAABB(moving: QueryAABB, target: QueryAABB, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32) QueryAABBSweepResult {
    const a = moving.normalized();
    const b = target.normalized();
    if (a.isEmpty() or b.isEmpty() or max_distance < 0.0) return .{};

    if (a.overlaps(b)) {
        return .{ .hit = true, .toi = 0.0, .distance = 0.0 };
    }

    const half_x = (a.max_x - a.min_x) * 0.5;
    const half_y = (a.max_y - a.min_y) * 0.5;
    const half_z = (a.max_z - a.min_z) * 0.5;
    const center_x = (a.min_x + a.max_x) * 0.5;
    const center_y = (a.min_y + a.max_y) * 0.5;
    const center_z = (a.min_z + a.max_z) * 0.5;

    const expanded = QueryAABB.init(
        b.min_x - half_x,
        b.min_y - half_y,
        b.min_z - half_z,
        b.max_x + half_x,
        b.max_y + half_y,
        b.max_z + half_z,
    );
    const hit = raycastQueryAABB(expanded, .{
        .origin_x = center_x,
        .origin_y = center_y,
        .origin_z = center_z,
        .dir_x = dir_x,
        .dir_y = dir_y,
        .dir_z = dir_z,
        .max_distance = max_distance,
    });
    if (!hit.hit) return .{};

    return .{
        .hit = true,
        .toi = if (max_distance > 0.0) hit.distance / max_distance else 0.0,
        .distance = hit.distance,
        .normal_x = hit.normal_x,
        .normal_y = hit.normal_y,
        .normal_z = hit.normal_z,
    };
}

fn clipRayAABBAxis(
    origin: f32,
    dir: f32,
    min_value: f32,
    max_value: f32,
    t_min: *f32,
    t_max: *f32,
    normal_x: *f32,
    normal_y: *f32,
    normal_z: *f32,
    min_normal_x: f32,
    min_normal_y: f32,
    min_normal_z: f32,
    max_normal_x: f32,
    max_normal_y: f32,
    max_normal_z: f32,
) bool {
    if (dir == 0.0) {
        return origin >= min_value and origin < max_value;
    }

    const inv_dir = 1.0 / dir;
    var near_t = (min_value - origin) * inv_dir;
    var far_t = (max_value - origin) * inv_dir;
    var near_normal_x = min_normal_x;
    var near_normal_y = min_normal_y;
    var near_normal_z = min_normal_z;

    if (near_t > far_t) {
        const old_near_t = near_t;
        near_t = far_t;
        far_t = old_near_t;
        near_normal_x = max_normal_x;
        near_normal_y = max_normal_y;
        near_normal_z = max_normal_z;
    }

    if (near_t > t_min.*) {
        t_min.* = near_t;
        normal_x.* = near_normal_x;
        normal_y.* = near_normal_y;
        normal_z.* = near_normal_z;
    }
    t_max.* = @min(t_max.*, far_t);
    return t_min.* <= t_max.*;
}

// Backward-compatible integer voxel AABB for legacy physics/collision modules.
// It follows the same half-open min-inclusive, max-exclusive protocol.
pub const VoxelBox = struct {
    min_x: i32,
    min_y: i32,
    min_z: i32,
    max_x: i32,
    max_y: i32,
    max_z: i32,

    pub fn init(min_x: i32, min_y: i32, min_z: i32, max_x: i32, max_y: i32, max_z: i32) VoxelBox {
        return .{
            .min_x = @min(min_x, max_x),
            .min_y = @min(min_y, max_y),
            .min_z = @min(min_z, max_z),
            .max_x = @max(min_x, max_x),
            .max_y = @max(min_y, max_y),
            .max_z = @max(min_z, max_z),
        };
    }

    pub fn normalized(self: VoxelBox) VoxelBox {
        return init(self.min_x, self.min_y, self.min_z, self.max_x, self.max_y, self.max_z);
    }

    pub fn isEmpty(self: VoxelBox) bool {
        return self.max_x <= self.min_x or self.max_y <= self.min_y or self.max_z <= self.min_z;
    }

    pub fn overlaps(self: VoxelBox, other: VoxelBox) bool {
        const a = self.normalized();
        const b = other.normalized();
        if (a.isEmpty() or b.isEmpty()) return false;
        return a.min_x < b.max_x and a.max_x > b.min_x and
            a.min_y < b.max_y and a.max_y > b.min_y and
            a.min_z < b.max_z and a.max_z > b.min_z;
    }

    pub fn toQueryAABB(self: VoxelBox) QueryAABB {
        const a = self.normalized();
        return .{
            .min_x = @floatFromInt(a.min_x),
            .min_y = @floatFromInt(a.min_y),
            .min_z = @floatFromInt(a.min_z),
            .max_x = @floatFromInt(a.max_x),
            .max_y = @floatFromInt(a.max_y),
            .max_z = @floatFromInt(a.max_z),
        };
    }
};
pub const AABB = VoxelBox;

pub fn makeVoxelBox(min_x: i32, min_y: i32, min_z: i32, max_x: i32, max_y: i32, max_z: i32) VoxelBox {
    return VoxelBox.init(min_x, min_y, min_z, max_x, max_y, max_z);
}

pub fn voxelBoxOverlaps(a: VoxelBox, b: VoxelBox) bool {
    return a.overlaps(b);
}

/// Sphere for sphere queries
pub const QuerySphere = struct {
    center_x: f32,
    center_y: f32,
    center_z: f32,
    radius: f32,
};

/// Capsule for character queries
pub const QueryCapsule = struct {
    center_x: f32,
    center_y: f32,
    center_z: f32,
    radius: f32,
    half_height: f32,
};

test "query contract version is explicit" {
    const std = @import("std");
    const contract = getQueryContract();

    try std.testing.expectEqual(@as(u32, 2), contract.version);
    try std.testing.expectEqual(QUERY_CONTRACT_FLAGS, contract.flags);
    try std.testing.expect((contract.flags & QUERY_CONTRACT_DISTANCE_WORLD_UNITS) != 0);
    try std.testing.expect((contract.flags & QUERY_CONTRACT_TOI_RAY_PARAMETER) != 0);
    try std.testing.expect((contract.flags & QUERY_CONTRACT_POSITION_WORLD_SPACE) != 0);
    try std.testing.expect((contract.flags & QUERY_CONTRACT_POINT_QUERY_VOXEL_CENTER) != 0);
    try std.testing.expect((contract.flags & QUERY_CONTRACT_NORMAL_UNIT_OR_UP) != 0);
    try std.testing.expect((contract.flags & QUERY_CONTRACT_CLASSIFICATION_MIRRORED) != 0);
    try std.testing.expect((contract.flags & QUERY_CONTRACT_STABLE_DISTANCE_SORT) != 0);
}

test "QueryAABB canonicalizes bounds and uses half-open overlap semantics" {
    const std = @import("std");
    const a = QueryAABB.init(3.0, 4.0, 5.0, 1.0, 2.0, 3.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), a.min_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), a.max_x, 0.0001);
    try std.testing.expect(!a.isEmpty());
    try std.testing.expect(a.containsPoint(1.0, 2.0, 3.0));
    try std.testing.expect(!a.containsPoint(3.0, 4.0, 5.0));

    const touching = QueryAABB.init(3.0, 2.0, 3.0, 5.0, 4.0, 5.0);
    try std.testing.expect(!a.overlaps(touching));

    const overlapping = QueryAABB.init(2.5, 2.0, 3.0, 5.0, 4.0, 5.0);
    try std.testing.expect(queryAABBOverlaps(a, overlapping));
}

test "VoxelBox canonicalizes bounds and converts to QueryAABB" {
    const std = @import("std");
    const a = VoxelBox.init(4, 5, 6, 1, 2, 3);
    try std.testing.expectEqual(@as(i32, 1), a.min_x);
    try std.testing.expectEqual(@as(i32, 4), a.max_x);
    try std.testing.expect(!a.isEmpty());
    try std.testing.expect(!a.overlaps(VoxelBox.init(4, 2, 3, 8, 5, 6)));
    try std.testing.expect(a.overlaps(VoxelBox.init(3, 2, 3, 8, 5, 6)));

    const q = a.toQueryAABB();
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), q.min_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), q.max_x, 0.0001);
}

test "raycastQueryAABB uses slab clipping and reports entry normal" {
    const std = @import("std");
    const box = QueryAABB.init(2.0, 1.0, 1.0, 4.0, 3.0, 3.0);

    const hit = raycastQueryAABB(box, .{
        .origin_x = 0.0,
        .origin_y = 2.0,
        .origin_z = 2.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 10.0,
    });
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), hit.distance, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);

    const miss_parallel = raycastQueryAABB(box, .{
        .origin_x = 0.0,
        .origin_y = 5.0,
        .origin_z = 2.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 10.0,
    });
    try std.testing.expect(!miss_parallel.hit);
}

test "raycastQueryAABB handles inside starts and max distance clamp" {
    const std = @import("std");
    const box = QueryAABB.init(2.0, 1.0, 1.0, 4.0, 3.0, 3.0);

    const inside = box.raycast(.{
        .origin_x = 3.0,
        .origin_y = 2.0,
        .origin_z = 2.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 10.0,
    });
    try std.testing.expect(inside.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), inside.distance, 0.0001);

    const beyond_max = box.raycast(.{
        .origin_x = 0.0,
        .origin_y = 2.0,
        .origin_z = 2.0,
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 1.0,
    });
    try std.testing.expect(!beyond_max.hit);
}

test "sweepQueryAABB reports time of impact against expanded target" {
    const std = @import("std");
    const moving = QueryAABB.init(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    const target = QueryAABB.init(3.0, 0.0, 0.0, 4.0, 1.0, 1.0);

    const hit = sweepQueryAABB(moving, target, 1.0, 0.0, 0.0, 10.0);
    try std.testing.expect(hit.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), hit.distance, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), hit.toi, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -1.0), hit.normal_x, 0.0001);

    const miss = moving.sweep(target, 0.0, 1.0, 0.0, 10.0);
    try std.testing.expect(!miss.hit);
}

test "sweepQueryAABB returns immediate hit for initial overlap and clamps range" {
    const std = @import("std");
    const moving = QueryAABB.init(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    const target = QueryAABB.init(1.0, 1.0, 1.0, 3.0, 3.0, 3.0);

    const overlap = sweepQueryAABB(moving, target, 1.0, 0.0, 0.0, 5.0);
    try std.testing.expect(overlap.hit);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), overlap.distance, 0.0001);

    const far_target = QueryAABB.init(6.0, 0.0, 0.0, 7.0, 1.0, 1.0);
    const too_short = sweepQueryAABB(QueryAABB.init(0.0, 0.0, 0.0, 1.0, 1.0, 1.0), far_target, 1.0, 0.0, 0.0, 2.0);
    try std.testing.expect(!too_short.hit);
}

test "query ray distance helpers separate parameter from world distance" {
    const std = @import("std");
    const ray = QueryRay{
        .origin_x = 0.0,
        .origin_y = 0.0,
        .origin_z = 0.0,
        .dir_x = 2.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .max_distance = 10.0,
    };

    try std.testing.expectApproxEqAbs(@as(f32, 2.0), queryRayDirectionLength(ray), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 6.0), queryRayParameterToDistance(ray, 3.0), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), queryRayDistanceToParameter(ray, 6.0), 0.0001);

    var hit: QueryHit = .{ .hit = true };
    setQueryHitDistance(&hit, -1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.distance, 0.0001);
}

test "query hit normal helper normalizes and falls back on degenerate input" {
    const std = @import("std");

    var hit: QueryHit = .{ .hit = true };
    setQueryHitNormal(&hit, 0.0, 2.0, 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), hit.normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_z, 0.0001);

    setQueryHitNormal(&hit, 0.0, 0.0, 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), hit.normal_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), hit.normal_z, 0.0001);
}

test "query hit position helpers store world-space points" {
    const std = @import("std");

    var hit: QueryHit = .{ .hit = true };
    setQueryHitPosition(&hit, 1.25, 2.5, 3.75);
    try std.testing.expectApproxEqAbs(@as(f32, 1.25), hit.position_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.5), hit.position_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.75), hit.position_z, 0.0001);

    setQueryHitVoxelCenterPosition(&hit, -1, 2, 4);
    try std.testing.expectApproxEqAbs(@as(f32, -0.5), hit.position_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.5), hit.position_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), hit.position_z, 0.0001);
}

test "query hit classification helper mirrors flattened legacy fields" {
    const std = @import("std");

    var hit: QueryHit = .{ .hit = true };
    const classification = ContactClassification{
        .body_type = .dynamic,
        .surface_type = .ice,
        .medium_type = .liquid,
        .material_type = .elastic,
        .surface_condition = .slippery,
        .hard_surface = false,
    };

    setQueryHitClassification(&hit, classification);

    try std.testing.expect(hit.classification.material_type == .elastic);
    try std.testing.expect(hit.classification.surface_condition == .slippery);
    try std.testing.expect(hit.classification.medium_type == .liquid);
    try std.testing.expect(hit.classification.body_type == .dynamic);
    try std.testing.expect(hit.material_type == hit.classification.material_type);
    try std.testing.expect(hit.surface_condition == hit.classification.surface_condition);
    try std.testing.expect(hit.medium_type == hit.classification.medium_type);
    try std.testing.expect(hit.body_type == hit.classification.body_type);
}

test "query hit consistency validation catches broken result invariants" {
    const std = @import("std");

    var hit: QueryHit = .{ .hit = true };
    setQueryHitNormal(&hit, 0.0, 2.0, 0.0);
    setQueryHitClassification(&hit, .{ .body_type = .dynamic, .material_type = .elastic });
    try std.testing.expect(queryHitIsConsistent(hit));

    var bad_distance = hit;
    bad_distance.distance = -1.0;
    try std.testing.expect(!queryHitIsConsistent(bad_distance));

    var bad_normal = hit;
    bad_normal.normal_y = 2.0;
    try std.testing.expect(!queryHitIsConsistent(bad_normal));

    var bad_mirror = hit;
    bad_mirror.material_type = .solid;
    try std.testing.expect(!queryHitIsConsistent(bad_mirror));
}

test "sortQueryHitsByDistanceStable orders hits by distance and preserves equal order" {
    const std = @import("std");
    resetQueryStats();
    var hits = [_]QueryHit{
        .{ .hit = true, .distance = 3.0, .toi = 3.0, .instance_idx = 3 },
        .{ .hit = true, .distance = 1.0, .toi = 1.0, .instance_idx = 1 },
        .{ .hit = true, .distance = 1.0, .toi = 1.0, .instance_idx = 2 },
        .{ .hit = false, .distance = 0.0, .instance_idx = 9 },
    };

    sortQueryHitsByDistanceStable(hits[0..]);

    try std.testing.expectEqual(@as(i16, 1), hits[0].instance_idx);
    try std.testing.expectEqual(@as(i16, 2), hits[1].instance_idx);
    try std.testing.expectEqual(@as(i16, 3), hits[2].instance_idx);
    try std.testing.expect(!hits[3].hit);

    const stats = getQueryStats();
    try std.testing.expectEqual(@as(u64, 1), stats.sort_calls);
    try std.testing.expectEqual(@as(u64, 4), stats.sorted_hits);
}
