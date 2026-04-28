//! Query - Unified Query Layer Compatibility Facade
//!
//! Keeps legacy imports stable while routing all behavior through the
//! split query modules (`query_world`, `query_raycast`, `query_sweep`,
//! `query_overlap`, `query_penetration`).

const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");
const query_raycast = @import("query_raycast.zig");
const query_sweep = @import("query_sweep.zig");
const query_overlap = @import("query_overlap.zig");
const query_penetration = @import("query_penetration.zig");
const query_debug = @import("query_debug.zig");
const query_benchmark = @import("query_benchmark.zig");
const query_regression = @import("query_regression.zig");

pub const BodyType = query_types.BodyType;
pub const QueryLayerMask = query_types.QueryLayerMask;
pub const QueryFilter = query_types.QueryFilter;
pub const SurfaceCondition = query_types.SurfaceCondition;
pub const ContactClassification = query_types.ContactClassification;
pub const ContactTelemetry = query_types.ContactTelemetry;
pub const QueryHit = query_types.QueryHit;
pub const OverlapResult = query_types.OverlapResult;
pub const OverlapBatchJob = query_overlap.OverlapBatchJob;
pub const PenetrationResult = query_types.PenetrationResult;
pub const QueryStats = query_types.QueryStats;
pub const QueryWorldView = query_types.QueryWorldView;
pub const QueryRay = query_types.QueryRay;
pub const QueryVoxelRequest = query_types.QueryVoxelRequest;
pub const QueryAABB = query_types.QueryAABB;
pub const QueryAABBRaycastResult = query_types.QueryAABBRaycastResult;
pub const QueryAABBSweepResult = query_types.QueryAABBSweepResult;
pub const QuerySphere = query_types.QuerySphere;
pub const QueryCapsule = query_types.QueryCapsule;
pub const QUERY_CONTRACT_VERSION = query_types.QUERY_CONTRACT_VERSION;
pub const QUERY_CONTRACT_DISTANCE_WORLD_UNITS = query_types.QUERY_CONTRACT_DISTANCE_WORLD_UNITS;
pub const QUERY_CONTRACT_TOI_RAY_PARAMETER = query_types.QUERY_CONTRACT_TOI_RAY_PARAMETER;
pub const QUERY_CONTRACT_POSITION_WORLD_SPACE = query_types.QUERY_CONTRACT_POSITION_WORLD_SPACE;
pub const QUERY_CONTRACT_POINT_QUERY_VOXEL_CENTER = query_types.QUERY_CONTRACT_POINT_QUERY_VOXEL_CENTER;
pub const QUERY_CONTRACT_NORMAL_UNIT_OR_UP = query_types.QUERY_CONTRACT_NORMAL_UNIT_OR_UP;
pub const QUERY_CONTRACT_CLASSIFICATION_MIRRORED = query_types.QUERY_CONTRACT_CLASSIFICATION_MIRRORED;
pub const QUERY_CONTRACT_STABLE_DISTANCE_SORT = query_types.QUERY_CONTRACT_STABLE_DISTANCE_SORT;
pub const QUERY_CONTRACT_FLAGS = query_types.QUERY_CONTRACT_FLAGS;
pub const QueryContract = query_types.QueryContract;
pub const QueryAnyVoxelBatchJob = query_world.QueryAnyVoxelBatchJob;
pub const QueryVoxelSimpleBatchJob = query_world.QueryVoxelSimpleBatchJob;
pub const QueryScratchPool = query_world.QueryScratchPool;
pub const QueryDebugPrimitiveKind = query_debug.QueryDebugPrimitiveKind;
pub const QueryDebugColor = query_debug.QueryDebugColor;
pub const QueryDebugPrimitive = query_debug.QueryDebugPrimitive;
pub const QueryDebugBuffer = query_debug.QueryDebugBuffer;
pub const QueryBenchmarkConfig = query_benchmark.QueryBenchmarkConfig;
pub const QueryBenchmarkResult = query_benchmark.QueryBenchmarkResult;
pub const QueryRegressionResult = query_regression.QueryRegressionResult;
pub const QUERY_SCRATCH_REQUEST_CAPACITY = query_world.QUERY_SCRATCH_REQUEST_CAPACITY;
pub const QUERY_SCRATCH_HIT_CAPACITY = query_world.QUERY_SCRATCH_HIT_CAPACITY;
pub const QUERY_SCRATCH_BOOL_CAPACITY = query_world.QUERY_SCRATCH_BOOL_CAPACITY;
pub const sortQueryHitsByDistanceStable = query_types.sortQueryHitsByDistanceStable;
pub const makeQueryAABB = query_types.makeQueryAABB;
pub const queryAABBOverlaps = query_types.queryAABBOverlaps;
pub const queryAABBContainsPoint = query_types.queryAABBContainsPoint;
pub const raycastQueryAABB = query_types.raycastQueryAABB;
pub const sweepQueryAABB = query_types.sweepQueryAABB;
pub const resetQueryStats = query_types.resetQueryStats;
pub const getQueryStats = query_types.getQueryStats;
pub const getQueryContract = query_types.getQueryContract;
pub const queryHitIsConsistent = query_types.queryHitIsConsistent;
pub const setQueryHitDistance = query_types.setQueryHitDistance;
pub const setQueryHitPosition = query_types.setQueryHitPosition;
pub const setQueryHitVoxelCenterPosition = query_types.setQueryHitVoxelCenterPosition;
pub const setQueryHitClassification = query_types.setQueryHitClassification;
pub const setQueryHitNormal = query_types.setQueryHitNormal;
pub const normalizeQueryHitNormal = query_types.normalizeQueryHitNormal;

pub fn queryEnvironmentVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool {
    return query_world.queryEnvironmentVoxel(world, gx, gy, gz);
}

pub fn queryInstanceVoxel(world: *const QueryWorldView, inst_idx: u8, gx: i32, gy: i32, gz: i32) bool {
    return query_world.queryInstanceVoxel(world, inst_idx, gx, gy, gz);
}

pub fn getInstanceBodyType(world: *const QueryWorldView, inst_idx: u8) BodyType {
    return query_world.getInstanceBodyType(world, inst_idx);
}

pub fn queryAnyVoxel(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32, filter: QueryFilter) QueryHit {
    return query_world.queryAnyVoxel(world, gx, gy, gz, filter);
}

pub fn queryVoxelSimple(world: *const QueryWorldView, gx: i32, gy: i32, gz: i32) bool {
    return query_world.queryVoxelSimple(world, gx, gy, gz);
}

pub fn queryAnyVoxelBatch(world: *const QueryWorldView, requests: []const QueryVoxelRequest, filter: QueryFilter, out_hits: []QueryHit) u16 {
    return query_world.queryAnyVoxelBatch(world, requests, filter, out_hits);
}

pub fn queryVoxelSimpleBatch(world: *const QueryWorldView, requests: []const QueryVoxelRequest, out_hits: []bool) u16 {
    return query_world.queryVoxelSimpleBatch(world, requests, out_hits);
}

pub fn beginQueryAnyVoxelBatch(requests: []const QueryVoxelRequest, filter: QueryFilter, out_hits: []QueryHit) QueryAnyVoxelBatchJob {
    return QueryAnyVoxelBatchJob.begin(requests, filter, out_hits);
}

pub fn beginQueryVoxelSimpleBatch(requests: []const QueryVoxelRequest, out_hits: []bool) QueryVoxelSimpleBatchJob {
    return QueryVoxelSimpleBatchJob.begin(requests, out_hits);
}

pub fn appendPointQueryDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, gx: i32, gy: i32, gz: i32, filter: QueryFilter) QueryHit {
    return query_debug.appendPointQueryDebug(buffer, world, gx, gy, gz, filter);
}

pub fn appendRaycastDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter) QueryHit {
    return query_debug.appendRaycastDebug(buffer, world, ray, filter);
}

pub fn appendAABBDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) OverlapResult {
    return query_debug.appendAABBDebug(buffer, world, min_x, min_y, min_z, max_x, max_y, max_z, filter);
}

pub fn appendQueryAABBDebug(buffer: *QueryDebugBuffer, world: *const QueryWorldView, aabb: QueryAABB, filter: QueryFilter) OverlapResult {
    return query_debug.appendQueryAABBDebug(buffer, world, aabb, filter);
}

pub fn runQueryBenchmark(allocator: @import("std").mem.Allocator, config: QueryBenchmarkConfig) !QueryBenchmarkResult {
    return query_benchmark.runQueryBenchmark(allocator, config);
}

pub fn runQueryRegressionSuite(allocator: @import("std").mem.Allocator) !QueryRegressionResult {
    return query_regression.runQueryRegressionSuite(allocator);
}

pub fn overlapAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapAABB(world, min_x, min_y, min_z, max_x, max_y, max_z, filter);
}

pub fn overlapQueryAABB(world: *const QueryWorldView, aabb: QueryAABB, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapQueryAABB(world, aabb, filter);
}

pub fn overlapAABBSingle(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) QueryHit {
    return query_overlap.overlapAABBSingle(world, min_x, min_y, min_z, max_x, max_y, max_z, filter);
}

pub fn overlapQueryAABBSingle(world: *const QueryWorldView, aabb: QueryAABB, filter: QueryFilter) QueryHit {
    return query_overlap.overlapQueryAABBSingle(world, aabb, filter);
}

pub fn overlapSphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapSphere(world, center_x, center_y, center_z, radius, filter);
}

pub fn overlapHemisphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, axis_x: f32, axis_y: f32, axis_z: f32, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapHemisphere(world, center_x, center_y, center_z, radius, axis_x, axis_y, axis_z, filter);
}

pub fn overlapHemisphereWithLayerMask(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, axis_x: f32, axis_y: f32, axis_z: f32, layer_mask: QueryLayerMask) OverlapResult {
    return overlapHemisphere(world, center_x, center_y, center_z, radius, axis_x, axis_y, axis_z, .{ .layer_mask = layer_mask });
}

pub fn overlapCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapCapsule(world, center_x, center_y, center_z, radius, half_height, filter);

}

pub fn overlapBatchAABBs(world: *const QueryWorldView, boxes: []const QueryAABB, filter: QueryFilter, out_results: []OverlapResult) u16 {
    return query_overlap.overlapBatchAABBs(world, boxes, filter, out_results);
}

pub fn raycastSingleWithLayerMask(world: *const QueryWorldView, ray: QueryRay, layer_mask: QueryLayerMask) QueryHit {
    return raycastSingle(world, ray, .{ .layer_mask = layer_mask });
}

pub fn raycastIgnoreSelf(world: *const QueryWorldView, ray: QueryRay, ignore_idx: u8) QueryHit {
    return query_raycast.raycastIgnoreSelf(world, ray, ignore_idx);
}

pub fn raycastAll(world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter, out_hits: []QueryHit) u16 {
    return query_raycast.raycastAll(world, ray, filter, out_hits);
}

pub fn raycastBatch(world: *const QueryWorldView, rays: []const QueryRay, filter: QueryFilter, out_hits: []QueryHit) u16 {
    return query_raycast.raycastBatch(world, rays, filter, out_hits);
}

pub fn queryGroundBelowPoint(world: *const QueryWorldView, world_x: f32, world_y: f32, world_z: f32, filter: QueryFilter) QueryHit {
    return query_raycast.queryGroundBelowPoint(world, world_x, world_y, world_z, filter);
}

pub fn sphereCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    return query_sweep.sphereCast(world, center_x, center_y, center_z, radius, dir_x, dir_y, dir_z, max_distance, filter);
}

pub fn sphereCastWithLayerMask(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, layer_mask: QueryLayerMask) QueryHit {
    return sphereCast(world, center_x, center_y, center_z, radius, dir_x, dir_y, dir_z, max_distance, .{ .layer_mask = layer_mask });
}

pub fn capsuleCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    return query_sweep.capsuleCast(world, center_x, center_y, center_z, radius, half_height, dir_x, dir_y, dir_z, max_distance, filter);
}

pub fn boxCast(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    return query_sweep.boxCast(world, min_x, min_y, min_z, max_x, max_y, max_z, dir_x, dir_y, dir_z, max_distance, filter);
}

pub fn boxCastWithLayerMask(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, layer_mask: QueryLayerMask) QueryHit {
    return boxCast(world, min_x, min_y, min_z, max_x, max_y, max_z, dir_x, dir_y, dir_z, max_distance, .{ .layer_mask = layer_mask });
}

pub fn overlapAABBWithLayerMask(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, layer_mask: QueryLayerMask) QueryHit {
    return overlapAABBSingle(world, min_x, min_y, min_z, max_x, max_y, max_z, .{ .layer_mask = layer_mask });
}

pub fn computePenetrationBox(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) PenetrationResult {
    return query_penetration.computePenetrationBox(world, min_x, min_y, min_z, max_x, max_y, max_z, filter);
}

pub fn computePenetrationQueryAABB(world: *const QueryWorldView, aabb: QueryAABB, filter: QueryFilter) PenetrationResult {
    return query_penetration.computePenetrationQueryAABB(world, aabb, filter);
}

pub fn computePenetrationAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) PenetrationResult {
    return query_penetration.computePenetrationAABB(world, min_x, min_y, min_z, max_x, max_y, max_z, filter);
}

pub fn computePenetrationBoxWithLayerMask(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, layer_mask: QueryLayerMask) PenetrationResult {
    return computePenetrationBox(world, min_x, min_y, min_z, max_x, max_y, max_z, .{ .layer_mask = layer_mask });
}

pub fn computePenetrationAABBWithLayerMask(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, layer_mask: QueryLayerMask) PenetrationResult {
    return computePenetrationAABB(world, min_x, min_y, min_z, max_x, max_y, max_z, .{ .layer_mask = layer_mask });
}

pub fn computePenetrationCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) PenetrationResult {
    return query_penetration.computePenetrationCapsule(world, center_x, center_y, center_z, radius, half_height, filter);
}

pub fn computePenetrationCapsuleWithLayerMask(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, layer_mask: QueryLayerMask) PenetrationResult {
    return computePenetrationCapsule(world, center_x, center_y, center_z, radius, half_height, .{ .layer_mask = layer_mask });
}

pub fn computePenetrationHemisphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, axis_x: f32, axis_y: f32, axis_z: f32, filter: QueryFilter) PenetrationResult {
    return query_penetration.computePenetrationHemisphere(world, center_x, center_y, center_z, radius, axis_x, axis_y, axis_z, filter);
}

pub fn computePenetrationHemisphereWithLayerMask(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, axis_x: f32, axis_y: f32, axis_z: f32, layer_mask: QueryLayerMask) PenetrationResult {
    return computePenetrationHemisphere(world, center_x, center_y, center_z, radius, axis_x, axis_y, axis_z, .{ .layer_mask = layer_mask });
}
