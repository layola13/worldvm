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

pub const BodyType = query_types.BodyType;
pub const QueryLayerMask = query_types.QueryLayerMask;
pub const QueryFilter = query_types.QueryFilter;
pub const SurfaceCondition = query_types.SurfaceCondition;
pub const ContactClassification = query_types.ContactClassification;
pub const ContactTelemetry = query_types.ContactTelemetry;
pub const QueryHit = query_types.QueryHit;
pub const OverlapResult = query_types.OverlapResult;
pub const PenetrationResult = query_types.PenetrationResult;
pub const QueryWorldView = query_types.QueryWorldView;
pub const QueryRay = query_types.QueryRay;
pub const QueryAABB = query_types.QueryAABB;
pub const QuerySphere = query_types.QuerySphere;
pub const QueryCapsule = query_types.QueryCapsule;

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

pub fn overlapAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapAABB(world, min_x, min_y, min_z, max_x, max_y, max_z, filter);
}

pub fn overlapSphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapSphere(world, center_x, center_y, center_z, radius, filter);
}

pub fn overlapCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) OverlapResult {
    return query_overlap.overlapCapsule(world, center_x, center_y, center_z, radius, half_height, filter);
}

pub fn raycastSingle(world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter) QueryHit {
    return query_raycast.raycastSingle(world, ray, filter);
}

pub fn raycastIgnoreSelf(world: *const QueryWorldView, ray: QueryRay, ignore_idx: u8) QueryHit {
    return query_raycast.raycastIgnoreSelf(world, ray, ignore_idx);
}

pub fn raycastAll(world: *const QueryWorldView, ray: QueryRay, filter: QueryFilter, out_hits: []QueryHit) u16 {
    return query_raycast.raycastAll(world, ray, filter, out_hits);
}

pub fn sphereCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    return query_sweep.sphereCast(world, center_x, center_y, center_z, radius, dir_x, dir_y, dir_z, max_distance, filter);
}

pub fn capsuleCast(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    return query_sweep.capsuleCast(world, center_x, center_y, center_z, radius, half_height, dir_x, dir_y, dir_z, max_distance, filter);
}

pub fn boxCast(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, dir_x: f32, dir_y: f32, dir_z: f32, max_distance: f32, filter: QueryFilter) QueryHit {
    return query_sweep.boxCast(world, min_x, min_y, min_z, max_x, max_y, max_z, dir_x, dir_y, dir_z, max_distance, filter);
}

pub fn computePenetrationAABB(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) PenetrationResult {
    return query_penetration.computePenetrationAABB(world, min_x, min_y, min_z, max_x, max_y, max_z, filter);
}

pub fn computePenetrationCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) PenetrationResult {
    return query_penetration.computePenetrationCapsule(world, center_x, center_y, center_z, radius, half_height, filter);
}
