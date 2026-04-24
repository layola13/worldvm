//! Query Types - Unified Query Layer Core Types
//!
//! Task A: Unified Query Layer
//! Provides consistent data structures for all physics queries

const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");

/// Body type for collision filtering
pub const BodyType = enum(u8) {
    static = 0,
    dynamic = 1,
    kinematic = 2,
    sensor = 3,
};

/// Query layer mask - 32 bits for collision layers
pub const QueryLayerMask = u32;

/// Query filter - controls what objects are hit by queries
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
};

/// Overlap query result
pub const OverlapResult = struct {
    hit: bool = false,
    count: u16 = 0,
    first_instance_idx: i16 = -1,
    environment_overlap: bool = false,
};

/// Penetration query result - for depenetration and collision resolution
pub const PenetrationResult = struct {
    overlapping: bool = false,
    depth: f32 = 0,
    dir_x: f32 = 0,
    dir_y: f32 = 1,
    dir_z: f32 = 0,
    instance_idx: i16 = -1,
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

/// AABB for box queries
pub const QueryAABB = struct {
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
};

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
