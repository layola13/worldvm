//! Query Penetration - L4 Penetration Queries
//!
//! Task A: Unified Query Layer
//! Penetration depth and depenetration direction queries

const std = @import("std");
const physics = @import("physics.zig");
const query_types = @import("query_types.zig");
const query_world = @import("query_world.zig");

const PenetrationResult = query_types.PenetrationResult;
const ContactPoint = query_types.ContactPoint;
const QueryAABB = query_types.QueryAABB;
const QueryFilter = query_types.QueryFilter;
const QueryWorldView = query_types.QueryWorldView;

pub const CapsuleBoxClosestPoint = struct {
    seg_y: f32,
    closest_x: f32,
    closest_y: f32,
    closest_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    dist_sq: f32,
};

pub const SphereBoxClosestPoint = struct {
    closest_x: f32,
    closest_y: f32,
    closest_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    dist_sq: f32,
};

pub const AABBVoxelRange = struct {
    start_x: i32,
    start_y: i32,
    start_z: i32,
    end_x: i32,
    end_y: i32,
    end_z: i32,
};

pub const SphereVoxelRange = struct {
    start_x: i32,
    start_y: i32,
    start_z: i32,
    end_x: i32,
    end_y: i32,
    end_z: i32,
};

pub const CapsuleVoxelRange = struct {
    start_x: i32,
    start_y: i32,
    start_z: i32,
    end_x: i32,
    end_y: i32,
    end_z: i32,
};

fn updateBestPenetration(result: *PenetrationResult, depth: f32, dir_x: f32, dir_y: f32, dir_z: f32, hit: query_types.QueryHit) void {
    if (depth <= 0) return;
    if (!result.overlapping or depth < result.depth) {
        result.overlapping = true;
        result.depth = depth;
        result.dir_x = dir_x;
        result.dir_y = dir_y;
        result.dir_z = dir_z;
        result.instance_idx = hit.instance_idx;
        result.entity_id = hit.entity_id;
        result.hit_environment = hit.hit_environment;
        result.classification = hit.classification;
        result.telemetry = hit.telemetry;
    }
}

const ManifoldAxis = enum {
    x,
    y,
    z,
};

const ManifoldAccumulator = struct {
    valid: bool = false,
    axis: ManifoldAxis = .x,
    plane: f32 = 0,
    min_a: f32 = 0,
    max_a: f32 = 0,
    min_b: f32 = 0,
    max_b: f32 = 0,
};

fn approxEqAbs(a: f32, b: f32, tolerance: f32) bool {
    return @abs(a - b) <= tolerance;
}

fn resetManifold(result: *PenetrationResult) void {
    result.manifold_point_count = 0;
    var i: usize = 0;
    while (i < result.manifold_points.len) : (i += 1) {
        result.manifold_points[i] = .{};
    }
}

fn appendUniqueManifoldPoint(result: *PenetrationResult, x: f32, y: f32, z: f32) void {
    const tolerance: f32 = 0.0001;
    var i: usize = 0;
    while (i < result.manifold_point_count) : (i += 1) {
        const existing = result.manifold_points[i];
        if (approxEqAbs(existing.x, x, tolerance) and
            approxEqAbs(existing.y, y, tolerance) and
            approxEqAbs(existing.z, z, tolerance))
        {
            return;
        }
    }

    if (result.manifold_point_count >= result.manifold_points.len) return;
    result.manifold_points[result.manifold_point_count] = ContactPoint{
        .x = x,
        .y = y,
        .z = z,
    };
    result.manifold_point_count += 1;
}

fn accumulateManifoldPatch(acc: *ManifoldAccumulator, axis: ManifoldAxis, plane: f32, min_a: f32, max_a: f32, min_b: f32, max_b: f32) void {
    if (max_a <= min_a or max_b <= min_b) return;

    if (!acc.valid) {
        acc.* = .{
            .valid = true,
            .axis = axis,
            .plane = plane,
            .min_a = min_a,
            .max_a = max_a,
            .min_b = min_b,
            .max_b = max_b,
        };
        return;
    }

    acc.min_a = @min(acc.min_a, min_a);
    acc.max_a = @max(acc.max_a, max_a);
    acc.min_b = @min(acc.min_b, min_b);
    acc.max_b = @max(acc.max_b, max_b);
}

fn writeAccumulatedManifold(result: *PenetrationResult, acc: ManifoldAccumulator) void {
    if (!acc.valid) return;

    resetManifold(result);
    switch (acc.axis) {
        .x => {
            appendUniqueManifoldPoint(result, acc.plane, acc.min_a, acc.min_b);
            appendUniqueManifoldPoint(result, acc.plane, acc.min_a, acc.max_b);
            appendUniqueManifoldPoint(result, acc.plane, acc.max_a, acc.min_b);
            appendUniqueManifoldPoint(result, acc.plane, acc.max_a, acc.max_b);
        },
        .y => {
            appendUniqueManifoldPoint(result, acc.min_a, acc.plane, acc.min_b);
            appendUniqueManifoldPoint(result, acc.min_a, acc.plane, acc.max_b);
            appendUniqueManifoldPoint(result, acc.max_a, acc.plane, acc.min_b);
            appendUniqueManifoldPoint(result, acc.max_a, acc.plane, acc.max_b);
        },
        .z => {
            appendUniqueManifoldPoint(result, acc.min_a, acc.min_b, acc.plane);
            appendUniqueManifoldPoint(result, acc.min_a, acc.max_b, acc.plane);
            appendUniqueManifoldPoint(result, acc.max_a, acc.min_b, acc.plane);
            appendUniqueManifoldPoint(result, acc.max_a, acc.max_b, acc.plane);
        },
    }
}

fn accumulateBestFacePatch(
    acc: *ManifoldAccumulator,
    result: PenetrationResult,
    hit_instance_idx: i16,
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
    voxel_min_x: f32,
    voxel_min_y: f32,
    voxel_min_z: f32,
    voxel_max_x: f32,
    voxel_max_y: f32,
    voxel_max_z: f32,
) void {
    if (!result.overlapping) return;
    if (hit_instance_idx != result.instance_idx) return;

    const tolerance: f32 = 0.0001;
    if (result.dir_x < 0 and approxEqAbs(max_x - voxel_min_x, result.depth, tolerance)) {
        accumulateManifoldPatch(
            acc,
            .x,
            voxel_min_x,
            @max(min_y, voxel_min_y),
            @min(max_y, voxel_max_y),
            @max(min_z, voxel_min_z),
            @min(max_z, voxel_max_z),
        );
    } else if (result.dir_x > 0 and approxEqAbs(voxel_max_x - min_x, result.depth, tolerance)) {
        accumulateManifoldPatch(
            acc,
            .x,
            voxel_max_x,
            @max(min_y, voxel_min_y),
            @min(max_y, voxel_max_y),
            @max(min_z, voxel_min_z),
            @min(max_z, voxel_max_z),
        );
    } else if (result.dir_y < 0 and approxEqAbs(max_y - voxel_min_y, result.depth, tolerance)) {
        accumulateManifoldPatch(
            acc,
            .y,
            voxel_min_y,
            @max(min_x, voxel_min_x),
            @min(max_x, voxel_max_x),
            @max(min_z, voxel_min_z),
            @min(max_z, voxel_max_z),
        );
    } else if (result.dir_y > 0 and approxEqAbs(voxel_max_y - min_y, result.depth, tolerance)) {
        accumulateManifoldPatch(
            acc,
            .y,
            voxel_max_y,
            @max(min_x, voxel_min_x),
            @min(max_x, voxel_max_x),
            @max(min_z, voxel_min_z),
            @min(max_z, voxel_max_z),
        );
    } else if (result.dir_z < 0 and approxEqAbs(max_z - voxel_min_z, result.depth, tolerance)) {
        accumulateManifoldPatch(
            acc,
            .z,
            voxel_min_z,
            @max(min_x, voxel_min_x),
            @min(max_x, voxel_max_x),
            @max(min_y, voxel_min_y),
            @min(max_y, voxel_max_y),
        );
    } else if (result.dir_z > 0 and approxEqAbs(voxel_max_z - min_z, result.depth, tolerance)) {
        accumulateManifoldPatch(
            acc,
            .z,
            voxel_max_z,
            @max(min_x, voxel_min_x),
            @min(max_x, voxel_max_x),
            @max(min_y, voxel_min_y),
            @min(max_y, voxel_max_y),
        );
    }
}

fn clampf(value: f32, min_value: f32, max_value: f32) f32 {
    return @max(min_value, @min(max_value, value));
}

pub fn computeAABBVoxelRange(min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32) AABBVoxelRange {
    return computeQueryAABBVoxelRange(QueryAABB.init(min_x, min_y, min_z, max_x, max_y, max_z));
}

pub fn computeQueryAABBVoxelRange(aabb: QueryAABB) AABBVoxelRange {
    const box = aabb.normalized();
    return .{
        .start_x = @intFromFloat(@floor(box.min_x)),
        .start_y = @intFromFloat(@floor(box.min_y)),
        .start_z = @intFromFloat(@floor(box.min_z)),
        .end_x = @as(i32, @intFromFloat(@ceil(box.max_x))) - 1,
        .end_y = @as(i32, @intFromFloat(@ceil(box.max_y))) - 1,
        .end_z = @as(i32, @intFromFloat(@ceil(box.max_z))) - 1,
    };
}

pub fn computeSphereVoxelRange(center_x: f32, center_y: f32, center_z: f32, radius: f32) SphereVoxelRange {
    return .{
        .start_x = @intFromFloat(@floor(center_x - radius)),
        .start_y = @intFromFloat(@floor(center_y - radius)),
        .start_z = @intFromFloat(@floor(center_z - radius)),
        .end_x = @intFromFloat(@ceil(center_x + radius)),
        .end_y = @intFromFloat(@ceil(center_y + radius)),
        .end_z = @intFromFloat(@ceil(center_z + radius)),
    };
}

pub fn computeCapsuleVoxelRange(center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32) CapsuleVoxelRange {
    const seg_min_y = center_y - half_height;
    const seg_max_y = center_y + half_height;
    return .{
        .start_x = @intFromFloat(@floor(center_x - radius)),
        .start_y = @intFromFloat(@floor(seg_min_y - radius)),
        .start_z = @intFromFloat(@floor(center_z - radius)),
        .end_x = @intFromFloat(@ceil(center_x + radius)),
        .end_y = @intFromFloat(@ceil(seg_max_y + radius)),
        .end_z = @intFromFloat(@ceil(center_z + radius)),
    };
}

pub fn computeSphereBoxClosestPoint(center_x: f32, center_y: f32, center_z: f32, box_min_x: f32, box_min_y: f32, box_min_z: f32, box_max_x: f32, box_max_y: f32, box_max_z: f32) SphereBoxClosestPoint {
    const closest_x = clampf(center_x, box_min_x, box_max_x);
    const closest_y = clampf(center_y, box_min_y, box_max_y);
    const closest_z = clampf(center_z, box_min_z, box_max_z);
    const dir_x = center_x - closest_x;
    const dir_y = center_y - closest_y;
    const dir_z = center_z - closest_z;
    return .{
        .closest_x = closest_x,
        .closest_y = closest_y,
        .closest_z = closest_z,
        .dir_x = dir_x,
        .dir_y = dir_y,
        .dir_z = dir_z,
        .dist_sq = dir_x * dir_x + dir_y * dir_y + dir_z * dir_z,
    };
}

pub fn computeCapsuleBoxClosestPoint(center_x: f32, center_y: f32, center_z: f32, seg_min_y: f32, seg_max_y: f32, box_min_x: f32, box_min_y: f32, box_min_z: f32, box_max_x: f32, box_max_y: f32, box_max_z: f32) CapsuleBoxClosestPoint {
    var seg_y = center_y;
    if (seg_max_y < box_min_y) {
        seg_y = seg_max_y;
    } else if (seg_min_y > box_max_y) {
        seg_y = seg_min_y;
    } else {
        seg_y = clampf(center_y, box_min_y, box_max_y);
    }

    const closest_x = clampf(center_x, box_min_x, box_max_x);
    const closest_y = clampf(seg_y, box_min_y, box_max_y);
    const closest_z = clampf(center_z, box_min_z, box_max_z);
    const dir_x = center_x - closest_x;
    const dir_y = seg_y - closest_y;
    const dir_z = center_z - closest_z;
    return .{
        .seg_y = seg_y,
        .closest_x = closest_x,
        .closest_y = closest_y,
        .closest_z = closest_z,
        .dir_x = dir_x,
        .dir_y = dir_y,
        .dir_z = dir_z,
        .dist_sq = dir_x * dir_x + dir_y * dir_y + dir_z * dir_z,
    };
}

pub const PlaneBoxDistance = struct {
    min_dot: f32,
    max_dot: f32,
};

/// Compute min/max dot products of box corners with plane normal
pub fn computePlaneBoxDistance(plane_origin_x: f32, plane_origin_y: f32, plane_origin_z: f32, axis_x: f32, axis_y: f32, axis_z: f32, box_min_x: f32, box_min_y: f32, box_min_z: f32, box_max_x: f32, box_max_y: f32, box_max_z: f32) PlaneBoxDistance {
    const corners = [_][3]f32{
        .{ box_min_x, box_min_y, box_min_z },
        .{ box_min_x, box_min_y, box_max_z },
        .{ box_min_x, box_max_y, box_min_z },
        .{ box_min_x, box_max_y, box_max_z },
        .{ box_max_x, box_min_y, box_min_z },
        .{ box_max_x, box_min_y, box_max_z },
        .{ box_max_x, box_max_y, box_min_z },
        .{ box_max_x, box_max_y, box_max_z },
    };

    var min_dot: f32 = std.math.inf(f32);
    var max_dot: f32 = -std.math.inf(f32);

    for (corners) |c| {
        const dx = c[0] - plane_origin_x;
        const dy = c[1] - plane_origin_y;
        const dz = c[2] - plane_origin_z;
        const dot = dx * axis_x + dy * axis_y + dz * axis_z;
        min_dot = @min(min_dot, dot);
        max_dot = @max(max_dot, dot);
    }

    return .{ .min_dot = min_dot, .max_dot = max_dot };
}

fn updateBestCapsulePenetration(result: *PenetrationResult, depth: f32, dir_x: f32, dir_y: f32, dir_z: f32, hit: query_types.QueryHit) void {
    if (depth <= 0) return;
    if (!result.overlapping or depth > result.depth) {
        result.overlapping = true;
        result.depth = depth;
        result.dir_x = dir_x;
        result.dir_y = dir_y;
        result.dir_z = dir_z;
        result.instance_idx = hit.instance_idx;
        result.entity_id = hit.entity_id;
        result.hit_environment = hit.hit_environment;
        result.classification = hit.classification;
        result.telemetry = hit.telemetry;
    }
}

/// Compute penetration of hemisphere into world
pub fn computePenetrationHemisphere(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, axis_x: f32, axis_y: f32, axis_z: f32, filter: QueryFilter) PenetrationResult {
    var result: PenetrationResult = .{};
    const radius_sq = radius * radius;
    const range = computeSphereVoxelRange(center_x, center_y, center_z, radius);
    var cache = query_world.QueryAnyVoxelCache.init(filter);

    var gx = range.start_x;
    while (gx <= range.end_x) : (gx += 1) {
        var gy = range.start_y;
        while (gy <= range.end_y) : (gy += 1) {
            var gz = range.start_z;
            while (gz <= range.end_z) : (gz += 1) {
                const box_min_x = @as(f32, @floatFromInt(gx));
                const box_min_y = @as(f32, @floatFromInt(gy));
                const box_min_z = @as(f32, @floatFromInt(gz));
                const box_max_x = box_min_x + 1.0;
                const box_max_y = box_min_y + 1.0;
                const box_max_z = box_min_z + 1.0;

                const closest = computeSphereBoxClosestPoint(center_x, center_y, center_z, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                if (closest.dist_sq > radius_sq) continue;

                // Check if the closest point is on the hemisphere side
                const dx = closest.closest_x - center_x;
                const dy = closest.closest_y - center_y;
                const dz = closest.closest_z - center_z;
                const dot = dx * axis_x + dy * axis_y + dz * axis_z;

                if (dot < 0) {
                    const plane_dist = computePlaneBoxDistance(center_x, center_y, center_z, axis_x, axis_y, axis_z, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                    if (plane_dist.max_dot < 0) continue;
                }

                const hit = query_world.queryAnyVoxelCached(world, &cache, gx, gy, gz);
                if (!hit.hit) continue;

                // If it's a deep penetration (inside), we need to resolve it.
                // For simplicity, we use the sphere resolution if it's in front of the plane,
                // or we push it to the plane if it's straddling.

                if (dot >= 0) {
                    if (closest.dist_sq > 0.000001) {
                        const dist = @sqrt(closest.dist_sq);
                        const depth = radius - dist;
                        updateBestPenetration(&result, depth, closest.dir_x / dist, closest.dir_y / dist, closest.dir_z / dist, hit);
                    } else {
                        // Interior - push out shortest way
                        const pdir = resolveInteriorCapsuleDirection(center_x, center_y, center_z, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                        updateBestPenetration(&result, radius, pdir.x, pdir.y, pdir.z, hit);
                    }
                } else {
                    // Straddling the plane from behind - push to plane
                    const plane_dist = computePlaneBoxDistance(center_x, center_y, center_z, axis_x, axis_y, axis_z, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                    updateBestPenetration(&result, plane_dist.max_dot, axis_x, axis_y, axis_z, hit);
                }
            }
        }
    }

    return result;
}

const PenetrationDir = struct {
    x: f32,
    y: f32,
    z: f32,
};

fn resolveInteriorCapsuleDirection(seg_x: f32, seg_y: f32, seg_z: f32, box_min_x: f32, box_min_y: f32, box_min_z: f32, box_max_x: f32, box_max_y: f32, box_max_z: f32) PenetrationDir {
    const push_neg_x = seg_x - box_min_x;
    const push_pos_x = box_max_x - seg_x;
    const push_neg_y = seg_y - box_min_y;
    const push_pos_y = box_max_y - seg_y;
    const push_neg_z = seg_z - box_min_z;
    const push_pos_z = box_max_z - seg_z;

    var best_depth = push_neg_x;
    var dir = PenetrationDir{ .x = -1.0, .y = 0.0, .z = 0.0 };

    if (push_pos_x < best_depth) {
        best_depth = push_pos_x;
        dir = .{ .x = 1.0, .y = 0.0, .z = 0.0 };
    }
    if (push_neg_y < best_depth) {
        best_depth = push_neg_y;
        dir = .{ .x = 0.0, .y = -1.0, .z = 0.0 };
    }
    if (push_pos_y < best_depth) {
        best_depth = push_pos_y;
        dir = .{ .x = 0.0, .y = 1.0, .z = 0.0 };
    }
    if (push_neg_z < best_depth) {
        best_depth = push_neg_z;
        dir = .{ .x = 0.0, .y = 0.0, .z = -1.0 };
    }
    if (push_pos_z < best_depth) {
        dir = .{ .x = 0.0, .y = 0.0, .z = 1.0 };
    }

    return dir;
}

/// Compute penetration of Box (AABB) into world
pub fn computePenetrationBox(world: *const QueryWorldView, min_x: f32, min_y: f32, min_z: f32, max_x: f32, max_y: f32, max_z: f32, filter: QueryFilter) PenetrationResult {
    return computePenetrationQueryAABB(world, QueryAABB.init(min_x, min_y, min_z, max_x, max_y, max_z), filter);
}

pub fn computePenetrationQueryAABB(world: *const QueryWorldView, aabb: QueryAABB, filter: QueryFilter) PenetrationResult {
    const box = aabb.normalized();
    if (box.isEmpty()) return .{};

    var result: PenetrationResult = .{};
    const range = computeQueryAABBVoxelRange(box);
    var cache = query_world.QueryAnyVoxelCache.init(filter);

    var gx = range.start_x;
    while (gx <= range.end_x) : (gx += 1) {
        var gy = range.start_y;
        while (gy <= range.end_y) : (gy += 1) {
            var gz = range.start_z;
            while (gz <= range.end_z) : (gz += 1) {
                const voxel_min_x = @as(f32, @floatFromInt(gx));
                const voxel_min_y = @as(f32, @floatFromInt(gy));
                const voxel_min_z = @as(f32, @floatFromInt(gz));
                const voxel_max_x = voxel_min_x + 1.0;
                const voxel_max_y = voxel_min_y + 1.0;
                const voxel_max_z = voxel_min_z + 1.0;

                const overlap_x = @min(box.max_x, voxel_max_x) - @max(box.min_x, voxel_min_x);
                const overlap_y = @min(box.max_y, voxel_max_y) - @max(box.min_y, voxel_min_y);
                const overlap_z = @min(box.max_z, voxel_max_z) - @max(box.min_z, voxel_min_z);

                if (overlap_x > 0 and overlap_y > 0 and overlap_z > 0) {
                    const hit = query_world.queryAnyVoxelCached(world, &cache, gx, gy, gz);
                    if (hit.hit) {
                        const push_neg_x = box.max_x - voxel_min_x;
                        const push_pos_x = voxel_max_x - box.min_x;
                        const push_neg_y = box.max_y - voxel_min_y;
                        const push_pos_y = voxel_max_y - box.min_y;
                        const push_neg_z = box.max_z - voxel_min_z;
                        const push_pos_z = voxel_max_z - box.min_z;

                        updateBestPenetration(&result, push_neg_x, -1, 0, 0, hit);
                        updateBestPenetration(&result, push_pos_x, 1, 0, 0, hit);
                        updateBestPenetration(&result, push_neg_y, 0, -1, 0, hit);
                        updateBestPenetration(&result, push_pos_y, 0, 1, 0, hit);
                        updateBestPenetration(&result, push_neg_z, 0, 0, -1, hit);
                        updateBestPenetration(&result, push_pos_z, 0, 0, 1, hit);
                    }
                }
            }
        }
    }

    if (!result.overlapping) return result;

    var accumulator: ManifoldAccumulator = .{};
    gx = range.start_x;
    while (gx <= range.end_x) : (gx += 1) {
        var gy = range.start_y;
        while (gy <= range.end_y) : (gy += 1) {
            var gz = range.start_z;
            while (gz <= range.end_z) : (gz += 1) {
                const voxel_min_x = @as(f32, @floatFromInt(gx));
                const voxel_min_y = @as(f32, @floatFromInt(gy));
                const voxel_min_z = @as(f32, @floatFromInt(gz));
                const voxel_max_x = voxel_min_x + 1.0;
                const voxel_max_y = voxel_min_y + 1.0;
                const voxel_max_z = voxel_min_z + 1.0;

                const overlap_x = @min(box.max_x, voxel_max_x) - @max(box.min_x, voxel_min_x);
                const overlap_y = @min(box.max_y, voxel_max_y) - @max(box.min_y, voxel_min_y);
                const overlap_z = @min(box.max_z, voxel_max_z) - @max(box.min_z, voxel_min_z);
                if (overlap_x <= 0 or overlap_y <= 0 or overlap_z <= 0) continue;

                const hit = query_world.queryAnyVoxelCached(world, &cache, gx, gy, gz);
                if (!hit.hit) continue;

                accumulateBestFacePatch(
                    &accumulator,
                    result,
                    hit.instance_idx,
                    box.min_x,
                    box.min_y,
                    box.min_z,
                    box.max_x,
                    box.max_y,
                    box.max_z,
                    voxel_min_x,
                    voxel_min_y,
                    voxel_min_z,
                    voxel_max_x,
                    voxel_max_y,
                    voxel_max_z,
                );
            }
        }
    }

    writeAccumulatedManifold(&result, accumulator);
    return result;
}

/// Backward-compatible alias for AABB penetration queries.
pub fn computePenetrationAABB(
    world: *const QueryWorldView,
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
    filter: QueryFilter,
) PenetrationResult {
    return computePenetrationBox(
        world,
        min_x,
        min_y,
        min_z,
        max_x,
        max_y,
        max_z,
        filter,
    );
}

test "computePenetrationAABB returns smallest translation for single voxel overlap" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

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

    const result = computePenetrationBox(&world, 1.8, 2.1, 2.1, 2.4, 2.9, 2.9, .{});
    try std.testing.expect(result.overlapping);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), result.depth, 0.0001);
    try std.testing.expect(result.dir_x == -1);
    try std.testing.expect(result.dir_y == 0);
    try std.testing.expect(result.dir_z == 0);
    try std.testing.expect(result.manifold_point_count > 0);
}

test "computePenetrationAABB returns no overlap when box is clear" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const result = computePenetrationBox(&world, 1.1, 1.1, 1.1, 1.8, 1.8, 1.8, .{});
    try std.testing.expect(!result.overlapping);
    try std.testing.expect(result.depth == 0);
    try std.testing.expect(result.manifold_point_count == 0);
}

test "computePenetrationAABB keeps manifold aligned with best translation voxel" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const near_addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 0,
        .lz = 0,
    });
    const far_addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 2,
        .ly = 0,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(near_addr, true);
    try s1024.setVoxelAtGlobal(far_addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const result = computePenetrationBox(&world, 1.8, 0.2, 0.2, 2.2, 0.8, 0.8, .{});
    try std.testing.expect(result.overlapping);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), result.depth, 0.0001);
    try std.testing.expect(result.dir_x == 1);
    try std.testing.expect(result.manifold_point_count > 0);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), result.manifold_points[0].x, 0.0001);
}

test "computePenetrationAABB aggregates manifold across coplanar support voxels" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 0,
        .lz = 0,
    }), true);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 1,
        .ly = 1,
        .lz = 0,
    }), true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const result = computePenetrationBox(&world, 1.8, 0.2, 0.2, 2.2, 1.8, 0.8, .{});
    try std.testing.expect(result.overlapping);
    try std.testing.expect(result.dir_x == 1);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), result.manifold_points[0].x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2), result.manifold_points[0].y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.8), result.manifold_points[2].y, 0.0001);
}

/// Compute penetration of capsule into world
pub fn computePenetrationCapsule(world: *const QueryWorldView, center_x: f32, center_y: f32, center_z: f32, radius: f32, half_height: f32, filter: QueryFilter) PenetrationResult {
    var result: PenetrationResult = .{};
    const seg_min_y = center_y - half_height;
    const seg_max_y = center_y + half_height;
    const radius_sq = radius * radius;
    const range = computeCapsuleVoxelRange(center_x, center_y, center_z, radius, half_height);
    var cache = query_world.QueryAnyVoxelCache.init(filter);

    var gx = range.start_x;
    while (gx <= range.end_x) : (gx += 1) {
        var gy = range.start_y;
        while (gy <= range.end_y) : (gy += 1) {
            var gz = range.start_z;
            while (gz <= range.end_z) : (gz += 1) {
                const hit = query_world.queryAnyVoxelCached(world, &cache, gx, gy, gz);
                if (!hit.hit) continue;

                const box_min_x = @as(f32, @floatFromInt(gx));
                const box_min_y = @as(f32, @floatFromInt(gy));
                const box_min_z = @as(f32, @floatFromInt(gz));
                const box_max_x = box_min_x + 1.0;
                const box_max_y = box_min_y + 1.0;
                const box_max_z = box_min_z + 1.0;

                const closest = computeCapsuleBoxClosestPoint(center_x, center_y, center_z, seg_min_y, seg_max_y, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                if (closest.dist_sq > radius_sq) continue;

                if (closest.dist_sq > 0.000001) {
                    const dist = @sqrt(closest.dist_sq);
                    const depth = radius - dist;
                    updateBestCapsulePenetration(&result, depth, closest.dir_x / dist, closest.dir_y / dist, closest.dir_z / dist, hit);
                } else {
                    const interior_dir = resolveInteriorCapsuleDirection(center_x, closest.seg_y, center_z, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
                    updateBestCapsulePenetration(&result, radius, interior_dir.x, interior_dir.y, interior_dir.z, hit);
                }
            }
        }
    }

    return result;
}

test "computePenetrationCapsule detects side overlap against voxel" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 2,
        .ly = 2,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const result = computePenetrationCapsule(&world, 1.6, 2.5, 0.5, 0.7, 0.5, .{});
    try std.testing.expect(result.overlapping);
    try std.testing.expect(result.depth > 0);
    try std.testing.expect(result.dir_x < 0);
}

test "computePenetrationCapsule detects top-cap overlap" {
    const scene1024 = @import("scene1024.zig");
    const entity16 = @import("entity16.zig");
    const address = @import("address.zig");

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    const addr = address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 0,
        .ly = 4,
        .lz = 0,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var entities = [_]entity16.Entity16{};
    const world = QueryWorldView{
        .s1024 = &s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities[0..],
    };

    const result = computePenetrationCapsule(&world, 0.5, 3.2, 0.5, 0.8, 0.7, .{});
    try std.testing.expect(result.overlapping);
    try std.testing.expect(result.depth > 0);
    try std.testing.expect(result.dir_y < 0);
}
