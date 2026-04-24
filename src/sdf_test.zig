const std = @import("std");
const sdf = @import("sdf.zig");
const entity16 = @import("entity16.zig");

test "SDF Sphere voxelization" {
    const sphere_node = sdf.SDFNode{ .sphere = .{ .radius = 0.5 } };
    const e = entity16.Entity16.fromSDF(sphere_node);
    
    // Center (8,8,8) should be set
    try std.testing.expect(entity16.testVoxel(&e, 8, 8, 8));
    // Far corner (0,0,0) should be empty
    try std.testing.expect(!entity16.testVoxel(&e, 0, 0, 0));
}

test "SDF Box voxelization" {
    const box_node = sdf.SDFNode{ .box = .{ .size = sdf.Vec3.init(0.2, 0.2, 0.2) } };
    const e = entity16.Entity16.fromSDF(box_node);
    
    try std.testing.expect(entity16.testVoxel(&e, 8, 8, 8));
    // (8+3, 8, 8) is (11,8,8). 3/7.5 = 0.4 > 0.2, should be empty
    try std.testing.expect(!entity16.testVoxel(&e, 12, 8, 8));
}

test "SDF Union operation" {
    const s1 = sdf.SDFNode{ .sphere = .{ .radius = 0.2 } };
    const s2 = sdf.SDFNode{ .sphere = .{ .radius = 0.2 } };
    // Move s2 conceptually (offset math is currently handled by evaluate if we had translate)
    // For now just test the operator logic
    const union_node = sdf.SDFNode{ .union_op = .{ .left = &s1, .right = &s2 } };
    const e = entity16.Entity16.fromSDF(union_node);
    
    try std.testing.expect(entity16.testVoxel(&e, 8, 8, 8));
}
