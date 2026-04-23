//! Unit tests for Physics module

const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const physics = @import("physics.zig");

test "AABB intersection" {
    const a = physics.makeAABB(0, 0, 0, 16);
    const b = physics.makeAABB(10, 10, 10, 16);
    try std.testing.expect(physics.aabbHit(a, b));
    const c = physics.makeAABB(100, 100, 100, 16);
    try std.testing.expect(!physics.aabbHit(a, c));
}

test "Static entity does not fall" {
    var scene = scene32.initScene();
    var entities: [4]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.floor();
    const inst = scene32.Instance{ .entity_id = 0, .pos_x = 0, .pos_y = 0, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 3 };
    _ = scene32.addInstance(&scene, inst);
    scene32.rebuildOccupancy(&scene, &entities);
    const result = physics.checkFall(&scene, &scene.instances[0], &entities);
    try std.testing.expect(!result.can_fall);
}

test "Dynamic entity falls when unsupported" {
    var scene = scene32.initScene();
    var entities: [4]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();
    const inst = scene32.Instance{ .entity_id = 0, .pos_x = 0, .pos_y = 10, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 3 };
    _ = scene32.addInstance(&scene, inst);
    scene32.rebuildOccupancy(&scene, &entities);
    const result = physics.checkFall(&scene, &scene.instances[0], &entities);
    try std.testing.expect(result.can_fall);
    try std.testing.expect(result.target_y == 9);
}

test "Break check for fragile material" {
    const result = physics.checkBreak(100, .fragile, 100);
    try std.testing.expect(result.did_break);
    try std.testing.expect(result.fragments == 4);
}

test "Break check below threshold" {
    const result = physics.checkBreak(30, .fragile, 100);
    try std.testing.expect(!result.did_break);
}

test "Impact calculation" {
    const impact = physics.calcImpact(-100, 50);
    try std.testing.expect(impact == 50);
}

test "Non-liquid entity does not flow" {
var scene = scene32.initScene();
    var entities: [2]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.apple();
    const inst = scene32.Instance{ .entity_id = 0, .pos_x = 0, .pos_y = 10, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 3 };
    _ = scene32.addInstance(&scene, inst);
    const result = physics.checkFlow(&scene, &scene.instances[0], &entities);
    try std.testing.expect(!result.flowed);
    try std.testing.expect(result.dir == .hold);
}
