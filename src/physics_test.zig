//! Unit tests for Physics module
const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const physics = @import("physics.zig");
const tick_engine = @import("tick_engine.zig");
const scenarios = @import("scenarios.zig");

test "Static entity does not fall" {
    const allocator = std.testing.allocator;
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities: [4]entity16.Entity16 = undefined;
    entities[0] = entity16.Prototypes.floor();
    const inst = scene32.Instance{ .entity_id = 0, .pos_x = 0, .pos_y = 0, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 3 };
    _ = try s1024.addInstance(inst);
    try s1024.rebuildOccupancy(&entities);
    
    const result = physics.checkFall(&s1024, &s1024.instances[0], &entities);
    try std.testing.expect(!result.can_fall);
}

test "water_flow integration 1024" {
    const allocator = std.testing.allocator;
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    
    var entities: [64]entity16.Entity16 = undefined;
    scenarios.setupScenario(.water_flow, &s1024, &entities);
    
    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &s1024, &entities);
    
    _ = tick_engine.runTicks(&engine, 50);
    
    try std.testing.expect(s1024.instances[0].pos_y == 0);
    try std.testing.expect(s1024.instances[0].state == .flowing);
}
