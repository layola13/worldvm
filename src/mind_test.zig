const std = @import("std");
const mind = @import("mind.zig");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const scenarios = @import("scenarios.zig");

test "AffectBlock initialization" {
    const affect = mind.AffectBlock.init(10, 200, 100, 50);
    try std.testing.expect(affect.valence == 10);
    try std.testing.expect(affect.arousal == 200);
}

test "ShadowSandbox isolation" {
    const allocator = std.testing.allocator;
    var base_s1024 = scene1024.Scene1024.init(allocator);
    defer base_s1024.deinit();
    
    var entities: [64]entity16.Entity16 = undefined;
    _ = try base_s1024.getPage(0);
    scenarios.setupScenario(.apple_table, &base_s1024, &entities);
    
    const initial_y = base_s1024.instances[0].pos_y;
    
    // Create shadow sandbox
    var sandbox = try mind.ShadowSandbox.init(allocator, &base_s1024, &entities);
    defer sandbox.deinit();
    
    // Simulate in shadow world
    _ = sandbox.simulate(10);
    
    // Check that base world is untouched
    try std.testing.expect(base_s1024.instances[0].pos_y == initial_y);
}

test "AffectSystem update from bus" {
    var affect_sys = mind.AffectSystem.init();
    var tri_bus = mind.TriWorldBus.init();
    
    // Dispatch a break event
    tri_bus.onCollision(1, 10, -100, 30, true);
    
    affect_sys.update(&tri_bus);
    
    // Should have negative valence and positive arousal
    try std.testing.expect(affect_sys.registers.valence < 0);
    try std.testing.expect(affect_sys.registers.arousal > 0);
    try std.testing.expect(affect_sys.getPriorityMod() > 0);
}
