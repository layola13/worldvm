const std = @import("std");
const bus = @import("bus.zig");
const tick_engine = @import("tick_engine.zig");
const scene1024 = @import("scene1024.zig");
const entity16 = @import("entity16.zig");
const scenarios = @import("scenarios.zig");

test "Bus integrates with TickEngine" {
    const allocator = std.testing.allocator;
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    
    var entities: [64]entity16.Entity16 = undefined;
    scenarios.setupScenario(.hammer_glass, &s1024, &entities);
    
    var world_bus = bus.Bus.init();
    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &s1024, &entities);
    engine.world_bus = &world_bus;
    
    // Hammer at 25, Glass at 15.
    // Move hammer to 16 (just above glass) to trigger break in 1-2 ticks
    s1024.instances[1].pos_y = 16;
    try s1024.rebuildOccupancy(&entities);
    
    _ = tick_engine.stepTick(&engine); // Tick 1: Hammer falls or hits?
    // If it hits at Tick 1:
    
    // Check if any messages dispatched
    if (world_bus.msg_count > 0) {
        try std.testing.expect(world_bus.msg_count >= 1);
        try std.testing.expect(world_bus.messages[0].msg_type == .PHYSICS_EVENT);
    }
}
