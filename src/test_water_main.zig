const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const physics = @import("physics.zig");
const scenarios = @import("scenarios.zig");

pub fn main() void {
    var scene = scene32.initScene();
    var entities: [64]entity16.Entity16 = undefined;
    scenarios.setupScenario(.water_flow, &scene, &entities);
    scene32.rebuildOccupancy(&scene, &entities);
    
    const inst = scene.instances[0];
    std.debug.print("Water at pos=({},{},{})\n", .{inst.pos_x, inst.pos_y, inst.pos_z});
    std.debug.print("Floor at pos=({},{},{})\n", .{scene.instances[1].pos_x, scene.instances[1].pos_y, scene.instances[1].pos_z});
    
    // Check scene occupancy at various y levels
    std.debug.print("\nOccupancy at y=20: ", .{});
    for (0..32) |x| {
        if (scene32.isOccupied(&scene, @intCast(x), 20, 15)) std.debug.print("#", .{}) else std.debug.print(".", .{});
    }
    std.debug.print("\n", .{});
    
    std.debug.print("Occupancy at y=19: ", .{});
    for (0..32) |x| {
        if (scene32.isOccupied(&scene, @intCast(x), 19, 15)) std.debug.print("#", .{}) else std.debug.print(".", .{});
    }
    std.debug.print("\n", .{});
    
    const result = physics.checkFlow(&scene, &inst, &entities);
    std.debug.print("\ncheckFlow: flowed={}\n", .{result.flowed});
}
