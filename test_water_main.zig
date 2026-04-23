const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const physics = @import("physics.zig");
const scenarios = @import("scenarios.zig");

pub fn main() void {
    var scene = scene32.initScene();
    var entities: [64]entity16.Entity16 = undefined;
    scenarios.setupScenario(.water_flow, &scene, &entities);
    
    std.debug.print("Instance 0: entity_id={}, material={}\n", .{scene.instances[0].entity_id, entities[scene.instances[0].entity_id].physics.material});
    
    const result = physics.checkFlow(&scene, &scene.instances[0], &entities);
    std.debug.print("checkFlow: flowed={}, dir={}\n", .{result.flowed, {@tagName(result.dir)}});
}
