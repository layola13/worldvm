//! Built-in scenarios
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");

pub const Scenario = enum {
    apple_table, hammer_glass, water_flow,
};

pub fn setupScenario(scenario: Scenario, scene: *scene32.Scene32, entities: []entity16.Entity16) void {
    scene.* = scene32.initScene();
    
    entities[0] = entity16.Prototypes.apple();
    entities[1] = entity16.Prototypes.table();
    entities[2] = entity16.Prototypes.hammer();
    entities[3] = entity16.Prototypes.glass();
    entities[4] = entity16.Prototypes.water();
    entities[5] = entity16.Prototypes.floor();
    
    const default_inst = scene32.Instance{
        .entity_id = 0, .pos_x = 0, .pos_y = 0, .pos_z = 0,
        .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0,
        .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 3
    };
    
    switch (scenario) {
        .apple_table => {
            var inst = default_inst;
            inst.entity_id = 0; inst.pos_x = 10; inst.pos_y = 28; inst.pos_z = 15;
            _ = scene32.addInstance(scene, inst);
            inst.entity_id = 1; inst.pos_x = 8; inst.pos_y = 20; inst.pos_z = 15;
            _ = scene32.addInstance(scene, inst);
            inst.entity_id = 5; inst.pos_x = 0; inst.pos_y = 0; inst.pos_z = 0; inst.state = .resting;
            _ = scene32.addInstance(scene, inst);
        },
        .hammer_glass => {
            var inst = default_inst;
            inst.entity_id = 3; inst.pos_x = 10; inst.pos_y = 15; inst.pos_z = 15;
            _ = scene32.addInstance(scene, inst);
            inst.entity_id = 2; inst.pos_x = 10; inst.pos_y = 25; inst.pos_z = 15;
            _ = scene32.addInstance(scene, inst);
            inst.entity_id = 5; inst.pos_x = 0; inst.pos_y = 0; inst.pos_z = 0; inst.state = .resting;
            _ = scene32.addInstance(scene, inst);
        },
        .water_flow => {
            var inst = default_inst;
            inst.entity_id = 4; inst.pos_x = 12; inst.pos_y = 20; inst.pos_z = 15;
            _ = scene32.addInstance(scene, inst);
            inst.entity_id = 5; inst.pos_x = 0; inst.pos_y = 0; inst.pos_z = 0; inst.state = .resting;
            _ = scene32.addInstance(scene, inst);
        },
    }
    scene32.rebuildOccupancy(scene, entities);
}
