//! Built-in scenarios
const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");

pub const Scenario = enum {
    apple_table,
    hammer_glass,
    water_flow,
    bounce_test,
    domino_chain,
    pyramid_collapse,
    multi_stack,
    gas_expand,
};

pub fn setupScenario(scenario: Scenario, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) void {
    // Base prototypes
    entities[0] = entity16.Prototypes.apple();
    entities[1] = entity16.Prototypes.table();
    entities[2] = entity16.Prototypes.hammer();
    entities[3] = entity16.Prototypes.glass();
    entities[4] = entity16.Prototypes.water();
    entities[5] = entity16.Prototypes.floor();
    // Physics test prototypes
    entities[6] = entity16.Prototypes.ball();
    entities[7] = entity16.Prototypes.brick();
    entities[8] = entity16.Prototypes.domino();
    entities[9] = entity16.Prototypes.plate();

    const default_inst = scene32.Instance{ .entity_id = 0, .pos_x = 0, .pos_y = 0, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 2 };

    switch (scenario) {
        .apple_table => {
            var inst = default_inst;
            inst.entity_id = 0;
            inst.pos_x = 10;
            inst.pos_y = 28;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 1;
            inst.pos_x = 8;
            inst.pos_y = 20;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
        .hammer_glass => {
            var inst = default_inst;
            inst.entity_id = 3;
            inst.pos_x = 10;
            inst.pos_y = 15;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 2;
            inst.pos_x = 10;
            inst.pos_y = 25;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
        .water_flow => {
            var inst = default_inst;
            inst.entity_id = 4;
            inst.pos_x = 12;
            inst.pos_y = 20;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
        .bounce_test => {
            // Ball bouncing on plate - tests elastic collision
            var inst = default_inst;
            inst.entity_id = 6;
            inst.pos_x = 10;
            inst.pos_y = 20;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 9;
            inst.pos_x = 5;
            inst.pos_y = 10;
            inst.pos_z = 5;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
        .domino_chain => {
            // 5 dominoes in a row - tests sequential collision
            var inst = default_inst;
            inst.entity_id = 8;
            inst.pos_x = 5;
            inst.pos_y = 5;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 8;
            inst.pos_x = 8;
            inst.pos_y = 5;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 8;
            inst.pos_x = 11;
            inst.pos_y = 5;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 8;
            inst.pos_x = 14;
            inst.pos_y = 5;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 8;
            inst.pos_x = 17;
            inst.pos_y = 5;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 6;
            inst.pos_x = 2;
            inst.pos_y = 5;
            inst.pos_z = 15; // Ball to push first domino
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
        .pyramid_collapse => {
            // Pyramid of 6 bricks - tests stacked physics
            var inst = default_inst;
            // Bottom row (3 bricks)
            inst.entity_id = 7;
            inst.pos_x = 5;
            inst.pos_y = 1;
            inst.pos_z = 12;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 10;
            inst.pos_y = 1;
            inst.pos_z = 12;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 15;
            inst.pos_y = 1;
            inst.pos_z = 12;
            _ = s1024.addInstance(inst) catch {};
            // Middle row (2 bricks)
            inst.entity_id = 7;
            inst.pos_x = 7;
            inst.pos_y = 8;
            inst.pos_z = 12;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 12;
            inst.pos_y = 8;
            inst.pos_z = 12;
            _ = s1024.addInstance(inst) catch {};
            // Top row (1 brick)
            inst.entity_id = 7;
            inst.pos_x = 10;
            inst.pos_y = 15;
            inst.pos_z = 12;
            _ = s1024.addInstance(inst) catch {};
            // Ball to topple pyramid
            inst.entity_id = 6;
            inst.pos_x = 3;
            inst.pos_y = 1;
            inst.pos_z = 12;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
        .multi_stack => {
            // Multiple independent stacks - tests parallel stability
            var inst = default_inst;
            // Stack 1
            inst.entity_id = 7;
            inst.pos_x = 5;
            inst.pos_y = 1;
            inst.pos_z = 5;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 5;
            inst.pos_y = 8;
            inst.pos_z = 5;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 5;
            inst.pos_y = 15;
            inst.pos_z = 5;
            _ = s1024.addInstance(inst) catch {};
            // Stack 2
            inst.entity_id = 7;
            inst.pos_x = 15;
            inst.pos_y = 1;
            inst.pos_z = 5;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 15;
            inst.pos_y = 8;
            inst.pos_z = 5;
            _ = s1024.addInstance(inst) catch {};
            // Stack 3
            inst.entity_id = 7;
            inst.pos_x = 10;
            inst.pos_y = 1;
            inst.pos_z = 20;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 10;
            inst.pos_y = 8;
            inst.pos_z = 20;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 7;
            inst.pos_x = 10;
            inst.pos_y = 15;
            inst.pos_z = 20;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 6;
            inst.pos_x = 3;
            inst.pos_y = 1;
            inst.pos_z = 5; // Ball to topple stack 1
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
        .gas_expand => {
            // Liquid spreading - tests flow behavior
            var inst = default_inst;
            inst.entity_id = 4;
            inst.pos_x = 10;
            inst.pos_y = 20;
            inst.pos_z = 15;
            _ = s1024.addInstance(inst) catch {};
            inst.entity_id = 5;
            inst.pos_x = 0;
            inst.pos_y = 0;
            inst.pos_z = 0;
            inst.state = .resting;
            _ = s1024.addInstance(inst) catch {};
        },
    }
    s1024.rebuildOccupancy(entities) catch {};
}

fn findInstance(s1024: *const scene1024.Scene1024, entity_id: u16) ?scene32.Instance {
    var i: u8 = 0;
    while (i < s1024.instance_count) : (i += 1) {
        if (s1024.instances[i].entity_id == entity_id) return s1024.instances[i];
    }
    return null;
}

test "setupScenario initializes shared prototypes" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities: [10]entity16.Entity16 = undefined;

    setupScenario(.apple_table, &s1024, &entities);

    try std.testing.expect(entity16.countVoxels(&entities[0]) > 0);
    try std.testing.expect(entity16.countVoxels(&entities[1]) > 0);
    try std.testing.expectEqual(entity16.MaterialType.liquid, entities[4].physics.material);
    try std.testing.expectEqual(@as(u16, 0), entities[5].physics.mass);
    try std.testing.expectEqual(entity16.MaterialType.elastic, entities[6].physics.material);
}

test "setupScenario apple_table places apple table and resting floor" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities: [10]entity16.Entity16 = undefined;

    setupScenario(.apple_table, &s1024, &entities);

    try std.testing.expectEqual(@as(u8, 3), s1024.instance_count);
    const apple = findInstance(&s1024, 0).?;
    const table = findInstance(&s1024, 1).?;
    const floor = findInstance(&s1024, 5).?;
    try std.testing.expectEqual(@as(i32, 10), apple.pos_x);
    try std.testing.expectEqual(@as(i32, 28), apple.pos_y);
    try std.testing.expectEqual(@as(i32, 8), table.pos_x);
    try std.testing.expectEqual(scene32.InstanceState.resting, floor.state);
}

test "setupScenario domino_chain has five dominoes plus pusher and floor" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    var entities: [10]entity16.Entity16 = undefined;

    setupScenario(.domino_chain, &s1024, &entities);

    try std.testing.expectEqual(@as(u8, 7), s1024.instance_count);
    var domino_count: u8 = 0;
    var i: u8 = 0;
    while (i < s1024.instance_count) : (i += 1) {
        if (s1024.instances[i].entity_id == 8) domino_count += 1;
    }
    try std.testing.expectEqual(@as(u8, 5), domino_count);
    try std.testing.expect(findInstance(&s1024, 6) != null);
    try std.testing.expect(findInstance(&s1024, 5) != null);
}

test "setupScenario pyramid and multi_stack stay within Scene1024 instance capacity" {
    var entities: [10]entity16.Entity16 = undefined;

    var pyramid = scene1024.Scene1024.init(std.testing.allocator);
    defer pyramid.deinit();
    setupScenario(.pyramid_collapse, &pyramid, &entities);
    try std.testing.expectEqual(@as(u8, 8), pyramid.instance_count);

    var stacks = scene1024.Scene1024.init(std.testing.allocator);
    defer stacks.deinit();
    setupScenario(.multi_stack, &stacks, &entities);
    try std.testing.expectEqual(@as(u8, 10), stacks.instance_count);
}

test "setupScenario water and gas scenarios use water plus resting floor" {
    var entities: [10]entity16.Entity16 = undefined;

    var water = scene1024.Scene1024.init(std.testing.allocator);
    defer water.deinit();
    setupScenario(.water_flow, &water, &entities);
    try std.testing.expectEqual(@as(u8, 2), water.instance_count);
    try std.testing.expect(findInstance(&water, 4) != null);
    try std.testing.expectEqual(scene32.InstanceState.resting, findInstance(&water, 5).?.state);

    var gas = scene1024.Scene1024.init(std.testing.allocator);
    defer gas.deinit();
    setupScenario(.gas_expand, &gas, &entities);
    try std.testing.expectEqual(@as(u8, 2), gas.instance_count);
    try std.testing.expect(findInstance(&gas, 4) != null);
    try std.testing.expectEqual(scene32.InstanceState.resting, findInstance(&gas, 5).?.state);
}
