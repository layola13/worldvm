//! ASCII Renderer for debugging

const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");

pub const CHAR_EMPTY = '.';
pub const CHAR_SOLID = '#';

pub fn renderScene(scene: *scene32.Scene32, writer: anytype) !void {
    try writer.print("=== Scene (Tick {}) ===\n", .{scene.tick});
    try writer.print("TOP VIEW:\n", .{});
    try renderTopView(scene, writer);
}

fn renderTopView(scene: *scene32.Scene32, writer: anytype) !void {
    const dim: usize = 32;
    const z: u8 = 15;

    var y: usize = 0;
    while (y < dim) : (y += 1) {
        const y_r: i8 = @as(i8, @intCast(dim - 1 - y));
        try writer.print("{: >2} |", .{y_r});
        var x: i32 = 0;
        while (x < dim) : (x += 1) {
            if (scene32.isOccupied(scene, x, y_r, z)) {
                try writer.print("#", .{});
            } else {
                try writer.print(".", .{});
            }
        }
        try writer.print("|\n", .{});
    }
}

pub fn renderInstances(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, writer: anytype) !void {
    try writer.print("\nInstances:\n", .{});

    var i: u8 = 0;
    while (i < s1024.instance_count) : (i += 1) {
        const inst = s1024.instances[i];
        if (inst.entity_id < entities.len) {
            try writer.print("{}: pos=({},{},{}) state={s}\n", .{ i, inst.pos_x, inst.pos_y, inst.pos_z, @tagName(inst.state) });
        }
    }
}

test "renderScene emits header and occupied top-view voxel" {
    var scene = scene32.initScene();
    scene.tick = 42;
    scene32.setOccupied(&scene, 3, 4, 15);

    var buffer: [2048]u8 = undefined;
    var stream = std.io.fixedBufferStream(&buffer);
    try renderScene(&scene, stream.writer());
    const output = stream.getWritten();

    try std.testing.expect(std.mem.indexOf(u8, output, "=== Scene (Tick 42) ===") != null);
    try std.testing.expect(std.mem.indexOf(u8, output, "TOP VIEW:") != null);
    try std.testing.expect(std.mem.indexOf(u8, output, "#") != null);
}

test "renderInstances lists valid entity instances only" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var entities = [_]entity16.Entity16{entity16.initEntity16()};
    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 0;
    inst.pos_x = 1;
    inst.pos_y = 2;
    inst.pos_z = 3;
    inst.state = .moving;
    _ = try s1024.addInstance(inst);
    inst.entity_id = 99;
    _ = try s1024.addInstance(inst);

    var buffer: [256]u8 = undefined;
    var stream = std.io.fixedBufferStream(&buffer);
    try renderInstances(&s1024, &entities, stream.writer());
    const output = stream.getWritten();

    try std.testing.expect(std.mem.indexOf(u8, output, "Instances:") != null);
    try std.testing.expect(std.mem.indexOf(u8, output, "0: pos=(1,2,3) state=moving") != null);
    try std.testing.expect(std.mem.indexOf(u8, output, "1: pos=") == null);
}
