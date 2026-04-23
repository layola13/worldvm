//! Scene32: 32x32x32 voxel scene sandbox

const std = @import("std");
const entity16 = @import("entity16.zig");

pub const SCENE_DIM: usize = 32;
pub const SCENE_VOXELS: usize = SCENE_DIM * SCENE_DIM * SCENE_DIM;
pub const OCCUPANCY_WORDS: usize = SCENE_VOXELS / 64;
pub const MAX_INSTANCES: usize = 128;

pub const InstanceState = enum(u8) {
    idle = 0, moving = 1, falling = 2, resting = 3, broken = 4, flowing = 5,
};

pub const Instance = struct {
    entity_id: u16,
    pos_x: i8,
    pos_y: i8,
    pos_z: i8,
    rot_yaw: u8,
    rot_pitch: u8,
    rot_roll: u8,
    state: InstanceState,
    sleep_tick: u8,
    _reserved: [3]u8,
};

pub const Scene32 = struct {
    occupancy: [OCCUPANCY_WORDS]u64,
    instances: [MAX_INSTANCES]Instance,
    instance_count: u8,
    focus_x: u8,
    focus_y: u8,
    focus_z: u8,
    focus_radius: u8,
    tick: u32,
    tick_rate: u8,
    state_flags: u8,
    scene_id: u32,
    parent_event_id: u32,
    _reserved: [27]u8,
};

pub fn initScene() Scene32 {
    var scene: Scene32 = undefined;
    scene.occupancy = .{0} ** OCCUPANCY_WORDS;
    @memset(&scene.instances, std.mem.zeroes(Instance));
    scene.instance_count = 0;
    scene.focus_x = 15;
    scene.focus_y = 15;
    scene.focus_z = 15;
    scene.focus_radius = 8;
    scene.tick = 0;
    scene.tick_rate = 1;
    scene.state_flags = 0;
    scene.scene_id = 0;
    scene.parent_event_id = 0;
    scene._reserved = .{0} ** 27;
    return scene;
}

pub fn sceneCoordToBitIndex(x: u8, y: u8, z: u8) u15 {
    return @truncate(@as(u15, z) + @as(u15, x) * SCENE_DIM + @as(u15, y) * SCENE_DIM * SCENE_DIM);
}

pub fn inBounds(x: i8, y: i8, z: i8) bool {
    return x >= 0 and x < SCENE_DIM and y >= 0 and y < SCENE_DIM and z >= 0 and z < SCENE_DIM;
}

pub fn setOccupied(scene: *Scene32, x: i8, y: i8, z: i8) void {
    if (!inBounds(x, y, z)) return;
    const idx = sceneCoordToBitIndex(@intCast(x), @intCast(y), @intCast(z));
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx & 0x3F));
    scene.occupancy[word] |= @as(u64, 1) << bit;
}

pub fn clearOccupied(scene: *Scene32, x: i8, y: i8, z: i8) void {
    if (!inBounds(x, y, z)) return;
    const idx = sceneCoordToBitIndex(@intCast(x), @intCast(y), @intCast(z));
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx & 0x3F));
    scene.occupancy[word] &= ~(@as(u64, 1) << bit);
}

pub fn isOccupied(scene: *const Scene32, x: i8, y: i8, z: i8) bool {
    if (!inBounds(x, y, z)) return true;
    const idx = sceneCoordToBitIndex(@intCast(x), @intCast(y), @intCast(z));
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx & 0x3F));
    return (scene.occupancy[word] & (@as(u64, 1) << bit)) != 0;
}

pub fn addInstance(scene: *Scene32, instance: Instance) ?u8 {
    if (scene.instance_count >= MAX_INSTANCES) return null;
    const idx = scene.instance_count;
    scene.instances[idx] = instance;
    scene.instance_count += 1;
    return idx;
}

pub fn projectEntityToScene(scene: *Scene32, entity: *const entity16.Entity16, pos_x: i8, pos_y: i8, pos_z: i8) void {
    var ex: usize = 0;
    while (ex < entity16.ENTITY_DIM) : (ex += 1) {
        var ey: usize = 0;
        while (ey < entity16.ENTITY_DIM) : (ey += 1) {
            var ez: usize = 0;
            while (ez < entity16.ENTITY_DIM) : (ez += 1) {
                if (entity16.testVoxel(entity, @truncate(ex), @truncate(ey), @truncate(ez))) {
                    const sx: i8 = @as(i8, @intCast(ex)) + pos_x;
                    const sy: i8 = @as(i8, @intCast(ey)) + pos_y;
                    const sz: i8 = @as(i8, @intCast(ez)) + pos_z;
                    if (inBounds(sx, sy, sz)) {
                        setOccupied(scene, @intCast(sx), @intCast(sy), @intCast(sz));
                    }
                }
            }
        }
    }
}

pub fn rebuildOccupancy(scene: *Scene32, entities: []const entity16.Entity16) void {
    var i: usize = 0;
    while (i < OCCUPANCY_WORDS) : (i += 1) scene.occupancy[i] = 0;
    i = 0;
    while (i < scene.instance_count) : (i += 1) {
        const inst = scene.instances[i];
        if (inst.entity_id < entities.len) {
            projectEntityToScene(scene, &entities[inst.entity_id], inst.pos_x, inst.pos_y, inst.pos_z);
        }
    }
}

pub fn clearScene(scene: *Scene32) void {
    var i: usize = 0;
    while (i < OCCUPANCY_WORDS) : (i += 1) scene.occupancy[i] = 0;
    i = 0;
    while (i < MAX_INSTANCES) : (i += 1) scene.instances[i] = .{ .entity_id = 0, .pos_x = 0, .pos_y = 0, .pos_z = 0, .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0, .state = .idle, .sleep_tick = 0, ._reserved = .{0} ** 3 };
    scene.instance_count = 0;
    scene.tick = 0;
    scene.state_flags = 0;
}

pub fn isInFocus(scene: *const Scene32, x: i8, y: i8, z: i8) bool {
    const dx: i16 = @as(i16, x) - @as(i16, scene.focus_x);
    const dy: i16 = @as(i16, y) - @as(i16, scene.focus_y);
    const dz: i16 = @as(i16, z) - @as(i16, scene.focus_z);
    const dist2 = dx*dx + dy*dy + dz*dz;
    return dist2 <= @as(i16, scene.focus_radius) * @as(i16, scene.focus_radius);
}
