//! Scene32: 32x32x32 voxel scene sandbox

const std = @import("std");
const entity16 = @import("entity16.zig");

pub const SCENE_DIM: usize = 32;
pub const SCENE_VOXELS: usize = SCENE_DIM * SCENE_DIM * SCENE_DIM;
pub const OCCUPANCY_WORDS: usize = SCENE_VOXELS / 64;
pub const MAX_INSTANCES: usize = 128;

pub const InstanceState = enum(u8) {
    idle = 0,
    moving = 1,
    falling = 2,
    resting = 3,
    broken = 4,
    flowing = 5,
};

pub const Instance = struct {
    entity_id: u16,
    pos_x: i32,
    pos_y: i32,
    pos_z: i32,
    rot_yaw: u8,
    rot_pitch: u8,
    rot_roll: u8,
    state: InstanceState,
    sleep_tick: u8,
    vel_x: i16 = 0,
    vel_y: i16 = 0,
    vel_z: i16 = 0,
    ang_x: i8 = 0,
    ang_y: i8 = 0,
    ang_z: i8 = 0,
    _reserved: [2]u8,
};

pub const Scene32 = struct {
    occupancy: [OCCUPANCY_WORDS]u64,
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

pub fn inBounds(x: i32, y: i32, z: i32) bool {
    return x >= 0 and x < SCENE_DIM and y >= 0 and y < SCENE_DIM and z >= 0 and z < SCENE_DIM;
}

pub fn setOccupied(scene: *Scene32, x: i32, y: i32, z: i32) void {
    if (!inBounds(x, y, z)) return;
    const idx = sceneCoordToBitIndex(@intCast(x), @intCast(y), @intCast(z));
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx & 0x3F));
    scene.occupancy[word] |= @as(u64, 1) << bit;
}

pub fn clearOccupied(scene: *Scene32, x: i32, y: i32, z: i32) void {
    if (!inBounds(x, y, z)) return;
    const idx = sceneCoordToBitIndex(@intCast(x), @intCast(y), @intCast(z));
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx & 0x3F));
    scene.occupancy[word] &= ~(@as(u64, 1) << bit);
}

pub fn isOccupied(scene: *const Scene32, x: i32, y: i32, z: i32) bool {
    if (!inBounds(x, y, z)) return true;
    const idx = sceneCoordToBitIndex(@intCast(x), @intCast(y), @intCast(z));
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx & 0x3F));
    return (scene.occupancy[word] & (@as(u64, 1) << bit)) != 0;
}

pub fn clearScene(scene: *Scene32) void {
    var i: usize = 0;
    while (i < OCCUPANCY_WORDS) : (i += 1) scene.occupancy[i] = 0;
    scene.tick = 0;
    scene.state_flags = 0;
}

pub fn isInFocus(scene: *const Scene32, x: i32, y: i32, z: i32) bool {
    const dx: i32 = x - @as(i32, @intCast(scene.focus_x));
    const dy: i32 = y - @as(i32, @intCast(scene.focus_y));
    const dz: i32 = z - @as(i32, @intCast(scene.focus_z));
    const dist2 = dx * dx + dy * dy + dz * dz;
    return dist2 <= @as(i32, @intCast(scene.focus_radius)) * @as(i32, @intCast(scene.focus_radius));
}

test "Scene32 occupancy set clear and out-of-bounds semantics" {
    var scene = initScene();

    try std.testing.expect(!isOccupied(&scene, 1, 2, 3));
    setOccupied(&scene, 1, 2, 3);
    try std.testing.expect(isOccupied(&scene, 1, 2, 3));

    setOccupied(&scene, -1, 2, 3);
    setOccupied(&scene, 32, 2, 3);
    try std.testing.expect(isOccupied(&scene, -1, 2, 3));
    try std.testing.expect(isOccupied(&scene, 32, 2, 3));
    try std.testing.expect(isOccupied(&scene, 1, 2, 3));

    clearOccupied(&scene, 1, 2, 3);
    try std.testing.expect(!isOccupied(&scene, 1, 2, 3));

    setOccupied(&scene, 31, 31, 31);
    clearScene(&scene);
    try std.testing.expect(!isOccupied(&scene, 31, 31, 31));
    try std.testing.expectEqual(@as(u32, 0), scene.tick);
    try std.testing.expectEqual(@as(u8, 0), scene.state_flags);
}

test "Scene32 focus uses spherical radius around focus point" {
    var scene = initScene();
    scene.focus_x = 10;
    scene.focus_y = 10;
    scene.focus_z = 10;
    scene.focus_radius = 5;

    try std.testing.expect(isInFocus(&scene, 10, 10, 10));
    try std.testing.expect(isInFocus(&scene, 15, 10, 10));
    try std.testing.expect(isInFocus(&scene, 13, 14, 10));
    try std.testing.expect(!isInFocus(&scene, 16, 10, 10));
    try std.testing.expect(!isInFocus(&scene, 14, 14, 14));
}
