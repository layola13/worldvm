//! Entity16: 4KB fixed-size entity
const std = @import("std");

pub const ENTITY_SIZE: usize = 4096;
pub const TOPOLOGY_WORDS: usize = 64;
pub const ENTITY_DIM: usize = 16;

pub const MaterialType = enum(u8) {
    solid = 0, liquid = 1, gas = 2, fragile = 3, elastic = 4, composite = 5,
};

pub const PhysicsBlock = struct {
    mass: u16 = 0, hardness: u16 = 100, material: MaterialType = .solid,
    friction: u8 = 128, restitution: u8 = 64, conductivity: u8 = 0,
    flags: u8 = 0, temp_state: u8 = 0, stability: u16 = 100,
};

pub const VisualBlock = struct {
    color_pack: u8 = 0, color_alpha: u8 = 255, reflection: u8 = 0, glow: u8 = 0,
};

pub const EdgeSlot = struct {
    target_id: u16 = 0, relation_type: u8 = 0, weight: u8 = 0,
};

pub const Entity16 = struct {
    topology: [TOPOLOGY_WORDS]u64 = undefined,
    physics: PhysicsBlock = .{},
    visual: VisualBlock = .{},
    relations: [64]EdgeSlot = undefined,
    reserved: [4096 - 512 - 16 - 4 - 256]u8 = undefined,
};

pub fn coordToBitIndex(x: u8, y: u8, z: u8) u12 {
    return @truncate(z + x * ENTITY_DIM + y * ENTITY_DIM * ENTITY_DIM);
}

pub fn setVoxel(entity: *Entity16, x: u8, y: u8, z: u8) void {
    const idx = coordToBitIndex(x, y, z);
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx));
    entity.topology[word] |= @as(u64, 1) << @as(u6, bit);
}

pub fn clearVoxel(entity: *Entity16, x: u8, y: u8, z: u8) void {
    const idx = coordToBitIndex(x, y, z);
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx));
    entity.topology[word] &= ~(@as(u64, 1) << @as(u6, bit));
}

pub fn testVoxel(entity: *const Entity16, x: u8, y: u8, z: u8) bool {
    const idx = coordToBitIndex(x, y, z);
    const word = idx >> 6;
    const bit = @as(u6, @truncate(idx));
    return (entity.topology[word] & (@as(u64, 1) << @as(u6, bit))) != 0;
}

pub fn countVoxels(entity: *const Entity16) u12 {
    var count: u12 = 0;
    for (0..TOPOLOGY_WORDS) |i| count += @popCount(entity.topology[i]);
    return count;
}

pub fn fillBox(entity: *Entity16, x1: u8, y1: u8, z1: u8, x2: u8, y2: u8, z2: u8) void {
    const min_x = @min(x1, x2); const max_x = @max(x1, x2);
    const min_y = @min(y1, y2); const max_y = @max(y1, y2);
    const min_z = @min(z1, z2); const max_z = @max(z1, z2);
    var x: i32 = min_x; while (x <= max_x) : (x += 1) {
        var y: i32 = min_y; while (y <= max_y) : (y += 1) {
            var z: i32 = min_z; while (z <= max_z) : (z += 1) {
                setVoxel(entity, @intCast(x), @intCast(y), @intCast(z));
            }
        }
    }
}


pub fn fillSphere(entity: *Entity16, cx: u8, cy: u8, cz: u8, radius: u8) void {
    const r2 = radius * radius;
    var x: i32 = 0; while (x < @as(i32, ENTITY_DIM)) : (x += 1) {
        var y: i32 = 0; while (y < @as(i32, ENTITY_DIM)) : (y += 1) {
            var z: i32 = 0; while (z < @as(i32, ENTITY_DIM)) : (z += 1) {
                const dx: i32 = x - @as(i32, cx);
                const dy: i32 = y - @as(i32, cy);
                const dz: i32 = z - @as(i32, cz);
                if (dx*dx + dy*dy + dz*dz <= r2) setVoxel(entity, @intCast(x), @intCast(y), @intCast(z));
            }
        }
    }
}

pub fn fillHollowBox(entity: *Entity16, x1: u8, y1: u8, z1: u8, x2: u8, y2: u8, z2: u8) void {
    fillBox(entity, x1, y1, z1, x2, y2, z1);
    fillBox(entity, x1, y1, z2, x2, y2, z2);
    fillBox(entity, x1, y1, z1, x1, y2, z2);
    fillBox(entity, x2, y1, z1, x2, y2, z2);
    fillBox(entity, x1, y1, z1, x2, y1, z2);
    fillBox(entity, x1, y2, z1, x2, y2, z2);
}

pub const Prototypes = struct {
    pub fn apple() Entity16 {
        var e: Entity16 = .{}; e.topology = .{0} ** 64; e.physics.mass = 50; e.physics.material = .solid;
        fillSphere(&e, 8, 8, 8, 5); return e;
    }
    pub fn table() Entity16 {
        var e: Entity16 = .{}; e.topology = .{0} ** 64; e.physics.mass = 500; e.physics.material = .solid; e.physics.flags |= 0x01;
        fillBox(&e, 0, 0, 0, 15, 1, 15); fillBox(&e, 1, 1, 1, 3, 7, 3); fillBox(&e, 12, 1, 1, 14, 7, 3);
        fillBox(&e, 1, 1, 12, 3, 7, 14); fillBox(&e, 12, 1, 12, 14, 7, 14); return e;
    }
    pub fn hammer() Entity16 {
        var e: Entity16 = .{}; e.topology = .{0} ** 64; e.physics.mass = 200; e.physics.material = .solid;
        fillBox(&e, 5, 8, 6, 10, 14, 9); fillBox(&e, 3, 10, 4, 12, 12, 11); return e;
    }
    pub fn glass() Entity16 {
        var e: Entity16 = .{}; e.topology = .{0} ** 64; e.physics.mass = 30; e.physics.material = .fragile; e.physics.flags |= 0x04;
        fillHollowBox(&e, 2, 0, 2, 13, 12, 13); return e;
    }
    pub fn water() Entity16 {
        var e: Entity16 = .{}; e.topology = .{0} ** 64; e.physics.mass = 10; e.physics.material = .liquid;
        fillBox(&e, 0, 0, 0, 7, 7, 7); return e;
    }
    pub fn floor() Entity16 {
        var e: Entity16 = .{}; e.topology = .{0} ** 64; e.physics.mass = 0; e.physics.material = .solid; e.physics.flags |= 0x01;
        fillBox(&e, 0, 0, 0, 15, 0, 15); return e;
    }
};

pub fn initEntity16() Entity16 {
    var e: Entity16 = .{};
    e.topology = .{0} ** 64;
    return e;
}
