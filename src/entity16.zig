//! Entity16: 4KB fixed-size entity
const std = @import("std");

pub const ENTITY_SIZE: usize = 4096;
pub const TOPOLOGY_WORDS: usize = 64;
pub const ENTITY_DIM: usize = 16;

pub const MaterialType = enum(u8) {
    solid = 0,
    liquid = 1,
    gas = 2,
    fragile = 3,
    elastic = 4,
    composite = 5,
};

pub const PhysicsBlock = struct {
    mass: u16 = 0,
    hardness: u16 = 100,
    material: MaterialType = .solid,
    friction: u8 = 128,
    restitution: u8 = 64,
    conductivity: u8 = 0,
    flags: u8 = 0,
    temp_state: u8 = 0,
    stability: u16 = 100,
    group_id: u8 = 0,
    reserved: u16 = 0,
};

pub const VisualBlock = struct {
    color_pack: u8 = 0,
    color_alpha: u8 = 255,
    reflection: u8 = 0,
    glow: u8 = 0,
};

pub const EdgeSlot = struct {
    target_id: u16 = 0,
    relation_type: u8 = 0,
    weight: u8 = 0,
};

pub const ENTITY16_EXTENSION_ABI_VERSION: u16 = 1;

pub const ChemistryBlock = extern struct {
    chemical_signature: u32 = 0,
    smell_profile_id: u16 = 0,
    taste_profile_id: u16 = 0,
    reaction_rule_set: u16 = 0,
    toxicity_level: u16 = 0,
    extension_abi_version: u16 = ENTITY16_EXTENSION_ABI_VERSION,
    reserved: u16 = 0,
};

pub const SemanticsBlock = extern struct {
    role_tag: u16 = 0,
    category_tag: u16 = 0,
    priority: u8 = 0,
    affordance_flags: u8 = 0,
    reserved: [58]u8 = .{0} ** 58,
};

pub const AffectBlock = extern struct {
    valence: i8 = 0,
    arousal: u8 = 0,
    certainty: u8 = 128,
    control: u8 = 128,
    mood_flags: u8 = 0,
    reserved: [11]u8 = .{0} ** 11,
};

pub const BehaviorBlock = extern struct {
    on_contact_rule_id: u16 = 0,
    on_focus_rule_id: u16 = 0,
    on_threat_rule_id: u16 = 0,
    on_bind_rule_id: u16 = 0,
    on_decay_rule_id: u16 = 0,
    on_sound_event_rule_id: u16 = 0,
    on_chemical_event_rule_id: u16 = 0,
    rule_flags: u16 = 0,
    reserved: [112]u8 = .{0} ** 112,
};

pub const TOPOLOGY_SIZE: usize = TOPOLOGY_WORDS * @sizeOf(u64);
pub const PHYSICS_SIZE: usize = 16;
pub const CHEMISTRY_SIZE: usize = 16;
pub const VISUAL_SIZE: usize = 4;
pub const SEMANTICS_SIZE: usize = 64;
pub const AFFECT_SIZE: usize = 16;
pub const RELATIONS_SIZE: usize = 64 * @sizeOf(EdgeSlot);
pub const BEHAVIOR_SIZE: usize = 128;
pub const RESERVED_SIZE: usize = ENTITY_SIZE - TOPOLOGY_SIZE - PHYSICS_SIZE - CHEMISTRY_SIZE - VISUAL_SIZE - SEMANTICS_SIZE - AFFECT_SIZE - RELATIONS_SIZE - BEHAVIOR_SIZE;

const sdf = @import("sdf.zig");

pub const Entity16 = struct {
    topology: [TOPOLOGY_WORDS]u64 = undefined,
    physics: PhysicsBlock = .{},
    chemistry: ChemistryBlock = .{},
    visual: VisualBlock = .{},
    semantics: SemanticsBlock = .{},
    affect: AffectBlock = .{},
    relations: [64]EdgeSlot = undefined,
    behavior: BehaviorBlock = .{},
    reserved: [RESERVED_SIZE]u8 = undefined,

    pub fn fromSDF(node: sdf.SDFNode) Entity16 {
        var e = initEntity16();
        var x: u8 = 0;
        while (x < ENTITY_DIM) : (x += 1) {
            var y: u8 = 0;
            while (y < ENTITY_DIM) : (y += 1) {
                var z: u8 = 0;
                while (z < ENTITY_DIM) : (z += 1) {
                    // Map 0..15 to -1..1 range for SDF evaluation
                    const fx = (@as(f32, @floatFromInt(x)) - 7.5) / 7.5;
                    const fy = (@as(f32, @floatFromInt(y)) - 7.5) / 7.5;
                    const fz = (@as(f32, @floatFromInt(z)) - 7.5) / 7.5;

                    if (node.evaluate(sdf.Vec3.init(fx, fy, fz)) <= 0.0) {
                        setVoxel(&e, x, y, z);
                    }
                }
            }
        }
        return e;
    }
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
    const min_x = @min(x1, x2);
    const max_x = @max(x1, x2);
    const min_y = @min(y1, y2);
    const max_y = @max(y1, y2);
    const min_z = @min(z1, z2);
    const max_z = @max(z1, z2);
    var x: i32 = min_x;
    while (x <= max_x) : (x += 1) {
        var y: i32 = min_y;
        while (y <= max_y) : (y += 1) {
            var z: i32 = min_z;
            while (z <= max_z) : (z += 1) {
                setVoxel(entity, @intCast(x), @intCast(y), @intCast(z));
            }
        }
    }
}

pub fn fillSphere(entity: *Entity16, cx: u8, cy: u8, cz: u8, radius: u8) void {
    const r2 = radius * radius;
    var x: i32 = 0;
    while (x < @as(i32, ENTITY_DIM)) : (x += 1) {
        var y: i32 = 0;
        while (y < @as(i32, ENTITY_DIM)) : (y += 1) {
            var z: i32 = 0;
            while (z < @as(i32, ENTITY_DIM)) : (z += 1) {
                const dx: i32 = x - @as(i32, cx);
                const dy: i32 = y - @as(i32, cy);
                const dz: i32 = z - @as(i32, cz);
                if (dx * dx + dy * dy + dz * dz <= r2) setVoxel(entity, @intCast(x), @intCast(y), @intCast(z));
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
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 50;
        e.physics.material = .solid;
        fillSphere(&e, 8, 8, 8, 5);
        return e;
    }
    pub fn table() Entity16 {
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 500;
        e.physics.material = .solid;
        e.physics.flags |= 0x01;
        fillBox(&e, 0, 0, 0, 15, 1, 15);
        fillBox(&e, 1, 1, 1, 3, 7, 3);
        fillBox(&e, 12, 1, 1, 14, 7, 3);
        fillBox(&e, 1, 1, 12, 3, 7, 14);
        fillBox(&e, 12, 1, 12, 14, 7, 14);
        return e;
    }
    pub fn hammer() Entity16 {
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 1000;
        e.physics.material = .solid;
        fillBox(&e, 5, 8, 6, 10, 14, 9);
        fillBox(&e, 3, 10, 4, 12, 12, 11);
        return e;
    }
    pub fn glass() Entity16 {
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 30;
        e.physics.material = .fragile;
        e.physics.hardness = 30;
        e.physics.flags |= 0x04;
        fillBox(&e, 2, 0, 2, 13, 12, 13);
        return e;
    }
    pub fn water() Entity16 {
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 10;
        e.physics.material = .liquid;
        fillBox(&e, 0, 0, 0, 7, 7, 7);
        return e;
    }
    pub fn floor() Entity16 {
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 0;
        e.physics.material = .solid;
        e.physics.flags |= 0x01;
        fillBox(&e, 0, 0, 0, 15, 0, 15);
        return e;
    }
    // Physics test prototypes
    pub fn ball() Entity16 {
        // Small elastic ball for bounce testing
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 20;
        e.physics.material = .elastic;
        e.physics.restitution = 200;
        e.physics.hardness = 255;
        fillSphere(&e, 8, 8, 8, 4);
        return e;
    }
    pub fn brick() Entity16 {
        // Simple solid brick for stacking
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 100;
        e.physics.material = .solid;
        fillBox(&e, 2, 0, 2, 13, 6, 13);
        return e;
    }
    pub fn domino() Entity16 {
        // Thin tall box for domino effect
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 30;
        e.physics.material = .solid;
        fillBox(&e, 5, 0, 6, 10, 14, 9);
        return e;
    }
    pub fn plate() Entity16 {
        // Flat plate surface
        var e: Entity16 = .{};
        e.topology = .{0} ** 64;
        e.physics.mass = 50;
        e.physics.material = .solid;
        e.physics.flags |= 0x01; // Fixed
        fillBox(&e, 0, 0, 0, 15, 1, 15);
        return e;
    }
};

pub fn initEntity16() Entity16 {
    var e: Entity16 = .{};
    e.topology = .{0} ** 64;
    e.relations = .{EdgeSlot{}} ** 64;
    e.reserved = .{0} ** RESERVED_SIZE;
    return e;
}

test "Entity16 layout stays fixed at 4KB" {
    try std.testing.expectEqual(@as(usize, ENTITY_SIZE), @sizeOf(Entity16));
    try std.testing.expectEqual(@as(usize, 512), @sizeOf(@TypeOf(initEntity16().topology)));
    try std.testing.expectEqual(@as(usize, PHYSICS_SIZE), @sizeOf(PhysicsBlock));
    try std.testing.expectEqual(@as(usize, CHEMISTRY_SIZE), @sizeOf(ChemistryBlock));
    try std.testing.expectEqual(@as(usize, VISUAL_SIZE), @sizeOf(VisualBlock));
    try std.testing.expectEqual(@as(usize, SEMANTICS_SIZE), @sizeOf(SemanticsBlock));
    try std.testing.expectEqual(@as(usize, AFFECT_SIZE), @sizeOf(AffectBlock));
    try std.testing.expectEqual(@as(usize, RELATIONS_SIZE), @sizeOf(@TypeOf(initEntity16().relations)));
    try std.testing.expectEqual(@as(usize, BEHAVIOR_SIZE), @sizeOf(BehaviorBlock));
    try std.testing.expectEqual(
        @as(usize, ENTITY_SIZE),
        TOPOLOGY_SIZE + PHYSICS_SIZE + CHEMISTRY_SIZE + VISUAL_SIZE + SEMANTICS_SIZE + AFFECT_SIZE + RELATIONS_SIZE + BEHAVIOR_SIZE + RESERVED_SIZE,
    );
}

test "Entity16 named extension blocks initialize and remain writable" {
    var entity = initEntity16();

    try std.testing.expectEqual(ENTITY16_EXTENSION_ABI_VERSION, entity.chemistry.extension_abi_version);
    try std.testing.expectEqual(@as(u32, 0), entity.chemistry.chemical_signature);
    try std.testing.expectEqual(@as(i8, 0), entity.affect.valence);
    try std.testing.expectEqual(@as(u8, 128), entity.affect.certainty);
    try std.testing.expectEqual(@as(u16, 0), entity.behavior.on_contact_rule_id);

    entity.chemistry.chemical_signature = 0xCAFE_BABE;
    entity.chemistry.extension_abi_version = 2;
    entity.semantics.role_tag = 7;
    entity.affect.valence = -12;
    entity.behavior.on_chemical_event_rule_id = 42;

    try std.testing.expectEqual(@as(u32, 0xCAFE_BABE), entity.chemistry.chemical_signature);
    try std.testing.expectEqual(@as(u16, 2), entity.chemistry.extension_abi_version);
    try std.testing.expectEqual(@as(u16, 7), entity.semantics.role_tag);
    try std.testing.expectEqual(@as(i8, -12), entity.affect.valence);
    try std.testing.expectEqual(@as(u16, 42), entity.behavior.on_chemical_event_rule_id);
}

test "Entity16 extension ABI version occupies reserved chemistry slot" {
    try std.testing.expectEqual(@as(usize, 0), @offsetOf(ChemistryBlock, "chemical_signature"));
    try std.testing.expectEqual(@as(usize, 4), @offsetOf(ChemistryBlock, "smell_profile_id"));
    try std.testing.expectEqual(@as(usize, 6), @offsetOf(ChemistryBlock, "taste_profile_id"));
    try std.testing.expectEqual(@as(usize, 8), @offsetOf(ChemistryBlock, "reaction_rule_set"));
    try std.testing.expectEqual(@as(usize, 10), @offsetOf(ChemistryBlock, "toxicity_level"));
    try std.testing.expectEqual(@as(usize, 12), @offsetOf(ChemistryBlock, "extension_abi_version"));
    try std.testing.expectEqual(@as(usize, 14), @offsetOf(ChemistryBlock, "reserved"));

    const entity = initEntity16();
    const bytes = std.mem.asBytes(&entity.chemistry);
    try std.testing.expectEqual(@as(u8, @truncate(ENTITY16_EXTENSION_ABI_VERSION)), bytes[12]);
    try std.testing.expectEqual(@as(u8, ENTITY16_EXTENSION_ABI_VERSION >> 8), bytes[13]);
    try std.testing.expectEqual(@as(u8, 0), bytes[14]);
    try std.testing.expectEqual(@as(u8, 0), bytes[15]);
}

test "Entity16 voxel bit mapping covers full 16 cube" {
    var entity = initEntity16();

    try std.testing.expectEqual(@as(u12, 0), coordToBitIndex(0, 0, 0));
    try std.testing.expectEqual(@as(u12, 15), coordToBitIndex(0, 0, 15));
    try std.testing.expectEqual(@as(u12, 16), coordToBitIndex(1, 0, 0));
    try std.testing.expectEqual(@as(u12, 256), coordToBitIndex(0, 1, 0));
    try std.testing.expectEqual(@as(u12, 4095), coordToBitIndex(15, 15, 15));

    setVoxel(&entity, 0, 0, 0);
    setVoxel(&entity, 15, 15, 15);
    try std.testing.expect(testVoxel(&entity, 0, 0, 0));
    try std.testing.expect(testVoxel(&entity, 15, 15, 15));
    try std.testing.expectEqual(@as(u12, 2), countVoxels(&entity));

    clearVoxel(&entity, 0, 0, 0);
    try std.testing.expect(!testVoxel(&entity, 0, 0, 0));
    try std.testing.expect(testVoxel(&entity, 15, 15, 15));
    try std.testing.expectEqual(@as(u12, 1), countVoxels(&entity));
}

test "Entity16 fillBox is inclusive and order independent" {
    var entity = initEntity16();

    fillBox(&entity, 3, 4, 5, 1, 2, 3);
    try std.testing.expectEqual(@as(u12, 27), countVoxels(&entity));
    try std.testing.expect(testVoxel(&entity, 1, 2, 3));
    try std.testing.expect(testVoxel(&entity, 2, 3, 4));
    try std.testing.expect(testVoxel(&entity, 3, 4, 5));
    try std.testing.expect(!testVoxel(&entity, 0, 2, 3));
    try std.testing.expect(!testVoxel(&entity, 3, 4, 6));
}

test "Entity16 fillHollowBox writes only shell voxels" {
    var entity = initEntity16();

    fillHollowBox(&entity, 1, 1, 1, 3, 3, 3);
    try std.testing.expectEqual(@as(u12, 26), countVoxels(&entity));
    try std.testing.expect(testVoxel(&entity, 1, 1, 1));
    try std.testing.expect(testVoxel(&entity, 2, 1, 2));
    try std.testing.expect(testVoxel(&entity, 3, 3, 3));
    try std.testing.expect(!testVoxel(&entity, 2, 2, 2));
}

test "Entity16 prototypes initialize expected material contracts" {
    const water = Prototypes.water();
    const floor = Prototypes.floor();
    const ball = Prototypes.ball();

    try std.testing.expectEqual(MaterialType.liquid, water.physics.material);
    try std.testing.expect(countVoxels(&water) > 0);
    try std.testing.expectEqual(@as(u16, 0), floor.physics.mass);
    try std.testing.expect((floor.physics.flags & 0x01) != 0);
    try std.testing.expectEqual(MaterialType.elastic, ball.physics.material);
    try std.testing.expect(ball.physics.restitution > 128);
}
