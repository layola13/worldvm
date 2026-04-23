//! Physics Operators: FORCE_FALL, FLOW_STEP, BREAK, FORCE_PUSH, AABB

const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");

pub const FallResult = struct { can_fall: bool, target_y: i8, blocked: bool, blocker_id: u8 };

// Check if a voxel position is occupied by another instance (not self)
pub fn isOccupiedByOther(scene: *scene32.Scene32, inst: *const scene32.Instance, entities: []entity16.Entity16, sx: i8, sy: i8, sz: i8) bool {
    if (!scene32.inBounds(sx, sy, sz)) return true;
    if (!scene32.isOccupied(scene, sx, sy, sz)) return false;
    // Check if occupied by self
    const entity = &entities[inst.entity_id];
    const local_x = sx - inst.pos_x;
    const local_y = sy - inst.pos_y;
    const local_z = sz - inst.pos_z;
    if (local_x >= 0 and local_x < 16 and local_y >= 0 and local_y < 16 and local_z >= 0 and local_z < 16) {
        if (entity16.testVoxel(entity, @intCast(local_x), @intCast(local_y), @intCast(local_z))) return false;
    }
    return true;
}

pub fn checkFall(scene: *scene32.Scene32, inst: *const scene32.Instance, entities: []entity16.Entity16) FallResult {
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0) return .{ .can_fall = false, .target_y = inst.pos_y, .blocked = false, .blocker_id = 0 };
    if (entity.physics.mass == 0) return .{ .can_fall = false, .target_y = inst.pos_y, .blocked = false, .blocker_id = 0 };
    const below_y = inst.pos_y - 1;
    if (below_y < 0) return .{ .can_fall = false, .target_y = below_y, .blocked = true, .blocker_id = 255 };
    
    var ex: u8 = 0;
    while (ex < 16) : (ex += 1) {
        var ez: u8 = 0;
        while (ez < 16) : (ez += 1) {
            if (entity16.testVoxel(entity, ex, 0, ez)) {
                const sx = inst.pos_x + @as(i8, @intCast(ex));
                const sz = inst.pos_z + @as(i8, @intCast(ez));
                if (isOccupiedByOther(scene, inst, entities, sx, below_y, sz)) return .{ .can_fall = false, .target_y = below_y, .blocked = true, .blocker_id = 0 };
            }
        }
    }
    return .{ .can_fall = true, .target_y = below_y, .blocked = false, .blocker_id = 0 };
}

pub fn applyFall(inst: *scene32.Instance, target_y: i8) void { inst.pos_y = target_y; inst.state = .falling; }

pub const FlowDir = enum(u4) { hold = 0, down = 1, side_pos_x = 2, side_neg_x = 3, side_pos_z = 4, side_neg_z = 5 };
pub const FlowResult = struct { flowed: bool, dir: FlowDir, new_x: i8, new_y: i8, new_z: i8 };

pub fn checkFlow(scene: *scene32.Scene32, inst: *const scene32.Instance, entities: []entity16.Entity16) FlowResult {
    const entity = &entities[inst.entity_id];
    if (entity.physics.material != .liquid) return .{ .flowed = false, .dir = .hold, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
    const dirs = [_]struct { dx: i8, dy: i8, dz: i8, dir: FlowDir }{ 
        .{.dx=0,.dy=-1,.dz=0,.dir=.down}, 
        .{.dx=1,.dy=0,.dz=0,.dir=.side_pos_x}, 
        .{.dx=-1,.dy=0,.dz=0,.dir=.side_neg_x}, 
        .{.dx=0,.dy=0,.dz=1,.dir=.side_pos_z}, 
        .{.dx=0,.dy=0,.dz=-1,.dir=.side_neg_z} 
    };
    for (dirs) |d| {
        const nx = inst.pos_x + d.dx; const ny = inst.pos_y + d.dy; const nz = inst.pos_z + d.dz;
        // Basic instance bounds check to keep it within scene-reachable area
        if (nx < -16 or nx >= 32 or ny < -16 or ny >= 32 or nz < -16 or nz >= 32) continue;
        
        var ok = true;
        outer: for (0..16) |ex| { for (0..16) |ey| { for (0..16) |ez| {
            if (entity16.testVoxel(entity, @truncate(ex), @truncate(ey), @truncate(ez))) {
                if (isOccupiedByOther(scene, inst, entities, nx + @as(i8, @intCast(ex)), ny + @as(i8, @intCast(ey)), nz + @as(i8, @intCast(ez)))) { 
                    ok = false; break :outer; 
                }
            }
        } } }
        if (ok) return .{ .flowed = true, .dir = d.dir, .new_x = nx, .new_y = ny, .new_z = nz };
    }
    return .{ .flowed = false, .dir = .hold, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
}

pub fn applyFlow(inst: *scene32.Instance, r: FlowResult) void { inst.pos_x = r.new_x; inst.pos_y = r.new_y; inst.pos_z = r.new_z; inst.state = .flowing; }

// BREAK operator
pub const BreakResult = struct { did_break: bool, fragments: u8 };
pub fn checkBreak(impact: u16, material: entity16.MaterialType, hardness: u16) BreakResult {
    switch (material) {
        .fragile => if (impact >= 50) return .{ .did_break = true, .fragments = 4 },
        .solid => if (impact >= hardness) return .{ .did_break = true, .fragments = 2 },
        else => {},
    }
    return .{ .did_break = false, .fragments = 0 };
}
pub fn calcImpact(vel: i16, mass: u16) u16 { return if (vel < 0) @truncate((@as(u16, @intCast(-vel)) * mass) / 100) else 0; }

// AABB collision
pub const AABB = struct { min_x: i8, min_y: i8, min_z: i8, max_x: i8, max_y: i8, max_z: i8 };
pub fn makeAABB(x: i8, y: i8, z: i8, d: usize) AABB { return .{ .min_x=x, .min_y=y, .min_z=z, .max_x=x+@as(i8,@intCast(d)), .max_y=y+@as(i8,@intCast(d)), .max_z=z+@as(i8,@intCast(d)) }; }
pub fn aabbHit(a: AABB, b: AABB) bool { return !(a.max_x <= b.min_x or b.max_x <= a.min_x or a.max_y <= b.min_y or b.max_y <= a.min_y or a.max_z <= b.min_z or b.max_z <= a.min_z); }

// FORCE_PUSH
pub const PushResult = struct { pushed: bool, new_x: i8, new_y: i8, new_z: i8 };
pub fn checkPush(scene: *scene32.Scene32, inst: *const scene32.Instance, entities: []entity16.Entity16, dx: i8, dy: i8, dz: i8) PushResult {
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0) return .{ .pushed = false, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
    const nx = inst.pos_x + dx; const ny = inst.pos_y + dy; const nz = inst.pos_z + dz;
    if (nx < -16 or nx >= 32 or ny < -16 or ny >= 32 or nz < -16 or nz >= 32) return .{ .pushed = false, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
    
    for (0..16) |ex| { for (0..16) |ey| { for (0..16) |ez| {
        if (entity16.testVoxel(entity, @truncate(ex), @truncate(ey), @truncate(ez))) {
            if (isOccupiedByOther(scene, inst, entities, nx + @as(i8,@intCast(ex)), ny + @as(i8,@intCast(ey)), nz + @as(i8,@intCast(ez)))) return .{ .pushed = false, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
        }
    } } }
    return .{ .pushed = true, .new_x = nx, .new_y = ny, .new_z = nz };
}
pub fn applyPush(inst: *scene32.Instance, r: PushResult) void { inst.pos_x = r.new_x; inst.pos_y = r.new_y; inst.pos_z = r.new_z; inst.state = .moving; }
