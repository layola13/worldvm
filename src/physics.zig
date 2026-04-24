//! Physics Operators: FORCE_FALL, FLOW_STEP, BREAK, FORCE_PUSH, AABB, CONTINUOUS_PHYSICS
//!
//! Phase 1: Continuous Physics System
//! - Velocity-based movement with gravity, friction, and restitution
//! - Angular velocity for rotation

const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const address = @import("address.zig");

// ============================================================================
// Constants
// ============================================================================

pub const GRAVITY: i16 = -100;           // Default gravity acceleration per tick
pub const TERMINAL_VELOCITY: i32 = 500;  // Max fall velocity
pub const DAMPING: u8 = 245;             // Velocity damping (0-255, 255=no damping)
pub const ANGULAR_DAMPING: u8 = 240;      // Angular velocity damping

// ============================================================================
// Velocity & Continuous Physics
// ============================================================================

pub const FallResult = struct { can_fall: bool, target_y: i32, blocked: bool, blocker_id: u8 };

/// Apply gravity acceleration to velocity
pub fn applyGravity(vel_y: *i16, mass: u16) void {
    if (mass == 0) return;  // Static objects don't fall
    const new_vel = @as(i32, vel_y.*) + @as(i32, GRAVITY);
    vel_y.* = @truncate(@min(TERMINAL_VELOCITY, new_vel));
}

/// Apply linear damping (airs resistance)
pub fn applyDamping(vel_x: *i16, vel_y: *i16, vel_z: *i16) void {
    vel_x.* = @truncate((@as(i32, vel_x.*) * DAMPING) >> 8);
    vel_y.* = @truncate((@as(i32, vel_y.*) * DAMPING) >> 8);
    vel_z.* = @truncate((@as(i32, vel_z.*) * DAMPING) >> 8);
}

/// Apply angular damping to rotation
pub fn applyAngularDamping(ang_x: *i8, ang_y: *i8, ang_z: *i8) void {
    ang_x.* = @truncate((@as(i16, ang_x.*) * ANGULAR_DAMPING) >> 8);
    ang_y.* = @truncate((@as(i16, ang_y.*) * ANGULAR_DAMPING) >> 8);
    ang_z.* = @truncate((@as(i16, ang_z.*) * ANGULAR_DAMPING) >> 8);
}

/// Calculate new position from velocity (Euler integration)
pub fn integratePosition(pos_x: i32, pos_y: i32, pos_z: i32, vel_x: i16, vel_y: i16, vel_z: i16) struct { x: i32, y: i32, z: i32 } {
    return .{
        .x = pos_x + @as(i32, vel_x),
        .y = pos_y + @as(i32, vel_y),
        .z = pos_z + @as(i32, vel_z),
    };
}

/// Apply restitution (bounce) on collision
/// Returns new velocity after bounce based on restitution coefficient
pub fn applyRestitution(vel: i16, restitution: u8) i16 {
    if (vel == 0) return 0;
    // restitution is 0-255, where 255 = perfectly elastic (100% bounce)
    // We scale to 0-1 range: r = restitution / 255
    // bounce = -vel * r
    const r = @as(i32, restitution);
    return @truncate(@divTrunc(-(@as(i32, vel) * r), 255));
}

/// Apply friction to horizontal velocity
pub fn applyFriction(vel_x: *i16, vel_z: *i16, friction: u8) void {
    const f = @as(i32, friction);
    // Reduce velocity by friction factor
    const new_vx = @divTrunc(@as(i32, vel_x.*) * (256 - f), 256);
    const new_vz = @divTrunc(@as(i32, vel_z.*) * (256 - f), 256);
    vel_x.* = @truncate(new_vx);
    vel_z.* = @truncate(new_vz);
}

// ============================================================================
// Global occupancy check (existing)
// ============================================================================

// Global occupancy check that handles page boundaries
pub fn isOccupiedGlobal(s1024: *scene1024.Scene1024, inst: *const scene32.Instance, entities: []entity16.Entity16, gx: i32, gy: i32, gz: i32, blocker_out: ?*u8) bool {
    // 1. Boundary check for the 1024^3 world
    if (gx < 0 or gx >= 1024 or gy < 0 or gy >= 1024 or gz < 0 or gz >= 1024) return true;

    // 2. Check if it's occupied by self
    const entity = &entities[inst.entity_id];
    const local_x = gx - inst.pos_x;
    const local_y = gy - inst.pos_y;
    const local_z = gz - inst.pos_z;
    if (local_x >= 0 and local_x < 16 and local_y >= 0 and local_y < 16 and local_z >= 0 and local_z < 16) {
        if (entity16.testVoxel(entity, @intCast(local_x), @intCast(local_y), @intCast(local_z))) return false;
    }

    // 3. Map global to page and local
    const px: u5 = @intCast(@divFloor(gx, 32));
    const py: u5 = @intCast(@divFloor(gy, 32));
    const pz: u5 = @intCast(@divFloor(gz, 32));
    const lx: u5 = @intCast(@mod(gx, 32));
    const ly: u5 = @intCast(@mod(gy, 32));
    const lz: u5 = @intCast(@mod(gz, 32));

    const addr = address.encode(.{
        .world = 0, .px = px, .py = py, .pz = pz, .lx = lx, .ly = ly, .lz = lz
    });

    // Check all instances in s1024 for this voxel (slow but correct for MVP)
    for (0..s1024.instance_count) |i| {
        const other = &s1024.instances[i];
        if (other == inst) continue;

        const ox = gx - other.pos_x;
        const oy = gy - other.pos_y;
        const oz = gz - other.pos_z;

        if (ox >= 0 and ox < 16 and oy >= 0 and oy < 16 and oz >= 0 and oz < 16) {
            if (entity16.testVoxel(&entities[other.entity_id], @intCast(ox), @intCast(oy), @intCast(oz))) {
                if (blocker_out != null) blocker_out.?.* = @intCast(i);
                return true;
            }
        }
    }

    const occupied = s1024.getVoxelAtGlobal(addr) catch true;
    if (occupied and blocker_out != null) blocker_out.?.* = 255; // Environment
    return occupied;
}

// ============================================================================
// Fall with Continuous Physics
// ============================================================================

pub fn checkFall(s1024: *scene1024.Scene1024, inst: *const scene32.Instance, entities: []entity16.Entity16) FallResult {
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0) return .{ .can_fall = false, .target_y = inst.pos_y, .blocked = false, .blocker_id = 0 };
    if (entity.physics.mass == 0) return .{ .can_fall = false, .target_y = inst.pos_y, .blocked = false, .blocker_id = 0 };
    const below_y = inst.pos_y - 1;
    if (below_y < 0) return .{ .can_fall = false, .target_y = below_y, .blocked = true, .blocker_id = 255 };

    // Optimization: check bottom layer of entity
    var ex: u8 = 0;
    var blocker: u8 = 0;
    while (ex < 16) : (ex += 1) {
        var ez: u8 = 0;
        while (ez < 16) : (ez += 1) {
            if (entity16.testVoxel(entity, ex, 0, ez)) {
                if (isOccupiedGlobal(s1024, inst, entities, inst.pos_x + ex, below_y, inst.pos_z + ez, &blocker)) {
                    return .{ .can_fall = false, .target_y = below_y, .blocked = true, .blocker_id = blocker };
                }
            }
        }
    }
    return .{ .can_fall = true, .target_y = below_y, .blocked = false, .blocker_id = 0 };
}

/// Check continuous fall using velocity - returns how far the entity can fall this tick
pub fn checkContinuousFall(s1024: *scene1024.Scene1024, inst: *const scene32.Instance, entities: []entity16.Entity16) FallResult {
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0) return .{ .can_fall = false, .target_y = inst.pos_y, .blocked = false, .blocker_id = 0 };
    if (entity.physics.mass == 0) return .{ .can_fall = false, .target_y = inst.pos_y, .blocked = false, .blocker_id = 0 };

    // Velocity-based fall: move by vel_y (negative = falling down)
    const vel_y = inst.vel_y;
    if (vel_y >= 0) return .{ .can_fall = false, .target_y = inst.pos_y, .blocked = false, .blocker_id = 0 };

    // Calculate target Y position
    var target_y = inst.pos_y;
    var remaining = @as(i32, -vel_y);  // How many voxels to fall
    var blocker: u8 = 0;

    while (remaining > 0) {
        const next_y = target_y - 1;
        if (next_y < 0) return .{ .can_fall = false, .target_y = 0, .blocked = true, .blocker_id = 255 };

        // Check bottom layer of entity at next position
        var ex: u8 = 0;
        var hit_blocker = false;
        while (ex < 16) : (ex += 1) {
            var ez: u8 = 0;
            while (ez < 16) : (ez += 1) {
                if (entity16.testVoxel(entity, ex, 0, ez)) {
                    if (isOccupiedGlobal(s1024, inst, entities, inst.pos_x + ex, next_y, inst.pos_z + ez, &blocker)) {
                        hit_blocker = true;
                        break;
                    }
                }
            }
            if (hit_blocker) break;
        }

        if (hit_blocker) {
            return .{ .can_fall = false, .target_y = target_y, .blocked = true, .blocker_id = blocker };
        }

        target_y = next_y;
        remaining -= 1;
    }

    return .{ .can_fall = true, .target_y = target_y, .blocked = false, .blocker_id = 0 };
}

pub fn applyFall(inst: *scene32.Instance, target_y: i32) void { inst.pos_y = target_y; inst.state = .falling; }

// ============================================================================
// Flow (for liquids)
// ============================================================================

pub const FlowDir = enum(u4) { hold = 0, down = 1, side_pos_x = 2, side_neg_x = 3, side_pos_z = 4, side_neg_z = 5 };
pub const FlowResult = struct { flowed: bool, dir: FlowDir, new_x: i32, new_y: i32, new_z: i32 };

pub fn checkFlow(s1024: *scene1024.Scene1024, inst: *const scene32.Instance, entities: []entity16.Entity16) FlowResult {
    const entity = &entities[inst.entity_id];
    if (entity.physics.material != .liquid) return .{ .flowed = false, .dir = .hold, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
    const dirs = [_]struct { dx: i8, dy: i8, dz: i8, dir: FlowDir }{
        .{.dx=0,.dy=-1,.dz=0,.dir=.down},
        .{.dx=1,.dy=0,.dz=0,.dir=.side_pos_x}, .{.dx=-1,.dy=0,.dz=0,.dir=.side_neg_x},
        .{.dx=0,.dy=0,.dz=1,.dir=.side_pos_z}, .{.dx=0,.dy=0,.dz=-1,.dir=.side_neg_z}
    };
    for (dirs) |d| {
        const nx = inst.pos_x + d.dx; const ny = inst.pos_y + d.dy; const nz = inst.pos_z + d.dz;
        var ok = true;
        for (0..64) |w_idx| {
            const word = entity.topology[w_idx];
            if (word == 0) continue;
            for (0..64) |b_idx| {
                if ((word & (@as(u64, 1) << @as(u6, @truncate(b_idx)))) != 0) {
                    const idx = (w_idx << 6) | b_idx;
                    const ex: i32 = @intCast((idx >> 4) & 0xF);
                    const ey: i32 = @intCast(idx >> 8);
                    const ez: i32 = @intCast(idx & 0xF);
                    if (isOccupiedGlobal(s1024, inst, entities, nx + ex, ny + ey, nz + ez, null)) {
                        ok = false; break;
                    }
                }
            }
            if (!ok) break;
        }
        if (ok) return .{ .flowed = true, .dir = d.dir, .new_x = nx, .new_y = ny, .new_z = nz };
    }
    return .{ .flowed = false, .dir = .hold, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
}

pub fn applyFlow(inst: *scene32.Instance, r: FlowResult) void { inst.pos_x = r.new_x; inst.pos_y = r.new_y; inst.pos_z = r.new_z; inst.state = .flowing; }

// ============================================================================
// Break
// ============================================================================

pub const BreakResult = struct { did_break: bool, fragments: u8 };
pub fn checkBreak(impact: u16, material: entity16.MaterialType, hardness: u16) BreakResult {
    switch (material) {
        .fragile => if (impact >= 50) return .{ .did_break = true, .fragments = 4 },
        .solid => if (impact >= hardness) return .{ .did_break = true, .fragments = 2 },
        else => {},
    }
    return .{ .did_break = false, .fragments = 0 };
}
pub fn calcImpact(vel: i16, mass: u16) u16 {
    if (vel >= 0) return 0;
    // Use i32 to avoid overflow when negating i16.MIN (-32768)
    const abs_vel: u32 = @intCast(-@as(i32, vel));
    const impact: u32 = (abs_vel * @as(u32, mass)) / 100;
    return @truncate(@min(impact, 65535));
}

// ============================================================================
// AABB
// ============================================================================

pub const AABB = struct { min_x: i32, min_y: i32, min_z: i32, max_x: i32, max_y: i32, max_z: i32 };
pub fn makeAABB(x: i32, y: i32, z: i32, d: usize) AABB { return .{ .min_x=x, .min_y=y, .min_z=z, .max_x=x+@as(i32,@intCast(d)), .max_y=y+@as(i32,@intCast(d)), .max_z=z+@as(i32,@intCast(d)) }; }
pub fn aabbHit(a: AABB, b: AABB) bool { return !(a.max_x <= b.min_x or b.max_x <= a.min_x or a.max_y <= b.min_y or b.max_y <= a.min_y or a.max_z <= b.min_z or b.max_z <= a.min_z); }

// ============================================================================
// Push
// ============================================================================

pub const PushResult = struct { pushed: bool, new_x: i32, new_y: i32, new_z: i32 };
pub fn checkPush(s1024: *scene1024.Scene1024, inst: *const scene32.Instance, entities: []entity16.Entity16, dx: i8, dy: i8, dz: i8) PushResult {
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0) return .{ .pushed = false, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
    const nx = inst.pos_x + dx; const ny = inst.pos_y + dy; const nz = inst.pos_z + dz;

    var ok = true;
    for (0..64) |w_idx| {
        const word = entity.topology[w_idx];
        if (word == 0) continue;
        for (0..64) |b_idx| {
            if ((word & (@as(u64, 1) << @as(u6, @truncate(b_idx)))) != 0) {
                const idx = (w_idx << 6) | b_idx;
                const ex: i32 = @intCast((idx >> 4) & 0xF);
                const ey: i32 = @intCast(idx >> 8);
                const ez: i32 = @intCast(idx & 0xF);
                if (isOccupiedGlobal(s1024, inst, entities, nx + ex, ny + ey, nz + ez, null)) {
                    ok = false; break;
                }
            }
        }
        if (!ok) break;
    }
    if (ok) return .{ .pushed = true, .new_x = nx, .new_y = ny, .new_z = nz };
    return .{ .pushed = false, .new_x = inst.pos_x, .new_y = inst.pos_y, .new_z = inst.pos_z };
}
pub fn applyPush(inst: *scene32.Instance, r: PushResult) void { inst.pos_x = r.new_x; inst.pos_y = r.new_y; inst.pos_z = r.new_z; inst.state = .moving; }
