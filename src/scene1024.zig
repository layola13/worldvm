//! Scene1024: 1024^3 Virtual Paged Scene
const std = @import("std");
const scene32 = @import("scene32.zig");
const physics = @import("physics.zig");
const address = @import("address.zig");
const entity16 = @import("entity16.zig");

pub const MAX_ACTIVE_PAGES = 16;
pub const PAGE_SIZE_VOXELS = 32;

pub const PageEntry = struct {
    page_id: u32,
    resident: bool,
    dirty: bool,
    last_tick: u32,
    aabb: physics.AABB = undefined,
    scene: ?*scene32.Scene32 = null,
};

pub const Scene1024 = struct {
    pages: [MAX_ACTIVE_PAGES]PageEntry,
    active_count: u8,
    global_tick: u32,
    allocator: std.mem.Allocator,
    instances: [128]scene32.Instance,
    instance_count: u8,

    pub fn init(allocator: std.mem.Allocator) Scene1024 {
        var s = Scene1024{
            .pages = undefined,
            .active_count = 0,
            .global_tick = 0,
            .allocator = allocator,
            .instances = undefined,
            .instance_count = 0,
        };
        @memset(&s.instances, std.mem.zeroes(scene32.Instance));
        for (0..MAX_ACTIVE_PAGES) |i| {
            s.pages[i] = .{
                .page_id = 0,
                .resident = false,
                .dirty = false,
                .last_tick = 0,
                .scene = null,
            };
        }
        return s;
    }

    pub fn addInstance(self: *Scene1024, inst: scene32.Instance) !u8 {
        if (self.instance_count >= 128) return error.TooManyInstances;
        const idx = self.instance_count;
        self.instances[idx] = inst;
        self.instance_count += 1;
        return idx;
    }

    pub fn rebuildOccupancy(self: *Scene1024, entities: []const entity16.Entity16) !void {
        // 1. Clear all active pages
        for (0..MAX_ACTIVE_PAGES) |i| {
            if (self.pages[i].resident and self.pages[i].scene != null) {
                @memset(&self.pages[i].scene.?.occupancy, 0);
            }
        }

        // 2. Project all instances
        for (0..self.instance_count) |i| {
            const inst = &self.instances[i];
            const entity = &entities[inst.entity_id];
            
            var ex: u8 = 0;
            while (ex < 16) : (ex += 1) {
                var ey: u8 = 0;
                while (ey < 16) : (ey += 1) {
                    var ez: u8 = 0;
                    while (ez < 16) : (ez += 1) {
                        if (entity16.testVoxel(entity, ex, ey, ez)) {
                            const gx = inst.pos_x + ex;
                            const gy = inst.pos_y + ey;
                            const gz = inst.pos_z + ez;
                            
                            // Map global to page
                            const px: u32 = @intCast(@divFloor(gx, 32));
                            const py: u32 = @intCast(@divFloor(gy, 32));
                            const pz: u32 = @intCast(@divFloor(gz, 32));
                            const page_id = address.encode(.{
                                .world = 0, .px = @intCast(px), .py = @intCast(py), .pz = @intCast(pz),
                                .lx = 0, .ly = 0, .lz = 0
                            }) >> 15; // Extract page part
                            
                            // Only project to resident pages
                            for (0..MAX_ACTIVE_PAGES) |p_idx| {
                                if (self.pages[p_idx].resident and self.pages[p_idx].page_id == @as(u32, @truncate(page_id))) {
                                    const lx: i32 = @mod(gx, 32);
                                    const ly: i32 = @mod(gy, 32);
                                    const lz: i32 = @mod(gz, 32);
                                    scene32.setOccupied(self.pages[p_idx].scene.?, lx, ly, lz);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn deinit(self: *Scene1024) void {
        for (0..MAX_ACTIVE_PAGES) |i| {
            if (self.pages[i].scene) |s| {
                self.allocator.destroy(s);
            }
        }
    }

    pub fn getPage(self: *Scene1024, page_id: u32) !*PageEntry {
        // Find if resident
        for (0..MAX_ACTIVE_PAGES) |i| {
            if (self.pages[i].resident and self.pages[i].page_id == page_id) {
                self.pages[i].last_tick = self.global_tick;
                return &self.pages[i];
            }
        }

        // Not resident, need to load/allocate
        var target_idx: ?usize = null;
        if (self.active_count < MAX_ACTIVE_PAGES) {
            target_idx = self.active_count;
            self.active_count += 1;
        } else {
            // LRU replacement
            var oldest_tick = self.global_tick;
            var oldest_idx: usize = 0;
            for (0..MAX_ACTIVE_PAGES) |i| {
                if (self.pages[i].last_tick <= oldest_tick) {
                    oldest_tick = self.pages[i].last_tick;
                    oldest_idx = i;
                }
            }
            target_idx = oldest_idx;
        }

        const idx = target_idx.?;
        const entry = &self.pages[idx];
        
        if (entry.scene == null) {
            entry.scene = try self.allocator.create(scene32.Scene32);
        }
        
        entry.page_id = page_id;
        entry.resident = true;
        entry.dirty = false;
        entry.last_tick = self.global_tick;
        entry.scene.?.* = scene32.initScene();
        entry.scene.?.scene_id = page_id;
        
        return entry;
    }

    pub fn getVoxelAtGlobal(self: *Scene1024, addr_raw: address.WorldAddr) !bool {
        const parts = address.decode(addr_raw);
        const page_id = address.getPageId(addr_raw);
        const entry = try self.getPage(page_id);
        
        const sx: i8 = @intCast(parts.lx);
        const sy: i8 = @intCast(parts.ly);
        const sz: i8 = @intCast(parts.lz);
        
        return scene32.isOccupied(entry.scene.?, sx, sy, sz);
    }

    pub fn setVoxelAtGlobal(self: *Scene1024, addr_raw: address.WorldAddr, val: bool) !void {
        const parts = address.decode(addr_raw);
        const page_id = address.getPageId(addr_raw);
        const entry = try self.getPage(page_id);
        entry.dirty = true;
        
        const sx: i8 = @intCast(parts.lx);
        const sy: i8 = @intCast(parts.ly);
        const sz: i8 = @intCast(parts.lz);
        
        if (val) {
            scene32.setOccupied(entry.scene.?, sx, sy, sz);
        } else {
            scene32.clearOccupied(entry.scene.?, sx, sy, sz);
        }
    }
};

test "Global voxel addressing" {
    const allocator = std.testing.allocator;
    var s1024 = Scene1024.init(allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0, .px = 1, .py = 1, .pz = 1, .lx = 10, .ly = 10, .lz = 10,
    });

    try s1024.setVoxelAtGlobal(addr, true);
    const val = try s1024.getVoxelAtGlobal(addr);
    try std.testing.expect(val == true);
    
    const other_addr = address.encode(.{
        .world = 0, .px = 1, .py = 1, .pz = 1, .lx = 11, .ly = 11, .lz = 11,
    });
    try std.testing.expect((try s1024.getVoxelAtGlobal(other_addr)) == false);
}
