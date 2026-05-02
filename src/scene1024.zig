//! Scene1024: 1024^3 Virtual Paged Scene
const std = @import("std");
const scene32 = @import("scene32.zig");
const physics = @import("physics.zig");
const address = @import("address.zig");
const entity16 = @import("entity16.zig");

pub const MAX_ACTIVE_PAGES = 16;
pub const PAGE_SIZE_VOXELS = 32;
pub const INVALID_INSTANCE_HANDLE: u32 = 0;

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
    instances: [scene32.MAX_INSTANCES]scene32.Instance,
    instance_handles: [scene32.MAX_INSTANCES]u32,
    instance_count: u8,
    next_instance_handle: u32,

    pub fn init(allocator: std.mem.Allocator) Scene1024 {
        var s = Scene1024{
            .pages = undefined,
            .active_count = 0,
            .global_tick = 0,
            .allocator = allocator,
            .instances = undefined,
            .instance_handles = undefined,
            .instance_count = 0,
            .next_instance_handle = 1,
        };
        @memset(&s.instances, std.mem.zeroes(scene32.Instance));
        @memset(&s.instance_handles, INVALID_INSTANCE_HANDLE);
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
        if (self.instance_count >= scene32.MAX_INSTANCES) return error.TooManyInstances;
        const idx = self.instance_count;
        self.instances[idx] = inst;
        self.instance_handles[idx] = try self.allocateInstanceHandle();
        self.instance_count += 1;
        return idx;
    }

    pub fn removeInstance(self: *Scene1024, idx: u8) bool {
        if (idx >= self.instance_count) return false;
        const last_idx = self.instance_count - 1;
        if (idx != last_idx) {
            self.instances[idx] = self.instances[last_idx];
            self.instance_handles[idx] = self.instance_handles[last_idx];
        }
        self.instances[last_idx] = std.mem.zeroes(scene32.Instance);
        self.instance_handles[last_idx] = INVALID_INSTANCE_HANDLE;
        self.instance_count -= 1;
        return true;
    }

    pub fn getInstanceHandle(self: *const Scene1024, idx: u8) u32 {
        if (idx >= self.instance_count) return INVALID_INSTANCE_HANDLE;
        return self.instance_handles[idx];
    }

    pub fn resolveInstanceHandle(self: *const Scene1024, handle: u32) ?u8 {
        if (handle == INVALID_INSTANCE_HANDLE) return null;
        var i: u8 = 0;
        while (i < self.instance_count) : (i += 1) {
            if (self.instance_handles[i] == handle) return i;
        }
        return null;
    }

    pub fn getInstanceByHandle(self: *const Scene1024, handle: u32) ?*const scene32.Instance {
        const idx = self.resolveInstanceHandle(handle) orelse return null;
        return &self.instances[idx];
    }

    pub fn removeInstanceByHandle(self: *Scene1024, handle: u32) bool {
        const idx = self.resolveInstanceHandle(handle) orelse return false;
        return self.removeInstance(idx);
    }

    pub fn markInstanceBroken(self: *Scene1024, idx: u8) bool {
        if (idx >= self.instance_count) return false;
        self.instances[idx].state = .broken;
        self.instances[idx].vel_x = 0;
        self.instances[idx].vel_y = 0;
        self.instances[idx].vel_z = 0;
        self.instances[idx].ang_x = 0;
        self.instances[idx].ang_y = 0;
        self.instances[idx].ang_z = 0;
        return true;
    }

    pub fn markInstanceBrokenByHandle(self: *Scene1024, handle: u32) bool {
        const idx = self.resolveInstanceHandle(handle) orelse return false;
        return self.markInstanceBroken(idx);
    }

    pub fn compactBrokenInstances(self: *Scene1024) u8 {
        var removed: u8 = 0;
        var i: u8 = 0;
        while (i < self.instance_count) {
            if (self.instances[i].state == .broken) {
                _ = self.removeInstance(i);
                removed += 1;
                continue;
            }
            i += 1;
        }
        return removed;
    }

    pub fn rebuildOccupancy(self: *Scene1024, entities: []const entity16.Entity16) !void {
        // Static environment voxels live in paged Scene32 data and must remain
        // distinct from dynamic instance occupancy. Instance collision queries
        // already walk `self.instances` directly, so rebuilding should not erase
        // or overwrite page occupancy here.
        _ = self;
        _ = entities;
    }

    /// Return an instance by live index with bounds checking.
    pub fn getInstance(self: *const Scene1024, idx: u8) ?*scene32.Instance {
        if (idx >= self.instance_count) return null;
        return &self.instances[idx];
    }

    fn allocateInstanceHandle(self: *Scene1024) !u32 {
        var candidate = self.next_instance_handle;
        if (candidate == INVALID_INSTANCE_HANDLE) candidate = 1;

        var attempts: usize = 0;
        while (attempts <= scene32.MAX_INSTANCES) : (attempts += 1) {
            if (candidate == INVALID_INSTANCE_HANDLE) candidate = 1;
            if (self.resolveInstanceHandle(candidate) == null) {
                self.next_instance_handle = candidate +% 1;
                if (self.next_instance_handle == INVALID_INSTANCE_HANDLE) {
                    self.next_instance_handle = 1;
                }
                return candidate;
            }
            candidate +%= 1;
        }

        return error.InstanceHandleExhausted;
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
        .world = 0,
        .px = 1,
        .py = 1,
        .pz = 1,
        .lx = 10,
        .ly = 10,
        .lz = 10,
    });

    try s1024.setVoxelAtGlobal(addr, true);
    const val = try s1024.getVoxelAtGlobal(addr);
    try std.testing.expect(val == true);

    const other_addr = address.encode(.{
        .world = 0,
        .px = 1,
        .py = 1,
        .pz = 1,
        .lx = 11,
        .ly = 11,
        .lz = 11,
    });
    try std.testing.expect((try s1024.getVoxelAtGlobal(other_addr)) == false);
}

test "Scene1024 addInstance stores instances and enforces capacity" {
    var s1024 = Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 7;
    inst.pos_x = 10;
    inst.pos_y = 20;
    inst.pos_z = 30;
    inst.state = .moving;

    const first_idx = try s1024.addInstance(inst);
    try std.testing.expectEqual(@as(u8, 0), first_idx);
    try std.testing.expectEqual(@as(u8, 1), s1024.instance_count);
    try std.testing.expectEqual(inst, s1024.instances[first_idx]);

    while (s1024.instance_count < scene32.MAX_INSTANCES) {
        inst.entity_id = s1024.instance_count;
        _ = try s1024.addInstance(inst);
    }

    try std.testing.expectEqual(error.TooManyInstances, s1024.addInstance(inst));
    try std.testing.expectEqual(@as(u8, scene32.MAX_INSTANCES), s1024.instance_count);
}

test "Scene1024 removeInstance compacts live instances and rejects invalid index" {
    var s1024 = Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var first = std.mem.zeroes(scene32.Instance);
    first.entity_id = 1;
    first.pos_x = 10;
    var second = std.mem.zeroes(scene32.Instance);
    second.entity_id = 2;
    second.pos_x = 20;
    var third = std.mem.zeroes(scene32.Instance);
    third.entity_id = 3;
    third.pos_x = 30;

    _ = try s1024.addInstance(first);
    _ = try s1024.addInstance(second);
    _ = try s1024.addInstance(third);
    const second_handle = s1024.getInstanceHandle(1);
    const third_handle = s1024.getInstanceHandle(2);

    try std.testing.expect(s1024.removeInstance(1));
    try std.testing.expectEqual(@as(u8, 2), s1024.instance_count);
    try std.testing.expectEqual(@as(u16, 1), s1024.instances[0].entity_id);
    try std.testing.expectEqual(@as(u16, 3), s1024.instances[1].entity_id);
    try std.testing.expectEqual(@as(u16, 0), s1024.instances[2].entity_id);
    try std.testing.expectEqual(@as(?u8, null), s1024.resolveInstanceHandle(second_handle));
    try std.testing.expectEqual(@as(?u8, 1), s1024.resolveInstanceHandle(third_handle));
    try std.testing.expect(!s1024.removeInstance(2));
}

test "Scene1024 mark and compact broken instances" {
    var s1024 = Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 1;
    inst.state = .moving;
    inst.vel_x = 5;
    inst.ang_y = 2;
    _ = try s1024.addInstance(inst);
    inst.entity_id = 2;
    _ = try s1024.addInstance(inst);
    inst.entity_id = 3;
    _ = try s1024.addInstance(inst);

    try std.testing.expect(s1024.markInstanceBroken(1));
    try std.testing.expectEqual(scene32.InstanceState.broken, s1024.instances[1].state);
    try std.testing.expectEqual(@as(i16, 0), s1024.instances[1].vel_x);
    try std.testing.expectEqual(@as(i8, 0), s1024.instances[1].ang_y);

    try std.testing.expectEqual(@as(u8, 1), s1024.compactBrokenInstances());
    try std.testing.expectEqual(@as(u8, 2), s1024.instance_count);
    try std.testing.expectEqual(@as(u16, 1), s1024.instances[0].entity_id);
    try std.testing.expectEqual(@as(u16, 3), s1024.instances[1].entity_id);
}

test "Scene1024 stable handles survive moved instances and reject stale handles" {
    var s1024 = Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 10;
    _ = try s1024.addInstance(inst);
    inst.entity_id = 20;
    _ = try s1024.addInstance(inst);
    inst.entity_id = 30;
    _ = try s1024.addInstance(inst);

    const first_handle = s1024.getInstanceHandle(0);
    const second_handle = s1024.getInstanceHandle(1);
    const third_handle = s1024.getInstanceHandle(2);
    try std.testing.expect(first_handle != INVALID_INSTANCE_HANDLE);
    try std.testing.expect(second_handle != INVALID_INSTANCE_HANDLE);
    try std.testing.expect(third_handle != INVALID_INSTANCE_HANDLE);
    try std.testing.expect(first_handle != second_handle);
    try std.testing.expect(second_handle != third_handle);

    try std.testing.expect(s1024.removeInstanceByHandle(second_handle));
    try std.testing.expectEqual(@as(?u8, null), s1024.resolveInstanceHandle(second_handle));
    try std.testing.expectEqual(@as(?u8, 1), s1024.resolveInstanceHandle(third_handle));
    try std.testing.expectEqual(@as(u16, 30), s1024.getInstanceByHandle(third_handle).?.entity_id);
    try std.testing.expect(!s1024.removeInstanceByHandle(second_handle));

    try std.testing.expect(s1024.markInstanceBrokenByHandle(third_handle));
    try std.testing.expectEqual(scene32.InstanceState.broken, s1024.getInstanceByHandle(third_handle).?.state);
    try std.testing.expectEqual(@as(u8, 1), s1024.compactBrokenInstances());
    try std.testing.expectEqual(@as(?u8, null), s1024.resolveInstanceHandle(third_handle));
    try std.testing.expectEqual(@as(?u8, 0), s1024.resolveInstanceHandle(first_handle));
}

test "Scene1024 rebuildOccupancy preserves static pages and dynamic instances" {
    var s1024 = Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    const addr = address.encode(.{
        .world = 0,
        .px = 2,
        .py = 3,
        .pz = 4,
        .lx = 5,
        .ly = 6,
        .lz = 7,
    });
    try s1024.setVoxelAtGlobal(addr, true);

    var inst = std.mem.zeroes(scene32.Instance);
    inst.entity_id = 1;
    inst.pos_x = 42;
    inst.pos_y = 43;
    inst.pos_z = 44;
    _ = try s1024.addInstance(inst);

    const entities = [_]entity16.Entity16{entity16.initEntity16()};
    try s1024.rebuildOccupancy(&entities);

    try std.testing.expect(try s1024.getVoxelAtGlobal(addr));
    try std.testing.expectEqual(@as(u8, 1), s1024.instance_count);
    try std.testing.expectEqual(inst, s1024.instances[0]);
}

test "Scene1024 getPage reuses resident pages and evicts least recently used slot" {
    var s1024 = Scene1024.init(std.testing.allocator);
    defer s1024.deinit();

    for (0..MAX_ACTIVE_PAGES) |page_id| {
        s1024.global_tick = @intCast(page_id + 1);
        const page = try s1024.getPage(@intCast(page_id));
        try std.testing.expectEqual(@as(u32, @intCast(page_id)), page.page_id);
        try std.testing.expectEqual(@as(u32, @intCast(page_id)), page.scene.?.scene_id);
    }

    try std.testing.expectEqual(@as(u8, MAX_ACTIVE_PAGES), s1024.active_count);

    s1024.global_tick = 100;
    const refreshed = try s1024.getPage(0);
    try std.testing.expectEqual(@as(u32, 100), refreshed.last_tick);

    s1024.global_tick = 101;
    const loaded = try s1024.getPage(999);
    try std.testing.expectEqual(@as(u32, 999), loaded.page_id);
    try std.testing.expectEqual(@as(u32, 999), loaded.scene.?.scene_id);
    try std.testing.expectEqual(@as(u8, MAX_ACTIVE_PAGES), s1024.active_count);

    var has_page_0 = false;
    var has_page_1 = false;
    var has_page_999 = false;
    for (s1024.pages) |page| {
        if (!page.resident) continue;
        has_page_0 = has_page_0 or page.page_id == 0;
        has_page_1 = has_page_1 or page.page_id == 1;
        has_page_999 = has_page_999 or page.page_id == 999;
    }
    try std.testing.expect(has_page_0);
    try std.testing.expect(!has_page_1);
    try std.testing.expect(has_page_999);
}
