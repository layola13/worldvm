const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");
const physics_tests = @import("physics_tests.zig");

pub const MAX_CHAPTER_INSTANCES: usize = 32;

pub const InstanceSnapshot = struct {
    entity_id: u16,
    pos_x: i32,
    pos_y: i32,
    pos_z: i32,
    vel_x: i16,
    vel_y: i16,
    vel_z: i16,
    ang_x: i8,
    ang_y: i8,
    ang_z: i8,
    state: scene32.InstanceState,
};

pub const SceneSnapshot = struct {
    test_id: u32,
    name: []const u8,
    ticks_run: u32,
    stable: bool,
    instance_count: u8,
    instances: [MAX_CHAPTER_INSTANCES]InstanceSnapshot,

    pub fn instance(self: *const SceneSnapshot, idx: usize) *const InstanceSnapshot {
        return &self.instances[idx];
    }
};

pub fn makeInstance(entity_id: u8, x: i32, y: i32, z: i32, state: scene32.InstanceState) scene32.Instance {
    return .{
        .entity_id = entity_id,
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = state,
        .sleep_tick = 0,
        ._reserved = .{0} ** 2,
    };
}

pub fn runScenario(
    allocator: std.mem.Allocator,
    test_id: u32,
    name: []const u8,
    setup_fn: *const fn ([]entity16.Entity16, *[MAX_CHAPTER_INSTANCES]scene32.Instance) u8,
) !SceneSnapshot {
    physics_tests.createTestEntities();

    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();

    _ = try s1024.getPage(0);

    var test_instances: [MAX_CHAPTER_INSTANCES]scene32.Instance = undefined;
    @memset(@as([*]u8, @ptrCast(&test_instances))[0..@sizeOf(@TypeOf(test_instances))], 0);
    const instance_count = setup_fn(&physics_tests.test_entities, &test_instances);

    for (0..instance_count) |i| {
        _ = try s1024.addInstance(test_instances[i]);
    }

    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &s1024, &physics_tests.test_entities);

    var ticks: u32 = 0;
    const max_ticks: u32 = 100;

    while (ticks < max_ticks and !engine.stable) : (ticks += 1) {
        _ = tick_engine.stepTick(&engine);
    }

    var snapshot = SceneSnapshot{
        .test_id = test_id,
        .name = name,
        .ticks_run = ticks,
        .stable = engine.stable,
        .instance_count = @intCast(s1024.instance_count),
        .instances = undefined,
    };

    for (0..s1024.instance_count) |i| {
        const inst = s1024.instances[i];
        snapshot.instances[i] = .{
            .entity_id = inst.entity_id,
            .pos_x = inst.pos_x,
            .pos_y = inst.pos_y,
            .pos_z = inst.pos_z,
            .vel_x = inst.vel_x,
            .vel_y = inst.vel_y,
            .vel_z = inst.vel_z,
            .ang_x = inst.ang_x,
            .ang_y = inst.ang_y,
            .ang_z = inst.ang_z,
            .state = inst.state,
        };
    }

    return snapshot;
}

pub fn assertMoved(snapshot: *const SceneSnapshot, idx: usize, initial_x: i32, initial_y: i32, initial_z: i32) !void {
    const inst = snapshot.instance(idx);
    try std.testing.expect(inst.pos_x != initial_x or inst.pos_y != initial_y or inst.pos_z != initial_z);
}

pub fn assertVelocityChanged(snapshot: *const SceneSnapshot, idx: usize) !void {
    const inst = snapshot.instance(idx);
    try std.testing.expect(inst.vel_x != 0 or inst.vel_y != 0 or inst.vel_z != 0);
}

pub fn assertStateEq(snapshot: *const SceneSnapshot, idx: usize, expected: scene32.InstanceState) !void {
    try std.testing.expectEqual(expected, snapshot.instance(idx).state);
}

pub fn assertApproxEqI32(actual: i32, expected: i32, tolerance: i32) !void {
    const delta = @abs(actual - expected);
    try std.testing.expect(delta <= tolerance);
}

pub fn skipUnsupported(reason: []const u8) !void {
    return std.testing.skip(reason);
}
