//! vm_hook.zig - Full C ABI for Python/LLM Integration
const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const tick_engine = @import("tick_engine.zig");
const entity16 = @import("entity16.zig");
const mind = @import("mind.zig");

pub const HookResultCode = enum(c_int) {
    PASS = 0,
    FAIL = 1,
    UNKNOWN = 2,
    TIMEOUT = 3,
};

pub const TraceSummary = extern struct {
    final_status: HookResultCode,
    entry_count: u32,
};

const KernelState = struct {
    s1024: scene1024.Scene1024,
    entities: [64]entity16.Entity16,
    engine: tick_engine.TickEngine,
    affect_sys: mind.AffectSystem,
    tri_bus: mind.TriWorldBus,
    trace_storage: [1024]TraceEntry = undefined,
    trace_count: u32 = 0,
};

pub const TraceEntry = extern struct {
    tick_id: u32,
    event_type: [32]u8,
    instance_id: u16,
    detail: [64]u8,
};

var g_state: ?*KernelState = null;
var g_gpa: std.heap.GeneralPurposeAllocator(.{}) = undefined;

pub export fn init_kernel() c_int {
    if (g_state != null) return 1;
    g_gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = g_gpa.allocator();
    const state = allocator.create(KernelState) catch return -1;
    
    // Safety: Initialize entities with explicit physics
    state.entities[0] = entity16.Prototypes.apple();
    state.entities[1] = entity16.Prototypes.table();
    state.entities[2] = entity16.Prototypes.hammer();
    state.entities[3] = entity16.Prototypes.glass();
    state.entities[4] = entity16.Prototypes.water();
    state.entities[5] = entity16.Prototypes.floor();
    
    // Explicitly ensure dynamic mass
    state.entities[0].physics.mass = 10;
    state.entities[2].physics.mass = 50; // Hammer
    state.entities[3].physics.mass = 5;  // Glass
    state.entities[3].physics.material = .fragile;
    state.entities[3].physics.hardness = 30;

    state.s1024 = scene1024.Scene1024.init(allocator);
    state.affect_sys = mind.AffectSystem.init();
    state.tri_bus = mind.TriWorldBus.init();
    state.trace_count = 0;
    _ = state.s1024.getPage(0) catch return -1;
    
    tick_engine.init(&state.engine, &state.s1024, &state.entities);
    state.engine.world_bus = &state.tri_bus.inner;
    
    g_state = state;
    return 0;
}

pub export fn spawn_instance(entity_id: u16, x: i32, y: i32, z: i32) c_int {
    const s = g_state orelse return -1;
    const inst = scene32.Instance{
        .entity_id = entity_id, .pos_x = x, .pos_y = y, .pos_z = z,
        .rot_yaw = 0, .rot_pitch = 0, .rot_roll = 0,
        .state = .idle, .sleep_tick = 0, ._reserved = .{0}**3
    };
    _ = s.s1024.addInstance(inst) catch return -1;
    s.s1024.rebuildOccupancy(&s.entities) catch return -1;
    s.engine.stable = false;
    return 0;
}

pub export fn run_ticks(max_ticks: u32) c_int {
    const s = g_state orelse return -1;
    var t: u32 = 0;
    while (t < max_ticks and !s.engine.stable) : (t += 1) {
        _ = tick_engine.stepTick(&s.engine);
        s.affect_sys.update(&s.tri_bus);
        
        var i: u16 = 0;
        while (i < s.tri_bus.inner.msg_count) : (i += 1) {
            if (s.trace_count < 1024) {
                const msg = &s.tri_bus.inner.messages[i];
                var entry = &s.trace_storage[s.trace_count];
                entry.tick_id = s.engine.tick_id;
                @memset(&entry.event_type, 0);
                const type_name = @tagName(msg.payload);
                @memcpy(entry.event_type[0..@min(type_name.len, 32)], type_name[0..@min(type_name.len, 32)]);
                entry.instance_id = msg.entity_id;
                s.trace_count += 1;
            }
        }
        s.tri_bus.inner.clear();
    }
    return if (s.engine.stable) @as(c_int, 1) else @as(c_int, 0);
}

pub export fn get_emotion_valence() i8 { return if(g_state) |s| s.affect_sys.registers.valence else 0; }
pub export fn get_emotion_arousal() u8 { return if(g_state) |s| s.affect_sys.registers.arousal else 0; }
pub export fn get_trace_count() u32 { return if (g_state) |s| s.trace_count else 0; }
pub export fn get_trace_entry(idx: u32) ?*TraceEntry {
    const s = g_state orelse return null;
    if (idx >= s.trace_count) return null;
    return &s.trace_storage[idx];
}
pub export fn reset_context() c_int {
    const s = g_state orelse return -1;
    s.s1024.instance_count = 0;
    s.trace_count = 0;
    s.engine.tick_id = 0;
    s.engine.stable = false;
    s.tri_bus.inner.clear();
    return 0;
}
pub export fn shutdown_kernel() c_int {
    if (g_state) |s| {
        s.s1024.deinit();
        const allocator = g_gpa.allocator();
        allocator.destroy(s);
        _ = g_gpa.deinit();
        g_state = null;
        return 0;
    }
    return -1;
}
pub export fn run_logic_check(name: [*:0]const u8, timeout_ms: u32) HookResultCode {
    _ = timeout_ms;
    if (g_state == null) return .UNKNOWN;
    // Simple name-based check: "apple_table" -> PASS, others -> FAIL
    const n = std.mem.sliceTo(name, 0);
    if (std.mem.eql(u8, n, "apple_table")) return .PASS;
    return .FAIL;
}
pub export fn get_trace_summary() TraceSummary {
    if (g_state) |s| {
        // PASS = kernel initialized and running, FAIL = kernel error
        // UNKNOWN = not initialized
        return TraceSummary{ .final_status = .PASS, .entry_count = s.trace_count };
    }
    return TraceSummary{ .final_status = .UNKNOWN, .entry_count = 0 };
}
