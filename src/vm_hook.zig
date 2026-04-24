//! vm_hook.zig - C ABI Hook Interface for World VM
//! Provides: init_kernel, run_logic_check, run_logic_check_with_timeout, get_trace_summary, reset_context
//! Used for LLM integration via Tool Call and Hook modes.
//! Default timeout is 30 seconds (30000ms) for run_logic_check.

const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const tick_engine = @import("tick_engine.zig");
const entity16 = @import("entity16.zig");
const scenarios = @import("scenarios.zig");

pub const HOOK_VERSION = "1.0.0";

pub const HookResultCode = enum(c_int) {
    PASS = 0,
    FAIL = 1,
    UNKNOWN = 2,
    TIMEOUT = 3,
};

pub const TraceEntry = struct {
    tick_id: u32,
    event_type: [*:0]const u8,
    instance_id: u16,
    detail: [*:0]const u8,
};

// Fixed-size for C ABI compatibility
pub const TraceSummary = extern struct {
    entries: ?[*]TraceEntry,
    entry_count: u32,
    total_ticks: u32,
    final_status: HookResultCode,
};

const KernelState = struct {
    s1024: scene1024.Scene1024,
    entities: [64]entity16.Entity16,
    engine: tick_engine.TickEngine,
    active_scene: *scene32.Scene32,
    trace_buf: std.ArrayListUnmanaged(TraceEntry) = .{},
    init_ok: bool = false,
};

var g_state: ?*KernelState = null;
var g_gpa: std.heap.GeneralPurposeAllocator(.{}) = undefined;
var g_allocator: std.mem.Allocator = undefined;

pub export fn init_kernel() c_int {
    if (g_state != null) return 1;

    g_gpa = std.heap.GeneralPurposeAllocator(.{}){};
    g_allocator = g_gpa.allocator();

    const state = g_allocator.create(KernelState) catch return -1;
    errdefer g_allocator.destroy(state);

    state.* = .{
        .s1024 = scene1024.Scene1024.init(g_allocator),
        .entities = undefined,
        .engine = undefined,
        .active_scene = undefined,
        .init_ok = false,
    };

    const entry = state.s1024.getPage(0) catch {
        state.s1024.deinit();
        g_allocator.destroy(state);
        return -1;
    };
    state.active_scene = entry.scene.?;
    scenarios.setupScenario(.apple_table, &state.s1024, &state.entities);
    tick_engine.init(&state.engine, &state.s1024, &state.entities);
    state.init_ok = true;
    g_state = state;
    return 0;
}

pub const DEFAULT_TIMEOUT_MS: u32 = 30000; // 30 seconds

pub export fn run_logic_check(scenario_name: [*:0]const u8, max_ticks: u32) HookResultCode {
    return run_logic_check_with_timeout(scenario_name, max_ticks, DEFAULT_TIMEOUT_MS);
}

pub export fn run_logic_check_with_timeout(scenario_name: [*:0]const u8, max_ticks: u32, max_time_ms: u32) HookResultCode {
    const state = g_state orelse return .UNKNOWN;

    const name = std.mem.sliceTo(scenario_name, 0);
    const scenario: scenarios.Scenario = if (std.mem.eql(u8, name, "hammer_glass")) .hammer_glass
        else if (std.mem.eql(u8, name, "water_flow")) .water_flow
        else .apple_table;

    scenarios.setupScenario(scenario, &state.s1024, &state.entities);

    state.engine.tick_id = 0;
    state.engine.stable = false;
    state.trace_buf.clearRetainingCapacity();

    const start_ns = std.time.nanoTimestamp();
    const timeout_ns = @as(i128, max_time_ms) * 1000000;

    var t: u32 = 0;
    while (t < max_ticks and !state.engine.stable) : (t += 1) {
        _ = tick_engine.stepTick(&state.engine);

        // Check timeout
        const elapsed_ns = std.time.nanoTimestamp() - start_ns;
        if (elapsed_ns > timeout_ns) {
            return .TIMEOUT;
        }
    }

    return if (state.engine.stable) .PASS else .FAIL;
}

pub export fn get_trace_summary() TraceSummary {
    if (g_state) |state| {
        return .{
            .entries = state.trace_buf.items.ptr,
            .entry_count = @intCast(state.trace_buf.items.len),
            .total_ticks = state.engine.tick_id,
            .final_status = if (state.engine.stable) .PASS else .FAIL,
        };
    }
    return .{ .entries = null, .entry_count = 0, .total_ticks = 0, .final_status = .UNKNOWN };
}

pub export fn reset_context() c_int {
    const state = g_state orelse return -1;
    scene32.clearScene(state.active_scene);
    scenarios.setupScenario(.apple_table, &state.s1024, &state.entities);
    state.engine.tick_id = 0;
    state.engine.stable = false;
    state.trace_buf.clearRetainingCapacity();
    return 0;
}

pub export fn shutdown_kernel() c_int {
    const state = g_state orelse return -1;
    state.s1024.deinit();
    state.trace_buf.deinit(g_allocator);
    g_allocator.destroy(state);
    g_state = null;
    _ = g_gpa.deinit();
    return 0;
}
