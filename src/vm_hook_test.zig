const std = @import("std");
const vm_hook = @import("vm_hook.zig");

test "HookResultCode values match C int" {
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.PASS) == 0);
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.FAIL) == 1);
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.UNKNOWN) == 2);
    try std.testing.expect(@intFromEnum(vm_hook.HookResultCode.TIMEOUT) == 3);
}

test "reset_context returns -1 when kernel not initialized" {
    _ = vm_hook.shutdown_kernel();
    const rc = vm_hook.reset_context();
    try std.testing.expect(rc == -1);
}

test "init_kernel returns 0 on first call" {
    _ = vm_hook.shutdown_kernel();
    const r = vm_hook.init_kernel();
    try std.testing.expect(r == 0);
    _ = vm_hook.shutdown_kernel();
}

test "init_kernel returns 1 when already initialized" {
    _ = vm_hook.shutdown_kernel();
    _ = vm_hook.init_kernel();
    const r = vm_hook.init_kernel();
    try std.testing.expect(r == 1);
    _ = vm_hook.shutdown_kernel();
}

test "run_logic_check returns UNKNOWN when kernel not initialized" {
    _ = vm_hook.shutdown_kernel();
    const name_with_null: [*:0]const u8 = "apple_table";
    const r = vm_hook.run_logic_check(name_with_null, 50);
    try std.testing.expect(r == .UNKNOWN);
}

test "run_logic_check returns PASS or FAIL when kernel initialized" {
    _ = vm_hook.shutdown_kernel();
    _ = vm_hook.init_kernel();
    const name_with_null: [*:0]const u8 = "apple_table";
    const r = vm_hook.run_logic_check(name_with_null, 50);
    try std.testing.expect(r == .PASS or r == .FAIL);
    _ = vm_hook.shutdown_kernel();
}

test "get_trace_summary returns UNKNOWN when kernel not initialized" {
    _ = vm_hook.shutdown_kernel();
    const summary = vm_hook.get_trace_summary();
    try std.testing.expect(summary.final_status == .UNKNOWN);
    try std.testing.expect(summary.entry_count == 0);
}

test "get_trace_summary returns valid data after init" {
    _ = vm_hook.shutdown_kernel();
    _ = vm_hook.init_kernel();
    const summary = vm_hook.get_trace_summary();
    try std.testing.expect(summary.final_status == .PASS or summary.final_status == .FAIL);
    _ = vm_hook.shutdown_kernel();
}

test "shutdown_kernel returns 0 after init" {
    _ = vm_hook.init_kernel();
    const r = vm_hook.shutdown_kernel();
    try std.testing.expect(r == 0);
}

test "shutdown_kernel returns -1 when not initialized" {
    _ = vm_hook.shutdown_kernel();
    const r = vm_hook.shutdown_kernel();
    try std.testing.expect(r == -1);
}
