const std = @import("std");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");

test "Scene1024 paging and LRU" {
    const allocator = std.testing.allocator;
    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();

    // Load first page
    const p1 = try s1024.getPage(1);
    try std.testing.expect(p1.page_id == 1);
    try std.testing.expect(s1024.active_count == 1);

    // Load 16 pages
    for (2..17) |i| {
        _ = try s1024.getPage(@intCast(i));
    }
    try std.testing.expect(s1024.active_count == 16);

    // Update global tick to test LRU
    s1024.global_tick = 100;
    
    // Access page 1 again to make it fresh
    _ = try s1024.getPage(1);
    
    // Load page 17, should trigger LRU
    // Page 2 was the second oldest (if all had tick 0), 
    // but we didn't specify tick for others.
    // In our implementation, they all had tick 0 except page 1 which is now 100.
    // So page 2 (at index 1) will likely be replaced.
    _ = try s1024.getPage(17);
    
    // Check if page 1 is still there
    var found_p1 = false;
    for (0..16) |i| {
        if (s1024.pages[i].resident and s1024.pages[i].page_id == 1) {
            found_p1 = true;
            break;
        }
    }
    try std.testing.expect(found_p1);

    // Check if page 17 is there
    var found_p17 = false;
    for (0..16) |i| {
        if (s1024.pages[i].resident and s1024.pages[i].page_id == 17) {
            found_p17 = true;
            break;
        }
    }
    try std.testing.expect(found_p17);
}
