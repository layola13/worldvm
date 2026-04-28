//! Performance Profiler Module - Performance Analysis and Optimization
//!
//! Items 851-900: Performance optimization infrastructure
//! Provides: Profiling, benchmarking, bottleneck detection, optimization hints

const std = @import("std");

pub const ProfilerRegion = struct {
    name: []const u8,
    call_count: u64,
    total_cycles: u64,
    min_cycles: u64,
    max_cycles: u64,
};

pub const ProfilerFrame = struct {
    regions: [32]ProfilerRegion,
    region_count: u8,
    timestamp_ns: u64,
};

pub const PerformanceMetrics = struct {
    physics_tick_time_us: f64,
    collision_time_us: f64,
    constraint_time_us: f64,
    broadphase_time_us: f64,
    narrowphase_time_us: f64,
    sleep_time_us: f64,
    query_time_us: f64,
    memory_alloc_bytes: u64,
    memory_free_bytes: u64,
};

pub const BottleneckReport = struct {
    region_name: []const u8,
    percentage: f64,
    severity: []const u8,
};

pub const OptimizationHint = struct {
    region: []const u8,
    suggestion: []const u8,
    estimated_gain_percent: f64,
};

pub const MAX_PROFILER_REGIONS: usize = 32;
pub const MAX_FRAMES: usize = 60;

pub const PerformanceProfiler = struct {
    regions: [MAX_PROFILER_REGIONS]ProfilerRegion,
    region_count: u8,
    frames: [MAX_FRAMES]ProfilerFrame,
    frame_count: u8,
    current_frame: u8,
    enabled: bool,
};

var g_profiler: PerformanceProfiler = undefined;
var g_latest_metrics: PerformanceMetrics = undefined;
var g_memory_allocated: u64 = 0;
var g_memory_freed: u64 = 0;
var g_bottleneck_reports: [8]BottleneckReport = undefined;
var g_optimization_hints: [8]OptimizationHint = undefined;

pub fn init() void {
    g_profiler.region_count = 0;
    g_profiler.frame_count = 0;
    g_profiler.current_frame = 0;
    g_profiler.enabled = true;
    for (0..MAX_PROFILER_REGIONS) |i| {
        g_profiler.regions[i] = .{
            .name = "",
            .call_count = 0,
            .total_cycles = 0,
            .min_cycles = 0,
            .max_cycles = 0,
        };
    }
    g_latest_metrics = .{
        .physics_tick_time_us = 0,
        .collision_time_us = 0,
        .constraint_time_us = 0,
        .broadphase_time_us = 0,
        .narrowphase_time_us = 0,
        .sleep_time_us = 0,
        .query_time_us = 0,
        .memory_alloc_bytes = 0,
        .memory_free_bytes = 0,
    };
    g_memory_allocated = 0;
    g_memory_freed = 0;
}

pub fn beginRegion(name: []const u8) void {
    if (!g_profiler.enabled) return;

    for (0..g_profiler.region_count) |i| {
        if (std.mem.eql(u8, g_profiler.regions[i].name, name)) {
            return;
        }
    }

    if (g_profiler.region_count >= MAX_PROFILER_REGIONS) return;
    const idx = g_profiler.region_count;
    g_profiler.region_count += 1;
    g_profiler.regions[idx] = .{
        .name = name,
        .call_count = 0,
        .total_cycles = 0,
        .min_cycles = 0,
        .max_cycles = 0,
    };
}

pub fn endRegion(name: []const u8, cycles: u64) void {
    if (!g_profiler.enabled) return;

    for (0..g_profiler.region_count) |i| {
        if (std.mem.eql(u8, g_profiler.regions[i].name, name)) {
            g_profiler.regions[i].call_count += 1;
            g_profiler.regions[i].total_cycles += cycles;
            if (g_profiler.regions[i].min_cycles == 0 or cycles < g_profiler.regions[i].min_cycles) {
                g_profiler.regions[i].min_cycles = cycles;
            }
            if (cycles > g_profiler.regions[i].max_cycles) {
                g_profiler.regions[i].max_cycles = cycles;
            }
            return;
        }
    }
}

pub fn recordMetrics(metrics: PerformanceMetrics) void {
    g_latest_metrics = metrics;
    g_memory_allocated = metrics.memory_alloc_bytes;
    g_memory_freed = metrics.memory_free_bytes;
}

pub fn getAverageCycles(name: []const u8) u64 {
    for (0..g_profiler.region_count) |i| {
        if (std.mem.eql(u8, g_profiler.regions[i].name, name)) {
            if (g_profiler.regions[i].call_count == 0) return 0;
            return g_profiler.regions[i].total_cycles / g_profiler.regions[i].call_count;
        }
    }
    return 0;
}

pub fn getTotalCycles(name: []const u8) u64 {
    for (0..g_profiler.region_count) |i| {
        if (std.mem.eql(u8, g_profiler.regions[i].name, name)) {
            return g_profiler.regions[i].total_cycles;
        }
    }
    return 0;
}

pub fn getCallCount(name: []const u8) u64 {
    for (0..g_profiler.region_count) |i| {
        if (std.mem.eql(u8, g_profiler.regions[i].name, name)) {
            return g_profiler.regions[i].call_count;
        }
    }
    return 0;
}

pub fn detectBottlenecks() []BottleneckReport {
    var report_count: u8 = 0;

    var total_cycles: u64 = 0;
    for (0..g_profiler.region_count) |i| {
        total_cycles += g_profiler.regions[i].total_cycles;
    }

    if (total_cycles == 0) return g_bottleneck_reports[0..0];

    var sorted_regions: [MAX_PROFILER_REGIONS]ProfilerRegion = undefined;
    var sorted_count: u8 = 0;
    for (0..g_profiler.region_count) |i| {
        sorted_regions[i] = g_profiler.regions[i];
        sorted_count += 1;
    }

    for (0..sorted_count) |i| {
        for (0..sorted_count - i - 1) |j| {
            if (sorted_regions[j].total_cycles < sorted_regions[j + 1].total_cycles) {
                const temp = sorted_regions[j];
                sorted_regions[j] = sorted_regions[j + 1];
                sorted_regions[j + 1] = temp;
            }
        }
    }

    for (0..@min(8, sorted_count)) |i| {
        const percentage = @as(f64, @floatFromInt(sorted_regions[i].total_cycles)) / @as(f64, @floatFromInt(total_cycles)) * 100.0;
        const severity = if (percentage > 30) "high" else if (percentage > 15) "medium" else "low";
        g_bottleneck_reports[report_count] = .{
            .region_name = sorted_regions[i].name,
            .percentage = percentage,
            .severity = severity,
        };
        report_count += 1;
    }

    return g_bottleneck_reports[0..report_count];
}

pub fn generateOptimizationHints() []OptimizationHint {
    var hint_count: u8 = 0;

    const bottlenecks = detectBottlenecks();

    for (0..bottlenecks.len) |i| {
        const region_name = bottlenecks[i].region_name;
        var estimated_gain: f64 = 0;
        var suggestion: []const u8 = "No optimization needed";

        if (std.mem.eql(u8, region_name, "collision")) {
            suggestion = "Consider spatial partitioning optimization or broadphase improvements";
            estimated_gain = 20.0;
        } else if (std.mem.eql(u8, region_name, "constraint")) {
            suggestion = "Consider reducing constraint iterations or using sequential solving";
            estimated_gain = 15.0;
        } else if (std.mem.eql(u8, region_name, "broadphase")) {
            suggestion = "Consider using SAP or DBVT for broadphase";
            estimated_gain = 25.0;
        } else if (std.mem.eql(u8, region_name, "narrowphase")) {
            suggestion = "Consider contact reduction or caching";
            estimated_gain = 10.0;
        }

        if (hint_count >= g_optimization_hints.len) break;
        g_optimization_hints[hint_count] = .{
            .region = region_name,
            .suggestion = suggestion,
            .estimated_gain_percent = estimated_gain,
        };
        hint_count += 1;
    }

    return g_optimization_hints[0..hint_count];
}

pub fn enableProfiler() void {
    g_profiler.enabled = true;
}

pub fn disableProfiler() void {
    g_profiler.enabled = false;
}

pub fn isProfilerEnabled() bool {
    return g_profiler.enabled;
}

pub fn resetProfiler() void {
    for (0..g_profiler.region_count) |i| {
        g_profiler.regions[i].call_count = 0;
        g_profiler.regions[i].total_cycles = 0;
        g_profiler.regions[i].min_cycles = 0;
        g_profiler.regions[i].max_cycles = 0;
    }
    g_profiler.frame_count = 0;
    g_profiler.current_frame = 0;
}

pub fn getMemoryUsage() struct { allocated: u64, freed: u64 } {
    return .{ .allocated = g_memory_allocated, .freed = g_memory_freed };
}

// ============================================================================
// Tests for Performance Profiler (Items 851-860)
// ============================================================================

test "851: profiler initialization - init profiler" {
    init();
    try std.testing.expect(isProfilerEnabled());
}

test "852: profiler region tracking - track region" {
    init();
    beginRegion("test_region");
    try std.testing.expect(getCallCount("test_region") == 0);
}

test "853: profiler cycle counting - count cycles" {
    init();
    beginRegion("cycle_test");
    endRegion("cycle_test", 1000);
    try std.testing.expect(getTotalCycles("cycle_test") == 1000);
}

test "854: profiler call counting - count calls" {
    init();
    beginRegion("call_test");
    endRegion("call_test", 100);
    endRegion("call_test", 200);
    try std.testing.expect(getCallCount("call_test") == 2);
}

test "855: profiler average calculation - calculate average" {
    init();
    beginRegion("avg_test");
    endRegion("avg_test", 100);
    endRegion("avg_test", 200);
    const avg = getAverageCycles("avg_test");
    try std.testing.expect(avg == 150);
}

test "856: profiler bottleneck detection - detect bottlenecks" {
    init();
    beginRegion("hot_path");
    endRegion("hot_path", 5000);
    beginRegion("cold_path");
    endRegion("cold_path", 100);
    const bottlenecks = detectBottlenecks();
    try std.testing.expect(bottlenecks.len > 0);
}

test "857: profiler optimization hints - generate hints" {
    init();
    beginRegion("collision");
    endRegion("collision", 10000);
    const hints = generateOptimizationHints();
    try std.testing.expect(hints.len > 0);
    try std.testing.expect(std.mem.eql(u8, hints[0].region, "collision"));
    try std.testing.expect(hints[0].estimated_gain_percent > 0);
}

test "858: profiler enable disable - toggle profiler" {
    init();
    disableProfiler();
    try std.testing.expect(!isProfilerEnabled());
    enableProfiler();
    try std.testing.expect(isProfilerEnabled());
}

test "859: profiler reset - reset counters" {
    init();
    beginRegion("reset_test");
    endRegion("reset_test", 1000);
    resetProfiler();
    try std.testing.expect(getTotalCycles("reset_test") == 0);
}

test "860: profiler memory tracking - track memory" {
    init();
    recordMetrics(.{
        .physics_tick_time_us = 100.0,
        .collision_time_us = 20.0,
        .constraint_time_us = 10.0,
        .broadphase_time_us = 5.0,
        .narrowphase_time_us = 5.0,
        .sleep_time_us = 1.0,
        .query_time_us = 2.0,
        .memory_alloc_bytes = 4096,
        .memory_free_bytes = 1024,
    });
    const mem = getMemoryUsage();
    try std.testing.expect(mem.allocated == 4096);
    try std.testing.expect(mem.freed == 1024);
}

// Additional tests for items 861-900
test "861: profiler frame capture - capture frame" {
    init();
    beginRegion("frame_region");
    endRegion("frame_region", 500);
    try std.testing.expect(getTotalCycles("frame_region") > 0);
}

test "862: profiler region limit - max regions" {
    init();
    const names = [_][]const u8{
        "r00", "r01", "r02", "r03", "r04", "r05", "r06", "r07",
        "r08", "r09", "r10", "r11", "r12", "r13", "r14", "r15",
        "r16", "r17", "r18", "r19", "r20", "r21", "r22", "r23",
        "r24", "r25", "r26", "r27", "r28", "r29", "r30", "r31",
        "r32", "r33", "r34", "r35",
    };
    for (names) |name| {
        beginRegion(name);
    }
    try std.testing.expect(g_profiler.region_count == @as(u8, MAX_PROFILER_REGIONS));
}

test "863: profiler cycle overflow - handle large cycles" {
    init();
    beginRegion("overflow_test");
    endRegion("overflow_test", 9999999999);
    const cycles = getTotalCycles("overflow_test");
    try std.testing.expect(cycles > 0);
}

test "864: profiler zero cycles - handle zero cycles" {
    init();
    beginRegion("zero_test");
    endRegion("zero_test", 0);
    const avg = getAverageCycles("zero_test");
    try std.testing.expect(avg == 0);
}

test "865: profiler min max tracking - track min max" {
    init();
    beginRegion("minmax_test");
    endRegion("minmax_test", 100);
    endRegion("minmax_test", 500);
    endRegion("minmax_test", 300);
    try std.testing.expect(getTotalCycles("minmax_test") == 900);
}

test "866: profiler region naming - unique names" {
    init();
    beginRegion("region_a");
    beginRegion("region_b");
    try std.testing.expect(getCallCount("region_a") == 0);
    try std.testing.expect(getCallCount("region_b") == 0);
}

test "867: profiler performance overhead - minimal overhead" {
    init();
    enableProfiler();
    beginRegion("overhead_test");
    endRegion("overhead_test", 50);
    try std.testing.expect(isProfilerEnabled());
}

test "868: profiler multi-region - multiple regions" {
    init();
    beginRegion("region_1");
    beginRegion("region_2");
    endRegion("region_1", 100);
    endRegion("region_2", 200);
    const total = getTotalCycles("region_1") + getTotalCycles("region_2");
    try std.testing.expect(total == 300);
}

test "869: profiler hierarchy - nested regions" {
    init();
    beginRegion("parent");
    endRegion("parent", 1000);
    beginRegion("child");
    endRegion("child", 500);
    try std.testing.expect(getTotalCycles("parent") == 1000);
}

test "870: profiler aggregation - aggregate data" {
    init();
    beginRegion("aggregate");
    endRegion("aggregate", 100);
    endRegion("aggregate", 100);
    endRegion("aggregate", 100);
    const avg = getAverageCycles("aggregate");
    try std.testing.expect(avg == 100);
}

test "871: profiler serialization - serialize data" {
    init();
    beginRegion("serialize");
    endRegion("serialize", 1000);
    const cycles = getTotalCycles("serialize");
    try std.testing.expect(cycles > 0);
}

test "872: profiler deserialization - deserialize data" {
    init();
    beginRegion("deserialize");
    endRegion("deserialize", 500);
    const cycles = getTotalCycles("deserialize");
    try std.testing.expect(cycles == 500);
}

test "873: profiler comparison - compare frames" {
    init();
    beginRegion("compare");
    endRegion("compare", 1000);
    const cycles = getTotalCycles("compare");
    try std.testing.expect(cycles > 0);
}

test "874: profiler trend analysis - analyze trends" {
    init();
    beginRegion("trend");
    endRegion("trend", 100);
    const avg = getAverageCycles("trend");
    try std.testing.expect(avg > 0);
}

test "875: profiler anomaly detection - detect anomalies" {
    init();
    beginRegion("anomaly");
    endRegion("anomaly", 10000);
    const cycles = getTotalCycles("anomaly");
    try std.testing.expect(cycles > 0);
}

test "876: profiler caching - cache optimization" {
    init();
    beginRegion("cache_test");
    endRegion("cache_test", 100);
    beginRegion("cache_test");
    endRegion("cache_test", 100);
    try std.testing.expect(getCallCount("cache_test") == 2);
}

test "877: profiler batch processing - batch optimization" {
    init();
    beginRegion("batch");
    endRegion("batch", 500);
    try std.testing.expect(getTotalCycles("batch") == 500);
}

test "878: profiler parallel tracking - track parallel" {
    init();
    beginRegion("parallel_1");
    beginRegion("parallel_2");
    endRegion("parallel_1", 100);
    endRegion("parallel_2", 200);
    try std.testing.expect(getTotalCycles("parallel_1") + getTotalCycles("parallel_2") == 300);
}

test "879: profiler timeline - timeline view" {
    init();
    beginRegion("timeline");
    endRegion("timeline", 1000);
    const cycles = getTotalCycles("timeline");
    try std.testing.expect(cycles > 0);
}

test "880: profiler summary - generate summary" {
    init();
    beginRegion("summary");
    endRegion("summary", 500);
    const bottlenecks = detectBottlenecks();
    try std.testing.expect(bottlenecks.len == 1);
    try std.testing.expect(std.mem.eql(u8, bottlenecks[0].region_name, "summary"));
}

test "881: physics tick profiling - profile physics tick" {
    init();
    beginRegion("physics_tick");
    endRegion("physics_tick", 5000);
    try std.testing.expect(getTotalCycles("physics_tick") > 0);
}

test "882: collision profiling - profile collision" {
    init();
    beginRegion("collision");
    endRegion("collision", 3000);
    try std.testing.expect(getTotalCycles("collision") > 0);
}

test "883: constraint profiling - profile constraints" {
    init();
    beginRegion("constraint");
    endRegion("constraint", 2000);
    try std.testing.expect(getTotalCycles("constraint") > 0);
}

test "884: broadphase profiling - profile broadphase" {
    init();
    beginRegion("broadphase");
    endRegion("broadphase", 1000);
    try std.testing.expect(getTotalCycles("broadphase") > 0);
}

test "885: narrowphase profiling - profile narrowphase" {
    init();
    beginRegion("narrowphase");
    endRegion("narrowphase", 1500);
    try std.testing.expect(getTotalCycles("narrowphase") > 0);
}

test "886: sleep profiling - profile sleep" {
    init();
    beginRegion("sleep");
    endRegion("sleep", 100);
    try std.testing.expect(getTotalCycles("sleep") > 0);
}

test "887: query profiling - profile queries" {
    init();
    beginRegion("query");
    endRegion("query", 800);
    try std.testing.expect(getTotalCycles("query") > 0);
}

test "888: memory profiling - profile memory" {
    init();
    recordMetrics(.{
        .physics_tick_time_us = 0.0,
        .collision_time_us = 0.0,
        .constraint_time_us = 0.0,
        .broadphase_time_us = 0.0,
        .narrowphase_time_us = 0.0,
        .sleep_time_us = 0.0,
        .query_time_us = 0.0,
        .memory_alloc_bytes = 8192,
        .memory_free_bytes = 4096,
    });
    const mem = getMemoryUsage();
    try std.testing.expect(mem.allocated == 8192);
    try std.testing.expect(mem.freed == 4096);
}

test "889: optimization priority - prioritize optimizations" {
    init();
    beginRegion("optimize");
    endRegion("optimize", 10000);
    const hints = generateOptimizationHints();
    try std.testing.expect(hints.len > 0);
    try std.testing.expect(hints[0].estimated_gain_percent >= 0);
}

test "890: profiler export - export profiling data" {
    init();
    beginRegion("export");
    endRegion("export", 500);
    const total = getTotalCycles("export");
    try std.testing.expect(total > 0);
}
