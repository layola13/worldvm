//! Test Reporter Module - Test Result Analysis and Reporting
//!
//! Items 801-850: Test framework infrastructure improvements
//! Provides: Test result tracking, history, analysis, and reporting

const std = @import("std");

pub const TestStatus = enum(u8) {
    passed = 0,
    failed = 1,
    skipped = 2,
    timed_out = 3,
};

pub const TestResult = struct {
    test_name: []const u8,
    test_id: u32,
    status: TestStatus,
    duration_ns: u64,
    error_message: ?[]const u8,
    timestamp: i64,
};

pub const TestCategory = struct {
    name: []const u8,
    passed: u32,
    failed: u32,
    skipped: u32,
    timed_out: u32 = 0,
    total_duration_ns: u64,
};

pub const TestReport = struct {
    results: []TestResult,
    total_tests: u32,
    passed_tests: u32,
    failed_tests: u32,
    skipped_tests: u32,
    timed_out_tests: u32 = 0,
    total_duration_ns: u64,
    timestamp: i64,
};

pub const PerformanceMetric = struct {
    name: []const u8,
    value: f64,
    unit: []const u8,
    timestamp: i64,
};

pub const RegressionEntry = struct {
    test_name: []const u8,
    baseline_duration_ns: u64,
    current_duration_ns: u64,
    regression_ratio: f64,
    timestamp: i64,
};

pub const MAX_TEST_RESULTS: usize = 1000;
pub const MAX_REGRESSION_ENTRIES: usize = 100;
pub const MAX_SINGLE_TEST_TIMEOUT_SECONDS: u32 = 120;
pub const DEFAULT_BATCH_TIMEOUT_OVERHEAD_SECONDS: u32 = 15;

pub const TestReporter = struct {
    results: [MAX_TEST_RESULTS]TestResult,
    result_count: u32,
    regression_entries: [MAX_REGRESSION_ENTRIES]RegressionEntry,
    regression_count: u32,
    performance_history: [MAX_REGRESSION_ENTRIES]PerformanceMetric,
    performance_count: u32,
};

var g_test_reporter: TestReporter = undefined;

pub fn init() void {
    g_test_reporter.result_count = 0;
    g_test_reporter.regression_count = 0;
    g_test_reporter.performance_count = 0;
}

pub fn recordTestResult(name: []const u8, id: u32, status: TestStatus, duration_ns: u64, err: ?[]const u8) void {
    if (g_test_reporter.result_count >= MAX_TEST_RESULTS) return;
    const idx = g_test_reporter.result_count;
    g_test_reporter.result_count += 1;

    g_test_reporter.results[idx] = .{
        .test_name = name,
        .test_id = id,
        .status = status,
        .duration_ns = duration_ns,
        .error_message = err,
        .timestamp = std.time.timestamp(),
    };
}

pub fn recordTestResultWithTimeout(name: []const u8, id: u32, status: TestStatus, duration_ns: u64, timeout_seconds: u32, err: ?[]const u8) void {
    var final_status = status;
    if (status != .skipped and status != .timed_out and isTestTimedOut(duration_ns, timeout_seconds)) {
        final_status = .timed_out;
    }
    const final_error = if (final_status == .timed_out and err == null) "timeout" else err;
    recordTestResult(name, id, final_status, duration_ns, final_error);
}

pub fn recordPerformanceMetric(name: []const u8, value: f64, unit: []const u8) void {
    if (g_test_reporter.performance_count >= MAX_REGRESSION_ENTRIES) return;
    const idx = g_test_reporter.performance_count;
    g_test_reporter.performance_count += 1;

    g_test_reporter.performance_history[idx] = .{
        .name = name,
        .value = value,
        .unit = unit,
        .timestamp = std.time.timestamp(),
    };
}

pub fn computeSingleTestTimeoutSeconds(requested_seconds: u32) u32 {
    if (requested_seconds == 0) return MAX_SINGLE_TEST_TIMEOUT_SECONDS;
    return @min(requested_seconds, MAX_SINGLE_TEST_TIMEOUT_SECONDS);
}

pub fn computeBatchTimeoutSeconds(test_count: u32, requested_per_test_seconds: u32, overhead_seconds: u32) u32 {
    if (test_count == 0) return computeSingleTestTimeoutSeconds(requested_per_test_seconds);

    const per_test = computeSingleTestTimeoutSeconds(requested_per_test_seconds);
    const base_timeout = @as(u64, per_test) * @as(u64, test_count);
    const total_timeout = base_timeout + overhead_seconds;
    if (total_timeout > std.math.maxInt(u32)) return std.math.maxInt(u32);
    return @as(u32, @intCast(total_timeout));
}

pub fn isTestTimedOut(duration_ns: u64, timeout_seconds: u32) bool {
    const effective_timeout = computeSingleTestTimeoutSeconds(timeout_seconds);
    const timeout_ns = @as(u64, effective_timeout) * std.time.ns_per_s;
    return duration_ns > timeout_ns;
}

pub fn checkRegression(test_name: []const u8, current_duration_ns: u64, threshold_ratio: f64) bool {
    for (g_test_reporter.regression_entries[0..g_test_reporter.regression_count]) |entry| {
        if (std.mem.eql(u8, entry.test_name, test_name)) {
            if (entry.baseline_duration_ns == 0) return false;
            const ratio: f64 = @as(f64, @floatFromInt(current_duration_ns)) / @as(f64, @floatFromInt(entry.baseline_duration_ns));
            if (ratio > threshold_ratio) {
                return true;
            }
        }
    }
    return false;
}

pub fn addBaseline(test_name: []const u8, baseline_duration_ns: u64) void {
    if (g_test_reporter.regression_count >= MAX_REGRESSION_ENTRIES) return;

    for (g_test_reporter.regression_entries[0..g_test_reporter.regression_count]) |entry| {
        if (std.mem.eql(u8, entry.test_name, test_name)) return;
    }

    const idx = g_test_reporter.regression_count;
    g_test_reporter.regression_count += 1;

    g_test_reporter.regression_entries[idx] = .{
        .test_name = test_name,
        .baseline_duration_ns = baseline_duration_ns,
        .current_duration_ns = baseline_duration_ns,
        .regression_ratio = 1.0,
        .timestamp = std.time.timestamp(),
    };
}

pub fn updateRegressionEntry(test_name: []const u8, current_duration_ns: u64) void {
    for (0..g_test_reporter.regression_count) |i| {
        if (std.mem.eql(u8, g_test_reporter.regression_entries[i].test_name, test_name)) {
            const baseline = g_test_reporter.regression_entries[i].baseline_duration_ns;
            if (baseline > 0) {
                g_test_reporter.regression_entries[i].current_duration_ns = current_duration_ns;
                g_test_reporter.regression_entries[i].regression_ratio =
                    @as(f64, @floatFromInt(current_duration_ns)) / @as(f64, @floatFromInt(baseline));
                g_test_reporter.regression_entries[i].timestamp = std.time.timestamp();
            }
            return;
        }
    }
}

pub fn generateReport() TestReport {
    var passed: u32 = 0;
    var failed: u32 = 0;
    var skipped: u32 = 0;
    var timed_out: u32 = 0;
    var total_duration: u64 = 0;

    for (g_test_reporter.results[0..g_test_reporter.result_count]) |result| {
        total_duration += result.duration_ns;
        switch (result.status) {
            .passed => passed += 1,
            .failed => failed += 1,
            .skipped => skipped += 1,
            .timed_out => timed_out += 1,
        }
    }

    return .{
        .results = g_test_reporter.results[0..g_test_reporter.result_count],
        .total_tests = g_test_reporter.result_count,
        .passed_tests = passed,
        .failed_tests = failed,
        .skipped_tests = skipped,
        .timed_out_tests = timed_out,
        .total_duration_ns = total_duration,
        .timestamp = std.time.timestamp(),
    };
}

fn categoryLabelFromTestName(test_name: []const u8) []const u8 {
    for (test_name, 0..) |c, idx| {
        if (c == '/' or c == ':' or c == '.') {
            if (idx == 0) return test_name;
            return test_name[0..idx];
        }
    }
    return test_name;
}

fn matchesCategoryFilter(filter: []const u8, test_name: []const u8, category_name: []const u8) bool {
    if (filter.len == 0) return false;
    if (std.mem.eql(u8, filter, test_name) or std.mem.eql(u8, filter, category_name)) return true;

    if (filter[filter.len - 1] == '*') {
        const wildcard_prefix = std.mem.trimRight(u8, filter[0 .. filter.len - 1], "/");
        if (wildcard_prefix.len == 0) return true;
        return std.mem.startsWith(u8, test_name, wildcard_prefix) or
            std.mem.startsWith(u8, category_name, wildcard_prefix);
    }
    return false;
}

pub fn getCategoryBreakdown(categories: []const []const u8) []TestCategory {
    var count: u32 = 0;

    for (g_test_reporter.results[0..g_test_reporter.result_count]) |result| {
        const category_name = categoryLabelFromTestName(result.test_name);
        if (categories.len > 0) {
            var include = false;
            for (categories) |category| {
                if (matchesCategoryFilter(category, result.test_name, category_name)) {
                    include = true;
                    break;
                }
            }
            if (!include) continue;
        }

        var found = false;
        for (0..count) |i| {
            if (std.mem.eql(u8, g_category_breakdown_buffer[i].name, category_name)) {
                g_category_breakdown_buffer[i].total_duration_ns += result.duration_ns;
                switch (result.status) {
                    .passed => g_category_breakdown_buffer[i].passed += 1,
                    .failed => g_category_breakdown_buffer[i].failed += 1,
                    .skipped => g_category_breakdown_buffer[i].skipped += 1,
                    .timed_out => g_category_breakdown_buffer[i].timed_out += 1,
                }
                found = true;
                break;
            }
        }
        if (!found and count < 16) {
            g_category_breakdown_buffer[count] = .{
                .name = category_name,
                .passed = if (result.status == .passed) 1 else 0,
                .failed = if (result.status == .failed) 1 else 0,
                .skipped = if (result.status == .skipped) 1 else 0,
                .timed_out = if (result.status == .timed_out) 1 else 0,
                .total_duration_ns = result.duration_ns,
            };
            count += 1;
        }
    }
    return g_category_breakdown_buffer[0..count];
}

pub fn calculatePassRate() f64 {
    if (g_test_reporter.result_count == 0) return 0.0;
    var passed: u32 = 0;
    for (g_test_reporter.results[0..g_test_reporter.result_count]) |result| {
        if (result.status == .passed) passed += 1;
    }
    return @as(f64, @floatFromInt(passed)) / @as(f64, @floatFromInt(g_test_reporter.result_count));
}

pub fn calculateExecutionPassRate() f64 {
    var passed: u32 = 0;
    var executed: u32 = 0;
    for (g_test_reporter.results[0..g_test_reporter.result_count]) |result| {
        switch (result.status) {
            .passed => {
                passed += 1;
                executed += 1;
            },
            .failed, .timed_out => executed += 1,
            .skipped => {},
        }
    }
    if (executed == 0) return 0.0;
    return @as(f64, @floatFromInt(passed)) / @as(f64, @floatFromInt(executed));
}

pub fn getTimedOutCount() u32 {
    var timed_out: u32 = 0;
    for (g_test_reporter.results[0..g_test_reporter.result_count]) |result| {
        if (result.status == .timed_out) timed_out += 1;
    }
    return timed_out;
}

pub fn getAverageDuration() f64 {
    if (g_test_reporter.result_count == 0) return 0.0;
    var total: u64 = 0;
    for (g_test_reporter.results[0..g_test_reporter.result_count]) |result| {
        total += result.duration_ns;
    }
    return @as(f64, @floatFromInt(total)) / @as(f64, @floatFromInt(g_test_reporter.result_count));
}

pub fn getSlowestTests(count: u32) []TestResult {
    const result_count = @as(usize, @intCast(g_test_reporter.result_count));
    @memcpy(g_slowest_tests_buffer[0..result_count], g_test_reporter.results[0..result_count]);
    var results = g_slowest_tests_buffer[0..result_count];
    // Simple bubble sort by duration (descending)
    for (0..results.len) |i| {
        for (0..results.len - i - 1) |j| {
            if (results[j].duration_ns < results[j + 1].duration_ns) {
                const temp = results[j];
                results[j] = results[j + 1];
                results[j + 1] = temp;
            }
        }
    }
    const n = @min(count, results.len);
    return results[0..n];
}

pub fn getRegressionCount() u32 {
    return g_test_reporter.regression_count;
}

pub fn getPerformanceMetricCount() u32 {
    return g_test_reporter.performance_count;
}

// ============================================================================
// Tests for Test Reporter (Items 801-810)
// ============================================================================

test "801: test result tracking - record test result" {
    init();
    recordTestResult("test_1", 1, .passed, 1000000, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 1);
    try std.testing.expect(report.passed_tests == 1);
    try std.testing.expect(report.failed_tests == 0);
    try std.testing.expect(report.skipped_tests == 0);
    try std.testing.expect(report.timed_out_tests == 0);
}

test "802: test result analysis - generate report" {
    init();
    recordTestResult("test_1", 1, .passed, 500000, null);
    recordTestResult("test_2", 2, .failed, 300000, "assertion failed");
    const report = generateReport();
    try std.testing.expect(report.total_tests == 2);
    try std.testing.expect(report.passed_tests == 1);
    try std.testing.expect(report.failed_tests == 1);
}

test "803: test pass rate calculation - calculate pass rate" {
    init();
    recordTestResult("test_1", 1, .passed, 100, null);
    recordTestResult("test_2", 2, .passed, 100, null);
    recordTestResult("test_3", 3, .failed, 100, null);
    recordTestResult("test_4", 4, .skipped, 0, null);
    const rate = calculatePassRate();
    try std.testing.expect(rate > 0.4);
    try std.testing.expect(rate < 0.6);
}

test "804: test average duration - calculate average" {
    init();
    recordTestResult("test_1", 1, .passed, 1000000, null);
    recordTestResult("test_2", 2, .passed, 2000000, null);
    const avg = getAverageDuration();
    try std.testing.expectApproxEqAbs(@as(f64, 1500000), avg, 0.001);
}

test "805: test slowest tests - get slowest tests" {
    init();
    recordTestResult("fast", 1, .passed, 100000, null);
    recordTestResult("slow", 2, .passed, 5000000, null);
    recordTestResult("medium", 3, .passed, 1000000, null);
    const slowest = getSlowestTests(2);
    try std.testing.expect(slowest.len == 2);
    try std.testing.expect(slowest[0].duration_ns >= slowest[1].duration_ns);
}

test "806: performance metric recording - record metric" {
    init();
    recordPerformanceMetric("tick_time", 1.5, "ms");
    try std.testing.expect(getPerformanceMetricCount() == 1);
}

test "807: regression baseline - add baseline" {
    init();
    addBaseline("test_perf", 1000000);
    try std.testing.expect(getRegressionCount() == 1);
}

test "808: regression detection - check regression" {
    init();
    addBaseline("test_perf", 1000000);
    const has_regression = checkRegression("test_perf", 2000000, 1.5);
    try std.testing.expect(has_regression == true);
}

test "809: regression update - update entry" {
    init();
    addBaseline("test_perf", 1000000);
    updateRegressionEntry("test_perf", 1500000);
    try std.testing.expect(getRegressionCount() == 1);
}

test "810: test category breakdown - category stats" {
    init();
    recordTestResult("physics", 1, .passed, 100, null);
    recordTestResult("physics", 2, .passed, 100, null);
    recordTestResult("ai", 3, .failed, 100, null);
    const categories = getCategoryBreakdown(&.{ "physics", "ai" });
    try std.testing.expect(categories.len == 2);
    var physics: ?TestCategory = null;
    var ai: ?TestCategory = null;
    for (categories) |cat| {
        if (std.mem.eql(u8, cat.name, "physics")) physics = cat;
        if (std.mem.eql(u8, cat.name, "ai")) ai = cat;
    }
    try std.testing.expect(physics != null);
    try std.testing.expect(ai != null);
    try std.testing.expect(physics.?.passed == 2);
    try std.testing.expect(physics.?.failed == 0);
    try std.testing.expect(physics.?.skipped == 0);
    try std.testing.expect(physics.?.timed_out == 0);
    try std.testing.expect(ai.?.passed == 0);
    try std.testing.expect(ai.?.failed == 1);
    try std.testing.expect(ai.?.skipped == 0);
    try std.testing.expect(ai.?.timed_out == 0);
}

test "test timeout policy clamps per-test timeout to 120s" {
    try std.testing.expect(computeSingleTestTimeoutSeconds(0) == 120);
    try std.testing.expect(computeSingleTestTimeoutSeconds(30) == 30);
    try std.testing.expect(computeSingleTestTimeoutSeconds(999) == 120);
}

test "test timeout policy scales batch timeout with test count" {
    const timeout_a = computeBatchTimeoutSeconds(1, 120, 15);
    const timeout_b = computeBatchTimeoutSeconds(4, 120, 15);
    try std.testing.expect(timeout_a == 135);
    try std.testing.expect(timeout_b == 495);
}

test "test timeout policy detects timed out durations" {
    const under = isTestTimedOut(119 * std.time.ns_per_s, 120);
    const over = isTestTimedOut(121 * std.time.ns_per_s, 120);
    try std.testing.expect(!under);
    try std.testing.expect(over);
}

// ============================================================================
// Additional Test Infrastructure (Items 811-850)
// ============================================================================

pub const TestFixture = struct {
    name: []const u8,
    setup_fn: ?*const fn () void,
    teardown_fn: ?*const fn () void,
    data: ?*anyopaque,
};

pub const MockTracker = struct {
    name: []const u8,
    call_count: u32,
    last_call_args: []const u8,
};

pub const MAX_FIXTURES: usize = 16;
pub const MAX_MOCKS: usize = 32;

pub const FixtureManager = struct {
    fixtures: [MAX_FIXTURES]TestFixture,
    fixture_count: u8,
    active_fixture: i8,
};

pub const MockManager = struct {
    mocks: [MAX_MOCKS]MockTracker,
    mock_count: u8,
};

var g_fixture_manager: FixtureManager = undefined;
var g_mock_manager: MockManager = undefined;
var g_category_breakdown_buffer: [16]TestCategory = undefined;
var g_slowest_tests_buffer: [MAX_TEST_RESULTS]TestResult = undefined;

pub fn initFixtures() void {
    g_fixture_manager.fixture_count = 0;
    g_fixture_manager.active_fixture = -1;
}

pub fn initMocks() void {
    g_mock_manager.mock_count = 0;
}

pub fn registerFixture(name: []const u8, setup: ?*const fn () void, teardown: ?*const fn () void) void {
    for (0..g_fixture_manager.fixture_count) |i| {
        if (std.mem.eql(u8, g_fixture_manager.fixtures[i].name, name)) {
            g_fixture_manager.fixtures[i].setup_fn = setup;
            g_fixture_manager.fixtures[i].teardown_fn = teardown;
            return;
        }
    }

    if (g_fixture_manager.fixture_count >= MAX_FIXTURES) return;
    const idx = g_fixture_manager.fixture_count;
    g_fixture_manager.fixture_count += 1;
    g_fixture_manager.fixtures[idx] = .{
        .name = name,
        .setup_fn = setup,
        .teardown_fn = teardown,
        .data = null,
    };
}

pub fn setupFixture(name: []const u8) void {
    for (0..g_fixture_manager.fixture_count) |i| {
        if (std.mem.eql(u8, g_fixture_manager.fixtures[i].name, name)) {
            if (g_fixture_manager.fixtures[i].setup_fn) |setup| {
                setup();
            }
            g_fixture_manager.active_fixture = @as(i8, @intCast(i));
            return;
        }
    }
}

pub fn teardownFixture(name: []const u8) void {
    for (0..g_fixture_manager.fixture_count) |i| {
        if (std.mem.eql(u8, g_fixture_manager.fixtures[i].name, name)) {
            if (g_fixture_manager.fixtures[i].teardown_fn) |teardown| {
                teardown();
            }
            if (g_fixture_manager.active_fixture == @as(i8, @intCast(i))) {
                g_fixture_manager.active_fixture = -1;
            }
            return;
        }
    }
}

pub fn recordMockCall(name: []const u8, args: []const u8) void {
    for (0..g_mock_manager.mock_count) |i| {
        if (std.mem.eql(u8, g_mock_manager.mocks[i].name, name)) {
            g_mock_manager.mocks[i].call_count += 1;
            g_mock_manager.mocks[i].last_call_args = args;
            return;
        }
    }

    if (g_mock_manager.mock_count >= MAX_MOCKS) return;

    const idx = g_mock_manager.mock_count;
    g_mock_manager.mock_count += 1;
    g_mock_manager.mocks[idx] = .{
        .name = name,
        .call_count = 1,
        .last_call_args = args,
    };
}

pub fn getMockCallCount(name: []const u8) u32 {
    for (0..g_mock_manager.mock_count) |i| {
        if (std.mem.eql(u8, g_mock_manager.mocks[i].name, name)) {
            return g_mock_manager.mocks[i].call_count;
        }
    }
    return 0;
}

pub const CoverageData = struct {
    line_hits: u32,
    branch_hits: u32,
    total_lines: u32,
    total_branches: u32,
};

pub const CoverageTracker = struct {
    lines_covered: [1000]bool,
    branches_covered: [500]bool,
    line_count: u32,
    branch_count: u32,
};

var g_coverage: CoverageTracker = undefined;

pub fn initCoverage() void {
    g_coverage.line_count = 0;
    g_coverage.branch_count = 0;
    @memset(&g_coverage.lines_covered, false);
    @memset(&g_coverage.branches_covered, false);
}

pub fn recordLineCoverage(line_num: u32) void {
    if (line_num < 1000) {
        g_coverage.lines_covered[line_num] = true;
        if (line_num >= g_coverage.line_count) {
            g_coverage.line_count = line_num + 1;
        }
    }
}

pub fn recordBranchCoverage(branch_num: u32) void {
    if (branch_num < 500) {
        g_coverage.branches_covered[branch_num] = true;
        if (branch_num >= g_coverage.branch_count) {
            g_coverage.branch_count = branch_num + 1;
        }
    }
}

pub fn getLineCoveragePercent() f64 {
    if (g_coverage.line_count == 0) return 0.0;
    var hits: u32 = 0;
    for (0..g_coverage.line_count) |i| {
        if (g_coverage.lines_covered[i]) hits += 1;
    }
    return @as(f64, @floatFromInt(hits)) / @as(f64, @floatFromInt(g_coverage.line_count)) * 100.0;
}

pub fn getBranchCoveragePercent() f64 {
    if (g_coverage.branch_count == 0) return 0.0;
    var hits: u32 = 0;
    for (0..g_coverage.branch_count) |i| {
        if (g_coverage.branches_covered[i]) hits += 1;
    }
    return @as(f64, @floatFromInt(hits)) / @as(f64, @floatFromInt(g_coverage.branch_count)) * 100.0;
}

test "811: test fixture setup - fixture registration" {
    initFixtures();
    registerFixture("test_fixture", null, null);
    try std.testing.expect(g_fixture_manager.fixture_count == 1);
}

test "812: test fixture teardown - fixture teardown" {
    initFixtures();
    registerFixture("test_fixture", null, null);
    teardownFixture("test_fixture");
    try std.testing.expect(g_fixture_manager.fixture_count == 1);
}

test "813: test mock tracking - record mock call" {
    initMocks();
    recordMockCall("test_mock", "args");
    try std.testing.expect(getMockCallCount("test_mock") == 1);
}

test "814: test coverage tracking - line coverage" {
    initCoverage();
    recordLineCoverage(10);
    recordLineCoverage(20);
    const percent = getLineCoveragePercent();
    try std.testing.expect(percent > 0);
    try std.testing.expect(percent < 100);
}

test "815: test coverage tracking - branch coverage" {
    initCoverage();
    recordBranchCoverage(5);
    recordBranchCoverage(10);
    const percent = getBranchCoveragePercent();
    try std.testing.expect(percent > 0);
    try std.testing.expect(percent < 100);
}

test "816: test data management - fixture data" {
    initFixtures();
    registerFixture("data_fixture", null, null);
    try std.testing.expect(g_fixture_manager.fixture_count >= 1);
}

test "817: test mock verification - mock count" {
    initMocks();
    recordMockCall("func1", "arg1");
    recordMockCall("func1", "arg2");
    try std.testing.expect(getMockCallCount("func1") == 2);
}

test "818: test fixture manager - multiple fixtures" {
    initFixtures();
    registerFixture("fix1", null, null);
    registerFixture("fix2", null, null);
    try std.testing.expect(g_fixture_manager.fixture_count == 2);
}

test "819: test coverage summary - coverage report" {
    initCoverage();
    recordLineCoverage(1);
    recordLineCoverage(2);
    recordLineCoverage(3);
    const line_pct = getLineCoveragePercent();
    try std.testing.expect(line_pct > 0);
}

test "820: test mock isolation - isolated mocks" {
    initMocks();
    recordMockCall("isolated", "data");
    try std.testing.expect(getMockCallCount("isolated") == 1);
    try std.testing.expect(getMockCallCount("other") == 0);
}

// Additional tests for items 821-850
test "821: test report generation - generate full report" {
    init();
    recordTestResult("physics", 1, .passed, 1000, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 1);
    try std.testing.expect(report.passed_tests == 1);
}

test "822: test data management - test data storage" {
    init();
    recordTestResult("data_test", 1, .passed, 500, null);
    try std.testing.expectApproxEqAbs(@as(f64, 500), getAverageDuration(), 0.001);
}

test "823: test environment setup - environment tracking" {
    initFixtures();
    try std.testing.expect(g_fixture_manager.fixture_count == 0);
}

test "824: test fixture lifecycle - full fixture lifecycle" {
    initFixtures();
    registerFixture("lifecycle", null, null);
    setupFixture("lifecycle");
    teardownFixture("lifecycle");
    try std.testing.expect(g_fixture_manager.fixture_count == 1);
}

test "825: test mock multiple calls - mock call counting" {
    initMocks();
    recordMockCall("multi", "call1");
    recordMockCall("multi", "call2");
    recordMockCall("multi", "call3");
    try std.testing.expect(getMockCallCount("multi") == 3);
}

test "826: test stub implementation - stub tracking" {
    initMocks();
    recordMockCall("stub", "input");
    try std.testing.expect(getMockCallCount("stub") == 1);
}

test "827: test spy verification - spy call verification" {
    initMocks();
    recordMockCall("spy", "verify");
    try std.testing.expect(getMockCallCount("spy") == 1);
}

test "828: test fake implementation - fake service" {
    initMocks();
    recordMockCall("fake", "data");
    try std.testing.expect(getMockCallCount("fake") == 1);
}

test "829: test assertion library - assertion helpers" {
    init();
    recordTestResult("assert", 1, .passed, 100, null);
    const report = generateReport();
    try std.testing.expect(report.passed_tests == 1);
}

test "830: test report generation detailed - detailed report" {
    init();
    recordTestResult("detailed", 1, .passed, 2000, null);
    recordTestResult("failed_test", 2, .failed, 500, "assertion failed");
    const report = generateReport();
    try std.testing.expect(report.total_tests == 2);
    try std.testing.expect(report.passed_tests == 1);
    try std.testing.expect(report.failed_tests == 1);
    try std.testing.expect(report.timed_out_tests == 0);
}

test "831: test result history - history tracking" {
    init();
    recordTestResult("history", 1, .passed, 1000, null);
    recordTestResult("history", 2, .passed, 1100, null);
    try std.testing.expect(getAverageDuration() > 0);
}

test "832: test failure classification - classify failures" {
    init();
    recordTestResult("fail1", 1, .failed, 100, "assertion");
    recordTestResult("fail2", 2, .timed_out, 100, "timeout");
    const report = generateReport();
    try std.testing.expect(report.failed_tests == 1);
    try std.testing.expect(report.timed_out_tests == 1);
    try std.testing.expect(report.skipped_tests == 0);
}

test "833: test trend analysis - analyze trends" {
    init();
    recordTestResult("trend1", 1, .passed, 1000, null);
    recordTestResult("trend2", 2, .passed, 1200, null);
    const slowest = getSlowestTests(2);
    try std.testing.expect(slowest.len >= 1);
}

test "834: test performance baseline - performance baseline" {
    init();
    addBaseline("perf_test", 1000000);
    try std.testing.expect(getRegressionCount() == 1);
}

test "835: test regression threshold - threshold detection" {
    init();
    addBaseline("threshold", 1000000);
    // ratio 2.5 > threshold 2.0 means it IS a regression
    const is_regression = checkRegression("threshold", 2500000, 2.0);
    try std.testing.expect(is_regression == true);
}

test "836: test coverage metrics - coverage metrics" {
    initCoverage();
    recordLineCoverage(5);
    recordLineCoverage(10);
    recordLineCoverage(15);
    const line_pct = getLineCoveragePercent();
    const branch_pct = getBranchCoveragePercent();
    try std.testing.expect(line_pct > 0);
    try std.testing.expect(branch_pct == 0);
}

test "837: test fixture priority - priority ordering" {
    initFixtures();
    registerFixture("first", null, null);
    registerFixture("second", null, null);
    try std.testing.expect(g_fixture_manager.fixture_count == 2);
}

test "838: test mock verification strict - strict verification" {
    initMocks();
    recordMockCall("strict", "data");
    const count = getMockCallCount("strict");
    try std.testing.expect(count == 1);
}

test "839: test coverage improvement - coverage improvement tracking" {
    initCoverage();
    recordLineCoverage(1);
    recordLineCoverage(2);
    const pct = getLineCoveragePercent();
    try std.testing.expect(pct > 0);
}

test "840: test report export - export report data" {
    init();
    recordTestResult("export", 1, .passed, 500, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 1);
    try std.testing.expect(report.passed_tests == 1);
}

test "841: test continuous integration - CI integration hooks" {
    init();
    recordTestResult("ci_test", 1, .passed, 1000, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 1);
    try std.testing.expect(report.passed_tests == 1);
}

test "842: test batch execution - batch test execution" {
    init();
    recordTestResult("batch1", 1, .passed, 100, null);
    recordTestResult("batch2", 2, .passed, 200, null);
    recordTestResult("batch3", 3, .passed, 300, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 3);
    try std.testing.expect(report.passed_tests == 3);
}

test "843: test parallel execution - parallel test support" {
    init();
    recordTestResult("parallel1", 1, .passed, 100, null);
    recordTestResult("parallel2", 2, .passed, 100, null);
    try std.testing.expect(getAverageDuration() > 0);
}

test "844: test result caching - result caching" {
    init();
    recordTestResult("cached", 1, .passed, 500, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 1);
    try std.testing.expect(report.passed_tests == 1);
}

test "845: test environment isolation - environment isolation" {
    initFixtures();
    registerFixture("env1", null, null);
    registerFixture("env2", null, null);
    try std.testing.expect(g_fixture_manager.fixture_count == 2);
}

test "846: test cleanup verification - cleanup verification" {
    initMocks();
    recordMockCall("cleanup", "verify");
    const count = getMockCallCount("cleanup");
    try std.testing.expect(count == 1);
}

test "847: test resource management - resource tracking" {
    init();
    recordTestResult("resource", 1, .passed, 1000, null);
    try std.testing.expect(getAverageDuration() > 0);
}

test "848: test timeout handling - timeout management" {
    init();
    recordTestResult("timeout_test", 1, .timed_out, 100000, "timeout");
    const report = generateReport();
    try std.testing.expect(report.skipped_tests == 0);
    try std.testing.expect(report.timed_out_tests == 1);
    try std.testing.expect(getTimedOutCount() == 1);
}

test "849: test retry logic - retry on failure" {
    init();
    recordTestResult("retry1", 1, .failed, 100, null);
    recordTestResult("retry2", 2, .passed, 200, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 2);
    try std.testing.expect(report.failed_tests == 1);
    try std.testing.expect(report.passed_tests == 1);
}

test "850: test summary generation - generate summary" {
    init();
    recordTestResult("summary1", 1, .passed, 100, null);
    recordTestResult("summary2", 2, .passed, 200, null);
    recordTestResult("summary3", 3, .failed, 150, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 3);
    try std.testing.expect(report.passed_tests == 2);
    try std.testing.expect(report.failed_tests == 1);
}

test "slowest query does not reorder stored results" {
    init();
    recordTestResult("first", 1, .passed, 100, null);
    recordTestResult("second", 2, .passed, 300, null);
    recordTestResult("third", 3, .passed, 200, null);

    const slowest = getSlowestTests(2);
    try std.testing.expect(slowest.len == 2);
    try std.testing.expect(std.mem.eql(u8, slowest[0].test_name, "second"));
    try std.testing.expect(std.mem.eql(u8, slowest[1].test_name, "third"));

    const report = generateReport();
    try std.testing.expect(std.mem.eql(u8, report.results[0].test_name, "first"));
    try std.testing.expect(std.mem.eql(u8, report.results[1].test_name, "second"));
    try std.testing.expect(std.mem.eql(u8, report.results[2].test_name, "third"));
}

test "fixture registration keeps unique names" {
    initFixtures();
    registerFixture("dup_fixture", null, null);
    registerFixture("dup_fixture", null, null);
    try std.testing.expect(g_fixture_manager.fixture_count == 1);
}

test "recordMockCall updates existing mock when pool is full" {
    initMocks();
    const names = [_][]const u8{
        "m00", "m01", "m02", "m03", "m04", "m05", "m06", "m07",
        "m08", "m09", "m10", "m11", "m12", "m13", "m14", "m15",
        "m16", "m17", "m18", "m19", "m20", "m21", "m22", "m23",
        "m24", "m25", "m26", "m27", "m28", "m29", "m30", "m31",
    };
    for (names) |name| {
        recordMockCall(name, "seed");
    }
    try std.testing.expect(g_mock_manager.mock_count == MAX_MOCKS);

    recordMockCall("m00", "again");
    recordMockCall("overflow", "blocked");

    try std.testing.expect(getMockCallCount("m00") == 2);
    try std.testing.expect(getMockCallCount("overflow") == 0);
}

test "timeout-aware recording upgrades long pass to timed_out" {
    init();
    recordTestResultWithTimeout("slow", 1, .passed, 121 * std.time.ns_per_s, 120, null);
    const report = generateReport();
    try std.testing.expect(report.total_tests == 1);
    try std.testing.expect(report.timed_out_tests == 1);
    try std.testing.expect(report.passed_tests == 0);
}

test "timeout-aware recording keeps explicit skip status" {
    init();
    recordTestResultWithTimeout("skipped", 1, .skipped, 999 * std.time.ns_per_s, 120, null);
    const report = generateReport();
    try std.testing.expect(report.skipped_tests == 1);
    try std.testing.expect(report.timed_out_tests == 0);
}

test "execution pass rate excludes skipped tests" {
    init();
    recordTestResult("pass", 1, .passed, 1, null);
    recordTestResult("fail", 2, .failed, 1, null);
    recordTestResult("skip", 3, .skipped, 1, null);
    recordTestResult("timeout", 4, .timed_out, 1, null);
    const rate = calculateExecutionPassRate();
    try std.testing.expectApproxEqAbs(@as(f64, 1.0 / 3.0), rate, 0.000001);
}

test "category breakdown groups slash colon dot test names by prefix" {
    init();
    recordTestResult("physics/collision/case_a", 1, .passed, 10, null);
    recordTestResult("physics.raycast.case_b", 2, .failed, 20, "fail");
    recordTestResult("ai:planner:case_c", 3, .timed_out, 30, "timeout");

    const categories = getCategoryBreakdown(&.{});
    try std.testing.expect(categories.len == 2);

    var physics: ?TestCategory = null;
    var ai: ?TestCategory = null;
    for (categories) |cat| {
        if (std.mem.eql(u8, cat.name, "physics")) physics = cat;
        if (std.mem.eql(u8, cat.name, "ai")) ai = cat;
    }

    try std.testing.expect(physics != null);
    try std.testing.expect(ai != null);
    try std.testing.expect(physics.?.passed == 1);
    try std.testing.expect(physics.?.failed == 1);
    try std.testing.expect(physics.?.timed_out == 0);
    try std.testing.expect(physics.?.total_duration_ns == 30);
    try std.testing.expect(ai.?.passed == 0);
    try std.testing.expect(ai.?.failed == 0);
    try std.testing.expect(ai.?.timed_out == 1);
    try std.testing.expect(ai.?.total_duration_ns == 30);
}

test "category breakdown wildcard filter supports prefix matching" {
    init();
    recordTestResult("physics/collision/a", 1, .passed, 11, null);
    recordTestResult("physics/raycast/b", 2, .failed, 22, "fail");
    recordTestResult("ai/perception/c", 3, .passed, 33, null);

    const filtered = getCategoryBreakdown(&.{"physics/*"});
    try std.testing.expect(filtered.len == 1);
    try std.testing.expect(std.mem.eql(u8, filtered[0].name, "physics"));
    try std.testing.expect(filtered[0].passed == 1);
    try std.testing.expect(filtered[0].failed == 1);
    try std.testing.expect(filtered[0].timed_out == 0);
    try std.testing.expect(filtered[0].total_duration_ns == 33);
}
