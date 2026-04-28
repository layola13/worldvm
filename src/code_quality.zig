//! Code Quality Module - Static Analysis and Quality Metrics
//!
//! Items 901-950: Code quality infrastructure
//! Provides: Complexity analysis, style checking, convention enforcement, quality metrics

const std = @import("std");

pub const ComplexityMetric = struct {
    cyclomatic_complexity: u32,
    cognitive_complexity: u32,
    lines_of_code: u32,
    comment_lines: u32,
    function_count: u32,
};

pub const NamingConvention = enum(u8) {
    camelCase = 0,
    snake_case = 1,
    PascalCase = 2,
    SCREAMING_SNAKE_CASE = 3,
};

pub const CodeSmell = struct {
    name: []const u8,
    severity: []const u8,
    line: u32,
    description: []const u8,
};

pub const QualityReport = struct {
    complexity: ComplexityMetric,
    smell_count: u32,
    convention_violations: u32,
    overall_score: f64,
};

pub const DependencyInfo = struct {
    name: []const u8,
    version: []const u8,
    is_direct: bool,
};

pub const MAX_DEPENDENCIES: usize = 64;
pub const MAX_CODE_SMELLS: usize = 32;

pub const CodeQualityChecker = struct {
    total_lines: u32,
    function_count: u32,
    comment_lines: u32,
    smells: [MAX_CODE_SMELLS]CodeSmell,
    smell_count: u8,
    dependencies: [MAX_DEPENDENCIES]DependencyInfo,
    dependency_count: u8,
};

var g_quality_checker: CodeQualityChecker = undefined;

pub fn init() void {
    g_quality_checker.total_lines = 0;
    g_quality_checker.function_count = 0;
    g_quality_checker.comment_lines = 0;
    g_quality_checker.smell_count = 0;
    g_quality_checker.dependency_count = 0;
}

pub fn analyzeComplexity(loc: u32, functions: u32, branches: u32) ComplexityMetric {
    const cyclomatic = branches + 1;
    const cognitive = cyclomatic + (branches / 2);
    return .{
        .cyclomatic_complexity = cyclomatic,
        .cognitive_complexity = cognitive,
        .lines_of_code = loc,
        .comment_lines = 0,
        .function_count = functions,
    };
}

pub fn checkNamingConvention(name: []const u8, expected: NamingConvention) bool {
    if (name.len == 0) return false;

    switch (expected) {
        .camelCase => {
            // camelCase: first char lowercase, no underscores, uppercase allowed after first char
            if (name[0] >= 'A' and name[0] <= 'Z') return false;
            for (name) |c| {
                if (c == '_') return false;
            }
            return true;
        },
        .snake_case => {
            // snake_case: all lowercase, underscores allowed, no uppercase
            for (name) |c| {
                if (c >= 'A' and c <= 'Z') return false;
            }
            return true;
        },
        .PascalCase => {
            // PascalCase: first char uppercase, no underscores, no lowercase-only
            if (name[0] >= 'a' and name[0] <= 'z') return false;
            for (name) |c| {
                if (c == '_') return false;
            }
            return true;
        },
        .SCREAMING_SNAKE_CASE => {
            // SCREAMING_SNAKE_CASE: all uppercase or underscore, no lowercase
            for (name) |c| {
                if (c >= 'a' and c <= 'z') return false;
            }
            return true;
        },
    }
}

pub fn recordSmell(name: []const u8, severity: []const u8, line: u32, description: []const u8) void {
    if (g_quality_checker.smell_count >= MAX_CODE_SMELLS) return;
    const idx = g_quality_checker.smell_count;
    g_quality_checker.smell_count += 1;
    g_quality_checker.smells[idx] = .{
        .name = name,
        .severity = severity,
        .line = line,
        .description = description,
    };
}

pub fn getSmellCount() u8 {
    return g_quality_checker.smell_count;
}

pub fn getSmells() []CodeSmell {
    return g_quality_checker.smells[0..g_quality_checker.smell_count];
}

pub fn calculateQualityScore(complexity: ComplexityMetric, smell_count: u32) f64 {
    var score: f64 = 100.0;

    if (complexity.cyclomatic_complexity > 10) score -= 10;
    if (complexity.cyclomatic_complexity > 20) score -= 15;
    if (complexity.cyclomatic_complexity > 50) score -= 25;

    score -= @as(f64, @floatFromInt(smell_count)) * 2.0;

    if (complexity.lines_of_code > 500) score -= 5;
    if (complexity.lines_of_code > 1000) score -= 10;

    return @max(0, score);
}

pub fn detectLongFunction(loc: u32) bool {
    return loc > 100;
}

pub fn detectDeepNesting(nesting_level: u32) bool {
    return nesting_level > 5;
}

pub fn detectDuplicateCode(pattern: []const u8, occurrences: u32) bool {
    if (occurrences == 0 or pattern.len == 0) return false;

    var significant_chars: u32 = 0;
    for (pattern) |c| {
        if (std.ascii.isAlphanumeric(c)) significant_chars += 1;
    }

    const threshold: u32 = if (significant_chars < 6)
        8
    else if (significant_chars < 20)
        5
    else
        3;
    return occurrences >= threshold;
}

pub fn checkMagicNumbers(value: i64) bool {
    // Returns true if value is a magic number (suspicious constant)
    const magic_values = [_]i64{ 0, 1, -1 };
    for (magic_values) |m| {
        if (value == m) return true;
    }
    return false;
}

pub fn checkParameterCount(count: u32) bool {
    return count <= 7;
}

pub fn checkReturnStatements(count: u32, branches: u32) bool {
    if (branches == 0) return count <= 2;
    const min_expected = @max(@as(u32, 1), branches / 4);
    const max_expected = branches + 3;
    return count >= min_expected and count <= max_expected;
}

pub fn addDependency(name: []const u8, version: []const u8, direct: bool) void {
    if (g_quality_checker.dependency_count >= MAX_DEPENDENCIES) return;
    const idx = g_quality_checker.dependency_count;
    g_quality_checker.dependency_count += 1;
    g_quality_checker.dependencies[idx] = .{
        .name = name,
        .version = version,
        .is_direct = direct,
    };
}

pub fn getDependencyCount() u8 {
    return g_quality_checker.dependency_count;
}

pub fn getDirectDependencies() u8 {
    var count: u8 = 0;
    for (0..g_quality_checker.dependency_count) |i| {
        if (g_quality_checker.dependencies[i].is_direct) count += 1;
    }
    return count;
}

pub fn generateQualityReport(complexity: ComplexityMetric) QualityReport {
    return .{
        .complexity = complexity,
        .smell_count = g_quality_checker.smell_count,
        .convention_violations = 0,
        .overall_score = calculateQualityScore(complexity, g_quality_checker.smell_count),
    };
}

pub fn checkCodeStyle(source: []const u8) bool {
    var brace_balance: i32 = 0;
    var line_iter = std.mem.splitScalar(u8, source, '\n');
    while (line_iter.next()) |line| {
        if (line.len > 120) return false;
        if (line.len > 0 and (line[line.len - 1] == ' ' or line[line.len - 1] == '\t')) return false;
        if (std.mem.indexOfScalar(u8, line, '\t') != null) return false;

        for (line) |c| {
            switch (c) {
                '{' => brace_balance += 1,
                '}' => {
                    brace_balance -= 1;
                    if (brace_balance < 0) return false;
                },
                '\r' => return false,
                else => {},
            }
        }
    }
    return brace_balance == 0;
}

pub fn checkDocumentationCoverage(public_functions: u32, documented: u32) f64 {
    if (public_functions == 0) return 100.0;
    return @as(f64, @floatFromInt(documented)) / @as(f64, @floatFromInt(public_functions)) * 100.0;
}

pub fn checkTestCoverage(covered_lines: u32, total_lines: u32) f64 {
    if (total_lines == 0) return 0.0;
    return @as(f64, @floatFromInt(covered_lines)) / @as(f64, @floatFromInt(total_lines)) * 100.0;
}

// ============================================================================
// Tests for Code Quality (Items 901-910)
// ============================================================================

test "901: complexity analysis - analyze complexity" {
    const complexity = analyzeComplexity(100, 5, 10);
    try std.testing.expect(complexity.cyclomatic_complexity == 11);
}

test "902: naming convention camelCase - check camelCase" {
    try std.testing.expect(checkNamingConvention("camelCase", .camelCase));
    try std.testing.expect(!checkNamingConvention("snake_case", .camelCase));
}

test "903: naming convention snake_case - check snake_case" {
    try std.testing.expect(checkNamingConvention("snake_case", .snake_case));
    try std.testing.expect(!checkNamingConvention("camelCase", .snake_case));
}

test "904: naming convention PascalCase - check PascalCase" {
    try std.testing.expect(checkNamingConvention("PascalCase", .PascalCase));
    try std.testing.expect(!checkNamingConvention("camelCase", .PascalCase));
}

test "905: naming convention SCREAMING_SNAKE - check screaming" {
    try std.testing.expect(checkNamingConvention("SCREAMING_SNAKE", .SCREAMING_SNAKE_CASE));
    try std.testing.expect(!checkNamingConvention("snake_case", .SCREAMING_SNAKE_CASE));
}

test "906: code smell recording - record smell" {
    init();
    recordSmell("long_function", "medium", 42, "Function exceeds 100 lines");
    try std.testing.expect(getSmellCount() == 1);
}

test "907: quality score calculation - calculate score" {
    const complexity = analyzeComplexity(100, 5, 10);
    const score = calculateQualityScore(complexity, 0);
    try std.testing.expect(score > 0);
}

test "908: long function detection - detect long function" {
    try std.testing.expect(detectLongFunction(150));
    try std.testing.expect(!detectLongFunction(50));
}

test "909: deep nesting detection - detect deep nesting" {
    try std.testing.expect(detectDeepNesting(7));
    try std.testing.expect(!detectDeepNesting(3));
}

test "910: magic number detection - detect magic numbers" {
    // 0, 1, -1 are considered magic numbers (suspicious constants)
    try std.testing.expect(checkMagicNumbers(0));
    try std.testing.expect(checkMagicNumbers(1));
    try std.testing.expect(checkMagicNumbers(-1));
    try std.testing.expect(!checkMagicNumbers(42));
}

// Additional tests for items 911-950
test "911: parameter count check - check parameter count" {
    try std.testing.expect(checkParameterCount(5));
    try std.testing.expect(!checkParameterCount(10));
}

test "912: duplicate code detection - detect duplicates" {
    try std.testing.expect(detectDuplicateCode("for(i=0;i<n;i++){sum+=arr[i];}", 5));
    try std.testing.expect(!detectDuplicateCode("pattern", 2));
}

test "913: dependency tracking - track dependencies" {
    init();
    addDependency("zmath", "1.0.0", true);
    try std.testing.expect(getDependencyCount() == 1);
}

test "914: direct dependency count - count direct deps" {
    init();
    addDependency("zmath", "1.0.0", true);
    addDependency("zimg", "0.1.0", false);
    try std.testing.expect(getDirectDependencies() == 1);
}

test "915: quality report generation - generate report" {
    const complexity = analyzeComplexity(100, 5, 10);
    const report = generateQualityReport(complexity);
    try std.testing.expect(report.overall_score > 0);
}

test "916: code style checking - check style" {
    try std.testing.expect(checkCodeStyle("fn test() void {}"));
    try std.testing.expect(!checkCodeStyle("fn test()\tvoid {}"));
    try std.testing.expect(!checkCodeStyle("fn test() void {} "));
}

test "917: documentation coverage - check docs" {
    const coverage = checkDocumentationCoverage(8, 6);
    try std.testing.expect(coverage > 70);
}

test "918: test coverage calculation - calculate coverage" {
    const coverage = checkTestCoverage(80, 100);
    try std.testing.expect(coverage == 80);
}

test "919: smell severity levels - check severity" {
    init();
    recordSmell("smell1", "high", 10, "High severity");
    recordSmell("smell2", "low", 20, "Low severity");
    try std.testing.expect(getSmellCount() == 2);
}

test "920: multiple smells tracking - track multiple" {
    init();
    recordSmell("smell1", "medium", 10, "First");
    recordSmell("smell2", "medium", 20, "Second");
    recordSmell("smell3", "medium", 30, "Third");
    try std.testing.expect(getSmellCount() == 3);
}

test "921: complexity weight calculation - calculate weight" {
    const complexity = analyzeComplexity(200, 10, 20);
    try std.testing.expect(complexity.cyclomatic_complexity > 0);
}

test "922: cognitive complexity - measure cognitive" {
    const complexity = analyzeComplexity(50, 3, 8);
    try std.testing.expect(complexity.cognitive_complexity > complexity.cyclomatic_complexity);
}

test "923: lines of code tracking - track loc" {
    const complexity = analyzeComplexity(300, 8, 15);
    try std.testing.expect(complexity.lines_of_code == 300);
}

test "924: function count tracking - track functions" {
    const complexity = analyzeComplexity(100, 12, 5);
    try std.testing.expect(complexity.function_count == 12);
}

test "925: branch count analysis - analyze branches" {
    const complexity = analyzeComplexity(100, 5, 12);
    try std.testing.expect(complexity.cyclomatic_complexity == 13);
}

test "926: convention violation counting - count violations" {
    init();
    try std.testing.expect(!checkNamingConvention("InvalidName", .snake_case));
}

test "927: smell retrieval - get smells" {
    init();
    recordSmell("test_smell", "high", 50, "Test description");
    const smells = getSmells();
    try std.testing.expect(smells.len == 1);
}

test "928: quality threshold checking - check thresholds" {
    const complexity = analyzeComplexity(1000, 50, 100);
    const score = calculateQualityScore(complexity, 20);
    try std.testing.expect(score < 100);
}

test "929: dependency version tracking - track versions" {
    init();
    addDependency("zig", "0.12.0", true);
    try std.testing.expect(getDependencyCount() == 1);
}

test "930: transitive dependency handling - handle transitive" {
    init();
    addDependency("dep_a", "1.0.0", true);
    addDependency("dep_b", "2.0.0", false);
    addDependency("dep_c", "3.0.0", false);
    try std.testing.expect(getDependencyCount() == 3);
}

test "931: code smell line numbers - track line numbers" {
    init();
    recordSmell("smell_at_line", "medium", 123, "Found at line 123");
    const smells = getSmells();
    try std.testing.expect(smells[0].line == 123);
}

test "932: complexity growth detection - detect growth" {
    const old = analyzeComplexity(100, 5, 10);
    const new = analyzeComplexity(500, 20, 50);
    try std.testing.expect(new.cyclomatic_complexity > old.cyclomatic_complexity);
}

test "933: return statement analysis - analyze returns" {
    try std.testing.expect(checkReturnStatements(5, 10));
    try std.testing.expect(checkReturnStatements(1, 0));
    try std.testing.expect(!checkReturnStatements(12, 3));
}

test "934: documentation coverage zero - zero documented" {
    const coverage = checkDocumentationCoverage(10, 0);
    try std.testing.expect(coverage == 0);
}

test "935: test coverage zero - zero coverage" {
    const coverage = checkTestCoverage(0, 100);
    try std.testing.expect(coverage == 0);
}

test "936: quality score bounds - score bounds" {
    const complexity = analyzeComplexity(10, 1, 1);
    const score = calculateQualityScore(complexity, 0);
    try std.testing.expect(score <= 100);
    try std.testing.expect(score >= 0);
}

test "937: smell limit enforcement - enforce limit" {
    init();
    var i: u8 = 0;
    while (i < 40) : (i += 1) {
        recordSmell("smell", "low", i, "Smell");
    }
    try std.testing.expect(getSmellCount() == MAX_CODE_SMELLS);
}

test "938: dependency limit enforcement - enforce limit" {
    init();
    var i: u8 = 0;
    while (i < 70) : (i += 1) {
        addDependency("dep", "1.0.0", true);
    }
    try std.testing.expect(getDependencyCount() == MAX_DEPENDENCIES);
}

test "939: empty source handling - handle empty" {
    const complexity = analyzeComplexity(0, 0, 0);
    try std.testing.expect(complexity.cyclomatic_complexity == 1);
}

test "940: very long function detection - detect very long" {
    try std.testing.expect(detectLongFunction(500));
    try std.testing.expect(detectLongFunction(1000));
}

test "941: very deep nesting detection - detect very deep" {
    try std.testing.expect(detectDeepNesting(15));
    try std.testing.expect(detectDeepNesting(20));
}

test "942: negative magic numbers - check negative" {
    try std.testing.expect(checkMagicNumbers(-1));
    try std.testing.expect(!checkMagicNumbers(-42));
}

test "943: large parameter count - check large params" {
    try std.testing.expect(!checkParameterCount(15));
    try std.testing.expect(!checkParameterCount(20));
}

test "944: many duplicate occurrences - many duplicates" {
    try std.testing.expect(detectDuplicateCode("x", 10));
}

test "945: quality score precision - check precision" {
    const complexity = analyzeComplexity(50, 2, 3);
    const score = calculateQualityScore(complexity, 1);
    try std.testing.expectApproxEqAbs(@as(f64, 98.0), score, 0.0001);
}

test "946: complexity metric structure - check structure" {
    const complexity = analyzeComplexity(100, 5, 8);
    try std.testing.expect(complexity.lines_of_code > 0);
    try std.testing.expect(complexity.function_count > 0);
}

test "947: smell severity strings - check strings" {
    init();
    recordSmell("high_smell", "high", 10, "High");
    recordSmell("medium_smell", "medium", 20, "Medium");
    recordSmell("low_smell", "low", 30, "Low");
    try std.testing.expect(getSmellCount() == 3);
}

test "948: dependency info structure - check structure" {
    init();
    addDependency("test_dep", "1.2.3", true);
    try std.testing.expect(getDependencyCount() == 1);
}

test "949: quality report structure - check structure" {
    const complexity = analyzeComplexity(100, 5, 10);
    const report = generateQualityReport(complexity);
    try std.testing.expect(report.overall_score > 0);
}

test "950: comprehensive quality check - full check" {
    init();
    addDependency("zmath", "1.0.0", true);
    recordSmell("long_function", "medium", 100, "Function too long");
    const complexity = analyzeComplexity(200, 10, 25);
    const report = generateQualityReport(complexity);
    try std.testing.expect(report.smell_count > 0);
    try std.testing.expect(getDependencyCount() > 0);
}
