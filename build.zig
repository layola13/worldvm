const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const exe = b.addExecutable(.{
        .name = "worldvm",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });

    // Strip symbols in ReleaseSmall
    if (optimize == .ReleaseSmall) {
        exe.root_module.strip = true;
    }

    b.installArtifact(exe);

    // Shared library for Python FFI
    const lib = b.addSharedLibrary(.{
        .name = "worldvm",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/vm_hook.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    b.installArtifact(lib);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const test_files = [_][]const u8{
        "src/address.zig",
        "src/physics_test.zig",
        "src/scene1024_test.zig",
        "src/mind_test.zig",
        "src/sdf_test.zig",
        "src/bus_test.zig",
        "src/vm_hook_test.zig",
        "src/bench.zig",
        "src/physics_tests.zig",
    };

    const test_step = b.step("test", "Run all unit tests");

    inline for (test_files) |path| {
        const t = b.addTest(.{
            .root_module = b.createModule(.{
                .root_source_file = b.path(path),
                .target = target,
                .optimize = optimize,
            }),
        });
        const rt = b.addRunArtifact(t);
        test_step.dependOn(&rt.step);
    }
}
