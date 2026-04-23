const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.graph.host; // Use host arch
    const optimize = std.builtin.OptimizeMode.Debug;

    const exe = b.addExecutable(.{
        .name = "worldvm",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }
}
