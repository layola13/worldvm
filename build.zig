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

    // P1 fix: add explicit "run" step so `zig build run` works
    const run_step = b.step("run", "Run the worldvm executable");
    run_step.dependOn(&run_cmd.step);

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
        "src/chapter01_motion_gravity.zig",
        "src/chapter02_collision_detection.zig",
        "src/chapter03_friction_elasticity.zig",
        "src/chapter04_stacking_sleep.zig",
        "src/chapter05_joints.zig",
        "src/chapter06_highspeed_ccd.zig",
        "src/chapter07_raycasting.zig",
        "src/chapter08_kinematic_dynamic.zig",
        "src/chapter09_extreme.zig",
        "src/chapter10_fluids.zig",
        "src/chapter11_kcc.zig",
        "src/chapter12_ballistics.zig",
        "src/chapter13_destruction.zig",
        "src/chapter14_ragdoll.zig",
        "src/chapter15_vehicle.zig",
        "src/chapter16_network.zig",
        "src/chapter17_crash_defense.zig",
        "src/chapter18_raycasting.zig",
        "src/chapter19_joints.zig",
        "src/chapter20_ccd.zig",
        "src/chapter21_fluids.zig",
        "src/chapter22_particles.zig",
        "src/chapter23_softbody.zig",
        "src/chapter24_terrain.zig",
        "src/chapter25_forces.zig",
        "src/chapter26_composite.zig",
        "src/chapter27_scalability.zig",
        "src/chapter28_determinism.zig",
        "src/chapter29_realtime.zig",
        "src/chapter30_integration.zig",
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
