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
        "src/main.zig",
        "src/address.zig",
        "src/entity16.zig",
        "src/scene32.zig",
        "src/scene1024.zig",
        "src/physics.zig",
        "src/collision.zig",
        "src/ccd.zig",
        "src/contact_response.zig",
        "src/physics_kernel.zig",
        "src/physics_world.zig",
        "src/tick_engine.zig",
        "src/sleep_response.zig",
        "src/material_pairing.zig",
        "src/terrain.zig",
        "src/kcc.zig",
        "src/joint.zig",
        "src/rewind.zig",
        "src/ragdoll.zig",
        "src/ai_traffic.zig",
        "src/tire.zig",
        "src/suspension.zig",
        "src/drivetrain.zig",
        "src/braking.zig",
        "src/aerodynamics.zig",
        "src/soft_tissue.zig",
        "src/sports.zig",
        "src/bus.zig",
        "src/crash_defense.zig",
        "src/biomechanics.zig",
        "src/planner.zig",
        "src/safety.zig",
        "src/weather.zig",
        "src/destruction.zig",
        "src/ballistics.zig",
        "src/query_types.zig",
        "src/query_debug.zig",
        "src/query_benchmark.zig",
        "src/query_regression.zig",
        "src/raycast.zig",
        "src/network.zig",
        "src/disasters.zig",
        "src/sensors.zig",
        "src/break_response.zig",
        "src/collision_event.zig",
        "src/renderer.zig",
        "src/scenarios.zig",
        "src/mind.zig",
        "src/query.zig",
        "src/query_world.zig",
        "src/query_raycast.zig",
        "src/query_overlap.zig",
        "src/query_sweep.zig",
        "src/query_penetration.zig",
        "src/prediction.zig",
        "src/vehicle.zig",
        "src/physics_test.zig",
        "src/scene1024_test.zig",
        "src/mind_test.zig",
        "src/sdf_test.zig",
        "src/sdf.zig",
        "src/chapter_test_support.zig",
        "src/bus_test.zig",
        "src/vm_hook.zig",
        "src/vm_hook_test.zig",
        "src/bench.zig",
        "src/physics_tests.zig",
        "src/physics_systems_test.zig",
        "src/test_reporter.zig",
        "src/performance_profiler.zig",
        "src/code_quality.zig",
        "src/toolchain.zig",
        "src/fluid.zig",
        "src/particle.zig",
        "src/softbody.zig",
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
    const test_full_step = b.step("test-full", "Run the full Zig test matrix");
    const test_fast_step = b.step("test-fast", "Run core Zig tests for fast iteration");

    const check_matrix_cmd = b.addSystemCommand(&.{
        "python3",
        "tools/check_zig_test_matrix.py",
    });
    const check_matrix_step = b.step("check-matrix", "Verify every src/*.zig file is in the test matrix");
    check_matrix_step.dependOn(&check_matrix_cmd.step);
    test_step.dependOn(&check_matrix_cmd.step);
    test_full_step.dependOn(&check_matrix_cmd.step);

    const fast_test_files = [_][]const u8{
        "src/address.zig",
        "src/entity16.zig",
        "src/scene32.zig",
        "src/scene1024.zig",
        "src/query_types.zig",
        "src/query_world.zig",
        "src/query_raycast.zig",
        "src/query_overlap.zig",
        "src/query_sweep.zig",
        "src/query_penetration.zig",
        "src/physics.zig",
        "src/collision.zig",
        "src/ccd.zig",
        "src/contact_response.zig",
        "src/physics_world.zig",
        "src/tick_engine.zig",
        "src/sleep_response.zig",
        "src/material_pairing.zig",
        "src/terrain.zig",
        "src/break_response.zig",
        "src/collision_event.zig",
    };

    inline for (fast_test_files) |path| {
        const t = b.addTest(.{
            .root_module = b.createModule(.{
                .root_source_file = b.path(path),
                .target = target,
                .optimize = optimize,
            }),
        });
        t.setExecCmd(&[_]?[]const u8{
            "timeout",
            "120s",
            null,
        });
        const rt = b.addRunArtifact(t);
        test_fast_step.dependOn(&rt.step);
    }

    inline for (test_files) |path| {
        const t = b.addTest(.{
            .root_module = b.createModule(.{
                .root_source_file = b.path(path),
                .target = target,
                .optimize = optimize,
            }),
        });
        // Enforce per-test-binary timeout (120s max per test execution).
        t.setExecCmd(&[_]?[]const u8{
            "timeout",
            "120s",
            null,
        });
        const rt = b.addRunArtifact(t);
        test_step.dependOn(&rt.step);
        test_full_step.dependOn(&rt.step);
    }
}
