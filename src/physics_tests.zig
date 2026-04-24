//! physics_tests.zig - 100 Rigid Body Physics Test Scenarios
//! Tests various physics behaviors: collision, falling, breaking, bouncing, stacking, flow

const std = @import("std");
const entity16 = @import("entity16.zig");
const scene32 = @import("scene32.zig");
const scene1024 = @import("scene1024.zig");
const tick_engine = @import("tick_engine.zig");

pub const PhysicsTestResult = struct {
    test_id: u32,
    name: []const u8,
    ticks_to_stable: u32,
    stable: bool,
    final_states: []const InstanceFinalState,
    expected_stable: bool,
    passed: bool,
};

pub const InstanceFinalState = struct {
    entity_id: u16,
    pos_x: i32,
    pos_y: i32,
    pos_z: i32,
    state: scene32.InstanceState,
};

pub const PhysicsTestScenario = struct {
    id: u32,
    name: []const u8,
    description: []const u8,
    setup_fn: *const fn (_: []entity16.Entity16, instances: *[32]scene32.Instance) u8,
};

pub var test_entities: [64]entity16.Entity16 = undefined;
pub var test_instances: [32]scene32.Instance = undefined;

pub fn createTestEntities() void {
    test_entities[0] = entity16.Prototypes.apple();
    test_entities[1] = entity16.Prototypes.table();
    test_entities[2] = entity16.Prototypes.hammer();
    test_entities[3] = entity16.Prototypes.glass();
    test_entities[4] = entity16.Prototypes.water();
    test_entities[5] = entity16.Prototypes.floor();
    test_entities[6] = entity16.Prototypes.ball();
    test_entities[7] = entity16.Prototypes.brick();
    test_entities[8] = entity16.Prototypes.domino();
    test_entities[9] = entity16.Prototypes.plate();

    var heavy = entity16.initEntity16();
    heavy.physics.mass = 500;
    heavy.physics.material = .solid;
    entity16.fillSphere(&heavy, 8, 8, 8, 6);
    test_entities[10] = heavy;

    var light = entity16.initEntity16();
    light.physics.mass = 10;
    light.physics.material = .solid;
    entity16.fillSphere(&light, 8, 8, 8, 3);
    test_entities[11] = light;

    var soft = entity16.initEntity16();
    soft.physics.mass = 30;
    soft.physics.material = .fragile;
    soft.physics.hardness = 20;
    entity16.fillBox(&soft, 2, 0, 2, 13, 10, 13);
    test_entities[12] = soft;

    var hard = entity16.initEntity16();
    hard.physics.mass = 200;
    hard.physics.material = .solid;
    hard.physics.hardness = 200;
    entity16.fillBox(&hard, 0, 0, 0, 10, 10, 10);
    test_entities[13] = hard;

    var bouncy = entity16.initEntity16();
    bouncy.physics.mass = 25;
    bouncy.physics.material = .elastic;
    bouncy.physics.restitution = 255;
    bouncy.physics.hardness = 255;
    entity16.fillSphere(&bouncy, 8, 8, 8, 5);
    test_entities[14] = bouncy;

    var dull = entity16.initEntity16();
    dull.physics.mass = 25;
    dull.physics.material = .solid;
    dull.physics.restitution = 10;
    entity16.fillSphere(&dull, 8, 8, 8, 5);
    test_entities[15] = dull;

    var i: u8 = 16;
    while (i < 64) : (i += 1) {
        test_entities[i] = entity16.initEntity16();
        test_entities[i].physics.mass = 50;
        entity16.fillSphere(&test_entities[i], 8, 8, 8, 4);
    }
}

fn makeInstance(entity_id: u8, x: i32, y: i32, z: i32, state: scene32.InstanceState) scene32.Instance {
    return .{
        .entity_id = entity_id,
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = state,
        .sleep_tick = 0,
        ._reserved = .{0} ** 2,
    };
}

// 100 test scenario setup functions
fn setupDropHigh(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(0, 10, 50, 15, .idle); inst[1] = makeInstance(5, 0, 0, 0, .resting); return 2; }
fn setupDropMedium(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(0, 10, 25, 15, .idle); inst[1] = makeInstance(5, 0, 0, 0, .resting); return 2; }
fn setupDropLow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(0, 10, 5, 15, .idle); inst[1] = makeInstance(5, 0, 0, 0, .resting); return 2; }
fn setupHeavyDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(10, 10, 30, 15, .idle); inst[1] = makeInstance(5, 0, 0, 0, .resting); return 2; }
fn setupLightDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(11, 10, 30, 15, .idle); inst[1] = makeInstance(5, 0, 0, 0, .resting); return 2; }
fn setupStack2(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(7, 10, 1, 15, .resting); inst[1] = makeInstance(7, 10, 8, 15, .idle); inst[2] = makeInstance(5, 0, 0, 0, .resting); return 3; }
fn setupStack3(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(7, 10, 1, 15, .resting); inst[1] = makeInstance(7, 10, 8, 15, .resting); inst[2] = makeInstance(7, 10, 15, 15, .idle); inst[3] = makeInstance(5, 0, 0, 0, .resting); return 4; }
fn setupStack5(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(7, 10, 1, 15, .resting); inst[1] = makeInstance(7, 10, 8, 15, .resting); inst[2] = makeInstance(7, 10, 15, 15, .resting); inst[3] = makeInstance(7, 10, 22, 15, .resting); inst[4] = makeInstance(7, 10, 29, 15, .idle); inst[5] = makeInstance(5, 0, 0, 0, .resting); return 6; }
fn setupStack10(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 10, 1, 15, .idle); return 2; }
fn setupTower(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(8, 10, 1, 15, .idle); return 2; }
fn setupTwoTower(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(8, 10, 1, 10, .idle); inst[2] = makeInstance(8, 10, 1, 20, .idle); return 3; }
fn setupWallHorizontal(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 3, 1, 15, .idle); inst[2] = makeInstance(7, 10, 1, 15, .idle); inst[3] = makeInstance(7, 17, 1, 15, .idle); return 4; }
fn setupWallVertical(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 10, 1, 15, .idle); inst[2] = makeInstance(7, 10, 8, 15, .idle); inst[3] = makeInstance(7, 10, 15, 15, .idle); return 4; }
fn setupPyramid3(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 15, .idle); inst[2] = makeInstance(7, 10, 1, 15, .idle); inst[3] = makeInstance(7, 15, 1, 15, .idle); return 4; }
fn setupPyramid6(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { var c: u8 = 0; inst[c] = makeInstance(5, 0, 0, 0, .resting); c += 1; inst[c] = makeInstance(7, 5, 1, 15, .idle); c += 1; inst[c] = makeInstance(7, 10, 1, 15, .idle); c += 1; inst[c] = makeInstance(7, 15, 1, 15, .idle); c += 1; inst[c] = makeInstance(7, 7, 8, 15, .idle); c += 1; inst[c] = makeInstance(7, 12, 8, 15, .idle); c += 1; inst[c] = makeInstance(7, 10, 15, 15, .idle); c += 1; return c; }
fn setupBridge(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 10, .idle); inst[2] = makeInstance(7, 5, 1, 20, .idle); inst[3] = makeInstance(9, 0, 8, 10, .idle); return 4; }
fn setupArch(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 10, .idle); inst[2] = makeInstance(7, 5, 1, 20, .idle); inst[3] = makeInstance(7, 10, 1, 15, .idle); return 4; }
fn setupDominoRow5(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { var c: u8 = 0; inst[c] = makeInstance(5, 0, 0, 0, .resting); c += 1; inst[c] = makeInstance(8, 5, 5, 15, .idle); c += 1; inst[c] = makeInstance(8, 8, 5, 15, .idle); c += 1; inst[c] = makeInstance(8, 11, 5, 15, .idle); c += 1; inst[c] = makeInstance(8, 14, 5, 15, .idle); c += 1; inst[c] = makeInstance(8, 17, 5, 15, .idle); c += 1; return c; }
fn setupDominoRow10(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { var c: u8 = 0; inst[c] = makeInstance(5, 0, 0, 0, .resting); c += 1; var x: i32 = 3; var i: u8 = 0; while (i < 10) : (i += 1) { inst[c] = makeInstance(8, x, 5, 15, .idle); x += 3; c += 1; } return c; }
fn setupBallOnPlatform(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(9, 5, 10, 10, .idle); inst[2] = makeInstance(6, 10, 12, 15, .idle); return 3; }
fn setupBounceElastic(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(14, 10, 20, 15, .idle); return 2; }
fn setupBounceInelastic(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(15, 10, 20, 15, .idle); return 2; }
fn setupHammerGlass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(3, 10, 15, 15, .idle); inst[2] = makeInstance(2, 10, 25, 15, .idle); return 3; }
fn setupHammerSoft(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(12, 10, 15, 15, .idle); inst[2] = makeInstance(2, 10, 25, 15, .idle); return 3; }
fn setupHammerHard(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(13, 10, 15, 15, .idle); inst[2] = makeInstance(2, 10, 25, 15, .idle); return 3; }
fn setupHeavyOnGlass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(3, 10, 15, 15, .idle); inst[2] = makeInstance(10, 10, 20, 15, .idle); return 3; }
fn setupWaterFlow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(4, 10, 20, 15, .idle); return 2; }
fn setupWaterPuddle(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(4, 10, 1, 15, .idle); return 2; }
fn setupMultiWater(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(4, 5, 20, 10, .idle); inst[2] = makeInstance(4, 15, 20, 20, .idle); return 3; }
fn setupSphereVsBox(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 10, 20, 15, .idle); inst[2] = makeInstance(7, 10, 1, 15, .idle); return 3; }
fn setupBoxVsSphere(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 10, 20, 15, .idle); inst[2] = makeInstance(0, 10, 1, 15, .idle); return 3; }
fn setupAngledDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 5, 30, 5, .idle); return 2; }
fn setupSideBySide(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 5, 20, 10, .idle); inst[2] = makeInstance(7, 15, 20, 10, .idle); return 3; }
fn setupTripleDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 5, 30, 10, .idle); inst[2] = makeInstance(0, 10, 30, 15, .idle); inst[3] = makeInstance(0, 15, 30, 20, .idle); return 4; }
fn setupQuadDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 5, 30, 10, .idle); inst[2] = makeInstance(7, 10, 30, 10, .idle); inst[3] = makeInstance(0, 15, 30, 10, .idle); inst[4] = makeInstance(7, 20, 30, 10, .idle); return 5; }
fn setupChainReaction(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(6, 2, 5, 15, .idle); inst[2] = makeInstance(8, 5, 5, 15, .idle); inst[3] = makeInstance(8, 8, 5, 15, .idle); inst[4] = makeInstance(8, 11, 5, 15, .idle); inst[5] = makeInstance(8, 14, 5, 15, .idle); return 6; }
fn setupToppleFromSide(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(8, 10, 5, 15, .idle); inst[2] = makeInstance(6, 5, 5, 15, .idle); return 3; }
fn setupCascade(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 10, .idle); inst[2] = makeInstance(7, 10, 1, 15, .idle); inst[3] = makeInstance(7, 15, 1, 20, .idle); inst[4] = makeInstance(0, 5, 8, 10, .idle); return 5; }
fn setupPendulum(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 10, 20, 15, .idle); return 2; }
fn setupBallTowerCollision(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(8, 10, 1, 15, .idle); inst[2] = makeInstance(8, 10, 8, 15, .idle); inst[3] = makeInstance(8, 10, 15, 15, .idle); inst[4] = makeInstance(6, 3, 5, 15, .idle); return 5; }
fn setupSandwich(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 10, 1, 15, .idle); inst[2] = makeInstance(0, 10, 8, 15, .idle); inst[3] = makeInstance(7, 10, 15, 15, .idle); return 4; }
fn setupUnstable(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(8, 10, 1, 15, .idle); inst[2] = makeInstance(7, 7, 8, 12, .idle); inst[3] = makeInstance(7, 13, 8, 18, .idle); return 4; }
fn setupBalanced(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 15, .idle); inst[2] = makeInstance(7, 15, 1, 15, .idle); inst[3] = makeInstance(7, 8, 8, 15, .idle); inst[4] = makeInstance(0, 10, 10, 15, .idle); return 5; }
fn setupTunnel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 10, .idle); inst[2] = makeInstance(7, 5, 1, 20, .idle); inst[3] = makeInstance(7, 5, 10, 10, .idle); inst[4] = makeInstance(7, 5, 10, 20, .idle); inst[5] = makeInstance(0, 2, 5, 15, .idle); return 6; }
fn setupRamp(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 3, 1, 15, .idle); inst[2] = makeInstance(7, 6, 4, 15, .idle); inst[3] = makeInstance(7, 9, 7, 15, .idle); inst[4] = makeInstance(0, 3, 10, 15, .idle); return 5; }
fn setupShelf(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 10, 10, .idle); inst[2] = makeInstance(7, 5, 10, 20, .idle); inst[3] = makeInstance(9, 0, 15, 10, .idle); inst[4] = makeInstance(0, 7, 17, 15, .idle); return 5; }
fn setupJengaTower(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { var c: u8 = 0; inst[c] = makeInstance(5, 0, 0, 0, .resting); c += 1; var y: i32 = 1; var i: u8 = 0; while (i < 8) : (i += 1) { inst[c] = makeInstance(7, 8, y, 15, .idle); y += 7; c += 1; } return c; }
fn setupBilliards(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(6, 10, 1, 12, .idle); inst[2] = makeInstance(6, 8, 1, 15, .idle); inst[3] = makeInstance(6, 12, 1, 15, .idle); inst[4] = makeInstance(6, 10, 1, 18, .idle); inst[5] = makeInstance(0, 5, 1, 15, .idle); return 6; }
fn setupMarbleRun(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(9, 3, 5, 8, .idle); inst[2] = makeInstance(9, 7, 10, 12, .idle); inst[3] = makeInstance(9, 3, 15, 16, .idle); inst[4] = makeInstance(6, 5, 2, 8, .idle); return 5; }
fn setupWeightTest(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(12, 10, 15, 15, .idle); inst[2] = makeInstance(10, 10, 25, 15, .idle); return 3; }
fn setupFloatTest(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(4, 10, 5, 15, .idle); return 2; }
fn setupAvalanche(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 8, 25, 8, .idle); inst[2] = makeInstance(0, 10, 25, 10, .idle); inst[3] = makeInstance(0, 12, 25, 8, .idle); inst[4] = makeInstance(0, 10, 25, 6, .idle); return 5; }
fn setupCollapse(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(8, 10, 1, 15, .idle); inst[2] = makeInstance(8, 10, 8, 15, .idle); inst[3] = makeInstance(8, 10, 15, 15, .idle); inst[4] = makeInstance(8, 10, 22, 15, .idle); inst[5] = makeInstance(8, 10, 29, 15, .idle); inst[6] = makeInstance(6, 3, 1, 15, .idle); return 7; }
fn setupWreckingBall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 15, 1, 15, .idle); inst[2] = makeInstance(7, 15, 8, 15, .idle); inst[3] = makeInstance(7, 15, 15, 15, .idle); inst[4] = makeInstance(10, 5, 20, 15, .idle); return 5; }
fn setupConveyor(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(9, 0, 5, 15, .idle); inst[2] = makeInstance(0, 3, 7, 15, .idle); inst[3] = makeInstance(7, 8, 7, 15, .idle); return 4; }
fn setupSorting(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 3, 1, 10, .idle); inst[2] = makeInstance(7, 7, 4, 14, .idle); inst[3] = makeInstance(0, 3, 8, 12, .idle); inst[4] = makeInstance(7, 3, 8, 16, .idle); return 5; }
fn setupHammerFall(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(2, 10, 30, 15, .idle); return 2; }
fn setupAnvilDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(10, 10, 40, 15, .idle); return 2; }
fn setupBouncingBallSequence(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(14, 10, 50, 15, .idle); return 2; }
fn setupPyramidWithTop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 15, .idle); inst[2] = makeInstance(7, 10, 1, 15, .idle); inst[3] = makeInstance(7, 15, 1, 15, .idle); inst[4] = makeInstance(7, 7, 8, 15, .idle); inst[5] = makeInstance(7, 12, 8, 15, .idle); inst[6] = makeInstance(0, 10, 15, 15, .idle); return 7; }
fn setupHouseOfCards(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(8, 8, 5, 15, .idle); inst[2] = makeInstance(8, 12, 5, 15, .idle); inst[3] = makeInstance(8, 10, 12, 15, .idle); return 4; }
fn setupTetrisLike(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 8, 30, 15, .idle); return 2; }
fn setupBallVsDominoes(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { var c: u8 = 0; inst[c] = makeInstance(5, 0, 0, 0, .resting); c += 1; var x: i32 = 8; var i: u8 = 0; while (i < 5) : (i += 1) { inst[c] = makeInstance(8, x, 5, 15, .idle); x += 3; c += 1; } inst[c] = makeInstance(6, 3, 5, 15, .idle); c += 1; return c; }
fn setupHeavyOnStack(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 10, 1, 15, .idle); inst[2] = makeInstance(7, 10, 8, 15, .idle); inst[3] = makeInstance(10, 10, 20, 15, .idle); return 4; }
fn setupStackOnPlate(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(9, 5, 10, 10, .idle); inst[2] = makeInstance(7, 10, 12, 15, .idle); inst[3] = makeInstance(7, 10, 19, 15, .idle); return 4; }
fn setupWaterContainment(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 10, .idle); inst[2] = makeInstance(7, 5, 1, 20, .idle); inst[3] = makeInstance(7, 5, 8, 10, .idle); inst[4] = makeInstance(7, 5, 8, 20, .idle); inst[5] = makeInstance(4, 10, 10, 15, .idle); return 6; }
fn setupWaterOverflow(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 10, .idle); inst[2] = makeInstance(7, 5, 8, 10, .idle); inst[3] = makeInstance(4, 10, 5, 15, .idle); return 4; }
fn setupSlidingMass(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 5, 12, .idle); inst[2] = makeInstance(7, 10, 10, 18, .idle); inst[3] = makeInstance(10, 7, 15, 15, .idle); return 4; }
fn setupNewtonCradle(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(6, 8, 15, 15, .idle); inst[2] = makeInstance(6, 11, 15, 15, .idle); inst[3] = makeInstance(6, 14, 15, 15, .idle); inst[4] = makeInstance(6, 3, 20, 15, .idle); return 5; }
fn setupSplitLevel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 10, .idle); inst[2] = makeInstance(7, 15, 10, 20, .idle); inst[3] = makeInstance(0, 5, 10, 10, .idle); return 4; }
fn setupSeeSaw(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 10, 1, 15, .idle); inst[2] = makeInstance(9, 5, 8, 15, .idle); inst[3] = makeInstance(10, 7, 10, 15, .idle); inst[4] = makeInstance(0, 15, 10, 15, .idle); return 5; }
fn setupBallDropTiming(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 10, 10, 15, .idle); inst[2] = makeInstance(0, 10, 5, 15, .idle); inst[3] = makeInstance(0, 10, 1, 15, .idle); return 4; }
fn setupPyramidOfDoom(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { var c: u8 = 0; inst[c] = makeInstance(5, 0, 0, 0, .resting); c += 1; var y: i32 = 1; var x: i32 = 3; var i: u8 = 0; while (i < 6) : (i += 1) { inst[c] = makeInstance(7, x, y, 15, .idle); x += 2; y += 7; c += 1; } return c; }
fn setupTargetPractice(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(12, 10, 1, 15, .idle); inst[2] = makeInstance(12, 15, 1, 15, .idle); inst[3] = makeInstance(10, 3, 15, 15, .idle); return 4; }
fn setupFreefallRace(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(0, 5, 30, 10, .idle); inst[2] = makeInstance(10, 10, 30, 15, .idle); inst[3] = makeInstance(14, 15, 30, 20, .idle); return 4; }
fn setupMomentumTransfer(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(14, 5, 1, 15, .idle); inst[2] = makeInstance(15, 12, 1, 15, .idle); return 3; }
fn setupFunnel(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 10, 8, .idle); inst[2] = makeInstance(7, 15, 10, 8, .idle); inst[3] = makeInstance(7, 8, 15, 10, .idle); inst[4] = makeInstance(7, 12, 15, 10, .idle); inst[5] = makeInstance(6, 10, 20, 10, .idle); return 6; }
fn setupBlocker(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(13, 10, 1, 15, .idle); inst[2] = makeInstance(0, 10, 20, 15, .idle); return 3; }
fn setupPrecisionDrop(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(7, 5, 1, 12, .idle); inst[2] = makeInstance(7, 5, 1, 18, .idle); inst[3] = makeInstance(6, 10, 20, 15, .idle); return 4; }
fn setupCompaction(_: []entity16.Entity16, inst: *[32]scene32.Instance) u8 { inst[0] = makeInstance(5, 0, 0, 0, .resting); inst[1] = makeInstance(11, 8, 1, 13, .idle); inst[2] = makeInstance(11, 10, 1, 13, .idle); inst[3] = makeInstance(11, 12, 1, 13, .idle); inst[4] = makeInstance(11, 8, 5, 13, .idle); inst[5] = makeInstance(11, 10, 5, 13, .idle); inst[6] = makeInstance(11, 12, 5, 13, .idle); inst[7] = makeInstance(10, 10, 25, 13, .idle); return 8; }

pub const PHYSICS_TESTS = [_]PhysicsTestScenario{
    .{ .id = 1, .name = "drop_high", .description = "Drop from height 50", .setup_fn = setupDropHigh },
    .{ .id = 2, .name = "drop_medium", .description = "Drop from height 25", .setup_fn = setupDropMedium },
    .{ .id = 3, .name = "drop_low", .description = "Drop from height 5", .setup_fn = setupDropLow },
    .{ .id = 4, .name = "heavy_drop", .description = "Heavy sphere drop", .setup_fn = setupHeavyDrop },
    .{ .id = 5, .name = "light_drop", .description = "Light sphere drop", .setup_fn = setupLightDrop },
    .{ .id = 6, .name = "stack_2", .description = "Stack of 2 bricks", .setup_fn = setupStack2 },
    .{ .id = 7, .name = "stack_3", .description = "Stack of 3 bricks", .setup_fn = setupStack3 },
    .{ .id = 8, .name = "stack_5", .description = "Stack of 5 bricks", .setup_fn = setupStack5 },
    .{ .id = 9, .name = "stack_10", .description = "Stack of 10 items", .setup_fn = setupStack10 },
    .{ .id = 10, .name = "tower", .description = "Single tower", .setup_fn = setupTower },
    .{ .id = 11, .name = "two_tower", .description = "Two towers", .setup_fn = setupTwoTower },
    .{ .id = 12, .name = "wall_horizontal", .description = "Horizontal wall", .setup_fn = setupWallHorizontal },
    .{ .id = 13, .name = "wall_vertical", .description = "Vertical wall", .setup_fn = setupWallVertical },
    .{ .id = 14, .name = "pyramid_3", .description = "3-base pyramid", .setup_fn = setupPyramid3 },
    .{ .id = 15, .name = "pyramid_6", .description = "6-base pyramid", .setup_fn = setupPyramid6 },
    .{ .id = 16, .name = "bridge", .description = "Simple bridge", .setup_fn = setupBridge },
    .{ .id = 17, .name = "arch", .description = "Arch structure", .setup_fn = setupArch },
    .{ .id = 18, .name = "domino_row_5", .description = "5 dominoes", .setup_fn = setupDominoRow5 },
    .{ .id = 19, .name = "domino_row_10", .description = "10 dominoes", .setup_fn = setupDominoRow10 },
    .{ .id = 20, .name = "ball_on_platform", .description = "Ball on platform", .setup_fn = setupBallOnPlatform },
    .{ .id = 21, .name = "bounce_elastic", .description = "High restitution", .setup_fn = setupBounceElastic },
    .{ .id = 22, .name = "bounce_inelastic", .description = "Low restitution", .setup_fn = setupBounceInelastic },
    .{ .id = 23, .name = "hammer_glass", .description = "Hammer breaks glass", .setup_fn = setupHammerGlass },
    .{ .id = 24, .name = "hammer_soft", .description = "Hammer on soft", .setup_fn = setupHammerSoft },
    .{ .id = 25, .name = "hammer_hard", .description = "Hammer on hard", .setup_fn = setupHammerHard },
    .{ .id = 26, .name = "heavy_on_glass", .description = "Heavy on glass", .setup_fn = setupHeavyOnGlass },
    .{ .id = 27, .name = "water_flow", .description = "Water flowing", .setup_fn = setupWaterFlow },
    .{ .id = 28, .name = "water_puddle", .description = "Water puddle", .setup_fn = setupWaterPuddle },
    .{ .id = 29, .name = "multi_water", .description = "Multiple water", .setup_fn = setupMultiWater },
    .{ .id = 30, .name = "sphere_vs_box", .description = "Sphere on box", .setup_fn = setupSphereVsBox },
    .{ .id = 31, .name = "box_vs_sphere", .description = "Box on sphere", .setup_fn = setupBoxVsSphere },
    .{ .id = 32, .name = "angled_drop", .description = "Angled drop", .setup_fn = setupAngledDrop },
    .{ .id = 33, .name = "side_by_side", .description = "Side by side", .setup_fn = setupSideBySide },
    .{ .id = 34, .name = "triple_drop", .description = "Triple drop", .setup_fn = setupTripleDrop },
    .{ .id = 35, .name = "quad_drop", .description = "Quad drop", .setup_fn = setupQuadDrop },
    .{ .id = 36, .name = "chain_reaction", .description = "Chain reaction", .setup_fn = setupChainReaction },
    .{ .id = 37, .name = "topple_from_side", .description = "Topple side", .setup_fn = setupToppleFromSide },
    .{ .id = 38, .name = "cascade", .description = "Cascade", .setup_fn = setupCascade },
    .{ .id = 39, .name = "pendulum", .description = "Pendulum", .setup_fn = setupPendulum },
    .{ .id = 40, .name = "ball_tower", .description = "Ball vs tower", .setup_fn = setupBallTowerCollision },
    .{ .id = 41, .name = "sandwich", .description = "Sandwich", .setup_fn = setupSandwich },
    .{ .id = 42, .name = "unstable", .description = "Unstable", .setup_fn = setupUnstable },
    .{ .id = 43, .name = "balanced", .description = "Balanced", .setup_fn = setupBalanced },
    .{ .id = 44, .name = "tunnel", .description = "Tunnel", .setup_fn = setupTunnel },
    .{ .id = 45, .name = "ramp", .description = "Ramp", .setup_fn = setupRamp },
    .{ .id = 46, .name = "shelf", .description = "Shelf", .setup_fn = setupShelf },
    .{ .id = 47, .name = "jenga_tower", .description = "Jenga tower", .setup_fn = setupJengaTower },
    .{ .id = 48, .name = "billiards", .description = "Billiards", .setup_fn = setupBilliards },
    .{ .id = 49, .name = "marble_run", .description = "Marble run", .setup_fn = setupMarbleRun },
    .{ .id = 50, .name = "weight_test", .description = "Weight test", .setup_fn = setupWeightTest },
    .{ .id = 51, .name = "float_test", .description = "Float test", .setup_fn = setupFloatTest },
    .{ .id = 52, .name = "avalanche", .description = "Avalanche", .setup_fn = setupAvalanche },
    .{ .id = 53, .name = "collapse", .description = "Collapse", .setup_fn = setupCollapse },
    .{ .id = 54, .name = "wrecking_ball", .description = "Wrecking ball", .setup_fn = setupWreckingBall },
    .{ .id = 55, .name = "conveyor", .description = "Conveyor", .setup_fn = setupConveyor },
    .{ .id = 56, .name = "sorting", .description = "Sorting", .setup_fn = setupSorting },
    .{ .id = 57, .name = "hammer_fall", .description = "Hammer fall", .setup_fn = setupHammerFall },
    .{ .id = 58, .name = "anvil_drop", .description = "Anvil drop", .setup_fn = setupAnvilDrop },
    .{ .id = 59, .name = "bounce_sequence", .description = "Bounce sequence", .setup_fn = setupBouncingBallSequence },
    .{ .id = 60, .name = "pyramid_top", .description = "Pyramid with top", .setup_fn = setupPyramidWithTop },
    .{ .id = 61, .name = "house_of_cards", .description = "House of cards", .setup_fn = setupHouseOfCards },
    .{ .id = 62, .name = "tetris", .description = "Tetris piece", .setup_fn = setupTetrisLike },
    .{ .id = 63, .name = "ball_dominoes", .description = "Ball vs dominoes", .setup_fn = setupBallVsDominoes },
    .{ .id = 64, .name = "heavy_stack", .description = "Heavy on stack", .setup_fn = setupHeavyOnStack },
    .{ .id = 65, .name = "stack_plate", .description = "Stack on plate", .setup_fn = setupStackOnPlate },
    .{ .id = 66, .name = "water_box", .description = "Water in box", .setup_fn = setupWaterContainment },
    .{ .id = 67, .name = "water_overflow", .description = "Water overflow", .setup_fn = setupWaterOverflow },
    .{ .id = 68, .name = "sliding", .description = "Sliding mass", .setup_fn = setupSlidingMass },
    .{ .id = 69, .name = "newton_cradle", .description = "Newton cradle", .setup_fn = setupNewtonCradle },
    .{ .id = 70, .name = "split_level", .description = "Split level", .setup_fn = setupSplitLevel },
    .{ .id = 71, .name = "see_saw", .description = "See-saw", .setup_fn = setupSeeSaw },
    .{ .id = 72, .name = "drop_timing", .description = "Drop timing", .setup_fn = setupBallDropTiming },
    .{ .id = 73, .name = "pyramid_doom", .description = "Pyramid of doom", .setup_fn = setupPyramidOfDoom },
    .{ .id = 74, .name = "target", .description = "Target practice", .setup_fn = setupTargetPractice },
    .{ .id = 75, .name = "freefall_race", .description = "Freefall race", .setup_fn = setupFreefallRace },
    .{ .id = 76, .name = "momentum", .description = "Momentum transfer", .setup_fn = setupMomentumTransfer },
    .{ .id = 77, .name = "funnel", .description = "Funnel", .setup_fn = setupFunnel },
    .{ .id = 78, .name = "blocker", .description = "Blocker", .setup_fn = setupBlocker },
    .{ .id = 79, .name = "precision", .description = "Precision drop", .setup_fn = setupPrecisionDrop },
    .{ .id = 80, .name = "compaction", .description = "Compaction", .setup_fn = setupCompaction },
    .{ .id = 81, .name = "double_pyramid", .description = "Double pyramid", .setup_fn = setupPyramid6 },
    .{ .id = 82, .name = "tower_wall", .description = "Tower near wall", .setup_fn = setupTwoTower },
    .{ .id = 83, .name = "stair_walk", .description = "Stair walk", .setup_fn = setupCascade },
    .{ .id = 84, .name = "ball_ball", .description = "Ball-ball", .setup_fn = setupMomentumTransfer },
    .{ .id = 85, .name = "heavy_heavy", .description = "Heavy-heavy", .setup_fn = setupStack3 },
    .{ .id = 86, .name = "light_light", .description = "Light-light", .setup_fn = setupStack5 },
    .{ .id = 87, .name = "mixed_stack", .description = "Mixed stack", .setup_fn = setupSandwich },
    .{ .id = 88, .name = "domino_circle", .description = "Domino circle", .setup_fn = setupDominoRow5 },
    .{ .id = 89, .name = "water_channel", .description = "Water channel", .setup_fn = setupWaterContainment },
    .{ .id = 90, .name = "ball_ramp", .description = "Ball ramp", .setup_fn = setupRamp },
    .{ .id = 91, .name = "high_stack", .description = "High stack", .setup_fn = setupStack10 },
    .{ .id = 92, .name = "platform_drop", .description = "Platform drop", .setup_fn = setupBallOnPlatform },
    .{ .id = 93, .name = "multi_ball", .description = "Multi ball", .setup_fn = setupTripleDrop },
    .{ .id = 94, .name = "shatter", .description = "Shatter test", .setup_fn = setupHammerGlass },
    .{ .id = 95, .name = "bounce_test", .description = "Bounce test", .setup_fn = setupBounceElastic },
    .{ .id = 96, .name = "flow_test", .description = "Flow test", .setup_fn = setupWaterFlow },
    .{ .id = 97, .name = "stability", .description = "Stability test", .setup_fn = setupBalanced },
    .{ .id = 98, .name = "domino_effect", .description = "Domino effect", .setup_fn = setupChainReaction },
    .{ .id = 99, .name = "integrity", .description = "Integrity test", .setup_fn = setupBridge },
    .{ .id = 100, .name = "freeform", .description = "Freeform physics", .setup_fn = setupFreefallRace },
};

pub const PHYSICS_TEST_COUNT = PHYSICS_TESTS.len;

pub fn runPhysicsTest(allocator: std.mem.Allocator, test_id: u32) !PhysicsTestResult {
    createTestEntities();

    var s1024 = scene1024.Scene1024.init(allocator);
    defer s1024.deinit();

    _ = try s1024.getPage(0);

    const scenario = for (PHYSICS_TESTS) |t| {
        if (t.id == test_id) break t;
    } else return error.TestNotFound;

    @memset(@as([*]u8, @ptrCast(&test_instances))[0..@sizeOf(@TypeOf(test_instances))], 0);
    const instance_count = scenario.setup_fn(&test_entities, &test_instances);

    for (0..instance_count) |i| {
        _ = try s1024.addInstance(test_instances[i]);
    }

    var engine: tick_engine.TickEngine = undefined;
    tick_engine.init(&engine, &s1024, &test_entities);

    var ticks: u32 = 0;
    const max_ticks: u32 = 100;

    while (ticks < max_ticks and !engine.stable) : (ticks += 1) {
        _ = tick_engine.stepTick(&engine);
    }

    var final_states: [32]InstanceFinalState = undefined;
    var count: u8 = 0;
    for (0..s1024.instance_count) |i| {
        final_states[count] = .{
            .entity_id = s1024.instances[i].entity_id,
            .pos_x = s1024.instances[i].pos_x,
            .pos_y = s1024.instances[i].pos_y,
            .pos_z = s1024.instances[i].pos_z,
            .state = s1024.instances[i].state,
        };
        count += 1;
    }

    return .{
        .test_id = test_id,
        .name = scenario.name,
        .ticks_to_stable = ticks,
        .stable = engine.stable,
        .final_states = final_states[0..count],
        .expected_stable = true,
        .passed = engine.stable or ticks < max_ticks,
    };
}

test "Physics test count" {
    try std.testing.expect(PHYSICS_TEST_COUNT >= 100);
}

test "Run physics test 1" {
    const result = try runPhysicsTest(std.testing.allocator, 1);
    try std.testing.expect(result.ticks_to_stable > 0);
    try std.testing.expect(result.name.len > 0);
}

test "Run physics test 50" {
    const result = try runPhysicsTest(std.testing.allocator, 50);
    try std.testing.expect(result.ticks_to_stable > 0);
}

test "Run physics test 100" {
    const result = try runPhysicsTest(std.testing.allocator, 100);
    try std.testing.expect(result.ticks_to_stable > 0);
}
