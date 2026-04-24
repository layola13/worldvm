# Physics Test Cases (100 Scenarios)

This document describes the 100 physics test scenarios implemented in `physics_tests.zig`.

## Test Infrastructure

- **Test Count**: 100 scenarios
- **Run Command**: `zig test src/physics_tests.zig`
- **API**: `runPhysicsTest(allocator, test_id)` returns `PhysicsTestResult`

## Entity Types

| ID | Type | Mass | Material | Properties |
|----|------|------|----------|------------|
| 0 | Apple (sphere) | 50 | solid | Basic test object |
| 1 | Table | 500 | solid | Fixed platform |
| 2 | Hammer | 1000 | solid | Heavy impact |
| 3 | Glass | 30 | fragile | hardness=30, breaks easily |
| 4 | Water | 10 | liquid | Flows in 5 directions |
| 5 | Floor | 0 | solid | Fixed infinite plane |
| 6 | Ball (elastic) | 25 | elastic | restitution=200 |
| 7 | Brick | 100 | solid | Standard building block |
| 8 | Domino | 30 | solid | Tall thin shape |
| 9 | Plate | 50 | solid | Fixed platform |
| 10 | Heavy Sphere | 500 | solid | Large mass |
| 11 | Light Sphere | 10 | solid | Small mass |
| 12 | Soft Object | 30 | fragile | hardness=20 |
| 13 | Hard Object | 200 | solid | hardness=200 |
| 14 | Bouncy Ball | 25 | elastic | restitution=255 |
| 15 | Dull Ball | 25 | solid | restitution=10 |

---

## Test Case Definitions

### Drop Tests (ID: 1-5)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 1 | drop_high | Drop apple from height 50 | Stable quickly |
| 2 | drop_medium | Drop apple from height 25 | Stable quickly |
| 3 | drop_low | Drop apple from height 5 | Stable quickly |
| 4 | heavy_drop | Heavy sphere (mass=500) drop | Falls faster |
| 5 | light_drop | Light sphere (mass=10) drop | Same rate (gravity) |

### Stacking Tests (ID: 6-11)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 6 | stack_2 | Two bricks stacked | Stable |
| 7 | stack_3 | Three bricks stacked | Stable or topple |
| 8 | stack_5 | Five bricks stacked | May topple |
| 9 | stack_10 | Ten items stacked | Likely unstable |
| 10 | tower | Single domino tower | Stable or fall |
| 11 | two_tower | Two towers side by side | Independent |

### Structure Tests (ID: 12-17)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 12 | wall_horizontal | Three bricks in a row | Stable wall |
| 13 | wall_vertical | Three bricks stacked vertically | May topple |
| 14 | pyramid_3 | 3-base pyramid | Stable |
| 15 | pyramid_6 | 6-base pyramid | Stable structure |
| 16 | bridge | Platform on two pillars | Stable |
| 17 | arch | Arch structure | Tests compression |

### Domino Tests (ID: 18-19)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 18 | domino_row_5 | Five dominoes in a row | Cascade |
| 19 | domino_row_10 | Ten dominoes in a row | Long cascade |

### Bounce Tests (ID: 20-22)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 20 | ball_on_platform | Ball on raised plate | Bounces |
| 21 | bounce_elastic | High restitution ball (255) | Many bounces |
| 22 | bounce_inelastic | Low restitution ball (10) | Few bounces |

### Impact/Break Tests (ID: 23-26)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 23 | hammer_glass | Hammer breaks glass | Glass breaks |
| 24 | hammer_soft | Hammer on soft (hardness=20) | Soft breaks |
| 25 | hammer_hard | Hammer on hard (hardness=200) | Neither breaks |
| 26 | heavy_on_glass | Heavy sphere on glass | Glass may break |

### Liquid Tests (ID: 27-29)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 27 | water_flow | Water flowing down | Spreads |
| 28 | water_puddle | Water on floor | Expands |
| 29 | multi_water | Two water sources | Independent flows |

### Collision Tests (ID: 30-35)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 30 | sphere_vs_box | Sphere lands on box | Stack forms |
| 31 | box_vs_sphere | Box lands on sphere | Unstable |
| 32 | angled_drop | Drop from corner | Rolls/slides |
| 33 | side_by_side | Two objects drop | Parallel fall |
| 34 | triple_drop | Three simultaneous drops | Stack |
| 35 | quad_drop | Four simultaneous drops | Complex stack |

### Chain Reaction Tests (ID: 36-40)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 36 | chain_reaction | Ball hits dominoes | Cascade |
| 37 | topple_from_side | Push domino from side | Fall sideways |
| 38 | cascade | Stair descent | Rolls down |
| 39 | pendulum | Ball on string | Swing |
| 40 | ball_tower_collision | Ball hits tower | Tower falls |

### Complex Structure Tests (ID: 41-49)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 41 | sandwich | Object between plates | Stable |
| 42 | unstable | Narrow base wide top | Tips over |
| 43 | balanced | Balanced stack | Stable |
| 44 | tunnel | Ball through tunnel | Passes through |
| 45 | ramp | Ball down incline | Accelerates |
| 46 | shelf | Shelf with items | Items rest |
| 47 | jenga_tower | 8-layer horizontal stack | Fragile |
| 48 | billiards | Ball triangle + cue | Scatter |
| 49 | marble_run | Zigzag chute | Follows path |

### Material Tests (ID: 50-54)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 50 | weight_test | Heavy on fragile | Break test |
| 51 | float_test | Liquid material | Flows |
| 52 | avalanche | Ball pile collapse | Cascades |
| 53 | collapse | Tall tower hit at base | Falls |
| 54 | wrecking_ball | Heavy hits wall | Wall breaks |

### Mechanism Tests (ID: 55-59)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 55 | conveyor | Items on slope | Slide |
| 56 | sorting | Ball vs brick separation | Different paths |
| 57 | hammer_fall | Simple hammer drop | Impact |
| 58 | anvil_drop | Very heavy drop | Strong impact |
| 59 | bouncing_ball_sequence | Multiple bounces | Decays |

### Advanced Structure Tests (ID: 60-67)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 60 | pyramid_with_top | Pyramid with apple on top | Structure stable |
| 61 | house_of_cards | Triangular domino arrangement | Tips easily |
| 62 | tetris_like | Tetromino shape drop | Rotation matters |
| 63 | ball_vs_dominoes | Ball hits domino row | Cascade |
| 64 | heavy_on_stack | Heavy on stack top | Stack crushes |
| 65 | stack_on_plate | Stack on platform | Platform holds |
| 66 | water_containment | Water in box | Stays inside |
| 67 | water_overflow | Water exceeds container | Spills out |

### Motion Tests (ID: 68-76)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 68 | sliding_mass | Mass on slope | Slides |
| 69 | newton_cradle | 4 balls + 1 raised | Momentum transfer |
| 70 | split_level | Different height platforms | Falls to lower |
| 71 | see_saw | Pivot with weights | Tips |
| 72 | ball_drop_timing | Multiple drop timings | Staggered |
| 73 | pyramid_of_doom | Large pyramid | Stable |
| 74 | target_practice | Projectile hits targets | Targets move |
| 75 | freefall_race | Different masses fall | Same rate |
| 76 | momentum_transfer | Moving hits stationary | Exchange |

### Precision Tests (ID: 77-80)

| ID | Name | Description | Expected |
|----|------|-------------|----------|
| 77 | funnel | Ball through funnel | Converges |
| 78 | blocker | Deflection test | Redirects |
| 79 | precision_drop | Narrow gap landing | Hits target |
| 80 | compaction | Crusher on balls | Compression |

### Extended Tests (ID: 81-100)

| ID | Name | Description |
|----|------|-------------|
| 81 | double_pyramid | Two pyramids |
| 82 | tower_wall | Tower near wall |
| 83 | stair_walk | Staircase descent |
| 84 | ball_ball | Ball-ball collision |
| 85 | heavy_heavy | Heavy-heavy stack |
| 86 | light_light | Light-light stack |
| 87 | mixed_stack | Mixed materials |
| 88 | domino_circle | Circular dominoes |
| 89 | water_channel | Water channeling |
| 90 | ball_ramp | Ball down ramp |
| 91 | high_stack | Very tall stack |
| 92 | platform_drop | Drop onto platform |
| 93 | multi_ball | Multiple balls |
| 94 | shatter_test | Break threshold |
| 95 | bounce_test | Bounce behavior |
| 96 | flow_test | Liquid flow |
| 97 | stack_stability | Stability test |
| 98 | domino_effect | Cascade test |
| 99 | structural_integrity | Load test |
| 100 | freeform_physics | General physics |

---

## Running Tests

```bash
# Run all physics tests
zig test src/physics_tests.zig

# Run specific test
zig test src/physics_tests.zig --test-name "Run physics test 1"

# Run via main
./zig-out/bin/worldvm run --scenario drop_high --ticks 100
```

## Expected Results

| Test Type | Expected Stable | Reason |
|----------|----------------|--------|
| Drop tests | Yes | Gravity settles |
| Stack tests | Yes/No | Depends on stability |
| Domino tests | No initially | Cascade takes time |
| Liquid tests | No | Continues flowing |
| Break tests | Yes | Breaks or settles |

## Test Result Structure

```zig
pub const PhysicsTestResult = struct {
    test_id: u32,
    name: []const u8,
    ticks_to_stable: u32,
    stable: bool,
    final_states: []InstanceFinalState,
    expected_stable: bool,
    passed: bool,
};
```
