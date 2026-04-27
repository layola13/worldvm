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

---

## Cross-Domain Scenario Extensions (Planned)

The scenarios below extend pure physics tests with chemical, sound, personal-universe, and temporal-causal logic checks.
They are **planned cross-domain cases** for upcoming test suites and are not part of the current `runPhysicsTest(allocator, test_id)` 1-100 implementation.

### Chemical Cases (ID: 101-110)

| ID | Name | Input | Key Assertions | Expected |
|----|------|-------|----------------|----------|
| 101 | chem_smell_fruit | Human near fruit with `chemical_signature` | smell profile detected, `affect_delta` produced | PASS |
| 102 | chem_taste_sugar | Human ingests sugar object | taste profile hit, `physio_delta` updated | PASS |
| 103 | chem_rotten_avoid | Human detects rotten odor | aversion bias increases, approach action gated | PASS |
| 104 | chem_unknown_safe_guard | Unknown chemical ingestion | `run_chemical_check` returns risk | FAIL or CLARIFY |
| 105 | chem_dual_source | Two simultaneous odor sources | strongest source wins or weighted merge | PASS |
| 106 | chem_memory_anchor | Known smell linked to memory | memory anchor trigger logged | PASS |
| 107 | chem_overdose_guard | High concentration toxic input | safety block with reason code | `C_REACTION_UNSAFE` |
| 108 | chem_ingest_then_move | Ingest + locomotion | no crash, state remains deterministic | PASS |
| 109 | chem_replay_determinism | Same chemical event replayed twice | same `reason_code` and deltas | PASS |
| 110 | chem_headless_budget | `sim-check --domain chem` | p95 within configured budget | PASS |

### Sound Cases (ID: 111-120)

| ID | Name | Input | Key Assertions | Expected |
|----|------|-------|----------------|----------|
| 111 | sound_music_sad | Sad music event | `valence` drops, trace recorded | PASS |
| 112 | sound_alarm_high_arousal | Alarm event | arousal rises, risk gate tightened | PASS |
| 113 | sound_speech_command | Speech command event | parsed intent + sound features both present | PASS |
| 114 | sound_noise_filter | Background noise + command | command survives noise floor | PASS |
| 115 | sound_loop_adaptation | Looping same track | adaptation curve appears | PASS |
| 116 | sound_memory_trigger | Familiar voice | memory anchor hit count increases | PASS |
| 117 | sound_multi_source | Competing sound sources | deterministic source merge | PASS |
| 118 | sound_to_behavior | Sound event changes action priority | action whitelist updated | PASS |
| 119 | sound_replay_determinism | Replay identical sound input | identical deltas and codes | PASS |
| 120 | sound_headless_budget | Headless auditory check | within latency budget | PASS |

### Personal Universe Cases (ID: 121-130)

| ID | Name | Input | Key Assertions | Expected |
|----|------|-------|----------------|----------|
| 121 | pu_basic_event_loop | Visual/social event to human | human root page activates and commits | PASS |
| 122 | pu_affect_gate | High-threat input | gate restricts risky actions | PASS |
| 123 | pu_shadow_branch | One event, multiple responses | branch scoring generated | PASS |
| 124 | pu_sound_chem_fusion | Concurrent sound+chemical events | fused deltas reflected in gate | PASS |
| 125 | pu_recovery_curve | Strong negative event then idle | recovery trend appears over ticks | PASS |
| 126 | pu_low_freq_idle | No events for long period | low-frequency heartbeat only | PASS |
| 127 | pu_bus_roundtrip | Physics -> Personal -> scheduler | no missing bus message | PASS |
| 128 | pu_stability_24h | Long running simulation | no leak, no state corruption | PASS |
| 129 | pu_replay_determinism | Same seed + same inputs | same outputs and traces | PASS |
| 130 | pu_headless_profile | profile in headless mode | no render dependency | PASS |

### Temporal-Causal Logic Cases (ID: 131-140)

| ID | Name | Input | Key Assertions | Expected |
|----|------|-------|----------------|----------|
| 131 | temporal_parent_marriage_son | marriage at `t1`, son born at `t2>t1` | invite precondition fails correctly | `T_EXISTENCE_VIOLATION` |
| 132 | temporal_before_after_conflict | contradictory before/after edges | conflict detected | `T_ORDER_CONFLICT` |
| 133 | temporal_cause_reverse | result precedes cause | causal rule triggers | `T_CAUSAL_CONFLICT` |
| 134 | temporal_role_inactive | action requires inactive role | role window rejection | `T_ROLE_INACTIVE` |
| 135 | temporal_valid_timeline | consistent birth-school-work order | no temporal errors | PASS |
| 136 | temporal_missing_time_clarify | insufficient timestamp evidence | clarify required, no unsafe execute | FAIL or CLARIFY |
| 137 | temporal_cross_module_guard | temporal fail + physics intent | execution blocked before physics step | FAIL |
| 138 | temporal_hook_protocol | hook response check | includes `reason_code` and `break_time` | PASS |
| 139 | temporal_replay_determinism | same temporal input replay | identical reason code and break_time | PASS |
| 140 | temporal_batch_cases | mixed valid/invalid timeline batch | stable classification accuracy | PASS |

## Extended Result Structure Suggestion

```zig
pub const CrossDomainResult = struct {
    test_id: u32,
    domain: enum { physics, chemical, sound, personal, temporal },
    passed: bool,
    reason_code: []const u8, // E_OK / C_* / T_* ...
    break_time: i64,         // temporal conflict point, -1 if N/A
    trace_digest: []const u8,
};
```

## Implementation Note

- Current implemented suite: `src/physics_tests.zig` (`test_id` 1-100)
- Planned extension suite: cross-domain cases (`test_id` 101-140)
- Recommended split:
  - `src/physics_tests.zig` for rigid-body baseline
  - `src/cross_domain_tests.zig` for chemical/sound/personal/temporal scenarios

## Implementation Roadmap (Recommended)

1. Add a new entry API:
```zig
pub fn runCrossDomainTest(
    allocator: std.mem.Allocator,
    test_id: u32,
) !CrossDomainResult
```
2. Keep ID partition stable:
   - `101-110`: chemical
   - `111-120`: sound
   - `121-130`: personal
   - `131-140`: temporal
3. Reuse existing reason code definitions (`E_*`, `C_*`, `T_*`) from appendix docs.
4. Ensure deterministic replay for all temporal and personal cases.
5. Add CLI path aligned with current docs:
```bash
./zig-out/bin/worldvm sim-check --domain physics --case 35
./zig-out/bin/worldvm sim-check --domain chem --case 110
./zig-out/bin/worldvm sim-check --domain temporal --case 131
```

## Acceptance Gates for Cross-Domain Cases

To avoid "case name only" tests, each cross-domain case should define explicit pass gates:

1. Functional gate:
   - Expected `passed == true` for PASS cases
   - Expected `reason_code` match for guarded FAIL cases (for example `T_*`, `C_*`)
2. Determinism gate:
   - Same seed + same input -> same `reason_code`, `trace_digest`, and `break_time`
3. Performance gate:
   - `sim-check` p95 within configured budget (`<= 1000ms` on CPU-only target)
4. Safety gate:
   - Unsafe scenarios must not proceed silently; they must return structured failure

### Suggested CI Layout

```bash
# baseline physics
zig test src/physics_tests.zig

# cross-domain extension (when implemented)
zig test src/cross_domain_tests.zig

# sampled headless checks
./zig-out/bin/worldvm sim-check --domain chem --case 110
./zig-out/bin/worldvm sim-check --domain temporal --case 131
```
