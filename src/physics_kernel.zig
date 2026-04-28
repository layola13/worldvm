//! Shared physics kernel helpers for world coordinators.

const std = @import("std");
const address = @import("address.zig");
const scene1024 = @import("scene1024.zig");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");
const bus = @import("bus.zig");
const physics = @import("physics.zig");
const terrain = @import("terrain.zig");
const material_pairing = @import("material_pairing.zig");
const contact_response = @import("contact_response.zig");
const collision_event = @import("collision_event.zig");
const collision = @import("collision.zig");
const query_types = @import("query_types.zig");
const query_penetration = @import("query_penetration.zig");
const prediction = @import("prediction.zig");
const rewind = @import("rewind.zig");
const sleep_response = @import("sleep_response.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const ragdoll = @import("ragdoll.zig");
const ai_traffic = @import("ai_traffic.zig");
const ballistics = @import("ballistics.zig");
const joint = @import("joint.zig");
const destruction = @import("destruction.zig");

pub const MAX_BROADPHASE_PAIRS: usize = scene32.MAX_INSTANCES * (scene32.MAX_INSTANCES - 1) / 2;

pub const BroadPhasePair = struct {
    a: u8,
    b: u8,
};

pub const SleepIsland = sleep_response.SleepIsland;

fn sameBroadPhasePair(a: BroadPhasePair, b: BroadPhasePair) bool {
    return a.a == b.a and a.b == b.b;
}

pub const BroadPhaseStats = struct {
    candidate_count: usize = 0,
    aabb_builds: usize = 0,
    pair_tests: usize = 0,
    pair_count: usize = 0,
};

const BroadPhaseCandidate = struct {
    index: u8,
    is_static: bool,
    swept_aabb: physics.VoxelBox,
};

pub const ConstraintStageResult = struct {
    pair_count: usize,
    changed: bool,
};

pub const ConstraintRowDebugSnapshot = struct {
    kind: ConstraintRowKind,
    index: usize,
    priority: f32,
    base_residual: f32,
    metadata: ConstraintRowMetadata,
    equation: ConstraintRowEquation,
};

const ConstraintSubsystem = enum(u8) {
    joint,
    contact,
    environment,
};

const ConstraintRowKind = enum(u8) {
    joint_anchor,
    joint_limit,
    joint_drive,
    contact_normal,
    contact_friction,
    environment,
};

const ConstraintRowMetadata = struct {
    predictive_residual_hint: f32 = 0.0,
    speculative_contact: bool = false,
    speculative_bias_scale: f32 = 1.0,
    speculative_bias_tier: SpeculativeContactBiasTier = .none,
    speculative_predictive_excess: f32 = 0.0,
    preconditioner_effective_mass: f32 = 0.0,
    preconditioner_priority_scale: f32 = 1.0,
    preconditioner_active: bool = false,
};

const SpeculativeContactBiasTier = enum(u8) {
    none = 0,
    base = 1,
    mid = 2,
    high = 3,
};

const ConstraintRow = struct {
    kind: ConstraintRowKind,
    index: usize,
    priority: f32,
    base_residual: f32,
    metadata: ConstraintRowMetadata = .{},
    equation: ConstraintRowEquation,
};

const ConstraintRowState = struct {
    kind: ConstraintRowKind,
    index: usize,
    retained_residual: f32,
    accumulated_impulse: f32,
    last_impulse_delta: f32,
    touched_iteration: u8,
};

const ConstraintRowExecResult = struct {
    changed: bool,
    residual_before: f32,
    residual_after: f32,
    applied_impulse: f32,

    fn improved(self: ConstraintRowExecResult) bool {
        return self.residual_after + 0.0001 < self.residual_before;
    }
};

const IslandRowBuildContext = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    joint_indices: []const usize,
    pair_subset: []const BroadPhasePair,
    instance_indices: []const u8,
    row_states: []ConstraintRowState,
    state_count: usize,
    out_rows: []ConstraintRow,
    count: *usize,
};

const IslandRowDispatchEntry = struct {
    subsystem: ConstraintSubsystem,
};

const IslandRowDispatchOrderEntry = struct {
    entry: *const IslandRowDispatchEntry,
    stress: f32,
};

const JointRowSpecBuildContext = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    joint_idx: usize,
};

const ContactRowSpecBuildContext = struct {
    prepared: ContactPreparedPair,
    pair_idx: usize,
};

const EnvironmentRowSpecBuildContext = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    local_idx: usize,
};

const RowSpecBuildContext = union(enum) {
    joint: JointRowSpecBuildContext,
    contact: ContactRowSpecBuildContext,
    environment: EnvironmentRowSpecBuildContext,
};

const JointRowEnabledFn = *const fn (joint_def: *const joint.Joint) bool;
const ContactDirectionalPayloadFn = *const fn (prepared: *const ContactPreparedPair) DirectionalRowPayload;
const JointRowPlanBuilderFn = *const fn (
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) DirectionalRowPlan;
const EnvironmentRowPlanBuilderFn = *const fn (
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) DirectionalRowPlan;

const ContactDirectionalRowSpecEntry = struct {
    kind: ConstraintRowKind,
    mode: DirectionalRowSpecMode,
    enabled: bool = true,
    payload_selector: ContactDirectionalPayloadFn,
};

const EnvironmentPlanRowSpecEntry = struct {
    kind: ConstraintRowKind,
    enabled: bool = true,
    plan_builder: EnvironmentRowPlanBuilderFn,
};

const JointPlanRowSpecEntry = struct {
    kind: ConstraintRowKind,
    enabled: bool = true,
    row_enabled: JointRowEnabledFn,
    plan_builder: JointRowPlanBuilderFn,
};

const JointSolveRuntimeContext = struct {
    kind: ConstraintRowKind,
    descriptor: *const JointRuntimeRowDescriptor,
    joint_def: *joint.Joint,
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    before: f32,
    mass_data: JointMassData,
};

const JointRowSolverFn = *const fn (
    ctx: JointSolveRuntimeContext,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult;

const JointPrepareChannelFn = *const fn (
    ctx: JointSolveRuntimeContext,
) JointPreparedOutcome(JointPreparedChannel);

const JointEquationBuilderFn = *const fn (
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation;

const RowSpecBuilderEntry = union(enum) {
    contact_directional: ContactDirectionalRowSpecEntry,
    environment_plan: EnvironmentPlanRowSpecEntry,
    joint_plan: JointPlanRowSpecEntry,
};

fn jointRowSpecContext(ctx: *const RowSpecBuildContext) ?*const JointRowSpecBuildContext {
    return switch (ctx.*) {
        .joint => |*joint_ctx| joint_ctx,
        else => null,
    };
}

fn contactRowSpecContext(ctx: *const RowSpecBuildContext) ?*const ContactRowSpecBuildContext {
    return switch (ctx.*) {
        .contact => |*contact_ctx| contact_ctx,
        else => null,
    };
}

fn environmentRowSpecContext(ctx: *const RowSpecBuildContext) ?*const EnvironmentRowSpecBuildContext {
    return switch (ctx.*) {
        .environment => |*environment_ctx| environment_ctx,
        else => null,
    };
}

const ConstraintRowEquation = struct {
    effective_mass: f32,
    bias: f32,
    max_impulse: f32,
};

const ConstraintPreconditionerPolicy = struct {
    reference_effective_mass: f32 = 1.0,
    min_effective_mass: f32 = 0.05,
    max_effective_mass: f32 = 20.0,
    min_priority_scale: f32 = 0.75,
    max_priority_scale: f32 = 1.25,
};

const ConstraintPreconditionerModel = struct {
    effective_mass: f32,
    priority_scale: f32,
    active: bool,
};

const PairImpulsePolicy = struct {
    min_inverse_mass: f32 = 0.0001,
    max_impulse: ?f32 = null,
};

const PairVelocityImpulseSpec = struct {
    speed: f32,
    inv_mass_a: f32,
    inv_mass_b: f32,
    multiplier: f32 = 1.0,
    policy: PairImpulsePolicy = .{},
};

const DirectionalRowSpecMode = enum {
    normal,
    tangent,
};

const ConstraintBodyMode = enum {
    single,
    pair,
};

const ConstraintApplyChannel = enum {
    linear_displacement,
    linear_velocity,
    angular_position,
    angular_velocity,
    linear_directional,
};

const ConstraintPairBodyData = struct {
    inv_mass_a: f32,
    inv_mass_b: f32,
    ratio_a: f32,
    ratio_b: f32,

    fn fromInverseMasses(inv_mass_a: f32, inv_mass_b: f32) ?ConstraintPairBodyData {
        const total_inv_mass = inv_mass_a + inv_mass_b;
        if (total_inv_mass <= 0.0001) return null;
        return .{
            .inv_mass_a = inv_mass_a,
            .inv_mass_b = inv_mass_b,
            .ratio_a = inv_mass_a / total_inv_mass,
            .ratio_b = inv_mass_b / total_inv_mass,
        };
    }

    fn fromJointMassData(mass_data: JointMassData) ?ConstraintPairBodyData {
        if (mass_data.inv_mass_a + mass_data.inv_mass_b <= 0.0001) return null;
        return .{
            .inv_mass_a = mass_data.inv_mass_a,
            .inv_mass_b = mass_data.inv_mass_b,
            .ratio_a = mass_data.ratio_a,
            .ratio_b = mass_data.ratio_b,
        };
    }
};

const ConstraintApplyPrimitive = struct {
    body_mode: ConstraintBodyMode,
    channel: ConstraintApplyChannel,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    warm_impulse: f32 = 0.0,
    magnitude_slop: f32 = 0.0,
    clamp_non_negative: bool = false,
    axis_x: f32 = 0.0,
    axis_y: f32 = 0.0,
    axis_z: f32 = 0.0,
    pair_bodies: ?ConstraintPairBodyData = null,
};

fn makePairPrimitiveFromJointMassData(
    mass_data: JointMassData,
    channel: ConstraintApplyChannel,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
) ConstraintApplyPrimitive {
    return .{
        .body_mode = .pair,
        .channel = channel,
        .equation = equation,
        .bias_scale = bias_scale,
        .axis_x = axis_x,
        .axis_y = axis_y,
        .axis_z = axis_z,
        .pair_bodies = ConstraintPairBodyData.fromJointMassData(mass_data),
    };
}

fn makePairPrimitiveFromInverseMasses(
    inv_mass_a: f32,
    inv_mass_b: f32,
    channel: ConstraintApplyChannel,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
) ConstraintApplyPrimitive {
    return .{
        .body_mode = .pair,
        .channel = channel,
        .equation = equation,
        .bias_scale = bias_scale,
        .axis_x = axis_x,
        .axis_y = axis_y,
        .axis_z = axis_z,
        .pair_bodies = ConstraintPairBodyData.fromInverseMasses(inv_mass_a, inv_mass_b),
    };
}

const JointMassData = struct {
    inv_mass_a: f32,
    inv_mass_b: f32,
    ratio_a: f32,
    ratio_b: f32,
};

const JointAxis = enum { x, y, z };

const JointAxisVector = struct {
    x: f32,
    y: f32,
    z: f32,
};

const JointAxisProjection = struct {
    along: f32,
    perp_x: f32,
    perp_y: f32,
    perp_z: f32,
    perp_len: f32,
};

const PreparedJointLinearConstraint = struct {
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
};

const PreparedJointAngularConstraint = struct {
    angle_a: f32,
    angle_b: f32,
    magnitude: f32,
};

const PreparedJointAngularDriveConstraint = struct {
    angle_a: f32,
    angle_b: f32,
    signed_step: f32,
    desired_velocity: f32,
};

const JointAngularConstraintSpec = struct {
    axis: JointAxis,
    angle_a: f32,
    angle_b: f32,
    correction: f32,
};

const JointAngularVelocityConstraintSpec = struct {
    axis: JointAxis,
    signed_bias: f32,
};

const PreparedJointDriveConstraint = struct {
    signed_step: f32,
    desired_velocity: f32,
};

const PreparedJointPulleyConstraint = struct {
    axis: JointAxisVector,
    ratio: f32,
    pulley_error: f32,
    constraint_speed: f32,
    damping_scale: f32,
};

const JointPreparedLimitChannel = union(enum) {
    linear: struct {
        axis: JointAxisVector,
        constraint: PreparedJointLinearConstraint,
    },
    angular: struct {
        axis: JointAxis,
        constraint: PreparedJointAngularConstraint,
        postprocess: JointLimitPostprocess = .none,
    },
    pulley: PreparedJointPulleyConstraint,
};

const JointPreparedAnchorChannel = struct {
    axis: JointAxisVector,
    constraint: PreparedJointLinearConstraint,
    damping_scale: f32,
    postprocess: JointAnchorPostprocess = .none,
};

const JointAnchorPostprocess = union(enum) {
    none,
    hinge_limit_damping,
    slider_limit_damping: JointAxisVector,
};

const JointLimitPostprocess = union(enum) {
    none,
    angular_velocity_damping: JointAxis,
};

const JointPreparedDrivePostprocess = enum {
    none,
    velocity_bias,
};

const JointPreparedDriveChannel = union(enum) {
    linear: struct {
        axis: JointAxisVector,
        constraint: PreparedJointDriveConstraint,
        postprocess: JointPreparedDrivePostprocess = .none,
    },
    angular: struct {
        axis: JointAxis,
        constraint: PreparedJointAngularDriveConstraint,
        postprocess: JointPreparedDrivePostprocess = .none,
    },
};

const JointPreparedChannel = union(enum) {
    limit: JointPreparedLimitChannel,
    anchor: JointPreparedAnchorChannel,
    drive: JointPreparedDriveChannel,
};

const JointPreparedPostprocess = union(enum) {
    none,
    anchor: JointAnchorPostprocess,
    limit: JointLimitPostprocess,
    drive_velocity_bias: JointPreparedDriveChannel,
};

const JointPreparedSolveResult = struct {
    solve_step: ConstraintSolveStep,
    impulse_hint: f32 = 0.0,
    postprocess: JointPreparedPostprocess = .none,

    fn applyTo(result: JointPreparedSolveResult, state: *JointRowExecutionState) void {
        state.applyImpulseHint(result.impulse_hint);
        state.applyStep(result.solve_step);
    }

    fn applyPostprocess(
        result: JointPreparedSolveResult,
        inst_a: *scene32.Instance,
        inst_b: *scene32.Instance,
        joint_def: *const joint.Joint,
        mass_data: JointMassData,
    ) bool {
        return switch (result.postprocess) {
            .none => false,
            .anchor => |postprocess| applyPreparedJointAnchorPostprocess(
                inst_a,
                inst_b,
                joint_def,
                mass_data,
                postprocess,
            ),
            .limit => |postprocess| applyPreparedJointLimitPostprocess(
                inst_a,
                inst_b,
                mass_data,
                postprocess,
            ),
            .drive_velocity_bias => |drive| applyPreparedJointDriveVelocityBias(
                inst_a,
                inst_b,
                joint_def,
                mass_data,
                drive,
            ),
        };
    }

    fn applyAll(
        result: JointPreparedSolveResult,
        state: *JointRowExecutionState,
        inst_a: *scene32.Instance,
        inst_b: *scene32.Instance,
        joint_def: *const joint.Joint,
        mass_data: JointMassData,
    ) void {
        result.applyTo(state);
        state.applyChanged(result.applyPostprocess(
            inst_a,
            inst_b,
            joint_def,
            mass_data,
        ));
    }
};

const JointPreparedUnavailablePolicy = enum {
    finalize_no_change,
    finalize_exec_state,
    stalled,
};

const JointAnchorPostprocessPolicy = enum {
    none,
    by_joint_type,
};

const JointDrivePostprocessPolicy = enum {
    none,
    velocity_bias,
};

const JointSignedCorrectionPolicy = struct {
    retention_scale: f32 = 0.0,
    bias_scale: f32 = 1.0,
    min_abs_correction: f32 = 0.0,
    planned: bool = false,
};

const JointImpulseHintPolicy = struct {
    enabled: bool = false,
    scale: f32 = 1.0,
};

const JointRuntimeRowPolicy = struct {
    residual_scale: f32,
    warm_impulse_scale: f32,
    inactive_outcome: JointPreparedUnavailablePolicy,
    stalled_prepare_outcome: JointPreparedUnavailablePolicy,
    signed_correction_policy: JointSignedCorrectionPolicy = .{},
    impulse_hint_policy: JointImpulseHintPolicy = .{},
    anchor_postprocess_policy: JointAnchorPostprocessPolicy = .none,
    drive_postprocess_policy: JointDrivePostprocessPolicy = .none,
};

fn JointPreparedOutcome(comptime T: type) type {
    return union(enum) {
        inactive,
        stalled_prepare,
        ready: T,
    };
}

fn JointPreparedResolution(comptime T: type) type {
    return union(enum) {
        ready: T,
        outcome: JointSolveResultOutcome,
    };
}

fn MapJointPreparedOutcomeFn(comptime In: type, comptime Out: type) type {
    return *const fn (
        ctx: JointSolveRuntimeContext,
        prepared: In,
    ) Out;
}

fn PrepareResolvedJointSolveResultFn(comptime T: type) type {
    return *const fn (
        ctx: JointSolveRuntimeContext,
        exec_state: *const JointRowExecutionState,
        row_state: ?*const ConstraintRowState,
        equation: ConstraintRowEquation,
        prepared: T,
    ) JointPreparedSolveResult;
}

const JointSolveResultOutcome = union(enum) {
    ready: JointPreparedSolveResult,
    no_change,
    finalize_exec_state,
    stalled,
};

const JointRuntimeRowDescriptor = struct {
    kind: ConstraintRowKind,
    policy: JointRuntimeRowPolicy,
    row_enabled: JointRowEnabledFn,
    prepare_channel: JointPrepareChannelFn,
};

const JointRuntimeSolveContext = struct {
    runtime: JointSolveRuntimeContext,
};

const JointRuntimeSolveContextOutcome = union(enum) {
    inactive,
    stalled,
    ready: JointRuntimeSolveContext,
};

const JointRowExecutionState = struct {
    changed: bool = false,
    applied_impulse: f32 = 0.0,

    fn init(initial_impulse: f32) JointRowExecutionState {
        return .{
            .changed = false,
            .applied_impulse = initial_impulse,
        };
    }

    fn applyStep(state: *JointRowExecutionState, step: ConstraintSolveStep) void {
        state.changed = step.changed or state.changed;
        state.applied_impulse = @max(state.applied_impulse, step.applied_impulse);
    }

    fn solveStep(state: JointRowExecutionState) ConstraintSolveStep {
        return .{
            .changed = state.changed,
            .applied_impulse = state.applied_impulse,
        };
    }

    fn applyImpulseHint(state: *JointRowExecutionState, impulse_hint: f32) void {
        state.applied_impulse = @max(state.applied_impulse, @abs(impulse_hint));
    }

    fn applyChanged(state: *JointRowExecutionState, changed: bool) void {
        state.changed = changed or state.changed;
    }
};

const ConstraintSolveStep = struct {
    changed: bool,
    applied_impulse: f32,
};

fn withSolveStepAppliedImpulse(step: ConstraintSolveStep, applied_impulse: f32) ConstraintSolveStep {
    var result = step;
    result.applied_impulse = applied_impulse;
    return result;
}

const PreparedDirectionalConstraint = struct {
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    depth: f32,
    predicted_depth: f32,
    predictive_residual_hint: f32 = 0.0,
};

const DirectionalRowPayload = struct {
    direction: PreparedDirectionalConstraint,
    equation: ConstraintRowEquation,
};

const PredictiveConstraintGain = struct {
    residual_hint: f32,
    bias_delta: f32,
    impulse_delta: f32,
};

const PredictiveConstraintPolicy = struct {
    residual_scale: f32,
    bias_scale: f32,
    impulse_scale: f32,
};

const PredictiveRowPlan = struct {
    horizon_dt: f32,
    current_depth: f32,
    predicted_depth: f32,
    urgency: f32,
    allowed_correction_budget: f32,
    gain: PredictiveConstraintGain,
};

const DirectionalRowPlan = struct {
    residual: f32,
    metadata: ConstraintRowMetadata = .{},
    equation: ConstraintRowEquation,
};

const ConstraintRowBuildSpec = struct {
    kind: ConstraintRowKind,
    index: usize,
    residual: f32,
    metadata: ConstraintRowMetadata = .{},
    equation: ConstraintRowEquation,
};

const KernelJointDriveState = struct {
    position: f32,
    relative_velocity: f32,
};

const KernelJointDrivePlan = struct {
    signed_step: f32,
    desired_velocity: f32,
};

fn buildPreparedDirectionalConstraint(
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    depth: f32,
    predicted_depth: f32,
) PreparedDirectionalConstraint {
    return .{
        .dir_x = dir_x,
        .dir_y = dir_y,
        .dir_z = dir_z,
        .depth = depth,
        .predicted_depth = predicted_depth,
    };
}

fn withPredictiveResidualHint(
    direction: PreparedDirectionalConstraint,
    predictive_residual_hint: f32,
) PreparedDirectionalConstraint {
    var resolved = direction;
    resolved.predictive_residual_hint = predictive_residual_hint;
    return resolved;
}

fn makeConstraintRowMetadata(predictive_residual_hint: f32) ConstraintRowMetadata {
    return .{
        .predictive_residual_hint = predictive_residual_hint,
    };
}

fn withSpeculativeContactMetadata(
    metadata: ConstraintRowMetadata,
    speculative_contact: bool,
) ConstraintRowMetadata {
    var resolved = metadata;
    resolved.speculative_contact = speculative_contact;
    return resolved;
}

fn withSpeculativeContactBiasScale(
    metadata: ConstraintRowMetadata,
    speculative_bias_scale: f32,
) ConstraintRowMetadata {
    var resolved = metadata;
    resolved.speculative_bias_scale = speculative_bias_scale;
    return resolved;
}

fn withSpeculativeContactBiasTier(
    metadata: ConstraintRowMetadata,
    speculative_bias_tier: SpeculativeContactBiasTier,
) ConstraintRowMetadata {
    var resolved = metadata;
    resolved.speculative_bias_tier = speculative_bias_tier;
    return resolved;
}

fn withSpeculativeContactPredictiveExcess(
    metadata: ConstraintRowMetadata,
    speculative_predictive_excess: f32,
) ConstraintRowMetadata {
    var resolved = metadata;
    resolved.speculative_predictive_excess = speculative_predictive_excess;
    return resolved;
}

fn withSpeculativeContactDirectionalMetadata(
    metadata: ConstraintRowMetadata,
    speculative_contact: bool,
    direction: PreparedDirectionalConstraint,
) ConstraintRowMetadata {
    var resolved = withSpeculativeContactMetadata(metadata, speculative_contact);
    if (!speculative_contact) return resolved;
    const profile = measureSpeculativeContactBiasProfileDefault(direction);
    const predictive_excess = profile.predictive_excess;
    resolved = withSpeculativeContactBiasScale(
        resolved,
        profile.scale,
    );
    resolved = withSpeculativeContactBiasTier(resolved, profile.tier);
    resolved = withSpeculativeContactPredictiveExcess(resolved, predictive_excess);
    return resolved;
}

fn measureSpeculativeContactPriorityFloor(base_priority: f32, metadata: ConstraintRowMetadata) f32 {
    if (!metadata.speculative_contact) return base_priority;
    const policy = speculativeContactPriorityFloorPolicy();
    const tier_scale = switch (metadata.speculative_bias_tier) {
        .none => policy.legacy_fallback_scale,
        .base => policy.base_scale,
        .mid => policy.mid_scale,
        .high => policy.high_scale,
    };
    return @max(base_priority, base_priority * tier_scale);
}

const SpeculativeContactPriorityFloorPolicy = struct {
    legacy_fallback_scale: f32,
    base_scale: f32,
    mid_scale: f32,
    high_scale: f32,
};

fn speculativeContactPriorityFloorPolicy() SpeculativeContactPriorityFloorPolicy {
    return .{
        .legacy_fallback_scale = 1.05,
        .base_scale = 1.05,
        .mid_scale = 1.075,
        .high_scale = 1.1,
    };
}

fn constraintPreconditionerPolicy() ConstraintPreconditionerPolicy {
    return .{};
}

fn neutralConstraintPreconditionerModel() ConstraintPreconditionerModel {
    return .{
        .effective_mass = 0.0,
        .priority_scale = 1.0,
        .active = false,
    };
}

fn computeConstraintPreconditionerModel(
    equation: ConstraintRowEquation,
    policy: ConstraintPreconditionerPolicy,
) ConstraintPreconditionerModel {
    if (equation.effective_mass <= 0.0) return neutralConstraintPreconditionerModel();
    const mass = @max(policy.min_effective_mass, @min(policy.max_effective_mass, equation.effective_mass));
    const normalized = mass / @max(policy.min_effective_mass, policy.reference_effective_mass);
    const raw_scale = @sqrt(normalized);
    const priority_scale = @max(policy.min_priority_scale, @min(policy.max_priority_scale, raw_scale));
    return .{
        .effective_mass = mass,
        .priority_scale = priority_scale,
        .active = priority_scale != 1.0,
    };
}

fn withConstraintPreconditionerMetadata(
    metadata: ConstraintRowMetadata,
    preconditioner: ConstraintPreconditionerModel,
) ConstraintRowMetadata {
    var resolved = metadata;
    resolved.preconditioner_effective_mass = preconditioner.effective_mass;
    resolved.preconditioner_priority_scale = preconditioner.priority_scale;
    resolved.preconditioner_active = preconditioner.active;
    return resolved;
}

fn preconditionConstraintRowPriority(priority: f32, preconditioner: ConstraintPreconditionerModel) f32 {
    if (priority <= 0.0) return priority;
    return priority * preconditioner.priority_scale;
}

fn idleKernelJointDrivePlan() KernelJointDrivePlan {
    return .{
        .signed_step = 0.0,
        .desired_velocity = 0.0,
    };
}

fn kernelPredictionDt() f32 {
    return 1.0 / 60.0;
}

fn predictKernelInstanceState(inst: *const scene32.Instance, dt: f32) prediction.LinearState {
    return prediction.predictLinearState(prediction.linearStateFromInstance(inst), dt);
}

const DirectionalConstraintChannel = enum {
    normal,
    tangent,
    stress,
};

const DirectionalConstraintMetrics = struct {
    penetration_depth: f32,
    normal_speed: f32,
    tangential_speed: f32,

    fn residual(self: DirectionalConstraintMetrics, channel: DirectionalConstraintChannel) f32 {
        return switch (channel) {
            .normal => @max(self.penetration_depth, self.normal_speed * 0.25),
            .tangent => self.tangential_speed * 0.125,
            .stress => @max(
                self.penetration_depth,
                @max(self.normal_speed * 0.25, self.tangential_speed * 0.125),
            ),
        };
    }

    fn stress(self: DirectionalConstraintMetrics) f32 {
        return self.residual(.stress);
    }
};

const ContactConstraintMetrics = DirectionalConstraintMetrics;

const DirectionalVelocityComponents = struct {
    normal_speed_signed: f32,
    tangent_x: f32,
    tangent_y: f32,
    tangent_z: f32,

    fn normalSpeed(self: DirectionalVelocityComponents) f32 {
        return @abs(self.normal_speed_signed);
    }

    fn tangentialSpeed(self: DirectionalVelocityComponents) f32 {
        return @sqrt(self.tangent_x * self.tangent_x + self.tangent_y * self.tangent_y + self.tangent_z * self.tangent_z);
    }
};

const DirectionalTangentFrame = struct {
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    speed: f32,
};

const ContactConstraintProbe = struct {
    entity_a: *const entity16.Entity16,
    entity_b: *const entity16.Entity16,
    aabb_a: physics.AABB,
    aabb_b: physics.AABB,
    manifold: collision.ContactManifold,
    velocity_components: DirectionalVelocityComponents,
};

pub const ContactNarrowPhaseStats = struct {
    probe_calls: usize = 0,
    aabb_builds: usize = 0,
    aabb_tests: usize = 0,
    narrowphase_calls: usize = 0,
};

pub const ContactDetectedManifold = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    aabb_a: physics.AABB,
    aabb_b: physics.AABB,
    manifold: collision.ContactManifold,
    centroid: collision.ContactPoint,
};

pub const ContactMaterialClassification = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    material_a: entity16.MaterialType,
    material_b: entity16.MaterialType,
    response: material_pairing.ContactResponse,
};

pub const ContactSurfaceClassification = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    material_a: entity16.MaterialType,
    material_b: entity16.MaterialType,
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
    hard_surface_a: bool,
    hard_surface_b: bool,
};

pub const ContactMediumClassification = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    material_a: entity16.MaterialType,
    material_b: entity16.MaterialType,
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
    medium_a: terrain.MediumType,
    medium_b: terrain.MediumType,
};

pub const ContactBodyTypeClassification = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    material_a: entity16.MaterialType,
    material_b: entity16.MaterialType,
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
    medium_a: terrain.MediumType,
    medium_b: terrain.MediumType,
    body_type_a: query_types.BodyType,
    body_type_b: query_types.BodyType,
};

pub const ContactConditionClassification = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    material_a: entity16.MaterialType,
    material_b: entity16.MaterialType,
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
    medium_a: terrain.MediumType,
    medium_b: terrain.MediumType,
    body_type_a: query_types.BodyType,
    body_type_b: query_types.BodyType,
    condition_a: query_types.SurfaceCondition,
    condition_b: query_types.SurfaceCondition,
};

pub const ContactClassification = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    material_a: entity16.MaterialType,
    material_b: entity16.MaterialType,
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
    medium_a: terrain.MediumType,
    medium_b: terrain.MediumType,
    body_type_a: query_types.BodyType,
    body_type_b: query_types.BodyType,
    condition_a: query_types.SurfaceCondition,
    condition_b: query_types.SurfaceCondition,
    response: material_pairing.ContactResponse,
};

pub const ContactClassificationConsistency = struct {
    entity_ids_valid: bool = false,
    pair_order_valid: bool = false,
    materials_match_entities: bool = false,
    surfaces_match_materials: bool = false,
    mediums_match_surfaces: bool = false,
    body_types_match_entities: bool = false,
    conditions_match_surfaces: bool = false,
    response_matches_pair: bool = false,

    pub fn isConsistent(self: ContactClassificationConsistency) bool {
        return self.entity_ids_valid and
            self.pair_order_valid and
            self.materials_match_entities and
            self.surfaces_match_materials and
            self.mediums_match_surfaces and
            self.body_types_match_entities and
            self.conditions_match_surfaces and
            self.response_matches_pair;
    }
};

pub const ContactClassificationPipelineStats = struct {
    detected_count: usize = 0,
    entity_lookup_count: usize = 0,
    classified_count: usize = 0,
    skipped_invalid_entities: usize = 0,
};

pub const ContactClassificationCacheStats = struct {
    lookup_count: usize = 0,
    hit_count: usize = 0,
    miss_count: usize = 0,
    insert_count: usize = 0,
    update_count: usize = 0,
    overflow_count: usize = 0,
};

pub const ContactClassificationCacheEntry = struct {
    pair: BroadPhasePair,
    classification: ContactClassification,
};

pub const ContactClassificationCache = struct {
    entries: []ContactClassificationCacheEntry,
    count: usize = 0,
    stats: ContactClassificationCacheStats = .{},

    pub fn init(entries: []ContactClassificationCacheEntry) ContactClassificationCache {
        return .{
            .entries = entries,
        };
    }

    pub fn reset(self: *ContactClassificationCache) void {
        self.count = 0;
        self.stats = .{};
    }

    pub fn get(self: *ContactClassificationCache, pair: BroadPhasePair) ?ContactClassification {
        self.stats.lookup_count += 1;
        if (self.findIndex(pair)) |index| {
            self.stats.hit_count += 1;
            return self.entries[index].classification;
        }
        self.stats.miss_count += 1;
        return null;
    }

    pub fn put(self: *ContactClassificationCache, classification: ContactClassification) bool {
        if (self.findIndex(classification.pair)) |index| {
            self.entries[index].classification = classification;
            self.stats.update_count += 1;
            return true;
        }
        if (self.count >= self.entries.len) {
            self.stats.overflow_count += 1;
            return false;
        }

        self.entries[self.count] = .{
            .pair = classification.pair,
            .classification = classification,
        };
        self.count += 1;
        self.stats.insert_count += 1;
        return true;
    }

    pub fn getOrBuild(
        self: *ContactClassificationCache,
        condition_classification: ContactConditionClassification,
        entities: []entity16.Entity16,
    ) ?ContactClassification {
        if (self.get(condition_classification.pair)) |classification| return classification;

        const classification = buildContactClassification(condition_classification, entities) orelse return null;
        _ = self.put(classification);
        return classification;
    }

    fn findIndex(self: *const ContactClassificationCache, pair: BroadPhasePair) ?usize {
        for (self.entries[0..self.count], 0..) |entry, index| {
            if (sameBroadPhasePair(entry.pair, pair)) return index;
        }
        return null;
    }
};

pub const MaterialPairResponse = struct {
    pair: BroadPhasePair,
    material_a: entity16.MaterialType,
    material_b: entity16.MaterialType,
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
    response: material_pairing.ContactResponse,
};

pub const Contact = struct {
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    aabb_a: physics.AABB,
    aabb_b: physics.AABB,
    manifold: collision.ContactManifold,
    centroid: collision.ContactPoint,
    classification: ContactClassification,
};

pub const ContactTelemetry = struct {
    pair: BroadPhasePair,
    friction: f32 = 0.0,
    restitution: f32 = 0.0,
    damage_modifier: f32 = 0.0,
    penetration_resistance: f32 = 0.0,
    buoyancy: f32 = 0.0,
    medium_buoyancy_force: f32 = 0.0,
    medium_buoyancy_magnitude: f32 = 0.0,
    medium_buoyancy_displaced_volume: f32 = 0.0,
    medium_buoyancy_velocity_delta: f32 = 0.0,
    medium_submerged_fraction: f32 = 0.0,
    medium_buoyancy_center_x: f32 = 0.0,
    medium_buoyancy_center_y: f32 = 0.0,
    medium_buoyancy_center_z: f32 = 0.0,
    medium_buoyancy_active: bool = false,
    medium_drag_force: f32 = 0.0,
    medium_drag_magnitude: f32 = 0.0,
    medium_drag_coefficient: f32 = 0.0,
    medium_drag_exposure: f32 = 0.0,
    medium_drag_normal_delta: f32 = 0.0,
    medium_drag_tangent_delta: f32 = 0.0,
    medium_drag_active: bool = false,
    medium_vapor_resistance_force: f32 = 0.0,
    medium_vapor_resistance_coefficient: f32 = 0.0,
    medium_vapor_resistance_exposure: f32 = 0.0,
    medium_vapor_resistance_dynamic_pressure: f32 = 0.0,
    medium_vapor_resistance_normal_delta: f32 = 0.0,
    medium_vapor_resistance_tangent_delta: f32 = 0.0,
    medium_vapor_resistance_active: bool = false,
    medium_vacuum_pressure: f32 = 1.0,
    medium_vacuum_exposure: f32 = 0.0,
    medium_vacuum_drag_loss: f32 = 0.0,
    medium_vacuum_thermal_isolation: f32 = 0.0,
    medium_vacuum_sound_attenuation: f32 = 0.0,
    medium_vacuum_active: bool = false,
    medium_transition_from: u8 = 0,
    medium_transition_to: u8 = 0,
    medium_transition_progress: f32 = 0.0,
    medium_transition_resistance: f32 = 0.0,
    medium_transition_pressure_delta: f32 = 0.0,
    medium_transition_active: bool = false,
    medium_mixing_primary: u8 = 0,
    medium_mixing_secondary: u8 = 0,
    medium_mixing_fraction: f32 = 0.0,
    medium_mixing_effective_density: f32 = 0.0,
    medium_mixing_effective_viscosity: f32 = 0.0,
    medium_mixing_blended_drag: f32 = 0.0,
    medium_mixing_blended_buoyancy: f32 = 0.0,
    medium_mixing_active: bool = false,
    medium_state_value: u8 = 0,
    medium_state_current: u8 = 0,
    medium_state_target: u8 = 0,
    medium_state_progress: f32 = 0.0,
    medium_state_stability: f32 = 1.0,
    medium_state_active: bool = false,
    medium_event_type: u8 = 0,
    medium_event_source: u8 = 0,
    medium_event_target: u8 = 0,
    medium_event_intensity: f32 = 0.0,
    medium_event_priority: u8 = 0,
    medium_event_active: bool = false,
    medium_animation_phase: f32 = 0.0,
    medium_animation_blend: f32 = 0.0,
    medium_animation_opacity: f32 = 0.0,
    medium_animation_ripple: f32 = 0.0,
    medium_animation_turbulence: f32 = 0.0,
    medium_animation_color_shift: f32 = 0.0,
    medium_animation_active: bool = false,
    medium_tow_force: f32 = 0.0,
    medium_tow_velocity_delta_x: f32 = 0.0,
    medium_tow_velocity_delta_y: f32 = 0.0,
    medium_tow_velocity_delta_z: f32 = 0.0,
    medium_tow_active: bool = false,
    medium_lift_force: f32 = 0.0,
    medium_lift_magnitude: f32 = 0.0,
    medium_lift_coefficient: f32 = 0.0,
    medium_lift_exposure: f32 = 0.0,
    medium_lift_dynamic_pressure: f32 = 0.0,
    medium_lift_velocity_delta_x: f32 = 0.0,
    medium_lift_velocity_delta_y: f32 = 0.0,
    medium_lift_velocity_delta_z: f32 = 0.0,
    medium_lift_active: bool = false,
    medium_added_mass_coefficient: f32 = 0.0,
    medium_added_mass_exposure: f32 = 0.0,
    medium_added_mass_displaced_volume: f32 = 0.0,
    medium_added_mass_value: f32 = 0.0,
    medium_added_mass_effective_mass: f32 = 0.0,
    medium_added_mass_normal_inertia_scale: f32 = 1.0,
    medium_added_mass_tangential_inertia_scale: f32 = 1.0,
    medium_added_mass_active: bool = false,
    medium_thermal_conducted_heat: f32 = 0.0,
    medium_thermal_conductivity: f32 = 0.0,
    medium_thermal_retained_heat: f32 = 0.0,
    medium_thermal_temperature_delta: f32 = 0.0,
    medium_thermal_active: bool = false,
    sink_depth: f32 = 0.0,
    sink_load: f32 = 0.0,
    sink_support_fraction: f32 = 1.0,
    sink_active: bool = false,
    sink_resistance_force: f32 = 0.0,
    sink_resistance_normal_delta: f32 = 0.0,
    sink_resistance_tangent_delta: f32 = 0.0,
    sink_resistance_active: bool = false,
    sink_recovery_depth: f32 = 0.0,
    sink_recovery_fraction: f32 = 1.0,
    sink_recovery_rate: f32 = 0.0,
    sink_recovery_active: bool = false,
    sink_recovered: bool = true,
    penetration_depth: f32 = 0.0,
    point_count: u8 = 0,
    fatigue_damage: f32 = 0.0,
    fatigue_remaining_life: f32 = 1.0,
    fatigue_failed: bool = false,
    thermal_energy: f32 = 0.0,
    thermal_friction_heat: f32 = 0.0,
    thermal_friction_heat_fraction: f32 = 0.0,
    thermal_conductivity: f32 = 0.0,
    thermal_temperature_delta: f32 = 0.0,
    sound_type: u8 = 0,
    sound_volume: f32 = 0.0,
    sound_pitch: f32 = 1.0,
    sound_duration: f32 = 0.0,
    dust_type: u8 = 0,
    dust_intensity: f32 = 0.0,
    dust_radius: f32 = 0.0,
    dust_duration: f32 = 0.0,
    deformation_total: f32 = 0.0,
    deformation_permanent: f32 = 0.0,
    deformation_recovery_fraction: f32 = 1.0,
    separation_state: ContactSeparationState = .persisting,
    separating: bool = false,
    separation_speed: f32 = 0.0,
    separation_time: f32 = 0.0,
    stabilization_bias_scale: f32 = 1.0,
    stabilization_impulse_scale: f32 = 1.0,
    stabilized_contact: bool = false,
    damage_impact: f32 = 0.0,
    damage_energy: f32 = 0.0,
    damage_amount: f32 = 0.0,
    damage_hardness_threshold: f32 = 0.0,
    damage_hardness_ratio: f32 = 0.0,
    damage_hardness_resistance: f32 = 0.0,
    damage_exceeds_hardness: bool = false,
    damage_should_break: bool = false,
    damage_fragment_count: u8 = 0,
    damage_generated_fragments: u8 = 0,
    damage_fragment_energy: f32 = 0.0,
    damage_debris_mass: f32 = 0.0,
    damage_crack_count: u8 = 0,
    damage_crack_severity: f32 = 0.0,
    damage_cracks_propagated: bool = false,
};

const ContactPreparedPair = struct {
    inv_mass_a: f32,
    inv_mass_b: f32,
    speculative: bool,
    normal: PreparedDirectionalConstraint,
    restitution: f32,
    friction: f32,
    rolling_friction: f32,
    anisotropic_friction_minor_axis_scale: f32,
    fatigue: ContactFatigueModel,
    medium_buoyancy: MediumPostBuoyancyModel,
    medium_drag: MediumPostDragModel,
    medium_vapor_resistance: MediumPostVaporResistanceModel,
    medium_vacuum: MediumPostVacuumModel,
    medium_transition: MediumPostTransitionModel,
    medium_mixing: MediumPostMixingModel,
    medium_state: MediumPostStateMachineModel,
    medium_event: MediumPostEventTriggerModel,
    medium_animation: MediumPostTransitionAnimationModel,
    medium_tow: MediumPostTowModel,
    medium_lift: MediumPostLiftModel,
    medium_added_mass: MediumPostAddedMassModel,
    medium_thermal: MediumPostThermalModel,
    sink_depth: SinkDepthModel,
    sink_resistance: SinkResistanceModel,
    sink_recovery: SinkRecoveryModel,
    thermal: ContactThermalModel,
    sound: ContactSoundModel,
    dust: ContactDustModel,
    deformation: ContactDeformationModel,
    separation: ContactSeparationModel,
    stabilization: ContactStabilizationModel,
    damage_impact: DamageEvalImpactModel,
    damage_hardness: DamageEvalHardnessModel,
    damage_break: DamageEvalBreakModel,
    damage_fragments: DamageEvalFragmentModel,
    damage_cracks: DamageEvalCrackModel,
    tangent: PreparedDirectionalConstraint,
    has_tangent: bool,
    normal_equation: ConstraintRowEquation,
    friction_equation: ConstraintRowEquation,

    fn directionalPayload(self: *const ContactPreparedPair, mode: DirectionalRowSpecMode) DirectionalRowPayload {
        return switch (mode) {
            .normal => .{
                .direction = self.normal,
                .equation = self.normal_equation,
            },
            .tangent => .{
                .direction = self.tangent,
                .equation = self.friction_equation,
            },
        };
    }
};

const ContactSolveAccumulator = struct {
    changed: bool = false,
    applied_impulse: f32 = 0.0,

    fn applyStep(accum: *ContactSolveAccumulator, step: ConstraintSolveStep) void {
        accum.changed = step.changed or accum.changed;
        accum.applied_impulse = @max(accum.applied_impulse, step.applied_impulse);
    }

    fn addWarmStart(accum: *ContactSolveAccumulator, step: ConstraintSolveStep) void {
        accum.changed = step.changed or accum.changed;
        accum.applied_impulse += step.applied_impulse;
    }

    fn finish(accum: ContactSolveAccumulator) ConstraintSolveStep {
        return .{
            .changed = accum.changed,
            .applied_impulse = accum.applied_impulse,
        };
    }
};

const ContactPairSolveResult = struct {
    normal_step: ConstraintSolveStep,
    friction_step: ConstraintSolveStep,

    fn changed(self: ContactPairSolveResult) bool {
        return self.normal_step.changed or self.friction_step.changed;
    }

    fn appliedImpulse(self: ContactPairSolveResult) f32 {
        return @max(self.normal_step.applied_impulse, self.friction_step.applied_impulse);
    }
};

const ContactSolveRequest = struct {
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState = null,
};

const ContactPositionCorrectionPolicy = struct {
    allowed_penetration_slop: f32 = 0.01,
    depth_correction_ratio: f32 = 1.0,
    bias_scale: f32 = 0.2,
};

const ContactStiffnessPolicy = struct {
    neutral_penetration_resistance: f32 = 0.5,
    response_scale: f32 = 0.5,
    min_scale: f32 = 0.75,
    max_scale: f32 = 1.25,
};

const ContactStiffnessModel = struct {
    penetration_resistance: f32,
    scale: f32,
};

const ContactDampingPolicy = struct {
    neutral_restitution: f32 = 0.5,
    response_scale: f32 = 0.5,
    min_scale: f32 = 0.75,
    max_scale: f32 = 1.25,
};

const ContactDampingModel = struct {
    restitution: f32,
    scale: f32,
};

const ContactFatiguePolicy = struct {
    neutral_damage_modifier: f32 = 1.0,
    damage_response_scale: f32 = 0.5,
    resistance_response_scale: f32 = 0.5,
    penetration_weight: f32 = 0.5,
    normal_speed_weight: f32 = 0.25,
    tangential_speed_weight: f32 = 0.1,
    base_cycle_damage: f32 = 0.01,
    min_damage_scale: f32 = 0.5,
    max_damage_scale: f32 = 2.0,
    max_accumulated_damage: f32 = 1.0,
};

const ContactFatigueInput = struct {
    penetration_depth: f32,
    relative_normal_speed: f32,
    tangential_speed: f32,
    damage_modifier: f32,
    penetration_resistance: f32,
    repeat_count: u32 = 1,
};

const ContactFatigueModel = struct {
    damage_modifier: f32,
    penetration_resistance: f32,
    stress: f32,
    single_cycle_damage: f32,
    accumulated_damage: f32,
    remaining_life_fraction: f32,
    failed: bool,
};

const ContactThermalPolicy = struct {
    normal_heat_weight: f32 = 0.05,
    friction_heat_weight: f32 = 0.1,
    rolling_friction_heat_weight: f32 = 0.05,
    anisotropic_friction_heat_scale: f32 = 0.5,
    penetration_heat_weight: f32 = 0.25,
    resistance_conductivity_scale: f32 = 0.5,
    buoyancy_cooling_scale: f32 = 0.5,
    solid_conductivity: f32 = 1.0,
    soft_conductivity: f32 = 0.6,
    liquid_conductivity: f32 = 1.4,
    vapor_conductivity: f32 = 0.2,
    plasma_conductivity: f32 = 2.0,
    min_conductivity: f32 = 0.1,
    max_conductivity: f32 = 2.5,
    max_friction_heat: f32 = 64.0,
};

const MediumPostBuoyancyPolicy = struct {
    entity_volume: f32 = 16.0 * 16.0 * 16.0,
    force_scale: f32 = 0.01,
    mass_reference: f32 = 100.0,
    full_submerged_depth: f32 = 1.0,
    center_depth_scale: f32 = 0.5,
    max_velocity_delta: f32 = 64.0,
};

const MediumPostBuoyancyInput = struct {
    buoyancy: f32,
    medium_type: terrain.MediumType,
    penetration_depth: f32,
    mass: f32,
    center_x: f32 = 0.0,
    center_y: f32 = 0.0,
    center_z: f32 = 0.0,
    normal_x: f32 = 0.0,
    normal_y: f32 = 1.0,
    normal_z: f32 = 0.0,
};

const MediumPostBuoyancyModel = struct {
    submerged_fraction: f32,
    displaced_volume: f32,
    magnitude: f32,
    center_x: f32,
    center_y: f32,
    center_z: f32,
    force: f32,
    velocity_delta_y: f32,
    active: bool,
};

const SinkDepthPolicy = struct {
    soft_depth_scale: f32 = 0.8,
    liquid_depth_scale: f32 = 0.35,
    penetration_weight: f32 = 0.65,
    normal_speed_weight: f32 = 0.08,
    tangential_speed_weight: f32 = 0.03,
    mass_weight: f32 = 0.001,
    resistance_reduction_scale: f32 = 0.8,
    buoyancy_reduction_scale: f32 = 0.5,
    max_depth: f32 = 4.0,
};

const SinkDepthInput = struct {
    medium_type: terrain.MediumType,
    penetration_depth: f32,
    relative_normal_speed: f32,
    tangential_speed: f32,
    mass: f32,
    penetration_resistance: f32,
    buoyancy: f32,
};

const SinkDepthModel = struct {
    depth: f32,
    load: f32,
    support_fraction: f32,
    active: bool,
};

const SinkResistancePolicy = struct {
    depth_force_scale: f32 = 2.0,
    load_force_scale: f32 = 0.5,
    normal_damping_scale: f32 = 0.45,
    tangential_damping_scale: f32 = 0.35,
    support_loss_force_scale: f32 = 0.25,
    max_force: f32 = 16.0,
    max_velocity_delta: f32 = 32.0,
};

const SinkResistanceInput = struct {
    sink: SinkDepthModel,
    relative_normal_speed: f32,
    tangential_speed: f32,
};

const SinkResistanceModel = struct {
    force: f32,
    normal_velocity_delta: f32,
    tangential_velocity_delta: f32,
    active: bool,
};

const SinkRecoveryPolicy = struct {
    support_recovery_scale: f32 = 0.6,
    resistance_penalty_scale: f32 = 0.04,
    normal_motion_penalty_scale: f32 = 0.05,
    tangential_motion_penalty_scale: f32 = 0.03,
    rate_scale: f32 = 2.0,
    max_rate: f32 = 8.0,
    recovered_epsilon: f32 = 0.0001,
};

const SinkRecoveryInput = struct {
    sink: SinkDepthModel,
    resistance: SinkResistanceModel,
    relative_normal_speed: f32,
    tangential_speed: f32,
};

const SinkRecoveryModel = struct {
    depth: f32,
    fraction: f32,
    rate: f32,
    active: bool,
    recovered: bool,
};

const MediumPostDragPolicy = struct {
    liquid_drag: f32 = 0.85,
    soft_drag: f32 = 0.35,
    vapor_drag: f32 = 0.08,
    plasma_drag: f32 = 0.15,
    penetration_weight: f32 = 0.5,
    buoyancy_weight: f32 = 0.5,
    tangential_scale: f32 = 0.75,
    max_velocity_delta: f32 = 64.0,
};

const MediumPostDragInput = struct {
    medium_type: terrain.MediumType,
    buoyancy: f32,
    penetration_depth: f32,
    normal_speed: f32,
    tangential_speed: f32,
};

const MediumPostDragModel = struct {
    coefficient: f32,
    exposure: f32,
    magnitude: f32,
    force: f32,
    normal_velocity_delta: f32,
    tangential_velocity_delta: f32,
    active: bool,
};

const MediumPostVaporResistancePolicy = struct {
    coefficient: f32 = 0.18,
    penetration_weight: f32 = 0.45,
    speed_weight: f32 = 0.04,
    tangential_scale: f32 = 0.6,
    pressure_scale: f32 = 0.5,
    drag_coupling: f32 = 0.35,
    max_velocity_delta: f32 = 32.0,
};

const MediumPostVaporResistanceInput = struct {
    medium_type: terrain.MediumType,
    penetration_depth: f32,
    normal_speed: f32,
    tangential_speed: f32,
    drag: MediumPostDragModel,
};

const MediumPostVaporResistanceModel = struct {
    coefficient: f32,
    exposure: f32,
    dynamic_pressure: f32,
    force: f32,
    normal_velocity_delta: f32,
    tangential_velocity_delta: f32,
    active: bool,
};

const MediumPostVacuumPolicy = struct {
    vacuum_pressure_threshold: f32 = 0.05,
    near_vacuum_pressure: f32 = 0.2,
    drag_loss_scale: f32 = 1.0,
    thermal_isolation_scale: f32 = 1.0,
    sound_attenuation_scale: f32 = 1.0,
};

const MediumPostVacuumInput = struct {
    medium_type: terrain.MediumType,
    ambient_pressure: f32,
    drag: MediumPostDragModel,
    vapor_resistance: MediumPostVaporResistanceModel,
    thermal: ContactThermalModel,
    sound: ContactSoundModel,
};

const MediumPostVacuumModel = struct {
    pressure: f32,
    exposure: f32,
    drag_loss: f32,
    thermal_isolation: f32,
    sound_attenuation: f32,
    active: bool,
};

const MediumPostTransitionPolicy = struct {
    penetration_weight: f32 = 0.5,
    response_weight: f32 = 0.3,
    coupling_weight: f32 = 0.2,
    drag_weight: f32 = 0.35,
    vapor_weight: f32 = 0.25,
    vacuum_weight: f32 = 0.2,
    pressure_weight: f32 = 0.2,
};

const MediumPostTransitionInput = struct {
    from_medium: terrain.MediumType,
    to_medium: terrain.MediumType,
    penetration_depth: f32,
    buoyancy: f32,
    drag: MediumPostDragModel,
    vapor_resistance: MediumPostVaporResistanceModel,
    vacuum: MediumPostVacuumModel,
};

const MediumPostTransitionModel = struct {
    from_medium: terrain.MediumType,
    to_medium: terrain.MediumType,
    progress: f32,
    resistance: f32,
    pressure_delta: f32,
    active: bool,
};

const MediumPostMixingPolicy = struct {
    solid_density: f32 = 1.0,
    soft_density: f32 = 0.65,
    liquid_density: f32 = 1.0,
    vapor_density: f32 = 0.08,
    plasma_density: f32 = 0.16,
    solid_viscosity: f32 = 0.0,
    soft_viscosity: f32 = 0.45,
    liquid_viscosity: f32 = 0.85,
    vapor_viscosity: f32 = 0.08,
    plasma_viscosity: f32 = 0.18,
    drag_weight: f32 = 0.45,
    buoyancy_weight: f32 = 0.35,
    resistance_weight: f32 = 0.2,
};

const MediumPostMixingInput = struct {
    primary_medium: terrain.MediumType,
    secondary_medium: terrain.MediumType,
    buoyancy: f32,
    transition: MediumPostTransitionModel,
    drag: MediumPostDragModel,
    vapor_resistance: MediumPostVaporResistanceModel,
    vacuum: MediumPostVacuumModel,
};

const MediumPostMixingModel = struct {
    primary_medium: terrain.MediumType,
    secondary_medium: terrain.MediumType,
    mix_fraction: f32,
    effective_density: f32,
    effective_viscosity: f32,
    blended_drag: f32,
    blended_buoyancy: f32,
    active: bool,
};

const MediumPostState = enum(u8) {
    stable = 0,
    entering = 1,
    mixing = 2,
    settling = 3,
    isolated = 4,
};

const MediumPostStateMachinePolicy = struct {
    entering_threshold: f32 = 0.05,
    mixing_threshold: f32 = 0.35,
    settling_threshold: f32 = 0.85,
    vacuum_isolation_threshold: f32 = 0.5,
    mixing_stability_weight: f32 = 0.5,
    transition_stability_weight: f32 = 0.3,
    vacuum_stability_weight: f32 = 0.2,
};

const MediumPostStateMachineInput = struct {
    current_medium: terrain.MediumType,
    target_medium: terrain.MediumType,
    transition: MediumPostTransitionModel,
    mixing: MediumPostMixingModel,
    vacuum: MediumPostVacuumModel,
};

const MediumPostStateMachineModel = struct {
    state: MediumPostState,
    current_medium: terrain.MediumType,
    target_medium: terrain.MediumType,
    progress: f32,
    stability: f32,
    active: bool,
};

const MediumPostEventType = enum(u8) {
    none = 0,
    enter = 1,
    mix = 2,
    settle = 3,
    isolate = 4,
};

const MediumPostEventTriggerPolicy = struct {
    min_progress: f32 = 0.05,
    min_intensity: f32 = 0.01,
    progress_weight: f32 = 0.55,
    instability_weight: f32 = 0.25,
    resistance_weight: f32 = 0.12,
    pressure_weight: f32 = 0.08,
    resistance_reference: f32 = 8.0,
    pressure_reference: f32 = 1.0,
    high_priority_intensity: f32 = 0.75,
    normal_priority_intensity: f32 = 0.25,
};

const MediumPostEventTriggerInput = struct {
    state: MediumPostStateMachineModel,
    transition: MediumPostTransitionModel,
    mixing: MediumPostMixingModel,
    vacuum: MediumPostVacuumModel,
};

const MediumPostEventTriggerModel = struct {
    event_type: MediumPostEventType,
    source_medium: terrain.MediumType,
    target_medium: terrain.MediumType,
    intensity: f32,
    priority: bus.BusPriority,
    active: bool,
};

const MediumPostTransitionAnimationPolicy = struct {
    min_progress: f32 = 0.01,
    phase_scale: f32 = 1.0,
    event_phase_weight: f32 = 0.25,
    opacity_min: f32 = 0.15,
    ripple_weight: f32 = 0.45,
    turbulence_weight: f32 = 0.35,
    vacuum_fade_weight: f32 = 0.5,
    color_shift_weight: f32 = 0.6,
};

const MediumPostTransitionAnimationInput = struct {
    state: MediumPostStateMachineModel,
    event: MediumPostEventTriggerModel,
    transition: MediumPostTransitionModel,
    mixing: MediumPostMixingModel,
    vacuum: MediumPostVacuumModel,
};

const MediumPostTransitionAnimationModel = struct {
    phase: f32,
    blend: f32,
    opacity: f32,
    ripple: f32,
    turbulence: f32,
    color_shift: f32,
    active: bool,
};

const MediumPostTowPolicy = struct {
    liquid_tow: f32 = 0.6,
    soft_tow: f32 = 0.2,
    vapor_tow: f32 = 0.08,
    plasma_tow: f32 = 0.12,
    penetration_weight: f32 = 0.5,
    buoyancy_weight: f32 = 0.5,
    drag_coupling: f32 = 0.5,
    min_flow_magnitude: f32 = 0.001,
    max_velocity_delta: f32 = 64.0,
};

const MediumPostTowInput = struct {
    medium_type: terrain.MediumType,
    buoyancy: f32,
    penetration_depth: f32,
    flow_x: f32,
    flow_y: f32,
    flow_z: f32,
    drag: MediumPostDragModel,
};

const MediumPostTowModel = struct {
    force: f32,
    velocity_delta_x: f32,
    velocity_delta_y: f32,
    velocity_delta_z: f32,
    active: bool,
};

const MediumPostLiftPolicy = struct {
    liquid_lift: f32 = 0.12,
    soft_lift: f32 = 0.03,
    vapor_lift: f32 = 0.08,
    plasma_lift: f32 = 0.16,
    penetration_weight: f32 = 0.4,
    buoyancy_weight: f32 = 0.4,
    tow_coupling: f32 = 0.2,
    min_flow_magnitude: f32 = 0.001,
    max_velocity_delta: f32 = 64.0,
};

const MediumPostLiftInput = struct {
    medium_type: terrain.MediumType,
    buoyancy: f32,
    penetration_depth: f32,
    flow_x: f32,
    flow_y: f32,
    flow_z: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    tow: MediumPostTowModel,
};

const MediumPostLiftModel = struct {
    coefficient: f32,
    exposure: f32,
    dynamic_pressure: f32,
    magnitude: f32,
    force: f32,
    velocity_delta_x: f32,
    velocity_delta_y: f32,
    velocity_delta_z: f32,
    active: bool,
};

const MediumPostAddedMassPolicy = struct {
    liquid_added_mass: f32 = 0.55,
    soft_added_mass: f32 = 0.18,
    vapor_added_mass: f32 = 0.02,
    plasma_added_mass: f32 = 0.05,
    penetration_weight: f32 = 0.35,
    buoyancy_weight: f32 = 0.35,
    drag_weight: f32 = 0.15,
    lift_weight: f32 = 0.15,
    displaced_volume_mass_scale: f32 = 0.01,
    max_inertia_ratio: f32 = 4.0,
    normal_inertia_weight: f32 = 1.0,
    tangential_inertia_weight: f32 = 0.65,
};

const MediumPostAddedMassInput = struct {
    medium_type: terrain.MediumType,
    buoyancy: f32,
    penetration_depth: f32,
    base_mass: f32,
    buoyancy_model: MediumPostBuoyancyModel,
    drag: MediumPostDragModel,
    lift: MediumPostLiftModel,
};

const MediumPostAddedMassModel = struct {
    coefficient: f32,
    exposure: f32,
    displaced_volume: f32,
    added_mass: f32,
    effective_mass: f32,
    normal_inertia_scale: f32,
    tangential_inertia_scale: f32,
    active: bool,
};

const MediumPostThermalPolicy = struct {
    liquid_conduction_scale: f32 = 0.55,
    soft_conduction_scale: f32 = 0.25,
    vapor_conduction_scale: f32 = 0.12,
    plasma_conduction_scale: f32 = 0.8,
    drag_heat_weight: f32 = 0.02,
    tow_heat_weight: f32 = 0.01,
    lift_heat_weight: f32 = 0.015,
    buoyancy_cooling_weight: f32 = 0.35,
    max_conducted_heat: f32 = 16.0,
};

const MediumPostThermalInput = struct {
    medium_type: terrain.MediumType,
    buoyancy: f32,
    thermal: ContactThermalModel,
    drag: MediumPostDragModel,
    tow: MediumPostTowModel,
    lift: MediumPostLiftModel,
};

const MediumPostThermalModel = struct {
    conducted_heat: f32,
    conductivity: f32,
    retained_heat: f32,
    temperature_delta: f32,
    active: bool,
};

const ContactThermalInput = struct {
    penetration_depth: f32,
    relative_normal_speed: f32,
    tangential_speed: f32,
    friction: f32,
    rolling_friction: f32 = 0.0,
    angular_speed: f32 = 0.0,
    anisotropic_minor_axis_scale: f32 = 1.0,
    penetration_resistance: f32,
    buoyancy: f32,
    medium_type: terrain.MediumType,
};

const ContactThermalModel = struct {
    generated_heat: f32,
    friction_heat: f32 = 0.0,
    friction_heat_fraction: f32 = 0.0,
    conductivity: f32,
    dissipated_heat: f32,
    retained_heat: f32,
    temperature_delta: f32,
};

const ContactSoundPolicy = struct {
    normal_volume_weight: f32 = 0.25,
    tangential_volume_weight: f32 = 0.1,
    penetration_volume_weight: f32 = 0.2,
    min_volume: f32 = 0.0,
    max_volume: f32 = 1.0,
    base_pitch: f32 = 1.0,
    restitution_pitch_scale: f32 = 0.4,
    friction_pitch_damping: f32 = 0.2,
    min_pitch: f32 = 0.5,
    max_pitch: f32 = 2.0,
    base_duration: f32 = 0.05,
    duration_per_penetration: f32 = 0.02,
    duration_per_tangent_speed: f32 = 0.005,
    max_duration: f32 = 0.5,
    liquid_volume_scale: f32 = 0.65,
    soft_volume_scale: f32 = 0.8,
    vapor_volume_scale: f32 = 0.35,
    plasma_volume_scale: f32 = 1.2,
};

const ContactSoundInput = struct {
    sound_type: u8,
    penetration_depth: f32,
    relative_normal_speed: f32,
    tangential_speed: f32,
    friction: f32,
    restitution: f32,
    medium_type: terrain.MediumType,
};

const ContactSoundModel = struct {
    sound_type: u8,
    volume: f32,
    pitch: f32,
    duration: f32,
    audible: bool,
};

const ContactDustPolicy = struct {
    normal_intensity_weight: f32 = 0.12,
    tangential_intensity_weight: f32 = 0.25,
    penetration_intensity_weight: f32 = 0.2,
    min_intensity: f32 = 0.0,
    max_intensity: f32 = 1.0,
    base_radius: f32 = 0.25,
    radius_per_intensity: f32 = 1.5,
    radius_per_tangent_speed: f32 = 0.05,
    max_radius: f32 = 4.0,
    base_duration: f32 = 0.1,
    duration_per_intensity: f32 = 0.75,
    max_duration: f32 = 2.0,
    soft_intensity_scale: f32 = 1.25,
    liquid_intensity_scale: f32 = 0.65,
    vapor_intensity_scale: f32 = 0.2,
    plasma_intensity_scale: f32 = 0.4,
    buoyancy_suppression_scale: f32 = 0.75,
};

const ContactDustInput = struct {
    dust_type: u8,
    penetration_depth: f32,
    relative_normal_speed: f32,
    tangential_speed: f32,
    friction: f32,
    buoyancy: f32,
    medium_type: terrain.MediumType,
};

const ContactDustModel = struct {
    dust_type: u8,
    intensity: f32,
    radius: f32,
    duration: f32,
    emitted: bool,
};

const ContactDeformationPolicy = struct {
    penetration_weight: f32 = 0.6,
    normal_speed_weight: f32 = 0.15,
    tangential_speed_weight: f32 = 0.05,
    resistance_reduction_scale: f32 = 0.75,
    damage_amplification_scale: f32 = 0.5,
    min_total: f32 = 0.0,
    max_total: f32 = 4.0,
    base_permanent_fraction: f32 = 0.1,
    damage_permanent_scale: f32 = 0.25,
    restitution_recovery_scale: f32 = 0.5,
    max_permanent_fraction: f32 = 0.9,
};

const ContactDeformationInput = struct {
    penetration_depth: f32,
    relative_normal_speed: f32,
    tangential_speed: f32,
    penetration_resistance: f32,
    damage_modifier: f32,
    restitution: f32,
};

const ContactDeformationModel = struct {
    total_depth: f32,
    elastic_depth: f32,
    permanent_depth: f32,
    recovery_fraction: f32,
    severe: bool,
};

pub const ContactSeparationState = enum(u8) {
    closing = 0,
    persisting = 1,
    separating = 2,
};

const ContactSeparationPolicy = struct {
    velocity_epsilon: f32 = 0.01,
    penetration_slop: f32 = 0.01,
    max_separation_time: f32 = 1.0,
};

const ContactSeparationInput = struct {
    penetration_depth: f32,
    predicted_penetration_depth: f32,
    normal_speed_signed: f32,
};

const ContactSeparationModel = struct {
    state: ContactSeparationState,
    separating: bool,
    speed: f32,
    estimated_time: f32,
};

const ContactStabilizationPolicy = struct {
    resting_speed_threshold: f32 = 0.05,
    shallow_penetration_depth: f32 = 0.25,
    persisting_bias_scale: f32 = 1.12,
    persisting_impulse_scale: f32 = 1.08,
    closing_bias_scale: f32 = 1.05,
    closing_impulse_scale: f32 = 1.05,
    separating_bias_scale: f32 = 0.5,
    separating_impulse_scale: f32 = 0.75,
    min_scale: f32 = 0.25,
    max_scale: f32 = 1.25,
};

const ContactStabilizationInput = struct {
    penetration_depth: f32,
    predicted_penetration_depth: f32,
    separation: ContactSeparationModel,
};

const ContactStabilizationModel = struct {
    bias_scale: f32,
    impulse_scale: f32,
    stabilized: bool,
};

const DamageEvalImpactPolicy = struct {
    legacy_mass_scale: f32 = 100.0,
    energy_scale: f32 = 1000.0,
    tangential_energy_scale: f32 = 0.25,
    max_legacy_impact: f32 = 65535.0,
};

const DamageEvalImpactInput = struct {
    normal_speed: f32,
    tangential_speed: f32 = 0.0,
    mass: f32,
    material: entity16.MaterialType,
    hardness: u16,
    damage_modifier: f32 = 1.0,
};

const DamageEvalImpactModel = struct {
    legacy_impact: u16,
    normal_speed: f32,
    tangential_speed: f32,
    kinetic_energy: f32,
    damage_amount: f32,
};

const DamageEvalHardnessPolicy = struct {
    fragile_impact_threshold: f32 = 50.0,
    min_impact_threshold: f32 = 1.0,
    max_hardness: f32 = 255.0,
};

const DamageEvalHardnessInput = struct {
    impact: DamageEvalImpactModel,
    material: entity16.MaterialType,
    hardness: u16,
};

const DamageEvalHardnessModel = struct {
    hardness: u16,
    impact_threshold: f32,
    impact_ratio: f32,
    hardness_resistance: f32,
    exceeds_hardness: bool,
};

const DamageEvalBreakInput = struct {
    impact: DamageEvalImpactModel,
    hardness: DamageEvalHardnessModel,
    material: entity16.MaterialType,
};

const DamageEvalBreakModel = struct {
    legacy_impact: u16,
    impact_threshold: f32,
    impact_ratio: f32,
    did_break: bool,
    fragments: u8,
};

const DamageEvalFragmentPolicy = struct {
    max_fragments: u8 = 16,
    impact_x: i8 = 8,
    impact_y: i8 = 8,
    impact_z: i8 = 8,
    min_fragment_count: u8 = 1,
    min_fragment_energy: f32 = 20.0,
    velocity_energy_scale: f32 = 2.0,
};

const DamageEvalFragmentInput = struct {
    entity: *const entity16.Entity16,
    impact: DamageEvalImpactModel,
    break_model: DamageEvalBreakModel,
    impact_velocity: i16,
    speed_scale: f32 = 1.0,
};

const DamageEvalFragmentModel = struct {
    requested_fragments: u8,
    generated_fragments: u8,
    fragment_energy: f32,
    debris_mass: f32,
    debris_speed: f32,
    fragments: [16]destruction.Fragment,
};

const DamageEvalCrackPolicy = struct {
    impact_x: i8 = 8,
    impact_y: i8 = 8,
    impact_z: i8 = 8,
    seed: u32 = 0xDACE_0001,
    min_propagation_ratio: f32 = 0.5,
    break_energy_scale: f32 = 1.0,
    sub_break_energy_scale: f32 = 0.25,
};

const DamageEvalCrackInput = struct {
    entity: *const entity16.Entity16,
    impact: DamageEvalImpactModel,
    hardness: DamageEvalHardnessModel,
    break_model: DamageEvalBreakModel,
    fragments: DamageEvalFragmentModel,
};

const DamageEvalCrackModel = struct {
    propagated: bool,
    crack_count: u8,
    max_severity: f32,
    energy: f32,
    pattern: destruction.FracturePattern,
};

const ContactRestitutionSolvePolicy = struct {
    velocity_threshold: f32 = 0.5,
    max_restitution: f32 = 1.0,
};

const ContactVelocitySolvePolicy = struct {
    restitution: ContactRestitutionSolvePolicy = .{},
};

const ContactFrictionSolvePolicy = struct {
    min_normal_impulse_limit: f32 = 0.5,
    max_impulse_per_friction: f32 = 8.0,
    min_equation_max_impulse: f32 = 0.25,
    ellipse_minor_axis_scale: f32 = 0.75,
};

const ContactFrictionConeApproximation = struct {
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    tangential_speed: f32,
    impulse_limit: f32,
};

const ContactFrictionEllipseApproximation = struct {
    cone: ContactFrictionConeApproximation,
    major_x: f32,
    major_y: f32,
    major_z: f32,
    minor_x: f32,
    minor_y: f32,
    minor_z: f32,
    major_impulse_limit: f32,
    minor_impulse_limit: f32,
    impulse_limit: f32,
};

const ContactTangentConstraintSpec = struct {
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    speed: f32,
    impulse_limit: f32,
};

const ContactNormalConstraintSpec = struct {
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    penetration_depth: f32,
    restitution: f32,
};

const ContactPairSolveRequest = struct {
    normal: ?ContactSolveRequest = null,
    friction: ?ContactSolveRequest = null,
};

const ContactPreparedPairPlan = struct {
    request: ContactPairSolveRequest,
    requires_tangent: bool = false,
};

const ContactExecutionContext = struct {
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    prepared: ContactPreparedPair,
};

const ContactExecutionOutcome = union(enum) {
    inactive,
    stalled,
    ready: ContactExecutionContext,
};

const ContactSolveChannel = enum(u8) {
    normal,
    friction,
};

const ContactRowPlanBuilderFn = *const fn (
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ContactPreparedPairPlan;

const ContactPreparedStepSelectorFn = *const fn (
    result: ContactPairSolveResult,
) ConstraintSolveStep;

const ContactRuntimeRowDescriptor = struct {
    kind: ConstraintRowKind,
    channel: ContactSolveChannel,
    settle_after_solve: bool,
    requires_tangent: bool,
    build_plan: ContactRowPlanBuilderFn,
    select_step: ContactPreparedStepSelectorFn,
};

const ContactRuntimeSolveContext = struct {
    kind: ConstraintRowKind,
    descriptor: *const ContactRuntimeRowDescriptor,
    contact: ContactExecutionContext,
    before: f32,
};

const ContactRuntimeSolveContextOutcome = union(enum) {
    inactive,
    stalled,
    ready: ContactRuntimeSolveContext,
};

const ContactBatchPlanBuilderFn = *const fn (
    ctx: ContactExecutionContext,
) ContactPreparedPairPlan;

const ContactBatchSolveDescriptor = struct {
    settle_after_solve: bool,
    build_plan: ContactBatchPlanBuilderFn,
};

const ContactBatchSolveContext = struct {
    descriptor: *const ContactBatchSolveDescriptor,
    contact: ContactExecutionContext,
};

const ContactBatchSolveContextOutcome = union(enum) {
    inactive,
    stalled,
    ready: ContactBatchSolveContext,
};

const ContactPreparedPairOutcome = union(enum) {
    ready: ContactPairSolveResult,
    stalled,
};

fn MapPreparedContactPairOutcomeReadyFn(comptime Context: type, comptime Out: type) type {
    return *const fn (ctx: Context, pair_result: ContactPairSolveResult) Out;
}

fn mapPreparedContactPairOutcome(
    comptime Context: type,
    comptime Out: type,
    ctx: Context,
    outcome: ContactPreparedPairOutcome,
    stalled_value: Out,
    map_ready: MapPreparedContactPairOutcomeReadyFn(Context, Out),
) Out {
    return switch (outcome) {
        .ready => |pair_result| map_ready(ctx, pair_result),
        .stalled => stalled_value,
    };
}

const PreparedEnvironmentConstraint = struct {
    normal: PreparedDirectionalConstraint,
    move_x: i32,
    move_y: i32,
    move_z: i32,
};

const EnvironmentPenetrationProbe = struct {
    inst: *const scene32.Instance,
    entity: *const entity16.Entity16,
    penetration: query_types.PenetrationResult,
};

const EnvironmentResolvedMotion = struct {
    move_x: i32,
    move_y: i32,
    move_z: i32,

    fn changed(self: EnvironmentResolvedMotion) bool {
        return self.move_x != 0 or self.move_y != 0 or self.move_z != 0;
    }
};

const EnvironmentSolveResult = struct {
    solve_step: ConstraintSolveStep,
    resolved_motion: EnvironmentResolvedMotion,

    fn applyResolvedMotion(
        result: EnvironmentSolveResult,
        inst: *scene32.Instance,
        entity: *const entity16.Entity16,
        previous_vel_x: i16,
        previous_vel_y: i16,
        previous_vel_z: i16,
    ) bool {
        return applyEnvironmentResolvedMotion(
            inst,
            entity,
            result.resolved_motion.move_x,
            result.resolved_motion.move_y,
            result.resolved_motion.move_z,
            previous_vel_x,
            previous_vel_y,
            previous_vel_z,
        );
    }
};

const EnvironmentConstraintMetrics = DirectionalConstraintMetrics;

const EnvironmentExecutionContext = struct {
    instance_idx: u8,
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    prepared: PreparedEnvironmentConstraint,
    previous_vel_x: i16,
    previous_vel_y: i16,
    previous_vel_z: i16,
};

const EnvironmentExecutionOutcome = union(enum) {
    inactive,
    stalled,
    ready: EnvironmentExecutionContext,
};

const EnvironmentRuntimeSolveContext = struct {
    execution: EnvironmentExecutionContext,
    before: f32,
};

const EnvironmentRuntimeSolveContextOutcome = union(enum) {
    inactive,
    stalled,
    ready: EnvironmentRuntimeSolveContext,
};

const EnvironmentBatchSolveContext = struct {
    execution: EnvironmentExecutionContext,
};

const EnvironmentBatchSolveContextOutcome = union(enum) {
    inactive,
    stalled,
    ready: EnvironmentBatchSolveContext,
};

fn MapPreparedExecutionOutcomeFn(comptime Context: type, comptime InReady: type, comptime OutReady: type) type {
    return *const fn (ctx: Context, ready: InReady) OutReady;
}

fn mapPreparedExecutionOutcome(
    comptime Context: type,
    comptime InReady: type,
    comptime OutReady: type,
    comptime InOutcome: type,
    comptime OutOutcome: type,
    ctx: Context,
    outcome: InOutcome,
    map_ready: MapPreparedExecutionOutcomeFn(Context, InReady, OutReady),
) OutOutcome {
    return switch (outcome) {
        .inactive => @unionInit(OutOutcome, "inactive", {}),
        .stalled => @unionInit(OutOutcome, "stalled", {}),
        .ready => |ready| @unionInit(OutOutcome, "ready", map_ready(ctx, ready)),
    };
}

fn preparePreparedRuntimeSolveContext(
    comptime Context: type,
    comptime InReady: type,
    comptime OutReady: type,
    comptime InOutcome: type,
    comptime OutOutcome: type,
    ctx: Context,
    base_residual: f32,
    outcome: InOutcome,
    map_ready: MapPreparedExecutionOutcomeFn(Context, InReady, OutReady),
) OutOutcome {
    if (base_residual <= 0.0) return .inactive;
    return mapPreparedExecutionOutcome(
        Context,
        InReady,
        OutReady,
        InOutcome,
        OutOutcome,
        ctx,
        outcome,
        map_ready,
    );
}

fn preparePreparedBatchSolveContext(
    comptime Context: type,
    comptime InReady: type,
    comptime OutReady: type,
    comptime InOutcome: type,
    comptime OutOutcome: type,
    ctx: Context,
    outcome: InOutcome,
    map_ready: MapPreparedExecutionOutcomeFn(Context, InReady, OutReady),
) OutOutcome {
    return mapPreparedExecutionOutcome(
        Context,
        InReady,
        OutReady,
        InOutcome,
        OutOutcome,
        ctx,
        outcome,
        map_ready,
    );
}

fn ExecutePreparedRuntimeReadyFn(comptime Context: type, comptime Ready: type) type {
    return *const fn (ctx: Context, ready: Ready) ConstraintRowExecResult;
}

fn executePreparedRuntimeOutcome(
    comptime Ready: type,
    comptime Outcome: type,
    comptime Context: type,
    outcome: Outcome,
    ctx: Context,
    base_residual: f32,
    execute_ready: ExecutePreparedRuntimeReadyFn(Context, Ready),
) ConstraintRowExecResult {
    return switch (outcome) {
        .inactive => inactiveConstraintRowResult(),
        .stalled => stalledConstraintRowResult(base_residual),
        .ready => |ready| execute_ready(ctx, ready),
    };
}

fn ExecutePreparedBatchReadyFn(comptime Context: type, comptime Ready: type) type {
    return *const fn (ctx: Context, ready: Ready) bool;
}

fn executePreparedBatchOutcome(
    comptime Ready: type,
    comptime Outcome: type,
    comptime Context: type,
    outcome: Outcome,
    ctx: Context,
    execute_ready: ExecutePreparedBatchReadyFn(Context, Ready),
) bool {
    return switch (outcome) {
        .inactive, .stalled => false,
        .ready => |ready| execute_ready(ctx, ready),
    };
}

fn ExecutePreparedBatchReadyDirectFn(comptime Ready: type) type {
    return *const fn (ready: Ready) bool;
}

fn executePreparedBatchOutcomeDirect(
    comptime Ready: type,
    comptime Outcome: type,
    outcome: Outcome,
    execute_ready: ExecutePreparedBatchReadyDirectFn(Ready),
) bool {
    return switch (outcome) {
        .inactive, .stalled => false,
        .ready => |ready| execute_ready(ready),
    };
}

pub const ImpactBreakDecision = struct {
    break_self: bool,
    break_blocker: bool,
};

pub const SweepMotionResult = struct {
    pos_x: i32,
    pos_y: i32,
    pos_z: i32,
    blocked: bool,
    blocker_id: u8,

    pub fn movedAlongAxis(self: SweepMotionResult, axis: physics.SweepAxis, start_x: i32, start_y: i32, start_z: i32) bool {
        return switch (axis) {
            .x => self.pos_x != start_x,
            .y => self.pos_y != start_y,
            .z => self.pos_z != start_z,
        };
    }
};

pub fn canParticipateInBroadPhase(inst: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (inst.state == .broken) return false;
    if (inst.entity_id >= entities.len) return false;
    return physics.computeEntityWorldAABB(inst, &entities[inst.entity_id]) != null;
}

pub fn canProcessDynamicInstance(inst: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (inst.state == .broken) return false;
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];
    return (entity.physics.flags & 0x01) == 0;
}

pub fn isStaticInstance(inst: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];
    return (entity.physics.flags & 0x01) != 0;
}

pub fn sortByDescendingPriority(items: anytype) void {
    if (items.len < 2) return;

    var i: usize = 0;
    while (i < items.len - 1) : (i += 1) {
        var j: usize = 0;
        while (j < items.len - i - 1) : (j += 1) {
            if (items[j].priority < items[j + 1].priority) {
                const tmp = items[j];
                items[j] = items[j + 1];
                items[j + 1] = tmp;
            }
        }
    }
}

pub fn writeConstraintRowDebugSnapshots(
    rows: []const ConstraintRow,
    out_snapshots: []ConstraintRowDebugSnapshot,
) usize {
    const count = @min(rows.len, out_snapshots.len);
    for (rows[0..count], 0..) |row, idx| {
        out_snapshots[idx] = .{
            .kind = row.kind,
            .index = row.index,
            .priority = row.priority,
            .base_residual = row.base_residual,
            .metadata = row.metadata,
            .equation = row.equation,
        };
    }
    return count;
}

pub fn collectConstraintRowDebugSnapshots(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    out_snapshots: []ConstraintRowDebugSnapshot,
) usize {
    var island_storage: [scene32.MAX_INSTANCES][scene32.MAX_INSTANCES]u8 = undefined;
    var island_lengths: [scene32.MAX_INSTANCES]usize = [_]usize{0} ** scene32.MAX_INSTANCES;
    const island_count = buildConstraintIslands(
        s1024,
        entities,
        joints,
        broadphase_pairs,
        island_storage[0..],
        island_lengths[0..],
    );
    if (island_count == 0 or out_snapshots.len == 0) return 0;

    var island_order: [scene32.MAX_INSTANCES]usize = undefined;
    sortConstraintIslandsByStress(
        s1024,
        entities,
        joints,
        broadphase_pairs,
        island_storage[0..island_count],
        island_lengths[0..island_count],
        island_count,
        island_order[0..island_count],
    );

    var total_count: usize = 0;
    var island_order_idx: usize = 0;
    while (island_order_idx < island_count and total_count < out_snapshots.len) : (island_order_idx += 1) {
        const island_idx = island_order[island_order_idx];
        const instance_indices = island_storage[island_idx][0..island_lengths[island_idx]];

        var joint_index_storage: [joint.MAX_JOINTS]usize = undefined;
        const joint_indices = copyJointIndexSubsetForIsland(joints, instance_indices, joint_index_storage[0..]);
        var pair_subset_storage: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined;
        const pair_subset = copyBroadPhaseSubsetForIsland(broadphase_pairs, instance_indices, pair_subset_storage[0..]);
        var empty_row_states: [0]ConstraintRowState = .{};

        var row_storage: [joint.MAX_JOINTS * 3 + MAX_BROADPHASE_PAIRS * 2 + scene32.MAX_INSTANCES]ConstraintRow = undefined;
        const row_count = buildConstraintRowsForIsland(
            s1024,
            entities,
            joints,
            joint_indices,
            pair_subset,
            instance_indices,
            empty_row_states[0..],
            0,
            row_storage[0..],
        );
        if (row_count == 0) continue;

        sortConstraintRows(row_storage[0..row_count], false);
        total_count += writeConstraintRowDebugSnapshots(
            row_storage[0..row_count],
            out_snapshots[total_count..],
        );
    }

    return total_count;
}

pub fn shouldGenerateBroadPhasePair(a: *const scene32.Instance, b: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (!canParticipateInBroadPhase(a, entities) or !canParticipateInBroadPhase(b, entities)) return false;

    const entity_a = &entities[a.entity_id];
    const entity_b = &entities[b.entity_id];
    const static_a = (entity_a.physics.flags & 0x01) != 0;
    const static_b = (entity_b.physics.flags & 0x01) != 0;
    if (static_a and static_b) return false;

    const swept_a = physics.computeSweptEntityWorldAABB(a, entity_a).?;
    const swept_b = physics.computeSweptEntityWorldAABB(b, entity_b).?;
    return physics.aabbHit(swept_a, swept_b);
}

fn buildBroadPhaseCandidates(instances: []const scene32.Instance, entities: []entity16.Entity16, out_candidates: []BroadPhaseCandidate, stats: *BroadPhaseStats) usize {
    var count: usize = 0;
    var i: usize = 0;
    while (i < instances.len and i < scene32.MAX_INSTANCES) : (i += 1) {
        const inst = &instances[i];
        if (inst.state == .broken) continue;
        if (inst.entity_id >= entities.len) continue;

        const entity = &entities[inst.entity_id];
        const swept_aabb = physics.computeSweptEntityWorldAABB(inst, entity) orelse continue;
        stats.aabb_builds += 1;
        if (count >= out_candidates.len) break;

        out_candidates[count] = .{
            .index = @intCast(i),
            .is_static = (entity.physics.flags & 0x01) != 0,
            .swept_aabb = swept_aabb,
        };
        count += 1;
    }
    stats.candidate_count = count;
    return count;
}

fn shouldGenerateBroadPhaseCandidatePair(a: BroadPhaseCandidate, b: BroadPhaseCandidate) bool {
    if (a.is_static and b.is_static) return false;
    return physics.aabbHit(a.swept_aabb, b.swept_aabb);
}

pub fn collectBroadPhasePairs(instances: []const scene32.Instance, entities: []entity16.Entity16, out_pairs: []BroadPhasePair) usize {
    var stats: BroadPhaseStats = .{};
    return collectBroadPhasePairsMeasured(instances, entities, out_pairs, &stats);
}

pub fn collectBroadPhasePairsMeasured(instances: []const scene32.Instance, entities: []entity16.Entity16, out_pairs: []BroadPhasePair, stats: *BroadPhaseStats) usize {
    stats.* = .{};
    var pair_count: usize = 0;
    var candidates: [scene32.MAX_INSTANCES]BroadPhaseCandidate = undefined;
    const candidate_count = buildBroadPhaseCandidates(instances, entities, candidates[0..], stats);

    var i: usize = 0;
    while (i < candidate_count) : (i += 1) {
        var j: usize = i + 1;
        while (j < candidate_count) : (j += 1) {
            stats.pair_tests += 1;
            if (!shouldGenerateBroadPhaseCandidatePair(candidates[i], candidates[j])) continue;
            if (pair_count >= out_pairs.len) {
                stats.pair_count = pair_count;
                return pair_count;
            }
            out_pairs[pair_count] = .{
                .a = @min(candidates[i].index, candidates[j].index),
                .b = @max(candidates[i].index, candidates[j].index),
            };
            pair_count += 1;
        }
    }

    stats.pair_count = pair_count;
    return pair_count;
}

const CollisionEmitPolicy = struct {
    min_abs_impact_velocity: i16 = 0,
};

const CollisionEmitInput = struct {
    entity_id: u16,
    impact_velocity: i16,
    entity: *const entity16.Entity16,
};

const CollisionEmitModel = struct {
    entity_id: u16,
    payload: bus.CollisionPayload,
    active: bool,
};

const SoundEmitPolicy = struct {
    min_volume: f32 = 0.01,
};

const SoundEmitInput = struct {
    entity_id: u16,
    sound: ContactSoundModel,
};

const SoundEmitModel = struct {
    entity_id: u16,
    payload: bus.SoundPayload,
    active: bool,
};

const ParticleEmitPolicy = struct {
    min_intensity: f32 = 0.01,
};

const ParticleEmitInput = struct {
    entity_id: u16,
    dust: ContactDustModel,
};

const ParticleEmitModel = struct {
    entity_id: u16,
    payload: bus.ParticlePayload,
    active: bool,
};

const DeformationEmitPolicy = struct {
    min_total_depth: f32 = 0.01,
};

const DeformationEmitInput = struct {
    entity_id: u16,
    deformation: ContactDeformationModel,
};

const DeformationEmitModel = struct {
    entity_id: u16,
    payload: bus.DeformationPayload,
    active: bool,
};

const BreakEmitPolicy = struct {};

const BreakEmitInput = struct {
    entity_id: u16,
    impact_velocity: i16,
    entity: *const entity16.Entity16,
};

const BreakEmitModel = struct {
    entity_id: u16,
    payload: bus.BreakPayload,
    active: bool,
};

fn collisionEmitPolicy() CollisionEmitPolicy {
    return .{};
}

fn neutralCollisionEmitModel(entity_id: u16) CollisionEmitModel {
    return .{
        .entity_id = entity_id,
        .payload = .{
            .impact_velocity = 0,
            .hardness = 0,
            .did_break = false,
        },
        .active = false,
    };
}

fn computeCollisionEmitModel(input: CollisionEmitInput, policy: CollisionEmitPolicy) CollisionEmitModel {
    if (@abs(input.impact_velocity) < policy.min_abs_impact_velocity) {
        return neutralCollisionEmitModel(input.entity_id);
    }

    return .{
        .entity_id = input.entity_id,
        .payload = collision_event.makeCollisionPayload(input.impact_velocity, input.entity),
        .active = true,
    };
}

fn enqueueCollisionEmitModel(queue: *collision_event.PendingCollisionQueue, model: CollisionEmitModel) bool {
    if (!model.active) return false;

    var i: u8 = 0;
    while (i < queue.count) : (i += 1) {
        if (queue.entity_ids[i] != model.entity_id) continue;
        const existing = queue.events[i];
        if (existing.impact_velocity == model.payload.impact_velocity and
            existing.hardness == model.payload.hardness and
            existing.did_break == model.payload.did_break)
        {
            return false;
        }
    }

    if (queue.count >= queue.events.len) return false;
    const idx = queue.count;
    queue.entity_ids[idx] = model.entity_id;
    queue.events[idx] = model.payload;
    queue.count += 1;
    return true;
}

fn soundEmitPolicy() SoundEmitPolicy {
    return .{};
}

fn neutralSoundEmitModel(entity_id: u16) SoundEmitModel {
    return .{
        .entity_id = entity_id,
        .payload = .{
            .sound_type = 0,
            .volume = 0.0,
            .pitch = 1.0,
            .duration = 0.0,
        },
        .active = false,
    };
}

fn computeSoundEmitModel(input: SoundEmitInput, policy: SoundEmitPolicy) SoundEmitModel {
    if (!input.sound.audible or input.sound.volume < policy.min_volume) {
        return neutralSoundEmitModel(input.entity_id);
    }

    return .{
        .entity_id = input.entity_id,
        .payload = .{
            .sound_type = input.sound.sound_type,
            .volume = input.sound.volume,
            .pitch = input.sound.pitch,
            .duration = input.sound.duration,
        },
        .active = true,
    };
}

fn enqueueSoundEmitModel(queue: *collision_event.PendingSoundQueue, model: SoundEmitModel) bool {
    if (!model.active) return false;
    const previous = queue.count;
    queue.enqueueEntity(model.entity_id, model.payload);
    return queue.count != previous;
}

fn particleEmitPolicy() ParticleEmitPolicy {
    return .{};
}

fn neutralParticleEmitModel(entity_id: u16) ParticleEmitModel {
    return .{
        .entity_id = entity_id,
        .payload = .{
            .particle_type = 0,
            .intensity = 0.0,
            .radius = 0.0,
            .duration = 0.0,
        },
        .active = false,
    };
}

fn computeParticleEmitModel(input: ParticleEmitInput, policy: ParticleEmitPolicy) ParticleEmitModel {
    if (!input.dust.emitted or input.dust.intensity < policy.min_intensity) {
        return neutralParticleEmitModel(input.entity_id);
    }

    return .{
        .entity_id = input.entity_id,
        .payload = .{
            .particle_type = input.dust.dust_type,
            .intensity = input.dust.intensity,
            .radius = input.dust.radius,
            .duration = input.dust.duration,
        },
        .active = true,
    };
}

fn enqueueParticleEmitModel(queue: *collision_event.PendingParticleQueue, model: ParticleEmitModel) bool {
    if (!model.active) return false;
    const previous = queue.count;
    queue.enqueueEntity(model.entity_id, model.payload);
    return queue.count != previous;
}

fn deformationEmitPolicy() DeformationEmitPolicy {
    return .{};
}

fn neutralDeformationEmitModel(entity_id: u16) DeformationEmitModel {
    return .{
        .entity_id = entity_id,
        .payload = .{
            .total_depth = 0.0,
            .permanent_depth = 0.0,
            .recovery_fraction = 1.0,
            .severe = false,
        },
        .active = false,
    };
}

fn computeDeformationEmitModel(input: DeformationEmitInput, policy: DeformationEmitPolicy) DeformationEmitModel {
    if (input.deformation.total_depth < policy.min_total_depth) {
        return neutralDeformationEmitModel(input.entity_id);
    }

    return .{
        .entity_id = input.entity_id,
        .payload = .{
            .total_depth = input.deformation.total_depth,
            .permanent_depth = input.deformation.permanent_depth,
            .recovery_fraction = input.deformation.recovery_fraction,
            .severe = input.deformation.severe,
        },
        .active = true,
    };
}

fn enqueueDeformationEmitModel(queue: *collision_event.PendingDeformationQueue, model: DeformationEmitModel) bool {
    if (!model.active) return false;
    const previous = queue.count;
    queue.enqueueEntity(model.entity_id, model.payload);
    return queue.count != previous;
}

fn breakEmitPolicy() BreakEmitPolicy {
    return .{};
}

fn neutralBreakEmitModel(entity_id: u16) BreakEmitModel {
    return .{
        .entity_id = entity_id,
        .payload = .{
            .impact_velocity = 0,
            .hardness = 0,
            .fragment_count = 0,
        },
        .active = false,
    };
}

fn computeBreakEmitModel(input: BreakEmitInput, policy: BreakEmitPolicy) BreakEmitModel {
    _ = policy;
    const impact = computeDamageEvalImpactFromVelocity(input.entity, input.impact_velocity);
    const hardness = computeDamageEvalHardnessForEntity(input.entity, impact);
    const break_model = computeDamageEvalBreakForEntity(input.entity, impact, hardness);
    if (!break_model.did_break) return neutralBreakEmitModel(input.entity_id);

    return .{
        .entity_id = input.entity_id,
        .payload = .{
            .impact_velocity = input.impact_velocity,
            .hardness = input.entity.physics.hardness,
            .fragment_count = break_model.fragments,
        },
        .active = true,
    };
}

fn enqueueBreakEmitModel(queue: *collision_event.PendingBreakQueue, model: BreakEmitModel) bool {
    if (!model.active) return false;
    const previous = queue.count;
    queue.enqueueEntity(model.entity_id, model.payload);
    return queue.count != previous;
}

fn computeCollisionPairSoundModel(
    source_entity: *const entity16.Entity16,
    blocker_entity: *const entity16.Entity16,
    impact_velocity: i16,
) ContactSoundModel {
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(source_entity.physics.material),
        source_entity.physics.material,
        material_pairing.getSurfaceForMaterial(blocker_entity.physics.material),
        blocker_entity.physics.material,
    );
    return computeContactSoundModel(
        .{
            .sound_type = response.sound_type,
            .penetration_depth = 1.0,
            .relative_normal_speed = @as(f32, @floatFromInt(@abs(impact_velocity))),
            .tangential_speed = 0.0,
            .friction = response.friction,
            .restitution = response.restitution,
            .medium_type = response.medium_type,
        },
        contactSoundPolicy(),
    );
}

fn computeCollisionPairDustModel(
    source_entity: *const entity16.Entity16,
    blocker_entity: *const entity16.Entity16,
    impact_velocity: i16,
) ContactDustModel {
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(source_entity.physics.material),
        source_entity.physics.material,
        material_pairing.getSurfaceForMaterial(blocker_entity.physics.material),
        blocker_entity.physics.material,
    );
    return computeContactDustModel(
        .{
            .dust_type = response.dust_type,
            .penetration_depth = 1.0,
            .relative_normal_speed = @as(f32, @floatFromInt(@abs(impact_velocity))),
            .tangential_speed = 0.0,
            .friction = response.friction,
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
        },
        contactDustPolicy(),
    );
}

fn computeCollisionPairDeformationModel(
    source_entity: *const entity16.Entity16,
    blocker_entity: *const entity16.Entity16,
    impact_velocity: i16,
) ContactDeformationModel {
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(source_entity.physics.material),
        source_entity.physics.material,
        material_pairing.getSurfaceForMaterial(blocker_entity.physics.material),
        blocker_entity.physics.material,
    );
    return computeContactDeformationModel(
        .{
            .penetration_depth = 1.0,
            .relative_normal_speed = @as(f32, @floatFromInt(@abs(impact_velocity))),
            .tangential_speed = 0.0,
            .penetration_resistance = response.penetration_resistance,
            .damage_modifier = response.damage_modifier,
            .restitution = response.restitution,
        },
        contactDeformationPolicy(),
    );
}

fn computeEnvironmentCollisionSoundModel(
    source_entity: *const entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
) ContactSoundModel {
    const surface = terrain.getSurfaceAt(inst.pos_x, inst.pos_z);
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(source_entity.physics.material),
        source_entity.physics.material,
        surface,
        .solid,
    );
    return computeContactSoundModel(
        .{
            .sound_type = response.sound_type,
            .penetration_depth = 1.0,
            .relative_normal_speed = @as(f32, @floatFromInt(@abs(impact_velocity))),
            .tangential_speed = 0.0,
            .friction = response.friction,
            .restitution = response.restitution,
            .medium_type = response.medium_type,
        },
        contactSoundPolicy(),
    );
}

fn computeEnvironmentCollisionDustModel(
    source_entity: *const entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
) ContactDustModel {
    const surface = terrain.getSurfaceAt(inst.pos_x, inst.pos_z);
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(source_entity.physics.material),
        source_entity.physics.material,
        surface,
        .solid,
    );
    return computeContactDustModel(
        .{
            .dust_type = response.dust_type,
            .penetration_depth = 1.0,
            .relative_normal_speed = @as(f32, @floatFromInt(@abs(impact_velocity))),
            .tangential_speed = 0.0,
            .friction = response.friction,
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
        },
        contactDustPolicy(),
    );
}

fn computeEnvironmentCollisionDeformationModel(
    source_entity: *const entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
) ContactDeformationModel {
    const surface = terrain.getSurfaceAt(inst.pos_x, inst.pos_z);
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(source_entity.physics.material),
        source_entity.physics.material,
        surface,
        .solid,
    );
    return computeContactDeformationModel(
        .{
            .penetration_depth = 1.0,
            .relative_normal_speed = @as(f32, @floatFromInt(@abs(impact_velocity))),
            .tangential_speed = 0.0,
            .penetration_resistance = response.penetration_resistance,
            .damage_modifier = response.damage_modifier,
            .restitution = response.restitution,
        },
        contactDeformationPolicy(),
    );
}

pub fn enqueueCollisionPairEvent(
    queue: *collision_event.PendingCollisionQueue,
    instances: []const scene32.Instance,
    entities: []entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    if (inst.entity_id >= entities.len) return;
    _ = enqueueCollisionEmitModel(
        queue,
        computeCollisionEmitModel(
            .{
                .entity_id = inst.entity_id,
                .impact_velocity = impact_velocity,
                .entity = &entities[inst.entity_id],
            },
            collisionEmitPolicy(),
        ),
    );

    if (blocker_id == 255 or blocker_id >= instances.len) return;
    const blocker = &instances[blocker_id];
    if (blocker.entity_id >= entities.len) return;
    _ = enqueueCollisionEmitModel(
        queue,
        computeCollisionEmitModel(
            .{
                .entity_id = blocker.entity_id,
                .impact_velocity = impact_velocity,
                .entity = &entities[blocker.entity_id],
            },
            collisionEmitPolicy(),
        ),
    );
}

pub fn enqueueSoundPairEvent(
    queue: *collision_event.PendingSoundQueue,
    instances: []const scene32.Instance,
    entities: []entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    if (inst.entity_id >= entities.len) return;
    if (blocker_id == 255) {
        const sound = computeEnvironmentCollisionSoundModel(&entities[inst.entity_id], inst, impact_velocity);
        _ = enqueueSoundEmitModel(
            queue,
            computeSoundEmitModel(
                .{
                    .entity_id = inst.entity_id,
                    .sound = sound,
                },
                soundEmitPolicy(),
            ),
        );
        return;
    }
    if (blocker_id >= instances.len) return;

    const blocker = &instances[blocker_id];
    if (blocker.entity_id >= entities.len) return;

    const sound = computeCollisionPairSoundModel(
        &entities[inst.entity_id],
        &entities[blocker.entity_id],
        impact_velocity,
    );
    _ = enqueueSoundEmitModel(
        queue,
        computeSoundEmitModel(
            .{
                .entity_id = inst.entity_id,
                .sound = sound,
            },
            soundEmitPolicy(),
        ),
    );
    _ = enqueueSoundEmitModel(
        queue,
        computeSoundEmitModel(
            .{
                .entity_id = blocker.entity_id,
                .sound = sound,
            },
            soundEmitPolicy(),
        ),
    );
}

pub fn enqueueParticlePairEvent(
    queue: *collision_event.PendingParticleQueue,
    instances: []const scene32.Instance,
    entities: []entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    if (inst.entity_id >= entities.len) return;
    if (blocker_id == 255) {
        const dust = computeEnvironmentCollisionDustModel(&entities[inst.entity_id], inst, impact_velocity);
        _ = enqueueParticleEmitModel(
            queue,
            computeParticleEmitModel(
                .{
                    .entity_id = inst.entity_id,
                    .dust = dust,
                },
                particleEmitPolicy(),
            ),
        );
        return;
    }
    if (blocker_id >= instances.len) return;

    const blocker = &instances[blocker_id];
    if (blocker.entity_id >= entities.len) return;

    const dust = computeCollisionPairDustModel(
        &entities[inst.entity_id],
        &entities[blocker.entity_id],
        impact_velocity,
    );
    _ = enqueueParticleEmitModel(
        queue,
        computeParticleEmitModel(
            .{
                .entity_id = inst.entity_id,
                .dust = dust,
            },
            particleEmitPolicy(),
        ),
    );
    _ = enqueueParticleEmitModel(
        queue,
        computeParticleEmitModel(
            .{
                .entity_id = blocker.entity_id,
                .dust = dust,
            },
            particleEmitPolicy(),
        ),
    );
}

pub fn enqueueDeformationPairEvent(
    queue: *collision_event.PendingDeformationQueue,
    instances: []const scene32.Instance,
    entities: []entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    if (inst.entity_id >= entities.len) return;
    if (blocker_id == 255) {
        const deformation = computeEnvironmentCollisionDeformationModel(&entities[inst.entity_id], inst, impact_velocity);
        _ = enqueueDeformationEmitModel(
            queue,
            computeDeformationEmitModel(
                .{
                    .entity_id = inst.entity_id,
                    .deformation = deformation,
                },
                deformationEmitPolicy(),
            ),
        );
        return;
    }
    if (blocker_id >= instances.len) return;

    const blocker = &instances[blocker_id];
    if (blocker.entity_id >= entities.len) return;

    const deformation = computeCollisionPairDeformationModel(
        &entities[inst.entity_id],
        &entities[blocker.entity_id],
        impact_velocity,
    );
    _ = enqueueDeformationEmitModel(
        queue,
        computeDeformationEmitModel(
            .{
                .entity_id = inst.entity_id,
                .deformation = deformation,
            },
            deformationEmitPolicy(),
        ),
    );
    _ = enqueueDeformationEmitModel(
        queue,
        computeDeformationEmitModel(
            .{
                .entity_id = blocker.entity_id,
                .deformation = deformation,
            },
            deformationEmitPolicy(),
        ),
    );
}

pub fn enqueueBreakPairEvent(
    queue: *collision_event.PendingBreakQueue,
    instances: []const scene32.Instance,
    entities: []entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    if (inst.entity_id >= entities.len) return;
    _ = enqueueBreakEmitModel(
        queue,
        computeBreakEmitModel(
            .{
                .entity_id = inst.entity_id,
                .impact_velocity = impact_velocity,
                .entity = &entities[inst.entity_id],
            },
            breakEmitPolicy(),
        ),
    );

    if (blocker_id == 255 or blocker_id >= instances.len) return;
    const blocker = &instances[blocker_id];
    if (blocker.entity_id >= entities.len) return;
    _ = enqueueBreakEmitModel(
        queue,
        computeBreakEmitModel(
            .{
                .entity_id = blocker.entity_id,
                .impact_velocity = impact_velocity,
                .entity = &entities[blocker.entity_id],
            },
            breakEmitPolicy(),
        ),
    );
}

pub fn applyVerticalContactResponse(inst: *scene32.Instance, entity: *const entity16.Entity16, blocker_id: u8, impact_velocity: i16) void {
    if (blocker_id != 255) {
        inst.vel_y = 0;
        contact_response.settleGroundContact(inst);
        return;
    }

    const surface_type = terrain.getSurfaceAt(inst.pos_x, inst.pos_z);
    contact_response.applyVerticalSurfaceContact(inst, entity, surface_type, impact_velocity);
}

pub fn handleBlockedFallContact(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    impact_velocity: i16,
) scene32.InstanceState {
    applyVerticalContactResponse(inst, entity, blocker_id, impact_velocity);
    return if (inst.vel_x == 0 and inst.vel_z == 0) .resting else .moving;
}

pub fn handleBlockedUpwardContact(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    impact_velocity: i16,
) void {
    _ = entity;
    _ = blocker_id;
    _ = impact_velocity;
    inst.vel_y = 0;
}

pub fn applyLateralContactResponse(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    axis: contact_response.ContactAxis,
) void {
    if (blocker_id != 255) {
        if (axis == .x) {
            inst.vel_x = 0;
        } else {
            inst.vel_z = 0;
        }
        return;
    }

    const surface_type = terrain.getSurfaceAt(inst.pos_x, inst.pos_z);
    contact_response.applyLateralSurfaceFriction(inst, entity, surface_type, axis);
}

pub fn handleBlockedLateralContact(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    blocker_id: u8,
    axis: contact_response.ContactAxis,
) void {
    applyLateralContactResponse(inst, entity, blocker_id, axis);
}

pub fn finalizeMotionState(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    next_x: i32,
    next_y: i32,
    next_z: i32,
    next_state: scene32.InstanceState,
    broke_this_tick: bool,
    sleep_time_threshold: u8,
) bool {
    var inst_moved = false;

    if (next_x != inst.pos_x or next_y != inst.pos_y or next_z != inst.pos_z) {
        inst.pos_x = next_x;
        inst.pos_y = next_y;
        inst.pos_z = next_z;
        if (!broke_this_tick) {
            inst.state = next_state;
        }
        inst_moved = true;
    } else if (!broke_this_tick and inst.vel_x == 0 and inst.vel_y == 0 and inst.vel_z == 0 and inst.state != .broken) {
        inst.state = .resting;
    }

    if (broke_this_tick) {
        return inst_moved;
    }

    if (wakeInstanceForMotion(inst, inst_moved)) {
        return inst_moved;
    } else if (shouldSleepInstance(inst, entity)) {
        if (inst.sleep_tick < sleep_time_threshold) {
            inst.sleep_tick += 1;
        }
        if (inst.sleep_tick >= sleep_time_threshold) {
            inst.state = .resting;
            inst.vel_x = 0;
            inst.vel_y = 0;
            inst.vel_z = 0;
            inst.ang_x = 0;
            inst.ang_y = 0;
            inst.ang_z = 0;
        }
    } else {
        inst.sleep_tick = 0;
    }

    return inst_moved;
}

pub fn sweepMotionAlongAxis(
    s1024: *scene1024.Scene1024,
    inst: *const scene32.Instance,
    entities: []entity16.Entity16,
    start_x: i32,
    start_y: i32,
    start_z: i32,
    vel: i16,
    axis: physics.SweepAxis,
) SweepMotionResult {
    const sweep = physics.sweepAxis(s1024, inst, entities, start_x, start_y, start_z, vel, axis);
    return .{
        .pos_x = sweep.pos_x,
        .pos_y = sweep.pos_y,
        .pos_z = sweep.pos_z,
        .blocked = sweep.blocked,
        .blocker_id = sweep.blocker_id,
    };
}

pub fn canOccupyTranslatedPosition(
    s1024: *scene1024.Scene1024,
    inst: *const scene32.Instance,
    entities: []entity16.Entity16,
    target_x: i32,
    target_y: i32,
    target_z: i32,
) bool {
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];

    for (0..64) |w_idx| {
        const word = entity.topology[w_idx];
        if (word == 0) continue;
        for (0..64) |b_idx| {
            if ((word & (@as(u64, 1) << @as(u6, @truncate(b_idx)))) == 0) continue;
            const idx = (w_idx << 6) | b_idx;
            const ex: i32 = @intCast((idx >> 4) & 0xF);
            const ey: i32 = @intCast(idx >> 8);
            const ez: i32 = @intCast(idx & 0xF);
            if (physics.isOccupiedGlobal(s1024, inst, entities, target_x + ex, target_y + ey, target_z + ez, null)) {
                return false;
            }
        }
    }

    return true;
}

pub fn invalidateBlockedTranslationIntents(
    intents: anytype,
    s1024: *scene1024.Scene1024,
    instances: []scene32.Instance,
    entities: []entity16.Entity16,
    nop_op: anytype,
    break_op: anytype,
) void {
    for (intents) |*intent| {
        if (intent.op == nop_op or intent.op == break_op) continue;
        if (intent.instance_idx >= instances.len) {
            intent.op = nop_op;
            continue;
        }

        const inst = &instances[intent.instance_idx];
        const nx = inst.pos_x + intent.dx;
        const ny = inst.pos_y + intent.dy;
        const nz = inst.pos_z + intent.dz;

        if (!canOccupyTranslatedPosition(s1024, inst, entities, nx, ny, nz)) {
            intent.op = nop_op;
        }
    }
}

pub fn appendClampedTranslationIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    op: anytype,
    dx: i32,
    dy: i32,
    dz: i32,
    priority: i16,
) bool {
    if (intent_count.* >= intents.len) return false;
    if (dx == 0 and dy == 0 and dz == 0) return false;

    intents[intent_count.*] = .{
        .instance_idx = instance_idx,
        .op = op,
        .dx = @intCast(@min(127, @max(-128, dx))),
        .dy = @intCast(@min(127, @max(-128, dy))),
        .dz = @intCast(@min(127, @max(-128, dz))),
        .priority = @intCast(priority),
    };
    intent_count.* += 1;
    return true;
}

pub fn appendUniqueInstanceIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    op: anytype,
    priority: u8,
) bool {
    var i: u8 = 0;
    while (i < intent_count.*) : (i += 1) {
        const existing = intents[i];
        if (existing.op == op and existing.instance_idx == instance_idx) return false;
    }
    if (intent_count.* >= intents.len) return false;

    intents[intent_count.*] = .{
        .instance_idx = instance_idx,
        .op = op,
        .priority = priority,
    };
    intent_count.* += 1;
    return true;
}

pub fn settleLowEnergyMotion(inst: *scene32.Instance, linear_threshold: i16) bool {
    if (@abs(inst.vel_x) >= linear_threshold or @abs(inst.vel_y) >= linear_threshold or @abs(inst.vel_z) >= linear_threshold) {
        return false;
    }

    inst.state = .resting;
    inst.vel_x = 0;
    inst.vel_y = 0;
    inst.vel_z = 0;
    return true;
}

pub fn markStaticResting(inst: *scene32.Instance) void {
    inst.state = .resting;
}

pub fn appendFallIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    current_y: i32,
    target_y: i32,
    op: anytype,
    priority: i16,
) bool {
    return appendClampedTranslationIntent(
        intents,
        intent_count,
        instance_idx,
        op,
        0,
        target_y - current_y,
        0,
        priority,
    );
}

pub fn appendFlowIntent(
    intents: anytype,
    intent_count: *u8,
    instance_idx: u8,
    current_x: i32,
    current_y: i32,
    current_z: i32,
    target_x: i32,
    target_y: i32,
    target_z: i32,
    op: anytype,
    priority: i16,
) bool {
    return appendClampedTranslationIntent(
        intents,
        intent_count,
        instance_idx,
        op,
        target_x - current_x,
        target_y - current_y,
        target_z - current_z,
        priority,
    );
}

pub fn applyFallDisplacement(inst: *scene32.Instance, dy: i8) bool {
    if (dy == 0) return false;
    inst.pos_y += dy;
    inst.state = if (inst.vel_y != 0) .falling else .resting;
    return true;
}

pub fn applyFlowDisplacement(inst: *scene32.Instance, dx: i8, dy: i8, dz: i8) bool {
    if (dx == 0 and dy == 0 and dz == 0) return false;
    inst.pos_x += dx;
    inst.pos_y += dy;
    inst.pos_z += dz;
    inst.state = .flowing;
    return true;
}

pub fn applyMoveDisplacement(inst: *scene32.Instance, dx: i8, dy: i8, dz: i8) bool {
    if (dx == 0 and dy == 0 and dz == 0) return false;
    inst.pos_x += dx;
    inst.pos_y += dy;
    inst.pos_z += dz;
    inst.state = .moving;
    return true;
}

pub fn applyContinuousPhysics(instances: []scene32.Instance, entities: []entity16.Entity16, time_scale: f32, sleep_time_threshold: u8) void {
    if (time_scale <= 0.0) return;

    var i: usize = 0;
    while (i < instances.len) : (i += 1) {
        const inst = &instances[i];
        if (inst.entity_id >= entities.len) continue;
        if (inst.state == .broken) continue;

        const entity = &entities[inst.entity_id];
        if ((entity.physics.flags & 0x01) != 0) continue;

        if (entity.physics.material != .liquid) {
            if (@abs(time_scale - 1.0) <= 0.0001) {
                physics.applyGravity(&inst.vel_y, entity.physics.mass);
            } else {
                const scaled_gravity = @as(i32, @intFromFloat(@round(@as(f32, @floatFromInt(physics.GRAVITY)) * time_scale)));
                inst.vel_y = physics.clampVerticalVelocity(@as(i32, inst.vel_y) + scaled_gravity);
            }
        }

        physics.applyDamping(&inst.vel_x, &inst.vel_y, &inst.vel_z);

        if (inst.ang_x != 0 or inst.ang_y != 0 or inst.ang_z != 0) {
            const yaw_delta: i16 = @intCast(@divTrunc(inst.ang_y, 10));
            const pitch_delta: i16 = @intCast(@divTrunc(inst.ang_x, 10));
            const roll_delta: i16 = @intCast(@divTrunc(inst.ang_z, 10));

            const new_yaw: i16 = @as(i16, @intCast(inst.rot_yaw)) + yaw_delta;
            const new_pitch: i16 = @as(i16, @intCast(inst.rot_pitch)) + pitch_delta;
            const new_roll: i16 = @as(i16, @intCast(inst.rot_roll)) + roll_delta;

            inst.rot_yaw = @intCast(@mod(new_yaw, 256));
            inst.rot_pitch = @intCast(@mod(new_pitch, 256));
            inst.rot_roll = @intCast(@mod(new_roll, 256));

            physics.applyAngularDamping(&inst.ang_x, &inst.ang_y, &inst.ang_z);
        }

        if (shouldSleepInstance(inst, entity)) {
            inst.sleep_tick += 1;
            if (inst.sleep_tick >= sleep_time_threshold) {
                inst.state = .resting;
                inst.vel_x = 0;
                inst.vel_y = 0;
                inst.vel_z = 0;
                inst.ang_x = 0;
                inst.ang_y = 0;
                inst.ang_z = 0;
            }
        } else {
            inst.sleep_tick = 0;
        }
    }
}

pub fn publishPendingCollisions(queue: *collision_event.PendingCollisionQueue, world_bus: ?*bus.Bus, tick: u32) void {
    const event_bus = world_bus orelse {
        queue.clear();
        return;
    };
    queue.publish(event_bus, tick);
}

pub fn publishPendingSounds(queue: *collision_event.PendingSoundQueue, world_bus: ?*bus.Bus, tick: u32) void {
    const event_bus = world_bus orelse {
        queue.clear();
        return;
    };
    queue.publish(event_bus, tick);
}

pub fn publishPendingParticles(queue: *collision_event.PendingParticleQueue, world_bus: ?*bus.Bus, tick: u32) void {
    const event_bus = world_bus orelse {
        queue.clear();
        return;
    };
    queue.publish(event_bus, tick);
}

pub fn publishPendingDeformations(queue: *collision_event.PendingDeformationQueue, world_bus: ?*bus.Bus, tick: u32) void {
    const event_bus = world_bus orelse {
        queue.clear();
        return;
    };
    queue.publish(event_bus, tick);
}

pub fn publishPendingBreaks(queue: *collision_event.PendingBreakQueue, world_bus: ?*bus.Bus, tick: u32) void {
    const event_bus = world_bus orelse {
        queue.clear();
        return;
    };
    queue.publish(event_bus, tick);
}

pub fn publishPendingJointBreaks(queue: *collision_event.PendingJointBreakQueue, world_bus: ?*bus.Bus, tick: u32) void {
    const event_bus = world_bus orelse {
        queue.clear();
        return;
    };
    queue.publish(event_bus, tick);
}

pub fn clearPendingCollisions(queue: *collision_event.PendingCollisionQueue) void {
    queue.clear();
}

pub fn clearPendingSounds(queue: *collision_event.PendingSoundQueue) void {
    queue.clear();
}

pub fn clearPendingParticles(queue: *collision_event.PendingParticleQueue) void {
    queue.clear();
}

pub fn clearPendingDeformations(queue: *collision_event.PendingDeformationQueue) void {
    queue.clear();
}

pub fn clearPendingBreaks(queue: *collision_event.PendingBreakQueue) void {
    queue.clear();
}

pub fn clearPendingJointBreaks(queue: *collision_event.PendingJointBreakQueue) void {
    queue.clear();
}

pub fn shouldSleep(inst: *scene32.Instance) bool {
    return sleep_response.shouldSleep(inst);
}

pub fn shouldSleepInstance(inst: *scene32.Instance, entity: *const entity16.Entity16) bool {
    return sleep_response.shouldSleepInstance(inst, entity);
}

pub fn computeSleepEnergy(inst: *const scene32.Instance, entity: *const entity16.Entity16) f32 {
    return sleep_response.computeSleepEnergy(inst, entity);
}

pub fn computeSleepStability(inst: *const scene32.Instance, entity: *const entity16.Entity16) u16 {
    return sleep_response.computeSleepStability(inst, entity);
}

pub fn isSleepStable(inst: *const scene32.Instance, entity: *const entity16.Entity16) bool {
    return sleep_response.isSleepStable(inst, entity);
}

pub fn detectSleepIslands(
    instances: []const scene32.Instance,
    entities: []const entity16.Entity16,
    pairs: []const BroadPhasePair,
    out_islands: []SleepIsland,
) usize {
    return sleep_response.detectSleepIslands(instances, entities, pairs, out_islands);
}

pub fn shouldWake(inst: *scene32.Instance, moved: bool) bool {
    return sleep_response.shouldWake(inst, moved);
}

pub fn wakeInstance(inst: *scene32.Instance) void {
    sleep_response.wakeInstance(inst);
}

pub fn wakeInstanceForMotion(inst: *scene32.Instance, moved: bool) bool {
    return sleep_response.wakeInstanceForMotion(inst, moved);
}

pub fn recordWorldSnapshot(tick: u32, s1024: *scene1024.Scene1024, entities: []entity16.Entity16) void {
    rewind.recordWorldSnapshot(tick, s1024, entities);
}

pub fn beginWorldStep(tick: *u32, s1024: *scene1024.Scene1024) void {
    tick.* += 1;
    s1024.global_tick = tick.*;
}

pub fn rebuildOccupancyIfNeeded(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, changed: bool) void {
    if (!changed) return;
    s1024.rebuildOccupancy(entities) catch {};
}

pub fn applyBreakStateAndWake(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    impact_velocity: i16,
    damage_scale: f32,
) bool {
    if (instance_idx >= s1024.instance_count) return false;
    const inst = &s1024.instances[instance_idx];
    if (inst.entity_id >= entities.len) return false;

    const entity = &entities[inst.entity_id];
    if (inst.state == .broken) return false;

    const damage_velocity = if (impact_velocity > 0) -impact_velocity else impact_velocity;
    const impact = computeDamageEvalImpactFromVelocity(entity, damage_velocity);
    const hardness = computeDamageEvalHardnessForEntity(entity, impact);
    const break_model = computeDamageEvalBreakForEntity(entity, impact, hardness);
    if (!break_model.did_break) return false;

    const fragments = computeDamageEvalFragmentsForEntity(
        entity,
        impact,
        break_model,
        impact_velocity,
        damage_scale,
    );
    _ = spawnDamageEvalDebrisFromFragments(inst, &fragments);

    inst.state = .broken;
    inst.vel_x = 0;
    inst.vel_y = 0;
    inst.vel_z = 0;
    inst.ang_x = 0;
    inst.ang_y = 0;
    inst.ang_z = 0;
    inst.sleep_tick = 0;
    sleep_response.wakeSupportedInstancesAfterBreak(
        s1024.instances[0..s1024.instance_count],
        entities,
        instance_idx,
    );
    return true;
}

pub fn computeBreakVelocity(inst: *const scene32.Instance) i16 {
    const break_velocity_i32 = @max(@as(i32, @abs(inst.vel_x)), @max(@as(i32, @abs(inst.vel_y)), @as(i32, @abs(inst.vel_z))));
    return @truncate(break_velocity_i32);
}

pub fn applyBreakFromInstanceAndWake(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    damage_scale: f32,
) bool {
    if (instance_idx >= s1024.instance_count) return false;
    const inst = &s1024.instances[instance_idx];
    const impact_velocity = computeBreakVelocity(inst);
    return applyBreakStateAndWake(s1024, entities, instance_idx, impact_velocity, damage_scale);
}

pub fn applyCollisionBreakPairAndWake(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    impact_velocity: i16,
    blocker_id: u8,
    damage_scale: f32,
) bool {
    var changed = applyBreakStateAndWake(s1024, entities, instance_idx, impact_velocity, damage_scale);
    if (blocker_id != 255 and blocker_id < s1024.instance_count) {
        changed = applyBreakStateAndWake(s1024, entities, blocker_id, impact_velocity, damage_scale) or changed;
    }
    return changed;
}

pub fn shouldBreakFromImpact(entities: []entity16.Entity16, inst: *const scene32.Instance, impact_velocity: i16) bool {
    if (inst.entity_id >= entities.len) return false;
    const entity = &entities[inst.entity_id];
    const impact = computeDamageEvalImpactFromVelocity(entity, impact_velocity);
    const hardness = computeDamageEvalHardnessForEntity(entity, impact);
    const break_model = computeDamageEvalBreakForEntity(entity, impact, hardness);
    return break_model.did_break;
}

pub fn evaluateImpactBreakPair(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    impact_velocity: i16,
    blocker_id: u8,
) ImpactBreakDecision {
    var decision: ImpactBreakDecision = .{
        .break_self = false,
        .break_blocker = false,
    };

    if (instance_idx < s1024.instance_count) {
        const inst = &s1024.instances[instance_idx];
        decision.break_self = shouldBreakFromImpact(entities, inst, impact_velocity);
    }

    if (blocker_id != 255 and blocker_id < s1024.instance_count) {
        const blocker = &s1024.instances[blocker_id];
        decision.break_blocker = shouldBreakFromImpact(entities, blocker, impact_velocity);
    }

    return decision;
}

pub fn updateKCCSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    kcc.updateSystem(s1024, entities, dt);
}

pub fn updateVehicleSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    vehicle.applyPredictiveVehicleAvoidance(dt);
    const vehicle_sys = vehicle.getSystem();
    var i: u8 = 0;

    // Bridge: AI traffic governed target speed → vehicle ai_target_speed
    const traffic_sys = ai_traffic.g_traffic_system;
    var ti: u16 = 0;
    while (ti < traffic_sys.vehicle_count) : (ti += 1) {
        const tv = &traffic_sys.vehicles[ti];
        if (tv.active and tv.vehicle_id > 0 and tv.vehicle_id - 1 < vehicle_sys.count) {
            vehicle_sys.vehicles[tv.vehicle_id - 1].ai_target_speed = tv.governed_target_vel;
        }
    }

    while (i < vehicle_sys.count) : (i += 1) {
        const v = &vehicle_sys.vehicles[i];
        vehicle.update(v, s1024, entities, dt);
    }
}

pub fn updateRagdollSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    const ragdoll_sys = ragdoll.getSystem();
    var i: u8 = 0;
    while (i < ragdoll_sys.count) : (i += 1) {
        const r = &ragdoll_sys.ragdolls[i];
        ragdoll.update(r, dt);
        ragdoll.solveJoints(r, s1024.instances[0..s1024.instance_count], entities);
    }
}

pub fn updateProjectileSystems(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, dt: f32) void {
    ballistics.simulateAll(s1024, entities, dt);
}

pub fn runPreStepSystems(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    time_scale: f32,
    sleep_time_threshold: u8,
    dt: f32,
) void {
    applyContinuousPhysics(s1024.instances[0..s1024.instance_count], entities, time_scale, sleep_time_threshold);
    updateKCCSystems(s1024, entities, dt);
    updateVehicleSystems(s1024, entities, dt);
    updateRagdollSystems(s1024, entities, dt);
    updateProjectileSystems(s1024, entities, dt);
}

pub fn solveJointConstraints(s1024: *scene1024.Scene1024, entities: []entity16.Entity16, joints: []joint.Joint) void {
    if (joints.len == 0) return;
    joint.solveJointsForTick(s1024.instances[0..s1024.instance_count], joints, entities);
}

fn instanceInverseMass(inst: *const scene32.Instance, entities: []entity16.Entity16) f32 {
    if (inst.entity_id >= entities.len) return 0.0;
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return 0.0;
    return 1.0 / @as(f32, @floatFromInt(entity.physics.mass));
}

fn computeJointMassData(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?JointMassData {
    const inv_mass_a = instanceInverseMass(inst_a, entities);
    const inv_mass_b = instanceInverseMass(inst_b, entities);
    const total_inv_mass = inv_mass_a + inv_mass_b;
    if (total_inv_mass <= 0.0001) return null;
    return .{
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .ratio_a = inv_mass_a / total_inv_mass,
        .ratio_b = inv_mass_b / total_inv_mass,
    };
}

fn jointDominantAxis(joint_def: *const joint.Joint) JointAxis {
    const abs_x = @abs(joint_def.axis_x);
    const abs_y = @abs(joint_def.axis_y);
    const abs_z = @abs(joint_def.axis_z);
    if (abs_x >= abs_y and abs_x >= abs_z) return .x;
    if (abs_y >= abs_z) return .y;
    return .z;
}

fn jointRotationByteToRadians(value: u8) f32 {
    const signed: i8 = @bitCast(value);
    return @as(f32, @floatFromInt(signed)) * (std.math.pi / 128.0);
}

fn jointRadiansToRotationByte(value: f32) u8 {
    const scaled = value * (128.0 / std.math.pi);
    const signed = @as(i8, @intFromFloat(@round(@max(-128.0, @min(127.0, scaled)))));
    return @bitCast(signed);
}

fn getKernelJointAngle(inst: *const scene32.Instance, joint_def: *const joint.Joint) f32 {
    return switch (jointDominantAxis(joint_def)) {
        .x => jointRotationByteToRadians(inst.rot_roll),
        .y => jointRotationByteToRadians(inst.rot_yaw),
        .z => jointRotationByteToRadians(inst.rot_pitch),
    };
}

fn setKernelJointAngle(inst: *scene32.Instance, joint_def: *const joint.Joint, angle: f32) void {
    switch (jointDominantAxis(joint_def)) {
        .x => inst.rot_roll = jointRadiansToRotationByte(angle),
        .y => inst.rot_yaw = jointRadiansToRotationByte(angle),
        .z => inst.rot_pitch = jointRadiansToRotationByte(angle),
    }
}

fn getKernelAngularVelocityOnAxis(inst: *const scene32.Instance, axis: JointAxis) f32 {
    return switch (axis) {
        .x => @as(f32, @floatFromInt(inst.ang_x)),
        .y => @as(f32, @floatFromInt(inst.ang_y)),
        .z => @as(f32, @floatFromInt(inst.ang_z)),
    };
}

fn applyKernelAngularVelocityDelta(inst: *scene32.Instance, axis: JointAxis, delta: i8) bool {
    if (delta == 0) return false;
    switch (axis) {
        .x => inst.ang_x += delta,
        .y => inst.ang_y += delta,
        .z => inst.ang_z += delta,
    }
    return true;
}

fn getKernelJointAxisVector(joint_def: *const joint.Joint) ?JointAxisVector {
    const ax = @as(f32, @floatFromInt(joint_def.axis_x));
    const ay = @as(f32, @floatFromInt(joint_def.axis_y));
    const az = @as(f32, @floatFromInt(joint_def.axis_z));
    const len = @sqrt(ax * ax + ay * ay + az * az);
    if (len <= 0.0001) return null;
    return .{ .x = ax / len, .y = ay / len, .z = az / len };
}

fn measureKernelJointAnchorDelta(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) JointAxisVector {
    return .{
        .x = @as(f32, @floatFromInt(inst_b.pos_x + joint_def.anchor_b_x - inst_a.pos_x - joint_def.anchor_a_x)),
        .y = @as(f32, @floatFromInt(inst_b.pos_y + joint_def.anchor_b_y - inst_a.pos_y - joint_def.anchor_a_y)),
        .z = @as(f32, @floatFromInt(inst_b.pos_z + joint_def.anchor_b_z - inst_a.pos_z - joint_def.anchor_a_z)),
    };
}

fn applyKernelPairDirectionalDisplacement(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
) bool {
    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        changed = applyKernelSingleDisplacement(inst_a, dir_x, dir_y, dir_z, magnitude * mass_data.ratio_a) or changed;
    }
    if (mass_data.inv_mass_b > 0.0) {
        changed = applyKernelSingleDisplacement(inst_b, -dir_x, -dir_y, -dir_z, magnitude * mass_data.ratio_b) or changed;
    }
    return changed;
}

fn applyKernelPairCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    corr_x: f32,
    corr_y: f32,
    corr_z: f32,
) bool {
    const magnitude = @sqrt(corr_x * corr_x + corr_y * corr_y + corr_z * corr_z);
    if (magnitude <= 0.0001) return false;
    return applyKernelPairDirectionalDisplacement(
        inst_a,
        inst_b,
        mass_data,
        corr_x / magnitude,
        corr_y / magnitude,
        corr_z / magnitude,
        magnitude,
    );
}

fn signedCorrection(value: f32, warm_impulse: f32) f32 {
    return value + if (value >= 0.0) warm_impulse else -warm_impulse;
}

fn cappedWarmRatio(warm_impulse: f32, magnitude: f32) f32 {
    if (warm_impulse <= 0.0) return 0.0;
    return @min(1.0, warm_impulse / @max(@abs(magnitude), 0.0001));
}

fn measureKernelRelativeLinearSpeed(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
) f32 {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    return rel_vel_x * dir_x + rel_vel_y * dir_y + rel_vel_z * dir_z;
}

fn lengthAndNormal(vector: JointAxisVector) ?struct { len: f32, nx: f32, ny: f32, nz: f32 } {
    const len = @sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
    if (len <= 0.0001) return null;
    return .{
        .len = len,
        .nx = vector.x / len,
        .ny = vector.y / len,
        .nz = vector.z / len,
    };
}

fn projectJointDeltaToAxis(delta: JointAxisVector, axis: JointAxisVector) JointAxisProjection {
    const along = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
    const perp_x = delta.x - axis.x * along;
    const perp_y = delta.y - axis.y * along;
    const perp_z = delta.z - axis.z * along;
    return .{
        .along = along,
        .perp_x = perp_x,
        .perp_y = perp_y,
        .perp_z = perp_z,
        .perp_len = @sqrt(perp_x * perp_x + perp_y * perp_y + perp_z * perp_z),
    };
}

fn prepareJointAnchorLinearConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointLinearConstraint {
    const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
    return switch (joint_def.joint_type) {
        .fixed, .ball_socket, .hinge => if (lengthAndNormal(delta)) |distance|
            .{
                .dir_x = distance.nx,
                .dir_y = distance.ny,
                .dir_z = distance.nz,
                .magnitude = distance.len,
            }
        else
            null,
        .slider => {
            const axis = getKernelJointAxisVector(joint_def) orelse return null;
            const projection = projectJointDeltaToAxis(delta, axis);
            if (projection.perp_len <= 0.0001) return null;
            return .{
                .dir_x = projection.perp_x / projection.perp_len,
                .dir_y = projection.perp_y / projection.perp_len,
                .dir_z = projection.perp_z / projection.perp_len,
                .magnitude = projection.perp_len,
            };
        },
        .spring, .pulley => null,
    };
}

fn prepareJointSpringLinearConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointLinearConstraint {
    const SpringDirection = struct {
        x: f32,
        y: f32,
        z: f32,
        len: f32,
    };
    const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
    const rest_length = @max(0.0, joint_def.limit_max);
    const direction: SpringDirection = if (lengthAndNormal(delta)) |distance|
        .{ .x = distance.nx, .y = distance.ny, .z = distance.nz, .len = distance.len }
    else blk: {
        if (rest_length <= 0.001) return null;
        const axis = getKernelJointAxisVector(joint_def) orelse return null;
        break :blk .{ .x = axis.x, .y = axis.y, .z = axis.z, .len = 0.0 };
    };

    const extension = direction.len - rest_length;
    const stiffness = @max(0.0, joint_def.stiffness) * 0.01;
    const rel_speed = measureKernelRelativeLinearSpeed(inst_a, inst_b, direction.x, direction.y, direction.z);
    const prediction_dt = kernelPredictionDt();
    const predicted_extension = extension + rel_speed * prediction_dt;
    const control_extension = if (extension * predicted_extension < 0.0)
        predicted_extension
    else if (@abs(predicted_extension) < @abs(extension))
        predicted_extension
    else
        extension;
    const requested = control_extension * stiffness;
    const max_correction = @abs(control_extension);
    return .{
        .dir_x = direction.x,
        .dir_y = direction.y,
        .dir_z = direction.z,
        .magnitude = @max(-max_correction, @min(max_correction, requested)),
    };
}

fn prepareJointSliderLimitLinearConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointLinearConstraint {
    const axis = getKernelJointAxisVector(joint_def) orelse return null;
    const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
    const projection = projectJointDeltaToAxis(delta, axis);
    const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
    const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
    const clamped_along = @max(min_limit, @min(max_limit, projection.along));
    return .{
        .dir_x = axis.x,
        .dir_y = axis.y,
        .dir_z = axis.z,
        .magnitude = projection.along - clamped_along,
    };
}

fn kernelPulleyRatio(joint_def: *const joint.Joint) f32 {
    const configured = @abs(joint_def.limit_min);
    return if (configured > 0.0001) configured else 1.0;
}

fn measureKernelPulleyCoordinate(
    inst: *const scene32.Instance,
    anchor_x: i32,
    anchor_y: i32,
    anchor_z: i32,
    axis: JointAxisVector,
) f32 {
    return @as(f32, @floatFromInt(inst.pos_x + anchor_x)) * axis.x +
        @as(f32, @floatFromInt(inst.pos_y + anchor_y)) * axis.y +
        @as(f32, @floatFromInt(inst.pos_z + anchor_z)) * axis.z;
}

fn measureKernelPulleyVelocity(inst: *const scene32.Instance, axis: JointAxisVector) f32 {
    return @as(f32, @floatFromInt(inst.vel_x)) * axis.x +
        @as(f32, @floatFromInt(inst.vel_y)) * axis.y +
        @as(f32, @floatFromInt(inst.vel_z)) * axis.z;
}

fn prepareJointPulleyConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointPulleyConstraint {
    const axis = getKernelJointAxisVector(joint_def) orelse return null;
    const ratio = kernelPulleyRatio(joint_def);
    const coord_a = measureKernelPulleyCoordinate(inst_a, joint_def.anchor_a_x, joint_def.anchor_a_y, joint_def.anchor_a_z, axis);
    const coord_b = measureKernelPulleyCoordinate(inst_b, joint_def.anchor_b_x, joint_def.anchor_b_y, joint_def.anchor_b_z, axis);
    const speed_a = measureKernelPulleyVelocity(inst_a, axis);
    const speed_b = measureKernelPulleyVelocity(inst_b, axis);
    const pulley_error = coord_a + ratio * coord_b - joint_def.limit_max;
    const constraint_speed = speed_a + ratio * speed_b;
    if (@abs(pulley_error) <= 0.0001 and @abs(constraint_speed) <= 0.0001) return null;
    return .{
        .axis = axis,
        .ratio = ratio,
        .pulley_error = pulley_error,
        .constraint_speed = constraint_speed,
        .damping_scale = @max(0.0, joint_def.damping) * 0.001,
    };
}

fn prepareJointHingeLimitAngularConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointAngularConstraint {
    if (joint_def.joint_type != .hinge) return null;
    const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
    const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
    const angle_a = getKernelJointAngle(inst_a, joint_def);
    const angle_b = getKernelJointAngle(inst_b, joint_def);
    const relative_angle = angle_b - angle_a;
    const clamped_relative = @max(min_angle, @min(max_angle, relative_angle));
    const angle_error = relative_angle - clamped_relative;
    if (@abs(angle_error) <= 0.0001) return null;
    return .{
        .angle_a = angle_a,
        .angle_b = angle_b,
        .magnitude = angle_error,
    };
}

fn prepareJointFixedAngularConstraintForAxis(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
    axis: JointAxis,
) ?PreparedJointAngularConstraint {
    if (joint_def.joint_type != .fixed) return null;
    const angle_a = switch (axis) {
        .x => jointRotationByteToRadians(inst_a.rot_roll),
        .y => jointRotationByteToRadians(inst_a.rot_yaw),
        .z => jointRotationByteToRadians(inst_a.rot_pitch),
    };
    const angle_b = switch (axis) {
        .x => jointRotationByteToRadians(inst_b.rot_roll),
        .y => jointRotationByteToRadians(inst_b.rot_yaw),
        .z => jointRotationByteToRadians(inst_b.rot_pitch),
    };
    const angle_error = angle_b - angle_a;
    const relative_velocity = getKernelAngularVelocityOnAxis(inst_b, axis) - getKernelAngularVelocityOnAxis(inst_a, axis);
    if (@abs(angle_error) <= 0.0001 and @abs(relative_velocity) <= 0.0001) return null;
    return .{
        .angle_a = angle_a,
        .angle_b = angle_b,
        .magnitude = angle_error,
    };
}

fn prepareJointFixedStrongestAngularConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?struct { axis: JointAxis, constraint: PreparedJointAngularConstraint } {
    var best_axis = JointAxis.x;
    var best_constraint: ?PreparedJointAngularConstraint = null;
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        if (prepareJointFixedAngularConstraintForAxis(inst_a, inst_b, joint_def, axis)) |constraint| {
            if (best_constraint == null or @abs(constraint.magnitude) > @abs(best_constraint.?.magnitude)) {
                best_axis = axis;
                best_constraint = constraint;
            }
        }
    }
    return if (best_constraint) |constraint| .{ .axis = best_axis, .constraint = constraint } else null;
}

const JointHingeAngularSelection = struct {
    axis: JointAxis,
    constraint: PreparedJointAngularConstraint,
    postprocess: JointLimitPostprocess,
    score: f32,
};

const JointAngularSelection = struct {
    axis: JointAxis,
    constraint: PreparedJointAngularConstraint,
    postprocess: JointLimitPostprocess,
    score: f32,
};

fn prepareJointLockedAngularConstraintForAxis(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    axis: JointAxis,
) ?JointAngularSelection {
    const angle_a = switch (axis) {
        .x => jointRotationByteToRadians(inst_a.rot_roll),
        .y => jointRotationByteToRadians(inst_a.rot_yaw),
        .z => jointRotationByteToRadians(inst_a.rot_pitch),
    };
    const angle_b = switch (axis) {
        .x => jointRotationByteToRadians(inst_b.rot_roll),
        .y => jointRotationByteToRadians(inst_b.rot_yaw),
        .z => jointRotationByteToRadians(inst_b.rot_pitch),
    };
    const angle_error = angle_b - angle_a;
    const relative_velocity = getKernelAngularVelocityOnAxis(inst_b, axis) - getKernelAngularVelocityOnAxis(inst_a, axis);
    const score = @max(@abs(angle_error), @abs(relative_velocity) * 0.25);
    if (score <= 0.0001) return null;

    return .{
        .axis = axis,
        .constraint = .{
            .angle_a = angle_a,
            .angle_b = angle_b,
            .magnitude = angle_error,
        },
        .postprocess = .{ .angular_velocity_damping = axis },
        .score = score,
    };
}

fn prepareJointLockedStrongestAngularConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
) ?JointAngularSelection {
    var best: ?JointAngularSelection = null;
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        if (prepareJointLockedAngularConstraintForAxis(inst_a, inst_b, axis)) |candidate| {
            if (best == null or candidate.score > best.?.score) {
                best = candidate;
            }
        }
    }
    return best;
}

fn prepareJointHingeSwingAngularConstraintForAxis(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
    axis: JointAxis,
) ?JointHingeAngularSelection {
    if (joint_def.joint_type != .hinge) return null;
    if (axis == jointDominantAxis(joint_def)) return null;

    const angle_a = switch (axis) {
        .x => jointRotationByteToRadians(inst_a.rot_roll),
        .y => jointRotationByteToRadians(inst_a.rot_yaw),
        .z => jointRotationByteToRadians(inst_a.rot_pitch),
    };
    const angle_b = switch (axis) {
        .x => jointRotationByteToRadians(inst_b.rot_roll),
        .y => jointRotationByteToRadians(inst_b.rot_yaw),
        .z => jointRotationByteToRadians(inst_b.rot_pitch),
    };
    const angle_error = angle_b - angle_a;
    const relative_velocity = getKernelAngularVelocityOnAxis(inst_b, axis) - getKernelAngularVelocityOnAxis(inst_a, axis);
    const score = @max(@abs(angle_error), @abs(relative_velocity) * 0.25);
    if (score <= 0.0001) return null;

    return .{
        .axis = axis,
        .constraint = .{
            .angle_a = angle_a,
            .angle_b = angle_b,
            .magnitude = angle_error,
        },
        .postprocess = .{ .angular_velocity_damping = axis },
        .score = score,
    };
}

fn prepareJointHingeStrongestAngularConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?JointHingeAngularSelection {
    var best: ?JointHingeAngularSelection = null;
    if (prepareJointHingeLimitAngularConstraint(inst_a, inst_b, joint_def)) |constraint| {
        best = .{
            .axis = jointDominantAxis(joint_def),
            .constraint = constraint,
            .postprocess = .none,
            .score = @abs(constraint.magnitude),
        };
    }

    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        if (prepareJointHingeSwingAngularConstraintForAxis(inst_a, inst_b, joint_def, axis)) |candidate| {
            if (best == null or candidate.score > best.?.score) {
                best = candidate;
            }
        }
    }
    return best;
}

fn prepareJointSliderStrongestLimitConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) JointPreparedOutcome(JointPreparedLimitChannel) {
    const linear = prepareJointSliderLimitLinearConstraint(inst_a, inst_b, joint_def);
    const angular = prepareJointLockedStrongestAngularConstraint(inst_a, inst_b);
    const linear_score = if (linear) |prepared| @abs(prepared.magnitude) else 0.0;
    const angular_score = if (angular) |prepared| prepared.score else 0.0;

    if (angular_score > linear_score) {
        const prepared = angular.?;
        return readyJointPreparedLimitAngularChannelWithPostprocess(
            prepared.axis,
            prepared.constraint,
            prepared.postprocess,
        );
    }
    if (linear) |prepared| {
        return readyJointPreparedLimitLinearChannel(jointAxisVectorFromPreparedLinearConstraint(prepared), prepared);
    }
    return .stalled_prepare;
}

fn prepareJointDriveConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointDriveConstraint {
    if (!joint_def.motor_enabled or joint_def.motor_speed <= 0.0) return null;
    if (joint_def.joint_type != .hinge and joint_def.joint_type != .slider) return null;

    const drive_state = measureKernelJointDriveState(inst_a, inst_b, joint_def) orelse return null;
    const min_step: f32 = switch (joint_def.joint_type) {
        .hinge => 0.001,
        .slider => if (@abs(drive_state.position - joint_def.motor_target) >= 1.0) 1.0 else 0.001,
        else => 0.001,
    };
    const drive_plan = computeKernelJointDrivePlan(joint_def, drive_state, min_step) orelse return null;
    return .{
        .signed_step = drive_plan.signed_step,
        .desired_velocity = drive_plan.desired_velocity,
    };
}

fn prepareJointHingeDriveAngularConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointAngularDriveConstraint {
    if (joint_def.joint_type != .hinge) return null;
    const drive = prepareJointDriveConstraint(inst_a, inst_b, joint_def) orelse return null;
    return .{
        .angle_a = getKernelJointAngle(inst_a, joint_def),
        .angle_b = getKernelJointAngle(inst_b, joint_def),
        .signed_step = drive.signed_step,
        .desired_velocity = drive.desired_velocity,
    };
}

fn prepareJointLimitChannel(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) JointPreparedOutcome(JointPreparedLimitChannel) {
    return switch (joint_def.joint_type) {
        .fixed => if (prepareJointFixedStrongestAngularConstraint(inst_a, inst_b, joint_def)) |prepared|
            readyJointPreparedLimitAngularChannelWithPostprocess(
                prepared.axis,
                prepared.constraint,
                .{ .angular_velocity_damping = prepared.axis },
            )
        else
            .stalled_prepare,
        .hinge => if (prepareJointHingeStrongestAngularConstraint(inst_a, inst_b, joint_def)) |prepared|
            readyJointPreparedLimitAngularChannelWithPostprocess(
                prepared.axis,
                prepared.constraint,
                prepared.postprocess,
            )
        else
            .stalled_prepare,
        .slider => prepareJointSliderStrongestLimitConstraint(inst_a, inst_b, joint_def),
        .pulley => if (prepareJointPulleyConstraint(inst_a, inst_b, joint_def)) |prepared|
            JointPreparedOutcome(JointPreparedLimitChannel){ .ready = .{ .pulley = prepared } }
        else
            .stalled_prepare,
        else => .inactive,
    };
}

fn prepareJointAnchorChannel(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) JointPreparedOutcome(JointPreparedAnchorChannel) {
    return switch (joint_def.joint_type) {
        .spring => if (prepareJointSpringLinearConstraint(inst_a, inst_b, joint_def)) |prepared|
            readyJointPreparedAnchorChannel(
                jointAxisVectorFromPreparedLinearConstraint(prepared),
                prepared,
                @max(0.0, joint_def.damping) * 0.001,
            )
        else
            .stalled_prepare,
        .fixed, .ball_socket, .hinge, .slider => if (prepareJointAnchorLinearConstraint(inst_a, inst_b, joint_def)) |prepared|
            readyJointPreparedAnchorChannel(
                jointAxisVectorFromPreparedLinearConstraint(prepared),
                prepared,
                @max(0.05, @min(1.0, @max(0.0, joint_def.damping) * 0.001)),
            )
        else
            .stalled_prepare,
        .pulley => .inactive,
    };
}

fn prepareJointDriveChannel(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) JointPreparedOutcome(JointPreparedDriveChannel) {
    return switch (joint_def.joint_type) {
        .hinge => if (prepareJointHingeDriveAngularConstraint(inst_a, inst_b, joint_def)) |prepared|
            readyJointPreparedDriveAngularChannel(jointDominantAxis(joint_def), prepared)
        else
            .stalled_prepare,
        .slider => {
            const axis = getKernelJointAxisVector(joint_def) orelse return .stalled_prepare;
            const prepared = prepareJointDriveConstraint(inst_a, inst_b, joint_def) orelse return .stalled_prepare;
            return readyJointPreparedDriveLinearChannel(axis, prepared);
        },
        else => .inactive,
    };
}

fn jointAxisVectorFromPreparedLinearConstraint(prepared: PreparedJointLinearConstraint) JointAxisVector {
    return .{
        .x = prepared.dir_x,
        .y = prepared.dir_y,
        .z = prepared.dir_z,
    };
}

fn readyJointPreparedLimitAngularChannel(
    axis: JointAxis,
    prepared: PreparedJointAngularConstraint,
) JointPreparedOutcome(JointPreparedLimitChannel) {
    return readyJointPreparedLimitAngularChannelWithPostprocess(axis, prepared, .none);
}

fn readyJointPreparedLimitAngularChannelWithPostprocess(
    axis: JointAxis,
    prepared: PreparedJointAngularConstraint,
    postprocess: JointLimitPostprocess,
) JointPreparedOutcome(JointPreparedLimitChannel) {
    return .{ .ready = .{ .angular = .{
        .axis = axis,
        .constraint = prepared,
        .postprocess = postprocess,
    } } };
}

fn readyJointPreparedLimitLinearChannel(
    axis: JointAxisVector,
    prepared: PreparedJointLinearConstraint,
) JointPreparedOutcome(JointPreparedLimitChannel) {
    return .{ .ready = .{ .linear = .{
        .axis = axis,
        .constraint = prepared,
    } } };
}

fn readyJointPreparedDriveAngularChannel(
    axis: JointAxis,
    prepared: PreparedJointAngularDriveConstraint,
) JointPreparedOutcome(JointPreparedDriveChannel) {
    return .{ .ready = .{ .angular = .{
        .axis = axis,
        .constraint = prepared,
    } } };
}

fn readyJointPreparedDriveLinearChannel(
    axis: JointAxisVector,
    prepared: PreparedJointDriveConstraint,
) JointPreparedOutcome(JointPreparedDriveChannel) {
    return .{ .ready = .{ .linear = .{
        .axis = axis,
        .constraint = prepared,
    } } };
}

fn readyJointPreparedAnchorChannel(
    axis: JointAxisVector,
    prepared: PreparedJointLinearConstraint,
    damping_scale: f32,
) JointPreparedOutcome(JointPreparedAnchorChannel) {
    return .{ .ready = .{
        .axis = axis,
        .constraint = prepared,
        .damping_scale = damping_scale,
    } };
}

fn mapJointPreparedOutcome(
    comptime In: type,
    comptime Out: type,
    ctx: JointSolveRuntimeContext,
    outcome: JointPreparedOutcome(In),
    map_ready: MapJointPreparedOutcomeFn(In, Out),
) JointPreparedOutcome(Out) {
    return switch (outcome) {
        .ready => |prepared| .{ .ready = map_ready(ctx, prepared) },
        .inactive => .inactive,
        .stalled_prepare => .stalled_prepare,
    };
}

fn prepareMappedJointDescriptorChannel(
    comptime T: type,
    ctx: JointSolveRuntimeContext,
    outcome: JointPreparedOutcome(T),
    map_ready: MapJointPreparedOutcomeFn(T, JointPreparedChannel),
) JointPreparedOutcome(JointPreparedChannel) {
    return mapJointPreparedOutcome(
        T,
        JointPreparedChannel,
        ctx,
        outcome,
        map_ready,
    );
}

fn mapJointDescriptorLimitChannel(
    ctx: JointSolveRuntimeContext,
    prepared: JointPreparedLimitChannel,
) JointPreparedChannel {
    _ = ctx;
    return .{ .limit = prepared };
}

fn prepareJointDescriptorLimitChannel(ctx: JointSolveRuntimeContext) JointPreparedOutcome(JointPreparedChannel) {
    return prepareMappedJointDescriptorChannel(
        JointPreparedLimitChannel,
        ctx,
        prepareJointLimitChannel(ctx.inst_a, ctx.inst_b, ctx.joint_def),
        mapJointDescriptorLimitChannel,
    );
}

fn prepareJointDescriptorAnchorPostprocess(ctx: JointSolveRuntimeContext) JointAnchorPostprocess {
    return switch (ctx.descriptor.policy.anchor_postprocess_policy) {
        .none => .none,
        .by_joint_type => prepareJointAnchorPostprocess(ctx.joint_def),
    };
}

fn withPreparedJointAnchorPostprocess(
    prepared: JointPreparedAnchorChannel,
    postprocess: JointAnchorPostprocess,
) JointPreparedAnchorChannel {
    var result = prepared;
    result.postprocess = postprocess;
    return result;
}

fn mapJointDescriptorAnchorChannel(
    ctx: JointSolveRuntimeContext,
    prepared: JointPreparedAnchorChannel,
) JointPreparedChannel {
    return .{ .anchor = withPreparedJointAnchorPostprocess(prepared, prepareJointDescriptorAnchorPostprocess(ctx)) };
}

fn prepareJointDescriptorAnchorChannel(ctx: JointSolveRuntimeContext) JointPreparedOutcome(JointPreparedChannel) {
    return prepareMappedJointDescriptorChannel(
        JointPreparedAnchorChannel,
        ctx,
        prepareJointAnchorChannel(ctx.inst_a, ctx.inst_b, ctx.joint_def),
        mapJointDescriptorAnchorChannel,
    );
}

fn prepareJointDescriptorDrivePostprocessPolicy(
    ctx: JointSolveRuntimeContext,
) JointPreparedDrivePostprocess {
    return switch (ctx.descriptor.policy.drive_postprocess_policy) {
        .none => .none,
        .velocity_bias => .velocity_bias,
    };
}

fn withPreparedJointDrivePostprocess(
    prepared: JointPreparedDriveChannel,
    postprocess: JointPreparedDrivePostprocess,
) JointPreparedDriveChannel {
    var result = prepared;
    switch (result) {
        .angular => |*angular| angular.postprocess = postprocess,
        .linear => |*linear| linear.postprocess = postprocess,
    }
    return result;
}

fn mapJointDescriptorDriveChannel(
    ctx: JointSolveRuntimeContext,
    prepared: JointPreparedDriveChannel,
) JointPreparedChannel {
    return .{ .drive = withPreparedJointDrivePostprocess(prepared, prepareJointDescriptorDrivePostprocessPolicy(ctx)) };
}

fn prepareJointDescriptorDriveChannel(ctx: JointSolveRuntimeContext) JointPreparedOutcome(JointPreparedChannel) {
    return prepareMappedJointDescriptorChannel(
        JointPreparedDriveChannel,
        ctx,
        prepareJointDriveChannel(ctx.inst_a, ctx.inst_b, ctx.joint_def),
        mapJointDescriptorDriveChannel,
    );
}

fn prepareJointAnchorPostprocess(joint_def: *const joint.Joint) JointAnchorPostprocess {
    return switch (joint_def.joint_type) {
        .hinge => .hinge_limit_damping,
        .slider => if (getKernelJointAxisVector(joint_def)) |axis|
            .{ .slider_limit_damping = axis }
        else
            .none,
        else => .none,
    };
}

fn clampKernelI8FromF32(value: f32) i8 {
    const clamped = @max(-128.0, @min(127.0, value));
    if (clamped > 0.0 and clamped < 1.0) return 1;
    if (clamped < 0.0 and clamped > -1.0) return -1;
    return @as(i8, @intFromFloat(@round(clamped)));
}

fn clampKernelI16FromF32(value: f32) i16 {
    const clamped = @max(-32768.0, @min(32767.0, value));
    if (clamped > 0.0 and clamped < 1.0) return 1;
    if (clamped < 0.0 and clamped > -1.0) return -1;
    return @as(i16, @intFromFloat(@round(clamped)));
}

fn applyKernelSingleDisplacement(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
) bool {
    if (magnitude == 0.0) return false;
    const dx = @as(i32, @intFromFloat(@round(dir_x * magnitude)));
    const dy = @as(i32, @intFromFloat(@round(dir_y * magnitude)));
    const dz = @as(i32, @intFromFloat(@round(dir_z * magnitude)));
    if (dx == 0 and dy == 0 and dz == 0) return false;
    inst.pos_x += dx;
    inst.pos_y += dy;
    inst.pos_z += dz;
    return true;
}

fn applyKernelLinearVelocityImpulsePair(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    signed_impulse: f32,
) bool {
    var changed = false;

    if (inv_mass_a > 0.0) {
        const delta_ax = clampKernelI16FromF32(-dir_x * signed_impulse * inv_mass_a);
        const delta_ay = clampKernelI16FromF32(-dir_y * signed_impulse * inv_mass_a);
        const delta_az = clampKernelI16FromF32(-dir_z * signed_impulse * inv_mass_a);
        if (delta_ax != 0 or delta_ay != 0 or delta_az != 0) {
            inst_a.vel_x += delta_ax;
            inst_a.vel_y += delta_ay;
            inst_a.vel_z += delta_az;
            changed = true;
        }
    }

    if (inv_mass_b > 0.0) {
        const delta_bx = clampKernelI16FromF32(dir_x * signed_impulse * inv_mass_b);
        const delta_by = clampKernelI16FromF32(dir_y * signed_impulse * inv_mass_b);
        const delta_bz = clampKernelI16FromF32(dir_z * signed_impulse * inv_mass_b);
        if (delta_bx != 0 or delta_by != 0 or delta_bz != 0) {
            inst_b.vel_x += delta_bx;
            inst_b.vel_y += delta_by;
            inst_b.vel_z += delta_bz;
            changed = true;
        }
    }

    return changed;
}

fn applyKernelLinearDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    relative_speed: f32,
    damping_scale: f32,
) bool {
    if (@abs(relative_speed) <= 0.0001 or damping_scale <= 0.0) return false;

    const damping_impulse = relative_speed * damping_scale;
    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        const dvx = clampKernelI16FromF32(dir_x * damping_impulse * mass_data.ratio_a);
        const dvy = clampKernelI16FromF32(dir_y * damping_impulse * mass_data.ratio_a);
        const dvz = clampKernelI16FromF32(dir_z * damping_impulse * mass_data.ratio_a);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_a.vel_x += dvx;
            inst_a.vel_y += dvy;
            inst_a.vel_z += dvz;
            changed = true;
        }
    }
    if (mass_data.inv_mass_b > 0.0) {
        const dvx = clampKernelI16FromF32(dir_x * damping_impulse * mass_data.ratio_b);
        const dvy = clampKernelI16FromF32(dir_y * damping_impulse * mass_data.ratio_b);
        const dvz = clampKernelI16FromF32(dir_z * damping_impulse * mass_data.ratio_b);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_b.vel_x -= dvx;
            inst_b.vel_y -= dvy;
            inst_b.vel_z -= dvz;
            changed = true;
        }
    }
    return changed;
}

fn wakeJointPair(inst_a: *scene32.Instance, inst_b: *scene32.Instance, changed: bool) void {
    if (!changed) return;
    wakeInstance(inst_a);
    wakeInstance(inst_b);
}

fn wakeSingleInstance(inst: *scene32.Instance, changed: bool) void {
    if (!changed) return;
    wakeInstance(inst);
}

const PairRowFinalizeContext = struct {
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
};

fn finalizePairRowResult(
    measure_after: f32,
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    before: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    wakeJointPair(inst_a, inst_b, changed);
    return finalizeConstraintRowResult(before, measure_after, changed, applied_impulse, equation);
}

fn finalizeRowResultFromSolveStep(
    comptime Context: type,
    ctx: Context,
    measure_after: f32,
    before: f32,
    solve_step: ConstraintSolveStep,
    equation: ConstraintRowEquation,
    finalize_row_result: *const fn (measure_after: f32, ctx: Context, before: f32, changed: bool, applied_impulse: f32, equation: ConstraintRowEquation) ConstraintRowExecResult,
) ConstraintRowExecResult {
    return finalize_row_result(
        measure_after,
        ctx,
        before,
        solve_step.changed,
        solve_step.applied_impulse,
        equation,
    );
}

fn finalizePairRowResultWithContext(
    measure_after: f32,
    ctx: PairRowFinalizeContext,
    before: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return finalizePairRowResult(measure_after, ctx.inst_a, ctx.inst_b, before, changed, applied_impulse, equation);
}

fn finalizePairSolveStepResult(
    measure_after: f32,
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    before: f32,
    solve_step: ConstraintSolveStep,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return finalizeRowResultFromSolveStep(
        PairRowFinalizeContext,
        .{ .inst_a = inst_a, .inst_b = inst_b },
        measure_after,
        before,
        solve_step,
        equation,
        finalizePairRowResultWithContext,
    );
}

fn finalizeSingleRowResult(
    measure_after: f32,
    inst: *scene32.Instance,
    before: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    wakeSingleInstance(inst, changed);
    return finalizeConstraintRowResult(before, measure_after, changed, applied_impulse, equation);
}

fn finalizeSingleRowResultWithContext(
    measure_after: f32,
    inst: *scene32.Instance,
    before: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return finalizeSingleRowResult(measure_after, inst, before, changed, applied_impulse, equation);
}

fn finalizeSingleSolveStepResult(
    measure_after: f32,
    inst: *scene32.Instance,
    before: f32,
    solve_step: ConstraintSolveStep,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return finalizeRowResultFromSolveStep(
        *scene32.Instance,
        inst,
        measure_after,
        before,
        solve_step,
        equation,
        finalizeSingleRowResultWithContext,
    );
}

fn settleAndWakeContactPair(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_y: f32,
    changed: bool,
) void {
    settleContactRestState(inst_a, inst_b, inv_mass_a, inv_mass_b, normal_y);
    wakeJointPair(inst_a, inst_b, changed);
}

fn finalizeJointRowNoChange(
    ctx: JointSolveRuntimeContext,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    const after = measureJointRowResidual(ctx.kind, instances, joints, joint_idx, entities);
    return finalizeConstraintRowResult(ctx.before, after, false, 0.0, equation);
}

fn initJointExecutionState(
    ctx: JointSolveRuntimeContext,
    row_state: ?*const ConstraintRowState,
    equation: ConstraintRowEquation,
) JointRowExecutionState {
    return JointRowExecutionState.init(
        constraintRowWarmImpulse(row_state, ctx.descriptor.policy.warm_impulse_scale, equation, 1.0),
    );
}

fn finalizeJointExecutionState(
    ctx: JointSolveRuntimeContext,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    exec_state: JointRowExecutionState,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    const after = measureJointRowResidual(ctx.kind, instances, joints, joint_idx, entities);
    return finalizePairSolveStepResult(after, ctx.inst_a, ctx.inst_b, ctx.before, exec_state.solveStep(), equation);
}

fn jointPreparedUnavailableOutcome(policy: JointPreparedUnavailablePolicy) JointSolveResultOutcome {
    return switch (policy) {
        .finalize_no_change => .no_change,
        .finalize_exec_state => .finalize_exec_state,
        .stalled => .stalled,
    };
}

fn resolveJointPreparedChannel(
    comptime T: type,
    ctx: JointSolveRuntimeContext,
    outcome: JointPreparedOutcome(T),
) JointPreparedResolution(T) {
    return switch (outcome) {
        .ready => |prepared| .{ .ready = prepared },
        .inactive => .{ .outcome = jointPreparedUnavailableOutcome(ctx.descriptor.policy.inactive_outcome) },
        .stalled_prepare => .{ .outcome = jointPreparedUnavailableOutcome(ctx.descriptor.policy.stalled_prepare_outcome) },
    };
}

fn prepareResolvedJointSolveResult(
    comptime T: type,
    ctx: JointSolveRuntimeContext,
    exec_state: *const JointRowExecutionState,
    row_state: ?*const ConstraintRowState,
    equation: ConstraintRowEquation,
    prepared_outcome: JointPreparedOutcome(T),
    build_ready: PrepareResolvedJointSolveResultFn(T),
) JointSolveResultOutcome {
    return switch (resolveJointPreparedChannel(T, ctx, prepared_outcome)) {
        .ready => |prepared| .{ .ready = build_ready(ctx, exec_state, row_state, equation, prepared) },
        .outcome => |outcome| outcome,
    };
}

fn buildReadyJointSolveResultFromPreparedChannel(
    ctx: JointSolveRuntimeContext,
    exec_state: *const JointRowExecutionState,
    row_state: ?*const ConstraintRowState,
    equation: ConstraintRowEquation,
    prepared: JointPreparedChannel,
) JointPreparedSolveResult {
    return switch (prepared) {
        .limit => |limit| applyPreparedJointLimitChannel(
            ctx.inst_a,
            ctx.inst_b,
            ctx.mass_data,
            limit,
            row_state,
            equation,
            ctx.descriptor.policy.signed_correction_policy,
            ctx.descriptor.policy.impulse_hint_policy,
        ),
        .anchor => |anchor| applyPreparedJointAnchorChannel(
            ctx.inst_a,
            ctx.inst_b,
            ctx.mass_data,
            anchor,
            exec_state.applied_impulse,
            equation,
        ),
        .drive => |drive| applyPreparedJointDriveChannel(
            ctx.inst_a,
            ctx.inst_b,
            ctx.mass_data,
            drive,
            row_state,
            equation,
            ctx.descriptor.policy.signed_correction_policy,
            ctx.descriptor.policy.impulse_hint_policy,
        ),
    };
}

fn prepareJointRuntimeSolveContext(
    kind: ConstraintRowKind,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    base_residual: f32,
) JointRuntimeSolveContextOutcome {
    const descriptor = jointRuntimeRowDescriptor(kind) orelse return .inactive;
    if (joint_idx >= joints.len) return .inactive;
    if (base_residual <= 0.0) return .inactive;

    const joint_def = &joints[joint_idx];
    if (!descriptor.row_enabled(joint_def)) return .inactive;
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return .inactive;

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return .inactive;

    const mass_data = computeJointMassData(inst_a, inst_b, entities) orelse return .stalled;

    return .{
        .ready = .{ .runtime = .{
            .kind = kind,
            .descriptor = descriptor,
            .joint_def = joint_def,
            .inst_a = inst_a,
            .inst_b = inst_b,
            .before = base_residual,
            .mass_data = mass_data,
        } },
    };
}

fn executePreparedJointRuntimeRow(
    ctx: JointRuntimeSolveContext,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    var exec_state = initJointExecutionState(ctx.runtime, row_state, equation);
    return switch (prepareResolvedJointSolveResult(
        JointPreparedChannel,
        ctx.runtime,
        &exec_state,
        row_state,
        equation,
        ctx.runtime.descriptor.prepare_channel(ctx.runtime),
        buildReadyJointSolveResultFromPreparedChannel,
    )) {
        .ready => |solve_result| blk: {
            solve_result.applyAll(&exec_state, ctx.runtime.inst_a, ctx.runtime.inst_b, ctx.runtime.joint_def, ctx.runtime.mass_data);
            break :blk finalizeJointExecutionState(
                ctx.runtime,
                instances,
                joints,
                joint_idx,
                entities,
                exec_state,
                equation,
            );
        },
        .no_change => finalizeJointRowNoChange(
            ctx.runtime,
            instances,
            joints,
            joint_idx,
            entities,
            equation,
        ),
        .finalize_exec_state => finalizeJointExecutionState(
            ctx.runtime,
            instances,
            joints,
            joint_idx,
            entities,
            exec_state,
            equation,
        ),
        .stalled => stalledConstraintRowResult(ctx.runtime.before),
    };
}

const ExecutePreparedJointRuntimeRowContext = struct {
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
};

fn runPreparedJointRuntimeRow(
    ctx: ExecutePreparedJointRuntimeRowContext,
    ready: JointRuntimeSolveContext,
) ConstraintRowExecResult {
    return executePreparedJointRuntimeRow(
        ready,
        ctx.instances,
        ctx.joints,
        ctx.joint_idx,
        ctx.entities,
        ctx.equation,
        ctx.row_state,
    );
}

fn executePreparedJointRuntimeOutcome(
    outcome: JointRuntimeSolveContextOutcome,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    return executePreparedRuntimeOutcome(
        JointRuntimeSolveContext,
        JointRuntimeSolveContextOutcome,
        ExecutePreparedJointRuntimeRowContext,
        outcome,
        .{
            .instances = instances,
            .joints = joints,
            .joint_idx = joint_idx,
            .entities = entities,
            .equation = equation,
            .row_state = row_state,
        },
        base_residual,
        runPreparedJointRuntimeRow,
    );
}

fn settleContactExecutionContext(
    ctx: ContactExecutionContext,
    changed: bool,
) void {
    settleAndWakeContactPair(
        ctx.inst_a,
        ctx.inst_b,
        ctx.prepared.inv_mass_a,
        ctx.prepared.inv_mass_b,
        ctx.prepared.normal.dir_y,
        changed,
    );
}

fn finishSettledContactPairChanged(
    settle_after_solve: bool,
    ctx: ContactExecutionContext,
    changed: bool,
) bool {
    if (settle_after_solve) {
        settleContactExecutionContext(ctx, changed);
    }
    return changed;
}

fn finishPreparedContactOutcome(
    comptime Context: type,
    comptime Result: type,
    ctx: Context,
    outcome: ContactPreparedPairOutcome,
    stalled_result: Result,
    finish_ready: *const fn (ctx: Context, pair_result: ContactPairSolveResult) Result,
) Result {
    return mapPreparedContactPairOutcome(
        Context,
        Result,
        ctx,
        outcome,
        stalled_result,
        finish_ready,
    );
}

fn finalizePreparedContactRowResult(
    kind: ConstraintRowKind,
    ctx: ContactExecutionContext,
    entities: []entity16.Entity16,
    before: f32,
    solve_step: ConstraintSolveStep,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    const after = measureContactRowResidual(kind, ctx.inst_a, ctx.inst_b, entities);
    return finalizePairSolveStepResult(after, ctx.inst_a, ctx.inst_b, before, solve_step, equation);
}

const FinishContactSolveResultOutcomeContext = struct {
    descriptor: *const ContactRuntimeRowDescriptor,
    kind: ConstraintRowKind,
    ctx: ContactExecutionContext,
    entities: []entity16.Entity16,
    before: f32,
    equation: ConstraintRowEquation,
};

fn finishPreparedContactSolveResultReady(
    ctx: FinishContactSolveResultOutcomeContext,
    pair_result: ContactPairSolveResult,
) ConstraintRowExecResult {
    const solve_step = ctx.descriptor.select_step(pair_result);
    _ = finishSettledContactPairChanged(ctx.descriptor.settle_after_solve, ctx.ctx, solve_step.changed);
    return finalizePreparedContactRowResult(ctx.kind, ctx.ctx, ctx.entities, ctx.before, solve_step, ctx.equation);
}

fn finishContactSolveResultOutcome(
    descriptor: *const ContactRuntimeRowDescriptor,
    kind: ConstraintRowKind,
    ctx: ContactExecutionContext,
    entities: []entity16.Entity16,
    before: f32,
    outcome: ContactPreparedPairOutcome,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return finishPreparedContactOutcome(
        FinishContactSolveResultOutcomeContext,
        ConstraintRowExecResult,
        .{
            .descriptor = descriptor,
            .kind = kind,
            .ctx = ctx,
            .entities = entities,
            .before = before,
            .equation = equation,
        },
        outcome,
        stalledConstraintRowResult(before),
        finishPreparedContactSolveResultReady,
    );
}

const FinishContactBatchSolveOutcomeContext = struct {
    descriptor: *const ContactBatchSolveDescriptor,
    ctx: ContactExecutionContext,
};

fn finishPreparedContactBatchSolveReady(
    ctx: FinishContactBatchSolveOutcomeContext,
    pair_result: ContactPairSolveResult,
) bool {
    return finishSettledContactPairChanged(ctx.descriptor.settle_after_solve, ctx.ctx, pair_result.changed());
}

fn finishContactBatchSolveOutcome(
    descriptor: *const ContactBatchSolveDescriptor,
    ctx: ContactExecutionContext,
    outcome: ContactPreparedPairOutcome,
) bool {
    return finishPreparedContactOutcome(
        FinishContactBatchSolveOutcomeContext,
        bool,
        .{
            .descriptor = descriptor,
            .ctx = ctx,
        },
        outcome,
        false,
        finishPreparedContactBatchSolveReady,
    );
}

fn prepareContactExecutionContext(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    pair: BroadPhasePair,
) ContactExecutionOutcome {
    if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) return .inactive;

    const inst_a = &s1024.instances[pair.a];
    const inst_b = &s1024.instances[pair.b];
    if (inst_a.state == .broken or inst_b.state == .broken) return .inactive;
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return .inactive;

    const prepared = prepareContactConstraintPair(inst_a, inst_b, entities) orelse return .stalled;
    return .{
        .ready = .{
            .inst_a = inst_a,
            .inst_b = inst_b,
            .prepared = prepared,
        },
    };
}

const ContactRuntimeSolveContextMapContext = struct {
    kind: ConstraintRowKind,
    descriptor: *const ContactRuntimeRowDescriptor,
    before: f32,
};

fn buildContactRuntimeSolveContext(
    ctx: ContactRuntimeSolveContextMapContext,
    ready: ContactExecutionContext,
) ContactRuntimeSolveContext {
    return .{
        .kind = ctx.kind,
        .descriptor = ctx.descriptor,
        .contact = ready,
        .before = ctx.before,
    };
}

fn buildContactBatchSolveContext(
    descriptor: *const ContactBatchSolveDescriptor,
    ready: ContactExecutionContext,
) ContactBatchSolveContext {
    return .{
        .descriptor = descriptor,
        .contact = ready,
    };
}

fn prepareContactRuntimeSolveContext(
    kind: ConstraintRowKind,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    pair: BroadPhasePair,
    base_residual: f32,
) ContactRuntimeSolveContextOutcome {
    const descriptor = contactRuntimeRowDescriptor(kind) orelse return .inactive;
    return preparePreparedRuntimeSolveContext(
        ContactRuntimeSolveContextMapContext,
        ContactExecutionContext,
        ContactRuntimeSolveContext,
        ContactExecutionOutcome,
        ContactRuntimeSolveContextOutcome,
        .{
            .kind = kind,
            .descriptor = descriptor,
            .before = base_residual,
        },
        base_residual,
        prepareContactExecutionContext(s1024, entities, pair),
        buildContactRuntimeSolveContext,
    );
}

fn prepareContactBatchSolveContext(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    pair: BroadPhasePair,
) ContactBatchSolveContextOutcome {
    return preparePreparedBatchSolveContext(
        *const ContactBatchSolveDescriptor,
        ContactExecutionContext,
        ContactBatchSolveContext,
        ContactExecutionOutcome,
        ContactBatchSolveContextOutcome,
        &contact_batch_descriptor,
        prepareContactExecutionContext(s1024, entities, pair),
        buildContactBatchSolveContext,
    );
}

fn solvePreparedContactPair(
    ctx: ContactExecutionContext,
    request: ContactPairSolveRequest,
) ContactPairSolveResult {
    return .{
        .normal_step = if (request.normal) |normal|
            solvePreparedContactNormalSteps(
                ctx.inst_a,
                ctx.inst_b,
                ctx.prepared,
                normal.equation,
                normal.row_state,
            )
        else
            .{ .changed = false, .applied_impulse = 0.0 },
        .friction_step = if (request.friction) |friction|
            solvePreparedContactFrictionSteps(
                ctx.inst_a,
                ctx.inst_b,
                ctx.prepared,
                friction.equation,
                friction.row_state,
            )
        else
            .{ .changed = false, .applied_impulse = 0.0 },
    };
}

fn buildContactNormalSolvePlan(
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ContactPreparedPairPlan {
    return .{
        .request = .{
            .normal = .{
                .equation = equation,
                .row_state = row_state,
            },
        },
        .requires_tangent = false,
    };
}

fn buildContactFrictionSolvePlan(
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ContactPreparedPairPlan {
    return .{
        .request = .{
            .friction = .{
                .equation = equation,
                .row_state = row_state,
            },
        },
        .requires_tangent = true,
    };
}

fn selectContactNormalSolveStep(result: ContactPairSolveResult) ConstraintSolveStep {
    return result.normal_step;
}

fn selectContactFrictionSolveStep(result: ContactPairSolveResult) ConstraintSolveStep {
    return result.friction_step;
}

fn buildContactBatchSolvePlan(
    ctx: ContactExecutionContext,
) ContactPreparedPairPlan {
    return .{
        .request = .{
            .normal = .{ .equation = ctx.prepared.normal_equation },
            .friction = .{ .equation = ctx.prepared.friction_equation },
        },
        .requires_tangent = false,
    };
}

fn solvePreparedContactPlan(
    ctx: ContactExecutionContext,
    plan: ContactPreparedPairPlan,
) ContactPreparedPairOutcome {
    if (plan.requires_tangent and !ctx.prepared.has_tangent) return .stalled;
    return .{
        .ready = solvePreparedContactPair(ctx, plan.request),
    };
}

fn executePreparedContactRuntimeRow(
    ctx: ContactRuntimeSolveContext,
    entities: []entity16.Entity16,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    return finishContactSolveResultOutcome(
        ctx.descriptor,
        ctx.kind,
        ctx.contact,
        entities,
        ctx.before,
        solvePreparedContactPlan(ctx.contact, ctx.descriptor.build_plan(equation, row_state)),
        equation,
    );
}

const ExecutePreparedContactRuntimeRowContext = struct {
    entities: []entity16.Entity16,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
};

fn runPreparedContactRuntimeRow(
    ctx: ExecutePreparedContactRuntimeRowContext,
    ready: ContactRuntimeSolveContext,
) ConstraintRowExecResult {
    return executePreparedContactRuntimeRow(
        ready,
        ctx.entities,
        ctx.equation,
        ctx.row_state,
    );
}

fn executePreparedContactRuntimeOutcome(
    outcome: ContactRuntimeSolveContextOutcome,
    entities: []entity16.Entity16,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    return executePreparedRuntimeOutcome(
        ContactRuntimeSolveContext,
        ContactRuntimeSolveContextOutcome,
        ExecutePreparedContactRuntimeRowContext,
        outcome,
        .{
            .entities = entities,
            .equation = equation,
            .row_state = row_state,
        },
        base_residual,
        runPreparedContactRuntimeRow,
    );
}

fn executePreparedContactBatch(
    ctx: ContactBatchSolveContext,
) bool {
    return finishContactBatchSolveOutcome(
        ctx.descriptor,
        ctx.contact,
        solvePreparedContactPlan(ctx.contact, ctx.descriptor.build_plan(ctx.contact)),
    );
}

fn executePreparedContactBatchOutcome(
    outcome: ContactBatchSolveContextOutcome,
) bool {
    return executePreparedBatchOutcomeDirect(
        ContactBatchSolveContext,
        ContactBatchSolveContextOutcome,
        outcome,
        executePreparedContactBatch,
    );
}

fn applyEnvironmentSurfaceResponse(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    move_x: i32,
    move_y: i32,
    move_z: i32,
    previous_vel_x: i16,
    previous_vel_y: i16,
    previous_vel_z: i16,
) void {
    if (move_y != 0) {
        applyVerticalContactResponse(inst, entity, 255, previous_vel_y);
    } else {
        if (move_x != 0) {
            applyLateralContactResponse(inst, entity, 255, .x);
            inst.vel_x = 0;
            if (previous_vel_x != 0 and inst.vel_z == previous_vel_z) {
                inst.vel_z = previous_vel_z;
            }
        }
        if (move_z != 0) {
            applyLateralContactResponse(inst, entity, 255, .z);
            inst.vel_z = 0;
            if (previous_vel_z != 0 and inst.vel_x == previous_vel_x) {
                inst.vel_x = previous_vel_x;
            }
        }
    }

    if (move_y > 0 and @abs(inst.vel_x) <= 1 and @abs(inst.vel_z) <= 1) {
        inst.state = .resting;
    }
}

fn applyEnvironmentResolvedMotion(
    inst: *scene32.Instance,
    entity: *const entity16.Entity16,
    move_x: i32,
    move_y: i32,
    move_z: i32,
    previous_vel_x: i16,
    previous_vel_y: i16,
    previous_vel_z: i16,
) bool {
    if (move_x == 0 and move_y == 0 and move_z == 0) return false;
    inst.pos_x += move_x;
    inst.pos_y += move_y;
    inst.pos_z += move_z;
    applyEnvironmentSurfaceResponse(
        inst,
        entity,
        move_x,
        move_y,
        move_z,
        previous_vel_x,
        previous_vel_y,
        previous_vel_z,
    );
    return true;
}

fn applyEnvironmentWarmStartDisplacement(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    warm_move: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applySingleDisplacementRowStep(
        inst,
        dir_x,
        dir_y,
        dir_z,
        warm_move,
        .{
            .body_mode = .single,
            .channel = .linear_displacement,
            .equation = equation,
            .bias_scale = 0.15,
            .clamp_non_negative = true,
            .axis_x = dir_x,
            .axis_y = dir_y,
            .axis_z = dir_z,
        },
    );
}

fn solvePrimitiveMagnitude(raw_magnitude: f32, primitive: ConstraintApplyPrimitive) f32 {
    var solve_magnitude = constraintRowSolveMagnitude(raw_magnitude, primitive.equation, primitive.bias_scale);
    if (primitive.clamp_non_negative) solve_magnitude = @max(0.0, solve_magnitude);
    return solve_magnitude;
}

const DirectionalDisplacementBodies = union(enum) {
    single: *scene32.Instance,
    pair: struct {
        inst_a: *scene32.Instance,
        inst_b: *scene32.Instance,
    },
};

fn applyDirectionalDisplacementPrimitive(
    bodies: DirectionalDisplacementBodies,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    const solve_magnitude = solvePrimitiveMagnitude(raw_magnitude, primitive);
    return switch (bodies) {
        .single => |inst| .{
            .changed = applyKernelSingleDisplacement(inst, dir_x, dir_y, dir_z, solve_magnitude),
            .applied_impulse = solve_magnitude,
        },
        .pair => |pair| blk: {
            const pair_bodies = primitive.pair_bodies orelse break :blk .{ .changed = false, .applied_impulse = 0.0 };
            if (solve_magnitude <= 0.0 and primitive.magnitude_slop <= 0.0) {
                break :blk .{ .changed = false, .applied_impulse = 0.0 };
            }

            break :blk .{
                .changed = applyKernelPairDirectionalDisplacement(
                    pair.inst_a,
                    pair.inst_b,
                    .{
                        .inv_mass_a = pair_bodies.inv_mass_a,
                        .inv_mass_b = pair_bodies.inv_mass_b,
                        .ratio_a = pair_bodies.ratio_a,
                        .ratio_b = pair_bodies.ratio_b,
                    },
                    dir_x,
                    dir_y,
                    dir_z,
                    solve_magnitude + primitive.magnitude_slop,
                ),
                .applied_impulse = solve_magnitude,
            };
        },
    };
}

fn applySingleDisplacementRowStep(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    return applyDirectionalDisplacementPrimitive(
        .{ .single = inst },
        dir_x,
        dir_y,
        dir_z,
        raw_magnitude,
        primitive,
    );
}

fn applyPairDirectionalDisplacementRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    return applyDirectionalDisplacementPrimitive(
        .{ .pair = .{
            .inst_a = inst_a,
            .inst_b = inst_b,
        } },
        dir_x,
        dir_y,
        dir_z,
        raw_magnitude,
        primitive,
    );
}

fn applyEnvironmentSolveDisplacement(
    inst: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applySingleDisplacementRowStep(
        inst,
        dir_x,
        dir_y,
        dir_z,
        raw_magnitude,
        .{
            .body_mode = .single,
            .channel = .linear_displacement,
            .equation = equation,
            .bias_scale = 0.35,
            .clamp_non_negative = true,
            .axis_x = dir_x,
            .axis_y = dir_y,
            .axis_z = dir_z,
        },
    );
}

fn solvePreparedEnvironmentMotion(
    inst: *scene32.Instance,
    prepared: PreparedEnvironmentConstraint,
    equation: ConstraintRowEquation,
) EnvironmentSolveResult {
    const solve_step = applyEnvironmentSolveDisplacement(
        inst,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.normal.depth,
        equation,
    );
    return .{
        .solve_step = solve_step,
        .resolved_motion = .{
            .move_x = @as(i32, @intFromFloat(@round(prepared.normal.dir_x * solve_step.applied_impulse))),
            .move_y = @as(i32, @intFromFloat(@round(prepared.normal.dir_y * solve_step.applied_impulse))),
            .move_z = @as(i32, @intFromFloat(@round(prepared.normal.dir_z * solve_step.applied_impulse))),
        },
    };
}

fn prepareEnvironmentResolvedMotion(prepared: PreparedEnvironmentConstraint) EnvironmentResolvedMotion {
    return .{
        .move_x = prepared.move_x,
        .move_y = prepared.move_y,
        .move_z = prepared.move_z,
    };
}

fn buildPreparedEnvironmentBatchSolveResult(
    prepared: PreparedEnvironmentConstraint,
) EnvironmentSolveResult {
    const resolved_motion = prepareEnvironmentResolvedMotion(prepared);
    return .{
        .solve_step = .{
            .changed = resolved_motion.changed(),
            .applied_impulse = prepared.normal.depth,
        },
        .resolved_motion = resolved_motion,
    };
}

fn prepareEnvironmentExecutionContext(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) EnvironmentExecutionOutcome {
    if (instance_idx >= s1024.instance_count) return .inactive;

    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return .stalled;
    if (inst.entity_id >= entities.len) return .stalled;

    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return .stalled;
    const prepared = prepareEnvironmentConstraint(s1024, entities, instance_idx) orelse return .stalled;

    return .{
        .ready = .{
            .instance_idx = instance_idx,
            .inst = inst,
            .entity = entity,
            .prepared = prepared,
            .previous_vel_x = inst.vel_x,
            .previous_vel_y = inst.vel_y,
            .previous_vel_z = inst.vel_z,
        },
    };
}

fn buildEnvironmentRuntimeSolveContext(
    before: f32,
    ready: EnvironmentExecutionContext,
) EnvironmentRuntimeSolveContext {
    return .{
        .execution = ready,
        .before = before,
    };
}

fn prepareEnvironmentRuntimeSolveContext(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    base_residual: f32,
) EnvironmentRuntimeSolveContextOutcome {
    return preparePreparedRuntimeSolveContext(
        f32,
        EnvironmentExecutionContext,
        EnvironmentRuntimeSolveContext,
        EnvironmentExecutionOutcome,
        EnvironmentRuntimeSolveContextOutcome,
        base_residual,
        base_residual,
        prepareEnvironmentExecutionContext(s1024, entities, instance_idx),
        buildEnvironmentRuntimeSolveContext,
    );
}

fn buildEnvironmentBatchSolveContext(
    _: void,
    ready: EnvironmentExecutionContext,
) EnvironmentBatchSolveContext {
    return .{
        .execution = ready,
    };
}

fn prepareEnvironmentBatchSolveContext(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) EnvironmentBatchSolveContextOutcome {
    return preparePreparedBatchSolveContext(
        void,
        EnvironmentExecutionContext,
        EnvironmentBatchSolveContext,
        EnvironmentExecutionOutcome,
        EnvironmentBatchSolveContextOutcome,
        {},
        prepareEnvironmentExecutionContext(s1024, entities, instance_idx),
        buildEnvironmentBatchSolveContext,
    );
}

fn executePreparedEnvironmentResolvedMotion(
    ctx: EnvironmentExecutionContext,
    result: EnvironmentSolveResult,
) bool {
    const moved = result.applyResolvedMotion(
        ctx.inst,
        ctx.entity,
        ctx.previous_vel_x,
        ctx.previous_vel_y,
        ctx.previous_vel_z,
    );
    if (moved) wakeInstance(ctx.inst);
    return moved;
}

fn executePreparedEnvironmentBatch(
    ctx: EnvironmentBatchSolveContext,
) bool {
    return executePreparedEnvironmentResolvedMotion(
        ctx.execution,
        buildPreparedEnvironmentBatchSolveResult(ctx.execution.prepared),
    );
}

fn executePreparedEnvironmentBatchOutcome(
    outcome: EnvironmentBatchSolveContextOutcome,
) bool {
    return executePreparedBatchOutcomeDirect(
        EnvironmentBatchSolveContext,
        EnvironmentBatchSolveContextOutcome,
        outcome,
        executePreparedEnvironmentBatch,
    );
}

const ExecutePreparedEnvironmentRuntimeRowContext = struct {
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
};

fn runPreparedEnvironmentRuntimeRow(
    ctx: ExecutePreparedEnvironmentRuntimeRowContext,
    ready: EnvironmentRuntimeSolveContext,
) ConstraintRowExecResult {
    return executePreparedEnvironmentRow(
        ctx.s1024,
        ctx.entities,
        ready.execution,
        ready.before,
        ctx.equation,
        ctx.row_state,
    );
}

fn finalizePreparedEnvironmentRowResult(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    ctx: EnvironmentExecutionContext,
    before: f32,
    solve_result: EnvironmentSolveResult,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    const after = measureEnvironmentRowResidual(s1024, entities, ctx.instance_idx);
    return finalizeSingleSolveStepResult(
        after,
        ctx.inst,
        before,
        withSolveStepAppliedImpulse(solve_result.solve_step, applied_impulse),
        equation,
    );
}

fn executePreparedEnvironmentRuntimeOutcome(
    outcome: EnvironmentRuntimeSolveContextOutcome,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    return executePreparedRuntimeOutcome(
        EnvironmentRuntimeSolveContext,
        EnvironmentRuntimeSolveContextOutcome,
        ExecutePreparedEnvironmentRuntimeRowContext,
        outcome,
        .{
            .s1024 = s1024,
            .entities = entities,
            .equation = equation,
            .row_state = row_state,
        },
        base_residual,
        runPreparedEnvironmentRuntimeRow,
    );
}

fn executePreparedEnvironmentRow(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    ctx: EnvironmentExecutionContext,
    before: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    var applied_impulse: f32 = 0.0;

    if (row_state) |_| {
        const warm_move = @min(
            constraintRowWarmImpulse(row_state, 0.25, equation, 1.0),
            @min(ctx.prepared.normal.depth * 0.5, equation.max_impulse),
        );
        const warm_step = applyEnvironmentWarmStartDisplacement(
            ctx.inst,
            ctx.prepared.normal.dir_x,
            ctx.prepared.normal.dir_y,
            ctx.prepared.normal.dir_z,
            warm_move,
            equation,
        );
        applied_impulse += warm_step.applied_impulse;
    }

    const solve_result = solvePreparedEnvironmentMotion(
        ctx.inst,
        ctx.prepared,
        equation,
    );
    applied_impulse = @max(applied_impulse, solve_result.solve_step.applied_impulse);

    _ = executePreparedEnvironmentResolvedMotion(ctx, solve_result);
    return finalizePreparedEnvironmentRowResult(
        s1024,
        entities,
        ctx,
        before,
        solve_result,
        applied_impulse,
        equation,
    );
}

fn applyPairAngularPositionRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    axis: JointAxis,
    angle_a: f32,
    angle_b: f32,
    raw_correction: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    const pair_bodies = primitive.pair_bodies orelse return .{ .changed = false, .applied_impulse = 0.0 };
    const solve_correction = solvePrimitiveMagnitude(raw_correction, primitive);
    if (solve_correction == 0.0) return .{ .changed = false, .applied_impulse = 0.0 };

    var changed = false;
    if (pair_bodies.inv_mass_a > 0.0) {
        switch (axis) {
            .x => inst_a.rot_roll = jointRadiansToRotationByte(angle_a + solve_correction * pair_bodies.ratio_a),
            .y => inst_a.rot_yaw = jointRadiansToRotationByte(angle_a + solve_correction * pair_bodies.ratio_a),
            .z => inst_a.rot_pitch = jointRadiansToRotationByte(angle_a + solve_correction * pair_bodies.ratio_a),
        }
        changed = true;
    }
    if (pair_bodies.inv_mass_b > 0.0) {
        switch (axis) {
            .x => inst_b.rot_roll = jointRadiansToRotationByte(angle_b - solve_correction * pair_bodies.ratio_b),
            .y => inst_b.rot_yaw = jointRadiansToRotationByte(angle_b - solve_correction * pair_bodies.ratio_b),
            .z => inst_b.rot_pitch = jointRadiansToRotationByte(angle_b - solve_correction * pair_bodies.ratio_b),
        }
        changed = true;
    }

    return .{
        .changed = changed,
        .applied_impulse = @abs(solve_correction),
    };
}

fn applyPairAngularVelocityRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    axis: JointAxis,
    raw_signed_bias: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    const pair_bodies = primitive.pair_bodies orelse return .{ .changed = false, .applied_impulse = 0.0 };
    const solve_bias = solvePrimitiveMagnitude(raw_signed_bias, primitive);
    const quantized_bias = clampKernelI8FromF32(@max(-32.0, @min(32.0, solve_bias)));
    if (quantized_bias == 0) return .{ .changed = false, .applied_impulse = 0.0 };

    var changed = false;
    if (pair_bodies.inv_mass_a > 0.0) {
        const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(quantized_bias)) * pair_bodies.ratio_a);
        changed = applyKernelAngularVelocityDelta(inst_a, axis, -delta) or changed;
    }
    if (pair_bodies.inv_mass_b > 0.0) {
        const delta = clampKernelI8FromF32(@as(f32, @floatFromInt(quantized_bias)) * pair_bodies.ratio_b);
        changed = applyKernelAngularVelocityDelta(inst_b, axis, delta) or changed;
    }

    return .{
        .changed = changed,
        .applied_impulse = @abs(@as(f32, @floatFromInt(quantized_bias))),
    };
}

fn solveJointAngularPositionConstraintRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    angular: JointAngularConstraintSpec,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applyPairAngularPositionRowStep(
        inst_a,
        inst_b,
        angular.axis,
        angular.angle_a,
        angular.angle_b,
        angular.correction,
        makePairPrimitiveFromJointMassData(mass_data, .angular_position, equation, 0.35, 0.0, 0.0, 0.0),
    );
}

fn solveJointAngularVelocityConstraintRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    angular_velocity: JointAngularVelocityConstraintSpec,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applyPairAngularVelocityRowStep(
        inst_a,
        inst_b,
        angular_velocity.axis,
        angular_velocity.signed_bias,
        makePairPrimitiveFromJointMassData(mass_data, .angular_velocity, equation, 0.0, 0.0, 0.0, 0.0),
    );
}

fn applyPairDirectionalConstraintRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_magnitude: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    const pair_bodies = primitive.pair_bodies orelse return .{ .changed = false, .applied_impulse = 0.0 };

    const solve_magnitude = solvePrimitiveMagnitude(raw_magnitude, primitive);
    if (solve_magnitude == 0.0) return .{ .changed = false, .applied_impulse = 0.0 };

    const scaled_magnitude = solve_magnitude * (1.0 + cappedWarmRatio(primitive.warm_impulse, solve_magnitude));
    return .{
        .changed = applyKernelPairDirectionalDisplacement(
            inst_a,
            inst_b,
            .{
                .inv_mass_a = pair_bodies.inv_mass_a,
                .inv_mass_b = pair_bodies.inv_mass_b,
                .ratio_a = pair_bodies.ratio_a,
                .ratio_b = pair_bodies.ratio_b,
            },
            dir_x,
            dir_y,
            dir_z,
            scaled_magnitude,
        ),
        .applied_impulse = @abs(solve_magnitude) * (1.0 + cappedWarmRatio(primitive.warm_impulse, solve_magnitude)),
    };
}

fn applyPairLinearVelocityDampingStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    relative_speed: f32,
    damping_scale: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    const pair_bodies = primitive.pair_bodies orelse return .{ .changed = false, .applied_impulse = 0.0 };
    const changed = applyKernelLinearDamping(
        inst_a,
        inst_b,
        .{
            .inv_mass_a = pair_bodies.inv_mass_a,
            .inv_mass_b = pair_bodies.inv_mass_b,
            .ratio_a = pair_bodies.ratio_a,
            .ratio_b = pair_bodies.ratio_b,
        },
        dir_x,
        dir_y,
        dir_z,
        relative_speed,
        damping_scale,
    );
    return .{
        .changed = changed,
        .applied_impulse = @abs(relative_speed * damping_scale),
    };
}

fn applyJointAxisScalarCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis: JointAxisVector,
    correction: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return applyPairDirectionalConstraintRowStep(
        inst_a,
        inst_b,
        axis.x,
        axis.y,
        axis.z,
        correction,
        makePairPrimitiveFromJointMassData(mass_data, .linear_directional, equation, 0.35, axis.x, axis.y, axis.z),
    );
}

fn applyJointDirectionalConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    magnitude: f32,
    warm_impulse: f32,
    damping: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const displacement_step = applyPairDirectionalConstraintRowStep(
        inst_a,
        inst_b,
        dir_x,
        dir_y,
        dir_z,
        magnitude,
        blk: {
            var primitive = makePairPrimitiveFromJointMassData(mass_data, .linear_directional, equation, 0.25, dir_x, dir_y, dir_z);
            primitive.warm_impulse = warm_impulse;
            break :blk primitive;
        },
    );
    const rel_speed = measureKernelRelativeLinearSpeed(inst_a, inst_b, dir_x, dir_y, dir_z);
    const damping_step = applyPairLinearVelocityDampingStep(
        inst_a,
        inst_b,
        dir_x,
        dir_y,
        dir_z,
        rel_speed,
        damping,
        makePairPrimitiveFromJointMassData(mass_data, .linear_velocity, equation, 0.0, dir_x, dir_y, dir_z),
    );
    return .{
        .changed = displacement_step.changed or damping_step.changed,
        .applied_impulse = @max(displacement_step.applied_impulse, damping_step.applied_impulse),
    };
}

fn jointRowSignedCorrection(
    row_state: ?*const ConstraintRowState,
    equation: ConstraintRowEquation,
    value: f32,
    policy: JointSignedCorrectionPolicy,
) f32 {
    return if (policy.planned)
        constraintRowPlannedSignedCorrection(
            row_state,
            policy.retention_scale,
            equation,
            policy.bias_scale,
            value,
        )
    else
        constraintRowSignedCorrection(
            row_state,
            policy.retention_scale,
            equation,
            policy.bias_scale,
            value,
            policy.min_abs_correction,
        );
}

fn prepareJointDriveSolvePostprocess(
    prepared: JointPreparedDriveChannel,
    postprocess: JointPreparedDrivePostprocess,
) JointPreparedPostprocess {
    return switch (postprocess) {
        .none => .none,
        .velocity_bias => .{ .drive_velocity_bias = prepared },
    };
}

fn jointImpulseHint(value: f32, policy: JointImpulseHintPolicy) f32 {
    if (!policy.enabled) return 0.0;
    return value * policy.scale;
}

fn prepareJointAnchorSolvePostprocess(
    postprocess: JointAnchorPostprocess,
) JointPreparedPostprocess {
    return .{ .anchor = postprocess };
}

fn prepareJointLimitSolvePostprocess(
    postprocess: JointLimitPostprocess,
) JointPreparedPostprocess {
    return switch (postprocess) {
        .none => .none,
        else => .{ .limit = postprocess },
    };
}

fn withJointSolveResultMetadata(
    result: JointPreparedSolveResult,
    impulse_hint: f32,
    postprocess: JointPreparedPostprocess,
) JointPreparedSolveResult {
    var updated = result;
    updated.impulse_hint = impulse_hint;
    updated.postprocess = postprocess;
    return updated;
}

fn buildJointAngularPositionSolveResult(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis: JointAxis,
    angle_a: f32,
    angle_b: f32,
    correction: f32,
    equation: ConstraintRowEquation,
) JointPreparedSolveResult {
    return .{
        .solve_step = solveJointAngularPositionConstraintRowStep(
            inst_a,
            inst_b,
            mass_data,
            .{
                .axis = axis,
                .angle_a = angle_a,
                .angle_b = angle_b,
                .correction = correction,
            },
            equation,
        ),
    };
}

fn buildJointLinearAxisSolveResult(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis: JointAxisVector,
    correction: f32,
    equation: ConstraintRowEquation,
) JointPreparedSolveResult {
    return .{
        .solve_step = applyJointAxisScalarCorrection(inst_a, inst_b, mass_data, axis, correction, equation),
    };
}

fn applyJointPulleyConstraintStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    pulley: PreparedJointPulleyConstraint,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const denom = mass_data.inv_mass_a + pulley.ratio * pulley.ratio * mass_data.inv_mass_b;
    if (denom <= 0.0001) return .{ .changed = false, .applied_impulse = 0.0 };

    const primitive = ConstraintApplyPrimitive{
        .body_mode = .pair,
        .channel = .linear_directional,
        .equation = equation,
        .bias_scale = 0.35,
        .axis_x = pulley.axis.x,
        .axis_y = pulley.axis.y,
        .axis_z = pulley.axis.z,
        .pair_bodies = ConstraintPairBodyData.fromJointMassData(mass_data),
    };
    const solve_error = solvePrimitiveMagnitude(pulley.pulley_error, primitive);
    var changed = false;
    if (solve_error != 0.0) {
        const impulse = solve_error / denom;
        if (mass_data.inv_mass_a > 0.0) {
            changed = applyKernelSingleDisplacement(
                inst_a,
                -pulley.axis.x,
                -pulley.axis.y,
                -pulley.axis.z,
                impulse * mass_data.inv_mass_a,
            ) or changed;
        }
        if (mass_data.inv_mass_b > 0.0) {
            changed = applyKernelSingleDisplacement(
                inst_b,
                -pulley.axis.x,
                -pulley.axis.y,
                -pulley.axis.z,
                pulley.ratio * impulse * mass_data.inv_mass_b,
            ) or changed;
        }
    }

    if (@abs(pulley.constraint_speed) > 0.0001 and pulley.damping_scale > 0.0) {
        const damping_impulse = pulley.constraint_speed * pulley.damping_scale / denom;
        if (mass_data.inv_mass_a > 0.0) {
            inst_a.vel_x -= clampKernelI16FromF32(pulley.axis.x * damping_impulse * mass_data.inv_mass_a);
            inst_a.vel_y -= clampKernelI16FromF32(pulley.axis.y * damping_impulse * mass_data.inv_mass_a);
            inst_a.vel_z -= clampKernelI16FromF32(pulley.axis.z * damping_impulse * mass_data.inv_mass_a);
            changed = true;
        }
        if (mass_data.inv_mass_b > 0.0) {
            inst_b.vel_x -= clampKernelI16FromF32(pulley.axis.x * pulley.ratio * damping_impulse * mass_data.inv_mass_b);
            inst_b.vel_y -= clampKernelI16FromF32(pulley.axis.y * pulley.ratio * damping_impulse * mass_data.inv_mass_b);
            inst_b.vel_z -= clampKernelI16FromF32(pulley.axis.z * pulley.ratio * damping_impulse * mass_data.inv_mass_b);
            changed = true;
        }
    }

    return .{
        .changed = changed,
        .applied_impulse = @max(@abs(solve_error), @abs(pulley.constraint_speed * pulley.damping_scale)),
    };
}

fn applyPreparedJointLimitChannel(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    prepared: JointPreparedLimitChannel,
    row_state: ?*const ConstraintRowState,
    equation: ConstraintRowEquation,
    correction_policy: JointSignedCorrectionPolicy,
    impulse_hint_policy: JointImpulseHintPolicy,
) JointPreparedSolveResult {
    _ = impulse_hint_policy;
    return switch (prepared) {
        .angular => |angular| blk: {
            const correction = jointRowSignedCorrection(row_state, equation, angular.constraint.magnitude, correction_policy);
            break :blk withJointSolveResultMetadata(buildJointAngularPositionSolveResult(
                inst_a,
                inst_b,
                mass_data,
                angular.axis,
                angular.constraint.angle_a,
                angular.constraint.angle_b,
                correction,
                equation,
            ), 0.0, prepareJointLimitSolvePostprocess(angular.postprocess));
        },
        .linear => |linear| blk: {
            const correction = jointRowSignedCorrection(row_state, equation, linear.constraint.magnitude, correction_policy);
            break :blk buildJointLinearAxisSolveResult(
                inst_a,
                inst_b,
                mass_data,
                linear.axis,
                correction,
                equation,
            );
        },
        .pulley => |pulley| .{
            .solve_step = applyJointPulleyConstraintStep(
                inst_a,
                inst_b,
                mass_data,
                pulley,
                equation,
            ),
        },
    };
}

fn applyPreparedJointAnchorChannel(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    prepared: JointPreparedAnchorChannel,
    warm_impulse: f32,
    equation: ConstraintRowEquation,
) JointPreparedSolveResult {
    return withJointSolveResultMetadata(.{
        .solve_step = applyJointDirectionalConstraint(
            inst_a,
            inst_b,
            mass_data,
            prepared.axis.x,
            prepared.axis.y,
            prepared.axis.z,
            prepared.constraint.magnitude,
            warm_impulse,
            prepared.damping_scale,
            equation,
        ),
    }, 0.0, prepareJointAnchorSolvePostprocess(prepared.postprocess));
}

fn applyPreparedJointDriveChannel(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    prepared: JointPreparedDriveChannel,
    row_state: ?*const ConstraintRowState,
    equation: ConstraintRowEquation,
    correction_policy: JointSignedCorrectionPolicy,
    impulse_hint_policy: JointImpulseHintPolicy,
) JointPreparedSolveResult {
    return switch (prepared) {
        .angular => |angular| blk: {
            const signed_step = jointRowSignedCorrection(row_state, equation, angular.constraint.signed_step, correction_policy);
            break :blk withJointSolveResultMetadata(buildJointAngularPositionSolveResult(
                inst_a,
                inst_b,
                mass_data,
                angular.axis,
                angular.constraint.angle_a,
                angular.constraint.angle_b,
                -signed_step,
                equation,
            ), jointImpulseHint(angular.constraint.signed_step, impulse_hint_policy), prepareJointDriveSolvePostprocess(prepared, angular.postprocess));
        },
        .linear => |linear| blk: {
            const signed_step = jointRowSignedCorrection(row_state, equation, linear.constraint.signed_step, correction_policy);
            break :blk withJointSolveResultMetadata(buildJointLinearAxisSolveResult(
                inst_a,
                inst_b,
                mass_data,
                linear.axis,
                -signed_step,
                equation,
            ), jointImpulseHint(linear.constraint.signed_step, impulse_hint_policy), prepareJointDriveSolvePostprocess(prepared, linear.postprocess));
        },
    };
}

fn applyPreparedJointAnchorPostprocess(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    postprocess: JointAnchorPostprocess,
) bool {
    return switch (postprocess) {
        .none => false,
        .hinge_limit_damping => applyKernelAngularLimitVelocityDamping(inst_a, inst_b, joint_def, mass_data),
        .slider_limit_damping => |axis| blk: {
            const current_delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
            const along = current_delta.x * axis.x + current_delta.y * axis.y + current_delta.z * axis.z;
            const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
            const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
            break :blk applyKernelSliderLimitVelocityDamping(
                inst_a,
                inst_b,
                mass_data,
                axis.x,
                axis.y,
                axis.z,
                along,
                min_limit,
                max_limit,
            );
        },
    };
}

fn applyKernelAngularVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis: JointAxis,
) bool {
    const relative_velocity = getKernelAngularVelocityOnAxis(inst_b, axis) - getKernelAngularVelocityOnAxis(inst_a, axis);
    if (@abs(relative_velocity) <= 0.0001) return false;

    return solveJointAngularVelocityConstraintRowStep(
        inst_a,
        inst_b,
        mass_data,
        .{
            .axis = axis,
            .signed_bias = -relative_velocity,
        },
        .{
            .effective_mass = 1.0,
            .bias = 0.0,
            .max_impulse = 32.0,
        },
    ).changed;
}

fn applyPreparedJointLimitPostprocess(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    postprocess: JointLimitPostprocess,
) bool {
    return switch (postprocess) {
        .none => false,
        .angular_velocity_damping => |axis| applyKernelAngularVelocityDamping(inst_a, inst_b, mass_data, axis),
    };
}

fn applyPreparedJointDriveVelocityBias(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    prepared: JointPreparedDriveChannel,
) bool {
    return switch (prepared) {
        .angular => |angular| applyKernelAngularMotorVelocityBias(
            inst_a,
            inst_b,
            joint_def,
            mass_data,
            angular.constraint.desired_velocity,
        ),
        .linear => |linear| applyKernelLinearMotorVelocityBias(
            inst_a,
            inst_b,
            joint_def,
            mass_data,
            linear.axis.x,
            linear.axis.y,
            linear.axis.z,
            linear.constraint.desired_velocity,
        ),
    };
}

fn applyKernelAngularLimitVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
) bool {
    const axis = jointDominantAxis(joint_def);
    const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
    const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
    const angle_tolerance: f32 = 0.05;
    const relative_angle = getKernelJointAngle(inst_b, joint_def) - getKernelJointAngle(inst_a, joint_def);
    const relative_velocity = getKernelAngularVelocityOnAxis(inst_b, axis) - getKernelAngularVelocityOnAxis(inst_a, axis);

    const pushes_past_min = relative_angle <= min_angle + angle_tolerance and relative_velocity < 0.0;
    const pushes_past_max = relative_angle >= max_angle - angle_tolerance and relative_velocity > 0.0;
    if (!pushes_past_min and !pushes_past_max) return false;

    const velocity_equation: ConstraintRowEquation = .{
        .effective_mass = 1.0,
        .bias = 0.0,
        .max_impulse = 32.0,
    };
    return solveJointAngularVelocityConstraintRowStep(
        inst_a,
        inst_b,
        mass_data,
        .{
            .axis = axis,
            .signed_bias = -relative_velocity,
        },
        velocity_equation,
    ).changed;
}

fn applyKernelSliderLimitVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: JointMassData,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    along: f32,
    min_limit: f32,
    max_limit: f32,
) bool {
    const rel_speed = measureKernelRelativeLinearSpeed(inst_a, inst_b, axis_x, axis_y, axis_z);

    const pushes_past_min = along <= min_limit + 0.0001 and rel_speed < 0.0;
    const pushes_past_max = along >= max_limit - 0.0001 and rel_speed > 0.0;
    if (!pushes_past_min and !pushes_past_max) return false;

    return applyPairLinearVelocityDampingStep(
        inst_a,
        inst_b,
        axis_x,
        axis_y,
        axis_z,
        @max(-64.0, @min(64.0, rel_speed)),
        1.0,
        makePairPrimitiveFromJointMassData(
            mass_data,
            .linear_velocity,
            .{ .effective_mass = 1.0, .bias = 0.0, .max_impulse = 64.0 },
            0.0,
            axis_x,
            axis_y,
            axis_z,
        ),
    ).changed;
}

fn measureKernelJointDriveState(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?KernelJointDriveState {
    return switch (joint_def.joint_type) {
        .hinge => .{
            .position = getKernelJointAngle(inst_b, joint_def) - getKernelJointAngle(inst_a, joint_def),
            .relative_velocity = getKernelAngularVelocityOnAxis(inst_b, jointDominantAxis(joint_def)) -
                getKernelAngularVelocityOnAxis(inst_a, jointDominantAxis(joint_def)),
        },
        .slider => blk: {
            const axis = getKernelJointAxisVector(joint_def) orelse return null;
            const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
            break :blk .{
                .position = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z,
                .relative_velocity = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x)) * axis.x +
                    @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y)) * axis.y +
                    @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z)) * axis.z,
            };
        },
        else => null,
    };
}

fn computeKernelJointDrivePlan(
    joint_def: *const joint.Joint,
    drive_state: KernelJointDriveState,
    min_step: f32,
) ?KernelJointDrivePlan {
    const prediction_dt = kernelPredictionDt();
    const max_drive_velocity = @max(1.0, joint_def.motor_speed);
    const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
    const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
    const target = @max(min_limit, @min(max_limit, joint_def.motor_target));
    const position_error = target - drive_state.position;
    if (@abs(position_error) <= 0.0001) {
        if (@abs(drive_state.relative_velocity) <= 0.0001) return null;
        return idleKernelJointDrivePlan();
    }

    const predicted_position = drive_state.position + drive_state.relative_velocity * prediction_dt;
    const predicted_error = target - predicted_position;
    if (@abs(predicted_error) <= 0.0001) {
        return idleKernelJointDrivePlan();
    }

    const predictive_plan = makeJointDrivePredictiveRowPlan(position_error, predicted_error, prediction_dt);
    const use_predictive_brake = position_error * predicted_error < 0.0 or
        predictive_plan.predicted_depth < predictive_plan.current_depth;
    const control_error = if (use_predictive_brake)
        if (predicted_error < 0.0) -predictive_plan.predicted_depth else predictive_plan.predicted_depth
    else if (position_error < 0.0) -predictive_plan.current_depth else predictive_plan.current_depth;
    const speed_step = if (joint_def.motor_speed > 0.0) joint_def.motor_speed * prediction_dt else @abs(control_error);
    const torque_step = if (joint_def.motor_max_torque > 0.0) joint_def.motor_max_torque * 0.001 else @abs(control_error);
    const requested_step = @min(@abs(control_error), @min(speed_step, torque_step));
    const max_step = @max(min_step, requested_step);
    const signed_step = if (control_error < 0.0) -max_step else max_step;
    const desired_velocity = if (use_predictive_brake)
        @max(-max_drive_velocity, @min(max_drive_velocity, predicted_error / prediction_dt))
    else
        signed_step * max_drive_velocity;

    return .{
        .signed_step = signed_step,
        .desired_velocity = desired_velocity,
    };
}

fn applyKernelAngularMotorVelocityBias(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    desired_velocity: f32,
) bool {
    const axis = jointDominantAxis(joint_def);
    const drive_state = measureKernelJointDriveState(inst_a, inst_b, joint_def) orelse return false;
    const current_rel_velocity = drive_state.relative_velocity;
    if (desired_velocity != 0.0 and current_rel_velocity != 0.0) {
        if ((desired_velocity > 0.0 and current_rel_velocity > 0.0 and current_rel_velocity >= desired_velocity) or
            (desired_velocity < 0.0 and current_rel_velocity < 0.0 and current_rel_velocity <= desired_velocity))
        {
            return false;
        }
    }

    const velocity_equation: ConstraintRowEquation = .{
        .effective_mass = 1.0,
        .bias = 0.0,
        .max_impulse = 32.0,
    };
    return solveJointAngularVelocityConstraintRowStep(
        inst_a,
        inst_b,
        mass_data,
        .{
            .axis = axis,
            .signed_bias = desired_velocity - current_rel_velocity,
        },
        velocity_equation,
    ).changed;
}

fn applyKernelLinearMotorVelocityBias(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const joint.Joint,
    mass_data: JointMassData,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    desired_speed: f32,
) bool {
    const drive_state = measureKernelJointDriveState(inst_a, inst_b, joint_def) orelse return false;
    const current_rel_speed = drive_state.relative_velocity;
    if (desired_speed != 0.0 and current_rel_speed != 0.0) {
        if ((desired_speed > 0.0 and current_rel_speed > 0.0 and current_rel_speed >= desired_speed) or
            (desired_speed < 0.0 and current_rel_speed < 0.0 and current_rel_speed <= desired_speed))
        {
            return false;
        }
    }

    const vel_bias = @max(-64.0, @min(64.0, desired_speed - current_rel_speed));
    if (vel_bias == 0.0) return false;

    var changed = false;
    if (mass_data.inv_mass_a > 0.0) {
        const dvx = clampKernelI16FromF32(axis_x * vel_bias * mass_data.ratio_a);
        const dvy = clampKernelI16FromF32(axis_y * vel_bias * mass_data.ratio_a);
        const dvz = clampKernelI16FromF32(axis_z * vel_bias * mass_data.ratio_a);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_a.vel_x -= dvx;
            inst_a.vel_y -= dvy;
            inst_a.vel_z -= dvz;
            changed = true;
        }
    }
    if (mass_data.inv_mass_b > 0.0) {
        const dvx = clampKernelI16FromF32(axis_x * vel_bias * mass_data.ratio_b);
        const dvy = clampKernelI16FromF32(axis_y * vel_bias * mass_data.ratio_b);
        const dvz = clampKernelI16FromF32(axis_z * vel_bias * mass_data.ratio_b);
        if (dvx != 0 or dvy != 0 or dvz != 0) {
            inst_b.vel_x += dvx;
            inst_b.vel_y += dvy;
            inst_b.vel_z += dvz;
            changed = true;
        }
    }

    return changed;
}

fn buildContactNormalEquation(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
) ConstraintRowEquation {
    return buildContactNormalEquationWithResponse(
        inv_mass_a,
        inv_mass_b,
        penetration_depth,
        relative_normal_speed,
        neutralContactStiffnessModel(),
        neutralContactDampingModel(),
    );
}

fn buildContactNormalEquationWithStiffness(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    stiffness: ContactStiffnessModel,
) ConstraintRowEquation {
    return buildContactNormalEquationWithResponse(
        inv_mass_a,
        inv_mass_b,
        penetration_depth,
        relative_normal_speed,
        stiffness,
        neutralContactDampingModel(),
    );
}

fn buildContactNormalEquationWithResponse(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    stiffness: ContactStiffnessModel,
    damping: ContactDampingModel,
) ConstraintRowEquation {
    return buildContactNormalEquationWithResponseAndStabilization(
        inv_mass_a,
        inv_mass_b,
        penetration_depth,
        relative_normal_speed,
        stiffness,
        damping,
        neutralContactStabilizationModel(),
    );
}

fn buildContactNormalEquationWithResponseAndStabilization(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    stiffness: ContactStiffnessModel,
    damping: ContactDampingModel,
    stabilization: ContactStabilizationModel,
) ConstraintRowEquation {
    return applyContactStabilizationToEquation(applyContactStiffnessToEquation(buildNormalConstraintEquation(
        effectiveMassFromPairInverseMasses(inv_mass_a, inv_mass_b),
        penetration_depth,
        relative_normal_speed,
        0.2,
        0.05 * damping.scale,
        8.0,
        0.5 * damping.scale,
    ), stiffness), stabilization);
}

fn contactStiffnessPolicy() ContactStiffnessPolicy {
    return .{};
}

fn neutralContactStiffnessModel() ContactStiffnessModel {
    return .{
        .penetration_resistance = contactStiffnessPolicy().neutral_penetration_resistance,
        .scale = 1.0,
    };
}

fn computeContactStiffnessModel(
    penetration_resistance: f32,
    policy: ContactStiffnessPolicy,
) ContactStiffnessModel {
    const resistance = @max(0.0, @min(1.0, penetration_resistance));
    const raw_scale = 1.0 + (resistance - policy.neutral_penetration_resistance) * policy.response_scale;
    return .{
        .penetration_resistance = resistance,
        .scale = @max(policy.min_scale, @min(policy.max_scale, raw_scale)),
    };
}

fn applyContactStiffnessToEquation(
    equation: ConstraintRowEquation,
    stiffness: ContactStiffnessModel,
) ConstraintRowEquation {
    return .{
        .effective_mass = equation.effective_mass,
        .bias = equation.bias * stiffness.scale,
        .max_impulse = @max(0.0, equation.max_impulse * stiffness.scale),
    };
}

fn contactDampingPolicy() ContactDampingPolicy {
    return .{};
}

fn neutralContactDampingModel() ContactDampingModel {
    return .{
        .restitution = contactDampingPolicy().neutral_restitution,
        .scale = 1.0,
    };
}

fn computeContactDampingModel(
    restitution: f32,
    policy: ContactDampingPolicy,
) ContactDampingModel {
    const clamped_restitution = @max(0.0, @min(1.0, restitution));
    const raw_scale = 1.0 + (policy.neutral_restitution - clamped_restitution) * policy.response_scale;
    return .{
        .restitution = clamped_restitution,
        .scale = @max(policy.min_scale, @min(policy.max_scale, raw_scale)),
    };
}

fn contactFatiguePolicy() ContactFatiguePolicy {
    return .{};
}

fn neutralContactFatigueModel() ContactFatigueModel {
    return .{
        .damage_modifier = contactFatiguePolicy().neutral_damage_modifier,
        .penetration_resistance = contactStiffnessPolicy().neutral_penetration_resistance,
        .stress = 0.0,
        .single_cycle_damage = 0.0,
        .accumulated_damage = 0.0,
        .remaining_life_fraction = 1.0,
        .failed = false,
    };
}

fn computeContactFatigueModel(
    input: ContactFatigueInput,
    policy: ContactFatiguePolicy,
) ContactFatigueModel {
    const penetration = @max(0.0, input.penetration_depth);
    const normal_speed = @max(0.0, input.relative_normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const damage_modifier = @max(0.0, input.damage_modifier);
    const resistance = @max(0.0, @min(1.0, input.penetration_resistance));
    const repeat_count = @max(@as(u32, 1), input.repeat_count);

    const stress = penetration * policy.penetration_weight +
        normal_speed * policy.normal_speed_weight +
        tangential_speed * policy.tangential_speed_weight;
    const raw_damage_scale = 1.0 +
        (damage_modifier - policy.neutral_damage_modifier) * policy.damage_response_scale +
        (1.0 - resistance) * policy.resistance_response_scale;
    const damage_scale = @max(policy.min_damage_scale, @min(policy.max_damage_scale, raw_damage_scale));
    const single_cycle_damage = @max(0.0, stress * policy.base_cycle_damage * damage_scale);
    const accumulated_damage = @min(
        policy.max_accumulated_damage,
        single_cycle_damage * @as(f32, @floatFromInt(repeat_count)),
    );

    return .{
        .damage_modifier = damage_modifier,
        .penetration_resistance = resistance,
        .stress = stress,
        .single_cycle_damage = single_cycle_damage,
        .accumulated_damage = accumulated_damage,
        .remaining_life_fraction = if (policy.max_accumulated_damage > 0.0)
            @max(0.0, (policy.max_accumulated_damage - accumulated_damage) / policy.max_accumulated_damage)
        else
            0.0,
        .failed = accumulated_damage >= policy.max_accumulated_damage,
    };
}

fn mediumPostBuoyancyPolicy() MediumPostBuoyancyPolicy {
    return .{};
}

fn neutralMediumPostBuoyancyModel() MediumPostBuoyancyModel {
    return .{
        .submerged_fraction = 0.0,
        .displaced_volume = 0.0,
        .magnitude = 0.0,
        .center_x = 0.0,
        .center_y = 0.0,
        .center_z = 0.0,
        .force = 0.0,
        .velocity_delta_y = 0.0,
        .active = false,
    };
}

fn computeMediumPostBuoyancyModel(
    input: MediumPostBuoyancyInput,
    policy: MediumPostBuoyancyPolicy,
) MediumPostBuoyancyModel {
    if (input.medium_type != .liquid) return neutralMediumPostBuoyancyModel();

    const buoyancy = @max(0.0, @min(1.0, input.buoyancy));
    const mass = @max(0.0, input.mass);
    if (buoyancy <= 0.0 or mass <= 0.0) return neutralMediumPostBuoyancyModel();

    const submerged_fraction = @max(
        0.0,
        @min(1.0, input.penetration_depth / @max(0.0001, policy.full_submerged_depth)),
    );
    if (submerged_fraction <= 0.0) return neutralMediumPostBuoyancyModel();

    const displaced_volume = submerged_fraction * policy.entity_volume;
    const magnitude = buoyancy * displaced_volume * policy.force_scale;
    const force = magnitude;
    const raw_velocity_delta = force * policy.mass_reference / mass;
    const velocity_delta_y = @min(policy.max_velocity_delta, raw_velocity_delta);
    const center_offset = input.penetration_depth * submerged_fraction * policy.center_depth_scale;

    return .{
        .submerged_fraction = submerged_fraction,
        .displaced_volume = displaced_volume,
        .magnitude = magnitude,
        .center_x = input.center_x - input.normal_x * center_offset,
        .center_y = input.center_y - input.normal_y * center_offset,
        .center_z = input.center_z - input.normal_z * center_offset,
        .force = force,
        .velocity_delta_y = velocity_delta_y,
        .active = velocity_delta_y > 0.0,
    };
}

fn sinkDepthPolicy() SinkDepthPolicy {
    return .{};
}

fn neutralSinkDepthModel() SinkDepthModel {
    return .{
        .depth = 0.0,
        .load = 0.0,
        .support_fraction = 1.0,
        .active = false,
    };
}

fn sinkDepthMediumScale(medium_type: terrain.MediumType, policy: SinkDepthPolicy) f32 {
    return switch (medium_type) {
        .solid => 0.0,
        .soft => policy.soft_depth_scale,
        .liquid => policy.liquid_depth_scale,
        .vapor => 0.0,
        .plasma => 0.0,
    };
}

fn computeSinkDepthModel(input: SinkDepthInput, policy: SinkDepthPolicy) SinkDepthModel {
    const medium_scale = sinkDepthMediumScale(input.medium_type, policy);
    if (medium_scale <= 0.0) return neutralSinkDepthModel();

    const penetration = @max(0.0, input.penetration_depth);
    const normal_speed = @max(0.0, input.relative_normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const mass = @max(0.0, input.mass);
    const resistance = @max(0.0, @min(1.0, input.penetration_resistance));
    const buoyancy = @max(0.0, @min(1.0, input.buoyancy));
    const load = penetration * policy.penetration_weight +
        normal_speed * policy.normal_speed_weight +
        tangential_speed * policy.tangential_speed_weight +
        mass * policy.mass_weight;
    if (load <= 0.0) return neutralSinkDepthModel();

    const resistance_scale = 1.0 - resistance * policy.resistance_reduction_scale;
    const buoyancy_scale = 1.0 - buoyancy * policy.buoyancy_reduction_scale;
    const depth = @max(0.0, @min(policy.max_depth, load * medium_scale * resistance_scale * buoyancy_scale));

    return .{
        .depth = depth,
        .load = load,
        .support_fraction = if (policy.max_depth > 0.0) @max(0.0, 1.0 - depth / policy.max_depth) else 1.0,
        .active = depth > 0.0,
    };
}

fn sinkResistancePolicy() SinkResistancePolicy {
    return .{};
}

fn neutralSinkResistanceModel() SinkResistanceModel {
    return .{
        .force = 0.0,
        .normal_velocity_delta = 0.0,
        .tangential_velocity_delta = 0.0,
        .active = false,
    };
}

fn computeSinkResistanceModel(input: SinkResistanceInput, policy: SinkResistancePolicy) SinkResistanceModel {
    if (!input.sink.active or input.sink.depth <= 0.0) return neutralSinkResistanceModel();

    const normal_speed = @max(0.0, input.relative_normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const support_loss = @max(0.0, 1.0 - @max(0.0, @min(1.0, input.sink.support_fraction)));
    const raw_force =
        input.sink.depth * policy.depth_force_scale +
        input.sink.load * policy.load_force_scale +
        support_loss * policy.support_loss_force_scale;
    const force = @max(0.0, @min(policy.max_force, raw_force));
    const normal_delta = @min(policy.max_velocity_delta, normal_speed * support_loss * policy.normal_damping_scale + force * 0.05);
    const tangent_delta = @min(policy.max_velocity_delta, tangential_speed * support_loss * policy.tangential_damping_scale + force * 0.03);

    return .{
        .force = force,
        .normal_velocity_delta = normal_delta,
        .tangential_velocity_delta = tangent_delta,
        .active = force > 0.0 or normal_delta > 0.0 or tangent_delta > 0.0,
    };
}

fn sinkRecoveryPolicy() SinkRecoveryPolicy {
    return .{};
}

fn neutralSinkRecoveryModel() SinkRecoveryModel {
    return .{
        .depth = 0.0,
        .fraction = 1.0,
        .rate = 0.0,
        .active = false,
        .recovered = true,
    };
}

fn computeSinkRecoveryModel(input: SinkRecoveryInput, policy: SinkRecoveryPolicy) SinkRecoveryModel {
    if (!input.sink.active or input.sink.depth <= 0.0) return neutralSinkRecoveryModel();

    const support = @max(0.0, @min(1.0, input.sink.support_fraction));
    const normal_speed = @max(0.0, input.relative_normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const motion_penalty =
        normal_speed * policy.normal_motion_penalty_scale +
        tangential_speed * policy.tangential_motion_penalty_scale;
    const resistance_penalty = @max(0.0, input.resistance.force) * policy.resistance_penalty_scale;
    const raw_fraction = support * policy.support_recovery_scale - motion_penalty - resistance_penalty;
    const fraction = @max(0.0, @min(1.0, raw_fraction));
    const depth = input.sink.depth * fraction;
    const rate = @min(policy.max_rate, depth * policy.rate_scale);
    const recovered = input.sink.depth - depth <= policy.recovered_epsilon;

    return .{
        .depth = depth,
        .fraction = fraction,
        .rate = rate,
        .active = rate > 0.0,
        .recovered = recovered,
    };
}

fn mediumPostDragPolicy() MediumPostDragPolicy {
    return .{};
}

fn neutralMediumPostDragModel() MediumPostDragModel {
    return .{
        .coefficient = 0.0,
        .exposure = 0.0,
        .magnitude = 0.0,
        .force = 0.0,
        .normal_velocity_delta = 0.0,
        .tangential_velocity_delta = 0.0,
        .active = false,
    };
}

fn mediumPostDragCoefficient(medium_type: terrain.MediumType, policy: MediumPostDragPolicy) f32 {
    return switch (medium_type) {
        .solid => 0.0,
        .soft => policy.soft_drag,
        .liquid => policy.liquid_drag,
        .vapor => policy.vapor_drag,
        .plasma => policy.plasma_drag,
    };
}

fn computeMediumPostDragModel(
    input: MediumPostDragInput,
    policy: MediumPostDragPolicy,
) MediumPostDragModel {
    const coefficient = mediumPostDragCoefficient(input.medium_type, policy);
    if (coefficient <= 0.0) return neutralMediumPostDragModel();

    const penetration_factor = @min(1.0, @max(0.0, input.penetration_depth)) * policy.penetration_weight;
    const buoyancy_factor = @min(1.0, @max(0.0, input.buoyancy)) * policy.buoyancy_weight;
    const exposure = @min(1.0, penetration_factor + buoyancy_factor);
    if (exposure <= 0.0) return neutralMediumPostDragModel();

    const normal_speed = @max(0.0, input.normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const magnitude = coefficient * exposure * (normal_speed + tangential_speed * policy.tangential_scale);
    const force = magnitude;
    const normal_delta = @min(policy.max_velocity_delta, normal_speed * coefficient * exposure);
    const tangent_delta = @min(policy.max_velocity_delta, tangential_speed * coefficient * exposure * policy.tangential_scale);

    return .{
        .coefficient = coefficient,
        .exposure = exposure,
        .magnitude = magnitude,
        .force = force,
        .normal_velocity_delta = normal_delta,
        .tangential_velocity_delta = tangent_delta,
        .active = force > 0.0,
    };
}

fn mediumPostVaporResistancePolicy() MediumPostVaporResistancePolicy {
    return .{};
}

fn neutralMediumPostVaporResistanceModel() MediumPostVaporResistanceModel {
    return .{
        .coefficient = 0.0,
        .exposure = 0.0,
        .dynamic_pressure = 0.0,
        .force = 0.0,
        .normal_velocity_delta = 0.0,
        .tangential_velocity_delta = 0.0,
        .active = false,
    };
}

fn computeMediumPostVaporResistanceModel(
    input: MediumPostVaporResistanceInput,
    policy: MediumPostVaporResistancePolicy,
) MediumPostVaporResistanceModel {
    if (input.medium_type != .vapor) return neutralMediumPostVaporResistanceModel();

    const normal_speed = @max(0.0, input.normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const speed = normal_speed + tangential_speed * policy.tangential_scale;
    const penetration_factor = @min(1.0, @max(0.0, input.penetration_depth)) * policy.penetration_weight;
    const speed_factor = @min(1.0, speed * policy.speed_weight);
    const exposure = @min(1.0, penetration_factor + speed_factor);
    if (exposure <= 0.0) return neutralMediumPostVaporResistanceModel();

    const dynamic_pressure = speed * speed * policy.pressure_scale * exposure;
    const force = dynamic_pressure * policy.coefficient + @max(0.0, input.drag.force) * policy.drag_coupling;
    const normal_delta = @min(policy.max_velocity_delta, normal_speed * policy.coefficient * exposure);
    const tangent_delta = @min(policy.max_velocity_delta, tangential_speed * policy.coefficient * exposure * policy.tangential_scale);

    return .{
        .coefficient = policy.coefficient,
        .exposure = exposure,
        .dynamic_pressure = dynamic_pressure,
        .force = force,
        .normal_velocity_delta = normal_delta,
        .tangential_velocity_delta = tangent_delta,
        .active = force > 0.0,
    };
}

fn mediumPostVacuumPolicy() MediumPostVacuumPolicy {
    return .{};
}

fn neutralMediumPostVacuumModel() MediumPostVacuumModel {
    return .{
        .pressure = 1.0,
        .exposure = 0.0,
        .drag_loss = 0.0,
        .thermal_isolation = 0.0,
        .sound_attenuation = 0.0,
        .active = false,
    };
}

fn mediumPostAmbientPressure(medium_type: terrain.MediumType) f32 {
    return switch (medium_type) {
        .solid => 1.0,
        .soft => 1.0,
        .liquid => 1.0,
        .vapor => 1.0,
        .plasma => 1.0,
    };
}

fn computeMediumPostVacuumModel(
    input: MediumPostVacuumInput,
    policy: MediumPostVacuumPolicy,
) MediumPostVacuumModel {
    _ = input.medium_type;
    const pressure = @max(0.0, input.ambient_pressure);
    if (pressure >= policy.near_vacuum_pressure) return neutralMediumPostVacuumModel();

    const pressure_span = @max(0.0001, policy.near_vacuum_pressure - policy.vacuum_pressure_threshold);
    const exposure = if (pressure <= policy.vacuum_pressure_threshold)
        1.0
    else
        @max(0.0, @min(1.0, (policy.near_vacuum_pressure - pressure) / pressure_span));
    if (exposure <= 0.0) return neutralMediumPostVacuumModel();

    const drag_loss = (@max(0.0, input.drag.force) + @max(0.0, input.vapor_resistance.force)) * exposure * policy.drag_loss_scale;
    const thermal_isolation = @min(1.0, exposure * policy.thermal_isolation_scale);
    const sound_attenuation = @min(1.0, exposure * policy.sound_attenuation_scale);

    return .{
        .pressure = pressure,
        .exposure = exposure,
        .drag_loss = drag_loss,
        .thermal_isolation = thermal_isolation,
        .sound_attenuation = sound_attenuation,
        .active = true,
    };
}

fn mediumPostTransitionPolicy() MediumPostTransitionPolicy {
    return .{};
}

fn neutralMediumPostTransitionModel() MediumPostTransitionModel {
    return .{
        .from_medium = .solid,
        .to_medium = .solid,
        .progress = 0.0,
        .resistance = 0.0,
        .pressure_delta = 0.0,
        .active = false,
    };
}

fn mediumTransitionPressure(medium_type: terrain.MediumType, vacuum: MediumPostVacuumModel) f32 {
    return if (vacuum.active) vacuum.pressure else mediumPostAmbientPressure(medium_type);
}

fn computeMediumPostTransitionModel(
    input: MediumPostTransitionInput,
    policy: MediumPostTransitionPolicy,
) MediumPostTransitionModel {
    if (input.from_medium == input.to_medium) return neutralMediumPostTransitionModel();

    const penetration_factor = @min(1.0, @max(0.0, input.penetration_depth)) * policy.penetration_weight;
    const response_factor = @min(1.0, @max(0.0, input.buoyancy)) * policy.response_weight;
    const coupling_factor = @min(
        1.0,
        @max(0.0, input.drag.exposure) +
            @max(0.0, input.vapor_resistance.exposure) +
            @max(0.0, input.vacuum.exposure),
    ) * policy.coupling_weight;
    const progress = @min(1.0, penetration_factor + response_factor + coupling_factor);
    if (progress <= 0.0) return neutralMediumPostTransitionModel();

    const from_pressure = mediumTransitionPressure(input.from_medium, input.vacuum);
    const to_pressure = mediumTransitionPressure(input.to_medium, input.vacuum);
    const pressure_delta = @abs(to_pressure - from_pressure);
    const resistance =
        @max(0.0, input.drag.force) * policy.drag_weight +
        @max(0.0, input.vapor_resistance.force) * policy.vapor_weight +
        @max(0.0, input.vacuum.drag_loss) * policy.vacuum_weight +
        pressure_delta * policy.pressure_weight;

    return .{
        .from_medium = input.from_medium,
        .to_medium = input.to_medium,
        .progress = progress,
        .resistance = resistance,
        .pressure_delta = pressure_delta,
        .active = true,
    };
}

fn mediumPostMixingPolicy() MediumPostMixingPolicy {
    return .{};
}

fn neutralMediumPostMixingModel() MediumPostMixingModel {
    return .{
        .primary_medium = .solid,
        .secondary_medium = .solid,
        .mix_fraction = 0.0,
        .effective_density = 0.0,
        .effective_viscosity = 0.0,
        .blended_drag = 0.0,
        .blended_buoyancy = 0.0,
        .active = false,
    };
}

fn mediumPostMixingDensity(medium_type: terrain.MediumType, policy: MediumPostMixingPolicy) f32 {
    return switch (medium_type) {
        .solid => policy.solid_density,
        .soft => policy.soft_density,
        .liquid => policy.liquid_density,
        .vapor => policy.vapor_density,
        .plasma => policy.plasma_density,
    };
}

fn mediumPostMixingViscosity(medium_type: terrain.MediumType, policy: MediumPostMixingPolicy) f32 {
    return switch (medium_type) {
        .solid => policy.solid_viscosity,
        .soft => policy.soft_viscosity,
        .liquid => policy.liquid_viscosity,
        .vapor => policy.vapor_viscosity,
        .plasma => policy.plasma_viscosity,
    };
}

fn computeMediumPostMixingModel(
    input: MediumPostMixingInput,
    policy: MediumPostMixingPolicy,
) MediumPostMixingModel {
    if (input.primary_medium == input.secondary_medium) return neutralMediumPostMixingModel();
    if (!input.transition.active) return neutralMediumPostMixingModel();

    const mix_fraction = @min(1.0, @max(0.0, input.transition.progress));
    if (mix_fraction <= 0.0) return neutralMediumPostMixingModel();

    const primary_density = mediumPostMixingDensity(input.primary_medium, policy);
    const secondary_density = mediumPostMixingDensity(input.secondary_medium, policy);
    const primary_viscosity = mediumPostMixingViscosity(input.primary_medium, policy);
    const secondary_viscosity = mediumPostMixingViscosity(input.secondary_medium, policy);
    const inverse_fraction = 1.0 - mix_fraction;
    const effective_density = primary_density * inverse_fraction + secondary_density * mix_fraction;
    const effective_viscosity = primary_viscosity * inverse_fraction + secondary_viscosity * mix_fraction;
    const coupled_drag =
        @max(0.0, input.drag.force) * policy.drag_weight +
        @max(0.0, input.vapor_resistance.force) * policy.drag_weight +
        @max(0.0, input.transition.resistance) * policy.resistance_weight -
        @max(0.0, input.vacuum.drag_loss) * policy.resistance_weight;
    const blended_drag = @max(0.0, coupled_drag) * mix_fraction;
    const blended_buoyancy = @max(0.0, input.buoyancy) * effective_density * mix_fraction * policy.buoyancy_weight;

    return .{
        .primary_medium = input.primary_medium,
        .secondary_medium = input.secondary_medium,
        .mix_fraction = mix_fraction,
        .effective_density = effective_density,
        .effective_viscosity = effective_viscosity,
        .blended_drag = blended_drag,
        .blended_buoyancy = blended_buoyancy,
        .active = effective_density > 0.0 or effective_viscosity > 0.0 or blended_drag > 0.0 or blended_buoyancy > 0.0,
    };
}

fn mediumPostStateMachinePolicy() MediumPostStateMachinePolicy {
    return .{};
}

fn neutralMediumPostStateMachineModel() MediumPostStateMachineModel {
    return .{
        .state = .stable,
        .current_medium = .solid,
        .target_medium = .solid,
        .progress = 0.0,
        .stability = 1.0,
        .active = false,
    };
}

fn computeMediumPostStateMachineModel(
    input: MediumPostStateMachineInput,
    policy: MediumPostStateMachinePolicy,
) MediumPostStateMachineModel {
    const transition_progress = if (input.transition.active)
        @min(1.0, @max(0.0, input.transition.progress))
    else
        0.0;
    const mixing_progress = if (input.mixing.active)
        @min(1.0, @max(0.0, input.mixing.mix_fraction))
    else
        0.0;
    const vacuum_progress = if (input.vacuum.active)
        @min(1.0, @max(0.0, input.vacuum.exposure))
    else
        0.0;
    const progress = @max(transition_progress, @max(mixing_progress, vacuum_progress));
    if (progress <= 0.0 and input.current_medium == input.target_medium) return neutralMediumPostStateMachineModel();

    const state: MediumPostState = if (vacuum_progress >= policy.vacuum_isolation_threshold)
        .isolated
    else if (mixing_progress >= policy.mixing_threshold and mixing_progress < policy.settling_threshold)
        .mixing
    else if (transition_progress >= policy.settling_threshold or mixing_progress >= policy.settling_threshold)
        .settling
    else if (transition_progress >= policy.entering_threshold or input.current_medium != input.target_medium)
        .entering
    else
        .stable;

    const instability =
        mixing_progress * policy.mixing_stability_weight +
        transition_progress * policy.transition_stability_weight +
        vacuum_progress * policy.vacuum_stability_weight;
    const stability = @max(0.0, @min(1.0, 1.0 - instability));

    return .{
        .state = state,
        .current_medium = input.current_medium,
        .target_medium = input.target_medium,
        .progress = progress,
        .stability = stability,
        .active = state != .stable,
    };
}

fn mediumPostEventTriggerPolicy() MediumPostEventTriggerPolicy {
    return .{};
}

fn neutralMediumPostEventTriggerModel() MediumPostEventTriggerModel {
    return .{
        .event_type = .none,
        .source_medium = .solid,
        .target_medium = .solid,
        .intensity = 0.0,
        .priority = .LOW,
        .active = false,
    };
}

fn mediumPostEventTypeForState(state: MediumPostState) MediumPostEventType {
    return switch (state) {
        .stable => .none,
        .entering => .enter,
        .mixing => .mix,
        .settling => .settle,
        .isolated => .isolate,
    };
}

fn mediumPostEventPriority(intensity: f32, policy: MediumPostEventTriggerPolicy) bus.BusPriority {
    if (intensity >= policy.high_priority_intensity) return .HIGH;
    if (intensity >= policy.normal_priority_intensity) return .NORMAL;
    return .LOW;
}

fn computeMediumPostEventTriggerModel(
    input: MediumPostEventTriggerInput,
    policy: MediumPostEventTriggerPolicy,
) MediumPostEventTriggerModel {
    if (!input.state.active) return neutralMediumPostEventTriggerModel();

    const event_type = mediumPostEventTypeForState(input.state.state);
    if (event_type == .none) return neutralMediumPostEventTriggerModel();

    const progress = @min(1.0, @max(0.0, input.state.progress));
    if (progress < policy.min_progress) return neutralMediumPostEventTriggerModel();

    const instability = 1.0 - @min(1.0, @max(0.0, input.state.stability));
    const resistance_factor = @min(1.0, @max(0.0, input.transition.resistance) / @max(0.0001, policy.resistance_reference));
    const pressure_factor = @min(
        1.0,
        (@max(0.0, input.transition.pressure_delta) + @max(0.0, input.vacuum.exposure)) / @max(0.0001, policy.pressure_reference),
    );
    const mixing_factor = if (input.mixing.active) @min(1.0, @max(0.0, input.mixing.mix_fraction)) else 0.0;
    const raw_intensity =
        progress * policy.progress_weight +
        @max(instability, mixing_factor) * policy.instability_weight +
        resistance_factor * policy.resistance_weight +
        pressure_factor * policy.pressure_weight;
    const intensity = @min(1.0, @max(0.0, raw_intensity));
    if (intensity < policy.min_intensity) return neutralMediumPostEventTriggerModel();

    return .{
        .event_type = event_type,
        .source_medium = input.state.current_medium,
        .target_medium = input.state.target_medium,
        .intensity = intensity,
        .priority = mediumPostEventPriority(intensity, policy),
        .active = true,
    };
}

fn mediumPostTransitionAnimationPolicy() MediumPostTransitionAnimationPolicy {
    return .{};
}

fn neutralMediumPostTransitionAnimationModel() MediumPostTransitionAnimationModel {
    return .{
        .phase = 0.0,
        .blend = 0.0,
        .opacity = 0.0,
        .ripple = 0.0,
        .turbulence = 0.0,
        .color_shift = 0.0,
        .active = false,
    };
}

fn computeMediumPostTransitionAnimationModel(
    input: MediumPostTransitionAnimationInput,
    policy: MediumPostTransitionAnimationPolicy,
) MediumPostTransitionAnimationModel {
    const state_progress = if (input.state.active) @min(1.0, @max(0.0, input.state.progress)) else 0.0;
    const transition_progress = if (input.transition.active) @min(1.0, @max(0.0, input.transition.progress)) else 0.0;
    const mixing_progress = if (input.mixing.active) @min(1.0, @max(0.0, input.mixing.mix_fraction)) else 0.0;
    const vacuum_progress = if (input.vacuum.active) @min(1.0, @max(0.0, input.vacuum.exposure)) else 0.0;
    const event_intensity = if (input.event.active) @min(1.0, @max(0.0, input.event.intensity)) else 0.0;
    const progress = @max(state_progress, @max(transition_progress, @max(mixing_progress, vacuum_progress)));
    if (progress < policy.min_progress) return neutralMediumPostTransitionAnimationModel();

    const phase = @min(1.0, progress * policy.phase_scale + event_intensity * policy.event_phase_weight);
    const blend = @min(1.0, @max(transition_progress, mixing_progress));
    const opacity = @min(1.0, policy.opacity_min + progress * (1.0 - policy.opacity_min));
    const ripple = @min(1.0, (event_intensity + mixing_progress) * policy.ripple_weight);
    const turbulence = @min(
        1.0,
        (@max(0.0, input.transition.resistance) * 0.05 + @max(0.0, input.mixing.blended_drag) * 0.1 + event_intensity) *
            policy.turbulence_weight,
    );
    const color_shift = @min(
        1.0,
        blend * policy.color_shift_weight + vacuum_progress * policy.vacuum_fade_weight,
    );

    return .{
        .phase = phase,
        .blend = blend,
        .opacity = opacity,
        .ripple = ripple,
        .turbulence = turbulence,
        .color_shift = color_shift,
        .active = opacity > 0.0,
    };
}

fn mediumPostTowPolicy() MediumPostTowPolicy {
    return .{};
}

fn neutralMediumPostTowModel() MediumPostTowModel {
    return .{
        .force = 0.0,
        .velocity_delta_x = 0.0,
        .velocity_delta_y = 0.0,
        .velocity_delta_z = 0.0,
        .active = false,
    };
}

fn mediumPostTowCoefficient(medium_type: terrain.MediumType, policy: MediumPostTowPolicy) f32 {
    return switch (medium_type) {
        .solid => 0.0,
        .soft => policy.soft_tow,
        .liquid => policy.liquid_tow,
        .vapor => policy.vapor_tow,
        .plasma => policy.plasma_tow,
    };
}

fn computeMediumPostTowModel(
    input: MediumPostTowInput,
    policy: MediumPostTowPolicy,
) MediumPostTowModel {
    const coefficient = mediumPostTowCoefficient(input.medium_type, policy);
    if (coefficient <= 0.0) return neutralMediumPostTowModel();

    const flow_len = @sqrt(input.flow_x * input.flow_x + input.flow_y * input.flow_y + input.flow_z * input.flow_z);
    if (flow_len <= policy.min_flow_magnitude) return neutralMediumPostTowModel();

    const penetration_factor = @min(1.0, @max(0.0, input.penetration_depth)) * policy.penetration_weight;
    const buoyancy_factor = @min(1.0, @max(0.0, input.buoyancy)) * policy.buoyancy_weight;
    const exposure = @min(1.0, penetration_factor + buoyancy_factor);
    if (exposure <= 0.0) return neutralMediumPostTowModel();

    const coupled_drag = input.drag.tangential_velocity_delta * policy.drag_coupling;
    const delta_magnitude = @min(policy.max_velocity_delta, flow_len * coefficient * exposure + coupled_drag);
    const force = flow_len * coefficient * exposure + input.drag.force * policy.drag_coupling;
    const inv_len = 1.0 / flow_len;

    return .{
        .force = force,
        .velocity_delta_x = input.flow_x * inv_len * delta_magnitude,
        .velocity_delta_y = input.flow_y * inv_len * delta_magnitude,
        .velocity_delta_z = input.flow_z * inv_len * delta_magnitude,
        .active = delta_magnitude > 0.0,
    };
}

fn mediumPostLiftPolicy() MediumPostLiftPolicy {
    return .{};
}

fn neutralMediumPostLiftModel() MediumPostLiftModel {
    return .{
        .coefficient = 0.0,
        .exposure = 0.0,
        .dynamic_pressure = 0.0,
        .magnitude = 0.0,
        .force = 0.0,
        .velocity_delta_x = 0.0,
        .velocity_delta_y = 0.0,
        .velocity_delta_z = 0.0,
        .active = false,
    };
}

fn mediumPostLiftCoefficient(medium_type: terrain.MediumType, policy: MediumPostLiftPolicy) f32 {
    return switch (medium_type) {
        .solid => 0.0,
        .soft => policy.soft_lift,
        .liquid => policy.liquid_lift,
        .vapor => policy.vapor_lift,
        .plasma => policy.plasma_lift,
    };
}

fn computeMediumPostLiftModel(
    input: MediumPostLiftInput,
    policy: MediumPostLiftPolicy,
) MediumPostLiftModel {
    const coefficient = mediumPostLiftCoefficient(input.medium_type, policy);
    if (coefficient <= 0.0) return neutralMediumPostLiftModel();

    const flow_len = @sqrt(input.flow_x * input.flow_x + input.flow_y * input.flow_y + input.flow_z * input.flow_z);
    if (flow_len <= policy.min_flow_magnitude) return neutralMediumPostLiftModel();

    const normal = normalizeContactVector(input.normal_x, input.normal_y, input.normal_z) orelse return neutralMediumPostLiftModel();
    const penetration_factor = @min(1.0, @max(0.0, input.penetration_depth)) * policy.penetration_weight;
    const buoyancy_factor = @min(1.0, @max(0.0, input.buoyancy)) * policy.buoyancy_weight;
    const exposure = @min(1.0, penetration_factor + buoyancy_factor);
    if (exposure <= 0.0) return neutralMediumPostLiftModel();

    const flow_normal_alignment = @abs(contactDot(
        input.flow_x / flow_len,
        input.flow_y / flow_len,
        input.flow_z / flow_len,
        normal.x,
        normal.y,
        normal.z,
    ));
    const dynamic_pressure = flow_len * flow_len * coefficient * exposure;
    const coupled_tow = input.tow.force * policy.tow_coupling;
    const magnitude = dynamic_pressure * (0.5 + flow_normal_alignment) + coupled_tow;
    const force = magnitude;
    const delta_magnitude = @min(policy.max_velocity_delta, force);

    return .{
        .coefficient = coefficient,
        .exposure = exposure,
        .dynamic_pressure = dynamic_pressure,
        .magnitude = magnitude,
        .force = force,
        .velocity_delta_x = normal.x * delta_magnitude,
        .velocity_delta_y = normal.y * delta_magnitude,
        .velocity_delta_z = normal.z * delta_magnitude,
        .active = delta_magnitude > 0.0,
    };
}

fn mediumPostAddedMassPolicy() MediumPostAddedMassPolicy {
    return .{};
}

fn neutralMediumPostAddedMassModel() MediumPostAddedMassModel {
    return .{
        .coefficient = 0.0,
        .exposure = 0.0,
        .displaced_volume = 0.0,
        .added_mass = 0.0,
        .effective_mass = 0.0,
        .normal_inertia_scale = 1.0,
        .tangential_inertia_scale = 1.0,
        .active = false,
    };
}

fn mediumPostAddedMassCoefficient(medium_type: terrain.MediumType, policy: MediumPostAddedMassPolicy) f32 {
    return switch (medium_type) {
        .solid => 0.0,
        .soft => policy.soft_added_mass,
        .liquid => policy.liquid_added_mass,
        .vapor => policy.vapor_added_mass,
        .plasma => policy.plasma_added_mass,
    };
}

fn computeMediumPostAddedMassModel(
    input: MediumPostAddedMassInput,
    policy: MediumPostAddedMassPolicy,
) MediumPostAddedMassModel {
    const coefficient = mediumPostAddedMassCoefficient(input.medium_type, policy);
    if (coefficient <= 0.0) return neutralMediumPostAddedMassModel();

    const base_mass = @max(0.0, input.base_mass);
    if (base_mass <= 0.0) return neutralMediumPostAddedMassModel();

    const penetration_factor = @min(1.0, @max(0.0, input.penetration_depth)) * policy.penetration_weight;
    const buoyancy_factor = @min(1.0, @max(0.0, input.buoyancy)) * policy.buoyancy_weight;
    const drag_factor = @min(1.0, @max(0.0, input.drag.exposure)) * policy.drag_weight;
    const lift_factor = @min(1.0, @max(0.0, input.lift.exposure)) * policy.lift_weight;
    const exposure = @min(1.0, penetration_factor + buoyancy_factor + drag_factor + lift_factor);
    if (exposure <= 0.0) return neutralMediumPostAddedMassModel();

    const fallback_volume = mediumPostBuoyancyPolicy().entity_volume * @min(1.0, @max(0.0, input.penetration_depth));
    const displaced_volume = if (input.buoyancy_model.displaced_volume > 0.0)
        input.buoyancy_model.displaced_volume
    else
        fallback_volume;
    if (displaced_volume <= 0.0) return neutralMediumPostAddedMassModel();

    const added_mass = displaced_volume * policy.displaced_volume_mass_scale * coefficient * exposure;
    if (added_mass <= 0.0) return neutralMediumPostAddedMassModel();

    const inertia_ratio = @min(policy.max_inertia_ratio, added_mass / @max(0.0001, base_mass));
    return .{
        .coefficient = coefficient,
        .exposure = exposure,
        .displaced_volume = displaced_volume,
        .added_mass = added_mass,
        .effective_mass = base_mass + added_mass,
        .normal_inertia_scale = 1.0 + inertia_ratio * policy.normal_inertia_weight,
        .tangential_inertia_scale = 1.0 + inertia_ratio * policy.tangential_inertia_weight,
        .active = true,
    };
}

fn mediumPostThermalPolicy() MediumPostThermalPolicy {
    return .{};
}

fn neutralMediumPostThermalModel() MediumPostThermalModel {
    return .{
        .conducted_heat = 0.0,
        .conductivity = 0.0,
        .retained_heat = 0.0,
        .temperature_delta = 0.0,
        .active = false,
    };
}

fn mediumPostThermalConductivityScale(medium_type: terrain.MediumType, policy: MediumPostThermalPolicy) f32 {
    return switch (medium_type) {
        .solid => 0.0,
        .soft => policy.soft_conduction_scale,
        .liquid => policy.liquid_conduction_scale,
        .vapor => policy.vapor_conduction_scale,
        .plasma => policy.plasma_conduction_scale,
    };
}

fn computeMediumPostThermalModel(
    input: MediumPostThermalInput,
    policy: MediumPostThermalPolicy,
) MediumPostThermalModel {
    const medium_scale = mediumPostThermalConductivityScale(input.medium_type, policy);
    if (medium_scale <= 0.0) return neutralMediumPostThermalModel();

    const source_heat = @max(0.0, input.thermal.retained_heat);
    const mechanical_heat =
        @max(0.0, input.drag.force) * policy.drag_heat_weight +
        @max(0.0, input.tow.force) * policy.tow_heat_weight +
        @max(0.0, input.lift.force) * policy.lift_heat_weight;
    const available_heat = source_heat + mechanical_heat;
    if (available_heat <= 0.0) return neutralMediumPostThermalModel();

    const buoyancy = @max(0.0, @min(1.0, input.buoyancy));
    const conductivity = @min(1.0, medium_scale + buoyancy * policy.buoyancy_cooling_weight);
    const conducted_heat = @min(policy.max_conducted_heat, available_heat * conductivity);
    const retained_heat = @max(0.0, available_heat - conducted_heat);

    return .{
        .conducted_heat = conducted_heat,
        .conductivity = conductivity,
        .retained_heat = retained_heat,
        .temperature_delta = retained_heat,
        .active = conducted_heat > 0.0,
    };
}

fn contactThermalPolicy() ContactThermalPolicy {
    return .{};
}

fn neutralContactThermalModel() ContactThermalModel {
    return .{
        .generated_heat = 0.0,
        .conductivity = contactThermalPolicy().solid_conductivity,
        .dissipated_heat = 0.0,
        .retained_heat = 0.0,
        .temperature_delta = 0.0,
    };
}

fn contactMediumBaseConductivity(medium_type: terrain.MediumType, policy: ContactThermalPolicy) f32 {
    return switch (medium_type) {
        .solid => policy.solid_conductivity,
        .soft => policy.soft_conductivity,
        .liquid => policy.liquid_conductivity,
        .vapor => policy.vapor_conductivity,
        .plasma => policy.plasma_conductivity,
    };
}

fn computeContactFrictionHeat(input: ContactThermalInput, policy: ContactThermalPolicy) f32 {
    const tangential_speed = @max(0.0, input.tangential_speed);
    const friction = @max(0.0, input.friction);
    const rolling_friction = @max(0.0, input.rolling_friction);
    const angular_speed = @max(0.0, input.angular_speed);
    const anisotropic_scale = @max(0.0, @min(1.0, input.anisotropic_minor_axis_scale));
    const anisotropic_resistance = 1.0 - anisotropic_scale;

    const sliding_heat = tangential_speed * friction * policy.friction_heat_weight;
    const rolling_heat = angular_speed * rolling_friction * policy.rolling_friction_heat_weight;
    const anisotropic_heat = sliding_heat * anisotropic_resistance * @max(0.0, policy.anisotropic_friction_heat_scale);
    return @min(@max(0.0, policy.max_friction_heat), sliding_heat + rolling_heat + anisotropic_heat);
}

fn computeContactThermalModel(
    input: ContactThermalInput,
    policy: ContactThermalPolicy,
) ContactThermalModel {
    const penetration = @max(0.0, input.penetration_depth);
    const normal_speed = @max(0.0, input.relative_normal_speed);
    const resistance = @max(0.0, @min(1.0, input.penetration_resistance));
    const buoyancy = @max(0.0, @min(1.0, input.buoyancy));

    const normal_heat = normal_speed * normal_speed * policy.normal_heat_weight;
    const friction_heat = computeContactFrictionHeat(input, policy);
    const penetration_heat = penetration * (1.0 + resistance) * policy.penetration_heat_weight;
    const generated_heat = normal_heat + friction_heat + penetration_heat;
    const friction_heat_fraction = if (generated_heat <= 0.0001) 0.0 else friction_heat / generated_heat;
    const base_conductivity = contactMediumBaseConductivity(input.medium_type, policy);
    const raw_conductivity = base_conductivity *
        (1.0 + resistance * policy.resistance_conductivity_scale) *
        (1.0 + buoyancy * policy.buoyancy_cooling_scale);
    const conductivity = @max(policy.min_conductivity, @min(policy.max_conductivity, raw_conductivity));
    const dissipated_heat = generated_heat * @min(1.0, conductivity * 0.5);
    const retained_heat = @max(0.0, generated_heat - dissipated_heat);

    return .{
        .generated_heat = generated_heat,
        .friction_heat = friction_heat,
        .friction_heat_fraction = friction_heat_fraction,
        .conductivity = conductivity,
        .dissipated_heat = dissipated_heat,
        .retained_heat = retained_heat,
        .temperature_delta = retained_heat,
    };
}

fn contactSoundPolicy() ContactSoundPolicy {
    return .{};
}

fn neutralContactSoundModel() ContactSoundModel {
    return .{
        .sound_type = 0,
        .volume = 0.0,
        .pitch = contactSoundPolicy().base_pitch,
        .duration = 0.0,
        .audible = false,
    };
}

fn contactMediumVolumeScale(medium_type: terrain.MediumType, policy: ContactSoundPolicy) f32 {
    return switch (medium_type) {
        .solid => 1.0,
        .soft => policy.soft_volume_scale,
        .liquid => policy.liquid_volume_scale,
        .vapor => policy.vapor_volume_scale,
        .plasma => policy.plasma_volume_scale,
    };
}

fn computeContactSoundModel(
    input: ContactSoundInput,
    policy: ContactSoundPolicy,
) ContactSoundModel {
    const penetration = @max(0.0, input.penetration_depth);
    const normal_speed = @max(0.0, input.relative_normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const friction = @max(0.0, input.friction);
    const restitution = @max(0.0, @min(1.0, input.restitution));
    const medium_scale = contactMediumVolumeScale(input.medium_type, policy);

    const raw_volume = (normal_speed * policy.normal_volume_weight +
        tangential_speed * friction * policy.tangential_volume_weight +
        penetration * policy.penetration_volume_weight) * medium_scale;
    const volume = @max(policy.min_volume, @min(policy.max_volume, raw_volume));
    const raw_pitch = policy.base_pitch +
        restitution * policy.restitution_pitch_scale -
        friction * policy.friction_pitch_damping;
    const pitch = @max(policy.min_pitch, @min(policy.max_pitch, raw_pitch));
    const duration = @min(
        policy.max_duration,
        policy.base_duration +
            penetration * policy.duration_per_penetration +
            tangential_speed * policy.duration_per_tangent_speed,
    );

    return .{
        .sound_type = input.sound_type,
        .volume = volume,
        .pitch = pitch,
        .duration = if (volume > 0.0 and input.sound_type != 0) duration else 0.0,
        .audible = volume > 0.0 and input.sound_type != 0,
    };
}

fn contactDustPolicy() ContactDustPolicy {
    return .{};
}

fn neutralContactDustModel() ContactDustModel {
    return .{
        .dust_type = 0,
        .intensity = 0.0,
        .radius = 0.0,
        .duration = 0.0,
        .emitted = false,
    };
}

fn contactMediumDustScale(medium_type: terrain.MediumType, policy: ContactDustPolicy) f32 {
    return switch (medium_type) {
        .solid => 1.0,
        .soft => policy.soft_intensity_scale,
        .liquid => policy.liquid_intensity_scale,
        .vapor => policy.vapor_intensity_scale,
        .plasma => policy.plasma_intensity_scale,
    };
}

fn computeContactDustModel(
    input: ContactDustInput,
    policy: ContactDustPolicy,
) ContactDustModel {
    const penetration = @max(0.0, input.penetration_depth);
    const normal_speed = @max(0.0, input.relative_normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const friction = @max(0.0, input.friction);
    const buoyancy = @max(0.0, @min(1.0, input.buoyancy));
    const medium_scale = contactMediumDustScale(input.medium_type, policy);
    const buoyancy_scale = 1.0 - buoyancy * policy.buoyancy_suppression_scale;

    const raw_intensity = (normal_speed * policy.normal_intensity_weight +
        tangential_speed * friction * policy.tangential_intensity_weight +
        penetration * policy.penetration_intensity_weight) * medium_scale * buoyancy_scale;
    const intensity = @max(policy.min_intensity, @min(policy.max_intensity, raw_intensity));
    const emitted = input.dust_type != 0 and intensity > 0.0;
    const radius = if (emitted)
        @min(policy.max_radius, policy.base_radius + intensity * policy.radius_per_intensity + tangential_speed * policy.radius_per_tangent_speed)
    else
        0.0;
    const duration = if (emitted)
        @min(policy.max_duration, policy.base_duration + intensity * policy.duration_per_intensity)
    else
        0.0;

    return .{
        .dust_type = input.dust_type,
        .intensity = intensity,
        .radius = radius,
        .duration = duration,
        .emitted = emitted,
    };
}

fn contactDeformationPolicy() ContactDeformationPolicy {
    return .{};
}

fn neutralContactDeformationModel() ContactDeformationModel {
    return .{
        .total_depth = 0.0,
        .elastic_depth = 0.0,
        .permanent_depth = 0.0,
        .recovery_fraction = 1.0,
        .severe = false,
    };
}

fn computeContactDeformationModel(
    input: ContactDeformationInput,
    policy: ContactDeformationPolicy,
) ContactDeformationModel {
    const penetration = @max(0.0, input.penetration_depth);
    const normal_speed = @max(0.0, input.relative_normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const resistance = @max(0.0, @min(1.0, input.penetration_resistance));
    const damage_modifier = @max(0.0, input.damage_modifier);
    const restitution = @max(0.0, @min(1.0, input.restitution));

    const load = penetration * policy.penetration_weight +
        normal_speed * policy.normal_speed_weight +
        tangential_speed * policy.tangential_speed_weight;
    const resistance_scale = 1.0 - resistance * policy.resistance_reduction_scale;
    const damage_scale = 1.0 + @max(0.0, damage_modifier - 1.0) * policy.damage_amplification_scale;
    const total_depth = @max(policy.min_total, @min(policy.max_total, load * resistance_scale * damage_scale));
    const raw_permanent_fraction = policy.base_permanent_fraction +
        @max(0.0, damage_modifier - 1.0) * policy.damage_permanent_scale +
        (1.0 - restitution) * policy.restitution_recovery_scale;
    const permanent_fraction = @max(0.0, @min(policy.max_permanent_fraction, raw_permanent_fraction));
    const permanent_depth = total_depth * permanent_fraction;
    const elastic_depth = @max(0.0, total_depth - permanent_depth);

    return .{
        .total_depth = total_depth,
        .elastic_depth = elastic_depth,
        .permanent_depth = permanent_depth,
        .recovery_fraction = if (total_depth > 0.0) elastic_depth / total_depth else 1.0,
        .severe = permanent_fraction >= policy.max_permanent_fraction or total_depth >= policy.max_total,
    };
}

fn contactSeparationPolicy() ContactSeparationPolicy {
    return .{};
}

fn neutralContactSeparationModel() ContactSeparationModel {
    return .{
        .state = .persisting,
        .separating = false,
        .speed = 0.0,
        .estimated_time = 0.0,
    };
}

fn computeContactSeparationModel(
    input: ContactSeparationInput,
    policy: ContactSeparationPolicy,
) ContactSeparationModel {
    const penetration = @max(0.0, input.penetration_depth);
    const predicted_penetration = @max(0.0, input.predicted_penetration_depth);
    const signed_speed = input.normal_speed_signed;
    const threshold = @max(0.0, policy.velocity_epsilon);
    const slop = @max(0.0, policy.penetration_slop);

    if (signed_speed < -threshold) {
        return .{
            .state = .closing,
            .separating = false,
            .speed = -signed_speed,
            .estimated_time = 0.0,
        };
    }

    if (signed_speed > threshold and predicted_penetration <= penetration + slop) {
        const separation_time = if (penetration > slop)
            @min(policy.max_separation_time, penetration / signed_speed)
        else
            0.0;
        return .{
            .state = .separating,
            .separating = true,
            .speed = signed_speed,
            .estimated_time = separation_time,
        };
    }

    return .{
        .state = .persisting,
        .separating = false,
        .speed = if (signed_speed > 0.0) signed_speed else 0.0,
        .estimated_time = 0.0,
    };
}

fn contactStabilizationPolicy() ContactStabilizationPolicy {
    return .{};
}

fn neutralContactStabilizationModel() ContactStabilizationModel {
    return .{
        .bias_scale = 1.0,
        .impulse_scale = 1.0,
        .stabilized = false,
    };
}

fn clampContactStabilizationScale(value: f32, policy: ContactStabilizationPolicy) f32 {
    return @max(policy.min_scale, @min(policy.max_scale, value));
}

fn computeContactStabilizationModel(
    input: ContactStabilizationInput,
    policy: ContactStabilizationPolicy,
) ContactStabilizationModel {
    const penetration = @max(0.0, input.penetration_depth);
    const predicted = @max(0.0, input.predicted_penetration_depth);
    const shallow_contact = penetration <= policy.shallow_penetration_depth and predicted <= policy.shallow_penetration_depth;
    const resting = input.separation.speed <= policy.resting_speed_threshold;

    const raw_bias_scale = switch (input.separation.state) {
        .closing => policy.closing_bias_scale,
        .persisting => if (resting or shallow_contact) policy.persisting_bias_scale else 1.0,
        .separating => policy.separating_bias_scale,
    };
    const raw_impulse_scale = switch (input.separation.state) {
        .closing => policy.closing_impulse_scale,
        .persisting => if (resting or shallow_contact) policy.persisting_impulse_scale else 1.0,
        .separating => policy.separating_impulse_scale,
    };
    const bias_scale = clampContactStabilizationScale(raw_bias_scale, policy);
    const impulse_scale = clampContactStabilizationScale(raw_impulse_scale, policy);

    return .{
        .bias_scale = bias_scale,
        .impulse_scale = impulse_scale,
        .stabilized = bias_scale != 1.0 or impulse_scale != 1.0,
    };
}

fn applyContactStabilizationToEquation(
    equation: ConstraintRowEquation,
    stabilization: ContactStabilizationModel,
) ConstraintRowEquation {
    return .{
        .effective_mass = equation.effective_mass,
        .bias = equation.bias * stabilization.bias_scale,
        .max_impulse = @max(0.0, equation.max_impulse * stabilization.impulse_scale),
    };
}

fn damageEvalImpactPolicy() DamageEvalImpactPolicy {
    return .{};
}

fn neutralDamageEvalImpactModel() DamageEvalImpactModel {
    return .{
        .legacy_impact = 0,
        .normal_speed = 0.0,
        .tangential_speed = 0.0,
        .kinetic_energy = 0.0,
        .damage_amount = 0.0,
    };
}

fn damageEvalHardnessPolicy() DamageEvalHardnessPolicy {
    return .{};
}

fn neutralDamageEvalHardnessModel() DamageEvalHardnessModel {
    return .{
        .hardness = 0,
        .impact_threshold = 0.0,
        .impact_ratio = 0.0,
        .hardness_resistance = 0.0,
        .exceeds_hardness = false,
    };
}

fn neutralDamageEvalBreakModel() DamageEvalBreakModel {
    return .{
        .legacy_impact = 0,
        .impact_threshold = 0.0,
        .impact_ratio = 0.0,
        .did_break = false,
        .fragments = 0,
    };
}

fn inactiveDamageEvalFragments() [16]destruction.Fragment {
    var fragments: [16]destruction.Fragment = undefined;
    for (0..fragments.len) |i| {
        fragments[i] = .{
            .local_x = 0,
            .local_y = 0,
            .local_z = 0,
            .size = 0,
            .velocity_x = 0.0,
            .velocity_y = 0.0,
            .velocity_z = 0.0,
            .rotation_x = 0.0,
            .rotation_y = 0.0,
            .rotation_z = 0.0,
            .active = false,
        };
    }
    return fragments;
}

fn damageEvalFragmentPolicy() DamageEvalFragmentPolicy {
    return .{};
}

fn neutralDamageEvalFragmentModel() DamageEvalFragmentModel {
    return .{
        .requested_fragments = 0,
        .generated_fragments = 0,
        .fragment_energy = 0.0,
        .debris_mass = 0.0,
        .debris_speed = 0.0,
        .fragments = inactiveDamageEvalFragments(),
    };
}

fn inactiveDamageEvalFracturePattern() destruction.FracturePattern {
    var pattern: destruction.FracturePattern = undefined;
    pattern.seed = 0;
    pattern.crack_count = 0;
    pattern.fragment_count = 0;
    for (0..pattern.cracks.len) |i| {
        pattern.cracks[i] = .{
            .start_x = 0,
            .start_y = 0,
            .start_z = 0,
            .end_x = 0,
            .end_y = 0,
            .end_z = 0,
            .severity = 0.0,
        };
    }
    pattern.fragments = inactiveDamageEvalFragments();
    return pattern;
}

fn damageEvalCrackPolicy() DamageEvalCrackPolicy {
    return .{};
}

fn neutralDamageEvalCrackModel() DamageEvalCrackModel {
    return .{
        .propagated = false,
        .crack_count = 0,
        .max_severity = 0.0,
        .energy = 0.0,
        .pattern = inactiveDamageEvalFracturePattern(),
    };
}

fn damageEvalMaterialImpactScale(material: entity16.MaterialType) f32 {
    return switch (material) {
        .fragile => 3.0,
        .solid => 1.5,
        .elastic => 0.5,
        .composite => 1.0,
        else => 1.0,
    };
}

fn damageEvalHardnessThreshold(
    material: entity16.MaterialType,
    hardness: u16,
    policy: DamageEvalHardnessPolicy,
) f32 {
    return switch (material) {
        .fragile => policy.fragile_impact_threshold,
        .solid => @max(policy.min_impact_threshold, @as(f32, @floatFromInt(hardness))),
        else => @max(policy.min_impact_threshold, @as(f32, @floatFromInt(hardness))),
    };
}

fn computeDamageEvalImpactModel(
    input: DamageEvalImpactInput,
    policy: DamageEvalImpactPolicy,
) DamageEvalImpactModel {
    const normal_speed = @max(0.0, input.normal_speed);
    const tangential_speed = @max(0.0, input.tangential_speed);
    const mass = @max(0.0, input.mass);
    if (normal_speed <= 0.0 or mass <= 0.0) return neutralDamageEvalImpactModel();

    const raw_legacy_impact = @min(
        policy.max_legacy_impact,
        (normal_speed * mass) / @max(1.0, policy.legacy_mass_scale),
    );
    const speed_energy = normal_speed * normal_speed +
        tangential_speed * tangential_speed * policy.tangential_energy_scale;
    const kinetic_energy = 0.5 * mass * speed_energy;
    const hardness_factor = 1.0 - @min(1.0, @max(0.0, @as(f32, @floatFromInt(input.hardness)) / 255.0));
    const damage_amount = (kinetic_energy / @max(1.0, policy.energy_scale)) *
        damageEvalMaterialImpactScale(input.material) *
        @max(0.0, input.damage_modifier) *
        hardness_factor;

    return .{
        .legacy_impact = @intFromFloat(raw_legacy_impact),
        .normal_speed = normal_speed,
        .tangential_speed = tangential_speed,
        .kinetic_energy = kinetic_energy,
        .damage_amount = damage_amount,
    };
}

fn computeDamageEvalHardnessModel(
    input: DamageEvalHardnessInput,
    policy: DamageEvalHardnessPolicy,
) DamageEvalHardnessModel {
    const threshold = damageEvalHardnessThreshold(input.material, input.hardness, policy);
    const impact_value: f32 = @floatFromInt(input.impact.legacy_impact);
    const impact_ratio = if (threshold > 0.0) impact_value / threshold else 0.0;
    const hardness_resistance = @min(
        1.0,
        @max(0.0, @as(f32, @floatFromInt(input.hardness)) / @max(1.0, policy.max_hardness)),
    );

    return .{
        .hardness = input.hardness,
        .impact_threshold = threshold,
        .impact_ratio = impact_ratio,
        .hardness_resistance = hardness_resistance,
        .exceeds_hardness = impact_value >= threshold and threshold > 0.0,
    };
}

fn computeDamageEvalHardnessForEntity(
    entity: *const entity16.Entity16,
    impact: DamageEvalImpactModel,
) DamageEvalHardnessModel {
    return computeDamageEvalHardnessModel(
        .{
            .impact = impact,
            .material = entity.physics.material,
            .hardness = entity.physics.hardness,
        },
        damageEvalHardnessPolicy(),
    );
}

fn computeDamageEvalBreakModel(input: DamageEvalBreakInput) DamageEvalBreakModel {
    const break_result = physics.checkBreak(
        input.impact.legacy_impact,
        input.material,
        input.hardness.hardness,
    );

    return .{
        .legacy_impact = input.impact.legacy_impact,
        .impact_threshold = input.hardness.impact_threshold,
        .impact_ratio = input.hardness.impact_ratio,
        .did_break = break_result.did_break,
        .fragments = break_result.fragments,
    };
}

fn computeDamageEvalBreakForEntity(
    entity: *const entity16.Entity16,
    impact: DamageEvalImpactModel,
    hardness: DamageEvalHardnessModel,
) DamageEvalBreakModel {
    return computeDamageEvalBreakModel(.{
        .impact = impact,
        .hardness = hardness,
        .material = entity.physics.material,
    });
}

fn countActiveDamageEvalFragments(fragments: *const [16]destruction.Fragment, limit: u8) u8 {
    var count: u8 = 0;
    const clamped_limit = @min(@as(usize, limit), fragments.len);
    for (fragments[0..clamped_limit]) |fragment| {
        if (fragment.active and fragment.size > 0) count += 1;
    }
    return count;
}

fn computeDamageEvalFragmentModel(
    input: DamageEvalFragmentInput,
    policy: DamageEvalFragmentPolicy,
) DamageEvalFragmentModel {
    if (!input.break_model.did_break) return neutralDamageEvalFragmentModel();

    const requested_fragments = @max(
        policy.min_fragment_count,
        @min(policy.max_fragments, input.break_model.fragments),
    );
    const debris_speed = @as(f32, @floatFromInt(@abs(@as(i32, input.impact_velocity)))) *
        @max(0.0, input.speed_scale);
    const fragment_energy = @max(
        policy.min_fragment_energy,
        @max(debris_speed * policy.velocity_energy_scale, @as(f32, @floatFromInt(input.impact.legacy_impact))),
    );
    var fragments = destruction.generateFragments(
        input.entity,
        policy.impact_x,
        policy.impact_y,
        policy.impact_z,
        fragment_energy,
    );
    const generated_fragments = countActiveDamageEvalFragments(&fragments, requested_fragments);
    const divisor = @max(@as(u8, 1), generated_fragments);
    const debris_mass = @as(f32, @floatFromInt(input.entity.physics.mass)) /
        @as(f32, @floatFromInt(divisor));

    return .{
        .requested_fragments = requested_fragments,
        .generated_fragments = generated_fragments,
        .fragment_energy = fragment_energy,
        .debris_mass = debris_mass,
        .debris_speed = debris_speed,
        .fragments = fragments,
    };
}

fn computeDamageEvalFragmentsForEntity(
    entity: *const entity16.Entity16,
    impact: DamageEvalImpactModel,
    break_model: DamageEvalBreakModel,
    impact_velocity: i16,
    speed_scale: f32,
) DamageEvalFragmentModel {
    return computeDamageEvalFragmentModel(
        .{
            .entity = entity,
            .impact = impact,
            .break_model = break_model,
            .impact_velocity = impact_velocity,
            .speed_scale = speed_scale,
        },
        damageEvalFragmentPolicy(),
    );
}

fn damageEvalImpactVelocityHint(impact: DamageEvalImpactModel) i16 {
    const speed = @min(32767.0, @max(0.0, impact.normal_speed));
    return -@as(i16, @intFromFloat(speed));
}

fn spawnDamageEvalDebrisFromFragments(
    inst: *const scene32.Instance,
    fragments: *const DamageEvalFragmentModel,
) u8 {
    var spawned: u8 = 0;
    const limit = @min(@as(usize, fragments.requested_fragments), fragments.fragments.len);
    for (fragments.fragments[0..limit]) |fragment| {
        if (!fragment.active or fragment.size == 0) continue;
        if (destruction.spawnDebris(
            @floatFromInt(inst.pos_x),
            @floatFromInt(inst.pos_y),
            @floatFromInt(inst.pos_z),
            fragment.local_x,
            fragment.local_y,
            fragment.local_z,
            fragment.velocity_x,
            fragment.velocity_y,
            fragment.velocity_z,
            @floatFromInt(fragment.size),
            fragments.debris_mass,
        ) != null) {
            spawned += 1;
        }
    }
    return spawned;
}

fn maxDamageEvalCrackSeverity(pattern: *const destruction.FracturePattern) f32 {
    var max_severity: f32 = 0.0;
    for (pattern.cracks[0..pattern.crack_count]) |crack| {
        max_severity = @max(max_severity, crack.severity);
    }
    return max_severity;
}

fn computeDamageEvalCrackModel(
    input: DamageEvalCrackInput,
    policy: DamageEvalCrackPolicy,
) DamageEvalCrackModel {
    if (input.impact.legacy_impact == 0) return neutralDamageEvalCrackModel();
    if (!input.break_model.did_break and input.hardness.impact_ratio < policy.min_propagation_ratio) {
        return neutralDamageEvalCrackModel();
    }

    const base_energy = if (input.break_model.did_break)
        @max(input.fragments.fragment_energy, input.impact.kinetic_energy / 1000.0) * policy.break_energy_scale
    else
        input.impact.damage_amount * policy.sub_break_energy_scale;
    if (base_energy <= 0.0) return neutralDamageEvalCrackModel();

    var pattern = destruction.generateFracture(
        input.entity,
        policy.impact_x,
        policy.impact_y,
        policy.impact_z,
        base_energy,
        policy.seed +% @as(u32, input.impact.legacy_impact) +% rewind.getNextRandom(),
    );
    pattern.fragment_count = input.fragments.generated_fragments;
    pattern.fragments = input.fragments.fragments;

    return .{
        .propagated = pattern.crack_count > 0,
        .crack_count = pattern.crack_count,
        .max_severity = maxDamageEvalCrackSeverity(&pattern),
        .energy = base_energy,
        .pattern = pattern,
    };
}

fn computeDamageEvalCracksForEntity(
    entity: *const entity16.Entity16,
    impact: DamageEvalImpactModel,
    hardness: DamageEvalHardnessModel,
    break_model: DamageEvalBreakModel,
    fragments: DamageEvalFragmentModel,
) DamageEvalCrackModel {
    return computeDamageEvalCrackModel(
        .{
            .entity = entity,
            .impact = impact,
            .hardness = hardness,
            .break_model = break_model,
            .fragments = fragments,
        },
        damageEvalCrackPolicy(),
    );
}

fn computeDamageEvalImpactForEntity(
    entity: *const entity16.Entity16,
    normal_speed: f32,
    tangential_speed: f32,
    damage_modifier: f32,
) DamageEvalImpactModel {
    return computeDamageEvalImpactModel(
        .{
            .normal_speed = normal_speed,
            .tangential_speed = tangential_speed,
            .mass = @floatFromInt(entity.physics.mass),
            .material = entity.physics.material,
            .hardness = entity.physics.hardness,
            .damage_modifier = damage_modifier,
        },
        damageEvalImpactPolicy(),
    );
}

fn computeDamageEvalImpactFromVelocity(
    entity: *const entity16.Entity16,
    impact_velocity: i16,
) DamageEvalImpactModel {
    const normal_speed = if (impact_velocity < 0)
        @as(f32, @floatFromInt(-@as(i32, impact_velocity)))
    else
        0.0;
    return computeDamageEvalImpactForEntity(entity, normal_speed, 0.0, 1.0);
}

fn contactFrictionSolvePolicy() ContactFrictionSolvePolicy {
    return .{};
}

fn computeContactFrictionImpulseLimit(
    friction_coeff: f32,
    normal_impulse_limit: f32,
    policy: ContactFrictionSolvePolicy,
) f32 {
    const clamped_friction = @max(0.0, friction_coeff);
    const normal_limit = @max(policy.min_normal_impulse_limit, normal_impulse_limit);
    return @min(
        clamped_friction * normal_limit,
        clamped_friction * policy.max_impulse_per_friction,
    );
}

fn computeContactStaticFrictionCoefficient(dynamic_friction: f32, tangential_speed: f32) f32 {
    return material_pairing.computeStaticFrictionCoefficient(
        dynamic_friction,
        tangential_speed,
        material_pairing.defaultStaticFrictionPolicy(),
    );
}

fn computeContactDynamicFrictionCoefficient(base_friction: f32, tangential_speed: f32) f32 {
    return material_pairing.computeDynamicFrictionCoefficient(
        base_friction,
        tangential_speed,
        material_pairing.defaultDynamicFrictionPolicy(),
    );
}

fn computeContactEffectiveFrictionCoefficient(base_friction: f32, tangential_speed: f32) f32 {
    const dynamic_friction = computeContactDynamicFrictionCoefficient(base_friction, tangential_speed);
    return computeContactStaticFrictionCoefficient(dynamic_friction, tangential_speed);
}

fn computeContactRollingFrictionCoefficient(dynamic_friction: f32, angular_speed: f32) f32 {
    return material_pairing.computeRollingFrictionCoefficient(
        dynamic_friction,
        angular_speed,
        material_pairing.defaultRollingFrictionPolicy(),
    );
}

fn computeContactAnisotropicFrictionMinorAxisScale(
    surface_a: terrain.SurfaceType,
    surface_b: terrain.SurfaceType,
) f32 {
    return material_pairing.computeAnisotropicFrictionMinorAxisScale(
        surface_a,
        surface_b,
        material_pairing.defaultAnisotropicFrictionPolicy(),
    );
}

fn contactAngularSpeed(inst_a: *const scene32.Instance, inst_b: *const scene32.Instance) f32 {
    const ax = @as(f32, @floatFromInt(inst_a.ang_x)) - @as(f32, @floatFromInt(inst_b.ang_x));
    const ay = @as(f32, @floatFromInt(inst_a.ang_y)) - @as(f32, @floatFromInt(inst_b.ang_y));
    const az = @as(f32, @floatFromInt(inst_a.ang_z)) - @as(f32, @floatFromInt(inst_b.ang_z));
    return @sqrt(ax * ax + ay * ay + az * az);
}

fn approximateContactFrictionCone(
    velocity_components: DirectionalVelocityComponents,
    friction_coeff: f32,
    normal_impulse_limit: f32,
    policy: ContactFrictionSolvePolicy,
) ?ContactFrictionConeApproximation {
    const tangent_frame = buildDirectionalTangentFrame(velocity_components) orelse return null;
    return .{
        .dir_x = tangent_frame.dir_x,
        .dir_y = tangent_frame.dir_y,
        .dir_z = tangent_frame.dir_z,
        .tangential_speed = tangent_frame.speed,
        .impulse_limit = computeContactFrictionImpulseLimit(
            friction_coeff,
            normal_impulse_limit,
            policy,
        ),
    };
}

fn normalizeContactVector(x: f32, y: f32, z: f32) ?JointAxisVector {
    const len = @sqrt(x * x + y * y + z * z);
    if (len <= 0.0001) return null;
    return .{ .x = x / len, .y = y / len, .z = z / len };
}

fn contactDot(ax: f32, ay: f32, az: f32, bx: f32, by: f32, bz: f32) f32 {
    return ax * bx + ay * by + az * bz;
}

fn buildContactFrictionEllipseAxes(
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
) struct { major: JointAxisVector, minor: JointAxisVector } {
    const normal = normalizeContactVector(normal_x, normal_y, normal_z) orelse JointAxisVector{ .x = 0.0, .y = 1.0, .z = 0.0 };
    const seed: JointAxisVector = if (@abs(normal.y) < 0.9)
        .{ .x = 0.0, .y = 1.0, .z = 0.0 }
    else
        .{ .x = 1.0, .y = 0.0, .z = 0.0 };
    const major = normalizeContactVector(
        seed.x - normal.x * contactDot(seed.x, seed.y, seed.z, normal.x, normal.y, normal.z),
        seed.y - normal.y * contactDot(seed.x, seed.y, seed.z, normal.x, normal.y, normal.z),
        seed.z - normal.z * contactDot(seed.x, seed.y, seed.z, normal.x, normal.y, normal.z),
    ) orelse JointAxisVector{ .x = 1.0, .y = 0.0, .z = 0.0 };
    const minor = normalizeContactVector(
        normal.y * major.z - normal.z * major.y,
        normal.z * major.x - normal.x * major.z,
        normal.x * major.y - normal.y * major.x,
    ) orelse JointAxisVector{ .x = 0.0, .y = 0.0, .z = 1.0 };
    return .{ .major = major, .minor = minor };
}

fn approximateContactFrictionEllipse(
    cone: ContactFrictionConeApproximation,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    anisotropic_minor_axis_scale: f32,
    policy: ContactFrictionSolvePolicy,
) ContactFrictionEllipseApproximation {
    const axes = buildContactFrictionEllipseAxes(normal_x, normal_y, normal_z);
    const major_limit = @max(0.0, cone.impulse_limit);
    const policy_minor_scale = @max(0.0, @min(1.0, policy.ellipse_minor_axis_scale));
    const anisotropic_scale = @max(0.0, @min(1.0, anisotropic_minor_axis_scale));
    const minor_limit = @max(0.0, major_limit * policy_minor_scale * anisotropic_scale);
    const major_component = contactDot(cone.dir_x, cone.dir_y, cone.dir_z, axes.major.x, axes.major.y, axes.major.z);
    const minor_component = contactDot(cone.dir_x, cone.dir_y, cone.dir_z, axes.minor.x, axes.minor.y, axes.minor.z);
    const denominator = @sqrt(
        (major_component * major_component) / @max(0.0001, major_limit * major_limit) +
            (minor_component * minor_component) / @max(0.0001, minor_limit * minor_limit),
    );
    const directional_limit = if (denominator <= 0.0001) 0.0 else @min(major_limit, 1.0 / denominator);
    return .{
        .cone = cone,
        .major_x = axes.major.x,
        .major_y = axes.major.y,
        .major_z = axes.major.z,
        .minor_x = axes.minor.x,
        .minor_y = axes.minor.y,
        .minor_z = axes.minor.z,
        .major_impulse_limit = major_limit,
        .minor_impulse_limit = minor_limit,
        .impulse_limit = directional_limit,
    };
}

fn projectContactFrictionImpulseToCone(
    requested_impulse: f32,
    cone: ContactFrictionConeApproximation,
) f32 {
    const limit = @max(0.0, cone.impulse_limit);
    return @max(-limit, @min(limit, requested_impulse));
}

fn projectContactFrictionImpulseToEllipse(
    requested_impulse: f32,
    ellipse: ContactFrictionEllipseApproximation,
) f32 {
    const cone_projected = projectContactFrictionImpulseToCone(requested_impulse, ellipse.cone);
    const limit = @max(0.0, ellipse.impulse_limit);
    return @max(-limit, @min(limit, cone_projected));
}

fn buildContactTangentConstraintSpecFromEllipse(
    ellipse: ContactFrictionEllipseApproximation,
) ContactTangentConstraintSpec {
    return .{
        .dir_x = ellipse.cone.dir_x,
        .dir_y = ellipse.cone.dir_y,
        .dir_z = ellipse.cone.dir_z,
        .speed = ellipse.cone.tangential_speed,
        .impulse_limit = ellipse.impulse_limit,
    };
}

fn computeContactTangentConstraintImpulse(
    tangent: ContactTangentConstraintSpec,
    inv_mass_a: f32,
    inv_mass_b: f32,
) f32 {
    return computePairImpulseMagnitude(.{
        .speed = tangent.speed,
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .policy = .{
            .max_impulse = tangent.impulse_limit,
        },
    });
}

fn projectContactTangentConstraintImpulse(
    requested_impulse: f32,
    tangent: ContactTangentConstraintSpec,
) f32 {
    const limit = @max(0.0, tangent.impulse_limit);
    return @max(-limit, @min(limit, requested_impulse));
}

fn buildContactFrictionEquation(
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangential_speed: f32,
    friction_coeff: f32,
    normal_impulse_limit: f32,
) ConstraintRowEquation {
    const effective_mass = effectiveMassFromPairInverseMasses(inv_mass_a, inv_mass_b);
    const policy = contactFrictionSolvePolicy();
    return .{
        .effective_mass = effective_mass,
        .bias = @abs(tangential_speed) * 0.05,
        .max_impulse = @max(
            policy.min_equation_max_impulse,
            computeContactFrictionImpulseLimit(friction_coeff, normal_impulse_limit, policy),
        ),
    };
}

fn contactPositionCorrectionPolicy() ContactPositionCorrectionPolicy {
    return .{};
}

fn computeContactPositionCorrectionDepth(
    penetration_depth: f32,
    equation: ConstraintRowEquation,
    policy: ContactPositionCorrectionPolicy,
) f32 {
    const positional_depth = @max(0.0, penetration_depth - policy.allowed_penetration_slop);
    if (positional_depth <= 0.0) return 0.0;

    const target_depth = positional_depth * policy.depth_correction_ratio;
    const bias_depth = equation.bias * equation.effective_mass * policy.bias_scale;
    return @min(equation.max_impulse, target_depth + bias_depth);
}

fn contactVelocitySolvePolicy() ContactVelocitySolvePolicy {
    return .{};
}

fn computeContactEffectiveRestitution(
    closing_speed: f32,
    restitution: f32,
    policy: ContactRestitutionSolvePolicy,
) f32 {
    if (closing_speed < policy.velocity_threshold) return 0.0;
    return @max(0.0, @min(policy.max_restitution, restitution));
}

fn computeContactRestitutionImpulseMultiplier(
    closing_speed: f32,
    restitution: f32,
    policy: ContactRestitutionSolvePolicy,
) f32 {
    return 1.0 + computeContactEffectiveRestitution(closing_speed, restitution, policy);
}

fn computePairImpulseDenominator(inv_mass_a: f32, inv_mass_b: f32, policy: PairImpulsePolicy) f32 {
    return @max(inv_mass_a + inv_mass_b, policy.min_inverse_mass);
}

fn computePairImpulseMagnitude(spec: PairVelocityImpulseSpec) f32 {
    if (spec.speed <= 0.0 or spec.multiplier <= 0.0) return 0.0;
    const unclamped = (spec.speed * spec.multiplier) / computePairImpulseDenominator(
        spec.inv_mass_a,
        spec.inv_mass_b,
        spec.policy,
    );
    if (spec.policy.max_impulse) |max_impulse| {
        return @min(@max(0.0, max_impulse), unclamped);
    }
    return unclamped;
}

fn computeContactNormalVelocityImpulse(
    relative_normal_velocity: f32,
    inv_mass_a: f32,
    inv_mass_b: f32,
    restitution: f32,
    policy: ContactVelocitySolvePolicy,
) f32 {
    if (relative_normal_velocity >= 0.0) return 0.0;
    const closing_speed = -relative_normal_velocity;
    return computePairImpulseMagnitude(.{
        .speed = closing_speed,
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .multiplier = computeContactRestitutionImpulseMultiplier(
            closing_speed,
            restitution,
            policy.restitution,
        ),
    });
}

fn buildContactNormalConstraintSpec(
    direction: PreparedDirectionalConstraint,
    restitution: f32,
) ContactNormalConstraintSpec {
    return .{
        .dir_x = direction.dir_x,
        .dir_y = direction.dir_y,
        .dir_z = direction.dir_z,
        .penetration_depth = direction.depth,
        .restitution = restitution,
    };
}

fn computeContactNormalConstraintPositionImpulse(
    normal: ContactNormalConstraintSpec,
    equation: ConstraintRowEquation,
) f32 {
    return computeContactPositionCorrectionDepth(
        normal.penetration_depth,
        equation,
        contactPositionCorrectionPolicy(),
    );
}

fn computeContactNormalConstraintVelocityImpulse(
    normal: ContactNormalConstraintSpec,
    relative_normal_velocity: f32,
    inv_mass_a: f32,
    inv_mass_b: f32,
) f32 {
    return computeContactNormalVelocityImpulse(
        relative_normal_velocity,
        inv_mass_a,
        inv_mass_b,
        normal.restitution,
        contactVelocitySolvePolicy(),
    );
}

fn buildContactNormalRowPlan(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    direction: PreparedDirectionalConstraint,
) DirectionalRowPlan {
    return buildContactNormalRowPlanWithStiffness(
        inv_mass_a,
        inv_mass_b,
        penetration_depth,
        relative_normal_speed,
        direction,
        neutralContactStiffnessModel(),
    );
}

fn buildContactNormalRowPlanWithStiffness(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    direction: PreparedDirectionalConstraint,
    stiffness: ContactStiffnessModel,
) DirectionalRowPlan {
    return buildContactNormalRowPlanWithResponse(
        inv_mass_a,
        inv_mass_b,
        penetration_depth,
        relative_normal_speed,
        direction,
        stiffness,
        neutralContactDampingModel(),
    );
}

fn buildContactNormalRowPlanWithResponse(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    direction: PreparedDirectionalConstraint,
    stiffness: ContactStiffnessModel,
    damping: ContactDampingModel,
) DirectionalRowPlan {
    return buildContactNormalRowPlanWithResponseAndStabilization(
        inv_mass_a,
        inv_mass_b,
        penetration_depth,
        relative_normal_speed,
        direction,
        stiffness,
        damping,
        neutralContactStabilizationModel(),
    );
}

fn buildContactNormalRowPlanWithResponseAndStabilization(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    direction: PreparedDirectionalConstraint,
    stiffness: ContactStiffnessModel,
    damping: ContactDampingModel,
    stabilization: ContactStabilizationModel,
) DirectionalRowPlan {
    return buildDirectionalRowPlan(
        buildContactNormalEquationWithResponseAndStabilization(
            inv_mass_a,
            inv_mass_b,
            penetration_depth,
            relative_normal_speed,
            stiffness,
            damping,
            stabilization,
        ),
        direction,
        contactNormalPredictiveConstraintPolicy(),
    );
}

fn buildContactFrictionRowPlan(
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangential_speed: f32,
    friction_coeff: f32,
    normal_impulse_limit: f32,
    direction: PreparedDirectionalConstraint,
) DirectionalRowPlan {
    return buildDirectionalResidualRowPlan(
        .tangent,
        direction,
        buildContactFrictionEquation(
            inv_mass_a,
            inv_mass_b,
            tangential_speed,
            friction_coeff,
            normal_impulse_limit,
        ),
    );
}

fn buildEnvironmentEquation(
    inverse_mass: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
) ConstraintRowEquation {
    return buildNormalConstraintEquation(
        effectiveMassFromInverseMass(inverse_mass),
        penetration_depth,
        relative_normal_speed,
        0.25,
        0.05,
        6.0,
        0.5,
    );
}

fn buildEnvironmentDirectionalRowPlan(
    inverse_mass: f32,
    metrics: EnvironmentConstraintMetrics,
    direction: PreparedDirectionalConstraint,
) DirectionalRowPlan {
    return buildDirectionalRowPlan(
        buildEnvironmentEquation(
            inverse_mass,
            metrics.penetration_depth,
            metrics.normal_speed,
        ),
        direction,
        environmentPredictiveConstraintPolicy(),
    );
}

fn buildJointRowPlan(
    base_residual: f32,
    equation: ConstraintRowEquation,
) DirectionalRowPlan {
    return makeDirectionalRowPlan(base_residual, equation, 0.0);
}

fn zeroConstraintEquation() ConstraintRowEquation {
    return .{
        .effective_mass = 0.0,
        .bias = 0.0,
        .max_impulse = 0.0,
    };
}

fn zeroDirectionalRowPlan() DirectionalRowPlan {
    return makeDirectionalRowPlan(0.0, zeroConstraintEquation(), 0.0);
}

fn buildJointEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    const priority = joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities);
    if (priority <= 0.0 or joint_idx >= joints.len) return zeroConstraintEquation();

    const joint_def = &joints[joint_idx];
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return zeroConstraintEquation();

    const inv_mass_a = instanceInverseMass(&instances[joint_def.entity_a], entities);
    const inv_mass_b = instanceInverseMass(&instances[joint_def.entity_b], entities);
    const effective_mass = effectiveMassFromPairInverseMasses(inv_mass_a, inv_mass_b);

    return .{
        .effective_mass = effective_mass,
        .bias = priority * 0.15,
        .max_impulse = @max(0.5, priority * 4.0),
    };
}

fn buildJointDriveEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    const priority = joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities);
    if (priority <= 0.0 or joint_idx >= joints.len) return zeroConstraintEquation();

    const joint_def = &joints[joint_idx];
    if (!joint_def.motor_enabled or joint_def.motor_speed <= 0.0) return zeroConstraintEquation();
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return zeroConstraintEquation();

    const inv_mass_a = instanceInverseMass(&instances[joint_def.entity_a], entities);
    const inv_mass_b = instanceInverseMass(&instances[joint_def.entity_b], entities);
    const effective_mass = effectiveMassFromPairInverseMasses(inv_mass_a, inv_mass_b);

    return .{
        .effective_mass = effective_mass,
        .bias = @max(priority * 0.2, joint_def.motor_speed * 0.05),
        .max_impulse = @max(0.5, @max(priority * 3.0, joint_def.motor_max_torque * 0.01)),
    };
}

fn jointHasAnchorRow(joint_def: *const joint.Joint) bool {
    return switch (joint_def.joint_type) {
        .fixed, .hinge, .slider, .spring, .ball_socket => true,
        .pulley => false,
    };
}

fn jointHasLimitRow(joint_def: *const joint.Joint) bool {
    return switch (joint_def.joint_type) {
        .fixed => true,
        .hinge => @abs(joint_def.limit_max - joint_def.limit_min) < 6.0,
        .slider => true,
        .spring => true,
        .pulley => true,
        else => false,
    };
}

fn buildJointAnchorEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    return buildJointEquation(instances, joints, joint_idx, entities);
}

fn buildJointKindRowPlan(
    kind: ConstraintRowKind,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    equation_builder: JointEquationBuilderFn,
) DirectionalRowPlan {
    return buildJointRowPlan(
        measureJointRowResidual(kind, instances, joints, joint_idx, entities),
        equation_builder(instances, joints, joint_idx, entities),
    );
}

fn buildJointAnchorRowPlan(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) DirectionalRowPlan {
    return buildJointKindRowPlan(
        .joint_anchor,
        instances,
        joints,
        joint_idx,
        entities,
        buildJointAnchorEquation,
    );
}

fn buildJointLimitEquation(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ConstraintRowEquation {
    const base = buildJointEquation(instances, joints, joint_idx, entities);
    return .{
        .effective_mass = base.effective_mass,
        .bias = base.bias * 1.1,
        .max_impulse = base.max_impulse * 0.9,
    };
}

fn buildJointLimitRowPlan(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) DirectionalRowPlan {
    return buildJointKindRowPlan(
        .joint_limit,
        instances,
        joints,
        joint_idx,
        entities,
        buildJointLimitEquation,
    );
}

fn buildJointDriveRowPlan(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) DirectionalRowPlan {
    return buildJointKindRowPlan(
        .joint_drive,
        instances,
        joints,
        joint_idx,
        entities,
        buildJointDriveEquation,
    );
}

fn probeContactConstraintWithAABBs(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entity_a: *const entity16.Entity16,
    entity_b: *const entity16.Entity16,
    aabb_a: physics.AABB,
    aabb_b: physics.AABB,
    stats: ?*ContactNarrowPhaseStats,
) ?ContactConstraintProbe {
    if (stats) |s| {
        s.probe_calls += 1;
        s.aabb_tests += 1;
    }
    if (!physics.aabbHit(aabb_a, aabb_b)) return null;

    if (stats) |s| s.narrowphase_calls += 1;
    const contact = collision.narrowPhaseAABB(aabb_a, aabb_b);
    if (!contact.hit) return null;
    const manifold = contact.manifold;

    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const velocity_components = measureDirectionalVelocityComponents(
        rel_vel_x,
        rel_vel_y,
        rel_vel_z,
        manifold.normal_x,
        manifold.normal_y,
        manifold.normal_z,
    );

    return .{
        .entity_a = entity_a,
        .entity_b = entity_b,
        .aabb_a = aabb_a,
        .aabb_b = aabb_b,
        .manifold = manifold,
        .velocity_components = velocity_components,
    };
}

fn probeContactConstraintMeasured(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
    stats: *ContactNarrowPhaseStats,
) ?ContactConstraintProbe {
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return null;

    const entity_a = &entities[inst_a.entity_id];
    const entity_b = &entities[inst_b.entity_id];
    const aabb_a = physics.computeEntityWorldAABB(inst_a, entity_a) orelse return null;
    const aabb_b = physics.computeEntityWorldAABB(inst_b, entity_b) orelse return null;
    stats.aabb_builds += 2;
    return probeContactConstraintWithAABBs(inst_a, inst_b, entity_a, entity_b, aabb_a, aabb_b, stats);
}

fn probeContactConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ContactConstraintProbe {
    var stats: ContactNarrowPhaseStats = .{};
    return probeContactConstraintMeasured(inst_a, inst_b, entities, &stats);
}

fn measureContactConstraintMetrics(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ContactConstraintMetrics {
    const probe = probeContactConstraint(inst_a, inst_b, entities) orelse return null;

    return .{
        .penetration_depth = probe.manifold.penetration_depth,
        .normal_speed = probe.velocity_components.normalSpeed(),
        .tangential_speed = probe.velocity_components.tangentialSpeed(),
    };
}

pub fn detectContactManifoldsMeasured(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
    out_manifolds: []ContactDetectedManifold,
    stats: *ContactNarrowPhaseStats,
) usize {
    var count: usize = 0;
    for (broadphase_pairs) |pair| {
        if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;
        const inst_a = &s1024.instances[pair.a];
        const inst_b = &s1024.instances[pair.b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        const probe = probeContactConstraintMeasured(inst_a, inst_b, entities, stats) orelse continue;
        if (count >= out_manifolds.len) return count;

        out_manifolds[count] = .{
            .pair = pair,
            .entity_id_a = inst_a.entity_id,
            .entity_id_b = inst_b.entity_id,
            .aabb_a = probe.aabb_a,
            .aabb_b = probe.aabb_b,
            .manifold = probe.manifold,
            .centroid = collision.computeContactManifoldCentroid(probe.manifold),
        };
        count += 1;
    }
    return count;
}

pub fn detectContactManifolds(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
    out_manifolds: []ContactDetectedManifold,
) usize {
    var stats: ContactNarrowPhaseStats = .{};
    return detectContactManifoldsMeasured(s1024, entities, broadphase_pairs, out_manifolds, &stats);
}

pub fn classifyContactMaterials(
    detected_manifolds: []const ContactDetectedManifold,
    entities: []entity16.Entity16,
    out_classifications: []ContactMaterialClassification,
) usize {
    var count: usize = 0;
    for (detected_manifolds) |detected| {
        if (detected.entity_id_a >= entities.len or detected.entity_id_b >= entities.len) continue;
        if (count >= out_classifications.len) return count;

        const entity_a = &entities[detected.entity_id_a];
        const entity_b = &entities[detected.entity_id_b];
        const material_a = entity_a.physics.material;
        const material_b = entity_b.physics.material;
        const surface_a = material_pairing.getSurfaceForMaterial(material_a);
        const surface_b = material_pairing.getSurfaceForMaterial(material_b);

        out_classifications[count] = .{
            .pair = detected.pair,
            .entity_id_a = detected.entity_id_a,
            .entity_id_b = detected.entity_id_b,
            .material_a = material_a,
            .material_b = material_b,
            .response = material_pairing.getPairedResponse(surface_a, material_a, surface_b, material_b),
        };
        count += 1;
    }
    return count;
}

pub fn classifyContactSurfaces(
    material_classifications: []const ContactMaterialClassification,
    out_classifications: []ContactSurfaceClassification,
) usize {
    var count: usize = 0;
    for (material_classifications) |classified| {
        if (count >= out_classifications.len) return count;

        const surface_a = material_pairing.getSurfaceForMaterial(classified.material_a);
        const surface_b = material_pairing.getSurfaceForMaterial(classified.material_b);
        out_classifications[count] = .{
            .pair = classified.pair,
            .entity_id_a = classified.entity_id_a,
            .entity_id_b = classified.entity_id_b,
            .material_a = classified.material_a,
            .material_b = classified.material_b,
            .surface_a = surface_a,
            .surface_b = surface_b,
            .hard_surface_a = material_pairing.isHardSurface(surface_a),
            .hard_surface_b = material_pairing.isHardSurface(surface_b),
        };
        count += 1;
    }
    return count;
}

pub fn classifyContactMediums(
    surface_classifications: []const ContactSurfaceClassification,
    out_classifications: []ContactMediumClassification,
) usize {
    var count: usize = 0;
    for (surface_classifications) |classified| {
        if (count >= out_classifications.len) return count;

        out_classifications[count] = .{
            .pair = classified.pair,
            .entity_id_a = classified.entity_id_a,
            .entity_id_b = classified.entity_id_b,
            .material_a = classified.material_a,
            .material_b = classified.material_b,
            .surface_a = classified.surface_a,
            .surface_b = classified.surface_b,
            .medium_a = material_pairing.getMediumType(classified.surface_a),
            .medium_b = material_pairing.getMediumType(classified.surface_b),
        };
        count += 1;
    }
    return count;
}

fn classifyContactBodyTypeForEntity(entity: *const entity16.Entity16) query_types.BodyType {
    if ((entity.physics.flags & 0x01) != 0) return .static;
    if ((entity.physics.flags & 0x08) != 0) return .sensor;
    if ((entity.physics.flags & 0x02) != 0) return .kinematic;
    return .dynamic;
}

pub fn classifyContactBodyTypes(
    medium_classifications: []const ContactMediumClassification,
    entities: []entity16.Entity16,
    out_classifications: []ContactBodyTypeClassification,
) usize {
    var count: usize = 0;
    for (medium_classifications) |classified| {
        if (classified.entity_id_a >= entities.len or classified.entity_id_b >= entities.len) continue;
        if (count >= out_classifications.len) return count;

        const entity_a = &entities[classified.entity_id_a];
        const entity_b = &entities[classified.entity_id_b];
        out_classifications[count] = .{
            .pair = classified.pair,
            .entity_id_a = classified.entity_id_a,
            .entity_id_b = classified.entity_id_b,
            .material_a = classified.material_a,
            .material_b = classified.material_b,
            .surface_a = classified.surface_a,
            .surface_b = classified.surface_b,
            .medium_a = classified.medium_a,
            .medium_b = classified.medium_b,
            .body_type_a = classifyContactBodyTypeForEntity(entity_a),
            .body_type_b = classifyContactBodyTypeForEntity(entity_b),
        };
        count += 1;
    }
    return count;
}

fn classifyContactSurfaceCondition(surface: terrain.SurfaceType, medium: terrain.MediumType) query_types.SurfaceCondition {
    return switch (surface) {
        .asphalt_wet => .wet,
        .gravel, .sand => .loose,
        .grass, .mud, .mud_ruts, .snow, .cloth, .carpet => .deformable,
        .ice, .rubber => .slippery,
        .water => .submerged,
        else => switch (medium) {
            .liquid => .submerged,
            .soft => .deformable,
            else => .dry,
        },
    };
}

fn contactFiniteF32(value: f32) bool {
    return value == value and value != std.math.inf(f32) and value != -std.math.inf(f32);
}

fn contactApproxEq(a: f32, b: f32) bool {
    if (!contactFiniteF32(a) or !contactFiniteF32(b)) return false;
    const delta = if (a >= b) a - b else b - a;
    return delta <= 0.0001;
}

fn contactResponseApproxEq(a: material_pairing.ContactResponse, b: material_pairing.ContactResponse) bool {
    return contactApproxEq(a.restitution, b.restitution) and
        contactApproxEq(a.friction, b.friction) and
        contactApproxEq(a.damage_modifier, b.damage_modifier) and
        contactApproxEq(a.penetration_resistance, b.penetration_resistance) and
        contactApproxEq(a.buoyancy, b.buoyancy) and
        a.sound_type == b.sound_type and
        a.dust_type == b.dust_type and
        a.medium_type == b.medium_type;
}

pub fn classifyContactConditions(
    body_type_classifications: []const ContactBodyTypeClassification,
    out_classifications: []ContactConditionClassification,
) usize {
    var count: usize = 0;
    for (body_type_classifications) |classified| {
        if (count >= out_classifications.len) return count;

        out_classifications[count] = .{
            .pair = classified.pair,
            .entity_id_a = classified.entity_id_a,
            .entity_id_b = classified.entity_id_b,
            .material_a = classified.material_a,
            .material_b = classified.material_b,
            .surface_a = classified.surface_a,
            .surface_b = classified.surface_b,
            .medium_a = classified.medium_a,
            .medium_b = classified.medium_b,
            .body_type_a = classified.body_type_a,
            .body_type_b = classified.body_type_b,
            .condition_a = classifyContactSurfaceCondition(classified.surface_a, classified.medium_a),
            .condition_b = classifyContactSurfaceCondition(classified.surface_b, classified.medium_b),
        };
        count += 1;
    }
    return count;
}

pub fn validateContactClassification(
    classification: ContactClassification,
    entities: []entity16.Entity16,
) ContactClassificationConsistency {
    var result: ContactClassificationConsistency = .{
        .pair_order_valid = classification.pair.a < classification.pair.b,
    };
    if (classification.entity_id_a >= entities.len or classification.entity_id_b >= entities.len) return result;

    result.entity_ids_valid = true;
    const entity_a = &entities[classification.entity_id_a];
    const entity_b = &entities[classification.entity_id_b];
    const expected_surface_a = material_pairing.getSurfaceForMaterial(entity_a.physics.material);
    const expected_surface_b = material_pairing.getSurfaceForMaterial(entity_b.physics.material);
    const expected_medium_a = material_pairing.getMediumType(classification.surface_a);
    const expected_medium_b = material_pairing.getMediumType(classification.surface_b);
    const expected_response = material_pairing.getPairedResponse(
        expected_surface_a,
        entity_a.physics.material,
        expected_surface_b,
        entity_b.physics.material,
    );

    result.materials_match_entities =
        classification.material_a == entity_a.physics.material and
        classification.material_b == entity_b.physics.material;
    result.surfaces_match_materials =
        classification.surface_a == expected_surface_a and
        classification.surface_b == expected_surface_b;
    result.mediums_match_surfaces =
        classification.medium_a == expected_medium_a and
        classification.medium_b == expected_medium_b;
    result.body_types_match_entities =
        classification.body_type_a == classifyContactBodyTypeForEntity(entity_a) and
        classification.body_type_b == classifyContactBodyTypeForEntity(entity_b);
    result.conditions_match_surfaces =
        classification.condition_a == classifyContactSurfaceCondition(classification.surface_a, classification.medium_a) and
        classification.condition_b == classifyContactSurfaceCondition(classification.surface_b, classification.medium_b);
    result.response_matches_pair = contactResponseApproxEq(classification.response, expected_response);
    return result;
}

pub fn contactClassificationIsConsistent(
    classification: ContactClassification,
    entities: []entity16.Entity16,
) bool {
    return validateContactClassification(classification, entities).isConsistent();
}

fn buildContactClassification(
    condition_classification: ContactConditionClassification,
    entities: []entity16.Entity16,
) ?ContactClassification {
    if (condition_classification.entity_id_a >= entities.len or condition_classification.entity_id_b >= entities.len) return null;

    const entity_a = &entities[condition_classification.entity_id_a];
    const entity_b = &entities[condition_classification.entity_id_b];
    const response = material_pairing.getPairedResponse(
        condition_classification.surface_a,
        entity_a.physics.material,
        condition_classification.surface_b,
        entity_b.physics.material,
    );
    return .{
        .pair = condition_classification.pair,
        .entity_id_a = condition_classification.entity_id_a,
        .entity_id_b = condition_classification.entity_id_b,
        .material_a = entity_a.physics.material,
        .material_b = entity_b.physics.material,
        .surface_a = condition_classification.surface_a,
        .surface_b = condition_classification.surface_b,
        .medium_a = condition_classification.medium_a,
        .medium_b = condition_classification.medium_b,
        .body_type_a = condition_classification.body_type_a,
        .body_type_b = condition_classification.body_type_b,
        .condition_a = condition_classification.condition_a,
        .condition_b = condition_classification.condition_b,
        .response = response,
    };
}

fn buildContactClassificationForEntities(
    pair: BroadPhasePair,
    entity_id_a: u16,
    entity_id_b: u16,
    entity_a: *const entity16.Entity16,
    entity_b: *const entity16.Entity16,
) ContactClassification {
    const material_a = entity_a.physics.material;
    const material_b = entity_b.physics.material;
    const surface_a = material_pairing.getSurfaceForMaterial(material_a);
    const surface_b = material_pairing.getSurfaceForMaterial(material_b);
    const medium_a = material_pairing.getMediumType(surface_a);
    const medium_b = material_pairing.getMediumType(surface_b);
    return .{
        .pair = pair,
        .entity_id_a = entity_id_a,
        .entity_id_b = entity_id_b,
        .material_a = material_a,
        .material_b = material_b,
        .surface_a = surface_a,
        .surface_b = surface_b,
        .medium_a = medium_a,
        .medium_b = medium_b,
        .body_type_a = classifyContactBodyTypeForEntity(entity_a),
        .body_type_b = classifyContactBodyTypeForEntity(entity_b),
        .condition_a = classifyContactSurfaceCondition(surface_a, medium_a),
        .condition_b = classifyContactSurfaceCondition(surface_b, medium_b),
        .response = material_pairing.getPairedResponse(surface_a, material_a, surface_b, material_b),
    };
}

pub fn buildContacts(
    detected_manifolds: []const ContactDetectedManifold,
    condition_classifications: []const ContactConditionClassification,
    entities: []entity16.Entity16,
    out_contacts: []Contact,
) usize {
    var count: usize = 0;
    for (detected_manifolds) |detected| {
        for (condition_classifications) |classified| {
            if (classified.pair.a != detected.pair.a or classified.pair.b != detected.pair.b) continue;
            if (count >= out_contacts.len) return count;

            const classification = buildContactClassification(classified, entities) orelse break;
            out_contacts[count] = .{
                .pair = detected.pair,
                .entity_id_a = detected.entity_id_a,
                .entity_id_b = detected.entity_id_b,
                .aabb_a = detected.aabb_a,
                .aabb_b = detected.aabb_b,
                .manifold = detected.manifold,
                .centroid = detected.centroid,
                .classification = classification,
            };
            count += 1;
            break;
        }
    }
    return count;
}

pub fn buildContactsOptimizedMeasured(
    detected_manifolds: []const ContactDetectedManifold,
    entities: []entity16.Entity16,
    cache: ?*ContactClassificationCache,
    out_contacts: []Contact,
    stats: *ContactClassificationPipelineStats,
) usize {
    stats.* = .{
        .detected_count = detected_manifolds.len,
    };

    var count: usize = 0;
    for (detected_manifolds) |detected| {
        if (count >= out_contacts.len) return count;
        if (detected.entity_id_a >= entities.len or detected.entity_id_b >= entities.len) {
            stats.skipped_invalid_entities += 1;
            continue;
        }

        stats.entity_lookup_count += 2;
        const entity_a = &entities[detected.entity_id_a];
        const entity_b = &entities[detected.entity_id_b];
        const classification = if (cache) |classification_cache|
            classification_cache.get(detected.pair) orelse blk: {
                const built = buildContactClassificationForEntities(
                    detected.pair,
                    detected.entity_id_a,
                    detected.entity_id_b,
                    entity_a,
                    entity_b,
                );
                _ = classification_cache.put(built);
                break :blk built;
            }
        else
            buildContactClassificationForEntities(
                detected.pair,
                detected.entity_id_a,
                detected.entity_id_b,
                entity_a,
                entity_b,
            );

        out_contacts[count] = .{
            .pair = detected.pair,
            .entity_id_a = detected.entity_id_a,
            .entity_id_b = detected.entity_id_b,
            .aabb_a = detected.aabb_a,
            .aabb_b = detected.aabb_b,
            .manifold = detected.manifold,
            .centroid = detected.centroid,
            .classification = classification,
        };
        count += 1;
        stats.classified_count += 1;
    }
    return count;
}

pub fn buildContactsOptimized(
    detected_manifolds: []const ContactDetectedManifold,
    entities: []entity16.Entity16,
    cache: ?*ContactClassificationCache,
    out_contacts: []Contact,
) usize {
    var stats: ContactClassificationPipelineStats = .{};
    return buildContactsOptimizedMeasured(detected_manifolds, entities, cache, out_contacts, &stats);
}

pub fn buildContactsCached(
    detected_manifolds: []const ContactDetectedManifold,
    condition_classifications: []const ContactConditionClassification,
    entities: []entity16.Entity16,
    cache: *ContactClassificationCache,
    out_contacts: []Contact,
) usize {
    var count: usize = 0;
    for (detected_manifolds) |detected| {
        for (condition_classifications) |classified| {
            if (!sameBroadPhasePair(classified.pair, detected.pair)) continue;
            if (count >= out_contacts.len) return count;

            const classification = cache.getOrBuild(classified, entities) orelse break;
            out_contacts[count] = .{
                .pair = detected.pair,
                .entity_id_a = detected.entity_id_a,
                .entity_id_b = detected.entity_id_b,
                .aabb_a = detected.aabb_a,
                .aabb_b = detected.aabb_b,
                .manifold = detected.manifold,
                .centroid = detected.centroid,
                .classification = classification,
            };
            count += 1;
            break;
        }
    }
    return count;
}

pub fn makeMaterialPairResponse(contact: *const Contact) MaterialPairResponse {
    return .{
        .pair = contact.pair,
        .material_a = contact.classification.material_a,
        .material_b = contact.classification.material_b,
        .surface_a = contact.classification.surface_a,
        .surface_b = contact.classification.surface_b,
        .response = contact.classification.response,
    };
}

pub fn makeContactTelemetry(contact: *const Contact) ContactTelemetry {
    const response = contact.classification.response;
    const fatigue = computeContactFatigueModel(
        .{
            .penetration_depth = contact.manifold.penetration_depth,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
            .damage_modifier = response.damage_modifier,
            .penetration_resistance = response.penetration_resistance,
            .repeat_count = @max(@as(u32, 1), @as(u32, contact.manifold.point_count)),
        },
        contactFatiguePolicy(),
    );
    const thermal = computeContactThermalModel(
        .{
            .penetration_depth = contact.manifold.penetration_depth,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
            .friction = response.friction,
            .penetration_resistance = response.penetration_resistance,
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
        },
        contactThermalPolicy(),
    );
    const sound = computeContactSoundModel(
        .{
            .sound_type = response.sound_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
            .friction = response.friction,
            .restitution = response.restitution,
            .medium_type = response.medium_type,
        },
        contactSoundPolicy(),
    );
    const dust = computeContactDustModel(
        .{
            .dust_type = response.dust_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
            .friction = response.friction,
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
        },
        contactDustPolicy(),
    );
    const deformation = computeContactDeformationModel(
        .{
            .penetration_depth = contact.manifold.penetration_depth,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
            .penetration_resistance = response.penetration_resistance,
            .damage_modifier = response.damage_modifier,
            .restitution = response.restitution,
        },
        contactDeformationPolicy(),
    );
    const separation = computeContactSeparationModel(
        .{
            .penetration_depth = contact.manifold.penetration_depth,
            .predicted_penetration_depth = contact.manifold.penetration_depth,
            .normal_speed_signed = 0.0,
        },
        contactSeparationPolicy(),
    );
    const stabilization = computeContactStabilizationModel(
        .{
            .penetration_depth = contact.manifold.penetration_depth,
            .predicted_penetration_depth = contact.manifold.penetration_depth,
            .separation = separation,
        },
        contactStabilizationPolicy(),
    );
    const medium_buoyancy = computeMediumPostBuoyancyModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .mass = mediumPostBuoyancyPolicy().mass_reference,
            .center_x = contact.centroid.x,
            .center_y = contact.centroid.y,
            .center_z = contact.centroid.z,
            .normal_x = contact.manifold.normal_x,
            .normal_y = contact.manifold.normal_y,
            .normal_z = contact.manifold.normal_z,
        },
        mediumPostBuoyancyPolicy(),
    );
    const medium_drag = computeMediumPostDragModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .normal_speed = 0.0,
            .tangential_speed = 0.0,
        },
        mediumPostDragPolicy(),
    );
    const medium_vapor_resistance = computeMediumPostVaporResistanceModel(
        .{
            .medium_type = response.medium_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .normal_speed = 0.0,
            .tangential_speed = 0.0,
            .drag = medium_drag,
        },
        mediumPostVaporResistancePolicy(),
    );
    const medium_vacuum = computeMediumPostVacuumModel(
        .{
            .medium_type = response.medium_type,
            .ambient_pressure = mediumPostAmbientPressure(response.medium_type),
            .drag = medium_drag,
            .vapor_resistance = medium_vapor_resistance,
            .thermal = thermal,
            .sound = sound,
        },
        mediumPostVacuumPolicy(),
    );
    const medium_transition = computeMediumPostTransitionModel(
        .{
            .from_medium = contact.classification.medium_a,
            .to_medium = contact.classification.medium_b,
            .penetration_depth = contact.manifold.penetration_depth,
            .buoyancy = response.buoyancy,
            .drag = medium_drag,
            .vapor_resistance = medium_vapor_resistance,
            .vacuum = medium_vacuum,
        },
        mediumPostTransitionPolicy(),
    );
    const medium_mixing = computeMediumPostMixingModel(
        .{
            .primary_medium = contact.classification.medium_a,
            .secondary_medium = contact.classification.medium_b,
            .buoyancy = response.buoyancy,
            .transition = medium_transition,
            .drag = medium_drag,
            .vapor_resistance = medium_vapor_resistance,
            .vacuum = medium_vacuum,
        },
        mediumPostMixingPolicy(),
    );
    const medium_state = computeMediumPostStateMachineModel(
        .{
            .current_medium = contact.classification.medium_a,
            .target_medium = contact.classification.medium_b,
            .transition = medium_transition,
            .mixing = medium_mixing,
            .vacuum = medium_vacuum,
        },
        mediumPostStateMachinePolicy(),
    );
    const medium_event = computeMediumPostEventTriggerModel(
        .{
            .state = medium_state,
            .transition = medium_transition,
            .mixing = medium_mixing,
            .vacuum = medium_vacuum,
        },
        mediumPostEventTriggerPolicy(),
    );
    const medium_animation = computeMediumPostTransitionAnimationModel(
        .{
            .state = medium_state,
            .event = medium_event,
            .transition = medium_transition,
            .mixing = medium_mixing,
            .vacuum = medium_vacuum,
        },
        mediumPostTransitionAnimationPolicy(),
    );
    const medium_tow = computeMediumPostTowModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .flow_x = 0.0,
            .flow_y = 0.0,
            .flow_z = 0.0,
            .drag = medium_drag,
        },
        mediumPostTowPolicy(),
    );
    const medium_lift = computeMediumPostLiftModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .flow_x = 0.0,
            .flow_y = 0.0,
            .flow_z = 0.0,
            .normal_x = contact.manifold.normal_x,
            .normal_y = contact.manifold.normal_y,
            .normal_z = contact.manifold.normal_z,
            .tow = medium_tow,
        },
        mediumPostLiftPolicy(),
    );
    const medium_added_mass = computeMediumPostAddedMassModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .base_mass = mediumPostBuoyancyPolicy().mass_reference,
            .buoyancy_model = medium_buoyancy,
            .drag = medium_drag,
            .lift = medium_lift,
        },
        mediumPostAddedMassPolicy(),
    );
    const medium_thermal = computeMediumPostThermalModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .thermal = thermal,
            .drag = medium_drag,
            .tow = medium_tow,
            .lift = medium_lift,
        },
        mediumPostThermalPolicy(),
    );
    const sink_depth = computeSinkDepthModel(
        .{
            .medium_type = response.medium_type,
            .penetration_depth = contact.manifold.penetration_depth,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
            .mass = mediumPostBuoyancyPolicy().mass_reference,
            .penetration_resistance = response.penetration_resistance,
            .buoyancy = response.buoyancy,
        },
        sinkDepthPolicy(),
    );
    const sink_resistance = computeSinkResistanceModel(
        .{
            .sink = sink_depth,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
        },
        sinkResistancePolicy(),
    );
    const sink_recovery = computeSinkRecoveryModel(
        .{
            .sink = sink_depth,
            .resistance = sink_resistance,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
        },
        sinkRecoveryPolicy(),
    );
    const damage_impact = neutralDamageEvalImpactModel();
    const damage_hardness = neutralDamageEvalHardnessModel();
    const damage_break = neutralDamageEvalBreakModel();
    const damage_fragments = neutralDamageEvalFragmentModel();
    const damage_cracks = neutralDamageEvalCrackModel();
    return .{
        .pair = contact.pair,
        .friction = response.friction,
        .restitution = response.restitution,
        .damage_modifier = response.damage_modifier,
        .penetration_resistance = response.penetration_resistance,
        .buoyancy = response.buoyancy,
        .penetration_depth = contact.manifold.penetration_depth,
        .point_count = contact.manifold.point_count,
        .fatigue_damage = fatigue.accumulated_damage,
        .fatigue_remaining_life = fatigue.remaining_life_fraction,
        .fatigue_failed = fatigue.failed,
        .thermal_energy = thermal.generated_heat,
        .thermal_friction_heat = thermal.friction_heat,
        .thermal_friction_heat_fraction = thermal.friction_heat_fraction,
        .thermal_conductivity = thermal.conductivity,
        .thermal_temperature_delta = thermal.temperature_delta,
        .sound_type = sound.sound_type,
        .sound_volume = sound.volume,
        .sound_pitch = sound.pitch,
        .sound_duration = sound.duration,
        .dust_type = dust.dust_type,
        .dust_intensity = dust.intensity,
        .dust_radius = dust.radius,
        .dust_duration = dust.duration,
        .deformation_total = deformation.total_depth,
        .deformation_permanent = deformation.permanent_depth,
        .deformation_recovery_fraction = deformation.recovery_fraction,
        .separation_state = separation.state,
        .separating = separation.separating,
        .separation_speed = separation.speed,
        .separation_time = separation.estimated_time,
        .stabilization_bias_scale = stabilization.bias_scale,
        .stabilization_impulse_scale = stabilization.impulse_scale,
        .stabilized_contact = stabilization.stabilized,
        .damage_impact = @floatFromInt(damage_impact.legacy_impact),
        .damage_energy = damage_impact.kinetic_energy,
        .damage_amount = damage_impact.damage_amount,
        .damage_hardness_threshold = damage_hardness.impact_threshold,
        .damage_hardness_ratio = damage_hardness.impact_ratio,
        .damage_hardness_resistance = damage_hardness.hardness_resistance,
        .damage_exceeds_hardness = damage_hardness.exceeds_hardness,
        .damage_should_break = damage_break.did_break,
        .damage_fragment_count = damage_break.fragments,
        .damage_generated_fragments = damage_fragments.generated_fragments,
        .damage_fragment_energy = damage_fragments.fragment_energy,
        .damage_debris_mass = damage_fragments.debris_mass,
        .damage_crack_count = damage_cracks.crack_count,
        .damage_crack_severity = damage_cracks.max_severity,
        .damage_cracks_propagated = damage_cracks.propagated,
        .medium_buoyancy_force = medium_buoyancy.force,
        .medium_buoyancy_magnitude = medium_buoyancy.magnitude,
        .medium_buoyancy_displaced_volume = medium_buoyancy.displaced_volume,
        .medium_buoyancy_velocity_delta = medium_buoyancy.velocity_delta_y,
        .medium_submerged_fraction = medium_buoyancy.submerged_fraction,
        .medium_buoyancy_center_x = medium_buoyancy.center_x,
        .medium_buoyancy_center_y = medium_buoyancy.center_y,
        .medium_buoyancy_center_z = medium_buoyancy.center_z,
        .medium_buoyancy_active = medium_buoyancy.active,
        .medium_drag_force = medium_drag.force,
        .medium_drag_magnitude = medium_drag.magnitude,
        .medium_drag_coefficient = medium_drag.coefficient,
        .medium_drag_exposure = medium_drag.exposure,
        .medium_drag_normal_delta = medium_drag.normal_velocity_delta,
        .medium_drag_tangent_delta = medium_drag.tangential_velocity_delta,
        .medium_drag_active = medium_drag.active,
        .medium_vapor_resistance_force = medium_vapor_resistance.force,
        .medium_vapor_resistance_coefficient = medium_vapor_resistance.coefficient,
        .medium_vapor_resistance_exposure = medium_vapor_resistance.exposure,
        .medium_vapor_resistance_dynamic_pressure = medium_vapor_resistance.dynamic_pressure,
        .medium_vapor_resistance_normal_delta = medium_vapor_resistance.normal_velocity_delta,
        .medium_vapor_resistance_tangent_delta = medium_vapor_resistance.tangential_velocity_delta,
        .medium_vapor_resistance_active = medium_vapor_resistance.active,
        .medium_vacuum_pressure = medium_vacuum.pressure,
        .medium_vacuum_exposure = medium_vacuum.exposure,
        .medium_vacuum_drag_loss = medium_vacuum.drag_loss,
        .medium_vacuum_thermal_isolation = medium_vacuum.thermal_isolation,
        .medium_vacuum_sound_attenuation = medium_vacuum.sound_attenuation,
        .medium_vacuum_active = medium_vacuum.active,
        .medium_transition_from = @intFromEnum(medium_transition.from_medium),
        .medium_transition_to = @intFromEnum(medium_transition.to_medium),
        .medium_transition_progress = medium_transition.progress,
        .medium_transition_resistance = medium_transition.resistance,
        .medium_transition_pressure_delta = medium_transition.pressure_delta,
        .medium_transition_active = medium_transition.active,
        .medium_mixing_primary = @intFromEnum(medium_mixing.primary_medium),
        .medium_mixing_secondary = @intFromEnum(medium_mixing.secondary_medium),
        .medium_mixing_fraction = medium_mixing.mix_fraction,
        .medium_mixing_effective_density = medium_mixing.effective_density,
        .medium_mixing_effective_viscosity = medium_mixing.effective_viscosity,
        .medium_mixing_blended_drag = medium_mixing.blended_drag,
        .medium_mixing_blended_buoyancy = medium_mixing.blended_buoyancy,
        .medium_mixing_active = medium_mixing.active,
        .medium_state_value = @intFromEnum(medium_state.state),
        .medium_state_current = @intFromEnum(medium_state.current_medium),
        .medium_state_target = @intFromEnum(medium_state.target_medium),
        .medium_state_progress = medium_state.progress,
        .medium_state_stability = medium_state.stability,
        .medium_state_active = medium_state.active,
        .medium_event_type = @intFromEnum(medium_event.event_type),
        .medium_event_source = @intFromEnum(medium_event.source_medium),
        .medium_event_target = @intFromEnum(medium_event.target_medium),
        .medium_event_intensity = medium_event.intensity,
        .medium_event_priority = @intFromEnum(medium_event.priority),
        .medium_event_active = medium_event.active,
        .medium_animation_phase = medium_animation.phase,
        .medium_animation_blend = medium_animation.blend,
        .medium_animation_opacity = medium_animation.opacity,
        .medium_animation_ripple = medium_animation.ripple,
        .medium_animation_turbulence = medium_animation.turbulence,
        .medium_animation_color_shift = medium_animation.color_shift,
        .medium_animation_active = medium_animation.active,
        .medium_tow_force = medium_tow.force,
        .medium_tow_velocity_delta_x = medium_tow.velocity_delta_x,
        .medium_tow_velocity_delta_y = medium_tow.velocity_delta_y,
        .medium_tow_velocity_delta_z = medium_tow.velocity_delta_z,
        .medium_tow_active = medium_tow.active,
        .medium_lift_force = medium_lift.force,
        .medium_lift_magnitude = medium_lift.magnitude,
        .medium_lift_coefficient = medium_lift.coefficient,
        .medium_lift_exposure = medium_lift.exposure,
        .medium_lift_dynamic_pressure = medium_lift.dynamic_pressure,
        .medium_lift_velocity_delta_x = medium_lift.velocity_delta_x,
        .medium_lift_velocity_delta_y = medium_lift.velocity_delta_y,
        .medium_lift_velocity_delta_z = medium_lift.velocity_delta_z,
        .medium_lift_active = medium_lift.active,
        .medium_added_mass_coefficient = medium_added_mass.coefficient,
        .medium_added_mass_exposure = medium_added_mass.exposure,
        .medium_added_mass_displaced_volume = medium_added_mass.displaced_volume,
        .medium_added_mass_value = medium_added_mass.added_mass,
        .medium_added_mass_effective_mass = medium_added_mass.effective_mass,
        .medium_added_mass_normal_inertia_scale = medium_added_mass.normal_inertia_scale,
        .medium_added_mass_tangential_inertia_scale = medium_added_mass.tangential_inertia_scale,
        .medium_added_mass_active = medium_added_mass.active,
        .medium_thermal_conducted_heat = medium_thermal.conducted_heat,
        .medium_thermal_conductivity = medium_thermal.conductivity,
        .medium_thermal_retained_heat = medium_thermal.retained_heat,
        .medium_thermal_temperature_delta = medium_thermal.temperature_delta,
        .medium_thermal_active = medium_thermal.active,
        .sink_depth = sink_depth.depth,
        .sink_load = sink_depth.load,
        .sink_support_fraction = sink_depth.support_fraction,
        .sink_active = sink_depth.active,
        .sink_resistance_force = sink_resistance.force,
        .sink_resistance_normal_delta = sink_resistance.normal_velocity_delta,
        .sink_resistance_tangent_delta = sink_resistance.tangential_velocity_delta,
        .sink_resistance_active = sink_resistance.active,
        .sink_recovery_depth = sink_recovery.depth,
        .sink_recovery_fraction = sink_recovery.fraction,
        .sink_recovery_rate = sink_recovery.rate,
        .sink_recovery_active = sink_recovery.active,
        .sink_recovered = sink_recovery.recovered,
    };
}

fn prepareContactConstraintPairMeasured(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
    stats: ?*ContactNarrowPhaseStats,
) ?ContactPreparedPair {
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return null;

    const entity_a = &entities[inst_a.entity_id];
    const entity_b = &entities[inst_b.entity_id];
    const inv_mass_a = instanceInverseMass(inst_a, entities);
    const inv_mass_b = instanceInverseMass(inst_b, entities);
    const aabb_a = physics.computeEntityWorldAABB(inst_a, entity_a) orelse return null;
    const aabb_b = physics.computeEntityWorldAABB(inst_b, entity_b) orelse return null;
    if (stats) |s| s.aabb_builds += 2;
    const prediction_dt = kernelPredictionDt();
    const predicted_pos_a = predictKernelInstanceState(inst_a, prediction_dt);
    const predicted_pos_b = predictKernelInstanceState(inst_b, prediction_dt);
    const predicted_penetration_depth = computePredictedAABBOverlapDepth(
        aabb_a,
        inst_a,
        predicted_pos_a,
        aabb_b,
        inst_b,
        predicted_pos_b,
    );
    const probe = probeContactConstraintWithAABBs(inst_a, inst_b, entity_a, entity_b, aabb_a, aabb_b, stats);

    if (probe == null) {
        if (predicted_penetration_depth <= 0.0) return null;
        const static_a = (entity_a.physics.flags & 0x01) != 0;
        const static_b = (entity_b.physics.flags & 0x01) != 0;
        if (static_a or static_b) return null;
        const normal_direction = buildPredictedAABBSeparationDirection(
            aabb_a,
            inst_a,
            predicted_pos_a,
            aabb_b,
            inst_b,
            predicted_pos_b,
            predicted_penetration_depth,
        );
        const normal_row_plan = buildContactNormalRowPlan(
            inv_mass_a,
            inv_mass_b,
            normal_direction.depth,
            0.0,
            normal_direction,
        );
        const speculative_normal_equation = applySpeculativeContactBiasBoost(
            normal_row_plan.equation,
            normal_direction,
        );
        return .{
            .inv_mass_a = inv_mass_a,
            .inv_mass_b = inv_mass_b,
            .speculative = true,
            .normal = withPredictiveResidualHint(normal_direction, normal_row_plan.metadata.predictive_residual_hint),
            .restitution = 0.0,
            .friction = 0.0,
            .rolling_friction = 0.0,
            .anisotropic_friction_minor_axis_scale = 1.0,
            .fatigue = neutralContactFatigueModel(),
            .medium_buoyancy = neutralMediumPostBuoyancyModel(),
            .medium_drag = neutralMediumPostDragModel(),
            .medium_vapor_resistance = neutralMediumPostVaporResistanceModel(),
            .medium_vacuum = neutralMediumPostVacuumModel(),
            .medium_transition = neutralMediumPostTransitionModel(),
            .medium_mixing = neutralMediumPostMixingModel(),
            .medium_state = neutralMediumPostStateMachineModel(),
            .medium_event = neutralMediumPostEventTriggerModel(),
            .medium_animation = neutralMediumPostTransitionAnimationModel(),
            .medium_tow = neutralMediumPostTowModel(),
            .medium_lift = neutralMediumPostLiftModel(),
            .medium_added_mass = neutralMediumPostAddedMassModel(),
            .medium_thermal = neutralMediumPostThermalModel(),
            .sink_depth = neutralSinkDepthModel(),
            .sink_resistance = neutralSinkResistanceModel(),
            .sink_recovery = neutralSinkRecoveryModel(),
            .thermal = neutralContactThermalModel(),
            .sound = neutralContactSoundModel(),
            .dust = neutralContactDustModel(),
            .deformation = neutralContactDeformationModel(),
            .separation = neutralContactSeparationModel(),
            .stabilization = neutralContactStabilizationModel(),
            .damage_impact = neutralDamageEvalImpactModel(),
            .damage_hardness = neutralDamageEvalHardnessModel(),
            .damage_break = neutralDamageEvalBreakModel(),
            .damage_fragments = neutralDamageEvalFragmentModel(),
            .damage_cracks = neutralDamageEvalCrackModel(),
            .tangent = buildPreparedDirectionalConstraint(0.0, 0.0, 0.0, 0.0, 0.0),
            .has_tangent = false,
            .normal_equation = speculative_normal_equation,
            .friction_equation = zeroConstraintEquation(),
        };
    }

    const resolved_probe = probe.?;
    const surface_a = material_pairing.getSurfaceForMaterial(resolved_probe.entity_a.physics.material);
    const surface_b = material_pairing.getSurfaceForMaterial(resolved_probe.entity_b.physics.material);
    const response = material_pairing.getPairedResponse(
        surface_a,
        resolved_probe.entity_a.physics.material,
        surface_b,
        resolved_probe.entity_b.physics.material,
    );
    const normal_direction = buildPreparedDirectionalConstraint(
        resolved_probe.manifold.normal_x,
        resolved_probe.manifold.normal_y,
        resolved_probe.manifold.normal_z,
        resolved_probe.manifold.penetration_depth,
        predicted_penetration_depth,
    );
    const stiffness = computeContactStiffnessModel(
        response.penetration_resistance,
        contactStiffnessPolicy(),
    );
    const damping = computeContactDampingModel(
        response.restitution,
        contactDampingPolicy(),
    );
    const separation = computeContactSeparationModel(
        .{
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .predicted_penetration_depth = predicted_penetration_depth,
            .normal_speed_signed = resolved_probe.velocity_components.normal_speed_signed,
        },
        contactSeparationPolicy(),
    );
    const stabilization = computeContactStabilizationModel(
        .{
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .predicted_penetration_depth = predicted_penetration_depth,
            .separation = separation,
        },
        contactStabilizationPolicy(),
    );
    const normal_row_plan = buildContactNormalRowPlanWithResponseAndStabilization(
        inv_mass_a,
        inv_mass_b,
        resolved_probe.manifold.penetration_depth,
        resolved_probe.velocity_components.normalSpeed(),
        normal_direction,
        stiffness,
        damping,
        stabilization,
    );
    const friction_policy = contactFrictionSolvePolicy();
    const initial_tangential_speed = resolved_probe.velocity_components.tangentialSpeed();
    const friction_cone = approximateContactFrictionCone(
        resolved_probe.velocity_components,
        computeContactEffectiveFrictionCoefficient(response.friction, initial_tangential_speed),
        normal_row_plan.equation.max_impulse,
        friction_policy,
    );
    const has_tangent = friction_cone != null;
    const tangential_speed = if (friction_cone) |cone| cone.tangential_speed else 0.0;
    const dynamic_friction = computeContactDynamicFrictionCoefficient(response.friction, tangential_speed);
    const effective_friction = computeContactStaticFrictionCoefficient(dynamic_friction, tangential_speed);
    const angular_speed = contactAngularSpeed(inst_a, inst_b);
    const rolling_friction = computeContactRollingFrictionCoefficient(
        dynamic_friction,
        angular_speed,
    );
    const anisotropic_minor_axis_scale = computeContactAnisotropicFrictionMinorAxisScale(surface_a, surface_b);
    const fatigue = computeContactFatigueModel(
        .{
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
            .damage_modifier = response.damage_modifier,
            .penetration_resistance = response.penetration_resistance,
        },
        contactFatiguePolicy(),
    );
    const contact_centroid = collision.computeContactManifoldCentroid(resolved_probe.manifold);
    const medium_buoyancy = computeMediumPostBuoyancyModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .mass = @floatFromInt(resolved_probe.entity_a.physics.mass),
            .center_x = contact_centroid.x,
            .center_y = contact_centroid.y,
            .center_z = contact_centroid.z,
            .normal_x = resolved_probe.manifold.normal_x,
            .normal_y = resolved_probe.manifold.normal_y,
            .normal_z = resolved_probe.manifold.normal_z,
        },
        mediumPostBuoyancyPolicy(),
    );
    const medium_drag = computeMediumPostDragModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
        },
        mediumPostDragPolicy(),
    );
    const medium_vapor_resistance = computeMediumPostVaporResistanceModel(
        .{
            .medium_type = response.medium_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
            .drag = medium_drag,
        },
        mediumPostVaporResistancePolicy(),
    );
    const medium_vacuum = computeMediumPostVacuumModel(
        .{
            .medium_type = response.medium_type,
            .ambient_pressure = mediumPostAmbientPressure(response.medium_type),
            .drag = medium_drag,
            .vapor_resistance = medium_vapor_resistance,
            .thermal = neutralContactThermalModel(),
            .sound = neutralContactSoundModel(),
        },
        mediumPostVacuumPolicy(),
    );
    const medium_transition = computeMediumPostTransitionModel(
        .{
            .from_medium = material_pairing.getMediumType(surface_a),
            .to_medium = material_pairing.getMediumType(surface_b),
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .buoyancy = response.buoyancy,
            .drag = medium_drag,
            .vapor_resistance = medium_vapor_resistance,
            .vacuum = medium_vacuum,
        },
        mediumPostTransitionPolicy(),
    );
    const medium_mixing = computeMediumPostMixingModel(
        .{
            .primary_medium = material_pairing.getMediumType(surface_a),
            .secondary_medium = material_pairing.getMediumType(surface_b),
            .buoyancy = response.buoyancy,
            .transition = medium_transition,
            .drag = medium_drag,
            .vapor_resistance = medium_vapor_resistance,
            .vacuum = medium_vacuum,
        },
        mediumPostMixingPolicy(),
    );
    const medium_state = computeMediumPostStateMachineModel(
        .{
            .current_medium = material_pairing.getMediumType(surface_a),
            .target_medium = material_pairing.getMediumType(surface_b),
            .transition = medium_transition,
            .mixing = medium_mixing,
            .vacuum = medium_vacuum,
        },
        mediumPostStateMachinePolicy(),
    );
    const medium_event = computeMediumPostEventTriggerModel(
        .{
            .state = medium_state,
            .transition = medium_transition,
            .mixing = medium_mixing,
            .vacuum = medium_vacuum,
        },
        mediumPostEventTriggerPolicy(),
    );
    const medium_animation = computeMediumPostTransitionAnimationModel(
        .{
            .state = medium_state,
            .event = medium_event,
            .transition = medium_transition,
            .mixing = medium_mixing,
            .vacuum = medium_vacuum,
        },
        mediumPostTransitionAnimationPolicy(),
    );
    const medium_tow = computeMediumPostTowModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .flow_x = if (friction_cone) |cone| cone.dir_x * cone.tangential_speed else 0.0,
            .flow_y = if (friction_cone) |cone| cone.dir_y * cone.tangential_speed else 0.0,
            .flow_z = if (friction_cone) |cone| cone.dir_z * cone.tangential_speed else 0.0,
            .drag = medium_drag,
        },
        mediumPostTowPolicy(),
    );
    const medium_lift = computeMediumPostLiftModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .flow_x = if (friction_cone) |cone| cone.dir_x * cone.tangential_speed else 0.0,
            .flow_y = if (friction_cone) |cone| cone.dir_y * cone.tangential_speed else 0.0,
            .flow_z = if (friction_cone) |cone| cone.dir_z * cone.tangential_speed else 0.0,
            .normal_x = resolved_probe.manifold.normal_x,
            .normal_y = resolved_probe.manifold.normal_y,
            .normal_z = resolved_probe.manifold.normal_z,
            .tow = medium_tow,
        },
        mediumPostLiftPolicy(),
    );
    const medium_added_mass = computeMediumPostAddedMassModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .base_mass = @floatFromInt(resolved_probe.entity_a.physics.mass),
            .buoyancy_model = medium_buoyancy,
            .drag = medium_drag,
            .lift = medium_lift,
        },
        mediumPostAddedMassPolicy(),
    );
    const sink_depth = computeSinkDepthModel(
        .{
            .medium_type = response.medium_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
            .mass = @floatFromInt(resolved_probe.entity_a.physics.mass),
            .penetration_resistance = response.penetration_resistance,
            .buoyancy = response.buoyancy,
        },
        sinkDepthPolicy(),
    );
    const sink_resistance = computeSinkResistanceModel(
        .{
            .sink = sink_depth,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
        },
        sinkResistancePolicy(),
    );
    const sink_recovery = computeSinkRecoveryModel(
        .{
            .sink = sink_depth,
            .resistance = sink_resistance,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
        },
        sinkRecoveryPolicy(),
    );
    const thermal = computeContactThermalModel(
        .{
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
            .friction = effective_friction,
            .rolling_friction = rolling_friction,
            .angular_speed = angular_speed,
            .anisotropic_minor_axis_scale = anisotropic_minor_axis_scale,
            .penetration_resistance = response.penetration_resistance,
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
        },
        contactThermalPolicy(),
    );
    const medium_thermal = computeMediumPostThermalModel(
        .{
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
            .thermal = thermal,
            .drag = medium_drag,
            .tow = medium_tow,
            .lift = medium_lift,
        },
        mediumPostThermalPolicy(),
    );
    const sound = computeContactSoundModel(
        .{
            .sound_type = response.sound_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
            .friction = response.friction,
            .restitution = response.restitution,
            .medium_type = response.medium_type,
        },
        contactSoundPolicy(),
    );
    const dust = computeContactDustModel(
        .{
            .dust_type = response.dust_type,
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
            .friction = response.friction,
            .buoyancy = response.buoyancy,
            .medium_type = response.medium_type,
        },
        contactDustPolicy(),
    );
    const deformation = computeContactDeformationModel(
        .{
            .penetration_depth = resolved_probe.manifold.penetration_depth,
            .relative_normal_speed = resolved_probe.velocity_components.normalSpeed(),
            .tangential_speed = tangential_speed,
            .penetration_resistance = response.penetration_resistance,
            .damage_modifier = response.damage_modifier,
            .restitution = response.restitution,
        },
        contactDeformationPolicy(),
    );
    const damage_impact = computeDamageEvalImpactForEntity(
        resolved_probe.entity_a,
        resolved_probe.velocity_components.normalSpeed(),
        tangential_speed,
        response.damage_modifier,
    );
    const damage_hardness = computeDamageEvalHardnessForEntity(resolved_probe.entity_a, damage_impact);
    const damage_break = computeDamageEvalBreakForEntity(resolved_probe.entity_a, damage_impact, damage_hardness);
    const damage_fragments = computeDamageEvalFragmentsForEntity(
        resolved_probe.entity_a,
        damage_impact,
        damage_break,
        damageEvalImpactVelocityHint(damage_impact),
        1.0,
    );
    const damage_cracks = computeDamageEvalCracksForEntity(
        resolved_probe.entity_a,
        damage_impact,
        damage_hardness,
        damage_break,
        damage_fragments,
    );
    const tangent_direction = buildPreparedDirectionalConstraint(
        if (friction_cone) |cone| cone.dir_x else 0.0,
        if (friction_cone) |cone| cone.dir_y else 0.0,
        if (friction_cone) |cone| cone.dir_z else 0.0,
        tangential_speed,
        tangential_speed,
    );
    const friction_row_plan = buildContactFrictionRowPlan(
        inv_mass_a,
        inv_mass_b,
        tangential_speed,
        effective_friction,
        normal_row_plan.equation.max_impulse,
        tangent_direction,
    );

    return .{
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .speculative = false,
        .normal = withPredictiveResidualHint(normal_direction, normal_row_plan.metadata.predictive_residual_hint),
        .restitution = response.restitution,
        .friction = effective_friction,
        .rolling_friction = rolling_friction,
        .anisotropic_friction_minor_axis_scale = anisotropic_minor_axis_scale,
        .fatigue = fatigue,
        .medium_buoyancy = medium_buoyancy,
        .medium_drag = medium_drag,
        .medium_vapor_resistance = medium_vapor_resistance,
        .medium_vacuum = medium_vacuum,
        .medium_transition = medium_transition,
        .medium_mixing = medium_mixing,
        .medium_state = medium_state,
        .medium_event = medium_event,
        .medium_animation = medium_animation,
        .medium_tow = medium_tow,
        .medium_lift = medium_lift,
        .medium_added_mass = medium_added_mass,
        .medium_thermal = medium_thermal,
        .sink_depth = sink_depth,
        .sink_resistance = sink_resistance,
        .sink_recovery = sink_recovery,
        .thermal = thermal,
        .sound = sound,
        .dust = dust,
        .deformation = deformation,
        .separation = separation,
        .stabilization = stabilization,
        .damage_impact = damage_impact,
        .damage_hardness = damage_hardness,
        .damage_break = damage_break,
        .damage_fragments = damage_fragments,
        .damage_cracks = damage_cracks,
        .tangent = tangent_direction,
        .has_tangent = has_tangent,
        .normal_equation = normal_row_plan.equation,
        .friction_equation = friction_row_plan.equation,
    };
}

fn prepareContactConstraintPair(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ContactPreparedPair {
    return prepareContactConstraintPairMeasured(inst_a, inst_b, entities, null);
}

fn computeContactSolvePriorityMagnitudeMeasured(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
    stats: ?*ContactNarrowPhaseStats,
) f32 {
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return 0.0;

    const entity_a = &entities[inst_a.entity_id];
    const entity_b = &entities[inst_b.entity_id];
    const aabb_a = physics.computeEntityWorldAABB(inst_a, entity_a) orelse return 0.0;
    const aabb_b = physics.computeEntityWorldAABB(inst_b, entity_b) orelse return 0.0;
    if (stats) |s| s.aabb_builds += 2;
    const prediction_dt = kernelPredictionDt();
    const predicted_pos_a = predictKernelInstanceState(inst_a, prediction_dt);
    const predicted_pos_b = predictKernelInstanceState(inst_b, prediction_dt);
    const predicted_penetration_depth = computePredictedAABBOverlapDepth(
        aabb_a,
        inst_a,
        predicted_pos_a,
        aabb_b,
        inst_b,
        predicted_pos_b,
    );
    const probe = probeContactConstraintWithAABBs(inst_a, inst_b, entity_a, entity_b, aabb_a, aabb_b, stats) orelse {
        if (predicted_penetration_depth <= 0.0) return 0.0;
        return computeDirectionalPredictiveSolvePriorityMagnitude(
            0.0,
            buildPreparedDirectionalConstraint(
                0.0,
                0.0,
                0.0,
                0.0,
                predicted_penetration_depth,
            ),
            contactNormalPredictiveConstraintPolicy(),
        );
    };
    const metrics: ContactConstraintMetrics = .{
        .penetration_depth = probe.manifold.penetration_depth,
        .normal_speed = probe.velocity_components.normalSpeed(),
        .tangential_speed = probe.velocity_components.tangentialSpeed(),
    };
    return computeDirectionalPredictiveSolvePriorityMagnitude(
        metrics.stress(),
        buildPreparedDirectionalConstraint(
            probe.manifold.normal_x,
            probe.manifold.normal_y,
            probe.manifold.normal_z,
            probe.manifold.penetration_depth,
            predicted_penetration_depth,
        ),
        contactNormalPredictiveConstraintPolicy(),
    );
}

fn computeContactSolvePriorityMagnitude(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) f32 {
    return computeContactSolvePriorityMagnitudeMeasured(inst_a, inst_b, entities, null);
}

fn computeMaxActiveContactConstraintMagnitude(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) f32 {
    var max_magnitude: f32 = 0.0;

    for (broadphase_pairs) |pair| {
        if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;
        const inst_a = &s1024.instances[pair.a];
        const inst_b = &s1024.instances[pair.b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        max_magnitude = @max(
            max_magnitude,
            computeContactSolvePriorityMagnitude(inst_a, inst_b, entities),
        );
    }

    return max_magnitude;
}

fn computeContactIterationBudget(broadphase_pairs: []const BroadPhasePair, max_stress: f32) u8 {
    var budget: u8 = if (broadphase_pairs.len >= 16)
        4
    else if (broadphase_pairs.len >= 8)
        3
    else
        2;

    if (max_stress >= 1.0) budget = addIterationBudgetWithClamp(budget, 1);
    if (max_stress >= 4.0) budget = addIterationBudgetWithClamp(budget, 1);
    return clampIterationBudget(budget);
}

const ITERATION_BUDGET_CAP: u8 = 6;
const STRESS_SETTLE_HIGH_THRESHOLD: f32 = 4.0;
const STRESS_SETTLE_MID_THRESHOLD: f32 = 1.0;
const STRESS_SETTLE_HIGH_VALUE: f32 = 0.01;
const STRESS_SETTLE_MID_VALUE: f32 = 0.025;
const STRESS_SETTLE_BASE_VALUE: f32 = 0.05;

fn clampIterationBudget(budget: u8) u8 {
    return @min(ITERATION_BUDGET_CAP, budget);
}

fn addIterationBudgetWithClamp(base_budget: u8, extra_passes: u8) u8 {
    const combined: u16 = @as(u16, base_budget) + @as(u16, extra_passes);
    return clampIterationBudget(@intCast(@min(combined, @as(u16, 255))));
}

const SpeculativeContactSummary = struct {
    pair_count: usize = 0,
    max_tier: SpeculativeContactBiasTier = .none,
};

fn mergeHigherSpeculativeContactTier(
    current: SpeculativeContactBiasTier,
    candidate: SpeculativeContactBiasTier,
) SpeculativeContactBiasTier {
    if (@intFromEnum(candidate) > @intFromEnum(current)) return candidate;
    return current;
}

fn measureSpeculativeContactSummary(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) SpeculativeContactSummary {
    var summary: SpeculativeContactSummary = .{};
    for (broadphase_pairs) |pair| {
        if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;
        const inst_a = &s1024.instances[pair.a];
        const inst_b = &s1024.instances[pair.b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;
        const prepared = prepareContactConstraintPair(inst_a, inst_b, entities) orelse continue;
        if (!prepared.speculative) continue;

        summary.pair_count += 1;
        const tier = measureSpeculativeContactBiasProfileDefault(prepared.normal).tier;
        summary.max_tier = mergeHigherSpeculativeContactTier(summary.max_tier, tier);
    }
    return summary;
}

fn countSpeculativeContactPairs(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) usize {
    return measureSpeculativeContactSummary(s1024, entities, broadphase_pairs).pair_count;
}

fn measureMaxSpeculativeContactTier(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) SpeculativeContactBiasTier {
    return measureSpeculativeContactSummary(s1024, entities, broadphase_pairs).max_tier;
}

const SpeculativeContactIterationBudgetPolicy = struct {
    low_tier_extra_passes: u8,
    high_tier_extra_passes: u8,
    dense_pair_threshold: usize,
    dense_pair_extra_passes: u8,
};

fn speculativeContactIterationBudgetPolicy() SpeculativeContactIterationBudgetPolicy {
    return .{
        .low_tier_extra_passes = 1,
        .high_tier_extra_passes = 2,
        .dense_pair_threshold = 4,
        .dense_pair_extra_passes = 1,
    };
}

fn computeSpeculativeIterationExtraPasses(
    policy: SpeculativeContactIterationBudgetPolicy,
    speculative_pair_count: usize,
    max_speculative_tier: SpeculativeContactBiasTier,
) u8 {
    if (speculative_pair_count == 0) return 0;

    var extra_passes: u8 = switch (max_speculative_tier) {
        .none, .base, .mid => policy.low_tier_extra_passes,
        .high => policy.high_tier_extra_passes,
    };
    if (speculative_pair_count >= policy.dense_pair_threshold) {
        extra_passes = addIterationBudgetWithClamp(extra_passes, policy.dense_pair_extra_passes);
    }
    return extra_passes;
}

fn boostIterationBudgetForSpeculativeContacts(
    base_budget: u8,
    speculative_pair_count: usize,
    max_speculative_tier: SpeculativeContactBiasTier,
) u8 {
    const policy = speculativeContactIterationBudgetPolicy();
    const extra_passes = computeSpeculativeIterationExtraPasses(policy, speculative_pair_count, max_speculative_tier);
    return addIterationBudgetWithClamp(base_budget, extra_passes);
}

fn boostIterationBudgetForSpeculativePairs(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
    base_budget: u8,
) u8 {
    if (base_budget >= ITERATION_BUDGET_CAP) return ITERATION_BUDGET_CAP;
    if (broadphase_pairs.len == 0) return clampIterationBudget(base_budget);
    const speculative_summary = measureSpeculativeContactSummary(s1024, entities, broadphase_pairs);
    return boostIterationBudgetForSpeculativeContacts(
        base_budget,
        speculative_summary.pair_count,
        speculative_summary.max_tier,
    );
}

fn computeStressSettleThreshold(max_stress: f32) f32 {
    if (max_stress >= STRESS_SETTLE_HIGH_THRESHOLD) return STRESS_SETTLE_HIGH_VALUE;
    if (max_stress >= STRESS_SETTLE_MID_THRESHOLD) return STRESS_SETTLE_MID_VALUE;
    return STRESS_SETTLE_BASE_VALUE;
}

fn computeContactSettleThreshold(max_stress: f32) f32 {
    return computeStressSettleThreshold(max_stress);
}

fn queryEnvironmentPenetrationForAABB(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    aabb: physics.AABB,
    dx: i32,
    dy: i32,
    dz: i32,
) ?query_types.PenetrationResult {
    const world_view: query_types.QueryWorldView = .{
        .s1024 = s1024,
        .instances = s1024.instances[0..s1024.instance_count],
        .entities = entities,
    };
    const penetration = query_penetration.computePenetrationAABB(
        &world_view,
        @floatFromInt(aabb.min_x + dx),
        @floatFromInt(aabb.min_y + dy),
        @floatFromInt(aabb.min_z + dz),
        @floatFromInt(aabb.max_x + dx),
        @floatFromInt(aabb.max_y + dy),
        @floatFromInt(aabb.max_z + dz),
        .{
            .include_static = false,
            .include_dynamic = false,
            .include_kinematic = false,
            .include_sensors = false,
            .ignore_environment = false,
            .ignore_instance_idx = instance_idx,
        },
    );
    if (!penetration.overlapping or penetration.depth <= 0.0 or penetration.instance_idx != -1) return null;
    return penetration;
}

fn probeEnvironmentPenetration(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) ?EnvironmentPenetrationProbe {
    if (instance_idx >= s1024.instance_count) return null;
    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return null;
    if (inst.entity_id >= entities.len) return null;

    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return null;

    const aabb = physics.computeEntityWorldAABB(inst, entity) orelse return null;
    const penetration = queryEnvironmentPenetrationForAABB(s1024, entities, instance_idx, aabb, 0, 0, 0) orelse return null;

    return .{
        .inst = inst,
        .entity = entity,
        .penetration = penetration,
    };
}

fn measureEnvironmentConstraintMetrics(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) ?EnvironmentConstraintMetrics {
    const probe = probeEnvironmentPenetration(s1024, entities, instance_idx) orelse return null;

    const velocity_components = measureDirectionalVelocityComponents(
        @as(f32, @floatFromInt(probe.inst.vel_x)),
        @as(f32, @floatFromInt(probe.inst.vel_y)),
        @as(f32, @floatFromInt(probe.inst.vel_z)),
        probe.penetration.dir_x,
        probe.penetration.dir_y,
        probe.penetration.dir_z,
    );

    return .{
        .penetration_depth = probe.penetration.depth,
        .normal_speed = velocity_components.normalSpeed(),
        .tangential_speed = velocity_components.tangentialSpeed(),
    };
}

fn prepareEnvironmentConstraint(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) ?PreparedEnvironmentConstraint {
    const probe = probeEnvironmentPenetration(s1024, entities, instance_idx) orelse return null;

    const move_x = @as(i32, @intFromFloat(@round(probe.penetration.dir_x * probe.penetration.depth)));
    const move_y = @as(i32, @intFromFloat(@round(probe.penetration.dir_y * probe.penetration.depth)));
    const move_z = @as(i32, @intFromFloat(@round(probe.penetration.dir_z * probe.penetration.depth)));
    if (move_x == 0 and move_y == 0 and move_z == 0) return null;

    return .{
        .normal = buildPreparedDirectionalConstraint(
            probe.penetration.dir_x,
            probe.penetration.dir_y,
            probe.penetration.dir_z,
            probe.penetration.depth,
            measurePredictiveEnvironmentDepth(s1024, entities, instance_idx, kernelPredictionDt()),
        ),
        .move_x = move_x,
        .move_y = move_y,
        .move_z = move_z,
    };
}

fn measurePredictiveEnvironmentDepth(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    prediction_dt: f32,
) f32 {
    if (instance_idx >= s1024.instance_count) return 0.0;
    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return 0.0;
    if (inst.entity_id >= entities.len) return 0.0;

    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return 0.0;

    const aabb = physics.computeEntityWorldAABB(inst, entity) orelse return 0.0;
    const predicted_state = predictKernelInstanceState(inst, prediction_dt);
    const dx = @as(i32, @intFromFloat(@round(predicted_state.pos_x))) - inst.pos_x;
    const dy = @as(i32, @intFromFloat(@round(predicted_state.pos_y))) - inst.pos_y;
    const dz = @as(i32, @intFromFloat(@round(predicted_state.pos_z))) - inst.pos_z;
    const penetration = queryEnvironmentPenetrationForAABB(s1024, entities, instance_idx, aabb, dx, dy, dz) orelse return 0.0;
    return penetration.depth;
}

fn computeEnvironmentSolvePriorityMagnitude(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) f32 {
    const metrics = measureEnvironmentConstraintMetrics(s1024, entities, instance_idx) orelse return 0.0;
    const direction = buildPreparedDirectionalConstraint(
        0.0,
        0.0,
        0.0,
        metrics.penetration_depth,
        measurePredictiveEnvironmentDepth(s1024, entities, instance_idx, kernelPredictionDt()),
    );
    return computeDirectionalPredictiveSolvePriorityMagnitude(
        metrics.stress(),
        direction,
        environmentPredictiveConstraintPolicy(),
    );
}

fn measureDirectionalVelocityComponents(
    rel_vel_x: f32,
    rel_vel_y: f32,
    rel_vel_z: f32,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
) DirectionalVelocityComponents {
    const normal_speed_signed = rel_vel_x * dir_x + rel_vel_y * dir_y + rel_vel_z * dir_z;
    return .{
        .normal_speed_signed = normal_speed_signed,
        .tangent_x = rel_vel_x - normal_speed_signed * dir_x,
        .tangent_y = rel_vel_y - normal_speed_signed * dir_y,
        .tangent_z = rel_vel_z - normal_speed_signed * dir_z,
    };
}

fn buildDirectionalTangentFrame(
    velocity_components: DirectionalVelocityComponents,
) ?DirectionalTangentFrame {
    const tangential_speed = velocity_components.tangentialSpeed();
    if (tangential_speed <= 0.0001) return null;
    return .{
        .dir_x = velocity_components.tangent_x / tangential_speed,
        .dir_y = velocity_components.tangent_y / tangential_speed,
        .dir_z = velocity_components.tangent_z / tangential_speed,
        .speed = tangential_speed,
    };
}

fn computePredictedAABBOverlapDepth(
    aabb_a: physics.AABB,
    origin_a: *const scene32.Instance,
    predicted_a: prediction.LinearState,
    aabb_b: physics.AABB,
    origin_b: *const scene32.Instance,
    predicted_b: prediction.LinearState,
) f32 {
    const predicted_min_ax = predicted_a.pos_x + @as(f32, @floatFromInt(aabb_a.min_x - origin_a.pos_x));
    const predicted_max_ax = predicted_a.pos_x + @as(f32, @floatFromInt(aabb_a.max_x - origin_a.pos_x));
    const predicted_min_ay = predicted_a.pos_y + @as(f32, @floatFromInt(aabb_a.min_y - origin_a.pos_y));
    const predicted_max_ay = predicted_a.pos_y + @as(f32, @floatFromInt(aabb_a.max_y - origin_a.pos_y));
    const predicted_min_az = predicted_a.pos_z + @as(f32, @floatFromInt(aabb_a.min_z - origin_a.pos_z));
    const predicted_max_az = predicted_a.pos_z + @as(f32, @floatFromInt(aabb_a.max_z - origin_a.pos_z));
    const predicted_min_bx = predicted_b.pos_x + @as(f32, @floatFromInt(aabb_b.min_x - origin_b.pos_x));
    const predicted_max_bx = predicted_b.pos_x + @as(f32, @floatFromInt(aabb_b.max_x - origin_b.pos_x));
    const predicted_min_by = predicted_b.pos_y + @as(f32, @floatFromInt(aabb_b.min_y - origin_b.pos_y));
    const predicted_max_by = predicted_b.pos_y + @as(f32, @floatFromInt(aabb_b.max_y - origin_b.pos_y));
    const predicted_min_bz = predicted_b.pos_z + @as(f32, @floatFromInt(aabb_b.min_z - origin_b.pos_z));
    const predicted_max_bz = predicted_b.pos_z + @as(f32, @floatFromInt(aabb_b.max_z - origin_b.pos_z));
    const predicted_overlap_x = @min(predicted_max_ax, predicted_max_bx) - @max(predicted_min_ax, predicted_min_bx);
    const predicted_overlap_y = @min(predicted_max_ay, predicted_max_by) - @max(predicted_min_ay, predicted_min_by);
    const predicted_overlap_z = @min(predicted_max_az, predicted_max_bz) - @max(predicted_min_az, predicted_min_bz);
    return @max(0.0, @min(predicted_overlap_x, @min(predicted_overlap_y, predicted_overlap_z)));
}

fn buildPredictedAABBSeparationDirection(
    aabb_a: physics.AABB,
    origin_a: *const scene32.Instance,
    predicted_a: prediction.LinearState,
    aabb_b: physics.AABB,
    origin_b: *const scene32.Instance,
    predicted_b: prediction.LinearState,
    predicted_penetration_depth: f32,
) PreparedDirectionalConstraint {
    const predicted_min_ax = predicted_a.pos_x + @as(f32, @floatFromInt(aabb_a.min_x - origin_a.pos_x));
    const predicted_max_ax = predicted_a.pos_x + @as(f32, @floatFromInt(aabb_a.max_x - origin_a.pos_x));
    const predicted_min_ay = predicted_a.pos_y + @as(f32, @floatFromInt(aabb_a.min_y - origin_a.pos_y));
    const predicted_max_ay = predicted_a.pos_y + @as(f32, @floatFromInt(aabb_a.max_y - origin_a.pos_y));
    const predicted_min_az = predicted_a.pos_z + @as(f32, @floatFromInt(aabb_a.min_z - origin_a.pos_z));
    const predicted_max_az = predicted_a.pos_z + @as(f32, @floatFromInt(aabb_a.max_z - origin_a.pos_z));
    const predicted_min_bx = predicted_b.pos_x + @as(f32, @floatFromInt(aabb_b.min_x - origin_b.pos_x));
    const predicted_max_bx = predicted_b.pos_x + @as(f32, @floatFromInt(aabb_b.max_x - origin_b.pos_x));
    const predicted_min_by = predicted_b.pos_y + @as(f32, @floatFromInt(aabb_b.min_y - origin_b.pos_y));
    const predicted_max_by = predicted_b.pos_y + @as(f32, @floatFromInt(aabb_b.max_y - origin_b.pos_y));
    const predicted_min_bz = predicted_b.pos_z + @as(f32, @floatFromInt(aabb_b.min_z - origin_b.pos_z));
    const predicted_max_bz = predicted_b.pos_z + @as(f32, @floatFromInt(aabb_b.max_z - origin_b.pos_z));
    const predicted_overlap_x = @min(predicted_max_ax, predicted_max_bx) - @max(predicted_min_ax, predicted_min_bx);
    const predicted_overlap_y = @min(predicted_max_ay, predicted_max_by) - @max(predicted_min_ay, predicted_min_by);
    const predicted_overlap_z = @min(predicted_max_az, predicted_max_bz) - @max(predicted_min_az, predicted_min_bz);
    const center_dx = predicted_b.pos_x - predicted_a.pos_x;
    const center_dy = predicted_b.pos_y - predicted_a.pos_y;
    const center_dz = predicted_b.pos_z - predicted_a.pos_z;

    if (predicted_overlap_x <= predicted_overlap_y and predicted_overlap_x <= predicted_overlap_z) {
        return buildPreparedDirectionalConstraint(
            if (center_dx >= 0.0) -1.0 else 1.0,
            0.0,
            0.0,
            predicted_penetration_depth,
            predicted_penetration_depth,
        );
    }
    if (predicted_overlap_y <= predicted_overlap_z) {
        return buildPreparedDirectionalConstraint(
            0.0,
            if (center_dy >= 0.0) -1.0 else 1.0,
            0.0,
            predicted_penetration_depth,
            predicted_penetration_depth,
        );
    }
    return buildPreparedDirectionalConstraint(
        0.0,
        0.0,
        if (center_dz >= 0.0) -1.0 else 1.0,
        predicted_penetration_depth,
        predicted_penetration_depth,
    );
}

fn buildEnvironmentPreparedRowPlan(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) DirectionalRowPlan {
    if (instance_idx >= s1024.instance_count) return zeroDirectionalRowPlan();
    const inst = &s1024.instances[instance_idx];
    const prepared = prepareEnvironmentConstraint(s1024, entities, instance_idx) orelse return zeroDirectionalRowPlan();
    const metrics = measureEnvironmentConstraintMetrics(s1024, entities, instance_idx) orelse return zeroDirectionalRowPlan();
    return buildEnvironmentDirectionalRowPlan(
        instanceInverseMass(inst, entities),
        metrics,
        prepared.normal,
    );
}

fn computeMaxActiveEnvironmentConstraintMagnitude(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) f32 {
    var max_magnitude: f32 = 0.0;
    var idx: u8 = 0;
    while (idx < s1024.instance_count) : (idx += 1) {
        max_magnitude = @max(max_magnitude, computeEnvironmentSolvePriorityMagnitude(s1024, entities, idx));
    }
    return max_magnitude;
}

fn computeEnvironmentIterationBudget(instance_count: usize, max_stress: f32) u8 {
    var budget: u8 = if (instance_count >= 16)
        4
    else if (instance_count >= 8)
        3
    else
        2;

    if (max_stress >= 1.0) budget = addIterationBudgetWithClamp(budget, 1);
    if (max_stress >= 4.0) budget = addIterationBudgetWithClamp(budget, 1);
    return clampIterationBudget(budget);
}

fn computeEnvironmentSettleThreshold(max_stress: f32) f32 {
    return computeStressSettleThreshold(max_stress);
}

fn buildEnvironmentPriorityOrder(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []u8,
) usize {
    var count: usize = 0;
    var magnitudes: [scene32.MAX_INSTANCES]f32 = undefined;

    var idx: u8 = 0;
    while (idx < s1024.instance_count) : (idx += 1) {
        const magnitude = computeEnvironmentSolvePriorityMagnitude(s1024, entities, idx);
        if (magnitude <= settle_threshold) continue;
        out_indices[count] = idx;
        magnitudes[count] = magnitude;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                out_indices[j] > out_indices[best]
            else
                out_indices[j] < out_indices[best];
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_idx = out_indices[i];
            out_indices[i] = out_indices[best];
            out_indices[best] = tmp_idx;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn buildEnvironmentPriorityOrderForIndices(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_indices: []const u8,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []u8,
) usize {
    var count: usize = 0;
    var magnitudes: [scene32.MAX_INSTANCES]f32 = undefined;

    for (instance_indices) |instance_idx| {
        const magnitude = computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx);
        if (magnitude <= settle_threshold) continue;
        out_indices[count] = instance_idx;
        magnitudes[count] = magnitude;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                out_indices[j] > out_indices[best]
            else
                out_indices[j] < out_indices[best];
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_idx = out_indices[i];
            out_indices[i] = out_indices[best];
            out_indices[best] = tmp_idx;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn computeConstraintBlockIterationBudget(joint_stress: f32, contact_stress: f32, environment_stress: f32) u8 {
    const max_stress = @max(joint_stress, @max(contact_stress, environment_stress));
    var active_count: u8 = 0;
    if (joint_stress > 0.0) active_count += 1;
    if (contact_stress > 0.0) active_count += 1;
    if (environment_stress > 0.0) active_count += 1;

    var budget: u8 = 2;
    if (active_count >= 2) budget = addIterationBudgetWithClamp(budget, 1);
    if (active_count >= 3) budget = addIterationBudgetWithClamp(budget, 1);
    if (max_stress >= 1.0) budget = addIterationBudgetWithClamp(budget, 1);
    if (max_stress >= 4.0) budget = addIterationBudgetWithClamp(budget, 1);
    return clampIterationBudget(budget);
}

fn computeConstraintBlockSettleThreshold(joint_stress: f32, contact_stress: f32, environment_stress: f32) f32 {
    const max_stress = @max(joint_stress, @max(contact_stress, environment_stress));
    return computeStressSettleThreshold(max_stress);
}

fn findConstraintRowStateIndex(
    row_states: []const ConstraintRowState,
    kind: ConstraintRowKind,
    index: usize,
) ?usize {
    for (row_states, 0..) |state, state_idx| {
        if (state.kind == kind and state.index == index) return state_idx;
    }
    return null;
}

fn getConstraintRowState(
    row_states: []ConstraintRowState,
    state_count: usize,
    kind: ConstraintRowKind,
    index: usize,
) ?*const ConstraintRowState {
    const state_idx = findConstraintRowStateIndex(row_states[0..state_count], kind, index) orelse return null;
    return &row_states[state_idx];
}

fn appendBuiltConstraintRow(
    out_rows: []ConstraintRow,
    count: *usize,
    maybe_row: ?ConstraintRow,
) void {
    if (maybe_row) |row| {
        out_rows[count.*] = row;
        count.* += 1;
    }
}

fn buildConstraintRowFromSpec(
    row_state: ?*const ConstraintRowState,
    spec: ConstraintRowBuildSpec,
) ?ConstraintRow {
    return buildConstraintRow(
        spec.kind,
        spec.index,
        row_state,
        spec.residual,
        spec.metadata,
        spec.equation,
    );
}

fn appendConstraintRowSpec(
    out_rows: []ConstraintRow,
    count: *usize,
    row_states: []ConstraintRowState,
    state_count: usize,
    spec: ConstraintRowBuildSpec,
) void {
    const state = getConstraintRowState(row_states, state_count, spec.kind, spec.index);
    appendBuiltConstraintRow(out_rows, count, buildConstraintRowFromSpec(state, spec));
}

fn appendOptionalConstraintRowSpec(
    out_rows: []ConstraintRow,
    count: *usize,
    row_states: []ConstraintRowState,
    state_count: usize,
    maybe_spec: ?ConstraintRowBuildSpec,
) void {
    if (maybe_spec) |spec| {
        appendConstraintRowSpec(out_rows, count, row_states, state_count, spec);
    }
}

fn appendConstraintRowSpecsFromEntries(
    ctx: *const RowSpecBuildContext,
    entries: []const RowSpecBuilderEntry,
    row_states: []ConstraintRowState,
    state_count: usize,
    out_rows: []ConstraintRow,
    count: *usize,
) void {
    for (entries) |entry| {
        appendOptionalConstraintRowSpec(
            out_rows,
            count,
            row_states,
            state_count,
            buildConstraintRowSpecFromEntry(ctx, &entry),
        );
    }
}

fn getOrCreateConstraintRowState(
    row_states: []ConstraintRowState,
    state_count: *usize,
    kind: ConstraintRowKind,
    index: usize,
) *ConstraintRowState {
    if (findConstraintRowStateIndex(row_states[0..state_count.*], kind, index)) |state_idx| {
        return &row_states[state_idx];
    }

    const new_idx = state_count.*;
    row_states[new_idx] = .{
        .kind = kind,
        .index = index,
        .retained_residual = 0.0,
        .accumulated_impulse = 0.0,
        .last_impulse_delta = 0.0,
        .touched_iteration = 0,
    };
    state_count.* += 1;
    return &row_states[new_idx];
}

fn measureConstraintRowMetadataPriorityFloor(base_priority: f32, metadata: ConstraintRowMetadata) f32 {
    return measureSpeculativeContactPriorityFloor(
        @max(base_priority, metadata.predictive_residual_hint),
        metadata,
    );
}

fn measureConstraintRowCachedPriority(base_priority: f32, metadata: ConstraintRowMetadata, state: ?*const ConstraintRowState) f32 {
    const shaped_base_priority = measureConstraintRowMetadataPriorityFloor(base_priority, metadata);
    if (state == null) return shaped_base_priority;
    const cached = state.?;
    return @max(
        shaped_base_priority,
        cached.retained_residual * 0.75 + @abs(cached.accumulated_impulse) * 0.25,
    );
}

fn updateConstraintRowState(
    state: *ConstraintRowState,
    exec_result: ConstraintRowExecResult,
    iteration: u8,
) void {
    const applied = if (exec_result.applied_impulse > 0.0)
        exec_result.applied_impulse
    else
        @max(0.0, exec_result.residual_before - exec_result.residual_after);
    state.retained_residual = exec_result.residual_after;
    state.last_impulse_delta = applied;
    state.accumulated_impulse = if (exec_result.changed)
        state.accumulated_impulse * 0.35 + applied * 0.65
    else
        state.accumulated_impulse * 0.5;
    state.touched_iteration = iteration;
}

fn constraintRowWarmImpulse(
    row_state: ?*const ConstraintRowState,
    retention_scale: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
) f32 {
    const state = row_state orelse return 0.0;
    return @min(
        state.accumulated_impulse * retention_scale + equation.bias * equation.effective_mass * bias_scale,
        equation.max_impulse,
    );
}

fn constraintRowSignedCorrection(
    row_state: ?*const ConstraintRowState,
    retention_scale: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    value: f32,
    threshold: f32,
) f32 {
    if (@abs(value) <= threshold) return 0.0;
    return signedCorrection(
        value,
        constraintRowWarmImpulse(row_state, retention_scale, equation, bias_scale),
    );
}

fn constraintRowPlannedSignedCorrection(
    row_state: ?*const ConstraintRowState,
    retention_scale: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
    value: f32,
) f32 {
    if (value == 0.0) return 0.0;
    return signedCorrection(
        value,
        constraintRowWarmImpulse(row_state, retention_scale, equation, bias_scale),
    );
}

fn constraintRowSolveMagnitude(
    raw_magnitude: f32,
    equation: ConstraintRowEquation,
    bias_scale: f32,
) f32 {
    if (raw_magnitude == 0.0) return 0.0;
    const signed_bias = if (raw_magnitude >= 0.0)
        equation.bias * equation.effective_mass * bias_scale
    else
        -equation.bias * equation.effective_mass * bias_scale;
    return @max(-equation.max_impulse, @min(equation.max_impulse, raw_magnitude + signed_bias));
}

fn makePredictiveConstraintGain(predicted_depth: f32, current_depth: f32, residual_scale: f32, bias_scale: f32, impulse_scale: f32) PredictiveConstraintGain {
    const predictive_excess = @max(0.0, predicted_depth - current_depth);
    return .{
        .residual_hint = @max(current_depth, predicted_depth * residual_scale),
        .bias_delta = predictive_excess * bias_scale,
        .impulse_delta = predicted_depth * impulse_scale,
    };
}

fn makePredictiveConstraintGainWithPolicy(
    predicted_depth: f32,
    current_depth: f32,
    policy: PredictiveConstraintPolicy,
) PredictiveConstraintGain {
    return makePredictiveConstraintGain(
        predicted_depth,
        current_depth,
        policy.residual_scale,
        policy.bias_scale,
        policy.impulse_scale,
    );
}

fn environmentPredictiveConstraintPolicy() PredictiveConstraintPolicy {
    return .{
        .residual_scale = 0.75,
        .bias_scale = 0.2,
        .impulse_scale = 0.5,
    };
}

fn environmentPredictiveConstraintGain(predicted_depth: f32, current_depth: f32) PredictiveConstraintGain {
    return makePredictiveConstraintGainWithPolicy(
        predicted_depth,
        current_depth,
        environmentPredictiveConstraintPolicy(),
    );
}

fn jointDrivePredictiveConstraintPolicy() PredictiveConstraintPolicy {
    return .{
        .residual_scale = 1.0,
        .bias_scale = 0.0,
        .impulse_scale = 1.0,
    };
}

fn contactNormalPredictiveConstraintPolicy() PredictiveConstraintPolicy {
    return .{
        .residual_scale = 0.75,
        .bias_scale = 0.15,
        .impulse_scale = 0.5,
    };
}

fn effectiveMassFromInverseMass(inverse_mass: f32) f32 {
    return if (inverse_mass > 0.0001) 1.0 / inverse_mass else 0.0;
}

fn effectiveMassFromPairInverseMasses(inv_mass_a: f32, inv_mass_b: f32) f32 {
    return effectiveMassFromInverseMass(inv_mass_a + inv_mass_b);
}

fn buildNormalConstraintEquation(
    effective_mass: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    penetration_bias_scale: f32,
    velocity_bias_scale: f32,
    impulse_depth_scale: f32,
    impulse_speed_scale: f32,
) ConstraintRowEquation {
    const bias = penetration_depth * penetration_bias_scale + @max(0.0, -relative_normal_speed) * velocity_bias_scale;
    return .{
        .effective_mass = effective_mass,
        .bias = bias,
        .max_impulse = @max(0.5, penetration_depth * impulse_depth_scale + @abs(relative_normal_speed) * impulse_speed_scale),
    };
}

fn applyPredictiveConstraintGain(equation: ConstraintRowEquation, gain: PredictiveConstraintGain) ConstraintRowEquation {
    return .{
        .effective_mass = equation.effective_mass,
        .bias = equation.bias + gain.bias_delta,
        .max_impulse = equation.max_impulse + gain.impulse_delta,
    };
}

fn measureSpeculativeContactBiasScale(direction: PreparedDirectionalConstraint) f32 {
    return measureSpeculativeContactBiasScaleWithPolicy(direction, speculativeContactBiasPolicy());
}

fn measureSpeculativeContactBiasScaleWithPolicy(
    direction: PreparedDirectionalConstraint,
    policy: SpeculativeContactBiasPolicy,
) f32 {
    return measureSpeculativeContactBiasProfile(direction, policy).scale;
}

fn measureSpeculativeContactBiasProfileDefault(
    direction: PreparedDirectionalConstraint,
) SpeculativeContactBiasProfile {
    return measureSpeculativeContactBiasProfile(direction, speculativeContactBiasPolicy());
}

const SpeculativeContactBiasProfile = struct {
    scale: f32,
    tier: SpeculativeContactBiasTier,
    predictive_excess: f32,
};

fn measureSpeculativeContactBiasProfile(
    direction: PreparedDirectionalConstraint,
    policy: SpeculativeContactBiasPolicy,
) SpeculativeContactBiasProfile {
    const predictive_excess = @max(0.0, direction.predicted_depth - direction.depth);
    if (predictive_excess >= policy.high_excess_threshold) {
        return .{
            .scale = policy.high_bias_scale,
            .tier = .high,
            .predictive_excess = predictive_excess,
        };
    }
    if (predictive_excess >= policy.mid_excess_threshold) {
        return .{
            .scale = policy.mid_bias_scale,
            .tier = .mid,
            .predictive_excess = predictive_excess,
        };
    }
    return .{
        .scale = policy.base_bias_scale,
        .tier = .base,
        .predictive_excess = predictive_excess,
    };
}

const SpeculativeContactBiasPolicy = struct {
    base_bias_scale: f32,
    mid_bias_scale: f32,
    high_bias_scale: f32,
    mid_excess_threshold: f32,
    high_excess_threshold: f32,
};

fn speculativeContactBiasPolicy() SpeculativeContactBiasPolicy {
    return .{
        .base_bias_scale = 1.1,
        .mid_bias_scale = 1.15,
        .high_bias_scale = 1.2,
        .mid_excess_threshold = 1.0,
        .high_excess_threshold = 2.0,
    };
}

fn applySpeculativeContactBiasBoost(
    equation: ConstraintRowEquation,
    direction: PreparedDirectionalConstraint,
) ConstraintRowEquation {
    const scale = measureSpeculativeContactBiasProfileDefault(direction).scale;
    return .{
        .effective_mass = equation.effective_mass,
        .bias = equation.bias * scale,
        .max_impulse = equation.max_impulse,
    };
}

fn makeDirectionalPredictiveConstraintGain(
    direction: PreparedDirectionalConstraint,
    policy: PredictiveConstraintPolicy,
) PredictiveConstraintGain {
    return makePredictiveConstraintGainWithPolicy(
        direction.predicted_depth,
        direction.depth,
        policy,
    );
}

fn makePredictiveRowPlan(
    current_depth: f32,
    predicted_depth: f32,
    horizon_dt: f32,
    policy: PredictiveConstraintPolicy,
) PredictiveRowPlan {
    const gain = makePredictiveConstraintGainWithPolicy(predicted_depth, current_depth, policy);
    return .{
        .horizon_dt = horizon_dt,
        .current_depth = current_depth,
        .predicted_depth = predicted_depth,
        .urgency = gain.residual_hint,
        .allowed_correction_budget = gain.impulse_delta,
        .gain = gain,
    };
}

fn makeDirectionalPredictiveRowPlan(
    direction: PreparedDirectionalConstraint,
    policy: PredictiveConstraintPolicy,
) PredictiveRowPlan {
    return makePredictiveRowPlan(
        direction.depth,
        direction.predicted_depth,
        kernelPredictionDt(),
        policy,
    );
}

fn makeJointDrivePredictiveRowPlan(position_error: f32, predicted_error: f32, horizon_dt: f32) PredictiveRowPlan {
    return makePredictiveRowPlan(
        @abs(position_error),
        @abs(predicted_error),
        horizon_dt,
        jointDrivePredictiveConstraintPolicy(),
    );
}

fn finalizeDirectionalResidual(predictive_hint: f32, equation: ConstraintRowEquation) f32 {
    return @max(predictive_hint, equation.bias * 1.25);
}

fn resolveDirectionalPredictiveResidualHint(direction: PreparedDirectionalConstraint) f32 {
    return if (direction.predictive_residual_hint > 0.0) direction.predictive_residual_hint else direction.predicted_depth;
}

fn computeDirectionalPredictiveSolvePriorityMagnitude(
    current_stress: f32,
    direction: PreparedDirectionalConstraint,
    policy: PredictiveConstraintPolicy,
) f32 {
    const predictive_plan = makeDirectionalPredictiveRowPlan(direction, policy);
    return @max(current_stress, predictive_plan.urgency);
}

fn makeDirectionalRowPlan(
    residual: f32,
    equation: ConstraintRowEquation,
    predictive_residual_hint: f32,
) DirectionalRowPlan {
    return .{
        .residual = residual,
        .metadata = makeConstraintRowMetadata(predictive_residual_hint),
        .equation = equation,
    };
}

fn buildDirectionalRowPlan(
    base_equation: ConstraintRowEquation,
    direction: PreparedDirectionalConstraint,
    policy: PredictiveConstraintPolicy,
) DirectionalRowPlan {
    const predictive_plan = makeDirectionalPredictiveRowPlan(direction, policy);
    const equation = applyPredictiveConstraintGain(base_equation, predictive_plan.gain);
    return makeDirectionalRowPlan(
        finalizeDirectionalResidual(predictive_plan.urgency, equation),
        equation,
        predictive_plan.urgency,
    );
}

fn buildDirectionalNormalResidual(direction: PreparedDirectionalConstraint, equation: ConstraintRowEquation) f32 {
    return finalizeDirectionalResidual(resolveDirectionalPredictiveResidualHint(direction), equation);
}

fn buildDirectionalTangentResidual(direction: PreparedDirectionalConstraint) f32 {
    return direction.depth * 0.125;
}

fn buildDirectionalResidualRowPlan(
    mode: DirectionalRowSpecMode,
    direction: PreparedDirectionalConstraint,
    equation: ConstraintRowEquation,
) DirectionalRowPlan {
    return makeDirectionalRowPlan(
        switch (mode) {
            .normal => buildDirectionalNormalResidual(direction, equation),
            .tangent => buildDirectionalTangentResidual(direction),
        },
        equation,
        0.0,
    );
}

fn buildConstraintRow(
    kind: ConstraintRowKind,
    index: usize,
    row_state: ?*const ConstraintRowState,
    base_residual: f32,
    metadata: ConstraintRowMetadata,
    equation: ConstraintRowEquation,
) ?ConstraintRow {
    const preconditioner = computeConstraintPreconditionerModel(equation, constraintPreconditionerPolicy());
    const resolved_metadata = withConstraintPreconditionerMetadata(metadata, preconditioner);
    const base_priority = measureConstraintRowCachedPriority(base_residual, resolved_metadata, row_state);
    const priority = preconditionConstraintRowPriority(base_priority, preconditioner);
    if (priority <= 0.0) return null;
    return .{
        .kind = kind,
        .index = index,
        .priority = priority,
        .base_residual = base_residual,
        .metadata = resolved_metadata,
        .equation = equation,
    };
}

fn makeConstraintRowBuildSpec(
    kind: ConstraintRowKind,
    index: usize,
    residual: f32,
    metadata: ConstraintRowMetadata,
    equation: ConstraintRowEquation,
) ConstraintRowBuildSpec {
    return .{
        .kind = kind,
        .index = index,
        .residual = residual,
        .metadata = metadata,
        .equation = equation,
    };
}

fn makeConstraintRowBuildSpecFromPlan(
    kind: ConstraintRowKind,
    index: usize,
    row_plan: DirectionalRowPlan,
) ConstraintRowBuildSpec {
    return makeConstraintRowBuildSpec(
        kind,
        index,
        row_plan.residual,
        row_plan.metadata,
        row_plan.equation,
    );
}

fn buildDirectionalRowSpec(
    kind: ConstraintRowKind,
    index: usize,
    row_plan: DirectionalRowPlan,
) ConstraintRowBuildSpec {
    return makeConstraintRowBuildSpecFromPlan(kind, index, row_plan);
}

fn buildDirectionalRowSpecWithMetadata(
    kind: ConstraintRowKind,
    index: usize,
    row_plan: DirectionalRowPlan,
    metadata: ConstraintRowMetadata,
) ConstraintRowBuildSpec {
    var spec = makeConstraintRowBuildSpecFromPlan(kind, index, row_plan);
    spec.metadata = metadata;
    return spec;
}

fn makeConstraintRowBuildSpecFromDirectionalMode(
    kind: ConstraintRowKind,
    index: usize,
    mode: DirectionalRowSpecMode,
    direction: PreparedDirectionalConstraint,
    equation: ConstraintRowEquation,
) ConstraintRowBuildSpec {
    return buildDirectionalRowSpec(
        kind,
        index,
        buildDirectionalResidualRowPlan(mode, direction, equation),
    );
}

fn buildDirectionalModeRowSpec(
    kind: ConstraintRowKind,
    index: usize,
    mode: DirectionalRowSpecMode,
    direction: PreparedDirectionalConstraint,
    equation: ConstraintRowEquation,
) ConstraintRowBuildSpec {
    return makeConstraintRowBuildSpecFromDirectionalMode(kind, index, mode, direction, equation);
}

fn makeOptionalConstraintRowSpec(
    enabled: bool,
    row_spec: ConstraintRowBuildSpec,
) ?ConstraintRowBuildSpec {
    if (!enabled) return null;
    return row_spec;
}

fn buildOptionalConstraintRowSpecFromPlan(
    kind: ConstraintRowKind,
    index: usize,
    enabled: bool,
    row_plan: DirectionalRowPlan,
) ?ConstraintRowBuildSpec {
    return makeOptionalConstraintRowSpec(
        enabled,
        makeConstraintRowBuildSpecFromPlan(kind, index, row_plan),
    );
}

fn buildOptionalDirectionalRowSpec(
    kind: ConstraintRowKind,
    index: usize,
    enabled: bool,
    payload: DirectionalRowPayload,
    mode: DirectionalRowSpecMode,
) ?ConstraintRowBuildSpec {
    return makeOptionalConstraintRowSpec(
        enabled,
        makeConstraintRowBuildSpecFromDirectionalMode(kind, index, mode, payload.direction, payload.equation),
    );
}

fn finalizeConstraintRowResult(
    before: f32,
    after: f32,
    changed: bool,
    applied_impulse: f32,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return .{
        .changed = changed or after < before,
        .residual_before = before,
        .residual_after = after,
        .applied_impulse = @min(
            equation.max_impulse,
            @max(applied_impulse, @max(0.0, before - after) + equation.bias * equation.effective_mass),
        ),
    };
}

fn inactiveConstraintRowResult() ConstraintRowExecResult {
    return .{
        .changed = false,
        .residual_before = 0.0,
        .residual_after = 0.0,
        .applied_impulse = 0.0,
    };
}

fn stalledConstraintRowResult(before: f32) ConstraintRowExecResult {
    return .{
        .changed = false,
        .residual_before = before,
        .residual_after = before,
        .applied_impulse = 0.0,
    };
}

fn measureJointRowResidual(
    kind: ConstraintRowKind,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) f32 {
    if (joint_idx >= joints.len) return 0.0;
    const joint_def = &joints[joint_idx];
    const descriptor = jointRuntimeRowDescriptor(kind) orelse return 0.0;
    if (!descriptor.row_enabled(joint_def)) return 0.0;
    return joint.measureJointSolvePriorityForIndex(instances, joints, joint_idx, entities) * descriptor.policy.residual_scale;
}

fn measureContactRowResidual(
    kind: ConstraintRowKind,
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) f32 {
    const metrics = measureContactConstraintMetrics(inst_a, inst_b, entities) orelse return 0.0;
    return switch (kind) {
        .contact_normal => metrics.residual(.normal),
        .contact_friction => metrics.residual(.tangent),
        else => 0.0,
    };
}

fn measureEnvironmentRowResidual(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
) f32 {
    return computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx);
}

fn buildConstraintSubsystemOrder(
    joint_stress: f32,
    contact_stress: f32,
    environment_stress: f32,
    reverse_tiebreak: bool,
    out_order: *[3]ConstraintSubsystem,
) usize {
    var count: usize = 0;
    var magnitudes: [3]f32 = undefined;

    if (joint_stress > 0.0) {
        out_order[count] = .joint;
        magnitudes[count] = joint_stress;
        count += 1;
    }
    if (contact_stress > 0.0) {
        out_order[count] = .contact;
        magnitudes[count] = contact_stress;
        count += 1;
    }
    if (environment_stress > 0.0) {
        out_order[count] = .environment;
        magnitudes[count] = environment_stress;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                @intFromEnum(out_order[j]) > @intFromEnum(out_order[best])
            else
                @intFromEnum(out_order[j]) < @intFromEnum(out_order[best]);
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_order = out_order[i];
            out_order[i] = out_order[best];
            out_order[best] = tmp_order;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn instanceHasActiveConstraint(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    instance_idx: u8,
) bool {
    if (instance_idx >= s1024.instance_count) return false;
    const inst = &s1024.instances[instance_idx];
    if (inst.state == .broken) return false;

    if (computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx) > 0.0) return true;

    for (broadphase_pairs) |pair| {
        if (pair.a == instance_idx or pair.b == instance_idx) {
            if (computeMaxActiveContactConstraintMagnitude(s1024, entities, &.{pair}) > 0.0) return true;
        }
    }

    for (joints) |joint_def| {
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a != instance_idx and joint_def.entity_b != instance_idx) continue;
        return true;
    }

    return false;
}

fn buildConstraintInstanceIsland(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    start_idx: u8,
    assigned: []bool,
    out_indices: []u8,
) usize {
    var queue: [scene32.MAX_INSTANCES]u8 = undefined;
    var queue_head: usize = 0;
    var queue_tail: usize = 0;
    var count: usize = 0;

    assigned[start_idx] = true;
    queue[queue_tail] = start_idx;
    queue_tail += 1;

    while (queue_head < queue_tail) : (queue_head += 1) {
        const current = queue[queue_head];
        out_indices[count] = current;
        count += 1;

        for (broadphase_pairs) |pair| {
            var neighbor: ?u8 = null;
            if (pair.a == current) neighbor = pair.b;
            if (pair.b == current) neighbor = pair.a;
            if (neighbor) |instance_idx| {
                if (instance_idx >= s1024.instance_count) continue;
                if (assigned[instance_idx]) continue;
                if (!instanceHasActiveConstraint(s1024, entities, joints, broadphase_pairs, instance_idx)) continue;
                assigned[instance_idx] = true;
                queue[queue_tail] = instance_idx;
                queue_tail += 1;
            }
        }

        for (joints) |joint_def| {
            if (!joint_def.enabled) continue;
            var neighbor: ?u8 = null;
            if (joint_def.entity_a == current) neighbor = @intCast(joint_def.entity_b);
            if (joint_def.entity_b == current) neighbor = @intCast(joint_def.entity_a);
            if (neighbor) |instance_idx| {
                if (instance_idx >= s1024.instance_count) continue;
                if (assigned[instance_idx]) continue;
                if (!instanceHasActiveConstraint(s1024, entities, joints, broadphase_pairs, instance_idx)) continue;
                assigned[instance_idx] = true;
                queue[queue_tail] = instance_idx;
                queue_tail += 1;
            }
        }
    }

    return count;
}

fn buildConstraintIslands(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    island_storage: [][scene32.MAX_INSTANCES]u8,
    island_lengths: []usize,
) usize {
    var assigned: [scene32.MAX_INSTANCES]bool = [_]bool{false} ** scene32.MAX_INSTANCES;
    var island_count: usize = 0;
    var instance_idx: u8 = 0;
    while (instance_idx < s1024.instance_count) : (instance_idx += 1) {
        if (assigned[instance_idx]) continue;
        if (!instanceHasActiveConstraint(s1024, entities, joints, broadphase_pairs, instance_idx)) continue;
        island_lengths[island_count] = buildConstraintInstanceIsland(
            s1024,
            entities,
            joints,
            broadphase_pairs,
            instance_idx,
            assigned[0..],
            island_storage[island_count][0..],
        );
        island_count += 1;
    }
    return island_count;
}

fn islandContainsInstance(instance_indices: []const u8, instance_idx: u8) bool {
    for (instance_indices) |candidate| {
        if (candidate == instance_idx) return true;
    }
    return false;
}

fn copyJointSubsetForIsland(
    joints: []joint.Joint,
    instance_indices: []const u8,
    out_joints: []joint.Joint,
) []joint.Joint {
    var count: usize = 0;
    for (joints) |joint_def| {
        if (!joint_def.enabled) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_a))) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_b))) continue;
        out_joints[count] = joint_def;
        count += 1;
    }
    return out_joints[0..count];
}

fn copyJointIndexSubsetForIsland(
    joints: []joint.Joint,
    instance_indices: []const u8,
    out_indices: []usize,
) []usize {
    var count: usize = 0;
    for (joints, 0..) |joint_def, joint_idx| {
        if (!joint_def.enabled) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_a))) continue;
        if (!islandContainsInstance(instance_indices, @intCast(joint_def.entity_b))) continue;
        out_indices[count] = joint_idx;
        count += 1;
    }
    return out_indices[0..count];
}

fn copyBroadPhaseSubsetForIsland(
    broadphase_pairs: []const BroadPhasePair,
    instance_indices: []const u8,
    out_pairs: []BroadPhasePair,
) []BroadPhasePair {
    var count: usize = 0;
    for (broadphase_pairs) |pair| {
        if (!islandContainsInstance(instance_indices, pair.a)) continue;
        if (!islandContainsInstance(instance_indices, pair.b)) continue;
        out_pairs[count] = pair;
        count += 1;
    }
    return out_pairs[0..count];
}

fn computeMaxActiveEnvironmentConstraintMagnitudeForIndices(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_indices: []const u8,
) f32 {
    var max_magnitude: f32 = 0.0;
    for (instance_indices) |instance_idx| {
        max_magnitude = @max(max_magnitude, computeEnvironmentSolvePriorityMagnitude(s1024, entities, instance_idx));
    }
    return max_magnitude;
}

fn computeConstraintIslandStress(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    instance_indices: []const u8,
) f32 {
    var joint_index_storage: [joint.MAX_JOINTS]usize = undefined;
    const joint_indices = copyJointIndexSubsetForIsland(joints, instance_indices, joint_index_storage[0..]);
    var pair_subset_storage: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined;
    const pair_subset = copyBroadPhaseSubsetForIsland(broadphase_pairs, instance_indices, pair_subset_storage[0..]);

    const joint_stress = if (joint_indices.len != 0)
        joint.measureJointSolveStressForIndices(s1024.instances[0..s1024.instance_count], joints, joint_indices, entities)
    else
        0.0;
    const contact_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, pair_subset);
    const environment_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(s1024, entities, instance_indices);
    return @max(joint_stress, @max(contact_stress, environment_stress));
}

fn buildContactRowSpecForPair(
    ctx: *const RowSpecBuildContext,
    kind: ConstraintRowKind,
    mode: DirectionalRowSpecMode,
    enabled: bool,
    payload_selector: ContactDirectionalPayloadFn,
) ?ConstraintRowBuildSpec {
    const contact = contactRowSpecContext(ctx) orelse return null;
    const payload = payload_selector(&contact.prepared);
    const row_plan = buildDirectionalResidualRowPlan(mode, payload.direction, payload.equation);
    const metadata = if (kind == .contact_normal)
        withSpeculativeContactDirectionalMetadata(
            row_plan.metadata,
            contact.prepared.speculative,
            payload.direction,
        )
    else
        row_plan.metadata;
    return makeOptionalConstraintRowSpec(
        enabled,
        buildDirectionalRowSpecWithMetadata(
            kind,
            contact.pair_idx,
            row_plan,
            metadata,
        ),
    );
}

fn selectContactNormalDirectionalPayload(prepared: *const ContactPreparedPair) DirectionalRowPayload {
    return prepared.directionalPayload(.normal);
}

fn selectContactTangentDirectionalPayload(prepared: *const ContactPreparedPair) DirectionalRowPayload {
    return prepared.directionalPayload(.tangent);
}

const contact_row_spec_builder_entries = [_]RowSpecBuilderEntry{
    .{ .contact_directional = .{ .kind = .contact_normal, .mode = .normal, .enabled = true, .payload_selector = selectContactNormalDirectionalPayload } },
    .{ .contact_directional = .{ .kind = .contact_friction, .mode = .tangent, .enabled = true, .payload_selector = selectContactTangentDirectionalPayload } },
};

const environment_row_spec_builder_entries = [_]RowSpecBuilderEntry{
    .{ .environment_plan = .{ .kind = .environment, .enabled = true, .plan_builder = buildEnvironmentPreparedRowPlan } },
};

fn jointHasDriveRow(joint_def: *const joint.Joint) bool {
    return joint_def.motor_enabled and joint_def.motor_speed > 0.0;
}

const joint_runtime_rows = [_]JointRuntimeRowDescriptor{
    .{
        .kind = .joint_anchor,
        .policy = .{
            .residual_scale = 1.0,
            .warm_impulse_scale = 0.25,
            .inactive_outcome = .finalize_exec_state,
            .stalled_prepare_outcome = .stalled,
            .signed_correction_policy = .{},
            .impulse_hint_policy = .{},
            .anchor_postprocess_policy = .by_joint_type,
            .drive_postprocess_policy = .none,
        },
        .row_enabled = jointHasAnchorRow,
        .prepare_channel = prepareJointDescriptorAnchorChannel,
    },
    .{
        .kind = .joint_limit,
        .policy = .{
            .residual_scale = 0.9,
            .warm_impulse_scale = 0.0,
            .inactive_outcome = .finalize_no_change,
            .stalled_prepare_outcome = .stalled,
            .signed_correction_policy = .{
                .retention_scale = 0.1,
                .bias_scale = 1.0,
                .min_abs_correction = 0.0001,
                .planned = false,
            },
            .impulse_hint_policy = .{},
            .anchor_postprocess_policy = .none,
            .drive_postprocess_policy = .none,
        },
        .row_enabled = jointHasLimitRow,
        .prepare_channel = prepareJointDescriptorLimitChannel,
    },
    .{
        .kind = .joint_drive,
        .policy = .{
            .residual_scale = 0.75,
            .warm_impulse_scale = 0.35,
            .inactive_outcome = .stalled,
            .stalled_prepare_outcome = .stalled,
            .signed_correction_policy = .{
                .retention_scale = 0.35,
                .bias_scale = 1.0,
                .min_abs_correction = 0.0,
                .planned = true,
            },
            .impulse_hint_policy = .{
                .enabled = true,
                .scale = 1.0,
            },
            .anchor_postprocess_policy = .none,
            .drive_postprocess_policy = .velocity_bias,
        },
        .row_enabled = jointHasDriveRow,
        .prepare_channel = prepareJointDescriptorDriveChannel,
    },
};

const contact_runtime_rows = [_]ContactRuntimeRowDescriptor{
    .{
        .kind = .contact_normal,
        .channel = .normal,
        .settle_after_solve = true,
        .requires_tangent = false,
        .build_plan = buildContactNormalSolvePlan,
        .select_step = selectContactNormalSolveStep,
    },
    .{
        .kind = .contact_friction,
        .channel = .friction,
        .settle_after_solve = false,
        .requires_tangent = true,
        .build_plan = buildContactFrictionSolvePlan,
        .select_step = selectContactFrictionSolveStep,
    },
};

const contact_batch_descriptor = ContactBatchSolveDescriptor{
    .settle_after_solve = true,
    .build_plan = buildContactBatchSolvePlan,
};

fn jointRuntimeRowDescriptor(kind: ConstraintRowKind) ?*const JointRuntimeRowDescriptor {
    inline for (&joint_runtime_rows) |*entry| {
        if (entry.kind == kind) return entry;
    }
    return null;
}

fn contactRuntimeRowDescriptor(kind: ConstraintRowKind) ?*const ContactRuntimeRowDescriptor {
    inline for (&contact_runtime_rows) |*entry| {
        if (entry.kind == kind) return entry;
    }
    return null;
}

fn jointRowEnabled(enabled: bool, row_enabled: JointRowEnabledFn, joint_def: *const joint.Joint) bool {
    return enabled and row_enabled(joint_def);
}

fn buildJointRowSpecForJoint(
    ctx: *const RowSpecBuildContext,
    kind: ConstraintRowKind,
    enabled: bool,
    row_enabled: JointRowEnabledFn,
    plan_builder: JointRowPlanBuilderFn,
) ?ConstraintRowBuildSpec {
    const joint_ctx = jointRowSpecContext(ctx) orelse return null;
    const joint_def = if (joint_ctx.joint_idx < joint_ctx.joints.len)
        &joint_ctx.joints[joint_ctx.joint_idx]
    else
        return null;
    return buildOptionalConstraintRowSpecFromPlan(
        kind,
        joint_ctx.joint_idx,
        jointRowEnabled(enabled, row_enabled, joint_def),
        plan_builder(
            joint_ctx.s1024.instances[0..joint_ctx.s1024.instance_count],
            joint_ctx.joints,
            joint_ctx.joint_idx,
            joint_ctx.entities,
        ),
    );
}

fn buildEnvironmentRowSpecForInstance(
    ctx: *const RowSpecBuildContext,
    kind: ConstraintRowKind,
    enabled: bool,
    plan_builder: EnvironmentRowPlanBuilderFn,
) ?ConstraintRowBuildSpec {
    const environment = environmentRowSpecContext(ctx) orelse return null;
    return buildOptionalConstraintRowSpecFromPlan(
        kind,
        environment.local_idx,
        enabled,
        plan_builder(
            environment.s1024,
            environment.entities,
            environment.instance_idx,
        ),
    );
}

const joint_row_spec_builder_entries = [_]RowSpecBuilderEntry{
    .{ .joint_plan = .{ .kind = .joint_anchor, .enabled = true, .row_enabled = jointHasAnchorRow, .plan_builder = buildJointAnchorRowPlan } },
    .{ .joint_plan = .{ .kind = .joint_limit, .enabled = true, .row_enabled = jointHasLimitRow, .plan_builder = buildJointLimitRowPlan } },
    .{ .joint_plan = .{ .kind = .joint_drive, .enabled = true, .row_enabled = jointHasDriveRow, .plan_builder = buildJointDriveRowPlan } },
};

fn buildConstraintRowSpecFromEntry(
    ctx: *const RowSpecBuildContext,
    entry: *const RowSpecBuilderEntry,
) ?ConstraintRowBuildSpec {
    return switch (entry.*) {
        .contact_directional => |contact_entry| buildContactRowSpecForPair(
            ctx,
            contact_entry.kind,
            contact_entry.mode,
            contact_entry.enabled,
            contact_entry.payload_selector,
        ),
        .environment_plan => |environment_entry| buildEnvironmentRowSpecForInstance(
            ctx,
            environment_entry.kind,
            environment_entry.enabled,
            environment_entry.plan_builder,
        ),
        .joint_plan => |joint_entry| buildJointRowSpecForJoint(
            ctx,
            joint_entry.kind,
            joint_entry.enabled,
            joint_entry.row_enabled,
            joint_entry.plan_builder,
        ),
    };
}

fn islandRowDispatchEntryItemCount(ctx: *const IslandRowBuildContext, entry: *const IslandRowDispatchEntry) usize {
    return switch (entry.subsystem) {
        .joint => ctx.joint_indices.len,
        .contact => ctx.pair_subset.len,
        .environment => ctx.instance_indices.len,
    };
}

fn islandRowDispatchEntryBuilderEntries(entry: *const IslandRowDispatchEntry) []const RowSpecBuilderEntry {
    return switch (entry.subsystem) {
        .joint => joint_row_spec_builder_entries[0..],
        .contact => contact_row_spec_builder_entries[0..],
        .environment => environment_row_spec_builder_entries[0..],
    };
}

fn initIslandRowDispatchEntryContext(
    ctx: *const IslandRowBuildContext,
    entry: *const IslandRowDispatchEntry,
    local_idx: usize,
) ?RowSpecBuildContext {
    return switch (entry.subsystem) {
        .joint => .{
            .joint = .{
                .s1024 = ctx.s1024,
                .entities = ctx.entities,
                .joints = ctx.joints,
                .joint_idx = ctx.joint_indices[local_idx],
            },
        },
        .contact => blk: {
            const pair = ctx.pair_subset[local_idx];
            if (pair.a >= ctx.s1024.instance_count or pair.b >= ctx.s1024.instance_count) break :blk null;
            const inst_a = &ctx.s1024.instances[pair.a];
            const inst_b = &ctx.s1024.instances[pair.b];
            const prepared = prepareContactConstraintPair(inst_a, inst_b, ctx.entities) orelse break :blk null;
            break :blk .{
                .contact = .{
                    .prepared = prepared,
                    .pair_idx = local_idx,
                },
            };
        },
        .environment => .{
            .environment = .{
                .s1024 = ctx.s1024,
                .entities = ctx.entities,
                .instance_idx = ctx.instance_indices[local_idx],
                .local_idx = local_idx,
            },
        },
    };
}

fn runIslandRowDispatchEntry(ctx: *IslandRowBuildContext, entry: *const IslandRowDispatchEntry) void {
    const item_count = islandRowDispatchEntryItemCount(ctx, entry);
    const builder_entries = islandRowDispatchEntryBuilderEntries(entry);
    var local_idx: usize = 0;
    while (local_idx < item_count) : (local_idx += 1) {
        const spec_ctx = initIslandRowDispatchEntryContext(ctx, entry, local_idx) orelse continue;
        appendConstraintRowSpecsFromEntries(
            &spec_ctx,
            builder_entries,
            ctx.row_states,
            ctx.state_count,
            ctx.out_rows,
            ctx.count,
        );
    }
}

const island_row_dispatch_entries = [_]IslandRowDispatchEntry{
    .{ .subsystem = .joint },
    .{ .subsystem = .contact },
    .{ .subsystem = .environment },
};

fn measureIslandRowDispatchEntryStress(ctx: *const IslandRowBuildContext, entry: *const IslandRowDispatchEntry) f32 {
    return switch (entry.subsystem) {
        .joint => if (ctx.joint_indices.len != 0)
            joint.measureJointSolveStressForIndices(ctx.s1024.instances[0..ctx.s1024.instance_count], ctx.joints, ctx.joint_indices, ctx.entities)
        else
            0.0,
        .contact => computeMaxActiveContactConstraintMagnitude(ctx.s1024, ctx.entities, ctx.pair_subset),
        .environment => computeMaxActiveEnvironmentConstraintMagnitudeForIndices(ctx.s1024, ctx.entities, ctx.instance_indices),
    };
}

fn buildIslandRowDispatchOrder(ctx: *const IslandRowBuildContext, out_order: *[island_row_dispatch_entries.len]IslandRowDispatchOrderEntry) usize {
    var count: usize = 0;
    for (&island_row_dispatch_entries) |*entry| {
        const stress = measureIslandRowDispatchEntryStress(ctx, entry);
        if (stress <= 0.0) continue;
        out_order[count] = .{
            .entry = entry,
            .stress = stress,
        };
        count += 1;
    }

    if (count < 2) return count;

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const candidate = out_order[j];
            const current_best = out_order[best];
            const better_stress = candidate.stress > current_best.stress;
            const tied_but_earlier = std.math.approxEqAbs(f32, candidate.stress, current_best.stress, 0.0001) and
                @intFromEnum(candidate.entry.subsystem) < @intFromEnum(current_best.entry.subsystem);
            if (better_stress or tied_but_earlier) {
                best = j;
            }
        }
        if (best == i) continue;
        const tmp = out_order[i];
        out_order[i] = out_order[best];
        out_order[best] = tmp;
    }

    return count;
}

fn buildConstraintRowsForIsland(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    joint_indices: []const usize,
    pair_subset: []const BroadPhasePair,
    instance_indices: []const u8,
    row_states: []ConstraintRowState,
    state_count: usize,
    out_rows: []ConstraintRow,
) usize {
    var count: usize = 0;
    var ctx = IslandRowBuildContext{
        .s1024 = s1024,
        .entities = entities,
        .joints = joints,
        .joint_indices = joint_indices,
        .pair_subset = pair_subset,
        .instance_indices = instance_indices,
        .row_states = row_states,
        .state_count = state_count,
        .out_rows = out_rows,
        .count = &count,
    };

    var subsystem_order: [island_row_dispatch_entries.len]IslandRowDispatchOrderEntry = undefined;
    const subsystem_count = buildIslandRowDispatchOrder(&ctx, &subsystem_order);

    for (subsystem_order[0..subsystem_count]) |ordered| {
        runIslandRowDispatchEntry(&ctx, ordered.entry);
    }

    return count;
}

fn enforceJointBreaksForIndices(
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_indices: []const usize,
    entities: []entity16.Entity16,
    joint_break_queue: ?*collision_event.PendingJointBreakQueue,
) bool {
    var changed = false;
    for (joint_indices) |joint_idx| {
        if (joint_idx >= joints.len) continue;
        const joint_def = &joints[joint_idx];
        const entity_a = joint_def.entity_a;
        const entity_b = joint_def.entity_b;
        const break_ratio = if (joint_def.breaking_force > 0.0)
            @max(1.0, joint_def.break_accum / @max(0.0001, joint_def.breaking_force))
        else
            0.0;

        if (!joint.enforceJointBreakForIndex(instances, joints, joint_idx, entities)) continue;
        changed = true;
        if (joint_break_queue) |queue| {
            queue.enqueueJoint(@intCast(joint_idx), .{
                .joint_idx = @intCast(@min(joint_idx, std.math.maxInt(u8))),
                .entity_a = entity_a,
                .entity_b = entity_b,
                .break_ratio = break_ratio,
            });
        }
    }
    return changed;
}

fn sortConstraintRows(rows: []ConstraintRow, reverse_tiebreak: bool) void {
    if (rows.len < 2) return;

    var i: usize = 0;
    while (i < rows.len) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < rows.len) : (j += 1) {
            const better_priority = rows[j].priority > rows[best].priority;
            const equal_priority = @abs(rows[j].priority - rows[best].priority) <= 0.0001;
            const lhs_key = (@as(u32, @intFromEnum(rows[j].kind)) << 16) | @as(u32, @intCast(rows[j].index));
            const rhs_key = (@as(u32, @intFromEnum(rows[best].kind)) << 16) | @as(u32, @intCast(rows[best].index));
            const better_tiebreak = if (reverse_tiebreak) lhs_key > rhs_key else lhs_key < rhs_key;
            if (better_priority or (equal_priority and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp = rows[i];
            rows[i] = rows[best];
            rows[best] = tmp;
        }
    }
}

fn sortConstraintIslandsByStress(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    island_storage: []const [scene32.MAX_INSTANCES]u8,
    island_lengths: []const usize,
    island_count: usize,
    out_order: []usize,
) void {
    var island_stress: [scene32.MAX_INSTANCES]f32 = undefined;

    var idx: usize = 0;
    while (idx < island_count) : (idx += 1) {
        out_order[idx] = idx;
        island_stress[idx] = computeConstraintIslandStress(
            s1024,
            entities,
            joints,
            broadphase_pairs,
            island_storage[idx][0..island_lengths[idx]],
        );
    }

    var i: usize = 0;
    while (i < island_count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < island_count) : (j += 1) {
            const better_stress = island_stress[j] > island_stress[best];
            const equal_stress = @abs(island_stress[j] - island_stress[best]) <= 0.0001;
            const better_tiebreak = out_order[j] < out_order[best];
            if (better_stress or (equal_stress and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_order = out_order[i];
            out_order[i] = out_order[best];
            out_order[best] = tmp_order;

            const tmp_stress = island_stress[i];
            island_stress[i] = island_stress[best];
            island_stress[best] = tmp_stress;
        }
    }
}

fn buildContactPriorityOrder(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []usize,
) usize {
    var count: usize = 0;
    var magnitudes: [MAX_BROADPHASE_PAIRS]f32 = undefined;

    for (broadphase_pairs, 0..) |pair, idx| {
        if (pair.a >= s1024.instance_count or pair.b >= s1024.instance_count) continue;

        const inst_a = &s1024.instances[pair.a];
        const inst_b = &s1024.instances[pair.b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        const magnitude = computeContactSolvePriorityMagnitude(inst_a, inst_b, entities);
        if (magnitude <= settle_threshold) continue;

        out_indices[count] = idx;
        magnitudes[count] = magnitude;
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                out_indices[j] > out_indices[best]
            else
                out_indices[j] < out_indices[best];
            if (better_magnitude or (equal_magnitude and better_tiebreak)) best = j;
        }

        if (best != i) {
            const tmp_idx = out_indices[i];
            out_indices[i] = out_indices[best];
            out_indices[best] = tmp_idx;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;
        }
    }

    return count;
}

fn applyPairVelocityImpulseRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    dir_x: f32,
    dir_y: f32,
    dir_z: f32,
    raw_signed_impulse: f32,
    primitive: ConstraintApplyPrimitive,
) ConstraintSolveStep {
    const pair_bodies = primitive.pair_bodies orelse return .{ .changed = false, .applied_impulse = 0.0 };
    if (raw_signed_impulse == 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    const solve_impulse = solvePrimitiveMagnitude(raw_signed_impulse, primitive);
    if (solve_impulse == 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    return .{
        .changed = applyKernelLinearVelocityImpulsePair(
            inst_a,
            inst_b,
            pair_bodies.inv_mass_a,
            pair_bodies.inv_mass_b,
            dir_x,
            dir_y,
            dir_z,
            solve_impulse,
        ),
        .applied_impulse = @abs(solve_impulse),
    };
}

fn applyContactWarmStartRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    equation: ConstraintRowEquation,
    accumulated_impulse: f32,
) ConstraintSolveStep {
    return applyContactNormalWarmStartRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        .{
            .dir_x = normal_x,
            .dir_y = normal_y,
            .dir_z = normal_z,
            .penetration_depth = 0.0,
            .restitution = 0.0,
        },
        equation,
        accumulated_impulse,
    );
}

fn applyContactNormalWarmStartRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal: ContactNormalConstraintSpec,
    equation: ConstraintRowEquation,
    accumulated_impulse: f32,
) ConstraintSolveStep {
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        normal.dir_x,
        normal.dir_y,
        normal.dir_z,
        @max(0.0, accumulated_impulse),
        blk: {
            var primitive = makePairPrimitiveFromInverseMasses(inv_mass_a, inv_mass_b, .linear_velocity, equation, 0.15, normal.dir_x, normal.dir_y, normal.dir_z);
            primitive.clamp_non_negative = true;
            break :blk primitive;
        },
    );
}

fn applyContactPositionRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    penetration_depth: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return solveContactNormalPositionConstraintRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        .{
            .dir_x = normal_x,
            .dir_y = normal_y,
            .dir_z = normal_z,
            .penetration_depth = penetration_depth,
            .restitution = 0.0,
        },
        equation,
    );
}

fn solveContactNormalPositionConstraintRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal: ContactNormalConstraintSpec,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const correction_depth = computeContactNormalConstraintPositionImpulse(
        normal,
        equation,
    );
    if (correction_depth <= 0.0) return .{ .changed = false, .applied_impulse = 0.0 };

    return applyPairDirectionalDisplacementRowStep(
        inst_a,
        inst_b,
        -normal.dir_x,
        -normal.dir_y,
        -normal.dir_z,
        correction_depth,
        blk: {
            var primitive = makePairPrimitiveFromInverseMasses(inv_mass_a, inv_mass_b, .linear_displacement, equation, 0.0, -normal.dir_x, -normal.dir_y, -normal.dir_z);
            primitive.clamp_non_negative = true;
            break :blk primitive;
        },
    );
}

fn applyContactVelocityRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    restitution: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    return solveContactNormalVelocityConstraintRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        .{
            .dir_x = normal_x,
            .dir_y = normal_y,
            .dir_z = normal_z,
            .penetration_depth = 0.0,
            .restitution = restitution,
        },
        equation,
    );
}

fn solveContactNormalVelocityConstraintRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal: ContactNormalConstraintSpec,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_normal_vel = rel_vel_x * normal.dir_x + rel_vel_y * normal.dir_y + rel_vel_z * normal.dir_z;
    const raw_impulse = computeContactNormalConstraintVelocityImpulse(
        normal,
        rel_normal_vel,
        inv_mass_a,
        inv_mass_b,
    );
    if (raw_impulse <= 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    const velocity_equation: ConstraintRowEquation = .{
        .effective_mass = equation.effective_mass,
        // Restitution is a velocity solve, so positional bias should not dilute or flip the bounce impulse.
        .bias = 0.0,
        .max_impulse = @max(equation.max_impulse, @abs(raw_impulse)),
    };
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        normal.dir_x,
        normal.dir_y,
        normal.dir_z,
        raw_impulse,
        makePairPrimitiveFromInverseMasses(inv_mass_a, inv_mass_b, .linear_velocity, velocity_equation, 0.2, normal.dir_x, normal.dir_y, normal.dir_z),
    );
}

fn applyContactTangentConstraintRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangent: ContactTangentConstraintSpec,
    equation: ConstraintRowEquation,
    signed_impulse: f32,
) ConstraintSolveStep {
    const projected_impulse = projectContactTangentConstraintImpulse(signed_impulse, tangent);
    if (projected_impulse == 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    var tangent_equation = equation;
    tangent_equation.max_impulse = @max(0.0, @min(equation.max_impulse, tangent.impulse_limit));
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        tangent.dir_x,
        tangent.dir_y,
        tangent.dir_z,
        projected_impulse,
        makePairPrimitiveFromInverseMasses(
            inv_mass_a,
            inv_mass_b,
            .linear_velocity,
            tangent_equation,
            0.1,
            tangent.dir_x,
            tangent.dir_y,
            tangent.dir_z,
        ),
    );
}

fn solveContactTangentConstraintRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangent: ContactTangentConstraintSpec,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const raw_impulse = computeContactTangentConstraintImpulse(
        tangent,
        inv_mass_a,
        inv_mass_b,
    );
    return applyContactTangentConstraintRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        tangent,
        equation,
        -raw_impulse,
    );
}

fn solveContactFrictionRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    friction_coeff: f32,
    normal_impulse_limit: f32,
    anisotropic_minor_axis_scale: f32,
    equation: ConstraintRowEquation,
) ConstraintSolveStep {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const velocity_components = measureDirectionalVelocityComponents(
        rel_vel_x,
        rel_vel_y,
        rel_vel_z,
        normal_x,
        normal_y,
        normal_z,
    );
    const friction_policy = contactFrictionSolvePolicy();
    const friction_cone = approximateContactFrictionCone(
        velocity_components,
        friction_coeff,
        normal_impulse_limit,
        friction_policy,
    ) orelse return .{
        .changed = false,
        .applied_impulse = 0.0,
    };
    const friction_ellipse = approximateContactFrictionEllipse(
        friction_cone,
        normal_x,
        normal_y,
        normal_z,
        anisotropic_minor_axis_scale,
        friction_policy,
    );
    const tangent = buildContactTangentConstraintSpecFromEllipse(friction_ellipse);
    return solveContactTangentConstraintRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        tangent,
        equation,
    );
}

fn applyContactFrictionWarmStartRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangent_x: f32,
    tangent_y: f32,
    tangent_z: f32,
    normal_x: f32,
    normal_y: f32,
    normal_z: f32,
    equation: ConstraintRowEquation,
    friction_coeff: f32,
    normal_impulse_limit: f32,
    anisotropic_minor_axis_scale: f32,
    accumulated_impulse: f32,
    tangential_speed: f32,
) ConstraintSolveStep {
    if (@abs(tangential_speed) <= 0.0001) return .{ .changed = false, .applied_impulse = 0.0 };
    const signed_impulse = if (tangential_speed > 0.0)
        -@max(0.0, accumulated_impulse)
    else
        @max(0.0, accumulated_impulse);
    const friction_policy = contactFrictionSolvePolicy();
    const cone: ContactFrictionConeApproximation = .{
        .dir_x = tangent_x,
        .dir_y = tangent_y,
        .dir_z = tangent_z,
        .tangential_speed = @abs(tangential_speed),
        .impulse_limit = computeContactFrictionImpulseLimit(
            friction_coeff,
            normal_impulse_limit,
            friction_policy,
        ),
    };
    const ellipse = approximateContactFrictionEllipse(
        cone,
        normal_x,
        normal_y,
        normal_z,
        anisotropic_minor_axis_scale,
        friction_policy,
    );
    var tangent: ContactTangentConstraintSpec = buildContactTangentConstraintSpecFromEllipse(ellipse);
    tangent.dir_x = tangent_x;
    tangent.dir_y = tangent_y;
    tangent.dir_z = tangent_z;
    return applyContactTangentConstraintRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        tangent,
        equation,
        projectContactFrictionImpulseToEllipse(signed_impulse, ellipse),
    );
}

fn solvePreparedContactNormalSteps(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    prepared: ContactPreparedPair,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintSolveStep {
    var accum = ContactSolveAccumulator{};
    const normal = buildContactNormalConstraintSpec(
        prepared.normal,
        prepared.restitution,
    );
    if (row_state) |_| {
        const warm_impulse = constraintRowWarmImpulse(row_state, 0.35, equation, 0.0);
        const warm_step = applyContactNormalWarmStartRowStep(
            inst_a,
            inst_b,
            prepared.inv_mass_a,
            prepared.inv_mass_b,
            normal,
            equation,
            warm_impulse,
        );
        accum.addWarmStart(warm_step);
    }
    const position_step = solveContactNormalPositionConstraintRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        normal,
        equation,
    );
    accum.applyStep(position_step);
    const velocity_step = solveContactNormalVelocityConstraintRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        normal,
        equation,
    );
    accum.applyStep(velocity_step);
    return accum.finish();
}

fn solvePreparedContactFrictionSteps(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    prepared: ContactPreparedPair,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintSolveStep {
    if (!prepared.has_tangent) return .{ .changed = false, .applied_impulse = 0.0 };

    var accum = ContactSolveAccumulator{};
    if (row_state) |_| {
        const friction_hint = constraintRowWarmImpulse(row_state, 0.35, equation, 0.0);
        const warm_step = applyContactFrictionWarmStartRowStep(
            inst_a,
            inst_b,
            prepared.inv_mass_a,
            prepared.inv_mass_b,
            prepared.tangent.dir_x,
            prepared.tangent.dir_y,
            prepared.tangent.dir_z,
            prepared.normal.dir_x,
            prepared.normal.dir_y,
            prepared.normal.dir_z,
            equation,
            prepared.friction,
            prepared.normal_equation.max_impulse,
            prepared.anisotropic_friction_minor_axis_scale,
            friction_hint,
            prepared.tangent.depth,
        );
        accum.addWarmStart(warm_step);
    }
    const friction_step = solveContactFrictionRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.friction,
        prepared.normal_equation.max_impulse,
        prepared.anisotropic_friction_minor_axis_scale,
        equation,
    );
    accum.applyStep(friction_step);
    return accum.finish();
}

fn settleContactRestState(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal_y: f32,
) void {
    if (@abs(normal_y) < 0.5) return;

    if (inv_mass_a > 0.0 and @abs(inst_a.vel_x) <= 1 and @abs(inst_a.vel_y) <= 1 and @abs(inst_a.vel_z) <= 1) {
        inst_a.vel_x = 0;
        inst_a.vel_y = 0;
        inst_a.vel_z = 0;
        inst_a.state = .resting;
    }

    if (inv_mass_b > 0.0 and @abs(inst_b.vel_x) <= 1 and @abs(inst_b.vel_y) <= 1 and @abs(inst_b.vel_z) <= 1) {
        inst_b.vel_x = 0;
        inst_b.vel_y = 0;
        inst_b.vel_z = 0;
        inst_b.state = .resting;
    }
}

fn solveContactConstraintsForPairs(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) bool {
    var changed = false;
    if (broadphase_pairs.len == 0) return changed;

    const initial_max_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, broadphase_pairs);
    const settle_threshold = computeContactSettleThreshold(initial_max_stress);
    const iterations = boostIterationBudgetForSpeculativePairs(
        s1024,
        entities,
        broadphase_pairs,
        computeContactIterationBudget(broadphase_pairs, initial_max_stress),
    );
    var priority_order: [MAX_BROADPHASE_PAIRS]usize = undefined;
    var iter: u8 = 0;
    while (iter < iterations) : (iter += 1) {
        const active_count = buildContactPriorityOrder(
            s1024,
            entities,
            broadphase_pairs,
            settle_threshold,
            (iter & 1) != 0,
            priority_order[0..],
        );
        if (active_count == 0) break;

        var order_idx: usize = 0;
        while (order_idx < active_count) : (order_idx += 1) {
            const pair = broadphase_pairs[priority_order[order_idx]];
            changed = executePreparedContactBatchOutcome(
                prepareContactBatchSolveContext(s1024, entities, pair),
            ) or changed;
        }

        if (computeMaxActiveContactConstraintMagnitude(s1024, entities, broadphase_pairs) <= settle_threshold) break;
    }

    return changed;
}

fn solveContactRuntimeRow(
    kind: ConstraintRowKind,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    pair: BroadPhasePair,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    return executePreparedContactRuntimeOutcome(
        prepareContactRuntimeSolveContext(kind, s1024, entities, pair, base_residual),
        entities,
        base_residual,
        equation,
        row_state,
    );
}

fn solveJointRuntimeRow(
    kind: ConstraintRowKind,
    instances: []scene32.Instance,
    joints: []joint.Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    return executePreparedJointRuntimeOutcome(
        prepareJointRuntimeSolveContext(
            kind,
            instances,
            joints,
            joint_idx,
            entities,
            base_residual,
        ),
        instances,
        joints,
        joint_idx,
        entities,
        base_residual,
        equation,
        row_state,
    );
}

pub fn solveContactConstraints(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    broadphase_pairs: []const BroadPhasePair,
) bool {
    return solveContactConstraintsForPairs(s1024, entities, broadphase_pairs);
}

fn solveEnvironmentConstraintsForIndices(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_indices: []const u8,
) bool {
    var changed = false;
    const initial_max_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(s1024, entities, instance_indices);
    const settle_threshold = computeEnvironmentSettleThreshold(initial_max_stress);
    const iterations = computeEnvironmentIterationBudget(instance_indices.len, initial_max_stress);
    var priority_order: [scene32.MAX_INSTANCES]u8 = undefined;
    var iter: u8 = 0;
    while (iter < iterations) : (iter += 1) {
        const subset_active_count = buildEnvironmentPriorityOrderForIndices(
            s1024,
            entities,
            instance_indices,
            settle_threshold,
            (iter & 1) != 0,
            priority_order[0..],
        );
        if (subset_active_count == 0) break;

        var order_idx: usize = 0;
        while (order_idx < subset_active_count) : (order_idx += 1) {
            const i = priority_order[order_idx];
            const moved = executePreparedEnvironmentBatchOutcome(
                prepareEnvironmentBatchSolveContext(s1024, entities, i),
            );
            changed = moved or changed;
        }

        if (computeMaxActiveEnvironmentConstraintMagnitudeForIndices(s1024, entities, instance_indices) <= settle_threshold) break;
    }

    return changed;
}

fn solveEnvironmentRow(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    instance_idx: u8,
    base_residual: f32,
    equation: ConstraintRowEquation,
    row_state: ?*const ConstraintRowState,
) ConstraintRowExecResult {
    return executePreparedEnvironmentRuntimeOutcome(
        prepareEnvironmentRuntimeSolveContext(s1024, entities, instance_idx, base_residual),
        s1024,
        entities,
        base_residual,
        equation,
        row_state,
    );
}

pub fn solveEnvironmentConstraints(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
) bool {
    var all_indices: [scene32.MAX_INSTANCES]u8 = undefined;
    var count: usize = 0;
    var idx: u8 = 0;
    while (idx < s1024.instance_count) : (idx += 1) {
        all_indices[count] = idx;
        count += 1;
    }
    return solveEnvironmentConstraintsForIndices(s1024, entities, all_indices[0..count]);
}

pub fn solveConstraintBlock(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    broadphase_pairs: []const BroadPhasePair,
    joint_break_queue: ?*collision_event.PendingJointBreakQueue,
) bool {
    var changed = false;
    var island_storage: [scene32.MAX_INSTANCES][scene32.MAX_INSTANCES]u8 = undefined;
    var island_lengths: [scene32.MAX_INSTANCES]usize = [_]usize{0} ** scene32.MAX_INSTANCES;
    const island_count = buildConstraintIslands(
        s1024,
        entities,
        joints,
        broadphase_pairs,
        island_storage[0..],
        island_lengths[0..],
    );
    if (island_count == 0) return false;

    var island_order: [scene32.MAX_INSTANCES]usize = undefined;
    sortConstraintIslandsByStress(
        s1024,
        entities,
        joints,
        broadphase_pairs,
        island_storage[0..island_count],
        island_lengths[0..island_count],
        island_count,
        island_order[0..island_count],
    );

    var island_order_idx: usize = 0;
    while (island_order_idx < island_count) : (island_order_idx += 1) {
        const island_idx = island_order[island_order_idx];
        const instance_indices = island_storage[island_idx][0..island_lengths[island_idx]];

        var joint_index_storage: [joint.MAX_JOINTS]usize = undefined;
        const joint_indices = copyJointIndexSubsetForIsland(joints, instance_indices, joint_index_storage[0..]);
        changed = enforceJointBreaksForIndices(
            s1024.instances[0..s1024.instance_count],
            joints,
            joint_indices,
            entities,
            joint_break_queue,
        ) or changed;
        var pair_subset_storage: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined;
        const pair_subset = copyBroadPhaseSubsetForIsland(broadphase_pairs, instance_indices, pair_subset_storage[0..]);

        const initial_joint_stress = if (joint_indices.len != 0)
            joint.measureJointSolveStressForIndices(s1024.instances[0..s1024.instance_count], joints, joint_indices, entities)
        else
            0.0;
        const initial_contact_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, pair_subset);
        const initial_environment_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(
            s1024,
            entities,
            instance_indices,
        );
        const iterations = boostIterationBudgetForSpeculativePairs(
            s1024,
            entities,
            pair_subset,
            computeConstraintBlockIterationBudget(
                initial_joint_stress,
                initial_contact_stress,
                initial_environment_stress,
            ),
        );
        const settle_threshold = computeConstraintBlockSettleThreshold(
            initial_joint_stress,
            initial_contact_stress,
            initial_environment_stress,
        );

        var iter: u8 = 0;
        var row_state_storage: [joint.MAX_JOINTS * 3 + MAX_BROADPHASE_PAIRS * 2 + scene32.MAX_INSTANCES]ConstraintRowState = undefined;
        var row_state_count: usize = 0;
        while (iter < iterations) : (iter += 1) {
            var iter_changed = false;
            var iter_improved = false;

            const joint_stress = if (joint_indices.len != 0)
                joint.measureJointSolveStressForIndices(s1024.instances[0..s1024.instance_count], joints, joint_indices, entities)
            else
                0.0;
            const contact_stress = computeMaxActiveContactConstraintMagnitude(s1024, entities, pair_subset);
            const environment_stress = computeMaxActiveEnvironmentConstraintMagnitudeForIndices(
                s1024,
                entities,
                instance_indices,
            );
            const iter_max_stress = @max(joint_stress, @max(contact_stress, environment_stress));
            if (iter_max_stress <= settle_threshold) break;

            var row_storage: [joint.MAX_JOINTS * 3 + MAX_BROADPHASE_PAIRS * 2 + scene32.MAX_INSTANCES]ConstraintRow = undefined;
            const row_count = buildConstraintRowsForIsland(
                s1024,
                entities,
                joints,
                joint_indices,
                pair_subset,
                instance_indices,
                row_state_storage[0..],
                row_state_count,
                row_storage[0..],
            );
            if (row_count == 0) break;
            sortConstraintRows(row_storage[0..row_count], (iter & 1) != 0);

            var iter_max_before: f32 = 0.0;
            var iter_max_after: f32 = 0.0;

            for (row_storage[0..row_count]) |row| {
                const row_state = getOrCreateConstraintRowState(
                    row_state_storage[0..],
                    &row_state_count,
                    row.kind,
                    row.index,
                );
                const exec_result = switch (row.kind) {
                    .joint_anchor => blk: {
                        break :blk solveJointRuntimeRow(
                            row.kind,
                            s1024.instances[0..s1024.instance_count],
                            joints,
                            row.index,
                            entities,
                            row.base_residual,
                            row.equation,
                            row_state,
                        );
                    },
                    .joint_limit => blk: {
                        break :blk solveJointRuntimeRow(
                            row.kind,
                            s1024.instances[0..s1024.instance_count],
                            joints,
                            row.index,
                            entities,
                            row.base_residual,
                            row.equation,
                            row_state,
                        );
                    },
                    .joint_drive => blk: {
                        break :blk solveJointRuntimeRow(
                            row.kind,
                            s1024.instances[0..s1024.instance_count],
                            joints,
                            row.index,
                            entities,
                            row.base_residual,
                            row.equation,
                            row_state,
                        );
                    },
                    .contact_normal => solveContactRuntimeRow(.contact_normal, s1024, entities, pair_subset[row.index], row.base_residual, row.equation, row_state),
                    .contact_friction => solveContactRuntimeRow(.contact_friction, s1024, entities, pair_subset[row.index], row.base_residual, row.equation, row_state),
                    .environment => solveEnvironmentRow(
                        s1024,
                        entities,
                        instance_indices[row.index],
                        row.base_residual,
                        row.equation,
                        row_state,
                    ),
                };

                updateConstraintRowState(row_state, exec_result, iter);
                iter_changed = exec_result.changed or iter_changed;
                iter_improved = exec_result.improved() or iter_improved;
                iter_max_before = @max(iter_max_before, exec_result.residual_before);
                iter_max_after = @max(iter_max_after, exec_result.residual_after);
            }

            changed = iter_changed or changed;
            if (!iter_changed) break;
            if (iter_max_after <= settle_threshold) break;
            if (!iter_improved and iter_max_after + 0.0001 >= iter_max_before) break;
        }
    }

    return changed;
}

pub fn runConstraintStage(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    out_pairs: []BroadPhasePair,
    joint_break_queue: ?*collision_event.PendingJointBreakQueue,
) ConstraintStageResult {
    const pair_count = collectBroadPhasePairs(
        s1024.instances[0..s1024.instance_count],
        entities,
        out_pairs,
    );
    const changed = solveConstraintBlock(
        s1024,
        entities,
        joints,
        out_pairs[0..pair_count],
        joint_break_queue,
    );
    return .{
        .pair_count = pair_count,
        .changed = changed,
    };
}

pub fn runPreMotionConstraintStage(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    out_pairs: []BroadPhasePair,
    time_scale: f32,
    sleep_time_threshold: u8,
    dt: f32,
    joint_break_queue: ?*collision_event.PendingJointBreakQueue,
) ConstraintStageResult {
    runPreStepSystems(s1024, entities, time_scale, sleep_time_threshold, dt);
    return runConstraintStage(s1024, entities, joints, out_pairs, joint_break_queue);
}

pub fn runPostMotionConstraintStage(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    out_pairs: []BroadPhasePair,
    joint_break_queue: ?*collision_event.PendingJointBreakQueue,
) ConstraintStageResult {
    rebuildOccupancyIfNeeded(s1024, entities, true);
    const result = runConstraintStage(s1024, entities, joints, out_pairs, joint_break_queue);
    rebuildOccupancyIfNeeded(s1024, entities, result.changed);
    return result;
}

pub fn mergeObservedPairCount(previous_pair_count: usize, current_pair_count: usize) usize {
    return @max(previous_pair_count, current_pair_count);
}

pub fn finishWorldStep(
    queue: *collision_event.PendingCollisionQueue,
    sound_queue: *collision_event.PendingSoundQueue,
    particle_queue: *collision_event.PendingParticleQueue,
    deformation_queue: *collision_event.PendingDeformationQueue,
    break_queue: *collision_event.PendingBreakQueue,
    joint_break_queue: *collision_event.PendingJointBreakQueue,
    world_bus: ?*bus.Bus,
    tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    debris_dt: f32,
) void {
    destruction.updateDebris(debris_dt);
    publishPendingCollisions(queue, world_bus, tick);
    publishPendingSounds(sound_queue, world_bus, tick);
    publishPendingParticles(particle_queue, world_bus, tick);
    publishPendingDeformations(deformation_queue, world_bus, tick);
    publishPendingBreaks(break_queue, world_bus, tick);
    publishPendingJointBreaks(joint_break_queue, world_bus, tick);
    recordWorldSnapshot(tick, s1024, entities);
}

pub fn initCoreSubsystems() void {
    kcc.init();
    vehicle.init();
    ragdoll.init();
    ballistics.init();
    destruction.init();
    rewind.init();
}

test "computeContactIterationBudget increases for higher contact stress" {
    const dense_pairs = [_]BroadPhasePair{.{ .a = 0, .b = 1 }} ** 16;

    try std.testing.expectEqual(@as(u8, 2), computeContactIterationBudget(&.{}, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeContactIterationBudget(&.{.{ .a = 0, .b = 1 }}, 1.5));
    try std.testing.expectEqual(@as(u8, 4), computeContactIterationBudget(&.{.{ .a = 0, .b = 1 }}, 5.0));
    try std.testing.expectEqual(
        ITERATION_BUDGET_CAP,
        computeContactIterationBudget(dense_pairs[0..], 5.0),
    );
}

test "clampIterationBudget enforces iteration cap at six" {
    try std.testing.expectEqual(@as(u8, 0), clampIterationBudget(0));
    try std.testing.expectEqual(@as(u8, 5), clampIterationBudget(5));
    try std.testing.expectEqual(ITERATION_BUDGET_CAP, clampIterationBudget(ITERATION_BUDGET_CAP));
    try std.testing.expectEqual(ITERATION_BUDGET_CAP, clampIterationBudget(7));
    try std.testing.expectEqual(ITERATION_BUDGET_CAP, clampIterationBudget(255));
}

test "addIterationBudgetWithClamp avoids overflow and enforces cap" {
    try std.testing.expectEqual(@as(u8, 5), addIterationBudgetWithClamp(2, 3));
    try std.testing.expectEqual(@as(u8, 6), addIterationBudgetWithClamp(5, 2));
    try std.testing.expectEqual(@as(u8, 6), addIterationBudgetWithClamp(255, 255));
}

test "collectBroadPhasePairsMeasured precomputes candidate AABBs once" {
    var dynamic_entity = entity16.initEntity16();
    dynamic_entity.physics.mass = 10;
    dynamic_entity.physics.material = .solid;
    entity16.setVoxel(&dynamic_entity, 0, 0, 0);

    var static_entity = entity16.initEntity16();
    static_entity.physics.mass = 0;
    static_entity.physics.material = .solid;
    static_entity.physics.flags |= 0x01;
    entity16.setVoxel(&static_entity, 0, 0, 0);

    var entities = [_]entity16.Entity16{ dynamic_entity, static_entity };
    const instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 2,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 0,
            .pos_x = 2,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 10,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .resting,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 10,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .resting,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };

    var pairs: [MAX_BROADPHASE_PAIRS]BroadPhasePair = undefined;
    var stats: BroadPhaseStats = .{};
    const pair_count = collectBroadPhasePairsMeasured(instances[0..], entities[0..], pairs[0..], &stats);

    try std.testing.expectEqual(@as(usize, 1), pair_count);
    try std.testing.expectEqual(@as(usize, 4), stats.candidate_count);
    try std.testing.expectEqual(@as(usize, 4), stats.aabb_builds);
    try std.testing.expectEqual(@as(usize, 6), stats.pair_tests);
    try std.testing.expectEqual(pair_count, stats.pair_count);
    try std.testing.expectEqual(@as(u8, 0), pairs[0].a);
    try std.testing.expectEqual(@as(u8, 1), pairs[0].b);
}

test "computeCollisionEmitModel builds collision payload" {
    var entity = entity16.initEntity16();
    entity.physics.mass = 10;
    entity.physics.hardness = 42;
    entity.physics.material = .solid;

    const emit = computeCollisionEmitModel(
        .{
            .entity_id = 7,
            .impact_velocity = -120,
            .entity = &entity,
        },
        collisionEmitPolicy(),
    );

    try std.testing.expect(emit.active);
    try std.testing.expectEqual(@as(u16, 7), emit.entity_id);
    try std.testing.expectEqual(@as(i16, -120), emit.payload.impact_velocity);
    try std.testing.expectEqual(@as(u16, 42), emit.payload.hardness);
}

test "enqueueCollisionPairEvent emits both participants once" {
    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.hardness = 42;
    entity16.setVoxel(&mover, 0, 0, 0);

    var blocker = entity16.initEntity16();
    blocker.physics.mass = 0;
    blocker.physics.hardness = 99;
    blocker.physics.flags |= 0x01;
    entity16.setVoxel(&blocker, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, blocker };
    const instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 1,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .resting,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var queue: collision_event.PendingCollisionQueue = .{};

    enqueueCollisionPairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);
    enqueueCollisionPairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);

    try std.testing.expectEqual(@as(u8, 2), queue.count);
    try std.testing.expectEqual(@as(u16, 0), queue.entity_ids[0]);
    try std.testing.expectEqual(@as(u16, 1), queue.entity_ids[1]);
    try std.testing.expectEqual(@as(u16, 42), queue.events[0].hardness);
    try std.testing.expectEqual(@as(u16, 99), queue.events[1].hardness);
}

test "computeSoundEmitModel builds audible sound payload" {
    const sound = computeContactSoundModel(
        .{
            .sound_type = 14,
            .penetration_depth = 1.0,
            .relative_normal_speed = 4.0,
            .tangential_speed = 0.0,
            .friction = 0.7,
            .restitution = 0.3,
            .medium_type = .solid,
        },
        contactSoundPolicy(),
    );
    const emit = computeSoundEmitModel(
        .{
            .entity_id = 5,
            .sound = sound,
        },
        soundEmitPolicy(),
    );

    try std.testing.expect(emit.active);
    try std.testing.expectEqual(@as(u16, 5), emit.entity_id);
    try std.testing.expectEqual(@as(u8, 14), emit.payload.sound_type);
    try std.testing.expect(emit.payload.volume > 0.0);
    try std.testing.expect(emit.payload.duration > 0.0);
}

test "enqueueSoundPairEvent emits audible pair sound once" {
    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var blocker = entity16.initEntity16();
    blocker.physics.mass = 0;
    blocker.physics.material = .solid;
    blocker.physics.flags |= 0x01;
    entity16.setVoxel(&blocker, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, blocker };
    const instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 1,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .resting,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var queue: collision_event.PendingSoundQueue = .{};

    enqueueSoundPairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);
    enqueueSoundPairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);

    try std.testing.expectEqual(@as(u8, 2), queue.count);
    try std.testing.expectEqual(@as(u16, 0), queue.entity_ids[0]);
    try std.testing.expectEqual(@as(u16, 1), queue.entity_ids[1]);
    try std.testing.expect(queue.events[0].volume > 0.0);
    try std.testing.expect(queue.events[1].duration > 0.0);
}

test "computeParticleEmitModel builds emitted particle payload" {
    const dust = computeContactDustModel(
        .{
            .dust_type = 10,
            .penetration_depth = 1.0,
            .relative_normal_speed = 4.0,
            .tangential_speed = 0.0,
            .friction = 0.7,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        contactDustPolicy(),
    );
    const emit = computeParticleEmitModel(
        .{
            .entity_id = 5,
            .dust = dust,
        },
        particleEmitPolicy(),
    );

    try std.testing.expect(emit.active);
    try std.testing.expectEqual(@as(u16, 5), emit.entity_id);
    try std.testing.expectEqual(@as(u8, 10), emit.payload.particle_type);
    try std.testing.expect(emit.payload.intensity > 0.0);
    try std.testing.expect(emit.payload.duration > 0.0);
}

test "enqueueParticlePairEvent emits dust particles once" {
    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var blocker = entity16.initEntity16();
    blocker.physics.mass = 0;
    blocker.physics.material = .solid;
    blocker.physics.flags |= 0x01;
    entity16.setVoxel(&blocker, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, blocker };
    const instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 1,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .resting,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var queue: collision_event.PendingParticleQueue = .{};

    enqueueParticlePairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);
    enqueueParticlePairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);

    try std.testing.expectEqual(@as(u8, 2), queue.count);
    try std.testing.expectEqual(@as(u16, 0), queue.entity_ids[0]);
    try std.testing.expectEqual(@as(u16, 1), queue.entity_ids[1]);
    try std.testing.expect(queue.events[0].intensity > 0.0);
    try std.testing.expect(queue.events[1].duration > 0.0);
}

test "computeDeformationEmitModel builds deformation payload" {
    const deformation = computeContactDeformationModel(
        .{
            .penetration_depth = 1.0,
            .relative_normal_speed = 4.0,
            .tangential_speed = 0.0,
            .penetration_resistance = 0.2,
            .damage_modifier = 1.5,
            .restitution = 0.3,
        },
        contactDeformationPolicy(),
    );
    const emit = computeDeformationEmitModel(
        .{
            .entity_id = 5,
            .deformation = deformation,
        },
        deformationEmitPolicy(),
    );

    try std.testing.expect(emit.active);
    try std.testing.expectEqual(@as(u16, 5), emit.entity_id);
    try std.testing.expect(emit.payload.total_depth > 0.0);
    try std.testing.expect(emit.payload.permanent_depth > 0.0);
    try std.testing.expect(emit.payload.recovery_fraction >= 0.0);
}

test "enqueueDeformationPairEvent emits deformation once" {
    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var blocker = entity16.initEntity16();
    blocker.physics.mass = 0;
    blocker.physics.material = .solid;
    blocker.physics.flags |= 0x01;
    entity16.setVoxel(&blocker, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, blocker };
    const instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 1,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .resting,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var queue: collision_event.PendingDeformationQueue = .{};

    enqueueDeformationPairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);
    enqueueDeformationPairEvent(&queue, instances[0..], entities[0..], &instances[0], -120, 1);

    try std.testing.expectEqual(@as(u8, 2), queue.count);
    try std.testing.expectEqual(@as(u16, 0), queue.entity_ids[0]);
    try std.testing.expectEqual(@as(u16, 1), queue.entity_ids[1]);
    try std.testing.expect(queue.events[0].total_depth > 0.0);
    try std.testing.expect(queue.events[1].permanent_depth > 0.0);
}

test "computeBreakEmitModel builds break payload for fragile impact" {
    var fragile = entity16.initEntity16();
    fragile.physics.mass = 400;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);

    const emit = computeBreakEmitModel(
        .{
            .entity_id = 5,
            .impact_velocity = -20,
            .entity = &fragile,
        },
        breakEmitPolicy(),
    );

    try std.testing.expect(emit.active);
    try std.testing.expectEqual(@as(u16, 5), emit.entity_id);
    try std.testing.expectEqual(@as(i16, -20), emit.payload.impact_velocity);
    try std.testing.expectEqual(@as(u16, 10), emit.payload.hardness);
    try std.testing.expect(emit.payload.fragment_count > 0);
}

test "enqueueBreakPairEvent emits break once" {
    var mover = entity16.initEntity16();
    mover.physics.mass = 400;
    mover.physics.material = .fragile;
    mover.physics.hardness = 10;
    entity16.setVoxel(&mover, 0, 0, 0);

    var blocker = entity16.initEntity16();
    blocker.physics.mass = 0;
    blocker.physics.material = .solid;
    blocker.physics.hardness = 200;
    blocker.physics.flags |= 0x01;
    entity16.setVoxel(&blocker, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, blocker };
    const instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 1,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .falling,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = -20,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .resting,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var queue: collision_event.PendingBreakQueue = .{};

    enqueueBreakPairEvent(&queue, instances[0..], entities[0..], &instances[0], -20, 1);
    enqueueBreakPairEvent(&queue, instances[0..], entities[0..], &instances[0], -20, 1);

    try std.testing.expectEqual(@as(u8, 1), queue.count);
    try std.testing.expectEqual(@as(u16, 0), queue.entity_ids[0]);
    try std.testing.expectEqual(@as(i16, -20), queue.events[0].impact_velocity);
    try std.testing.expectEqual(@as(u16, 10), queue.events[0].hardness);
    try std.testing.expect(queue.events[0].fragment_count > 0);
}

test "prepareContactConstraintPairMeasured reuses current AABBs for narrowphase" {
    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    const inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const inst_b: scene32.Instance = .{
        .entity_id = 1,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var stats: ContactNarrowPhaseStats = .{};

    const prepared = prepareContactConstraintPairMeasured(&inst_a, &inst_b, entities[0..], &stats) orelse return error.TestUnexpectedResult;

    try std.testing.expect(!prepared.speculative);
    try std.testing.expect(prepared.fatigue.accumulated_damage > 0.0);
    try std.testing.expect(prepared.fatigue.remaining_life_fraction < 1.0);
    try std.testing.expect(!prepared.fatigue.failed);
    try std.testing.expect(prepared.thermal.generated_heat > 0.0);
    try std.testing.expect(prepared.thermal.friction_heat >= 0.0);
    try std.testing.expect(prepared.thermal.friction_heat_fraction >= 0.0);
    try std.testing.expect(prepared.thermal.conductivity > 0.0);
    try std.testing.expect(prepared.sound.audible);
    try std.testing.expect(prepared.sound.volume > 0.0);
    try std.testing.expect(prepared.dust.intensity >= 0.0);
    try std.testing.expectEqual(@as(u8, 1), prepared.dust.dust_type);
    try std.testing.expect(prepared.deformation.total_depth > 0.0);
    try std.testing.expect(prepared.deformation.recovery_fraction >= 0.0);
    try std.testing.expectEqual(ContactSeparationState.persisting, prepared.separation.state);
    try std.testing.expect(!prepared.separation.separating);
    try std.testing.expect(prepared.stabilization.stabilized);
    try std.testing.expect(prepared.stabilization.bias_scale >= 1.0);
    try std.testing.expect(prepared.stabilization.impulse_scale >= 1.0);
    try std.testing.expect(!prepared.medium_buoyancy.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.displaced_volume, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.center_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.center_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.center_z, 0.0001);
    try std.testing.expect(!prepared.medium_drag.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_drag.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_drag.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_drag.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_drag.force, 0.0001);
    try std.testing.expect(!prepared.medium_vapor_resistance.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_vapor_resistance.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_vapor_resistance.dynamic_pressure, 0.0001);
    try std.testing.expect(!prepared.medium_vacuum.active);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.medium_vacuum.pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_vacuum.exposure, 0.0001);
    try std.testing.expect(!prepared.medium_transition.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_transition.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_transition.resistance, 0.0001);
    try std.testing.expect(!prepared.medium_mixing.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_mixing.mix_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_mixing.effective_density, 0.0001);
    try std.testing.expect(!prepared.medium_state.active);
    try std.testing.expectEqual(MediumPostState.stable, prepared.medium_state.state);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.medium_state.stability, 0.0001);
    try std.testing.expect(!prepared.medium_event.active);
    try std.testing.expectEqual(MediumPostEventType.none, prepared.medium_event.event_type);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_event.intensity, 0.0001);
    try std.testing.expect(!prepared.medium_animation.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_animation.phase, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_animation.opacity, 0.0001);
    try std.testing.expect(!prepared.medium_tow.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_tow.force, 0.0001);
    try std.testing.expect(!prepared.medium_lift.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.dynamic_pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.force, 0.0001);
    try std.testing.expect(!prepared.medium_added_mass.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_added_mass.added_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_added_mass.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.medium_added_mass.normal_inertia_scale, 0.0001);
    try std.testing.expect(!prepared.medium_thermal.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_thermal.conducted_heat, 0.0001);
    try std.testing.expect(!prepared.sink_depth.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.sink_depth.depth, 0.0001);
    try std.testing.expect(!prepared.sink_resistance.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.sink_resistance.force, 0.0001);
    try std.testing.expect(!prepared.sink_recovery.active);
    try std.testing.expect(prepared.sink_recovery.recovered);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.sink_recovery.depth, 0.0001);
    try std.testing.expectEqual(@as(u16, 0), prepared.damage_impact.legacy_impact);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.damage_impact.damage_amount, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 100.0), prepared.damage_hardness.impact_threshold, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.damage_hardness.impact_ratio, 0.0001);
    try std.testing.expect(!prepared.damage_hardness.exceeds_hardness);
    try std.testing.expect(!prepared.damage_break.did_break);
    try std.testing.expectEqual(@as(u8, 0), prepared.damage_break.fragments);
    try std.testing.expectEqual(@as(u8, 0), prepared.damage_fragments.generated_fragments);
    try std.testing.expectEqual(@as(u8, 0), prepared.damage_cracks.crack_count);
    try std.testing.expectEqual(@as(usize, 2), stats.aabb_builds);
    try std.testing.expectEqual(@as(usize, 1), stats.probe_calls);
    try std.testing.expectEqual(@as(usize, 1), stats.aabb_tests);
    try std.testing.expectEqual(@as(usize, 1), stats.narrowphase_calls);
}

test "computeContactSolvePriorityMagnitudeMeasured reuses current AABBs for narrowphase" {
    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    const inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const inst_b: scene32.Instance = .{
        .entity_id = 1,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var stats: ContactNarrowPhaseStats = .{};

    const magnitude = computeContactSolvePriorityMagnitudeMeasured(&inst_a, &inst_b, entities[0..], &stats);

    try std.testing.expect(magnitude > 0.0);
    try std.testing.expectEqual(@as(usize, 2), stats.aabb_builds);
    try std.testing.expectEqual(@as(usize, 1), stats.probe_calls);
    try std.testing.expectEqual(@as(usize, 1), stats.aabb_tests);
    try std.testing.expectEqual(@as(usize, 1), stats.narrowphase_calls);
}

test "detectContactManifoldsMeasured emits stable pair manifold and centroid" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel, voxel };
    s1024.instance_count = 3;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 20,
        .pos_y = 20,
        .pos_z = 20,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const pairs = [_]BroadPhasePair{
        .{ .a = 0, .b = 1 },
        .{ .a = 0, .b = 2 },
    };
    var manifolds: [2]ContactDetectedManifold = undefined;
    var stats: ContactNarrowPhaseStats = .{};

    const count = detectContactManifoldsMeasured(&s1024, entities[0..], pairs[0..], manifolds[0..], &stats);
    const detected = manifolds[0];
    const expected_centroid = collision.computeContactManifoldCentroid(detected.manifold);
    const normal_len_sq =
        detected.manifold.normal_x * detected.manifold.normal_x +
        detected.manifold.normal_y * detected.manifold.normal_y +
        detected.manifold.normal_z * detected.manifold.normal_z;

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(u8, 0), detected.pair.a);
    try std.testing.expectEqual(@as(u8, 1), detected.pair.b);
    try std.testing.expectEqual(@as(u16, 0), detected.entity_id_a);
    try std.testing.expectEqual(@as(u16, 1), detected.entity_id_b);
    try std.testing.expect(detected.manifold.point_count > 0);
    try std.testing.expect(detected.manifold.penetration_depth > 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), normal_len_sq, 0.0001);
    try std.testing.expectApproxEqAbs(expected_centroid.x, detected.centroid.x, 0.0001);
    try std.testing.expectApproxEqAbs(expected_centroid.y, detected.centroid.y, 0.0001);
    try std.testing.expectApproxEqAbs(expected_centroid.z, detected.centroid.z, 0.0001);
    try std.testing.expectEqual(@as(usize, 4), stats.aabb_builds);
    try std.testing.expectEqual(@as(usize, 2), stats.probe_calls);
    try std.testing.expectEqual(@as(usize, 2), stats.aabb_tests);
    try std.testing.expectEqual(@as(usize, 1), stats.narrowphase_calls);
}

test "classifyContactMaterials preserves pair materials and paired response" {
    var elastic = entity16.initEntity16();
    elastic.physics.material = .elastic;
    var fragile = entity16.initEntity16();
    fragile.physics.material = .fragile;
    var entities = [_]entity16.Entity16{ elastic, fragile };
    const detected = [_]ContactDetectedManifold{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 1, .max_y = 1, .max_z = 1 },
        .aabb_b = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 1, .max_y = 1, .max_z = 1 },
        .manifold = .{},
        .centroid = .{ .x = 0.0, .y = 0.0, .z = 0.0 },
    }};
    var classifications: [1]ContactMaterialClassification = undefined;

    const count = classifyContactMaterials(detected[0..], entities[0..], classifications[0..]);
    const classification = classifications[0];
    const expected_response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(.elastic),
        .elastic,
        material_pairing.getSurfaceForMaterial(.fragile),
        .fragile,
    );

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(u8, 0), classification.pair.a);
    try std.testing.expectEqual(@as(u8, 1), classification.pair.b);
    try std.testing.expectEqual(entity16.MaterialType.elastic, classification.material_a);
    try std.testing.expectEqual(entity16.MaterialType.fragile, classification.material_b);
    try std.testing.expectApproxEqAbs(expected_response.restitution, classification.response.restitution, 0.0001);
    try std.testing.expectApproxEqAbs(expected_response.damage_modifier, classification.response.damage_modifier, 0.0001);
    try std.testing.expect(classification.response.restitution > 0.5);
    try std.testing.expect(classification.response.damage_modifier > 0.6);
}

test "classifyContactSurfaces maps material classifications to representative surfaces" {
    const material_classifications = [_]ContactMaterialClassification{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .liquid,
        .material_b = .elastic,
        .response = material_pairing.getPairedResponse(
            material_pairing.getSurfaceForMaterial(.liquid),
            .liquid,
            material_pairing.getSurfaceForMaterial(.elastic),
            .elastic,
        ),
    }};
    var surface_classifications: [1]ContactSurfaceClassification = undefined;

    const count = classifyContactSurfaces(material_classifications[0..], surface_classifications[0..]);
    const classification = surface_classifications[0];

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(u8, 0), classification.pair.a);
    try std.testing.expectEqual(@as(u8, 1), classification.pair.b);
    try std.testing.expectEqual(terrain.SurfaceType.water, classification.surface_a);
    try std.testing.expectEqual(terrain.SurfaceType.rubber, classification.surface_b);
    try std.testing.expect(!classification.hard_surface_a);
    try std.testing.expect(classification.hard_surface_b);
}

test "classifyContactMediums derives medium from classified surfaces" {
    const surface_classifications = [_]ContactSurfaceClassification{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .liquid,
        .material_b = .solid,
        .surface_a = .water,
        .surface_b = .concrete,
        .hard_surface_a = false,
        .hard_surface_b = true,
    }};
    var medium_classifications: [1]ContactMediumClassification = undefined;

    const count = classifyContactMediums(surface_classifications[0..], medium_classifications[0..]);
    const classification = medium_classifications[0];

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(u8, 0), classification.pair.a);
    try std.testing.expectEqual(@as(u8, 1), classification.pair.b);
    try std.testing.expectEqual(terrain.MediumType.liquid, classification.medium_a);
    try std.testing.expectEqual(terrain.MediumType.solid, classification.medium_b);
    try std.testing.expectEqual(terrain.SurfaceType.water, classification.surface_a);
    try std.testing.expectEqual(terrain.SurfaceType.concrete, classification.surface_b);
}

test "classifyContactBodyTypes follows query body type flag precedence" {
    var static_entity = entity16.initEntity16();
    static_entity.physics.flags |= 0x01;
    var sensor_and_kinematic = entity16.initEntity16();
    sensor_and_kinematic.physics.flags |= 0x08;
    sensor_and_kinematic.physics.flags |= 0x02;
    var entities = [_]entity16.Entity16{ static_entity, sensor_and_kinematic };
    const medium_classifications = [_]ContactMediumClassification{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .solid,
        .material_b = .solid,
        .surface_a = .concrete,
        .surface_b = .concrete,
        .medium_a = .solid,
        .medium_b = .solid,
    }};
    var body_classifications: [1]ContactBodyTypeClassification = undefined;

    const count = classifyContactBodyTypes(medium_classifications[0..], entities[0..], body_classifications[0..]);
    const classification = body_classifications[0];

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(u8, 0), classification.pair.a);
    try std.testing.expectEqual(@as(u8, 1), classification.pair.b);
    try std.testing.expectEqual(query_types.BodyType.static, classification.body_type_a);
    try std.testing.expectEqual(query_types.BodyType.sensor, classification.body_type_b);
    try std.testing.expectEqual(terrain.MediumType.solid, classification.medium_a);
    try std.testing.expectEqual(terrain.MediumType.solid, classification.medium_b);
}

test "classifyContactConditions matches query surface condition semantics" {
    const body_classifications = [_]ContactBodyTypeClassification{
        .{
            .pair = .{ .a = 0, .b = 1 },
            .entity_id_a = 0,
            .entity_id_b = 1,
            .material_a = .liquid,
            .material_b = .elastic,
            .surface_a = .water,
            .surface_b = .rubber,
            .medium_a = .liquid,
            .medium_b = .solid,
            .body_type_a = .dynamic,
            .body_type_b = .dynamic,
        },
        .{
            .pair = .{ .a = 2, .b = 3 },
            .entity_id_a = 2,
            .entity_id_b = 3,
            .material_a = .solid,
            .material_b = .solid,
            .surface_a = .gravel,
            .surface_b = .asphalt_wet,
            .medium_a = .solid,
            .medium_b = .solid,
            .body_type_a = .static,
            .body_type_b = .kinematic,
        },
    };
    var condition_classifications: [2]ContactConditionClassification = undefined;

    const count = classifyContactConditions(body_classifications[0..], condition_classifications[0..]);

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(query_types.SurfaceCondition.submerged, condition_classifications[0].condition_a);
    try std.testing.expectEqual(query_types.SurfaceCondition.slippery, condition_classifications[0].condition_b);
    try std.testing.expectEqual(query_types.SurfaceCondition.loose, condition_classifications[1].condition_a);
    try std.testing.expectEqual(query_types.SurfaceCondition.wet, condition_classifications[1].condition_b);
    try std.testing.expectEqual(query_types.BodyType.kinematic, condition_classifications[1].body_type_b);
}

test "buildContacts joins detected manifold with full contact classification" {
    var elastic = entity16.initEntity16();
    elastic.physics.material = .elastic;
    var liquid = entity16.initEntity16();
    liquid.physics.material = .liquid;
    var entities = [_]entity16.Entity16{ elastic, liquid };
    const manifold: collision.ContactManifold = .{
        .point_count = 1,
        .normal_y = 1.0,
        .penetration_depth = 1.0,
        .points = .{collision.ContactPoint{ .x = 1.0, .y = 2.0, .z = 3.0 }} ** collision.CONTACT_MANIFOLD_MAX_POINTS,
    };
    const detected = [_]ContactDetectedManifold{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 },
        .aabb_b = physics.AABB{ .min_x = 1, .min_y = 1, .min_z = 1, .max_x = 3, .max_y = 3, .max_z = 3 },
        .manifold = manifold,
        .centroid = collision.computeContactManifoldCentroid(manifold),
    }};
    const conditions = [_]ContactConditionClassification{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .elastic,
        .material_b = .liquid,
        .surface_a = .rubber,
        .surface_b = .water,
        .medium_a = .solid,
        .medium_b = .liquid,
        .body_type_a = .dynamic,
        .body_type_b = .dynamic,
        .condition_a = .slippery,
        .condition_b = .submerged,
    }};
    var contacts: [1]Contact = undefined;

    const count = buildContacts(detected[0..], conditions[0..], entities[0..], contacts[0..]);
    const contact = contacts[0];

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(u8, 0), contact.pair.a);
    try std.testing.expectEqual(@as(u8, 1), contact.pair.b);
    try std.testing.expectEqual(@as(u8, 1), contact.manifold.point_count);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), contact.centroid.x, 0.0001);
    try std.testing.expectEqual(entity16.MaterialType.elastic, contact.classification.material_a);
    try std.testing.expectEqual(entity16.MaterialType.liquid, contact.classification.material_b);
    try std.testing.expectEqual(terrain.SurfaceType.rubber, contact.classification.surface_a);
    try std.testing.expectEqual(terrain.SurfaceType.water, contact.classification.surface_b);
    try std.testing.expectEqual(query_types.SurfaceCondition.slippery, contact.classification.condition_a);
    try std.testing.expectEqual(query_types.SurfaceCondition.submerged, contact.classification.condition_b);
    try std.testing.expect(contact.classification.response.buoyancy > 0.0);
}

test "buildContactsCached reuses contact classification by broadphase pair" {
    var elastic = entity16.initEntity16();
    elastic.physics.material = .elastic;
    var liquid = entity16.initEntity16();
    liquid.physics.material = .liquid;
    var entities = [_]entity16.Entity16{ elastic, liquid };
    const manifold: collision.ContactManifold = .{
        .point_count = 1,
        .normal_y = 1.0,
        .penetration_depth = 1.0,
        .points = .{collision.ContactPoint{ .x = 1.0, .y = 2.0, .z = 3.0 }} ** collision.CONTACT_MANIFOLD_MAX_POINTS,
    };
    const detected = [_]ContactDetectedManifold{
        .{
            .pair = .{ .a = 0, .b = 1 },
            .entity_id_a = 0,
            .entity_id_b = 1,
            .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 },
            .aabb_b = physics.AABB{ .min_x = 1, .min_y = 1, .min_z = 1, .max_x = 3, .max_y = 3, .max_z = 3 },
            .manifold = manifold,
            .centroid = collision.computeContactManifoldCentroid(manifold),
        },
        .{
            .pair = .{ .a = 0, .b = 1 },
            .entity_id_a = 0,
            .entity_id_b = 1,
            .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 },
            .aabb_b = physics.AABB{ .min_x = 1, .min_y = 1, .min_z = 1, .max_x = 3, .max_y = 3, .max_z = 3 },
            .manifold = manifold,
            .centroid = collision.computeContactManifoldCentroid(manifold),
        },
    };
    const conditions = [_]ContactConditionClassification{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .elastic,
        .material_b = .liquid,
        .surface_a = .rubber,
        .surface_b = .water,
        .medium_a = .solid,
        .medium_b = .liquid,
        .body_type_a = .dynamic,
        .body_type_b = .dynamic,
        .condition_a = .slippery,
        .condition_b = .submerged,
    }};
    var cache_storage: [1]ContactClassificationCacheEntry = undefined;
    var cache = ContactClassificationCache.init(cache_storage[0..]);
    var contacts: [2]Contact = undefined;

    const count = buildContactsCached(detected[0..], conditions[0..], entities[0..], &cache, contacts[0..]);

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 1), cache.count);
    try std.testing.expectEqual(@as(usize, 2), cache.stats.lookup_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.miss_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.hit_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.insert_count);
    try std.testing.expectEqual(@as(usize, 0), cache.stats.overflow_count);
    try std.testing.expectEqual(entity16.MaterialType.elastic, contacts[1].classification.material_a);
    try std.testing.expectEqual(query_types.SurfaceCondition.submerged, contacts[1].classification.condition_b);
}

test "buildContactsCached keeps output deterministic when classification cache is full" {
    var solid = entity16.initEntity16();
    solid.physics.material = .solid;
    var entities = [_]entity16.Entity16{ solid, solid };
    const manifold: collision.ContactManifold = .{
        .point_count = 1,
        .normal_y = 1.0,
        .penetration_depth = 0.5,
        .points = .{collision.ContactPoint{ .x = 0.0, .y = 1.0, .z = 0.0 }} ** collision.CONTACT_MANIFOLD_MAX_POINTS,
    };
    const detected = [_]ContactDetectedManifold{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 1, .max_y = 1, .max_z = 1 },
        .aabb_b = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 1, .max_y = 1, .max_z = 1 },
        .manifold = manifold,
        .centroid = collision.computeContactManifoldCentroid(manifold),
    }};
    const conditions = [_]ContactConditionClassification{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .solid,
        .material_b = .solid,
        .surface_a = .concrete,
        .surface_b = .concrete,
        .medium_a = .solid,
        .medium_b = .solid,
        .body_type_a = .dynamic,
        .body_type_b = .dynamic,
        .condition_a = .dry,
        .condition_b = .dry,
    }};
    var cache_storage: [0]ContactClassificationCacheEntry = .{};
    var cache = ContactClassificationCache.init(cache_storage[0..]);
    var contacts: [1]Contact = undefined;

    const count = buildContactsCached(detected[0..], conditions[0..], entities[0..], &cache, contacts[0..]);

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(usize, 0), cache.count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.lookup_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.miss_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.overflow_count);
    try std.testing.expectEqual(entity16.MaterialType.solid, contacts[0].classification.material_a);
    try std.testing.expectEqual(query_types.SurfaceCondition.dry, contacts[0].classification.condition_a);
}

test "validateContactClassification accepts buildContacts classification" {
    var elastic = entity16.initEntity16();
    elastic.physics.material = .elastic;
    var liquid = entity16.initEntity16();
    liquid.physics.material = .liquid;
    var entities = [_]entity16.Entity16{ elastic, liquid };
    const response = material_pairing.getPairedResponse(.rubber, .elastic, .water, .liquid);
    const classification: ContactClassification = .{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .elastic,
        .material_b = .liquid,
        .surface_a = .rubber,
        .surface_b = .water,
        .medium_a = .solid,
        .medium_b = .liquid,
        .body_type_a = .dynamic,
        .body_type_b = .dynamic,
        .condition_a = .slippery,
        .condition_b = .submerged,
        .response = response,
    };

    const consistency = validateContactClassification(classification, entities[0..]);

    try std.testing.expect(consistency.entity_ids_valid);
    try std.testing.expect(consistency.pair_order_valid);
    try std.testing.expect(consistency.materials_match_entities);
    try std.testing.expect(consistency.surfaces_match_materials);
    try std.testing.expect(consistency.mediums_match_surfaces);
    try std.testing.expect(consistency.body_types_match_entities);
    try std.testing.expect(consistency.conditions_match_surfaces);
    try std.testing.expect(consistency.response_matches_pair);
    try std.testing.expect(consistency.isConsistent());
    try std.testing.expect(contactClassificationIsConsistent(classification, entities[0..]));
}

test "validateContactClassification reports broken classification fields" {
    var solid = entity16.initEntity16();
    solid.physics.material = .solid;
    var elastic = entity16.initEntity16();
    elastic.physics.material = .elastic;
    var entities = [_]entity16.Entity16{ solid, elastic };
    const valid: ContactClassification = .{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .material_a = .solid,
        .material_b = .elastic,
        .surface_a = .concrete,
        .surface_b = .rubber,
        .medium_a = .solid,
        .medium_b = .solid,
        .body_type_a = .dynamic,
        .body_type_b = .dynamic,
        .condition_a = .dry,
        .condition_b = .slippery,
        .response = material_pairing.getPairedResponse(.concrete, .solid, .rubber, .elastic),
    };

    var bad_material = valid;
    bad_material.material_b = .fragile;
    var bad_surface = valid;
    bad_surface.surface_a = .water;
    var bad_medium = valid;
    bad_medium.medium_b = .liquid;
    var bad_body = valid;
    bad_body.body_type_a = .static;
    var bad_condition = valid;
    bad_condition.condition_b = .dry;
    var bad_response = valid;
    bad_response.response.friction += 0.25;
    var bad_pair_order = valid;
    bad_pair_order.pair = .{ .a = 1, .b = 0 };
    var bad_entity = valid;
    bad_entity.entity_id_b = 9;

    try std.testing.expect(!validateContactClassification(bad_material, entities[0..]).materials_match_entities);
    try std.testing.expect(!validateContactClassification(bad_surface, entities[0..]).surfaces_match_materials);
    try std.testing.expect(!validateContactClassification(bad_medium, entities[0..]).mediums_match_surfaces);
    try std.testing.expect(!validateContactClassification(bad_body, entities[0..]).body_types_match_entities);
    try std.testing.expect(!validateContactClassification(bad_condition, entities[0..]).conditions_match_surfaces);
    try std.testing.expect(!validateContactClassification(bad_response, entities[0..]).response_matches_pair);
    try std.testing.expect(!validateContactClassification(bad_pair_order, entities[0..]).pair_order_valid);
    try std.testing.expect(!validateContactClassification(bad_entity, entities[0..]).entity_ids_valid);
    try std.testing.expect(!contactClassificationIsConsistent(bad_response, entities[0..]));
}

test "buildContactsOptimizedMeasured matches staged contact classification" {
    var elastic = entity16.initEntity16();
    elastic.physics.material = .elastic;
    var fragile = entity16.initEntity16();
    fragile.physics.material = .fragile;
    var entities = [_]entity16.Entity16{ elastic, fragile };
    const manifold: collision.ContactManifold = .{
        .point_count = 1,
        .normal_y = 1.0,
        .penetration_depth = 0.75,
        .points = .{collision.ContactPoint{ .x = 2.0, .y = 3.0, .z = 4.0 }} ** collision.CONTACT_MANIFOLD_MAX_POINTS,
    };
    const detected = [_]ContactDetectedManifold{.{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 },
        .aabb_b = physics.AABB{ .min_x = 1, .min_y = 1, .min_z = 1, .max_x = 3, .max_y = 3, .max_z = 3 },
        .manifold = manifold,
        .centroid = collision.computeContactManifoldCentroid(manifold),
    }};
    var material_classifications: [1]ContactMaterialClassification = undefined;
    var surface_classifications: [1]ContactSurfaceClassification = undefined;
    var medium_classifications: [1]ContactMediumClassification = undefined;
    var body_classifications: [1]ContactBodyTypeClassification = undefined;
    var condition_classifications: [1]ContactConditionClassification = undefined;
    var staged_contacts: [1]Contact = undefined;
    var optimized_contacts: [1]Contact = undefined;
    var stats: ContactClassificationPipelineStats = .{};

    _ = classifyContactMaterials(detected[0..], entities[0..], material_classifications[0..]);
    _ = classifyContactSurfaces(material_classifications[0..], surface_classifications[0..]);
    _ = classifyContactMediums(surface_classifications[0..], medium_classifications[0..]);
    _ = classifyContactBodyTypes(medium_classifications[0..], entities[0..], body_classifications[0..]);
    _ = classifyContactConditions(body_classifications[0..], condition_classifications[0..]);
    const staged_count = buildContacts(detected[0..], condition_classifications[0..], entities[0..], staged_contacts[0..]);
    const optimized_count = buildContactsOptimizedMeasured(detected[0..], entities[0..], null, optimized_contacts[0..], &stats);

    try std.testing.expectEqual(staged_count, optimized_count);
    try std.testing.expectEqual(@as(usize, 1), stats.detected_count);
    try std.testing.expectEqual(@as(usize, 2), stats.entity_lookup_count);
    try std.testing.expectEqual(@as(usize, 1), stats.classified_count);
    try std.testing.expectEqual(@as(usize, 0), stats.skipped_invalid_entities);
    try std.testing.expectEqual(staged_contacts[0].classification.material_a, optimized_contacts[0].classification.material_a);
    try std.testing.expectEqual(staged_contacts[0].classification.material_b, optimized_contacts[0].classification.material_b);
    try std.testing.expectEqual(staged_contacts[0].classification.surface_a, optimized_contacts[0].classification.surface_a);
    try std.testing.expectEqual(staged_contacts[0].classification.condition_b, optimized_contacts[0].classification.condition_b);
    try std.testing.expectApproxEqAbs(
        staged_contacts[0].classification.response.damage_modifier,
        optimized_contacts[0].classification.response.damage_modifier,
        0.0001,
    );
    try std.testing.expect(contactClassificationIsConsistent(optimized_contacts[0].classification, entities[0..]));
}

test "buildContactsOptimizedMeasured reuses classification cache without staging arrays" {
    var elastic = entity16.initEntity16();
    elastic.physics.material = .elastic;
    var liquid = entity16.initEntity16();
    liquid.physics.material = .liquid;
    var entities = [_]entity16.Entity16{ elastic, liquid };
    const manifold: collision.ContactManifold = .{
        .point_count = 1,
        .normal_y = 1.0,
        .penetration_depth = 0.5,
        .points = .{collision.ContactPoint{ .x = 0.0, .y = 1.0, .z = 0.0 }} ** collision.CONTACT_MANIFOLD_MAX_POINTS,
    };
    const detected = [_]ContactDetectedManifold{
        .{
            .pair = .{ .a = 0, .b = 1 },
            .entity_id_a = 0,
            .entity_id_b = 1,
            .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 },
            .aabb_b = physics.AABB{ .min_x = 1, .min_y = 1, .min_z = 1, .max_x = 3, .max_y = 3, .max_z = 3 },
            .manifold = manifold,
            .centroid = collision.computeContactManifoldCentroid(manifold),
        },
        .{
            .pair = .{ .a = 0, .b = 1 },
            .entity_id_a = 0,
            .entity_id_b = 1,
            .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 },
            .aabb_b = physics.AABB{ .min_x = 1, .min_y = 1, .min_z = 1, .max_x = 3, .max_y = 3, .max_z = 3 },
            .manifold = manifold,
            .centroid = collision.computeContactManifoldCentroid(manifold),
        },
    };
    var cache_storage: [1]ContactClassificationCacheEntry = undefined;
    var cache = ContactClassificationCache.init(cache_storage[0..]);
    var contacts: [2]Contact = undefined;
    var stats: ContactClassificationPipelineStats = .{};

    const count = buildContactsOptimizedMeasured(detected[0..], entities[0..], &cache, contacts[0..], &stats);

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 2), stats.classified_count);
    try std.testing.expectEqual(@as(usize, 2), cache.stats.lookup_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.miss_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.hit_count);
    try std.testing.expectEqual(@as(usize, 1), cache.stats.insert_count);
    try std.testing.expectEqual(query_types.SurfaceCondition.submerged, contacts[1].classification.condition_b);
}

test "computeContactPositionCorrectionDepth applies slop and impulse cap" {
    const equation: ConstraintRowEquation = .{
        .effective_mass = 0.5,
        .bias = 1.0,
        .max_impulse = 4.0,
    };
    const policy: ContactPositionCorrectionPolicy = .{
        .allowed_penetration_slop = 0.1,
        .depth_correction_ratio = 0.75,
        .bias_scale = 0.2,
    };

    try std.testing.expectEqual(@as(f32, 0.0), computeContactPositionCorrectionDepth(0.05, equation, policy));
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.775),
        computeContactPositionCorrectionDepth(1.0, equation, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 4.0),
        computeContactPositionCorrectionDepth(16.0, equation, policy),
        0.0001,
    );
}

test "computeContactStiffnessModel maps penetration resistance to conservative scale" {
    const policy = contactStiffnessPolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 1.2),
        computeContactStiffnessModel(0.9, policy).scale,
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.8),
        computeContactStiffnessModel(0.1, policy).scale,
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        policy.max_scale,
        computeContactStiffnessModel(4.0, policy).scale,
        0.0001,
    );
}

test "buildContactNormalEquationWithStiffness scales bias and impulse cap" {
    const stiff = computeContactStiffnessModel(0.9, contactStiffnessPolicy());
    const soft = computeContactStiffnessModel(0.1, contactStiffnessPolicy());

    const stiff_equation = buildContactNormalEquationWithStiffness(0.1, 0.1, 2.0, -4.0, stiff);
    const soft_equation = buildContactNormalEquationWithStiffness(0.1, 0.1, 2.0, -4.0, soft);

    try std.testing.expectApproxEqAbs(@as(f32, 5.0), stiff_equation.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.72), stiff_equation.bias, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 21.6), stiff_equation.max_impulse, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.48), soft_equation.bias, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 14.4), soft_equation.max_impulse, 0.0001);
}

test "computeContactDampingModel maps restitution to damping scale" {
    const policy = contactDampingPolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 1.2),
        computeContactDampingModel(0.1, policy).scale,
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.8),
        computeContactDampingModel(0.9, policy).scale,
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        policy.max_scale,
        computeContactDampingModel(-4.0, policy).scale,
        0.0001,
    );
}

test "buildContactNormalEquationWithResponse scales velocity damping terms" {
    const stiffness = neutralContactStiffnessModel();
    const damped = computeContactDampingModel(0.1, contactDampingPolicy());
    const bouncy = computeContactDampingModel(0.9, contactDampingPolicy());

    const damped_equation = buildContactNormalEquationWithResponse(0.1, 0.1, 2.0, -4.0, stiffness, damped);
    const bouncy_equation = buildContactNormalEquationWithResponse(0.1, 0.1, 2.0, -4.0, stiffness, bouncy);

    try std.testing.expectApproxEqAbs(@as(f32, 0.64), damped_equation.bias, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 18.4), damped_equation.max_impulse, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.56), bouncy_equation.bias, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 17.6), bouncy_equation.max_impulse, 0.0001);
}

test "computeContactFatigueModel accumulates repeated contact damage" {
    const fatigue = computeContactFatigueModel(
        .{
            .penetration_depth = 2.0,
            .relative_normal_speed = 4.0,
            .tangential_speed = 3.0,
            .damage_modifier = 1.5,
            .penetration_resistance = 0.2,
            .repeat_count = 10,
        },
        contactFatiguePolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 2.3), fatigue.stress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.03795), fatigue.single_cycle_damage, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.3795), fatigue.accumulated_damage, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6205), fatigue.remaining_life_fraction, 0.0001);
    try std.testing.expect(!fatigue.failed);
}

test "computeContactFatigueModel clamps exhausted contact life" {
    const fatigue = computeContactFatigueModel(
        .{
            .penetration_depth = 2.0,
            .relative_normal_speed = 4.0,
            .tangential_speed = 3.0,
            .damage_modifier = 1.5,
            .penetration_resistance = 0.2,
            .repeat_count = 100,
        },
        contactFatiguePolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 1.0), fatigue.accumulated_damage, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), fatigue.remaining_life_fraction, 0.0001);
    try std.testing.expect(fatigue.failed);
}

test "computeMediumPostBuoyancyModel applies upward liquid force" {
    const buoyancy = computeMediumPostBuoyancyModel(
        .{
            .buoyancy = 1.0,
            .medium_type = .liquid,
            .penetration_depth = 0.5,
            .mass = 200.0,
            .center_x = 10.0,
            .center_y = 20.0,
            .center_z = 30.0,
            .normal_y = 1.0,
        },
        mediumPostBuoyancyPolicy(),
    );

    try std.testing.expect(buoyancy.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), buoyancy.submerged_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), buoyancy.center_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 19.875), buoyancy.center_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 30.0), buoyancy.center_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2048.0), buoyancy.displaced_volume, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 20.48), buoyancy.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 20.48), buoyancy.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 10.24), buoyancy.velocity_delta_y, 0.0001);
}

test "computeMediumPostBuoyancyModel stays neutral outside liquid" {
    const buoyancy = computeMediumPostBuoyancyModel(
        .{
            .buoyancy = 1.0,
            .medium_type = .solid,
            .penetration_depth = 1.0,
            .mass = 100.0,
        },
        mediumPostBuoyancyPolicy(),
    );

    try std.testing.expect(!buoyancy.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buoyancy.center_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buoyancy.center_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buoyancy.center_z, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buoyancy.displaced_volume, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buoyancy.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buoyancy.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), buoyancy.velocity_delta_y, 0.0001);
}

test "computeSinkDepthModel estimates soft medium sinking depth" {
    const sink = computeSinkDepthModel(
        .{
            .medium_type = .soft,
            .penetration_depth = 1.0,
            .relative_normal_speed = 2.0,
            .tangential_speed = 3.0,
            .mass = 100.0,
            .penetration_resistance = 0.25,
            .buoyancy = 0.0,
        },
        sinkDepthPolicy(),
    );

    try std.testing.expect(sink.active);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), sink.load, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.64), sink.depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.84), sink.support_fraction, 0.0001);
}

test "computeSinkDepthModel ignores solid and reduces liquid sinking by buoyancy" {
    const solid = computeSinkDepthModel(
        .{
            .medium_type = .solid,
            .penetration_depth = 1.0,
            .relative_normal_speed = 2.0,
            .tangential_speed = 3.0,
            .mass = 100.0,
            .penetration_resistance = 0.0,
            .buoyancy = 0.0,
        },
        sinkDepthPolicy(),
    );
    const liquid = computeSinkDepthModel(
        .{
            .medium_type = .liquid,
            .penetration_depth = 1.0,
            .relative_normal_speed = 0.0,
            .tangential_speed = 0.0,
            .mass = 100.0,
            .penetration_resistance = 0.0,
            .buoyancy = 1.0,
        },
        sinkDepthPolicy(),
    );

    try std.testing.expect(!solid.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.depth, 0.0001);
    try std.testing.expect(liquid.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.13125), liquid.depth, 0.0001);
}

test "computeSinkResistanceModel derives damping from sink depth" {
    const resistance = computeSinkResistanceModel(
        .{
            .sink = .{
                .depth = 0.64,
                .load = 1.0,
                .support_fraction = 0.84,
                .active = true,
            },
            .relative_normal_speed = 2.0,
            .tangential_speed = 3.0,
        },
        sinkResistancePolicy(),
    );

    try std.testing.expect(resistance.active);
    try std.testing.expectApproxEqAbs(@as(f32, 1.82), resistance.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.235), resistance.normal_velocity_delta, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2226), resistance.tangential_velocity_delta, 0.0001);
}

test "computeSinkResistanceModel stays neutral without active sink" {
    const resistance = computeSinkResistanceModel(
        .{
            .sink = neutralSinkDepthModel(),
            .relative_normal_speed = 8.0,
            .tangential_speed = 8.0,
        },
        sinkResistancePolicy(),
    );

    try std.testing.expect(!resistance.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), resistance.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), resistance.normal_velocity_delta, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), resistance.tangential_velocity_delta, 0.0001);
}

test "computeSinkRecoveryModel derives recoverable depth from support" {
    const recovery = computeSinkRecoveryModel(
        .{
            .sink = .{
                .depth = 0.64,
                .load = 1.0,
                .support_fraction = 0.84,
                .active = true,
            },
            .resistance = .{
                .force = 1.82,
                .normal_velocity_delta = 0.235,
                .tangential_velocity_delta = 0.2226,
                .active = true,
            },
            .relative_normal_speed = 2.0,
            .tangential_speed = 3.0,
        },
        sinkRecoveryPolicy(),
    );

    try std.testing.expect(recovery.active);
    try std.testing.expect(!recovery.recovered);
    try std.testing.expectApproxEqAbs(@as(f32, 0.2412), recovery.fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.154368), recovery.depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.308736), recovery.rate, 0.0001);
}

test "computeSinkRecoveryModel stays neutral without active sink" {
    const recovery = computeSinkRecoveryModel(
        .{
            .sink = neutralSinkDepthModel(),
            .resistance = neutralSinkResistanceModel(),
            .relative_normal_speed = 8.0,
            .tangential_speed = 8.0,
        },
        sinkRecoveryPolicy(),
    );

    try std.testing.expect(!recovery.active);
    try std.testing.expect(recovery.recovered);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), recovery.depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), recovery.fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), recovery.rate, 0.0001);
}

test "computeMediumPostDragModel attenuates liquid motion" {
    const drag = computeMediumPostDragModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .normal_speed = 10.0,
            .tangential_speed = 4.0,
        },
        mediumPostDragPolicy(),
    );

    try std.testing.expect(drag.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.85), drag.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), drag.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 11.05), drag.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 11.05), drag.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 8.5), drag.normal_velocity_delta, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.55), drag.tangential_velocity_delta, 0.0001);
}

test "computeMediumPostDragModel ignores solid medium" {
    const drag = computeMediumPostDragModel(
        .{
            .medium_type = .solid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .normal_speed = 10.0,
            .tangential_speed = 4.0,
        },
        mediumPostDragPolicy(),
    );

    try std.testing.expect(!drag.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), drag.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), drag.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), drag.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), drag.force, 0.0001);
}

test "computeMediumPostVaporResistanceModel damps vapor motion" {
    const vapor = computeMediumPostVaporResistanceModel(
        .{
            .medium_type = .vapor,
            .penetration_depth = 0.5,
            .normal_speed = 10.0,
            .tangential_speed = 5.0,
            .drag = .{
                .coefficient = 0.08,
                .exposure = 0.5,
                .magnitude = 2.0,
                .force = 2.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
        },
        mediumPostVaporResistancePolicy(),
    );

    try std.testing.expect(vapor.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.18), vapor.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.745), vapor.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 62.9525), vapor.dynamic_pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 12.03145), vapor.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.341), vapor.normal_velocity_delta, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4023), vapor.tangential_velocity_delta, 0.0001);
}

test "computeMediumPostVaporResistanceModel ignores solid and leaves still vapor inactive" {
    const solid = computeMediumPostVaporResistanceModel(
        .{
            .medium_type = .solid,
            .penetration_depth = 1.0,
            .normal_speed = 10.0,
            .tangential_speed = 5.0,
            .drag = neutralMediumPostDragModel(),
        },
        mediumPostVaporResistancePolicy(),
    );
    const still = computeMediumPostVaporResistanceModel(
        .{
            .medium_type = .vapor,
            .penetration_depth = 1.0,
            .normal_speed = 0.0,
            .tangential_speed = 0.0,
            .drag = neutralMediumPostDragModel(),
        },
        mediumPostVaporResistancePolicy(),
    );

    try std.testing.expect(!solid.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.dynamic_pressure, 0.0001);
    try std.testing.expect(!still.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.18), still.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.45), still.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), still.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), still.normal_velocity_delta, 0.0001);
}

test "computeMediumPostVacuumModel removes medium coupling at low pressure" {
    const vacuum = computeMediumPostVacuumModel(
        .{
            .medium_type = .vapor,
            .ambient_pressure = 0.02,
            .drag = .{
                .coefficient = 0.08,
                .exposure = 1.0,
                .magnitude = 3.0,
                .force = 3.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .vapor_resistance = .{
                .coefficient = 0.18,
                .exposure = 1.0,
                .dynamic_pressure = 10.0,
                .force = 2.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .thermal = neutralContactThermalModel(),
            .sound = neutralContactSoundModel(),
        },
        mediumPostVacuumPolicy(),
    );

    try std.testing.expect(vacuum.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.02), vacuum.pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), vacuum.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), vacuum.drag_loss, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), vacuum.thermal_isolation, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), vacuum.sound_attenuation, 0.0001);
}

test "computeMediumPostVacuumModel stays neutral at normal pressure" {
    const vacuum = computeMediumPostVacuumModel(
        .{
            .medium_type = .vapor,
            .ambient_pressure = 1.0,
            .drag = neutralMediumPostDragModel(),
            .vapor_resistance = neutralMediumPostVaporResistanceModel(),
            .thermal = neutralContactThermalModel(),
            .sound = neutralContactSoundModel(),
        },
        mediumPostVacuumPolicy(),
    );

    try std.testing.expect(!vacuum.active);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), vacuum.pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), vacuum.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), vacuum.drag_loss, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), vacuum.thermal_isolation, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), vacuum.sound_attenuation, 0.0001);
}

test "computeMediumPostTransitionModel reports cross-medium transition" {
    const transition = computeMediumPostTransitionModel(
        .{
            .from_medium = .solid,
            .to_medium = .liquid,
            .penetration_depth = 1.0,
            .buoyancy = 0.5,
            .drag = .{
                .coefficient = 0.85,
                .exposure = 0.5,
                .magnitude = 4.0,
                .force = 4.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .vapor_resistance = .{
                .coefficient = 0.18,
                .exposure = 0.25,
                .dynamic_pressure = 8.0,
                .force = 2.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostTransitionPolicy(),
    );

    try std.testing.expect(transition.active);
    try std.testing.expectEqual(terrain.MediumType.solid, transition.from_medium);
    try std.testing.expectEqual(terrain.MediumType.liquid, transition.to_medium);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), transition.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.9), transition.resistance, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), transition.pressure_delta, 0.0001);
}

test "computeMediumPostTransitionModel stays neutral within same medium" {
    const transition = computeMediumPostTransitionModel(
        .{
            .from_medium = .liquid,
            .to_medium = .liquid,
            .penetration_depth = 1.0,
            .buoyancy = 1.0,
            .drag = neutralMediumPostDragModel(),
            .vapor_resistance = neutralMediumPostVaporResistanceModel(),
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostTransitionPolicy(),
    );

    try std.testing.expect(!transition.active);
    try std.testing.expectEqual(terrain.MediumType.solid, transition.from_medium);
    try std.testing.expectEqual(terrain.MediumType.solid, transition.to_medium);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), transition.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), transition.resistance, 0.0001);
}

test "computeMediumPostMixingModel blends cross-medium properties" {
    const mixing = computeMediumPostMixingModel(
        .{
            .primary_medium = .liquid,
            .secondary_medium = .vapor,
            .buoyancy = 0.8,
            .transition = .{
                .from_medium = .liquid,
                .to_medium = .vapor,
                .progress = 0.25,
                .resistance = 4.0,
                .pressure_delta = 0.0,
                .active = true,
            },
            .drag = .{
                .coefficient = 0.85,
                .exposure = 0.5,
                .magnitude = 6.0,
                .force = 6.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .vapor_resistance = .{
                .coefficient = 0.18,
                .exposure = 0.25,
                .dynamic_pressure = 8.0,
                .force = 2.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostMixingPolicy(),
    );

    try std.testing.expect(mixing.active);
    try std.testing.expectEqual(terrain.MediumType.liquid, mixing.primary_medium);
    try std.testing.expectEqual(terrain.MediumType.vapor, mixing.secondary_medium);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), mixing.mix_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.77), mixing.effective_density, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6575), mixing.effective_viscosity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.1), mixing.blended_drag, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0539), mixing.blended_buoyancy, 0.0001);
}

test "computeMediumPostMixingModel stays neutral without transition or medium change" {
    const same_medium = computeMediumPostMixingModel(
        .{
            .primary_medium = .liquid,
            .secondary_medium = .liquid,
            .buoyancy = 1.0,
            .transition = .{
                .from_medium = .liquid,
                .to_medium = .liquid,
                .progress = 1.0,
                .resistance = 4.0,
                .pressure_delta = 0.0,
                .active = true,
            },
            .drag = neutralMediumPostDragModel(),
            .vapor_resistance = neutralMediumPostVaporResistanceModel(),
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostMixingPolicy(),
    );
    const inactive_transition = computeMediumPostMixingModel(
        .{
            .primary_medium = .liquid,
            .secondary_medium = .vapor,
            .buoyancy = 1.0,
            .transition = neutralMediumPostTransitionModel(),
            .drag = neutralMediumPostDragModel(),
            .vapor_resistance = neutralMediumPostVaporResistanceModel(),
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostMixingPolicy(),
    );

    try std.testing.expect(!same_medium.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), same_medium.mix_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), same_medium.effective_density, 0.0001);
    try std.testing.expect(!inactive_transition.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), inactive_transition.blended_drag, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), inactive_transition.blended_buoyancy, 0.0001);
}

test "computeMediumPostStateMachineModel derives mixing and settling states" {
    const mixing_state = computeMediumPostStateMachineModel(
        .{
            .current_medium = .liquid,
            .target_medium = .vapor,
            .transition = .{
                .from_medium = .liquid,
                .to_medium = .vapor,
                .progress = 0.5,
                .resistance = 1.0,
                .pressure_delta = 0.0,
                .active = true,
            },
            .mixing = .{
                .primary_medium = .liquid,
                .secondary_medium = .vapor,
                .mix_fraction = 0.5,
                .effective_density = 0.54,
                .effective_viscosity = 0.465,
                .blended_drag = 0.4,
                .blended_buoyancy = 0.1,
                .active = true,
            },
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostStateMachinePolicy(),
    );
    const settling_state = computeMediumPostStateMachineModel(
        .{
            .current_medium = .liquid,
            .target_medium = .vapor,
            .transition = .{
                .from_medium = .liquid,
                .to_medium = .vapor,
                .progress = 0.9,
                .resistance = 1.0,
                .pressure_delta = 0.0,
                .active = true,
            },
            .mixing = .{
                .primary_medium = .liquid,
                .secondary_medium = .vapor,
                .mix_fraction = 0.9,
                .effective_density = 0.172,
                .effective_viscosity = 0.157,
                .blended_drag = 0.4,
                .blended_buoyancy = 0.1,
                .active = true,
            },
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostStateMachinePolicy(),
    );

    try std.testing.expect(mixing_state.active);
    try std.testing.expectEqual(MediumPostState.mixing, mixing_state.state);
    try std.testing.expectEqual(terrain.MediumType.liquid, mixing_state.current_medium);
    try std.testing.expectEqual(terrain.MediumType.vapor, mixing_state.target_medium);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), mixing_state.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6), mixing_state.stability, 0.0001);
    try std.testing.expect(settling_state.active);
    try std.testing.expectEqual(MediumPostState.settling, settling_state.state);
    try std.testing.expectApproxEqAbs(@as(f32, 0.9), settling_state.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.28), settling_state.stability, 0.0001);
}

test "computeMediumPostStateMachineModel isolates vacuum and stays neutral at rest" {
    const isolated = computeMediumPostStateMachineModel(
        .{
            .current_medium = .vapor,
            .target_medium = .vapor,
            .transition = neutralMediumPostTransitionModel(),
            .mixing = neutralMediumPostMixingModel(),
            .vacuum = .{
                .pressure = 0.02,
                .exposure = 0.75,
                .drag_loss = 1.0,
                .thermal_isolation = 0.75,
                .sound_attenuation = 0.75,
                .active = true,
            },
        },
        mediumPostStateMachinePolicy(),
    );
    const stable = computeMediumPostStateMachineModel(
        .{
            .current_medium = .liquid,
            .target_medium = .liquid,
            .transition = neutralMediumPostTransitionModel(),
            .mixing = neutralMediumPostMixingModel(),
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostStateMachinePolicy(),
    );

    try std.testing.expect(isolated.active);
    try std.testing.expectEqual(MediumPostState.isolated, isolated.state);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), isolated.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.85), isolated.stability, 0.0001);
    try std.testing.expect(!stable.active);
    try std.testing.expectEqual(MediumPostState.stable, stable.state);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), stable.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), stable.stability, 0.0001);
}

test "computeMediumPostEventTriggerModel emits mixing and isolation triggers" {
    const mixing_event = computeMediumPostEventTriggerModel(
        .{
            .state = .{
                .state = .mixing,
                .current_medium = .liquid,
                .target_medium = .vapor,
                .progress = 0.5,
                .stability = 0.6,
                .active = true,
            },
            .transition = .{
                .from_medium = .liquid,
                .to_medium = .vapor,
                .progress = 0.5,
                .resistance = 2.0,
                .pressure_delta = 0.0,
                .active = true,
            },
            .mixing = .{
                .primary_medium = .liquid,
                .secondary_medium = .vapor,
                .mix_fraction = 0.5,
                .effective_density = 0.54,
                .effective_viscosity = 0.465,
                .blended_drag = 0.4,
                .blended_buoyancy = 0.1,
                .active = true,
            },
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostEventTriggerPolicy(),
    );
    const isolated_event = computeMediumPostEventTriggerModel(
        .{
            .state = .{
                .state = .isolated,
                .current_medium = .vapor,
                .target_medium = .vapor,
                .progress = 0.75,
                .stability = 0.85,
                .active = true,
            },
            .transition = neutralMediumPostTransitionModel(),
            .mixing = neutralMediumPostMixingModel(),
            .vacuum = .{
                .pressure = 0.02,
                .exposure = 0.75,
                .drag_loss = 1.0,
                .thermal_isolation = 0.75,
                .sound_attenuation = 0.75,
                .active = true,
            },
        },
        mediumPostEventTriggerPolicy(),
    );

    try std.testing.expect(mixing_event.active);
    try std.testing.expectEqual(MediumPostEventType.mix, mixing_event.event_type);
    try std.testing.expectEqual(terrain.MediumType.liquid, mixing_event.source_medium);
    try std.testing.expectEqual(terrain.MediumType.vapor, mixing_event.target_medium);
    try std.testing.expectApproxEqAbs(@as(f32, 0.43), mixing_event.intensity, 0.0001);
    try std.testing.expectEqual(bus.BusPriority.NORMAL, mixing_event.priority);
    try std.testing.expect(isolated_event.active);
    try std.testing.expectEqual(MediumPostEventType.isolate, isolated_event.event_type);
    try std.testing.expectApproxEqAbs(@as(f32, 0.51), isolated_event.intensity, 0.0001);
    try std.testing.expectEqual(bus.BusPriority.NORMAL, isolated_event.priority);
}

test "computeMediumPostEventTriggerModel stays neutral without active state" {
    const event = computeMediumPostEventTriggerModel(
        .{
            .state = neutralMediumPostStateMachineModel(),
            .transition = neutralMediumPostTransitionModel(),
            .mixing = neutralMediumPostMixingModel(),
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostEventTriggerPolicy(),
    );

    try std.testing.expect(!event.active);
    try std.testing.expectEqual(MediumPostEventType.none, event.event_type);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), event.intensity, 0.0001);
    try std.testing.expectEqual(bus.BusPriority.LOW, event.priority);
}

test "computeMediumPostTransitionAnimationModel derives transition visuals" {
    const animation = computeMediumPostTransitionAnimationModel(
        .{
            .state = .{
                .state = .mixing,
                .current_medium = .liquid,
                .target_medium = .vapor,
                .progress = 0.5,
                .stability = 0.6,
                .active = true,
            },
            .event = .{
                .event_type = .mix,
                .source_medium = .liquid,
                .target_medium = .vapor,
                .intensity = 0.43,
                .priority = .NORMAL,
                .active = true,
            },
            .transition = .{
                .from_medium = .liquid,
                .to_medium = .vapor,
                .progress = 0.5,
                .resistance = 2.0,
                .pressure_delta = 0.0,
                .active = true,
            },
            .mixing = .{
                .primary_medium = .liquid,
                .secondary_medium = .vapor,
                .mix_fraction = 0.5,
                .effective_density = 0.54,
                .effective_viscosity = 0.465,
                .blended_drag = 0.4,
                .blended_buoyancy = 0.1,
                .active = true,
            },
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostTransitionAnimationPolicy(),
    );

    try std.testing.expect(animation.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6075), animation.phase, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), animation.blend, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.575), animation.opacity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4185), animation.ripple, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1995), animation.turbulence, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.3), animation.color_shift, 0.0001);
}

test "computeMediumPostTransitionAnimationModel stays neutral without progress" {
    const animation = computeMediumPostTransitionAnimationModel(
        .{
            .state = neutralMediumPostStateMachineModel(),
            .event = neutralMediumPostEventTriggerModel(),
            .transition = neutralMediumPostTransitionModel(),
            .mixing = neutralMediumPostMixingModel(),
            .vacuum = neutralMediumPostVacuumModel(),
        },
        mediumPostTransitionAnimationPolicy(),
    );

    try std.testing.expect(!animation.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), animation.phase, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), animation.opacity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), animation.color_shift, 0.0001);
}

test "computeMediumPostTowModel carries body along medium flow" {
    const tow = computeMediumPostTowModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .flow_x = 3.0,
            .flow_y = 0.0,
            .flow_z = 4.0,
            .drag = .{
                .coefficient = 0.85,
                .exposure = 1.0,
                .magnitude = 10.0,
                .force = 10.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 2.0,
                .active = true,
            },
        },
        mediumPostTowPolicy(),
    );

    try std.testing.expect(tow.active);
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), tow.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.4), tow.velocity_delta_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), tow.velocity_delta_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.2), tow.velocity_delta_z, 0.0001);
}

test "computeMediumPostTowModel stays neutral without flow" {
    const tow = computeMediumPostTowModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .flow_x = 0.0,
            .flow_y = 0.0,
            .flow_z = 0.0,
            .drag = neutralMediumPostDragModel(),
        },
        mediumPostTowPolicy(),
    );

    try std.testing.expect(!tow.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), tow.force, 0.0001);
}

test "computeMediumPostLiftModel produces normal lift from liquid flow" {
    const lift = computeMediumPostLiftModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .flow_x = 0.0,
            .flow_y = 6.0,
            .flow_z = 0.0,
            .normal_x = 0.0,
            .normal_y = 1.0,
            .normal_z = 0.0,
            .tow = .{
                .force = 2.0,
                .velocity_delta_x = 0.0,
                .velocity_delta_y = 0.0,
                .velocity_delta_z = 0.0,
                .active = true,
            },
        },
        mediumPostLiftPolicy(),
    );

    try std.testing.expect(lift.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.12), lift.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), lift.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.456), lift.dynamic_pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.584), lift.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.584), lift.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), lift.velocity_delta_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.584), lift.velocity_delta_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), lift.velocity_delta_z, 0.0001);
}

test "computeMediumPostLiftModel stays neutral without flow or solid medium" {
    const no_flow = computeMediumPostLiftModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .flow_x = 0.0,
            .flow_y = 0.0,
            .flow_z = 0.0,
            .normal_x = 0.0,
            .normal_y = 1.0,
            .normal_z = 0.0,
            .tow = neutralMediumPostTowModel(),
        },
        mediumPostLiftPolicy(),
    );
    const solid = computeMediumPostLiftModel(
        .{
            .medium_type = .solid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .flow_x = 0.0,
            .flow_y = 6.0,
            .flow_z = 0.0,
            .normal_x = 0.0,
            .normal_y = 1.0,
            .normal_z = 0.0,
            .tow = .{
                .force = 2.0,
                .velocity_delta_x = 0.0,
                .velocity_delta_y = 0.0,
                .velocity_delta_z = 0.0,
                .active = true,
            },
        },
        mediumPostLiftPolicy(),
    );

    try std.testing.expect(!no_flow.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), no_flow.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), no_flow.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), no_flow.dynamic_pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), no_flow.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), no_flow.force, 0.0001);
    try std.testing.expect(!solid.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.dynamic_pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.force, 0.0001);
}

test "computeMediumPostAddedMassModel derives liquid inertia from displaced volume" {
    const added_mass = computeMediumPostAddedMassModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .base_mass = 100.0,
            .buoyancy_model = .{
                .submerged_fraction = 1.0,
                .displaced_volume = 1000.0,
                .magnitude = 0.0,
                .center_x = 0.0,
                .center_y = 0.0,
                .center_z = 0.0,
                .force = 0.0,
                .velocity_delta_y = 0.0,
                .active = true,
            },
            .drag = .{
                .coefficient = 0.85,
                .exposure = 1.0,
                .magnitude = 0.0,
                .force = 0.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .lift = .{
                .coefficient = 0.12,
                .exposure = 1.0,
                .dynamic_pressure = 0.0,
                .magnitude = 0.0,
                .force = 0.0,
                .velocity_delta_x = 0.0,
                .velocity_delta_y = 0.0,
                .velocity_delta_z = 0.0,
                .active = true,
            },
        },
        mediumPostAddedMassPolicy(),
    );

    try std.testing.expect(added_mass.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.55), added_mass.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), added_mass.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1000.0), added_mass.displaced_volume, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.5), added_mass.added_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 105.5), added_mass.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.055), added_mass.normal_inertia_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.03575), added_mass.tangential_inertia_scale, 0.0001);
}

test "computeMediumPostAddedMassModel stays neutral outside fluid or massless body" {
    const solid = computeMediumPostAddedMassModel(
        .{
            .medium_type = .solid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .base_mass = 100.0,
            .buoyancy_model = neutralMediumPostBuoyancyModel(),
            .drag = neutralMediumPostDragModel(),
            .lift = neutralMediumPostLiftModel(),
        },
        mediumPostAddedMassPolicy(),
    );
    const massless = computeMediumPostAddedMassModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .penetration_depth = 1.0,
            .base_mass = 0.0,
            .buoyancy_model = neutralMediumPostBuoyancyModel(),
            .drag = neutralMediumPostDragModel(),
            .lift = neutralMediumPostLiftModel(),
        },
        mediumPostAddedMassPolicy(),
    );

    try std.testing.expect(!solid.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.added_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), solid.normal_inertia_scale, 0.0001);
    try std.testing.expect(!massless.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), massless.added_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), massless.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), massless.tangential_inertia_scale, 0.0001);
}

test "computeContactThermalModel converts impact and friction into retained heat" {
    const thermal = computeContactThermalModel(
        .{
            .penetration_depth = 2.0,
            .relative_normal_speed = 4.0,
            .tangential_speed = 3.0,
            .friction = 0.8,
            .penetration_resistance = 0.5,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        contactThermalPolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 1.79), thermal.generated_heat, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.24), thermal.friction_heat, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1340782), thermal.friction_heat_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), thermal.conductivity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.67125), thermal.dissipated_heat, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.11875), thermal.retained_heat, 0.0001);
    try std.testing.expectApproxEqAbs(thermal.retained_heat, thermal.temperature_delta, 0.0001);
}

test "computeContactFrictionHeat includes rolling and anisotropic heating" {
    const policy = contactThermalPolicy();
    const base: ContactThermalInput = .{
        .penetration_depth = 0.0,
        .relative_normal_speed = 0.0,
        .tangential_speed = 10.0,
        .friction = 0.5,
        .penetration_resistance = 0.0,
        .buoyancy = 0.0,
        .medium_type = .solid,
    };
    var rolling = base;
    rolling.rolling_friction = 0.1;
    rolling.angular_speed = 20.0;
    var anisotropic = rolling;
    anisotropic.anisotropic_minor_axis_scale = 0.4;

    try std.testing.expectApproxEqAbs(@as(f32, 0.5), computeContactFrictionHeat(base, policy), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.6), computeContactFrictionHeat(rolling, policy), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), computeContactFrictionHeat(anisotropic, policy), 0.0001);
}

test "computeContactThermalModel exposes friction heat dominated contacts" {
    const thermal = computeContactThermalModel(
        .{
            .penetration_depth = 0.0,
            .relative_normal_speed = 0.0,
            .tangential_speed = 10.0,
            .friction = 0.5,
            .rolling_friction = 0.1,
            .angular_speed = 20.0,
            .anisotropic_minor_axis_scale = 0.4,
            .penetration_resistance = 0.0,
            .buoyancy = 0.0,
            .medium_type = .solid,
        },
        contactThermalPolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 0.75), thermal.generated_heat, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), thermal.friction_heat, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), thermal.friction_heat_fraction, 0.0001);
    try std.testing.expect(thermal.retained_heat > 0.0);
}

test "computeContactThermalModel treats liquid contacts as stronger heat sinks" {
    const base: ContactThermalInput = .{
        .penetration_depth = 2.0,
        .relative_normal_speed = 4.0,
        .tangential_speed = 3.0,
        .friction = 0.8,
        .penetration_resistance = 0.5,
        .buoyancy = 0.0,
        .medium_type = .solid,
    };
    var liquid_input = base;
    liquid_input.medium_type = .liquid;
    liquid_input.buoyancy = 1.0;

    const solid = computeContactThermalModel(base, contactThermalPolicy());
    const liquid = computeContactThermalModel(liquid_input, contactThermalPolicy());

    try std.testing.expect(liquid.conductivity > solid.conductivity);
    try std.testing.expect(liquid.retained_heat < solid.retained_heat);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), liquid.retained_heat, 0.0001);
}

test "computeMediumPostThermalModel conducts retained heat through liquid medium" {
    const medium_thermal = computeMediumPostThermalModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .thermal = .{
                .generated_heat = 12.0,
                .conductivity = 0.5,
                .dissipated_heat = 2.0,
                .retained_heat = 10.0,
                .temperature_delta = 10.0,
            },
            .drag = .{
                .coefficient = 0.85,
                .exposure = 1.0,
                .magnitude = 20.0,
                .force = 20.0,
                .normal_velocity_delta = 0.0,
                .tangential_velocity_delta = 0.0,
                .active = true,
            },
            .tow = .{
                .force = 5.0,
                .velocity_delta_x = 0.0,
                .velocity_delta_y = 0.0,
                .velocity_delta_z = 0.0,
                .active = true,
            },
            .lift = .{
                .coefficient = 0.12,
                .exposure = 1.0,
                .dynamic_pressure = 4.0,
                .magnitude = 4.0,
                .force = 4.0,
                .velocity_delta_x = 0.0,
                .velocity_delta_y = 0.0,
                .velocity_delta_z = 0.0,
                .active = true,
            },
        },
        mediumPostThermalPolicy(),
    );

    try std.testing.expect(medium_thermal.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.9), medium_thermal.conductivity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 9.459), medium_thermal.conducted_heat, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.051), medium_thermal.retained_heat, 0.0001);
    try std.testing.expectApproxEqAbs(medium_thermal.retained_heat, medium_thermal.temperature_delta, 0.0001);
}

test "computeMediumPostThermalModel stays neutral for solid medium or no heat" {
    const thermal: ContactThermalModel = .{
        .generated_heat = 12.0,
        .conductivity = 0.5,
        .dissipated_heat = 2.0,
        .retained_heat = 10.0,
        .temperature_delta = 10.0,
    };
    const solid = computeMediumPostThermalModel(
        .{
            .medium_type = .solid,
            .buoyancy = 1.0,
            .thermal = thermal,
            .drag = neutralMediumPostDragModel(),
            .tow = neutralMediumPostTowModel(),
            .lift = neutralMediumPostLiftModel(),
        },
        mediumPostThermalPolicy(),
    );
    const no_heat = computeMediumPostThermalModel(
        .{
            .medium_type = .liquid,
            .buoyancy = 1.0,
            .thermal = neutralContactThermalModel(),
            .drag = neutralMediumPostDragModel(),
            .tow = neutralMediumPostTowModel(),
            .lift = neutralMediumPostLiftModel(),
        },
        mediumPostThermalPolicy(),
    );

    try std.testing.expect(!solid.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), solid.conducted_heat, 0.0001);
    try std.testing.expect(!no_heat.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), no_heat.conducted_heat, 0.0001);
}

test "computeContactSoundModel maps impact to audible event" {
    const sound = computeContactSoundModel(
        .{
            .sound_type = 14,
            .penetration_depth = 1.5,
            .relative_normal_speed = 3.0,
            .tangential_speed = 2.0,
            .friction = 0.8,
            .restitution = 0.5,
            .medium_type = .solid,
        },
        contactSoundPolicy(),
    );

    try std.testing.expectEqual(@as(u8, 14), sound.sound_type);
    try std.testing.expect(sound.audible);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), sound.volume, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.04), sound.pitch, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.09), sound.duration, 0.0001);
}

test "computeContactSoundModel attenuates liquid contact volume" {
    const base: ContactSoundInput = .{
        .sound_type = 10,
        .penetration_depth = 1.0,
        .relative_normal_speed = 2.0,
        .tangential_speed = 1.0,
        .friction = 0.5,
        .restitution = 0.4,
        .medium_type = .solid,
    };
    var liquid_input = base;
    liquid_input.medium_type = .liquid;

    const solid = computeContactSoundModel(base, contactSoundPolicy());
    const liquid = computeContactSoundModel(liquid_input, contactSoundPolicy());

    try std.testing.expect(liquid.volume < solid.volume);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4875), liquid.volume, 0.0001);
    try std.testing.expect(liquid.audible);
}

test "computeContactDustModel maps sliding soft contact to dust plume" {
    const dust = computeContactDustModel(
        .{
            .dust_type = 10,
            .penetration_depth = 1.5,
            .relative_normal_speed = 2.0,
            .tangential_speed = 3.0,
            .friction = 0.8,
            .buoyancy = 0.0,
            .medium_type = .soft,
        },
        contactDustPolicy(),
    );

    try std.testing.expectEqual(@as(u8, 10), dust.dust_type);
    try std.testing.expect(dust.emitted);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), dust.intensity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.9), dust.radius, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.85), dust.duration, 0.0001);
}

test "computeContactDustModel suppresses liquid dust plume" {
    const base: ContactDustInput = .{
        .dust_type = 12,
        .penetration_depth = 1.0,
        .relative_normal_speed = 2.0,
        .tangential_speed = 1.0,
        .friction = 0.5,
        .buoyancy = 0.0,
        .medium_type = .solid,
    };
    var liquid_input = base;
    liquid_input.medium_type = .liquid;
    liquid_input.buoyancy = 1.0;

    const solid = computeContactDustModel(base, contactDustPolicy());
    const liquid = computeContactDustModel(liquid_input, contactDustPolicy());

    try std.testing.expect(liquid.intensity < solid.intensity);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0918125), liquid.intensity, 0.0001);
    try std.testing.expect(liquid.emitted);
}

test "computeContactDeformationModel splits elastic and permanent depth" {
    const deformation = computeContactDeformationModel(
        .{
            .penetration_depth = 2.0,
            .relative_normal_speed = 4.0,
            .tangential_speed = 3.0,
            .penetration_resistance = 0.2,
            .damage_modifier = 1.5,
            .restitution = 0.2,
        },
        contactDeformationPolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 2.071875), deformation.total_depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.7769531), deformation.elastic_depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.2949219), deformation.permanent_depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.375), deformation.recovery_fraction, 0.0001);
    try std.testing.expect(!deformation.severe);
}

test "computeContactDeformationModel flags severe deformation at clamp" {
    const deformation = computeContactDeformationModel(
        .{
            .penetration_depth = 16.0,
            .relative_normal_speed = 16.0,
            .tangential_speed = 8.0,
            .penetration_resistance = 0.0,
            .damage_modifier = 4.0,
            .restitution = 0.0,
        },
        contactDeformationPolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 4.0), deformation.total_depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), deformation.elastic_depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.6), deformation.permanent_depth, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.1), deformation.recovery_fraction, 0.0001);
    try std.testing.expect(deformation.severe);
}

test "computeContactSeparationModel detects closing contact" {
    const separation = computeContactSeparationModel(
        .{
            .penetration_depth = 1.0,
            .predicted_penetration_depth = 1.25,
            .normal_speed_signed = -0.25,
        },
        contactSeparationPolicy(),
    );

    try std.testing.expectEqual(ContactSeparationState.closing, separation.state);
    try std.testing.expect(!separation.separating);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), separation.speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), separation.estimated_time, 0.0001);
}

test "computeContactSeparationModel keeps slow drift persisting" {
    const separation = computeContactSeparationModel(
        .{
            .penetration_depth = 1.0,
            .predicted_penetration_depth = 1.0,
            .normal_speed_signed = 0.005,
        },
        contactSeparationPolicy(),
    );

    try std.testing.expectEqual(ContactSeparationState.persisting, separation.state);
    try std.testing.expect(!separation.separating);
    try std.testing.expectApproxEqAbs(@as(f32, 0.005), separation.speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), separation.estimated_time, 0.0001);
}

test "computeContactSeparationModel estimates separating contact time" {
    const separation = computeContactSeparationModel(
        .{
            .penetration_depth = 0.5,
            .predicted_penetration_depth = 0.25,
            .normal_speed_signed = 0.25,
        },
        contactSeparationPolicy(),
    );

    try std.testing.expectEqual(ContactSeparationState.separating, separation.state);
    try std.testing.expect(separation.separating);
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), separation.speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), separation.estimated_time, 0.0001);
}

test "computeContactStabilizationModel boosts resting persistent contact" {
    const stabilization = computeContactStabilizationModel(
        .{
            .penetration_depth = 0.1,
            .predicted_penetration_depth = 0.1,
            .separation = .{
                .state = .persisting,
                .separating = false,
                .speed = 0.0,
                .estimated_time = 0.0,
            },
        },
        contactStabilizationPolicy(),
    );

    try std.testing.expect(stabilization.stabilized);
    try std.testing.expectApproxEqAbs(@as(f32, 1.12), stabilization.bias_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.08), stabilization.impulse_scale, 0.0001);
}

test "computeContactStabilizationModel softens separating contact" {
    const stabilization = computeContactStabilizationModel(
        .{
            .penetration_depth = 0.5,
            .predicted_penetration_depth = 0.25,
            .separation = .{
                .state = .separating,
                .separating = true,
                .speed = 0.25,
                .estimated_time = 1.0,
            },
        },
        contactStabilizationPolicy(),
    );

    try std.testing.expect(stabilization.stabilized);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), stabilization.bias_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), stabilization.impulse_scale, 0.0001);
}

test "computeDamageEvalImpactModel derives legacy impact and damage energy" {
    const impact = computeDamageEvalImpactModel(
        .{
            .normal_speed = 20.0,
            .tangential_speed = 10.0,
            .mass = 500.0,
            .material = .fragile,
            .hardness = 10,
            .damage_modifier = 1.5,
        },
        damageEvalImpactPolicy(),
    );

    try std.testing.expectEqual(@as(u16, 100), impact.legacy_impact);
    try std.testing.expectApproxEqAbs(@as(f32, 106250.0), impact.kinetic_energy, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 459.375), impact.damage_amount, 0.0001);
}

test "computeDamageEvalImpactFromVelocity preserves legacy falling impact semantics" {
    var entity = entity16.initEntity16();
    entity.physics.mass = 500;
    entity.physics.material = .fragile;
    entity.physics.hardness = 10;

    const impact = computeDamageEvalImpactFromVelocity(&entity, -20);
    const upward = computeDamageEvalImpactFromVelocity(&entity, 20);

    try std.testing.expectEqual(physics.calcImpact(-20, entity.physics.mass), impact.legacy_impact);
    try std.testing.expect(impact.damage_amount > 0.0);
    try std.testing.expectEqual(@as(u16, 0), upward.legacy_impact);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), upward.damage_amount, 0.0001);
}

test "computeDamageEvalHardnessModel compares solid impact against hardness threshold" {
    const hardness = computeDamageEvalHardnessModel(
        .{
            .impact = .{
                .legacy_impact = 60,
                .normal_speed = 0.0,
                .tangential_speed = 0.0,
                .kinetic_energy = 0.0,
                .damage_amount = 0.0,
            },
            .material = .solid,
            .hardness = 40,
        },
        damageEvalHardnessPolicy(),
    );

    try std.testing.expectEqual(@as(u16, 40), hardness.hardness);
    try std.testing.expectApproxEqAbs(@as(f32, 40.0), hardness.impact_threshold, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), hardness.impact_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 40.0 / 255.0), hardness.hardness_resistance, 0.0001);
    try std.testing.expect(hardness.exceeds_hardness);
}

test "computeDamageEvalHardnessModel keeps high hardness below threshold" {
    const hardness = computeDamageEvalHardnessModel(
        .{
            .impact = .{
                .legacy_impact = 60,
                .normal_speed = 0.0,
                .tangential_speed = 0.0,
                .kinetic_energy = 0.0,
                .damage_amount = 0.0,
            },
            .material = .solid,
            .hardness = 200,
        },
        damageEvalHardnessPolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 200.0), hardness.impact_threshold, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.3), hardness.impact_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 200.0 / 255.0), hardness.hardness_resistance, 0.0001);
    try std.testing.expect(!hardness.exceeds_hardness);
}

test "computeDamageEvalHardnessModel preserves fragile legacy threshold" {
    const hardness = computeDamageEvalHardnessModel(
        .{
            .impact = .{
                .legacy_impact = 60,
                .normal_speed = 0.0,
                .tangential_speed = 0.0,
                .kinetic_energy = 0.0,
                .damage_amount = 0.0,
            },
            .material = .fragile,
            .hardness = 200,
        },
        damageEvalHardnessPolicy(),
    );

    try std.testing.expectApproxEqAbs(@as(f32, 50.0), hardness.impact_threshold, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.2), hardness.impact_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 200.0 / 255.0), hardness.hardness_resistance, 0.0001);
    try std.testing.expect(hardness.exceeds_hardness);
}

test "computeDamageEvalBreakModel preserves solid hardness break semantics" {
    const impact: DamageEvalImpactModel = .{
        .legacy_impact = 60,
        .normal_speed = 0.0,
        .tangential_speed = 0.0,
        .kinetic_energy = 0.0,
        .damage_amount = 0.0,
    };
    const hardness = computeDamageEvalHardnessModel(
        .{
            .impact = impact,
            .material = .solid,
            .hardness = 40,
        },
        damageEvalHardnessPolicy(),
    );
    const break_model = computeDamageEvalBreakModel(.{
        .impact = impact,
        .hardness = hardness,
        .material = .solid,
    });

    try std.testing.expectEqual(@as(u16, 60), break_model.legacy_impact);
    try std.testing.expectApproxEqAbs(@as(f32, 40.0), break_model.impact_threshold, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), break_model.impact_ratio, 0.0001);
    try std.testing.expect(break_model.did_break);
    try std.testing.expectEqual(@as(u8, 2), break_model.fragments);
}

test "computeDamageEvalBreakModel preserves fragile fixed threshold semantics" {
    const impact: DamageEvalImpactModel = .{
        .legacy_impact = 49,
        .normal_speed = 0.0,
        .tangential_speed = 0.0,
        .kinetic_energy = 0.0,
        .damage_amount = 0.0,
    };
    const hardness = computeDamageEvalHardnessModel(
        .{
            .impact = impact,
            .material = .fragile,
            .hardness = 1,
        },
        damageEvalHardnessPolicy(),
    );
    const break_model = computeDamageEvalBreakModel(.{
        .impact = impact,
        .hardness = hardness,
        .material = .fragile,
    });

    try std.testing.expectApproxEqAbs(@as(f32, 50.0), break_model.impact_threshold, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.98), break_model.impact_ratio, 0.0001);
    try std.testing.expect(!break_model.did_break);
    try std.testing.expectEqual(@as(u8, 0), break_model.fragments);
}

test "shouldBreakFromImpact uses DamageEval break model" {
    var fragile = entity16.initEntity16();
    fragile.physics.mass = 500;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 200;

    var solid = entity16.initEntity16();
    solid.physics.mass = 500;
    solid.physics.material = .solid;
    solid.physics.hardness = 200;

    var entities = [_]entity16.Entity16{ fragile, solid };
    const fragile_inst: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var solid_inst = fragile_inst;
    solid_inst.entity_id = 1;

    try std.testing.expect(shouldBreakFromImpact(entities[0..], &fragile_inst, -10));
    try std.testing.expect(!shouldBreakFromImpact(entities[0..], &solid_inst, -10));
    try std.testing.expect(!shouldBreakFromImpact(entities[0..], &fragile_inst, 10));
}

test "computeDamageEvalFragmentModel generates active fragments for broken entity" {
    var fragile = entity16.initEntity16();
    fragile.physics.mass = 400;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);
    entity16.setVoxel(&fragile, 4, 0, 0);
    entity16.setVoxel(&fragile, 8, 0, 0);
    entity16.setVoxel(&fragile, 12, 0, 0);

    const impact = computeDamageEvalImpactFromVelocity(&fragile, -20);
    const hardness = computeDamageEvalHardnessForEntity(&fragile, impact);
    const break_model = computeDamageEvalBreakForEntity(&fragile, impact, hardness);
    const fragments = computeDamageEvalFragmentsForEntity(&fragile, impact, break_model, -20, 0.5);

    try std.testing.expect(break_model.did_break);
    try std.testing.expectEqual(@as(u8, 4), fragments.requested_fragments);
    try std.testing.expectEqual(@as(u8, 4), fragments.generated_fragments);
    try std.testing.expectApproxEqAbs(@as(f32, 100.0), fragments.debris_mass, 0.0001);
    try std.testing.expect(fragments.fragment_energy >= 20.0);
}

test "applyBreakStateAndWake spawns DamageEval fragments" {
    destruction.initDebris();

    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var fragile = entity16.initEntity16();
    fragile.physics.mass = 400;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);
    entity16.setVoxel(&fragile, 4, 0, 0);
    entity16.setVoxel(&fragile, 8, 0, 0);
    entity16.setVoxel(&fragile, 12, 0, 0);

    var entities = [_]entity16.Entity16{fragile};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 20,
        .pos_z = 30,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -20,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expect(applyBreakStateAndWake(&s1024, entities[0..], 0, -20, 0.5));

    const debris_sys = destruction.getDebrisSystem();
    try std.testing.expectEqual(@as(u16, 4), debris_sys.count);
    try std.testing.expect(s1024.instances[0].state == .broken);
    try std.testing.expect(debris_sys.debris[0].active);
    try std.testing.expectApproxEqAbs(@as(f32, 100.0), debris_sys.debris[0].mass, 0.0001);
}

test "computeDamageEvalCrackModel propagates cracks from break energy" {
    var fragile = entity16.initEntity16();
    fragile.physics.mass = 400;
    fragile.physics.material = .fragile;
    fragile.physics.hardness = 10;
    entity16.setVoxel(&fragile, 0, 0, 0);
    entity16.setVoxel(&fragile, 4, 0, 0);
    entity16.setVoxel(&fragile, 8, 0, 0);
    entity16.setVoxel(&fragile, 12, 0, 0);

    const impact = computeDamageEvalImpactFromVelocity(&fragile, -20);
    const hardness = computeDamageEvalHardnessForEntity(&fragile, impact);
    const break_model = computeDamageEvalBreakForEntity(&fragile, impact, hardness);
    const fragments = computeDamageEvalFragmentsForEntity(&fragile, impact, break_model, -20, 1.0);
    const cracks = computeDamageEvalCracksForEntity(&fragile, impact, hardness, break_model, fragments);

    try std.testing.expect(cracks.propagated);
    try std.testing.expect(cracks.crack_count > 0);
    try std.testing.expect(cracks.max_severity > 0.0);
    try std.testing.expectEqual(fragments.generated_fragments, cracks.pattern.fragment_count);
}

test "computeDamageEvalCrackModel suppresses low sub-threshold impact" {
    var solid = entity16.initEntity16();
    solid.physics.mass = 500;
    solid.physics.material = .solid;
    solid.physics.hardness = 200;
    entity16.setVoxel(&solid, 0, 0, 0);

    const impact = computeDamageEvalImpactFromVelocity(&solid, -10);
    const hardness = computeDamageEvalHardnessForEntity(&solid, impact);
    const break_model = computeDamageEvalBreakForEntity(&solid, impact, hardness);
    const fragments = computeDamageEvalFragmentsForEntity(&solid, impact, break_model, -10, 1.0);
    const cracks = computeDamageEvalCracksForEntity(&solid, impact, hardness, break_model, fragments);

    try std.testing.expect(!break_model.did_break);
    try std.testing.expect(!cracks.propagated);
    try std.testing.expectEqual(@as(u8, 0), cracks.crack_count);
}

test "applyContactStabilizationToEquation scales normal row terms" {
    const equation: ConstraintRowEquation = .{
        .effective_mass = 0.5,
        .bias = 2.0,
        .max_impulse = 8.0,
    };
    const stabilized = applyContactStabilizationToEquation(equation, .{
        .bias_scale = 1.12,
        .impulse_scale = 1.08,
        .stabilized = true,
    });

    try std.testing.expectApproxEqAbs(@as(f32, 0.5), stabilized.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.24), stabilized.bias, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 8.64), stabilized.max_impulse, 0.0001);
}

test "solveJointAngularPositionConstraintRowStep applies angular correction by mass ratio" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;

    const step = solveJointAngularPositionConstraintRowStep(
        &inst_a,
        &inst_b,
        .{
            .inv_mass_a = 0.5,
            .inv_mass_b = 0.5,
            .ratio_a = 0.5,
            .ratio_b = 0.5,
        },
        .{
            .axis = .y,
            .angle_a = 0.0,
            .angle_b = 0.0,
            .correction = 1.0,
        },
        .{
            .effective_mass = 1.0,
            .bias = 0.0,
            .max_impulse = 8.0,
        },
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), step.applied_impulse, 0.0001);
    try std.testing.expect(inst_a.rot_yaw != 0);
    try std.testing.expect(inst_b.rot_yaw != 0);
    try std.testing.expect(inst_a.rot_yaw != inst_b.rot_yaw);
}

test "solveJointAngularVelocityConstraintRowStep applies angular velocity bias by mass ratio" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;

    const step = solveJointAngularVelocityConstraintRowStep(
        &inst_a,
        &inst_b,
        .{
            .inv_mass_a = 0.5,
            .inv_mass_b = 0.5,
            .ratio_a = 0.5,
            .ratio_b = 0.5,
        },
        .{
            .axis = .y,
            .signed_bias = 10.0,
        },
        .{
            .effective_mass = 1.0,
            .bias = 0.0,
            .max_impulse = 32.0,
        },
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), step.applied_impulse, 0.0001);
    try std.testing.expectEqual(@as(i16, -5), inst_a.ang_y);
    try std.testing.expectEqual(@as(i16, 5), inst_b.ang_y);
}

test "computeContactNormalConstraintPositionImpulse uses normal spec depth" {
    const normal: ContactNormalConstraintSpec = .{
        .dir_x = 0.0,
        .dir_y = 1.0,
        .dir_z = 0.0,
        .penetration_depth = 1.0,
        .restitution = 0.0,
    };
    const equation: ConstraintRowEquation = .{
        .effective_mass = 0.5,
        .bias = 1.0,
        .max_impulse = 4.0,
    };

    try std.testing.expectApproxEqAbs(
        @as(f32, 1.09),
        computeContactNormalConstraintPositionImpulse(normal, equation),
        0.0001,
    );
}

test "applyContactPositionRowStep corrects dynamic body away from static contact" {
    var inst_dynamic: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_static = inst_dynamic;
    inst_static.entity_id = 1;
    const equation: ConstraintRowEquation = .{
        .effective_mass = 10.0,
        .bias = 0.2,
        .max_impulse = 8.0,
    };

    const step = applyContactPositionRowStep(
        &inst_dynamic,
        &inst_static,
        0.1,
        0.0,
        0.0,
        1.0,
        0.0,
        1.0,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 1.39), step.applied_impulse, 0.0001);
    try std.testing.expectEqual(@as(i32, 10), inst_static.pos_y);
    try std.testing.expect(inst_dynamic.pos_y < 10);
}

test "solveContactNormalPositionConstraintRowStep corrects through normal spec" {
    var inst_dynamic: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_static = inst_dynamic;
    inst_static.entity_id = 1;
    const normal: ContactNormalConstraintSpec = .{
        .dir_x = 0.0,
        .dir_y = 1.0,
        .dir_z = 0.0,
        .penetration_depth = 1.0,
        .restitution = 0.0,
    };
    const equation: ConstraintRowEquation = .{
        .effective_mass = 10.0,
        .bias = 0.2,
        .max_impulse = 8.0,
    };

    const step = solveContactNormalPositionConstraintRowStep(
        &inst_dynamic,
        &inst_static,
        0.1,
        0.0,
        normal,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 1.39), step.applied_impulse, 0.0001);
    try std.testing.expect(inst_dynamic.pos_y < 10);
}

test "computeContactNormalVelocityImpulse suppresses separating and low speed bounce" {
    const policy: ContactVelocitySolvePolicy = .{
        .restitution = .{
            .velocity_threshold = 0.5,
            .max_restitution = 1.0,
        },
    };

    try std.testing.expectEqual(
        @as(f32, 0.0),
        computeContactNormalVelocityImpulse(1.0, 0.1, 0.1, 1.0, policy),
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 2.0),
        computeContactNormalVelocityImpulse(-0.4, 0.1, 0.1, 1.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 15.0),
        computeContactNormalVelocityImpulse(-1.5, 0.1, 0.1, 1.0, policy),
        0.0001,
    );
}

test "computeContactEffectiveRestitution standardizes threshold and clamping" {
    const policy: ContactRestitutionSolvePolicy = .{
        .velocity_threshold = 0.5,
        .max_restitution = 1.0,
    };

    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeContactEffectiveRestitution(0.49, 0.8, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeContactEffectiveRestitution(1.0, -0.5, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0),
        computeContactEffectiveRestitution(1.0, 2.5, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.6),
        computeContactRestitutionImpulseMultiplier(1.0, 0.6, policy),
        0.0001,
    );
}

test "computePairImpulseMagnitude standardizes inverse mass and impulse caps" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 20.0),
        computePairImpulseMagnitude(.{
            .speed = 4.0,
            .inv_mass_a = 0.1,
            .inv_mass_b = 0.1,
        }),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 8.0),
        computePairImpulseMagnitude(.{
            .speed = 4.0,
            .inv_mass_a = 0.1,
            .inv_mass_b = 0.1,
            .policy = .{ .max_impulse = 8.0 },
        }),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 10000.0),
        computePairImpulseMagnitude(.{
            .speed = 1.0,
            .inv_mass_a = 0.0,
            .inv_mass_b = 0.0,
        }),
        0.0001,
    );
    try std.testing.expectEqual(
        @as(f32, 0.0),
        computePairImpulseMagnitude(.{
            .speed = -1.0,
            .inv_mass_a = 0.1,
            .inv_mass_b = 0.1,
        }),
    );
}

test "computeContactFrictionImpulseLimit standardizes friction caps" {
    const policy = contactFrictionSolvePolicy();

    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0),
        computeContactFrictionImpulseLimit(0.5, 2.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 16.0),
        computeContactFrictionImpulseLimit(2.0, 20.0, policy),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeContactFrictionImpulseLimit(-1.0, 20.0, policy),
        0.0001,
    );

    const equation = buildContactFrictionEquation(0.1, 0.1, 3.0, -1.0, 20.0);
    try std.testing.expectApproxEqAbs(policy.min_equation_max_impulse, equation.max_impulse, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.15), equation.bias, 0.0001);
}

test "computeContactStaticFrictionCoefficient raises low-speed friction only" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.625),
        computeContactStaticFrictionCoefficient(0.5, 0.0),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.5),
        computeContactStaticFrictionCoefficient(0.5, 2.0),
        0.0001,
    );

    const low_speed_equation = buildContactFrictionEquation(
        0.1,
        0.1,
        0.25,
        computeContactStaticFrictionCoefficient(0.5, 0.25),
        4.0,
    );
    const dynamic_equation = buildContactFrictionEquation(0.1, 0.1, 0.25, 0.5, 4.0);
    try std.testing.expect(low_speed_equation.max_impulse > dynamic_equation.max_impulse);
}

test "computeContactDynamicFrictionCoefficient decays high-speed friction" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.8),
        computeContactDynamicFrictionCoefficient(0.8, 2.0),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.6),
        computeContactDynamicFrictionCoefficient(0.8, 16.0),
        0.0001,
    );

    const high_speed_equation = buildContactFrictionEquation(
        0.1,
        0.1,
        16.0,
        computeContactEffectiveFrictionCoefficient(0.8, 16.0),
        4.0,
    );
    const base_equation = buildContactFrictionEquation(0.1, 0.1, 16.0, 0.8, 4.0);
    try std.testing.expect(high_speed_equation.max_impulse < base_equation.max_impulse);
}

test "computeContactRollingFrictionCoefficient derives bounded angular resistance" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.064),
        computeContactRollingFrictionCoefficient(0.8, 0.0),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.096),
        computeContactRollingFrictionCoefficient(0.8, 16.0),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeContactRollingFrictionCoefficient(-0.8, 16.0),
        0.0001,
    );
}

test "approximateContactFrictionCone projects tangential impulse to Coulomb limit" {
    const policy = contactFrictionSolvePolicy();
    const cone = approximateContactFrictionCone(
        .{
            .normal_speed_signed = 0.0,
            .tangent_x = 3.0,
            .tangent_y = 4.0,
            .tangent_z = 0.0,
        },
        0.5,
        4.0,
        policy,
    ).?;

    try std.testing.expectApproxEqAbs(@as(f32, 0.6), cone.dir_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.8), cone.dir_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), cone.tangential_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), cone.impulse_limit, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -2.0), projectContactFrictionImpulseToCone(-5.0, cone), 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.25), projectContactFrictionImpulseToCone(1.25, cone), 0.0001);
}

test "approximateContactFrictionEllipse scales impulse by tangent direction" {
    const policy = contactFrictionSolvePolicy();
    const major_cone: ContactFrictionConeApproximation = .{
        .dir_x = 1.0,
        .dir_y = 0.0,
        .dir_z = 0.0,
        .tangential_speed = 10.0,
        .impulse_limit = 2.0,
    };
    const major_ellipse = approximateContactFrictionEllipse(
        major_cone,
        0.0,
        1.0,
        0.0,
        1.0,
        policy,
    );
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), major_ellipse.major_impulse_limit, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), major_ellipse.minor_impulse_limit, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), major_ellipse.impulse_limit, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -2.0), projectContactFrictionImpulseToEllipse(-5.0, major_ellipse), 0.0001);

    const minor_cone: ContactFrictionConeApproximation = .{
        .dir_x = 0.0,
        .dir_y = 0.0,
        .dir_z = 1.0,
        .tangential_speed = 10.0,
        .impulse_limit = 2.0,
    };
    const minor_ellipse = approximateContactFrictionEllipse(
        minor_cone,
        0.0,
        1.0,
        0.0,
        1.0,
        policy,
    );
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), minor_ellipse.impulse_limit, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), projectContactFrictionImpulseToEllipse(5.0, minor_ellipse), 0.0001);
}

test "approximateContactFrictionEllipse applies anisotropic minor axis scale" {
    const policy = contactFrictionSolvePolicy();
    const minor_cone: ContactFrictionConeApproximation = .{
        .dir_x = 0.0,
        .dir_y = 0.0,
        .dir_z = 1.0,
        .tangential_speed = 10.0,
        .impulse_limit = 2.0,
    };
    const ellipse = approximateContactFrictionEllipse(
        minor_cone,
        0.0,
        1.0,
        0.0,
        0.5,
        policy,
    );

    try std.testing.expectApproxEqAbs(@as(f32, 0.75), ellipse.minor_impulse_limit, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), ellipse.impulse_limit, 0.0001);
}

test "computeContactAnisotropicFrictionMinorAxisScale derives surface pair scale" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.0),
        computeContactAnisotropicFrictionMinorAxisScale(.concrete, .rubber),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.4),
        computeContactAnisotropicFrictionMinorAxisScale(.mud_ruts, .carpet),
        0.0001,
    );
}

test "computeContactTangentConstraintImpulse clamps tangent row impulse" {
    const tangent: ContactTangentConstraintSpec = .{
        .dir_x = 0.0,
        .dir_y = 0.0,
        .dir_z = 1.0,
        .speed = 100.0,
        .impulse_limit = 1.5,
    };

    try std.testing.expectApproxEqAbs(
        @as(f32, 1.5),
        computeContactTangentConstraintImpulse(tangent, 0.1, 0.1),
        0.0001,
    );

    var idle_tangent = tangent;
    idle_tangent.speed = 0.0;
    try std.testing.expectApproxEqAbs(
        @as(f32, 0.0),
        computeContactTangentConstraintImpulse(idle_tangent, 0.1, 0.1),
        0.0001,
    );
}

test "solveContactTangentConstraintRowStep applies signed tangent impulse" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;

    const tangent: ContactTangentConstraintSpec = .{
        .dir_x = 0.0,
        .dir_y = 0.0,
        .dir_z = 1.0,
        .speed = 100.0,
        .impulse_limit = 1.5,
    };
    const equation: ConstraintRowEquation = .{
        .effective_mass = 5.0,
        .bias = 0.0,
        .max_impulse = 8.0,
    };

    const step = solveContactTangentConstraintRowStep(
        &inst_a,
        &inst_b,
        0.1,
        0.1,
        tangent,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), step.applied_impulse, 0.0001);
    try std.testing.expect(inst_a.vel_z > 0);
    try std.testing.expect(inst_b.vel_z < 0);
}

test "solveContactFrictionRowStep enforces friction cone normal impulse radius" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;
    inst_b.vel_x = 100;

    const equation: ConstraintRowEquation = .{
        .effective_mass = 5.0,
        .bias = 0.0,
        .max_impulse = 8.0,
    };
    const step = solveContactFrictionRowStep(
        &inst_a,
        &inst_b,
        0.1,
        0.1,
        0.0,
        1.0,
        0.0,
        0.25,
        4.0,
        1.0,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), step.applied_impulse, 0.0001);
    try std.testing.expect(inst_a.vel_x > 0);
    try std.testing.expect(inst_b.vel_x < 100);

    var no_friction_a = inst_a;
    var no_friction_b = inst_b;
    const no_friction_step = solveContactFrictionRowStep(
        &no_friction_a,
        &no_friction_b,
        0.1,
        0.1,
        0.0,
        1.0,
        0.0,
        0.0,
        4.0,
        1.0,
        equation,
    );
    try std.testing.expect(!no_friction_step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), no_friction_step.applied_impulse, 0.0001);
}

test "solveContactFrictionRowStep enforces friction ellipse minor axis radius" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;
    inst_b.vel_z = 100;

    const equation: ConstraintRowEquation = .{
        .effective_mass = 5.0,
        .bias = 0.0,
        .max_impulse = 8.0,
    };
    const step = solveContactFrictionRowStep(
        &inst_a,
        &inst_b,
        0.1,
        0.1,
        0.0,
        1.0,
        0.0,
        0.25,
        4.0,
        1.0,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), step.applied_impulse, 0.0001);
    try std.testing.expect(inst_a.vel_z > 0);
    try std.testing.expect(inst_b.vel_z < 100);
}

test "solveContactFrictionRowStep applies anisotropic friction scale" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;
    inst_b.vel_z = 100;

    const equation: ConstraintRowEquation = .{
        .effective_mass = 5.0,
        .bias = 0.0,
        .max_impulse = 8.0,
    };
    const step = solveContactFrictionRowStep(
        &inst_a,
        &inst_b,
        0.1,
        0.1,
        0.0,
        1.0,
        0.0,
        0.25,
        4.0,
        0.5,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 0.375), step.applied_impulse, 0.0001);
}

test "applyContactVelocityRowStep clamps restitution to velocity solve policy" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;
    inst_b.vel_y = -2;
    const equation: ConstraintRowEquation = .{
        .effective_mass = 5.0,
        .bias = 3.0,
        .max_impulse = 1.0,
    };

    const step = applyContactVelocityRowStep(
        &inst_a,
        &inst_b,
        0.1,
        0.1,
        0.0,
        1.0,
        0.0,
        4.0,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 20.0), step.applied_impulse, 0.0001);
    try std.testing.expect(inst_a.vel_y < 0);
    try std.testing.expect(inst_b.vel_y > -2);
}

test "solveContactNormalVelocityConstraintRowStep applies restitution through normal spec" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;
    inst_b.vel_y = -2;
    const normal: ContactNormalConstraintSpec = .{
        .dir_x = 0.0,
        .dir_y = 1.0,
        .dir_z = 0.0,
        .penetration_depth = 0.0,
        .restitution = 4.0,
    };
    const equation: ConstraintRowEquation = .{
        .effective_mass = 5.0,
        .bias = 3.0,
        .max_impulse = 1.0,
    };

    const step = solveContactNormalVelocityConstraintRowStep(
        &inst_a,
        &inst_b,
        0.1,
        0.1,
        normal,
        equation,
    );

    try std.testing.expect(step.changed);
    try std.testing.expectApproxEqAbs(@as(f32, 20.0), step.applied_impulse, 0.0001);
    try std.testing.expect(inst_a.vel_y < 0);
    try std.testing.expect(inst_b.vel_y > -2);
}

test "material pair response and contact telemetry derive from Contact" {
    const response = material_pairing.getPairedResponse(.rubber, .elastic, .water, .liquid);
    const contact: Contact = .{
        .pair = .{ .a = 0, .b = 1 },
        .entity_id_a = 0,
        .entity_id_b = 1,
        .aabb_a = physics.AABB{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 },
        .aabb_b = physics.AABB{ .min_x = 1, .min_y = 1, .min_z = 1, .max_x = 3, .max_y = 3, .max_z = 3 },
        .manifold = .{
            .point_count = 2,
            .normal_y = 1.0,
            .penetration_depth = 1.5,
            .points = .{collision.ContactPoint{ .x = 1.0, .y = 2.0, .z = 3.0 }} ** collision.CONTACT_MANIFOLD_MAX_POINTS,
        },
        .centroid = .{ .x = 1.0, .y = 2.0, .z = 3.0 },
        .classification = .{
            .pair = .{ .a = 0, .b = 1 },
            .entity_id_a = 0,
            .entity_id_b = 1,
            .material_a = .elastic,
            .material_b = .liquid,
            .surface_a = .rubber,
            .surface_b = .water,
            .medium_a = .solid,
            .medium_b = .liquid,
            .body_type_a = .dynamic,
            .body_type_b = .dynamic,
            .condition_a = .slippery,
            .condition_b = .submerged,
            .response = response,
        },
    };

    const pair_response = makeMaterialPairResponse(&contact);
    const telemetry = makeContactTelemetry(&contact);

    try std.testing.expectEqual(entity16.MaterialType.elastic, pair_response.material_a);
    try std.testing.expectEqual(entity16.MaterialType.liquid, pair_response.material_b);
    try std.testing.expectApproxEqAbs(response.friction, pair_response.response.friction, 0.0001);
    try std.testing.expectApproxEqAbs(response.restitution, telemetry.restitution, 0.0001);
    try std.testing.expectApproxEqAbs(response.damage_modifier, telemetry.damage_modifier, 0.0001);
    try std.testing.expectApproxEqAbs(response.penetration_resistance, telemetry.penetration_resistance, 0.0001);
    try std.testing.expectApproxEqAbs(response.buoyancy, telemetry.buoyancy, 0.0001);
    try std.testing.expect(telemetry.medium_buoyancy_active);
    try std.testing.expect(telemetry.medium_buoyancy_force > 0.0);
    try std.testing.expectApproxEqAbs(telemetry.medium_buoyancy_force, telemetry.medium_buoyancy_magnitude, 0.0001);
    try std.testing.expect(telemetry.medium_buoyancy_displaced_volume > 0.0);
    try std.testing.expect(telemetry.medium_buoyancy_velocity_delta > 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), telemetry.medium_submerged_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), telemetry.medium_buoyancy_center_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.25), telemetry.medium_buoyancy_center_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), telemetry.medium_buoyancy_center_z, 0.0001);
    try std.testing.expect(!telemetry.medium_drag_active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_drag_magnitude, 0.0001);
    try std.testing.expect(telemetry.medium_drag_coefficient > 0.0);
    try std.testing.expect(telemetry.medium_drag_exposure > 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_drag_force, 0.0001);
    try std.testing.expect(!telemetry.medium_vapor_resistance_active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_vapor_resistance_force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_vapor_resistance_dynamic_pressure, 0.0001);
    try std.testing.expect(!telemetry.medium_vacuum_active);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), telemetry.medium_vacuum_pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_vacuum_exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_vacuum_drag_loss, 0.0001);
    try std.testing.expect(telemetry.medium_transition_active);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.solid), telemetry.medium_transition_from);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.liquid), telemetry.medium_transition_to);
    try std.testing.expect(telemetry.medium_transition_progress > 0.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_transition_pressure_delta, 0.0001);
    try std.testing.expect(telemetry.medium_mixing_active);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.solid), telemetry.medium_mixing_primary);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.liquid), telemetry.medium_mixing_secondary);
    try std.testing.expectApproxEqAbs(telemetry.medium_transition_progress, telemetry.medium_mixing_fraction, 0.0001);
    try std.testing.expect(telemetry.medium_mixing_effective_density > 0.0);
    try std.testing.expect(telemetry.medium_mixing_effective_viscosity > 0.0);
    try std.testing.expect(telemetry.medium_mixing_blended_buoyancy > 0.0);
    try std.testing.expect(telemetry.medium_state_active);
    try std.testing.expectEqual(@intFromEnum(MediumPostState.mixing), telemetry.medium_state_value);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.solid), telemetry.medium_state_current);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.liquid), telemetry.medium_state_target);
    try std.testing.expectApproxEqAbs(telemetry.medium_transition_progress, telemetry.medium_state_progress, 0.0001);
    try std.testing.expect(telemetry.medium_state_stability < 1.0);
    try std.testing.expect(telemetry.medium_event_active);
    try std.testing.expectEqual(@intFromEnum(MediumPostEventType.mix), telemetry.medium_event_type);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.solid), telemetry.medium_event_source);
    try std.testing.expectEqual(@intFromEnum(terrain.MediumType.liquid), telemetry.medium_event_target);
    try std.testing.expect(telemetry.medium_event_intensity > 0.0);
    try std.testing.expect(telemetry.medium_event_priority >= @intFromEnum(bus.BusPriority.LOW));
    try std.testing.expect(telemetry.medium_animation_active);
    try std.testing.expect(telemetry.medium_animation_phase > 0.0);
    try std.testing.expectApproxEqAbs(telemetry.medium_mixing_fraction, telemetry.medium_animation_blend, 0.0001);
    try std.testing.expect(telemetry.medium_animation_opacity > 0.0);
    try std.testing.expect(telemetry.medium_animation_ripple > 0.0);
    try std.testing.expect(telemetry.medium_animation_color_shift > 0.0);
    try std.testing.expect(!telemetry.medium_tow_active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_tow_force, 0.0001);
    try std.testing.expect(!telemetry.medium_lift_active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_lift_magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_lift_coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_lift_exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_lift_dynamic_pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_lift_force, 0.0001);
    try std.testing.expect(telemetry.medium_added_mass_active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.55), telemetry.medium_added_mass_coefficient, 0.0001);
    try std.testing.expect(telemetry.medium_added_mass_exposure > 0.0);
    try std.testing.expectApproxEqAbs(telemetry.medium_buoyancy_displaced_volume, telemetry.medium_added_mass_displaced_volume, 0.0001);
    try std.testing.expect(telemetry.medium_added_mass_value > 0.0);
    try std.testing.expect(telemetry.medium_added_mass_effective_mass > mediumPostBuoyancyPolicy().mass_reference);
    try std.testing.expect(telemetry.medium_added_mass_normal_inertia_scale > 1.0);
    try std.testing.expect(telemetry.medium_added_mass_tangential_inertia_scale > 1.0);
    try std.testing.expect(!telemetry.medium_thermal_active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.medium_thermal_conducted_heat, 0.0001);
    try std.testing.expect(telemetry.sink_active);
    try std.testing.expect(telemetry.sink_depth > 0.0);
    try std.testing.expect(telemetry.sink_support_fraction < 1.0);
    try std.testing.expect(telemetry.sink_resistance_active);
    try std.testing.expect(telemetry.sink_resistance_force > 0.0);
    try std.testing.expect(telemetry.sink_recovery_active);
    try std.testing.expect(telemetry.sink_recovery_depth > 0.0);
    try std.testing.expect(telemetry.sink_recovery_fraction < 1.0);
    try std.testing.expect(!telemetry.sink_recovered);
    try std.testing.expectApproxEqAbs(@as(f32, 1.5), telemetry.penetration_depth, 0.0001);
    try std.testing.expectEqual(@as(u8, 2), telemetry.point_count);
    try std.testing.expect(telemetry.fatigue_damage > 0.0);
    try std.testing.expect(telemetry.fatigue_remaining_life < 1.0);
    try std.testing.expect(!telemetry.fatigue_failed);
    try std.testing.expect(telemetry.thermal_energy > 0.0);
    try std.testing.expect(telemetry.thermal_friction_heat >= 0.0);
    try std.testing.expect(telemetry.thermal_friction_heat_fraction >= 0.0);
    try std.testing.expect(telemetry.thermal_conductivity > 0.0);
    try std.testing.expect(telemetry.thermal_temperature_delta >= 0.0);
    try std.testing.expectEqual(response.sound_type, telemetry.sound_type);
    try std.testing.expect(telemetry.sound_volume > 0.0);
    try std.testing.expect(telemetry.sound_pitch > 0.0);
    try std.testing.expect(telemetry.sound_duration > 0.0);
    try std.testing.expectEqual(response.dust_type, telemetry.dust_type);
    try std.testing.expect(telemetry.dust_intensity >= 0.0);
    try std.testing.expect(telemetry.dust_radius >= 0.0);
    try std.testing.expect(telemetry.dust_duration >= 0.0);
    try std.testing.expect(telemetry.deformation_total > 0.0);
    try std.testing.expect(telemetry.deformation_permanent >= 0.0);
    try std.testing.expect(telemetry.deformation_recovery_fraction >= 0.0);
    try std.testing.expectEqual(ContactSeparationState.persisting, telemetry.separation_state);
    try std.testing.expect(!telemetry.separating);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.separation_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.separation_time, 0.0001);
    try std.testing.expect(telemetry.stabilized_contact);
    try std.testing.expect(telemetry.stabilization_bias_scale >= 1.0);
    try std.testing.expect(telemetry.stabilization_impulse_scale >= 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_impact, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_energy, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_amount, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_hardness_threshold, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_hardness_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_hardness_resistance, 0.0001);
    try std.testing.expect(!telemetry.damage_exceeds_hardness);
    try std.testing.expect(!telemetry.damage_should_break);
    try std.testing.expectEqual(@as(u8, 0), telemetry.damage_fragment_count);
    try std.testing.expectEqual(@as(u8, 0), telemetry.damage_generated_fragments);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_fragment_energy, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_debris_mass, 0.0001);
    try std.testing.expectEqual(@as(u8, 0), telemetry.damage_crack_count);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), telemetry.damage_crack_severity, 0.0001);
    try std.testing.expect(!telemetry.damage_cracks_propagated);
}

test "countSpeculativeContactPairs detects predictive-only contact pair" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expectEqual(
        @as(usize, 1),
        countSpeculativeContactPairs(&s1024, entities[0..], &.{.{ .a = 0, .b = 1 }}),
    );
}

test "measureSpeculativeContactSummary reports pair count and highest tier" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const prepared = prepareContactConstraintPair(&s1024.instances[0], &s1024.instances[1], entities[0..]) orelse unreachable;
    try std.testing.expect(prepared.speculative);
    const expected_tier = measureSpeculativeContactBiasProfileDefault(prepared.normal).tier;

    const summary = measureSpeculativeContactSummary(&s1024, entities[0..], &.{.{ .a = 0, .b = 1 }});
    try std.testing.expectEqual(@as(usize, 1), summary.pair_count);
    try std.testing.expectEqual(expected_tier, summary.max_tier);
}

test "measureSpeculativeContactSummary skips broken instances" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .broken,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const summary = measureSpeculativeContactSummary(&s1024, entities[0..], &.{.{ .a = 0, .b = 1 }});
    try std.testing.expectEqual(@as(usize, 0), summary.pair_count);
    try std.testing.expectEqual(SpeculativeContactBiasTier.none, summary.max_tier);
}

test "mergeHigherSpeculativeContactTier preserves max tier ordering" {
    try std.testing.expectEqual(
        SpeculativeContactBiasTier.none,
        mergeHigherSpeculativeContactTier(.none, .none),
    );
    try std.testing.expectEqual(
        SpeculativeContactBiasTier.base,
        mergeHigherSpeculativeContactTier(.none, .base),
    );
    try std.testing.expectEqual(
        SpeculativeContactBiasTier.high,
        mergeHigherSpeculativeContactTier(.mid, .high),
    );
    try std.testing.expectEqual(
        SpeculativeContactBiasTier.high,
        mergeHigherSpeculativeContactTier(.high, .base),
    );
}

test "measureMaxSpeculativeContactTier reports highest speculative profile tier" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const inst_a = &s1024.instances[0];
    const inst_b = &s1024.instances[1];
    const prepared = prepareContactConstraintPair(inst_a, inst_b, entities[0..]) orelse unreachable;
    try std.testing.expect(prepared.speculative);
    const expected_tier = measureSpeculativeContactBiasProfileDefault(prepared.normal).tier;

    const tier = measureMaxSpeculativeContactTier(&s1024, entities[0..], &.{.{ .a = 0, .b = 1 }});
    try std.testing.expectEqual(expected_tier, tier);
}

test "boostIterationBudgetForSpeculativeContacts scales by speculative tier and clamps" {
    try std.testing.expectEqual(@as(u8, 4), boostIterationBudgetForSpeculativeContacts(4, 0, .high));
    try std.testing.expectEqual(@as(u8, 3), boostIterationBudgetForSpeculativeContacts(2, 1, .base));
    try std.testing.expectEqual(@as(u8, 3), boostIterationBudgetForSpeculativeContacts(2, 1, .mid));
    try std.testing.expectEqual(@as(u8, 4), boostIterationBudgetForSpeculativeContacts(2, 1, .high));
    try std.testing.expectEqual(@as(u8, 4), boostIterationBudgetForSpeculativeContacts(2, 4, .base));
    try std.testing.expectEqual(@as(u8, 5), boostIterationBudgetForSpeculativeContacts(2, 4, .high));
    try std.testing.expectEqual(@as(u8, 6), boostIterationBudgetForSpeculativeContacts(6, 1, .high));
    try std.testing.expectEqual(@as(u8, 6), boostIterationBudgetForSpeculativeContacts(5, 8, .high));
    try std.testing.expectEqual(@as(u8, 4), boostIterationBudgetForSpeculativeContacts(4, 0, .none));
}

test "computeSpeculativeIterationExtraPasses matches tier and density policy" {
    const policy = speculativeContactIterationBudgetPolicy();
    try std.testing.expectEqual(@as(u8, 0), computeSpeculativeIterationExtraPasses(policy, 0, .high));
    try std.testing.expectEqual(@as(u8, 1), computeSpeculativeIterationExtraPasses(policy, 1, .base));
    try std.testing.expectEqual(@as(u8, 1), computeSpeculativeIterationExtraPasses(policy, 2, .mid));
    try std.testing.expectEqual(@as(u8, 2), computeSpeculativeIterationExtraPasses(policy, 1, .high));
    try std.testing.expectEqual(@as(u8, 2), computeSpeculativeIterationExtraPasses(policy, 4, .base));
    try std.testing.expectEqual(@as(u8, 3), computeSpeculativeIterationExtraPasses(policy, 4, .high));
}

test "computeSpeculativeIterationExtraPasses clamps extreme policy inputs" {
    const policy: SpeculativeContactIterationBudgetPolicy = .{
        .low_tier_extra_passes = 250,
        .high_tier_extra_passes = 255,
        .dense_pair_threshold = 1,
        .dense_pair_extra_passes = 255,
    };

    try std.testing.expectEqual(@as(u8, 0), computeSpeculativeIterationExtraPasses(policy, 0, .high));
    try std.testing.expectEqual(ITERATION_BUDGET_CAP, computeSpeculativeIterationExtraPasses(policy, 1, .base));
    try std.testing.expectEqual(ITERATION_BUDGET_CAP, computeSpeculativeIterationExtraPasses(policy, 1, .high));
}

test "boostIterationBudgetForSpeculativePairs uses summary-driven tier-aware boost" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const pairs = [_]BroadPhasePair{.{ .a = 0, .b = 1 }};
    const summary = measureSpeculativeContactSummary(&s1024, entities[0..], pairs[0..]);
    const direct = boostIterationBudgetForSpeculativeContacts(2, summary.pair_count, summary.max_tier);
    const via_helper = boostIterationBudgetForSpeculativePairs(&s1024, entities[0..], pairs[0..], 2);
    try std.testing.expectEqual(direct, via_helper);
}

test "boostIterationBudgetForSpeculativePairs returns clamped base budget when pair list is empty" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{entity16.initEntity16()};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expectEqual(
        @as(u8, 4),
        boostIterationBudgetForSpeculativePairs(&s1024, entities[0..], &.{}, 4),
    );
    try std.testing.expectEqual(
        @as(u8, 6),
        boostIterationBudgetForSpeculativePairs(&s1024, entities[0..], &.{}, 8),
    );
}

test "boostIterationBudgetForSpeculativePairs short-circuits when base budget already capped" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var entities = [_]entity16.Entity16{entity16.initEntity16()};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    try std.testing.expectEqual(
        ITERATION_BUDGET_CAP,
        boostIterationBudgetForSpeculativePairs(&s1024, entities[0..], &.{.{ .a = 0, .b = 0 }}, ITERATION_BUDGET_CAP),
    );
}

test "computeEnvironmentIterationBudget increases for higher environment stress" {
    try std.testing.expectEqual(@as(u8, 2), computeEnvironmentIterationBudget(1, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeEnvironmentIterationBudget(1, 1.5));
    try std.testing.expectEqual(@as(u8, 4), computeEnvironmentIterationBudget(1, 5.0));
    try std.testing.expectEqual(ITERATION_BUDGET_CAP, computeEnvironmentIterationBudget(16, 5.0));
}

test "computeConstraintBlockIterationBudget increases with active stressed subsystems" {
    try std.testing.expectEqual(@as(u8, 2), computeConstraintBlockIterationBudget(0.0, 0.0, 0.0));
    try std.testing.expectEqual(@as(u8, 2), computeConstraintBlockIterationBudget(0.5, 0.0, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeConstraintBlockIterationBudget(0.5, 0.5, 0.0));
    try std.testing.expectEqual(@as(u8, 6), computeConstraintBlockIterationBudget(5.0, 5.0, 5.0));
}

test "computeStressSettleThreshold maps stress bands consistently" {
    try std.testing.expectEqual(STRESS_SETTLE_BASE_VALUE, computeStressSettleThreshold(0.0));
    try std.testing.expectEqual(STRESS_SETTLE_BASE_VALUE, computeStressSettleThreshold(0.999));
    try std.testing.expectEqual(STRESS_SETTLE_MID_VALUE, computeStressSettleThreshold(STRESS_SETTLE_MID_THRESHOLD));
    try std.testing.expectEqual(STRESS_SETTLE_MID_VALUE, computeStressSettleThreshold(3.999));
    try std.testing.expectEqual(STRESS_SETTLE_HIGH_VALUE, computeStressSettleThreshold(STRESS_SETTLE_HIGH_THRESHOLD));
}

test "subsystem settle thresholds share stress band policy" {
    const stresses = [_]f32{ 0.0, STRESS_SETTLE_MID_THRESHOLD, STRESS_SETTLE_HIGH_THRESHOLD };

    for (stresses) |stress| {
        const expected = computeStressSettleThreshold(stress);
        try std.testing.expectEqual(expected, computeContactSettleThreshold(stress));
        try std.testing.expectEqual(expected, computeEnvironmentSettleThreshold(stress));
        try std.testing.expectEqual(expected, computeConstraintBlockSettleThreshold(stress, 0.0, 0.0));
        try std.testing.expectEqual(expected, computeConstraintBlockSettleThreshold(0.0, stress, 0.0));
        try std.testing.expectEqual(expected, computeConstraintBlockSettleThreshold(0.0, 0.0, stress));
    }
}

test "buildContactPriorityOrder uses stable tiebreak when contact stress matches" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var wide = entity16.initEntity16();
    wide.physics.mass = 10;
    wide.physics.material = .solid;
    entity16.setVoxel(&wide, 0, 0, 0);
    entity16.setVoxel(&wide, 1, 0, 0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ wide, wide, voxel, voxel };
    s1024.instance_count = 4;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 11,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 20,
        .pos_y = 20,
        .pos_z = 20,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[3] = .{
        .entity_id = 3,
        .pos_x = 20,
        .pos_y = 20,
        .pos_z = 20,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const pairs = [_]BroadPhasePair{
        .{ .a = 2, .b = 3 },
        .{ .a = 0, .b = 1 },
    };
    var order: [2]usize = undefined;

    const count = buildContactPriorityOrder(
        &s1024,
        entities[0..],
        pairs[0..],
        0.0,
        false,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 0), order[0]);
    try std.testing.expectEqual(@as(usize, 1), order[1]);
}

test "buildContactPriorityOrder prioritizes predictive overlap over weaker current overlap" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var wide = entity16.initEntity16();
    wide.physics.mass = 10;
    wide.physics.material = .solid;
    entity16.setVoxel(&wide, 0, 0, 0);
    entity16.setVoxel(&wide, 1, 0, 0);

    var entities = [_]entity16.Entity16{ wide, wide, wide, wide };
    s1024.instance_count = 4;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 11,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 20,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[3] = .{
        .entity_id = 3,
        .pos_x = 21,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = -24,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const pairs = [_]BroadPhasePair{
        .{ .a = 0, .b = 1 },
        .{ .a = 2, .b = 3 },
    };
    var order: [2]usize = undefined;

    const count = buildContactPriorityOrder(
        &s1024,
        entities[0..],
        pairs[0..],
        0.0,
        false,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 1), order[0]);
    try std.testing.expectEqual(@as(usize, 0), order[1]);
}

test "buildContactPriorityOrder includes non-overlapping predictive contact candidate" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel, voxel, voxel };
    s1024.instance_count = 4;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 30,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[3] = .{
        .entity_id = 3,
        .pos_x = 40,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const pairs = [_]BroadPhasePair{
        .{ .a = 1, .b = 2 },
        .{ .a = 0, .b = 1 },
    };
    var order: [2]usize = undefined;

    const count = buildContactPriorityOrder(
        &s1024,
        entities[0..],
        pairs[0..],
        0.0,
        false,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(usize, 1), order[0]);
}

test "prepareContactConstraintPair returns speculative normal-only payload for predictive candidate" {
    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    const inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const inst_b: scene32.Instance = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const prepared = prepareContactConstraintPair(&inst_a, &inst_b, entities[0..]) orelse return error.TestUnexpectedResult;
    const baseline_plan = buildContactNormalRowPlan(
        instanceInverseMass(&inst_a, entities[0..]),
        instanceInverseMass(&inst_b, entities[0..]),
        prepared.normal.depth,
        0.0,
        prepared.normal,
    );

    try std.testing.expect(prepared.speculative);
    try std.testing.expect(prepared.normal.depth > 0.0);
    try std.testing.expectEqual(@as(f32, 0.0), prepared.restitution);
    try std.testing.expectEqual(@as(f32, 0.0), prepared.friction);
    try std.testing.expectEqual(ContactSeparationState.persisting, prepared.separation.state);
    try std.testing.expect(!prepared.separation.separating);
    try std.testing.expect(!prepared.stabilization.stabilized);
    try std.testing.expect(!prepared.medium_buoyancy.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.displaced_volume, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.center_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.center_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_buoyancy.center_z, 0.0001);
    try std.testing.expect(!prepared.medium_drag.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_drag.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_drag.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_drag.exposure, 0.0001);
    try std.testing.expect(!prepared.medium_vapor_resistance.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_vapor_resistance.force, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_vapor_resistance.dynamic_pressure, 0.0001);
    try std.testing.expect(!prepared.medium_vacuum.active);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.medium_vacuum.pressure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_vacuum.exposure, 0.0001);
    try std.testing.expect(!prepared.medium_transition.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_transition.progress, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_transition.resistance, 0.0001);
    try std.testing.expect(!prepared.medium_mixing.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_mixing.mix_fraction, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_mixing.effective_density, 0.0001);
    try std.testing.expect(!prepared.medium_state.active);
    try std.testing.expectEqual(MediumPostState.stable, prepared.medium_state.state);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.medium_state.stability, 0.0001);
    try std.testing.expect(!prepared.medium_event.active);
    try std.testing.expectEqual(MediumPostEventType.none, prepared.medium_event.event_type);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_event.intensity, 0.0001);
    try std.testing.expect(!prepared.medium_animation.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_animation.phase, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_animation.opacity, 0.0001);
    try std.testing.expect(!prepared.medium_tow.active);
    try std.testing.expect(!prepared.medium_lift.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.coefficient, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.exposure, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_lift.dynamic_pressure, 0.0001);
    try std.testing.expect(!prepared.medium_added_mass.active);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_added_mass.added_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.medium_added_mass.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.medium_added_mass.normal_inertia_scale, 0.0001);
    try std.testing.expect(!prepared.medium_thermal.active);
    try std.testing.expect(!prepared.sink_depth.active);
    try std.testing.expect(!prepared.sink_resistance.active);
    try std.testing.expect(!prepared.sink_recovery.active);
    try std.testing.expect(prepared.sink_recovery.recovered);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.stabilization.bias_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.stabilization.impulse_scale, 0.0001);
    try std.testing.expectEqual(@as(u16, 0), prepared.damage_impact.legacy_impact);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.damage_hardness.impact_threshold, 0.0001);
    try std.testing.expect(!prepared.damage_hardness.exceeds_hardness);
    try std.testing.expect(!prepared.damage_break.did_break);
    try std.testing.expectEqual(@as(u8, 0), prepared.damage_fragments.generated_fragments);
    try std.testing.expectEqual(@as(u8, 0), prepared.damage_cracks.crack_count);
    try std.testing.expectEqual(false, prepared.has_tangent);
    try std.testing.expect(prepared.normal_equation.bias > baseline_plan.equation.bias);
    try std.testing.expectEqual(@as(f32, 0.0), prepared.friction_equation.bias);
    try std.testing.expectEqual(@as(f32, 0.0), prepared.friction_equation.max_impulse);
}

test "buildPredictedAABBSeparationDirection uses minimum predicted overlap axis" {
    const aabb_a: physics.AABB = .{ .min_x = 0, .min_y = 0, .min_z = 0, .max_x = 2, .max_y = 2, .max_z = 2 };
    const aabb_b: physics.AABB = .{ .min_x = 2, .min_y = 1, .min_z = 0, .max_x = 4, .max_y = 3, .max_z = 2 };
    const origin_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const origin_b: scene32.Instance = .{
        .entity_id = 1,
        .pos_x = 2,
        .pos_y = 1,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const predicted_a: prediction.LinearState = .{ .pos_x = 0.0, .pos_y = 0.0, .pos_z = 0.0, .vel_x = 0.0, .vel_y = 0.0, .vel_z = 0.0 };
    const predicted_b: prediction.LinearState = .{ .pos_x = 0.0, .pos_y = 0.25, .pos_z = 0.0, .vel_x = 0.0, .vel_y = 0.0, .vel_z = 0.0 };

    const direction = buildPredictedAABBSeparationDirection(
        aabb_a,
        &origin_a,
        predicted_a,
        aabb_b,
        &origin_b,
        predicted_b,
        0.5,
    );

    try std.testing.expectEqual(@as(f32, 0.0), direction.dir_x);
    try std.testing.expectEqual(@as(f32, -1.0), direction.dir_y);
    try std.testing.expectEqual(@as(f32, 0.0), direction.dir_z);
}

test "buildEnvironmentPriorityOrder uses stable tiebreak when environment stress matches" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 10,
        .lz = 10,
    }), true);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 20,
        .ly = 20,
        .lz = 20,
    }), true);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 20,
        .pos_y = 20,
        .pos_z = 20,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var order: [scene32.MAX_INSTANCES]u8 = undefined;
    const count = buildEnvironmentPriorityOrder(
        &s1024,
        entities[0..],
        0.0,
        false,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(u8, 0), order[0]);
    try std.testing.expectEqual(@as(u8, 1), order[1]);
}

test "buildConstraintSubsystemOrder prioritizes highest stress subsystem first" {
    var order: [3]ConstraintSubsystem = undefined;

    const count = buildConstraintSubsystemOrder(
        0.5,
        3.0,
        1.0,
        false,
        &order,
    );

    try std.testing.expectEqual(@as(usize, 3), count);
    try std.testing.expectEqual(ConstraintSubsystem.contact, order[0]);
    try std.testing.expectEqual(ConstraintSubsystem.environment, order[1]);
    try std.testing.expectEqual(ConstraintSubsystem.joint, order[2]);
}

test "buildIslandRowDispatchOrder prioritizes stressed subsystems for island row build" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 10,
        .lz = 10,
    }), true);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel, voxel };
    s1024.instance_count = 3;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 30,
        .pos_y = 30,
        .pos_z = 30,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = -1,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const pairs = [_]BroadPhasePair{
        .{ .a = 0, .b = 1 },
    };
    const instance_indices = [_]u8{ 0, 1, 2 };
    var order: [island_row_dispatch_entries.len]IslandRowDispatchOrderEntry = undefined;
    var count_storage: usize = 0;
    var ctx = IslandRowBuildContext{
        .s1024 = &s1024,
        .entities = entities[0..],
        .joints = &.{},
        .joint_indices = &.{},
        .pair_subset = pairs[0..],
        .instance_indices = instance_indices[0..],
        .row_states = &.{},
        .state_count = 0,
        .out_rows = &.{},
        .count = &count_storage,
    };

    const count = buildIslandRowDispatchOrder(&ctx, &order);

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(ConstraintSubsystem.contact, order[0].entry.subsystem);
    try std.testing.expectEqual(ConstraintSubsystem.environment, order[1].entry.subsystem);
    try std.testing.expect(order[0].stress >= order[1].stress);
}

test "sortConstraintIslandsByStress prioritizes higher stress island first" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 10,
        .lz = 10,
    }), true);

    var voxel = entity16.initEntity16();
    voxel.physics.mass = 10;
    voxel.physics.material = .solid;
    entity16.setVoxel(&voxel, 0, 0, 0);

    var entities = [_]entity16.Entity16{ voxel, voxel, voxel };
    s1024.instance_count = 3;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 40,
        .pos_y = 40,
        .pos_z = 40,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[2] = .{
        .entity_id = 2,
        .pos_x = 40,
        .pos_y = 40,
        .pos_z = 40,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const broadphase_pairs = [_]BroadPhasePair{
        .{ .a = 1, .b = 2 },
    };
    var joints = [_]joint.Joint{};
    const island_storage = [_][scene32.MAX_INSTANCES]u8{
        blk: {
            var row: [scene32.MAX_INSTANCES]u8 = [_]u8{0} ** scene32.MAX_INSTANCES;
            row[0] = 0;
            break :blk row;
        },
        blk: {
            var row: [scene32.MAX_INSTANCES]u8 = [_]u8{0} ** scene32.MAX_INSTANCES;
            row[0] = 1;
            row[1] = 2;
            break :blk row;
        },
    };
    const island_lengths = [_]usize{ 1, 2 };
    var island_order: [2]usize = undefined;

    sortConstraintIslandsByStress(
        &s1024,
        entities[0..],
        joints[0..],
        broadphase_pairs[0..],
        island_storage[0..],
        island_lengths[0..],
        2,
        island_order[0..],
    );

    try std.testing.expectEqual(@as(usize, 0), island_order[0]);
    try std.testing.expectEqual(@as(usize, 1), island_order[1]);
}

test "prepareJointDriveChannel distinguishes inactive from stalled_prepare" {
    var inst_a = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = scene32.Instance{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    const inactive_joint = joint.Joint{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
    };
    try std.testing.expectEqual(
        JointPreparedOutcome(JointPreparedDriveChannel).inactive,
        prepareJointDriveChannel(&inst_a, &inst_b, &inactive_joint),
    );

    const stalled_joint = joint.Joint{
        .joint_type = .slider,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .axis_x = 0,
        .axis_y = 0,
        .axis_z = 0,
        .limit_min = -10.0,
        .limit_max = 10.0,
        .motor_enabled = true,
        .motor_target = 5.0,
        .motor_speed = 6.0,
        .motor_max_torque = 10.0,
    };
    try std.testing.expectEqual(
        JointPreparedOutcome(JointPreparedDriveChannel).stalled_prepare,
        prepareJointDriveChannel(&inst_a, &inst_b, &stalled_joint),
    );
}

test "fixed joint limit row locks relative angular drift" {
    var dynamic = entity16.initEntity16();
    dynamic.physics.mass = 10;
    var entities = [_]entity16.Entity16{ dynamic, dynamic };

    var instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(0.75),
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(-0.75),
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 8,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var joints = [_]joint.Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .axis_y = 1,
        .axis_z = 0,
    }};

    const before_angle = @abs(getKernelJointAngle(&instances[1], &joints[0]) - getKernelJointAngle(&instances[0], &joints[0]));
    const result = solveJointRuntimeRow(
        .joint_limit,
        instances[0..],
        joints[0..],
        0,
        entities[0..],
        joint.measureJointSolvePriorityForIndex(instances[0..], joints[0..], 0, entities[0..]),
        buildJointLimitEquation(instances[0..], joints[0..], 0, entities[0..]),
        null,
    );
    const after_angle = @abs(getKernelJointAngle(&instances[1], &joints[0]) - getKernelJointAngle(&instances[0], &joints[0]));

    try std.testing.expect(result.changed);
    try std.testing.expect(after_angle < before_angle);
    try std.testing.expect(@abs(instances[1].ang_y - instances[0].ang_y) < 8);
}

test "hinge joint limit row locks swing axis drift only" {
    var dynamic = entity16.initEntity16();
    dynamic.physics.mass = 10;
    var entities = [_]entity16.Entity16{ dynamic, dynamic };

    var instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(-0.25),
            .rot_pitch = 0,
            .rot_roll = jointRadiansToRotationByte(-0.75),
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(0.25),
            .rot_pitch = 0,
            .rot_roll = jointRadiansToRotationByte(0.75),
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 8,
            .ang_y = 8,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var joints = [_]joint.Joint{.{
        .joint_type = .hinge,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .axis_y = 1,
        .axis_z = 0,
        .limit_min = -1.0,
        .limit_max = 1.0,
    }};

    const before_swing = @abs(jointRotationByteToRadians(instances[1].rot_roll) - jointRotationByteToRadians(instances[0].rot_roll));
    const before_free = @abs(getKernelJointAngle(&instances[1], &joints[0]) - getKernelJointAngle(&instances[0], &joints[0]));
    const result = solveJointRuntimeRow(
        .joint_limit,
        instances[0..],
        joints[0..],
        0,
        entities[0..],
        joint.measureJointSolvePriorityForIndex(instances[0..], joints[0..], 0, entities[0..]),
        buildJointLimitEquation(instances[0..], joints[0..], 0, entities[0..]),
        null,
    );
    const after_swing = @abs(jointRotationByteToRadians(instances[1].rot_roll) - jointRotationByteToRadians(instances[0].rot_roll));
    const after_free = @abs(getKernelJointAngle(&instances[1], &joints[0]) - getKernelJointAngle(&instances[0], &joints[0]));

    try std.testing.expect(result.changed);
    try std.testing.expect(after_swing < before_swing);
    try std.testing.expectApproxEqAbs(before_free, after_free, 0.05);
    try std.testing.expect(@abs(instances[1].ang_x - instances[0].ang_x) < 8);
    try std.testing.expectEqual(@as(i8, 8), instances[1].ang_y - instances[0].ang_y);
}

test "slider joint limit row locks angular drift without clamping rail travel" {
    var dynamic = entity16.initEntity16();
    dynamic.physics.mass = 10;
    var entities = [_]entity16.Entity16{ dynamic, dynamic };

    var instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(-0.75),
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 12,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(0.75),
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 8,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var joints = [_]joint.Joint{.{
        .joint_type = .slider,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .axis_x = 1,
        .axis_y = 0,
        .axis_z = 0,
        .limit_min = -100.0,
        .limit_max = 100.0,
    }};

    const before_angle = @abs(jointRotationByteToRadians(instances[1].rot_yaw) - jointRotationByteToRadians(instances[0].rot_yaw));
    const before_x = instances[1].pos_x - instances[0].pos_x;
    const result = solveJointRuntimeRow(
        .joint_limit,
        instances[0..],
        joints[0..],
        0,
        entities[0..],
        joint.measureJointSolvePriorityForIndex(instances[0..], joints[0..], 0, entities[0..]),
        buildJointLimitEquation(instances[0..], joints[0..], 0, entities[0..]),
        null,
    );
    const after_angle = @abs(jointRotationByteToRadians(instances[1].rot_yaw) - jointRotationByteToRadians(instances[0].rot_yaw));
    const after_x = instances[1].pos_x - instances[0].pos_x;

    try std.testing.expect(result.changed);
    try std.testing.expect(after_angle < before_angle);
    try std.testing.expectEqual(before_x, after_x);
    try std.testing.expect(@abs(instances[1].ang_y - instances[0].ang_y) < 8);
}

test "ball socket joint anchor row converges offset anchors and preserves angular freedom" {
    var dynamic = entity16.initEntity16();
    dynamic.physics.mass = 10;
    var entities = [_]entity16.Entity16{ dynamic, dynamic };

    var instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(-0.5),
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 10,
            .pos_y = 0,
            .pos_z = 0,
            .rot_yaw = jointRadiansToRotationByte(0.75),
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 8,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var joints = [_]joint.Joint{.{
        .joint_type = .ball_socket,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 5,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = -5,
        .anchor_b_y = 6,
        .anchor_b_z = 0,
        .damping = 100.0,
    }};

    const before_delta = measureKernelJointAnchorDelta(&instances[0], &instances[1], &joints[0]);
    const before_dist = @sqrt(before_delta.x * before_delta.x + before_delta.y * before_delta.y + before_delta.z * before_delta.z);
    const before_angle = jointRotationByteToRadians(instances[1].rot_yaw) - jointRotationByteToRadians(instances[0].rot_yaw);
    const result = solveJointRuntimeRow(
        .joint_anchor,
        instances[0..],
        joints[0..],
        0,
        entities[0..],
        joint.measureJointSolvePriorityForIndex(instances[0..], joints[0..], 0, entities[0..]),
        buildJointAnchorEquation(instances[0..], joints[0..], 0, entities[0..]),
        null,
    );
    const after_delta = measureKernelJointAnchorDelta(&instances[0], &instances[1], &joints[0]);
    const after_dist = @sqrt(after_delta.x * after_delta.x + after_delta.y * after_delta.y + after_delta.z * after_delta.z);
    const after_angle = jointRotationByteToRadians(instances[1].rot_yaw) - jointRotationByteToRadians(instances[0].rot_yaw);

    try std.testing.expect(result.changed);
    try std.testing.expect(after_dist < before_dist);
    try std.testing.expectApproxEqAbs(before_angle, after_angle, 0.05);
    try std.testing.expectEqual(@as(i8, 8), instances[1].ang_y - instances[0].ang_y);
}

test "pulley joint limit row transfers weighted axis motion" {
    var dynamic = entity16.initEntity16();
    dynamic.physics.mass = 10;
    var entities = [_]entity16.Entity16{ dynamic, dynamic };

    var instances = [_]scene32.Instance{
        .{
            .entity_id = 0,
            .pos_x = 0,
            .pos_y = 6,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
        .{
            .entity_id = 1,
            .pos_x = 0,
            .pos_y = 10,
            .pos_z = 0,
            .rot_yaw = 0,
            .rot_pitch = 0,
            .rot_roll = 0,
            .state = .moving,
            .sleep_tick = 0,
            .vel_x = 0,
            .vel_y = 0,
            .vel_z = 0,
            .ang_x = 0,
            .ang_y = 0,
            .ang_z = 0,
            ._reserved = .{0} ** 2,
        },
    };
    var joints = [_]joint.Joint{.{
        .joint_type = .pulley,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .axis_x = 0,
        .axis_y = 1,
        .axis_z = 0,
        .limit_min = 1.0,
        .limit_max = 10.0,
        .damping = 100.0,
    }};

    const before = prepareJointPulleyConstraint(&instances[0], &instances[1], &joints[0]).?;
    const before_b_y = instances[1].pos_y;
    const result = solveJointRuntimeRow(
        .joint_limit,
        instances[0..],
        joints[0..],
        0,
        entities[0..],
        joint.measureJointSolvePriorityForIndex(instances[0..], joints[0..], 0, entities[0..]),
        buildJointLimitEquation(instances[0..], joints[0..], 0, entities[0..]),
        null,
    );
    const after = prepareJointPulleyConstraint(&instances[0], &instances[1], &joints[0]).?;

    try std.testing.expect(result.changed);
    try std.testing.expect(@abs(after.pulley_error) < @abs(before.pulley_error));
    try std.testing.expect(instances[1].pos_y < before_b_y);
}

test "prepareJointSpringLinearConstraint clamps high stiffness correction" {
    var inst_a = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = inst_a;
    inst_b.entity_id = 1;
    inst_b.pos_x = 20;

    const spring = joint.Joint{
        .joint_type = .spring,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .limit_max = 4.0,
        .stiffness = 10000.0,
    };

    const prepared = prepareJointSpringLinearConstraint(&inst_a, &inst_b, &spring).?;

    try std.testing.expectApproxEqAbs(@as(f32, 16.0), prepared.magnitude, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.dir_x, 0.0001);
}

test "prepareJointSpringLinearConstraint uses configured axis for coincident anchors" {
    const inst = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const spring = joint.Joint{
        .joint_type = .spring,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .axis_x = 0,
        .axis_y = 1,
        .axis_z = 0,
        .limit_max = 6.0,
        .stiffness = 10000.0,
    };

    const prepared = prepareJointSpringLinearConstraint(&inst, &inst, &spring).?;

    try std.testing.expectApproxEqAbs(@as(f32, 0.0), prepared.dir_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), prepared.dir_y, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, -6.0), prepared.magnitude, 0.0001);
}

test "prepareResolvedJointSolveResult maps stalled_prepare to stalled outcome" {
    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var inst_a = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = scene32.Instance{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var stalled_joint = joint.Joint{
        .joint_type = .slider,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .axis_x = 0,
        .axis_y = 0,
        .axis_z = 0,
        .limit_min = -10.0,
        .limit_max = 10.0,
        .motor_enabled = true,
        .motor_target = 5.0,
        .motor_speed = 6.0,
        .motor_max_torque = 10.0,
    };

    const mass_data = computeJointMassData(&inst_a, &inst_b, entities[0..]).?;
    const descriptor = jointRuntimeRowDescriptor(.joint_drive).?;
    const ctx = JointSolveRuntimeContext{
        .kind = .joint_drive,
        .descriptor = descriptor,
        .joint_def = &stalled_joint,
        .inst_a = &inst_a,
        .inst_b = &inst_b,
        .before = 1.0,
        .mass_data = mass_data,
    };

    const outcome = prepareResolvedJointSolveResult(
        JointPreparedChannel,
        ctx,
        &JointRowExecutionState.init(0.0),
        null,
        .{
            .effective_mass = 1.0,
            .bias = 0.0,
            .max_impulse = 1.0,
        },
        descriptor.prepare_channel(ctx),
        buildReadyJointSolveResultFromPreparedChannel,
    );

    try std.testing.expectEqual(JointSolveResultOutcome.stalled, outcome);
}

test "jointPreparedUnavailableOutcome maps stalled_prepare policy to solve outcome" {
    const outcome = jointPreparedUnavailableOutcome(.finalize_exec_state);

    try std.testing.expectEqual(JointSolveResultOutcome.finalize_exec_state, outcome);
}

test "resolveJointPreparedChannel maps inactive and stalled via descriptor policy" {
    const descriptor = JointRuntimeRowDescriptor{
        .kind = .joint_limit,
        .policy = .{
            .residual_scale = 1.0,
            .warm_impulse_scale = 0.0,
            .inactive_outcome = .finalize_no_change,
            .stalled_prepare_outcome = .finalize_exec_state,
            .anchor_postprocess_policy = .none,
            .drive_postprocess_policy = .none,
        },
        .row_enabled = jointHasLimitRow,
        .prepare_channel = prepareJointDescriptorLimitChannel,
    };
    var joint_def = joint.Joint{
        .joint_type = .hinge,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
    };
    var inst_a = scene32.Instance{
        .entity_id = 0,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b = scene32.Instance{
        .entity_id = 1,
        .pos_x = 0,
        .pos_y = 0,
        .pos_z = 0,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    const ctx = JointSolveRuntimeContext{
        .kind = .joint_limit,
        .descriptor = &descriptor,
        .joint_def = &joint_def,
        .inst_a = &inst_a,
        .inst_b = &inst_b,
        .before = 1.0,
        .mass_data = .{
            .inv_mass_a = 0.1,
            .inv_mass_b = 0.1,
            .ratio_a = 0.5,
            .ratio_b = 0.5,
        },
    };

    const inactive = resolveJointPreparedChannel(
        JointPreparedLimitChannel,
        ctx,
        JointPreparedOutcome(JointPreparedLimitChannel).inactive,
    );
    try std.testing.expectEqual(
        JointPreparedResolution(JointPreparedLimitChannel){ .outcome = .no_change },
        inactive,
    );

    const stalled = resolveJointPreparedChannel(
        JointPreparedLimitChannel,
        ctx,
        JointPreparedOutcome(JointPreparedLimitChannel).stalled_prepare,
    );
    try std.testing.expectEqual(
        JointPreparedResolution(JointPreparedLimitChannel){ .outcome = .finalize_exec_state },
        stalled,
    );
}

test "makeConstraintRowBuildSpecFromPlan preserves predictive residual hint" {
    const spec = makeConstraintRowBuildSpecFromPlan(
        .contact_normal,
        3,
        .{
            .residual = 2.5,
            .metadata = .{
                .predictive_residual_hint = 4.0,
            },
            .equation = .{
                .effective_mass = 1.0,
                .bias = 0.5,
                .max_impulse = 3.0,
            },
        },
    );

    try std.testing.expectEqual(@as(usize, 3), spec.index);
    try std.testing.expectEqual(@as(f32, 2.5), spec.residual);
    try std.testing.expectEqual(@as(f32, 4.0), spec.metadata.predictive_residual_hint);
    try std.testing.expectEqual(@as(f32, 0.5), spec.equation.bias);
}

test "measureConstraintRowCachedPriority respects predictive metadata floor" {
    const priority = measureConstraintRowCachedPriority(
        1.0,
        .{
            .predictive_residual_hint = 2.5,
        },
        null,
    );

    try std.testing.expectEqual(@as(f32, 2.5), priority);
}

test "measureConstraintRowCachedPriority lifts speculative contact priority floor" {
    const policy = speculativeContactPriorityFloorPolicy();
    const priority = measureConstraintRowCachedPriority(
        2.0,
        .{
            .speculative_contact = true,
            .speculative_bias_tier = .base,
        },
        null,
    );

    try std.testing.expectApproxEqAbs(@as(f32, 2.0) * policy.base_scale, priority, 0.0001);
}

test "measureConstraintRowCachedPriority applies stronger speculative floor for higher tier" {
    const policy = speculativeContactPriorityFloorPolicy();
    const base_priority = measureConstraintRowCachedPriority(
        2.0,
        .{
            .speculative_contact = true,
            .speculative_bias_tier = .base,
        },
        null,
    );
    const mid_priority = measureConstraintRowCachedPriority(
        2.0,
        .{
            .speculative_contact = true,
            .speculative_bias_tier = .mid,
        },
        null,
    );
    const high_priority = measureConstraintRowCachedPriority(
        2.0,
        .{
            .speculative_contact = true,
            .speculative_bias_tier = .high,
        },
        null,
    );

    try std.testing.expectApproxEqAbs(@as(f32, 2.0) * policy.base_scale, base_priority, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0) * policy.mid_scale, mid_priority, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0) * policy.high_scale, high_priority, 0.0001);
    try std.testing.expect(base_priority < mid_priority);
    try std.testing.expect(mid_priority < high_priority);
}

test "computeConstraintPreconditionerModel clamps priority scale by effective mass" {
    const policy = constraintPreconditionerPolicy();
    const heavy = computeConstraintPreconditionerModel(
        .{ .effective_mass = 4.0, .bias = 0.0, .max_impulse = 1.0 },
        policy,
    );
    const light = computeConstraintPreconditionerModel(
        .{ .effective_mass = 0.01, .bias = 0.0, .max_impulse = 1.0 },
        policy,
    );

    try std.testing.expect(heavy.active);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), heavy.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(policy.max_priority_scale, heavy.priority_scale, 0.0001);
    try std.testing.expect(light.active);
    try std.testing.expectApproxEqAbs(policy.min_effective_mass, light.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(policy.min_priority_scale, light.priority_scale, 0.0001);
}

test "buildConstraintRow applies preconditioned priority without mutating equation" {
    const row = buildConstraintRow(
        .contact_normal,
        3,
        null,
        2.0,
        .{},
        .{
            .effective_mass = 4.0,
            .bias = 0.5,
            .max_impulse = 3.0,
        },
    ) orelse return error.TestUnexpectedResult;

    try std.testing.expectApproxEqAbs(@as(f32, 2.5), row.priority, 0.0001);
    try std.testing.expect(row.metadata.preconditioner_active);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), row.metadata.preconditioner_effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.25), row.metadata.preconditioner_priority_scale, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), row.equation.effective_mass, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), row.equation.bias, 0.0001);
}

test "measureSpeculativeContactBiasScale increases with predictive excess" {
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.1),
        measureSpeculativeContactBiasScale(buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 1.0)),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.15),
        measureSpeculativeContactBiasScale(buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 1.5)),
        0.0001,
    );
    try std.testing.expectApproxEqAbs(
        @as(f32, 1.2),
        measureSpeculativeContactBiasScale(buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 2.5)),
        0.0001,
    );
}

test "measureSpeculativeContactBiasScale matches policy helper" {
    const direction = buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 1.75);
    const policy = speculativeContactBiasPolicy();

    try std.testing.expectApproxEqAbs(
        measureSpeculativeContactBiasScale(direction),
        measureSpeculativeContactBiasScaleWithPolicy(direction, policy),
        0.0001,
    );
}

test "measureSpeculativeContactBiasProfile reports tier and scale consistently" {
    const policy = speculativeContactBiasPolicy();
    const base_profile = measureSpeculativeContactBiasProfile(
        buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 0.75),
        policy,
    );
    const mid_profile = measureSpeculativeContactBiasProfile(
        buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 1.75),
        policy,
    );
    const high_profile = measureSpeculativeContactBiasProfile(
        buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 2.75),
        policy,
    );

    try std.testing.expectEqual(SpeculativeContactBiasTier.base, base_profile.tier);
    try std.testing.expectEqual(SpeculativeContactBiasTier.mid, mid_profile.tier);
    try std.testing.expectEqual(SpeculativeContactBiasTier.high, high_profile.tier);
    try std.testing.expectApproxEqAbs(policy.base_bias_scale, base_profile.scale, 0.0001);
    try std.testing.expectApproxEqAbs(policy.mid_bias_scale, mid_profile.scale, 0.0001);
    try std.testing.expectApproxEqAbs(policy.high_bias_scale, high_profile.scale, 0.0001);
}

test "withSpeculativeContactDirectionalMetadata applies speculative fields from direction" {
    const base: ConstraintRowMetadata = .{
        .predictive_residual_hint = 0.5,
    };
    const direction = buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 0.5, 2.5);

    const speculative = withSpeculativeContactDirectionalMetadata(base, true, direction);
    const non_speculative = withSpeculativeContactDirectionalMetadata(base, false, direction);

    try std.testing.expect(speculative.speculative_contact);
    try std.testing.expectApproxEqAbs(@as(f32, 1.2), speculative.speculative_bias_scale, 0.0001);
    try std.testing.expectEqual(SpeculativeContactBiasTier.high, speculative.speculative_bias_tier);
    try std.testing.expectApproxEqAbs(@as(f32, 2.0), speculative.speculative_predictive_excess, 0.0001);
    try std.testing.expectEqual(base.predictive_residual_hint, speculative.predictive_residual_hint);

    try std.testing.expect(!non_speculative.speculative_contact);
    try std.testing.expectEqual(@as(f32, 1.0), non_speculative.speculative_bias_scale);
    try std.testing.expectEqual(SpeculativeContactBiasTier.none, non_speculative.speculative_bias_tier);
    try std.testing.expectEqual(@as(f32, 0.0), non_speculative.speculative_predictive_excess);
}

test "makeDirectionalPredictiveRowPlan preserves predictive gain semantics" {
    const direction = buildPreparedDirectionalConstraint(0.0, 1.0, 0.0, 2.0, 6.0);
    const policy = contactNormalPredictiveConstraintPolicy();
    const plan = makeDirectionalPredictiveRowPlan(direction, policy);
    const gain = makeDirectionalPredictiveConstraintGain(direction, policy);

    try std.testing.expectEqual(kernelPredictionDt(), plan.horizon_dt);
    try std.testing.expectEqual(@as(f32, 2.0), plan.current_depth);
    try std.testing.expectEqual(@as(f32, 6.0), plan.predicted_depth);
    try std.testing.expectEqual(gain.residual_hint, plan.urgency);
    try std.testing.expectEqual(gain.impulse_delta, plan.allowed_correction_budget);
    try std.testing.expectEqual(gain.residual_hint, plan.gain.residual_hint);
    try std.testing.expectEqual(gain.bias_delta, plan.gain.bias_delta);
    try std.testing.expectEqual(gain.impulse_delta, plan.gain.impulse_delta);
}

test "makeJointDrivePredictiveRowPlan preserves error magnitudes for brake decisions" {
    const plan = makeJointDrivePredictiveRowPlan(3.5, -1.25, kernelPredictionDt());

    try std.testing.expectEqual(kernelPredictionDt(), plan.horizon_dt);
    try std.testing.expectEqual(@as(f32, 3.5), plan.current_depth);
    try std.testing.expectEqual(@as(f32, 1.25), plan.predicted_depth);
    try std.testing.expectEqual(@as(f32, 3.5), plan.urgency);
    try std.testing.expectEqual(@as(f32, 1.25), plan.allowed_correction_budget);
}

test "writeConstraintRowDebugSnapshots exports row metadata" {
    const rows = [_]ConstraintRow{
        .{
            .kind = .environment,
            .index = 2,
            .priority = 3.5,
            .base_residual = 1.25,
            .metadata = .{
                .predictive_residual_hint = 0.75,
                .speculative_contact = true,
                .speculative_bias_scale = 1.2,
                .speculative_bias_tier = .high,
                .speculative_predictive_excess = 2.25,
            },
            .equation = .{
                .effective_mass = 2.0,
                .bias = 0.25,
                .max_impulse = 5.0,
            },
        },
    };
    var snapshots: [1]ConstraintRowDebugSnapshot = undefined;

    const count = writeConstraintRowDebugSnapshots(rows[0..], snapshots[0..]);

    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(ConstraintRowKind.environment, snapshots[0].kind);
    try std.testing.expectEqual(@as(usize, 2), snapshots[0].index);
    try std.testing.expectEqual(@as(f32, 3.5), snapshots[0].priority);
    try std.testing.expectEqual(@as(f32, 1.25), snapshots[0].base_residual);
    try std.testing.expectEqual(@as(f32, 0.75), snapshots[0].metadata.predictive_residual_hint);
    try std.testing.expect(snapshots[0].metadata.speculative_contact);
    try std.testing.expectEqual(@as(f32, 1.2), snapshots[0].metadata.speculative_bias_scale);
    try std.testing.expectEqual(SpeculativeContactBiasTier.high, snapshots[0].metadata.speculative_bias_tier);
    try std.testing.expectEqual(@as(f32, 2.25), snapshots[0].metadata.speculative_predictive_excess);
    try std.testing.expectEqual(@as(f32, 0.25), snapshots[0].equation.bias);
}

test "collectConstraintRowDebugSnapshots includes environment row" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);
    try s1024.setVoxelAtGlobal(address.encode(.{
        .world = 0,
        .px = 0,
        .py = 0,
        .pz = 0,
        .lx = 10,
        .ly = 10,
        .lz = 10,
    }), true);

    var body = entity16.initEntity16();
    body.physics.mass = 20;
    body.physics.material = .solid;
    entity16.setVoxel(&body, 0, 0, 0);

    var entities = [_]entity16.Entity16{body};
    s1024.instance_count = 1;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var snapshots: [8]ConstraintRowDebugSnapshot = undefined;
    const count = collectConstraintRowDebugSnapshots(
        &s1024,
        entities[0..],
        &.{},
        &.{},
        snapshots[0..],
    );

    try std.testing.expect(count != 0);

    var found_environment = false;
    for (snapshots[0..count]) |snapshot| {
        if (snapshot.kind != .environment) continue;
        found_environment = true;
        try std.testing.expect(snapshot.base_residual > 0.0);
        try std.testing.expect(snapshot.equation.max_impulse > 0.0);
    }

    try std.testing.expect(found_environment);
}

test "collectConstraintRowDebugSnapshots includes contact normal and friction rows" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var floor_like = entity16.initEntity16();
    floor_like.physics.mass = 0;
    floor_like.physics.material = .solid;
    floor_like.physics.flags |= 0x01;
    floor_like.physics.friction = 255;
    entity16.setVoxel(&floor_like, 0, 0, 0);

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    mover.physics.friction = 255;
    entity16.setVoxel(&mover, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, floor_like };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 8,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .resting,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var snapshots: [8]ConstraintRowDebugSnapshot = undefined;
    const count = collectConstraintRowDebugSnapshots(
        &s1024,
        entities[0..],
        &.{},
        &.{.{ .a = 0, .b = 1 }},
        snapshots[0..],
    );

    try std.testing.expect(count >= 2);

    var found_normal = false;
    var found_friction = false;
    for (snapshots[0..count]) |snapshot| {
        if (snapshot.kind == .contact_normal) {
            found_normal = true;
            try std.testing.expect(snapshot.base_residual > 0.0);
        }
        if (snapshot.kind == .contact_friction) {
            found_friction = true;
            try std.testing.expect(snapshot.base_residual > 0.0);
        }
    }

    try std.testing.expect(found_normal);
    try std.testing.expect(found_friction);
}

test "collectConstraintRowDebugSnapshots marks speculative contact normal metadata" {
    var inst_a: scene32.Instance = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 120,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    var inst_b: scene32.Instance = .{
        .entity_id = 1,
        .pos_x = 12,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var mover = entity16.initEntity16();
    mover.physics.mass = 10;
    mover.physics.material = .solid;
    entity16.setVoxel(&mover, 0, 0, 0);

    var entities = [_]entity16.Entity16{ mover, mover };
    const prepared = prepareContactConstraintPair(&inst_a, &inst_b, entities[0..]) orelse return error.TestUnexpectedResult;
    try std.testing.expect(prepared.speculative);

    const ctx: RowSpecBuildContext = .{
        .contact = .{
            .prepared = prepared,
            .pair_idx = 0,
        },
    };

    const normal_spec = buildConstraintRowSpecFromEntry(&ctx, &contact_row_spec_builder_entries[0]) orelse return error.TestUnexpectedResult;
    const friction_spec = buildConstraintRowSpecFromEntry(&ctx, &contact_row_spec_builder_entries[1]);
    const expected_predictive_excess = @max(0.0, prepared.normal.predicted_depth - prepared.normal.depth);
    const expected_profile = measureSpeculativeContactBiasProfileDefault(prepared.normal);

    try std.testing.expect(normal_spec.metadata.speculative_contact);
    try std.testing.expectApproxEqAbs(expected_profile.scale, normal_spec.metadata.speculative_bias_scale, 0.0001);
    try std.testing.expectEqual(expected_profile.tier, normal_spec.metadata.speculative_bias_tier);
    try std.testing.expect(normal_spec.metadata.speculative_predictive_excess >= 0.0);
    try std.testing.expectApproxEqAbs(expected_predictive_excess, normal_spec.metadata.speculative_predictive_excess, 0.0001);
    if (friction_spec) |spec| {
        try std.testing.expectEqual(@as(f32, 0.0), spec.metadata.predictive_residual_hint);
        try std.testing.expect(!spec.metadata.speculative_contact);
        try std.testing.expectEqual(@as(f32, 1.0), spec.metadata.speculative_bias_scale);
        try std.testing.expectEqual(SpeculativeContactBiasTier.none, spec.metadata.speculative_bias_tier);
        try std.testing.expectEqual(@as(f32, 0.0), spec.metadata.speculative_predictive_excess);
    }
}

test "collectConstraintRowDebugSnapshots includes joint anchor limit and drive rows" {
    var s1024 = scene1024.Scene1024.init(std.testing.allocator);
    defer s1024.deinit();
    _ = try s1024.getPage(0);

    var body = entity16.initEntity16();
    body.physics.mass = 10;
    body.physics.material = .solid;
    entity16.setVoxel(&body, 0, 0, 0);

    var entities = [_]entity16.Entity16{ body, body };
    s1024.instance_count = 2;
    s1024.instances[0] = .{
        .entity_id = 0,
        .pos_x = 10,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };
    s1024.instances[1] = .{
        .entity_id = 1,
        .pos_x = 14,
        .pos_y = 10,
        .pos_z = 10,
        .rot_yaw = 0,
        .rot_pitch = 0,
        .rot_roll = 0,
        .state = .moving,
        .sleep_tick = 0,
        .vel_x = 0,
        .vel_y = 0,
        .vel_z = 0,
        .ang_x = 0,
        .ang_y = 0,
        .ang_z = 0,
        ._reserved = .{0} ** 2,
    };

    var joints = [_]joint.Joint{
        .{
            .enabled = true,
            .joint_type = .slider,
            .entity_a = 0,
            .entity_b = 1,
            .anchor_a_x = 0,
            .anchor_a_y = 0,
            .anchor_a_z = 0,
            .anchor_b_x = 0,
            .anchor_b_y = 0,
            .anchor_b_z = 0,
            .axis_x = 1,
            .axis_y = 0,
            .axis_z = 0,
            .limit_min = -1.0,
            .limit_max = 1.0,
            .motor_enabled = true,
            .motor_target = 5.0,
            .motor_speed = 6.0,
            .motor_max_torque = 10.0,
        },
    };

    var snapshots: [8]ConstraintRowDebugSnapshot = undefined;
    const count = collectConstraintRowDebugSnapshots(
        &s1024,
        entities[0..],
        joints[0..],
        &.{},
        snapshots[0..],
    );

    try std.testing.expect(count >= 3);

    var found_anchor = false;
    var found_limit = false;
    var found_drive = false;
    for (snapshots[0..count]) |snapshot| {
        if (snapshot.kind == .joint_anchor) found_anchor = true;
        if (snapshot.kind == .joint_limit) found_limit = true;
        if (snapshot.kind == .joint_drive) found_drive = true;
    }

    try std.testing.expect(found_anchor);
    try std.testing.expect(found_limit);
    try std.testing.expect(found_drive);
}
