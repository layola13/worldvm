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
const break_response = @import("break_response.zig");
const kcc = @import("kcc.zig");
const vehicle = @import("vehicle.zig");
const ragdoll = @import("ragdoll.zig");
const ballistics = @import("ballistics.zig");
const joint = @import("joint.zig");
const destruction = @import("destruction.zig");

pub const MAX_BROADPHASE_PAIRS: usize = scene32.MAX_INSTANCES * (scene32.MAX_INSTANCES - 1) / 2;

pub const BroadPhasePair = struct {
    a: u8,
    b: u8,
};

pub const ConstraintStageResult = struct {
    pair_count: usize,
    changed: bool,
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

const ConstraintRow = struct {
    kind: ConstraintRowKind,
    index: usize,
    priority: f32,
    base_residual: f32,
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

const PreparedJointDriveConstraint = struct {
    signed_step: f32,
    desired_velocity: f32,
};

const JointPreparedLimitChannel = union(enum) {
    linear: struct {
        axis: JointAxisVector,
        constraint: PreparedJointLinearConstraint,
    },
    angular: struct {
        axis: JointAxis,
        constraint: PreparedJointAngularConstraint,
    },
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

const DirectionalRowPlan = struct {
    residual: f32,
    predictive_residual_hint: f32 = 0.0,
    equation: ConstraintRowEquation,
};

const ConstraintRowBuildSpec = struct {
    kind: ConstraintRowKind,
    index: usize,
    residual: f32,
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

const ContactPreparedPair = struct {
    inv_mass_a: f32,
    inv_mass_b: f32,
    normal: PreparedDirectionalConstraint,
    restitution: f32,
    friction: f32,
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

pub fn collectBroadPhasePairs(instances: []const scene32.Instance, entities: []entity16.Entity16, out_pairs: []BroadPhasePair) usize {
    var pair_count: usize = 0;

    var i: u8 = 0;
    while (i < instances.len) : (i += 1) {
        const inst_a = &instances[i];
        if (!canParticipateInBroadPhase(inst_a, entities)) continue;

        var j: u8 = i + 1;
        while (j < instances.len) : (j += 1) {
            const inst_b = &instances[j];
            if (!shouldGenerateBroadPhasePair(inst_a, inst_b, entities)) continue;
            if (pair_count >= out_pairs.len) return pair_count;
            out_pairs[pair_count] = .{
                .a = @min(i, j),
                .b = @max(i, j),
            };
            pair_count += 1;
        }
    }

    return pair_count;
}

pub fn enqueueCollisionPairEvent(
    queue: *collision_event.PendingCollisionQueue,
    instances: []const scene32.Instance,
    entities: []entity16.Entity16,
    inst: *const scene32.Instance,
    impact_velocity: i16,
    blocker_id: u8,
) void {
    queue.enqueuePair(instances, entities, inst, impact_velocity, blocker_id);
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

    if (inst_moved or inst.vel_x != 0 or inst.vel_y != 0 or inst.vel_z != 0 or inst.ang_x != 0 or inst.ang_y != 0 or inst.ang_z != 0) {
        wakeInstance(inst);
    } else if (shouldSleep(inst)) {
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

        if (shouldSleep(inst)) {
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

pub fn clearPendingCollisions(queue: *collision_event.PendingCollisionQueue) void {
    queue.clear();
}

pub fn shouldSleep(inst: *scene32.Instance) bool {
    return sleep_response.shouldSleep(inst);
}

pub fn wakeInstance(inst: *scene32.Instance) void {
    sleep_response.wakeInstance(inst);
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
    const changed = break_response.applyBreakState(inst, entity, impact_velocity, damage_scale);
    if (changed) {
        sleep_response.wakeSupportedInstancesAfterBreak(
            s1024.instances[0..s1024.instance_count],
            entities,
            instance_idx,
        );
    }
    return changed;
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
    const impact = physics.calcImpact(impact_velocity, entity.physics.mass);
    const break_result = physics.checkBreak(impact, entity.physics.material, entity.physics.hardness);
    return break_result.did_break;
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
        .spring => null,
    };
}

fn prepareJointSpringLinearConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) ?PreparedJointLinearConstraint {
    const delta = measureKernelJointAnchorDelta(inst_a, inst_b, joint_def);
    if (lengthAndNormal(delta)) |distance| {
        if (distance.len <= 0.001) return null;
        const rest_length = @max(0.0, joint_def.limit_max);
        const extension = distance.len - rest_length;
        const stiffness = @max(0.0, joint_def.stiffness) * 0.01;
        const rel_speed = measureKernelRelativeLinearSpeed(inst_a, inst_b, distance.nx, distance.ny, distance.nz);
        const prediction_dt = kernelPredictionDt();
        const predicted_extension = extension + rel_speed * prediction_dt;
        const control_extension = if (extension * predicted_extension < 0.0)
            predicted_extension
        else if (@abs(predicted_extension) < @abs(extension))
            predicted_extension
        else
            extension;
        return .{
            .dir_x = distance.nx,
            .dir_y = distance.ny,
            .dir_z = distance.nz,
            .magnitude = control_extension * stiffness,
        };
    }
    return null;
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
        .hinge => if (prepareJointHingeLimitAngularConstraint(inst_a, inst_b, joint_def)) |prepared|
            .{ .ready = .{ .angular = .{ .axis = jointDominantAxis(joint_def), .constraint = prepared } } }
        else
            .stalled_prepare,
        .slider => if (prepareJointSliderLimitLinearConstraint(inst_a, inst_b, joint_def)) |prepared|
            .{ .ready = .{
                .linear = .{
                    .axis = .{
                        .x = prepared.dir_x,
                        .y = prepared.dir_y,
                        .z = prepared.dir_z,
                    },
                    .constraint = prepared,
                },
            } }
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
            .{ .ready = .{
                .axis = .{
                    .x = prepared.dir_x,
                    .y = prepared.dir_y,
                    .z = prepared.dir_z,
                },
                .constraint = prepared,
                .damping_scale = @max(0.0, joint_def.damping) * 0.001,
            } }
        else
            .stalled_prepare,
        .fixed, .ball_socket, .hinge, .slider => if (prepareJointAnchorLinearConstraint(inst_a, inst_b, joint_def)) |prepared|
            .{ .ready = .{
                .axis = .{
                    .x = prepared.dir_x,
                    .y = prepared.dir_y,
                    .z = prepared.dir_z,
                },
                .constraint = prepared,
                .damping_scale = @max(0.05, @min(1.0, @max(0.0, joint_def.damping) * 0.001)),
            } }
        else
            .stalled_prepare,
    };
}

fn prepareJointDriveChannel(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const joint.Joint,
) JointPreparedOutcome(JointPreparedDriveChannel) {
    return switch (joint_def.joint_type) {
        .hinge => if (prepareJointHingeDriveAngularConstraint(inst_a, inst_b, joint_def)) |prepared|
            .{ .ready = .{ .angular = .{ .axis = jointDominantAxis(joint_def), .constraint = prepared } } }
        else
            .stalled_prepare,
        .slider => {
            const axis = getKernelJointAxisVector(joint_def) orelse return .stalled_prepare;
            const prepared = prepareJointDriveConstraint(inst_a, inst_b, joint_def) orelse return .stalled_prepare;
            return .{ .ready = .{
                .linear = .{
                    .axis = axis,
                    .constraint = prepared,
                },
            } };
        },
        else => .inactive,
    };
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
    return switch (prepared) {
        .angular => |angular| .{ .angular = .{
            .axis = angular.axis,
            .constraint = angular.constraint,
            .postprocess = postprocess,
        } },
        .linear => |linear| .{ .linear = .{
            .axis = linear.axis,
            .constraint = linear.constraint,
            .postprocess = postprocess,
        } },
    };
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

fn finalizePairSolveStepResult(
    measure_after: f32,
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    before: f32,
    solve_step: ConstraintSolveStep,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return finalizePairRowResult(
        measure_after,
        inst_a,
        inst_b,
        before,
        solve_step.changed,
        solve_step.applied_impulse,
        equation,
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

fn finalizeSingleSolveStepResult(
    measure_after: f32,
    inst: *scene32.Instance,
    before: f32,
    solve_step: ConstraintSolveStep,
    equation: ConstraintRowEquation,
) ConstraintRowExecResult {
    return finalizeSingleRowResult(
        measure_after,
        inst,
        before,
        solve_step.changed,
        solve_step.applied_impulse,
        equation,
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
    return mapPreparedContactPairOutcome(
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
    return mapPreparedContactPairOutcome(
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
    if (base_residual <= 0.0) return .inactive;
    return mapPreparedExecutionOutcome(
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
        prepareContactExecutionContext(s1024, entities, pair),
        buildContactRuntimeSolveContext,
    );
}

fn prepareContactBatchSolveContext(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    pair: BroadPhasePair,
) ContactBatchSolveContextOutcome {
    return mapPreparedExecutionOutcome(
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

fn runPreparedContactBatch(
    _: void,
    ready: ContactBatchSolveContext,
) bool {
    return executePreparedContactBatch(ready);
}

fn executePreparedContactBatchOutcome(
    outcome: ContactBatchSolveContextOutcome,
) bool {
    return executePreparedBatchOutcome(
        ContactBatchSolveContext,
        ContactBatchSolveContextOutcome,
        void,
        outcome,
        {},
        runPreparedContactBatch,
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
    if (base_residual <= 0.0) return .inactive;
    return mapPreparedExecutionOutcome(
        f32,
        EnvironmentExecutionContext,
        EnvironmentRuntimeSolveContext,
        EnvironmentExecutionOutcome,
        EnvironmentRuntimeSolveContextOutcome,
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
    return mapPreparedExecutionOutcome(
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

fn runPreparedEnvironmentBatch(
    _: void,
    ready: EnvironmentBatchSolveContext,
) bool {
    return executePreparedEnvironmentBatch(ready);
}

fn executePreparedEnvironmentBatchOutcome(
    outcome: EnvironmentBatchSolveContextOutcome,
) bool {
    return executePreparedBatchOutcome(
        EnvironmentBatchSolveContext,
        EnvironmentBatchSolveContextOutcome,
        void,
        outcome,
        {},
        runPreparedEnvironmentBatch,
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
        .{
            .changed = solve_result.solve_step.changed,
            .applied_impulse = applied_impulse,
        },
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
        .solve_step = applyPairAngularPositionRowStep(
            inst_a,
            inst_b,
            axis,
            angle_a,
            angle_b,
            correction,
            makePairPrimitiveFromJointMassData(mass_data, .angular_position, equation, 0.35, 0.0, 0.0, 0.0),
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
            break :blk buildJointAngularPositionSolveResult(
                inst_a,
                inst_b,
                mass_data,
                angular.axis,
                angular.constraint.angle_a,
                angular.constraint.angle_b,
                correction,
                equation,
            );
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
    return applyPairAngularVelocityRowStep(
        inst_a,
        inst_b,
        axis,
        -relative_velocity,
        makePairPrimitiveFromJointMassData(mass_data, .angular_velocity, velocity_equation, 0.0, 0.0, 0.0, 0.0),
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

    const use_predictive_brake = position_error * predicted_error < 0.0 or @abs(predicted_error) < @abs(position_error);
    const control_error = if (use_predictive_brake) predicted_error else position_error;
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
    return applyPairAngularVelocityRowStep(
        inst_a,
        inst_b,
        axis,
        desired_velocity - current_rel_velocity,
        makePairPrimitiveFromJointMassData(mass_data, .angular_velocity, velocity_equation, 0.0, 0.0, 0.0, 0.0),
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
    return buildNormalConstraintEquation(
        effectiveMassFromPairInverseMasses(inv_mass_a, inv_mass_b),
        penetration_depth,
        relative_normal_speed,
        0.2,
        0.05,
        8.0,
        0.5,
    );
}

fn buildContactFrictionEquation(
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangential_speed: f32,
    friction_coeff: f32,
    normal_impulse_limit: f32,
) ConstraintRowEquation {
    const effective_mass = effectiveMassFromPairInverseMasses(inv_mass_a, inv_mass_b);
    return .{
        .effective_mass = effective_mass,
        .bias = @abs(tangential_speed) * 0.05,
        .max_impulse = @max(0.25, @min(friction_coeff * @max(0.5, normal_impulse_limit), friction_coeff * 8.0)),
    };
}

fn buildContactNormalRowPlan(
    inv_mass_a: f32,
    inv_mass_b: f32,
    penetration_depth: f32,
    relative_normal_speed: f32,
    direction: PreparedDirectionalConstraint,
) DirectionalRowPlan {
    return buildDirectionalRowPlan(
        buildContactNormalEquation(
            inv_mass_a,
            inv_mass_b,
            penetration_depth,
            relative_normal_speed,
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
    };
}

fn jointHasLimitRow(joint_def: *const joint.Joint) bool {
    return switch (joint_def.joint_type) {
        .hinge, .slider => @abs(joint_def.limit_max - joint_def.limit_min) < 6.0,
        .spring => true,
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

fn probeContactConstraint(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ContactConstraintProbe {
    if (inst_a.entity_id >= entities.len or inst_b.entity_id >= entities.len) return null;

    const entity_a = &entities[inst_a.entity_id];
    const entity_b = &entities[inst_b.entity_id];
    const aabb_a = physics.computeEntityWorldAABB(inst_a, entity_a) orelse return null;
    const aabb_b = physics.computeEntityWorldAABB(inst_b, entity_b) orelse return null;
    if (!physics.aabbHit(aabb_a, aabb_b)) return null;

    const manifold = collision.buildAABBContactManifold(aabb_a, aabb_b);
    if (manifold.point_count == 0 or manifold.penetration_depth <= 0.0) return null;

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

fn prepareContactConstraintPair(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ContactPreparedPair {
    const probe = probeContactConstraint(inst_a, inst_b, entities) orelse return null;
    const response = material_pairing.getPairedResponse(
        material_pairing.getSurfaceForMaterial(probe.entity_a.physics.material),
        probe.entity_a.physics.material,
        material_pairing.getSurfaceForMaterial(probe.entity_b.physics.material),
        probe.entity_b.physics.material,
    );
    const inv_mass_a = instanceInverseMass(inst_a, entities);
    const inv_mass_b = instanceInverseMass(inst_b, entities);
    const tangent_frame = buildDirectionalTangentFrame(probe.velocity_components);
    const has_tangent = tangent_frame != null;
    const tangential_speed = if (tangent_frame) |frame| frame.speed else 0.0;
    const prediction_dt = kernelPredictionDt();
    const predicted_pos_a = predictKernelInstanceState(inst_a, prediction_dt);
    const predicted_pos_b = predictKernelInstanceState(inst_b, prediction_dt);
    const predicted_penetration_depth = computePredictedAABBOverlapDepth(
        probe.aabb_a,
        inst_a,
        predicted_pos_a,
        probe.aabb_b,
        inst_b,
        predicted_pos_b,
    );
    const normal_direction = buildPreparedDirectionalConstraint(
        probe.manifold.normal_x,
        probe.manifold.normal_y,
        probe.manifold.normal_z,
        probe.manifold.penetration_depth,
        predicted_penetration_depth,
    );
    const tangent_direction = buildPreparedDirectionalConstraint(
        if (tangent_frame) |frame| frame.dir_x else 0.0,
        if (tangent_frame) |frame| frame.dir_y else 0.0,
        if (tangent_frame) |frame| frame.dir_z else 0.0,
        tangential_speed,
        tangential_speed,
    );
    const normal_row_plan = buildContactNormalRowPlan(
        inv_mass_a,
        inv_mass_b,
        probe.manifold.penetration_depth,
        probe.velocity_components.normalSpeed(),
        normal_direction,
    );
    const friction_row_plan = buildContactFrictionRowPlan(
        inv_mass_a,
        inv_mass_b,
        tangential_speed,
        response.friction,
        normal_row_plan.equation.max_impulse,
        tangent_direction,
    );

    return .{
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .normal = withPredictiveResidualHint(normal_direction, normal_row_plan.predictive_residual_hint),
        .restitution = response.restitution,
        .friction = response.friction,
        .tangent = tangent_direction,
        .has_tangent = has_tangent,
        .normal_equation = normal_row_plan.equation,
        .friction_equation = friction_row_plan.equation,
    };
}

fn computeContactSolvePriorityMagnitude(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) f32 {
    const metrics = measureContactConstraintMetrics(inst_a, inst_b, entities) orelse return 0.0;
    return metrics.stress();
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

    if (max_stress >= 1.0) budget += 1;
    if (max_stress >= 4.0) budget += 1;
    return @min(@as(u8, 6), budget);
}

fn computeContactSettleThreshold(max_stress: f32) f32 {
    if (max_stress >= 4.0) return 0.01;
    if (max_stress >= 1.0) return 0.025;
    return 0.05;
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

    if (max_stress >= 1.0) budget += 1;
    if (max_stress >= 4.0) budget += 1;
    return @min(@as(u8, 6), budget);
}

fn computeEnvironmentSettleThreshold(max_stress: f32) f32 {
    if (max_stress >= 4.0) return 0.01;
    if (max_stress >= 1.0) return 0.025;
    return 0.05;
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
    if (active_count >= 2) budget += 1;
    if (active_count >= 3) budget += 1;
    if (max_stress >= 1.0) budget += 1;
    if (max_stress >= 4.0) budget += 1;
    return @min(@as(u8, 6), budget);
}

fn computeConstraintBlockSettleThreshold(joint_stress: f32, contact_stress: f32, environment_stress: f32) f32 {
    const max_stress = @max(joint_stress, @max(contact_stress, environment_stress));
    if (max_stress >= 4.0) return 0.01;
    if (max_stress >= 1.0) return 0.025;
    return 0.05;
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

fn measureConstraintRowCachedPriority(base_priority: f32, state: ?*const ConstraintRowState) f32 {
    if (state == null) return base_priority;
    const cached = state.?;
    return @max(
        base_priority,
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
    const gain = makeDirectionalPredictiveConstraintGain(direction, policy);
    return @max(current_stress, gain.residual_hint);
}

fn makeDirectionalRowPlan(
    residual: f32,
    equation: ConstraintRowEquation,
    predictive_residual_hint: f32,
) DirectionalRowPlan {
    return .{
        .residual = residual,
        .predictive_residual_hint = predictive_residual_hint,
        .equation = equation,
    };
}

fn buildDirectionalRowPlan(
    base_equation: ConstraintRowEquation,
    direction: PreparedDirectionalConstraint,
    policy: PredictiveConstraintPolicy,
) DirectionalRowPlan {
    const gain = makeDirectionalPredictiveConstraintGain(direction, policy);
    const equation = applyPredictiveConstraintGain(base_equation, gain);
    return makeDirectionalRowPlan(
        finalizeDirectionalResidual(gain.residual_hint, equation),
        equation,
        gain.residual_hint,
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
    equation: ConstraintRowEquation,
) ?ConstraintRow {
    const priority = measureConstraintRowCachedPriority(base_residual, row_state);
    if (priority <= 0.0) return null;
    return .{
        .kind = kind,
        .index = index,
        .priority = priority,
        .base_residual = base_residual,
        .equation = equation,
    };
}

fn makeConstraintRowBuildSpec(
    kind: ConstraintRowKind,
    index: usize,
    residual: f32,
    equation: ConstraintRowEquation,
) ConstraintRowBuildSpec {
    return .{
        .kind = kind,
        .index = index,
        .residual = residual,
        .equation = equation,
    };
}

fn makeConstraintRowBuildSpecFromPlan(
    kind: ConstraintRowKind,
    index: usize,
    row_plan: DirectionalRowPlan,
) ConstraintRowBuildSpec {
    return makeConstraintRowBuildSpec(kind, index, row_plan.residual, row_plan.equation);
}

fn buildDirectionalRowSpec(
    kind: ConstraintRowKind,
    index: usize,
    row_plan: DirectionalRowPlan,
) ConstraintRowBuildSpec {
    return makeConstraintRowBuildSpecFromPlan(kind, index, row_plan);
}

fn buildDirectionalModeRowSpec(
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
        buildDirectionalModeRowSpec(kind, index, mode, payload.direction, payload.equation),
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
    return buildOptionalDirectionalRowSpec(
        kind,
        contact.pair_idx,
        enabled,
        payload,
        mode,
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
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        normal_x,
        normal_y,
        normal_z,
        @max(0.0, accumulated_impulse),
        blk: {
            var primitive = makePairPrimitiveFromInverseMasses(inv_mass_a, inv_mass_b, .linear_velocity, equation, 0.15, normal_x, normal_y, normal_z);
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
    return applyPairDirectionalDisplacementRowStep(
        inst_a,
        inst_b,
        -normal_x,
        -normal_y,
        -normal_z,
        penetration_depth,
        blk: {
            var primitive = makePairPrimitiveFromInverseMasses(inv_mass_a, inv_mass_b, .linear_displacement, equation, 0.2, -normal_x, -normal_y, -normal_z);
            primitive.magnitude_slop = 0.01;
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
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_normal_vel = rel_vel_x * normal_x + rel_vel_y * normal_y + rel_vel_z * normal_z;
    if (rel_normal_vel >= 0.0) return .{ .changed = false, .applied_impulse = 0.0 };
    const raw_impulse = -((1.0 + @max(0.0, @min(1.0, restitution))) * rel_normal_vel) / @max(inv_mass_a + inv_mass_b, 0.0001);
    const velocity_equation: ConstraintRowEquation = .{
        .effective_mass = equation.effective_mass,
        // Restitution is a velocity solve, so positional bias should not dilute or flip the bounce impulse.
        .bias = 0.0,
        .max_impulse = @max(equation.max_impulse, @abs(raw_impulse)),
    };
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        normal_x,
        normal_y,
        normal_z,
        raw_impulse,
        makePairPrimitiveFromInverseMasses(inv_mass_a, inv_mass_b, .linear_velocity, velocity_equation, 0.2, normal_x, normal_y, normal_z),
    );
}

fn applyContactFrictionRowStep(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    inv_mass_a: f32,
    inv_mass_b: f32,
    tangent_x: f32,
    tangent_y: f32,
    tangent_z: f32,
    equation: ConstraintRowEquation,
    signed_impulse: f32,
) ConstraintSolveStep {
    return applyPairVelocityImpulseRowStep(
        inst_a,
        inst_b,
        tangent_x,
        tangent_y,
        tangent_z,
        signed_impulse,
        makePairPrimitiveFromInverseMasses(
            inv_mass_a,
            inv_mass_b,
            .linear_velocity,
            equation,
            0.1,
            tangent_x,
            tangent_y,
            tangent_z,
        ),
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
    const tangent_frame = buildDirectionalTangentFrame(velocity_components) orelse return .{
        .changed = false,
        .applied_impulse = 0.0,
    };
    const raw_impulse = @min(tangent_frame.speed / @max(inv_mass_a + inv_mass_b, 0.0001), friction_coeff * 8.0);
    const signed_impulse = -raw_impulse;
    return applyContactFrictionRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        tangent_frame.dir_x,
        tangent_frame.dir_y,
        tangent_frame.dir_z,
        equation,
        signed_impulse,
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
    equation: ConstraintRowEquation,
    accumulated_impulse: f32,
    tangential_speed: f32,
) ConstraintSolveStep {
    if (@abs(tangential_speed) <= 0.0001) return .{ .changed = false, .applied_impulse = 0.0 };
    const signed_impulse = if (tangential_speed > 0.0)
        -@max(0.0, accumulated_impulse)
    else
        @max(0.0, accumulated_impulse);
    return applyContactFrictionRowStep(
        inst_a,
        inst_b,
        inv_mass_a,
        inv_mass_b,
        tangent_x,
        tangent_y,
        tangent_z,
        equation,
        signed_impulse,
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
    if (row_state) |_| {
        const warm_impulse = constraintRowWarmImpulse(row_state, 0.35, equation, 0.0);
        const warm_step = applyContactWarmStartRowStep(
            inst_a,
            inst_b,
            prepared.inv_mass_a,
            prepared.inv_mass_b,
            prepared.normal.dir_x,
            prepared.normal.dir_y,
            prepared.normal.dir_z,
            equation,
            warm_impulse,
        );
        accum.addWarmStart(warm_step);
    }
    const position_step = applyContactPositionRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.normal.depth,
        equation,
    );
    accum.applyStep(position_step);
    const velocity_step = applyContactVelocityRowStep(
        inst_a,
        inst_b,
        prepared.inv_mass_a,
        prepared.inv_mass_b,
        prepared.normal.dir_x,
        prepared.normal.dir_y,
        prepared.normal.dir_z,
        prepared.restitution,
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
            equation,
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
    const iterations = computeContactIterationBudget(broadphase_pairs, initial_max_stress);
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
        const iterations = computeConstraintBlockIterationBudget(
            initial_joint_stress,
            initial_contact_stress,
            initial_environment_stress,
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
) ConstraintStageResult {
    runPreStepSystems(s1024, entities, time_scale, sleep_time_threshold, dt);
    return runConstraintStage(s1024, entities, joints, out_pairs);
}

pub fn runPostMotionConstraintStage(
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    joints: []joint.Joint,
    out_pairs: []BroadPhasePair,
) ConstraintStageResult {
    rebuildOccupancyIfNeeded(s1024, entities, true);
    const result = runConstraintStage(s1024, entities, joints, out_pairs);
    rebuildOccupancyIfNeeded(s1024, entities, result.changed);
    return result;
}

pub fn mergeObservedPairCount(previous_pair_count: usize, current_pair_count: usize) usize {
    return @max(previous_pair_count, current_pair_count);
}

pub fn finishWorldStep(
    queue: *collision_event.PendingCollisionQueue,
    world_bus: ?*bus.Bus,
    tick: u32,
    s1024: *scene1024.Scene1024,
    entities: []entity16.Entity16,
    debris_dt: f32,
) void {
    destruction.updateDebris(debris_dt);
    publishPendingCollisions(queue, world_bus, tick);
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
    try std.testing.expectEqual(@as(u8, 2), computeContactIterationBudget(&.{}, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeContactIterationBudget(&.{.{ .a = 0, .b = 1 }}, 1.5));
    try std.testing.expectEqual(@as(u8, 4), computeContactIterationBudget(&.{.{ .a = 0, .b = 1 }}, 5.0));
}

test "computeEnvironmentIterationBudget increases for higher environment stress" {
    try std.testing.expectEqual(@as(u8, 2), computeEnvironmentIterationBudget(1, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeEnvironmentIterationBudget(1, 1.5));
    try std.testing.expectEqual(@as(u8, 4), computeEnvironmentIterationBudget(1, 5.0));
}

test "computeConstraintBlockIterationBudget increases with active stressed subsystems" {
    try std.testing.expectEqual(@as(u8, 2), computeConstraintBlockIterationBudget(0.0, 0.0, 0.0));
    try std.testing.expectEqual(@as(u8, 2), computeConstraintBlockIterationBudget(0.5, 0.0, 0.0));
    try std.testing.expectEqual(@as(u8, 3), computeConstraintBlockIterationBudget(0.5, 0.5, 0.0));
    try std.testing.expectEqual(@as(u8, 6), computeConstraintBlockIterationBudget(5.0, 5.0, 5.0));
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
