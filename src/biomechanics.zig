//! Biomechanics Module - Items 701-725
//! Research: Muscle, tendon, skeletal, joint biomechanics, soft tissue, skin, blood flow,
//! respiratory, energy, fatigue, injury, rehabilitation, motion capture, recognition,
//! prediction, generation, optimization, learning, retargeting, blending, smoothing

const std = @import("std");
const max_motion_samples: usize = 100;

var g_marker_trajectory_buffer: [max_motion_samples]MarkerPosition = undefined;
var g_generated_motion_buffer: [max_motion_samples]MarkerPosition = undefined;
var g_optimized_motion_buffer: [max_motion_samples]MarkerPosition = undefined;
var g_retargeted_buffer: [max_motion_samples]f32 = undefined;
var g_blended_motion_buffer: [max_motion_samples]MarkerPosition = undefined;
var g_smoothed_motion_buffer: [max_motion_samples]MarkerPosition = undefined;

// ============================================================================
// Item 701: Muscle Model
// ============================================================================

pub const MuscleModel = struct {
    activation_level: f32,
    max_force: f32,
    optimal_length: f32,
    tendon_length: f32,
    fiber_length: f32,
};

pub const MuscleState = struct {
    activation: f32,
    length: f32,
    velocity: f32,
    force: f32,
};

pub fn computeMuscleForce(model: *const MuscleModel, state: *const MuscleState, dt: f32) f32 {
    const safe_dt = @max(0.0001, dt);
    const activation = @max(0.0, @min(1.0, state.activation * (1.0 - safe_dt) + model.activation_level * safe_dt));
    const normalized_length = if (model.optimal_length > 0.0001) state.length / model.optimal_length else 1.0;
    const force_length_factor = @exp(-(std.math.pow(f32, normalized_length - 1.0, 2.0) / 0.45));
    const velocity_term = 1.0 / (1.0 + @exp(3.5 * state.velocity));
    const fiber_ratio = model.tendon_length / (model.fiber_length + 0.0001);
    const tendon_factor = @max(0.25, @min(2.0, fiber_ratio));
    return activation * model.max_force * force_length_factor * (0.6 + 0.4 * velocity_term) * tendon_factor;
}

pub fn updateMuscleState(state: *MuscleState, activation: f32, dt: f32) void {
    const activation_rate: f32 = 10.0;
    state.activation += (activation - state.activation) * activation_rate * dt;
    state.activation = @max(0.0, @min(1.0, state.activation));
}

// ============================================================================
// Item 702: Tendon Model
// ============================================================================

pub const TendonModel = struct {
    stiffness: f32,
    damping: f32,
    max_length: f32,
};

pub fn computeTendonForce(model: *const TendonModel, length: f32, velocity: f32) f32 {
    const strain = (length - model.max_length) / model.max_length;
    const elastic_force = model.stiffness * strain;
    const damping_force = model.damping * velocity;
    return elastic_force + damping_force;
}

// ============================================================================
// Item 703: Bone Model
// ============================================================================

pub const BoneModel = struct {
    length: f32,
    cross_section_area: f32,
    moment_of_inertia: f32,
    density: f32,
    youngs_modulus: f32,
};

pub fn computeBoneStress(model: *const BoneModel, axial_force: f32, bending_moment: f32) f32 {
    const axial_stress = @abs(axial_force) / model.cross_section_area;
    const bending_stress = @abs(bending_moment) * model.length / model.moment_of_inertia;
    return axial_stress + bending_stress;
}

pub fn computeBoneStrain(stress: f32, youngs_modulus: f32) f32 {
    return stress / youngs_modulus;
}

// ============================================================================
// Item 704: Joint Biomechanics
// ============================================================================

pub const JointType = enum(u8) {
    hinge = 0,
    ball_socket = 1,
    saddle = 2,
    ellipsoid = 3,
    planar = 4,
    free = 5,
};

pub const JointLimits = struct {
    min_position: f32,
    max_position: f32,
    min_velocity: f32,
    max_velocity: f32,
};

pub fn computeJointTorque(joint_type: JointType, position: f32, velocity: f32, limits: *const JointLimits) f32 {
    const stiffness: f32, const damping: f32 = switch (joint_type) {
        .hinge => .{ 120.0, 55.0 },
        .ball_socket => .{ 90.0, 45.0 },
        .saddle => .{ 80.0, 42.0 },
        .ellipsoid => .{ 75.0, 40.0 },
        .planar => .{ 60.0, 35.0 },
        .free => .{ 25.0, 20.0 },
    };
    var torque: f32 = -velocity * damping * 0.02;
    if (position < limits.min_position) {
        torque += (limits.min_position - position) * stiffness;
    } else if (position > limits.max_position) {
        torque -= (position - limits.max_position) * stiffness;
    }
    if (velocity < limits.min_velocity) {
        torque += (limits.min_velocity - velocity) * damping;
    } else if (velocity > limits.max_velocity) {
        torque -= (velocity - limits.max_velocity) * damping;
    }
    return torque;
}

// ============================================================================
// Item 705: Soft Tissue Model
// ============================================================================

pub const SoftTissueModel = struct {
    stiffness: f32,
    damping: f32,
    mass: f32,
    rest_volume: f32,
};

pub fn computeSoftTissueForce(model: *const SoftTissueModel, deformation: f32, deformation_velocity: f32) f32 {
    const elastic_force = model.stiffness * deformation;
    const damping_force = model.damping * deformation_velocity;
    return elastic_force + damping_force;
}

// ============================================================================
// Item 706: Skin Model
// ============================================================================

pub const SkinLayer = struct {
    epidermis_thickness: f32,
    dermis_thickness: f32,
    subcutaneous_thickness: f32,
    elasticity: f32,
};

pub fn computeSkinDeformation(layer: *const SkinLayer, pressure: f32) f32 {
    const total_thickness = layer.epidermis_thickness + layer.dermis_thickness + layer.subcutaneous_thickness;
    return pressure * total_thickness / layer.elasticity;
}

// ============================================================================
// Item 707: Blood Flow Model
// ============================================================================

pub const BloodFlowModel = struct {
    heart_rate: f32,
    stroke_volume: f32,
    blood_viscosity: f32,
    vessel_radius: f32,
};

pub fn computeCardiacOutput(model: *const BloodFlowModel) f32 {
    return model.heart_rate * model.stroke_volume;
}

pub fn computeBloodFlowRate(model: *const BloodFlowModel, pressure_gradient: f32) f32 {
    const resistance = 8.0 * model.blood_viscosity / (std.math.pow(f32, model.vessel_radius, 4.0) * 3.14159);
    return pressure_gradient / resistance;
}

// ============================================================================
// Item 708: Respiratory Model
// ============================================================================

pub const RespiratoryModel = struct {
    lung_capacity: f32,
    breathing_rate: f32,
    tidal_volume: f32,
    oxygen_affinity: f32,
};

pub fn computeOxygenUptake(model: *const RespiratoryModel, alveolar_ventilation: f32) f32 {
    return alveolar_ventilation * model.breathing_rate * 0.21 * model.oxygen_affinity;
}

// ============================================================================
// Item 709: Energy Consumption Model
// ============================================================================

pub const EnergyConsumptionModel = struct {
    basal_metabolic_rate: f32,
    activity_factor: f32,
    muscle_efficiency: f32,
};

pub fn computeEnergyExpenditure(model: *const EnergyConsumptionModel, mechanical_work: f32) f32 {
    return model.basal_metabolic_rate + (mechanical_work / model.muscle_efficiency) * model.activity_factor;
}

// ============================================================================
// Item 710: Fatigue Model
// ============================================================================

pub const FatigueModel = struct {
    recovery_rate: f32,
    fatigue_threshold: f32,
    max_fatigue: f32,
};

pub fn updateFatigueLevel(model: *const FatigueModel, current_fatigue: f32, exertion: f32, dt: f32) f32 {
    const fatigue_accumulation = exertion * dt;
    const fatigue_recovery = model.recovery_rate * dt * (1.0 - current_fatigue / model.max_fatigue);
    var new_fatigue = current_fatigue + fatigue_accumulation - fatigue_recovery;
    new_fatigue = @max(0.0, @min(model.max_fatigue, new_fatigue));
    return new_fatigue;
}

pub fn computeFatigueFactor(fatigue_level: f32, max_fatigue: f32) f32 {
    return 1.0 - fatigue_level / max_fatigue;
}

// ============================================================================
// Item 711: Injury Model
// ============================================================================

pub const InjuryType = enum(u8) {
    bruise = 0,
    sprain = 1,
    strain = 2,
    fracture = 3,
    laceration = 4,
    concussion = 5,
};

pub const InjuryModel = struct {
    injury_threshold: f32,
    healing_rate: f32,
    scar_formation: f32,
};

pub fn assessInjury(model: *const InjuryModel, impact_force: f32) ?InjuryType {
    if (impact_force > model.injury_threshold * 3.0) return .fracture;
    if (impact_force > model.injury_threshold * 2.5) return .laceration;
    if (impact_force > model.injury_threshold * 2.0) return .concussion;
    if (impact_force > model.injury_threshold * 1.5) return .sprain;
    if (impact_force > model.injury_threshold * 1.0) return .strain;
    if (impact_force > model.injury_threshold * 0.5) return .bruise;
    return null;
}

pub fn computeHealingProgress(model: *const InjuryModel, time_elapsed: f32, severity: f32) f32 {
    return @min(1.0, time_elapsed * model.healing_rate / severity);
}

// ============================================================================
// Item 712: Rehabilitation Model
// ============================================================================

pub const RehabilitationPhase = enum(u8) {
    acute = 0,
    subacute = 1,
    strengthening = 2,
    functional = 3,
    return_to_activity = 4,
};

pub fn computeRehabilitationProgress(phase: RehabilitationPhase, time_in_phase: f32) f32 {
    const phase_duration: f32 = switch (phase) {
        .acute => 7.0,
        .subacute => 14.0,
        .strengthening => 21.0,
        .functional => 14.0,
        .return_to_activity => 7.0,
    };
    return @min(1.0, time_in_phase / phase_duration);
}

// ============================================================================
// Item 713: Motion Capture
// ============================================================================

pub const MarkerPosition = struct {
    x: f32,
    y: f32,
    z: f32,
    confidence: f32,
};

pub const MotionCaptureFrame = struct {
    timestamp: u64,
    marker_count: u32,
    markers: [100]MarkerPosition,
};

pub fn computeMarkerTrajectory(frames: []const MotionCaptureFrame, marker_id: u32) ?[]const MarkerPosition {
    if (frames.len == 0) return null;
    if (marker_id >= max_motion_samples) return null;

    var count: usize = 0;
    for (frames) |frame| {
        if (count >= max_motion_samples) break;
        if (marker_id < frame.marker_count) {
            g_marker_trajectory_buffer[count] = frame.markers[marker_id];
            count += 1;
        }
    }

    if (count == 0) return null;
    return g_marker_trajectory_buffer[0..count];
}

// ============================================================================
// Item 714: Motion Recognition
// ============================================================================

pub const MotionGesture = enum(u8) {
    walk = 0,
    run = 1,
    jump = 2,
    sit = 3,
    stand = 4,
    fall = 5,
    wave = 6,
    nod = 7,
    shake_head = 8,
    point = 9,
};

pub fn recognizeMotion(positions: []const MarkerPosition, velocities: []const MarkerPosition) MotionGesture {
    if (positions.len < 3) return .stand;
    const avg_velocity = computeAverageVelocity(velocities);
    if (avg_velocity > 3.0) return .run;
    if (avg_velocity > 0.5) return .walk;
    if (avg_velocity < 0.1) return .stand;
    return .stand;
}

fn computeAverageVelocity(velocities: []const MarkerPosition) f32 {
    if (velocities.len == 0) return 0;
    var sum: f32 = 0;
    for (velocities) |v| {
        sum += @sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }
    return sum / @as(f32, @floatFromInt(velocities.len));
}

// ============================================================================
// Item 715: Motion Prediction
// ============================================================================

pub fn predictFuturePositions(current: MarkerPosition, velocity: MarkerPosition, acceleration: MarkerPosition, t: f32) MarkerPosition {
    return .{
        .x = current.x + velocity.x * t + 0.5 * acceleration.x * t * t,
        .y = current.y + velocity.y * t + 0.5 * acceleration.y * t * t,
        .z = current.z + velocity.z * t + 0.5 * acceleration.z * t * t,
        .confidence = current.confidence,
    };
}

// ============================================================================
// Item 716: Motion Generation
// ============================================================================

pub const MotionGenerator = struct {
    target_positions: []MarkerPosition,
    blend_factor: f32,
    smoothing: f32,
};

pub fn generateBlendedMotion(gen: *const MotionGenerator, motion_a: []MarkerPosition, motion_b: []MarkerPosition) []MarkerPosition {
    const count = @min(@min(motion_a.len, motion_b.len), max_motion_samples);
    if (count == 0) return g_generated_motion_buffer[0..0];

    const blend = @max(0.0, @min(1.0, gen.blend_factor));
    const target_weight = @max(0.0, @min(1.0, gen.smoothing));
    const temporal_smoothing = target_weight * 0.5;

    for (0..count) |i| {
        const target = if (i < gen.target_positions.len)
            gen.target_positions[i]
        else
            MarkerPosition{
                .x = motion_a[i].x * (1.0 - blend) + motion_b[i].x * blend,
                .y = motion_a[i].y * (1.0 - blend) + motion_b[i].y * blend,
                .z = motion_a[i].z * (1.0 - blend) + motion_b[i].z * blend,
                .confidence = @min(motion_a[i].confidence, motion_b[i].confidence),
            };

        g_generated_motion_buffer[i] = .{
            .x = (motion_a[i].x * (1.0 - blend) + motion_b[i].x * blend) * (1.0 - target_weight) + target.x * target_weight,
            .y = (motion_a[i].y * (1.0 - blend) + motion_b[i].y * blend) * (1.0 - target_weight) + target.y * target_weight,
            .z = (motion_a[i].z * (1.0 - blend) + motion_b[i].z * blend) * (1.0 - target_weight) + target.z * target_weight,
            .confidence = @max(0.0, @min(1.0, (motion_a[i].confidence + motion_b[i].confidence + target.confidence) / 3.0)),
        };

        if (i > 0 and temporal_smoothing > 0.0) {
            g_generated_motion_buffer[i].x = g_generated_motion_buffer[i].x * (1.0 - temporal_smoothing) + g_generated_motion_buffer[i - 1].x * temporal_smoothing;
            g_generated_motion_buffer[i].y = g_generated_motion_buffer[i].y * (1.0 - temporal_smoothing) + g_generated_motion_buffer[i - 1].y * temporal_smoothing;
            g_generated_motion_buffer[i].z = g_generated_motion_buffer[i].z * (1.0 - temporal_smoothing) + g_generated_motion_buffer[i - 1].z * temporal_smoothing;
        }
    }

    return g_generated_motion_buffer[0..count];
}

// ============================================================================
// Item 717: Motion Optimization
// ============================================================================

pub fn optimizeMotionTrajectory(cost_function: fn ([]MarkerPosition) f32, initial: []MarkerPosition, iterations: u32) []MarkerPosition {
    const count = @min(initial.len, max_motion_samples);
    if (count == 0) return g_optimized_motion_buffer[0..0];

    @memcpy(g_optimized_motion_buffer[0..count], initial[0..count]);
    if (count < 3 or iterations == 0) return g_optimized_motion_buffer[0..count];

    var best_cost = cost_function(g_optimized_motion_buffer[0..count]);
    var iter: u32 = 0;
    while (iter < iterations) : (iter += 1) {
        const alpha = 0.25 / @as(f32, @floatFromInt(iter + 1));
        var i: usize = 1;
        while (i + 1 < count) : (i += 1) {
            const neighbor_x = (g_optimized_motion_buffer[i - 1].x + g_optimized_motion_buffer[i + 1].x) * 0.5;
            const neighbor_y = (g_optimized_motion_buffer[i - 1].y + g_optimized_motion_buffer[i + 1].y) * 0.5;
            const neighbor_z = (g_optimized_motion_buffer[i - 1].z + g_optimized_motion_buffer[i + 1].z) * 0.5;

            g_optimized_motion_buffer[i].x = g_optimized_motion_buffer[i].x * (1.0 - alpha) + neighbor_x * alpha;
            g_optimized_motion_buffer[i].y = g_optimized_motion_buffer[i].y * (1.0 - alpha) + neighbor_y * alpha;
            g_optimized_motion_buffer[i].z = g_optimized_motion_buffer[i].z * (1.0 - alpha) + neighbor_z * alpha;
        }

        const current_cost = cost_function(g_optimized_motion_buffer[0..count]);
        if (current_cost <= best_cost) {
            best_cost = current_cost;
        } else {
            for (0..count) |idx| {
                g_optimized_motion_buffer[idx].x = g_optimized_motion_buffer[idx].x * 0.8 + initial[idx].x * 0.2;
                g_optimized_motion_buffer[idx].y = g_optimized_motion_buffer[idx].y * 0.8 + initial[idx].y * 0.2;
                g_optimized_motion_buffer[idx].z = g_optimized_motion_buffer[idx].z * 0.8 + initial[idx].z * 0.2;
            }
        }
    }

    return g_optimized_motion_buffer[0..count];
}

// ============================================================================
// Item 718: Motion Learning
// ============================================================================

pub const MotionLearningModel = struct {
    learning_rate: f32,
    momentum: f32,
    training_samples: u32,
};

pub fn updateMotionModel(model: *MotionLearningModel, observation: MarkerPosition, target: MarkerPosition) void {
    const dx = target.x - observation.x;
    const dy = target.y - observation.y;
    const dz = target.z - observation.z;
    const error_magnitude = @sqrt(dx * dx + dy * dy + dz * dz);

    const sample_scale = 1.0 / (1.0 + @as(f32, @floatFromInt(model.training_samples)) * 0.001);
    model.learning_rate = @max(0.0001, @min(1.0, model.learning_rate * (0.95 + 0.05 * sample_scale) + error_magnitude * 0.0005));
    model.momentum = @max(0.0, @min(0.99, model.momentum * 0.98 + @min(0.01, error_magnitude * 0.001)));
    if (model.training_samples < std.math.maxInt(u32)) {
        model.training_samples += 1;
    }
}

// ============================================================================
// Item 719: Motion Retargeting
// ============================================================================

pub fn retargetMotion(source_rig: []const f32, source_scale: f32, target_scale: f32) []const f32 {
    const count = @min(source_rig.len, max_motion_samples);
    if (count == 0) return g_retargeted_buffer[0..0];

    const safe_source_scale = if (@abs(source_scale) < 0.0001) 1.0 else source_scale;
    const scale_factor = target_scale / safe_source_scale;
    for (0..count) |i| {
        g_retargeted_buffer[i] = source_rig[i] * scale_factor;
    }
    return g_retargeted_buffer[0..count];
}

// ============================================================================
// Item 720: Motion Blending
// ============================================================================

pub fn blendMotions(motion_a: []MarkerPosition, motion_b: []MarkerPosition, blend_param: f32) []MarkerPosition {
    const count = @min(@min(motion_a.len, motion_b.len), max_motion_samples);
    if (count == 0) return g_blended_motion_buffer[0..0];

    const blend = @max(0.0, @min(1.0, blend_param));
    for (0..count) |i| {
        g_blended_motion_buffer[i] = .{
            .x = motion_a[i].x * (1.0 - blend) + motion_b[i].x * blend,
            .y = motion_a[i].y * (1.0 - blend) + motion_b[i].y * blend,
            .z = motion_a[i].z * (1.0 - blend) + motion_b[i].z * blend,
            .confidence = @max(0.0, @min(1.0, motion_a[i].confidence * (1.0 - blend) + motion_b[i].confidence * blend)),
        };
    }
    return g_blended_motion_buffer[0..count];
}

// ============================================================================
// Item 721: Motion Smoothing
// ============================================================================

pub fn smoothMotion(motion: []MarkerPosition, window_size: u32) []MarkerPosition {
    const point_count = @min(motion.len, max_motion_samples);
    if (point_count == 0) return g_smoothed_motion_buffer[0..0];

    const window = @max(@as(usize, 1), @as(usize, @intCast(window_size)));
    const half_window = window / 2;
    for (0..point_count) |i| {
        var sum_x: f32 = 0;
        var sum_y: f32 = 0;
        var sum_z: f32 = 0;
        var sum_confidence: f32 = 0;
        var sample_count: f32 = 0;
        const start = if (i > half_window) i - half_window else 0;
        const end = @min(point_count, i + half_window + 1);

        for (start..end) |idx| {
            sum_x += motion[idx].x;
            sum_y += motion[idx].y;
            sum_z += motion[idx].z;
            sum_confidence += motion[idx].confidence;
            sample_count += 1;
        }

        g_smoothed_motion_buffer[i] = .{
            .x = sum_x / sample_count,
            .y = sum_y / sample_count,
            .z = sum_z / sample_count,
            .confidence = @max(0.0, @min(1.0, sum_confidence / sample_count)),
        };
    }
    return g_smoothed_motion_buffer[0..point_count];
}

// ============================================================================
// Item 722: Motion Physics
// ============================================================================

pub fn applyGravityToMotion(motion: []MarkerPosition, gravity: f32, dt: f32) void {
    for (motion) |*m| {
        m.y -= gravity * dt * dt * 0.5;
    }
}

// ============================================================================
// Item 723: Motion Animation
// ============================================================================

pub const AnimationClip = struct {
    name: []const u8,
    duration: f32,
    frame_rate: f32,
    tracks: [10]AnimationTrack,
};

pub const AnimationTrack = struct {
    joint_index: u32,
    keyframes: [50]Keyframe,
    keyframe_count: u32,
};

pub const Keyframe = struct {
    time: f32,
    value: f32,
};

pub fn sampleAnimation(clip: *const AnimationClip, time: f32, joint_index: u32) f32 {
    if (joint_index >= clip.tracks.len) return 0;
    const track = clip.tracks[joint_index];
    for (0..track.keyframe_count) |i| {
        if (track.keyframes[i].time >= time) {
            return track.keyframes[i].value;
        }
    }
    return if (track.keyframe_count > 0) track.keyframes[track.keyframe_count - 1].value else 0;
}

// ============================================================================
// Item 724: Motion Control
// ============================================================================

pub const MotionControlMode = enum(u8) {
    kinematic = 0,
    dynamic = 1,
    hybrid = 2,
};

pub fn applyMotionControl(mode: MotionControlMode, target: MarkerPosition, current: MarkerPosition, dt: f32) MarkerPosition {
    const diff_x = target.x - current.x;
    const diff_y = target.y - current.y;
    const diff_z = target.z - current.z;
    const max_correction: f32 = 10.0 * dt;
    const correction_factor = @min(1.0, max_correction / @sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z + 0.001));
    return switch (mode) {
        .kinematic => target,
        .dynamic => .{
            .x = current.x + diff_x * correction_factor,
            .y = current.y + diff_y * correction_factor,
            .z = current.z + diff_z * correction_factor,
            .confidence = target.confidence,
        },
        .hybrid => .{
            .x = current.x + diff_x * correction_factor * 0.5,
            .y = current.y + diff_y * correction_factor * 0.5,
            .z = current.z + diff_z * correction_factor * 0.5,
            .confidence = target.confidence,
        },
    };
}

// ============================================================================
// Item 725: Motion Evaluation
// ============================================================================

pub const EvaluationMetric = struct {
    name: []const u8,
    value: f32,
    score: f32,
};

var g_motion_evaluation_buffer: [2]EvaluationMetric = undefined;

pub fn evaluateMotionQuality(motion: []MarkerPosition, reference: []MarkerPosition) *const [2]EvaluationMetric {
    const count = @min(motion.len, max_motion_samples);
    const ref_count = @min(reference.len, count);

    var smoothness_value: f32 = 1.0;
    if (count >= 3) {
        var curvature_sum: f32 = 0.0;
        var i: usize = 1;
        while (i + 1 < count) : (i += 1) {
            const ddx = motion[i + 1].x - 2.0 * motion[i].x + motion[i - 1].x;
            const ddy = motion[i + 1].y - 2.0 * motion[i].y + motion[i - 1].y;
            const ddz = motion[i + 1].z - 2.0 * motion[i].z + motion[i - 1].z;
            curvature_sum += @sqrt(ddx * ddx + ddy * ddy + ddz * ddz);
        }
        const avg_curvature = curvature_sum / @as(f32, @floatFromInt(count - 2));
        smoothness_value = 1.0 / (1.0 + avg_curvature);
    }

    var naturalness_value: f32 = 1.0;
    if (ref_count > 0) {
        var error_sum: f32 = 0.0;
        for (0..ref_count) |i| {
            const dx = motion[i].x - reference[i].x;
            const dy = motion[i].y - reference[i].y;
            const dz = motion[i].z - reference[i].z;
            error_sum += dx * dx + dy * dy + dz * dz;
        }
        const rms_error = @sqrt(error_sum / @as(f32, @floatFromInt(ref_count)));
        naturalness_value = 1.0 / (1.0 + rms_error);
    } else if (count > 0) {
        var confidence_sum: f32 = 0.0;
        for (0..count) |i| {
            confidence_sum += motion[i].confidence;
        }
        naturalness_value = @max(0.0, @min(1.0, confidence_sum / @as(f32, @floatFromInt(count))));
    }

    g_motion_evaluation_buffer = .{
        .{
            .name = "smoothness",
            .value = smoothness_value,
            .score = @max(0.0, @min(100.0, smoothness_value * 100.0)),
        },
        .{
            .name = "naturalness",
            .value = naturalness_value,
            .score = @max(0.0, @min(100.0, naturalness_value * 100.0)),
        },
    };
    return &g_motion_evaluation_buffer;
}

// ============================================================================
// Tests for Biomechanics (Items 701-725)
// ============================================================================

test "701: muscle model computes force" {
    var model = MuscleModel{
        .activation_level = 0.8,
        .max_force = 5000.0,
        .optimal_length = 0.1,
        .tendon_length = 0.05,
        .fiber_length = 0.1,
    };
    var state = MuscleState{
        .activation = 0.8,
        .length = 0.1,
        .velocity = 0.0,
        .force = 0.0,
    };
    const force = computeMuscleForce(&model, &state, 0.01);
    try std.testing.expect(force > 0);
}

test "702: tendon model computes force" {
    var model = TendonModel{
        .stiffness = 10000.0,
        .damping = 50.0,
        .max_length = 0.2,
    };
    const force = computeTendonForce(&model, 0.21, 0.1);
    try std.testing.expect(force > 0);
}

test "703: bone model computes stress" {
    var model = BoneModel{
        .length = 0.4,
        .cross_section_area = 0.0001,
        .moment_of_inertia = 0.0001,
        .density = 1850.0,
        .youngs_modulus = 15e9,
    };
    const stress = computeBoneStress(&model, 100.0, 5.0);
    try std.testing.expect(stress > 0);
}

test "704: joint biomechanics computes torque" {
    var limits = JointLimits{
        .min_position = -1.57,
        .max_position = 1.57,
        .min_velocity = -10.0,
        .max_velocity = 10.0,
    };
    const torque = computeJointTorque(.hinge, 0.5, 0.0, &limits);
    try std.testing.expect(torque == 0);
}

test "705: soft tissue model computes force" {
    var model = SoftTissueModel{
        .stiffness = 1000.0,
        .damping = 10.0,
        .mass = 1.0,
        .rest_volume = 0.001,
    };
    const force = computeSoftTissueForce(&model, 0.01, 0.1);
    try std.testing.expect(force > 0);
}

test "706: skin model computes deformation" {
    var layer = SkinLayer{
        .epidermis_thickness = 0.0001,
        .dermis_thickness = 0.002,
        .subcutaneous_thickness = 0.015,
        .elasticity = 1e6,
    };
    const deformation = computeSkinDeformation(&layer, 1000.0);
    try std.testing.expect(deformation > 0);
}

test "707: blood flow model computes cardiac output" {
    var model = BloodFlowModel{
        .heart_rate = 70.0,
        .stroke_volume = 0.07,
        .blood_viscosity = 0.004,
        .vessel_radius = 0.01,
    };
    const co = computeCardiacOutput(&model);
    try std.testing.expectApproxEqAbs(@as(f32, 4.9), co, 0.1);
}

test "708: respiratory model computes oxygen uptake" {
    var model = RespiratoryModel{
        .lung_capacity = 6.0,
        .breathing_rate = 15.0,
        .tidal_volume = 0.5,
        .oxygen_affinity = 0.95,
    };
    const oxygen = computeOxygenUptake(&model, 0.3);
    try std.testing.expect(oxygen > 0);
}

test "709: energy consumption model computes expenditure" {
    var model = EnergyConsumptionModel{
        .basal_metabolic_rate = 80.0,
        .activity_factor = 1.5,
        .muscle_efficiency = 0.25,
    };
    const expenditure = computeEnergyExpenditure(&model, 100.0);
    try std.testing.expect(expenditure > 80.0);
}

test "710: fatigue model updates fatigue level" {
    var model = FatigueModel{
        .recovery_rate = 0.1,
        .fatigue_threshold = 0.5,
        .max_fatigue = 100.0,
    };
    const fatigue = updateFatigueLevel(&model, 20.0, 5.0, 0.1);
    try std.testing.expect(fatigue > 20.0);
}

test "711: injury model assesses injury" {
    var model = InjuryModel{
        .injury_threshold = 1000.0,
        .healing_rate = 0.05,
        .scar_formation = 0.1,
    };
    const injury = assessInjury(&model, 3500.0);
    try std.testing.expect(injury != null);
}

test "712: rehabilitation model computes progress" {
    const progress = computeRehabilitationProgress(.subacute, 7.0);
    try std.testing.expect(progress > 0.4 and progress < 0.6);
}

test "713: motion capture trajectory extraction" {
    var frames: [3]MotionCaptureFrame = undefined;
    for (0..frames.len) |i| {
        frames[i] = .{
            .timestamp = @intCast(1000 + i),
            .marker_count = 2,
            .markers = undefined,
        };
        frames[i].markers[0] = .{ .x = @floatFromInt(i), .y = 0.0, .z = 0.0, .confidence = 0.95 };
        frames[i].markers[1] = .{ .x = @floatFromInt(10 + i), .y = @floatFromInt(i), .z = 1.0, .confidence = 0.9 };
    }

    const trajectory = computeMarkerTrajectory(&frames, 1) orelse return error.TestUnexpectedResult;
    try std.testing.expect(trajectory.len == 3);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), trajectory[0].x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), trajectory[2].x, 0.0001);
}

test "714: motion recognition identifies walk" {
    var positions: [3]MarkerPosition = .{
        .{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 0.5, .y = 0, .z = 0.5, .confidence = 1.0 },
        .{ .x = 1.0, .y = 0, .z = 1.0, .confidence = 1.0 },
    };
    var velocities: [3]MarkerPosition = .{
        .{ .x = 0.5, .y = 0, .z = 0.5, .confidence = 1.0 },
        .{ .x = 0.5, .y = 0, .z = 0.5, .confidence = 1.0 },
        .{ .x = 0.5, .y = 0, .z = 0.5, .confidence = 1.0 },
    };
    const gesture = recognizeMotion(&positions, &velocities);
    try std.testing.expect(gesture == .walk);
}

test "715: motion prediction computes future position" {
    const current = MarkerPosition{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 };
    const velocity = MarkerPosition{ .x = 1.0, .y = 0, .z = 0, .confidence = 1.0 };
    const acceleration = MarkerPosition{ .x = 0, .y = -9.8, .z = 0, .confidence = 1.0 };
    const future = predictFuturePositions(current, velocity, acceleration, 1.0);
    try std.testing.expect(future.x > 0);
}

test "716: motion generation creates blended motion" {
    var targets: [3]MarkerPosition = .{
        .{ .x = 0, .y = 2, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 2, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 2, .z = 0, .confidence = 1.0 },
    };
    var gen = MotionGenerator{ .target_positions = targets[0..], .blend_factor = 0.5, .smoothing = 0.2 };
    var motion_a: [3]MarkerPosition = .{
        .{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 0, .z = 0, .confidence = 1.0 },
    };
    var motion_b: [3]MarkerPosition = .{
        .{ .x = 0, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 1, .z = 0, .confidence = 1.0 },
    };
    const result = generateBlendedMotion(&gen, &motion_a, &motion_b);
    try std.testing.expect(result.len == 3);
    try std.testing.expect(result[0].y > 0.5);
    try std.testing.expect(result[0].y < 1.5);
    try std.testing.expect(result[1].x > result[0].x);
}

test "717: motion optimization returns trajectory" {
    const Cost = struct {
        fn f(positions: []MarkerPosition) f32 {
            if (positions.len < 2) return 0;
            var variation: f32 = 0;
            for (1..positions.len) |i| {
                variation += @abs(positions[i].y - positions[i - 1].y);
            }
            return variation;
        }
    };

    var initial: [5]MarkerPosition = .{
        .{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 2, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = -1, .z = 0, .confidence = 1.0 },
        .{ .x = 3, .y = 2, .z = 0, .confidence = 1.0 },
        .{ .x = 4, .y = 0, .z = 0, .confidence = 1.0 },
    };
    const initial_cost = Cost.f(initial[0..]);
    const result = optimizeMotionTrajectory(
        Cost.f,
        initial[0..],
        20,
    );
    const optimized_cost = Cost.f(result);
    try std.testing.expect(result.len == initial.len);
    try std.testing.expect(optimized_cost <= initial_cost);
}

test "718: motion learning updates model" {
    var model = MotionLearningModel{
        .learning_rate = 0.01,
        .momentum = 0.9,
        .training_samples = 100,
    };
    const obs = MarkerPosition{ .x = 1, .y = 2, .z = 3, .confidence = 0.9 };
    const target = MarkerPosition{ .x = 2, .y = 3, .z = 4, .confidence = 0.9 };
    updateMotionModel(&model, obs, target);
    try std.testing.expect(model.training_samples == 101);
    try std.testing.expect(model.learning_rate > 0.0);
    try std.testing.expect(model.momentum > 0.0 and model.momentum < 1.0);
}

test "719: motion retargeting scales rig" {
    var source: [5]f32 = .{ 1.0, 2.0, 3.0, 4.0, 5.0 };
    const result = retargetMotion(&source, 1.0, 2.0);
    try std.testing.expect(result[0] == 2.0);
}

test "720: motion blending blends two motions" {
    var motion_a: [3]MarkerPosition = .{
        .{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 0, .z = 0, .confidence = 1.0 },
    };
    var motion_b: [3]MarkerPosition = .{
        .{ .x = 0, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 1, .z = 0, .confidence = 1.0 },
    };
    const blended = blendMotions(&motion_a, &motion_b, 0.5);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), blended[0].y, 0.001);
}

test "721: motion smoothing smooths motion" {
    var motion: [5]MarkerPosition = .{
        .{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 3, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 4, .y = 0, .z = 0, .confidence = 1.0 },
    };
    const smoothed = smoothMotion(&motion, 3);
    try std.testing.expect(smoothed.len > 0);
}

test "722: motion physics applies gravity" {
    var motion: [3]MarkerPosition = .{
        .{ .x = 0, .y = 10, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 10, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 10, .z = 0, .confidence = 1.0 },
    };
    applyGravityToMotion(&motion, 9.8, 0.1);
    try std.testing.expect(motion[0].y < 10.0);
}

test "723: animation clip samples animation" {
    var clip = AnimationClip{
        .name = "test",
        .duration = 1.0,
        .frame_rate = 30.0,
        .tracks = undefined,
    };
    var keyframes: [50]Keyframe = undefined;
    keyframes[0] = Keyframe{ .time = 0.0, .value = 0.0 };
    keyframes[1] = Keyframe{ .time = 0.5, .value = 1.0 };
    keyframes[2] = Keyframe{ .time = 1.0, .value = 0.0 };
    clip.tracks[0] = .{
        .joint_index = 0,
        .keyframes = keyframes,
        .keyframe_count = 3,
    };
    const value = sampleAnimation(&clip, 0.25, 0);
    try std.testing.expect(value == 1.0);
}

test "724: motion control applies control" {
    const target = MarkerPosition{ .x = 10, .y = 10, .z = 10, .confidence = 1.0 };
    const current = MarkerPosition{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 };
    const result = applyMotionControl(.dynamic, target, current, 0.016);
    try std.testing.expect(result.x > 0);
}

test "725: motion evaluation returns metrics" {
    var motion: [3]MarkerPosition = .{
        .{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 0, .z = 0, .confidence = 1.0 },
    };
    var reference: [3]MarkerPosition = .{
        .{ .x = 0, .y = 0, .z = 0, .confidence = 1.0 },
        .{ .x = 1, .y = 1, .z = 0, .confidence = 1.0 },
        .{ .x = 2, .y = 0, .z = 0, .confidence = 1.0 },
    };
    const metrics = evaluateMotionQuality(&motion, &reference);
    try std.testing.expect(std.mem.eql(u8, metrics[0].name, "smoothness"));
    try std.testing.expect(std.mem.eql(u8, metrics[1].name, "naturalness"));
    try std.testing.expect(metrics[0].score >= 0.0 and metrics[0].score <= 100.0);
    try std.testing.expect(metrics[1].score > 95.0);
}
