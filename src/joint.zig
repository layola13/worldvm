//! Joint System - Constraints for Rigid Body Physics
//!
//! Phase 2: Joint constraints (Fixed, Hinge, Slider, Spring, BallSocket, Pulley)

const std = @import("std");
const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");

const JointAxis = enum { x, y, z };

pub const JointType = enum(u8) {
    fixed = 0,
    hinge = 1,
    slider = 2,
    spring = 3,
    ball_socket = 4,
    pulley = 5,
};

pub const Joint = struct {
    joint_type: JointType,
    entity_a: u16,
    entity_b: u16,
    anchor_a_x: i32,
    anchor_a_y: i32,
    anchor_a_z: i32,
    anchor_b_x: i32,
    anchor_b_y: i32,
    anchor_b_z: i32,
    axis_x: i32 = 0,
    axis_y: i32 = 0,
    axis_z: i32 = 1,
    limit_min: f32 = -3.14159,
    limit_max: f32 = 3.14159,
    breaking_force: f32 = 0,
    stiffness: f32 = 1000,
    damping: f32 = 100,
    motor_enabled: bool = false,
    motor_target: f32 = 0,
    motor_speed: f32 = 0,
    motor_max_torque: f32 = 0,
    preload_linear_x: f32 = 0,
    preload_linear_y: f32 = 0,
    preload_linear_z: f32 = 0,
    preload_angular: f32 = 0,
    enabled: bool = true,
    warm_linear_x: f32 = 0,
    warm_linear_y: f32 = 0,
    warm_linear_z: f32 = 0,
    warm_angular: f32 = 0,
    warm_drive_velocity: f32 = 0,
    break_accum: f32 = 0,
    fatigue_damage: f32 = 0,
    fatigue_limit: f32 = 0,
    fatigue_rate: f32 = 1,
    fatigue_recovery: f32 = 0.05,
    temperature: f32 = 0,
    temperature_limit: f32 = 0,
    temperature_rate: f32 = 1,
    temperature_cooling: f32 = 0.1,
};

pub const MAX_JOINTS: usize = 64;

pub const JointSystem = struct {
    joints: [MAX_JOINTS]Joint,
    joint_count: u8 = 0,
};

pub const JointDriveState = struct {
    position: f32,
    relative_velocity: f32,
};

pub const JointStressSample = struct {
    stress: f32 = 0,
    geometry_error: f32 = 0,
    limit_error: f32 = 0,
    drive_error: f32 = 0,
    residual_speed: f32 = 0,
    break_ratio: f32 = 0,
    fatigue_ratio: f32 = 0,
    temperature_ratio: f32 = 0,
};

const JointDrivePlan = struct {
    target: f32,
    position_error: f32,
    predicted_position: f32,
    predicted_error: f32,
    signed_step: f32,
    desired_velocity: f32,
};

const JointWarmStart = struct {
    linear_x: f32 = 0,
    linear_y: f32 = 0,
    linear_z: f32 = 0,
    linear_magnitude: f32 = 0,
    angular: f32 = 0,
    drive_velocity: f32 = 0,
};

var g_joint_system: JointSystem = .{
    .joints = undefined,
    .joint_count = 0,
};

pub fn init(system: *JointSystem) void {
    system.joint_count = 0;
}

pub fn initGlobal() void {
    init(&g_joint_system);
}

pub fn addJoint(system: *JointSystem, joint: Joint) ?u8 {
    if (system.joint_count >= MAX_JOINTS) return null;
    const idx = system.joint_count;
    system.joints[idx] = joint;
    system.joint_count += 1;
    return idx;
}

pub fn removeJoint(system: *JointSystem, joint_idx: u8) void {
    if (joint_idx >= system.joint_count) return;
    var i = joint_idx;
    while (i < system.joint_count - 1) : (i += 1) {
        system.joints[i] = system.joints[i + 1];
    }
    system.joint_count -= 1;
}

pub fn getSystem() *JointSystem {
    return &g_joint_system;
}

pub fn addGlobalJoint(j: Joint) ?u8 {
    return addJoint(&g_joint_system, j);
}

pub fn clearGlobalJoints() void {
    g_joint_system.joint_count = 0;
}

pub fn setMotorEnabled(joint_def: *Joint, enabled: bool) void {
    joint_def.motor_enabled = enabled;
}

pub fn configureMotor(
    joint_def: *Joint,
    target: f32,
    speed: f32,
    max_torque: f32,
) void {
    joint_def.motor_enabled = true;
    joint_def.motor_target = target;
    joint_def.motor_speed = @max(0.0, speed);
    joint_def.motor_max_torque = @max(0.0, max_torque);
}

const ConstraintMassData = struct {
    inv_mass_a: f32,
    inv_mass_b: f32,
    ratio_a: f32,
    ratio_b: f32,
};

const AnchorDelta = struct {
    dx: f32,
    dy: f32,
    dz: f32,
    dist: f32,
};

fn inverseMass(inst: *const scene32.Instance, entities: []entity16.Entity16) f32 {
    if (inst.entity_id >= entities.len) return 0.0;
    const entity = &entities[inst.entity_id];
    if ((entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0) return 0.0;
    return 1.0 / @as(f32, @floatFromInt(entity.physics.mass));
}

fn computeConstraintMassData(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    entities: []entity16.Entity16,
) ?ConstraintMassData {
    const inv_mass_a = inverseMass(inst_a, entities);
    const inv_mass_b = inverseMass(inst_b, entities);
    const total_inv_mass = inv_mass_a + inv_mass_b;
    if (total_inv_mass <= 0.0001) return null;

    return .{
        .inv_mass_a = inv_mass_a,
        .inv_mass_b = inv_mass_b,
        .ratio_a = inv_mass_a / total_inv_mass,
        .ratio_b = inv_mass_b / total_inv_mass,
    };
}

fn computeAnchorDelta(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const Joint,
) AnchorDelta {
    const dx_i = inst_b.pos_x + joint_def.anchor_b_x - inst_a.pos_x - joint_def.anchor_a_x;
    const dy_i = inst_b.pos_y + joint_def.anchor_b_y - inst_a.pos_y - joint_def.anchor_a_y;
    const dz_i = inst_b.pos_z + joint_def.anchor_b_z - inst_a.pos_z - joint_def.anchor_a_z;
    const dx = @as(f32, @floatFromInt(dx_i));
    const dy = @as(f32, @floatFromInt(dy_i));
    const dz = @as(f32, @floatFromInt(dz_i));
    return .{
        .dx = dx,
        .dy = dy,
        .dz = dz,
        .dist = @sqrt(dx * dx + dy * dy + dz * dz),
    };
}

fn applyPairCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: ConstraintMassData,
    corr_x: f32,
    corr_y: f32,
    corr_z: f32,
) void {
    if (mass_data.inv_mass_a > 0.0) {
        inst_a.pos_x += @as(i32, @intFromFloat(@round(corr_x * mass_data.ratio_a)));
        inst_a.pos_y += @as(i32, @intFromFloat(@round(corr_y * mass_data.ratio_a)));
        inst_a.pos_z += @as(i32, @intFromFloat(@round(corr_z * mass_data.ratio_a)));
    }

    if (mass_data.inv_mass_b > 0.0) {
        inst_b.pos_x -= @as(i32, @intFromFloat(@round(corr_x * mass_data.ratio_b)));
        inst_b.pos_y -= @as(i32, @intFromFloat(@round(corr_y * mass_data.ratio_b)));
        inst_b.pos_z -= @as(i32, @intFromFloat(@round(corr_z * mass_data.ratio_b)));
    }
}

fn recordLinearWarmStart(
    warm_start: *JointWarmStart,
    corr_x: f32,
    corr_y: f32,
    corr_z: f32,
) void {
    const magnitude = @sqrt(corr_x * corr_x + corr_y * corr_y + corr_z * corr_z);
    if (magnitude <= warm_start.linear_magnitude) return;
    warm_start.linear_x = corr_x;
    warm_start.linear_y = corr_y;
    warm_start.linear_z = corr_z;
    warm_start.linear_magnitude = magnitude;
}

fn recordAngularWarmStart(warm_start: *JointWarmStart, correction: f32) void {
    if (@abs(correction) <= @abs(warm_start.angular)) return;
    warm_start.angular = correction;
}

fn recordDriveVelocityWarmStart(warm_start: *JointWarmStart, desired_velocity: f32) void {
    if (@abs(desired_velocity) <= @abs(warm_start.drive_velocity)) return;
    warm_start.drive_velocity = desired_velocity;
}

fn computeJointBreakRiskRatio(joint_def: *const Joint) f32 {
    if (joint_def.breaking_force <= 0.0) return 0.0;
    return @min(1.0, joint_def.break_accum / @max(0.0001, joint_def.breaking_force));
}

pub fn computeJointFatigueRatio(joint_def: *const Joint) f32 {
    if (joint_def.fatigue_limit <= 0.0) return 0.0;
    return @min(1.0, joint_def.fatigue_damage / @max(0.0001, joint_def.fatigue_limit));
}

pub fn computeJointTemperatureRatio(joint_def: *const Joint) f32 {
    if (joint_def.temperature_limit <= 0.0) return 0.0;
    return @min(1.0, joint_def.temperature / @max(0.0001, joint_def.temperature_limit));
}

fn computeJointWarmStartRetention(joint_def: *const Joint) f32 {
    const break_ratio = computeJointBreakRiskRatio(joint_def);
    if (break_ratio >= 0.85) return 0.9;
    if (break_ratio >= 0.5) return 0.8;
    return 0.7;
}

fn decayWarmStartValue(value: f32, retention: f32) f32 {
    const decayed = value * retention;
    if (@abs(decayed) <= 0.0001) return 0.0;
    return decayed;
}

fn prepareJointWarmStart(joint_def: *Joint) void {
    const retention = computeJointWarmStartRetention(joint_def);
    joint_def.warm_linear_x = decayWarmStartValue(joint_def.warm_linear_x, retention);
    joint_def.warm_linear_y = decayWarmStartValue(joint_def.warm_linear_y, retention);
    joint_def.warm_linear_z = decayWarmStartValue(joint_def.warm_linear_z, retention);
    joint_def.warm_angular = decayWarmStartValue(joint_def.warm_angular, retention);
    joint_def.warm_drive_velocity = decayWarmStartValue(joint_def.warm_drive_velocity, retention);
}

fn applyJointWarmStart(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    entities: []entity16.Entity16,
) void {
    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;

    const warm_linear_scale: f32 = 0.6;
    const warm_angular_scale: f32 = 0.6;
    const warm_drive_scale: f32 = 0.5;

    if (@abs(joint_def.warm_linear_x) > 0.0001 or
        @abs(joint_def.warm_linear_y) > 0.0001 or
        @abs(joint_def.warm_linear_z) > 0.0001)
    {
        applyPairCorrection(
            inst_a,
            inst_b,
            mass_data,
            joint_def.warm_linear_x * warm_linear_scale,
            joint_def.warm_linear_y * warm_linear_scale,
            joint_def.warm_linear_z * warm_linear_scale,
        );
    }

    switch (joint_def.joint_type) {
        .hinge => {
            if (@abs(joint_def.warm_angular) > 0.0001) {
                const angle_a = getJointAngle(inst_a, joint_def);
                const angle_b = getJointAngle(inst_b, joint_def);
                const correction = joint_def.warm_angular * warm_angular_scale;
                if (mass_data.inv_mass_a > 0.0) {
                    setJointAngle(inst_a, joint_def, angle_a + correction * mass_data.ratio_a);
                }
                if (mass_data.inv_mass_b > 0.0) {
                    setJointAngle(inst_b, joint_def, angle_b - correction * mass_data.ratio_b);
                }
            }
            if (@abs(joint_def.warm_drive_velocity) > 0.0001) {
                applyAngularMotorVelocityBias(
                    inst_a,
                    inst_b,
                    joint_def,
                    mass_data,
                    joint_def.warm_drive_velocity * warm_drive_scale,
                );
            }
        },
        .slider => {
            if (@abs(joint_def.warm_drive_velocity) > 0.0001) {
                const axis = normalizedAxis(joint_def) orelse return;
                applyLinearMotorVelocityBias(
                    inst_a,
                    inst_b,
                    joint_def,
                    mass_data,
                    axis.x,
                    axis.y,
                    axis.z,
                    joint_def.warm_drive_velocity * warm_drive_scale,
                );
            }
        },
        .fixed, .ball_socket, .spring, .pulley => {},
    }

    applyJointPreload(inst_a, inst_b, joint_def, mass_data);
}

fn applyJointPreload(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
) void {
    if (@abs(joint_def.preload_linear_x) <= 0.0001 and
        @abs(joint_def.preload_linear_y) <= 0.0001 and
        @abs(joint_def.preload_linear_z) <= 0.0001 and
        @abs(joint_def.preload_angular) <= 0.0001)
    {
        return;
    }

    switch (joint_def.joint_type) {
        .fixed, .ball_socket => {
            const delta = computeAnchorDelta(inst_a, inst_b, joint_def);
            const error_x = delta.dx - joint_def.preload_linear_x;
            const error_y = delta.dy - joint_def.preload_linear_y;
            const error_z = delta.dz - joint_def.preload_linear_z;
            if (@abs(error_x) <= 0.0001 and @abs(error_y) <= 0.0001 and @abs(error_z) <= 0.0001) return;
            applyPairCorrection(inst_a, inst_b, mass_data, error_x, error_y, error_z);
        },
        .hinge => {
            var preload_warm: JointWarmStart = .{};
            applyHingePreloadRotation(inst_a, inst_b, joint_def, mass_data, &preload_warm);
        },
        .slider => {
            const axis = normalizedAxis(joint_def) orelse return;
            const preload_along = joint_def.preload_linear_x * axis.x + joint_def.preload_linear_y * axis.y + joint_def.preload_linear_z * axis.z;
            if (@abs(preload_along) <= 0.0001) return;
            const delta = computeAnchorDelta(inst_a, inst_b, joint_def);
            const along = delta.dx * axis.x + delta.dy * axis.y + delta.dz * axis.z;
            const target = @max(joint_def.limit_min, @min(joint_def.limit_max, preload_along));
            const preload_error = along - target;
            if (@abs(preload_error) <= 0.0001) return;
            applyPairCorrection(
                inst_a,
                inst_b,
                mass_data,
                axis.x * preload_error,
                axis.y * preload_error,
                axis.z * preload_error,
            );
        },
        .spring, .pulley => {},
    }
}

fn storeJointWarmStart(joint_def: *Joint, warm_start: JointWarmStart) void {
    joint_def.warm_linear_x = warm_start.linear_x;
    joint_def.warm_linear_y = warm_start.linear_y;
    joint_def.warm_linear_z = warm_start.linear_z;
    joint_def.warm_angular = warm_start.angular;
    joint_def.warm_drive_velocity = warm_start.drive_velocity;
}

fn normalizedAxis(joint_def: *const Joint) ?struct { x: f32, y: f32, z: f32 } {
    const ax = @as(f32, @floatFromInt(joint_def.axis_x));
    const ay = @as(f32, @floatFromInt(joint_def.axis_y));
    const az = @as(f32, @floatFromInt(joint_def.axis_z));
    const len = @sqrt(ax * ax + ay * ay + az * az);
    if (len <= 0.0001) return null;
    return .{
        .x = ax / len,
        .y = ay / len,
        .z = az / len,
    };
}

fn dominantAxis(joint_def: *const Joint) JointAxis {
    const abs_x = @abs(joint_def.axis_x);
    const abs_y = @abs(joint_def.axis_y);
    const abs_z = @abs(joint_def.axis_z);
    if (abs_x >= abs_y and abs_x >= abs_z) return .x;
    if (abs_y >= abs_z) return .y;
    return .z;
}

fn rotationByteToRadians(value: u8) f32 {
    const signed: i8 = @bitCast(value);
    return @as(f32, @floatFromInt(signed)) * (std.math.pi / 128.0);
}

fn radiansToRotationByte(value: f32) u8 {
    const scaled = value * (128.0 / std.math.pi);
    const signed = @as(i8, @intFromFloat(@round(@max(-128.0, @min(127.0, scaled)))));
    return @bitCast(signed);
}

fn jointConstraintMagnitude(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const Joint,
) f32 {
    const delta = computeAnchorDelta(inst_a, inst_b, joint_def);
    return switch (joint_def.joint_type) {
        .fixed => @max(delta.dist, fixedRelativeAngularError(inst_a, inst_b)),
        .ball_socket => delta.dist,
        .hinge => @max(delta.dist, hingeSwingAngularError(inst_a, inst_b, joint_def)),
        .slider => blk: {
            const axis = normalizedAxis(joint_def) orelse break :blk delta.dist;
            const along = delta.dx * axis.x + delta.dy * axis.y + delta.dz * axis.z;
            const perp_x = delta.dx - axis.x * along;
            const perp_y = delta.dy - axis.y * along;
            const perp_z = delta.dz - axis.z * along;
            const perp = @sqrt(perp_x * perp_x + perp_y * perp_y + perp_z * perp_z);
            const clamped = @max(joint_def.limit_min, @min(joint_def.limit_max, along));
            break :blk @max(perp + @abs(along - clamped), fixedRelativeAngularError(inst_a, inst_b));
        },
        .spring => blk: {
            const rest_length = @max(0.0, joint_def.limit_max);
            break :blk @abs(delta.dist - rest_length) * @max(1.0, joint_def.stiffness);
        },
        .pulley => blk: {
            const state = measurePulleyState(inst_a, inst_b, joint_def) orelse break :blk delta.dist;
            break :blk @abs(state.pulley_error);
        },
    };
}

const JointConstraintMetrics = struct {
    geometry_error: f32,
    limit_error: f32,
    drive_error: f32,
    residual_speed: f32,

    fn stress(self: JointConstraintMetrics) f32 {
        return @max(self.geometry_error, @max(self.limit_error, @max(self.drive_error, self.residual_speed * 0.25)));
    }
};

fn measureJointConstraintMetrics(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const Joint,
) JointConstraintMetrics {
    var metrics = JointConstraintMetrics{
        .geometry_error = jointConstraintMagnitude(inst_a, inst_b, joint_def),
        .limit_error = 0.0,
        .drive_error = 0.0,
        .residual_speed = 0.0,
    };

    switch (joint_def.joint_type) {
        .hinge => {
            const relative_angle = getJointAngle(inst_b, joint_def) - getJointAngle(inst_a, joint_def);
            const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
            const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
            if (relative_angle < min_angle) metrics.limit_error = @max(metrics.limit_error, @abs(relative_angle - min_angle));
            if (relative_angle > max_angle) metrics.limit_error = @max(metrics.limit_error, @abs(relative_angle - max_angle));
            metrics.limit_error = @max(metrics.limit_error, hingeSwingAngularError(inst_a, inst_b, joint_def));

            metrics.residual_speed = switch (dominantAxis(joint_def)) {
                .x => @abs(@as(f32, @floatFromInt(inst_b.ang_x - inst_a.ang_x))),
                .y => @abs(@as(f32, @floatFromInt(inst_b.ang_y - inst_a.ang_y))),
                .z => @abs(@as(f32, @floatFromInt(inst_b.ang_z - inst_a.ang_z))),
            };
            metrics.residual_speed = @max(metrics.residual_speed, hingeSwingAngularSpeed(inst_a, inst_b, joint_def));

            if (joint_def.motor_enabled) {
                if (measureJointDriveState(inst_a, inst_b, joint_def)) |drive_state| {
                    metrics.drive_error = @abs(joint_def.motor_target - drive_state.position);
                    metrics.residual_speed = @max(metrics.residual_speed, @abs(drive_state.relative_velocity));
                }
            }
        },
        .slider => {
            if (measureJointDriveState(inst_a, inst_b, joint_def)) |drive_state| {
                metrics.residual_speed = @abs(drive_state.relative_velocity);
                const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
                const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
                if (drive_state.position < min_limit) metrics.limit_error = @abs(drive_state.position - min_limit);
                if (drive_state.position > max_limit) metrics.limit_error = @abs(drive_state.position - max_limit);
                metrics.limit_error = @max(metrics.limit_error, fixedRelativeAngularError(inst_a, inst_b));
                metrics.residual_speed = @max(metrics.residual_speed, fixedRelativeAngularSpeed(inst_a, inst_b));
                if (joint_def.motor_enabled) {
                    metrics.drive_error = @abs(joint_def.motor_target - drive_state.position);
                }
            }
        },
        .fixed, .ball_socket, .spring => {
            const delta = computeAnchorDelta(inst_a, inst_b, joint_def);
            if (delta.dist > 0.0001) {
                const nx = delta.dx / delta.dist;
                const ny = delta.dy / delta.dist;
                const nz = delta.dz / delta.dist;
                const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
                const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
                const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
                metrics.residual_speed = @abs(rel_vel_x * nx + rel_vel_y * ny + rel_vel_z * nz);
            }
            if (joint_def.joint_type == .fixed) {
                metrics.limit_error = fixedRelativeAngularError(inst_a, inst_b);
                metrics.residual_speed = @max(metrics.residual_speed, fixedRelativeAngularSpeed(inst_a, inst_b));
            }
        },
        .pulley => {
            if (measurePulleyState(inst_a, inst_b, joint_def)) |state| {
                metrics.geometry_error = @abs(state.pulley_error);
                metrics.residual_speed = @abs(state.constraint_speed);
            }
        },
    }

    return metrics;
}

fn makeJointStressSample(joint_def: *const Joint, metrics: JointConstraintMetrics) JointStressSample {
    return .{
        .stress = computeJointStressWithBreakMemory(joint_def, metrics),
        .geometry_error = metrics.geometry_error,
        .limit_error = metrics.limit_error,
        .drive_error = metrics.drive_error,
        .residual_speed = metrics.residual_speed,
        .break_ratio = computeJointBreakRiskRatio(joint_def),
        .fatigue_ratio = computeJointFatigueRatio(joint_def),
        .temperature_ratio = computeJointTemperatureRatio(joint_def),
    };
}

fn computeJointStressWithBreakMemory(joint_def: *const Joint, metrics: JointConstraintMetrics) f32 {
    const base_stress = metrics.stress();
    if (joint_def.breaking_force <= 0.0) return base_stress;
    return @max(base_stress, joint_def.breaking_force * computeJointBreakRiskRatio(joint_def));
}

fn enforceJointBreak(joint_def: *Joint, inst_a: *const scene32.Instance, inst_b: *const scene32.Instance) bool {
    if (joint_def.breaking_force <= 0.0) return false;
    const stress = measureJointConstraintMetrics(inst_a, inst_b, joint_def).stress();
    if (stress >= joint_def.breaking_force) {
        joint_def.enabled = false;
        joint_def.break_accum = 0.0;
        return true;
    }

    const accumulate_floor = joint_def.breaking_force * 0.5;
    if (stress >= accumulate_floor) {
        const overload_ratio = (stress - accumulate_floor) / @max(0.0001, joint_def.breaking_force - accumulate_floor);
        joint_def.break_accum = @min(
            joint_def.breaking_force * 2.0,
            joint_def.break_accum * 0.95 + overload_ratio * joint_def.breaking_force * 0.7,
        );
    } else {
        joint_def.break_accum *= 0.6;
    }

    if (joint_def.break_accum < joint_def.breaking_force) return false;
    joint_def.enabled = false;
    joint_def.break_accum = 0.0;
    return true;
}

fn updateJointFatigue(joint_def: *Joint, inst_a: *const scene32.Instance, inst_b: *const scene32.Instance) bool {
    if (joint_def.fatigue_limit <= 0.0) return false;
    const stress = measureJointConstraintMetrics(inst_a, inst_b, joint_def).stress();
    const normalized_stress = if (joint_def.breaking_force > 0.0)
        stress / @max(0.0001, joint_def.breaking_force)
    else
        stress;

    if (normalized_stress > 0.25) {
        const overload = normalized_stress - 0.25;
        joint_def.fatigue_damage = @min(
            joint_def.fatigue_limit * 2.0,
            joint_def.fatigue_damage + overload * @max(0.0, joint_def.fatigue_rate),
        );
    } else {
        joint_def.fatigue_damage = @max(0.0, joint_def.fatigue_damage - @max(0.0, joint_def.fatigue_recovery));
    }

    if (joint_def.fatigue_damage < joint_def.fatigue_limit) return false;
    joint_def.enabled = false;
    return true;
}

fn updateJointTemperature(joint_def: *Joint, inst_a: *const scene32.Instance, inst_b: *const scene32.Instance) bool {
    if (joint_def.temperature_limit <= 0.0) return false;
    const metrics = measureJointConstraintMetrics(inst_a, inst_b, joint_def);
    const stress = metrics.stress();
    const normalized_stress = if (joint_def.breaking_force > 0.0)
        stress / @max(0.0001, joint_def.breaking_force)
    else
        stress;

    const heat_gain = @max(0.0, normalized_stress) * @max(0.0, joint_def.temperature_rate);
    const cooling = @max(0.0, joint_def.temperature) * @max(0.0, joint_def.temperature_cooling);
    joint_def.temperature = @max(0.0, joint_def.temperature + heat_gain - cooling);

    if (joint_def.temperature < joint_def.temperature_limit) return false;
    joint_def.enabled = false;
    return true;
}

fn clampHingeRelativeRotation(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
    warm_start: *JointWarmStart,
) void {
    const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
    const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
    const angle_a = getJointAngle(inst_a, joint_def);
    const angle_b = getJointAngle(inst_b, joint_def);
    const relative_angle = angle_b - angle_a;
    const clamped_relative = @max(min_angle, @min(max_angle, relative_angle));
    const angle_error = relative_angle - clamped_relative;
    if (@abs(angle_error) <= 0.0001) return;
    recordAngularWarmStart(warm_start, angle_error);

    if (mass_data.inv_mass_a > 0.0) {
        setJointAngle(inst_a, joint_def, angle_a + angle_error * mass_data.ratio_a);
    }
    if (mass_data.inv_mass_b > 0.0) {
        setJointAngle(inst_b, joint_def, angle_b - angle_error * mass_data.ratio_b);
    }
}

fn getJointAngle(inst: *const scene32.Instance, joint_def: *const Joint) f32 {
    return getJointAngleForAxis(inst, dominantAxis(joint_def));
}

fn getJointAngleForAxis(inst: *const scene32.Instance, axis: JointAxis) f32 {
    return switch (axis) {
        .x => rotationByteToRadians(inst.rot_roll),
        .y => rotationByteToRadians(inst.rot_yaw),
        .z => rotationByteToRadians(inst.rot_pitch),
    };
}

fn setJointAngle(inst: *scene32.Instance, joint_def: *const Joint, angle: f32) void {
    setJointAngleForAxis(inst, dominantAxis(joint_def), angle);
}

fn setJointAngleForAxis(inst: *scene32.Instance, axis: JointAxis, angle: f32) void {
    switch (axis) {
        .x => inst.rot_roll = radiansToRotationByte(angle),
        .y => inst.rot_yaw = radiansToRotationByte(angle),
        .z => inst.rot_pitch = radiansToRotationByte(angle),
    }
}

fn getJointAngularVelocityForAxis(inst: *const scene32.Instance, axis: JointAxis) f32 {
    return switch (axis) {
        .x => @as(f32, @floatFromInt(inst.ang_x)),
        .y => @as(f32, @floatFromInt(inst.ang_y)),
        .z => @as(f32, @floatFromInt(inst.ang_z)),
    };
}

fn applyJointAngularVelocityDelta(inst: *scene32.Instance, axis: JointAxis, delta: i8) void {
    switch (axis) {
        .x => inst.ang_x += delta,
        .y => inst.ang_y += delta,
        .z => inst.ang_z += delta,
    }
}

fn fixedRelativeAngularError(inst_a: *const scene32.Instance, inst_b: *const scene32.Instance) f32 {
    var max_error: f32 = 0.0;
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        max_error = @max(max_error, @abs(getJointAngleForAxis(inst_b, axis) - getJointAngleForAxis(inst_a, axis)));
    }
    return max_error;
}

fn fixedRelativeAngularSpeed(inst_a: *const scene32.Instance, inst_b: *const scene32.Instance) f32 {
    var max_speed: f32 = 0.0;
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        max_speed = @max(
            max_speed,
            @abs(getJointAngularVelocityForAxis(inst_b, axis) - getJointAngularVelocityForAxis(inst_a, axis)),
        );
    }
    return max_speed;
}

fn hingeSwingAngularError(inst_a: *const scene32.Instance, inst_b: *const scene32.Instance, joint_def: *const Joint) f32 {
    const free_axis = dominantAxis(joint_def);
    var max_error: f32 = 0.0;
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        if (axis != free_axis) {
            max_error = @max(max_error, @abs(getJointAngleForAxis(inst_b, axis) - getJointAngleForAxis(inst_a, axis)));
        }
    }
    return max_error;
}

fn hingeSwingAngularSpeed(inst_a: *const scene32.Instance, inst_b: *const scene32.Instance, joint_def: *const Joint) f32 {
    const free_axis = dominantAxis(joint_def);
    var max_speed: f32 = 0.0;
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        if (axis != free_axis) {
            max_speed = @max(
                max_speed,
                @abs(getJointAngularVelocityForAxis(inst_b, axis) - getJointAngularVelocityForAxis(inst_a, axis)),
            );
        }
    }
    return max_speed;
}

pub fn measureJointDriveState(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const Joint,
) ?JointDriveState {
    return switch (joint_def.joint_type) {
        .hinge => .{
            .position = getJointAngle(inst_b, joint_def) - getJointAngle(inst_a, joint_def),
            .relative_velocity = switch (dominantAxis(joint_def)) {
                .x => @as(f32, @floatFromInt(inst_b.ang_x - inst_a.ang_x)),
                .y => @as(f32, @floatFromInt(inst_b.ang_y - inst_a.ang_y)),
                .z => @as(f32, @floatFromInt(inst_b.ang_z - inst_a.ang_z)),
            },
        },
        .slider => blk: {
            const axis = normalizedAxis(joint_def) orelse return null;
            const delta = computeAnchorDelta(inst_a, inst_b, joint_def);
            break :blk .{
                .position = delta.dx * axis.x + delta.dy * axis.y + delta.dz * axis.z,
                .relative_velocity = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x)) * axis.x +
                    @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y)) * axis.y +
                    @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z)) * axis.z,
            };
        },
        else => null,
    };
}

fn computeJointDrivePlan(
    joint_def: *const Joint,
    drive_state: JointDriveState,
    min_step: f32,
) ?JointDrivePlan {
    const prediction_dt = 1.0 / 60.0;
    const max_drive_velocity = @max(1.0, joint_def.motor_speed);
    const min_limit = @min(joint_def.limit_min, joint_def.limit_max);
    const max_limit = @max(joint_def.limit_min, joint_def.limit_max);
    const target = @max(min_limit, @min(max_limit, joint_def.motor_target));
    const position_error = target - drive_state.position;
    if (@abs(position_error) <= 0.0001) {
        if (@abs(drive_state.relative_velocity) <= 0.0001) return null;
        return .{
            .target = target,
            .position_error = position_error,
            .predicted_position = drive_state.position + drive_state.relative_velocity * prediction_dt,
            .predicted_error = target - (drive_state.position + drive_state.relative_velocity * prediction_dt),
            .signed_step = 0.0,
            .desired_velocity = 0.0,
        };
    }

    const predicted_position = drive_state.position + drive_state.relative_velocity * prediction_dt;
    const predicted_error = target - predicted_position;
    if (@abs(predicted_error) <= 0.0001) {
        return .{
            .target = target,
            .position_error = position_error,
            .predicted_position = predicted_position,
            .predicted_error = predicted_error,
            .signed_step = 0.0,
            .desired_velocity = 0.0,
        };
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
        .target = target,
        .position_error = position_error,
        .predicted_position = predicted_position,
        .predicted_error = predicted_error,
        .signed_step = signed_step,
        .desired_velocity = desired_velocity,
    };
}

fn clampI8FromF32(value: f32) i8 {
    const clamped = @max(-128.0, @min(127.0, value));
    if (clamped > 0.0 and clamped < 1.0) return 1;
    if (clamped < 0.0 and clamped > -1.0) return -1;
    return @as(i8, @intFromFloat(@round(clamped)));
}

fn clampI16FromF32(value: f32) i16 {
    const clamped = @max(-32768.0, @min(32767.0, value));
    if (clamped > 0.0 and clamped < 1.0) return 1;
    if (clamped < 0.0 and clamped > -1.0) return -1;
    return @as(i16, @intFromFloat(@round(clamped)));
}

fn applyAngularMotorVelocityBias(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
    desired_velocity: f32,
) void {
    const drive_state = measureJointDriveState(inst_a, inst_b, joint_def) orelse return;
    const current_rel_velocity = drive_state.relative_velocity;
    if (desired_velocity != 0.0 and current_rel_velocity != 0.0) {
        if ((desired_velocity > 0.0 and current_rel_velocity > 0.0 and current_rel_velocity >= desired_velocity) or
            (desired_velocity < 0.0 and current_rel_velocity < 0.0 and current_rel_velocity <= desired_velocity))
        {
            return;
        }
    }
    const angular_bias = clampI8FromF32(@max(-32.0, @min(32.0, desired_velocity - current_rel_velocity)));
    if (angular_bias == 0) return;

    switch (dominantAxis(joint_def)) {
        .x => {
            if (mass_data.inv_mass_a > 0.0) inst_a.ang_x -= clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
            if (mass_data.inv_mass_b > 0.0) inst_b.ang_x += clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
        },
        .y => {
            if (mass_data.inv_mass_a > 0.0) inst_a.ang_y -= clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
            if (mass_data.inv_mass_b > 0.0) inst_b.ang_y += clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
        },
        .z => {
            if (mass_data.inv_mass_a > 0.0) inst_a.ang_z -= clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
            if (mass_data.inv_mass_b > 0.0) inst_b.ang_z += clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
        },
    }
}

fn applyLinearMotorVelocityBias(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    desired_speed: f32,
) void {
    const drive_state = measureJointDriveState(inst_a, inst_b, joint_def) orelse return;
    const current_rel_speed = drive_state.relative_velocity;
    if (desired_speed != 0.0 and current_rel_speed != 0.0) {
        if ((desired_speed > 0.0 and current_rel_speed > 0.0 and current_rel_speed >= desired_speed) or
            (desired_speed < 0.0 and current_rel_speed < 0.0 and current_rel_speed <= desired_speed))
        {
            return;
        }
    }
    const vel_bias = @max(-64.0, @min(64.0, desired_speed - current_rel_speed));

    if (mass_data.inv_mass_a > 0.0) {
        inst_a.vel_x -= clampI16FromF32(axis_x * vel_bias * mass_data.ratio_a);
        inst_a.vel_y -= clampI16FromF32(axis_y * vel_bias * mass_data.ratio_a);
        inst_a.vel_z -= clampI16FromF32(axis_z * vel_bias * mass_data.ratio_a);
    }
    if (mass_data.inv_mass_b > 0.0) {
        inst_b.vel_x += clampI16FromF32(axis_x * vel_bias * mass_data.ratio_b);
        inst_b.vel_y += clampI16FromF32(axis_y * vel_bias * mass_data.ratio_b);
        inst_b.vel_z += clampI16FromF32(axis_z * vel_bias * mass_data.ratio_b);
    }
}

fn applyHingeLimitVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
) void {
    const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
    const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
    const angle_tolerance: f32 = 0.05;
    const relative_angle = getJointAngle(inst_b, joint_def) - getJointAngle(inst_a, joint_def);
    const relative_velocity = switch (dominantAxis(joint_def)) {
        .x => @as(f32, @floatFromInt(inst_b.ang_x - inst_a.ang_x)),
        .y => @as(f32, @floatFromInt(inst_b.ang_y - inst_a.ang_y)),
        .z => @as(f32, @floatFromInt(inst_b.ang_z - inst_a.ang_z)),
    };

    const pushes_past_min = relative_angle <= min_angle + angle_tolerance and relative_velocity < 0.0;
    const pushes_past_max = relative_angle >= max_angle - angle_tolerance and relative_velocity > 0.0;
    if (!pushes_past_min and !pushes_past_max) return;

    const angular_bias = clampI8FromF32(@max(-32.0, @min(32.0, -relative_velocity)));
    if (angular_bias == 0) return;

    switch (dominantAxis(joint_def)) {
        .x => {
            if (mass_data.inv_mass_a > 0.0) inst_a.ang_x -= clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
            if (mass_data.inv_mass_b > 0.0) inst_b.ang_x += clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
        },
        .y => {
            if (mass_data.inv_mass_a > 0.0) inst_a.ang_y -= clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
            if (mass_data.inv_mass_b > 0.0) inst_b.ang_y += clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
        },
        .z => {
            if (mass_data.inv_mass_a > 0.0) inst_a.ang_z -= clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a);
            if (mass_data.inv_mass_b > 0.0) inst_b.ang_z += clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b);
        },
    }
}

fn applySliderLimitVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: ConstraintMassData,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    along: f32,
    min_limit: f32,
    max_limit: f32,
) void {
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_speed = rel_vel_x * axis_x + rel_vel_y * axis_y + rel_vel_z * axis_z;

    const pushes_past_min = along <= min_limit + 0.0001 and rel_speed < 0.0;
    const pushes_past_max = along >= max_limit - 0.0001 and rel_speed > 0.0;
    if (!pushes_past_min and !pushes_past_max) return;

    const vel_bias = @max(-64.0, @min(64.0, -rel_speed));
    if (vel_bias == 0.0) return;

    if (mass_data.inv_mass_a > 0.0) {
        inst_a.vel_x -= clampI16FromF32(axis_x * vel_bias * mass_data.ratio_a);
        inst_a.vel_y -= clampI16FromF32(axis_y * vel_bias * mass_data.ratio_a);
        inst_a.vel_z -= clampI16FromF32(axis_z * vel_bias * mass_data.ratio_a);
    }
    if (mass_data.inv_mass_b > 0.0) {
        inst_b.vel_x += clampI16FromF32(axis_x * vel_bias * mass_data.ratio_b);
        inst_b.vel_y += clampI16FromF32(axis_y * vel_bias * mass_data.ratio_b);
        inst_b.vel_z += clampI16FromF32(axis_z * vel_bias * mass_data.ratio_b);
    }
}

fn applyHingeMotor(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    if (!joint_def.motor_enabled) return;

    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    const current_a = getJointAngle(inst_a, joint_def);
    const current_b = getJointAngle(inst_b, joint_def);
    const drive_state = measureJointDriveState(inst_a, inst_b, joint_def) orelse return;
    const drive_plan = computeJointDrivePlan(joint_def, drive_state, 0.001) orelse return;
    recordDriveVelocityWarmStart(warm_start, drive_plan.desired_velocity);

    if (drive_plan.signed_step != 0.0 and mass_data.inv_mass_a > 0.0) {
        setJointAngle(inst_a, joint_def, current_a - drive_plan.signed_step * mass_data.ratio_a);
    }
    if (drive_plan.signed_step != 0.0 and mass_data.inv_mass_b > 0.0) {
        setJointAngle(inst_b, joint_def, current_b + drive_plan.signed_step * mass_data.ratio_b);
    }
    applyAngularMotorVelocityBias(inst_a, inst_b, joint_def, mass_data, drive_plan.desired_velocity);
}

fn applySliderMotor(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    entities: []entity16.Entity16,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    warm_start: *JointWarmStart,
) void {
    if (!joint_def.motor_enabled) return;

    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    const drive_state = measureJointDriveState(inst_a, inst_b, joint_def) orelse return;
    const quantized_min_step: f32 = if (@abs(drive_state.position - joint_def.motor_target) >= 1.0) 1.0 else 0.001;
    const drive_plan = computeJointDrivePlan(joint_def, drive_state, quantized_min_step) orelse return;
    recordDriveVelocityWarmStart(warm_start, drive_plan.desired_velocity);

    if (drive_plan.signed_step != 0.0) {
        recordLinearWarmStart(
            warm_start,
            -axis_x * drive_plan.signed_step,
            -axis_y * drive_plan.signed_step,
            -axis_z * drive_plan.signed_step,
        );
        applyPairCorrection(
            inst_a,
            inst_b,
            mass_data,
            -axis_x * drive_plan.signed_step,
            -axis_y * drive_plan.signed_step,
            -axis_z * drive_plan.signed_step,
        );
    }
    applyLinearMotorVelocityBias(inst_a, inst_b, joint_def, mass_data, axis_x, axis_y, axis_z, drive_plan.desired_velocity);
}

/// Solve distance constraint (Fixed joint, BallSocket)
pub fn solveDistanceConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    const delta = computeAnchorDelta(inst_a, inst_b, joint);
    const error_x = delta.dx - joint.preload_linear_x;
    const error_y = delta.dy - joint.preload_linear_y;
    const error_z = delta.dz - joint.preload_linear_z;
    const error_dist = @sqrt(error_x * error_x + error_y * error_y + error_z * error_z);
    if (error_dist <= 0.0001) return;
    recordLinearWarmStart(warm_start, error_x, error_y, error_z);
    applyPairCorrection(inst_a, inst_b, mass_data, error_x, error_y, error_z);

    const nx = error_x / error_dist;
    const ny = error_y / error_dist;
    const nz = error_z / error_dist;
    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_speed = rel_vel_x * nx + rel_vel_y * ny + rel_vel_z * nz;
    if (@abs(rel_speed) <= 0.0001) return;

    const damping = @max(0.05, @min(1.0, @max(0.0, joint.damping) * 0.001));
    const damping_impulse = rel_speed * damping;
    if (mass_data.inv_mass_a > 0.0) {
        inst_a.vel_x += clampI16FromF32(nx * damping_impulse * mass_data.ratio_a);
        inst_a.vel_y += clampI16FromF32(ny * damping_impulse * mass_data.ratio_a);
        inst_a.vel_z += clampI16FromF32(nz * damping_impulse * mass_data.ratio_a);
    }
    if (mass_data.inv_mass_b > 0.0) {
        inst_b.vel_x -= clampI16FromF32(nx * damping_impulse * mass_data.ratio_b);
        inst_b.vel_y -= clampI16FromF32(ny * damping_impulse * mass_data.ratio_b);
        inst_b.vel_z -= clampI16FromF32(nz * damping_impulse * mass_data.ratio_b);
    }
}

fn applyAngularVelocityBiasForAxis(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: ConstraintMassData,
    axis: JointAxis,
    signed_bias: f32,
) void {
    const angular_bias = clampI8FromF32(@max(-32.0, @min(32.0, signed_bias)));
    if (angular_bias == 0) return;

    if (mass_data.inv_mass_a > 0.0) {
        applyJointAngularVelocityDelta(
            inst_a,
            axis,
            -clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_a),
        );
    }
    if (mass_data.inv_mass_b > 0.0) {
        applyJointAngularVelocityDelta(
            inst_b,
            axis,
            clampI8FromF32(@as(f32, @floatFromInt(angular_bias)) * mass_data.ratio_b),
        );
    }
}

fn lockFixedRelativeRotation(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: ConstraintMassData,
    warm_start: *JointWarmStart,
) void {
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        const angle_a = getJointAngleForAxis(inst_a, axis);
        const angle_b = getJointAngleForAxis(inst_b, axis);
        const angle_error = angle_b - angle_a;
        if (@abs(angle_error) > 0.0001) {
            recordAngularWarmStart(warm_start, angle_error);

            if (mass_data.inv_mass_a > 0.0) {
                setJointAngleForAxis(inst_a, axis, angle_a + angle_error * mass_data.ratio_a);
            }
            if (mass_data.inv_mass_b > 0.0) {
                setJointAngleForAxis(inst_b, axis, angle_b - angle_error * mass_data.ratio_b);
            }
        }
    }
}

fn dampFixedRelativeAngularVelocity(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
) void {
    const damping = @min(1.0, @max(0.0, joint_def.damping) * 0.001);
    if (damping <= 0.0) return;
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        const relative_velocity = getJointAngularVelocityForAxis(inst_b, axis) -
            getJointAngularVelocityForAxis(inst_a, axis);
        if (@abs(relative_velocity) > 0.0001) {
            applyAngularVelocityBiasForAxis(inst_a, inst_b, mass_data, axis, -relative_velocity * damping);
        }
    }
}

fn lockHingeSwingRotation(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
    warm_start: *JointWarmStart,
) void {
    const free_axis = dominantAxis(joint_def);
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        if (axis != free_axis) {
            const angle_a = getJointAngleForAxis(inst_a, axis);
            const angle_b = getJointAngleForAxis(inst_b, axis);
            const angle_error = angle_b - angle_a;
            if (@abs(angle_error) > 0.0001) {
                recordAngularWarmStart(warm_start, angle_error);

                if (mass_data.inv_mass_a > 0.0) {
                    setJointAngleForAxis(inst_a, axis, angle_a + angle_error * mass_data.ratio_a);
                }
                if (mass_data.inv_mass_b > 0.0) {
                    setJointAngleForAxis(inst_b, axis, angle_b - angle_error * mass_data.ratio_b);
                }
            }
        }
    }
}

fn dampHingeSwingAngularVelocity(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
) void {
    const damping = @min(1.0, @max(0.0, joint_def.damping) * 0.001);
    if (damping <= 0.0) return;
    const free_axis = dominantAxis(joint_def);
    inline for (.{ JointAxis.x, JointAxis.y, JointAxis.z }) |axis| {
        if (axis != free_axis) {
            const relative_velocity = getJointAngularVelocityForAxis(inst_b, axis) -
                getJointAngularVelocityForAxis(inst_a, axis);
            if (@abs(relative_velocity) > 0.0001) {
                applyAngularVelocityBiasForAxis(inst_a, inst_b, mass_data, axis, -relative_velocity * damping);
            }
        }
    }
}

fn applyHingePreloadRotation(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *const Joint,
    mass_data: ConstraintMassData,
    warm_start: *JointWarmStart,
) void {
    if (@abs(joint_def.preload_angular) <= 0.0001) return;
    const min_angle = @min(joint_def.limit_min, joint_def.limit_max);
    const max_angle = @max(joint_def.limit_min, joint_def.limit_max);
    const target_angle = @max(min_angle, @min(max_angle, joint_def.preload_angular));
    const angle_a = getJointAngle(inst_a, joint_def);
    const angle_b = getJointAngle(inst_b, joint_def);
    const angle_error = (angle_b - angle_a) - target_angle;
    if (@abs(angle_error) <= 0.0001) return;
    recordAngularWarmStart(warm_start, angle_error);

    if (mass_data.inv_mass_a > 0.0) {
        setJointAngle(inst_a, joint_def, angle_a + angle_error * mass_data.ratio_a);
    }
    if (mass_data.inv_mass_b > 0.0) {
        setJointAngle(inst_b, joint_def, angle_b - angle_error * mass_data.ratio_b);
    }
}

fn clampSpringCorrectionMagnitude(control_extension: f32, stiffness: f32) f32 {
    const requested = control_extension * stiffness;
    const max_correction = @abs(control_extension);
    return @max(-max_correction, @min(max_correction, requested));
}

const PulleyState = struct {
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
    ratio: f32,
    pulley_error: f32,
    constraint_speed: f32,
};

fn pulleyRatio(joint_def: *const Joint) f32 {
    const configured = @abs(joint_def.limit_min);
    return if (configured > 0.0001) configured else 1.0;
}

fn pulleyCoordinate(
    inst: *const scene32.Instance,
    anchor_x: i32,
    anchor_y: i32,
    anchor_z: i32,
    axis_x: f32,
    axis_y: f32,
    axis_z: f32,
) f32 {
    return @as(f32, @floatFromInt(inst.pos_x + anchor_x)) * axis_x +
        @as(f32, @floatFromInt(inst.pos_y + anchor_y)) * axis_y +
        @as(f32, @floatFromInt(inst.pos_z + anchor_z)) * axis_z;
}

fn measurePulleyState(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const Joint,
) ?PulleyState {
    const axis = normalizedAxis(joint_def) orelse return null;
    const ratio = pulleyRatio(joint_def);
    const coord_a = pulleyCoordinate(inst_a, joint_def.anchor_a_x, joint_def.anchor_a_y, joint_def.anchor_a_z, axis.x, axis.y, axis.z);
    const coord_b = pulleyCoordinate(inst_b, joint_def.anchor_b_x, joint_def.anchor_b_y, joint_def.anchor_b_z, axis.x, axis.y, axis.z);
    const rest = joint_def.limit_max;
    const rel_speed_a = @as(f32, @floatFromInt(inst_a.vel_x)) * axis.x +
        @as(f32, @floatFromInt(inst_a.vel_y)) * axis.y +
        @as(f32, @floatFromInt(inst_a.vel_z)) * axis.z;
    const rel_speed_b = @as(f32, @floatFromInt(inst_b.vel_x)) * axis.x +
        @as(f32, @floatFromInt(inst_b.vel_y)) * axis.y +
        @as(f32, @floatFromInt(inst_b.vel_z)) * axis.z;
    return .{
        .axis_x = axis.x,
        .axis_y = axis.y,
        .axis_z = axis.z,
        .ratio = ratio,
        .pulley_error = coord_a + ratio * coord_b - rest,
        .constraint_speed = rel_speed_a + ratio * rel_speed_b,
    };
}

fn applyPulleyCorrection(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: ConstraintMassData,
    state: PulleyState,
) void {
    const denom = mass_data.inv_mass_a + state.ratio * state.ratio * mass_data.inv_mass_b;
    if (denom <= 0.0001 or @abs(state.pulley_error) <= 0.0001) return;
    const impulse = state.pulley_error / denom;
    if (mass_data.inv_mass_a > 0.0) {
        inst_a.pos_x -= @as(i32, @intFromFloat(@round(state.axis_x * impulse * mass_data.inv_mass_a)));
        inst_a.pos_y -= @as(i32, @intFromFloat(@round(state.axis_y * impulse * mass_data.inv_mass_a)));
        inst_a.pos_z -= @as(i32, @intFromFloat(@round(state.axis_z * impulse * mass_data.inv_mass_a)));
    }
    if (mass_data.inv_mass_b > 0.0) {
        inst_b.pos_x -= @as(i32, @intFromFloat(@round(state.axis_x * state.ratio * impulse * mass_data.inv_mass_b)));
        inst_b.pos_y -= @as(i32, @intFromFloat(@round(state.axis_y * state.ratio * impulse * mass_data.inv_mass_b)));
        inst_b.pos_z -= @as(i32, @intFromFloat(@round(state.axis_z * state.ratio * impulse * mass_data.inv_mass_b)));
    }
}

fn applyPulleyVelocityDamping(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    mass_data: ConstraintMassData,
    state: PulleyState,
    damping: f32,
) void {
    const denom = mass_data.inv_mass_a + state.ratio * state.ratio * mass_data.inv_mass_b;
    if (denom <= 0.0001 or @abs(state.constraint_speed) <= 0.0001 or damping <= 0.0) return;
    const impulse = state.constraint_speed * damping / denom;
    if (mass_data.inv_mass_a > 0.0) {
        inst_a.vel_x -= clampI16FromF32(state.axis_x * impulse * mass_data.inv_mass_a);
        inst_a.vel_y -= clampI16FromF32(state.axis_y * impulse * mass_data.inv_mass_a);
        inst_a.vel_z -= clampI16FromF32(state.axis_z * impulse * mass_data.inv_mass_a);
    }
    if (mass_data.inv_mass_b > 0.0) {
        inst_b.vel_x -= clampI16FromF32(state.axis_x * state.ratio * impulse * mass_data.inv_mass_b);
        inst_b.vel_y -= clampI16FromF32(state.axis_y * state.ratio * impulse * mass_data.inv_mass_b);
        inst_b.vel_z -= clampI16FromF32(state.axis_z * state.ratio * impulse * mass_data.inv_mass_b);
    }
}

/// Solve fixed joint constraint (locked translation and relative orientation)
pub fn solveFixedConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    solveDistanceConstraint(inst_a, inst_b, joint, entities, warm_start);
    lockFixedRelativeRotation(inst_a, inst_b, mass_data, warm_start);
    dampFixedRelativeAngularVelocity(inst_a, inst_b, joint, mass_data);
}

/// Solve hinge constraint (rotation around axis only)
pub fn solveHingeConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    solveDistanceConstraint(inst_a, inst_b, joint, entities, warm_start);
    lockHingeSwingRotation(inst_a, inst_b, joint, mass_data, warm_start);
    applyHingePreloadRotation(inst_a, inst_b, joint, mass_data, warm_start);
    applyHingeMotor(inst_a, inst_b, joint, entities, warm_start);
    clampHingeRelativeRotation(inst_a, inst_b, joint, mass_data, warm_start);
    applyHingeLimitVelocityDamping(inst_a, inst_b, joint, mass_data);
    dampHingeSwingAngularVelocity(inst_a, inst_b, joint, mass_data);
}

/// Solve slider constraint (translation along axis only)
pub fn solveSliderConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    const axis = normalizedAxis(joint) orelse return;
    const delta = computeAnchorDelta(inst_a, inst_b, joint);

    const along = delta.dx * axis.x + delta.dy * axis.y + delta.dz * axis.z;
    const perp_x = delta.dx - axis.x * along;
    const perp_y = delta.dy - axis.y * along;
    const perp_z = delta.dz - axis.z * along;
    recordLinearWarmStart(warm_start, perp_x, perp_y, perp_z);
    applyPairCorrection(inst_a, inst_b, mass_data, perp_x, perp_y, perp_z);

    const preload_along = joint.preload_linear_x * axis.x + joint.preload_linear_y * axis.y + joint.preload_linear_z * axis.z;
    if (@abs(preload_along) > 0.0001) {
        const preload_target = @max(joint.limit_min, @min(joint.limit_max, preload_along));
        const preload_error = along - preload_target;
        if (@abs(preload_error) > 0.0001) {
            recordLinearWarmStart(warm_start, axis.x * preload_error, axis.y * preload_error, axis.z * preload_error);
            applyPairCorrection(
                inst_a,
                inst_b,
                mass_data,
                axis.x * preload_error,
                axis.y * preload_error,
                axis.z * preload_error,
            );
        }
    }

    applySliderMotor(inst_a, inst_b, joint, entities, axis.x, axis.y, axis.z, warm_start);
    lockFixedRelativeRotation(inst_a, inst_b, mass_data, warm_start);
    dampFixedRelativeAngularVelocity(inst_a, inst_b, joint, mass_data);

    const min_limit = @min(joint.limit_min, joint.limit_max);
    const max_limit = @max(joint.limit_min, joint.limit_max);
    const clamped_along = @max(min_limit, @min(max_limit, along));
    const axis_error = along - clamped_along;
    if (@abs(axis_error) <= 0.0001) return;

    recordLinearWarmStart(warm_start, axis.x * axis_error, axis.y * axis_error, axis.z * axis_error);
    applyPairCorrection(
        inst_a,
        inst_b,
        mass_data,
        axis.x * axis_error,
        axis.y * axis_error,
        axis.z * axis_error,
    );
    applySliderLimitVelocityDamping(inst_a, inst_b, mass_data, axis.x, axis.y, axis.z, clamped_along, min_limit, max_limit);
}

/// Solve spring constraint (elastic connection)
pub fn solveSpringConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    const SpringAxis = struct {
        x: f32,
        y: f32,
        z: f32,
        dist: f32,
    };
    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    const delta = computeAnchorDelta(inst_a, inst_b, joint);
    const rest_length = @max(0.0, joint.limit_max);
    const spring_axis: SpringAxis = if (delta.dist >= 0.001)
        .{ .x = delta.dx / delta.dist, .y = delta.dy / delta.dist, .z = delta.dz / delta.dist, .dist = delta.dist }
    else blk: {
        if (rest_length <= 0.001) return;
        const axis = normalizedAxis(joint) orelse return;
        break :blk .{ .x = axis.x, .y = axis.y, .z = axis.z, .dist = 0.0 };
    };

    const extension = spring_axis.dist - rest_length;
    const stiffness = @max(0.0, joint.stiffness) * 0.01;
    const damping = @max(0.0, joint.damping) * 0.001;
    const nx = spring_axis.x;
    const ny = spring_axis.y;
    const nz = spring_axis.z;

    const rel_vel_x = @as(f32, @floatFromInt(inst_b.vel_x - inst_a.vel_x));
    const rel_vel_y = @as(f32, @floatFromInt(inst_b.vel_y - inst_a.vel_y));
    const rel_vel_z = @as(f32, @floatFromInt(inst_b.vel_z - inst_a.vel_z));
    const rel_speed = rel_vel_x * nx + rel_vel_y * ny + rel_vel_z * nz;
    const prediction_dt = 1.0 / 60.0;
    const predicted_extension = extension + rel_speed * prediction_dt;
    const control_extension = if (extension * predicted_extension < 0.0)
        predicted_extension
    else if (@abs(predicted_extension) < @abs(extension))
        predicted_extension
    else
        extension;
    const correction_mag = clampSpringCorrectionMagnitude(control_extension, stiffness);
    recordLinearWarmStart(warm_start, nx * correction_mag, ny * correction_mag, nz * correction_mag);

    applyPairCorrection(
        inst_a,
        inst_b,
        mass_data,
        nx * correction_mag,
        ny * correction_mag,
        nz * correction_mag,
    );

    const damping_impulse = rel_speed * damping;

    if (mass_data.inv_mass_a > 0.0) {
        inst_a.vel_x += clampI16FromF32(nx * damping_impulse * mass_data.ratio_a);
        inst_a.vel_y += clampI16FromF32(ny * damping_impulse * mass_data.ratio_a);
        inst_a.vel_z += clampI16FromF32(nz * damping_impulse * mass_data.ratio_a);
    }
    if (mass_data.inv_mass_b > 0.0) {
        inst_b.vel_x -= clampI16FromF32(nx * damping_impulse * mass_data.ratio_b);
        inst_b.vel_y -= clampI16FromF32(ny * damping_impulse * mass_data.ratio_b);
        inst_b.vel_z -= clampI16FromF32(nz * damping_impulse * mass_data.ratio_b);
    }
}

/// Solve pulley constraint along the configured axis.
pub fn solvePulleyConstraint(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint: *const Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    const mass_data = computeConstraintMassData(inst_a, inst_b, entities) orelse return;
    const state = measurePulleyState(inst_a, inst_b, joint) orelse return;
    recordLinearWarmStart(
        warm_start,
        state.axis_x * state.pulley_error,
        state.axis_y * state.pulley_error,
        state.axis_z * state.pulley_error,
    );
    applyPulleyCorrection(inst_a, inst_b, mass_data, state);
    applyPulleyVelocityDamping(inst_a, inst_b, mass_data, state, @max(0.0, joint.damping) * 0.001);
}

/// Check if joint should break based on force
pub fn shouldBreakJoint(joint: *Joint, force_magnitude: f32) bool {
    if (joint.breaking_force <= 0) return false;
    return force_magnitude >= joint.breaking_force;
}

fn solveJointPair(
    inst_a: *scene32.Instance,
    inst_b: *scene32.Instance,
    joint_def: *Joint,
    entities: []entity16.Entity16,
    warm_start: *JointWarmStart,
) void {
    switch (joint_def.joint_type) {
        .fixed => {
            solveFixedConstraint(inst_a, inst_b, joint_def, entities, warm_start);
        },
        .ball_socket => {
            solveDistanceConstraint(inst_a, inst_b, joint_def, entities, warm_start);
        },
        .hinge => {
            solveHingeConstraint(inst_a, inst_b, joint_def, entities, warm_start);
        },
        .slider => {
            solveSliderConstraint(inst_a, inst_b, joint_def, entities, warm_start);
        },
        .spring => {
            solveSpringConstraint(inst_a, inst_b, joint_def, entities, warm_start);
        },
        .pulley => {
            solvePulleyConstraint(inst_a, inst_b, joint_def, entities, warm_start);
        },
    }
}

fn computeJointGraphComplexity(joints: []const Joint) u8 {
    if (joints.len == 0) return 0;

    var unique_entities: [MAX_JOINTS * 2]u16 = undefined;
    var degree_by_entity: [MAX_JOINTS * 2]u8 = [_]u8{0} ** (MAX_JOINTS * 2);
    var entity_count: usize = 0;
    var max_degree: u8 = 0;

    for (joints) |joint_def| {
        if (!joint_def.enabled) continue;

        const pair = [_]u16{ joint_def.entity_a, joint_def.entity_b };
        for (pair) |entity_id| {
            var entity_idx: usize = 0;
            var found = false;
            while (entity_idx < entity_count) : (entity_idx += 1) {
                if (unique_entities[entity_idx] == entity_id) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                unique_entities[entity_count] = entity_id;
                entity_idx = entity_count;
                entity_count += 1;
            }

            degree_by_entity[entity_idx] += 1;
            max_degree = @max(max_degree, degree_by_entity[entity_idx]);
        }
    }

    if (entity_count == 0) return 0;

    const enabled_joint_count = @as(i32, @intCast(joints.len));
    const enabled_entity_count = @as(i32, @intCast(entity_count));
    const cycle_edges = @max(0, enabled_joint_count - enabled_entity_count + 1);

    var complexity: u8 = 0;
    if (max_degree >= 3) complexity += 1;
    if (max_degree >= 4) complexity += 1;
    if (cycle_edges >= 1) complexity += 1;
    if (cycle_edges >= 2) complexity += 1;
    return complexity;
}

fn computeJointRiskComplexity(joints: []const Joint) u8 {
    var max_break_ratio: f32 = 0.0;

    for (joints) |joint_def| {
        if (!joint_def.enabled) continue;
        if (joint_def.breaking_force <= 0.0) continue;

        max_break_ratio = @max(
            max_break_ratio,
            computeJointBreakRiskRatio(&joint_def),
        );
    }

    var complexity: u8 = 0;
    if (max_break_ratio >= 0.5) complexity += 1;
    if (max_break_ratio >= 0.85) complexity += 1;
    return complexity;
}

fn computeJointMaxBreakRatio(joints: []const Joint) f32 {
    var max_break_ratio: f32 = 0.0;

    for (joints) |joint_def| {
        if (!joint_def.enabled) continue;
        if (joint_def.breaking_force <= 0.0) continue;

        max_break_ratio = @max(
            max_break_ratio,
            computeJointBreakRiskRatio(&joint_def),
        );
    }

    return max_break_ratio;
}

fn computeJointIslandRiskScore(joints: []Joint, joint_indices: []const usize) f32 {
    var max_break_ratio: f32 = 0.0;
    var accum_break_ratio: f32 = 0.0;
    var counted: usize = 0;

    for (joint_indices) |joint_idx| {
        const joint_def = &joints[joint_idx];
        if (!joint_def.enabled) continue;

        const break_ratio = computeJointBreakRiskRatio(joint_def);
        max_break_ratio = @max(max_break_ratio, break_ratio);
        accum_break_ratio += break_ratio;
        counted += 1;
    }

    if (counted == 0) return 0.0;
    return max_break_ratio * 2.0 + accum_break_ratio / @as(f32, @floatFromInt(counted));
}

fn computeJointIslandUrgencyScore(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_indices: []const usize,
) f32 {
    const base_stress = computeMaxActiveJointConstraintMagnitudeForIndices(instances, joints, joint_indices);
    const risk_score = computeJointIslandRiskScore(joints, joint_indices);
    if (risk_score <= 0.0) return base_stress;

    var strongest_breaking_force: f32 = 0.0;
    for (joint_indices) |joint_idx| {
        const joint_def = &joints[joint_idx];
        if (!joint_def.enabled) continue;
        strongest_breaking_force = @max(strongest_breaking_force, joint_def.breaking_force);
    }

    return @max(base_stress, strongest_breaking_force * risk_score);
}

fn computeJointIterationBudget(joints: []const Joint) u8 {
    const joint_count = joints.len;
    var budget: u8 = if (joint_count >= 16)
        8
    else if (joint_count >= 8)
        6
    else
        4;

    budget += computeJointGraphComplexity(joints);
    budget += computeJointRiskComplexity(joints);
    return @min(@as(u8, 12), budget);
}

fn computeJointSettleThreshold(joints: []const Joint) f32 {
    const break_ratio = computeJointMaxBreakRatio(joints);
    if (break_ratio >= 0.85) return 0.015;
    if (break_ratio >= 0.5) return 0.03;
    return 0.05;
}

fn copyJointSubset(
    joints: []Joint,
    joint_indices: []const usize,
    out_joints: []Joint,
) []Joint {
    for (joint_indices, 0..) |joint_idx, out_idx| {
        out_joints[out_idx] = joints[joint_idx];
    }
    return out_joints[0..joint_indices.len];
}

fn computeMaxActiveJointConstraintMagnitude(
    instances: []scene32.Instance,
    joints: []Joint,
) f32 {
    var max_magnitude: f32 = 0.0;
    for (joints) |*joint_def| {
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) continue;

        const inst_a = &instances[joint_def.entity_a];
        const inst_b = &instances[joint_def.entity_b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        max_magnitude = @max(max_magnitude, computeJointSolvePriorityMagnitude(inst_a, inst_b, joint_def));
    }
    return max_magnitude;
}

fn computeMaxActiveJointConstraintMagnitudeForIndices(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_indices: []const usize,
) f32 {
    var max_magnitude: f32 = 0.0;
    for (joint_indices) |joint_idx| {
        const joint_def = &joints[joint_idx];
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) continue;

        const inst_a = &instances[joint_def.entity_a];
        const inst_b = &instances[joint_def.entity_b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        max_magnitude = @max(max_magnitude, computeJointSolvePriorityMagnitude(inst_a, inst_b, joint_def));
    }
    return max_magnitude;
}

fn computeJointSolvePriorityMagnitude(
    inst_a: *const scene32.Instance,
    inst_b: *const scene32.Instance,
    joint_def: *const Joint,
) f32 {
    return computeJointStressWithBreakMemory(joint_def, measureJointConstraintMetrics(inst_a, inst_b, joint_def));
}

fn isStaticInstance(inst: *const scene32.Instance, entities: []entity16.Entity16) bool {
    if (inst.entity_id >= entities.len) return true;
    const entity = &entities[inst.entity_id];
    return (entity.physics.flags & 0x01) != 0 or entity.physics.mass == 0;
}

fn jointTouchesStatic(
    instances: []scene32.Instance,
    joint_def: *const Joint,
    entities: []entity16.Entity16,
) bool {
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return false;
    return isStaticInstance(&instances[joint_def.entity_a], entities) or
        isStaticInstance(&instances[joint_def.entity_b], entities);
}

fn computeJointStaticAnchorDepths(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_indices: []const usize,
    entities: []entity16.Entity16,
    out_depths: []u8,
) void {
    var queue: [MAX_JOINTS]usize = undefined;
    var queue_head: usize = 0;
    var queue_tail: usize = 0;
    var has_static_seed = false;

    for (joint_indices, 0..) |joint_idx, idx| {
        out_depths[idx] = 255;
        if (jointTouchesStatic(instances, &joints[joint_idx], entities)) {
            out_depths[idx] = 0;
            queue[queue_tail] = idx;
            queue_tail += 1;
            has_static_seed = true;
        }
    }

    if (!has_static_seed) {
        for (joint_indices, 0..) |_, idx| out_depths[idx] = 0;
        return;
    }

    while (queue_head < queue_tail) : (queue_head += 1) {
        const local_idx = queue[queue_head];
        const current_depth = out_depths[local_idx];
        const joint_def = &joints[joint_indices[local_idx]];

        for (joint_indices, 0..) |other_joint_idx, other_local_idx| {
            if (out_depths[other_local_idx] != 255) continue;
            if (!jointsShareEntity(joint_def, &joints[other_joint_idx])) continue;
            out_depths[other_local_idx] = current_depth + 1;
            queue[queue_tail] = other_local_idx;
            queue_tail += 1;
        }
    }
}

fn buildJointPriorityOrder(
    instances: []scene32.Instance,
    joints: []Joint,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []usize,
) usize {
    var count: usize = 0;
    var magnitudes: [MAX_JOINTS]f32 = undefined;

    for (joints, 0..) |*joint_def, idx| {
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) continue;

        const inst_a = &instances[joint_def.entity_a];
        const inst_b = &instances[joint_def.entity_b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        const magnitude = computeJointSolvePriorityMagnitude(inst_a, inst_b, joint_def);
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

fn buildJointPriorityOrderForIndices(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_indices: []const usize,
    static_anchor_depths: []const u8,
    settle_threshold: f32,
    reverse_tiebreak: bool,
    out_indices: []usize,
) usize {
    var count: usize = 0;
    var magnitudes: [MAX_JOINTS]f32 = undefined;
    var depths: [MAX_JOINTS]u8 = undefined;

    for (joint_indices, 0..) |joint_idx, local_idx| {
        const joint_def = &joints[joint_idx];
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) continue;

        const inst_a = &instances[joint_def.entity_a];
        const inst_b = &instances[joint_def.entity_b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;

        const magnitude = computeJointSolvePriorityMagnitude(inst_a, inst_b, joint_def);
        if (magnitude <= settle_threshold) continue;

        out_indices[count] = joint_idx;
        magnitudes[count] = magnitude;
        depths[count] = static_anchor_depths[local_idx];
        count += 1;
    }

    var i: usize = 0;
    while (i < count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < count) : (j += 1) {
            const better_depth = if (reverse_tiebreak)
                depths[j] > depths[best]
            else
                depths[j] < depths[best];
            const equal_depth = depths[j] == depths[best];
            const better_magnitude = magnitudes[j] > magnitudes[best];
            const equal_magnitude = @abs(magnitudes[j] - magnitudes[best]) <= 0.0001;
            const better_tiebreak = if (reverse_tiebreak)
                out_indices[j] > out_indices[best]
            else
                out_indices[j] < out_indices[best];
            if (better_depth or (equal_depth and (better_magnitude or (equal_magnitude and better_tiebreak)))) best = j;
        }
        if (best != i) {
            const tmp_idx = out_indices[i];
            out_indices[i] = out_indices[best];
            out_indices[best] = tmp_idx;

            const tmp_mag = magnitudes[i];
            magnitudes[i] = magnitudes[best];
            magnitudes[best] = tmp_mag;

            const tmp_depth = depths[i];
            depths[i] = depths[best];
            depths[best] = tmp_depth;
        }
    }

    return count;
}

fn jointsShareEntity(a: *const Joint, b: *const Joint) bool {
    return a.entity_a == b.entity_a or
        a.entity_a == b.entity_b or
        a.entity_b == b.entity_a or
        a.entity_b == b.entity_b;
}

fn buildJointIsland(
    joints: []Joint,
    start_idx: usize,
    assigned: []bool,
    out_indices: []usize,
) usize {
    var queue: [MAX_JOINTS]usize = undefined;
    var queue_head: usize = 0;
    var queue_tail: usize = 0;
    var island_count: usize = 0;

    assigned[start_idx] = true;
    queue[queue_tail] = start_idx;
    queue_tail += 1;

    while (queue_head < queue_tail) : (queue_head += 1) {
        const joint_idx = queue[queue_head];
        out_indices[island_count] = joint_idx;
        island_count += 1;

        const joint_def = &joints[joint_idx];
        var other_idx: usize = 0;
        while (other_idx < joints.len) : (other_idx += 1) {
            if (assigned[other_idx]) continue;
            const other_joint = &joints[other_idx];
            if (!other_joint.enabled) continue;
            if (!jointsShareEntity(joint_def, other_joint)) continue;

            assigned[other_idx] = true;
            queue[queue_tail] = other_idx;
            queue_tail += 1;
        }
    }

    return island_count;
}

fn solveJointIsland(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_indices: []const usize,
    entities: []entity16.Entity16,
    warm_starts: []JointWarmStart,
) void {
    if (joint_indices.len == 0) return;

    var priority_order: [MAX_JOINTS]usize = undefined;
    var static_anchor_depths: [MAX_JOINTS]u8 = undefined;
    var island_joint_copy: [MAX_JOINTS]Joint = undefined;
    const island_joints = copyJointSubset(joints, joint_indices, island_joint_copy[0..]);
    const settle_threshold = computeJointSettleThreshold(island_joints);
    const iterations = computeJointIterationBudget(island_joints);
    computeJointStaticAnchorDepths(instances, joints, joint_indices, entities, static_anchor_depths[0..joint_indices.len]);
    var iter: u8 = 0;

    while (iter < iterations) : (iter += 1) {
        const active_count = buildJointPriorityOrderForIndices(
            instances,
            joints,
            joint_indices,
            static_anchor_depths[0..joint_indices.len],
            settle_threshold,
            (iter & 1) != 0,
            priority_order[0..],
        );
        if (active_count == 0) break;

        var order_idx: usize = 0;
        while (order_idx < active_count) : (order_idx += 1) {
            const joint_idx = priority_order[order_idx];
            const joint_def = &joints[joint_idx];
            const inst_a = &instances[joint_def.entity_a];
            const inst_b = &instances[joint_def.entity_b];
            if (updateJointTemperature(joint_def, inst_a, inst_b)) continue;
            if (updateJointFatigue(joint_def, inst_a, inst_b)) continue;
            if (enforceJointBreak(joint_def, inst_a, inst_b)) continue;

            solveJointPair(inst_a, inst_b, joint_def, entities, &warm_starts[joint_idx]);
        }

        if (computeMaxActiveJointConstraintMagnitudeForIndices(instances, joints, joint_indices) <= settle_threshold) break;
    }
}

fn sortJointIslandsByStress(
    instances: []scene32.Instance,
    joints: []Joint,
    island_storage: []const [MAX_JOINTS]usize,
    island_lengths: []const usize,
    island_count: usize,
    out_order: []usize,
) void {
    var island_urgency: [MAX_JOINTS]f32 = undefined;

    var idx: usize = 0;
    while (idx < island_count) : (idx += 1) {
        out_order[idx] = idx;
        island_urgency[idx] = computeJointIslandUrgencyScore(
            instances,
            joints,
            island_storage[idx][0..island_lengths[idx]],
        );
    }

    var i: usize = 0;
    while (i < island_count) : (i += 1) {
        var best = i;
        var j: usize = i + 1;
        while (j < island_count) : (j += 1) {
            const better_stress = island_urgency[j] > island_urgency[best];
            const equal_stress = @abs(island_urgency[j] - island_urgency[best]) <= 0.0001;
            const better_tiebreak = out_order[j] < out_order[best];
            if (better_stress or (equal_stress and better_tiebreak)) best = j;
        }
        if (best != i) {
            const tmp_order = out_order[i];
            out_order[i] = out_order[best];
            out_order[best] = tmp_order;

            const tmp_stress = island_urgency[i];
            island_urgency[i] = island_urgency[best];
            island_urgency[best] = tmp_stress;
        }
    }
}

fn solveJointSlice(
    instances: []scene32.Instance,
    joints: []Joint,
    entities: []entity16.Entity16,
) void {
    if (joints.len == 0) return;
    var warm_starts: [MAX_JOINTS]JointWarmStart = undefined;
    for (joints, 0..) |*joint_def, idx| {
        prepareJointWarmStart(joint_def);
        warm_starts[idx] = .{};
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) continue;

        const inst_a = &instances[joint_def.entity_a];
        const inst_b = &instances[joint_def.entity_b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;
        applyJointWarmStart(inst_a, inst_b, joint_def, entities);
    }

    var assigned: [MAX_JOINTS]bool = [_]bool{false} ** MAX_JOINTS;
    var island_storage: [MAX_JOINTS][MAX_JOINTS]usize = undefined;
    var island_lengths: [MAX_JOINTS]usize = [_]usize{0} ** MAX_JOINTS;
    var island_order: [MAX_JOINTS]usize = undefined;
    var island_count_total: usize = 0;
    var island_indices: [MAX_JOINTS]usize = undefined;
    var joint_idx: usize = 0;
    while (joint_idx < joints.len) : (joint_idx += 1) {
        if (assigned[joint_idx]) continue;
        if (!joints[joint_idx].enabled) continue;

        const island_count = buildJointIsland(joints, joint_idx, assigned[0..joints.len], island_indices[0..]);
        island_lengths[island_count_total] = island_count;
        for (island_indices[0..island_count], 0..) |island_joint_idx, out_idx| {
            island_storage[island_count_total][out_idx] = island_joint_idx;
        }
        island_count_total += 1;
    }

    sortJointIslandsByStress(
        instances,
        joints,
        island_storage[0..island_count_total],
        island_lengths[0..island_count_total],
        island_count_total,
        island_order[0..island_count_total],
    );

    var island_order_idx: usize = 0;
    while (island_order_idx < island_count_total) : (island_order_idx += 1) {
        const island_idx = island_order[island_order_idx];
        solveJointIsland(
            instances,
            joints,
            island_storage[island_idx][0..island_lengths[island_idx]],
            entities,
            warm_starts[0..joints.len],
        );
    }

    for (joints, 0..) |*joint_def, idx| {
        if (!joint_def.enabled) {
            storeJointWarmStart(joint_def, .{});
            continue;
        }
        storeJointWarmStart(joint_def, warm_starts[idx]);
    }
}

/// Main solver - iterate through all joints
pub fn solveJoints(
    system: *JointSystem,
    instances: [*]scene32.Instance,
    instance_count: u8,
    entities: []entity16.Entity16,
) void {
    solveJointSlice(instances[0..instance_count], system.joints[0..system.joint_count], entities);
}

/// Solver for external use with slices
pub fn solveJointsForTick(
    instances: []scene32.Instance,
    joints: []Joint,
    entities: []entity16.Entity16,
) void {
    solveJointSlice(instances, joints, entities);
}

pub fn measureJointSolveStressForTick(
    instances: []scene32.Instance,
    joints: []Joint,
    entities: []entity16.Entity16,
) f32 {
    _ = entities;
    return computeMaxActiveJointConstraintMagnitude(instances, joints);
}

pub fn measureJointStressForIndex(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) ?JointStressSample {
    _ = entities;
    if (joint_idx >= joints.len) return null;
    const joint_def = &joints[joint_idx];
    if (!joint_def.enabled) return null;
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return null;

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return null;
    return makeJointStressSample(joint_def, measureJointConstraintMetrics(inst_a, inst_b, joint_def));
}

pub fn measureJointSolveStressForIndices(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_indices: []const usize,
    entities: []entity16.Entity16,
) f32 {
    _ = entities;
    return computeMaxActiveJointConstraintMagnitudeForIndices(instances, joints, joint_indices);
}

pub fn measureJointSolvePriorityForIndex(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) f32 {
    _ = entities;
    if (joint_idx >= joints.len) return 0.0;
    const joint_def = &joints[joint_idx];
    if (!joint_def.enabled) return 0.0;
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return 0.0;

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return 0.0;
    return computeJointSolvePriorityMagnitude(inst_a, inst_b, joint_def);
}

pub fn enforceJointBreakForIndex(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) bool {
    _ = entities;
    if (joint_idx >= joints.len) return false;
    const joint_def = &joints[joint_idx];
    if (!joint_def.enabled) return false;
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return false;

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return false;
    if (updateJointTemperature(joint_def, inst_a, inst_b)) return true;
    if (updateJointFatigue(joint_def, inst_a, inst_b)) return true;
    return enforceJointBreak(joint_def, inst_a, inst_b);
}

pub fn solveJointIndicesForTick(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_indices: []const usize,
    entities: []entity16.Entity16,
) void {
    if (joint_indices.len == 0) return;

    var warm_starts: [MAX_JOINTS]JointWarmStart = undefined;
    for (joint_indices) |joint_idx| {
        const joint_def = &joints[joint_idx];
        prepareJointWarmStart(joint_def);
        warm_starts[joint_idx] = .{};
        if (!joint_def.enabled) continue;
        if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) continue;

        const inst_a = &instances[joint_def.entity_a];
        const inst_b = &instances[joint_def.entity_b];
        if (inst_a.state == .broken or inst_b.state == .broken) continue;
        applyJointWarmStart(inst_a, inst_b, joint_def, entities);
    }

    solveJointIsland(instances, joints, joint_indices, entities, warm_starts[0..joints.len]);

    for (joint_indices) |joint_idx| {
        storeJointWarmStart(&joints[joint_idx], warm_starts[joint_idx]);
    }
}

pub fn solveJointIndexForTick(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
) bool {
    return solveJointIndexForTickWithHint(instances, joints, joint_idx, entities, 0.0);
}

pub fn solveJointIndexForTickWithHint(
    instances: []scene32.Instance,
    joints: []Joint,
    joint_idx: usize,
    entities: []entity16.Entity16,
    extra_impulse_hint: f32,
) bool {
    if (joint_idx >= joints.len) return false;

    const joint_def = &joints[joint_idx];
    prepareJointWarmStart(joint_def);
    var warm_start: JointWarmStart = .{};

    if (!joint_def.enabled) return false;
    if (joint_def.entity_a >= instances.len or joint_def.entity_b >= instances.len) return false;

    const inst_a = &instances[joint_def.entity_a];
    const inst_b = &instances[joint_def.entity_b];
    if (inst_a.state == .broken or inst_b.state == .broken) return false;

    var solve_joint = joint_def.*;
    if (extra_impulse_hint > 0.0) {
        const hint_scale = 1.0 + @min(extra_impulse_hint * 0.1, 0.5);
        solve_joint.warm_linear_x *= hint_scale;
        solve_joint.warm_linear_y *= hint_scale;
        solve_joint.warm_linear_z *= hint_scale;
        solve_joint.warm_angular *= hint_scale;
        solve_joint.warm_drive_velocity *= hint_scale;
    }

    applyJointWarmStart(inst_a, inst_b, &solve_joint, entities);
    if (updateJointTemperature(joint_def, inst_a, inst_b)) {
        storeJointWarmStart(joint_def, warm_start);
        return true;
    }
    if (updateJointFatigue(joint_def, inst_a, inst_b)) {
        storeJointWarmStart(joint_def, warm_start);
        return true;
    }
    if (enforceJointBreak(joint_def, inst_a, inst_b)) {
        storeJointWarmStart(joint_def, warm_start);
        return true;
    }

    const before = computeJointSolvePriorityMagnitude(inst_a, inst_b, joint_def);
    solveJointPair(inst_a, inst_b, joint_def, entities, &warm_start);
    storeJointWarmStart(joint_def, warm_start);
    const after = computeJointSolvePriorityMagnitude(inst_a, inst_b, joint_def);
    return after < before or warm_start.linear_magnitude > 0.0 or
        @abs(warm_start.angular) > 0.0 or
        @abs(warm_start.drive_velocity) > 0.0;
}

fn makeTestInstance(entity_id: u16, x: i32, y: i32, z: i32) scene32.Instance {
    return .{
        .entity_id = entity_id,
        .pos_x = x,
        .pos_y = y,
        .pos_z = z,
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
}

test "slider constraint removes perpendicular drift and preserves rail axis" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 7, 0),
    };
    var joints = [_]Joint{.{
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

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[1].pos_y - instances[0].pos_y)), 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[1].pos_z - instances[0].pos_z)), 0.5);
    try std.testing.expectApproxEqAbs(@as(f32, 10.0), @as(f32, @floatFromInt(instances[1].pos_x - instances[0].pos_x)), 0.5);
}

test "slider constraint locks relative rotation while preserving rail travel" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 12, 0, 0),
    };
    instances[0].rot_roll = radiansToRotationByte(-0.5);
    instances[1].rot_roll = radiansToRotationByte(0.5);
    instances[0].rot_yaw = radiansToRotationByte(-0.75);
    instances[1].rot_yaw = radiansToRotationByte(0.75);
    instances[0].rot_pitch = radiansToRotationByte(-1.0);
    instances[1].rot_pitch = radiansToRotationByte(1.0);

    var joints = [_]Joint{.{
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

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expectApproxEqAbs(@as(f32, 12.0), @as(f32, @floatFromInt(instances[1].pos_x - instances[0].pos_x)), 0.5);
    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .x) - getJointAngleForAxis(&instances[0], .x)) <= 0.05);
    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .y) - getJointAngleForAxis(&instances[0], .y)) <= 0.05);
    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .z) - getJointAngleForAxis(&instances[0], .z)) <= 0.05);
}

test "slider constraint damps relative angular velocity" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 12, 0, 0),
    };
    instances[0].ang_z = 0;
    instances[1].ang_z = 8;

    var joints = [_]Joint{.{
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

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(@abs(instances[1].ang_z - instances[0].ang_z) < 8);
}

test "hinge constraint clamps relative dominant axis rotation to configured limits" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    instances[0].rot_yaw = radiansToRotationByte(1.5);
    instances[1].rot_yaw = radiansToRotationByte(-1.5);

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .limit_min = -0.5,
        .limit_max = 0.5,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    const relative_angle = getJointAngle(&instances[1], &joints[0]) - getJointAngle(&instances[0], &joints[0]);
    try std.testing.expect(relative_angle <= 0.55);
    try std.testing.expect(relative_angle >= -0.55);
}

test "hinge constraint locks swing axes while preserving free axis" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    instances[0].rot_roll = radiansToRotationByte(-0.5);
    instances[1].rot_roll = radiansToRotationByte(0.5);
    instances[0].rot_pitch = radiansToRotationByte(-0.75);
    instances[1].rot_pitch = radiansToRotationByte(0.75);
    instances[0].rot_yaw = radiansToRotationByte(-0.25);
    instances[1].rot_yaw = radiansToRotationByte(0.25);

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .limit_min = -1.0,
        .limit_max = 1.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .x) - getJointAngleForAxis(&instances[0], .x)) <= 0.05);
    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .z) - getJointAngleForAxis(&instances[0], .z)) <= 0.05);
    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .y) - getJointAngleForAxis(&instances[0], .y)) > 0.1);
}

test "hinge constraint damps swing angular velocity" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    instances[0].ang_x = 0;
    instances[1].ang_x = 8;
    instances[0].ang_y = 0;
    instances[1].ang_y = 8;

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .limit_min = -1.0,
        .limit_max = 1.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(@abs(instances[1].ang_x - instances[0].ang_x) < 8);
    try std.testing.expectEqual(@as(i8, 8), instances[1].ang_y - instances[0].ang_y);
}

test "breakable joint disables itself when stretch exceeds threshold" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 20, 0, 0),
    };
    var joints = [_]Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .breaking_force = 5.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(!joints[0].enabled);
}

test "breakable joint accumulates sustained overload across repeated solves" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    const inst_a = makeTestInstance(0, 0, 0, 0);
    const inst_b = makeTestInstance(1, 4, 0, 0);
    var joints = [_]Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .breaking_force = 5.0,
    }};

    var tick: u8 = 0;
    while (tick < 12 and joints[0].enabled) : (tick += 1) {
        _ = enforceJointBreak(&joints[0], &inst_a, &inst_b);
    }

    try std.testing.expect(!joints[0].enabled);
}

test "fixed joint damps residual stretch velocity after position correction" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 6, 0, 0),
    };
    instances[0].vel_x = 0;
    instances[1].vel_x = 5;

    var joints = [_]Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .damping = 100.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[1].vel_x < 5);
}

test "fixed joint locks relative rotation across all axes" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    instances[0].rot_roll = radiansToRotationByte(-0.5);
    instances[0].rot_yaw = radiansToRotationByte(0.75);
    instances[0].rot_pitch = radiansToRotationByte(-1.0);
    instances[1].rot_roll = radiansToRotationByte(0.5);
    instances[1].rot_yaw = radiansToRotationByte(-0.75);
    instances[1].rot_pitch = radiansToRotationByte(1.0);

    var joints = [_]Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .x) - getJointAngleForAxis(&instances[0], .x)) <= 0.05);
    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .y) - getJointAngleForAxis(&instances[0], .y)) <= 0.05);
    try std.testing.expect(@abs(getJointAngleForAxis(&instances[1], .z) - getJointAngleForAxis(&instances[0], .z)) <= 0.05);
}

test "fixed joint damps relative angular velocity" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    instances[0].ang_y = 0;
    instances[1].ang_y = 8;

    var joints = [_]Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(@abs(instances[1].ang_y - instances[0].ang_y) < 8);
}

test "ball socket joint damps residual stretch velocity after position correction" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 6, 0),
    };
    instances[0].vel_y = 0;
    instances[1].vel_y = 5;

    var joints = [_]Joint{.{
        .joint_type = .ball_socket,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .damping = 100.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[1].vel_y < 5);
}

test "ball socket joint converges offset anchors without locking rotation" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 0, 0),
    };
    instances[0].rot_yaw = radiansToRotationByte(-0.5);
    instances[1].rot_yaw = radiansToRotationByte(0.75);
    instances[1].ang_y = 8;

    var joints = [_]Joint{.{
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

    const before_delta = computeAnchorDelta(&instances[0], &instances[1], &joints[0]);
    const before_angle = getJointAngleForAxis(&instances[1], .y) - getJointAngleForAxis(&instances[0], .y);
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const after_delta = computeAnchorDelta(&instances[0], &instances[1], &joints[0]);
    const after_angle = getJointAngleForAxis(&instances[1], .y) - getJointAngleForAxis(&instances[0], .y);

    try std.testing.expect(after_delta.dist < before_delta.dist);
    try std.testing.expectApproxEqAbs(before_angle, after_angle, 0.05);
    try std.testing.expectEqual(@as(i8, 8), instances[1].ang_y - instances[0].ang_y);
}

test "pulley constraint transfers axis motion through rope ratio" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 6, 0),
        makeTestInstance(1, 0, 10, 0),
    };
    var joints = [_]Joint{.{
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

    const before_state = measurePulleyState(&instances[0], &instances[1], &joints[0]).?;
    const before_b_y = instances[1].pos_y;
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const after_state = measurePulleyState(&instances[0], &instances[1], &joints[0]).?;

    try std.testing.expect(@abs(after_state.pulley_error) < @abs(before_state.pulley_error));
    try std.testing.expect(instances[1].pos_y < before_b_y);
}

test "pulley constraint damps weighted rope velocity" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 10, 0),
    };
    instances[0].vel_y = 4;
    instances[1].vel_y = 4;
    var joints = [_]Joint{.{
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
        .damping = 1000.0,
    }};

    const before_speed = measurePulleyState(&instances[0], &instances[1], &joints[0]).?.constraint_speed;
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const after_speed = measurePulleyState(&instances[0], &instances[1], &joints[0]).?.constraint_speed;

    try std.testing.expect(@abs(after_speed) < @abs(before_speed));
}

test "spring constraint reduces extension toward rest length" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 0, 0),
    };
    var joints = [_]Joint{.{
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
        .stiffness = 100.0,
        .damping = 0.0,
    }};

    const initial_dist = @as(f32, @floatFromInt(instances[1].pos_x - instances[0].pos_x));
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const final_dist = @as(f32, @floatFromInt(instances[1].pos_x - instances[0].pos_x));

    try std.testing.expect(@abs(final_dist - joints[0].limit_max) < @abs(initial_dist - joints[0].limit_max));
}

test "spring constraint clamps high stiffness correction without overshoot" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 20, 0, 0),
    };
    var joints = [_]Joint{.{
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
        .damping = 0.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const final_dist = @as(f32, @floatFromInt(instances[1].pos_x - instances[0].pos_x));

    try std.testing.expect(final_dist >= joints[0].limit_max);
    try std.testing.expect(final_dist < 20.0);
}

test "spring constraint expands coincident anchors along configured axis" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    var joints = [_]Joint{.{
        .joint_type = .spring,
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
        .limit_max = 6.0,
        .stiffness = 10000.0,
        .damping = 0.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(@abs(instances[1].pos_x - instances[0].pos_x) > 0);
    try std.testing.expectEqual(@as(i32, 0), instances[1].pos_y - instances[0].pos_y);
    try std.testing.expectEqual(@as(i32, 0), instances[1].pos_z - instances[0].pos_z);
}

test "spring constraint damps residual relative speed near rest length" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 4, 0, 0),
    };
    instances[0].vel_x = 0;
    instances[1].vel_x = 6;

    var joints = [_]Joint{.{
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
        .stiffness = 0.0,
        .damping = 100.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[1].vel_x < 6);
}

test "joint solver propagates chain correction across multiple links" {
    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;
    entities[2].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 20, 0, 0),
        makeTestInstance(2, 40, 8, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
    };

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[1].pos_y - instances[0].pos_y)), 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[2].pos_y - instances[1].pos_y)), 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[2].pos_y - instances[0].pos_y)), 2.0);
}

test "fixed joint preload preserves configured anchor offset" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };

    var joints = [_]Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .preload_linear_x = 4.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expectEqual(@as(i32, 4), instances[1].pos_x - instances[0].pos_x);
}

test "hinge joint preload biases relative angle" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .axis_z = 1,
        .limit_min = -1.0,
        .limit_max = 1.0,
        .preload_angular = 0.5,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    const relative_angle = getJointAngle(&instances[1], &joints[0]) - getJointAngle(&instances[0], &joints[0]);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), relative_angle, 0.05);
}

test "measureJointStressForIndex reports error components and break memory" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 12, 0, 0),
    };
    instances[1].vel_x = 4;

    var joints = [_]Joint{.{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .breaking_force = 20.0,
        .break_accum = 10.0,
    }};

    const sample = measureJointStressForIndex(instances[0..], joints[0..], 0, entities[0..]).?;

    try std.testing.expectApproxEqAbs(@as(f32, 12.0), sample.geometry_error, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), sample.residual_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), sample.break_ratio, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 12.0), sample.stress, 0.0001);
    try std.testing.expect(measureJointStressForIndex(instances[0..], joints[0..], 99, entities[0..]) == null);
}

test "joint fatigue accumulates sustained stress and recovers at rest" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 12, 0, 0),
    };

    var stressed_joint = Joint{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .breaking_force = 20.0,
        .fatigue_limit = 0.7,
        .fatigue_rate = 1.0,
        .fatigue_recovery = 0.1,
    };

    try std.testing.expect(!updateJointFatigue(&stressed_joint, &instances[0], &instances[1]));
    try std.testing.expectApproxEqAbs(@as(f32, 0.35), stressed_joint.fatigue_damage, 0.0001);
    try std.testing.expect(updateJointFatigue(&stressed_joint, &instances[0], &instances[1]));
    try std.testing.expect(!stressed_joint.enabled);

    var resting_joint = stressed_joint;
    resting_joint.enabled = true;
    resting_joint.fatigue_damage = 0.5;
    instances[1].pos_x = 0;
    try std.testing.expect(!updateJointFatigue(&resting_joint, &instances[0], &instances[1]));
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), resting_joint.fatigue_damage, 0.0001);
}

test "joint temperature accumulates stress heat and cools at rest" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 12, 0, 0),
    };

    var hot_joint = Joint{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .breaking_force = 20.0,
        .temperature_limit = 5.0,
        .temperature_rate = 5.0,
        .temperature_cooling = 0.1,
    };

    try std.testing.expect(!updateJointTemperature(&hot_joint, &instances[0], &instances[1]));
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), hot_joint.temperature, 0.0001);
    try std.testing.expect(updateJointTemperature(&hot_joint, &instances[0], &instances[1]));
    try std.testing.expect(!hot_joint.enabled);

    var cooling_joint = hot_joint;
    cooling_joint.enabled = true;
    cooling_joint.temperature = 5.0;
    instances[1].pos_x = 0;
    try std.testing.expect(!updateJointTemperature(&cooling_joint, &instances[0], &instances[1]));
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), cooling_joint.temperature, 0.0001);
}

test "computeJointIterationBudget scales with joint count" {
    const j = Joint{
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
    var one = [_]Joint{j};
    var seven = [_]Joint{ j, j, j, j, j, j, j };
    var eight = [_]Joint{ j, j, j, j, j, j, j, j };
    var fifteen = [_]Joint{ j, j, j, j, j, j, j, j, j, j, j, j, j, j, j };
    var sixteen = [_]Joint{ j, j, j, j, j, j, j, j, j, j, j, j, j, j, j, j };

    for (&seven, 0..) |*joint_def, idx| {
        joint_def.entity_a = @as(u16, @intCast(idx * 2));
        joint_def.entity_b = @as(u16, @intCast(idx * 2 + 1));
    }
    for (&eight, 0..) |*joint_def, idx| {
        joint_def.entity_a = @as(u16, @intCast(idx * 2));
        joint_def.entity_b = @as(u16, @intCast(idx * 2 + 1));
    }
    for (&fifteen, 0..) |*joint_def, idx| {
        joint_def.entity_a = @as(u16, @intCast(idx * 2));
        joint_def.entity_b = @as(u16, @intCast(idx * 2 + 1));
    }
    for (&sixteen, 0..) |*joint_def, idx| {
        joint_def.entity_a = @as(u16, @intCast(idx * 2));
        joint_def.entity_b = @as(u16, @intCast(idx * 2 + 1));
    }

    try std.testing.expectEqual(@as(u8, 4), computeJointIterationBudget(one[0..]));
    try std.testing.expectEqual(@as(u8, 4), computeJointIterationBudget(seven[0..]));
    try std.testing.expectEqual(@as(u8, 6), computeJointIterationBudget(eight[0..]));
    try std.testing.expectEqual(@as(u8, 6), computeJointIterationBudget(fifteen[0..]));
    try std.testing.expectEqual(@as(u8, 8), computeJointIterationBudget(sixteen[0..]));
}

test "computeJointIterationBudget increases for cyclic joint graph" {
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 2,
            .entity_b = 0,
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
        },
    };

    try std.testing.expectEqual(@as(u8, 5), computeJointIterationBudget(joints[0..]));
}

test "computeJointIterationBudget increases for near breaking joint risk" {
    var joints = [_]Joint{
        .{
            .joint_type = .fixed,
            .entity_a = 0,
            .entity_b = 1,
            .anchor_a_x = 0,
            .anchor_a_y = 0,
            .anchor_a_z = 0,
            .anchor_b_x = 0,
            .anchor_b_y = 0,
            .anchor_b_z = 0,
            .breaking_force = 10.0,
            .break_accum = 9.0,
        },
    };

    try std.testing.expectEqual(@as(u8, 6), computeJointIterationBudget(joints[0..]));
}

test "computeJointSettleThreshold tightens for near breaking joint risk" {
    var joints = [_]Joint{
        .{
            .joint_type = .fixed,
            .entity_a = 0,
            .entity_b = 1,
            .anchor_a_x = 0,
            .anchor_a_y = 0,
            .anchor_a_z = 0,
            .anchor_b_x = 0,
            .anchor_b_y = 0,
            .anchor_b_z = 0,
            .breaking_force = 10.0,
            .break_accum = 9.0,
        },
    };

    try std.testing.expectApproxEqAbs(@as(f32, 0.015), computeJointSettleThreshold(joints[0..]), 0.0001);
}

test "prepareJointWarmStart retains more history for near breaking joints" {
    var safe_joint = Joint{
        .joint_type = .fixed,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .warm_linear_x = 10.0,
        .warm_angular = 5.0,
        .warm_drive_velocity = 4.0,
    };
    var risky_joint = safe_joint;
    risky_joint.breaking_force = 10.0;
    risky_joint.break_accum = 9.0;

    prepareJointWarmStart(&safe_joint);
    prepareJointWarmStart(&risky_joint);

    try std.testing.expectApproxEqAbs(@as(f32, 7.0), safe_joint.warm_linear_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 9.0), risky_joint.warm_linear_x, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.5), safe_joint.warm_angular, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 4.5), risky_joint.warm_angular, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 2.8), safe_joint.warm_drive_velocity, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 3.6), risky_joint.warm_drive_velocity, 0.0001);
}

test "joint solver stabilizes cyclic slider loop without recursion or divergence" {
    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;
    entities[2].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 7, 0),
        makeTestInstance(2, 20, -6, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 2,
            .entity_b = 0,
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
        },
    };

    const initial_magnitude = computeMaxActiveJointConstraintMagnitude(instances[0..], joints[0..]);
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const final_magnitude = computeMaxActiveJointConstraintMagnitude(instances[0..], joints[0..]);

    try std.testing.expect(final_magnitude < initial_magnitude);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[1].pos_y - instances[0].pos_y)), 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[2].pos_y - instances[1].pos_y)), 2.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[0].pos_y - instances[2].pos_y)), 2.0);
}

test "joint solver isolates disconnected islands so each cluster still converges" {
    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    for (&entities) |*entity| entity.physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 6, 0),
        makeTestInstance(1, 10, 0, 0),
        makeTestInstance(2, 100, -5, 0),
        makeTestInstance(3, 110, 4, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 2,
            .entity_b = 3,
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
        },
    };

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[0].pos_y - instances[1].pos_y)), 1.0);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatFromInt(instances[2].pos_y - instances[3].pos_y)), 1.0);
}

test "sortJointIslandsByStress prioritizes most stressed island first" {
    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 6, 0),
        makeTestInstance(1, 10, 0, 0),
        makeTestInstance(2, 100, 1, 0),
        makeTestInstance(3, 110, 0, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 2,
            .entity_b = 3,
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
        },
    };

    const island_storage = [_][MAX_JOINTS]usize{
        blk: {
            var row: [MAX_JOINTS]usize = [_]usize{0} ** MAX_JOINTS;
            row[0] = 0;
            break :blk row;
        },
        blk: {
            var row: [MAX_JOINTS]usize = [_]usize{0} ** MAX_JOINTS;
            row[0] = 1;
            break :blk row;
        },
    };
    const island_lengths = [_]usize{ 1, 1 };
    var island_order: [2]usize = undefined;

    sortJointIslandsByStress(
        instances[0..],
        joints[0..],
        island_storage[0..],
        island_lengths[0..],
        2,
        island_order[0..],
    );

    try std.testing.expectEqual(@as(usize, 0), island_order[0]);
    try std.testing.expectEqual(@as(usize, 1), island_order[1]);
}

test "sortJointIslandsByStress prioritizes island with higher break risk" {
    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 1, 0),
        makeTestInstance(1, 10, 0, 0),
        makeTestInstance(2, 100, 4, 0),
        makeTestInstance(3, 110, 0, 0),
    };
    var joints = [_]Joint{
        .{
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
            .breaking_force = 10.0,
            .break_accum = 9.0,
        },
        .{
            .joint_type = .slider,
            .entity_a = 2,
            .entity_b = 3,
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
        },
    };

    const island_storage = [_][MAX_JOINTS]usize{
        blk: {
            var row: [MAX_JOINTS]usize = [_]usize{0} ** MAX_JOINTS;
            row[0] = 0;
            break :blk row;
        },
        blk: {
            var row: [MAX_JOINTS]usize = [_]usize{0} ** MAX_JOINTS;
            row[0] = 1;
            break :blk row;
        },
    };
    const island_lengths = [_]usize{ 1, 1 };
    var island_order: [2]usize = undefined;

    sortJointIslandsByStress(
        instances[0..],
        joints[0..],
        island_storage[0..],
        island_lengths[0..],
        2,
        island_order[0..],
    );

    try std.testing.expectEqual(@as(usize, 0), island_order[0]);
    try std.testing.expectEqual(@as(usize, 1), island_order[1]);
}

test "buildJointPriorityOrderForIndices prioritizes static anchored joints within island" {
    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.flags |= 0x01;
    entities[1].physics.mass = 10;
    entities[2].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 2, 0),
        makeTestInstance(2, 20, 8, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
    };
    const joint_indices = [_]usize{ 0, 1 };
    var depths: [2]u8 = undefined;
    var order: [2]usize = undefined;

    computeJointStaticAnchorDepths(instances[0..], joints[0..], joint_indices[0..], entities[0..], depths[0..]);
    const count = buildJointPriorityOrderForIndices(
        instances[0..],
        joints[0..],
        joint_indices[0..],
        depths[0..],
        0.0,
        false,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 0), order[0]);
    try std.testing.expectEqual(@as(usize, 1), order[1]);
}

test "buildJointPriorityOrderForIndices reverses static anchored sweep on alternating pass" {
    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.flags |= 0x01;
    entities[1].physics.mass = 10;
    entities[2].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 2, 0),
        makeTestInstance(2, 20, 8, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
    };
    const joint_indices = [_]usize{ 0, 1 };
    var depths: [2]u8 = undefined;
    var order: [2]usize = undefined;

    computeJointStaticAnchorDepths(instances[0..], joints[0..], joint_indices[0..], entities[0..], depths[0..]);
    const count = buildJointPriorityOrderForIndices(
        instances[0..],
        joints[0..],
        joint_indices[0..],
        depths[0..],
        0.0,
        true,
        order[0..],
    );

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 1), order[0]);
    try std.testing.expectEqual(@as(usize, 0), order[1]);
}

test "buildJointPriorityOrder prioritizes largest active constraint first" {
    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 1, 0),
        makeTestInstance(2, 20, 5, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
    };
    var order: [MAX_JOINTS]usize = undefined;

    const count = buildJointPriorityOrder(instances[0..], joints[0..], 0.0, false, order[0..]);
    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 1), order[0]);
    try std.testing.expectEqual(@as(usize, 0), order[1]);
}

test "buildJointPriorityOrder skips constraints already within settle threshold" {
    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 0, 0),
        makeTestInstance(2, 20, 5, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
    };
    var order: [MAX_JOINTS]usize = undefined;

    instances[0].vel_x = 0;
    instances[1].vel_x = 0;
    instances[2].vel_x = 0;

    const count = buildJointPriorityOrder(instances[0..], joints[0..], 0.05, false, order[0..]);
    try std.testing.expectEqual(@as(usize, 1), count);
    try std.testing.expectEqual(@as(usize, 1), order[0]);
}

test "buildJointPriorityOrder prioritizes joint with higher accumulated break stress" {
    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 10, 1, 0),
        makeTestInstance(2, 20, 2, 0),
    };
    var joints = [_]Joint{
        .{
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
            .breaking_force = 10.0,
            .break_accum = 8.0,
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
    };
    var order: [MAX_JOINTS]usize = undefined;

    const count = buildJointPriorityOrder(instances[0..], joints[0..], 0.0, false, order[0..]);

    try std.testing.expectEqual(@as(usize, 2), count);
    try std.testing.expectEqual(@as(usize, 0), order[0]);
}

test "joint solver reduces chain residual across adaptive iterations" {
    var entities = [_]entity16.Entity16{
        entity16.initEntity16(),
        entity16.initEntity16(),
        entity16.initEntity16(),
    };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;
    entities[2].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 20, 9, 0),
        makeTestInstance(2, 40, -9, 0),
    };
    var joints = [_]Joint{
        .{
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
        },
        .{
            .joint_type = .slider,
            .entity_a = 1,
            .entity_b = 2,
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
        },
    };

    const initial_magnitude = computeMaxActiveJointConstraintMagnitude(instances[0..], joints[0..]);
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const final_magnitude = computeMaxActiveJointConstraintMagnitude(instances[0..], joints[0..]);

    try std.testing.expect(final_magnitude < initial_magnitude);
}

test "hinge motor drives relative angle toward target" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .limit_min = -1.0,
        .limit_max = 1.0,
        .motor_enabled = true,
        .motor_target = 0.6,
        .motor_speed = 6.0,
        .motor_max_torque = 100.0,
    }};

    const initial_error = @abs(joints[0].motor_target - (getJointAngle(&instances[1], &joints[0]) - getJointAngle(&instances[0], &joints[0])));
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const final_error = @abs(joints[0].motor_target - (getJointAngle(&instances[1], &joints[0]) - getJointAngle(&instances[0], &joints[0])));

    try std.testing.expect(final_error < initial_error);
}

test "slider motor drives relative axis displacement toward target" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };

    var joints = [_]Joint{.{
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
        .limit_min = -2.0,
        .limit_max = 2.0,
        .motor_enabled = true,
        .motor_target = 1.5,
        .motor_speed = 6.0,
        .motor_max_torque = 100.0,
    }};

    const initial_error = @abs(joints[0].motor_target - @as(f32, @floatFromInt(instances[1].pos_x - instances[0].pos_x)));
    solveJointsForTick(instances[0..], joints[0..], entities[0..]);
    const final_error = @abs(joints[0].motor_target - @as(f32, @floatFromInt(instances[1].pos_x - instances[0].pos_x)));

    try std.testing.expect(final_error < initial_error);
}

test "configureMotor stores shared motor settings" {
    var j = Joint{
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

    setMotorEnabled(&j, false);
    try std.testing.expect(!j.motor_enabled);

    configureMotor(&j, 0.5, 7.0, 90.0);
    try std.testing.expect(j.motor_enabled);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), j.motor_target, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 7.0), j.motor_speed, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 90.0), j.motor_max_torque, 0.0001);
}

test "setMotorEnabled disables shared motor state without clearing target" {
    var j = Joint{
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

    configureMotor(&j, 0.5, 7.0, 90.0);
    setMotorEnabled(&j, false);

    try std.testing.expect(!j.motor_enabled);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), j.motor_target, 0.0001);
}

test "hinge motor also injects angular velocity bias" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .limit_min = -1.0,
        .limit_max = 1.0,
    }};
    configureMotor(&joints[0], 0.6, 6.0, 100.0);

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[0].ang_y != 0 or instances[1].ang_y != 0);
}

test "slider motor also injects linear velocity bias" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };

    var joints = [_]Joint{.{
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
        .limit_min = -2.0,
        .limit_max = 2.0,
    }};
    configureMotor(&joints[0], 1.5, 6.0, 100.0);

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[0].vel_x != 0 or instances[1].vel_x != 0);
}

test "hinge motor brakes residual angular velocity at target" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    instances[0].rot_yaw = radiansToRotationByte(0.5);
    instances[1].rot_yaw = radiansToRotationByte(0.5);
    instances[0].ang_y = 0;
    instances[1].ang_y = 3;

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .limit_min = -1.0,
        .limit_max = 1.0,
    }};
    configureMotor(&joints[0], 0.0, 6.0, 100.0);

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[1].ang_y < 3);
}

test "slider motor brakes residual linear velocity at target" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 4, 0, 0),
    };
    instances[0].vel_x = 0;
    instances[1].vel_x = 4;

    var joints = [_]Joint{.{
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
        .limit_min = -10.0,
        .limit_max = 10.0,
    }};
    configureMotor(&joints[0], 4.0, 6.0, 100.0);

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[1].vel_x < 4);
}

test "hinge limit damps angular velocity pushing past max angle" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 0, 0, 0),
    };
    instances[0].rot_yaw = radiansToRotationByte(0.0);
    instances[1].rot_yaw = radiansToRotationByte(0.5);
    instances[1].ang_y = 4;

    var joints = [_]Joint{.{
        .joint_type = .hinge,
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
        .limit_min = -0.5,
        .limit_max = 0.5,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[1].ang_y < 4);
}

test "slider limit damps linear velocity pushing past max travel" {
    var entities = [_]entity16.Entity16{ entity16.initEntity16(), entity16.initEntity16() };
    entities[0].physics.mass = 10;
    entities[1].physics.mass = 10;

    var instances = [_]scene32.Instance{
        makeTestInstance(0, 0, 0, 0),
        makeTestInstance(1, 5, 0, 0),
    };
    instances[1].vel_x = 4;

    var joints = [_]Joint{.{
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
        .limit_min = -2.0,
        .limit_max = 2.0,
    }};

    solveJointsForTick(instances[0..], joints[0..], entities[0..]);

    try std.testing.expect(instances[1].vel_x < 4);
}

test "measureJointDriveState returns hinge angle and angular velocity" {
    var inst_a = makeTestInstance(0, 0, 0, 0);
    var inst_b = makeTestInstance(1, 0, 0, 0);
    inst_a.rot_yaw = radiansToRotationByte(-0.25);
    inst_b.rot_yaw = radiansToRotationByte(0.5);
    inst_a.ang_y = -2;
    inst_b.ang_y = 3;

    const joint_def = Joint{
        .joint_type = .hinge,
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
    };

    const state = measureJointDriveState(&inst_a, &inst_b, &joint_def).?;
    try std.testing.expectApproxEqAbs(@as(f32, 0.75), state.position, 0.1);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), state.relative_velocity, 0.01);
}

test "measureJointDriveState returns slider displacement and projected velocity" {
    var inst_a = makeTestInstance(0, 0, 0, 0);
    var inst_b = makeTestInstance(1, 4, 3, 0);
    inst_a.vel_x = 1;
    inst_b.vel_x = 5;

    const joint_def = Joint{
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
    };

    const state = measureJointDriveState(&inst_a, &inst_b, &joint_def).?;
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), state.position, 0.01);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), state.relative_velocity, 0.01);
}

test "computeJointDrivePlan clamps target to joint limits" {
    const joint_def = Joint{
        .joint_type = .hinge,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .limit_min = -0.5,
        .limit_max = 0.5,
        .motor_enabled = true,
        .motor_target = 2.0,
        .motor_speed = 6.0,
        .motor_max_torque = 120.0,
    };
    const state = JointDriveState{
        .position = 0.1,
        .relative_velocity = 0.0,
    };

    const plan = computeJointDrivePlan(&joint_def, state, 0.001).?;
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), plan.target, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), plan.position_error, 0.0001);
    try std.testing.expect(plan.signed_step > 0.0);
}

test "computeJointDrivePlan enforces minimum quantized step" {
    const joint_def = Joint{
        .joint_type = .slider,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .limit_min = -10.0,
        .limit_max = 10.0,
        .motor_enabled = true,
        .motor_target = 5.0,
        .motor_speed = 0.1,
        .motor_max_torque = 0.1,
    };
    const state = JointDriveState{
        .position = 0.0,
        .relative_velocity = 0.0,
    };

    const plan = computeJointDrivePlan(&joint_def, state, 1.0).?;
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), plan.target, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 5.0), plan.position_error, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), plan.signed_step, 0.0001);
}

test "computeJointDrivePlan suppresses extra push when predicted motion already reaches target" {
    const joint_def = Joint{
        .joint_type = .hinge,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .limit_min = -1.0,
        .limit_max = 1.0,
        .motor_enabled = true,
        .motor_target = 1.0,
        .motor_speed = 6.0,
        .motor_max_torque = 120.0,
    };
    const state = JointDriveState{
        .position = 0.95,
        .relative_velocity = 3.0,
    };

    const plan = computeJointDrivePlan(&joint_def, state, 0.001).?;
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), plan.target, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.05), plan.position_error, 0.0001);
    try std.testing.expect(plan.predicted_error <= 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.signed_step, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.desired_velocity, 0.0001);
}

test "computeJointDrivePlan requests braking velocity when predicted motion overshoots target" {
    const joint_def = Joint{
        .joint_type = .hinge,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .limit_min = -1.0,
        .limit_max = 1.0,
        .motor_enabled = true,
        .motor_target = 1.0,
        .motor_speed = 6.0,
        .motor_max_torque = 120.0,
    };
    const state = JointDriveState{
        .position = 0.99,
        .relative_velocity = 6.0,
    };

    const plan = computeJointDrivePlan(&joint_def, state, 0.001).?;
    try std.testing.expect(plan.predicted_error < 0.0);
    try std.testing.expect(plan.desired_velocity < 0.0);
    try std.testing.expect(plan.signed_step < 0.0);
}

test "computeJointDrivePlan brakes residual velocity even when position is already on target" {
    const joint_def = Joint{
        .joint_type = .hinge,
        .entity_a = 0,
        .entity_b = 1,
        .anchor_a_x = 0,
        .anchor_a_y = 0,
        .anchor_a_z = 0,
        .anchor_b_x = 0,
        .anchor_b_y = 0,
        .anchor_b_z = 0,
        .limit_min = -1.0,
        .limit_max = 1.0,
        .motor_enabled = true,
        .motor_target = 1.0,
        .motor_speed = 6.0,
        .motor_max_torque = 120.0,
    };
    const state = JointDriveState{
        .position = 1.0,
        .relative_velocity = 2.0,
    };

    const plan = computeJointDrivePlan(&joint_def, state, 0.001).?;
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.position_error, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.signed_step, 0.0001);
    try std.testing.expectApproxEqAbs(@as(f32, 0.0), plan.desired_velocity, 0.0001);
    try std.testing.expect(plan.predicted_error < 0.0);
}
