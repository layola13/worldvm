//! Sports Physics Module - Items 751-800
//! Research: Running, walking, jumping, swimming, climbing, fighting, ball sports,
//! racket sports, golf, bowling, billiards, team sports, winter sports, water sports,
//! cycling, extreme sports physics

const std = @import("std");

// ============================================================================
// Item 751: Running Physics
// ============================================================================

pub const RunningModel = struct {
    stride_length: f32,
    stride_frequency: f32,
    ground_contact_time: f32,
    leg_length: f32,
    center_of_mass_height: f32,
};

pub fn computeRunningVelocity(model: *const RunningModel) f32 {
    return model.stride_length * model.stride_frequency;
}

pub fn computeRunningPower(
    model: *const RunningModel,
    body_mass: f32,
    grade: f32,
) f32 {
    const velocity = computeRunningVelocity(model);
    const metabolic_power = 4.0 * body_mass * velocity / 1000.0;
    const grade_power = body_mass * 9.81 * velocity * grade;
    return metabolic_power + grade_power;
}

pub fn computeRunningEnergyCost(velocity: f32, body_mass: f32) f32 {
    const metabolic_rate = 70.0 + 1.0 * body_mass * velocity / 1000.0;
    return metabolic_rate;
}

// ============================================================================
// Item 752: Walking Physics
// ============================================================================

pub const WalkingModel = struct {
    stride_length: f32,
    stride_frequency: f32,
    double_support_time: f32,
    step_width: f32,
};

pub fn computeWalkingVelocity(model: *const WalkingModel) f32 {
    return model.stride_length * model.stride_frequency;
}

pub fn computeWalkingStability(model: *const WalkingModel, velocity: f32) f32 {
    const lateral_distance = model.step_width * 0.5;
    const turn_rate = velocity / (model.stride_length + 0.001);
    return lateral_distance * turn_rate;
}

pub fn computeWalkingGroundReaction(
    model: *const WalkingModel,
    body_mass: f32,
    gravity: f32,
) f32 {
    const impact_force = body_mass * gravity * 1.5;
    const cycle_time = 1.0 / model.stride_frequency;
    const force_rate = impact_force / (cycle_time * model.double_support_time);
    return force_rate;
}

// ============================================================================
// Item 753: Jumping Physics
// ============================================================================

pub const JumpModel = struct {
    take_off_velocity: f32,
    flight_time: f32,
    peak_height: f32,
    leg_strength: f32,
};

pub fn computeJumpHeight(model: *const JumpModel, gravity: f32) f32 {
    const h = model.take_off_velocity * model.take_off_velocity / (2.0 * gravity);
    return @min(h, model.peak_height);
}

pub fn computeJumpDistance(
    model: *const JumpModel,
    take_off_angle: f32,
    gravity: f32,
) f32 {
    const vx = model.take_off_velocity * @cos(take_off_angle);
    const vy = model.take_off_velocity * @sin(take_off_angle);
    const flight_time = 2.0 * vy / gravity;
    return vx * flight_time;
}

pub fn computeLandingForce(
    impact_velocity: f32,
    body_mass: f32,
    landing_duration: f32,
) f32 {
    const momentum = body_mass * impact_velocity;
    return momentum / landing_duration;
}

// ============================================================================
// Item 754: Swimming Physics
// ============================================================================

pub const SwimmingModel = struct {
    drag_coefficient: f32,
    thrust_force: f32,
    body_position_angle: f32,
    kick_frequency: f32,
    arm_stroke_frequency: f32,
};

pub fn computeSwimmingDrag(
    model: *const SwimmingModel,
    velocity: f32,
    fluid_density: f32,
    cross_section_area: f32,
) f32 {
    const drag_force = 0.5 * model.drag_coefficient * fluid_density * cross_section_area * velocity * velocity;
    return drag_force;
}

pub fn computeSwimmingVelocity(
    model: *const SwimmingModel,
    drag_coefficient: f32,
    fluid_density: f32,
    cross_section_area: f32,
) f32 {
    const net_force = model.thrust_force;
    const drag_per_velocity = 0.5 * drag_coefficient * fluid_density * cross_section_area;
    return @sqrt(net_force / drag_per_velocity);
}

pub fn computeSwimmingPower(
    model: *const SwimmingModel,
    velocity: f32,
    drag_force: f32,
) f32 {
    return drag_force * velocity + model.thrust_force * velocity * 0.2;
}

// ============================================================================
// Item 755: Climbing Physics
// ============================================================================

pub const ClimbingModel = struct {
    grip_strength: f32,
    arm_span: f32,
    leg_strength: f32,
    body_weight: f32,
    wall_friction_coefficient: f32,
};

pub fn computeClimbingForce(
    model: *const ClimbingModel,
    angle_from_vertical: f32,
) f32 {
    const normal_force = model.body_weight * @cos(angle_from_vertical);
    const max_friction_force = normal_force * model.wall_friction_coefficient;
    return max_friction_force;
}

pub fn computeClimbingPower(
    model: *const ClimbingModel,
    vertical_velocity: f32,
    efficiency: f32,
) f32 {
    return model.body_weight * vertical_velocity / efficiency;
}

pub fn computeGripLimit(
    model: *const ClimbingModel,
    surface_angle: f32,
) f32 {
    const normal_component = model.body_weight * @cos(surface_angle);
    return model.grip_strength * normal_component;
}

// ============================================================================
// Item 756: Fighting Physics
// ============================================================================

pub const FightingModel = struct {
    strike_speed: f32,
    strike_force: f32,
    reaction_time: f32,
    block_coverage: f32,
    dodge_ability: f32,
};

pub fn computeStrikeImpact(
    model: *const FightingModel,
    mass: f32,
    velocity: f32,
) f32 {
    const momentum = mass * velocity;
    const kinetic_energy = 0.5 * mass * velocity * velocity;
    return model.strike_force * momentum / (kinetic_energy + 1.0);
}

pub fn computeBlockEffectiveness(
    model: *const FightingModel,
    strike_angle: f32,
    block_timing: f32,
) f32 {
    const angle_factor = 1.0 - @abs(strike_angle) / std.math.pi;
    const timing_factor = 1.0 - @abs(block_timing - model.reaction_time) / model.reaction_time;
    return model.block_coverage * angle_factor * timing_factor;
}

pub fn computeDodgeProbability(
    model: *const FightingModel,
    attack_speed: f32,
    dodge_distance: f32,
) f32 {
    const dodge_time = dodge_distance / (model.dodge_ability * 100.0);
    const attack_time = 1.0 / attack_speed;
    return @min(1.0, dodge_time / attack_time);
}

// ============================================================================
// Item 757: Ball Sports Physics
// ============================================================================

pub const BallModel = struct {
    mass: f32,
    radius: f32,
    restitution: f32,
    drag_coefficient: f32,
};

pub fn computeBallTrajectory(
    ball: *const BallModel,
    initial_velocity_x: f32,
    initial_velocity_y: f32,
    initial_velocity_z: f32,
    gravity: f32,
    dt: f32,
    steps: u32,
) struct { x: f32, y: f32, z: f32 } {
    var vx = initial_velocity_x;
    var vy = initial_velocity_y;
    var vz = initial_velocity_z;
    var x: f32 = 0;
    var y: f32 = 0;
    var z: f32 = 0;

    var i: u32 = 0;
    while (i < steps) : (i += 1) {
        const speed = @sqrt(vx * vx + vy * vy + vz * vz);
        const drag = 0.5 * ball.drag_coefficient * speed * speed * dt;
        vx -= drag * vx / (speed + 0.001);
        vy -= drag * vy / (speed + 0.001);
        vz -= drag * vz / (speed + 0.001);
        vy -= gravity * dt;
        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
    }

    return .{ .x = x, .y = y, .z = z };
}

pub fn computeBallBounce(
    ball: *const BallModel,
    incoming_velocity: f32,
    surface_angle: f32,
) f32 {
    const normal_component = incoming_velocity * @cos(surface_angle);
    const tangent_component = incoming_velocity * @sin(surface_angle);
    const reflected_normal = -normal_component * ball.restitution;
    return @sqrt(reflected_normal * reflected_normal + tangent_component * tangent_component);
}

// ============================================================================
// Item 758: Racket Sports Physics
// ============================================================================

pub const RacketModel = struct {
    head_size: f32,
    string_tension: f32,
    sweet_spot_size: f32,
    weight: f32,
    balance_point: f32,
};

pub fn computeRacketPower(
    model: *const RacketModel,
    swing_speed: f32,
    ball_velocity: f32,
) f32 {
    const power_factor = model.string_tension * model.head_size / 1000.0;
    const speed_factor = swing_speed * swing_speed / (ball_velocity + 1.0);
    return power_factor * speed_factor * 0.01;
}

pub fn computeSweetSpotEffect(
    model: *const RacketModel,
    hit_position_x: f32,
    hit_position_y: f32,
) f32 {
    const dist_from_center = @sqrt(hit_position_x * hit_position_x + hit_position_y * hit_position_y);
    const normalized_dist = dist_from_center / model.sweet_spot_size;
    return 1.0 - normalized_dist * normalized_dist * 0.3;
}

pub fn computeRacketSpin(
    model: *const RacketModel,
    swing_angle: f32,
    contact_point_offset: f32,
) f32 {
    return model.string_tension * swing_angle * contact_point_offset * 0.001;
}

// ============================================================================
// Item 759: Golf Physics
// ============================================================================

pub const GolfModel = struct {
    club_head_mass: f32,
    club_length: f32,
    loft_angle: f32,
    swing_speed: f32,
    ball_compression: f32,
};

pub fn computeGolfDriveDistance(
    model: *const GolfModel,
    launch_angle: f32,
    ball_mass: f32,
    gravity: f32,
) f32 {
    const club_head_speed = model.swing_speed * model.club_length / 0.4;
    const ball_speed = club_head_speed * (model.club_head_mass / (model.club_head_mass + ball_mass)) * 1.5;
    const vy = ball_speed * @sin(launch_angle);
    const vx = ball_speed * @cos(launch_angle);
    const flight_time = 2.0 * vy / gravity;
    return vx * flight_time;
}

pub fn computeGolfSpinRate(
    model: *const GolfModel,
    hit_location_offset: f32,
    swing_speed: f32,
) f32 {
    const backspin_factor = (model.club_length - hit_location_offset) / model.club_length;
    return backspin_factor * swing_speed * 0.1;
}

pub fn computeGolfBallFlightTime(
    initial_velocity: f32,
    launch_angle: f32,
    gravity: f32,
) f32 {
    const vy = initial_velocity * @sin(launch_angle);
    return 2.0 * vy / gravity;
}

// ============================================================================
// Item 760: Bowling Physics
// ============================================================================

pub const BowlingModel = struct {
    ball_mass: f32,
    ball_radius: f32,
    finger_hole_depth: f32,
    lane_length: f32,
    lane_friction: f32,
};

pub fn computeBowlingVelocity(
    model: *const BowlingModel,
    initial_speed: f32,
    release_angle: f32,
) f32 {
    const friction_deceleration = model.lane_friction * 9.81;
    const distance_factor = model.lane_length / 20.0;
    const release_efficiency = @max(0.3, @cos(@abs(release_angle)));
    const radius_factor = @max(0.8, 1.0 - model.ball_radius * 0.2);
    const speed = initial_speed * release_efficiency * radius_factor - friction_deceleration * distance_factor * 0.02;
    return @max(0.0, speed);
}

pub fn computeBowlingCurve(
    model: *const BowlingModel,
    axis_rotation: f32,
    velocity: f32,
    hook_potential: f32,
) f32 {
    const grip_factor = 1.0 + model.finger_hole_depth * 5.0;
    const lane_response = 1.0 + model.lane_friction * 2.0;
    const curve_force = axis_rotation * hook_potential * velocity * grip_factor * lane_response * 0.001;
    return curve_force;
}

pub fn computePinImpact(
    model: *const BowlingModel,
    pin_mass: f32,
    ball_velocity: f32,
    impact_angle: f32,
) f32 {
    const momentum_transfer = pin_mass * ball_velocity / (pin_mass + model.ball_mass);
    const angle_factor = @sin(impact_angle);
    return momentum_transfer * angle_factor;
}

// ============================================================================
// Item 761: Billiards Physics
// ============================================================================

pub const BilliardsModel = struct {
    ball_radius: f32,
    ball_mass: f32,
    friction_coefficient: f32,
    cushion_restitution: f32,
};

pub fn computeBilliardBallCollision(
    ball_a_velocity_x: f32,
    ball_a_velocity_y: f32,
    ball_b_velocity_x: f32,
    ball_b_velocity_y: f32,
    collision_normal_x: f32,
    collision_normal_y: f32,
) struct { ax: f32, ay: f32, bx: f32, by: f32 } {
    const rel_vx = ball_a_velocity_x - ball_b_velocity_x;
    const rel_vy = ball_a_velocity_y - ball_b_velocity_y;
    const rel_vn = rel_vx * collision_normal_x + rel_vy * collision_normal_y;
    if (rel_vn <= 0) {
        return .{ .ax = ball_a_velocity_x, .ay = ball_a_velocity_y, .bx = ball_b_velocity_x, .by = ball_b_velocity_y };
    }
    const impulse = rel_vn;
    const ax = ball_a_velocity_x - impulse * collision_normal_x;
    const ay = ball_a_velocity_y - impulse * collision_normal_y;
    const bx = ball_b_velocity_x + impulse * collision_normal_x;
    const by = ball_b_velocity_y + impulse * collision_normal_y;
    return .{ .ax = ax, .ay = ay, .bx = bx, .by = by };
}

pub fn computeBilliardFriction(
    model: *const BilliardsModel,
    initial_velocity: f32,
    time_elapsed: f32,
) f32 {
    const deceleration = model.friction_coefficient * 9.81;
    const final_velocity = initial_velocity - deceleration * time_elapsed;
    return @max(0.0, final_velocity);
}

pub fn computeCushionBounce(
    model: *const BilliardsModel,
    incoming_velocity: f32,
    cushion_angle: f32,
) f32 {
    const normal_velocity = incoming_velocity * @cos(cushion_angle);
    const tangent_velocity = incoming_velocity * @sin(cushion_angle);
    const reflected_normal = -normal_velocity * model.cushion_restitution;
    return @sqrt(reflected_normal * reflected_normal + tangent_velocity * tangent_velocity);
}

// ============================================================================
// Item 762: Table Tennis Physics
// ============================================================================

pub const TableTennisModel = struct {
    ball_mass: f32,
    ball_radius: f32,
    racket_speed: f32,
    spin_factor: f32,
    air_resistance: f32,
};

pub fn computeTableTennisServe(
    model: *const TableTennisModel,
    hit_angle: f32,
    spin_rate: f32,
    gravity: f32,
) struct { vx: f32, vy: f32, vz: f32, spin: f32 } {
    const gravity_compensation = @max(0.6, 1.0 - gravity * 0.005);
    const speed = model.racket_speed * 0.8 * gravity_compensation;
    const vx = speed * @cos(hit_angle);
    const vy = speed * @sin(hit_angle);
    const vz = -gravity * 0.01;
    const spin = spin_rate * model.spin_factor * (speed + gravity * 0.02);
    return .{ .vx = vx, .vy = vy, .vz = vz, .spin = spin };
}

pub fn computeSpinEffect(
    model: *const TableTennisModel,
    spin_rate: f32,
    velocity: f32,
) struct { drag_x: f32, drag_z: f32 } {
    const magnus_force = 0.5 * model.ball_mass * spin_rate * velocity * 0.0001;
    const drag_x = magnus_force * 0.1;
    const drag_z = magnus_force * 0.5;
    return .{ .drag_x = drag_x, .drag_z = drag_z };
}

pub fn computeTableTennisBallCurve(
    model: *const TableTennisModel,
    spin_rate: f32,
    velocity: f32,
    air_density: f32,
    cross_section: f32,
) f32 {
    const magnus_coefficient = 0.5 * air_density * cross_section * model.spin_factor;
    return magnus_coefficient * spin_rate * velocity;
}

// ============================================================================
// Item 763: Tennis Physics
// ============================================================================

pub const TennisModel = struct {
    court_surface: []const u8,
    ball_bounce: f32,
    racket_power: f32,
    string_pattern: u8,
};

pub fn computeTennisServeVelocity(
    model: *const TennisModel,
    swing_speed: f32,
    impact_point_height: f32,
) f32 {
    const power_factor = model.racket_power * 0.01;
    const height_bonus = (impact_point_height - 1.0) * 0.1;
    return swing_speed * power_factor * (1.0 + height_bonus);
}

pub fn computeTennisBallBounce(
    model: *const TennisModel,
    incoming_velocity: f32,
    incoming_angle: f32,
) struct { velocity: f32, angle: f32 } {
    const bounce_velocity = incoming_velocity * model.ball_bounce;
    const bounce_angle = incoming_angle * 0.8;
    return .{ .velocity = bounce_velocity, .angle = bounce_angle };
}

pub fn computeCourtSurfaceEffect(
    model: *const TennisModel,
    slide_distance: f32,
) f32 {
    const hard_court = if (std.mem.eql(u8, model.court_surface, "hard")) 1.0 else 0.0;
    const clay_court = if (std.mem.eql(u8, model.court_surface, "clay")) 1.0 else 0.0;
    const grass_court = if (std.mem.eql(u8, model.court_surface, "grass")) 1.0 else 0.0;
    const slide_factor = hard_court * 0.2 + clay_court * 0.8 + grass_court * 0.1;
    return slide_distance * slide_factor;
}

// ============================================================================
// Item 764: Badminton Physics
// ============================================================================

pub const BadmintonModel = struct {
    shuttlecock_mass: f32,
    drag_coefficient: f32,
    racket_speed: f32,
    sweet_spot_size: f32,
};

pub fn computeShuttlecockTerminalVelocity(
    model: *const BadmintonModel,
    air_density: f32,
    cross_section: f32,
) f32 {
    const weight = model.shuttlecock_mass * 9.81;
    const drag_factor = 0.5 * model.drag_coefficient * air_density * cross_section;
    return @sqrt(weight / drag_factor);
}

pub fn computeBadmintonSmash(
    model: *const BadmintonModel,
    swing_speed: f32,
    impact_height: f32,
) f32 {
    const power = swing_speed * swing_speed * model.racket_speed * 0.001;
    const height_bonus = impact_height * 0.05;
    return power * (1.0 + height_bonus);
}

pub fn computeShuttlecockFlight(
    model: *const BadmintonModel,
    initial_velocity: f32,
    time_elapsed: f32,
) f32 {
    const drag_deceleration = model.drag_coefficient * initial_velocity * time_elapsed * 0.01;
    return @max(0.0, initial_velocity - drag_deceleration);
}

// ============================================================================
// Item 765: Soccer Physics
// ============================================================================

pub const SoccerModel = struct {
    ball_mass: f32,
    ball_radius: f32,
    air_resistance: f32,
    grass_friction: f32,
};

pub fn computeSoccerKick(
    model: *const SoccerModel,
    kick_force: f32,
    kick_angle: f32,
    ball_mass: f32,
) f32 {
    const effective_mass = if (ball_mass > 0.0) ball_mass else model.ball_mass;
    const impulse = kick_force * 0.01;
    const launch_speed = impulse / (effective_mass + 0.0001);
    const drag_loss = model.air_resistance * launch_speed * 0.02;
    const ground_loss = model.grass_friction * launch_speed * 0.03;
    return @max(0.0, launch_speed - drag_loss - ground_loss) * @cos(kick_angle);
}

pub fn computeSoccerBallCurve(
    model: *const SoccerModel,
    spin_rate: f32,
    velocity: f32,
    magnus_coefficient: f32,
) f32 {
    const flow_factor = @max(0.2, 1.0 - model.air_resistance * 0.3);
    return magnus_coefficient * spin_rate * velocity * flow_factor * 0.001;
}

pub fn computeSoccerBallDip(
    model: *const SoccerModel,
    velocity: f32,
    spin_rate: f32,
    gravity: f32,
) f32 {
    const downward_force = gravity * 0.5;
    const magnus_lift = model.air_resistance * spin_rate * velocity * 0.0001;
    return downward_force - magnus_lift;
}

pub fn computeGoalKeeperDiveReach(
    model: *const SoccerModel,
    initial_position_x: f32,
    initial_position_y: f32,
    dive_angle: f32,
    dive_speed: f32,
    time_to_ball: f32,
) struct { x: f32, y: f32 } {
    const surface_drag = 1.0 / (1.0 + model.grass_friction * time_to_ball);
    const aero_drag = 1.0 / (1.0 + model.air_resistance * 0.5 * time_to_ball);
    const dive_distance = dive_speed * time_to_ball * surface_drag * aero_drag;
    const x = initial_position_x + dive_distance * @cos(dive_angle);
    const y = initial_position_y + dive_distance * @sin(dive_angle);
    return .{ .x = x, .y = y };
}

// ============================================================================
// Item 766: Basketball Physics
// ============================================================================

pub const BasketballModel = struct {
    ball_circumference: f32,
    ball_mass: f32,
    rim_radius: f32,
    backboard_distance: f32,
};

pub fn computeBasketballShot(
    model: *const BasketballModel,
    release_velocity: f32,
    release_angle: f32,
    release_height: f32,
    gravity: f32,
) struct { distance: f32, entry_angle: f32 } {
    const vx = release_velocity * @cos(release_angle);
    const vy = release_velocity * @sin(release_angle);
    const target_height: f32 = 3.05;
    const a: f32 = -0.5 * gravity;
    const b: f32 = vy;
    const c: f32 = release_height - target_height;
    const discriminant = b * b - 4.0 * a * c;

    var time_to_rim: f32 = 0.0;
    if (discriminant >= 0.0 and @abs(a) > 0.00001) {
        const sqrt_disc = @sqrt(discriminant);
        const t1 = (-b + sqrt_disc) / (2.0 * a);
        const t2 = (-b - sqrt_disc) / (2.0 * a);

        const t1_valid = t1 > 0.0;
        const t2_valid = t2 > 0.0;
        if (t1_valid and t2_valid) {
            time_to_rim = @max(t1, t2);
        } else if (t1_valid) {
            time_to_rim = t1;
        } else if (t2_valid) {
            time_to_rim = t2;
        }
    }

    if (time_to_rim <= 0.0) {
        const fallback_distance = @max(model.backboard_distance, model.rim_radius * 4.0);
        time_to_rim = fallback_distance / (@max(0.1, vx));
    }

    const distance = @max(0.0, vx * time_to_rim);
    const vy_at_rim = vy - gravity * time_to_rim;
    const entry_angle = std.math.atan2(vy_at_rim, @max(0.1, vx));
    return .{ .distance = distance, .entry_angle = entry_angle };
}

pub fn computeBasketballBankShot(
    model: *const BasketballModel,
    incoming_angle: f32,
    backboard_restitution: f32,
) f32 {
    const distance_penalty = 1.0 / (1.0 + model.backboard_distance * 2.0);
    return incoming_angle * backboard_restitution * distance_penalty;
}

pub fn computeBasketballBounce(
    model: *const BasketballModel,
    incoming_velocity: f32,
    surface_restitution: f32,
    friction: f32,
) struct { vx: f32, vy: f32 } {
    const inertia_factor = 1.0 / (1.0 + model.ball_mass);
    const tangential_loss = @max(0.0, 1.0 - friction * (0.2 + inertia_factor));
    const vy = incoming_velocity * surface_restitution * (0.9 + 0.1 * inertia_factor);
    const vx = incoming_velocity * tangential_loss;
    return .{ .vx = vx, .vy = vy };
}

// ============================================================================
// Item 767: Volleyball Physics
// ============================================================================

pub const VolleyballModel = struct {
    ball_mass: f32,
    ball_radius: f32,
    net_height: f32,
    court_length: f32,
};

pub fn computeVolleyballServe(
    model: *const VolleyballModel,
    impact_velocity: f32,
    impact_angle: f32,
    player_height: f32,
    gravity: f32,
) f32 {
    const vx = impact_velocity * @cos(impact_angle);
    const vy = impact_velocity * @sin(impact_angle);
    if (vx <= 0.0) return 0.0;

    const half_court = model.court_length * 0.5;
    const time_to_net = half_court / vx;
    const height_at_net = player_height + vy * time_to_net - 0.5 * gravity * time_to_net * time_to_net;
    if (height_at_net <= model.net_height) return 0.0;

    const flight_time = (vy + @sqrt(vy * vy + 2.0 * gravity * player_height)) / gravity;
    return vx * flight_time;
}

pub fn computeVolleyballSpike(
    model: *const VolleyballModel,
    approach_velocity: f32,
    player_height: f32,
    vertical_jump: f32,
    arm_swing_speed: f32,
    gravity: f32,
) f32 {
    const peak_height = player_height + vertical_jump + arm_swing_speed * 0.5;
    const vy = @sqrt(2.0 * gravity * peak_height);
    const inertia = model.ball_mass * (model.ball_radius + 0.01);
    const swing_transfer = arm_swing_speed / (inertia + 0.1);
    return approach_velocity * 1.2 + swing_transfer + vy * 0.3;
}

pub fn computeVolleyballBlock(
    model: *const VolleyballModel,
    blocker_height: f32,
    attacker_height: f32,
    net_height: f32,
) f32 {
    const block_reach = blocker_height - net_height;
    const attack_angle = std.math.atan((attacker_height - net_height) / model.court_length);
    return block_reach * @tan(attack_angle);
}

// ============================================================================
// Item 768: Rugby Physics
// ============================================================================

pub const RugbyModel = struct {
    ball_mass: f32,
    ball_length: f32,
    try_distance: f32,
    conversion_angle: f32,
};

pub fn computeRugbyKick(
    model: *const RugbyModel,
    kick_velocity: f32,
    kick_angle: f32,
    gravity: f32,
) struct { distance: f32, height_at_goal: f32 } {
    const vx = kick_velocity * @cos(kick_angle);
    const vy = kick_velocity * @sin(kick_angle);
    const time = model.try_distance / vx;
    const height_at_goal = vy * time - 0.5 * gravity * time * time;
    return .{ .distance = model.try_distance, .height_at_goal = height_at_goal };
}

pub fn computeRugbyTackle(
    model: *const RugbyModel,
    tackler_velocity: f32,
    ball_carrier_mass: f32,
    tackler_mass: f32,
) f32 {
    const momentum_transfer = tackler_velocity * tackler_mass / (ball_carrier_mass + tackler_mass);
    const shape_drag = 1.0 + model.ball_length * 0.5;
    return momentum_transfer / shape_drag;
}

pub fn computeRugbyConversionKick(
    model: *const RugbyModel,
    from_try_line: f32,
    kick_angle_from_goal: f32,
    gravity: f32,
) f32 {
    const distance = from_try_line + model.conversion_angle * 10.0;
    const required_velocity = @sqrt(distance * gravity / @sin(2.0 * kick_angle_from_goal));
    return required_velocity;
}

// ============================================================================
// Item 769: Baseball Physics
// ============================================================================

pub const BaseballModel = struct {
    ball_mass: f32,
    ball_radius: f32,
    bat_mass: f32,
    bat_length: f32,
};

pub fn computeBaseballBatSpeed(
    model: *const BaseballModel,
    swing_torque: f32,
    contact_time: f32,
) f32 {
    const angular_acceleration = swing_torque / (model.bat_mass * model.bat_length * model.bat_length / 3.0);
    const final_angular_velocity = angular_acceleration * contact_time;
    return final_angular_velocity * model.bat_length;
}

pub fn computeBaseballBallExitVelocity(
    bat_speed: f32,
    ball_speed: f32,
    mass_ratio: f32,
) f32 {
    const collision_efficiency = 0.2;
    const relative_velocity = bat_speed + ball_speed;
    return relative_velocity * collision_efficiency * (1.0 + mass_ratio);
}

pub fn computeBaseballCurveBall(
    model: *const BaseballModel,
    spin_rate: f32,
    velocity: f32,
    magnus_coefficient: f32,
) f32 {
    const magnus_force = magnus_coefficient * spin_rate * velocity * 0.0001;
    return magnus_force * model.bat_length;
}

pub fn computeBaseballHomeRunDistance(
    exit_velocity: f32,
    launch_angle: f32,
    gravity: f32,
) f32 {
    const vy = exit_velocity * @sin(launch_angle);
    const vx = exit_velocity * @cos(launch_angle);
    const flight_time = 2.0 * vy / gravity;
    return vx * flight_time;
}

// ============================================================================
// Item 770: Golf Ball Flight Physics
// ============================================================================

pub fn computeGolfBallFlight(
    initial_velocity: f32,
    launch_angle: f32,
    spin_rate: f32,
    gravity: f32,
    air_density: f32,
    cross_section: f32,
    dt: f32,
) struct { x: f32, y: f32, z: f32 } {
    var vx = initial_velocity * @cos(launch_angle);
    var vy = initial_velocity * @sin(launch_angle);
    var vz: f32 = 0.0;
    var x: f32 = 0.0;
    var y: f32 = 0.0;
    var z: f32 = 0.0;
    const magnus_coefficient = 0.5 * air_density * cross_section;

    const total_time = 2.0 * initial_velocity * @sin(launch_angle) / gravity;
    const steps: u32 = @intFromFloat(total_time / dt);

    var i: u32 = 0;
    while (i < steps) : (i += 1) {
        const speed = @sqrt(vx * vx + vy * vy + vz * vz);
        const drag = 0.5 * 0.4 * air_density * cross_section * speed * speed;
        const magnus = magnus_coefficient * spin_rate * speed * 0.001;

        vx -= (drag * vx / (speed + 0.001)) * dt;
        vy -= (drag * vy / (speed + 0.001) + gravity) * dt;
        vz += magnus * dt;

        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
    }

    return .{ .x = x, .y = y, .z = z };
}

// ============================================================================
// Item 771: Ski Physics
// ============================================================================

pub const SkiModel = struct {
    ski_length: f32,
    ski_width: f32,
    edge_angle: f32,
    snow_friction: f32,
};

pub fn computeSkiTurnRadius(
    model: *const SkiModel,
    velocity: f32,
    edge_angle: f32,
    gravity: f32,
) f32 {
    const effective_edge = @max(0.01, edge_angle + model.edge_angle * 0.5);
    const ski_shape_factor = 1.0 + model.ski_width / @max(0.01, model.ski_length);
    const centripetal_force = gravity * @sin(effective_edge) * ski_shape_factor;
    const curvature = centripetal_force / (@max(0.01, velocity * velocity));
    return 1.0 / (curvature + 0.001);
}

pub fn computeSkiSpeed(
    model: *const SkiModel,
    slope_angle: f32,
    velocity: f32,
    gravity: f32,
) f32 {
    const acceleration = gravity * @sin(slope_angle) - model.snow_friction * gravity * @cos(slope_angle);
    return velocity + acceleration * 0.1;
}

pub fn computeSkiGForce(
    model: *const SkiModel,
    turn_radius: f32,
    velocity: f32,
    gravity: f32,
) f32 {
    const edge_stability = 1.0 + model.edge_angle * 0.5;
    const centripetal = velocity * velocity / (@max(0.1, turn_radius) * gravity);
    const effective_centripetal = centripetal * edge_stability;
    return @sqrt(1.0 + effective_centripetal * effective_centripetal);
}

// ============================================================================
// Item 772: Skate Physics
// ============================================================================

pub const SkateModel = struct {
    blade_length: f32,
    blade_radius: f32,
    wheel_circumference: f32,
    friction: f32,
};

pub fn computeSkateVelocity(
    model: *const SkateModel,
    stride_frequency: f32,
    stride_length: f32,
) f32 {
    const stride_distance = stride_length + model.wheel_circumference * 0.25;
    const slip_loss = model.friction * 0.2;
    return @max(0.0, stride_frequency * stride_distance * (1.0 - slip_loss));
}

pub fn computeSkateTurn(
    model: *const SkateModel,
    lean_angle: f32,
    velocity: f32,
    gravity: f32,
) f32 {
    const centripetal = gravity * @tan(lean_angle);
    const grip_factor = @max(0.1, 1.0 - model.friction * 8.0);
    return velocity * velocity / (centripetal * grip_factor + 0.001);
}

pub fn computeSkateFriction(
    model: *const SkateModel,
    normal_force: f32,
    velocity: f32,
) f32 {
    return model.friction * normal_force * velocity * 0.001;
}

// ============================================================================
// Item 773: Surf Physics
// ============================================================================

pub const SurfModel = struct {
    board_length: f32,
    board_width: f32,
    board_volume: f32,
    fin_size: f32,
};

pub fn computeSurfboardSpeed(
    model: *const SurfModel,
    wave_velocity: f32,
    board_length: f32,
) f32 {
    const effective_length = if (board_length > 0.0) board_length else model.board_length;
    const buoyancy_bonus = @min(1.5, model.board_volume * 8.0);
    const drag_loss = @min(0.4, model.board_width * 0.04);
    const glider_speed = @sqrt(wave_velocity * wave_velocity + 9.81 * effective_length * 0.01 * buoyancy_bonus);
    return @max(0.0, glider_speed * (1.0 - drag_loss));
}

pub fn computeSurfTurn(
    model: *const SurfModel,
    wave_direction: f32,
    board_angle: f32,
    wave_velocity: f32,
) f32 {
    const angle_diff = wave_direction - board_angle;
    const turn_rate = wave_velocity * @sin(angle_diff) * model.fin_size * 0.01;
    return turn_rate;
}

pub fn computeSurfboardDrag(
    model: *const SurfModel,
    velocity: f32,
    water_density: f32,
) f32 {
    const drag_coefficient = 0.1;
    const cross_section = model.board_width * 0.1;
    return 0.5 * drag_coefficient * water_density * cross_section * velocity * velocity;
}

// ============================================================================
// Item 774: Dive Physics
// ============================================================================

pub const DiveModel = struct {
    body_length: f32,
    moment_of_inertia: f32,
    entry_velocity: f32,
    entry_angle: f32,
};

pub fn computeDiveRotationRate(
    model: *const DiveModel,
    angular_momentum: f32,
) f32 {
    return angular_momentum / @max(0.01, model.moment_of_inertia);
}

pub fn computeDiveEntryForce(
    model: *const DiveModel,
    entry_velocity: f32,
    body_mass: f32,
    splash_duration: f32,
) f32 {
    const momentum = body_mass * entry_velocity;
    const aligned_entry = @max(0.2, @cos(@abs(model.entry_angle - std.math.pi / 2.0)));
    return momentum * aligned_entry / @max(0.001, splash_duration);
}

pub fn computeDiveAngleCorrectness(
    model: *const DiveModel,
    actual_angle: f32,
    target_angle: f32,
) f32 {
    const angle_error = @abs(actual_angle - target_angle);
    const tolerance_scale = @max(0.3, model.body_length * 0.05);
    return @max(0.0, @min(1.0, 1.0 - angle_error / (std.math.pi * tolerance_scale)));
}

// ============================================================================
// Item 775: Cycling Physics
// ============================================================================

pub const CyclingModel = struct {
    wheel_radius: f32,
    crank_length: f32,
    gear_ratio: f32,
    drag_coefficient: f32,
};

pub fn computeCyclingSpeed(
    model: *const CyclingModel,
    cadence: f32,
    gear_ratio: f32,
) f32 {
    const wheel_circumference = 2.0 * std.math.pi * model.wheel_radius;
    const wheel_rpm = cadence * gear_ratio;
    return wheel_circumference * wheel_rpm / 60.0;
}

pub fn computeCyclingPower(
    model: *const CyclingModel,
    speed: f32,
    grade: f32,
    body_mass: f32,
    gravity: f32,
    air_density: f32,
    frontal_area: f32,
) f32 {
    const rolling_resistance = 0.004 * body_mass * gravity * @cos(std.math.atan(grade));
    const hill_power = body_mass * gravity * speed * @sin(std.math.atan(grade));
    const drag_power = 0.5 * model.drag_coefficient * air_density * frontal_area * speed * speed * speed;
    return rolling_resistance * speed + hill_power + drag_power;
}

pub fn computeCyclingCadence(
    model: *const CyclingModel,
    speed: f32,
    target_cadence: f32,
) f32 {
    const current_cadence = speed / (2.0 * std.math.pi * model.wheel_radius) * 60.0 / model.gear_ratio;
    return current_cadence * 0.7 + target_cadence * 0.3;
}

// ============================================================================
// Item 776-800: Additional Sports and Extreme Sports Physics
// ============================================================================

pub const ExtremeSportsModel = struct {
    sport_type: []const u8,
    air_resistance: f32,
    impact_tolerance: f32,
    control_factor: f32,
};

pub fn computeSkateboardTrickRotation(
    model: *const ExtremeSportsModel,
    angular_velocity: f32,
    rotation_angle: f32,
) f32 {
    return angular_velocity * rotation_angle * model.control_factor * 0.01;
}

pub fn computeParkourVaultSpeed(
    model: *const ExtremeSportsModel,
    approach_velocity: f32,
    obstacle_height: f32,
) f32 {
    const speed_loss = obstacle_height * (0.12 - model.control_factor * 0.05);
    return @max(0.0, approach_velocity - speed_loss);
}

pub fn computeBungeeJumpTension(
    model: *const ExtremeSportsModel,
    rope_length: f32,
    stretched_length: f32,
    spring_constant: f32,
) f32 {
    const stretch = stretched_length - rope_length;
    const damping = @max(0.5, model.impact_tolerance);
    return spring_constant * @max(0.0, stretch) * damping;
}

pub fn computeSkydivingTerminalVelocity(
    model: *const ExtremeSportsModel,
    body_mass: f32,
    gravity: f32,
    air_density: f32,
    frontal_area: f32,
) f32 {
    const drag_factor = 0.5 * model.air_resistance * air_density * frontal_area;
    return @sqrt(body_mass * gravity / drag_factor);
}

pub fn computeSkydivingFreefallSpeed(
    model: *const ExtremeSportsModel,
    terminal_velocity: f32,
    time_elapsed: f32,
) f32 {
    const approach_rate = terminal_velocity * (0.08 + model.control_factor * 0.08);
    return terminal_velocity * (1.0 - @exp(-time_elapsed * approach_rate / @max(0.1, terminal_velocity)));
}

pub fn computeGliderFlightDistance(
    model: *const ExtremeSportsModel,
    altitude: f32,
    lift_coefficient: f32,
    drag_coefficient: f32,
    glide_ratio: f32,
) f32 {
    const lift_to_drag = lift_coefficient / @max(0.001, drag_coefficient);
    const control_gain = 0.6 + model.control_factor * 0.4;
    const drag_loss = @max(0.2, 1.0 - model.air_resistance * 0.3);
    return altitude * lift_to_drag * glide_ratio * control_gain * drag_loss * 0.01;
}

pub fn computeHotAirBalloonLift(
    model: *const ExtremeSportsModel,
    balloon_volume: f32,
    temperature_inside: f32,
    temperature_outside: f32,
    air_density: f32,
    gravity: f32,
) f32 {
    const temp_ratio = temperature_inside / temperature_outside;
    const density_difference = air_density * (1.0 - 1.0 / temp_ratio);
    const control_bonus = 0.8 + model.control_factor * 0.2;
    return balloon_volume * density_difference * gravity * control_bonus;
}

pub fn computeWingsuitGlideRatio(
    model: *const ExtremeSportsModel,
    wing_area: f32,
    aspect_ratio: f32,
    lift_coefficient: f32,
    drag_coefficient: f32,
) f32 {
    const induced_drag = lift_coefficient * lift_coefficient / (std.math.pi * @max(0.1, aspect_ratio));
    const wing_efficiency = 1.0 + wing_area * 0.05;
    const suit_drag = drag_coefficient + induced_drag + model.air_resistance * 0.02;
    return (lift_coefficient * wing_efficiency * (0.8 + model.control_factor * 0.2)) / suit_drag;
}

pub fn computeMotorcycleCorneringSpeed(
    model: *const ExtremeSportsModel,
    corner_radius: f32,
    lean_angle: f32,
    gravity: f32,
) f32 {
    const grip_factor = 0.7 + model.control_factor * 0.3;
    const aero_loss = @max(0.4, 1.0 - model.air_resistance * 0.2);
    const centripetal = gravity * @tan(lean_angle) * grip_factor * aero_loss;
    return @sqrt(corner_radius * centripetal);
}

pub fn computeKartingGForce(
    model: *const ExtremeSportsModel,
    velocity: f32,
    track_radius: f32,
    gravity: f32,
) f32 {
    const centripetal = velocity * velocity / (track_radius * gravity);
    const control_gain = 0.8 + model.control_factor * 0.2;
    return 1.0 + centripetal * 0.5 * control_gain;
}

pub fn computeOffroadSuspensionLoad(
    model: *const ExtremeSportsModel,
    vehicle_mass: f32,
    obstacle_height: f32,
    spring_rate: f32,
) f32 {
    const base_load = vehicle_mass * 9.81;
    const terrain_impulse = spring_rate * obstacle_height * 0.1 * (1.0 + model.air_resistance * 0.1);
    return base_load + terrain_impulse * (1.0 + (1.0 - model.impact_tolerance) * 0.2);
}

pub fn computeMountaineeringAltitudeEffect(
    model: *const ExtremeSportsModel,
    altitude: f32,
    base_oxygen: f32,
) f32 {
    const oxygen_reduction = altitude * 0.0001;
    const adaptation = 0.7 + model.control_factor * 0.3;
    return @max(0.0, base_oxygen * (1.0 - oxygen_reduction * adaptation));
}

pub fn computeDownhillSpeed(
    model: *const ExtremeSportsModel,
    slope_angle: f32,
    velocity: f32,
    gravity: f32,
    friction: f32,
) f32 {
    const aerodynamic_drag = model.air_resistance * velocity * velocity * 0.002;
    const control_brake = (1.0 - model.control_factor) * 0.2 * velocity;
    const acceleration = gravity * (@sin(slope_angle) - friction * @cos(slope_angle)) - aerodynamic_drag - control_brake;
    return @max(0.0, velocity + acceleration * 0.1);
}

pub fn computeBMXAcceleration(
    model: *const ExtremeSportsModel,
    pedal_force: f32,
    gear_ratio: f32,
    wheel_radius: f32,
    total_mass: f32,
) f32 {
    const torque = pedal_force * gear_ratio * (0.8 + model.control_factor * 0.2);
    const wheel_force = torque / @max(0.05, wheel_radius);
    return wheel_force / @max(1.0, total_mass);
}

pub fn computeUnicycleBalance(
    model: *const ExtremeSportsModel,
    lean_angle: f32,
    wheel_speed: f32,
    wheel_radius: f32,
) f32 {
    const correction_rate = wheel_speed / @max(0.05, wheel_radius);
    const controller_gain = 0.5 + model.control_factor * 0.5;
    return @abs(lean_angle) * correction_rate * controller_gain * 0.1;
}

pub fn computeScooterBrakingDistance(
    model: *const ExtremeSportsModel,
    initial_speed: f32,
    friction_coefficient: f32,
    gravity: f32,
) f32 {
    const effective_friction = friction_coefficient * (0.8 + model.control_factor * 0.2);
    const deceleration = @max(0.1, effective_friction * gravity);
    return initial_speed * initial_speed / (2.0 * deceleration);
}

pub fn computeDriftAngle(
    model: *const ExtremeSportsModel,
    velocity: f32,
    lateral_velocity: f32,
    gravity: f32,
    friction: f32,
) f32 {
    const slip_angle = @abs(std.math.atan(lateral_velocity / (velocity + 0.001)));
    const grip_limit = std.math.atan(friction * gravity / @max(1.0, velocity));
    const control_limit = grip_limit * (0.8 + model.control_factor * 0.2);
    return @min(slip_angle, control_limit);
}

pub fn computeDriftForce(
    model: *const ExtremeSportsModel,
    mass: f32,
    lateral_acceleration: f32,
) f32 {
    const aero_multiplier = 1.0 + model.air_resistance * 0.1;
    return mass * lateral_acceleration * aero_multiplier;
}

// ============================================================================
// Tests for Sports Physics (Items 751-800)
// ============================================================================

test "751: running velocity computation" {
    var model = RunningModel{
        .stride_length = 1.0,
        .stride_frequency = 2.0,
        .ground_contact_time = 0.2,
        .leg_length = 0.9,
        .center_of_mass_height = 1.0,
    };
    const velocity = computeRunningVelocity(&model);
    try std.testing.expect(velocity > 0);
}

test "751: running power computation" {
    var model = RunningModel{
        .stride_length = 1.0,
        .stride_frequency = 2.0,
        .ground_contact_time = 0.2,
        .leg_length = 0.9,
        .center_of_mass_height = 1.0,
    };
    const power = computeRunningPower(&model, 70.0, 0.05);
    try std.testing.expect(power > 0);
}

test "752: walking velocity computation" {
    var model = WalkingModel{
        .stride_length = 0.7,
        .stride_frequency = 1.8,
        .double_support_time = 0.1,
        .step_width = 0.2,
    };
    const velocity = computeWalkingVelocity(&model);
    try std.testing.expect(velocity > 0);
}

test "753: jump height computation" {
    var model = JumpModel{
        .take_off_velocity = 3.0,
        .flight_time = 0.6,
        .peak_height = 1.5,
        .leg_strength = 2000.0,
    };
    const height = computeJumpHeight(&model, 9.81);
    try std.testing.expect(height > 0);
}

test "753: jump distance computation" {
    var model = JumpModel{
        .take_off_velocity = 3.0,
        .flight_time = 0.6,
        .peak_height = 1.5,
        .leg_strength = 2000.0,
    };
    const distance = computeJumpDistance(&model, std.math.pi / 4.0, 9.81);
    try std.testing.expect(distance > 0);
}

test "754: swimming drag computation" {
    var model = SwimmingModel{
        .drag_coefficient = 0.9,
        .thrust_force = 100.0,
        .body_position_angle = 0.1,
        .kick_frequency = 2.0,
        .arm_stroke_frequency = 1.0,
    };
    const drag = computeSwimmingDrag(&model, 1.0, 1000.0, 0.3);
    try std.testing.expect(drag > 0);
}

test "755: climbing force computation" {
    var model = ClimbingModel{
        .grip_strength = 500.0,
        .arm_span = 2.0,
        .leg_strength = 1000.0,
        .body_weight = 700.0,
        .wall_friction_coefficient = 0.5,
    };
    const force = computeClimbingForce(&model, std.math.pi / 6.0);
    try std.testing.expect(force > 0);
}

test "756: fighting strike impact" {
    var model = FightingModel{
        .strike_speed = 10.0,
        .strike_force = 500.0,
        .reaction_time = 0.2,
        .block_coverage = 0.6,
        .dodge_ability = 0.8,
    };
    const impact = computeStrikeImpact(&model, 5.0, 8.0);
    try std.testing.expect(impact > 0);
}

test "757: ball trajectory" {
    var ball = BallModel{
        .mass = 0.4,
        .radius = 0.11,
        .restitution = 0.85,
        .drag_coefficient = 0.5,
    };
    const pos = computeBallTrajectory(&ball, 10.0, 5.0, 0.0, 9.81, 0.01, 100);
    try std.testing.expect(pos.x > 0);
}

test "758: racket power" {
    var model = RacketModel{
        .head_size = 650.0,
        .string_tension = 25.0,
        .sweet_spot_size = 0.08,
        .weight = 0.3,
        .balance_point = 0.32,
    };
    const power = computeRacketPower(&model, 30.0, 20.0);
    try std.testing.expect(power > 0);
}

test "759: golf drive distance" {
    var model = GolfModel{
        .club_head_mass = 0.2,
        .club_length = 0.9,
        .loft_angle = 0.2,
        .swing_speed = 40.0,
        .ball_compression = 0.8,
    };
    const distance = computeGolfDriveDistance(&model, std.math.pi / 6.0, 0.046, 9.81);
    try std.testing.expect(distance > 0);
}

test "760: bowling velocity" {
    var model = BowlingModel{
        .ball_mass = 7.0,
        .ball_radius = 0.11,
        .finger_hole_depth = 0.02,
        .lane_length = 20.0,
        .lane_friction = 0.2,
    };
    const straight_velocity = computeBowlingVelocity(&model, 8.0, 0.0);
    const angled_velocity = computeBowlingVelocity(&model, 8.0, 0.4);
    try std.testing.expect(straight_velocity > angled_velocity);
    try std.testing.expect(angled_velocity > 0);
}

test "761: billiards collision" {
    const result = computeBilliardBallCollision(5.0, 0.0, -5.0, 0.0, 1.0, 0.0);
    try std.testing.expect(result.ax < 5.0);
}

test "762: table tennis serve" {
    var model = TableTennisModel{
        .ball_mass = 0.0027,
        .ball_radius = 0.02,
        .racket_speed = 20.0,
        .spin_factor = 0.8,
        .air_resistance = 0.5,
    };
    const serve = computeTableTennisServe(&model, 0.3, 100.0, 9.81);
    try std.testing.expect(serve.vx > 0);
}

test "763: tennis serve velocity" {
    var model = TennisModel{
        .court_surface = "clay",
        .ball_bounce = 0.7,
        .racket_power = 70.0,
        .string_pattern = 16,
    };
    const velocity = computeTennisServeVelocity(&model, 40.0, 2.2);
    try std.testing.expect(velocity > 0);
}

test "764: badminton terminal velocity" {
    var model = BadmintonModel{
        .shuttlecock_mass = 0.005,
        .drag_coefficient = 0.6,
        .racket_speed = 30.0,
        .sweet_spot_size = 0.05,
    };
    const velocity = computeShuttlecockTerminalVelocity(&model, 1.2, 0.005);
    try std.testing.expect(velocity > 0);
}

test "765: soccer kick" {
    var model = SoccerModel{
        .ball_mass = 0.43,
        .ball_radius = 0.11,
        .air_resistance = 0.3,
        .grass_friction = 0.5,
    };
    const velocity = computeSoccerKick(&model, 1000.0, std.math.pi / 4.0, 0.43);
    const heavier_ball_velocity = computeSoccerKick(&model, 1000.0, std.math.pi / 4.0, 0.60);
    try std.testing.expect(velocity > heavier_ball_velocity);
    try std.testing.expect(heavier_ball_velocity > 0);
}

test "766: basketball shot" {
    var model = BasketballModel{
        .ball_circumference = 0.75,
        .ball_mass = 0.62,
        .rim_radius = 0.23,
        .backboard_distance = 0.15,
    };
    const shot = computeBasketballShot(&model, 7.0, std.math.pi / 4.0, 2.0, 9.81);
    try std.testing.expect(shot.distance > 0);
    try std.testing.expect(!std.math.isNan(shot.entry_angle));
    try std.testing.expect(@abs(shot.entry_angle) <= std.math.pi / 2.0);
}

test "767: volleyball serve" {
    var model = VolleyballModel{
        .ball_mass = 0.27,
        .ball_radius = 0.11,
        .net_height = 2.43,
        .court_length = 18.0,
    };
    const distance = computeVolleyballServe(&model, 20.0, std.math.pi / 6.0, 2.0, 9.81);
    try std.testing.expect(distance > model.court_length * 0.5);
}

test "768: rugby kick" {
    var model = RugbyModel{
        .ball_mass = 0.43,
        .ball_length = 0.28,
        .try_distance = 10.0,
        .conversion_angle = 0.3,
    };
    const kick = computeRugbyKick(&model, 20.0, std.math.pi / 4.0, 9.81);
    try std.testing.expect(kick.distance > 0);
}

test "769: baseball bat speed" {
    var model = BaseballModel{
        .ball_mass = 0.145,
        .ball_radius = 0.037,
        .bat_mass = 0.9,
        .bat_length = 0.86,
    };
    const speed = computeBaseballBatSpeed(&model, 500.0, 0.01);
    try std.testing.expect(speed > 0);
}

test "770: golf ball flight" {
    const flight = computeGolfBallFlight(70.0, std.math.pi / 6.0, 3000.0, 9.81, 1.2, 0.003, 0.01);
    try std.testing.expect(flight.x > 0);
}

test "771: ski turn radius" {
    var model = SkiModel{
        .ski_length = 1.7,
        .ski_width = 0.05,
        .edge_angle = 0.3,
        .snow_friction = 0.02,
    };
    const radius = computeSkiTurnRadius(&model, 15.0, std.math.pi / 6.0, 9.81);
    try std.testing.expect(radius > 0);
}

test "772: skate velocity" {
    var model = SkateModel{
        .blade_length = 0.4,
        .blade_radius = 0.015,
        .wheel_circumference = 2.0,
        .friction = 0.005,
    };
    const velocity = computeSkateVelocity(&model, 1.0, 1.5);
    try std.testing.expect(velocity > 0);
}

test "773: surfboard speed" {
    var model = SurfModel{
        .board_length = 2.0,
        .board_width = 0.6,
        .board_volume = 0.06,
        .fin_size = 0.15,
    };
    const speed = computeSurfboardSpeed(&model, 5.0, 2.0);
    try std.testing.expect(speed > 0);
}

test "774: dive rotation rate" {
    var model = DiveModel{
        .body_length = 1.8,
        .moment_of_inertia = 1.2,
        .entry_velocity = 10.0,
        .entry_angle = std.math.pi / 2.0,
    };
    const rate = computeDiveRotationRate(&model, 5.0);
    try std.testing.expect(rate > 0);
}

test "775: cycling speed" {
    var model = CyclingModel{
        .wheel_radius = 0.35,
        .crank_length = 0.175,
        .gear_ratio = 3.0,
        .drag_coefficient = 0.4,
    };
    const speed = computeCyclingSpeed(&model, 90.0, 3.0);
    try std.testing.expect(speed > 0);
}

test "776: skateboard trick rotation" {
    var model = ExtremeSportsModel{
        .sport_type = "skateboard",
        .air_resistance = 0.4,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const rotation = computeSkateboardTrickRotation(&model, 10.0, 2.0 * std.math.pi);
    try std.testing.expect(rotation > 0);
}

test "777: parkour vault speed" {
    var model = ExtremeSportsModel{
        .sport_type = "parkour",
        .air_resistance = 0.4,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const speed = computeParkourVaultSpeed(&model, 5.0, 1.5);
    try std.testing.expect(speed > 4.0);
}

test "778: bungee jump tension" {
    var model = ExtremeSportsModel{
        .sport_type = "bungee",
        .air_resistance = 0.3,
        .impact_tolerance = 0.6,
        .control_factor = 0.8,
    };
    const tension = computeBungeeJumpTension(&model, 30.0, 40.0, 100.0);
    try std.testing.expectApproxEqAbs(@as(f32, 600.0), tension, 0.001);
}

test "779: skydiving terminal velocity" {
    var model = ExtremeSportsModel{
        .sport_type = "skydiving",
        .air_resistance = 0.5,
        .impact_tolerance = 0.9,
        .control_factor = 0.7,
    };
    const velocity = computeSkydivingTerminalVelocity(&model, 80.0, 9.81, 1.2, 0.5);
    try std.testing.expect(velocity > 0);
}

test "780: glider flight distance" {
    var model = ExtremeSportsModel{
        .sport_type = "glider",
        .air_resistance = 0.3,
        .impact_tolerance = 0.8,
        .control_factor = 0.6,
    };
    const distance = computeGliderFlightDistance(&model, 1000.0, 0.8, 0.05, 20.0);
    try std.testing.expect(distance > 0);
}

test "781: hot air balloon lift" {
    var model = ExtremeSportsModel{
        .sport_type = "balloon",
        .air_resistance = 0.2,
        .impact_tolerance = 0.9,
        .control_factor = 0.5,
    };
    const lift = computeHotAirBalloonLift(&model, 2000.0, 373.0, 288.0, 1.2, 9.81);
    try std.testing.expect(lift > 0);
}

test "782: wingsuit glide ratio" {
    var model = ExtremeSportsModel{
        .sport_type = "wingsuit",
        .air_resistance = 0.4,
        .impact_tolerance = 0.7,
        .control_factor = 0.6,
    };
    const ratio = computeWingsuitGlideRatio(&model, 1.5, 5.0, 0.8, 0.1);
    try std.testing.expect(ratio > 0);
}

test "783: motorcycle cornering speed" {
    var model = ExtremeSportsModel{
        .sport_type = "motorcycle",
        .air_resistance = 0.5,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const speed = computeMotorcycleCorneringSpeed(&model, 20.0, std.math.pi / 6.0, 9.81);
    try std.testing.expect(speed > 0);
}

test "784: karting G-force" {
    var model = ExtremeSportsModel{
        .sport_type = "karting",
        .air_resistance = 0.4,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const gforce = computeKartingGForce(&model, 15.0, 10.0, 9.81);
    try std.testing.expect(gforce > 0);
}

test "785: offroad suspension load" {
    var model = ExtremeSportsModel{
        .sport_type = "offroad",
        .air_resistance = 0.5,
        .impact_tolerance = 0.9,
        .control_factor = 0.6,
    };
    const load = computeOffroadSuspensionLoad(&model, 1500.0, 0.3, 50000.0);
    try std.testing.expect(load > 0);
}

test "786: mountaineering altitude effect" {
    var model = ExtremeSportsModel{
        .sport_type = "mountaineering",
        .air_resistance = 0.3,
        .impact_tolerance = 0.7,
        .control_factor = 0.8,
    };
    const oxygen = computeMountaineeringAltitudeEffect(&model, 3000.0, 0.21);
    try std.testing.expect(oxygen > 0);
    try std.testing.expect(oxygen < 0.21);
}

test "787: downhill speed" {
    var model = ExtremeSportsModel{
        .sport_type = "downhill",
        .air_resistance = 0.4,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const speed = computeDownhillSpeed(&model, std.math.pi / 6.0, 5.0, 9.81, 0.1);
    try std.testing.expect(speed > 5.0);
}

test "788: BMX acceleration" {
    var model = ExtremeSportsModel{
        .sport_type = "bmx",
        .air_resistance = 0.4,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const accel = computeBMXAcceleration(&model, 500.0, 2.5, 0.3, 80.0);
    try std.testing.expect(accel > 10.0);
}

test "789: unicycle balance" {
    var model = ExtremeSportsModel{
        .sport_type = "unicycle",
        .air_resistance = 0.3,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const balance = computeUnicycleBalance(&model, 0.1, 5.0, 0.3);
    try std.testing.expect(balance > 0.1);
}

test "790: scooter braking distance" {
    var model = ExtremeSportsModel{
        .sport_type = "scooter",
        .air_resistance = 0.4,
        .impact_tolerance = 0.7,
        .control_factor = 0.8,
    };
    const distance = computeScooterBrakingDistance(&model, 10.0, 0.8, 9.81);
    try std.testing.expect(distance > 0);
}

test "791: drift angle" {
    var model = ExtremeSportsModel{
        .sport_type = "drift",
        .air_resistance = 0.5,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const angle = computeDriftAngle(&model, 15.0, 3.0, 9.81, 0.8);
    try std.testing.expect(angle > 0);
    try std.testing.expect(angle < 0.8);
}

test "792: drift force" {
    var model = ExtremeSportsModel{
        .sport_type = "drift",
        .air_resistance = 0.5,
        .impact_tolerance = 0.8,
        .control_factor = 0.7,
    };
    const force = computeDriftForce(&model, 1200.0, 8.0);
    try std.testing.expect(force > 0);
}
