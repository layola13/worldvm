import ctypes
import os
import sys

if sys.platform == "win32":
    lib_name = "worldvm.dll"
    lib_dir = os.path.join("zig-out", "bin")
else:
    lib_name = "libworldvm.so"
    lib_dir = os.path.join("zig-out", "lib")
lib_path = os.path.join(os.path.dirname(__file__), lib_dir, lib_name)

class TraceEntry(ctypes.Structure):
    _fields_ = [
        ("tick_id", ctypes.c_uint32),
        ("event_type", ctypes.c_char * 32),
        ("instance_id", ctypes.c_uint16),
        ("detail", ctypes.c_char * 64),
    ]

class KCCConfigFFI(ctypes.Structure):
    _fields_ = [
        ("move_speed", ctypes.c_float),
        ("jump_force", ctypes.c_float),
        ("gravity", ctypes.c_float),
        ("crouch_speed_mult", ctypes.c_float),
        ("push_force", ctypes.c_float),
        ("step_height", ctypes.c_float),
        ("stand_height", ctypes.c_float),
        ("crouch_height", ctypes.c_float),
        ("radius", ctypes.c_float),
    ]

class TireConfigFFI(ctypes.Structure):
    _fields_ = [
        ("radius", ctypes.c_float),
        ("width", ctypes.c_float),
        ("mass", ctypes.c_float),
        ("lateral_stiffness", ctypes.c_float),
        ("longitudinal_stiffness", ctypes.c_float),
        ("camber_thrust_coefficient", ctypes.c_float),
        ("peak_slip_ratio", ctypes.c_float),
        ("peak_slip_angle", ctypes.c_float),
        ("friction_coefficient", ctypes.c_float),
        ("rolling_resistance_coefficient", ctypes.c_float),
        ("heat_transfer_coefficient", ctypes.c_float),
        ("optimal_temperature", ctypes.c_float),
        ("max_temperature", ctypes.c_float),
    ]

class SuspensionConfigFFI(ctypes.Structure):
    _fields_ = [
        ("spring_rate", ctypes.c_float),
        ("damping_ratio", ctypes.c_float),
        ("bump_damping", ctypes.c_float),
        ("rebound_damping", ctypes.c_float),
        ("preloaded", ctypes.c_float),
        ("max_length", ctypes.c_float),
        ("min_length", ctypes.c_float),
        ("anti_roll_rate", ctypes.c_float),
    ]

class WorldVM:
    def __init__(self):
        self.lib = ctypes.CDLL(lib_path)
        self.lib.init_kernel.restype = ctypes.c_int
        self.lib.spawn_instance.argtypes = [ctypes.c_uint16, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32]
        self.lib.run_ticks.argtypes = [ctypes.c_uint32]
        self.lib.get_emotion_valence.restype = ctypes.c_int8
        self.lib.get_emotion_arousal.restype = ctypes.c_uint8
        self.lib.get_trace_count.restype = ctypes.c_uint32
        self.lib.get_trace_entry.argtypes = [ctypes.c_uint32]
        self.lib.get_trace_entry.restype = ctypes.POINTER(TraceEntry)
        self.lib.trace_get_visualization_entry_stride.restype = ctypes.c_uint32
        self.lib.trace_export_visualization.argtypes = [ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint16, ctypes.c_int, ctypes.c_uint16, ctypes.POINTER(ctypes.c_float), ctypes.c_uint32]
        self.lib.trace_export_visualization.restype = ctypes.c_uint32
        self.lib.get_last_step_changed.restype = ctypes.c_int
        self.lib.get_last_step_pair_count.restype = ctypes.c_uint32
        self.lib.get_last_step_event_count.restype = ctypes.c_uint32
        self.lib.get_last_step_snapshot_tick.restype = ctypes.c_uint32
        self.lib.get_last_step_state_hash.restype = ctypes.c_uint64
        self.lib.get_last_step_determinism_flags.restype = ctypes.c_uint32
        self.lib.query_get_contract_version.restype = ctypes.c_uint32
        self.lib.query_get_contract_flags.restype = ctypes.c_uint32
        self.lib.reset_context.restype = ctypes.c_int
        self.lib.apply_impulse.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_impulse.restype = ctypes.c_int
        self.lib.apply_force.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_force.restype = ctypes.c_int
        self.lib.apply_torque.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_torque.restype = ctypes.c_int
        self.lib.apply_explosion.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_explosion.restype = None
        self.lib.apply_point_explosion_field.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_point_explosion_field.restype = ctypes.c_uint32
        self.lib.apply_directional_force_field.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_directional_force_field.restype = ctypes.c_uint32
        self.lib.apply_vortex_force_field.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_vortex_force_field.restype = ctypes.c_uint32
        self.lib.apply_buoyancy.argtypes = [ctypes.c_uint8, ctypes.c_float]
        self.lib.apply_buoyancy.restype = ctypes.c_int
        self.lib.get_instance_velocity.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_float)]
        self.lib.get_instance_velocity.restype = ctypes.c_int
        self.lib.get_instance_angular_velocity.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_float)]
        self.lib.get_instance_angular_velocity.restype = ctypes.c_int
        self.lib.get_instance_pos.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_int32)]
        self.lib.get_instance_pos.restype = ctypes.c_int
        self.lib.is_instance_sleeping.argtypes = [ctypes.c_uint8]
        self.lib.is_instance_sleeping.restype = ctypes.c_int
        self.lib.wake_instance.argtypes = [ctypes.c_uint8]
        self.lib.wake_instance.restype = ctypes.c_int
        self.lib.is_instance_broken.argtypes = [ctypes.c_uint8]
        self.lib.is_instance_broken.restype = ctypes.c_int
        self.lib.get_instance_state.argtypes = [ctypes.c_uint8]
        self.lib.get_instance_state.restype = ctypes.c_int
        self.lib.entity_get_medium_type.argtypes = [ctypes.c_uint8]
        self.lib.entity_get_medium_type.restype = ctypes.c_int
        self.lib.entity_is_floating.argtypes = [ctypes.c_uint8]
        self.lib.entity_is_floating.restype = ctypes.c_int
        self.lib.set_time_scale.argtypes = [ctypes.c_float]
        self.lib.get_time_scale.restype = ctypes.c_float
        self.lib.get_joint_count.restype = ctypes.c_uint8
        self.lib.add_joint_fixed.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32]
        self.lib.add_joint_fixed.restype = ctypes.c_int
        self.lib.add_joint_hinge.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32]
        self.lib.add_joint_hinge.restype = ctypes.c_int
        self.lib.add_joint_spring.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_float, ctypes.c_float]
        self.lib.add_joint_spring.restype = ctypes.c_int
        self.lib.add_joint_ball_socket.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32]
        self.lib.add_joint_ball_socket.restype = ctypes.c_int
        self.lib.add_joint_pulley.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_float]
        self.lib.add_joint_pulley.restype = ctypes.c_int
        self.lib.solve_joints.restype = None
        self.lib.clear_joints.restype = None
        self.lib.set_joint_breaking_force.argtypes = [ctypes.c_uint8, ctypes.c_float]
        self.lib.set_joint_breaking_force.restype = ctypes.c_int
        self.lib.set_joint_limits.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float]
        self.lib.set_joint_limits.restype = ctypes.c_int
        self.lib.get_joint_limit_min.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_limit_min.restype = ctypes.c_float
        self.lib.get_joint_limit_max.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_limit_max.restype = ctypes.c_float
        self.lib.set_joint_damping.argtypes = [ctypes.c_uint8, ctypes.c_float]
        self.lib.set_joint_damping.restype = ctypes.c_int
        self.lib.get_joint_damping.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_damping.restype = ctypes.c_float
        self.lib.set_joint_preload.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.set_joint_preload.restype = ctypes.c_int
        self.lib.get_joint_preload_linear_x.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_preload_linear_x.restype = ctypes.c_float
        self.lib.get_joint_preload_linear_y.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_preload_linear_y.restype = ctypes.c_float
        self.lib.get_joint_preload_linear_z.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_preload_linear_z.restype = ctypes.c_float
        self.lib.get_joint_preload_angular.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_preload_angular.restype = ctypes.c_float
        self.lib.get_joint_stress.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_stress.restype = ctypes.c_float
        self.lib.get_joint_geometry_error.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_geometry_error.restype = ctypes.c_float
        self.lib.get_joint_limit_error.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_limit_error.restype = ctypes.c_float
        self.lib.get_joint_drive_error.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_drive_error.restype = ctypes.c_float
        self.lib.get_joint_residual_speed.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_residual_speed.restype = ctypes.c_float
        self.lib.configure_joint_fatigue.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.configure_joint_fatigue.restype = ctypes.c_int
        self.lib.clear_joint_fatigue.argtypes = [ctypes.c_uint8]
        self.lib.clear_joint_fatigue.restype = ctypes.c_int
        self.lib.get_joint_fatigue_damage.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_fatigue_damage.restype = ctypes.c_float
        self.lib.get_joint_fatigue_ratio.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_fatigue_ratio.restype = ctypes.c_float
        self.lib.configure_joint_temperature.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.configure_joint_temperature.restype = ctypes.c_int
        self.lib.clear_joint_temperature.argtypes = [ctypes.c_uint8]
        self.lib.clear_joint_temperature.restype = ctypes.c_int
        self.lib.get_joint_temperature.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_temperature.restype = ctypes.c_float
        self.lib.get_joint_temperature_ratio.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_temperature_ratio.restype = ctypes.c_float
        self.lib.configure_joint_motor.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.configure_joint_motor.restype = ctypes.c_int
        self.lib.set_joint_motor_enabled.argtypes = [ctypes.c_uint8, ctypes.c_int]
        self.lib.set_joint_motor_enabled.restype = ctypes.c_int
        self.lib.is_joint_motor_enabled.argtypes = [ctypes.c_uint8]
        self.lib.is_joint_motor_enabled.restype = ctypes.c_int
        self.lib.get_joint_motor_position.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_motor_position.restype = ctypes.c_float
        self.lib.is_joint_enabled.argtypes = [ctypes.c_uint8]
        self.lib.is_joint_enabled.restype = ctypes.c_int
        self.lib.is_joint_broken.argtypes = [ctypes.c_uint8]
        self.lib.is_joint_broken.restype = ctypes.c_int
        self.lib.get_joint_break_ratio.argtypes = [ctypes.c_uint8]
        self.lib.get_joint_break_ratio.restype = ctypes.c_float
        self.lib.raycast_single.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.raycast_single.restype = ctypes.c_int
        self.lib.sphere_cast.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.sphere_cast.restype = ctypes.c_int
        self.lib.box_cast.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.box_cast.restype = ctypes.c_int
        self.lib.compute_toi.argtypes = [ctypes.c_float]*18 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_toi.restype = ctypes.c_int
        self.lib.compute_time_of_entry.argtypes = [ctypes.c_float]*18 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_time_of_entry.restype = ctypes.c_int
        self.lib.compute_toi_iterative.argtypes = [ctypes.c_float]*18 + [ctypes.c_uint32, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_toi_iterative.restype = ctypes.c_int
        self.lib.compute_ccd_iteration_limit.argtypes = [ctypes.c_uint32, ctypes.c_float, ctypes.c_float, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_iteration_limit.restype = ctypes.c_int
        self.lib.compute_ccd_progress_watchdog.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_progress_watchdog.restype = ctypes.c_int
        self.lib.compute_box_ccd.argtypes = [ctypes.c_float]*18 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_box_ccd.restype = ctypes.c_int
        self.lib.compute_ccd_trigger_aabb.argtypes = [ctypes.c_float]*18 + [ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_trigger_aabb.restype = ctypes.c_int
        self.lib.compute_ccd_thin_wall_penetration.argtypes = [ctypes.c_float]*19 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_thin_wall_penetration.restype = ctypes.c_int
        self.lib.compute_ccd_tunnel_suppression.argtypes = [ctypes.c_float]*20 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_tunnel_suppression.restype = ctypes.c_int
        self.lib.compute_rotating_box_ccd.argtypes = [ctypes.c_float]*22 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_rotating_box_ccd.restype = ctypes.c_int
        self.lib.compute_angular_velocity_ccd.argtypes = [ctypes.c_float]*23 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_angular_velocity_ccd.restype = ctypes.c_int
        self.lib.compute_conservative_step.argtypes = [ctypes.c_float]*8 + [ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_conservative_step.restype = ctypes.c_int
        self.lib.compute_ccd_performance_plan.argtypes = [ctypes.c_float]*6 + [ctypes.c_uint32]*4 + [ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_performance_plan.restype = ctypes.c_int
        self.lib.compute_ccd_precision_plan.argtypes = [ctypes.c_float]*8 + [ctypes.c_uint32]*2 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_precision_plan.restype = ctypes.c_int
        self.lib.compute_ccd_stability_validation.argtypes = [ctypes.c_float]*13 + [ctypes.c_uint32]*3 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_stability_validation.restype = ctypes.c_int
        self.lib.compute_ccd_island_parallel_plan.argtypes = [ctypes.c_uint32]*9 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_island_parallel_plan.restype = ctypes.c_int
        self.lib.compute_ccd_thread_determinism_validation.argtypes = [ctypes.c_uint32]*9 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_thread_determinism_validation.restype = ctypes.c_int
        self.lib.compute_ccd_sleep_interaction.argtypes = [ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32] + [ctypes.c_float]*7 + [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_sleep_interaction.restype = ctypes.c_int
        self.lib.compute_ccd_substep_plan.argtypes = [ctypes.c_float]*8 + [ctypes.c_uint32, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_ccd_substep_plan.restype = ctypes.c_int
        self.lib.compute_polygon_ccd.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_uint32, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float), ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_polygon_ccd.restype = ctypes.c_int
        self.lib.compute_sphere_ccd.argtypes = [ctypes.c_float]*14 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_sphere_ccd.restype = ctypes.c_int
        self.lib.compute_capsule_ccd.argtypes = [ctypes.c_float]*16 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_capsule_ccd.restype = ctypes.c_int

        # KCC
        self.lib.kcc_init.restype = None
        self.lib.kcc_get_height.argtypes = [ctypes.c_uint8]
        self.lib.kcc_get_height.restype = ctypes.c_float
        self.lib.kcc_slide_along_wall.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.kcc_slide_along_wall.restype = None

        # KCC Character creation
        self.lib.kcc_create_character.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(KCCConfigFFI)]
        self.lib.kcc_create_character.restype = ctypes.c_int

        # Ballistics
        self.lib.ballistics_init.restype = None
        self.lib.ballistics_spawn_projectile.argtypes = [ctypes.c_float]*6 + [ctypes.c_float, ctypes.c_float]
        self.lib.ballistics_spawn_projectile.restype = ctypes.c_int
        self.lib.ballistics_get_speed.argtypes = [ctypes.c_uint8]
        self.lib.ballistics_get_speed.restype = ctypes.c_float
        self.lib.ballistics_get_kinetic_energy.argtypes = [ctypes.c_uint8]
        self.lib.ballistics_get_kinetic_energy.restype = ctypes.c_float
        self.lib.ballistics_calculate_deflection.argtypes = [ctypes.c_float]*6 + [ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.ballistics_calculate_deflection.restype = None

        # Destruction
        self.lib.destruction_init.restype = None
        self.lib.destruction_calculate_damage.argtypes = [ctypes.c_float, ctypes.c_uint8, ctypes.c_float]
        self.lib.destruction_calculate_damage.restype = ctypes.c_float
        self.lib.destruction_create_destroyable.argtypes = [ctypes.c_uint16, ctypes.c_float]
        self.lib.destruction_create_destroyable.restype = ctypes.c_int
        self.lib.destruction_should_shatter.argtypes = [ctypes.c_uint8]
        self.lib.destruction_should_shatter.restype = ctypes.c_int
        self.lib.destruction_generate_fracture.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.destruction_generate_fracture.restype = None

        # Ragdoll
        self.lib.ragdoll_init.restype = None
        self.lib.ragdoll_is_fully_broken.argtypes = [ctypes.c_uint8]
        self.lib.ragdoll_is_fully_broken.restype = ctypes.c_int
        self.lib.ragdoll_is_resurrection_ready.argtypes = [ctypes.c_uint8]
        self.lib.ragdoll_is_resurrection_ready.restype = ctypes.c_int
        self.lib.ragdoll_create_humanoid.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.ragdoll_create_humanoid.restype = ctypes.c_int
        self.lib.ragdoll_break_limb.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
        self.lib.ragdoll_break_limb.restype = None

        # Vehicle
        self.lib.vehicle_init.restype = None
        self.lib.vehicle_get_forward_dir.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_float)]
        self.lib.vehicle_get_forward_dir.restype = None
        self.lib.vehicle_check_flipped.argtypes = [ctypes.c_uint8]
        self.lib.vehicle_check_flipped.restype = ctypes.c_int
        self.lib.vehicle_create_car.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.vehicle_create_car.restype = ctypes.c_int
        self.lib.vehicle_create_aircraft.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.vehicle_create_aircraft.restype = ctypes.c_int
        self.lib.vehicle_create_boat.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.vehicle_create_boat.restype = ctypes.c_int
        self.lib.vehicle_create_hovercraft.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.vehicle_create_hovercraft.restype = ctypes.c_int

        # Network
        self.lib.network_init.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]
        self.lib.network_init.restype = None
        self.lib.network_get_tick.restype = ctypes.c_uint32
        self.lib.network_calculate_crc.argtypes = [ctypes.c_float]*6 + [ctypes.c_float]
        self.lib.network_calculate_crc.restype = ctypes.c_uint32
        self.lib.network_create_replica.argtypes = [ctypes.c_uint16]
        self.lib.network_create_replica.restype = ctypes.c_int

        # Crash Defense
        self.lib.crash_defense_init.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32]
        self.lib.crash_defense_init.restype = None
        self.lib.crash_defense_is_nan.argtypes = [ctypes.c_float]
        self.lib.crash_defense_is_nan.restype = ctypes.c_int
        self.lib.crash_defense_is_infinite.argtypes = [ctypes.c_float]
        self.lib.crash_defense_is_infinite.restype = ctypes.c_int
        self.lib.crash_defense_is_valid_float.argtypes = [ctypes.c_float]
        self.lib.crash_defense_is_valid_float.restype = ctypes.c_int
        self.lib.crash_defense_compute_nan_handling.argtypes = [ctypes.c_float]*4 + [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_nan_handling.restype = ctypes.c_int
        self.lib.crash_defense_compute_infinity_handling.argtypes = [ctypes.c_float]*4 + [ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_infinity_handling.restype = ctypes.c_int
        self.lib.crash_defense_compute_bounds_correction.argtypes = [ctypes.c_float]*6 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_bounds_correction.restype = ctypes.c_int
        self.lib.crash_defense_compute_energy_limit.argtypes = [ctypes.c_float]*5 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_energy_limit.restype = ctypes.c_int
        self.lib.crash_defense_compute_velocity_limit.argtypes = [ctypes.c_float]*5 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_velocity_limit.restype = ctypes.c_int
        self.lib.crash_defense_compute_position_range_limit.argtypes = [ctypes.c_float]*8 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_position_range_limit.restype = ctypes.c_int
        self.lib.crash_defense_compute_torque_limit.argtypes = [ctypes.c_float]*5 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_torque_limit.restype = ctypes.c_int
        self.lib.crash_defense_compute_solver_divergence.argtypes = [ctypes.c_float]*5 + [ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_solver_divergence.restype = ctypes.c_int
        self.lib.crash_defense_compute_iteration_timeout.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_float, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_iteration_timeout.restype = ctypes.c_int
        self.lib.crash_defense_compute_no_progress.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_float, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_no_progress.restype = ctypes.c_int
        self.lib.crash_defense_compute_emergency_stop.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_emergency_stop.restype = ctypes.c_int
        self.lib.crash_defense_compute_rollback.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_compute_rollback.restype = ctypes.c_int
        self.lib.crash_defense_is_emergency_stopped.restype = ctypes.c_int
        self.lib.crash_defense_get_emergency_reason.restype = ctypes.c_uint32
        self.lib.crash_defense_get_emergency_stop_count.restype = ctypes.c_uint32
        self.lib.crash_defense_record_error_log.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint8, ctypes.c_float, ctypes.c_float]
        self.lib.crash_defense_record_error_log.restype = None
        self.lib.crash_defense_clear_error_logs.restype = None
        self.lib.crash_defense_get_error_log_count.restype = ctypes.c_uint32
        self.lib.crash_defense_get_error_log_at.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_get_error_log_at.restype = ctypes.c_int
        self.lib.crash_defense_collect_diagnostics.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_collect_diagnostics.restype = ctypes.c_int
        self.lib.crash_defense_clear_diagnostics.restype = None
        self.lib.crash_defense_get_diagnostic_count.restype = ctypes.c_uint32
        self.lib.crash_defense_get_diagnostic_at.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_get_diagnostic_at.restype = ctypes.c_int
        self.lib.crash_defense_get_stats_report.argtypes = [ctypes.POINTER(ctypes.c_float)]
        self.lib.crash_defense_get_stats_report.restype = ctypes.c_int
        self.lib.crash_defense_is_stuck.argtypes = [ctypes.c_uint32]
        self.lib.crash_defense_is_stuck.restype = ctypes.c_int
        self.lib.crash_defense_emergency_stop.restype = None
        self.lib.crash_defense_reset_emergency_stop.restype = None
        self.lib.crash_defense_update_progress.argtypes = [ctypes.c_uint32]
        self.lib.crash_defense_update_progress.restype = None

        # Tire
        self.lib.tire_init.restype = None
        self.lib.tire_calculate_slip_ratio.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float]
        self.lib.tire_calculate_slip_ratio.restype = ctypes.c_float
        self.lib.tire_calculate_friction_circle.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.tire_calculate_friction_circle.restype = ctypes.c_float
        self.lib.tire_check_hydroplaning.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float]
        self.lib.tire_check_hydroplaning.restype = ctypes.c_int
        self.lib.tire_create.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(TireConfigFFI)]
        self.lib.tire_create.restype = ctypes.c_int

        # Suspension
        self.lib.suspension_init.restype = None
        self.lib.suspension_calculate_spring_force.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.Structure]
        self.lib.suspension_calculate_spring_force.restype = ctypes.c_float
        self.lib.suspension_calculate_natural_frequency.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.suspension_calculate_natural_frequency.restype = ctypes.c_float
        self.lib.suspension_create.argtypes = [ctypes.POINTER(SuspensionConfigFFI)]
        self.lib.suspension_create.restype = ctypes.c_int

        # Drivetrain
        self.lib.drivetrain_init.restype = None
        self.lib.drivetrain_calculate_torque_curve.argtypes = [ctypes.c_float]
        self.lib.drivetrain_calculate_torque_curve.restype = ctypes.c_float
        self.lib.drivetrain_calculate_horsepower.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.drivetrain_calculate_horsepower.restype = ctypes.c_float
        self.lib.drivetrain_get_gear_ratio.argtypes = [ctypes.c_int8]
        self.lib.drivetrain_get_gear_ratio.restype = ctypes.c_float
        self.lib.drivetrain_calculate_wheel_torque.argtypes = [ctypes.c_float, ctypes.c_int8, ctypes.c_float, ctypes.c_float]
        self.lib.drivetrain_calculate_wheel_torque.restype = ctypes.c_float
        self.lib.drivetrain_get_engine_rpm.restype = ctypes.c_float
        self.lib.drivetrain_get_engine_torque.restype = ctypes.c_float
        self.lib.drivetrain_apply_throttle.argtypes = [ctypes.c_float]
        self.lib.drivetrain_apply_throttle.restype = None

        # Aerodynamics
        self.lib.aerodynamics_init.restype = None
        self.lib.aerodynamics_calculate_drag_force.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.aerodynamics_calculate_drag_force.restype = ctypes.c_float
        self.lib.aerodynamics_calculate_downforce.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.aerodynamics_calculate_downforce.restype = ctypes.c_float
        self.lib.aerodynamics_get_drag_coefficient.restype = ctypes.c_float

        # Braking
        self.lib.braking_init.restype = None
        self.lib.braking_apply_brake.argtypes = [ctypes.c_float]
        self.lib.braking_apply_brake.restype = None
        self.lib.braking_apply_handbrake.argtypes = [ctypes.c_int]
        self.lib.braking_apply_handbrake.restype = None
        self.lib.braking_get_pedal_position.restype = ctypes.c_float
        self.lib.braking_is_abs_active.argtypes = [ctypes.c_uint8]
        self.lib.braking_is_abs_active.restype = ctypes.c_int
        self.lib.braking_is_handbrake_active.restype = ctypes.c_int

        # Terrain
        self.lib.terrain_init.restype = None
        self.lib.terrain_add_patch.argtypes = [ctypes.c_int32, ctypes.c_int32, ctypes.c_int32, ctypes.c_uint8]
        self.lib.terrain_add_patch.restype = None
        self.lib.terrain_get_surface_at.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.terrain_get_surface_at.restype = ctypes.c_uint8
        self.lib.terrain_get_friction_at.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.terrain_get_friction_at.restype = ctypes.c_float
        self.lib.terrain_get_rolling_resistance_at.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.terrain_get_rolling_resistance_at.restype = ctypes.c_float
        self.lib.terrain_calculate_hydroplaning_risk.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.terrain_calculate_hydroplaning_risk.restype = ctypes.c_float
        self.lib.terrain_get_weather_visibility.restype = ctypes.c_float

        # Material Pairing
        self.lib.material_pairing_get_restitution.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
        self.lib.material_pairing_get_restitution.restype = ctypes.c_float
        self.lib.material_pairing_get_friction.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
        self.lib.material_pairing_get_friction.restype = ctypes.c_float
        self.lib.material_pairing_calculate_impact_damage.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_uint8, ctypes.c_uint8]
        self.lib.material_pairing_calculate_impact_damage.restype = ctypes.c_float
        self.lib.material_pairing_get_buoyancy.argtypes = [ctypes.c_uint8]
        self.lib.material_pairing_get_buoyancy.restype = ctypes.c_float
        self.lib.material_pairing_get_medium_type.argtypes = [ctypes.c_uint8]
        self.lib.material_pairing_get_medium_type.restype = ctypes.c_uint8
        self.lib.material_pairing_is_hard_surface.argtypes = [ctypes.c_uint8]
        self.lib.material_pairing_is_hard_surface.restype = ctypes.c_int

        # Collision
        self.lib.collision_init.restype = None
        self.lib.collision_calculate_impact_energy.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.collision_calculate_impact_energy.restype = ctypes.c_float
        self.lib.collision_check_structural_failure.restype = ctypes.c_int
        self.lib.collision_get_structural_integrity.restype = ctypes.c_float

        # Disasters
        self.lib.disasters_init.restype = None
        self.lib.disasters_trigger.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.disasters_trigger.restype = None
        self.lib.disasters_calculate_seismic_intensity.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.disasters_calculate_seismic_intensity.restype = ctypes.c_float
        self.lib.disasters_check_chain_reaction.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.disasters_check_chain_reaction.restype = ctypes.c_int
        self.lib.disasters_get_wind_velocity.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.disasters_get_wind_velocity.restype = None
        self.lib.disasters_enable_chain_reactions.argtypes = [ctypes.c_int]
        self.lib.disasters_enable_chain_reactions.restype = None

        # Sensors
        self.lib.sensors_init.restype = None
        self.lib.sensors_add.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float]
        self.lib.sensors_add.restype = ctypes.c_int
        self.lib.sensors_get_detected_object_count.restype = ctypes.c_uint8
        self.lib.sensors_raycast_occlusion.argtypes = [ctypes.c_float]*9
        self.lib.sensors_raycast_occlusion.restype = ctypes.c_float
        self.lib.sensors_calculate_confidence.argtypes = [ctypes.c_float, ctypes.c_uint8, ctypes.c_float]
        self.lib.sensors_calculate_confidence.restype = ctypes.c_float

        # Rewind
        self.lib.rewind_init.restype = None
        self.lib.rewind_is_deterministic.restype = ctypes.c_int
        self.lib.rewind_get_buffer_count.restype = ctypes.c_uint32
        self.lib.rewind_calculate_state_hash.argtypes = [ctypes.c_uint32, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.rewind_calculate_state_hash.restype = ctypes.c_uint64
        self.lib.rewind_record_state.argtypes = [ctypes.c_uint32, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.rewind_record_state.restype = None
        self.lib.rewind_start_world_snapshot_playback.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int, ctypes.c_int]
        self.lib.rewind_start_world_snapshot_playback.restype = ctypes.c_int
        self.lib.rewind_stop_world_snapshot_playback.restype = None
        self.lib.rewind_get_world_snapshot_playback_state.argtypes = [ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        self.lib.rewind_get_world_snapshot_playback_state.restype = ctypes.c_int
        self.lib.rewind_next_world_snapshot_playback_tick.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
        self.lib.rewind_next_world_snapshot_playback_tick.restype = ctypes.c_int
        self.lib.rewind_get_active_world_snapshot_branch_id.restype = ctypes.c_uint32
        self.lib.rewind_create_world_snapshot_branch.argtypes = [ctypes.c_uint32]
        self.lib.rewind_create_world_snapshot_branch.restype = ctypes.c_int
        self.lib.rewind_switch_world_snapshot_branch.argtypes = [ctypes.c_uint32]
        self.lib.rewind_switch_world_snapshot_branch.restype = ctypes.c_int
        self.lib.rewind_delete_world_snapshot_branch.argtypes = [ctypes.c_uint32]
        self.lib.rewind_delete_world_snapshot_branch.restype = ctypes.c_int
        self.lib.rewind_get_world_snapshot_branch_info.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        self.lib.rewind_get_world_snapshot_branch_info.restype = ctypes.c_int
        self.lib.rewind_list_world_snapshot_branches.argtypes = [ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        self.lib.rewind_list_world_snapshot_branches.restype = ctypes.c_uint32
        self.lib.rewind_merge_world_snapshot_branches.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        self.lib.rewind_merge_world_snapshot_branches.restype = ctypes.c_int
        self.lib.rewind_set_world_snapshot_budget.argtypes = [ctypes.c_uint32]
        self.lib.rewind_set_world_snapshot_budget.restype = ctypes.c_int
        self.lib.rewind_get_world_snapshot_budget_info.argtypes = [ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        self.lib.rewind_get_world_snapshot_budget_info.restype = ctypes.c_int
        self.lib.rewind_collect_world_snapshot_garbage.argtypes = [ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32]
        self.lib.rewind_collect_world_snapshot_garbage.restype = ctypes.c_int
        self.lib.rewind_save_world_snapshots.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        self.lib.rewind_save_world_snapshots.restype = ctypes.c_int
        self.lib.rewind_load_world_snapshots.argtypes = [ctypes.c_char_p, ctypes.c_uint32]
        self.lib.rewind_load_world_snapshots.restype = ctypes.c_int
        self.lib.rewind_export_world_snapshot_network_packet.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
        self.lib.rewind_export_world_snapshot_network_packet.restype = ctypes.c_int
        self.lib.rewind_import_world_snapshot_network_packet.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
        self.lib.rewind_import_world_snapshot_network_packet.restype = ctypes.c_int
        self.lib.rewind_export_world_snapshot_network_packet_encrypted.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint64, ctypes.c_uint64, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
        self.lib.rewind_export_world_snapshot_network_packet_encrypted.restype = ctypes.c_int
        self.lib.rewind_import_world_snapshot_network_packet_encrypted.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.c_uint64, ctypes.c_uint64]
        self.lib.rewind_import_world_snapshot_network_packet_encrypted.restype = ctypes.c_int

        # AI Traffic
        self.lib.ai_traffic_init.restype = None
        self.lib.ai_traffic_spawn_vehicle.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8]
        self.lib.ai_traffic_spawn_vehicle.restype = ctypes.c_int
        self.lib.ai_traffic_add_traffic_light.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.ai_traffic_add_traffic_light.restype = ctypes.c_int
        self.lib.ai_traffic_get_vehicle_count.restype = ctypes.c_uint8
        self.lib.ai_traffic_trigger_emergency.argtypes = [ctypes.c_uint8]
        self.lib.ai_traffic_trigger_emergency.restype = None

        # Time scale
        self.lib.set_time_scale.argtypes = [ctypes.c_float]
        self.lib.set_time_scale.restype = None
        self.lib.get_time_scale.restype = ctypes.c_float

        if self.lib.init_kernel() < 0: raise RuntimeError("Init failed")

    def set_time_scale(self, scale: float):
        self.lib.set_time_scale(scale)

    def get_time_scale(self) -> float:
        return self.lib.get_time_scale()

    def spawn(self, eid, x, y, z): self.lib.spawn_instance(eid, x, y, z)
    def run(self, t=1): return self.lib.run_ticks(t)
    def last_step(self):
        return {
            "changed": bool(self.lib.get_last_step_changed()),
            "pair_count": self.lib.get_last_step_pair_count(),
            "event_count": self.lib.get_last_step_event_count(),
            "snapshot_tick": self.lib.get_last_step_snapshot_tick(),
            "state_hash": self.lib.get_last_step_state_hash(),
            "determinism_flags": self.lib.get_last_step_determinism_flags(),
        }
    def query_contract_version(self): return self.lib.query_get_contract_version()
    def query_contract_flags(self): return self.lib.query_get_contract_flags()
    def query_contract(self):
        return {
            "version": self.query_contract_version(),
            "flags": self.query_contract_flags(),
        }
    def emotions(self): return {"v": self.lib.get_emotion_valence(), "a": self.lib.get_emotion_arousal()}
    def events(self):
        return [{"t": self.lib.get_trace_entry(i).contents.tick_id,
                 "type": self.lib.get_trace_entry(i).contents.event_type.decode().rstrip('\0'),
                 "id": self.lib.get_trace_entry(i).contents.instance_id}
                for i in range(self.lib.get_trace_count())]
    def trace_visualization(self, include_pending=True, min_tick=0, max_tick=0xFFFFFFFF, type_mask=0, subject_id=None, max_entries=256):
        max_entries = max(0, int(max_entries))
        if max_entries == 0:
            return []
        stride = int(self.lib.trace_get_visualization_entry_stride())
        out = (ctypes.c_float * (stride * max_entries))()
        subject_filter_enabled = 1 if subject_id is not None else 0
        subject_value = 0 if subject_id is None else int(subject_id)
        count = int(self.lib.trace_export_visualization(
            int(bool(include_pending)),
            int(min_tick),
            int(max_tick),
            int(type_mask),
            subject_filter_enabled,
            subject_value,
            out,
            max_entries,
        ))
        type_names = {
            1: "collision",
            2: "sound",
            3: "particle",
            4: "deformation",
            5: "breakage",
            6: "joint_breakage",
        }
        return [{
            "tick": int(out[i * stride + 0]),
            "event_type": type_names.get(int(out[i * stride + 1]), "unknown"),
            "event_type_id": int(out[i * stride + 1]),
            "subject_id": int(out[i * stride + 2]),
            "intensity": float(out[i * stride + 3]),
            "value_a": float(out[i * stride + 4]),
            "value_b": float(out[i * stride + 5]),
            "value_c": float(out[i * stride + 6]),
            "lane": int(out[i * stride + 7]),
        } for i in range(count)]
    def apply_impulse(self, inst_idx, ix, iy, iz):
        return self.lib.apply_impulse(inst_idx, ix, iy, iz)
    def get_velocity(self, inst_idx):
        vel = (ctypes.c_float * 3)()
        self.lib.get_instance_velocity(inst_idx, vel)
        return list(vel)
    def get_angular_velocity(self, inst_idx):
        ang = (ctypes.c_float * 3)()
        self.lib.get_instance_angular_velocity(inst_idx, ang)
        return list(ang)
    def get_pos(self, inst_idx):
        pos = (ctypes.c_int32 * 3)()
        self.lib.get_instance_pos(inst_idx, pos)
        return list(pos)
    def rewind_start_playback(self, start_tick: int, end_tick: int, loop: bool = False, reverse: bool = False) -> bool:
        return bool(self.lib.rewind_start_world_snapshot_playback(int(start_tick), int(end_tick), int(bool(loop)), int(bool(reverse))))
    def rewind_stop_playback(self):
        self.lib.rewind_stop_world_snapshot_playback()
    def rewind_get_playback_state(self):
        out = (ctypes.c_uint32 * 7)()
        if self.lib.rewind_get_world_snapshot_playback_state(out, 7) != 0:
            return None
        return {
            "active": bool(out[0]),
            "loop": bool(out[1]),
            "reverse": bool(out[2]),
            "start_tick": int(out[3]),
            "end_tick": int(out[4]),
            "last_tick": int(out[5]),
            "has_emitted": bool(out[6]),
        }
    def rewind_next_playback_tick(self):
        tick = ctypes.c_uint32(0)
        if self.lib.rewind_next_world_snapshot_playback_tick(ctypes.byref(tick)) == 0:
            return None
        return int(tick.value)
    def rewind_get_active_branch_id(self) -> int:
        return int(self.lib.rewind_get_active_world_snapshot_branch_id())
    def rewind_create_branch(self, fork_tick: int):
        branch_id = int(self.lib.rewind_create_world_snapshot_branch(int(fork_tick)))
        return None if branch_id < 0 else branch_id
    def rewind_switch_branch(self, branch_id: int) -> bool:
        return bool(self.lib.rewind_switch_world_snapshot_branch(int(branch_id)))
    def rewind_delete_branch(self, branch_id: int) -> bool:
        return bool(self.lib.rewind_delete_world_snapshot_branch(int(branch_id)))
    def rewind_get_branch_info(self, branch_id: int):
        out = (ctypes.c_uint32 * 6)()
        if self.lib.rewind_get_world_snapshot_branch_info(int(branch_id), out, 6) != 0:
            return None
        return {
            "id": int(out[0]),
            "parent_id": int(out[1]),
            "fork_tick": int(out[2]),
            "head_tick": int(out[3]),
            "snapshot_count": int(out[4]),
            "active": bool(out[5]),
        }
    def rewind_list_branches(self, max_branches: int = 8):
        max_branches = max(1, int(max_branches))
        out = (ctypes.c_uint32 * (max_branches * 6))()
        count = int(self.lib.rewind_list_world_snapshot_branches(out, max_branches * 6))
        branches = []
        for i in range(count):
            base = i * 6
            branches.append({
                "id": int(out[base + 0]),
                "parent_id": int(out[base + 1]),
                "fork_tick": int(out[base + 2]),
                "head_tick": int(out[base + 3]),
                "snapshot_count": int(out[base + 4]),
                "active": bool(out[base + 5]),
            })
        return branches
    def rewind_merge_branches(self, target_branch_id: int, source_branch_id: int, strategy="keep_target"):
        if isinstance(strategy, str):
            strategy_map = {
                "keep_target": 0,
                "keep_source": 1,
                "keep_latest": 2,
            }
            if strategy not in strategy_map:
                raise ValueError(f"Unknown merge strategy: {strategy}")
            strategy_id = strategy_map[strategy]
        else:
            strategy_id = int(strategy)
        out = (ctypes.c_uint32 * 7)()
        if self.lib.rewind_merge_world_snapshot_branches(int(target_branch_id), int(source_branch_id), strategy_id, out, 7) != 0:
            return None
        return {
            "target_branch_id": int(out[0]),
            "source_branch_id": int(out[1]),
            "strategy": int(out[2]),
            "moved_count": int(out[3]),
            "conflict_count": int(out[4]),
            "resolved_by_source": int(out[5]),
            "resolved_by_target": int(out[6]),
        }
    def rewind_set_snapshot_budget(self, max_snapshots: int) -> bool:
        return bool(self.lib.rewind_set_world_snapshot_budget(int(max_snapshots)))
    def rewind_get_snapshot_budget_info(self):
        out = (ctypes.c_uint32 * 4)()
        if self.lib.rewind_get_world_snapshot_budget_info(out, 4) != 0:
            return None
        return {
            "budget": int(out[0]),
            "count": int(out[1]),
            "capacity": int(out[2]),
            "evicted_count": int(out[3]),
        }
    def rewind_collect_snapshot_garbage(self):
        out = (ctypes.c_uint32 * 4)()
        if self.lib.rewind_collect_world_snapshot_garbage(out, 4) != 0:
            return None
        return {
            "scanned_count": int(out[0]),
            "removed_count": int(out[1]),
            "removed_orphan_count": int(out[2]),
            "removed_duplicate_count": int(out[3]),
        }
    def rewind_save_snapshots(self, path: str) -> bool:
        path_bytes = os.fsencode(path)
        return bool(self.lib.rewind_save_world_snapshots(path_bytes, len(path_bytes)))
    def rewind_load_snapshots(self, path: str) -> bool:
        path_bytes = os.fsencode(path)
        return bool(self.lib.rewind_load_world_snapshots(path_bytes, len(path_bytes)))
    def rewind_export_snapshot_packet(self, tick: int, branch_id: int = None, max_bytes: int = 65536):
        if branch_id is None:
            branch_id = self.rewind_get_active_branch_id()
        max_bytes = max(1, int(max_bytes))
        out = (ctypes.c_uint8 * max_bytes)()
        written = int(self.lib.rewind_export_world_snapshot_network_packet(int(tick), int(branch_id), out, max_bytes))
        if written < 0:
            return None
        return bytes(out[:written])
    def rewind_import_snapshot_packet(self, packet: bytes) -> bool:
        packet_bytes = bytes(packet)
        if len(packet_bytes) == 0:
            return False
        packet_buf = (ctypes.c_uint8 * len(packet_bytes)).from_buffer_copy(packet_bytes)
        return self.lib.rewind_import_world_snapshot_network_packet(packet_buf, len(packet_bytes)) == 0
    def rewind_export_snapshot_packet_encrypted(self, tick: int, encryption_key: int, nonce: int, branch_id: int = None, max_bytes: int = 65536):
        if branch_id is None:
            branch_id = self.rewind_get_active_branch_id()
        max_bytes = max(1, int(max_bytes))
        out = (ctypes.c_uint8 * max_bytes)()
        written = int(self.lib.rewind_export_world_snapshot_network_packet_encrypted(
            int(tick),
            int(branch_id),
            int(encryption_key) & 0xFFFFFFFFFFFFFFFF,
            int(nonce) & 0xFFFFFFFFFFFFFFFF,
            out,
            max_bytes,
        ))
        if written < 0:
            return None
        return bytes(out[:written])
    def rewind_import_snapshot_packet_encrypted(self, packet: bytes, encryption_key: int, nonce: int) -> bool:
        packet_bytes = bytes(packet)
        if len(packet_bytes) == 0:
            return False
        packet_buf = (ctypes.c_uint8 * len(packet_bytes)).from_buffer_copy(packet_bytes)
        return self.lib.rewind_import_world_snapshot_network_packet_encrypted(
            packet_buf,
            len(packet_bytes),
            int(encryption_key) & 0xFFFFFFFFFFFFFFFF,
            int(nonce) & 0xFFFFFFFFFFFFFFFF,
        ) == 0
    def reset(self): self.lib.reset_context()
    def close(self): self.lib.shutdown_kernel()

if __name__ == "__main__":
    vm = WorldVM()
    vm.spawn(5, 0, 0, 0)     # Floor
    vm.spawn(3, 10, 5, 15)   # Glass
    vm.spawn(2, 10, 7, 15)   # Hammer (Drop from y=7 to hit glass at y=5 quickly)
    
    print("Initial:", vm.emotions())
    for i in range(20):
        vm.run(1)
        evs = vm.events()
        if evs:
            print(f"Tick {i} Events:", evs)
            break
    print("Final:", vm.emotions())
    vm.close()
