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
        self.lib.reset_context.restype = ctypes.c_int
        self.lib.apply_impulse.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.apply_impulse.restype = ctypes.c_int
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
        self.lib.solve_joints.restype = None
        self.lib.clear_joints.restype = None
        self.lib.raycast_single.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.raycast_single.restype = ctypes.c_int
        self.lib.sphere_cast.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.sphere_cast.restype = ctypes.c_int
        self.lib.box_cast.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.box_cast.restype = ctypes.c_int
        self.lib.compute_toi.argtypes = [ctypes.c_float]*18 + [ctypes.POINTER(ctypes.c_float)]
        self.lib.compute_toi.restype = ctypes.c_int

        # KCC
        self.lib.kcc_init.restype = None
        self.lib.kcc_get_height.argtypes = [ctypes.c_uint8]
        self.lib.kcc_get_height.restype = ctypes.c_float
        self.lib.kcc_slide_along_wall.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_float)]
        self.lib.kcc_slide_along_wall.restype = None

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

        # Ragdoll
        self.lib.ragdoll_init.restype = None
        self.lib.ragdoll_is_fully_broken.argtypes = [ctypes.c_uint8]
        self.lib.ragdoll_is_fully_broken.restype = ctypes.c_int
        self.lib.ragdoll_is_resurrection_ready.argtypes = [ctypes.c_uint8]
        self.lib.ragdoll_is_resurrection_ready.restype = ctypes.c_int

        # Vehicle
        self.lib.vehicle_init.restype = None
        self.lib.vehicle_get_forward_dir.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_float)]
        self.lib.vehicle_get_forward_dir.restype = None
        self.lib.vehicle_check_flipped.argtypes = [ctypes.c_uint8]
        self.lib.vehicle_check_flipped.restype = ctypes.c_int

        # Network
        self.lib.network_init.argtypes = [ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32]
        self.lib.network_init.restype = None
        self.lib.network_get_tick.restype = ctypes.c_uint32
        self.lib.network_calculate_crc.argtypes = [ctypes.c_float]*6 + [ctypes.c_float]
        self.lib.network_calculate_crc.restype = ctypes.c_uint32

        # Crash Defense
        self.lib.crash_defense_init.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint32]
        self.lib.crash_defense_init.restype = None
        self.lib.crash_defense_is_nan.argtypes = [ctypes.c_float]
        self.lib.crash_defense_is_nan.restype = ctypes.c_int
        self.lib.crash_defense_is_infinite.argtypes = [ctypes.c_float]
        self.lib.crash_defense_is_infinite.restype = ctypes.c_int
        self.lib.crash_defense_is_valid_float.argtypes = [ctypes.c_float]
        self.lib.crash_defense_is_valid_float.restype = ctypes.c_int
        self.lib.crash_defense_is_emergency_stopped.restype = ctypes.c_int
        self.lib.crash_defense_is_stuck.argtypes = [ctypes.c_uint32]
        self.lib.crash_defense_is_stuck.restype = ctypes.c_int

        # Tire
        self.lib.tire_init.restype = None
        self.lib.tire_calculate_slip_ratio.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float]
        self.lib.tire_calculate_slip_ratio.restype = ctypes.c_float
        self.lib.tire_calculate_friction_circle.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.tire_calculate_friction_circle.restype = ctypes.c_float
        self.lib.tire_check_hydroplaning.argtypes = [ctypes.c_uint8, ctypes.c_float, ctypes.c_float]
        self.lib.tire_check_hydroplaning.restype = ctypes.c_int

        # Suspension
        self.lib.suspension_init.restype = None
        self.lib.suspension_calculate_spring_force.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.Structure]
        self.lib.suspension_calculate_spring_force.restype = ctypes.c_float
        self.lib.suspension_calculate_natural_frequency.argtypes = [ctypes.c_float, ctypes.c_float]
        self.lib.suspension_calculate_natural_frequency.restype = ctypes.c_float

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

        # AI Traffic
        self.lib.ai_traffic_init.restype = None
        self.lib.ai_traffic_spawn_vehicle.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_uint8]
        self.lib.ai_traffic_spawn_vehicle.restype = ctypes.c_int
        self.lib.ai_traffic_add_traffic_light.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
        self.lib.ai_traffic_add_traffic_light.restype = ctypes.c_int
        self.lib.ai_traffic_get_vehicle_count.restype = ctypes.c_uint8
        self.lib.ai_traffic_trigger_emergency.argtypes = [ctypes.c_uint8]
        self.lib.ai_traffic_trigger_emergency.restype = None

        if self.lib.init_kernel() < 0: raise RuntimeError("Init failed")

    def spawn(self, eid, x, y, z): self.lib.spawn_instance(eid, x, y, z)
    def run(self, t=1): return self.lib.run_ticks(t)
    def emotions(self): return {"v": self.lib.get_emotion_valence(), "a": self.lib.get_emotion_arousal()}
    def events(self):
        return [{"t": self.lib.get_trace_entry(i).contents.tick_id,
                 "type": self.lib.get_trace_entry(i).contents.event_type.decode().rstrip('\0'),
                 "id": self.lib.get_trace_entry(i).contents.instance_id}
                for i in range(self.lib.get_trace_count())]
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
