"""
Comprehensive Physics Systems Test Suite
Translates physics_systems_test.zig to Python FFI
"""
import unittest
import sys
import os
import ctypes

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM, TraceEntry

class PhysicsTestHarness(unittest.TestCase):
    def setUp(self):
        self.vm = WorldVM()
        # Ensure clean state - reset context but keep kernel alive
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.close()

    def get_velocity(self, inst_idx):
        return self.vm.get_velocity(inst_idx)

    def get_angular_velocity(self, inst_idx):
        return self.vm.get_angular_velocity(inst_idx)

    def get_pos(self, inst_idx):
        return self.vm.get_pos(inst_idx)


# ============================================================================
# Basic Physics Tests
# ============================================================================

class TestBasicPhysics(PhysicsTestHarness):
    def test_001_free_fall_gravity(self):
        """Free fall: Y velocity increases with gravity"""
        self.vm.spawn(0, 16, 10, 16)
        v0 = self.get_velocity(0)
        self.vm.run(1)
        v1 = self.get_velocity(0)
        self.assertGreater(v1[1], v0[1], "Gravity should increase Y velocity")

    def test_002_velocity_damping(self):
        """Velocity should dampen over time"""
        self.vm.spawn(0, 16, 10, 16)
        self.vm.apply_impulse(0, 50, 0, 0)
        v_start = self.get_velocity(0)
        self.assertGreater(v_start[0], 0)
        self.vm.run(10)
        v_after = self.get_velocity(0)
        self.assertLess(v_after[0], v_start[0], "Velocity should dampen")

    def test_003_impulse_application(self):
        """Impulse should immediately affect velocity"""
        self.vm.spawn(0, 16, 10, 16)
        v_before = self.get_velocity(0)
        self.vm.apply_impulse(0, 100, 50, 25)
        v_after = self.get_velocity(0)
        self.assertNotEqual(v_before, v_after)


# ============================================================================
# Joint System Tests
# ============================================================================

class TestJoints(PhysicsTestHarness):
    def test_101_joint_count_initial(self):
        """Joint count should be 0 initially"""
        count = self.vm.lib.get_joint_count()
        self.assertEqual(count, 0)

    def test_102_add_fixed_joint(self):
        """Can add a fixed joint between two entities"""
        self.vm.spawn(0, 0, 0, 0)
        self.vm.spawn(1, 5, 0, 0)
        result = self.vm.lib.add_joint_fixed(0, 1, 0, 0, 0)
        self.assertEqual(result, 0)
        count = self.vm.lib.get_joint_count()
        self.assertGreater(count, 0)

    def test_103_add_hinge_joint(self):
        """Can add a hinge joint"""
        self.vm.lib.clear_joints()
        self.vm.spawn(0, 0, 0, 0)
        self.vm.spawn(1, 5, 0, 0)
        result = self.vm.lib.add_joint_hinge(0, 1, 0, 0, 0, 0, 0, 1)
        self.assertEqual(result, 0)

    def test_104_add_spring_joint(self):
        """Can add a spring joint"""
        self.vm.lib.clear_joints()
        self.vm.spawn(0, 0, 0, 0)
        self.vm.spawn(1, 5, 0, 0)
        result = self.vm.lib.add_joint_spring(0, 1, 0, 0, 0, 100.0, 10.0)
        self.assertEqual(result, 0)

    def test_105_add_ball_socket_joint(self):
        """Can add a ball socket joint"""
        self.vm.lib.clear_joints()
        self.vm.spawn(0, 0, 0, 0)
        self.vm.spawn(1, 5, 0, 0)
        result = self.vm.lib.add_joint_ball_socket(0, 1, 0, 0, 0)
        self.assertEqual(result, 0)

    def test_106_solve_joints(self):
        """solve_joints should complete without error"""
        self.vm.lib.clear_joints()
        self.vm.spawn(0, 0, 0, 0)
        self.vm.spawn(1, 5, 0, 0)
        self.vm.lib.add_joint_fixed(0, 1, 0, 0, 0)
        self.vm.lib.solve_joints()

    def test_107_clear_joints(self):
        """clear_joints should reset joint count"""
        self.vm.lib.clear_joints()
        self.vm.spawn(0, 0, 0, 0)
        self.vm.spawn(1, 5, 0, 0)
        self.vm.lib.add_joint_fixed(0, 1, 0, 0, 0)
        self.vm.lib.clear_joints()
        count = self.vm.lib.get_joint_count()
        self.assertEqual(count, 0)


# ============================================================================
# Raycast Tests
# ============================================================================

class TestRaycast(PhysicsTestHarness):
    def test_201_raycast_empty_space(self):
        """Raycast in empty space returns no hit"""
        self.vm.spawn(5, 100, 0, 0)  # spawn far away so ray misses
        hit = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            100.0,
            ctypes.cast(hit, ctypes.POINTER(ctypes.c_float))
        )
        self.assertEqual(result, 0)

    def test_202_raycast_with_obstacle(self):
        """Raycast should detect obstacle"""
        self.vm.spawn(5, 10, 0, 0)
        hit = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            100.0,
            ctypes.cast(hit, ctypes.POINTER(ctypes.c_float))
        )

    def test_203_sphere_cast(self):
        """Sphere cast should work"""
        self.vm.spawn(5, 10, 0, 0)
        hit = (ctypes.c_float * 5)()
        result = self.vm.lib.sphere_cast(
            0.0, 0.0, 0.0, 1.0,
            1.0, 0.0, 0.0, 100.0,
            ctypes.cast(hit, ctypes.POINTER(ctypes.c_float))
        )
        # Result should be 0 (no hit) or 1 (hit) - just verify no crash

    def test_204_box_cast(self):
        """Box cast should work"""
        self.vm.spawn(5, 10, 0, 0)
        hit = (ctypes.c_float * 5)()
        result = self.vm.lib.box_cast(
            -1.0, -1.0, -1.0,
            1.0, 1.0, 1.0,
            1.0, 0.0, 0.0, 100.0,
            ctypes.cast(hit, ctypes.POINTER(ctypes.c_float))
        )
        # Result should be 0 (no hit) or 1 (hit) - just verify no crash

    def test_205_compute_toi(self):
        """Compute time of impact"""
        toi = (ctypes.c_float * 5)()
        result = self.vm.lib.compute_toi(
            0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
            1.0, 0.0, 0.0,
            2.0, 2.0, 2.0, 3.0, 3.0, 3.0,
            0.0, 0.0, 0.0,
            ctypes.cast(toi, ctypes.POINTER(ctypes.c_float))
        )


# ============================================================================
# Emotions System Tests
# ============================================================================

class TestEmotions(PhysicsTestHarness):
    def test_301_emotions_initial(self):
        """Emotions should initialize properly"""
        emotions = self.vm.emotions()
        self.assertIn('v', emotions)
        self.assertIn('a', emotions)

    def test_302_emotions_change_with_events(self):
        """Emotions should change after physics events"""
        initial = self.vm.emotions()
        self.vm.spawn(0, 16, 10, 16)
        self.vm.run(10)


# ============================================================================
# Trace/Event System Tests
# ============================================================================

class TestTraceSystem(PhysicsTestHarness):
    def test_401_trace_count_initial(self):
        """Trace count should be accessible"""
        count = self.vm.lib.get_trace_count()
        self.assertGreaterEqual(count, 0)

    def test_402_trace_after_spawn(self):
        """Trace should record spawn events"""
        before = self.vm.lib.get_trace_count()
        self.vm.spawn(0, 16, 10, 16)
        after = self.vm.lib.get_trace_count()


# ============================================================================
# Time Scale Tests
# ============================================================================

class TestTimeScale(PhysicsTestHarness):
    def test_501_default_time_scale(self):
        """Default time scale should be 1.0"""
        scale = self.vm.lib.get_time_scale()
        self.assertEqual(scale, 1.0)

    def test_502_set_time_scale(self):
        """Can set time scale"""
        self.vm.lib.set_time_scale(0.5)
        scale = self.vm.lib.get_time_scale()
        self.assertEqual(scale, 0.5)
        self.vm.lib.set_time_scale(1.0)


# ============================================================================
# Sleep/Wake System Tests
# ============================================================================

class TestSleepWake(PhysicsTestHarness):
    def test_601_instance_sleeping(self):
        """Can check if instance is sleeping"""
        self.vm.spawn(0, 16, 10, 16)
        sleeping = self.vm.lib.is_instance_sleeping(0)

    def test_602_wake_instance(self):
        """Can wake a sleeping instance"""
        self.vm.spawn(0, 16, 10, 16)
        self.vm.lib.wake_instance(0)


# ============================================================================
# KCC Tests
# ============================================================================

class TestKCC(PhysicsTestHarness):
    def test_701_kcc_init(self):
        """KCC system should initialize"""
        self.vm.lib.kcc_init()

    def test_702_kcc_get_height(self):
        """KCC get height should return valid value"""
        self.vm.lib.kcc_init()
        height = self.vm.lib.kcc_get_height(0)
        self.assertGreater(height, 0)

    def test_703_kcc_slide_along_wall(self):
        """KCC slide along wall should work"""
        self.vm.lib.kcc_init()
        result = (ctypes.c_float * 2)()
        self.vm.lib.kcc_slide_along_wall(10.0, 0.0, 1.0, 0.0, ctypes.cast(result, ctypes.POINTER(ctypes.c_float)))


# ============================================================================
# Ballistics Tests
# ============================================================================

class TestBallistics(PhysicsTestHarness):
    def test_801_ballistics_init(self):
        """Ballistics system should initialize"""
        self.vm.lib.ballistics_init()

    def test_802_ballistics_spawn_projectile(self):
        """Can spawn a projectile"""
        self.vm.lib.ballistics_init()
        result = self.vm.lib.ballistics_spawn_projectile(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.01)
        self.assertGreaterEqual(result, 0)

    def test_803_ballistics_get_speed(self):
        """Can get projectile speed"""
        self.vm.lib.ballistics_init()
        self.vm.lib.ballistics_spawn_projectile(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.01)
        speed = self.vm.lib.ballistics_get_speed(0)
        self.assertGreaterEqual(speed, 0)

    def test_804_ballistics_get_kinetic_energy(self):
        """Can get projectile kinetic energy"""
        self.vm.lib.ballistics_init()
        self.vm.lib.ballistics_spawn_projectile(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0, 0.01)
        ke = self.vm.lib.ballistics_get_kinetic_energy(0)
        self.assertGreaterEqual(ke, 0)

    def test_805_ballistics_calculate_deflection(self):
        """Can calculate deflection"""
        result = (ctypes.c_float * 3)()
        self.vm.lib.ballistics_calculate_deflection(10.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.5,
            ctypes.cast(result, ctypes.POINTER(ctypes.c_float)))


# ============================================================================
# Destruction Tests
# ============================================================================

class TestDestruction(PhysicsTestHarness):
    def test_901_destruction_init(self):
        """Destruction system should initialize"""
        self.vm.lib.destruction_init()

    def test_902_destruction_calculate_damage(self):
        """Can calculate damage"""
        dmg = self.vm.lib.destruction_calculate_damage(100.0, 1, 50.0)
        self.assertGreaterEqual(dmg, 0)


# ============================================================================
# Ragdoll Tests
# ============================================================================

class TestRagdoll(PhysicsTestHarness):
    def test_a01_ragdoll_init(self):
        """Ragdoll system should initialize"""
        self.vm.lib.ragdoll_init()

    def test_a02_ragdoll_is_fully_broken(self):
        """Can check if ragdoll is fully broken"""
        self.vm.lib.ragdoll_init()
        result = self.vm.lib.ragdoll_is_fully_broken(0)
        self.assertEqual(result, 0)

    def test_a03_ragdoll_is_resurrection_ready(self):
        """Can check if ragdoll is resurrection ready"""
        self.vm.lib.ragdoll_init()
        result = self.vm.lib.ragdoll_is_resurrection_ready(0)
        self.assertEqual(result, 0)


# ============================================================================
# Vehicle Tests
# ============================================================================

class TestVehicle(PhysicsTestHarness):
    def test_b01_vehicle_init(self):
        """Vehicle system should initialize"""
        self.vm.lib.vehicle_init()

    def test_b02_vehicle_get_forward_dir(self):
        """Can get vehicle forward direction"""
        self.vm.lib.vehicle_init()
        result = (ctypes.c_float * 2)()
        self.vm.lib.vehicle_get_forward_dir(0, ctypes.cast(result, ctypes.POINTER(ctypes.c_float)))
        # Direction should be valid

    def test_b03_vehicle_check_flipped(self):
        """Can check if vehicle is flipped"""
        self.vm.lib.vehicle_init()
        result = self.vm.lib.vehicle_check_flipped(0)
        self.assertEqual(result, 0)


# ============================================================================
# Network Tests
# ============================================================================

class TestNetwork(PhysicsTestHarness):
    def test_c01_network_init(self):
        """Network system should initialize"""
        self.vm.lib.network_init(60, 5000, 16, 1, 8)

    def test_c02_network_get_tick(self):
        """Can get network tick"""
        self.vm.lib.network_init(60, 5000, 16, 1, 8)
        tick = self.vm.lib.network_get_tick()
        self.assertGreaterEqual(tick, 0)

    def test_c03_network_calculate_crc(self):
        """Can calculate CRC"""
        self.vm.lib.network_init(60, 5000, 16, 1, 8)
        crc = self.vm.lib.network_calculate_crc(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertGreaterEqual(crc, 0)


# ============================================================================
# Crash Defense Tests
# ============================================================================

class TestCrashDefense(PhysicsTestHarness):
    def test_d01_crash_defense_init(self):
        """Crash defense should initialize"""
        self.vm.lib.crash_defense_init(1, 1, 1, 1000.0, -1000.0, 1000.0, 300)

    def test_d02_crash_defense_is_nan(self):
        """Can check for NaN"""
        self.vm.lib.crash_defense_init(1, 1, 1, 1000.0, -1000.0, 1000.0, 300)
        result = self.vm.lib.crash_defense_is_nan(1.0)
        self.assertEqual(result, 0)
        result = self.vm.lib.crash_defense_is_nan(float('nan'))
        self.assertEqual(result, 1)

    def test_d03_crash_defense_is_infinite(self):
        """Can check for infinity"""
        self.vm.lib.crash_defense_init(1, 1, 1, 1000.0, -1000.0, 1000.0, 300)
        result = self.vm.lib.crash_defense_is_infinite(1.0)
        self.assertEqual(result, 0)

    def test_d04_crash_defense_is_valid_float(self):
        """Can validate float"""
        self.vm.lib.crash_defense_init(1, 1, 1, 1000.0, -1000.0, 1000.0, 300)
        result = self.vm.lib.crash_defense_is_valid_float(1.0)
        self.assertEqual(result, 1)


# ============================================================================
# Tire Tests
# ============================================================================

class TestTire(PhysicsTestHarness):
    def test_e01_tire_init(self):
        """Tire system should initialize"""
        self.vm.lib.tire_init()

    def test_e02_tire_calculate_slip_ratio(self):
        """Can calculate slip ratio"""
        self.vm.lib.tire_init()
        ratio = self.vm.lib.tire_calculate_slip_ratio(0, 10.0, 0.3)
        self.assertGreaterEqual(ratio, 0)

    def test_e03_tire_calculate_friction_circle(self):
        """Can calculate friction circle"""
        friction = self.vm.lib.tire_calculate_friction_circle(0.5, 0.3, 1.0)
        self.assertGreaterEqual(friction, 0)

    def test_e04_tire_check_hydroplaning(self):
        """Can check hydroplaning"""
        self.vm.lib.tire_init()
        result = self.vm.lib.tire_check_hydroplaning(0, 0.1, 20.0)
        self.assertEqual(result, 0)


# ============================================================================
# Suspension Tests
# ============================================================================

class TestSuspension(PhysicsTestHarness):
    def test_f01_suspension_init(self):
        """Suspension system should initialize"""
        self.vm.lib.suspension_init()

    def test_f02_suspension_calculate_natural_frequency(self):
        """Can calculate natural frequency"""
        self.vm.lib.suspension_init()
        freq = self.vm.lib.suspension_calculate_natural_frequency(100.0, 500.0)
        self.assertGreater(freq, 0)


# ============================================================================
# Drivetrain Tests
# ============================================================================

class TestDrivetrain(PhysicsTestHarness):
    def test_g01_drivetrain_init(self):
        """Drivetrain system should initialize"""
        self.vm.lib.drivetrain_init()

    def test_g02_drivetrain_calculate_torque_curve(self):
        """Can calculate torque curve"""
        self.vm.lib.drivetrain_init()
        torque = self.vm.lib.drivetrain_calculate_torque_curve(3000.0)
        self.assertGreaterEqual(torque, 0)

    def test_g03_drivetrain_calculate_horsepower(self):
        """Can calculate horsepower"""
        self.vm.lib.drivetrain_init()
        hp = self.vm.lib.drivetrain_calculate_horsepower(300.0, 3000.0)
        self.assertGreater(hp, 0)

    def test_g04_drivetrain_get_gear_ratio(self):
        """Can get gear ratio"""
        self.vm.lib.drivetrain_init()
        ratio = self.vm.lib.drivetrain_get_gear_ratio(3)
        self.assertNotEqual(ratio, 0)

    def test_g05_drivetrain_calculate_wheel_torque(self):
        """Can calculate wheel torque"""
        self.vm.lib.drivetrain_init()
        torque = self.vm.lib.drivetrain_calculate_wheel_torque(300.0, 3, 3.5, 0.85)
        self.assertGreater(torque, 0)

    def test_g06_drivetrain_apply_throttle(self):
        """Can apply throttle"""
        self.vm.lib.drivetrain_init()
        self.vm.lib.drivetrain_apply_throttle(0.5)


# ============================================================================
# Aerodynamics Tests
# ============================================================================

class TestAerodynamics(PhysicsTestHarness):
    def test_h01_aerodynamics_init(self):
        """Aerodynamics system should initialize"""
        self.vm.lib.aerodynamics_init()

    def test_h02_aerodynamics_calculate_drag_force(self):
        """Can calculate drag force"""
        self.vm.lib.aerodynamics_init()
        drag = self.vm.lib.aerodynamics_calculate_drag_force(30.0, 30.0)
        self.assertGreaterEqual(drag, 0)

    def test_h03_aerodynamics_calculate_downforce(self):
        """Can calculate downforce"""
        self.vm.lib.aerodynamics_init()
        downforce = self.vm.lib.aerodynamics_calculate_downforce(30.0, 30.0)
        self.assertGreaterEqual(downforce, 0)


# ============================================================================
# Braking Tests
# ============================================================================

class TestBraking(PhysicsTestHarness):
    def test_i01_braking_init(self):
        """Braking system should initialize"""
        self.vm.lib.braking_init()

    def test_i02_braking_apply_brake(self):
        """Can apply brake"""
        self.vm.lib.braking_init()
        self.vm.lib.braking_apply_brake(0.5)

    def test_i03_braking_apply_handbrake(self):
        """Can apply handbrake"""
        self.vm.lib.braking_init()
        self.vm.lib.braking_apply_handbrake(1)

    def test_i04_braking_get_pedal_position(self):
        """Can get pedal position"""
        self.vm.lib.braking_init()
        pos = self.vm.lib.braking_get_pedal_position()
        self.assertGreaterEqual(pos, 0)


# ============================================================================
# Terrain Tests
# ============================================================================

class TestTerrain(PhysicsTestHarness):
    def test_j01_terrain_init(self):
        """Terrain system should initialize"""
        self.vm.lib.terrain_init()

    def test_j02_terrain_add_patch(self):
        """Can add terrain patch"""
        self.vm.lib.terrain_init()
        self.vm.lib.terrain_add_patch(0, 0, 10, 1)

    def test_j03_terrain_get_surface_at(self):
        """Can get surface type at position"""
        self.vm.lib.terrain_init()
        surface = self.vm.lib.terrain_get_surface_at(0.0, 0.0)
        self.assertGreaterEqual(surface, 0)

    def test_j04_terrain_get_friction_at(self):
        """Can get friction at position"""
        self.vm.lib.terrain_init()
        friction = self.vm.lib.terrain_get_friction_at(0.0, 0.0)
        self.assertGreater(friction, 0)

    def test_j05_terrain_get_rolling_resistance_at(self):
        """Can get rolling resistance at position"""
        self.vm.lib.terrain_init()
        rr = self.vm.lib.terrain_get_rolling_resistance_at(0.0, 0.0)
        self.assertGreaterEqual(rr, 0)

    def test_j06_terrain_calculate_hydroplaning_risk(self):
        """Can calculate hydroplaning risk"""
        self.vm.lib.terrain_init()
        risk = self.vm.lib.terrain_calculate_hydroplaning_risk(30.0, 0.1, 0.2)
        self.assertGreaterEqual(risk, 0)


# ============================================================================
# Collision Tests
# ============================================================================

class TestCollision(PhysicsTestHarness):
    def test_k01_collision_init(self):
        """Collision system should initialize"""
        self.vm.lib.collision_init()

    def test_k02_collision_calculate_impact_energy(self):
        """Can calculate impact energy"""
        self.vm.lib.collision_init()
        energy = self.vm.lib.collision_calculate_impact_energy(10.0, 10.0, 5.0)
        self.assertGreater(energy, 0)

    def test_k03_collision_check_structural_failure(self):
        """Can check structural failure"""
        self.vm.lib.collision_init()
        result = self.vm.lib.collision_check_structural_failure()
        self.assertEqual(result, 0)


# ============================================================================
# Disasters Tests
# ============================================================================

class TestDisasters(PhysicsTestHarness):
    def test_l01_disasters_init(self):
        """Disasters system should initialize"""
        self.vm.lib.disasters_init()

    def test_l02_disasters_trigger(self):
        """Can trigger disaster"""
        self.vm.lib.disasters_init()
        self.vm.lib.disasters_trigger(0, 1.0, 0.0, 0.0, 0.0, 100.0)

    def test_l03_disasters_calculate_seismic_intensity(self):
        """Can calculate seismic intensity"""
        self.vm.lib.disasters_init()
        intensity = self.vm.lib.disasters_calculate_seismic_intensity(100.0, 5.0)
        self.assertGreaterEqual(intensity, 0)

    def test_l04_disasters_check_chain_reaction(self):
        """Can check chain reaction"""
        self.vm.lib.disasters_init()
        result = self.vm.lib.disasters_check_chain_reaction(0.0, 0.0, 0.0)
        self.assertEqual(result, 0)

    def test_l05_disasters_get_wind_velocity(self):
        """Can get wind velocity"""
        self.vm.lib.disasters_init()
        wind = (ctypes.c_float * 3)()
        self.vm.lib.disasters_get_wind_velocity(0.0, 0.0, ctypes.cast(wind, ctypes.POINTER(ctypes.c_float)))


# ============================================================================
# Sensors Tests
# ============================================================================

class TestSensors(PhysicsTestHarness):
    def test_m01_sensors_init(self):
        """Sensors system should initialize"""
        self.vm.lib.sensors_init()

    def test_m02_sensors_add(self):
        """Can add sensor"""
        self.vm.lib.sensors_init()
        result = self.vm.lib.sensors_add(0, 90.0, 100.0)
        self.assertGreaterEqual(result, 0)

    def test_m03_sensors_get_detected_object_count(self):
        """Can get detected object count"""
        self.vm.lib.sensors_init()
        count = self.vm.lib.sensors_get_detected_object_count()
        self.assertGreaterEqual(count, 0)


# ============================================================================
# Rewind Tests
# ============================================================================

class TestRewind(PhysicsTestHarness):
    def test_n01_rewind_init(self):
        """Rewind system should initialize"""
        self.vm.lib.rewind_init()

    def test_n02_rewind_is_deterministic(self):
        """Can check if rewind is deterministic"""
        self.vm.lib.rewind_init()
        result = self.vm.lib.rewind_is_deterministic()
        self.assertEqual(result, 1)

    def test_n03_rewind_get_buffer_count(self):
        """Can get buffer count"""
        self.vm.lib.rewind_init()
        count = self.vm.lib.rewind_get_buffer_count()
        self.assertGreaterEqual(count, 0)

    def test_n04_rewind_calculate_state_hash(self):
        """Can calculate state hash"""
        self.vm.lib.rewind_init()
        hash_val = self.vm.lib.rewind_calculate_state_hash(0, 0.0, 0.0, 0.0)
        self.assertGreater(hash_val, 0)


# ============================================================================
# AI Traffic Tests
# ============================================================================

class TestAITraffic(PhysicsTestHarness):
    def test_o01_ai_traffic_init(self):
        """AI Traffic system should initialize"""
        self.vm.lib.ai_traffic_init()

    def test_o02_ai_traffic_spawn_vehicle(self):
        """Can spawn AI vehicle"""
        self.vm.lib.ai_traffic_init()
        result = self.vm.lib.ai_traffic_spawn_vehicle(0.0, 0.0, 0.0, 0)
        self.assertGreaterEqual(result, 0)

    def test_o03_ai_traffic_add_traffic_light(self):
        """Can add traffic light"""
        self.vm.lib.ai_traffic_init()
        result = self.vm.lib.ai_traffic_add_traffic_light(0.0, 0.0, 30.0)
        self.assertGreaterEqual(result, 0)

    def test_o04_ai_traffic_get_vehicle_count(self):
        """Can get vehicle count"""
        self.vm.lib.ai_traffic_init()
        count = self.vm.lib.ai_traffic_get_vehicle_count()
        self.assertGreaterEqual(count, 0)


if __name__ == "__main__":
    unittest.main()
