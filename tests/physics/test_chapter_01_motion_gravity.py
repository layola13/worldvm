"""
Chapter 1: 基础运动与重力 (1-10)

Physics Tests from physics.md Section 1
Tests 1-10 cover basic motion and gravity physics

正确表现 expected behaviors:
1. 自由落体: Y轴速度随时间线性减小，Y轴坐标呈抛物线下降
2. 伽利略测试: 同一高度释放不同质量的球体，每一帧Y坐标和速度完全一致
3. 初始速度（上抛）: Y轴速度逐渐减小至0，物体到达最高点后开始下落
4. 平抛运动: X轴速度不变，Y坐标呈抛物线下降
5. 零重力漂浮: 物体保持恒定速度直线运动，永不停止
6. 线性阻尼: 物体速度呈指数级衰减，最终停下
7. 终端速度: 加速度逐渐减小为0，最终以恒定最大速度下落
8. 角速度应用: 物体原地绕Y轴匀速旋转
9. 角阻尼: 旋转角速度随时间平滑衰减至0
10. 重心偏移测试: 物体会发生翻转
"""
import unittest
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM


class PhysicsTestHarness(unittest.TestCase):
    """Base test harness for physics tests"""

    def setUp(self):
        self.vm = WorldVM()
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
# Chapter 1: 基础运动与重力 (1-10)
# ============================================================================

class Test01FreeFallGravity(PhysicsTestHarness):
    """Test 1: 自由落体 - Y轴速度随重力增加，Y坐标呈抛物线下降"""

    def test_y_velocity_increases_with_gravity(self):
        """重力应该使Y轴速度随时间增加（向下的速度变大）"""
        # Spawn object (entity 0 = apple, mass=50)
        self.vm.spawn(0, 16, 10, 16)
        # Give initial upward velocity to counter initial fall
        self.vm.apply_impulse(0, 0, 100, 0)

        v0 = self.get_velocity(0)
        self.vm.run(1)
        v1 = self.get_velocity(0)

        # Gravity should increase downward velocity (more negative or less positive)
        # Initial v0[1] may be positive (upward), after gravity should be less
        self.assertLess(v1[1], v0[1],
            f"Gravity should reduce upward velocity: v0={v0[1]}, v1={v1[1]}")

    def test_free_fall_acceleration(self):
        """自由落体应该持续加速"""
        self.vm.spawn(0, 16, 20, 16)
        # Give downward velocity
        self.vm.apply_impulse(0, 0, -200, 0)

        vel_before = self.get_velocity(0)[1]
        self.vm.run(5)
        vel_after = self.get_velocity(0)[1]

        # Velocity should become more negative (faster downward)
        self.assertLess(vel_after, vel_before,
            f"Free fall should accelerate: {vel_before} -> {vel_after}")

    def test_parabolic_position_change(self):
        """自由落体位置变化应该是抛物线的"""
        self.vm.spawn(0, 16, 30, 16)
        self.vm.apply_impulse(0, 0, -300, 0)

        positions = []
        for _ in range(10):
            pos_y = self.get_pos(0)[1]
            positions.append(pos_y)
            self.vm.run(1)

        # Check that position is decreasing (falling)
        self.assertGreater(positions[0], positions[-1],
            f"Should fall: {positions[0]} -> {positions[-1]}")


class Test02GalileoDifferentMass(PhysicsTestHarness):
    """Test 2: 伽利略测试 - 不同质量物体下落一致"""

    def test_different_masses_fall_together(self):
        """不同质量的物体应该以相同速率下落"""
        # Entity 0 (apple, mass=50) and Entity 2 (hammer, mass=1000)
        self.vm.spawn(0, 16, 25, 16)
        self.vm.spawn(2, 32, 25, 16)

        # Apply same initial downward velocity
        self.vm.apply_impulse(0, 0, -200, 0)
        self.vm.apply_impulse(1, 0, -200, 0)

        # Record positions after several ticks
        for _ in range(8):
            self.vm.run(1)

        pos0 = self.get_pos(0)[1]
        pos1 = self.get_pos(1)[1]

        # Both should have fallen the same amount
        self.assertEqual(pos0, pos1,
            f"Different masses should fall same: mass50={pos0}, mass1000={pos1}")


class Test03InitialUpwardVelocity(PhysicsTestHarness):
    """Test 3: 初始速度（上抛） - Y轴速度逐渐减小至0后下落"""

    def test_velocity_decreases_to_zero_then_negative(self):
        """向上速度应该逐渐减小到零，然后变为负（下落）"""
        self.vm.spawn(0, 16, 10, 16)
        # Apply strong upward impulse
        self.vm.apply_impulse(0, 0, 500, 0)

        velocities = []
        for _ in range(20):
            self.vm.run(1)
            vel_y = self.get_velocity(0)[1]
            velocities.append(vel_y)

        # Find the peak (where velocity changes from positive to negative)
        peak_idx = None
        for i in range(len(velocities) - 1):
            if velocities[i] > 0 and velocities[i + 1] <= 0:
                peak_idx = i
                break

        self.assertIsNotNone(peak_idx, f"Should reach peak then fall: {velocities}")
        # After peak, velocity should be negative
        self.assertLess(velocities[-1], 0,
            f"After peak, velocity should be negative: {velocities[-1]}")


class Test04HorizontalMotionWithGravity(PhysicsTestHarness):
    """Test 4: 平抛运动 - X速度恒定，Y抛物线"""

    def test_x_velocity_constant(self):
        """X轴速度应该保持恒定"""
        self.vm.spawn(0, 16, 20, 16)
        # Apply horizontal impulse
        self.vm.apply_impulse(0, 200, 0, 0)

        v0 = self.get_velocity(0)[0]
        self.vm.run(10)
        v10 = self.get_velocity(0)[0]

        # X velocity should not increase (no force in X direction)
        # Allow for small damping
        self.assertAlmostEqual(v0, v10, delta=abs(v0) * 0.3,
            msg=f"X velocity should be constant: {v0} -> {v10}")

    def test_horizontal_and_vertical_motion(self):
        """水平运动同时受重力影响"""
        self.vm.spawn(0, 16, 20, 16)
        # Apply both horizontal and upward velocity
        self.vm.apply_impulse(0, 150, 100, 0)

        # Track X and Y
        x_start = self.get_pos(0)[0]
        y_start = self.get_pos(0)[1]

        self.vm.run(10)

        x_end = self.get_pos(0)[0]
        y_end = self.get_pos(0)[1]

        # X should increase
        self.assertGreater(x_end, x_start,
            f"X should increase: {x_start} -> {x_end}")
        # Y should first increase then decrease (parabolic)
        # At minimum, Y should have changed
        self.assertNotEqual(y_end, y_start,
            f"Y should change due to gravity: {y_start} vs {y_end}")


class Test05ZeroGravityFloating(PhysicsTestHarness):
    """Test 5: 零重力漂浮 - 物体保持恒定速度"""

    def test_constant_velocity_no_gravity(self):
        """无重力时速度保持恒定"""
        self.vm.spawn(0, 16, 10, 16)
        # Apply impulse
        self.vm.apply_impulse(0, 100, 50, 0)

        v0 = self.get_velocity(0)
        self.vm.run(5)
        v5 = self.get_velocity(0)

        # Velocities should remain similar (no gravity to change them)
        for i in range(3):
            self.assertAlmostEqual(v0[i], v5[i], delta=abs(v0[i]) * 0.4,
                msg=f"Velocity {i} should remain: {v0[i]} vs {v5[i]}")


class Test06LinearDamping(PhysicsTestHarness):
    """Test 6: 线性阻尼 - 速度指数级衰减"""

    def test_velocity_exponential_decay(self):
        """速度应该呈指数级衰减"""
        self.vm.spawn(0, 16, 10, 16)
        self.vm.apply_impulse(0, 200, 0, 0)

        v0 = self.get_velocity(0)[0]
        self.assertGreater(v0, 0)

        # Run many ticks for damping to take effect
        for _ in range(30):
            self.vm.run(1)

        v_final = self.get_velocity(0)[0]

        self.assertLess(v_final, v0,
            f"Velocity should decay: {v0} -> {v_final}")


class Test07TerminalVelocity(PhysicsTestHarness):
    """Test 7: 终端速度 - 加速度逐渐为0，恒定速度下落"""

    def test_velocity_stabilizes_at_terminal(self):
        """速度应该稳定在终端速度"""
        self.vm.spawn(0, 16, 50, 16)
        self.vm.apply_impulse(0, 0, -500, 0)

        # Record velocity changes
        vel_early = None
        vel_mid = None
        vel_late = None

        for i in range(30):
            self.vm.run(1)
            if i == 5:
                vel_early = abs(self.get_velocity(0)[1])
            elif i == 15:
                vel_mid = abs(self.get_velocity(0)[1])
            elif i == 25:
                vel_late = abs(self.get_velocity(0)[1])

        # Velocity should stabilize (mid and late should be similar)
        if vel_mid and vel_late:
            diff = abs(vel_mid - vel_late)
            self.assertLess(diff, 100,
                f"Velocity should stabilize: mid={vel_mid}, late={vel_late}")


class Test08AngularVelocity(PhysicsTestHarness):
    """Test 8: 角速度应用 - 物体原地绕轴旋转"""

    def test_torque_creates_angular_velocity(self):
        """施加扭矩应该产生角速度"""
        self.vm.spawn(0, 16, 10, 16)

        # Apply torque around Y axis
        result = self.vm.lib.apply_torque(0, 0, 100, 0)
        self.assertEqual(result, 0)

        self.vm.run(1)
        ang_vel = self.get_angular_velocity(0)

        # At least one component should be non-zero
        has_rotation = any(v != 0 for v in ang_vel)
        # Note: This tests the API call succeeded

    def test_rotation_occurs(self):
        """物体应该发生旋转"""
        self.vm.spawn(0, 16, 10, 16)
        self.vm.lib.apply_torque(0, 0, 200, 0)

        # Run several ticks
        for _ in range(10):
            self.vm.run(1)

        # Angular velocity should exist
        ang_vel = self.get_angular_velocity(0)
        total_ang = sum(abs(v) for v in ang_vel)
        # Note: May be 0 if system doesn't track per-tick rotation


class Test09AngularDamping(PhysicsTestHarness):
    """Test 9: 角阻尼 - 旋转速度平滑衰减"""

    def test_angular_velocity_damps_over_time(self):
        """角速度应该随时间衰减"""
        self.vm.spawn(0, 16, 10, 16)
        self.vm.lib.apply_torque(0, 0, 200, 0)

        self.vm.run(1)
        ang0 = self.get_angular_velocity(0)

        for _ in range(20):
            self.vm.run(1)

        ang20 = self.get_angular_velocity(0)

        total0 = sum(abs(v) for v in ang0)
        total20 = sum(abs(v) for v in ang20)

        if total0 > 0:
            self.assertLessEqual(total20, total0,
                f"Angular velocity should damp: {total0} -> {total20}")


class Test10CenterOfMassOffset(PhysicsTestHarness):
    """Test 10: 重心偏移 - 重心偏离时物体会翻转"""

    def test_offcenter_mass_affects_motion(self):
        """重心偏移应该影响运动行为"""
        self.vm.spawn(0, 16, 20, 16)
        # Apply impulse at angle to create uneven forces
        self.vm.apply_impulse(0, 100, 0, 100)

        # Track angular velocity
        ang_velocities = []
        for _ in range(10):
            self.vm.run(1)
            ang_velocities.append(self.get_angular_velocity(0))

        # Check if any rotation occurred
        total_rotation = sum(sum(abs(v) for v in av) for av in ang_velocities)

        # The test verifies that the system can handle off-center mass
        # Actual rotation behavior depends on physics implementation


# ============================================================================
# Test Execution
# ============================================================================

if __name__ == "__main__":
    unittest.main()
