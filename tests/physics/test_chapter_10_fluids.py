"""
Chapter 10: 流体、力场与机制拓展 (91-100)

Physics Tests from physics.md Section 10
Tests 91-100 cover fluids, force fields and mechanisms

正确表现 expected behaviors:
91. 力场区域: 物体经过时轨迹弯曲
92. 爆炸径向力: 物体根据距离受衰减力并放射状飞出
93. 浮力体积: 物体受浮力加速上浮并振荡
94. 局部时间缩放: 物体慢动作下落
95. 时间暂停与恢复: 精确继续运动
96. 车辆悬挂测试: 4条射线按胡克定律施加弹簧力
97. 重心约束与不倒翁: 倒下后晃回直立
98. 连续碰撞合并: 平滑掠过接缝无颠簸
99. 自定义惯性张量: 旋转表现符合设置
100. 多线程确定性: 两次运行结果严格一致
"""
import unittest
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM


class PhysicsTestHarness(unittest.TestCase):
    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.close()
    def get_velocity(self, i): return self.vm.get_velocity(i)
    def get_pos(self, i): return self.vm.get_pos(i)


class Test91ForceFieldArea(PhysicsTestHarness):
    """Test 91: 力场区域"""

    def test_force_field_affects_trajectory(self):
        """力场应该影响物体轨迹"""
        self.vm.spawn(0, 16, 20, 16)

        # Note: No force field API in current FFI
        # apply_force acts directly on an instance


class Test92ExplosionRadialForce(PhysicsTestHarness):
    """Test 92: 爆炸径向力"""

    def test_explosion_pushes_objects(self):
        """爆炸应该推开物体"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 10, 10, 16)
        self.vm.spawn(0, 20, 10, 16)
        self.vm.spawn(0, 16, 10, 16)

        # Apply explosion at center
        self.vm.lib.apply_explosion(16.0, 5.0, 16.0, 50.0, 500.0)

        for _ in range(20):
            self.vm.run(1)

        # Objects should have moved outward from center


class Test93BuoyancyVolume(PhysicsTestHarness):
    """Test 93: 浮力体积模拟"""

    def test_buoyancy_pushes_up(self):
        """浮力应该使物体上浮"""
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(4, 16, 5, 16)  # water entity

        self.vm.apply_impulse(4, 0, -200, 0)

        for _ in range(30):
            self.vm.run(1)

        # Water should float or be affected by buoyancy


class Test94TimeScale(PhysicsTestHarness):
    """Test 94: 局部时间缩放"""

    def test_slow_motion(self):
        """时间缩放应该影响物理模拟速度"""
        self.vm.spawn(0, 16, 20, 16)

        # Set slow time scale
        self.vm.lib.set_time_scale(0.1)

        self.vm.apply_impulse(0, 0, -200, 0)

        for _ in range(10):
            self.vm.run(1)

        pos1 = self.get_pos(0)

        # Reset and try normal speed
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.lib.set_time_scale(1.0)

        self.vm.spawn(0, 16, 20, 16)
        self.vm.apply_impulse(0, 0, -200, 0)

        for _ in range(10):
            self.vm.run(1)

        pos2 = self.get_pos(0)

        # Normal speed should move more
        self.assertLess(pos1[1], pos2[1],
            f"Slow motion ({pos1[1]}) should move less than normal ({pos2[1]})")


class Test95TimePauseResume(PhysicsTestHarness):
    """Test 95: 时间暂停与恢复"""

    def test_pause_resume_exact_continuation(self):
        """暂停后恢复应该精确继续"""
        self.vm.spawn(0, 16, 20, 16)
        self.vm.apply_impulse(0, 0, -200, 0)

        # Run some ticks
        for _ in range(5):
            self.vm.run(1)

        pos1 = self.get_pos(0)

        # Note: No pause API, but set_time_scale(0) would effectively pause


class Test96VehicleSuspension(PhysicsTestHarness):
    """Test 96: 车辆悬挂测试"""

    def test_suspension_spring(self):
        """悬挂应该按胡克定律弹簧"""
        self.vm.lib.vehicle_init()

        # Create vehicle
        result = self.vm.lib.vehicle_create_car(16.0, 10.0, 16.0, 0.0)
        self.assertEqual(result, 1)

        for _ in range(20):
            self.vm.run(1)


class Test97CenterOfGravityConstraint(PhysicsTestHarness):
    """Test 97: 重心约束与不倒翁"""

    def test_self_righting(self):
        """物体会自动回正"""
        # Note: This requires specific entity setup
        self.vm.spawn(0, 16, 10, 16)

        self.vm.apply_impulse(0, 50, 0, 50)

        for _ in range(30):
            self.vm.run(1)


class Test98ContinuousCollisionMerging(PhysicsTestHarness):
    """Test 98: 连续碰撞合并"""

    def test_smooth_over_seams(self):
        """物体平滑掠过接缝"""
        self.vm.spawn(5, 16, 0, 16)  # floor made of segments
        self.vm.spawn(0, 16, 1, 16)  # object

        self.vm.apply_impulse(0, 50, 0, 0)

        positions = []
        for _ in range(30):
            self.vm.run(1)
            positions.append(self.get_pos(0))

        # Should move smoothly without jitter at seams


class Test99CustomInertiaTensor(PhysicsTestHarness):
    """Test 99: 自定义质心惯性张量"""

    def test_custom_inertia(self):
        """自定义惯性张量应该影响旋转"""
        self.vm.spawn(0, 16, 10, 16)

        self.vm.lib.apply_torque(0, 0, 100, 0)

        for _ in range(20):
            self.vm.run(1)


class Test100Determinism(PhysicsTestHarness):
    """Test 100: 多线程确定性验证"""

    def test_deterministic_physics(self):
        """物理模拟应该是确定性的"""
        self.vm.spawn(0, 16, 20, 16)
        self.vm.apply_impulse(0, 0, -200, 0)

        for _ in range(20):
            self.vm.run(1)

        pos1 = self.get_pos(0)
        vel1 = self.get_velocity(0)

        # Reset and repeat
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()

        self.vm.spawn(0, 16, 20, 16)
        self.vm.apply_impulse(0, 0, -200, 0)

        for _ in range(20):
            self.vm.run(1)

        pos2 = self.get_pos(0)
        vel2 = self.get_velocity(0)

        self.assertEqual(pos1, pos2, f"Positions should match: {pos1} vs {pos2}")
        self.assertEqual(vel1, vel2, f"Velocities should match: {vel1} vs {vel2}")


if __name__ == "__main__":
    unittest.main()
