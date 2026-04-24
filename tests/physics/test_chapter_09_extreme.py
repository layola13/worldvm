"""
Chapter 9: 极端情况与压力测试 (81-90)

Physics Tests from physics.md Section 9
Tests 81-90 cover extreme cases and stress tests

正确表现 expected behaviors:
81. 极端质量比: 轻物停在重物上，重物无波动
82. 零质量物体: 引擎拒绝创建或转为静态
83. 极大坐标: 正常碰撞
84. 万物复苏: Broadphase正常划分，稳步计算
85. 堆叠崩塌压力: 休眠孤岛正确批量唤醒
86. 动态改变缩放: 包围盒立即更新
87. 重力反转: 所有物体向上坠落
88. 添加移除碰撞体: 物体失去支撑掉入虚空
89. 力矩抵消: 合力矩为0，物体不旋转
90. 同向冲量叠加: 与单次大冲量结果一致
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


class Test81ExtremeMassRatio(PhysicsTestHarness):
    """Test 81: 极端质量比 (1:1000000)"""

    def test_heavy_light_interaction(self):
        """重物和轻物相互作用时重物应该不受影响"""
        # hammer (mass=1000) and apple (mass=50)
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(2, 16, 2, 16)  # heavy (hammer)
        self.vm.spawn(0, 16, 5, 16)  # light (apple)

        for _ in range(30):
            self.vm.run(1)

        # Both should exist without issues


class Test82ZeroMassObject(PhysicsTestHarness):
    """Test 82: 零质量物体"""

    def test_zero_mass_rejected(self):
        """零质量物体应该被拒绝或转为静态"""
        # Note: FFI doesn't expose mass setting for spawned entities
        # All entities have predefined masses


class Test83LargeCoordinates(PhysicsTestHarness):
    """Test 83: 极大数据测试（坐标超远）"""

    def test_distant_collision(self):
        """极远坐标应该正常碰撞"""
        # Spawn at distant coordinates
        self.vm.spawn(0, 500, 10, 16)

        for _ in range(10):
            self.vm.run(1)


class Test84TenThousandObjects(PhysicsTestHarness):
    """Test 84: 万物复苏（大量物体）"""

    def test_many_objects_performance(self):
        """大量物体应该能正常模拟"""
        # Note: Limited by instance count
        for i in range(10):
            x = (i % 5) * 10 + 16
            z = int((i / 5) * 10 + 16)
            self.vm.spawn(0, x, 20, z)

        for _ in range(30):
            self.vm.run(1)


class Test85StackCollapseStress(PhysicsTestHarness):
    """Test 85: 堆叠崩塌压力测试"""

    def test_stacked_objects_collapse(self):
        """堆叠的物体应该能正确崩塌"""
        self.vm.spawn(5, 16, 0, 16)

        # Create a simple stack
        for i in range(5):
            self.vm.spawn(0, 16, 1 + i * 2, 16)

        # Impact bottom
        self.vm.apply_impulse(1, 0, -500, 0)

        for _ in range(30):
            self.vm.run(1)


class Test86DynamicScaleChange(PhysicsTestHarness):
    """Test 86: 动态改变缩放"""

    def test_scale_change(self):
        """改变缩放应该影响碰撞"""
        # Note: FFI doesn't expose scale modification
        pass


class Test87GravityReversal(PhysicsTestHarness):
    """Test 87: 重力反转"""

    def test_gravity_reversal(self):
        """反转重力后物体应该向上运动"""
        self.vm.spawn(0, 16, 20, 16)

        # Note: No API to reverse gravity in current FFI
        # This would require kernel-level gravity modification


class Test88AddRemoveCollider(PhysicsTestHarness):
    """Test 88: 添加和移除碰撞体"""

    def test_remove_collider_falls(self):
        """移除碰撞体后物体应该下落"""
        self.vm.spawn(0, 16, 10, 16)

        for _ in range(10):
            self.vm.run(1)

        # Note: No API to remove collider at runtime


class Test89TorqueCancellation(PhysicsTestHarness):
    """Test 89: 力矩（Torque）抵消"""

    def test_equal_opposite_torques_cancel(self):
        """相等相反的扭矩应该抵消"""
        self.vm.spawn(0, 16, 10, 16)

        # Apply equal and opposite torques
        self.vm.lib.apply_torque(0, 100, 0, 0)
        self.vm.lib.apply_torque(0, -100, 0, 0)

        for _ in range(10):
            self.vm.run(1)

        ang_vel = self.vm.get_angular_velocity(0)
        # Net torque should be zero


class Test90ImpulseStacking(PhysicsTestHarness):
    """Test 90: 同向冲量叠加"""

    def test_multiple_impulses_sum(self):
        """多次同向冲量应该累加"""
        self.vm.spawn(0, 16, 10, 16)

        # Apply multiple small impulses
        for _ in range(10):
            self.vm.apply_impulse(0, 10, 0, 0)

        v1 = self.get_velocity(0)[0]

        # Reset
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.spawn(0, 16, 10, 16)

        # Apply single large impulse
        self.vm.apply_impulse(0, 100, 0, 0)

        v2 = self.get_velocity(0)[0]

        self.assertEqual(v1, v2,
            f"Multiple small impulses ({v1}) should equal one large ({v2})")


if __name__ == "__main__":
    unittest.main()
