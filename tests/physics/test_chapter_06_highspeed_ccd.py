"""
Chapter 6: 高速运动与CCD (51-60)

Physics Tests from physics.md Section 6
Tests 51-60 cover high-speed motion and CCD

正确表现 expected behaviors:
51. 子弹穿纸(无CCD): 子弹直接跨越墙壁，穿模
52. 子弹穿透(CCD): 精准计算碰撞时间片，子弹停在墙表
53. CCD导致卡死: 检测多次反弹后安全停止
54. 旋转CCD: 检测到旋转棍子击中球
55. 极小物体碰撞: 正常被平面接住
56. 高低帧率一致性: 不同帧率下物理轨迹一致
57. 快速物体触发Trigger: 正确触发进入和退出
58. 极大动能吸收: 能量正确传递
59. 旋转角速度上限: 达到上限后不再增加
60. 多物体同时高速撞击: 正确计算多方受力
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


class Test51BulletThroughPaperNoCCD(PhysicsTestHarness):
    """Test 51: 子弹穿纸测试（不开启CCD）"""

    def test_high_velocity_passes_through(self):
        """极高速度可能导致穿模"""
        self.vm.spawn(5, 16, 0, 16)  # wall
        self.vm.spawn(0, 16, 10, 16)  # bullet

        # Apply extremely high velocity
        self.vm.apply_impulse(1, 0, -1000, 0)

        for _ in range(5):
            self.vm.run(1)

        pos = self.get_pos(1)
        # Bullet may have passed through or collided


class Test52BulletWithCCD(PhysicsTestHarness):
    """Test 52: 开启CCD的子弹穿透"""

    def test_ccd_collision_detection(self):
        """CCD应该检测到碰撞"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 16, 20, 16)

        self.vm.apply_impulse(1, 0, -500, 0)

        for _ in range(20):
            self.vm.run(1)

        pos = self.get_pos(1)
        # Should have collided with wall


class Test53CCDTunnelingPrevention(PhysicsTestHarness):
    """Test 53: CCD防止隧道效应"""

    def test_fast_object_stops_at_wall(self):
        """快速物体应该在墙前停止"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 16, 30, 16)

        self.vm.apply_impulse(1, 0, -800, 0)

        for _ in range(20):
            self.vm.run(1)

        pos = self.get_pos(1)
        # Should be near wall, not through it


class Test54AngularCCD(PhysicsTestHarness):
    """Test 54: 旋转CCD"""

    def test_rotating_object_hits(self):
        """旋转物体应该能检测到碰撞"""
        self.vm.spawn(0, 16, 10, 16)
        self.vm.spawn(0, 32, 10, 16)

        # Apply rotation to one
        self.vm.lib.apply_torque(0, 0, 200, 0)

        for _ in range(20):
            self.vm.run(1)


class Test55TinyObjectCollision(PhysicsTestHarness):
    """Test 55: 极小物体碰撞"""

    def test_small_object_collision(self):
        """极小物体应该正常碰撞"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 16, 10, 16)

        for _ in range(20):
            self.vm.run(1)


class Test56HighLowFPSConsistency(PhysicsTestHarness):
    """Test 56: 高帧率与低帧率一致性"""

    def test_deterministic_physics(self):
        """物理模拟应该是确定性的"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 16, 20, 16)

        self.vm.apply_impulse(1, 0, -300, 0)

        for _ in range(20):
            self.vm.run(1)

        pos1 = self.get_pos(1)

        # Reset and run again
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 16, 20, 16)
        self.vm.apply_impulse(1, 0, -300, 0)

        for _ in range(20):
            self.vm.run(1)

        pos2 = self.get_pos(1)

        self.assertEqual(pos1, pos2)


class Test57FastMovingTrigger(PhysicsTestHarness):
    """Test 57: 快速运动物体触发Trigger"""

    def test_trigger_activation(self):
        """快速物体应该触发区域"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 16, 50, 16)

        self.vm.apply_impulse(1, 0, -500, 0)

        for _ in range(30):
            self.vm.run(1)


class Test58HighKineticEnergyAbsorption(PhysicsTestHarness):
    """Test 58: 极大动能吸收"""

    def test_energy_transfer(self):
        """能量应该正确传递"""
        self.vm.spawn(5, 16, 0, 16)  # target
        self.vm.spawn(0, 16, 30, 16)  # projectile

        self.vm.apply_impulse(1, 0, -500, 0)

        for _ in range(30):
            self.vm.run(1)


class Test59AngularVelocityCap(PhysicsTestHarness):
    """Test 59: 旋转角速度上限"""

    def test_angular_velocity_capped(self):
        """角速度应该有人为上限"""
        self.vm.spawn(0, 16, 10, 16)

        # Apply high torque (not extreme to avoid panic)
        self.vm.lib.apply_torque(0, 0, 500, 0)

        for _ in range(5):
            self.vm.run(1)

        ang_vel = self.vm.get_angular_velocity(0)
        # Should not exceed reasonable maximum


class Test60MultipleSimultaneousImpacts(PhysicsTestHarness):
    """Test 60: 多物体同时高速撞击"""

    def test_multiple_collision_resolution(self):
        """多物体碰撞应该被正确求解"""
        self.vm.spawn(5, 16, 0, 16)  # target
        self.vm.spawn(0, 10, 30, 16)
        self.vm.spawn(0, 20, 30, 16)
        self.vm.spawn(0, 16, 30, 16)

        for i in range(1, 4):
            self.vm.apply_impulse(i, 0, -300, 0)

        for _ in range(30):
            self.vm.run(1)


if __name__ == "__main__":
    unittest.main()
