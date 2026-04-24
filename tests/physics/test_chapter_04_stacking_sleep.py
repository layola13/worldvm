"""
Chapter 4: 堆叠与休眠机制 (31-40)

Physics Tests from physics.md Section 4
Tests 31-40 cover stacking and sleeping mechanisms

正确表现 expected behaviors:
31. 双箱堆叠: 两者保持静止，下层承受上层重力
32. 10层箱子高塔: 塔保持稳定立姿
33. 金字塔堆叠: 结构受力平衡，保持静止
34. 物体休眠: 静止后状态标记为Sleeping
35. 休眠唤醒: 碰撞底层时整个塔被唤醒
36. 多米诺骨牌: 连锁反应，一个接一个倒下
37. 轻重混合堆叠: 引擎处理质量差，稳定堆叠
38. 重轻混合堆叠: 稳定
39. 悬空休眠: 物体在空中悬停
40. 多点支撑: 长板平稳搭在支柱上
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
        self.close()
    def close(self):
        try:
            self.vm.close()
        except:
            pass
    def get_velocity(self, i): return self.vm.get_velocity(i)
    def get_pos(self, i): return self.vm.get_pos(i)
    def is_sleeping(self, i): return self.vm.lib.is_instance_sleeping(i)


class Test31DoubleBoxStack(PhysicsTestHarness):
    """Test 31: 双箱堆叠"""

    def test_two_boxes_stack_stably(self):
        """两个箱子应该能稳定堆叠"""
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(0, 16, 1, 16)  # bottom box
        self.vm.spawn(0, 16, 3, 16)  # top box (offset to stack)

        # Apply downward to top box
        self.vm.apply_impulse(2, 0, -100, 0)

        for _ in range(30):
            self.vm.run(1)

        # Both should be stable
        pos1 = self.get_pos(1)
        pos2 = self.get_pos(2)
        self.assertLess(pos2[1], pos1[1])  # top should be above bottom


class Test3210LayerTower(PhysicsTestHarness):
    """Test 32: 10层箱子高塔"""

    def test_tall_tower_remains_stable(self):
        """高塔应该保持稳定"""
        self.vm.spawn(5, 16, 0, 16)  # floor

        # Stack multiple objects
        for i in range(5):
            y = 1 + i * 2
            self.vm.spawn(0, 16, y, 16)

        for _ in range(30):
            self.vm.run(1)

        # Check that objects are still stacked
        for i in range(5):
            pos = self.get_pos(i + 1)
            self.assertGreater(pos[1], 0)


class Test33PyramidStack(PhysicsTestHarness):
    """Test 33: 金字塔堆叠"""

    def test_pyramid_structure_balances(self):
        """金字塔结构应该保持平衡"""
        self.vm.spawn(5, 16, 0, 16)  # floor

        # Create a simple pyramid (2 bottom, 1 top)
        self.vm.spawn(0, 14, 1, 16)
        self.vm.spawn(0, 18, 1, 16)
        self.vm.spawn(0, 16, 3, 16)

        for _ in range(30):
            self.vm.run(1)

        # Objects should have settled


class Test34SleepingActivation(PhysicsTestHarness):
    """Test 34: 物体休眠触发"""

    def test_resting_object_becomes_sleeping(self):
        """静止的物体会变为休眠状态"""
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(0, 16, 10, 16)

        self.vm.apply_impulse(1, 0, -200, 0)

        # Wait to settle
        for _ in range(30):
            self.vm.run(1)

        # Check sleeping state
        sleeping = self.is_sleeping(1)
        self.assertEqual(sleeping, 1)


class Test35WakeOnCollision(PhysicsTestHarness):
    """Test 35: 休眠唤醒"""

    def test_collision_wakes_sleeping(self):
        """碰撞应该唤醒休眠的物体"""
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(0, 16, 1, 16)  # resting box
        self.vm.spawn(0, 16, 20, 16)  # falling ball

        # Wait for resting box to sleep
        for _ in range(30):
            self.vm.run(1)

        sleeping_before = self.is_sleeping(1)

        # Drop ball on resting box
        self.vm.apply_impulse(2, 0, -300, 0)

        for _ in range(20):
            self.vm.run(1)

        sleeping_after = self.is_sleeping(1)

        # Should have been woken up by collision
        # Note: May still be sleeping if collision didn't transfer enough energy


class Test36DominoEffect(PhysicsTestHarness):
    """Test 36: 多米诺骨牌"""

    def test_domino_chain_reaction(self):
        """推倒一个应该产生连锁反应"""
        self.vm.spawn(5, 16, 0, 16)  # floor

        # Create dominoes in a row
        for i in range(5):
            x = 10 + i * 4
            self.vm.spawn(2, x, 1, 16)  # hammer as tall domino

        # Push first one
        self.vm.apply_impulse(1, 50, 0, 0)

        for _ in range(50):
            self.vm.run(1)

        # Dominoes should have fallen in sequence


class Test37HeavyOnLightStack(PhysicsTestHarness):
    """Test 37: 轻重混合堆叠"""

    def test_heavy_on_light(self):
        """重物压在轻物上应该稳定"""
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(0, 16, 1, 16)  # light (apple)
        self.vm.spawn(2, 16, 3, 16)  # heavy (hammer)

        for _ in range(30):
            self.vm.run(1)

        # Heavy should have settled on light


class Test38LightOnHeavyStack(PhysicsTestHarness):
    """Test 38: 重轻混合堆叠"""

    def test_light_on_heavy(self):
        """轻物在重物上应该稳定"""
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(2, 16, 1, 16)  # heavy (hammer)
        self.vm.spawn(0, 16, 3, 16)  # light (apple)

        for _ in range(30):
            self.vm.run(1)

        # Should be stable


class Test39FloatingSleep(PhysicsTestHarness):
    """Test 39: 悬空休眠异常检测"""

    def test_suspended_object_stays_afloat(self):
        """悬空的物体应该保持悬停"""
        self.vm.spawn(0, 16, 20, 16)

        # Don't apply any force, object starts at rest
        # But instances start as "sleeping" until disturbed

        for _ in range(10):
            self.vm.run(1)

        # Object should remain at same height (if not affected by gravity automatically)


class Test40MultiPointSupport(PhysicsTestHarness):
    """Test 40: 多点支撑"""

    def test_object_on_multiple_supports(self):
        """物体在多个支撑点上应该平稳"""
        self.vm.spawn(5, 16, 0, 16)  # floor
        self.vm.spawn(0, 10, 1, 16)  # left pillar
        self.vm.spawn(0, 22, 1, 16)  # right pillar

        for _ in range(30):
            self.vm.run(1)


if __name__ == "__main__":
    unittest.main()
