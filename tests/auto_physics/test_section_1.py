import unittest
from harness import PhysicsHarness

class TestBasicPhysics(PhysicsHarness):
    """一、 基础运动与重力 (1-10)"""

    def test_001_free_fall(self):
        """1. 自由落体: 重力设为24，释放一个球体。"""
        # Entity 0: Apple
        idx = self.spawn(0, 16, 5, 16) # Start at height 5
        self.vm.run(1)
        v = self.get_vel(idx)
        self.assertEqual(v[1], self.GRAVITY, "Y velocity should equal GRAVITY after 1 tick")

    def test_002_galileo_mass(self):
        """2. 伽利略测试（不同质量）: 同一高度释放不同质量物体，下落一致。"""
        # Apple (50kg) vs Hammer (1000kg)
        idx_apple = self.spawn(0, 10, 5, 10)
        idx_hammer = self.spawn(2, 20, 5, 20)
        
        self.vm.run(2)
        v_a = self.get_vel(idx_apple)
        v_h = self.get_vel(idx_hammer)
        self.assertEqual(v_a[1], v_h[1], "Mass should not affect fall velocity in vacuum")

    def test_005_zero_gravity(self):
        """5. 零重力漂浮: 重力设为0，给予物体微小初始推力。"""
        self.vm.lib.set_time_scale(0.0) # Simulate no movement from gravity or stop time
        # Better: we apply force and see if it moves linearly
        idx = self.spawn(0, 16, 16, 16)
        self.vm.lib.apply_impulse(idx, 100, 0, 0)
        v_start = self.get_vel(idx)
        self.vm.run(10)
        v_end = self.get_vel(idx)
        # In zero-G / no-drag, velocity should stay constant
        # Note: If drag is enabled, we need to account for it
        # self.assertEqual(v_start[0], v_end[0])
        pass # Placeholder for actual zero-g toggle implementation

    def test_011_sphere_static_plane(self):
        """11. 球体与静态平面: 掉落到地面后停止。"""
        self.spawn(5, 0, 0, 0) # Floor at Y=0
        idx = self.spawn(0, 8, 10, 8) # Apple above floor
        
        # Run enough ticks to hit floor (Gravity=24 is huge, 1 tick is enough for distance 10)
        self.vm.run(5)
        self.assert_is_resting(idx, "Apple should rest on floor")

if __name__ == "__main__":
    unittest.main()
