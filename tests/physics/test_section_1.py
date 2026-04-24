import unittest
from test_harness import PhysicsTestHarness
import time

class TestBasicMotion(PhysicsTestHarness):
    def test_001_free_fall(self):
        """1. 自由落体: 检查Y轴速度是否随重力增加 (WorldVM中Y向下为正)"""
        # Entity 0: Apple
        self.vm.spawn(0, 16, 10, 16) 
        
        # Initial velocity
        v0 = self.get_velocity(0)
        
        # Step 1 tick
        self.vm.run(1)
        v1 = self.get_velocity(0)
        # 刚才debug显示 v1[1] 变为了 24.0，说明重力导致Y速度增加
        self.assertGreater(v1[1], v0[1], "Gravity should increase Y velocity (downwards)")
        
        # Step more
        self.vm.run(5)
        v5 = self.get_velocity(0)
        self.assertGreater(v5[1], v1[1], "Velocity should continue increasing")

    def test_006_linear_damping(self):
        """6. 线性阻尼: 施加X轴冲量，检查速度是否随Tick衰减"""
        self.vm.spawn(0, 16, 10, 16)
        # Apply X impulse
        self.vm.lib.apply_impulse(0, 50, 0, 0)
        
        v_start = self.get_velocity(0)
        self.assertGreater(v_start[0], 0)
        
        self.vm.run(10)
        v_after = self.get_velocity(0)
        self.assertLess(v_after[0], v_start[0], "Velocity should dampen over time")

if __name__ == "__main__":
    unittest.main()
