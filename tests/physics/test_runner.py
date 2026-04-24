import unittest
import sys
import os
import ctypes

# Add root dir
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM

class PhysicsRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vm = WorldVM()

    def setUp(self):
        # 彻底重置内核上下文
        self.vm.lib.reset_context()

    def get_vel(self, idx):
        v = (ctypes.c_float * 3)()
        self.vm.lib.get_instance_velocity(idx, v)
        return list(v)

    def test_001_galileo_gravity(self):
        """1. 自由落体: 检查重力加速度的确定性"""
        # Spawn floor at Y=0
        self.vm.spawn(5, 0, 0, 0)
        # Spawn apple at Y=50 (High)
        # Note: In our current vm_hook, spawn returns c_int, 
        # but it doesn't explicitly return the index of the newly added instance.
        # However, since we reset_context, the first spawn is index 0, second is 1.
        self.vm.spawn(0, 16, 50, 16) # Apple should be index 1
        
        # Initial velocity of apple
        v_start = self.get_vel(1)
        self.assertEqual(v_start[1], 0, "Initial Y velocity should be 0")

        # Run 1 tick
        self.vm.run(1)
        v_tick1 = self.get_vel(1)
        
        # In WorldVM, GRAVITY is -10 (per physics.zig)
        # So Y velocity should become negative (falling towards 0)
        self.assertLess(v_tick1[1], 0, f"Apple should have negative Y velocity after 1 tick, got {v_tick1[1]}")
        
        # Run more ticks
        self.vm.run(5)
        v_tick6 = self.get_vel(1)
        self.assertLess(v_tick6[1], v_tick1[1], "Velocity should increase (become more negative)")

    def test_006_damping(self):
        """6. 线性阻尼: 检查速度衰减"""
        self.vm.spawn(0, 16, 10, 16) # Index 0
        # Apply massive X impulse
        self.vm.lib.apply_impulse(0, 200, 0, 0)
        
        v0 = self.get_vel(0)
        self.assertGreater(v0[0], 100) # Should be around 200
        
        # Run ticks and observe damping
        self.vm.run(10)
        v10 = self.get_vel(0)
        self.assertLess(abs(v10[0]), abs(v0[0]), "X velocity should decrease due to damping")

if __name__ == "__main__":
    unittest.main()
