import unittest
import sys
import os
import ctypes

# Add root dir to sys.path to import worldvm.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM, TraceEntry

class PhysicsTestHarness(unittest.TestCase):
    def setUp(self):
        self.vm = WorldVM()
        # Reset context for each test
        self.vm.lib.reset_context()

    def tearDown(self):
        self.vm.close()

    def get_velocity(self, inst_idx):
        vel = (ctypes.c_float * 3)()
        self.vm.lib.get_instance_velocity(inst_idx, vel)
        return list(vel)

    def get_angular_velocity(self, inst_idx):
        ang = (ctypes.c_float * 3)()
        self.vm.lib.get_instance_angular_velocity(inst_idx, ang)
        return list(ang)

    def get_pos(self, inst_idx):
        # Access instances array from g_state via some logic or additional exports
        # For now, let's assume we might need a get_instance_pos export if not available
        # Checking vm_hook.zig again, I don't see a get_instance_pos. 
        # I'll need to add it or use an alternative.
        pass

    def assert_velocity_zero(self, inst_idx, epsilon=0.01):
        v = self.get_velocity(inst_idx)
        self.assertTrue(all(abs(x) < epsilon for x in v), f"Velocity not zero: {v}")

    def assert_moving_down(self, inst_idx):
        v = self.get_velocity(inst_idx)
        self.assertLess(v[1], 0, f"Object not moving down: {v}")

