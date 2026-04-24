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
        self.vm.reset()

    def tearDown(self):
        self.vm.close()

    def get_velocity(self, inst_idx):
        return self.vm.get_velocity(inst_idx)

    def get_angular_velocity(self, inst_idx):
        return self.vm.get_angular_velocity(inst_idx)

    def get_pos(self, inst_idx):
        return self.vm.get_pos(inst_idx)

    def assert_velocity_zero(self, inst_idx, epsilon=0.01):
        v = self.get_velocity(inst_idx)
        self.assertTrue(all(abs(x) < epsilon for x in v), f"Velocity not zero: {v}")

    def assert_moving_down(self, inst_idx):
        v = self.get_velocity(inst_idx)
        self.assertLess(v[1], 0, f"Object not moving down: {v}")
