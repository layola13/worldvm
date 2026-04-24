import unittest
import sys
import os
import ctypes

# Add root dir to import worldvm.py
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM

class PhysicsHarness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vm = WorldVM()

    def setUp(self):
        self.vm.lib.reset_context()
        # Constants from Zig
        self.GRAVITY = 24 

    def get_vel(self, idx):
        v = (ctypes.c_float * 3)()
        self.vm.lib.get_instance_velocity(idx, v)
        return list(v)

    def is_sleeping(self, idx):
        return self.vm.lib.is_instance_sleeping(idx) != 0

    def spawn(self, eid, x, y, z):
        """Returns the index of the spawned instance"""
        # In our current hook, we assume serial indexing starting from 0
        # because we call reset_context() in setUp
        count_before = self.vm.lib.get_trace_count() # Not ideal but placeholder
        self.vm.spawn(eid, x, y, z)
        # Assuming current implementation adds to end of instances array
        # We'll need to keep track manually in this harness
        if not hasattr(self, '_spawn_count'): self._spawn_count = 0
        idx = self._spawn_count
        self._spawn_count += 1
        return idx

    def assert_is_moving_down(self, idx, msg=""):
        v = self.get_vel(idx)
        self.assertGreater(v[1], 0, f"{msg} | Velocity Y should be positive (down): {v}")

    def assert_is_resting(self, idx, msg=""):
        self.assertTrue(self.is_sleeping(idx), f"{msg} | Object should be sleeping/resting")

if __name__ == "__main__":
    unittest.main()
