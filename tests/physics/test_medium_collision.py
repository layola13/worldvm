"""
Test Medium Type Collision System
Integration tests for soft/liquid medium collision handling
per todo/1.md improvement verification
"""
import unittest
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM


class TestMediumTypeCollision(unittest.TestCase):
    """Test collision behavior with different medium types"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.terrain_init()
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.close()

    def test_terrain_surface_query_concrete(self):
        """Terrain surface query for concrete"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 2)  # concrete
        surface = self.vm.lib.terrain_get_surface_at(0.0, 0.0)
        self.assertEqual(surface, 2)  # concrete

    def test_terrain_surface_query_water(self):
        """Terrain surface query for water"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 9)  # water
        surface = self.vm.lib.terrain_get_surface_at(0.0, 0.0)
        self.assertEqual(surface, 9)  # water

    def test_terrain_friction_query(self):
        """Terrain friction should be queryable"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 2)  # concrete
        friction = self.vm.lib.terrain_get_friction_at(0.0, 0.0)
        self.assertGreater(friction, 0.5)

    def test_terrain_rolling_resistance(self):
        """Terrain rolling resistance should be queryable"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 2)
        rolling = self.vm.lib.terrain_get_rolling_resistance_at(0.0, 0.0)
        self.assertGreater(rolling, 0)


class TestSurfaceTypeDistribution(unittest.TestCase):
    """Test that different surface types have distinct physical properties"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.terrain_init()

    def tearDown(self):
        self.vm.close()

    def test_medium_type_classification(self):
        """Medium types should classify correctly"""
        test_cases = [
            (2, 0),   # concrete -> solid
            (6, 1),   # mud -> soft
            (9, 2),   # water -> liquid
            (7, 0),   # ice -> solid
            (4, 1),   # grass -> soft
            (12, 1),  # cloth -> soft
            (13, 0),  # rubber -> solid
            (15, 1),  # carpet -> soft
        ]

        for surface_type, expected_medium in test_cases:
            actual_medium = self.vm.lib.material_pairing_get_medium_type(surface_type)
            self.assertEqual(actual_medium, expected_medium,
                f"Surface {surface_type} should be medium {expected_medium}, got {actual_medium}")

    def test_surface_friction_differentiation(self):
        """Different surfaces should have different friction values"""
        self.vm.lib.terrain_add_patch(0, 0, 50, 2)   # concrete
        self.vm.lib.terrain_add_patch(100, 0, 50, 6)  # mud

        # Query at different locations (non-overlapping patches)
        concrete_friction = self.vm.lib.terrain_get_friction_at(0.0, 0.0)
        mud_friction = self.vm.lib.terrain_get_friction_at(100.0, 0.0)

        # Concrete friction is high (~0.9), mud friction is low (~0.2)
        self.assertGreater(concrete_friction, 0.5)
        self.assertLess(mud_friction, 0.5)


class TestInstanceStateAPI(unittest.TestCase):
    """Test instance state observation API"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.reset_context()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.close()

    def test_is_instance_broken_invalid_index(self):
        """is_instance_broken should return -1 for invalid index"""
        result = self.vm.lib.is_instance_broken(99)
        self.assertEqual(result, -1)

    def test_get_instance_state_invalid_index(self):
        """get_instance_state should return -1 for invalid index"""
        result = self.vm.lib.get_instance_state(99)
        self.assertEqual(result, -1)


if __name__ == "__main__":
    unittest.main()
