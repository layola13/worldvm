"""
Acceptance Test - Iron Ball / Glass Ball / Bowling Ball Scenarios
per docs/README.md verification scenario

Tests verify:
- Terrain surface queries (concrete, mud, water)
- Material pairing restitution/friction values
- Medium type classification (solid/soft/liquid)
- Instance state observation API
"""
import unittest
import sys
import os
import ctypes

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM


class TestTerrainSurfaceQuery(unittest.TestCase):
    """Test terrain surface queries work correctly"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.close()

    def test_concrete_surface(self):
        """Concrete terrain patch should return correct surface type"""
        self.vm.lib.terrain_add_patch(0, 0, 50, 2)  # concrete
        surface = self.vm.lib.terrain_get_surface_at(0.0, 0.0)
        self.assertEqual(surface, 2)  # concrete = 2

    def test_mud_surface(self):
        """Mud terrain patch should return correct surface type"""
        # Add mud at a different location to avoid overlap with concrete
        self.vm.lib.terrain_add_patch(100, 0, 50, 6)  # mud at x=100
        surface = self.vm.lib.terrain_get_surface_at(100.0, 0.0)
        self.assertEqual(surface, 6)  # mud = 6

    def test_water_surface(self):
        """Water terrain patch should return correct surface type"""
        # Add water at a different location
        self.vm.lib.terrain_add_patch(200, 0, 50, 9)  # water at x=200
        surface = self.vm.lib.terrain_get_surface_at(200.0, 0.0)
        self.assertEqual(surface, 9)  # water = 9


class TestMaterialPairingAcceptance(unittest.TestCase):
    """Verify material pairing system works end-to-end"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.reset_context()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.close()

    def test_restitution_values_by_surface(self):
        """Different surfaces should produce different restitution values"""
        surfaces = [
            (2, 0.2, 0.4),   # concrete ~0.3
            (6, 0.0, 0.3),    # mud ~0.1
            (7, 0.6, 0.9),    # ice ~0.7
            (13, 0.7, 1.0),   # rubber ~0.9
        ]

        for surface_type, min_rest, max_rest in surfaces:
            self.vm.lib.terrain_add_patch(0, 0, 50, surface_type)
            rest = self.vm.lib.material_pairing_get_restitution(128, surface_type)
            self.assertGreater(rest, min_rest, f"Surface {surface_type} restitution too low")
            self.assertLess(rest, max_rest, f"Surface {surface_type} restitution too high")

    def test_friction_values_by_surface(self):
        """Different surfaces should produce different friction values"""
        concrete_fric = self.vm.lib.material_pairing_get_friction(128, 2)  # concrete
        self.assertGreater(concrete_fric, 0.5)

        ice_fric = self.vm.lib.material_pairing_get_friction(128, 7)  # ice
        self.assertLess(ice_fric, 0.3)


class TestMediumTypeClassification(unittest.TestCase):
    """Test medium type classification for surfaces"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.terrain_init()
        self.vm.lib.reset_context()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.close()

    def test_concrete_is_solid(self):
        """Concrete should be solid medium (0)"""
        medium = self.vm.lib.material_pairing_get_medium_type(2)
        self.assertEqual(medium, 0)

    def test_mud_is_soft(self):
        """Mud should be soft medium (1)"""
        medium = self.vm.lib.material_pairing_get_medium_type(6)
        self.assertEqual(medium, 1)

    def test_water_is_liquid(self):
        """Water should be liquid medium (2)"""
        medium = self.vm.lib.material_pairing_get_medium_type(9)
        self.assertEqual(medium, 2)


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

    def test_entity_get_medium_type_invalid_index(self):
        """entity_get_medium_type should return -1 for invalid index"""
        result = self.vm.lib.entity_get_medium_type(99)
        self.assertEqual(result, -1)


class TestScenarioLevelMaterialMediumLifecycle(unittest.TestCase):
    """Scenario-level smoke linking terrain, material response, handles, and lifecycle observation"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.terrain_init()
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.close()

    def test_concrete_mud_water_instances_keep_distinct_medium_and_lifecycle(self):
        self.vm.lib.terrain_add_patch(0, 0, 20, 2)     # concrete -> solid
        self.vm.lib.terrain_add_patch(100, 0, 20, 6)   # mud -> soft
        self.vm.lib.terrain_add_patch(200, 0, 20, 9)   # water -> liquid

        concrete_handle = self.vm.spawn_handle(0, 0, 10, 0)
        mud_handle = self.vm.spawn_handle(0, 100, 10, 0)
        water_handle = self.vm.spawn_handle(0, 200, 10, 0)
        self.assertNotEqual(concrete_handle, 0)
        self.assertNotEqual(mud_handle, 0)
        self.assertNotEqual(water_handle, 0)

        concrete_idx = self.vm.resolve_instance_handle(concrete_handle)
        mud_idx = self.vm.resolve_instance_handle(mud_handle)
        water_idx = self.vm.resolve_instance_handle(water_handle)
        self.assertEqual((concrete_idx, mud_idx, water_idx), (0, 1, 2))

        self.assertEqual(self.vm.lib.entity_get_medium_type(concrete_idx), 0)
        self.assertEqual(self.vm.lib.entity_get_medium_type(mud_idx), 1)
        self.assertEqual(self.vm.lib.entity_get_medium_type(water_idx), 2)

        concrete_damage = self.vm.lib.material_pairing_calculate_impact_damage(10.0, 5.0, 2, 3)
        mud_damage = self.vm.lib.material_pairing_calculate_impact_damage(10.0, 5.0, 6, 3)
        water_buoyancy = self.vm.lib.material_pairing_get_buoyancy(9)
        self.assertGreater(concrete_damage, mud_damage)
        self.assertGreater(water_buoyancy, 0.0)

        self.assertEqual(self.vm.mark_instance_broken_handle(mud_handle), 0)
        self.assertEqual(self.vm.lib.is_instance_broken(mud_idx), 1)
        self.assertEqual(self.vm.compact_broken_instances(), 1)
        self.assertEqual(self.vm.resolve_instance_handle(mud_handle), -1)
        self.assertGreaterEqual(self.vm.resolve_instance_handle(water_handle), 0)
        self.assertEqual(self.vm.instance_count(), 2)


if __name__ == "__main__":
    unittest.main()
