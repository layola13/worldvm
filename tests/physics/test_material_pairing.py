"""
Test Material Pairing System - Contact Material x Material Response
Tests for the material pairing system that was previously missing per todo/1.md gap assessment
"""
import unittest
import sys
import os
import ctypes

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from worldvm import WorldVM


class TestMaterialPairing(unittest.TestCase):
    """Test material pairing system - verifies surface type response"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.terrain_init()
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()

    def tearDown(self):
        self.vm.lib.reset_context()
        self.vm.lib.clear_joints()
        self.vm.close()

    def test_concrete_restitution(self):
        """Concrete surface should have correct restitution (~0.3)"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 2)  # concrete
        restitution = self.vm.lib.material_pairing_get_restitution(128, 2)  # entity rest 0.5
        self.assertGreater(restitution, 0.2)
        self.assertLess(restitution, 0.4)

    def test_water_buoyancy(self):
        """Water surface should have buoyancy = 1.0"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 9)  # water
        buoyancy = self.vm.lib.material_pairing_get_buoyancy(9)
        self.assertEqual(buoyancy, 1.0)

    def test_mud_friction(self):
        """Mud surface should have high friction"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 6)  # mud
        friction = self.vm.lib.material_pairing_get_friction(128, 6)
        self.assertGreater(friction, 0.8)

    def test_ice_restitution(self):
        """Ice should have high restitution (slippery)"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 7)  # ice
        restitution = self.vm.lib.material_pairing_get_restitution(128, 7)
        self.assertGreater(restitution, 0.6)

    def test_medium_type_solid(self):
        """Concrete should be solid medium"""
        medium = self.vm.lib.material_pairing_get_medium_type(2)  # concrete
        self.assertEqual(medium, 0)  # solid = 0

    def test_medium_type_soft(self):
        """Mud should be soft medium"""
        medium = self.vm.lib.material_pairing_get_medium_type(6)  # mud
        self.assertEqual(medium, 1)  # soft = 1

    def test_medium_type_liquid(self):
        """Water should be liquid medium"""
        medium = self.vm.lib.material_pairing_get_medium_type(9)  # water
        self.assertEqual(medium, 2)  # liquid = 2

    def test_is_hard_surface_concrete(self):
        """Concrete should be hard surface"""
        result = self.vm.lib.material_pairing_is_hard_surface(2)
        self.assertEqual(result, 1)

    def test_is_hard_surface_mud(self):
        """Mud should NOT be hard surface"""
        result = self.vm.lib.material_pairing_is_hard_surface(6)  # mud
        self.assertEqual(result, 0)

    def test_cloth_surface(self):
        """Cloth surface should be soft with high friction"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 12)  # cloth
        medium = self.vm.lib.material_pairing_get_medium_type(12)
        self.assertEqual(medium, 1)  # soft
        friction = self.vm.lib.material_pairing_get_friction(128, 12)
        self.assertGreater(friction, 0.7)

    def test_rubber_surface(self):
        """Rubber surface should be solid with high restitution"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 13)  # rubber
        medium = self.vm.lib.material_pairing_get_medium_type(13)
        self.assertEqual(medium, 0)  # solid
        restitution = self.vm.lib.material_pairing_get_restitution(128, 13)
        self.assertGreater(restitution, 0.7)  # High restitution due to geometric mean

    def test_carpet_surface(self):
        """Carpet should be soft with high friction"""
        self.vm.lib.terrain_add_patch(0, 0, 100, 15)  # carpet
        medium = self.vm.lib.material_pairing_get_medium_type(15)
        self.assertEqual(medium, 1)  # soft
        friction = self.vm.lib.material_pairing_get_friction(128, 15)
        self.assertGreater(friction, 0.7)


class TestMediumTypeSystem(unittest.TestCase):
    """Test medium type system for surface classification"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.terrain_init()

    def tearDown(self):
        self.vm.close()

    def test_all_surface_types_exposed(self):
        """All surface types should be accessible via FFI"""
        surface_types = range(16)  # 0-15
        for surf_type in surface_types:
            medium = self.vm.lib.material_pairing_get_medium_type(surf_type)
            self.assertIn(medium, [0, 1, 2, 3, 4])  # solid, soft, liquid, vapor, plasma


class TestImpactDamage(unittest.TestCase):
    """Test impact damage calculation with material pairing"""

    def setUp(self):
        self.vm = WorldVM()
        self.vm.lib.terrain_init()

    def tearDown(self):
        self.vm.close()

    def test_impact_damage_calculation(self):
        """Impact damage should use material pairing response"""
        # Glass (fragile) hitting concrete
        damage = self.vm.lib.material_pairing_calculate_impact_damage(
            10.0, 5.0, 2, 3)  # 10 m/s, 5kg, concrete, fragile material
        self.assertGreater(damage, 0)

    def test_fragile_high_damage(self):
        """Fragile materials should have higher damage modifier"""
        damage_fragile = self.vm.lib.material_pairing_calculate_impact_damage(
            10.0, 5.0, 2, 3)  # fragile
        damage_solid = self.vm.lib.material_pairing_calculate_impact_damage(
            10.0, 5.0, 2, 0)  # solid
        self.assertGreater(damage_fragile, damage_solid)


if __name__ == "__main__":
    unittest.main()
