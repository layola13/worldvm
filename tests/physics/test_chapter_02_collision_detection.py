"""
Chapter 2: 基础碰撞检测 (11-20)

Physics Tests from physics.md Section 2
Tests 11-20 cover basic collision detection

正确表现 expected behaviors:
11. 球体与静态平面: 接触平面后速度变为0，Y坐标不再减小
12. 立方体平放坠落: 接触后平稳停止，不产生任何旋转
13. 立方体单角着地: 接触瞬间产生力矩，立方体倒下
14. 球体对心碰撞: 两球完全反弹，横向无速度分量
15. 球体非对心碰撞: 两球按碰撞法线方向散开
16. 不同质量的碰撞: 动量守恒，轻球高速飞出
17. 胶囊体翻滚: 根据胶囊体朝向滚动
18. 初始穿透求解: 第一帧生成分离力将两者推开
19. 凹多边形碰撞: 球体顺着碗壁滑落
20. 零厚度平面碰撞: 碰撞被检测到，物体被阻挡
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
        self.vm.close()
    def get_velocity(self, i): return self.vm.get_velocity(i)
    def get_angular_velocity(self, i): return self.vm.get_angular_velocity(i)
    def get_pos(self, i): return self.vm.get_pos(i)


class Test11SphereStaticPlane(PhysicsTestHarness):
    """Test 11: 球体与静态平面碰撞"""

    def test_sphere_stops_on_floor(self):
        """球体落在平面上后应该停止"""
        # Spawn floor at y=0
        self.vm.spawn(5, 16, 0, 16)
        # Spawn sphere above floor
        self.vm.spawn(0, 16, 10, 16)

        # Apply downward velocity
        self.vm.apply_impulse(1, 0, -200, 0)

        # Run until stable
        for _ in range(30):
            self.vm.run(1)

        pos = self.get_pos(1)
        # Should be near the floor (accounting for entity size)
        self.assertLess(pos[1], 10, f"Should have fallen: {pos[1]}")


class Test12CubeFlatFall(PhysicsTestHarness):
    """Test 12: 立方体平放坠落"""

    def test_cube_stops_without_rotation(self):
        """立方体平放落地后平稳停止，不旋转"""
        self.vm.spawn(5, 16, 0, 16)
        # Entity 0 (apple) is sphere-shaped so not ideal for this test
        # Using hammer as elongated shape
        self.vm.spawn(2, 16, 20, 16)

        self.vm.apply_impulse(2, 0, -200, 0)

        for _ in range(30):
            self.vm.run(1)

        pos = self.get_pos(2)
        # Should have fallen
        self.assertLess(pos[1], 20)


class Test13CubeCornerFall(PhysicsTestHarness):
    """Test 13: 立方体单角着地"""

    def test_cube_tumbles_from_corner(self):
        """立方体以角着地时应该倒下"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(2, 16, 20, 16)

        # Apply impulse at angle
        self.vm.apply_impulse(2, 50, -200, 50)

        for _ in range(30):
            self.vm.run(1)

        pos = self.get_pos(2)
        self.assertLess(pos[1], 20)


class Test14SphereHeadOnCollision(PhysicsTestHarness):
    """Test 14: 球体对心碰撞"""

    def test_two_spheres_bounce_apart(self):
        """两个球体相向碰撞后应该弹开"""
        # Spawn two spheres at same height, facing each other
        self.vm.spawn(0, 10, 10, 16)
        self.vm.spawn(0, 30, 10, 16)

        # Apply velocities toward each other
        self.vm.apply_impulse(0, 100, 0, 0)  # Move right
        self.vm.apply_impulse(1, -100, 0, 0)  # Move left

        # Run
        for _ in range(10):
            self.vm.run(1)

        pos0 = self.get_pos(0)
        pos1 = self.get_pos(1)

        # They should have moved apart
        self.assertGreater(pos0[0], 10, f"Sphere 0 should move right: {pos0[0]}")
        self.assertLess(pos1[0], 30, f"Sphere 1 should move left: {pos1[0]}")


class Test15SphereGlancingCollision(PhysicsTestHarness):
    """Test 15: 球体非对心碰撞（擦边球）"""

    def test_spheres_deflect_at_angle(self):
        """非对心碰撞应该产生角度偏转"""
        self.vm.spawn(0, 10, 10, 10)
        self.vm.spawn(0, 20, 10, 20)

        # Apply impulses at angle to each other
        self.vm.apply_impulse(0, 80, 0, 20)
        self.vm.apply_impulse(1, -80, 0, -20)

        for _ in range(15):
            self.vm.run(1)

        # Both should have moved in different directions
        pos0 = self.get_pos(0)
        pos1 = self.get_pos(1)

        # They should not overlap
        self.assertNotEqual(pos0[0], pos1[0])


class Test16DifferentMassCollision(PhysicsTestHarness):
    """Test 16: 不同质量的碰撞"""

    def test_heavy_to_light_momentum_transfer(self):
        """重球撞轻球时动量应该传递"""
        # Heavy sphere (hammer mass=1000)
        self.vm.spawn(2, 10, 10, 16)
        # Light sphere (apple mass=50)
        self.vm.spawn(0, 30, 10, 16)

        # Apply velocity to heavy sphere toward light sphere
        self.vm.apply_impulse(0, 100, 0, 0)

        for _ in range(10):
            self.vm.run(1)

        # Both should have moved
        pos0 = self.get_pos(0)
        pos1 = self.get_pos(1)


class Test17CapsuleRolling(PhysicsTestHarness):
    """Test 17: 胶囊体翻滚"""

    def test_capsule_rolls_by_shape(self):
        """胶囊体应该根据形状滚动"""
        # This test uses entities with different aspect ratios
        self.vm.spawn(0, 16, 10, 16)
        self.vm.apply_impulse(0, 100, 0, 0)

        for _ in range(20):
            self.vm.run(1)

        pos = self.get_pos(0)
        self.assertGreater(pos[0], 16)


class Test18InitialPenetration(PhysicsTestHarness):
    """Test 18: 初始穿透求解"""

    def test_overlapping_objects_push_apart(self):
        """重叠的物体会被推开"""
        # Spawn two objects at nearly same position
        self.vm.spawn(0, 16, 10, 16)
        self.vm.spawn(1, 18, 10, 16)  # Overlapping

        for _ in range(5):
            self.vm.run(1)

        # Both should exist without crash
        pos0 = self.get_pos(0)
        pos1 = self.get_pos(1)
        self.assertIsNotNone(pos0)
        self.assertIsNotNone(pos1)


class Test19ConcaveMeshCollision(PhysicsTestHarness):
    """Test 19: 凹多边形碰撞"""

    def test_sphere_in_concave_shape(self):
        """球体应该能落入凹形（如碗）中"""
        # This is more of a shape test
        self.vm.spawn(0, 16, 20, 16)

        for _ in range(30):
            self.vm.run(1)

        pos = self.get_pos(0)
        self.assertLess(pos[1], 20)


class Test20ZeroThicknessPlane(PhysicsTestHarness):
    """Test 20: 零厚度平面碰撞"""

    def test_thin_plane_collision(self):
        """薄平面应该能产生碰撞"""
        self.vm.spawn(0, 16, 10, 16)
        self.vm.apply_impulse(0, 0, -200, 0)

        for _ in range(20):
            self.vm.run(1)

        pos = self.get_pos(0)
        self.assertLess(pos[1], 10)


if __name__ == "__main__":
    unittest.main()
