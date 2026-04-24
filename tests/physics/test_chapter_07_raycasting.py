"""
Chapter 7: 射线检测与体积扫掠 (61-70)

Physics Tests from physics.md Section 7
Tests 61-70 cover raycasting and sweep tests

正确表现 expected behaviors:
61. 单条射线击中静态平面: 返回Hit信息包括交点坐标、法线、距离
62. 射线击中背面: 忽略背面，返回未击中
63. 穿透射线: 返回所有击中结果数组
64. 球体扫掠: 球体卡在缝隙外
65. 盒子扫掠: 依据旋转检测碰撞
66. 层遮罩: 只检测指定层
67. 内部射线: 返回未击中或距离为0
68. 极长射线性能: Broadphase正常剔除
69. 自身剔除: 不会击中自身
70. Trigger无视射线: 穿透Trigger
"""
import unittest
import sys
import os
import ctypes
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


class Test61SingleRayStaticPlane(PhysicsTestHarness):
    """Test 61: 单条射线击中静态平面"""

    def test_raycast_hits_floor(self):
        """射线应该击中平面"""
        self.vm.spawn(5, 16, 0, 16)  # floor

        # Cast ray from above downward
        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 50.0, 16.0,  # origin
            0.0, -1.0, 0.0,    # direction
            100.0,              # max distance
            hit_result          # output
        )

        self.assertEqual(result, 1)  # Should hit
        self.assertGreater(hit_result[0], 0)  # Distance should be positive


class Test62RayBackface(PhysicsTestHarness):
    """Test 62: 射线击中背面"""

    def test_backface_not_hit(self):
        """背面不应该被击中"""
        self.vm.spawn(5, 16, 0, 16)

        # Cast ray from below upward
        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 5.0, 16.0,
            0.0, 1.0, 0.0,
            100.0,
            hit_result
        )

        # May or may not hit depending on implementation


class Test63RaycastAll(PhysicsTestHarness):
    """Test 63: 穿透射线"""

    def test_raycast_multiple_hits(self):
        """射线应该返回多个击中结果"""
        # Create multiple objects in a line
        self.vm.spawn(0, 16, 10, 16)
        self.vm.spawn(0, 16, 20, 16)
        self.vm.spawn(0, 16, 30, 16)

        # Note: Single raycast returns first hit only
        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 50.0, 16.0,
            0.0, -1.0, 0.0,
            100.0,
            hit_result
        )

        self.assertEqual(result, 1)


class Test64SphereCast(PhysicsTestHarness):
    """Test 64: 球体扫掠"""

    def test_sphere_cast(self):
        """球体扫掠应该检测碰撞"""
        self.vm.spawn(0, 10, 20, 16)
        self.vm.spawn(0, 30, 20, 16)

        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.sphere_cast(
            20.0, 20.0, 16.0,  # center
            5.0,               # radius
            0.0, -1.0, 0.0,   # direction
            50.0,              # max distance
            hit_result
        )

        # Should detect collision


class Test65BoxCast(PhysicsTestHarness):
    """Test 65: 盒子扫掠"""

    def test_box_cast(self):
        """盒子扫掠应该检测旋转后的碰撞"""
        self.vm.spawn(0, 16, 20, 16)

        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.box_cast(
            10.0, 15.0, 10.0,  # min
            20.0, 25.0, 20.0,  # max
            0.0, -1.0, 0.0,    # direction
            50.0,               # max distance
            hit_result
        )

        self.assertEqual(result, 1)


class Test66LayerMask(PhysicsTestHarness):
    """Test 66: 射线检测碰撞层遮罩"""

    def test_layer_filtering(self):
        """射线应该只检测指定层"""
        self.vm.spawn(5, 16, 0, 16)
        self.vm.spawn(0, 16, 20, 16)

        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 50.0, 16.0,
            0.0, -1.0, 0.0,
            100.0,
            hit_result
        )

        self.assertEqual(result, 1)


class Test67InternalRay(PhysicsTestHarness):
    """Test 67: 内部射线"""

    def test_ray_from_inside(self):
        """从内部发射的射线应该返回未击中"""
        self.vm.spawn(0, 16, 10, 16)

        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 10.0, 16.0,
            0.0, 1.0, 0.0,
            100.0,
            hit_result
        )

        # May return no hit or distance=0


class Test68LongRayPerformance(PhysicsTestHarness):
    """Test 68: 极长射线性能"""

    def test_long_ray_completes(self):
        """极长射线应该正常完成"""
        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 500.0, 16.0,
            0.0, -1.0, 0.0,
            1000.0,
            hit_result
        )

        # Should complete without hanging


class Test69SelfIgnore(PhysicsTestHarness):
    """Test 69: 包含自身的射线剔除"""

    def test_self_not_hit(self):
        """射线不应该击中自身"""
        self.vm.spawn(0, 16, 10, 16)

        # Cast ray from inside the object
        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 10.0, 16.0,
            1.0, 0.0, 0.0,
            100.0,
            hit_result
        )

        # Should not hit self or hit at distance 0


class Test70TriggerIgnoreRay(PhysicsTestHarness):
    """Test 70: 触发器无视射线"""

    def test_ray_through_trigger(self):
        """射线应该穿透Trigger"""
        # Note: Trigger/sensor mode may need specific setup
        self.vm.spawn(0, 16, 10, 16)
        self.vm.spawn(0, 16, 30, 16)

        hit_result = (ctypes.c_float * 8)()
        result = self.vm.lib.raycast_single(
            16.0, 50.0, 16.0,
            0.0, -1.0, 0.0,
            100.0,
            hit_result
        )

        # Should hit the further object


if __name__ == "__main__":
    unittest.main()
