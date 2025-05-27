import math
import unittest

from pycram.datastructures.pose import PoseStamped, TransformStamped, Quaternion, Vector3, AxisIdentifier


class TestPose(unittest.TestCase):

    def test_pose_creation(self):
        p = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], "test")

        self.assertEqual(p.position.to_list(), [1, 2, 3])
        self.assertEqual(p.orientation.to_list(), [0, 0, 0, 1])
        self.assertEqual(p.frame_id, "test")
        self.assertEqual(p.pose.to_list(), [[1, 2, 3], [0, 0, 0, 1]])

    def test_pose_to_transform(self):
        p = PoseStamped.from_list([3, 2, 1], [0, 0, 1, 0], "map")

        transform = p.to_transform_stamped("test_frame")

        self.assertEqual(transform, TransformStamped.from_list([3, 2, 1], [0, 0, 1, 0], "map", "test_frame"))

    def test_pose_edit(self):
        p = PoseStamped.from_list([3, 4, 5], [0, 1, 0, 0], "map")

        p.position = Vector3(1, 1, 1)
        self.assertEqual(p.position.to_list(), [1, 1, 1])
        p.position.x = 2
        self.assertEqual(p.position.to_list(), [2, 1, 1])
        p.position = Vector3(3, 3, 3)
        self.assertEqual(p.position.to_list(), [3, 3, 3])

        p.orientation = Quaternion(0, 0, 0, 1)
        self.assertEqual(p.orientation.to_list(), [0, 0, 0, 1])
        p.orientation.x = 1
        self.assertEqual(p.orientation.to_list(), [1, 0, 0, 1])
        p.orientation = Quaternion(0, 0, 1, 0)
        self.assertEqual(p.orientation.to_list(), [0, 0, 1, 0])

    def test_pose_copy(self):
        p1 = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], "map")
        p2 = p1.copy()

        self.assertEqual(p1, p2)
        self.assertFalse(p1 is p2)

    def test_transform_creation(self):
        t = TransformStamped.from_list([1, 2, 3], [0, 0, 0, 1], "map", "test_frame")

        self.assertEqual(t.translation.to_list(), [1, 2, 3])
        self.assertEqual(t.rotation.to_list(), [0, 0, 0, 1])
        self.assertEqual(t.frame_id, "map")
        self.assertEqual(t.child_frame_id, "test_frame")

    def test_transform_edit(self):
        t = TransformStamped.from_list([3, 2, 1], [0, 1, 0, 0], "map", "test_frame")

        t.translation = Vector3(2, 2, 2)
        self.assertEqual(t.translation.to_list(), [2, 2, 2])
        t.translation.x = 3
        self.assertEqual(t.translation.to_list(), [3, 2, 2])
        t.translation = Vector3(1, 1, 1)
        self.assertEqual(t.translation.to_list(), [1, 1, 1])

        t.rotation = Quaternion(1, 0, 0, 0)
        self.assertEqual(t.rotation.to_list(), [1, 0, 0, 0])
        t.rotation.y = 1
        self.assertEqual(t.rotation.to_list(), [1, 1, 0, 0])
        t.rotation = Quaternion(0, 0, 0, 1)
        self.assertEqual(t.rotation.to_list(), [0, 0, 0, 1])

    def test_transform_copy(self):
        t = TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "test_frame")

        t_copy = t.copy()
        self.assertEqual(t, t_copy)
        self.assertFalse(t is t_copy)

    def test_transform_multiplication(self):
        t = TransformStamped.from_list([1, 2, 3], [0, 0, 0, 1], "map", "test_frame")
        t2 = TransformStamped.from_list([3, 2, 1], [0, 0, 0, 1], "test_frame", "final_frame")

        mul_t = t * t2

        self.assertEqual(mul_t.translation.to_list(), [4, 4, 4])

    def test_is_facing_2d_axis(self):
        a = PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1], "map")  # facing +x
        b = PoseStamped.from_list([1, 0, 0], [0, 0, 0, 1], "map")

        facing, angle = a.is_facing_2d_axis(b, axis=AxisIdentifier.X)
        self.assertTrue(facing)
        self.assertAlmostEqual(angle, 0, delta=1e-6)

        # now test Y alignment (should be 90 deg difference)
        facing_y, angle_y = a.is_facing_2d_axis(b, axis=AxisIdentifier.Y)
        self.assertFalse(facing_y)
        self.assertAlmostEqual(abs(angle_y), math.pi / 2, delta=1e-6)

    def test_is_facing_x_or_y(self):
        a = PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1], "map")
        b = PoseStamped.from_list([1, 0, 0], [0, 0, 0, 1], "map")

        self.assertTrue(a.is_facing_x_or_y(b))

        # reverse direction
        b.position.x = -1
        self.assertFalse(a.is_facing_x_or_y(b))