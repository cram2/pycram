import math
import unittest
from copy import deepcopy

import numpy as np

from pycram.datastructures.pose import (
    PoseStamped,
    TransformStamped,
    Quaternion,
    Vector3,
    AxisIdentifier,
)
from pycram.testing import ApartmentWorldTestCase


class TestPose(ApartmentWorldTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.test_body = cls.apartment_world.get_body_by_name("milk.stl")

    def test_pose_creation(self):
        p = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], self.world.root)

        self.assertEqual(p.position.to_list(), [1, 2, 3])
        self.assertEqual(p.orientation.to_list(), [0, 0, 0, 1])
        self.assertEqual(p.frame_id, self.world.root)
        self.assertEqual(p.pose.to_list(), [[1, 2, 3], [0, 0, 0, 1]])

    def test_pose_to_transform(self):
        p = PoseStamped.from_list([3, 2, 1], [0, 0, 1, 0], self.world.root)

        transform = p.to_transform_stamped(
            self.world.get_body_by_name("r_gripper_tool_frame")
        )

        self.assertEqual(
            transform,
            TransformStamped.from_list(
                [3, 2, 1],
                [0, 0, 1, 0],
                self.world.root,
                self.world.get_body_by_name("r_gripper_tool_frame"),
            ),
        )

    def test_pose_edit(self):
        p = PoseStamped.from_list([3, 4, 5], [0, 1, 0, 0], self.world.root)

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
        p1 = PoseStamped.from_list([1, 2, 3], [0, 0, 0, 1], self.world.root)
        p2 = deepcopy(p1)

        self.assertEqual(p1, p2)
        self.assertFalse(p1 is p2)

    def test_transform_creation(self):
        t = TransformStamped.from_list(
            [1, 2, 3], [0, 0, 0, 1], self.world.root, self.test_body
        )

        self.assertEqual(t.translation.to_list(), [1, 2, 3])
        self.assertEqual(t.rotation.to_list(), [0, 0, 0, 1])
        self.assertEqual(t.frame_id, self.world.root)
        self.assertEqual(t.child_frame_id, self.test_body)

    def test_transform_edit(self):
        t = TransformStamped.from_list(
            [3, 2, 1], [0, 1, 0, 0], self.world.root, self.test_body
        )

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
        t = TransformStamped.from_list(
            [1, 1, 1], [0, 0, 0, 1], self.world.root, self.test_body
        )

        t_copy = deepcopy(t)
        self.assertEqual(t, t_copy)
        self.assertFalse(t is t_copy)

    def test_transform_multiplication(self):
        t = TransformStamped.from_list(
            [1, 2, 3], [0, 0, 0, 1], self.world.root, self.test_body
        )
        t2 = TransformStamped.from_list(
            [3, 2, 1],
            [0, 0, 0, 1],
            self.test_body,
            self.world.get_body_by_name("r_gripper_tool_frame"),
        )

        mul_t = t * t2

        self.assertEqual(mul_t.translation.to_list(), [4, 4, 4])

    def test_is_facing_2d_axis(self):
        a = PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1], self.world.root)  # facing +x
        b = PoseStamped.from_list([1, 0, 0], [0, 0, 0, 1], self.world.root)

        facing, angle = a.is_facing_2d_axis(b, axis=AxisIdentifier.X)
        self.assertTrue(facing)
        self.assertAlmostEqual(angle, 0, delta=1e-6)

        # now test Y alignment (should be 90 deg difference)
        facing_y, angle_y = a.is_facing_2d_axis(b, axis=AxisIdentifier.Y)
        self.assertFalse(facing_y)
        self.assertAlmostEqual(abs(angle_y), math.pi / 2, delta=1e-6)

    def test_is_facing_x_or_y(self):
        a = PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1], self.world.root)
        b = PoseStamped.from_list([1, 0, 0], [0, 0, 0, 1], self.world.root)

        self.assertTrue(a.is_facing_x_or_y(b))

        # reverse direction
        b.position.x = -1
        self.assertFalse(a.is_facing_x_or_y(b))

    def test_transform_stamped_multiplication(self):
        t1 = TransformStamped.from_list(
            [1, 2, 3], [0, 0, 0, 1], self.world.root, self.robot_view.root
        )
        t2 = TransformStamped.from_list(
            [4, 5, 6],
            [0, 0, 0, 1],
            self.robot_view.root,
            self.world.get_body_by_name("r_gripper_tool_frame"),
        )

        result = t1 * t2

        self.assertEqual(result.translation.to_list(), [5, 7, 9])
        self.assertEqual(result.frame_id, self.world.root)
        self.assertEqual(
            result.child_frame_id, self.world.get_body_by_name("r_gripper_tool_frame")
        )

    def test_transform_multiplication_inverse(self):
        t1 = TransformStamped.from_list(
            [1, 2, 3], [0, 0, 0, 1], self.world.root, self.robot_view.root
        )
        t2 = TransformStamped.from_list(
            [4, 5, 6],
            [0, 0, 0, 1],
            self.robot_view.root,
            self.world.get_body_by_name("r_gripper_tool_frame"),
        )

        result = t1 * t2
        inverse_result = ~result

        self.assertEqual(inverse_result.translation.to_list(), [-5, -7, -9])
        self.assertEqual(
            inverse_result.frame_id, self.world.get_body_by_name("r_gripper_tool_frame")
        )
        self.assertEqual(inverse_result.child_frame_id, self.world.root)

    def test_transform_multiplication_roration(self):
        t1 = TransformStamped.from_list(
            [1, 1, 1], [0, 0, 1, 1], self.world.root, self.robot_view.root
        )
        t2 = TransformStamped.from_list(
            [1, 0, 0],
            [0, 0, 0, 1],
            self.robot_view.root,
            self.world.get_body_by_name("r_gripper_tool_frame"),
        )

        result = t1 * t2

        self.assertEqual([1, 2, 1], result.translation.to_list())
        self.assertEqual(result.frame_id, self.world.root)
        np.testing.assert_almost_equal(
            result.rotation.to_list(), [0, 0, 0.707, 0.707], decimal=3
        )

    def test_transform_multiplication_translation_inverse(self):
        t1 = TransformStamped.from_list(
            [1, 1, 1], [0, 0, 0, 1], self.world.root, self.robot_view.root
        )
        t2 = TransformStamped.from_list(
            [1, 0, 0],
            [0, 0, 0, 1],
            self.world.root,
            self.world.get_body_by_name("r_gripper_tool_frame"),
        )

        result = ~t1 * t2

        self.assertEqual(result.frame_id, self.robot_view.root)
        np.testing.assert_almost_equal(
            result.rotation.to_list(), [0, 0, 0, 1], decimal=3
        )
        self.assertEqual([0, -1, -1], result.translation.to_list())

    def test_transform_multiplication_with_inverse(self):
        t1 = TransformStamped.from_list(
            [1, 1, 1], [0, 0, 0, 1], self.world.root, self.robot_view.root
        )
        t2 = TransformStamped.from_list(
            [2, 2, 1],
            [0, 0, -1, 1],
            self.world.root,
            self.world.get_body_by_name("r_gripper_tool_frame"),
        )
        result = ~t1 * t2

        self.assertEqual(result.frame_id, self.robot_view.root)
        self.assertEqual(
            result.child_frame_id, self.world.get_body_by_name("r_gripper_tool_frame")
        )
        np.testing.assert_almost_equal(
            result.translation.to_list(), [1, 1, 0], decimal=3
        )
        np.testing.assert_almost_equal(
            result.rotation.to_list(), [0, 0, -0.707, 0.707], decimal=3
        )

    def test_rotation_multiplication(self):
        t1 = TransformStamped.from_list(
            [0, 0, 0], [0, 0, 1, 1], self.world.root, self.robot_view.root
        )
        t2 = TransformStamped.from_list(
            [0, 0, 0],
            [0, 0, 1, 0],
            self.robot_view.root,
            self.world.get_body_by_name("r_gripper_tool_frame"),
        )

        result = t1 * t2
        self.assertEqual(result.frame_id, self.world.root)
        np.testing.assert_almost_equal(
            result.rotation.to_list(), [0, 0, -0.707, 0.707], decimal=3
        )
        self.assertEqual(result.translation.to_list(), [0, 0, 0])
