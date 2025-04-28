import unittest

import numpy as np

from pycram.local_transformer import LocalTransformer
from pycram.datastructures.pose import PoseStamped, TransformStamped
from pycram.testing import BulletWorldTestCase
from pycram.ros import Time


class TestLocalTransformer(BulletWorldTestCase):

    def test_singelton(self):
        l1 = LocalTransformer()
        l1.update_transforms([TransformStamped()])
        l1_transforms = l1.get_all_frames()
        l2 = LocalTransformer()
        l2_transforms = l2.get_all_frames()
        self.assertTrue(l1 is l2)
        self.assertTrue(l1_transforms == l2_transforms)

    def test_transform_pose(self):
        l = LocalTransformer()
        l.update_transforms([TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "test_frame")])

        p = PoseStamped()
        transformed_pose = l.transform_pose(p, "test_frame")

        self.assertEqual([-1, -1, -1], transformed_pose.position.to_list())
        self.assertEqual([0, 0, 0, 1], transformed_pose.orientation.to_list())
        self.assertEqual("test_frame", transformed_pose.header.frame_id)

    def test_transform_pose_position(self):
        l = LocalTransformer()
        l.update_transforms([TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "test_frame")])

        p = PoseStamped()
        transformed_pose = l.transform_pose(p, "test_frame")

        self.assertTrue(transformed_pose.position == transformed_pose.pose.position)

    def test_transform_pose_rotation(self):
        l = LocalTransformer()
        l.update_transforms([TransformStamped.from_list([1, 0, 0], [0, 0, 1, 1], "map", "test_frame")])
        test_pose = PoseStamped.from_list([1, 0, 0], [0, 0, 0, 1], frame="test_frame")
        transformed_pose = l.transform_pose(test_pose, "map")
        for expect, actual in zip([1,1,0], transformed_pose.position.to_list()):
            self.assertAlmostEqual(expect, actual)

    @unittest.skip
    def test_update_for_object(self):
        l = LocalTransformer()
        self.milk.set_pose(PoseStamped.from_list([1, 2, 1]))
        self.milk.update_link_transforms()
        self.assertTrue(l.can_transform("map", self.milk.tf_frame, Time(0)))

    def test_transform_object_frame(self):
        p = PoseStamped()
        self.milk.set_pose(PoseStamped.from_list([1, 2, 1]))
        lt = LocalTransformer()
        new_pose = lt.transform_to_object_frame(p, self.milk)

        self.assertEqual([-1, -2, -1], new_pose.position.to_list())

    def test_link_wrt(self):
        relative = self.robot.links["torso_lift_link"].get_pose_wrt_link(self.robot.links["torso_lift_link"])
        self.assertEqual([0, 0, 0], relative.position.to_list())
        self.assertEqual([0, 0, 0, 1], relative.orientation.to_list())

        relative = self.robot.links["torso_lift_link"].get_pose_wrt_link(self.robot.links["base_link"])
        for excepted, actual in zip([-0.05, 0, 0.73967], relative.position.to_list()):
            self.assertAlmostEqual(excepted, actual, 4)
        self.assertEqual([0, 0, 0, 1], relative.orientation.to_list())

    def test_transform_to_base(self):
        pose = PoseStamped.from_list([1, 2, 1])
        lt = LocalTransformer()
        base_pose = lt.transform_pose(pose, self.robot.tf_frame)

        self.assertEqual([1,2,1], base_pose.position.to_list())
        self.assertEqual([0, 0, 0, 1], base_pose.orientation.to_list())
        self.assertEqual(self.robot.tf_frame, base_pose.header.frame_id)

    def test_transform_to_base_movement(self):
        pose = PoseStamped.from_list([1, 2, 1])
        lt = LocalTransformer()
        base_pose = lt.transform_pose(pose, self.robot.tf_frame)

        self.assertEqual([1, 2, 1], base_pose.position.to_list())
        self.assertEqual([0, 0, 0, 1], base_pose.orientation.to_list())
        self.assertEqual(self.robot.tf_frame, base_pose.header.frame_id)

        self.robot.pose = PoseStamped.from_list([1, 2, 1])
        base_pose = lt.transform_pose(pose, self.robot.tf_frame)

        self.assertEqual([0, 0, 0], base_pose.position.to_list())
        self.assertEqual([0, 0, 0, 1], base_pose.pose.orientation.to_list())

    def test_transform_base_movement(self):
        pose = PoseStamped.from_list([0, 0, 0])
        lt = LocalTransformer()
        gripper_pose = lt.transform_pose(pose, self.robot.links["l_gripper_tool_frame"].tf_frame)
        np.testing.assert_almost_equal([-0.951, -0.188, -0.7906], gripper_pose.position.to_list(), decimal=3)
        self.assertEqual([0, 0, 0, 1], gripper_pose.orientation.to_list())
        self.assertEqual(self.robot.links["l_gripper_tool_frame"].tf_frame, gripper_pose.header.frame_id)

        self.robot.pose = PoseStamped.from_list([1, 2, 1])
        gripper_pose = lt.transform_pose(pose, self.robot.links["l_gripper_tool_frame"].tf_frame)
        np.testing.assert_almost_equal([-1.951, -2.188, -1.7906], gripper_pose.position.to_list(), decimal=4)
        self.assertEqual([0, 0, 0, 1], gripper_pose.orientation.to_list())
        self.assertEqual(self.robot.links["l_gripper_tool_frame"].tf_frame, gripper_pose.header.frame_id)

    def test_tcp_transform(self):
        tcp_pose = PoseStamped.from_list(frame=self.robot.links["l_gripper_tool_frame"].tf_frame)
        lt = LocalTransformer()
        base_pose = lt.transform_pose(tcp_pose, self.robot.tf_frame)

        np.testing.assert_almost_equal([0.951, 0.188, 0.79067], base_pose.position.to_list(), decimal=5)
        self.assertEqual([0, 0, 0, 1], base_pose.orientation.to_list())
        self.assertEqual(self.robot.tf_frame, base_pose.header.frame_id)

    def test_tcp_transform_movement(self):
        tcp_pose = PoseStamped.from_list([0, 0, 0], frame=self.robot.links["l_gripper_tool_frame"].tf_frame)
        lt = LocalTransformer()
        base_pose = lt.transform_pose(tcp_pose,  self.robot.tf_frame)

        np.testing.assert_almost_equal([0.951, 0.188, 0.79067], base_pose.position.to_list(), decimal=5)
        self.assertEqual([0, 0, 0, 1], base_pose.orientation.to_list())
        self.assertEqual(self.robot.tf_frame, base_pose.header.frame_id)

        self.robot.pose = PoseStamped.from_list([1, 2, 1])
        base_pose = lt.transform_pose(tcp_pose, self.robot.tf_frame)
        np.testing.assert_almost_equal([0.951, 0.188, 0.79067], base_pose.position.to_list(), decimal=5)
        self.assertEqual([0, 0, 0, 1], base_pose.orientation.to_list())
        self.assertEqual(self.robot.tf_frame, base_pose.header.frame_id)

    def test_tcp_transform_rotation_movement(self):
        tcp_pose = PoseStamped.from_list([0, 0, 0], frame=self.robot.links["l_gripper_tool_frame"].tf_frame)
        lt = LocalTransformer()
        base_pose = lt.transform_pose(tcp_pose, self.robot.tf_frame)

        np.testing.assert_almost_equal([0.951, 0.188, 0.79067], base_pose.position.to_list(), decimal=5)
        self.assertEqual([0, 0, 0, 1], base_pose.orientation.to_list())
        self.assertEqual(self.robot.tf_frame, base_pose.header.frame_id)

        self.robot.pose = PoseStamped.from_list([1, 2, 1], [0, 0, 1, 1])

        robot_base_pose = lt.transform_pose(self.robot.pose, self.robot.tf_frame)
        np.testing.assert_almost_equal(robot_base_pose.position.to_list(), [0,0, 0])
        np.testing.assert_almost_equal(robot_base_pose.orientation.to_list(), [0,0, 0, 1])

        base_pose = lt.transform_pose(tcp_pose, self.robot.tf_frame)
        np.testing.assert_almost_equal([0.951, 0.188, 0.79067], base_pose.position.to_list(), decimal=5)
        self.assertEqual([0, 0, 0, 1], base_pose.orientation.to_list())
        self.assertEqual(self.robot.tf_frame, base_pose.header.frame_id)