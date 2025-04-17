import unittest

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
        # self.assertEqual([1,1,0], transformed_pose.position.to_list())
        for expect, actual in zip([1,1,0], transformed_pose.position.to_list()):
            self.assertAlmostEqual(expect, actual)

    @unittest.skip
    def test_update_for_object(self):
        l = LocalTransformer()
        self.milk.set_pose(PoseStamped.from_list([1, 2, 1]))
        self.milk.update_link_transforms()
        self.assertTrue(l.can_transform("map", self.milk.tf_frame, Time(0)))


