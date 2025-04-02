from pycram.local_transformer import LocalTransformer
from pycram.datastructures.pose import PoseStamped, TransformStamped
from pycram.testing import BulletWorldTestCase
from pycram.ros import Time


class TestLocalTransformer(BulletWorldTestCase):

    def test_singelton(self):
        l1 = LocalTransformer()
        l1.set_transform(TransformStamped.from_list().ros_message(), "test")
        l1_transforms = l1.get_all_frames()
        l2 = LocalTransformer()
        l2_transforms = l2.get_all_frames()
        self.assertTrue(l1 is l2)
        self.assertTrue(l1_transforms == l2_transforms)

    def test_transform_pose(self):
        l = LocalTransformer()
        l.set_transform(TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "test_frame").ros_message(), "test")

        p = PoseStamped()
        transformed_pose = l.transform_pose(p, "test_frame")

        self.assertTrue(transformed_pose.position.to_list() == [-1, -1, -1])
        self.assertTrue(transformed_pose.orientation.to_list() == [0, 0, 0, 1])
        self.assertTrue(transformed_pose.frame_id == "test_frame")

    def test_transform_pose_position(self):
        l = LocalTransformer()
        l.set_transform(TransformStamped.from_list([1, 1, 1], [0, 0, 0, 1], "map", "test_frame").ros_message(), "test")

        p = PoseStamped()
        transformed_pose = l.transform_pose(p, "test_frame")

        self.assertTrue(transformed_pose.position == transformed_pose.pose.position)

    def test_update_for_object(self):
        l = LocalTransformer()
        self.milk.set_pose(PoseStamped.from_list([1, 2, 1]))
        self.milk.update_link_transforms()
        self.assertTrue(l.can_transform("map", self.milk.tf_frame, Time(0)))


