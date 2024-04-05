import unittest

from pycram.datastructures.pose import Pose, Transform


class TestPose(unittest.TestCase):

    def test_pose_creation(self):
        p = Pose([1, 2, 3], [0, 0, 0, 1], "test")

        self.assertEqual(p.position_as_list(), [1, 2, 3])
        self.assertEqual(p.orientation_as_list(), [0, 0, 0, 1])
        self.assertEqual(p.frame, "test")
        self.assertEqual(p.to_list(), [[1, 2, 3], [0, 0, 0, 1]])

    def test_pose_to_transform(self):
        p = Pose([3, 2, 1], [0, 0, 1, 0], "map")

        transform = p.to_transform("test_frame")

        self.assertEqual(transform, Transform([3, 2, 1], [0, 0, 1, 0], "map", "test_frame"))

    def test_pose_edit(self):
        p = Pose([3, 4, 5], [0, 1, 0, 0], "map")

        p.position = [1, 1, 1]
        self.assertEqual(p.position_as_list(), [1, 1, 1])
        p.position.x = 2
        self.assertEqual(p.position_as_list(), [2, 1, 1])
        p.set_position([3, 3, 3])
        self.assertEqual(p.position_as_list(), [3, 3, 3])

        p.orientation = [0, 0, 0, 1]
        self.assertEqual(p.orientation_as_list(), [0, 0, 0, 1])
        p.orientation.x = 1
        self.assertEqual(p.orientation_as_list(), [1, 0, 0, 1])
        p.set_orientation([0, 0, 1, 0])
        self.assertEqual(p.orientation_as_list(), [0, 0, 1, 0])

    def test_pose_copy(self):
        p1 = Pose([1, 2, 3], [0, 0, 0, 1], "map")
        p2 = p1.copy()

        self.assertEqual(p1, p2)
        self.assertFalse(p1 is p2)

    def test_transform_creation(self):
        t = Transform([1, 2, 3], [0, 0, 0, 1], "map", "test_frame")

        self.assertEqual(t.translation_as_list(), [1, 2, 3])
        self.assertEqual(t.rotation_as_list(), [0, 0, 0, 1])
        self.assertEqual(t.frame, "map")
        self.assertEqual(t.child_frame_id, "test_frame")

    def test_transform_edit(self):
        t = Transform([3, 2, 1], [0, 1, 0, 0], "map", "test_frame")

        t.translation = [2, 2, 2]
        self.assertEqual(t.translation_as_list(), [2, 2, 2])
        t.translation.x = 3
        self.assertEqual(t.translation_as_list(), [3, 2, 2])
        t.set_translation([1, 1, 1])
        self.assertEqual(t.translation_as_list(), [1, 1, 1])

        t.rotation = [1, 0, 0, 0]
        self.assertEqual(t.rotation_as_list(), [1, 0, 0, 0])
        t.rotation.y = 1
        self.assertEqual(t.rotation_as_list(), [1, 1, 0, 0])
        t.set_rotation([0, 0, 0, 1])
        self.assertEqual(t.rotation_as_list(), [0, 0, 0, 1])

    def test_transform_copy(self):
        t = Transform([1, 1, 1], [0, 0, 0, 1], "map", "test_frame")

        t_copy = t.copy()
        self.assertEqual(t, t_copy)
        self.assertFalse(t is t_copy)
