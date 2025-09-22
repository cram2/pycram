from unittest import TestCase

import numpy as np
from pycram.tf_transformations import quaternion_from_euler

from pycram.datastructures.enums import JointType
from pycram.validation.error_checkers import calculate_angle_between_quaternions, \
    PoseErrorChecker, PositionErrorChecker, OrientationErrorChecker, RevoluteJointPositionErrorChecker, \
    PrismaticJointPositionErrorChecker, MultiJointPositionErrorChecker

from pycram.datastructures.pose import PoseStamped


class TestErrorCheckers(TestCase):
    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    def tearDown(self):
        pass

    def test_calculate_quaternion_error(self):
        quat_1 = [0.0, 0.0, 0.0, 1.0]
        quat_2 = [0.0, 0.0, 0.0, 1.0]
        error = calculate_angle_between_quaternions(quat_1, quat_2)
        self.assertEqual(error, 0.0)
        quat_2 = quaternion_from_euler(0, 0, np.pi/2)
        error = calculate_angle_between_quaternions(quat_1, quat_2)
        self.assertEqual(error, np.pi/2)

    def test_pose_error_checker(self):
        pose_1 = PoseStamped.from_list(None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        pose_2 = PoseStamped.from_list(None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
        error_checker = PoseErrorChecker()
        error = error_checker.calculate_error(pose_1, pose_2)
        self.assertEqual(error, [0.0, 0.0])
        self.assertTrue(error_checker.is_error_acceptable(pose_1, pose_2))
        quat = quaternion_from_euler(0, np.pi/2, 0)
        pose_2 = PoseStamped.from_list(None, [0, 1, np.sqrt(3)], quat)
        error = error_checker.calculate_error(pose_1, pose_2)
        self.assertAlmostEqual(error[0], 2, places=5)
        self.assertEqual(error[1], np.pi/2)
        self.assertFalse(error_checker.is_error_acceptable(pose_1, pose_2))
        quat = quaternion_from_euler(0, 0, np.pi/360)
        pose_2 = PoseStamped.from_list(None, [0, 0.0001, 0.0001], quat)
        self.assertTrue(error_checker.is_error_acceptable(pose_1, pose_2))
        quat = quaternion_from_euler(0, 0, np.pi / 179)
        pose_2 = PoseStamped.from_list(None, [0, 0.0001, 0.0001], quat)
        self.assertFalse(error_checker.is_error_acceptable(pose_1, pose_2))

    def test_position_error_checker(self):
        position_1 = [0.0, 0.0, 0.0]
        position_2 = [0.0, 0.0, 0.0]
        error_checker = PositionErrorChecker()
        error = error_checker.calculate_error(position_1, position_2)
        self.assertEqual(error, 0.0)
        self.assertTrue(error_checker.is_error_acceptable(position_1, position_2))
        position_2 = [1.0, 1.0, 1.0]
        error = error_checker.calculate_error(position_1, position_2)
        self.assertAlmostEqual(error, np.sqrt(3), places=5)
        self.assertFalse(error_checker.is_error_acceptable(position_1, position_2))

    def test_orientation_error_checker(self):
        quat_1 = [0.0, 0.0, 0.0, 1.0]
        quat_2 = [0.0, 0.0, 0.0, 1.0]
        error_checker = OrientationErrorChecker()
        error = error_checker.calculate_error(quat_1, quat_2)
        self.assertEqual(error, 0.0)
        self.assertTrue(error_checker.is_error_acceptable(quat_1, quat_2))
        quat_2 = quaternion_from_euler(0, 0, np.pi/2)
        error = error_checker.calculate_error(quat_1, quat_2)
        self.assertEqual(error, np.pi/2)
        self.assertFalse(error_checker.is_error_acceptable(quat_1, quat_2))

    def test_revolute_joint_position_error_checker(self):
        position_1 = 0.0
        position_2 = 0.0
        error_checker = RevoluteJointPositionErrorChecker()
        error = error_checker.calculate_error(position_1, position_2)
        self.assertEqual(error, 0.0)
        self.assertTrue(error_checker.is_error_acceptable(position_1, position_2))
        position_2 = np.pi/2
        error = error_checker.calculate_error(position_1, position_2)
        self.assertEqual(error, np.pi/2)
        self.assertFalse(error_checker.is_error_acceptable(position_1, position_2))

    def test_prismatic_joint_position_error_checker(self):
        position_1 = 0.0
        position_2 = 0.0
        error_checker = PrismaticJointPositionErrorChecker()
        error = error_checker.calculate_error(position_1, position_2)
        self.assertEqual(error, 0.0)
        self.assertTrue(error_checker.is_error_acceptable(position_1, position_2))
        position_2 = 1.0
        error = error_checker.calculate_error(position_1, position_2)
        self.assertEqual(error, 1.0)
        self.assertFalse(error_checker.is_error_acceptable(position_1, position_2))

    def test_list_of_poses_error_checker(self):
        poses_1 = [PoseStamped.from_list(None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                   PoseStamped.from_list(None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])]
        poses_2 = [PoseStamped.from_list(None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                   PoseStamped.from_list(None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])]
        error_checker = PoseErrorChecker(is_iterable=True)
        error = error_checker.calculate_error(poses_1, poses_2)
        self.assertEqual(error, [[0.0, 0.0], [0.0, 0.0]])
        self.assertTrue(error_checker.is_error_acceptable(poses_1, poses_2))
        quat = quaternion_from_euler(0, np.pi/2, 0)
        poses_2 = [PoseStamped.from_list(None, [0, 1, np.sqrt(3)], quat),
                   PoseStamped.from_list(None, [0, 1, np.sqrt(3)], quat)]
        error = error_checker.calculate_error(poses_1, poses_2)
        self.assertAlmostEqual(error[0][0], 2, places=5)
        self.assertEqual(error[0][1], np.pi/2)
        self.assertAlmostEqual(error[1][0], 2, places=5)
        self.assertEqual(error[1][1], np.pi/2)
        self.assertFalse(error_checker.is_error_acceptable(poses_1, poses_2))

    def test_multi_joint_error_checker(self):
        positions_1 = [0.0, 0.0]
        positions_2 = [np.pi/2, 0.1]
        joint_types = [JointType.REVOLUTE, JointType.PRISMATIC]
        error_checker = MultiJointPositionErrorChecker(joint_types)
        error = error_checker.calculate_error(positions_1, positions_2)
        self.assertEqual(error, [np.pi/2, 0.1])
        self.assertFalse(error_checker.is_error_acceptable(positions_1, positions_2))
        positions_2 = [np.pi/180, 0.0001]
        self.assertTrue(error_checker.is_error_acceptable(positions_1, positions_2))
