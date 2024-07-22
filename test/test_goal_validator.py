from tf.transformations import quaternion_from_euler

from bullet_world_testcase import BulletWorldTestCase
import numpy as np

from pycram.datastructures.enums import JointType
from pycram.robot_description import RobotDescription
from pycram.worlds.multiverse_functions.goal_validator import GoalValidator
from pycram.worlds.multiverse_functions.error_checkers import PoseErrorChecker, PositionErrorChecker, \
    OrientationErrorChecker, RevoluteJointPositionErrorChecker, PrismaticJointPositionErrorChecker, \
    MultiJointPositionErrorChecker

from pycram.datastructures.pose import Pose


class TestGoalValidator(BulletWorldTestCase):

    def test_single_pose_goal(self):
        milk_goal_pose = Pose([1.3, 1.5, 0.9])
        goal_validator = GoalValidator(milk_goal_pose, self.milk.get_pose, PoseErrorChecker())
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertAlmostEqual(goal_validator.current_error[0], 0.5, places=5)
        self.assertAlmostEqual(goal_validator.current_error[1], 0, places=5)
        self.milk.set_pose(milk_goal_pose)
        self.assertEqual(self.milk.get_pose(), milk_goal_pose)
        self.assertTrue(goal_validator.goal_achieved)
        print(goal_validator.current_error)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 1)
        self.assertAlmostEqual(goal_validator.current_error[0], 0, places=5)
        self.assertAlmostEqual(goal_validator.current_error[1], 0, places=5)

    def test_single_position_goal(self):
        cereal_goal_position = [1.3, 1.5, 0.95]
        goal_validator = GoalValidator(cereal_goal_position, self.cereal.get_position_as_list, PositionErrorChecker())
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, 0.8)
        self.cereal.set_position(cereal_goal_position)
        self.assertEqual(self.cereal.get_position_as_list(), cereal_goal_position)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 1)
        self.assertEqual(goal_validator.current_error, 0)

    def test_single_orientation_goal(self):
        cereal_goal_orientation = quaternion_from_euler(0, 0, np.pi/2)
        goal_validator = GoalValidator(cereal_goal_orientation, self.cereal.get_orientation_as_list,
                                       OrientationErrorChecker())
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, [np.pi/2])
        self.cereal.set_orientation(cereal_goal_orientation)
        for v1, v2 in zip(self.cereal.get_orientation_as_list(), cereal_goal_orientation.tolist()):
            self.assertAlmostEqual(v1, v2, places=5)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 1)
        self.assertAlmostEqual(goal_validator.current_error[0], 0, places=5)

    def test_single_revolute_joint_position_goal(self):
        goal_joint_position = -np.pi/4
        goal_validator = GoalValidator(goal_joint_position,
                                       lambda: self.robot.get_joint_position('l_shoulder_lift_joint'),
                                       RevoluteJointPositionErrorChecker())
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, abs(goal_joint_position))

        for percent in [0.5, 1]:
            self.robot.set_joint_position('l_shoulder_lift_joint', goal_joint_position*percent)
            self.assertEqual(self.robot.get_joint_position('l_shoulder_lift_joint'), goal_joint_position*percent)
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[0], abs(goal_joint_position)*(1-percent), places=5)

    def test_single_prismatic_joint_position_goal(self):
        goal_joint_position = 0.2
        torso = RobotDescription.current_robot_description.torso_joint
        goal_validator = GoalValidator(goal_joint_position,
                                       lambda: self.robot.get_joint_position(torso),
                                       PrismaticJointPositionErrorChecker())
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, abs(goal_joint_position))

        for percent in [0.5, 1]:
            self.robot.set_joint_position(torso, goal_joint_position*percent)
            self.assertEqual(self.robot.get_joint_position(torso), goal_joint_position*percent)
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[0], abs(goal_joint_position)*(1-percent), places=5)

    def test_multi_joint_goal(self):
        goal_joint_positions = [0.2, -np.pi/4]
        joint_names = ['torso_lift_joint', 'l_shoulder_lift_joint']
        joint_types = [JointType.PRISMATIC, JointType.REVOLUTE]
        goal_validator = GoalValidator(goal_joint_positions,
                                       lambda: [self.robot.get_joint_position('torso_lift_joint'),
                                                self.robot.get_joint_position('l_shoulder_lift_joint')],
                                       MultiJointPositionErrorChecker(joint_types))
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([0.2, abs(-np.pi/4)]), atol=0.001))

        for percent in [0.5, 1]:
            current_joint_positions = [0.2*percent, -np.pi/4*percent]
            self.robot.set_joint_positions(dict(zip(joint_names, current_joint_positions)))
            self.assertTrue(np.allclose(self.robot.get_joint_position('torso_lift_joint'), current_joint_positions[0],
                                        atol=0.001))
            self.assertTrue(np.allclose(self.robot.get_joint_position('l_shoulder_lift_joint'), current_joint_positions[1],
                                        atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[0], abs(0.2)*(1-percent), places=5)
            self.assertAlmostEqual(goal_validator.current_error[1], abs(-np.pi/4)*(1-percent), places=5)

    def test_list_of_poses_goal(self):
        position_goal = [0.0, 1.0, 0.0]
        orientation_goal = np.array([0, 0, np.pi/2])
        poses_goal = [Pose(position_goal, quaternion_from_euler(*orientation_goal.tolist())),
                      Pose(position_goal, quaternion_from_euler(*orientation_goal.tolist()))]
        goal_validator = GoalValidator(poses_goal, lambda: [self.robot.get_pose(), self.robot.get_pose()],
                                       PoseErrorChecker(is_iterable=True))
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([1.0, np.pi/2, 1.0, np.pi/2]), atol=0.001))

        for percent in [0.5, 1]:
            current_orientation_goal = orientation_goal * percent
            current_pose_goal = Pose([0.0, 1.0*percent, 0.0],
                                     quaternion_from_euler(*current_orientation_goal.tolist()))
            self.robot.set_pose(current_pose_goal)
            self.assertTrue(np.allclose(self.robot.get_position_as_list(), current_pose_goal.position_as_list(),
                                        atol=0.001))
            self.assertTrue(np.allclose(self.robot.get_orientation_as_list(), current_pose_goal.orientation_as_list(),
                                        atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[0], 1-percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[1], np.pi * (1-percent) /2, places=5)
            self.assertAlmostEqual(goal_validator.current_error[2], (1-percent), places=5)
            self.assertAlmostEqual(goal_validator.current_error[3], np.pi * (1-percent) /2, places=5)

    def test_list_of_positions_goal(self):
        position_goal = [0.0, 1.0, 0.0]
        positions_goal = [position_goal, position_goal]
        goal_validator = GoalValidator(positions_goal, lambda: [self.robot.get_position_as_list(), self.robot.get_position_as_list()],
                                       PositionErrorChecker(is_iterable=True))
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([1.0, 1.0]), atol=0.001))

        for percent in [0.5, 1]:
            current_position_goal = [0.0, 1.0*percent, 0.0]
            self.robot.set_position(current_position_goal)
            self.assertTrue(np.allclose(self.robot.get_position_as_list(), current_position_goal, atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[0], 1-percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[1], 1-percent, places=5)

    def test_list_of_orientations_goal(self):
        orientation_goal = np.array([0, 0, np.pi/2])
        orientations_goals = [quaternion_from_euler(*orientation_goal.tolist()),
                             quaternion_from_euler(*orientation_goal.tolist())]
        goal_validator = GoalValidator(orientations_goals, lambda: [self.robot.get_orientation_as_list(),
                                                                   self.robot.get_orientation_as_list()],
                                       OrientationErrorChecker(is_iterable=True))
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([np.pi/2, np.pi/2]), atol=0.001))

        for percent in [0.5, 1]:
            current_orientation_goal = orientation_goal * percent
            self.robot.set_orientation(quaternion_from_euler(*current_orientation_goal.tolist()))
            self.assertTrue(np.allclose(self.robot.get_orientation_as_list(),
                                        quaternion_from_euler(*current_orientation_goal.tolist()),
                                        atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[0], np.pi*(1-percent)/2, places=5)
            self.assertAlmostEqual(goal_validator.current_error[1], np.pi*(1-percent)/2, places=5)

    def test_list_of_revolute_joint_positions_goal(self):
        goal_joint_position = -np.pi/4
        goal_joint_positions = [goal_joint_position, goal_joint_position]
        goal_validator = GoalValidator(goal_joint_positions,
                                       lambda: [self.robot.get_joint_position('l_shoulder_lift_joint'),
                                                self.robot.get_joint_position('l_shoulder_lift_joint')],
                                       RevoluteJointPositionErrorChecker(is_iterable=True))
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error,
                                    np.array([abs(goal_joint_position), abs(goal_joint_position)]), atol=0.001))

        for percent in [0.5, 1]:
            current_joint_position = goal_joint_position * percent
            self.robot.set_joint_position('l_shoulder_lift_joint', current_joint_position)
            self.assertTrue(np.allclose(self.robot.get_joint_position('l_shoulder_lift_joint'), current_joint_position, atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error[0], abs(goal_joint_position)*(1-percent), places=5)
            self.assertAlmostEqual(goal_validator.current_error[1], abs(goal_joint_position)*(1-percent), places=5)

