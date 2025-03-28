import numpy as np
from pycram.tf_transformations import quaternion_from_euler
from typing_extensions import Optional, List

from pycram.testing import BulletWorldTestCase
from pycram.datastructures.enums import JointType
from pycram.datastructures.pose import PoseStamped
from pycram.robot_description import RobotDescription
from pycram.validation.error_checkers import PoseErrorChecker, PositionErrorChecker, \
    OrientationErrorChecker, RevoluteJointPositionErrorChecker, PrismaticJointPositionErrorChecker, \
    MultiJointPositionErrorChecker
from pycram.validation.goal_validator import GoalValidator, PoseGoalValidator, \
    PositionGoalValidator, OrientationGoalValidator, JointPositionGoalValidator, MultiJointPositionGoalValidator, \
    MultiPoseGoalValidator, MultiPositionGoalValidator, MultiOrientationGoalValidator


class TestGoalValidator(BulletWorldTestCase):

    def test_single_pose_goal(self):
        pose_goal_validators = PoseGoalValidator(self.milk.get_pose)
        self.validate_pose_goal(pose_goal_validators)

    def test_single_pose_goal_generic(self):
        pose_goal_validators = GoalValidator(PoseErrorChecker(), self.milk.get_pose)
        self.validate_pose_goal(pose_goal_validators)

    def validate_pose_goal(self, goal_validator):
        milk_goal_pose = PoseSteamped.from_list([1.3, 1.5, 0.9])
        goal_validator.register_goal(milk_goal_pose)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 0.5, places=5)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[1], 0, places=5)
        self.milk.set_pose(milk_goal_pose)
        self.assertEqual(self.milk.get_pose(), milk_goal_pose)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 1)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 0, places=5)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[1], 0, places=5)

    def test_single_position_goal_generic(self):
        goal_validator = GoalValidator(PositionErrorChecker(), self.cereal.get_position_as_list)
        self.validate_position_goal(goal_validator)

    def test_single_position_goal(self):
        goal_validator = PositionGoalValidator(self.cereal.get_position_as_list)
        self.validate_position_goal(goal_validator)

    def validate_position_goal(self, goal_validator):
        cereal_goal_position = [1.3, 1.5, 0.95]
        goal_validator.register_goal(cereal_goal_position)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, 0.8)
        self.cereal.set_position(cereal_goal_position)
        self.assertEqual(self.cereal.get_position_as_list(), cereal_goal_position)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 1)
        self.assertEqual(goal_validator.current_error, 0)

    def test_single_orientation_goal_generic(self):
        goal_validator = GoalValidator(OrientationErrorChecker(), self.cereal.get_orientation_as_list)
        self.validate_orientation_goal(goal_validator)

    def test_single_orientation_goal(self):
        goal_validator = OrientationGoalValidator(self.cereal.get_orientation_as_list)
        self.validate_orientation_goal(goal_validator)

    def validate_orientation_goal(self, goal_validator):
        cereal_goal_orientation = quaternion_from_euler(0, 0, np.pi / 2)
        goal_validator.register_goal(cereal_goal_orientation)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, [np.pi / 2])
        self.cereal.set_orientation(cereal_goal_orientation)
        for v1, v2 in zip(self.cereal.get_orientation_as_list(), cereal_goal_orientation):
            self.assertAlmostEqual(v1, v2, places=5)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, 1, places=5)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 0, places=5)

    def test_single_revolute_joint_position_goal_generic(self):
        goal_validator = GoalValidator(RevoluteJointPositionErrorChecker(), self.robot.get_joint_position)
        self.validate_revolute_joint_position_goal(goal_validator)

    def test_single_revolute_joint_position_goal(self):
        goal_validator = JointPositionGoalValidator(self.robot.get_joint_position)
        self.validate_revolute_joint_position_goal(goal_validator, JointType.REVOLUTE)

    def validate_revolute_joint_position_goal(self, goal_validator, joint_type: Optional[JointType] = None):
        goal_joint_position = -np.pi / 8
        joint_name = 'l_shoulder_lift_joint'
        if joint_type is not None:
            goal_validator.register_goal(goal_joint_position, joint_type, joint_name)
        else:
            goal_validator.register_goal(goal_joint_position, joint_name)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, abs(goal_joint_position))

        for percent in [0.5, 1]:
            self.robot.set_joint_position('l_shoulder_lift_joint', goal_joint_position * percent)
            self.assertEqual(self.robot.get_joint_position('l_shoulder_lift_joint'), goal_joint_position * percent)
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], abs(goal_joint_position) * (1 - percent),
                                   places=5)

    def test_single_prismatic_joint_position_goal_generic(self):
        goal_validator = GoalValidator(PrismaticJointPositionErrorChecker(), self.robot.get_joint_position)
        self.validate_prismatic_joint_position_goal(goal_validator)

    def test_single_prismatic_joint_position_goal(self):
        goal_validator = JointPositionGoalValidator(self.robot.get_joint_position)
        self.validate_prismatic_joint_position_goal(goal_validator, JointType.PRISMATIC)

    def validate_prismatic_joint_position_goal(self, goal_validator, joint_type: Optional[JointType] = None):
        goal_joint_position = 0.2
        torso = RobotDescription.current_robot_description.torso_joint
        if joint_type is not None:
            goal_validator.register_goal(goal_joint_position, joint_type, torso)
        else:
            goal_validator.register_goal(goal_joint_position, torso)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, abs(goal_joint_position))

        for percent in [0.5, 1]:
            self.robot.set_joint_position(torso, goal_joint_position * percent)
            self.assertEqual(self.robot.get_joint_position(torso), goal_joint_position * percent)
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], abs(goal_joint_position) * (1 - percent),
                                   places=5)

    def test_multi_joint_goal_generic(self):
        joint_types = [JointType.PRISMATIC, JointType.REVOLUTE]
        goal_validator = GoalValidator(MultiJointPositionErrorChecker(joint_types),
                                       lambda x: list(self.robot.get_multiple_joint_positions(x).values()))
        self.validate_multi_joint_goal(goal_validator)

    def test_multi_joint_goal(self):
        joint_types = [JointType.PRISMATIC, JointType.REVOLUTE]
        goal_validator = MultiJointPositionGoalValidator(
            lambda x: list(self.robot.get_multiple_joint_positions(x).values()))
        self.validate_multi_joint_goal(goal_validator, joint_types)

    def validate_multi_joint_goal(self, goal_validator, joint_types: Optional[List[JointType]] = None):
        goal_joint_positions = np.array([0.2, -np.pi / 4])
        joint_names = ['torso_lift_joint', 'l_shoulder_lift_joint']
        if joint_types is not None:
            goal_validator.register_goal(goal_joint_positions, joint_types, joint_names)
        else:
            goal_validator.register_goal(goal_joint_positions, joint_names)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([0.2, abs(-np.pi / 4)]), atol=0.001))

        for percent in [0.5, 1]:
            current_joint_positions = goal_joint_positions * percent
            self.robot.set_multiple_joint_positions(dict(zip(joint_names, current_joint_positions.tolist())))
            self.assertTrue(np.allclose(self.robot.get_joint_position('torso_lift_joint'), current_joint_positions[0],
                                        atol=0.001))
            self.assertTrue(
                np.allclose(self.robot.get_joint_position('l_shoulder_lift_joint'), current_joint_positions[1],
                            atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], abs(0.2) * (1 - percent), places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], abs(-np.pi / 4) * (1 - percent), places=5)

    def test_list_of_poses_goal_generic(self):
        goal_validator = GoalValidator(PoseErrorChecker(is_iterable=True),
                                       lambda: [self.robot.get_pose(), self.robot.get_pose()])
        self.validate_list_of_poses_goal(goal_validator)

    def test_list_of_poses_goal(self):
        goal_validator = MultiPoseGoalValidator(lambda: [self.robot.get_pose(), self.robot.get_pose()])
        self.validate_list_of_poses_goal(goal_validator)

    def validate_list_of_poses_goal(self, goal_validator):
        position_goal = [0.0, 1.0, 0.0]
        orientation_goal = np.array([0, 0, np.pi / 2])
        poses_goal = [PoseStamped(position_goal, quaternion_from_euler(*orientation_goal.tolist())),
                      PoseStamped(position_goal, quaternion_from_euler(*orientation_goal.tolist()))]
        goal_validator.register_goal(poses_goal)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertTrue(
            np.allclose(goal_validator.current_error, np.array([1.0, np.pi / 2, 1.0, np.pi / 2]), atol=0.001))

        for percent in [0.5, 1]:
            current_orientation_goal = orientation_goal * percent
            current_pose_goal = PoseSteamped.from_list([0.0, 1.0 * percent, 0.0],
                                            quaternion_from_euler(*current_orientation_goal.tolist()))
            self.robot.set_pose(current_pose_goal)
            self.assertTrue(np.allclose(self.robot.get_position_as_list(), current_pose_goal.position.to_list(),
                                        atol=0.001))
            self.assertTrue(np.allclose(self.robot.get_orientation_as_list(), current_pose_goal.orientation.to_list(),
                                        atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 1 - percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], np.pi * (1 - percent) / 2, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[2], (1 - percent), places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[3], np.pi * (1 - percent) / 2, places=5)

    def test_list_of_positions_goal_generic(self):
        goal_validator = GoalValidator(PositionErrorChecker(is_iterable=True),
                                       lambda: [self.robot.get_position_as_list(), self.robot.get_position_as_list()])
        self.validate_list_of_positions_goal(goal_validator)

    def test_list_of_positions_goal(self):
        goal_validator = MultiPositionGoalValidator(lambda: [self.robot.get_position_as_list(),
                                                             self.robot.get_position_as_list()])
        self.validate_list_of_positions_goal(goal_validator)

    def validate_list_of_positions_goal(self, goal_validator):
        position_goal = [0.0, 1.0, 0.0]
        positions_goal = [position_goal, position_goal]
        goal_validator.register_goal(positions_goal)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([1.0, 1.0]), atol=0.001))

        for percent in [0.5, 1]:
            current_position_goal = [0.0, 1.0 * percent, 0.0]
            self.robot.set_position(current_position_goal)
            self.assertTrue(np.allclose(self.robot.get_position_as_list(), current_position_goal, atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 1 - percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], 1 - percent, places=5)

    def test_list_of_orientations_goal_generic(self):
        goal_validator = GoalValidator(OrientationErrorChecker(is_iterable=True),
                                       lambda: [self.robot.get_orientation_as_list(),
                                                self.robot.get_orientation_as_list()])
        self.validate_list_of_orientations_goal(goal_validator)

    def test_list_of_orientations_goal(self):
        goal_validator = MultiOrientationGoalValidator(lambda: [self.robot.get_orientation_as_list(),
                                                                self.robot.get_orientation_as_list()])
        self.validate_list_of_orientations_goal(goal_validator)

    def validate_list_of_orientations_goal(self, goal_validator):
        orientation_goal = np.array([0, 0, np.pi / 2])
        orientations_goals = [quaternion_from_euler(*orientation_goal.tolist()),
                              quaternion_from_euler(*orientation_goal.tolist())]
        goal_validator.register_goal(orientations_goals)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([np.pi / 2, np.pi / 2]), atol=0.001))

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
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], np.pi * (1 - percent) / 2, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], np.pi * (1 - percent) / 2, places=5)

    def test_list_of_revolute_joint_positions_goal_generic(self):
        goal_validator = GoalValidator(RevoluteJointPositionErrorChecker(is_iterable=True),
                                       lambda x: list(self.robot.get_multiple_joint_positions(x).values()))
        self.validate_list_of_revolute_joint_positions_goal(goal_validator)

    def test_list_of_revolute_joint_positions_goal(self):
        goal_validator = MultiJointPositionGoalValidator(
            lambda x: list(self.robot.get_multiple_joint_positions(x).values()))
        self.validate_list_of_revolute_joint_positions_goal(goal_validator, [JointType.REVOLUTE, JointType.REVOLUTE])

    def validate_list_of_revolute_joint_positions_goal(self, goal_validator,
                                                       joint_types: Optional[List[JointType]] = None):
        goal_joint_position = -np.pi / 4
        goal_joint_positions = np.array([goal_joint_position, goal_joint_position])
        joint_names = ['l_shoulder_lift_joint', 'r_shoulder_lift_joint']
        if joint_types is not None:
            goal_validator.register_goal(goal_joint_positions, joint_types, joint_names)
        else:
            goal_validator.register_goal(goal_joint_positions, joint_names)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error,
                                    np.array([abs(goal_joint_position), abs(goal_joint_position)]), atol=0.001))

        for percent in [0.5, 1]:
            current_joint_position = goal_joint_positions * percent
            self.robot.set_multiple_joint_positions(dict(zip(joint_names, current_joint_position)))
            self.assertTrue(np.allclose(list(self.robot.get_multiple_joint_positions(joint_names).values()),
                                        current_joint_position, atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], abs(goal_joint_position) * (1 - percent),
                                   places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], abs(goal_joint_position) * (1 - percent),
                                   places=5)
