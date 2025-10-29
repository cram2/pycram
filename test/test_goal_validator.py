import numpy as np
from semantic_digital_twin.spatial_types import TransformationMatrix

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
        pose_goal_validators = PoseGoalValidator(
            lambda: PoseStamped.from_spatial_type(self.world.get_body_by_name("milk.stl").global_pose))
        self.validate_pose_goal(pose_goal_validators)

    def test_single_pose_goal_generic(self):
        pose_goal_validators = GoalValidator(PoseErrorChecker(), lambda: PoseStamped.from_spatial_type(
            self.world.get_body_by_name("milk.stl").global_pose))
        self.validate_pose_goal(pose_goal_validators)

    def validate_pose_goal(self, goal_validator):
        milk_goal_pose = PoseStamped.from_list( [2.5, 2.4, 1], frame=self.world.root)
        goal_validator.register_goal(milk_goal_pose)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 0.5, places=5)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[1], 0, places=5)
        self.world.get_body_by_name("milk.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(2.5, 2.4,
                                                                                                             1,
                                                                                                             reference_frame=self.world.root)
        self.assertEqual(PoseStamped.from_spatial_type(self.world.get_body_by_name("milk.stl").global_pose),
                         milk_goal_pose)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 1)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 0, places=5)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[1], 0, places=5)

    def test_single_position_goal_generic(self):
        goal_validator = GoalValidator(PositionErrorChecker(), lambda: PoseStamped.from_spatial_type(
            self.world.get_body_by_name("breakfast_cereal.stl").global_pose).position.to_list())
        self.validate_position_goal(goal_validator)

    def test_single_position_goal(self):
        goal_validator = PositionGoalValidator(lambda: PoseStamped.from_spatial_type(
            self.world.get_body_by_name("breakfast_cereal.stl").global_pose).position.to_list())
        self.validate_position_goal(goal_validator)

    def validate_position_goal(self, goal_validator):
        cereal_goal_position = [3, 1.8, 1]
        goal_validator.register_goal(cereal_goal_position)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertAlmostEqual(float(goal_validator.current_error), 0.8)
        self.world.get_body_by_name("breakfast_cereal.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(3,
                                                                                                                   1.8,
                                                                                                                   1,
                                                                                                                   reference_frame=self.world.root)
        self.assertEqual(
            PoseStamped.from_spatial_type(self.world.get_body_by_name("breakfast_cereal.stl").global_pose).position.to_list(),
            cereal_goal_position)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 1)
        self.assertEqual(goal_validator.current_error, 0)

    def test_single_orientation_goal_generic(self):
        goal_validator = GoalValidator(OrientationErrorChecker(), lambda: PoseStamped.from_spatial_type(
            self.world.get_body_by_name("breakfast_cereal.stl").global_pose).orientation.to_list())
        self.validate_orientation_goal(goal_validator)

    def test_single_orientation_goal(self):
        goal_validator = OrientationGoalValidator(lambda: PoseStamped.from_spatial_type(
            self.world.get_body_by_name("breakfast_cereal.stl").global_pose).orientation.to_list())
        self.validate_orientation_goal(goal_validator)

    def validate_orientation_goal(self, goal_validator):
        cereal_goal_orientation = quaternion_from_euler(0, 0, np.pi / 2)
        goal_validator.register_goal(cereal_goal_orientation)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, [np.pi / 2])
        self.world.get_body_by_name(
            "breakfast_cereal.stl").parent_connection.origin = TransformationMatrix.from_xyz_quaternion(
            quat_x=cereal_goal_orientation[0],
            quat_y=cereal_goal_orientation[1],
            quat_z=cereal_goal_orientation[2],
            quat_w=cereal_goal_orientation[3],
            reference_frame=self.world.root)
        for v1, v2 in zip(PoseStamped.from_spatial_type(
                self.world.get_body_by_name("breakfast_cereal.stl").global_pose).orientation.to_list(),
                          cereal_goal_orientation):
            self.assertAlmostEqual(v1, v2, places=5)
        self.assertTrue(goal_validator.goal_achieved)
        self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, 1, places=5)
        self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 0, places=5)

    def test_single_revolute_joint_position_goal_generic(self):
        goal_validator = GoalValidator(RevoluteJointPositionErrorChecker(), lambda name: self.world.state[
            self.world.get_degree_of_freedom_by_name(name).name].position)
        self.validate_revolute_joint_position_goal(goal_validator)

    def test_single_revolute_joint_position_goal(self):
        goal_validator = JointPositionGoalValidator(
            lambda name: self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position)
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
            self.world.state[self.world.get_degree_of_freedom_by_name(
                "l_shoulder_lift_joint").name].position = goal_joint_position * percent
            self.assertEqual(
                self.world.state[self.world.get_degree_of_freedom_by_name("l_shoulder_lift_joint").name].position,
                goal_joint_position * percent)
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], abs(goal_joint_position) * (1 - percent),
                                   places=5)

    def test_single_prismatic_joint_position_goal_generic(self):
        goal_validator = GoalValidator(PrismaticJointPositionErrorChecker(), lambda name: self.world.state[
            self.world.get_degree_of_freedom_by_name(name).name].position)
        self.validate_prismatic_joint_position_goal(goal_validator)

    def test_single_prismatic_joint_position_goal(self):
        goal_validator = JointPositionGoalValidator(
            lambda name: self.world.state[self.world.get_degree_of_freedom_by_name(name).name].position)
        self.validate_prismatic_joint_position_goal(goal_validator, JointType.PRISMATIC)

    def validate_prismatic_joint_position_goal(self, goal_validator, joint_type: Optional[JointType] = None):
        goal_joint_position = 0.2
        torso = "torso_lift_joint"
        achieved_percentage = [0.46946, 1]
        if joint_type is not None:
            goal_validator.register_goal(goal_joint_position, joint_type, torso)
        else:
            goal_validator.register_goal(goal_joint_position, torso)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertEqual(goal_validator.current_error, 0.1885)

        for percent, achieved_percentage in zip([0.5, 1], achieved_percentage):
            self.world.state[self.world.get_degree_of_freedom_by_name(
                "torso_lift_joint").name].position = goal_joint_position * percent
            self.assertEqual(
                self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position,
                goal_joint_position * percent)
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, achieved_percentage, places=4)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 0.2  * (1 - percent),
                                   places=3)

    def test_multi_joint_goal_generic(self):
        joint_types = [JointType.PRISMATIC, JointType.REVOLUTE]
        goal_validator = GoalValidator(MultiJointPositionErrorChecker(joint_types),
                                       lambda x: [self.world.state[
                                                      self.world.get_degree_of_freedom_by_name(name).name].position for
                                                  name in x])
        self.validate_multi_joint_goal(goal_validator)

    def test_multi_joint_goal(self):
        joint_types = [JointType.PRISMATIC, JointType.REVOLUTE]
        goal_validator = MultiJointPositionGoalValidator(
                                       lambda x: [self.world.state[
                                                      self.world.get_degree_of_freedom_by_name(name).name].position for
                                                  name in x])
        self.validate_multi_joint_goal(goal_validator, joint_types)

    def validate_multi_joint_goal(self, goal_validator, joint_types: Optional[List[JointType]] = None):
        goal_joint_positions = np.array([0.2, -np.pi / 4])
        achieved_percentage = [0.48474, 1]
        joint_names = ['torso_lift_joint', 'l_shoulder_lift_joint']
        if joint_types is not None:
            goal_validator.register_goal(goal_joint_positions, joint_types, joint_names)
        else:
            goal_validator.register_goal(goal_joint_positions, joint_names)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertTrue(np.allclose(goal_validator.current_error, np.array([0.1885, abs(-np.pi / 4)]), atol=0.001))

        for percent, achieved_percentage in zip([0.5, 1], achieved_percentage):
            current_joint_positions = goal_joint_positions * percent
            for joint_name, joint_position in zip(joint_names, current_joint_positions):
                self.world.state[self.world.get_degree_of_freedom_by_name(joint_name).name].position = joint_position
            self.assertTrue(np.allclose(
                self.world.state[self.world.get_degree_of_freedom_by_name("torso_lift_joint").name].position,
                current_joint_positions[0],
                atol=0.001))
            self.assertTrue(
                np.allclose(
                    self.world.state[self.world.get_degree_of_freedom_by_name("l_shoulder_lift_joint").name].position,
                    current_joint_positions[1],
                    atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, achieved_percentage, places=4)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], abs(0.2) * (1 - percent), places=4)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], abs(-np.pi / 4) * (1 - percent), places=5)

    def test_list_of_poses_goal_generic(self):
        goal_validator = GoalValidator(PoseErrorChecker(is_iterable=True),
                                       lambda: [PoseStamped.from_spatial_type(
                                           self.world.get_body_by_name("base_footprint").global_pose),
                                           PoseStamped.from_spatial_type(
                                               self.world.get_body_by_name("base_footprint").global_pose)])
        self.validate_list_of_poses_goal(goal_validator)

    def test_list_of_poses_goal(self):
        goal_validator = MultiPoseGoalValidator(
            lambda: [PoseStamped.from_spatial_type(self.world.get_body_by_name("base_footprint").global_pose),
                     PoseStamped.from_spatial_type(self.world.get_body_by_name("base_footprint").global_pose)])
        self.validate_list_of_poses_goal(goal_validator)

    def validate_list_of_poses_goal(self, goal_validator):
        position_goal = [0.0, 1.0, 0.0]
        orientation_goal = np.array([0, 0, np.pi / 2])
        poses_goal = [
            PoseStamped.from_list( position_goal, quaternion_from_euler(*orientation_goal.tolist()), self.world.root),
            PoseStamped.from_list( position_goal, quaternion_from_euler(*orientation_goal.tolist()), self.world.root)]
        goal_validator.register_goal(poses_goal)
        self.assertFalse(goal_validator.goal_achieved)
        self.assertEqual(goal_validator.actual_percentage_of_goal_achieved, 0)
        self.assertTrue(
            np.allclose(goal_validator.current_error, np.array([1.0, np.pi / 2, 1.0, np.pi / 2]), atol=0.001))

        for percent in [0.5, 1]:
            current_orientation_goal = orientation_goal * percent
            current_pose_goal = PoseStamped.from_list( [0.0, 1.0 * percent, 0.0],
                                                      quaternion_from_euler(*current_orientation_goal.tolist()), self.world.root)
            self.world.get_body_by_name("base_footprint").parent_connection.origin = current_pose_goal.to_spatial_type()
            self.assertTrue(np.allclose(PoseStamped.from_spatial_type(
                self.world.get_body_by_name("base_footprint").global_pose).position.to_list(),
                                        current_pose_goal.position.to_list(),
                                        atol=0.001))
            self.assertTrue(np.allclose(PoseStamped.from_spatial_type(
                self.world.get_body_by_name("base_footprint").global_pose).orientation.to_list(),
                                        current_pose_goal.orientation.to_list(),
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
                                       lambda: [PoseStamped.from_spatial_type(self.world.get_body_by_name(
                                           "base_footprint").global_pose).position.to_list(),
                                                PoseStamped.from_spatial_type(self.world.get_body_by_name(
                                                    "base_footprint").global_pose).position.to_list()])
        self.validate_list_of_positions_goal(goal_validator)

    def test_list_of_positions_goal(self):
        goal_validator = MultiPositionGoalValidator(lambda: [
            PoseStamped.from_spatial_type(self.world.get_body_by_name("base_footprint").global_pose).position.to_list(),
            PoseStamped.from_spatial_type(
                self.world.get_body_by_name("base_footprint").global_pose).position.to_list()])
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
            self.world.get_body_by_name("base_footprint").parent_connection.origin = TransformationMatrix.from_xyz_rpy(
                *current_position_goal)
            self.assertTrue(np.allclose(PoseStamped.from_spatial_type(
                self.world.get_body_by_name("base_footprint").global_pose).position.to_list(), current_position_goal,
                                        atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], 1 - percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], 1 - percent, places=5)

    def test_list_of_orientations_goal_generic(self):
        goal_validator = GoalValidator(OrientationErrorChecker(is_iterable=True),
                                       lambda: [PoseStamped.from_spatial_type(self.world.get_body_by_name(
                                           "base_footprint").global_pose).orientation.to_list(),
                                                PoseStamped.from_spatial_type(self.world.get_body_by_name(
                                                    "base_footprint").global_pose).orientation.to_list()])
        self.validate_list_of_orientations_goal(goal_validator)

    def test_list_of_orientations_goal(self):
        goal_validator = MultiOrientationGoalValidator(lambda: [PoseStamped.from_spatial_type(
            self.world.get_body_by_name("base_footprint").global_pose).orientation.to_list(),
                                                                PoseStamped.from_spatial_type(
                                                                    self.world.get_body_by_name(
                                                                        "base_footprint").global_pose).orientation.to_list()])
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
            self.world.get_body_by_name(
                "base_footprint").parent_connection.origin = TransformationMatrix.from_xyz_rpy(
                pitch=current_orientation_goal[0],
                roll=current_orientation_goal[1],
                yaw=current_orientation_goal[2], )
            self.assertTrue(np.allclose(PoseStamped.from_spatial_type(
                self.world.get_body_by_name("base_footprint").global_pose).orientation.to_list(),
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
                                       lambda x: [self.world.state[
                                                      self.world.get_degree_of_freedom_by_name(name).name].position for
                                                  name in x])
        self.validate_list_of_revolute_joint_positions_goal(goal_validator)

    def test_list_of_revolute_joint_positions_goal(self):
        goal_validator = MultiJointPositionGoalValidator(
            lambda x: [self.world.state[
                           self.world.get_degree_of_freedom_by_name(name).name].position for
                       name in x])
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
            for joint_name, joint_position in zip(joint_names, current_joint_position):
                self.world.state[self.world.get_degree_of_freedom_by_name(joint_name).name].position = joint_position
            self.world.notify_state_change()
            for joint_name, joint_position in zip(joint_names, current_joint_position):
                self.assertTrue(np.allclose(
                    self.world.state[self.world.get_degree_of_freedom_by_name(joint_name).name].position,
                    joint_position,
                    atol=0.001))
            if percent == 1:
                self.assertTrue(goal_validator.goal_achieved)
            else:
                self.assertFalse(goal_validator.goal_achieved)
            self.assertAlmostEqual(goal_validator.actual_percentage_of_goal_achieved, percent, places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[0], abs(goal_joint_position) * (1 - percent),
                                   places=5)
            self.assertAlmostEqual(goal_validator.current_error.tolist()[1], abs(goal_joint_position) * (1 - percent),
                                   places=5)
