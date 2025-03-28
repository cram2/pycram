from pycram.designators import action_designator
from pycram.designators.location_designator import *
from pycram.process_module import simulated_robot
from pycram.robot_description import RobotDescription
from pycram.datastructures.pose import PoseStamped
from pycram.testing import BulletWorldTestCase


class TestActionDesignatorGrounding(BulletWorldTestCase):

    def test_reachability(self):
        self.robot.set_joint_position(RobotDescription.current_robot_description.torso_joint, 0.3)
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        left_arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0}
        right_arm_park = {'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        self.robot.set_joint_positions(left_arm_park)
        self.robot.set_joint_positions(right_arm_park)
        location_desig = CostmapLocation(object_desig.resolve(), reachable_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()()) == 3)
        self.assertTrue(len(location.orientation.to_list()()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_reachability_pose(self):
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        left_arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0}
        right_arm_park = {'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        self.robot.set_joint_positions(left_arm_park)
        self.robot.set_joint_positions(right_arm_park)
        location_desig = CostmapLocation(PoseSteamped.from_list([0.4, 0.6, 0.9], [0, 0, 0, 1]), reachable_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()()) == 3)
        self.assertTrue(len(location.orientation.to_list()()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_visibility(self):
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        left_arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0}
        right_arm_park = {'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        self.robot.set_joint_positions(left_arm_park)
        self.robot.set_joint_positions(right_arm_park)
        location_desig = CostmapLocation(object_desig.resolve(), visible_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()()) == 3)
        self.assertTrue(len(location.orientation.to_list()()) == 4)

    def test_reachability_and_visibility(self):
        self.robot.set_joint_position(RobotDescription.current_robot_description.torso_joint, 0.3)
        left_arm_park = {'l_shoulder_pan_joint': 1.712,
                         'l_shoulder_lift_joint': -0.264,
                         'l_upper_arm_roll_joint': 1.38,
                         'l_elbow_flex_joint': -2.12,
                         'l_forearm_roll_joint': 16.996,
                         'l_wrist_flex_joint': -0.073,
                         'l_wrist_roll_joint': 0.0}
        right_arm_park = {'r_shoulder_pan_joint': -1.712,
                          'r_shoulder_lift_joint': -0.256,
                          'r_upper_arm_roll_joint': -1.463,
                          'r_elbow_flex_joint': -2.12,
                          'r_forearm_roll_joint': 1.766,
                          'r_wrist_flex_joint': -0.07,
                          'r_wrist_roll_joint': 0.051}
        self.robot.set_joint_positions(left_arm_park)
        self.robot.set_joint_positions(right_arm_park)
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        location_desig = CostmapLocation(object_desig.resolve(), reachable_for=robot_desig.resolve(),
                                         visible_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()()) == 3)
        self.assertTrue(len(location.orientation.to_list()()) == 4)
        #self.assertTrue(Arms.LEFT == location.reachable_arm or Arms.RIGHT == location.reachable_arm)

    def test_semantic_location(self):
        kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
        location_desig = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()()) == 3)
        self.assertTrue(len(location.orientation.to_list()()) == 4)

        milk_desig = ObjectDesignatorDescription(names=["milk"])
        location_desig = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(),
                                                 for_object=milk_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.position.to_list()()) == 3)
        self.assertTrue(len(location.orientation.to_list()()) == 4)
