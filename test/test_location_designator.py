from pycram.designators.location_designator import *
from pycram.robot_description import RobotDescription
from pycram.datastructures.pose import Pose
from bullet_world_testcase import BulletWorldTestCase


class TestActionDesignatorGrounding(BulletWorldTestCase):

    def test_reachability(self):
        self.robot.set_joint_position(RobotDescription.current_robot_description.torso_joint, 0.3)
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        location_desig = CostmapLocation(object_desig.resolve(), reachable_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose.position_as_list()) == 3)
        self.assertTrue(len(location.pose.orientation_as_list()) == 4)
        self.assertTrue(Arms.LEFT in location.reachable_arms or Arms.RIGHT in location.reachable_arms)

    def test_reachability_pose(self):
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        location_desig = CostmapLocation(Pose([0.4, 0.6, 0.9], [0, 0, 0, 1]), reachable_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose.position_as_list()) == 3)
        self.assertTrue(len(location.pose.orientation_as_list()) == 4)
        self.assertTrue(Arms.LEFT in location.reachable_arms or Arms.RIGHT in location.reachable_arms)

    def test_visibility(self):
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        location_desig = CostmapLocation(object_desig.resolve(), visible_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose.position_as_list()) == 3)
        self.assertTrue(len(location.pose.orientation_as_list()) == 4)

    def test_reachability_and_visibility(self):
        self.robot.set_joint_position(RobotDescription.current_robot_description.torso_joint, 0.3)
        object_desig = ObjectDesignatorDescription(names=["milk"])
        robot_desig = ObjectDesignatorDescription(names=[RobotDescription.current_robot_description.name])
        location_desig = CostmapLocation(object_desig.resolve(), reachable_for=robot_desig.resolve(), visible_for=robot_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose.position_as_list()) == 3)
        self.assertTrue(len(location.pose.orientation_as_list()) == 4)
        self.assertTrue(Arms.LEFT in location.reachable_arms or Arms.RIGHT in location.reachable_arms)

    def test_semantic_location(self):
        kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
        location_desig = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose.position_as_list()) == 3)
        self.assertTrue(len(location.pose.orientation_as_list()) == 4)

        milk_desig = ObjectDesignatorDescription(names=["milk"])
        location_desig = SemanticCostmapLocation("kitchen_island_surface", kitchen_desig.resolve(), for_object=milk_desig.resolve())
        location = location_desig.resolve()
        self.assertTrue(len(location.pose.position_as_list()) == 3)
        self.assertTrue(len(location.pose.orientation_as_list()) == 4)
