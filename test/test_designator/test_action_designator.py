import time
import unittest
from copy import deepcopy
from datetime import timedelta

from pycram.datastructures.pose import GraspDescription
from pycram.designator import ObjectDesignatorDescription
from pycram.designators import action_designator, object_designator
from pycram.designators.action_designator import PickUpAction, \
    NavigateAction, FaceAtAction, MoveTorsoAction, MoveTorsoActionDescription, SearchActionDescription, \
    ParkArmsActionDescription
from pycram.designators.motion_designator import MoveGripperMotion, MoveTCPWaypointsMotion
from pycram.failures import TorsoGoalNotReached, ConfigurationNotReached, ObjectNotGraspedError, \
    ObjectNotInGraspingArea, ObjectStillInContact, GripperIsNotOpen, NavigationGoalNotReachedError, \
    LookAtGoalNotReached, PerceptionObjectNotFound, ContainerManipulationError
from pycram.local_transformer import LocalTransformer
from pycram.robot_description import RobotDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.enums import ObjectType, Arms, GripperState, Grasp, DetectionTechnique, TorsoState, \
    StaticJointState
from pycram.testing import  BulletWorldTestCase
import numpy as np
from pycrap.ontologies import Milk


class TestActionDesignatorGrounding(BulletWorldTestCase):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        description = action_designator.MoveTorsoActionDescription([TorsoState.HIGH])
        torso_joint = RobotDescription.current_robot_description.torso_joint
        self.assertEqual(description.resolve().torso_state, TorsoState.HIGH)
        self._test_validate_action_pre_perform(description, TorsoGoalNotReached)
        with simulated_robot:
            description.resolve().perform()
        self.assertEqual(self.world.robot.get_joint_position(torso_joint),
                         0.3)

    def test_set_gripper(self):
        description = action_designator.SetGripperActionDescription([Arms.LEFT], [GripperState.OPEN, GripperState.CLOSE])
        self.assertEqual(description.resolve().gripper, Arms.LEFT)
        self.assertEqual(description.resolve().motion, GripperState.OPEN)
        # self.assertEqual(len(list(iter(description))), 2)
        with simulated_robot:
            description.resolve().perform()
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).get_static_gripper_state(GripperState.OPEN).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), state)

    def test_release(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.ReleaseActionDescription(object_description, [Arms.LEFT])
        self.assertEqual(description.resolve().gripper, Arms.LEFT)
        self.assertEqual(description.resolve().object_designator.name, "milk")

    def test_grip(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.GripActionDescription(object_description, [Arms.LEFT], [0.5])
        self.assertEqual(description.resolve().gripper, Arms.LEFT)
        self.assertEqual(description.resolve().object_designator.name, "milk")

    def test_park_arms(self):
        description = action_designator.ParkArmsActionDescription([Arms.BOTH])
        self.assertEqual(description.resolve().arm, Arms.BOTH)
        self._test_validate_action_pre_perform(description, ConfigurationNotReached)
        with simulated_robot:
            description.resolve().perform()
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("right",
                                                                                             StaticJointState.Park).items():
            joint_position = self.world.robot.get_joint_position(joint)
            self.assertEqual(joint_position, pose)
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left",
                                                                                             StaticJointState.Park).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), pose)

    def test_navigate(self):
        description = action_designator.NavigateActionDescription([PoseStamped.from_list([0.3, 0, 0], [0, 0, 0, 1])])
        with simulated_robot:
            self._test_validate_action_pre_perform(description, NavigationGoalNotReachedError)
            description.resolve().perform()
        self.assertEqual(description.resolve().target_location, PoseStamped.from_list([0.3, 0, 0], [0, 0, 0, 1]))
        self.assertEqual(self.robot.get_pose(), PoseStamped.from_list([0.3, 0, 0]))

    def test_reach_to_pick_up(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        performable = action_designator.ReachToPickUpAction(object_description.resolve(),
                                                                  Arms.LEFT, grasp_description)
        self.assertEqual(performable.object_designator.name, "milk")
        with simulated_robot:
            NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
            MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
            self._test_validate_action_pre_perform(performable, ObjectNotInGraspingArea)
            MoveGripperMotion(GripperState.OPEN, Arms.LEFT).perform()
            self._test_validate_action_pre_perform(performable, ObjectNotInGraspingArea)
            performable.perform()

    def test_pick_up(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        grasp_description = GraspDescription(Grasp.FRONT, None, False)
        description = action_designator.PickUpActionDescription(object_description, [Arms.LEFT], [grasp_description])
        self.assertEqual(description.resolve().object_designator.name, "milk")
        with simulated_robot:
            NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
            MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
            self._test_validate_action_pre_perform(description, ObjectNotGraspedError)
            description.resolve().perform()
        self.assertTrue(object_description.resolve() in self.robot.attachments.keys())

    def test_place(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceActionDescription(object_description, [PoseStamped.from_list([1.3, 1, 0.9],
                                                                                                [0, 0, 0, 1])],
                                                    [Arms.LEFT])
        self.assertEqual(description.resolve().object_designator.name, "milk")
        with simulated_robot:
            NavigateAction(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True).perform()
            MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
            grasp_description = GraspDescription(Grasp.FRONT, None, False)
            PickUpAction(object_description.resolve(), Arms.LEFT, grasp_description).perform()
            self._test_validate_action_pre_perform(description, ObjectStillInContact)
            description.resolve().perform()
        self.assertFalse(object_description.resolve() in self.robot.attachments.keys())

    def test_look_at(self):
        description = action_designator.LookAtAction.description([PoseStamped.from_list([1, 0, 1])])
        self.assertEqual(description.resolve().target, PoseStamped.from_list([1, 0, 1]))
        with simulated_robot:
            # self._test_validate_action_pre_perform(description, LookAtGoalNotReached)
            description.resolve().perform()

    @unittest.skip("validation isn't working")
    def test_detect(self):
        self.kitchen.set_pose(PoseStamped.from_list([10, 10, 0]))
        self.milk.set_pose(PoseStamped.from_list([1.5, 0, 1.2]))
        object_description = ObjectDesignatorDescription(types=[Milk])
        description = action_designator.DetectActionDescription(technique=DetectionTechnique.TYPES, object_designator=object_description)
        with simulated_robot:
            self._test_validate_action_pre_perform(description, PerceptionObjectNotFound)
            detected_object = description.resolve().perform()

        self.assertEqual(detected_object[0].name, "milk")
        self.assertEqual(detected_object[0].obj_type, Milk)
        self.assertEqual(detected_object[0].world, self.milk.world)

    # Skipped since openand close work only in the apartment at the moment
    def test_open(self):
        kitchen_designator = object_designator.ObjectDesignatorDescription(names=["kitchen"]).resolve()
        object_description = object_designator.ObjectPart(names=["kitchen_island_left_upper_drawer_main"],
                                                          part_of=kitchen_designator)
        description = action_designator.OpenActionDescription(object_description, [Arms.LEFT])
        self.assertEqual(description.resolve().object_designator.name, "kitchen_island_left_upper_drawer_main")
        self._test_validate_action_pre_perform(description, ContainerManipulationError)

        # TODO: This is a simulated effect of the action, not the action itself.
        link = self.kitchen.links["kitchen_island_left_upper_drawer_main"]
        joint = self.kitchen.find_joint_above_link(link.name)
        self.kitchen.joints[joint].position = self.kitchen.joints[joint].upper_limit

        description.resolve().validate()

    def test_close(self):
        kitchen_designator = object_designator.ObjectDesignatorDescription(names=["kitchen"]).resolve()
        object_description = object_designator.ObjectPart(names=["kitchen_island_left_upper_drawer_main"],
                                                          part_of=kitchen_designator)
        description = action_designator.CloseActionDescription(object_description, [Arms.LEFT])
        self.assertEqual(description.resolve().object_designator.name, "kitchen_island_left_upper_drawer_main")

        link = self.kitchen.links["kitchen_island_left_upper_drawer_main"]
        joint = self.kitchen.find_joint_above_link(link.name)
        self.kitchen.joints[joint].position = self.kitchen.joints[joint].upper_limit
        self._test_validate_action_pre_perform(description, ContainerManipulationError)

        # TODO: This is a simulated effect of the action, not the action itself.
        self.kitchen.joints[joint].position = self.kitchen.joints[joint].lower_limit
        description.resolve().validate()

    def test_transport(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.TransportActionDescription(object_description,
                                                        [PoseStamped.from_list([-1.4, 0.78, 0.95],
                                                                     [0.0, 0.0, 0.16439898301071468, 0.9863939245479175])],
                                                        [Arms.LEFT])
        with simulated_robot:
            action_designator.MoveTorsoActionDescription([TorsoState.HIGH]).resolve().perform()
            description.resolve().perform()
        self.assertEqual(description.resolve().object_designator.name, "milk")
        milk_position = np.array(self.milk.get_pose().position.to_list())
        dist = np.linalg.norm(milk_position - np.array([-1.4, 0.78, 0.95]))
        self.assertLessEqual(dist, 0.01)

    def test_grasping(self):
        self.milk.set_pose(PoseStamped.from_list([-1.4, 1, 1]))
        self.robot.set_pose(PoseStamped.from_list([-2.14, 1.06, 0]))
        milk_desig = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.GraspingActionDescription(milk_desig, [Arms.RIGHT] )
        with simulated_robot:
            self._test_validate_action_pre_perform(description, ObjectNotGraspedError)
            description.resolve().perform()
        dist = np.linalg.norm(
            np.array(self.robot.get_link_position_as_list(RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame())) -
            np.array(self.milk.get_position_as_list()))
        self.assertTrue(dist < 0.01)

    def test_facing(self):
        with simulated_robot:
            FaceAtAction(self.milk.pose, True).perform()
            milk_in_robot_frame = LocalTransformer().transform_to_object_frame(self.milk.pose, self.robot)
            self.assertAlmostEqual(milk_in_robot_frame.position.y, 0.)

    def test_move_tcp_waypoints(self):
        tcp = RobotDescription.current_robot_description.get_arm_tool_frame(arm=Arms.RIGHT)
        gripper_pose = self.robot.links[tcp].pose
        path = []
        for i in range(1, 3):
            new_pose = deepcopy(gripper_pose)
            new_pose.position.z += 0.05 * i
            path.append(new_pose)
        description = MoveTCPWaypointsMotion(path, Arms.RIGHT)
        with simulated_robot:
            description.perform()
        gripper_position_rounded = [round(x, 2) for x in self.robot.links[tcp].pose.position.to_list()]
        goal_position_rounded = [round(x, 2) for x in path[-1].position.to_list()]
        self.assertListEqual(gripper_position_rounded, goal_position_rounded)

    def _test_validate_action_pre_perform(self, action_description, failure):
        try:
            if hasattr(action_description, "resolve"):
                grounded = action_description.resolve()
            else:
                grounded = action_description
            grounded.validate(max_wait_time=timedelta(milliseconds=30))
            self.fail(f"{failure.__name__} should have been raised.")
        except failure:
            pass

    def test_search_action(self):
        description = SearchActionDescription(PoseStamped.from_list([1, 1, 1]), Milk)
        with simulated_robot:
            ParkArmsActionDescription([Arms.BOTH]).perform()
            milk = description.perform()
        self.assertTrue(milk)
        self.assertEqual(milk.obj_type, Milk)
        self.assertEqual(self.milk.pose, milk.pose)
