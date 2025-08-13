import unittest
from copy import deepcopy

import numpy as np

from pycram.datastructures.enums import Arms, GripperState, DetectionTechnique, TorsoState, \
    StaticJointState, ApproachDirection, VerticalAlignment
from pycram.datastructures.pose import PoseStamped
from pycram.designator import ObjectDesignatorDescription
from pycram.designators import object_designator
from pycram.failures import TorsoGoalNotReached, ConfigurationNotReached, ObjectNotGraspedError, \
    NavigationGoalNotReachedError, \
    PerceptionObjectNotFound, ContainerManipulationError
from pycram.local_transformer import LocalTransformer
from pycram.process_module import simulated_robot
from pycram.robot_description import RobotDescription
from pycram.robot_plans.actions import *
from pycram.robot_plans.motions import MoveTCPWaypointsMotion
from pycram.testing import BulletWorldTestCase
from pycrap.ontologies import Milk


class TestActionDesignatorGrounding(BulletWorldTestCase):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        description = MoveTorsoActionDescription([TorsoState.HIGH])
        plan = SequentialPlan(self.context, description)
        torso_joint = RobotDescription.current_robot_description.torso_joint
        self.assertEqual(description.resolve().torso_state, TorsoState.HIGH)
        self._test_validate_action_pre_perform(description, TorsoGoalNotReached)
        with simulated_robot:
            plan.perform()
        dof = self.apartment_world.get_degree_of_freedom_by_name("torso_lift_joint")
        self.assertEqual(self.apartment_world.state[dof.name].position, 0.3)


    def test_set_gripper(self):
        description = SetGripperActionDescription([Arms.LEFT], [GripperState.OPEN, GripperState.CLOSE])
        plan = SequentialPlan(self.context, description)
        self.assertEqual(description.resolve().gripper, Arms.LEFT)
        self.assertEqual(description.resolve().motion, GripperState.OPEN)
        # self.assertEqual(len(list(iter(description))), 2)
        with simulated_robot:
            plan.perform()
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(
                Arms.LEFT).get_static_gripper_state(GripperState.OPEN).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), state)

    def test_park_arms(self):
        description = ParkArmsActionDescription([Arms.BOTH])
        plan = SequentialPlan(self.context, description)
        self.assertEqual(description.resolve().arm, Arms.BOTH)
        self._test_validate_action_pre_perform(description, ConfigurationNotReached)
        with simulated_robot:
            plan.perform()
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("right",
                                                                                             StaticJointState.Park).items():
            joint_position = self.world.robot.get_joint_position(joint)
            self.assertEqual(joint_position, pose)
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left",
                                                                                             StaticJointState.Park).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), pose)

    def test_navigate(self):
        description = NavigateActionDescription([PoseStamped.from_list([0.3, 0, 0], [0, 0, 0, 1])])
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            self._test_validate_action_pre_perform(description, NavigationGoalNotReachedError)
            plan.perform()
        self.assertEqual(description.resolve().target_location, PoseStamped.from_list([0.3, 0, 0], [0, 0, 0, 1]))
        # self.assertEqual(self.robot.get_pose(), PoseStamped.from_list([0.3, 0, 0]))
        expected_pose = np.eye(4)
        expected_pose[:3, 3] = [0.3, 0, 0]
        np.testing.assert_almost_equal(
            self.apartment_world.compute_forward_kinematics_np(self.apartment_world.root,
                                                               self.apartment_world.get_body_by_name("base_footprint")),
            expected_pose, decimal=2)

    def test_reach_to_pick_up(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        grasp_description = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)
        performable = ReachToPickUpActionDescription(object_description.resolve(),
                                                     Arms.LEFT, grasp_description)
        plan = SequentialPlan(self.context,
                              NavigateActionDescription(PoseStamped.from_list([0.65, 0.4, 0], [0, 0, 0, 1]), True),
                              ParkArmsActionDescription(Arms.BOTH),
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                              SetGripperActionDescription(Arms.LEFT, GripperState.OPEN),
                              performable)
        with simulated_robot:
            plan.perform()

    def test_pick_up(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])

        grasp_description = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)
        description = PickUpActionDescription(object_description, [Arms.LEFT], [grasp_description])

        self.assertEqual(description.resolve().object_designator.name, "milk")
        plan = SequentialPlan(self.context,
                              NavigateActionDescription(PoseStamped.from_list([0.65, 0.4, 0], [0, 0, 0, 1]), True),
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                              description)
        with simulated_robot:
            plan.perform()
        self.assertTrue(object_description.resolve() in self.robot.attachments.keys())

    def test_place(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = PlaceActionDescription(object_description, [PoseStamped.from_list([1.3, 1, 0.9],
                                                                                        [0, 0, 0, 1])],
                                             [Arms.LEFT])
        self.assertEqual(description.resolve().object_designator.name, "milk")
        plan = SequentialPlan(self.context,
                              NavigateActionDescription(PoseStamped.from_list([0.6, 0.4, 0], [0, 0, 0, 1]), True),
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                              PickUpActionDescription(object_description.resolve(), Arms.LEFT,
                                                      GraspDescription(ApproachDirection.FRONT,
                                                                       VerticalAlignment.NoAlignment, False)),
                              description)
        with simulated_robot:
            plan.perform()
        self.assertFalse(object_description.resolve() in self.robot.attachments.keys())

    def test_look_at(self):
        description = LookAtAction.description([PoseStamped.from_list([1, 0, 1])])
        self.assertEqual(description.resolve().target, PoseStamped.from_list([1, 0, 1]))
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            # self._test_validate_action_pre_perform(description, LookAtGoalNotReached)
            plan.perform()

    @unittest.skip("validation isn't working")
    def test_detect(self):
        self.kitchen.set_pose(PoseStamped.from_list([10, 10, 0]))
        self.milk.set_pose(PoseStamped.from_list([1.5, 0, 1.2]))
        object_description = ObjectDesignatorDescription(types=[Milk])
        description = DetectActionDescription(technique=DetectionTechnique.TYPES, object_designator=object_description)
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            self._test_validate_action_pre_perform(description, PerceptionObjectNotFound)
            detected_object = plan.perform()

        self.assertEqual(detected_object[0].name, "milk")
        self.assertEqual(detected_object[0].obj_type, Milk)
        self.assertEqual(detected_object[0].world, self.milk.world)

    # Skipped since openand close work only in the apartment at the moment
    def test_open(self):
        kitchen_designator = object_designator.ObjectDesignatorDescription(names=["kitchen"]).resolve()
        object_description = object_designator.ObjectPart(names=["kitchen_island_left_upper_drawer_main"],
                                                          part_of=kitchen_designator)
        description = OpenActionDescription(object_description, [Arms.LEFT])
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
        description = CloseActionDescription(object_description, [Arms.LEFT])
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
        description = TransportActionDescription(object_description,
                                                 [PoseStamped.from_list([-1.4, 0.78, 0.95],
                                                                        [0.0, 0.0, 0.16439898301071468,
                                                                         0.9863939245479175])],
                                                 [Arms.LEFT])
        plan = SequentialPlan(self.context, MoveTorsoActionDescription([TorsoState.HIGH]),
                              description)
        with simulated_robot:
            plan.perform()
        self.assertEqual(description.resolve().object_designator.name, "milk")
        milk_position = np.array(self.milk.get_pose().position.to_list())
        dist = np.linalg.norm(milk_position - np.array([-1.4, 0.78, 0.95]))
        self.assertLessEqual(dist, 0.01)

    def test_grasping(self):
        self.milk.set_pose(PoseStamped.from_list([-1.4, 1, 1]))
        self.robot.set_pose(PoseStamped.from_list([-2.14, 1.06, 0]))
        milk_desig = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = GraspingActionDescription(milk_desig, [Arms.RIGHT])
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            self._test_validate_action_pre_perform(description, ObjectNotGraspedError)
            plan.perform()
        dist = np.linalg.norm(
            np.array(self.robot.get_link_position_as_list(
                RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame())) -
            np.array(self.milk.get_position_as_list()))
        self.assertTrue(dist < 0.01)

    def test_facing(self):
        with simulated_robot:
            plan = SequentialPlan(self.context, FaceAtActionDescription(self.milk.pose, True))
            plan.perform()
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
        plan = SequentialPlan(self.context, MoveTorsoActionDescription([TorsoState.HIGH]),
                              SearchActionDescription(PoseStamped.from_list([1, 1, 1]), Milk))
        with simulated_robot:
            milk = plan.perform()
        self.assertTrue(milk)
        self.assertEqual(milk.obj_type, Milk)
        self.assertEqual(self.milk.pose, milk.pose)
