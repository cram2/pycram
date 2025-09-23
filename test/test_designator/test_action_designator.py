import unittest
from copy import deepcopy

import numpy as np
import rustworkx
from semantic_world.adapters.viz_marker import VizMarkerPublisher
from semantic_world.robots import PR2

from pycram.datastructures.enums import Arms, GripperState, DetectionTechnique, TorsoState, \
    StaticJointState, ApproachDirection, VerticalAlignment
from pycram.datastructures.pose import PoseStamped
from pycram.designator import ObjectDesignatorDescription
from pycram.designators import object_designator
from pycram.failures import TorsoGoalNotReached, ConfigurationNotReached, ObjectNotGraspedError, \
    NavigationGoalNotReachedError, \
    PerceptionObjectNotFound, ContainerManipulationError
from pycram.process_module import simulated_robot
from pycram.robot_plans.actions import *
from pycram.robot_plans.motions import MoveTCPWaypointsMotion
from pycram.ros import node
from pycram.testing import BulletWorldTestCase
from pycrap.ontologies import Milk
import pycram.robot_descriptions.pr2_states


class TestActionDesignatorGrounding(BulletWorldTestCase):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        description = MoveTorsoActionDescription([TorsoState.HIGH])
        plan = SequentialPlan(self.context, self.robot_view, description)
        self.assertEqual(description.resolve().torso_state, TorsoState.HIGH)
        with simulated_robot:
            plan.perform()
        dof = self.world.get_degree_of_freedom_by_name("torso_lift_joint")
        self.assertEqual(self.world.state[dof.name].position, 0.3)


    def test_set_gripper(self):
        description = SetGripperActionDescription([Arms.LEFT], [GripperState.OPEN, GripperState.CLOSE])
        plan = SequentialPlan(self.context, self.robot_view, description)
        self.assertEqual(description.resolve().gripper, Arms.LEFT)
        self.assertEqual(description.resolve().motion, GripperState.OPEN)
        with simulated_robot:
            plan.perform()
        joint_state = JointStateManager().get_gripper_state(Arms.LEFT, GripperState.OPEN, self.robot_view)
        for joint, state in zip(joint_state.joint_names, joint_state.joint_positions):
            dof = self.world.get_degree_of_freedom_by_name(joint)
            self.assertEqual(self.world.state[dof.name].position, state)


    def test_park_arms(self):
        description = ParkArmsActionDescription([Arms.BOTH])
        plan = SequentialPlan(self.context, self.robot_view, description)
        self.assertEqual(description.resolve().arm, Arms.BOTH)
        with simulated_robot:
            plan.perform()
        joint_states_right = JointStateManager().get_arm_state(Arms.RIGHT,  StaticJointState.Park, self.robot_view)
        joint_states_left = JointStateManager().get_arm_state(Arms.LEFT, StaticJointState.Park, self.robot_view)
        for joint_name, joint_state in zip(joint_states_right.joint_names, joint_states_right.joint_positions):
            dof = self.world.get_degree_of_freedom_by_name(joint_name)
            self.assertEqual(self.world.state[dof.name].position, joint_state)
        for joint_name, joint_state in zip(joint_states_left.joint_names, joint_states_left.joint_positions):
            dof = self.world.get_degree_of_freedom_by_name(joint_name)
            self.assertEqual(self.world.state[dof.name].position, joint_state)


    def test_navigate(self):
        description = NavigateActionDescription([PoseStamped.from_list(self.world.root, [0.3, 0, 0], [0, 0, 0, 1])])
        plan = SequentialPlan(self.context, self.robot_view, description)
        with simulated_robot:
            plan.perform()
        self.assertEqual(description.resolve().target_location, PoseStamped.from_list(self.world.root, [0.3, 0, 0], [0, 0, 0, 1]))
        # self.assertEqual(self.robot.get_pose(), PoseStamped.from_list([0.3, 0, 0]))
        expected_pose = np.eye(4)
        expected_pose[:3, 3] = [0.3, 0, 0]
        np.testing.assert_almost_equal(
            self.world.compute_forward_kinematics_np(self.world.root,
                                                               self.world.get_body_by_name("base_footprint")),
            expected_pose, decimal=2)

    def test_reach_to_pick_up(self):
        grasp_description = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)
        performable = ReachToPickUpActionDescription(self.world.get_body_by_name("milk.stl"),
                                                     Arms.LEFT, grasp_description)
        plan = SequentialPlan(self.context,self.robot_view,
                              NavigateActionDescription(PoseStamped.from_list(self.world.root, [1.7, 1.5, 0], [0, 0, 0, 1]), True),
                              ParkArmsActionDescription(Arms.BOTH),
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                              SetGripperActionDescription(Arms.LEFT, GripperState.OPEN),
                              performable)
        with simulated_robot:
            plan.perform()

    def test_pick_up(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        grasp_description = GraspDescription(ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False)
        description = PickUpActionDescription(test_world.get_body_by_name("milk.stl"), [Arms.LEFT], [grasp_description])

        plan = SequentialPlan((test_world, None), test_robot,
                              NavigateActionDescription(PoseStamped.from_list(test_world.root, [1.7, 1.5, 0], [0, 0, 0, 1]), True),
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                              description)
        with simulated_robot:
            plan.perform()
        self.assertTrue(test_world.get_connection(test_world.get_body_by_name("l_gripper_tool_frame"), test_world.get_body_by_name("milk.stl")) is not None)

    def test_place(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        object_description = test_world.get_body_by_name("milk.stl")
        description = PlaceActionDescription(object_description, PoseStamped.from_list(test_world.root, [2.2, 2, 1],
                                                                                        [0, 0, 0, 1]),
                                             [Arms.LEFT])
        plan = SequentialPlan((test_world, None), test_robot,
                              NavigateActionDescription(PoseStamped.from_list(test_world.root, [1.7, 1.5, 0], [0, 0, 0, 1]), True),
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                              PickUpActionDescription(object_description, Arms.LEFT,
                                                      GraspDescription(ApproachDirection.FRONT,
                                                                       VerticalAlignment.NoAlignment, False)),
                              description)
        with simulated_robot:
            plan.perform()
        with self.assertRaises(rustworkx.NoEdgeBetweenNodes):
            self.assertTrue(test_world.get_connection(test_world.get_body_by_name("l_gripper_tool_frame"),
                                                      test_world.get_body_by_name("milk.stl")) is None)

    def test_look_at(self):
        description = LookAtAction.description([PoseStamped.from_list(self.world.root, [1, 0, 1])])
        self.assertEqual(description.resolve().target, PoseStamped.from_list(self.world.root, [1, 0, 1]))
        plan = SequentialPlan(self.context,self.robot_view, description)
        with simulated_robot:
            # self._test_validate_action_pre_perform(description, LookAtGoalNotReached)
            plan.perform()

    @unittest.skip("validation isn't working")
    def test_detect(self):
        self.kitchen.set_pose(PoseStamped.from_list([10, 10, 0]))
        self.milk.set_pose(PoseStamped.from_list([1.5, 0, 1.2]))
        object_description = ObjectDesignatorDescription(types=[Milk])
        description = DetectActionDescription(technique=DetectionTechnique.TYPES, object_designator=object_description)
        plan = SequentialPlan(self.context, self.robot_view, description)
        with simulated_robot:
            detected_object = plan.perform()

        self.assertEqual(detected_object[0].name, "milk")
        self.assertEqual(detected_object[0].obj_type, Milk)
        self.assertEqual(detected_object[0].world, self.milk.world)

    def test_open(self):
        plan = SequentialPlan(self.context,self.robot_view,
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                            ParkArmsActionDescription(Arms.BOTH),
                            NavigateActionDescription(PoseStamped.from_list(self.world.root, [1.75, 1.75, 0], [0, 0, 0.5, 1])),
                            OpenActionDescription(self.world.get_body_by_name("handle_cab10_t"), [Arms.LEFT]))
        with simulated_robot:
            plan.perform()
        self.assertEqual(self.world.state[self.world.get_degree_of_freedom_by_name("cabinet10_drawer_top_joint").name].position, 0.45)

    def test_close(self):
        self.world.state[self.world.get_degree_of_freedom_by_name("cabinet10_drawer_top_joint").name].position = 0.45
        self.world.notify_state_change()
        plan = SequentialPlan(self.context, self.robot_view,
                              MoveTorsoActionDescription([TorsoState.HIGH]),
                              ParkArmsActionDescription(Arms.BOTH),
                              NavigateActionDescription(
                                  PoseStamped.from_list(self.world.root, [1.75, 1.75, 0], [0, 0, 0.5, 1])),
                              CloseActionDescription(self.world.get_body_by_name("handle_cab10_t"), [Arms.LEFT]))
        with simulated_robot:
            plan.perform()
        self.assertEqual(
            self.world.state[self.world.get_degree_of_freedom_by_name("cabinet10_drawer_top_joint").name].position,
            0)

    def test_transport(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = TransportActionDescription(object_description,
                                                 [PoseStamped.from_list([-1.4, 0.78, 0.95],
                                                                        [0.0, 0.0, 0.16439898301071468,
                                                                         0.9863939245479175])],
                                                 [Arms.LEFT])
        plan = SequentialPlan(self.context,self.robot_view, MoveTorsoActionDescription([TorsoState.HIGH]),
                              description)
        with simulated_robot:
            plan.perform()
        self.assertEqual(description.resolve().object_designator.name, "milk")
        milk_position = np.array(self.milk.get_pose().position.to_list())
        dist = np.linalg.norm(milk_position - np.array([-1.4, 0.78, 0.95]))
        self.assertLessEqual(dist, 0.01)

    def test_grasping(self):
        description = GraspingActionDescription(self.world.get_body_by_name("milk.stl"), [Arms.RIGHT])
        plan = SequentialPlan(self.context,self.robot_view,
                              NavigateActionDescription(PoseStamped.from_list(self.world.root, [1.8, 1.8, 0]), True),
                              description)
        with simulated_robot:
            plan.perform()
        dist = np.linalg.norm(
          self.world.get_body_by_name("milk.stl").global_pose.to_np()[3, :3])
        self.assertTrue(dist < 0.01)

    def test_facing(self):
        with simulated_robot:
            milk_pose = PoseStamped.from_spatial_type(self.world.get_body_by_name("milk.stl").global_pose)
            plan = SequentialPlan(self.context, self.robot_view,FaceAtActionDescription(milk_pose, True))
            plan.perform()
            milk_in_robot_frame = self.world.transform(self.world.get_body_by_name("milk.stl").global_pose, self.robot_view.root)
            milk_in_robot_frame = PoseStamped.from_spatial_type(milk_in_robot_frame)
            # milk_in_robot_frame = LocalTransformer().transform_to_object_frame(self.milk.pose, self.robot)
            self.assertAlmostEqual(milk_in_robot_frame.position.y, 0.)

    def test_move_tcp_waypoints(self):
        gripper_pose = PoseStamped.from_spatial_type(self.world.get_body_by_name("r_gripper_tool_frame").global_pose)
        path = []
        for i in range(1, 3):
            new_pose = deepcopy(gripper_pose)
            new_pose.position.z += 0.05 * i
            path.append(new_pose)
        description = MoveTCPWaypointsMotion(path, Arms.RIGHT)
        plan = SequentialPlan(self.context, self.robot_view, description)
        with simulated_robot:
            plan.perform()
        gripper_position = PoseStamped.from_spatial_type(self.world.get_body_by_name("r_gripper_tool_frame").global_pose)
        self.assertAlmostEqual(path[-1].position.x, gripper_position.position.x)
        self.assertAlmostEqual(path[-1].position.y, gripper_position.position.y)
        self.assertAlmostEqual(path[-1].position.z, gripper_position.position.z)

        self.assertAlmostEqual(path[-1].orientation.x, gripper_position.orientation.x, places=4)
        self.assertAlmostEqual(path[-1].orientation.y, gripper_position.orientation.y, places=4)
        self.assertAlmostEqual(path[-1].orientation.z, gripper_position.orientation.z, places=4)
        self.assertAlmostEqual(path[-1].orientation.w, gripper_position.orientation.w, places=4)



    def test_search_action(self):
        plan = SequentialPlan(self.context,self.robot_view, MoveTorsoActionDescription([TorsoState.HIGH]),
                              SearchActionDescription(PoseStamped.from_list([1, 1, 1]), Milk))
        with simulated_robot:
            milk = plan.perform()
        self.assertTrue(milk)
        self.assertEqual(milk.obj_type, Milk)
        self.assertEqual(self.milk.pose, milk.pose)
