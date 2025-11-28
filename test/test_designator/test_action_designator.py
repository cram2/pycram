import unittest

import rustworkx
from semantic_digital_twin.adapters.procthor.procthor_semantic_annotations import Milk

from pycram.datastructures.dataclasses import Context
from pycram.process_module import simulated_robot
from pycram.robot_plans.actions import *
from pycram.robot_plans.motions import MoveTCPWaypointsMotion
from pycram.testing import ApartmentWorldTestCase


class TestActionDesignatorGrounding(ApartmentWorldTestCase):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        description = MoveTorsoActionDescription([TorsoState.HIGH])
        plan = SequentialPlan(self.context, description)
        self.assertEqual(description.resolve().torso_state, TorsoState.HIGH)
        with simulated_robot:
            plan.perform()
        dof = self.world.get_degree_of_freedom_by_name("torso_lift_joint")
        self.assertEqual(self.world.state[dof.name].position, 0.3)

    def test_set_gripper(self):
        description = SetGripperActionDescription(
            [Arms.LEFT], [GripperStateEnum.OPEN, GripperStateEnum.CLOSE]
        )
        plan = SequentialPlan(self.context, description)
        self.assertEqual(description.resolve().gripper, Arms.LEFT)
        self.assertEqual(description.resolve().motion, GripperStateEnum.OPEN)
        with simulated_robot:
            plan.perform()
        joint_state = JointStateManager().get_gripper_state(
            Arms.LEFT, GripperStateEnum.OPEN, self.robot_view
        )
        for joint, state in zip(joint_state.joint_names, joint_state.joint_positions):
            dof = self.world.get_degree_of_freedom_by_name(joint)
            self.assertEqual(self.world.state[dof.name].position, state)

    def test_park_arms(self):
        description = ParkArmsActionDescription([Arms.BOTH])
        plan = SequentialPlan(self.context, description)
        self.assertEqual(description.resolve().arm, Arms.BOTH)
        with simulated_robot:
            plan.perform()
        joint_states_right = JointStateManager().get_arm_state(
            Arms.RIGHT, StaticJointState.Park, self.robot_view
        )
        joint_states_left = JointStateManager().get_arm_state(
            Arms.LEFT, StaticJointState.Park, self.robot_view
        )
        for joint_name, joint_state in zip(
            joint_states_right.joint_names, joint_states_right.joint_positions
        ):
            dof = self.world.get_degree_of_freedom_by_name(joint_name)
            self.assertEqual(self.world.state[dof.name].position, joint_state)
        for joint_name, joint_state in zip(
            joint_states_left.joint_names, joint_states_left.joint_positions
        ):
            dof = self.world.get_degree_of_freedom_by_name(joint_name)
            self.assertEqual(self.world.state[dof.name].position, joint_state)

    def test_navigate(self):
        description = NavigateActionDescription(
            [PoseStamped.from_list([0.3, 0, 0], [0, 0, 0, 1], self.world.root)]
        )
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            plan.perform()
        self.assertEqual(
            description.resolve().target_location,
            PoseStamped.from_list([0.3, 0, 0], [0, 0, 0, 1], self.world.root),
        )
        # self.assertEqual(self.robot.get_pose(), PoseStamped.from_list([0.3, 0, 0]))
        expected_pose = np.eye(4)
        expected_pose[:3, 3] = [0.3, 0, 0]
        np.testing.assert_almost_equal(
            self.world.compute_forward_kinematics_np(
                self.world.root, self.world.get_body_by_name("base_footprint")
            ),
            expected_pose,
            decimal=2,
        )

    def test_reach_to_pick_up(self):
        grasp_description = GraspDescription(
            ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
        )
        performable = ReachToPickUpActionDescription(
            self.world.get_body_by_name("milk.stl"), Arms.LEFT, grasp_description
        )
        plan = SequentialPlan(
            self.context,
            NavigateActionDescription(
                PoseStamped.from_list([1.7, 1.5, 0], [0, 0, 0, 1], self.world.root),
                True,
            ),
            ParkArmsActionDescription(Arms.BOTH),
            MoveTorsoActionDescription([TorsoState.HIGH]),
            SetGripperActionDescription(Arms.LEFT, GripperStateEnum.OPEN),
            performable,
        )
        with simulated_robot:
            plan.perform()

    def test_pick_up(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        grasp_description = GraspDescription(
            ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
        )
        description = PickUpActionDescription(
            test_world.get_body_by_name("milk.stl"), [Arms.LEFT], [grasp_description]
        )

        plan = SequentialPlan(
            Context.from_world(test_world),
            NavigateActionDescription(
                PoseStamped.from_list([1.7, 1.5, 0], [0, 0, 0, 1], test_world.root),
                True,
            ),
            MoveTorsoActionDescription([TorsoState.HIGH]),
            description,
        )
        with simulated_robot:
            plan.perform()
        self.assertTrue(
            test_world.get_connection(
                test_world.get_body_by_name("l_gripper_tool_frame"),
                test_world.get_body_by_name("milk.stl"),
            )
            is not None
        )

    def test_place(self):
        test_world = deepcopy(self.world)
        test_robot = PR2.from_world(test_world)
        object_description = test_world.get_body_by_name("milk.stl")
        description = PlaceActionDescription(
            object_description,
            PoseStamped.from_list([2.2, 2, 1], [0, 0, 0, 1], test_world.root),
            [Arms.LEFT],
        )
        plan = SequentialPlan(
            Context.from_world(test_world),
            NavigateActionDescription(
                PoseStamped.from_list([1.7, 1.5, 0], [0, 0, 0, 1], test_world.root),
                True,
            ),
            MoveTorsoActionDescription([TorsoState.HIGH]),
            PickUpActionDescription(
                object_description,
                Arms.LEFT,
                GraspDescription(
                    ApproachDirection.FRONT, VerticalAlignment.NoAlignment, False
                ),
            ),
            description,
        )
        with simulated_robot:
            plan.perform()
        with self.assertRaises(rustworkx.NoEdgeBetweenNodes):
            self.assertTrue(
                test_world.get_connection(
                    test_world.get_body_by_name("l_gripper_tool_frame"),
                    test_world.get_body_by_name("milk.stl"),
                )
                is None
            )

    def test_look_at(self):
        description = LookAtAction.description(
            [PoseStamped.from_list([1, 0, 1], frame=self.world.root)]
        )
        self.assertEqual(
            description.resolve().target,
            PoseStamped.from_list([1, 0, 1], frame=self.world.root),
        )
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            # self._test_validate_action_pre_perform(description, LookAtGoalNotReached)
            plan.perform()

    def test_detect(self):
        milk_body = self.world.get_body_by_name("milk.stl")
        self.robot_view.root.parent_connection.origin = (
            TransformationMatrix.from_xyz_rpy(
                1.5, 2, 0, reference_frame=self.world.root
            )
        )

        description = DetectActionDescription(
            technique=DetectionTechnique.TYPES,
            object_sem_annotation=Milk,
        )
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            detected_object = plan.perform()

        self.assertEqual(detected_object[0].name.name, "milk.stl")
        self.assertIs(detected_object[0], milk_body)

    def test_open(self):
        plan = SequentialPlan(
            self.context,
            MoveTorsoActionDescription([TorsoState.HIGH]),
            ParkArmsActionDescription(Arms.BOTH),
            NavigateActionDescription(
                PoseStamped.from_list([1.75, 1.75, 0], [0, 0, 0.5, 1], self.world.root)
            ),
            OpenActionDescription(
                self.world.get_body_by_name("handle_cab10_t"), [Arms.LEFT]
            ),
        )
        with simulated_robot:
            plan.perform()
        self.assertEqual(
            self.world.state[
                self.world.get_degree_of_freedom_by_name(
                    "cabinet10_drawer_top_joint"
                ).name
            ].position,
            0.45,
        )

    def test_close(self):
        self.world.state[
            self.world.get_degree_of_freedom_by_name("cabinet10_drawer_top_joint").name
        ].position = 0.45
        self.world.notify_state_change()
        plan = SequentialPlan(
            self.context,
            MoveTorsoActionDescription([TorsoState.HIGH]),
            ParkArmsActionDescription(Arms.BOTH),
            NavigateActionDescription(
                PoseStamped.from_list([1.75, 1.75, 0], [0, 0, 0.5, 1], self.world.root)
            ),
            CloseActionDescription(
                self.world.get_body_by_name("handle_cab10_t"), [Arms.LEFT]
            ),
        )
        with simulated_robot:
            plan.perform()
        self.assertEqual(
            self.world.state[
                self.world.get_degree_of_freedom_by_name(
                    "cabinet10_drawer_top_joint"
                ).name
            ].position,
            0,
        )

    def test_transport(self):
        description = TransportActionDescription(
            self.world.get_body_by_name("milk.stl"),
            [
                PoseStamped.from_list(
                    [3, 2.2, 0.95], [0.0, 0.0, 1.0, 0.0], self.world.root
                )
            ],
            [Arms.LEFT],
        )
        plan = SequentialPlan(
            self.context, MoveTorsoActionDescription([TorsoState.HIGH]), description
        )
        with simulated_robot:
            plan.perform()
        milk_position = self.world.get_body_by_name("milk.stl").global_pose.to_np()[
            :3, 3
        ]
        dist = np.linalg.norm(milk_position - np.array([3, 2.2, 0.95]))
        self.assertLessEqual(dist, 0.01)

    def test_grasping(self):
        description = GraspingActionDescription(
            self.world.get_body_by_name("milk.stl"), [Arms.RIGHT]
        )
        plan = SequentialPlan(
            self.context,
            NavigateActionDescription(
                PoseStamped.from_list([1.8, 1.8, 0], frame=self.world.root), True
            ),
            description,
        )
        with simulated_robot:
            plan.perform()
        dist = np.linalg.norm(
            self.world.get_body_by_name("milk.stl").global_pose.to_np()[3, :3]
        )
        self.assertTrue(dist < 0.01)

    def test_facing(self):
        with simulated_robot:
            milk_pose = PoseStamped.from_spatial_type(
                self.world.get_body_by_name("milk.stl").global_pose
            )
            plan = SequentialPlan(
                self.context, FaceAtActionDescription(milk_pose, True)
            )
            plan.perform()
            milk_in_robot_frame = self.world.transform(
                self.world.get_body_by_name("milk.stl").global_pose,
                self.robot_view.root,
            )
            milk_in_robot_frame = PoseStamped.from_spatial_type(milk_in_robot_frame)
            self.assertAlmostEqual(milk_in_robot_frame.position.y, 0.0)

    def test_move_tcp_waypoints(self):
        gripper_pose = PoseStamped.from_spatial_type(
            self.world.get_body_by_name("r_gripper_tool_frame").global_pose
        )
        path = []
        for i in range(1, 3):
            new_pose = deepcopy(gripper_pose)
            new_pose.position.z += 0.05 * i
            path.append(new_pose)
        description = MoveTCPWaypointsMotion(path, Arms.RIGHT)
        plan = SequentialPlan(self.context, description)
        with simulated_robot:
            plan.perform()
        gripper_position = PoseStamped.from_spatial_type(
            self.world.get_body_by_name("r_gripper_tool_frame").global_pose
        )
        self.assertAlmostEqual(
            path[-1].position.x, gripper_position.position.x, places=4
        )
        self.assertAlmostEqual(
            path[-1].position.y, gripper_position.position.y, places=4
        )
        self.assertAlmostEqual(
            path[-1].position.z, gripper_position.position.z, places=4
        )

        self.assertAlmostEqual(
            path[-1].orientation.x, gripper_position.orientation.x, places=4
        )
        self.assertAlmostEqual(
            path[-1].orientation.y, gripper_position.orientation.y, places=4
        )
        self.assertAlmostEqual(
            path[-1].orientation.z, gripper_position.orientation.z, places=4
        )
        self.assertAlmostEqual(
            path[-1].orientation.w, gripper_position.orientation.w, places=4
        )

    @unittest.skip
    def test_search_action(self):
        plan = SequentialPlan(
            self.context,
            MoveTorsoActionDescription([TorsoState.HIGH]),
            SearchActionDescription(
                PoseStamped.from_list([2, 2, 1], self.world.root), Milk
            ),
        )
        with simulated_robot:
            milk = plan.perform()
        self.assertTrue(milk)
        self.assertEqual(milk.obj_type, Milk)
        self.assertEqual(self.milk.pose, milk.pose)
