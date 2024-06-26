import time
import unittest
from pycram.designators import action_designator, object_designator
from pycram.designators.action_designator import MoveTorsoActionPerformable, PickUpActionPerformable, \
    NavigateActionPerformable, FaceAtPerformable
from pycram.local_transformer import LocalTransformer
from pycram.robot_description import RobotDescription
from pycram.process_module import simulated_robot
from pycram.datastructures.pose import Pose
from pycram.datastructures.enums import ObjectType, Arms, GripperState, Grasp
from bullet_world_testcase import BulletWorldTestCase
import numpy as np


class TestActionDesignatorGrounding(BulletWorldTestCase):
    """Testcase for the grounding methods of action designators."""

    def test_move_torso(self):
        description = action_designator.MoveTorsoAction([0.3])
        # SOMA ontology seems not provide a corresponding concept yet for MoveTorso
        #self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().position, 0.3)
        with simulated_robot:
            description.resolve().perform()
        self.assertEqual(self.world.robot.get_joint_position(RobotDescription.current_robot_description.torso_joint), 0.3)

    def test_set_gripper(self):
        description = action_designator.SetGripperAction([Arms.LEFT], [GripperState.OPEN, GripperState.CLOSE])
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().gripper, Arms.LEFT)
        self.assertEqual(description.ground().motion, GripperState.OPEN)
        self.assertEqual(len(list(iter(description))), 2)
        with simulated_robot:
            description.resolve().perform()
        for joint, state in RobotDescription.current_robot_description.get_arm_chain(Arms.LEFT).get_static_gripper_state(GripperState.OPEN).items():
            self.assertEqual(self.world.robot.get_joint_position(joint), state)

    def test_release(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.ReleaseAction([Arms.LEFT], object_description)
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().gripper, Arms.LEFT)
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_grip(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.GripAction([Arms.LEFT], object_description, [0.5])
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().gripper, Arms.LEFT)
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_park_arms(self):
        description = action_designator.ParkArmsAction([Arms.BOTH])
        self.assertEqual(description.ground().arm, Arms.BOTH)
        self.assertTrue(description.ontology_concept_holders)
        with simulated_robot:
            description.resolve().perform()
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("right", "park").items():
            joint_position = self.world.robot.get_joint_position(joint)
            self.assertEqual(joint_position, pose)
        for joint, pose in RobotDescription.current_robot_description.get_static_joint_chain("left", "park").items():
            self.assertEqual(self.world.robot.get_joint_position(joint), pose)

    def test_navigate(self):
        description = action_designator.NavigateAction([Pose([1, 0, 0], [0, 0, 0, 1])])
        with simulated_robot:
            description.resolve().perform()
        self.assertEqual(description.ground().target_location, Pose([1, 0, 0], [0, 0, 0, 1]))
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(self.robot.get_pose(), Pose([1, 0, 0]))

    def test_pick_up(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PickUpAction(object_description, [Arms.LEFT], [Grasp.FRONT])
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            description.resolve().perform()
        self.assertTrue(object_description.resolve().world_object in self.robot.attachments.keys())

    def test_place(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.PlaceAction(object_description, [Pose([1.3, 1, 0.9], [0, 0, 0, 1])], [Arms.LEFT])
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            NavigateActionPerformable(Pose([0.6, 0.4, 0], [0, 0, 0, 1])).perform()
            MoveTorsoActionPerformable(0.3).perform()
            PickUpActionPerformable(object_description.resolve(), Arms.LEFT, Grasp.FRONT).perform()
            description.resolve().perform()
        self.assertFalse(object_description.resolve().world_object in self.robot.attachments.keys())

    def test_look_at(self):
        description = action_designator.LookAtAction([Pose([1, 0, 1])])
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().target, Pose([1, 0, 1]))
        with simulated_robot:
            description.resolve().perform()
        # TODO: Needs a way to test the approximate looking direction of the robot

    def test_detect(self):
        self.kitchen.set_pose(Pose([10, 10, 0]))
        self.milk.set_pose(Pose([1.5, 0, 1.2]))
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.DetectAction(object_description)
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().object_designator.name, "milk")
        with simulated_robot:
            detected_object = description.resolve().perform()
        self.assertEqual(detected_object.name, "milk")
        self.assertEqual(detected_object.obj_type, ObjectType.MILK)
        self.assertEqual(detected_object.world_object, self.milk)

    # Skipped since open and close work only in the apartment at the moment
    @unittest.skip
    def test_open(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.OpenAction(object_description, [Arms.LEFT])
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().object_designator.name, "milk")

    @unittest.skip
    def test_close(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.CloseAction(object_description, [Arms.LEFT])
        self.assertTrue(description.ontology_concept_holders)
        self.assertEqual(description.ground().object_designator.name, "milk")

    def test_transport(self):
        object_description = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.TransportAction(object_description,
                                                        [Arms.LEFT],
                                                        [Pose([-1.35, 0.78, 0.95],
                                                              [0.0, 0.0, 0.16439898301071468, 0.9863939245479175])])
        self.assertTrue(description.ontology_concept_holders)
        with simulated_robot:
            action_designator.MoveTorsoAction([0.2]).resolve().perform()
            description.resolve().perform()
        self.assertEqual(description.ground().object_designator.name, "milk")
        milk_position = np.array(self.milk.get_pose().position_as_list())
        dist = np.linalg.norm(milk_position - np.array([-1.35, 0.78, 0.95]))
        self.assertTrue(dist < 0.01)

    def test_grasping(self):
        self.milk.set_pose(Pose([-1.4, 1, 1]))
        self.robot.set_pose(Pose([-2.14, 1.06, 0]))
        milk_desig = object_designator.ObjectDesignatorDescription(names=["milk"])
        description = action_designator.GraspingAction([Arms.RIGHT], milk_desig)
        self.assertTrue(description.ontology_concept_holders)
        with simulated_robot:
            description.resolve().perform()
        dist = np.linalg.norm(
            np.array(self.robot.get_link_position_as_list(RobotDescription.current_robot_description.get_arm_chain(Arms.RIGHT).get_tool_frame())) -
            np.array(self.milk.get_position_as_list()))
        self.assertTrue(dist < 0.01)

    def test_facing(self):
        with simulated_robot:
            FaceAtPerformable(self.milk.pose).perform()
            milk_in_robot_frame = LocalTransformer().transform_to_object_frame(self.milk.pose, self.robot)
            self.assertAlmostEqual(milk_in_robot_frame.position.y, 0.)


if __name__ == '__main__':
    unittest.main()
