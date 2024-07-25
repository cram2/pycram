#!/usr/bin/env python3
import unittest

import psutil
from typing_extensions import Optional, List

from pycram.datastructures.dataclasses import Color, ContactPointsList, ContactPoint
from pycram.datastructures.enums import ObjectType, Arms
from pycram.datastructures.pose import Pose
from pycram.designators.object_designator import BelieveObject
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.world_concepts.world_object import Object

multiverse_installed = True
try:
    from pycram.worlds.multiverse import Multiverse
    from pycram.worlds.multiverse_communication.socket import SocketAddress
except ImportError:
    multiverse_installed = False

processes = psutil.process_iter()
process_names = [p.name() for p in processes]
multiverse_running = True
mujoco_running = True
if 'multiverse_server' not in process_names:
    multiverse_running = False
if 'mujoco' not in process_names:
    mujoco_running = False


@unittest.skipIf(not multiverse_installed, "Multiverse is not installed.")
@unittest.skipIf(not multiverse_running, "Multiverse server is not running.")
@unittest.skipIf(not mujoco_running, "Mujoco is not running.")
class MultiversePyCRAMTestCase(unittest.TestCase):
    if multiverse_installed:
        multiverse: Multiverse
    big_bowl: Optional[Object] = None

    @classmethod
    def setUpClass(cls):
        if not multiverse_installed:
            return
        cls.multiverse = Multiverse(simulation="pycram_test",
                                    is_prospection=False)
        # cls.big_bowl = cls.spawn_big_bowl()

    @classmethod
    def tearDownClass(cls):
        cls.multiverse.exit()

    def tearDown(self):
        # self.multiverse.multiverse_reset_world()
        self.multiverse.reset_world_and_remove_objects()
        # MultiversePyCRAMTestCase.big_bowl = self.spawn_big_bowl()

    def test_demo(self):
        extension = ObjectDescription.get_file_extension()

        robot = self.spawn_robot(robot_name='tiago_dual', position=[1, 2, 0.01])
        apartment = Object("apartment", ObjectType.ENVIRONMENT, f"apartment{extension}")

        milk = Object("milk_box", ObjectType.MILK, f"milk_box{extension}", pose=Pose([2.5, 2, 1.02]),
                      color=Color(1, 0, 0, 1))
        spoon = Object("soup_spoon", ObjectType.SPOON, f"SoupSpoon.obj", pose=Pose([2.5, 2.5, 1.02]),
                       color=Color(0, 0, 1, 1))
        bowl = Object("big_bowl", ObjectType.BOWL, f"BigBowl.obj", pose=Pose([2.5, 2.2, 1.02]),
                      color=Color(1, 1, 0, 1))
        bowl.attach(spoon)
        bowl.set_position([2.5, 2.3, 1.02])

        pick_pose = Pose([2.7, 2.15, 1])

        robot_desig = BelieveObject(names=["tiago_dual"])
        apartment_desig = BelieveObject(names=["apartment"])

    def test_reset_world(self):
        set_position = [1, 1, 0.1]
        milk = self.spawn_milk(set_position)
        milk.set_position(set_position)
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], set_position[:2])
        self.multiverse.reset_world()
        milk_pose = milk.get_pose()
        self.assert_list_is_equal(milk_pose.position_as_list()[:2],
                                  milk.original_pose.position_as_list()[:2])
        self.assert_list_is_equal(milk_pose.orientation_as_list(),
                                  milk.original_pose.orientation_as_list())

    def test_spawn_object(self):
        milk = self.spawn_milk([1, 1, 0.1])
        self.assertIsInstance(milk, Object)
        milk_pose = milk.get_pose()
        self.assert_list_is_equal(milk_pose.position_as_list()[:2], [1, 1])
        self.assert_list_is_equal(milk_pose.orientation_as_list(), milk.original_pose.orientation_as_list())

    def test_remove_object(self):
        milk = self.spawn_milk([1, 1, 0.1])
        milk.remove()
        self.assertTrue(milk not in self.multiverse.objects)
        self.assertFalse(self.multiverse.check_object_exists_in_multiverse(milk))

    def test_check_object_exists(self):
        milk = self.spawn_milk([1, 1, 0.1])
        self.assertTrue(self.multiverse.check_object_exists_in_multiverse(milk))

    def test_set_position(self):
        milk = self.spawn_milk([1, 1, 0.1])
        original_milk_position = milk.get_position_as_list()
        original_milk_position[0] += 1
        milk.set_position(original_milk_position)
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], original_milk_position[:2])

    def test_update_position(self):
        milk = self.spawn_milk([1, 1, 0.1])
        milk.update_pose()
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], [1, 1])

    def test_set_joint_position(self):
        if self.multiverse.robot is None:
            robot = self.spawn_robot()
        else:
            robot = self.multiverse.robot
        step = 0.2
        for joint in ['torso_lift_joint']:
            original_joint_position = robot.get_joint_position(joint)
            robot.set_joint_position(joint, original_joint_position + step)
            joint_position = robot.get_joint_position(joint)
            self.assertAlmostEqual(joint_position, original_joint_position + step, delta=0.01)

    def test_spawn_robot(self):
        if self.multiverse.robot is not None:
            robot = self.multiverse.robot
        else:
            robot = self.spawn_robot()
        self.assertIsInstance(robot, Object)
        self.assertTrue(robot in self.multiverse.objects)
        self.assertTrue(self.multiverse.robot.name == robot.name)

    # @unittest.skip("Not implemented feature yet.")
    def test_destroy_robot(self):
        if self.multiverse.robot is None:
            self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)
        self.multiverse.robot.remove()
        self.assertTrue(self.multiverse.robot not in self.multiverse.objects)

    def test_respawn_robot(self):
        self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)
        self.multiverse.robot.remove()
        self.assertTrue(self.multiverse.robot not in self.multiverse.objects)
        self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)

    # @unittest.skip("This will cause respawning of the robot.")
    def test_set_robot_position(self):
        for i in range(10):
            self.spawn_robot()
            new_position = [-3, -3, 0.001]
            # self.multiverse.writer.send_multiple_body_data_to_server({"odom_vel_lin_x_joint": {"joint_tvalue": [-4]}})
            self.multiverse.robot.set_position(new_position)
            robot_position = self.multiverse.robot.get_position_as_list()
            self.assert_list_is_equal(robot_position[:2], new_position[:2], delta=0.2)
            self.tearDown()

    def test_attach_object(self):
        milk = self.spawn_milk([1, 0, 0.1])
        cup = self.spawn_cup([1, 1, 0.1])
        milk.attach(cup)
        self.assertTrue(cup in milk.attachments)
        milk_position = milk.get_position_as_list()
        milk_position[0] += 1
        cup_position = cup.get_position_as_list()
        estimated_cup_position = cup_position.copy()
        estimated_cup_position[0] += 1
        milk.set_position(milk_position)
        new_cup_position = cup.get_position_as_list()
        self.assert_list_is_equal(new_cup_position[:2], estimated_cup_position[:2])

    # @unittest.skip("Not implemented feature yet.")
    def test_detach_object(self):
        for i in range(2):
            milk = self.spawn_milk([1, 0, 0.1])
            cup = self.spawn_cup([1, 1, 0.1])
            milk.attach(cup)
            self.assertTrue(cup in milk.attachments)
            milk.detach(cup)
            self.assertTrue(cup not in milk.attachments)
            milk_position = milk.get_position_as_list()
            milk_position[0] += 1
            cup_position = cup.get_position_as_list()
            estimated_cup_position = cup_position.copy()
            milk.set_position(milk_position)
            new_milk_position = milk.get_position_as_list()
            new_cup_position = cup.get_position_as_list()
            self.assert_list_is_equal(new_milk_position[:2], milk_position[:2], 0.005)
            self.assert_list_is_equal(new_cup_position[:2], estimated_cup_position[:2], 0.002)
            self.tearDown()

    def test_attach_with_robot(self):
        milk = self.spawn_milk([1, 1, 0.1])
        robot = self.spawn_robot()
        ee_link = self.multiverse.get_arm_tool_frame_link(Arms.RIGHT)
        # Get position of milk relative to robot end effector
        robot.attach(milk, ee_link.name, coincide_the_objects=True)
        self.assertTrue(robot in milk.attachments)
        milk_initial_pose = milk.root_link.get_pose_wrt_link(ee_link)
        robot_position = robot.get_joint_position("arm_right_2_joint")
        robot_position += 1.57
        robot.set_joint_position("arm_right_2_joint", robot_position)
        milk_pose = milk.root_link.get_pose_wrt_link(ee_link)
        self.assert_poses_are_equal(milk_initial_pose, milk_pose)

    def test_get_object_contact_points(self):
        for i in range(3):
            milk = self.spawn_milk([1, 1, 0], [0, -0.707, 0, 0.707])
            contact_points = self.multiverse.get_object_contact_points(milk)
            self.assertIsInstance(contact_points, ContactPointsList)
            self.assertEqual(len(contact_points), 1)
            self.assertIsInstance(contact_points[0], ContactPoint)
            self.assertTrue(contact_points[0].link_b.object, self.multiverse.floor)
            cup = self.spawn_cup([1, 1, 0.1])
            contact_points = self.multiverse.get_object_contact_points(cup)
            self.assertIsInstance(contact_points, ContactPointsList)
            self.assertEqual(len(contact_points), 1)
            self.assertIsInstance(contact_points[0], ContactPoint)
            self.assertTrue(contact_points[0].link_b.object, milk)
            self.tearDown()

    def test_get_contact_points_between_two_objects(self):
        for i in range(3):
            milk = self.spawn_milk([1, 1, 0.1], [0, -0.707, 0, 0.707])
            cup = self.spawn_cup([1, 1, 0.1])
            contact_points = self.multiverse.get_contact_points_between_two_objects(milk, cup)
            self.assertIsInstance(contact_points, ContactPointsList)
            self.assertEqual(len(contact_points), 1)
            self.assertIsInstance(contact_points[0], ContactPoint)
            self.assertTrue(contact_points[0].link_a.object, milk)
            self.assertTrue(contact_points[0].link_b.object, cup)
            self.tearDown()

    def test_get_one_ray(self):
        milk = self.spawn_milk([1, 1, 0.1])
        intersected_object = self.multiverse.ray_test([1, 2, 0.1], [1, 1.5, 0.1])
        self.assertTrue(intersected_object is None)
        intersected_object = self.multiverse.ray_test([1, 2, 0.1], [1, 1, 0.1])
        self.assertTrue(intersected_object == milk.id)

    def test_get_rays(self):
        milk = self.spawn_milk([1, 1, 0.1])
        intersected_objects = self.multiverse.ray_test_batch([[1, 2, 0.1], [1, 2, 0.1]],
                                                             [[1, 1.5, 0.1], [1, 1, 0.1]])
        self.assertTrue(intersected_objects[0] == -1)
        self.assertTrue(intersected_objects[1] == milk.id)

    @staticmethod
    def spawn_big_bowl() -> Object:
        big_bowl = Object("big_bowl", ObjectType.GENERIC_OBJECT, "BigBowl.obj",
                          pose=Pose([2, 2, 0.1], [0, 0, 0, 1]))
        return big_bowl

    @staticmethod
    def spawn_milk(position: List, orientation: Optional[List] = None) -> Object:
        if orientation is None:
            orientation = [0, 0, 0, 1]
        milk = Object("milk_box", ObjectType.MILK, "milk_box.urdf",
                      pose=Pose(position, orientation))
        return milk

    def spawn_robot(self, position: Optional[List[float]] = None,
                    robot_name: Optional[str] = 'tiago_dual',
                    replace: Optional[bool] = True) -> Object:
        if position is None:
            position = [-2, -2, 0.001]
        if self.multiverse.robot is None or replace:
            if self.multiverse.robot is not None:
                self.multiverse.robot.remove()
            robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}.urdf",
                           pose=Pose(position, [0, 0, 0, 1]))
        else:
            robot = self.multiverse.robot
            robot.set_position(position)
        return robot

    @staticmethod
    def spawn_cup(position: List) -> Object:
        cup = Object("cup", ObjectType.GENERIC_OBJECT, "Cup.obj",
                     pose=Pose(position, [0, 0, 0, 1]))
        return cup

    def assert_poses_are_equal(self, pose1: Pose, pose2: Pose,
                               position_delta: float = 0.002, orientation_delta: float = 0.002):
        self.assert_positon_is_equal(pose1.position_as_list(), pose2.position_as_list(), delta=position_delta)
        self.assert_orientation_is_equal(pose1.orientation_as_list(), pose2.orientation_as_list(), delta=orientation_delta)

    def assert_positon_is_equal(self, position1: List[float], position2: List[float], delta: float = 0.02):
        self.assert_list_is_equal(position1, position2, delta=delta)

    def assert_orientation_is_equal(self, orientation1: List[float], orientation2: List[float], delta: float = 0.01):
        self.assert_list_is_equal(orientation1, orientation2, delta=delta)

    def assert_list_is_equal(self, list1: List, list2: List, delta: float = 0.001):
        for i in range(len(list1)):
            self.assertAlmostEqual(list1[i], list2[i], delta=delta)
