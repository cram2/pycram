#!/usr/bin/env python3
import time
import unittest

import psutil
from typing_extensions import Optional, List

from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.world_concepts.world_object import Object

multiverse_installed = True
try:
    from pycram.worlds.multiverse import Multiverse
    from pycram.world_concepts.multiverse_socket import SocketAddress
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
                                    client_addr=SocketAddress(port="5481"),
                                    is_prospection=True)
        # cls.big_bowl = cls.spawn_big_bowl()

    @classmethod
    def tearDownClass(cls):
        cls.multiverse.disconnect_from_physics_server()

    def tearDown(self):
        # self.multiverse.multiverse_reset_world()
        self.multiverse.reset_world_and_remove_objects()
        # MultiversePyCRAMTestCase.big_bowl = self.spawn_big_bowl()

    def test_reset_world(self):
        set_position = [1, 1, 0.1]
        milk = self.spawn_milk(set_position)
        milk.set_position(set_position)
        time.sleep(0.1)
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], set_position[:2])
        self.multiverse.reset_world()
        time.sleep(0.1)
        milk_pose = milk.get_pose()
        self.assert_list_is_equal(milk_pose.position_as_list()[:2],
                                  milk.original_pose.position_as_list()[:2])
        self.assert_list_is_equal(milk_pose.orientation_as_list(),
                                  milk.original_pose.orientation_as_list())

    def test_spawn_object(self):
        milk = self.spawn_milk([1, 0, 0.1])
        self.assertIsInstance(milk, Object)
        milk_pose = milk.get_pose()
        self.assert_list_is_equal(milk_pose.position_as_list()[:2], [1, 0])
        self.assert_list_is_equal(milk_pose.orientation_as_list(), milk.original_pose.orientation_as_list())

    def test_remove_object(self):
        milk = self.spawn_milk([0, 0, 0.1])
        milk.remove()
        self.assertTrue(milk not in self.multiverse.objects)
        self.assertFalse(self.multiverse.check_object_exists_in_multiverse(milk.name))

    def test_check_object_exists(self):
        milk = self.spawn_milk([0, 0, 0.1])
        data = self.multiverse.get_all_objects_data_from_server()
        self.assertTrue(milk.name in data)

    def test_set_position(self):
        milk = self.spawn_milk([0, 0, 0.1])
        original_milk_position = milk.get_position_as_list()
        original_milk_position[0] += 1
        milk.set_position(original_milk_position)
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], original_milk_position[:2])

    def test_update_position(self):
        milk = self.spawn_milk([1, 0, 0.1])
        milk.update_pose()
        milk_position = milk.get_position_as_list()
        self.assert_list_is_equal(milk_position[:2], [1, 0], delta=0.002)

    def test_set_joint_position(self):
        if self.multiverse.robot is None:
            robot = self.spawn_robot()
        else:
            robot = self.multiverse.robot
        original_joint_position = robot.get_joint_position("joint1")
        step = 1.57
        i = 0
        while True:
            robot.set_joint_position("joint1", original_joint_position - step * i)
            robot.joints["joint1"]._update_position()
            joint_position = robot.get_joint_position("joint1")
            if joint_position <= original_joint_position - 1.57:
                break
            i += 1
            # time.sleep(0.1)
        self.assertAlmostEqual(joint_position, original_joint_position - 1.57, delta=0.02)

    # @unittest.skip("Not implemented feature yet.")
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

    @unittest.skip("Not implemented feature yet.")
    def test_respawn_robot(self):
        self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)
        self.multiverse.robot.remove()
        self.assertTrue(self.multiverse.robot not in self.multiverse.objects)
        self.spawn_robot(position=[0, 0, 0.1])
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)

    def test_set_robot_position(self):
        self.spawn_mobile_robot(robot_name='panda_free')
        new_position = [1, 1, 0.1]
        self.multiverse.robot.set_position(new_position)
        robot_position = self.multiverse.robot.get_position_as_list()
        self.assert_positon_is_equal(robot_position, new_position, delta=0.035)

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
        time.sleep(0.1)
        new_cup_position = cup.get_position_as_list()
        self.assertAlmostEqual(new_cup_position[0], estimated_cup_position[0], delta=0.001)

    # @unittest.skip("Not implemented feature yet.")
    def test_detach_object(self):
        for i in range(10):
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
            # time.sleep(0.1)  # TODO: This is a workaround for the issue that the position is not updated immediately.
            new_milk_position = milk.get_position_as_list()
            new_cup_position = cup.get_position_as_list()
            self.assertAlmostEqual(new_milk_position[0], milk_position[0], delta=0.001)
            self.assertAlmostEqual(new_cup_position[0], estimated_cup_position[0], delta=0.001)
            cup.remove()
            milk.remove()
            self.tearDown()

    def test_attach_with_robot(self):
        milk = self.spawn_milk([1, 0, 0.1])
        robot = self.spawn_robot()
        # Get position of milk relative to robot end effector
        milk_initial_pose = milk.root_link.get_pose_wrt_link(robot.tip_link)
        robot.attach(milk, robot.tip_link_name)
        self.assertTrue(robot in milk.attachments)
        robot_position = robot.get_joint_position("joint1")
        robot_position += 1.57
        robot.set_joint_position("joint1", robot_position)
        time.sleep(0.1)
        milk_pose = milk.root_link.get_pose_wrt_link(robot.tip_link)
        self.assert_poses_are_equal(milk_initial_pose, milk_pose)

    @unittest.skip("Not implemented feature yet.")
    def test_get_object_contact_points(self):
        milk = self.spawn_milk([1, 0, 0.1])
        # time.sleep(1)
        contact_points = self.multiverse.get_object_contact_points(milk)
        self.assertIsInstance(contact_points, list)
        self.assertEqual(len(contact_points), 1)
        self.assertTrue(contact_points[0].link_b.object, self.multiverse.floor)
        big_bowl_position = self.big_bowl.get_position_as_list()
        milk.set_position([big_bowl_position[0], big_bowl_position[1], big_bowl_position[2] + 0.5])
        # time.sleep(0.5)
        contact_points = self.multiverse.get_object_contact_points(milk)
        self.assertIsInstance(contact_points, list)
        self.assertEqual(len(contact_points), 1)
        self.assertTrue(contact_points[0].link_b.object, self.big_bowl)

    @staticmethod
    def spawn_big_bowl() -> Object:
        big_bowl = Object("big_bowl", ObjectType.GENERIC_OBJECT, "BigBowl.obj",
                          pose=Pose([2, 2, 0.1], [0, 0, 0, 1]))
        # time.sleep(0.1)
        return big_bowl

    @staticmethod
    def spawn_milk(position: List) -> Object:
        milk = Object("milk_box", ObjectType.MILK, "milk_box.urdf",
                      pose=Pose(position, [0, 0, 0, 1]))
        # time.sleep(0.5)
        return milk

    def spawn_mobile_robot(self, position: Optional[List[float]] = None, robot_name: Optional[str] = 'tiago_dual') -> Object:
        self.spawn_robot(position, robot_name, replace=True)

    def spawn_robot(self, position: Optional[List[float]] = None,
                    robot_name: Optional[str] = 'panda',
                    replace: Optional[bool] = False) -> Object:
        if position is None:
            position = [0, 0, 0.1]
        if self.multiverse.robot is None or replace:
            if self.multiverse.robot is not None:
                self.multiverse.robot.remove()
            robot = Object(robot_name, ObjectType.ROBOT, f"{robot_name}.urdf",
                           pose=Pose(position, [0, 0, 0, 1]))
        else:
            robot = self.multiverse.robot
            robot.set_position(position)
        # time.sleep(0.5)
        return robot

    @staticmethod
    def spawn_cup(position: List) -> Object:
        cup = Object("cup", ObjectType.GENERIC_OBJECT, "Cup.obj",
                     pose=Pose(position, [0, 0, 0, 1]))
        # time.sleep(0.5)
        return cup

    def assert_poses_are_equal(self, pose1: Pose, pose2: Pose,
                               position_delta: float = 0.02, orientation_delta: float = 0.01):
        self.assert_positon_is_equal(pose1.position_as_list(), pose2.position_as_list(), delta=position_delta)
        self.assert_orientation_is_equal(pose1.orientation_as_list(), pose2.orientation_as_list(), delta=orientation_delta)

    def assert_positon_is_equal(self, position1: List[float], position2: List[float], delta: float = 0.02):
        self.assert_list_is_equal(position1, position2, delta=delta)

    def assert_orientation_is_equal(self, orientation1: List[float], orientation2: List[float], delta: float = 0.01):
        self.assert_list_is_equal(orientation1, orientation2, delta=delta)

    def assert_list_is_equal(self, list1: List, list2: List, delta: float = 0.02):
        for i in range(len(list1)):
            self.assertAlmostEqual(list1[i], list2[i], delta=delta)
