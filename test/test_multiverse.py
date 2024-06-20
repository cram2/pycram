#!/usr/bin/env python3
import time
import unittest

import psutil
from typing_extensions import Optional, List

from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.world import UseProspectionWorld
from pycram.world_concepts.world_object import Object
from pycram.worlds.multiverse import Multiverse

multiverse_installed = True
try:
    from pycram.world_concepts.multiverse_socket import MultiverseMetaData, SocketAddress
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
# @unittest.skip("Needs Multiverse server and simulation to be running")
class MultiversePyCRAMTestCase(unittest.TestCase):
    multiverse: Multiverse
    big_bowl: Optional[Object] = None

    @classmethod
    def setUpClass(cls):
        cls.multiverse = Multiverse(simulation="pycram_test",
                                    client_addr=SocketAddress(port="5481"),
                                    is_prospection=True)
        cls.big_bowl = Object("big_bowl", ObjectType.GENERIC_OBJECT, "BigBowl.obj",
                              pose=Pose([2, 2, 1], [0, 0, 0, 1]))
        time.sleep(2)

    @classmethod
    def tearDownClass(cls):
        cls.multiverse.disconnect_from_physics_server()

    def tearDown(self):
        self.multiverse.reset_world()
        # self.multiverse.multiverse_reset_world()
        time.sleep(2)
        pass

    def test_reset_world(self):
        set_position = [1, 1, 0]
        self.big_bowl.set_position(set_position)
        bowl_position = self.big_bowl.get_position_as_list()
        for i in range(3):
            self.assertAlmostEqual(bowl_position[i], set_position[i])
        self.multiverse.reset_world()
        big_bowl_pose = self.big_bowl.get_pose()
        for v1, v2 in zip(big_bowl_pose.position_as_list()[:2], self.big_bowl.original_pose.position_as_list()[:2]):
            self.assertAlmostEqual(v1, v2, delta=0.001)
        for v1, v2 in zip(big_bowl_pose.orientation_as_list(), self.big_bowl.original_pose.orientation_as_list()):
            self.assertAlmostEqual(v1, v2, delta=0.001)

    def test_spawn_object(self):
        milk = self.spawn_milk([1, 0, 0])
        self.assertIsInstance(milk, Object)
        milk_pose = milk.get_pose()
        for v1, v2 in zip(milk_pose.position_as_list()[:2], [1, 0]):
            self.assertAlmostEqual(v1, v2, delta=0.002)
        for v1, v2 in zip(milk_pose.orientation_as_list(), milk.original_pose.orientation_as_list()):
            self.assertAlmostEqual(v1, v2, delta=0.001)

    def test_remove_object(self):
        milk = self.spawn_milk([0, 0, 2])
        milk.remove()

    def test_check_object_exists(self):
        milk = self.spawn_milk([0, 0, 2])
        data = self.multiverse.get_all_objects_data_from_server()
        self.assertTrue(milk.name in data)

    def test_set_position(self):
        milk = self.spawn_milk([0, 0, 2])
        original_milk_position = milk.get_position_as_list()
        original_milk_position[0] += 1
        milk.set_position(original_milk_position)
        milk_position = milk.get_position_as_list()
        self.assertAlmostEqual(milk_position, original_milk_position)

    def test_update_position(self):
        milk = self.spawn_milk([1, 0, 2])
        milk.update_pose()
        milk_position = milk.get_position_as_list()
        for i, v in enumerate([1, 0]):
            self.assertAlmostEqual(milk_position[i], v, delta=0.002)

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
        self.assertAlmostEqual(joint_position, original_joint_position - 1.57)

    def test_spawn_robot(self):
        if self.multiverse.robot is not None:
            robot = self.multiverse.robot
        else:
            robot = self.spawn_robot()
        self.assertIsInstance(robot, Object)
        self.assertTrue(robot in self.multiverse.objects)
        self.assertTrue(self.multiverse.robot.name == robot.name)

    def test_destroy_robot(self):
        if self.multiverse.robot is None:
            self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)
        self.multiverse.robot.remove()
        self.assertTrue(self.multiverse.robot not in self.multiverse.objects)

    @unittest.skip("Not implemented feature yet.")
    def test_respawmn_robot(self):
        self.spawn_robot()
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)
        self.multiverse.robot.remove()
        self.assertTrue(self.multiverse.robot not in self.multiverse.objects)
        self.spawn_robot(position=[0, 0, 1])
        self.assertTrue(self.multiverse.robot in self.multiverse.objects)

    def test_set_robot_position(self):
        if self.multiverse.robot is None:
            self.spawn_robot()
        new_position = [1, 1, 0]
        self.multiverse.robot.set_position(new_position)
        self.assertEqual(self.multiverse.robot.get_position_as_list(), new_position)

    @unittest.skip("Not working yet.")
    def test_attach_object(self):
        milk = self.spawn_milk([1, 0, 1])
        milk.attach(self.big_bowl)
        self.assertTrue(self.big_bowl in milk.attachments)
        milk_position = milk.get_position_as_list()
        milk_position[0] += 1
        big_bowl_position = self.big_bowl.get_position_as_list()
        estimated_bowl_position = big_bowl_position.copy()
        estimated_bowl_position[0] += 1
        milk.set_position(milk_position)
        new_bowl_position = self.big_bowl.get_position_as_list()
        self.assertAlmostEqual(new_bowl_position[0], estimated_bowl_position[0])

    @unittest.skip("Needs mobile base robot")
    def test_attach_with_robot(self):
        with UseProspectionWorld():
            if self.multiverse.robot is None:
                robot = self.spawn_robot()
            robot.attach(self.big_bowl)
            self.assertTrue(self.big_bowl in robot.attachments)
            bowl_position = self.big_bowl.get_position_as_list()
            robot.update_pose()
            robot_position = robot.get_position_as_list()
            robot_position[2] += 3
            robot.set_position(robot_position)
            new_bowl_position = self.big_bowl.get_position_as_list()
            estimated_bowl_position = bowl_position
            estimated_bowl_position[2] += 3
            self.assertAlmostEqual(new_bowl_position[0], estimated_bowl_position[0])

    @staticmethod
    def spawn_milk(position: List) -> Object:
        milk = Object("milk_box", ObjectType.MILK, "milk_box.urdf",
                      pose=Pose(position, [0, 0, 0, 1]))
        time.sleep(2)
        return milk

    @staticmethod
    def spawn_robot(position: Optional[List[float]] = None) -> Object:
        if position is None:
            position = [0, 0, 0]
        robot = Object("panda", ObjectType.ROBOT, "panda.urdf",
                       pose=Pose(position, [0, 0, 0, 1]))
        time.sleep(2)
        return robot
