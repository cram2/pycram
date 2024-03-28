#!/usr/bin/env python3
import time
import unittest
from unittest import skip

from typing_extensions import Optional

from pycram.datastructures.enums import ObjectType
from pycram.datastructures.pose import Pose
from pycram.world_concepts.world_object import Object
from pycram.worlds.multiverse import Multiverse

multiverse_installed = True
try:
    from pycram.world_concepts.multiverse_socket import MultiverseMetaData, SocketAddress
except ImportError:
    multiverse_installed = False


@unittest.skipIf(not multiverse_installed, "Multiverse is not installed.")
# @unittest.skip("Needs Multiverse server and simulation to be running")
class MultiversePyCRAMTestCase(unittest.TestCase):
    multiverse: Multiverse
    big_bowl: Optional[Object] = None
    robot: Optional[Object] = None

    @classmethod
    def setUpClass(cls):
        cls.multiverse = Multiverse(simulation="pycram_test",
                                    client_addr=SocketAddress(port="5481"),
                                    is_prospection=True)
        cls.big_bowl = Object("big_bowl", ObjectType.GENERIC_OBJECT, "BigBowl.obj",
                              pose=Pose([2, 2, 1], [0, 0, 0, 1]))

    @classmethod
    def tearDownClass(cls):
        cls.multiverse.disconnect_from_physics_server()

    def tearDown(self):
        self.multiverse.reset_world()
        # self.multiverse.multiverse_reset_world()
        pass

    def test_reset_world(self):
        self.big_bowl.set_position([1, 1, 0])
        self.assertAlmostEqual(self.big_bowl.get_position_as_list(), [1, 1, 0])
        self.multiverse.reset_world()
        big_bowl_pose = self.big_bowl.get_pose()
        self.assertAlmostEqual(big_bowl_pose, self.big_bowl.original_pose)

    def test_spawn_object(self):
        milk = self.spawn_milk()
        self.assertIsInstance(milk, Object)
        milk_pose = milk.get_pose()
        self.assertAlmostEqual(milk_pose, milk.original_pose)

    def test_remove_object(self):
        milk = self.spawn_milk()
        milk.remove()

    def test_check_object_exists(self):
        milk = self.spawn_milk()
        data = self.multiverse.get_all_objects_data_from_server()
        self.assertTrue(milk.name in data)

    def test_set_position(self):
        milk = self.spawn_milk()
        original_milk_position = milk.get_position_as_list()
        original_milk_position[0] += 1
        milk.set_position(original_milk_position)
        milk_position = milk.get_position_as_list()
        self.assertAlmostEqual(milk_position, original_milk_position)

    def test_update_position(self):
        milk = self.spawn_milk()
        milk.update_pose()
        milk_position = milk.get_position_as_list()
        for i, v in enumerate([0, 0, 2]):
            self.assertAlmostEqual(milk_position[i], v)

    # @skip("Not implemented")
    def test_set_joint_position(self):
        robot = self.spawn_robot()
        original_joint_position = robot.get_joint_position("joint1")
        step = 1.57
        i = 0
        while True:
            robot.set_joint_position("joint1", original_joint_position - step*i)
            robot.joints["joint1"]._update_position()
            joint_position = robot.get_joint_position("joint1")
            if joint_position <= original_joint_position-1.57:
                break
            i += 1
            # time.sleep(0.1)
        self.assertAlmostEqual(joint_position, original_joint_position-1.57)

    def test_destroy_robot(self):
        robot = self.spawn_robot()
        self.assertTrue(robot in self.multiverse.objects)
        robot.remove()
        self.assertTrue(robot not in self.multiverse.objects)

    @skip("Not implemented")
    def test_set_robot_position(self):
        robot = self.spawn_robot()
        self.robot.set_position([0, 0, 1])
        self.assertEqual(self.robot.get_position_as_list(), [0, 0, 1])

    @staticmethod
    def spawn_milk() -> Object:
        return Object("milk_box", ObjectType.MILK, "milk_box.urdf",
                      pose=Pose([0, 0, 2], [0, 0, 0, 1]))

    @staticmethod
    def spawn_robot() -> Object:
        return Object("panda", ObjectType.ROBOT, "panda.urdf",
                      pose=Pose([0, 0, 3], [0, 0, 0, 1]))
