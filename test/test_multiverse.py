#!/usr/bin/env python3

import unittest

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
    wooden_log: Optional[Object] = None
    robot: Optional[Object] = None

    @classmethod
    def setUpClass(cls):
        cls.multiverse = Multiverse(simulation="pycram_test",
                                    client_addr=SocketAddress(port="5481"),
                                    is_prospection=True)
        # cls.wooden_log = Object("wooden_log", ObjectType.GENERIC_OBJECT, "WoodenLog.urdf",
        #                         pose=Pose([0, 0, 0.5], [0, 0, 0, 1]))

    @classmethod
    def tearDownClass(cls):
        cls.multiverse.disconnect_from_physics_server()

    def tearDown(self):
        self.multiverse.reset_world()

    def test_reset_world(self):
        self.multiverse.reset_world()

    def test_spawn_object(self):
        milk = self.spawn_milk()

    def test_remove_object(self):
        milk = self.spawn_milk()
        milk.remove()

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

    def test_get_joint_position(self):
        # self.wooden_log.remove()
        self.spawn_robot()
        # joint_position = self.robot.get_joint_position("joint1")
        # self.assertAlmostEqual(joint_position, 0.0)

    def test_set_joint_position(self):
        self.spawn_robot()
        joint_position = self.robot.get_joint_position("shoulder_pan_joint")
        self.robot.set_joint_position("shoulder_pan_joint", joint_position - 1.0)
        self.robot.joints["shoulder_pan_joint"]._update_position()
        joint_position = self.robot.get_joint_position("shoulder_pan_joint")
        self.assertAlmostEqual(joint_position, -1.0)

    def test_set_robot_position(self):
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
