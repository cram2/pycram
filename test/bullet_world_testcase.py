import unittest

import numpy as np
import rospkg

from pycram.bullet_world import BulletWorld, Object, fix_missing_inertial
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.process_module import ProcessModule
from pycram.enums import ObjectType
import os
import tf


class BulletWorldTestCase(unittest.TestCase):
    world: BulletWorld

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, robot_description.name + ".urdf")
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([1.3, 0.7, 0.95]))
        cls.bigknife = Object("bigknife", ObjectType.BIGKNIFE, "big-knife.stl",
                              pose=Pose([0.9, 0.6, 0.8], [0, 0, 0, -1]))
        cls.cocumber = Object("cocumber", ObjectType.COCUMBER, "cocumber.stl",
                              pose=Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1]))
        ProcessModule.execution_delay = False

    def setUp(self):
        self.world.reset_bullet_world()

    # DO NOT WRITE TESTS HERE!!!
    # Test related to the BulletWorld should be written in test_bullet_world.py
    # Tests in here would not be properly executed in the CI

    def tearDown(self):
        self.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()
