import unittest

import numpy as np
import rospkg

import pycram.task
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
        ProcessModule.execution_delay = False

    def setUp(self):
        self.world.reset_bullet_world()

    # DO NOT WRITE TESTS HERE!!!
    # Test related to the BulletWorld should be written in test_bullet_world.py
    # Tests in here would not be properly executed in the CI

    def tearDown(self):
        self.world.reset_bullet_world()
        pycram.task.reset_tree()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()



