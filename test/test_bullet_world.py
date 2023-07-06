import unittest

from pycram.bullet_world import BulletWorld, Object, fix_missing_inertial
from pycram.robot_descriptions import robot_description
from pycram.process_module import ProcessModule
import os
import xml.etree.ElementTree as ET


class BulletWorldTest(unittest.TestCase):

    world: BulletWorld

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.name, "robot", robot_description.name + ".urdf")
        cls.kitchen = Object("kitchen", "environment", "kitchen.urdf")
        cls.milk = Object("milk", "milk", "milk.stl", position=[1.3, 1, 0.9])
        cls.cereal = Object("cereal", "cereal", "breakfast_cereal.stl", position=[1.3, 0.7, 0.95])
        ProcessModule.execution_delay = False

    def setUp(self):
        self.world.reset_bullet_world()

    @unittest.skip
    def test_object_movement(self):
        self.milk.set_position([0, 1, 1])
        self.assertEqual(list(self.milk.get_position()), [0, 1, 1])

    def tearDown(self):
        self.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()


class XMLTester(unittest.TestCase):

    def setUp(self) -> None:
        self.urdf_string = open(os.path.join("..", "resources", "pr2.urdf"), "r").read()

    def test_inertial(self):
        result = fix_missing_inertial(self.urdf_string)
        resulting_tree = ET.ElementTree(ET.fromstring(result))
        for element in resulting_tree.iter("link"):
            self.assertTrue(len([*element.iter("inertial")]) > 0)
