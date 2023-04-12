import unittest
from pycram.bullet_world import BulletWorld, Object
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
import pybullet

class BulletWorldTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.i.name, "robot", robot_description.i.name + ".urdf")
        cls.kitchen = Object("kitchen", "environment", "kitchen.urdf")
        cls.milk = Object("milk", "milk", "milk.stl", position=[1.3, 1, 0.9])
        cls.cereal = Object("cereal", "cereal", "breakfast_cereal.stl", position=[1.3, 0.7, 0.95])

    def setUp(cls):
        cls.world.reset_bullet_world()

    def test_object_movement(self):
        self.milk.set_position([0, 1, 1])
        self.assertEqual(list(self.milk.get_position()), [0, 1, 1])

    def tearDown(cls):
        cls.world.reset_bullet_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()
