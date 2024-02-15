import time
import unittest

from pycram.bullet_world import BulletWorld
from pycram.world_object import Object
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.process_module import ProcessModule
from pycram.enums import ObjectType, WorldMode
from pycram.urdf_interface import ObjectDescription


class BulletWorldTestCase(unittest.TestCase):

    world: BulletWorld
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=WorldMode.DIRECT)
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", ObjectDescription, pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(robot_description.name, ObjectType.ROBOT,
                           robot_description.name + cls.extension, ObjectDescription)
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen" + cls.extension, ObjectDescription)
        cls.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                            ObjectDescription, pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False

    def setUp(self):
        self.world.reset_world()

    # DO NOT WRITE TESTS HERE!!!
    # Test related to the BulletWorld should be written in test_bullet_world.py
    # Tests in here would not be properly executed in the CI

    def tearDown(self):
        time.sleep(0.2)
        self.world.reset_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()


class BulletWorldGUITestCase(unittest.TestCase):

    world: BulletWorld
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=WorldMode.GUI)
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", ObjectDescription, pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(robot_description.name, ObjectType.ROBOT,
                           robot_description.name + cls.extension, ObjectDescription)
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen" + cls.extension, ObjectDescription)
        cls.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                            ObjectDescription, pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False

    def setUp(self):
        self.world.reset_world()

    # DO NOT WRITE TESTS HERE!!!
    # Test related to the BulletWorld should be written in test_bullet_world.py
    # Tests in here would not be properly executed in the CI

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()
