import time
import unittest

import pycram.task
from pycram.worlds.bullet_world import BulletWorld
from pycram.world_concepts.world_object import Object
from pycram.datastructures.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.process_module import ProcessModule
from pycram.datastructures.enums import ObjectType, WorldMode
from pycram.object_descriptors.urdf import ObjectDescription
from pycram.ros.viz_marker_publisher import VizMarkerPublisher


class BulletWorldTestCase(unittest.TestCase):

    world: BulletWorld
    viz_marker_publisher: VizMarkerPublisher
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=WorldMode.DIRECT)
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(robot_description.name, ObjectType.ROBOT,
                           robot_description.name + cls.extension)
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen" + cls.extension)
        cls.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl",
                            ObjectDescription, pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False
        cls.viz_marker_publisher = VizMarkerPublisher()

    def setUp(self):
        self.world.reset_world()

    # DO NOT WRITE TESTS HERE!!!
    # Test related to the BulletWorld should be written in test_bullet_world.py
    # Tests in here would not be properly executed in the CI

    def tearDown(self):
        pycram.task.reset_tree()
        time.sleep(0.05)
        self.world.reset_world()

    @classmethod
    def tearDownClass(cls):
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()


class BulletWorldGUITestCase(unittest.TestCase):

    world: BulletWorld
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=WorldMode.GUI)
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(robot_description.name, ObjectType.ROBOT,
                           robot_description.name + cls.extension)
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen" + cls.extension)
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
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()

