import unittest

import pycram.task
from pycram.bullet_world import BulletWorld, Object
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.process_module import ProcessModule
from pycram.enums import ObjectType
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.ontology import OntologyManager, SOMA_ONTOLOGY_IRI

class BulletWorldTestCase(unittest.TestCase):

    world: BulletWorld
    viz_marker_publisher: VizMarkerPublisher

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, robot_description.name + ".urdf")
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False
        cls.viz_marker_publisher = VizMarkerPublisher()
        OntologyManager(SOMA_ONTOLOGY_IRI)

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
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()

