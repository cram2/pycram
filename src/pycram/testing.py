import time
import unittest

from .tasktree import task_tree
from .datastructures.world import UseProspectionWorld
from .worlds.bullet_world import BulletWorld
from .world_concepts.world_object import Object
from .datastructures.pose import Pose
from .robot_description import RobotDescription, RobotDescriptionManager
from .process_module import ProcessModule
from .datastructures.enums import ObjectType, WorldMode
from .object_descriptors.urdf import ObjectDescription
from .ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycrap import ontology, Milk, Robot, Kitchen, Cereal
import owlready2


class BulletWorldTestCase(unittest.TestCase):
    """
    Base class for unit tests that require and ordinary setup and teardown of the bullet-world.
    It spawns a bullet world in direct mode with a kitchen, milk and cereal.
    """

    world: BulletWorld
    viz_marker_publisher: VizMarkerPublisher
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        rdm = RobotDescriptionManager()
        rdm.load_description("pr2")
        cls.world = BulletWorld(mode=WorldMode.DIRECT)
        cls.milk = Object("milk", Milk, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(RobotDescription.current_robot_description.name, Robot,
                           RobotDescription.current_robot_description.name + cls.extension)
        cls.kitchen = Object("kitchen", Kitchen, "kitchen" + cls.extension)
        cls.cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                            pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False
        cls.viz_marker_publisher = VizMarkerPublisher()


    def setUp(self):
        self.world.reset_world(remove_saved_states=True)
        with UseProspectionWorld():
            pass


    def tearDown(self):
        task_tree.reset_tree()
        time.sleep(0.05)
        self.world.reset_world(remove_saved_states=True)
        with UseProspectionWorld():
            pass

    @classmethod
    def tearDownClass(cls):
        cls.world.ontology.destroy_individuals()
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()


class BulletWorldGUITestCase(unittest.TestCase):

    world: BulletWorld
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        for individual in ontology.individuals():
            owlready2.destroy_entity(individual)
        rdm = RobotDescriptionManager()
        rdm.load_description("pr2")
        cls.world = BulletWorld(mode=WorldMode.GUI)
        cls.milk = Object("milk", Milk, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(RobotDescription.current_robot_description.name, Robot,
                           RobotDescription.current_robot_description.name + cls.extension)
        cls.kitchen = Object("kitchen", Kitchen, "kitchen" + cls.extension)
        cls.cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                            pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False
        cls.viz_marker_publisher = VizMarkerPublisher()

    def setUp(self):
        self.world.reset_world()

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        for individual in ontology.individuals():
            owlready2.destroy_entity(individual)
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()

