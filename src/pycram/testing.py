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

class EmptyBulletWorldTestCase(unittest.TestCase):
    """
    Base class for unit tests that require and ordinary setup and teardown of the empty bullet-world.
    """

    world: BulletWorld
    viz_marker_publisher: VizMarkerPublisher
    extension: str = ObjectDescription.get_file_extension()
    render_mode = WorldMode.DIRECT

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld(mode=cls.render_mode)
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
        cls.viz_marker_publisher._stop_publishing()
        cls.world.exit()


class BulletWorldTestCase(EmptyBulletWorldTestCase):
    """
    Class for unit tests that require a bullet-world with a PR2, kitchen, milk and cereal.
    """

    world: BulletWorld
    viz_marker_publisher: VizMarkerPublisher
    extension: str = ObjectDescription.get_file_extension()

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rdm = RobotDescriptionManager()
        rdm.load_description("pr2")
        cls.milk = Object("milk", Milk, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.robot = Object(RobotDescription.current_robot_description.name, Robot,
                           RobotDescription.current_robot_description.name + cls.extension)
        cls.kitchen = Object("kitchen", Kitchen, "kitchen" + cls.extension)
        cls.cereal = Object("cereal", Cereal, "breakfast_cereal.stl",
                            pose=Pose([1.3, 0.7, 0.95]))



class BulletWorldGUITestCase(BulletWorldTestCase):
    render_mode = WorldMode.GUI
