import os
import threading
import time
import unittest
from copy import deepcopy
from datetime import timedelta

import pytest
from rclpy.node import Node
from semantic_world.adapters.mesh import STLParser
from semantic_world.adapters.urdf import URDFParser
from semantic_world.robots import PR2
from semantic_world.spatial_types.spatial_types import TransformationMatrix
from semantic_world.world import World

from .datastructures.enums import WorldMode
from .plan import Plan
from .ros import loginfo, get_node_names
from pycrap.ontologies import Milk, Robot, Kitchen, Cereal
from .robot_descriptions.pr2_states import *
try:
    # from .ros_utils.viz_marker_publisher import VizMarkerPublisher
    from semantic_world.adapters.viz_marker import VizMarkerPublisher
except ImportError:
    loginfo("Could not import VizMarkerPublisher. This is probably because you are not running ROS.")

@pytest.fixture(autouse=True, scope="session")
def cleanup_ros(self):
    """
    Fixture to ensure that ROS is properly cleaned up after all tests.
    """
    yield
    if os.environ.get('ROS_VERSION') == '2':
        import rclpy
        if rclpy.ok():
            rclpy.shutdown()

class SemanticWorldTestCase(unittest.TestCase):
    world: World

    @classmethod
    def setUpClass(cls):
        cls.pr2_sem_world = URDFParser(
            os.path.join(os.path.dirname(__file__), "..", "..", "resources", "robots", "pr2.urdf")).parse()
        cls.apartment_world = URDFParser(
            os.path.join(os.path.dirname(__file__), "..", "..", "resources", "worlds", "apartment.urdf")).parse()
        cls.apartment_world.merge_world(cls.pr2_sem_world)



class EmptyWorldTestCase(unittest.TestCase):
    """
    Base class for unit tests that require and ordinary setup and teardown of the empty bullet-world.
    """

    world: World
    #viz_marker_publisher: VizMarkerPublisher
    render_mode = WorldMode.DIRECT

    @classmethod
    def setUpClass(cls):
        cls.world = World()
        # if "ROS_VERSION" in os.environ:
        #     cls.viz_marker_publisher = VizMarkerPublisher()

    def setUp(self):
        Plan.current_plan = None


    def tearDown(self):
        time.sleep(0.05)


class BulletWorldTestCase(EmptyWorldTestCase):
    """
    Class for unit tests that require a bullet-world with a PR2, kitchen, milk and cereal.
    """

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.pr2_sem_world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "robots", "pr2.urdf")).parse()
        cls.apartment_world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "worlds", "apartment.urdf")).parse()
        cls.milk_world = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "milk.stl")).parse()
        cls.apartment_world.merge_world(cls.pr2_sem_world)
        cls.apartment_world.merge_world(cls.milk_world)

        cls.apartment_world.get_body_by_name("milk.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(2.2, 2, 1, reference_frame=cls.apartment_world.root)

        cls.n = Node("test")
        # cls.viz_marker_publisher = VizMarkerPublisher(cls.apartment_world, n)

        # cls.robot_view = PR2.from_world(cls.apartment_world)

        # cls.context = cls.apartment_world, None

    def setUp(self):
        self.world = deepcopy(self.apartment_world)
        self.context = (self.world, None)
        self.robot_view = PR2.from_world(self.world)
        # viz_marker_publisher = VizMarkerPublisher(self.world, self.n)

    @classmethod
    def tearDownClass(cls):
        pass
        # cls.viz_marker_publisher._stop_publishing()

class BulletWorldGUITestCase(BulletWorldTestCase):
    render_mode = WorldMode.GUI
