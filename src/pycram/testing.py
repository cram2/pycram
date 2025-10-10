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
from semantic_world.world_description.connections import OmniDrive

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
            os.path.join(os.path.dirname(__file__), "..", "..", "resources", "robots", "pr2_calibrated_with_ft.urdf")).parse()
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
        cls.pr2_sem_world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "robots", "pr2_calibrated_with_ft.urdf")).parse()
        cls.apartment_world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "worlds", "apartment.urdf")).parse()
        cls.milk_world = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "milk.stl")).parse()
        cls.cereal_world = STLParser(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "breakfast_cereal.stl")).parse()
        cls.apartment_world.merge_world(cls.pr2_sem_world)
        cls.apartment_world.merge_world(cls.milk_world)
        cls.apartment_world.merge_world(cls.cereal_world)
        print("parsed worlds")

        with cls.apartment_world.modify_world():
            pr2_root = cls.apartment_world.get_body_by_name("base_footprint")
            apartment_root = cls.apartment_world.root
            cls.apartment_world.remove_connection(pr2_root.parent_connection)
            c_root_bf = OmniDrive(parent=apartment_root, child=pr2_root, _world=cls.apartment_world)
            cls.apartment_world.add_connection(c_root_bf)


        cls.apartment_world.get_body_by_name("milk.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(2.2, 2, 1, reference_frame=cls.apartment_world.root)
        cls.apartment_world.get_body_by_name("breakfast_cereal.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(2.2, 1.8, 1, reference_frame=cls.apartment_world.root)

        cls.n = Node("test")
        print("created node")
        # cls.viz_marker_publisher = VizMarkerPublisher(cls.apartment_world, n)

        cls.robot_view = PR2.from_world(cls.apartment_world)

        cls.context = cls.apartment_world, None

        cls.original_state_data = deepcopy(cls.apartment_world.state.data)
        cls.world = cls.apartment_world
        # cls.original_state_data = cls.apartment_world.state.data.copy()
        print("setup class")

    def tearDown(self):
        print("tear down")
        self.world.state.data = deepcopy(self.original_state_data)
        self.world.notify_state_change()



    @classmethod
    def tearDownClass(cls):
        pass
        # cls.viz_marker_publisher._stop_publishing()

class BulletWorldGUITestCase(BulletWorldTestCase):
    render_mode = WorldMode.GUI
