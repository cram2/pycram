import logging
import os
import threading
import time
import unittest
from copy import deepcopy

import pytest
from rclpy.node import Node
from semantic_digital_twin.adapters.mesh import STLParser
from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.adapters.procthor.procthor_semantic_annotations import Milk
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.utils import rclpy_installed
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import OmniDrive

from .datastructures.dataclasses import Context
from .datastructures.enums import WorldMode
from .plan import Plan
from .robot_descriptions.pr2_states import *

logger = logging.getLogger(__name__)

try:
    from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
except ImportError:
    logger.info("Could not import VizMarkerPublisher. This is probably because you are not running ROS.")


@pytest.fixture(autouse=True, scope="session")
def cleanup_ros():
    """
    Fixture to ensure that ROS is properly cleaned up after all tests.
    """
    if os.environ.get('ROS_VERSION') == '2':
        import rclpy
        if not rclpy.ok():
            rclpy.init()
    yield
    if os.environ.get('ROS_VERSION') == '2':
        if rclpy.ok():
            rclpy.shutdown()

@pytest.fixture(scope="function")
def rclpy_node():
    if not rclpy_installed():
        pytest.skip("ROS not installed")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = rclpy.create_node("test_node")

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True, name="rclpy-executor")
    thread.start()
    time.sleep(0.1)
    try:
        yield node
    finally:
        # Stop executor cleanly and wait for the thread to exit
        executor.shutdown()
        thread.join(timeout=2.0)

        # Remove the node from the executor and destroy it
        # (executor.shutdown() takes care of spinning; add_node is safe to keep as-is)
        node.destroy_node()

        # Shut down the ROS client library
        rclpy.shutdown()


def setup_world() -> World:
    logger.setLevel(logging.DEBUG)

    pr2_sem_world = URDFParser.from_file(os.path.join(os.path.dirname(__file__), "..", "..", "resources", "robots",
                                                      "pr2_calibrated_with_ft.urdf")).parse()
    apartment_world = URDFParser.from_file(
        os.path.join(os.path.dirname(__file__), "..", "..", "resources", "worlds", "apartment.urdf")).parse()
    milk_world = STLParser(
        os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "milk.stl")).parse()
    cereal_world = STLParser(
        os.path.join(os.path.dirname(__file__), "..", "..", "resources", "objects", "breakfast_cereal.stl")).parse()
    # apartment_world.merge_world(pr2_sem_world)
    apartment_world.merge_world(milk_world)
    apartment_world.merge_world(cereal_world)

    with apartment_world.modify_world():
        pr2_root = pr2_sem_world.get_body_by_name("base_footprint")
        apartment_root = apartment_world.root
        c_root_bf = OmniDrive.create_with_dofs(parent=apartment_root, child=pr2_root, world=apartment_world)
        apartment_world.merge_world(pr2_sem_world, c_root_bf)
        c_root_bf.origin = TransformationMatrix.from_xyz_rpy(1.5, 2.5, 0)

    apartment_world.get_body_by_name("milk.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(2.37,
                                                                                                              2, 1.05,
                                                                                                              reference_frame=apartment_world.root)
    apartment_world.get_body_by_name(
        "breakfast_cereal.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(2.37, 1.8, 1.05,
                                                                                             reference_frame=apartment_world.root)
    milk_view = Milk(body=apartment_world.get_body_by_name("milk.stl"))
    with apartment_world.modify_world():
        apartment_world.add_semantic_annotation(milk_view)

    return apartment_world


class SemanticWorldTestCase(unittest.TestCase):
    world: World

    @classmethod
    def setUpClass(cls):
        cls.pr2_sem_world = URDFParser(
            os.path.join(os.path.dirname(__file__), "..", "..", "resources", "robots",
                         "pr2_calibrated_with_ft.urdf")).parse()
        cls.apartment_world = URDFParser(
            os.path.join(os.path.dirname(__file__), "..", "..", "resources", "worlds", "apartment.urdf")).parse()
        cls.apartment_world.merge_world(cls.pr2_sem_world)


class EmptyWorldTestCase(unittest.TestCase):
    """
    Base class for unit tests that require and ordinary setup and teardown of the empty bullet-world.
    """

    world: World
    # viz_marker_publisher: VizMarkerPublisher
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


class ApartmentWorldTestCase(EmptyWorldTestCase):
    """
    Class for unit tests that require a bullet-world with a PR2, kitchen, milk and cereal.
    """

    @classmethod
    def setUpClass(cls):
        super().setUpClass()

        cls.apartment_world = setup_world()

        cls.robot_view = PR2.from_world(cls.apartment_world)

        cls.context = Context(cls.apartment_world, cls.robot_view, None)

        cls.original_state_data = deepcopy(cls.apartment_world.state.data)
        cls.world = cls.apartment_world

    def tearDown(self):
        self.world.state.data = deepcopy(self.original_state_data)
        self.world.notify_state_change()

