import os
import time
import unittest

import networkx as nx
from matplotlib import pyplot as plt
from random_events.interval import SimpleInterval
from random_events.product_algebra import SimpleEvent

from pycram.ros import loginfo
from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.dataclasses import BoundingBox
from pycram.failures import PlanFailure
from pycram.graph_of_convex_sets import GraphOfConvexSets, PoseOccupiedError
from pycram.testing import BulletWorldTestCase
import plotly.graph_objects as go

try:
    from pycram.ros_utils.viz_marker_publisher import TrajectoryPublisher
except ImportError:
    loginfo("Could not import TrajectoryPublisher. This is probably because you are not running ROS.")

class GCSTestCase(unittest.TestCase):
    """
    Testcase to test the navigation around a unit box.
    """

    gcs: GraphOfConvexSets

    @classmethod
    def setUpClass(cls):
        gcs = GraphOfConvexSets()

        obstacle = BoundingBox(0, 0, 0, 1, 1, 1)

        z_lim = SimpleInterval(.45, .55)
        x_lim = SimpleInterval(-2, 3)
        y_lim = SimpleInterval(-2, 3)
        limiting_event = SimpleEvent({BoundingBox.z_variable: z_lim,
                                      BoundingBox.x_variable: x_lim,
                                      BoundingBox.y_variable: y_lim})
        obstacles = BoundingBox.from_event(
            ~obstacle.simple_event.as_composite_set() & limiting_event.as_composite_set())
        gcs.add_nodes_from(obstacles)
        gcs.calculate_connectivity()
        cls.gcs = gcs

    def test_reachability(self):
        start_pose = [-1, -1, 0.5]
        target_pose = [2, 2, 0.5]

        start_node = self.gcs.node_of_pose(*start_pose)
        target_node = self.gcs.node_of_pose(*target_pose)

        self.assertIsNotNone(start_node)
        self.assertIsNotNone(target_node)

        path = nx.shortest_path(self.gcs, start_node, target_node)
        self.assertEqual(len(path), 3)

    def test_plot(self):
        free_space_plot = go.Figure(self.gcs.plot_free_space())
        self.assertIsNotNone(free_space_plot)
        occupied_space_plot = go.Figure(self.gcs.plot_occupied_space())
        self.assertIsNotNone(occupied_space_plot)


class GCSFromWorldTestCase(BulletWorldTestCase):
    """
    Test the application of a connectivity graph to the belief state.
    """

    def test_from_world(self):
        search_space = BoundingBox(min_x=-1, max_x=1,
                                   min_y=-1, max_y=1,
                                   min_z=0.1, max_z=1)
        gcs = GraphOfConvexSets.free_space_from_world(self.world, search_space=search_space)
        self.assertIsNotNone(gcs)
        self.assertGreater(len(gcs.nodes), 0)
        self.assertGreater(len(gcs.edges), 0)

        start = PoseStamped.from_list([-0.9, -0.9, 0.4])
        target = PoseStamped.from_list([-0.9, 0.9, 0.9])

        path = gcs.path_from_to(start, target)

        if "ROS_VERSION" in os.environ:
            pub = TrajectoryPublisher()
            pub.visualize_trajectory(path)

        self.assertIsNotNone(path)
        self.assertGreater(len(path), 1)

        with self.assertRaises(PoseOccupiedError):
            start = PoseStamped.from_list([-10, -10, -10])
            target = PoseStamped.from_list([10, 10, 10])
            gcs.path_from_to(start, target)

    def test_navigation_map_from_world(self):
        search_space = BoundingBox(min_x=-1, max_x=1,
                                   min_y=-1, max_y=1,
                                   min_z=0.1, max_z=1)
        gcs = GraphOfConvexSets.navigation_map_from_world(self.world, search_space=search_space)
        self.assertGreater(len(gcs.nodes), 0)
        self.assertGreater(len(gcs.edges), 0)


if __name__ == '__main__':
    unittest.main()
