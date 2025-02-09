import time
import unittest

import networkx as nx
from matplotlib import pyplot as plt
from random_events.interval import SimpleInterval
from random_events.product_algebra import SimpleEvent

from pycram.datastructures.pose import Pose
from pycram.failures import PlanFailure
from pycram.graph_of_convex_sets import GraphOfConvexSets, Box, PoseOccupiedError
from pycram.ros_utils.viz_marker_publisher import TrajectoryPublisher
from pycram.testing import BulletWorldTestCase
import plotly.graph_objects as go


class GCSTestCase(unittest.TestCase):
    """
    Testcase to test the navigation around a unit box.
    """

    cg: GraphOfConvexSets

    @classmethod
    def setUpClass(cls):
        cg = GraphOfConvexSets()

        obstacle = Box()

        z_lim = SimpleInterval(.45, .55)
        x_lim = SimpleInterval(-2, 3)
        y_lim = SimpleInterval(-2, 3)
        limiting_event = SimpleEvent({Box.z: z_lim, Box.x: x_lim, Box.y: y_lim})
        obstacles = Box.from_event(~obstacle.simple_event.as_composite_set() & limiting_event.as_composite_set())
        cg.add_nodes_from(obstacles)
        cg.calculate_connectivity()
        cls.cg = cg

    def test_reachability(self):

        start_pose = [-1, -1, 0.5]
        target_pose = [2, 2, 0.5]

        start_node = self.cg.node_of_pose(*start_pose)
        target_node = self.cg.node_of_pose(*target_pose)

        self.assertIsNotNone(start_node)
        self.assertIsNotNone(target_node)

        path = nx.shortest_path(self.cg, start_node, target_node)
        self.assertEqual(len(path), 3)

    def test_plot(self):
        free_space_plot = go.Figure(self.cg.plot_free_space())
        self.assertIsNotNone(free_space_plot)
        occupied_space_plot = go.Figure(self.cg.plot_occupied_space())
        self.assertIsNotNone(occupied_space_plot)


class GCSFromWorldTestCase(BulletWorldTestCase):
    """
    Test the application of a connectivity graph to the belief state.
    """

    def test_from_world(self):
        search_space = Box(x=SimpleInterval(-1, 1),
                           y=SimpleInterval(-1, 1),
                           z=SimpleInterval(0.1, 1.))
        cg = GraphOfConvexSets.free_space_from_world(self.world, search_space=search_space)
        self.assertIsNotNone(cg)
        self.assertGreater(len(cg.nodes), 0)
        self.assertGreater(len(cg.edges), 0)

        start = Pose([-0.9, -0.9, 0.4])
        target = Pose([-0.9, 0.9, 0.9])

        path = cg.path_from_to(start, target)


        pub = TrajectoryPublisher()
        pub.visualize_trajectory(path)

        self.assertIsNotNone(path)
        self.assertGreater(len(path), 1)

        with self.assertRaises(PoseOccupiedError):
            start = Pose([-10, -10, -10])
            target = Pose([10, 10, 10])
            cg.path_from_to(start, target)

    def test_navigation_map_from_world(self):
        ...



if __name__ == '__main__':
    unittest.main()
