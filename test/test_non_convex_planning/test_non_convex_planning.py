import unittest

import networkx as nx
from random_events.interval import SimpleInterval
from random_events.product_algebra import SimpleEvent

from pycram.non_convex_planner import ConnectivityGraph, Box
from pycram.testing import BulletWorldTestCase


class ConnectivityGraphTestCase(unittest.TestCase):
    """
    Testcase to test the navigation around a unit box.
    """

    cg: ConnectivityGraph

    @classmethod
    def setUpClass(cls):
        cg = ConnectivityGraph()

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
        #
        # for source, target in zip(path, path[1:]):
        #     intersection: Box = cg[source][target]["intersection"]
        #     x_target = intersection.x_interval.center()
        #     y_target = intersection.y_interval.center()
        #     z_target = intersection.z_interval.center()
        #     print(f"Moving to {x_target}, {y_target}, {z_target}")


class ConnectivityGraphFromWorldTestCase(BulletWorldTestCase):
    """
    Test the application of a connectivity graph to the belief state.
    """

    def test_from_world(self):
        search_space = Box(x=SimpleInterval(-2, 3),
                           y=SimpleInterval(-2, 3),
                           z=SimpleInterval(.45, .55))
        cg = ConnectivityGraph.free_space_from_world(self.world, search_space=search_space)
        self.assertIsNotNone(cg)
        print(cg)


if __name__ == '__main__':
    unittest.main()
