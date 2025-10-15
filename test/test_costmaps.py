import unittest
from copy import deepcopy

import numpy as np
from random_events.variable import Continuous
#  import plotly.graph_objects as go
from random_events.product_algebra import Event, SimpleEvent
from random_events.interval import *
from semantic_world.spatial_types import TransformationMatrix

from pycram.testing import BulletWorldTestCase
from pycram.costmaps import OccupancyCostmap, AlgebraicSemanticCostmap, VisibilityCostmap
from pycram.probabilistic_costmap import ProbabilisticCostmap
from pycram.units import meter, centimeter
from pycram.datastructures.pose import PoseStamped
import plotly.graph_objects as go


class CostmapTestCase(BulletWorldTestCase):

    def test_attachment_exclusion(self):
        self.robot_view.root.parent_connection.origin = TransformationMatrix.from_xyz_rpy(-1.5, 1, 0, reference_frame=self.world.root)
        self.world.get_body_by_name("milk.stl").parent_connection.origin = TransformationMatrix.from_xyz_rpy(-1.8, 1, 1, reference_frame=self.world.root)

        test_world = deepcopy(self.world)
        with test_world.modify_world():
            test_world.move_branch(test_world.get_body_by_name("milk.stl"), test_world.get_body_by_name("r_gripper_tool_frame"))
        o = OccupancyCostmap(0.2, size=200, resolution=0.02,
                             origin=PoseStamped.from_list( [-1.5, 1, 0], [0, 0, 0, 1], test_world.root), world=test_world)

        self.assertEqual(400, np.sum(o.map[90:110, 90:110]))

        self.assertTrue(np.sum(o.map[80:90, 90:110]) != 0)

    def test_partition_into_rectangles(self):
        ocm = OccupancyCostmap(distance_to_obstacle=0.2, size=200, resolution=0.02,
                               origin=PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1], self.world.root), world=self.world)
        rectangles = ocm.partitioning_rectangles()
        ocm.visualize()

        x = Continuous("x")
        y = Continuous("y")

        events = []
        for rectangle in rectangles:

            event = SimpleEvent({x: open(rectangle.x_lower, rectangle.x_upper),
                           y: open(rectangle.y_lower, rectangle.y_upper)})
            events.append(event)

        event = Event(*events)
        # fig = go.Figure(event.plot(), event.plotly_layout())
        # fig.update_xaxes(range=[-2, 2])
        # fig.update_yaxes(range=[-2, 2])
        # fig.show()
        self.assertTrue(event.is_disjoint())

    def test_visualize(self):
        o = OccupancyCostmap(0.2, size=200, resolution=0.02,
                             origin=PoseStamped.from_list([0, 0, 0], [0, 0, 0, 1], self.world.root), world=self.world)
        o.visualize()

    def test_merge_costmap(self):
        o = OccupancyCostmap(0.2, size=200, resolution=0.02,
                             origin=PoseStamped.from_list( [0, 0, 0], [0, 0, 0, 1],self.world.root),world=self.world)
        o2 = OccupancyCostmap(0.2, size=200, resolution=0.02,
                              origin=PoseStamped.from_list( [0, 0, 0], [0, 0, 0, 1], self.world.root), world=self.world)
        o3 = o + o2
        self.assertTrue(np.all(o.map == o3.map))
        o2.map[100:120, 100:120] = 0
        o3 = o + o2
        self.assertTrue(np.all(o3.map[100:120, 100:120] == 0))
        self.assertTrue(np.all(o3.map[0:100, 0:100] == o.map[0:100, 0:100]))
        o2.map = np.zeros_like(o2.map)
        o3 = o + o2
        self.assertTrue(np.all(o3.map == o2.map))


class SemanticCostmapTestCase(BulletWorldTestCase):

    def test_generate_map(self):
        costmap = AlgebraicSemanticCostmap(self.world.get_body_by_name("island_countertop"))
        costmap.valid_area = costmap.valid_area.__deepcopy__() & costmap.left()
        costmap.valid_area = costmap.valid_area.__deepcopy__() & costmap.top()
        costmap.valid_area = costmap.valid_area.__deepcopy__() & costmap.border(0.2)
        self.assertEqual(len(costmap.valid_area.simple_sets), 2)

    def test_as_distribution(self):
        costmap = AlgebraicSemanticCostmap(self.world.get_body_by_name("island_countertop"))
        costmap.valid_area = costmap.valid_area.__deepcopy__() & costmap.right() & costmap.bottom() & costmap.border(0.2)
        model = costmap.as_distribution()
        self.assertEqual(len(model.nodes()), 7)
        # fig = go.Figure(model.plot(), model.plotly_layout())
        # fig.show()
        # supp = model.support
        # fig = go.Figure(supp.plot(), supp.plotly_layout())
        # fig.show()

    def test_iterate(self):
        costmap = AlgebraicSemanticCostmap(self.world.get_body_by_name("island_countertop"))
        costmap.valid_area = costmap.valid_area.__deepcopy__() & costmap.left() & costmap.bottom() & costmap.border(0.2)
        for sample in iter(costmap):
            self.assertIsInstance(sample, PoseStamped)
            self.assertTrue(costmap.valid_area.contains([sample.position.x, sample.position.y]))


@unittest.skip("Wait for PM Upgrade to go live")
class ProbabilisticCostmapTestCase(BulletWorldTestCase):



    def setUp(self):
        super().setUp()
        self.origin = PoseStamped.from_list( [1.5, 1, 0], [0, 0, 0, 1], self.world.root)
        self.costmap = ProbabilisticCostmap(self.origin, size = 200*centimeter)

    def test_setup(self):
        event = self.costmap.create_event_from_map()
        self.assertTrue(event.is_disjoint())

    def test_visualization(self):
        fig = go.Figure(self.costmap.distribution.plot(), self.costmap.distribution.plotly_layout())
        self.costmap.visualize()
        # fig = go.Figure(pcm.distribution.plot(surface=True), pcm.distribution.plotly_layout())
        # fig.show()

    def test_visibility_cm(self):
        costmap = ProbabilisticCostmap(self.origin, size = 200*centimeter,
                                       costmap_type=VisibilityCostmap)
        costmap.visualize()

    def test_merge_cm(self):
        visibility = ProbabilisticCostmap(self.origin, size = 200*centimeter,
                                       costmap_type=VisibilityCostmap)
        occupancy = ProbabilisticCostmap(self.origin, size = 200*centimeter)
        occupancy.distribution, _ = occupancy.distribution.conditional(visibility.create_event_from_map())
        occupancy.visualize()
