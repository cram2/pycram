import numpy as np
from random_events.variable import Continuous
#  import plotly.graph_objects as go
from random_events.product_algebra import Event, SimpleEvent
from random_events.interval import *

from bullet_world_testcase import BulletWorldTestCase
from pycram.costmaps import OccupancyCostmap
from pycram.datastructures.pose import Pose


class TestCostmapsCase(BulletWorldTestCase):

    def test_raytest_bug(self):
        for i in range(30):
            o = OccupancyCostmap(distance_to_obstacle=0.2, from_ros=False, size=200, resolution=0.02,
                                 origin=Pose([0, 0, 0], [0, 0, 0, 1]))

    def test_attachment_exclusion(self):
        self.kitchen.set_pose(Pose([50, 50, 0]))
        self.robot.set_pose(Pose([0, 0, 0]))
        self.milk.set_pose(Pose([0.5, 0, 1]))
        self.cereal.set_pose(Pose([50, 50, 0]))
        o = OccupancyCostmap(0.2, from_ros=False, size=200, resolution=0.02,
                             origin=Pose([0, 0, 0], [0, 0, 0, 1]))
        self.assertEqual(np.sum(o.map[115:135, 90:110]), 0)

        self.robot.attach(self.milk)
        self.assertTrue(np.sum(o.map[80:90, 90:110]) != 0)

    def test_partition_into_rectangles(self):
        ocm = OccupancyCostmap(distance_to_obstacle=0.2, from_ros=False, size=200, resolution=0.02,
                               origin=Pose([0, 0, 0], [0, 0, 0, 1]))
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
        o = OccupancyCostmap(0.2, from_ros=False, size=200, resolution=0.02,
                             origin=Pose([0, 0, 0], [0, 0, 0, 1]))
        o.visualize()
