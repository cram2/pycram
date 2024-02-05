from bullet_world_testcase import BulletWorldTestCase
from pycram.costmaps import OccupancyCostmap
from pycram.pose import Pose
import numpy as np


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
        o = OccupancyCostmap(0.2, from_ros=False, size=200, resolution=0.02, origin=Pose([0, 0, 0], [0, 0, 0, 1]))
        self.assertEqual(np.sum(o.map[115:135, 90:110]), 0)

        self.robot.attach(self.milk)
        self.assertTrue(np.sum(o.map[80:90, 90:110]) != 0)

