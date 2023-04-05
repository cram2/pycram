from test_bullet_world import BulletWorldTest
from pycram.costmaps import OccupancyCostmap


class TestCostmaps(BulletWorldTest):

    def test_raytest_bug(self):
        for i in range(300):
            o = OccupancyCostmap(distance_to_obstacle=0.2, from_ros=False, size=200, resolution=0.02, origin=[[0, 0, 0], [0, 0, 0, 1]])
