import numpy as np
import rospy
import time
from pycram.bullet_world import BulletWorld
from nav_msgs.msg import OccupancyGrid, MapMetaData


class OccupancyMap:

    def __init__(self, distance_to_obstacle):
        meta = self._get_map_metadata()
        self.resolution = meta.resolution
        self.width = meta.width
        self.height = meta.height
        self.origin = meta.origin
        self.map = np.reshape(self._get_map(), (self.width, self.height))
        # Nunber of cells that have to be between a valid cell and an obstacle
        self.distance_obstacle = int(distance_to_obstacle / self.resolution)

    @staticmethod
    def _get_map():
        print("Waiting for Map")
        map = rospy.wait_for_message("/map",  OccupancyGrid)
        print("Recived Map")
        return np.array(map.data)

    @staticmethod
    def _get_map_metadata():
        print("Waiting for Map Meta Data")
        meta = rospy.wait_for_message("/map_metadata", MapMetaData)
        print("Recived Meta Data")
        return meta

    def find_all_non_zero(self):
        indices = []
        for i in range(0, self.width):
            for j in range(0, self.height):
                if self.map[i][j] > 0:
                    indices.append((i, j))
        return indices

    def find_all_valid_non_zero(self):
        non_zero = self.find_all_non_zero()
        valid = []
        for w, h in non_zero:
            width = [1 for x in self.map[w][h-self.distance_obstacle:h+self.distance_obstacle] if x > 0]
            if sum(width) != 2* self.distance_obstacle:
                continue
            valid.append((w, h))
        return valid
