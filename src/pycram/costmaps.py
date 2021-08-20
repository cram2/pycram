import numpy as np
import pybullet as p
import rospy
import time
from pycram.bullet_world import BulletWorld
from nav_msgs.msg import OccupancyGrid, MapMetaData

import matplotlib.pyplot as plt
from matplotlib import colors



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

    def find_all_non_negativ(self):
        indices = []
        for i in range(0, self.width):
            for j in range(0, self.height):
                if self.map[i][j] >= 0:
                    indices.append((i, j))
        return indices

    def find_all_valid_non_negativ(self):
        non_zero = self.find_all_non_negativ()
        valid = []
        for w, h in non_zero:
            sub = self.map[w-self.distance_obstacle: w+self.distance_obstacle, h-self.distance_obstacle:h+self.distance_obstacle]
            if -1 in sub:
                continue
            else:
                valid.append((w,h))
        return valid

    def _merge_cells(self):
        new_map = np.zeros((int(self.width/2)+1, int(self.height/2)+1))
        new_width = 0
        new_height = 0
        for i in range(0, self.width, 2):
            for j in range(0, self.height, 2):
                sum = self.map[i][j] + self.map[i+1][j] + self.map[i][j+1] + self.map[i+1][j+1]
                avg = sum / 4
                new_map[new_width][new_height] = avg
                new_height += 1
            new_height = 0
            new_width += 1
        self.width = int(self.width/2)
        self.height = int(self.height/2)
        self.map = new_map
        self.resolution *= 2
        self.distance_obstacle = int(self.distance_obstacle/2)


def visualize_costmap(costmap, world):
    cells = []
    valid = costmap.find_all_valid_non_negativ()
    for width, height in valid:
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[costmap.resolution, costmap.resolution, 0.001],
            rgbaColor=[1, 0, 0, 1], visualFramePosition=[width*costmap.resolution, height*costmap.resolution, 0.])
        cells.append(vis)

    map_objs = []
    for part in _chunks(cells, 255):
        link_poses = [[0, 0, 0] for c in part]
        link_orientations =  [[0, 0, 0, 1] for c in part]
        link_masses = [1.0 for c in part]
        link_parent = [0 for c in part]
        link_joints = [p.JOINT_FIXED for c in part]
        link_collision = [-1 for c in part]
        link_joint_axis = [[1, 0, 0] for c in part]
        pose = costmap.origin.position
        base_pose = [pose.x, pose.y, pose.z]
        map_obj = p.createMultiBody(baseVisualShapeIndex=-1, linkVisualShapeIndices=part,
            basePosition=base_pose, baseOrientation=[0, 0, 0, 1], linkPositions=link_poses,
            linkMasses=link_masses, linkOrientations=link_orientations,
            linkInertialFramePositions=link_poses,
            linkInertialFrameOrientations=link_orientations,linkParentIndices=link_parent,
            linkJointTypes=link_joints, linkJointAxis=link_joint_axis,
            linkCollisionShapeIndices=link_collision)
        map_objs.append(map_obj)
    return map_objs

def _coords_to_map(indices, map):
    new_map = np.ones((map.width, map.height))
    for width, height in indices:
        new_map[width][height] = 2
    return new_map

def _chunks(lst, n):
    """Yield successive n-sized chunks from lst."""
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

cmap = colors.ListedColormap(['white', 'black', 'green', 'red', 'blue'])

def plot_grid(data):
    rows = data.shape[0]
    cols = data.shape[1]
    fig, ax = plt.subplots()
    ax.imshow(data, cmap=cmap)
    # draw gridlines
    #ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=1)
    ax.set_xticks(np.arange(0.5, rows, 1));
    ax.set_yticks(np.arange(0.5, cols, 1));
    plt.tick_params(axis='both', labelsize=0, length = 0)
    # fig.set_size_inches((8.5, 11), forward=False)
    #plt.savefig(saveImageName + ".png", dpi=500)
    plt.show()
