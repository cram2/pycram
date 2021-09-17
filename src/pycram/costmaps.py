import numpy as np
import pybullet as p
import rospy
import time
from .bullet_world import BulletWorld
from .bullet_world_reasoning import _get_images_for_target
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

class VisibilityCostmap():

    def __init__(self, location, resolution, min_height, max_height, map_size=100):
        #self.object = object
        self.location = location
        self.map = np.zeros((map_size, map_size))
        self.size = map_size
        self.resolution = resolution
        # for pr2 = 1.27
        self.max_height = max_height
        #for pr2 = 1.6
        self.min_height = min_height
        self.origin = [location,  [0, 0, 0, 1]]


    def _create_images(self):
        #object_pose = self.object.get_position_and_orientation()
        im_world = self._create_image_world()
        images = []
        camera_pose = [self.location, [0, 0, 0, 1]]

        images.append(_get_images_for_target([[self.location[0], self.location[1] +1, self.location[2]], [0, 0, 0, 1]],camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.location[0] -1, self.location[1], self.location[2]], [0, 0, 0, 1]], camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.location[0], self.location[1] -1, self.location[2]], [0, 0, 0, 1]],camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.location[0] +1, self.location[1], self.location[2]], [0, 0, 0, 1]], camera_pose, im_world, size=self.size )[1])

        # images [0] = depth, [1] = seg_mask
        im_world.exit()
        for i in range(0, 4):
            images[i] = self._depth_buffer_to_meter(images[i])
        return images

    def _depth_buffer_to_meter(self, buffer):
        near = 0.2
        far = 100
        return far * near / (far - (far-near)*buffer)

    def _create_image_world(self):
        world = BulletWorld.current_bullet_world.copy()
        for obj in world.objects:
            if BulletWorld.robot != None and obj.name == BulletWorld.robot.name \
                and obj.type == BulletWorld.robot.type:
                obj.remove
            if obj.get_position() == self.location:
                obj.remove
        return world

    def _choose_image(self, index):
        # Because the (0, 0) is in the middle of the map, this returns the angele
        # between the origin and the given index
        angle = np.arctan2(index[0], index[1]) + np.pi
        # return of np.arctan2 is between 2pi and pi
        if angle <= np.pi * 0.25 or angle >= np.pi * 1.75:
            return 0
        elif angle >= np.pi * 1.25 and angle < np.pi * 1.75:
            return 1
        elif angle >= np.pi * 0.75 and angle < np.pi * 1.25:
            return 2
        elif angle >= np.pi * 0.25 and angle < np.pi * 0.75:
            return 3

    def _choose_column(self, index):
        index_in_depth = [index[1], self.size - index[0]]
        column = (index_in_depth[0] / index_in_depth[1]) * (self.size / 2)
        column += self.size/2
        return round(column)

    def _compute_column_range(self, index, min_height, max_height):
        #obj_z = self.object.get_position()[2]
        height = self.location[2]
        distance = np.linalg.norm(index)
        if distance == 0:
            return 0, 0
        r_min = np.arctan((min_height-height) / distance) * self.size
        r_max = np.arctan((max_height-height) / distance) * self.size
        r_min += self.size/2
        r_max += self.size/2
        return min(round(r_min), self.size-1), min(round(r_max), self.size-1)

    def _generate_map(self):
        """
        This method generates the resulting density map by using the algorithm explained
        in Lorenz Mösenlechners PhD thesis: https://mediatum.ub.tum.de/doc/1239461/1239461.pdf p.178
        The resulting density map is then saved to self.map
        """
        depth_imgs = self._create_images()
        for x in range(int(-self.size/2), int(self.size/2)):
            for y in range(int(-self.size/2), int(self.size/2)):
                max_value = 0
                depth_index = self._choose_image([x, y])
                c = self._choose_column([x, y])
                d = np.linalg.norm([x, y])
                r_min, r_max = self._compute_column_range([x, y], self.min_height, self.max_height)
                v = 0
                for r in range(r_min, r_max+1):
                    if depth_imgs[depth_index][c][r] > d:
                        v += 1
                        max_value += 1

                if max_value > 0:
                    x_i = x + int(self.size/2)
                    y_i = y + int(self.size/2)
                    self.map[x_i][y_i] = v / max_value




def visualize_costmap(costmap, world):
    cells = []
    #valid = costmap.find_all_valid_non_negativ()
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
