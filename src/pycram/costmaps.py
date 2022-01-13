import numpy as np
import pybullet as p
import rospy
import time
from .bullet_world import BulletWorld
from .bullet_world_reasoning import _get_images_for_target
from nav_msgs.msg import OccupancyGrid, MapMetaData

import matplotlib.pyplot as plt
from matplotlib import colors

class Costmap():
    """
    The base class of all Costmaps which implemnets the visualization of costmaps
    in the BulletWorld.
    """
    def __init__(self, resolution, height, width, origin, map):
        """
        The constaructor of the base class of all Costmaps.
        :param resolution: The distance in metre in the real world which is represented
        by a single entry in the cotsmap.
        :param height: The height of the costmap.
        :param width: The width of the costmap.
        :param origin: The origin of the costmap, in world coordinate frame. The
        origin of the costmap is located in the centre of the costmap, the format
        of the origin is a list of x,y,z.
        :param map: The costmap represents as a 2D numpy array.
        """
        self.resolution = resolution
        self.height = height
        self.width = width
        self.origin = origin
        self.map = map
        self.vis_ids = []

    def visualize(self):
        """
        Visualizes a costmap in the BulletWorld, the visualisation works by
        subdividing the costmap in rectangles which are then visualized as pybullet
        visual shapes.
        :return: The pybullet unique body id.
        """
        if self.vis_ids != []:
            return

        # working on a copy of the costmap, since found rectangles are deleted
        map = np.copy(self.map)
        curr_width = 0
        curr_height = 0
        curr_pose = []
        boxes = []
        # Finding all rectangles in the costmap
        for i in range(0, map.shape[0]):
            for j in range(0, map.shape[0]):
                if map[i][j] > 0:
                    curr_width = self._find_consectuive_line((i, j), map)
                    curr_pose = (i, j)
                    curr_height = self._find_max_box_height((i,j), curr_width, map)
                    boxes.append([curr_pose, curr_height, curr_width])
                    map[i:i+curr_height, j:j+curr_width] = 0
        cells = []
        # Creation of the visual shapes, for documentation of the visual shapes
        # please look here: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.q1gn7v6o58bf
        for box in boxes:
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[(box[1] *self.resolution) / 2, (box[2]*self.resolution) / 2, 0.001],
                rgbaColor=[1, 0, 0, 0.6], visualFramePosition=[(box[0][0] + box[1]/2)*self.resolution, (box[0][1] + box[2]/2)*self.resolution, 0.])
            cells.append(visual)

        for cell_parts in self._chunks(cells, 255):
            # Dummy paramater since these are needed to spawn visual shapes as a
            # multibody.
            link_poses = [[0, 0, 0] for c in cell_parts]
            link_orientations =  [[0, 0, 0, 1] for c in cell_parts]
            link_masses = [1.0 for c in cell_parts]
            link_parent = [0 for c in cell_parts]
            link_joints = [p.JOINT_FIXED for c in cell_parts]
            link_collision = [-1 for c in cell_parts]
            link_joint_axis = [[1, 0, 0] for c in cell_parts]
            # The position at which the multibody will be spawned. Offset such that
            # the origin referes to the centre of the costmap.
            base_pose = [self.origin[0] + self.height/2*self.resolution, self.origin[1] + self.width / 2*self.resolution, self.origin[2]]
            map_obj = p.createMultiBody(baseVisualShapeIndex=-1, linkVisualShapeIndices=cell_parts,
                basePosition=base_pose, baseOrientation=[0, 0, 1, 0], linkPositions=link_poses,
                linkMasses=link_masses, linkOrientations=link_orientations,
                linkInertialFramePositions=link_poses,
                linkInertialFrameOrientations=link_orientations,linkParentIndices=link_parent,
                linkJointTypes=link_joints, linkJointAxis=link_joint_axis,
                linkCollisionShapeIndices=link_collision)
            self.vis_ids.append(map_obj)


    def _chunks(self, lst, n):
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]

    def close_visualization(self):
        for id in self.vis_ids:
            p.removeBody(id)
        self.vis_ids =  []

    def _find_consectuive_line(self, start, map):
        """
        Finds the number of consectuive entrys in the costmap which are greater
        than zero.
        :param start: The indicies in the costmap from which a the consectuive line
        should be found.
        :param map: The costmap in which the line should be found.
        :return: The lenght of the consectuive line of entrys greater than zero.
        """
        width = map.shape[1]
        lenght = 0
        for i in range(start[1], width):
            if map[start[0]][i] > 0:
                lenght += 1
            else:
                return lenght
        return lenght

    def _find_max_box_height(self, start, lenght, map):
        """
        Finds the maximal height for a rectangle for a given width in a costmap.
        The method traverses one row at a time and checks if all entrys for the
        given width are greater than zero. If a entry is less or equal than zero
        the height is returned.
        :param start: The indices in the cotsmap from which the method should start.
        :param length: The given width for a a portion of a row which should be checked.
        :param map: The costmap in which should be searched.
        :return: The height of the rectangle.
        """
        height, width = map.shape
        curr_height = 0
        for i in range(start[0], height):
            for j in range(start[1], start[1] + lenght):
                if map[i][j] <= 0:
                    return curr_height
            curr_height += 1
        return curr_height


class OccupancyMap(Costmap):
    """
    The occupancy Costmap represents a map of the environment where obstacles or
    positions which are inaccessiable for a robot have a value of -1.
    """
    def __init__(self, distance_to_obstacle):
        """
        The constructor for the Occupancy costmap, the actual costmap is received
        from the ROS map_server and wrapped by this class. Meta data about the
        costmap is also received from the map_server.
        :param distance_obstacle: The distance by which the obstacles should be
        inflated. Meaning that obstacles in the costmap are growing bigger by this
        distance.
        """
        meta = self._get_map_metadata()
        Costmap.__init__(self, meta.resolution, meta.height, meta.width, [meta.origin.x, meta.origin.y, meta.origin.z],
                    np.reshape(self._get_map(), (self.width, self.height)))
        # Nunber of cells that have to be between a valid cell and an obstacle
        self.distance_obstacle = int(distance_to_obstacle / self.resolution)

    @staticmethod
    def _get_map():
        """
        Receives the map array from the map_server converts it into a numpy array.
        :return: The costmap as a numpy array.
        """
        print("Waiting for Map")
        map = rospy.wait_for_message("/map",  OccupancyGrid)
        print("Recived Map")
        return np.array(map.data)

    @staticmethod
    def _get_map_metadata():
        """
        Receives the meta data about the costmap from the map_server and returns it.
        The meta data contains things like, height, width, origin and resolution.
        :return The meta data for the costmap array.
        """
        print("Waiting for Map Meta Data")
        meta = rospy.wait_for_message("/map_metadata", MapMetaData)
        print("Recived Meta Data")
        return meta

    def find_all_non_negativ(self):
        """
        Finds and returns all indicies for entry in the array which are greater
        than zero.
        :return: A list of tuple which are entries that are greater than zero.
        """
        indices = []
        for i in range(0, self.width):
            for j in range(0, self.height):
                if self.map[i][j] >= 0:
                    indices.append((i, j))
        return indices

    def find_all_valid_non_negativ(self):
        """
        Finds all entries in the costmap that are greater than zero and are further
        than distance_to_obstacle from any obstacle.
        :return: A list of tuple which represent entries in the costmap which are
        further than distance_to_obstacle from any obstacle.
        """
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

class VisibilityCostmap(Costmap):
    """
    A costmap that represents the visibility of a specific point for every position around
    this point. For a detailed explanation on how the creation of the costmap works
    please look here: https://mediatum.ub.tum.de/doc/1239461/1239461.pdf (page 173)
    """

    def __init__(self, location, resolution, min_height, max_height, map_size=100, world=None):
        """
        The constructor of the visibility costmap which assisgs the given paranmeter
        and triggeres the generation of the costmap.
        """
        #self.object = object
        self.world = world
        self.location = location
        self.map = np.zeros((map_size, map_size))
        self.size = map_size
        self.resolution = resolution
        # for pr2 = 1.27
        self.max_height = max_height
        #for pr2 = 1.6
        self.min_height = min_height
        self.origin = [location,  [0, 0, 0, 1]]
        self._generate_map()
        Costmap.__init__(self, resolution, map_size, map_size, location, self.map)


    def _create_images(self):
        """
        This method creates four depth images in every direction around the point
        for which the costmap should be created. The depoth images are converted
        to metre, meaning that every entry in the depth images represents the
        distance to the next object in metre.
        :return: A list of four depth images, the images are represented as 2D arrays.
        """
        #object_pose = self.object.get_position_and_orientation()
        im_world = self._create_image_world()
        images = []
        camera_pose = [self.location, [0, 0, 0, 1]]
        im_world = self.world

        images.append(_get_images_for_target([[self.location[0], self.location[1] +1, self.location[2]], [0, 0, 0, 1]],camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.location[0] -1, self.location[1], self.location[2]], [0, 0, 0, 1]], camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.location[0], self.location[1] -1, self.location[2]], [0, 0, 0, 1]],camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.location[0] +1, self.location[1], self.location[2]], [0, 0, 0, 1]], camera_pose, im_world, size=self.size )[1])

        # images [0] = depth, [1] = seg_mask
        #im_world.exit()
        for i in range(0, 4):
            images[i] = self._depth_buffer_to_meter(images[i])
        return images

    def _depth_buffer_to_meter(self, buffer):
        """
        This method converts the depth images generated by PyBullet to represent
        each position in metre.
        :return: The depth image in metre
        """
        near = 0.2
        far = 100
        return far * near / (far - (far-near)*buffer)

    def _create_image_world(self):
        """
        Creates a new BulletWorld which is used for creating the depth images.
        From the new Bullet World the robot and, if the costmap is created for an
        object, this is also removed.
        :return: The reference to the new BulletWorld
        """
        world = BulletWorld.current_bullet_world.copy()
        # for obj in world.objects:
        #     if BulletWorld.robot != None and obj.name == BulletWorld.robot.name \
        #         and obj.type == BulletWorld.robot.type:
        #         obj.remove
        #     if obj.get_position() == self.location:
        #         obj.remove
        return world

    def _choose_image(self, index):
        """
        Chooses the corresponding depth image for an index in the costmap.
        :param index: The index for which the depth image should be found.
        :return: The index of the corresponding depth image.
        """
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
        """
        Choooses the column in the depth image for an index. The values of this
        column are then used to calculate the value of the index in the costmap.
        :param index: The index for which the column should be choosen
        :return: The Colum in the depth image.
        """
        #index_in_depth = [index[1], self.size - index[0]]
        index_in_depth = self._calculate_index_in_depth(index)
        index_in_depth[0] = index_in_depth[0] - self.size/2
        index_in_depth[1] = abs(self.size/2 - index_in_depth[1])
        column = (index_in_depth[0] / index_in_depth[1]) * (self.size / 2)
        column += self.size/2
        return round(column)

    def _calculate_index_in_depth(self, index):
        x, y = index
        x += self.size/2
        y += self.size/2
        if y < self.size/2 and x >= y and x < (self.size - y):
            return  [x, y]
        elif x < self.size/2 and y > x and y < (self.size - x):
            return [self.size-y-1, x]
        elif y > self.size/2 and x< y and x >  (self.size - y - 1):
            return [self.size-x-1, y]
        else:
            return [y, self.size-x-1]

    def _compute_column_range(self, index, min_height, max_height):
        """
        The indices which determine the range of entries in the depth image which
        are used for calulating the entry in the costmap.
        :param index: The index in the costmap for which the range should be calculated.
        :param min_height: The minimal height on which the camera on the robot can be.
        :param max_height: The maximal height on which the camera on the robot can be.
        :return: The two indicies which determine the range in the column in the
        depth image.
        """
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
                #print(f"Collumn: {c} with range: {r_min} - {r_max}")
                for r in range(r_min, r_max+1):
                    #if depth_imgs[depth_index][c][r] > d:
                    if depth_imgs[depth_index][r][c] > d * self.resolution:
                        v += 1
                        max_value += 1

                if max_value > 0:
                    x_i = abs(int(self.size/2) -x -1)
                    y_i = y + int(self.size/2)
                    self.map[x_i][y_i] = v / max_value

class ReachabilityCostmap(Costmap):
    def __init__(self):

        Costmap.__init__(self,)


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
