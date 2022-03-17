import numpy as np
import pybullet as p
import rospy
import time
from scipy import signal
from pycram.robot_description import InitializedRobotDescription as robot_description
from .bullet_world import BulletWorld
from .bullet_world_reasoning import _get_images_for_target
from .ik import request_ik, IKError
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
        self.size = height
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
                    avg = np.average(map[i:i+curr_height, j:j+curr_width])
                    boxes.append([curr_pose, curr_height, curr_width, avg])
                    map[i:i+curr_height, j:j+curr_width] = 0
        cells = []
        #print(boxes)
        # Creation of the visual shapes, for documentation of the visual shapes
        # please look here: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.q1gn7v6o58bf
        for box in boxes:
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[(box[1] *self.resolution) / 2, (box[2]*self.resolution) / 2, 0.001],
                rgbaColor=[1, 0, 0, 0.6], visualFramePosition=[(box[0][0] + box[1]/2)*self.resolution, (box[0][1] + box[2]/2)*self.resolution, 0.])
            cells.append(visual)
        # Set to 127 for since this is the maximal amount of links in a multibody
        for cell_parts in self._chunks(cells, 127):
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
        curr_height = 1
        for i in range(start[0], height):
            for j in range(start[1], start[1] + lenght):
                if map[i][j] <= 0:
                    return curr_height
            curr_height += 1
        return curr_height

    def merge(self, other_cm):
        """
        Merges the values of two costmaps and returns a new costmap that has for
        every cell the merged values of both inputs. To merge two costmaps they
        have to be the same size, resolution and posess the same origin.
        :param other_cm: The other costmap with which this costmap should be merged.
        """
        if self.size != other_cm.size:
            print("To merge costmaps, the size has to be equal")
            return
        elif self.origin != other_cm.origin:
            print("To merge costmaps, the origin has to be equal")
            return
        elif self.resolution != other_cm.resolution:
            print("To merge cotsmaps, the resoulution has to be equal")
            return
        new_map = np.zeros((self.height, self.width))
        for x in range(0, self.height):
            for y in range(0, self.width):
                if self.map[x][y] == 0 or other_cm.map[x][y] == 0:
                    new_map[x][y] = 0
                else:
                    new_map[x][y] = self.map[x][y] * other_cm.map[x][y]

        new_map /= np.max(np.abs(new_map))
        return Costmap(self.resolution, self.height, self.width, self.origin, new_map)

    def __add__(self, other):
        if isinstance(other, Costmap):
            return self.merge(other)
        else:
            print("Can only combine two costmaps")
            return None

class OccupancyCostmap(Costmap):
    """
    The occupancy Costmap represents a map of the environment where obstacles or
    positions which are inaccessiable for a robot have a value of -1.
    """
    def __init__(self, distance_to_obstacle, from_ros=False, size=100, resolution=0.02, origin=[0,0,0], world=None):
        """
        The constructor for the Occupancy costmap, the actual costmap is received
        from the ROS map_server and wrapped by this class. Meta data about the
        costmap is also received from the map_server.
        :param distance_obstacle: The distance by which the obstacles should be
        inflated. Meaning that obstacles in the costmap are growing bigger by this
        distance.
        :param from_ros: This determines if the Occupancy map should be created
            from the map provided by the ROS map_server or from the BulletWorld.
            If True then the map from the ROS map_server will be used otherwise
            the Occupancy map will be created from the BulletWorld.
        :param size: The lenght of the side of the costmap. The costmap will be created
            as a square. This will only be used if from_ros is False.
        :param resolution: The resolution of this costmap. This determines how much
            meter one pixel in the costmap represents. This is only used if from_ros
            is Flase.
        :param origin: This determines the origin of the costmap. The origin will
            be in the middle of the costmap. This parameter is only used if from_ros
            is False.
        :param world: This parameter specifies the BulletWorld for which a Occupancy
            costmap should be created. This parameter is only used if from_ros is False.
        """
        if from_ros:
            meta = self._get_map_metadata()
            self.original_map = np.reshape(self._get_map(), (meta.height, meta.width))
            self.meta_origin = [meta.origin.position.x, meta.origin.position.y, meta.origin.position.z]
            self.resolution = meta.resolution
            self.height = meta.height
            self.width = meta.width
            # Nunber of cells that have to be between a valid cell and an obstacle
            self.distance_obstacle = max(int(distance_to_obstacle / self.resolution), 1)
            Costmap.__init__(self, meta.resolution, meta.height, meta.width,
                        self._calculate_diff_origin(meta.height, meta.width),
                        np.rot90(np.flip(self._convert_map(self.original_map, self.height), 0)))
        else:
            self.size = size
            self.origin = origin
            self.resolution = resolution
            self.distance_obstacle = max(int(distance_to_obstacle / self.resolution), 1)
            self.map = self._create_from_bullet(world, size, resolution, origin)
            Costmap.__init__(self, resolution, size, size, self.origin, self.map)

    def _calculate_diff_origin(self, height, width):
        """
        This method calculates the difference between the origin of the costmap
        as stated by the meta data and the actual middle of the costmap which
        is used by PyCRAM to visualize the cotsmap. The origin as stated by the
        meta data referes to the position of the global coordinate frame with
        the bottom left corner as reference.
        :param height: The Hieght of the costmap
        :param width: The width of the costmap
        :return: The difference between the actual origin and center of the costmap
        """
        actual_origin = [int(height/2) * self.resolution, int(width/2) * self.resolution, 0]
        origin = np.array(self.meta_origin) + np.array(actual_origin)
        return origin

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
        :return: The meta data for the costmap array.
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

    def _convert_map(self, map, size):
        """
        This Method converts the Occupancy Map received from ROS to be more consistent
        with how PyCRAM handles its costmap. Every possible cell for a robot to stand
        is set to one while anything else is set to zero. Additionally this method
        also takes into account the distance_to_obstacle parameter and sets cell values
        that are too close to an obstacle to 0.
        :param map: The map that should be converted. Represented as 2d numpy array
        :return: The converted map. Represented as 2d numpy array.
        """
        new_map = np.zeros((size, size))
        for x in range(0, size):
            for y in range(size):
                # Surrounding cells with respect to distance to obstacle
                surrounding_cells = map[x - self.distance_obstacle: x + self.distance_obstacle + 1,
                                        y - self.distance_obstacle: y + self.distance_obstacle + 1]
                # Checks if there are any cells with values greater than zero in
                # surrounding_cells.
                if np.sum(surrounding_cells) == 0 and surrounding_cells.size != 0:
                    new_map[x][y] = 1
        return new_map
        #return Costmap(self.resolution, self.height, self.width, self.origin, new_map)

    def _create_sub_map(self, sub_origin, size):
        """
        Creates a smaller map from the overall occupancy map, the new map is centered
        around the point specified by "sub_origin" and has the size "size". The
        resolution of the costmap stayes the same for the sub costmap.
        :param sub_origin: The point in global coordinate frame, around which the
            sub costmap should be centered.
        :param size: The size the sub costmap should have.
        :return The sub costmap, represented as 2d numpy array.
        """
        # To ensure this is a numpy array
        sub_origin = np.array(sub_origin)
        # Since origin obtained from the meta data uses bottom left corner as reference.
        sub_origin *= -1
        # Calculates origin of sub costmap as vector between origin and given sub_origin
        new_origin = np.array(self.meta_origin) + sub_origin
        # Convert from vector in meter to index values
        new_origin /= self.resolution
        #new_origin = np.array([abs(int(x)) for x in new_origin])
        new_origin = np.abs(new_origin)
        # Offset to top left corner, for easier slicing
        new_origin = (new_origin - size/2).astype(int)
        print(new_origin)

        # slices a submap with size "size" around the given origin
        sub_map = self.original_map[new_origin[1]: new_origin[1] + size,
                            new_origin[0]: new_origin[0] + size]
        # Convert map to fit with the other costmaps
        sub_map = np.rot90(np.flip(self._convert_map(sub_map, size), 0))
        return Costmap(self.resolution, size, size, list(sub_origin * -1), sub_map)

    def _create_from_bullet(self, world, size, resolution, origin):
        """
        This method creates a Occupancy Costmap for the specified BulletWorld.
        This map marks every position as valid that has no object above it. After
        creating the costmap the distance to obstacle parameter is applied.
        :param world: The BulletWorld for which the Occupancy Costmap should
            be created
        :param size: The size of this costmap. The size specifies the lenght of one
            side of the costmap. The costmap is created as a square.
        :param resolution: The resolution of this costmap. This determies how much
            meter a pixel in the costmap represents.
        :param origin: The origin of the costmap, this is the position in world coordinate
            frame around which the cotsmap should be created.

        """
        rays = []
        # Creating a 2D array with positions in world coordinate frame for every cell
        for x in range(int(-size/2), int(size/2)):
            for y in range(int(-size/2), int(size/2)):
                rays.append(([origin[0] + x * resolution, origin[1] + y * resolution, 0],
                            [origin[0] + x * resolution,origin[0] + y * resolution, 10]))

        res = []
        # Using the PyBullet rayTest to check if there is an object above the position
        # if there is no object the position is marked as valid
        for n in self._chunks(np.array(rays), 16383):
            r_t = p.rayTestBatch(n[:,0], n[:,1],numThreads=0)
            if BulletWorld.robot:
                res += (1 if ray[0] == -1 or ray[0] == BulletWorld.robot.id else 0 for ray in r_t)
            else:
                res += (1 if ray[0] == -1  else 0 for ray in r_t)

        res = np.flip(np.reshape(np.array(res), (size, size)))
        new_map = np.zeros((size, size))
        # Apply the distance to obstacle paramter
        for x in range(0, size):
            for y in range(0, size):
                surrounding_cells = res[x - self.distance_obstacle: x+self.distance_obstacle ,
                                        y - self.distance_obstacle: y+self.distance_obstacle ]
                #print(np.sum(surrounding_cells))
                if np.sum(surrounding_cells) == (self.distance_obstacle * 2) ** 2  and surrounding_cells.size != 0:
                    new_map[x][y] = 1

        return new_map

    def _chunks(self, lst, n):
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]



class VisibilityCostmap(Costmap):
    """
    A costmap that represents the visibility of a specific point for every position around
    this point. For a detailed explanation on how the creation of the costmap works
    please look here: https://mediatum.ub.tum.de/doc/1239461/1239461.pdf (page 173)
    """

    def __init__(self, min_height, max_height, size=100, resolution=0.02, origin=[0,0,0], world=None):
        """
        The constructor of the visibility costmap which assisgs the given paranmeter
        and triggeres the generation of the costmap.
        :param min_height: This is the minimal height the camera can be. This parameter
            is mostly relevant if the vertical position of the camera can change.
        :param max_height: This is the maximal height the camera can be. This is
            mostly revelevant if teh vertical position of the camera can change.
        :param size: The lenght of the side of the costmap, the costmap is created
            as a square.
        :param resolution: This parameter specifies how much meter a pixel in the
            costmap represents.
        :param origin: The position in world coordinate frame around which the
            costmap should be created.
        :param world: The BulletWorld for which the costmap should be created.
        """
        #self.object = object
        self.world = world if world else BulletWorld.current_bullet_world
        self.map = np.zeros((size, size))
        self.size = size
        self.resolution = resolution
        # for pr2 = 1.27
        self.max_height = max_height
        #for pr2 = 1.6
        self.min_height = min_height
        self.origin = origin
        self._generate_map()
        Costmap.__init__(self, resolution, size, size, origin, self.map)


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
        camera_pose = [self.origin, [0, 0, 0, 1]]

        images.append(_get_images_for_target([[self.origin[0], self.origin[1] +1, self.origin[2]], [0, 0, 0, 1]],camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.origin[0] -1, self.origin[1], self.origin[2]], [0, 0, 0, 1]], camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.origin[0], self.origin[1] -1, self.origin[2]], [0, 0, 0, 1]],camera_pose, im_world, size=self.size )[1])

        images.append(_get_images_for_target([[self.origin[0] +1, self.origin[1], self.origin[2]], [0, 0, 0, 1]], camera_pose, im_world, size=self.size )[1])

        # images [0] = depth, [1] = seg_mask
        im_world.exit()
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
        world = self.world.copy()
        # for obj in world.objects:
        #     if BulletWorld.robot != None and obj.name == BulletWorld.robot.name \
        #         and obj.type == BulletWorld.robot.type:
        #         obj.remove
        #     if obj.get_position() == self.origin:
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
            return 0 #0
        elif angle >= np.pi * 1.25 and angle < np.pi * 1.75:
            return 3 #1
        elif angle >= np.pi * 0.75 and angle < np.pi * 1.25:
            return 2 #2
        elif angle >= np.pi * 0.25 and angle < np.pi * 0.75:
            return 1 #3

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
        """
        This method calulates the index in the depth image for a given index in
        the costmap.
        :param index: The index in the costmap, this index is with the origin in
            the middle of the costmap
        :return: The corresponding index in the depth image
        """
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
        height = self.origin[2]
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
                    x_i = int(self.size/2) - x- 1
                    y_i = int(self.size/2) + y-1
                    self.map[x_i][y_i] = v / max_value


class GaussianCostmap(Costmap):
    def __init__(self, mean, sigma, resolution=0.02, origin=[0,0,0]):
        """
        This Costmap creates a 2D gaussian distribution around the origin with
        the specified size.
        :param mean: The mean input for the gaussian distribution, this also specifies
            the lenght of the side of the resulting cotsmap. The costmap is Created
            as a square.
        :param sigma: The sigma input for the gaussian distribution.
        :parma resolution: The resolution of the costmap, this specifies how much
            meter a pixel represents.
        :param origin: The origin of the costmap around which it will be created.
        """
        #self.gau = np.random.normal(mean, sigma, mean)
        self.gau = signal.gaussian(mean, sigma)
        self.gau = self._gaussian_window(mean, sigma)
        self.map = np.outer(self.gau, self.gau)
        self.size = mean
        self.origin = origin
        Costmap.__init__(self, resolution, mean, mean, origin, self.map)

    def _gaussian_window(self, mean, std):
        """
        This method creates a window of values with a gaussian distribution of
        size "mean" and standart deviation "std".
        Code from: https://github.com/scipy/scipy/blob/v0.14.0/scipy/signal/windows.py#L976
        """
        n = np.arange(0, mean) - (mean - 1.0) / 2.0
        sig2 = 2 * std * std
        w = np.exp(-n ** 2 / sig2)
        return w

cmap = colors.ListedColormap(['white', 'black', 'green', 'red', 'blue'])

# Mainly used for debugging
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
