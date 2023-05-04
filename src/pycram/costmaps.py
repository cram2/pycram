# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import numpy as np
import pybullet as p
import rospy
import matplotlib.pyplot as plt
from matplotlib import colors
import psutil
import time
from .robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from .bullet_world import BulletWorld, Use_shadow_world
from .bullet_world_reasoning import _get_images_for_target
from nav_msgs.msg import OccupancyGrid, MapMetaData
from typing import Tuple, List, Union, Optional




class Costmap:
    """
    The base class of all Costmaps which implemnets the visualization of costmaps
    in the BulletWorld.
    """
    def __init__(self, resolution: float,
                 height: int,
                 width: int,
                 origin: Tuple[List[float], List[float]],
                 map: np.ndarray):
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
        self.resolution: float = resolution
        self.size: int = height
        self.height: int = height
        self.width: int = width
        self.origin: Tuple[List[float], List[float]] = origin
        self.map: np.ndarray = map
        self.vis_ids: List[int] = []

    def visualize(self) -> None:
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
            for j in range(0, map.shape[1]):
                if map[i][j] > 0:
                    curr_width = self._find_consectuive_line((i, j), map)
                    curr_pose = (i, j)
                    curr_height = self._find_max_box_height((i,j), curr_width, map)
                    avg = np.average(map[i:i+curr_height, j:j+curr_width])
                    boxes.append([curr_pose, curr_height, curr_width, avg])
                    map[i:i+curr_height, j:j+curr_width] = 0
        cells = []
        # Creation of the visual shapes, for documentation of the visual shapes
        # please look here: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.q1gn7v6o58bf
        for box in boxes:
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[(box[1] * self.resolution) / 2, (box[2] * self.resolution) / 2, 0.001],
                rgbaColor=[1, 0, 0, 0.6], visualFramePosition=[(box[0][0] + box[1]/2) * self.resolution, (box[0][1] + box[2]/2) * self.resolution, 0.])
            cells.append(visual)
        # Set to 127 for since this is the maximal amount of links in a multibody
        for cell_parts in self._chunks(cells, 127):
            # Dummy paramater since these are needed to spawn visual shapes as a
            # multibody.
            link_poses = [[0, 0, 0] for c in cell_parts]
            link_orientations = [[0, 0, 0, 1] for c in cell_parts]
            link_masses = [1.0 for c in cell_parts]
            link_parent = [0 for c in cell_parts]
            link_joints = [p.JOINT_FIXED for c in cell_parts]
            link_collision = [-1 for c in cell_parts]
            link_joint_axis = [[1, 0, 0] for c in cell_parts]
            # The position at which the multibody will be spawned. Offset such that
            # the origin referes to the centre of the costmap.
            origin_pose = self.origin[0]
            base_pose = [origin_pose[0] - self.height/2*self.resolution, origin_pose[1] - self.width / 2*self.resolution, origin_pose[2]]

            offset = [[-self.height/2*self.resolution, -self.width/2 *self.resolution, 0.05], [0, 0, 0, 1]]
            new_pose = p.multiplyTransforms(self.origin[0], self.origin[1], offset[0], offset[1])

            map_obj = p.createMultiBody(baseVisualShapeIndex=-1, linkVisualShapeIndices=cell_parts,
                basePosition=new_pose[0], baseOrientation=new_pose[1], linkPositions=link_poses, # [0, 0, 1, 0]
                linkMasses=link_masses, linkOrientations=link_orientations,
                linkInertialFramePositions=link_poses,
                linkInertialFrameOrientations=link_orientations,linkParentIndices=link_parent,
                linkJointTypes=link_joints, linkJointAxis=link_joint_axis,
                linkCollisionShapeIndices=link_collision)
            self.vis_ids.append(map_obj)


    def _chunks(self, lst: List, n: int) -> List:
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]

    def close_visualization(self) -> None:
        for id in self.vis_ids:
            p.removeBody(id)
        self.vis_ids =  []

    def _find_consectuive_line(self, start: Tuple[int, int], map: np.ndarray) -> int:
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

    def _find_max_box_height(self, start: Tuple[int, int], lenght: int, map: np.ndarray) -> int:
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

    def merge(self, other_cm: Costmap) -> Union[None, Costmap]:
        """
        Merges the values of two costmaps and returns a new costmap that has for
        every cell the merged values of both inputs. To merge two costmaps they
        have to be the same size, resolution and posess the same origin.
        :param other_cm: The other costmap with which this costmap should be merged.
        """
        if self.size != other_cm.size:
            raise ValueError("You can only merge costmaps of the same size.")
        elif self.origin[0][0] != other_cm.origin[0][0] or self.origin[0][1] != other_cm.origin[0][1] or self.origin[1] != other_cm.origin[1]:
            raise ValueError("To merge costmaps, the x and y coordinate as well as the orientation must be equal.")
        elif self.resolution != other_cm.resolution:
            raise ValueError("To merge two costmaps their resolution must be equal.")
        new_map = np.zeros((self.height, self.width))
        # A nunpy array of the positions where both costmaps are greater than 0
        merge = np.logical_and(self.map > 0, other_cm.map > 0)
        new_map[merge] = self.map[merge] * other_cm.map[merge]
        new_map = (new_map / np.max(new_map)).reshape((self.height, self.width))
        return Costmap(self.resolution, self.height, self.width, self.origin, new_map)

    def __add__(self, other):
        if isinstance(other, Costmap):
            return self.merge(other)
        else:
            raise ValueError(f"Can only combine two costmaps other type was {type(other)}")


class OccupancyCostmap(Costmap):
    """
    The occupancy Costmap represents a map of the environment where obstacles or
    positions which are inaccessiable for a robot have a value of -1.
    """
    def __init__(self, distance_to_obstacle: float,
                 from_ros: Optional[bool] = False,
                 size: Optional[int] = 100,
                 resolution: Optional[float] = 0.02,
                 origin: Optional[List[float]] = [[0,0,0], [0, 0, 0, 1]],
                 world: Optional[BulletWorld] = None):
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

    def _calculate_diff_origin(self, height: int, width: int) -> List[float]:
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
    def _get_map() -> np.ndarray:
        """
        Receives the map array from the map_server converts it into a numpy array.
        :return: The costmap as a numpy array.
        """
        print("Waiting for Map")
        map = rospy.wait_for_message("/map",  OccupancyGrid)
        print("Recived Map")
        return np.array(map.data)

    @staticmethod
    def _get_map_metadata() -> MapMetaData:
        """
        Receives the meta data about the costmap from the map_server and returns it.
        The meta data contains things like, height, width, origin and resolution.
        :return: The meta data for the costmap array.
        """
        print("Waiting for Map Meta Data")
        meta = rospy.wait_for_message("/map_metadata", MapMetaData)
        print("Recived Meta Data")
        return meta

    def _convert_map(self, map: np.ndarray, size: int) -> np.ndarray:
        """
        This Method converts the Occupancy Map received from ROS to be more consistent
        with how PyCRAM handles its costmap. Every possible cell for a robot to stand
        is set to one while anything else is set to zero. Additionally this method
        also takes into account the distance_to_obstacle parameter and sets cell values
        that are too close to an obstacle to 0.
        :param map: The map that should be converted. Represented as 2d numpy array
        :return: The converted map. Represented as 2d numpy array.
        """
        map = np.pad(map, (int(self.distance_obstacle/2), int(self.distance_obstacle/2)))

        sub_shape = (self.distance_obstacle, self.distance_obstacle)
        view_shape = tuple(np.subtract(map.shape, sub_shape) + 1) + sub_shape
        strides = map.strides + map.strides

        sub_matrices = np.lib.stride_tricks.as_strided(map,view_shape,strides)
        sub_matrices = sub_matrices.reshape(sub_matrices.shape[:-2] + (-1,))
        sum = np.sum(sub_matrices, axis=2)
        return (sum == 0).astype('int16')

    def create_sub_map(self, sub_origin: List[float], size: int) -> Costmap:
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
        new_origin = np.abs(new_origin)
        # Offset to top left corner, for easier slicing
        new_origin = (new_origin - size/2).astype(int)

        # slices a submap with size "size" around the given origin
        sub_map = self.original_map[new_origin[1]: new_origin[1] + size,
                            new_origin[0]: new_origin[0] + size]
        # Convert map to fit with the other costmaps
        sub_map = np.rot90(np.flip(self._convert_map(sub_map, size), 0))
        return Costmap(self.resolution, size, size, list(sub_origin * -1), sub_map)

    def _create_from_bullet(self, world: BulletWorld, size: int, resolution: float, origin: List[float]):
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

        origin_position = self.origin[0]
        # Generate 2d grid with indices
        indices = np.concatenate(np.dstack(np.mgrid[int(-size/2):int(size/2),int(-size/2):int(size/2)]), axis=0) * resolution + np.array(origin_position[:2])
        # Add the z-coordinate to the grid, which is either 0 or 10
        indices_0 = np.pad(indices, (0, 1), mode='constant', constant_values=0)[:-1]
        indices_10 = np.pad(indices, (0,1), mode='constant', constant_values=10)[:-1]
        # Zips both arrays such that there are tuples for every coordinate that
        # only differ in the z-coordinate
        rays = np.dstack(np.dstack((indices_0, indices_10))).T

        res = np.zeros(len(rays))
        # Using the PyBullet rayTest to check if there is an object above the position
        # if there is no object the position is marked as valid
        # 16383 is the maximal number of rays that can be processed in a batch
        i = 0
        j = 0
        for n in self._chunks(np.array(rays), 16380):
            with Use_shadow_world():
                r_t = p.rayTestBatch(n[:, 0], n[:, 1], numThreads=0, physicsClientId=BulletWorld.current_bullet_world.client_id)
                while r_t is None:
                    r_t = p.rayTestBatch(n[:, 0], n[:, 1], numThreads=0,
                                         physicsClientId=BulletWorld.current_bullet_world.client_id)
                j += len(n)
                if BulletWorld.robot:
                    res[i:j] = [1 if ray[0] == -1 or ray[0] == BulletWorld.robot.id else 0 for ray in r_t]
                else:
                    res[i:j] = [1 if ray[0] == -1 else 0 for ray in r_t]
                i += len(n)

        res = np.flip(np.reshape(np.array(res), (size, size)))

        map = np.pad(res, (int(self.distance_obstacle/2), int(self.distance_obstacle/2)))

        sub_shape = (self.distance_obstacle*2, self.distance_obstacle*2)
        view_shape = tuple(np.subtract(map.shape, sub_shape) + 1) + sub_shape
        strides = map.strides + map.strides

        sub_matrices = np.lib.stride_tricks.as_strided(map,view_shape,strides)
        sub_matrices = sub_matrices.reshape(sub_matrices.shape[:-2] + (-1,))

        sum = np.sum(sub_matrices, axis=2)
        map = (sum == (self.distance_obstacle*2) ** 2).astype('int16')
        # The map loses some size due to the strides and because I dont want to
        # deal with indices outside of the index range
        offset = self.size - map.shape[0]
        odd = 0 if offset % 2 == 0 else 1
        map = np.pad(map, (offset // 2, offset // 2 + odd))

        return np.flip(map)

    def _chunks(self, lst: List, n: int) -> List:
        """Yield successive n-sized chunks from lst."""
        for i in range(0, len(lst), n):
            yield lst[i:i + n]


class VisibilityCostmap(Costmap):
    """
    A costmap that represents the visibility of a specific point for every position around
    this point. For a detailed explanation on how the creation of the costmap works
    please look here: https://mediatum.ub.tum.de/doc/1239461/1239461.pdf (page 173)
    """

    def __init__(self, min_height: float,
                 max_height: float,
                 size: Optional[int] = 100,
                 resolution: Optional[float] = 0.02,
                 origin: Optional[List[float]] = [[0,0,0], [0, 0, 0, 1]],
                 world: Optional[BulletWorld] = None):
        """
        The constructor of the visibility costmap which assisgs the given paranmeter
        and triggeres the generation of the costmap. If you want the visibility for
        an object just put the position of the object as the origin and the costmap
        will create the visibility for this.
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
        if (11*size**2 +size**3)*2 > psutil.virtual_memory().available:
            raise OSError("Not enough free RAM to calculate a costmap of this size")

        self.world = world if world else BulletWorld.current_bullet_world
        self.map = np.zeros((size, size))
        self.size = size
        self.resolution = resolution
        # for pr2 = 1.27
        self.max_height: float = max_height
        #for pr2 = 1.6
        self.min_height: float = min_height
        self.origin: List[float] = origin
        self._generate_map()
        Costmap.__init__(self, resolution, size, size, self.origin, self.map)

    def _create_images(self) -> List[np.ndarray]:
        """
        This method creates four depth images in every direction around the point
        for which the costmap should be created. The depoth images are converted
        to metre, meaning that every entry in the depth images represents the
        distance to the next object in metre.
        :return: A list of four depth images, the images are represented as 2D arrays.
        """
        #object_pose = self.object.get_position_and_orientation()
        #im_world = self._create_image_world()
        images = []
        camera_pose = self.origin

        with Use_shadow_world():
            images.append(_get_images_for_target([[self.origin[0][0], self.origin[0][1] +1, self.origin[0][2]], [0, 0, 0, 1]],camera_pose, BulletWorld.current_bullet_world, size=self.size )[1])

            images.append(_get_images_for_target([[self.origin[0][0] -1, self.origin[0][1], self.origin[0][2]], [0, 0, 0, 1]], camera_pose, BulletWorld.current_bullet_world, size=self.size )[1])

            images.append(_get_images_for_target([[self.origin[0][0], self.origin[0][1] -1, self.origin[0][2]], [0, 0, 0, 1]],camera_pose, BulletWorld.current_bullet_world, size=self.size )[1])

            images.append(_get_images_for_target([[self.origin[0][0] +1, self.origin[0][1], self.origin[0][2]], [0, 0, 0, 1]], camera_pose, BulletWorld.current_bullet_world, size=self.size )[1])

        # images [0] = depth, [1] = seg_mask
        #im_world.exit()
        for i in range(0, 4):
            images[i] = self._depth_buffer_to_meter(images[i])
        return images

    def _depth_buffer_to_meter(self, buffer: np.ndarray) -> np.ndarray:
        """
        This method converts the depth images generated by PyBullet to represent
        each position in metre.
        :return: The depth image in metre
        """
        near = 0.2
        far = 100
        return far * near / (far - (far-near)*buffer)

    def _generate_map(self):
        """
        This method generates the resulting density map by using the algorithm explained
        in Lorenz Mösenlechners PhD thesis: https://mediatum.ub.tum.de/doc/1239461/1239461.pdf p.178
        The resulting density map is then saved to self.map
        """
        depth_imgs = self._create_images()
        # A 2D array where every cell contains the arctan2 value with respect to
        # the middle of the array. Additionally the interval is shifted such that
        # it is between 0 and 2pi
        tan = np.arctan2(np.mgrid[-int(self.size/2): int(self.size/2),-int(self.size/2): int(self.size/2)][0], \
                        np.mgrid[-int(self.size/2): int(self.size/2),-int(self.size/2): int(self.size/2)][1]) + np.pi
        res = np.zeros(tan.shape)

        # Just for completion, since the res array has zeros in every position this
        # operation is not neccessary.
        #res[np.logical_and(tan <= np.pi * 0.25, tan >= np.pi * 1.75)] = 0

        # Creates a 2D array which contains the index of the depth image for every
        # coordinate
        res[np.logical_and(tan >= np.pi * 1.25, tan <= np.pi * 1.75)] = 3
        res[np.logical_and(tan >= np.pi * 0.75, tan < np.pi * 1.25)] = 2
        res[np.logical_and(tan >= np.pi * 0.25, tan < np.pi * 0.75)] = 1


        indices = np.dstack(np.mgrid[0: self.size, 0: self.size])
        depth_indices = np.zeros(indices.shape)
        # x-value of index: res == n, :1
        # y-value of index: res == n, 1:2

        # (y, size-x-1) for index between 1.25 pi and 1.75 pi
        depth_indices[res == 3, :1] = indices[res == 3, 1:2]
        depth_indices[res == 3, 1:2] = self.size - indices[res == 3,:1] -1


        # (size-x-1, y) for index between 0.75 pi and 1.25 pi
        depth_indices[res == 2, :1] = self.size - indices[res == 2,:1] -1
        depth_indices[res == 2, 1:2] = indices[res == 2, 1:2]

        # (size-y-1, x) for index between 0.25 pi and 0.75 pi
        depth_indices[res==1, :1] = self.size - indices[res==1, 1:2] - 1
        depth_indices[res==1, 1:2] = indices[res==1, :1]

        # (x, y) for index between 0.25 pi and 1.75 pi
        depth_indices[res==0, :1] = indices[res ==0, :1]
        depth_indices[res==0, 1:2] = indices[res==0, 1:2]

        # Convert back to origin in the middle of the costmap
        depth_indices[:, :, :1] -= self.size/2
        depth_indices[:, :, 1:2] = np.absolute(self.size/2 - depth_indices[:, :, 1:2] )

        # Sets the y index for the coordinates of the middle of the costmap to 1,
        # the computed value is 0 which would cause an error in the next step where
        # the calculation divides the x coordinates by the y coordinates
        depth_indices[int(self.size/2), int(self.size/2), 1] = 1


        # Calculate columns for the respective position in the costmap
        columns = np.around(((depth_indices[:, :, :1]/depth_indices[:, :, 1:2]) \
            * (self.size/2)) + self.size/2).reshape((self.size, self.size)).astype('int16')

        # An array with size * size that contains the euclidian distance to the
        # orgin (in the middle of the costmap) from every cell
        distances = np.maximum(np.linalg.norm(np.dstack(np.mgrid[-int(self.size/2): int(self.size/2), \
                                                        -int(self.size/2): int(self.size/2)]), axis=2), 0.001)

        # Row ranges
        # Calculation of the ranges of coordinates in the row which have to be
        # taken into accoount. The range is from r_min to r_max.
        # This are two arrays with shape: size*size, the r_min constains the beginig
        # of the range for every coordinate and r_max contains the end for each
        # coordinate
        r_min = (np.arctan((self.min_height-self.origin[0][2] ) / distances) * self.size) + self.size/2
        r_max = (np.arctan((self.max_height-self.origin[0][2] ) / distances) * self.size) + self.size/2

        r_min = np.minimum(np.around(r_min), self.size-1).astype('int16')
        r_max = np.minimum(np.around(r_max), self.size-1).astype('int16')



        rs = np.dstack((r_min, r_max+1)).reshape((self.size**2, 2))
        r = np.arange(self.size)
        # Calculates a mask from the r_min and r_max values. This mask is for every
        # coordinate respectivly and determines which values from the computed column
        # of the depth image should be taken into account for the costmap.
        # A Mask of a single coordinate has the length of the column of the depth image
        # and together with the computed column at this coordinate determines which
        # values of the depth image make up the value of the visibility costmap at this
        # point.
        mask = ((rs[:,0,None] <= r) & (rs[:,1,None] > r)).reshape((self.size, self.size, self.size))


        values = np.zeros((self.size, self.size))
        map = np.zeros((self.size, self.size))
        # This is done to itterate over the depth images one at a time
        for i in range(4):
            row_masks = mask[res==i].T
            # This statment does several things, first it takes the values from
            # the depth image for this quater of the costmap. The values taken are
            # the complete columns of the depth image (which where computed beforehand)
            # and checks if the values in them are greater than the distance to the
            # respective coordinates. This does not take the row ranges into account.
            values = depth_imgs[i][:,columns[res==i].flatten()] > \
                        np.tile(distances[res==i][:, None], (1, self.size)).T * self.resolution
            # This applies the created mask of the row ranges to the values of
            # the columns which are compared in the previous statment
            masked = np.ma.masked_array(values, mask=~row_masks)
            # The calculated values are added to the costmap
            map[res==i] = np.sum(masked, axis=0)
        map /= np.max(map)
        # Weird flipping shit so that the map fits the orientation of the visualization.
        # the costmap in itself is consistant and just needs to be flipped to fit the world coordinate system
        map = np.flip(map, axis=0)
        map = np.flip(map)
        self.map = map


class GaussianCostmap(Costmap):
    def __init__(self, mean: int, sigma: float, resolution: Optional[float] = 0.02, origin: Optional[List[float]] = [[0,0,0], [0, 0, 0, 1]]):
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
        self.gau: np.ndarray = self._gaussian_window(mean, sigma)
        self.map: np.ndarray = np.outer(self.gau, self.gau)
        self.size: float = mean
        self.origin: List[float] = origin
        Costmap.__init__(self, resolution, mean, mean, origin, self.map)

    def _gaussian_window(self, mean: int, std: float) -> np.ndarray:
        """
        This method creates a window of values with a gaussian distribution of
        size "mean" and standart deviation "std".
        Code from: https://github.com/scipy/scipy/blob/v0.14.0/scipy/signal/windows.py#L976
        """
        n = np.arange(0, mean) - (mean - 1.0) / 2.0
        sig2 = 2 * std * std
        w = np.exp(-n ** 2 / sig2)
        return w


class SemanticCostmap(Costmap):

    def __init__(self, object, urdf_link_name, size=100, resolution=0.02, world=None):
        self.world = world if world else BulletWorld.current_bullet_world
        self.object = object
        self.link = urdf_link_name
        self.resolution = resolution
        self.origin = object.get_link_position_and_orientation(urdf_link_name)
        self.height = 0
        self.width = 0
        self.map = []
        self.generate_map()

        Costmap.__init__(self, resolution, self.height, self.width, self.origin, self.map)

    def generate_map(self):
        link_position = self.object.get_link_position(self.link)
        min, max = self.get_aabb_for_link()
        self.height = int((max[0] - min[0]) // self.resolution)
        self.width = int((max[1] - min[1]) // self.resolution)
        self.map = np.ones((self.height, self.width))
        self.origin = [self.origin[0], self.origin[1]]

    def get_aabb_for_link(self):
        shadow_obj = BulletWorld.current_bullet_world.get_shadow_object(self.object)
        with Use_shadow_world():
            shadow_obj.set_orientation([0, 0, 0, 1])
            link_orientation = shadow_obj.get_link_orientation(self.link)
            inverse_orientation = p.invertTransform([0, 0, 0], link_orientation)[1]
            shadow_obj.set_orientation(inverse_orientation)
            return shadow_obj.get_AABB(self.link)


cmap = colors.ListedColormap(['white', 'black', 'green', 'red', 'blue'])


# Mainly used for debugging
# Data is 2d array
def plot_grid(data: np.ndarray) -> None:
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
