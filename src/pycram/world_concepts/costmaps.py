# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from typing_extensions import Tuple, List, Optional

import matplotlib.pyplot as plt
from dataclasses import dataclass

import numpy as np
import psutil
import rospy
from matplotlib import colors
from nav_msgs.msg import OccupancyGrid, MapMetaData

from ..world import UseProspectionWorld
from ..world_concepts.world_object import Object
from ..description import Link
from ..datastructures.local_transformer import LocalTransformer
from ..datastructures.pose import Pose, Transform
from ..world import World
from ..datastructures.dataclasses import AxisAlignedBoundingBox, BoxVisualShape, Color


@dataclass
class Rectangle:
    """
    A rectangle that is described by a lower and upper x and y value.
    """
    x_lower: float
    x_upper: float
    y_lower: float
    y_upper: float

    def translate(self, x: float, y: float):
        """Translate the rectangle by x and y"""
        self.x_lower += x
        self.x_upper += x
        self.y_lower += y
        self.y_upper += y

    def scale(self, x_factor: float, y_factor: float):
        """Scale the rectangle by x_factor and y_factor"""
        self.x_lower *= x_factor
        self.x_upper *= x_factor
        self.y_lower *= y_factor
        self.y_upper *= y_factor


class Costmap:
    """
    The base class of all Costmaps which implements the visualization of costmaps
    in the World.
    """

    def __init__(self, resolution: float,
                 height: int,
                 width: int,
                 origin: Pose,
                 map: np.ndarray,
                 world: Optional[World] = None):
        """
        The constructor of the base class of all Costmaps.

        :param resolution: The distance in metre in the real-world which is
         represented by a single entry in the costmap.
        :param height: The height of the costmap.
        :param width: The width of the costmap.
        :param origin: The origin of the costmap, in world coordinate frame. The origin of the costmap is located in the
         centre of the costmap.
        :param map: The costmap represents as a 2D numpy array.
        :param world: The World for which the costmap should be created.
        """
        self.world = world if world else World.current_world
        self.resolution: float = resolution
        self.size: int = height
        self.height: int = height
        self.width: int = width
        local_transformer = LocalTransformer()
        self.origin: Pose = local_transformer.transform_pose(origin, 'map')
        self.map: np.ndarray = map
        self.vis_ids: List[int] = []

    def visualize(self) -> None:
        """
        Visualizes a costmap in the World, the visualisation works by
        subdividing the costmap in rectangles which are then visualized as world visual shapes.
        """
        if self.vis_ids:
            return

        # working on a copy of the costmap, since found rectangles are deleted
        map = np.copy(self.map)
        boxes = []
        # Finding all rectangles in the costmap
        for i in range(0, map.shape[0]):
            for j in range(0, map.shape[1]):
                if map[i][j] > 0:
                    curr_width = self._find_consectuive_line((i, j), map)
                    curr_pose = (i, j)
                    curr_height = self._find_max_box_height((i, j), curr_width, map)
                    avg = np.average(map[i:i + curr_height, j:j + curr_width])
                    boxes.append([curr_pose, curr_height, curr_width, avg])
                    map[i:i + curr_height, j:j + curr_width] = 0
        cells = []
        # Creation of the visual shapes, for documentation of the visual shapes
        # please look here:
        # https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.q1gn7v6o58bf
        for box in boxes:
            visual_shape = BoxVisualShape(Color(1, 0, 0, 0.6),
                                          visual_frame_position=[(box[0][0] + box[1] / 2) * self.resolution,
                                                                 (box[0][1] + box[2] / 2) * self.resolution, 0.],
                                          half_extents=[(box[1] * self.resolution) / 2,
                                                        (box[2] * self.resolution) / 2, 0.001])
            visual = self.world.create_visual_shape(visual_shape)
            cells.append(visual)

        # Set to 127 for since this is the maximal amount of links in a multibody
        for cell_parts in self._chunks(cells, 127):
            offset = Transform([-self.height / 2 * self.resolution, -self.width / 2 * self.resolution, 0.05],
                               [0, 0, 0, 1])
            origin = Transform(self.origin.position_as_list(), self.origin.orientation_as_list())
            new_transform = origin * offset
            new_pose = new_transform.to_pose().to_list()

            map_obj = self.world.create_multi_body_from_visual_shapes(cell_parts, Pose(*new_pose))
            self.vis_ids.append(map_obj)

    def _chunks(self, lst: List, n: int) -> List:
        """
        Yield successive n-sized chunks from lst.

        :param lst: The list from which chunks should be yielded
        :param n: Size of the chunks
        :return: A list of size n from lst
        """
        for i in range(0, len(lst), n):
            yield lst[i:i + n]

    def close_visualization(self) -> None:
        """
        Removes the visualization from the World.
        """
        for v_id in self.vis_ids:
            self.world.remove_object(self.world.get_object_by_id(v_id))
        self.vis_ids = []

    def _find_consectuive_line(self, start: Tuple[int, int], map: np.ndarray) -> int:
        """
        Finds the number of consecutive entries in the costmap which are greater
        than zero.

        :param start: The indices in the costmap from which the consecutive line should be found.
        :param map: The costmap in which the line should be found.
        :return: The length of the consecutive line of entries greater than zero.
        """
        width = map.shape[1]
        lenght = 0
        for i in range(start[1], width):
            if map[start[0]][i] > 0:
                lenght += 1
            else:
                return lenght
        return lenght

    def _find_max_box_height(self, start: Tuple[int, int], length: int, map: np.ndarray) -> int:
        """
        Finds the maximal height for a rectangle with a given width in a costmap.
        The method traverses one row at a time and checks if all entries for the
        given width are greater than zero. If an entry is less or equal than zero
        the height is returned.

        :param start: The indices in the costmap from which the method should start.
        :param length: The given width for the rectangle
        :param map: The costmap in which should be searched.
        :return: The height of the rectangle.
        """
        height, width = map.shape
        curr_height = 1
        for i in range(start[0], height):
            for j in range(start[1], start[1] + length):
                if map[i][j] <= 0:
                    return curr_height
            curr_height += 1
        return curr_height

    def merge(self, other_cm: Costmap) -> Costmap:
        """
        Merges the values of two costmaps and returns a new costmap that has for
        every cell the merged values of both inputs. To merge two costmaps they
        need to fulfill 3 constrains:

        1. They need to have the same size
        2. They need to have the same x and y coordinates in the origin
        3. They need to have the same resolution

        If any of these constrains is not fulfilled a ValueError will be raised.

        :param other_cm: The other costmap with which this costmap should be merged.
        :return: A new costmap that contains the merged values
        """
        if self.size != other_cm.size:
            raise ValueError("You can only merge costmaps of the same size.")
        elif self.origin.position.x != other_cm.origin.position.x or self.origin.position.y != other_cm.origin.position.y \
                or self.origin.orientation != other_cm.origin.orientation:
            raise ValueError("To merge costmaps, the x and y coordinate as well as the orientation must be equal.")
        elif self.resolution != other_cm.resolution:
            raise ValueError("To merge two costmaps their resolution must be equal.")
        new_map = np.zeros((self.height, self.width))
        # A nunpy array of the positions where both costmaps are greater than 0
        merge = np.logical_and(self.map > 0, other_cm.map > 0)
        new_map[merge] = self.map[merge] * other_cm.map[merge]
        new_map = (new_map / np.max(new_map)).reshape((self.height, self.width))
        return Costmap(self.resolution, self.height, self.width, self.origin, new_map)

    def __add__(self, other: Costmap) -> Costmap:
        """
        Overloading of the "+" operator for merging of Costmaps. Furthermore, checks if 'other' is actual a Costmap and
        raises a ValueError if this is not the case. Please check :func:`~Costmap.merge` for further information of merging.

        :param other: Another Costmap
        :return: A new Costmap that contains the merged values from this Costmap and the other Costmap
        """
        if isinstance(other, Costmap):
            return self.merge(other)
        else:
            raise ValueError(f"Can only combine two costmaps other type was {type(other)}")

    def partitioning_rectangles(self) -> List[Rectangle]:
        """
        Partition the map attached to this costmap into rectangles. The rectangles are axis aligned, exhaustive and
        disjoint sets.

        :return: A list containing the partitioning rectangles
        """
        ocm_map = np.copy(self.map)
        origin = np.array([self.height / 2, self.width / 2]) * -1
        rectangles = []

        # for every index pair (i, j) in the occupancy costmap
        for i in range(0, self.map.shape[0]):
            for j in range(0, self.map.shape[1]):

                # if this index has not been used yet
                if ocm_map[i][j] > 0:
                    curr_width = self._find_consectuive_line((i, j), ocm_map)
                    curr_pose = (i, j)
                    curr_height = self._find_max_box_height((i, j), curr_width, ocm_map)

                    # calculate the rectangle in the costmap
                    x_lower = curr_pose[0]
                    x_upper = curr_pose[0] + curr_height
                    y_lower = curr_pose[1]
                    y_upper = curr_pose[1] + curr_width

                    # mark the found rectangle as occupied
                    ocm_map[i:i + curr_height, j:j + curr_width] = 0

                    # transform rectangle to map space
                    rectangle = Rectangle(x_lower, x_upper, y_lower, y_upper)
                    rectangle.translate(*origin)
                    rectangle.scale(self.resolution, self.resolution)
                    rectangles.append(rectangle)

        return rectangles


class OccupancyCostmap(Costmap):
    """
    The occupancy Costmap represents a map of the environment where obstacles or
    positions which are inaccessible for a robot have a value of -1.
    """

    def __init__(self, distance_to_obstacle: float,
                 from_ros: Optional[bool] = False,
                 size: Optional[int] = 100,
                 resolution: Optional[float] = 0.02,
                 origin: Optional[Pose] = None,
                 world: Optional[World] = None):
        """
        Constructor for the Occupancy costmap, the actual costmap is received
        from the ROS map_server and wrapped by this class. Meta-data about the
        costmap is also received from the map_server.

        :param distance_to_obstacle: The distance by which the obstacles should be
            inflated. Meaning that obstacles in the costmap are growing bigger by this
            distance.
        :param from_ros: This determines if the Occupancy map should be created
            from the map provided by the ROS map_server or from the World.
            If True then the map from the ROS map_server will be used otherwise
            the Occupancy map will be created from the World.
        :param size: The length of the side of the costmap. The costmap will be created
            as a square. This will only be used if from_ros is False.
        :param resolution: The resolution of this costmap. This determines how much
            meter one pixel in the costmap represents. This is only used if from_ros
            is False.
        :param origin: This determines the origin of the costmap. The origin will
            be in the middle of the costmap. This parameter is only used if from_ros
            is False.
        """
        self.world = world if world else World.current_world
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
                             np.rot90(np.flip(self._convert_map(self.original_map), 0)))
        else:
            self.size = size
            self.origin = Pose() if not origin else origin
            self.resolution = resolution
            self.distance_obstacle = max(int(distance_to_obstacle / self.resolution), 1)
            self.map = self._create_from_world(size, resolution)
            Costmap.__init__(self, resolution, size, size, self.origin, self.map)

    def _calculate_diff_origin(self, height: int, width: int) -> Pose:
        """
        Calculates the difference between the origin of the costmap
        as stated by the meta-data and the actual middle of the costmap which
        is used by PyCRAM to visualize the costmap. The origin as stated by the
        meta-data refers to the position of the global coordinate frame with
        the bottom left corner as reference.

        :param height: The height of the costmap
        :param width: The width of the costmap
        :return: The difference between the actual origin and center of the costmap
        """
        actual_origin = [int(height / 2) * self.resolution, int(width / 2) * self.resolution, 0]
        origin = np.array(self.meta_origin) + np.array(actual_origin)
        return Pose(origin.tolist())

    @staticmethod
    def _get_map() -> np.ndarray:
        """
        Receives the map array from the map_server converts it and into a numpy array.

        :return: The costmap as a numpy array.
        """
        print("Waiting for Map")
        map = rospy.wait_for_message("/map", OccupancyGrid)
        print("Recived Map")
        return np.array(map.data)

    @staticmethod
    def _get_map_metadata() -> MapMetaData:
        """
        Receives the meta-data about the costmap from the map_server and returns it.
        The meta-data contains things like, height, width, origin and resolution.

        :return: The meta-data for the costmap array.
        """
        print("Waiting for Map Meta Data")
        meta = rospy.wait_for_message("/map_metadata", MapMetaData)
        print("Recived Meta Data")
        return meta

    def _convert_map(self, map: np.ndarray) -> np.ndarray:
        """
        Converts the Occupancy Map received from ROS to be more consistent
        with how PyCRAM handles its costmap. Every possible cell for a robot to stand
        is set to one while anything else is set to zero. Additionally, this method
        also takes into account the distance_to_obstacle parameter and sets cell values
        that are too close to an obstacle to 0.

        :param map: The map that should be converted. Represented as 2d numpy array
        :return: The converted map. Represented as 2d numpy array.
        """
        map = np.pad(map, (int(self.distance_obstacle / 2), int(self.distance_obstacle / 2)))

        sub_shape = (self.distance_obstacle, self.distance_obstacle)
        view_shape = tuple(np.subtract(map.shape, sub_shape) + 1) + sub_shape
        strides = map.strides + map.strides

        sub_matrices = np.lib.stride_tricks.as_strided(map, view_shape, strides)
        sub_matrices = sub_matrices.reshape(sub_matrices.shape[:-2] + (-1,))
        sum = np.sum(sub_matrices, axis=2)
        return (sum == 0).astype('int16')

    def create_sub_map(self, sub_origin: Pose, size: int) -> Costmap:
        """
        Creates a smaller map from the overall occupancy map, the new map is centered
        around the point specified by "sub_origin" and has the size "size". The
        resolution of the costmap stays the same for the sub costmap.

        :param sub_origin: The point in global coordinate frame, around which the sub costmap should be centered.
        :param size: The size the sub costmap should have.
        :return: The sub costmap, represented as 2d numpy array.
        """
        # To ensure this is a numpy array
        sub_origin = np.array(sub_origin.position_as_list())
        # Since origin obtained from the meta data uses bottom left corner as reference.
        sub_origin *= -1
        # Calculates origin of sub costmap as vector between origin and given sub_origin
        new_origin = np.array(self.meta_origin) + sub_origin
        # Convert from vector in meter to index values
        new_origin /= self.resolution
        new_origin = np.abs(new_origin)
        # Offset to top left corner, for easier slicing
        new_origin = (new_origin - size / 2).astype(int)

        # slices a submap with size "size" around the given origin
        sub_map = self.original_map[new_origin[1]: new_origin[1] + size,
                  new_origin[0]: new_origin[0] + size]
        # Convert map to fit with the other costmaps
        sub_map = np.rot90(np.flip(self._convert_map(sub_map), 0))
        return Costmap(self.resolution, size, size, Pose(list(sub_origin * -1)), sub_map)

    def _create_from_world(self, size: int, resolution: float) -> np.ndarray:
        """
        Creates an Occupancy Costmap for the specified World.
        This map marks every position as valid that has no object above it. After
        creating the costmap the distance to obstacle parameter is applied.

        :param size: The size of this costmap. The size specifies the length of one side of the costmap. The costmap is created as a square.
        :param resolution: The resolution of this costmap. This determines how much meter a pixel in the costmap represents.
        """
        origin_position = self.origin.position_as_list()
        # Generate 2d grid with indices
        indices = np.concatenate(np.dstack(np.mgrid[int(-size / 2):int(size / 2), int(-size / 2):int(size / 2)]),
                                 axis=0) * resolution + np.array(origin_position[:2])
        # Add the z-coordinate to the grid, which is either 0 or 10
        indices_0 = np.pad(indices, (0, 1), mode='constant', constant_values=0)[:-1]
        indices_10 = np.pad(indices, (0, 1), mode='constant', constant_values=10)[:-1]
        # Zips both arrays such that there are tuples for every coordinate that
        # only differ in the z-coordinate
        rays = np.dstack(np.dstack((indices_0, indices_10))).T

        res = np.zeros(len(rays))
        # Using the World rayTest to check if there is an object above the position
        # if there is no object the position is marked as valid
        # 16383 is the maximal number of rays that can be processed in a batch
        i = 0
        j = 0
        for n in self._chunks(np.array(rays), 16380):
            with UseProspectionWorld():
                r_t = self.world.ray_test_batch(n[:, 0], n[:, 1], num_threads=0)
                while r_t is None:
                    r_t = self.world.ray_test_batch(n[:, 0], n[:, 1], num_threads=0)
                j += len(n)
                if World.robot:
                    shadow_robot = World.current_world.get_prospection_object_for_object(World.robot)
                    attached_objs = World.robot.attachments.keys()
                    attached_objs_shadow_id = [World.current_world.get_prospection_object_for_object(x).id for x
                                               in
                                               attached_objs]
                    res[i:j] = [
                        1 if ray[0] == -1 or ray[0] == shadow_robot.id or ray[0] in attached_objs_shadow_id else 0 for
                        ray in r_t]
                else:
                    res[i:j] = [1 if ray[0] == -1 else 0 for ray in r_t]
                i += len(n)

        res = np.flip(np.reshape(np.array(res), (size, size)))

        map = np.pad(res, (int(self.distance_obstacle / 2), int(self.distance_obstacle / 2)))

        sub_shape = (self.distance_obstacle * 2, self.distance_obstacle * 2)
        view_shape = tuple(np.subtract(map.shape, sub_shape) + 1) + sub_shape
        strides = map.strides + map.strides

        sub_matrices = np.lib.stride_tricks.as_strided(map, view_shape, strides)
        sub_matrices = sub_matrices.reshape(sub_matrices.shape[:-2] + (-1,))

        sum = np.sum(sub_matrices, axis=2)
        map = (sum == (self.distance_obstacle * 2) ** 2).astype('int16')
        # The map loses some size due to the strides and because I dont want to
        # deal with indices outside of the index range
        offset = self.size - map.shape[0]
        odd = 0 if offset % 2 == 0 else 1
        map = np.pad(map, (offset // 2, offset // 2 + odd))

        return np.flip(map)

    def _chunks(self, lst: List, n: int) -> List:
        """
        Yield successive n-sized chunks from lst.

        :param lst: The list from which chunks should be yielded
        :param n: Size of the chunks
        :return: A list of size n from lst
        """
        for i in range(0, len(lst), n):
            yield lst[i:i + n]


class VisibilityCostmap(Costmap):
    """
    A costmap that represents the visibility of a specific point for every position around
    this point. For a detailed explanation on how the creation of the costmap works
    please look here: `PhD Thesis (page 173) <https://mediatum.ub.tum.de/doc/1239461/1239461.pdf>`_
    """

    def __init__(self, min_height: float,
                 max_height: float,
                 size: Optional[int] = 100,
                 resolution: Optional[float] = 0.02,
                 origin: Optional[Pose] = None,
                 world: Optional[World] = None):
        """
        Visibility Costmaps show for every position around the origin pose if the origin can be seen from this pose.
        The costmap is able to deal with height differences of the camera while in a single position, for example, if
        the robot has a movable torso.

        :param min_height: This is the minimal height the camera can be. This parameter
            is mostly relevant if the vertical position of the camera can change.
        :param max_height: This is the maximal height the camera can be. This is
            mostly relevant if teh vertical position of the camera can change.
        :param size: The length of the side of the costmap, the costmap is created
            as a square.
        :param resolution: This parameter specifies how much meter a pixel in the
            costmap represents.
        :param origin: The pose in world coordinate frame around which the
            costmap should be created.
        :param world: The World for which the costmap should be created.
        """
        if (11 * size ** 2 + size ** 3) * 2 > psutil.virtual_memory().available:
            raise OSError("Not enough free RAM to calculate a costmap of this size")

        self.world = world if world else World.current_world
        self.map = np.zeros((size, size))
        self.size = size
        self.resolution = resolution
        # for pr2 = 1.27
        self.max_height: float = max_height
        # for pr2 = 1.6
        self.min_height: float = min_height
        self.origin: Pose = Pose() if not origin else origin
        self._generate_map()
        Costmap.__init__(self, resolution, size, size, self.origin, self.map)

    def _create_images(self) -> List[np.ndarray]:
        """
        Creates four depth images in every direction around the point
        for which the costmap should be created. The depth images are converted
        to metre, meaning that every entry in the depth images represents the
        distance to the next object in metre.

        :return: A list of four depth images, the images are represented as 2D arrays.
        """
        images = []
        camera_pose = self.origin

        with UseProspectionWorld():
            origin_copy = self.origin.copy()
            origin_copy.position.y += 1
            images.append(
                self.world.get_images_for_target(origin_copy, camera_pose, size=self.size)[1])

            origin_copy = self.origin.copy()
            origin_copy.position.x -= 1
            images.append(self.world.get_images_for_target(origin_copy, camera_pose, size=self.size)[1])

            origin_copy = self.origin.copy()
            origin_copy.position.y -= 1
            images.append(self.world.get_images_for_target(origin_copy, camera_pose, size=self.size)[1])

            origin_copy = self.origin.copy()
            origin_copy.position.x += 1
            images.append(self.world.get_images_for_target(origin_copy, camera_pose, size=self.size)[1])

        for i in range(0, 4):
            images[i] = self._depth_buffer_to_meter(images[i])
        return images

    def _depth_buffer_to_meter(self, buffer: np.ndarray) -> np.ndarray:
        """
        Converts the depth images generated by the World to represent
        each position in metre.

        :return: The depth image in metre
        """
        near = 0.2
        far = 100
        return far * near / (far - (far - near) * buffer)

    def _generate_map(self):
        """
        This method generates the resulting density map by using the algorithm explained
        in Lorenz Mösenlechners `PhD Thesis (page 178) <https://mediatum.ub.tum.de/doc/1239461/1239461.pdf>`_
        The resulting map is then saved to :py:attr:`self.map`
        """
        depth_imgs = self._create_images()
        # A 2D array where every cell contains the arctan2 value with respect to
        # the middle of the array. Additionally, the interval is shifted such that
        # it is between 0 and 2pi
        tan = np.arctan2(np.mgrid[-int(self.size / 2): int(self.size / 2), -int(self.size / 2): int(self.size / 2)][0],
                         np.mgrid[-int(self.size / 2): int(self.size / 2), -int(self.size / 2): int(self.size / 2)][
                             1]) + np.pi
        res = np.zeros(tan.shape)

        # Just for completion, since the res array has zeros in every position this
        # operation is not necessary.
        # res[np.logical_and(tan <= np.pi * 0.25, tan >= np.pi * 1.75)] = 0

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
        depth_indices[res == 3, 1:2] = self.size - indices[res == 3, :1] - 1

        # (size-x-1, y) for index between 0.75 pi and 1.25 pi
        depth_indices[res == 2, :1] = self.size - indices[res == 2, :1] - 1
        depth_indices[res == 2, 1:2] = indices[res == 2, 1:2]

        # (size-y-1, x) for index between 0.25 pi and 0.75 pi
        depth_indices[res == 1, :1] = self.size - indices[res == 1, 1:2] - 1
        depth_indices[res == 1, 1:2] = indices[res == 1, :1]

        # (x, y) for index between 0.25 pi and 1.75 pi
        depth_indices[res == 0, :1] = indices[res == 0, :1]
        depth_indices[res == 0, 1:2] = indices[res == 0, 1:2]

        # Convert back to origin in the middle of the costmap
        depth_indices[:, :, :1] -= self.size / 2
        depth_indices[:, :, 1:2] = np.absolute(self.size / 2 - depth_indices[:, :, 1:2])

        # Sets the y index for the coordinates of the middle of the costmap to 1,
        # the computed value is 0 which would cause an error in the next step where
        # the calculation divides the x coordinates by the y coordinates
        depth_indices[int(self.size / 2), int(self.size / 2), 1] = 1

        # Calculate columns for the respective position in the costmap
        columns = np.around(((depth_indices[:, :, :1] / depth_indices[:, :, 1:2])
                             * (self.size / 2)) + self.size / 2).reshape((self.size, self.size)).astype('int16')

        # An array with size * size that contains the euclidean distance to the
        # origin (in the middle of the costmap) from every cell
        distances = np.maximum(np.linalg.norm(np.dstack(np.mgrid[-int(self.size / 2): int(self.size / 2), \
                                                        -int(self.size / 2): int(self.size / 2)]), axis=2), 0.001)

        # Row ranges
        # Calculation of the ranges of coordinates in the row which have to be
        # taken into account. The range is from r_min to r_max.
        # These are two arrays with shape: size*size, the r_min constrains the beginning
        # of the range for every coordinate and r_max contains the end for each
        # coordinate
        r_min = (np.arctan((self.min_height - self.origin.position.z) / distances) * self.size) + self.size / 2
        r_max = (np.arctan((self.max_height - self.origin.position.z) / distances) * self.size) + self.size / 2

        r_min = np.minimum(np.around(r_min), self.size - 1).astype('int16')
        r_max = np.minimum(np.around(r_max), self.size - 1).astype('int16')

        rs = np.dstack((r_min, r_max + 1)).reshape((self.size ** 2, 2))
        r = np.arange(self.size)
        # Calculates a mask from the r_min and r_max values. This mask is for every
        # coordinate respectively and determines which values from the computed column
        # of the depth image should be taken into account for the costmap.
        # A Mask of a single coordinate has the length of the column of the depth image
        # and together with the computed column at this coordinate determines which
        # values of the depth image make up the value of the visibility costmap at this
        # point.
        mask = ((rs[:, 0, None] <= r) & (rs[:, 1, None] > r)).reshape((self.size, self.size, self.size))

        values = np.zeros((self.size, self.size))
        map = np.zeros((self.size, self.size))
        # This is done to iterate over the depth images one at a time
        for i in range(4):
            row_masks = mask[res == i].T
            # This statement does several things, first it takes the values from
            # the depth image for this quarter of the costmap. The values taken are
            # the complete columns of the depth image (which where computed beforehand)
            # and checks if the values in them are greater than the distance to the
            # respective coordinates. This does not take the row ranges into account.
            values = depth_imgs[i][:, columns[res == i].flatten()] > \
                     np.tile(distances[res == i][:, None], (1, self.size)).T * self.resolution
            # This applies the created mask of the row ranges to the values of
            # the columns which are compared in the previous statement
            masked = np.ma.masked_array(values, mask=~row_masks)
            # The calculated values are added to the costmap
            map[res == i] = np.sum(masked, axis=0)
        map /= np.max(map)
        # Weird flipping shit so that the map fits the orientation of the visualization.
        # the costmap in itself is consistent and just needs to be flipped to fit the world coordinate system
        map = np.flip(map, axis=0)
        map = np.flip(map)
        self.map = map


class GaussianCostmap(Costmap):
    """
    Gaussian Costmaps are 2D gaussian distributions around the origin with the given mean and sigma
    """

    def __init__(self, mean: int, sigma: float, resolution: Optional[float] = 0.02,
                 origin: Optional[Pose] = None):
        """
        This Costmap creates a 2D gaussian distribution around the origin with
        the specified size.

        :param mean: The mean input for the gaussian distribution, this also specifies
            the length of the side of the resulting costmap. The costmap is Created
            as a square.
        :param sigma: The sigma input for the gaussian distribution.
        :param resolution: The resolution of the costmap, this specifies how much
            meter a pixel represents.
        :param origin: The origin of the costmap around which it will be created.
        """
        self.gau: np.ndarray = self._gaussian_window(mean, sigma)
        self.map: np.ndarray = np.outer(self.gau, self.gau)
        self.size: float = mean
        self.origin: Pose = Pose() if not origin else origin
        Costmap.__init__(self, resolution, mean, mean, self.origin, self.map)

    def _gaussian_window(self, mean: int, std: float) -> np.ndarray:
        """
        This method creates a window of values with a gaussian distribution of
        size "mean" and standart deviation "std".
        Code from `Scipy <https://github.com/scipy/scipy/blob/v0.14.0/scipy/signal/windows.py#L976>`_
        """
        n = np.arange(0, mean) - (mean - 1.0) / 2.0
        sig2 = 2 * std * std
        w = np.exp(-n ** 2 / sig2)
        return w


class SemanticCostmap(Costmap):
    """
    Semantic Costmaps represent a 2D distribution over a link of an Object. An example of this would be a Costmap for a
    table surface.
    """

    def __init__(self, object, urdf_link_name, size=100, resolution=0.02, world=None):
        """
        Creates a semantic costmap for the given parameter. The semantic costmap will be on top of the link of the given
        Object.

        :param object: The object of which the link is a part
        :param urdf_link_name: The link name, as stated in the URDF
        :param resolution: Resolution of the final costmap
        :param world: The World from which the costmap should be created
        """
        self.world: World = world if world else World.current_world
        self.object: Object = object
        self.link: Link = object.get_link(urdf_link_name)
        self.resolution: float = resolution
        self.origin: Pose = object.get_link_pose(urdf_link_name)
        self.height: int = 0
        self.width: int = 0
        self.map: np.ndarray = []
        self.generate_map()

        Costmap.__init__(self, resolution, self.height, self.width, self.origin, self.map)

    def generate_map(self) -> None:
        """
        Generates the semantic costmap according to the provided parameters. To do this the axis aligned bounding box (AABB)
        for the link name will be used. Height and width of the final Costmap will be the x and y sizes of the AABB.
        """
        min_p, max_p = self.get_aabb_for_link().get_min_max_points()
        self.height = int((max_p.x - min_p.x) // self.resolution)
        self.width = int((max_p.y - min_p.y) // self.resolution)
        self.map = np.ones((self.height, self.width))

    def get_aabb_for_link(self) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box (AABB) of the link provided when creating this costmap. To try and let the
        AABB as close to the actual object as possible, the Object will be rotated such that the link will be in the
        identity orientation.

        :return: Two points in world coordinate space, which span a rectangle
        """
        prospection_object = World.current_world.get_prospection_object_for_object(self.object)
        with UseProspectionWorld():
            prospection_object.set_orientation(Pose(orientation=[0, 0, 0, 1]))
            link_pose_trans = self.link.transform
            inverse_trans = link_pose_trans.invert()
            prospection_object.set_orientation(inverse_trans.to_pose())
            return self.link.get_axis_aligned_bounding_box()


cmap = colors.ListedColormap(['white', 'black', 'green', 'red', 'blue'])


# Mainly used for debugging
# Data is 2d array
def plot_grid(data: np.ndarray) -> None:
    """
    An auxiliary method only used for debugging, it will plot a 2D numpy array using MatplotLib.
    """
    rows = data.shape[0]
    cols = data.shape[1]
    fig, ax = plt.subplots()
    ax.imshow(data, cmap=cmap)
    # draw gridlines
    # ax.grid(which='major', axis='both', linestyle='-', rgba_color='k', linewidth=1)
    ax.set_xticks(np.arange(0.5, rows, 1));
    ax.set_yticks(np.arange(0.5, cols, 1));
    plt.tick_params(axis='both', labelsize=0, length=0)
    # fig.set_size_inches((8.5, 11), forward=False)
    # plt.savefig(saveImageName + ".png", dpi=500)
    plt.show()
