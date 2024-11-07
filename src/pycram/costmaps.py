# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import psutil
import random_events
import tf
from matplotlib import colors
from nav_msgs.msg import OccupancyGrid, MapMetaData
from probabilistic_model.probabilistic_circuit.nx.distributions import UniformDistribution
from probabilistic_model.probabilistic_circuit.nx.probabilistic_circuit import ProbabilisticCircuit, ProductUnit
from random_events.interval import Interval, reals, closed_open, closed
from random_events.product_algebra import Event, SimpleEvent
from random_events.variable import Continuous
from tqdm import tqdm
from typing_extensions import Tuple, List, Optional, Iterator
from scipy.spatial.transform import Rotation as R

from .datastructures.enums import Grasp
from .ros.ros_tools import wait_for_message
from .datastructures.dataclasses import AxisAlignedBoundingBox
from .datastructures.pose import Pose
from .datastructures.world import UseProspectionWorld
from .datastructures.world import World
from .description import Link
from .local_transformer import LocalTransformer
from .ros_utils.viz_marker_publisher import CostmapPublisher
from .world_concepts.world_object import Object

from .datastructures.pose import Pose, Transform
from .datastructures.world import World
from .datastructures.dataclasses import AxisAlignedBoundingBox, BoxVisualShape, Color
from tf.transformations import quaternion_from_matrix


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
    Base class for all costmaps, providing visualization of costmaps in the World.

    This class handles essential properties such as resolution, dimensions, and origin
    of the costmap, along with the costmap data itself.
    """

    def __init__(self, resolution: float,
                 height: int,
                 width: int,
                 origin: Pose,
                 map: np.ndarray,
                 world: Optional[World] = None):
        """
        Initializes the Costmap with specified resolution, dimensions, origin, and map data.

        Args:
            resolution (float): The real-world distance in meters represented by a single
                                entry in the costmap.
            height (int): The height of the costmap in grid cells.
            width (int): The width of the costmap in grid cells.
            origin (Pose): The origin of the costmap in world coordinates, centered
                           in the middle of the costmap.
            map (np.ndarray): A 2D numpy array representing the costmap data.
            world (Optional[World]): The World instance in which this costmap will be used.
                                     Defaults to the current world.
        """
        self.world = world if world else World.current_world
        self.resolution: float = resolution
        self.size: int = height
        self.height: int = height
        self.width: int = width
        local_transformer = LocalTransformer()
        self.origin = Pose(origin.position, [0, 0, 0, 1])
        self.origin: Pose = local_transformer.transform_pose(self.origin, 'map')
        self.map: np.ndarray = map
        self.vis_ids: List[int] = []

    def visualize(self) -> None:
        """
        Visualizes a costmap in the BulletWorld, the visualisation works by
        subdividing the costmap in rectangles which are then visualized as pybullet
        visual shapes.
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
                    curr_height = self._find_max_box_height((i, j), curr_width, map)
                    avg = np.average(map[i:i + curr_height, j:j + curr_width])
                    boxes.append([curr_pose, curr_height, curr_width, avg])
                    map[i:i + curr_height, j:j + curr_width] = 0
        cells = []
        # Creation of the visual shapes, for documentation of the visual shapes
        # please look here: https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.q1gn7v6o58bf
        for box in boxes:
            box = BoxVisualShape(Color(1, 0, 0, 0.6),
                                 [(box[0][0] + box[1] / 2) * self.resolution,
                                  (box[0][1] + box[2] / 2) * self.resolution, 0.],
                                 [(box[1] * self.resolution) / 2, (box[2] * self.resolution) / 2, 0.001])
            visual = self.world.create_visual_shape(box)
            cells.append(visual)
        # Set to 127 for since this is the maximal amount of links in a multibody
        for cell_parts in self._chunks(cells, 127):
            offset = [[-self.height / 2 * self.resolution, -self.width / 2 * self.resolution, 0.05], [0, 0, 0, 1]]
            origin_transform = (Transform(self.origin.position_as_list(), self.origin.orientation_as_list())
                                .get_homogeneous_matrix())
            offset_transform = (Transform(offset[0], offset[1]).get_homogeneous_matrix())
            new_pose_transform = np.dot(origin_transform, offset_transform)
            new_pose = Pose(new_pose_transform[:3, 3].tolist(), quaternion_from_matrix(new_pose_transform))
            map_obj = self.world.create_multi_body_from_visual_shapes(cell_parts, new_pose)
            self.vis_ids.append(map_obj)

    def publish(self, weighted: bool = False, scale: float = 0.4):
        """
        Publishes the costmap to the World, rendering it for visualization.

        This method iterates over all positions in the costmap where values are greater than zero,
        transforming them into poses relative to the world. Optionally, the map values can be
        visualized with scaled weights.

        Args:
            weighted (bool): If True, scales the z-coordinate of each pose based on the costmap
                             value to visualize "height" as weight. Defaults to False.
            scale (float): A scaling factor for the weight values when `weighted` is True.
                           Defaults to 0.4.
        """
        indices = np.argwhere(self.map > 0)
        height, width = self.map.shape
        center = np.array([height // 2, width // 2])
        origin_to_map = self.origin.to_transform("origin").invert()
        poses = []

        if weighted:
            weights = np.round(self.map[indices[:, 0], indices[:, 1]], 2)

        for idx, ind in enumerate(tqdm(indices)):
            vector = (center - ind) * self.resolution

            point_to_origin = Transform(
                [*vector, 0], frame="point", child_frame="origin"
            )

            point_in_map = (point_to_origin * origin_to_map).invert()

            position = point_in_map.translation_as_list()

            if weighted:
                position[2] = weights[idx] * scale

            poses.append(Pose(position))

        costmap_publisher = CostmapPublisher()
        costmap_publisher.publish(poses=poses, size=self.resolution, scale=scale)

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
            self.world.remove_visual_object(v_id)
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

    def merge_or_prioritize(self, other_cm: Costmap, prioritize_overlap: bool = False) -> Costmap:
        """
        Merges two costmaps, creating a new costmap with updated cell values.

        If `prioritize_overlap` is set to True, overlapping regions are prioritized
        by adding 1 to the overlapping cells in the base costmap, instead of completely merging them and only keeping the overlap.
        Otherwise, the merged costmap will combine values by multiplying overlapping cells.

        Merging conditions:

        1. The origin (x, y coordinates and orientation) of both costmaps must match.
        2. The resolution of both costmaps must be identical.

        If these conditions are not met, a `ValueError` is raised.

        Args:
            other_cm (Costmap): The other costmap to merge with this costmap.
            prioritize_overlap (bool): If True, prioritize overlapping regions by adding
                                       1 to the overlapping cells. Defaults to False.

        Returns:
            Costmap: A new costmap containing the merged values of both inputs.

        Raises:
            ValueError: If the origin or resolution of the costmaps do not match.
        """
        if self.origin.position.x != other_cm.origin.position.x or self.origin.position.y != other_cm.origin.position.y \
                or self.origin.orientation != other_cm.origin.orientation:
            raise ValueError("To merge costmaps, the x and y coordinates as well as the orientation must be equal.")
        elif self.resolution != other_cm.resolution:
            raise ValueError("To merge two costmaps their resolution must be equal.")

        if self.size == other_cm.size:
            smaller_map_padded = other_cm.map
            larger_cm = self
        else:
            larger_cm, smaller_cm = (self, other_cm) if self.size > other_cm.size else (other_cm, self)
            larger_size, smaller_size = larger_cm.size, smaller_cm.size
            offset = int(larger_size - smaller_size)
            odd = offset % 2 != 0
            smaller_map_padded = np.pad(smaller_cm.map, (offset // 2, offset // 2 + odd))

        dimensions = larger_cm.map.shape[0]
        new_map = np.zeros((dimensions, dimensions))
        overlap_region = np.logical_and(larger_cm.map > 0, smaller_map_padded > 0)

        if prioritize_overlap:
            original_map = np.copy(larger_cm.map) if self.size >= other_cm.size else np.copy(smaller_map_padded)
            original_map[overlap_region] += 1
            new_map = original_map
        else:
            new_map[overlap_region] = larger_cm.map[overlap_region] * smaller_map_padded[overlap_region]

        min_val = new_map.min()
        max_val = new_map.max()
        if max_val > min_val:
            normalized_map = (new_map - min_val) / (max_val - min_val)
        else:
            normalized_map = np.ones_like(new_map)

        normalized_map = (normalized_map / np.max(normalized_map)).reshape((dimensions, dimensions))
        return Costmap(larger_cm.resolution, dimensions, dimensions, larger_cm.origin, normalized_map)

    def __add__(self, other: Costmap) -> Costmap:
        """
        Overloads the "+" operator to merge two Costmaps.

        If the `other` parameter is not a Costmap, raises a `ValueError`. The merging
        process follows the same behavior as the `merge_or_prioritize` method with
        default settings.

        Args:
            other (Costmap): The other Costmap to merge with this Costmap.

        Returns:
            Costmap: A new Costmap containing merged values from this and the other Costmap.

        Raises:
            ValueError: If `other` is not a Costmap instance.
        """
        if isinstance(other, Costmap):
            return self.merge_or_prioritize(other)
        else:
            raise ValueError(f"Can only combine two costmaps, but received type {type(other)}")

    def __mul__(self, other: Costmap) -> Costmap:
        """
        Overloads the "*" operator for prioritizing overlapping areas in two Costmaps.

        Uses the `merge_or_prioritize` method with `prioritize_overlap=True` to return
        a new Costmap where weights in overlapping regions are increased.

        Args:
            other (Costmap): The other Costmap to prioritize overlap with this Costmap.

        Returns:
            Costmap: A new Costmap with prioritized overlapping weights.

        Raises:
            ValueError: If `other` is not a Costmap instance.
        """
        if isinstance(other, Costmap):
            return self.merge_or_prioritize(other, prioritize_overlap=True)
        else:
            raise ValueError(f"Can only multiply with another Costmap, but received type {type(other)}")

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
            self.distance_obstacle = max(int(distance_to_obstacle / self.resolution), 2)
            self.map = self._create_from_world(size, resolution)
            Costmap.__init__(self, resolution, self.size, self.size, self.origin, self.map)

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
        map = wait_for_message("/map", OccupancyGrid)
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
        meta = wait_for_message("/map_metadata", MapMetaData)
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

        :param size: The size of this costmap in centimeters. The size specifies the length of one side of the costmap. The costmap is created as a square.
        :param resolution: The resolution of this costmap. This determines how much meter a pixel in the costmap represents.
        """
        size_m = size / 100.0

        num_pixels = int(size_m / resolution)

        origin_position = self.origin.position_as_list()

        half_num_pixels = num_pixels // 2
        upper_bound = half_num_pixels if num_pixels % 2 == 0 else half_num_pixels + 1

        indices = np.concatenate(np.dstack(np.mgrid[-half_num_pixels:upper_bound,
                                           -half_num_pixels:upper_bound]),
                                 axis=0) * resolution + np.array(origin_position[:2])

        # Add the z-coordinate to the grid, which is either 0 or 10
        indices_0 = np.pad(indices, (0, 1), mode='constant', constant_values=5)[:-1]
        indices_10 = np.pad(indices, (0, 1), mode='constant', constant_values=0)[:-1]
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
            r_t = World.current_world.ray_test_batch(n[:, 0], n[:, 1], num_threads=0)
            while r_t is None:
                r_t = World.current_world.ray_test_batch(n[:, 0], n[:, 1], num_threads=0)
            j += len(n)
            if World.robot:
                attached_objs_id = [o.id for o in self.world.robot.attachments.keys()]
                res[i:j] = [
                    1 if ray[0] == -1 or ray[0] == self.world.robot.id or ray[0] in attached_objs_id else 0 for
                    ray in r_t]
            else:
                res[i:j] = [1 if ray[0] == -1 else 0 for ray in r_t]
            i += len(n)

        res = np.flip(np.reshape(np.array(res), (num_pixels, num_pixels)))

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
        self.size = num_pixels
        offset = num_pixels - map.shape[0]
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
    A costmap representing a 2D Gaussian distribution centered at the origin.

    This class generates a square Gaussian distribution with specified mean and sigma.
    Optionally, it can apply a circular mask to create a circular distribution.
    """

    def __init__(self, mean: int, sigma: float, resolution: Optional[float] = 0.02,
                 origin: Optional[Pose] = None, circular: bool = False, distance: float = 0.0):
        """
        Initializes a 2D Gaussian costmap with specified distribution parameters and options.

        Args:
            mean (int): The mean for the Gaussian distribution, which also determines the
                        side length of the costmap in centimeters.
            sigma (float): The standard deviation (sigma) of the Gaussian distribution.
            resolution (Optional[float]): The resolution of the costmap, representing the
                                          real-world meters per pixel. Defaults to 0.02.
            origin (Optional[Pose]): The origin of the costmap in world coordinates. If None,
                                     defaults to the center.
            circular (bool): If True, applies a circular mask to the costmap to create a
                             circular distribution. Defaults to False.
            distance (float): The distance from the origin where the Gaussian peak should be placed. Defaults to 0.0,
                              which centers the peak at the origin.

        """
        self.size = mean / 100.0
        self.resolution = resolution
        num_pixels = int(self.size / self.resolution)
        self.distance = distance
        self.map: np.ndarray = self._gaussian_costmap(num_pixels, sigma)
        if circular:
            self.map = self._apply_circular_mask(self.map, num_pixels)
        self.origin: Pose = Pose() if not origin else origin
        Costmap.__init__(self, resolution, num_pixels, num_pixels, self.origin, self.map)

    def _gaussian_costmap(self, mean: int, std: float) -> np.ndarray:
        """
        Generates a 2D Gaussian ring costmap centered at the origin, where the peak is
        located at a specified distance from the center.

        This method creates a Gaussian distribution around a ring centered on the costmap,
        allowing for a peak intensity at a specific radius from the center.

        Args:
            mean (int): The side length of the square costmap in pixels.
            std (float): The standard deviation (sigma) of the Gaussian distribution.

        Returns:
            np.ndarray: A 2D numpy array representing the Gaussian ring distribution.
        """
        radius_in_pixels = self.distance / self.resolution

        y, x = np.ogrid[:mean, :mean]
        center = (mean - 1) / 2.0
        distance_from_center = np.sqrt((x - center) ** 2 + (y - center) ** 2)

        ring_costmap = np.exp(-((distance_from_center - radius_in_pixels) ** 2) / (2 * std ** 2))
        return ring_costmap

    def _apply_circular_mask(self, grid: np.ndarray, num_pixels: int) -> np.ndarray:
        """
        Applies a circular mask to a 2D grid, setting values outside the circle to zero.

        The radius of the circular mask is half the size of the grid, so only values within
        this radius from the center will be retained.

        Args:
            grid (np.ndarray): The 2D Gaussian grid to apply the mask to.
            num_pixels (int): The number of pixels along one axis of the square grid.

        Returns:
            np.ndarray: The masked grid with values outside the circular area set to zero.
        """
        y, x = np.ogrid[:num_pixels, :num_pixels]
        center = num_pixels / 2
        radius = center
        distance_from_center = np.sqrt((x - center) ** 2 + (y - center) ** 2)
        circular_mask = distance_from_center <= radius
        masked_grid = np.where(circular_mask, grid, 0)
        return masked_grid


class DirectionalCostmap(Costmap):
    """
    A 2D costmap focused in specific directions relative to the origin pose.

    The costmap is oriented such that it emphasizes the negative x-direction and both
    positive and negative y-directions, while excluding the positive x-direction.
     Should go towards the direction of the grasp.
    """

    def __init__(self, size: int, face: Grasp, resolution: Optional[float] = 0.02,
                 origin: Optional[Pose] = None, has_object: bool = False):
        """
        Initializes a directional costmap focused in the direction of the grasp.

        Args:
            size (int): The side length of the costmap in centimeters.
            face (Grasp): The specific grasp direction for costmap alignment.
            resolution (Optional[float]): The resolution of the costmap in meters per pixel.
                                          Defaults to 0.02.
            origin (Optional[Pose]): The origin pose of the costmap, determining its orientation.
                                     Defaults to the world center.
            has_object (bool): If True, considers that an object is present and adjusts the
                               costmap accordingly. Defaults to False.
        """
        self.size = size / 100.0
        self.resolution = resolution
        self.face = face
        self.has_object = has_object
        num_pixels = int(self.size / self.resolution)
        self.origin = origin.copy() if origin else Pose()
        self.origin.position.z = 0
        self.map: np.ndarray = self._create_directional_map(num_pixels)
        Costmap.__init__(self, resolution, num_pixels, num_pixels, self.origin, self.map)

    def _create_directional_map(self, num_pixels: int) -> np.ndarray:
        """
        Creates a directional costmap based on Gaussian distributions, masking out the
        positive x-direction relative to the local frame of the origin pose.

        The orientation of the costmap is determined by the specified face direction
        (e.g., front, back, left, right), with a mask applied to exclude areas in
        the positive x-direction.

        Args:
            num_pixels (int): The number of pixels along one axis of the square grid.

        Returns:
            np.ndarray: A 2D numpy array representing the directional costmap.
        """
        object_orientation = self.origin.orientation_as_list()
        relative_rotations = {
            Grasp.FRONT: [0, 0, 0, 1],
            Grasp.BACK: [0, 0, 1, 0],
            Grasp.LEFT: [0, 0, -0.707, 0.707],
            Grasp.RIGHT: [0, 0, 0.707, 0.707],
            Grasp.TOP: [0, 0.707, 0, 0.707],
            Grasp.BOTTOM: [0, -0.707, 0, 0.707]
        }
        # currently doesnt really do anything for TOP and BOTTOM, but didnt make any problems either
        # and i havent had the time to investigate further or think of better handling for these cases
        # right now, TOP and BOTTOM are filtered out in the CostmapLocation
        # TODO: investigate and improve handling for TOP and BOTTOM
        face_rotation = relative_rotations[self.face]

        object_rotation = R.from_quat(object_orientation)
        face_rotation = R.from_quat(face_rotation)
        combined_rotation = object_rotation * face_rotation
        combined_orientation = combined_rotation.as_quat()

        map = np.ones((num_pixels, num_pixels))

        rotation = R.from_quat(combined_orientation)
        rotation_matrix = rotation.inv().as_matrix() if self.has_object else rotation.as_matrix()

        center = np.ceil(num_pixels / 2)
        x, y = np.meshgrid(np.arange(num_pixels), np.arange(num_pixels))
        x_offset = (x - center) * self.resolution
        y_offset = (y - center) * self.resolution

        coords = np.stack((x_offset, y_offset, np.zeros_like(x_offset)), axis=-1)
        transformed_coords = coords.dot(rotation_matrix.T)

        mask = transformed_coords[:, :, 1] >= 0

        directional_map = np.where(mask, 0, map)
        return directional_map


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

        :return: The axis aligned bounding box (AABB) of the link provided when creating this costmap. To try and let
         the AABB as close to the actual object as possible, the Object will be rotated such that the link will be in the
        identity orientation.
        """
        prospection_object = World.current_world.get_prospection_object_for_object(self.object)
        with UseProspectionWorld():
            prospection_object.set_orientation(Pose(orientation=[0, 0, 0, 1]))
            link_pose_trans = self.link.transform
            inverse_trans = link_pose_trans.invert()
            prospection_object.set_orientation(inverse_trans.to_pose())
            return self.link.get_axis_aligned_bounding_box()


class AlgebraicSemanticCostmap(SemanticCostmap):
    """
    Class for a semantic costmap that is based on an algebraic set-description of the valid area.
    """
    x: Continuous = Continuous("x")
    """
    The variable for height.
    """

    y: Continuous = Continuous("y")
    """
    The variable for width.
    """

    original_valid_area: Optional[SimpleEvent]
    """
    The original rectangle of the valid area.
    """

    valid_area: Optional[Event]
    """
    A description of the valid positions as set.
    """

    number_of_samples: int
    """
    The number of samples to generate for the iter.
    """

    def __init__(self, object, urdf_link_name, world=None, number_of_samples=1000):
        super().__init__(object, urdf_link_name, world=world)
        self.number_of_samples = number_of_samples

    def check_valid_area_exists(self):
        assert self.valid_area is not None, ("The map has to be created before semantics can be applied. "
                                             "Call 'generate_map first'")

    def left(self, margin=0.) -> Event:
        """
        Create an event left of the origins Y-Coordinate.
        :param margin: The margin of the events left bound.
        :return: The left event.
        """
        self.check_valid_area_exists()
        y_origin = self.origin.position.y
        left = self.original_valid_area[self.y].simple_sets[0].lower
        left += margin
        event = SimpleEvent(
            {self.x: reals(), self.y: random_events.interval.open(left, y_origin)}).as_composite_set()
        return event

    def right(self, margin=0.) -> Event:
        """
        Create an event right of the origins Y-Coordinate.
        :param margin: The margin of the events right bound.
        :return: The right event.
        """
        self.check_valid_area_exists()
        y_origin = self.origin.position.y
        right = self.original_valid_area[self.y].simple_sets[0].upper
        right -= margin
        event = SimpleEvent({self.x: reals(), self.y: closed_open(y_origin, right)}).as_composite_set()
        return event

    def top(self, margin=0.) -> Event:
        """
        Create an event above the origins X-Coordinate.
        :param margin: The margin of the events upper bound.
        :return: The top event.
        """
        self.check_valid_area_exists()
        x_origin = self.origin.position.x
        top = self.original_valid_area[self.x].simple_sets[0].upper
        top -= margin
        event = SimpleEvent(
            {self.x: random_events.interval.closed_open(x_origin, top), self.y: reals()}).as_composite_set()
        return event

    def bottom(self, margin=0.) -> Event:
        """
        Create an event below the origins X-Coordinate.
        :param margin: The margin of the events lower bound.
        :return: The bottom event.
        """
        self.check_valid_area_exists()
        x_origin = self.origin.position.x
        lower = self.original_valid_area[self.x].simple_sets[0].lower
        lower += margin
        event = SimpleEvent(
            {self.x: random_events.interval.open(lower, x_origin), self.y: reals()}).as_composite_set()
        return event

    def inner(self, margin=0.2):
        min_x = self.original_valid_area[self.x].simple_sets[0].lower
        max_x = self.original_valid_area[self.x].simple_sets[0].upper
        min_y = self.original_valid_area[self.y].simple_sets[0].lower
        max_y = self.original_valid_area[self.y].simple_sets[0].upper

        min_x += margin
        max_x -= margin
        min_y += margin
        max_y -= margin

        inner_event = SimpleEvent({self.x: closed(min_x, max_x),
                                   self.y: closed(min_y, max_y)}).as_composite_set()
        return inner_event

    def border(self, margin=0.2):
        return ~self.inner(margin)

    def generate_map(self) -> None:
        super().generate_map()
        valid_area = Event()
        for rectangle in self.partitioning_rectangles():
            # rectangle.scale(1/self.resolution, 1/self.resolution)
            rectangle.translate(self.origin.position.x, self.origin.position.y)
            valid_area.simple_sets.add(SimpleEvent({self.x: closed(rectangle.x_lower, rectangle.x_upper),
                                                    self.y: closed(rectangle.y_lower, rectangle.y_upper)}))

        assert len(valid_area.simple_sets) == 1, ("The map at the basis of a Semantic costmap must be an axis aligned"
                                                  "bounding box")
        self.valid_area = valid_area
        self.original_valid_area = self.valid_area.simple_sets[0]

    def as_distribution(self) -> ProbabilisticCircuit:
        p_xy = ProductUnit()
        u_x = UniformDistribution(self.x, self.original_valid_area[self.x].simple_sets[0])
        u_y = UniformDistribution(self.y, self.original_valid_area[self.y].simple_sets[0])
        p_xy.add_subcircuit(u_x)
        p_xy.add_subcircuit(u_y)

        conditional, _ = p_xy.conditional(self.valid_area)
        return conditional.probabilistic_circuit

    def sample_to_pose(self, sample: np.ndarray) -> Pose:
        """
        Convert a sample from the costmap to a pose.
        :param sample: The sample to convert
        :return: The pose corresponding to the sample
        """
        x = sample[0]
        y = sample[1]
        position = [x, y, self.origin.position.z]
        angle = np.arctan2(position[1] - self.origin.position.y, position[0] - self.origin.position.x) + np.pi
        orientation = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return Pose(position, orientation, self.origin.frame)

    def __iter__(self) -> Iterator[Pose]:
        model = self.as_distribution()
        samples = model.sample(self.number_of_samples)
        for sample in samples:
            yield self.sample_to_pose(sample)


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
