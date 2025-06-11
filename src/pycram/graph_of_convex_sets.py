import time

import networkx as nx
import numpy as np
import plotly.graph_objects as go
from random_events.interval import reals
from random_events.product_algebra import SimpleEvent, Event
from rtree import index
from sortedcontainers import SortedSet
from tqdm import tqdm
from typing_extensions import Self, Optional, List

from .datastructures.dataclasses import BoundingBox, BoundingBoxCollection
from .datastructures.pose import PoseStamped
from .datastructures.world import World
from .failures import PlanFailure
from .ros import loginfo

try:
    from .ros_utils.viz_marker_publisher import TrajectoryPublisher, BoundingBoxPublisher
except ImportError:
    loginfo("Could not import TrajectoryPublisher. This is probably because you are not running ROS.")

class PoseOccupiedError(PlanFailure):
    """
    Error that is raised when a pose is occupied or not in the search space of a Connectivity Graphs.
    """

    def __init__(self, pose: PoseStamped):
        """
        Construct a new pose occupied error.
        :param pose: The pose that is occupied.
        """
        super().__init__(f"The pose {pose} is occupied.")
        self.pose = pose


class GraphOfConvexSets(nx.Graph):
    """
    A graph that represents the connectivity between convex sets.

    Every node in this graph represents a convex set.
    Every edge represents an adjacency between two convex sets.
    Furthermore, the adjacency is saved in and edge attribute called "intersection".
    """

    search_space: BoundingBoxCollection
    """
    The bounding box of the search space. Defaults to the entire three dimensional space.
    """

    def __init__(self, search_space: Optional[BoundingBoxCollection] = None):
        super().__init__()
        self.search_space = self._make_search_space(search_space)


    def calculate_connectivity(self, tolerance=0.001):
        """
        Faster connectivity:
        - No in-place node enlargement.
        - Uses precomputed tuples for R-tree queries.
        - Manual intersection test on original bounds.
        """

        # Build an R-tree in 3D
        p = index.Property()
        p.dimension = 3
        rtree_index = index.Index(properties=p)

        node_list = list(self.nodes)
        N = len(node_list)

        # Pre-extract original bounds (min/max in x,y,z) as tuples
        orig_mins = [None] * N
        orig_maxs = [None] * N
        expanded_bounds = [None] * N  # to insert into R-tree

        for i, node in enumerate(node_list):
            # Assume each BoundingBox has attributes .min_x,.min_y,.min_z,.max_x,.max_y,.max_z
            minx, miny, minz = node.min_x, node.min_y, node.min_z
            maxx, maxy, maxz = node.max_x, node.max_y, node.max_z

            orig_mins[i] = (minx, miny, minz)
            orig_maxs[i] = (maxx, maxy, maxz)

            # Expand by tolerance on all sides (for candidate search)
            ex_minx = minx - tolerance
            ex_miny = miny - tolerance
            ex_minz = minz - tolerance
            ex_maxx = maxx + tolerance
            ex_maxy = maxy + tolerance
            ex_maxz = maxz + tolerance

            expanded_bounds[i] = (ex_minx, ex_miny, ex_minz, ex_maxx, ex_maxy, ex_maxz)
            rtree_index.insert(i, expanded_bounds[i])

        # Now, for each node, query overlaps in the R-tree using the expanded box
        for i in tqdm(range(N), desc="Calculating connectivity"):
            min_i = orig_mins[i]
            max_i = orig_maxs[i]
            ex_bounds_i = expanded_bounds[i]

            # rtree returns indices j whose expanded bounds overlap ex_bounds_i
            for j in rtree_index.intersection(ex_bounds_i):
                if j <= i:
                    continue

                min_j = orig_mins[j]
                max_j = orig_maxs[j]

                # Manual AABB intersection test on original coords:
                # They intersect if they overlap in all three dimensions.
                if (
                        min_i[0] <= max_j[0] and min_j[0] <= max_i[0] and  # X overlap
                        min_i[1] <= max_j[1] and min_j[1] <= max_i[1] and  # Y overlap
                        min_i[2] <= max_j[2] and min_j[2] <= max_i[2]  # Z overlap
                ):
                    # Compute exact intersection bounds (original boxes, no tol-shift)
                    inter_min_x = max(min_i[0], min_j[0])
                    inter_min_y = max(min_i[1], min_j[1])
                    inter_min_z = max(min_i[2], min_j[2])

                    inter_max_x = min(max_i[0], max_j[0])
                    inter_max_y = min(max_i[1], max_j[1])
                    inter_max_z = min(max_i[2], max_j[2])

                    intersection_box = BoundingBox(
                        inter_min_x, inter_min_y, inter_min_z,
                        inter_max_x, inter_max_y, inter_max_z
                    )

                    # Add an edge: node_list[i] <--> node_list[j]
                    self.add_edge(node_list[i], node_list[j], intersection=intersection_box)

    def plot_free_space(self) -> List[go.Mesh3d]:
        """
        Plot the free space of the environment in blue.
        :return: A list of traces that can be put into a plotly figure.
        """
        free_space = Event(*[node.simple_event for node in self.nodes])
        return free_space.plot(color="blue")

    def plot_occupied_space(self) -> List[go.Mesh3d]:
        """
        Plot the occupied space of the environment in red.
        :return: A list of traces that can be put into a plotly figure.
        """
        free_space = Event(*[node.simple_event for node in self.nodes])
        occupied_space = ~free_space & self.search_space.event
        return occupied_space.plot(color="red")

    def node_of_pose(self, x, y, z) -> Optional[BoundingBox]:
        """
        Find the node that contains a point.

        :param x: The x-coordinate.
        :param y: The y-coordinate.
        :param z: The z-coordinate.
        :return: The node that contains the point or None if no node contains the point.
        """
        for node in self.nodes:
            if node.contains(x, y, z):
                return node
        return None

    def path_from_to(self, start: PoseStamped, goal: PoseStamped) -> Optional[List[PoseStamped]]:
        """
        Calculate a connected path from a start pose to a goal pose.

        :param start: The start pose.
        :param goal: The goal pose.
        :return: The path as a sequence of points to navigate to or None if no path exists.
        """

        # get poses from params
        start_node = self.node_of_pose(*start.position.to_list())
        goal_node = self.node_of_pose(*goal.position.to_list())

        # validate if the poses are part of the graph
        if start_node is None:
            raise PoseOccupiedError(start)
        if goal_node is None:
            raise PoseOccupiedError(goal)

        if start_node == goal_node:
            return [start, goal]

        # get the shortest path (perhaps replace with a*?)
        path = nx.shortest_path(self, start_node, goal_node)

        # if it is not possible to find a path
        if len(path) == 0:
            return None

        # build the path
        result = [start]

        for source, target in zip(path, path[1:]):
            intersection: BoundingBox = self[source][target]["intersection"]
            x_target = intersection.x_interval.center()
            y_target = intersection.y_interval.center()
            z_target = intersection.z_interval.center()
            result.append(PoseStamped.from_list([x_target, y_target, z_target], [0, 0, 0, 1]))

        result.append(goal)
        return result

    @classmethod
    def _make_search_space(cls, search_space: Optional[BoundingBoxCollection] = None):
        """
        Create the default search space if it is not given.
        """
        if search_space is None:
            search_space = BoundingBox(-np.inf, -np.inf, -np.inf, np.inf, np.inf, np.inf).as_collection()
        return search_space

    @classmethod
    def obstacles_of_world(cls, world: World, search_space: Optional[BoundingBoxCollection] = None, bloat_obstacles: float = 0.,
                           bloat_walls: float = 0.) -> Event:
        """
        Get all obstacles of the world besides the robot as a random event.
        """

        # create search space for calculations
        search_space = cls._make_search_space(search_space)
        search_event = search_space.event

        # initialize obstacles
        obstacles = None

        # create an event that describes the obstacles in the belief state of the robot
        for obj in world.objects:

            # don't take self-collision into account
            if obj.is_a_robot:
                continue

            # get the bounding box of every link in the object description
            for link in (obj.link_name_to_id.keys()):

                bbs = list(obj.get_link_bounding_box_collection(link))

                # plot_bounding_boxes_in_rviz(bbs)

                for bb in tqdm(bbs):

                    if "wall" in link.lower():
                        if bb.width > bb.depth:
                            bb = bb.bloat(bloat_walls, 0, 0.01)
                        else:
                            bb = bb.bloat(0, bloat_walls, 0.01)
                    else:
                        bb = bb.bloat(bloat_obstacles, bloat_obstacles, 0.01)
                    event = bb.simple_event.as_composite_set()
                    bb_event = event & search_event

                    # skip bounding boxes that are outside the search space
                    if bb_event.is_empty():
                        continue

                    # update obstacles
                    if obstacles is None:
                        obstacles = bb_event
                    else:
                        obstacles |= bb_event
        return obstacles

    # @staticmethod
    # def get_bounding_box(bb: AxisAlignedBoundingBox, link: Link) -> Event:
    #
    #     if link.origin is None:
    #         origin = TransformStamped()
    #     else:
    #         origin = link.origin.to_transform_stamped(link.name)
    #
    #     bb.get_rotated_box(origin)
    #
    #     corners = np.array([corner.to_list() for corner in bb.get_points()])
    #     corners += np.array(origin.position.to_list())
    #
    #     quat = link.pose.orientation.to_list()
    #     # rotate array to match the orientation of the link
    #     rotation_matrix = Rotation.from_quat(quat, scalar_first=False).as_matrix()
    #
    #     rotated = corners @ rotation_matrix.T
    #
    #     # Apply translation
    #     translated = rotated #+ corners
    #
    #     try:
    #         hull = ConvexHull(translated)
    #     except QhullError:
    #         variances = np.var(translated, axis=0)
    #         drop_dim = np.argmin(variances)
    #         # Project to 2D by removing the least informative axis
    #         reduced = np.delete(translated, drop_dim, axis=1)
    #         hull = ConvexHull(reduced)
    #
    #     A = hull.equations[:, :-1]
    #     b = -hull.equations[:, -1]
    #
    #     result = Polytope(A, b).outer_box_approximation(minimum_volume=np.inf)
    #     return result
    #

    @classmethod
    def get_doors_and_walls_of_world(cls, world: World, search_space: Optional[BoundingBoxCollection] = None,
                             bloat_walls: float = 0.) -> Event:
        """
        Get all walls (and doors) of the world as a random event.
        :param world: The world to get the walls from.
        :param search_space: The search space to limit the walls to.
        :param bloat_walls: The amount to bloat the walls.
        :return: An event that describes the walls in the world.
        """
        # create search space for calculations
        search_space = cls._make_search_space(search_space)
        search_event = search_space.event

        # initialize obstacles
        obstacles = None

        # create an event that describes the obstacles in the belief state of the robot
        for obj in world.objects:

            # don't take self-collision into account
            if obj.is_a_robot:
                continue

            # get the bounding box of every link in the object description
            for link in (obj.link_name_to_id.keys()):

                bbs = list(obj.get_link_bounding_box_collection(link))

                # plot_bounding_boxes_in_rviz(bbs)

                for bb in tqdm(bbs):


                    if any(x in link.lower() for x in ["wall", "door"]):
                        if bb.width > bb.depth:
                            bb = bb.bloat(bloat_walls, 0, 0.01)
                        else:
                            bb = bb.bloat(0, bloat_walls, 0.01)
                    else:
                        continue

                    event = bb.simple_event.as_composite_set()
                    bb_event = event & search_event

                    # skip bounding boxes that are outside the search space
                    if bb_event.is_empty():
                        continue

                    # update obstacles
                    if obstacles is None:
                        obstacles = bb_event
                    else:
                        obstacles |= bb_event
        return obstacles


    @classmethod
    def free_space_from_world(cls, world: World, tolerance=.001, search_space: Optional[BoundingBoxCollection] = None,
                              bloat_obstacles: float = 0., bloat_walls: float = 0.) -> Self:
        """
        Create a connectivity graph from the free space in the belief state of the robot.

        :param world: The belief state.
        :param tolerance: The tolerance for the intersection when calculating the connectivity.
        :param search_space: The search space for the connectivity graph.
        :return: The connectivity graph.
        """

        # create search space for calculations
        search_space = cls._make_search_space(search_space)
        search_event = search_space.event

        # get obstacles
        obstacles = cls.obstacles_of_world(world, search_space, bloat_obstacles, bloat_walls)

        start_time = time.time_ns()
        # calculate the free space and limit it to the searching space
        free_space = search_event - obstacles
        print(f"Obstacles calculated in {(time.time_ns() - start_time) / 1e6} ms")

        # create a connectivity graph from the free space and calculate the edges
        result = cls(search_space=search_space)
        result.add_nodes_from(BoundingBox.from_event(free_space))

        start_time = time.time_ns()
        result.calculate_connectivity(tolerance)
        print(f"Connectivity calculated in {(time.time_ns() - start_time) / 1e6} ms")

        return result

    @classmethod
    def navigation_map_from_world(cls, world: World, tolerance=.001,
                                  search_space: Optional[BoundingBoxCollection] = None, bloat_obstacles: float = 0.) -> Self:
        """
        Create a GCS from the free space in the belief state of the robot for navigation.
        The resulting GCS describes the paths for navigation, meaning that changing the z-axis position is not
        possible.
        Furthermore, it is taken into account that the robot has to fit through the entire space and not just
        through the floor level obstacles.

        :param world: The belief state.
        :param tolerance: The tolerance for the intersection when calculating the connectivity.
        :param search_space: The search space for the connectivity graph.
        :return: The connectivity graph.
        """

        xy = SortedSet([BoundingBox.x_variable, BoundingBox.y_variable])

        # create search space for calculations
        search_space = cls._make_search_space(search_space)

        # remove the z axis
        og_search_event = search_space.event
        search_event = og_search_event.marginal(xy)

        # initialize obstacles
        obstacles = None

        # create an event that describes the obstacles in the belief state of the robot
        for obj in world.objects:

            # don't take self-collision into account
            if obj.is_a_robot or obj.name == "floor":
                continue

            # get the bounding box of every link in the object description
            for link in (obj.link_name_to_id.keys()):
                bbs = list(obj.get_link_bounding_box_collection(link))

                for bb in tqdm(bbs):

                    bb = bb.bloat(bloat_obstacles, bloat_obstacles, 0.)

                    event = bb.simple_event.as_composite_set()

                    bb_event = event & search_event

                    # skip bounding boxes that are outside the search space
                    if bb_event.is_empty():
                        continue

                    bb_event = bb_event.marginal(xy)

                    # update obstacles
                    if obstacles is None:
                        obstacles = bb_event
                    else:
                        obstacles |= bb_event


        # get the 2d map from the obstacles
        search_event = search_event.marginal(xy)
        free_space = search_event - obstacles

        # create floor level
        z_event = SimpleEvent({BoundingBox.z_variable: reals()}).as_composite_set()
        z_event.fill_missing_variables(xy)
        free_space.fill_missing_variables(SortedSet([BoundingBox.z_variable]))
        free_space &= z_event
        free_space &= og_search_event

        # # update z for search space
        # search_space.z = SimpleInterval(0., 0., Bound.CLOSED, Bound.CLOSED)

        # create a connectivity graph from the free space and calculate the edges
        result = cls(search_space=search_space)
        result.add_nodes_from(BoundingBox.from_event(free_space))
        result.calculate_connectivity(tolerance)

        return result


def plot_path_in_rviz(path: List[PoseStamped]):
    """
    Plot a path in rviz.

    :param path: The path to plot.
    """

    def make_publisher():
        return TrajectoryPublisher()

    publisher = make_publisher()
    publisher.visualize_trajectory(path)

def plot_bounding_boxes_in_rviz(boxes: BoundingBoxCollection, duration=60.0):
    def make_publisher():
        return BoundingBoxPublisher()

    publisher = make_publisher()
    publisher.visualize(boxes, duration)