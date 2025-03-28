import itertools

import networkx as nx
import numpy as np
import plotly.graph_objects as go
from random_events.interval import SimpleInterval, Bound, singleton
from random_events.product_algebra import SimpleEvent, Event
from sortedcontainers import SortedSet
from tqdm import tqdm
from typing_extensions import Self, Optional, List

from .datastructures.world import World
from .datastructures.dataclasses import BoundingBox
from .datastructures.pose import PoseStamped
from .failures import PlanFailure
from .ros_utils.viz_marker_publisher import TrajectoryPublisher


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

    search_space: BoundingBox
    """
    The bounding box of the search space. Defaults to the entire three dimensional space.
    """

    def __init__(self, search_space: Optional[BoundingBox] = None):
        super().__init__()
        self.search_space = self._make_search_space(search_space)

    def calculate_connectivity(self, tolerance=.001):
        """
        Generate the edges for this graph.

        :param tolerance: The tolerance for the intersection.
        This value enlarges the boxes before the intersection to account for numeric imprecision.
        """

        number_of_nodes = len(self.nodes)

        # make the bounding boxes a bit larger
        [node.enlarge_all(tolerance) for node in self.nodes]

        def check_connectivity(node1: BoundingBox, node2: BoundingBox):
            """
            Check the connectivity between two nodes.
            Add an edge if they are connected.
            :param node1: The first node.
            :param node2: The second node.
            """
            intersection = node1.intersection_with(node2)
            if intersection:
                intersection.enlarge_all(-tolerance)
                self.add_edge(node1, node2, intersection=intersection)

        # calculate the connectivity for each edge pair
        [check_connectivity(n1, n2) for n1, n2 in tqdm(itertools.combinations(self.nodes, 2),
                                                       desc="Calculating connectivity",
                                                       total=number_of_nodes * (number_of_nodes - 1) // 2)]

        # recreate the original bounding boxes
        [node.enlarge_all(-tolerance) for node in self.nodes]

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
        occupied_space = ~free_space & self.search_space.simple_event.as_composite_set()
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
        start_node = self.node_of_pose(*start.position.to_list()())
        goal_node = self.node_of_pose(*goal.position.to_list()())

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
            result.append(PoseSteamped.from_list([x_target, y_target, z_target], [0, 0, 0, 1]))

        result.append(goal)
        return result

    @classmethod
    def _make_search_space(cls, search_space: Optional[BoundingBox] = None):
        """
        Create the default search space if it is not given.
        """
        if search_space is None:
            search_space = BoundingBox(-np.inf, -np.inf, -np.inf,
                                       np.inf, np.inf, np.inf)
        return search_space

    @classmethod
    def obstacles_of_world(cls, world: World, search_space: Optional[BoundingBox] = None) -> Event:
        """
        Get all obstacles of the world besides the robot as a random event.
        """

        # create search space for calculations
        search_space = cls._make_search_space(search_space)
        search_event = search_space.simple_event.as_composite_set()

        # initialize obstacles
        obstacles = None

        # create an event that describes the obstacles in the belief state of the robot
        for obj in world.objects:

            # don't take self-collision into account
            if obj.is_a_robot:
                continue

            # get the bounding box of every link in the object description
            for link in obj.link_name_to_id.keys():
                bb = obj.get_link_axis_aligned_bounding_box(link)

                # limit bounding box to search space
                bb_event = bb.simple_event.as_composite_set() & search_event

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
    def free_space_from_world(cls, world: World, tolerance=.001, search_space: Optional[BoundingBox] = None) -> Self:
        """
        Create a connectivity graph from the free space in the belief state of the robot.

        :param world: The belief state.
        :param tolerance: The tolerance for the intersection when calculating the connectivity.
        :param search_space: The search space for the connectivity graph.
        :return: The connectivity graph.
        """

        # create search space for calculations
        search_space = cls._make_search_space(search_space)
        search_event = search_space.simple_event.as_composite_set()

        # get obstacles
        obstacles = cls.obstacles_of_world(world, search_space)

        # calculate the free space and limit it to the searching space
        free_space = ~obstacles & search_event

        # create a connectivity graph from the free space and calculate the edges
        result = cls(search_space=search_space)
        result.add_nodes_from(BoundingBox.from_event(free_space))
        result.calculate_connectivity(tolerance)

        return result

    @classmethod
    def navigation_map_from_world(cls, world: World, tolerance=.001,
                                  search_space: Optional[BoundingBox] = None) -> Self:
        """
        Create a GCS from the free space in the belief state of the robot for navigation.
        The resulting GCS describes the paths for navigation, meaning that changing the z-axis position is not
        possible.
        Furthermore, it is taken into account that the robot has to fit through the entire space and not just
        through the floor level obstacles.

        TODO: i think explicitly 2d boxes would help here.

        :param world: The belief state.
        :param tolerance: The tolerance for the intersection when calculating the connectivity.
        :param search_space: The search space for the connectivity graph.
        :return: The connectivity graph.
        """

        xy = SortedSet([BoundingBox.x_variable, BoundingBox.y_variable])

        # create search space for calculations
        search_space = cls._make_search_space(search_space)

        # remove the z axis
        search_event = search_space.simple_event.as_composite_set()
        search_event = search_event.marginal(xy)

        # get obstacles
        obstacles = cls.obstacles_of_world(world, search_space)

        # get the 2d map from the obstacles
        obstacles = obstacles.marginal(xy)
        free_space = ~obstacles & search_event

        # create floor level
        z_event = SimpleEvent({BoundingBox.z_variable: singleton(0.)}).as_composite_set()
        z_event.fill_missing_variables(xy)
        free_space.fill_missing_variables(SortedSet([BoundingBox.z_variable]))
        free_space &= z_event

        # update z for search space
        search_space.z = SimpleInterval(0., 0., Bound.CLOSED, Bound.CLOSED)

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
