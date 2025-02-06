import itertools

import networkx as nx
import numpy as np
import plotly.graph_objects as go
import tqdm
from random_events.interval import SimpleInterval, Bound
from random_events.product_algebra import SimpleEvent, Event
from random_events.variable import Continuous
from typing_extensions import Self, Optional, List

from pycram import World
from pycram.datastructures.dataclasses import BoundingBox
from pycram.datastructures.pose import Pose
from pycram.failures import PlanFailure


class PoseOccupiedError(PlanFailure):
    """
    Error that is raised when a pose is occupied or not in the search space of a Connectivity Graphs.
    """

    def __init__(self, pose: Pose):
        """
        Construct a new pose occupied error.
        :param pose: The pose that is occupied.
        """
        super().__init__(f"The pose {pose} is occupied.")
        self.pose = pose


class Box:
    """
    A box in 3D space.
    A box is constructed by the cartesian product of three simple intervals.

    """
    simple_event: SimpleEvent
    """
    The algebraic simple event associated with this box.
    """

    x = Continuous("x")
    """
    The x-axis variable.
    """

    y = Continuous("y")
    """
    The y-axis variable.
    """

    z = Continuous("z")
    """
    The z-axis variable.
    """

    def __init__(self, x: SimpleInterval = SimpleInterval(0, 1),
                 y: SimpleInterval = SimpleInterval(0, 1),
                 z: SimpleInterval = SimpleInterval(0, 1)):
        """
        Construct a new box.
        :param x: The interval for the x-axis.
        :param y: The interval for the y-axis.
        :param z: The interval for the z-axis.
        """
        self.simple_event = SimpleEvent({self.x: x, self.y: y, self.z: z})

    def __repr__(self):
        return f"Box(x={self.simple_event[self.x].simple_sets[0]}, " \
               f"y={self.simple_event[self.y].simple_sets[0]}, " \
               f"z={self.simple_event[self.z].simple_sets[0]})"

    def scale(self, x: float = 1., y: float = 1., z: float = 1.):
        """
        Scale the box in-place.

        :param x: The x-axis scaling factor.
        :param y: The y-axis scaling factor.
        :param z: The z-axis scaling factor.
        """
        for variable, scale in zip([self.x, self.y, self.z], [x, y, z]):
            self.simple_event[variable].simple_sets[0].lower *= scale
            self.simple_event[variable].simple_sets[0].upper *= scale

    def enlarge(self, x_upper: float = 0., x_lower: float = 0.,
                y_upper: float = 0., y_lower: float = 0.,
                z_upper: float = 0., z_lower: float = 0.):
        """
        Enlarge the box in-place.

        :param x_upper: The enlargement of the x-axis upper bound.
        :param x_lower: The enlargement of the x-axis lower bound.
        :param y_upper: The enlargement of the y-axis upper bound.
        :param y_lower: The enlargement of the y-axis lower bound.
        :param z_upper: The enlargement of the z-axis upper bound.
        :param z_lower: The enlargement of the z-axis lower bound.
        """
        self.simple_event[self.x].simple_sets[0].upper += x_upper
        self.simple_event[self.x].simple_sets[0].lower -= x_lower
        self.simple_event[self.y].simple_sets[0].upper += y_upper
        self.simple_event[self.y].simple_sets[0].lower -= y_lower
        self.simple_event[self.z].simple_sets[0].upper += z_upper
        self.simple_event[self.z].simple_sets[0].lower -= z_lower

    @property
    def x_interval(self):
        """
        :return: The interval for the x-axis.
        """
        return self.simple_event[self.x].simple_sets[0]

    @property
    def y_interval(self):
        """
        :return: The interval for the y-axis.
        """
        return self.simple_event[self.y].simple_sets[0]

    @property
    def z_interval(self):
        """
        :return: The interval for the z-axis.
        """
        return self.simple_event[self.z].simple_sets[0]

    def intersection_with(self, other: Self) -> Optional[Self]:
        """
        Compute the intersection of this box with another box.

        :param other: The other box.
        :return: The intersection or None if the intersection is empty.
        """
        intersection = self.simple_event.__deepcopy__().intersection_with(other.simple_event.__deepcopy__())
        if intersection.is_empty():
            return None

        return Box(x=intersection[self.x].simple_sets[0], y=intersection[self.y].simple_sets[0],
                   z=intersection[self.z].simple_sets[0])

    def __hash__(self):
        return hash(id(self.simple_event))

    @classmethod
    def from_event(cls, event: Event) -> List[Self]:
        result = []
        for simple_event in event.simple_sets:
            simple_event: SimpleEvent

            for x, y, z in itertools.product(simple_event[cls.x].simple_sets,
                                             simple_event[cls.y].simple_sets,
                                             simple_event[cls.z].simple_sets):
                result.append(cls(x, y, z))
        return result

    def contains(self, x: float, y: float, z: float):
        """
        Check if a point is contained in the box.

        :param x: The x-coordinate.
        :param y: The y-coordinate.
        :param z: The z-coordinate.
        :return: True if the point is contained in the box.
        """
        return (self.simple_event[self.x].contains(x) and self.simple_event[self.y].contains(y)
                and self.simple_event[self.z].contains(z))


def bounding_box_as_random_event(bb: BoundingBox) -> SimpleEvent:
    """
    Convert a bounding box from the pycram world datastructures to a random event.

    :param bb: The bounding box.
    :return: The random event.
    """
    x = SimpleInterval(bb.min_x, bb.max_x, Bound.CLOSED, Bound.CLOSED)
    y = SimpleInterval(bb.min_y, bb.max_y, Bound.CLOSED, Bound.CLOSED)
    z = SimpleInterval(bb.min_z, bb.max_z, Bound.CLOSED, Bound.CLOSED)
    return SimpleEvent({Box.x: x, Box.y: y, Box.z: z})


class ConnectivityGraph(nx.Graph):
    """
    A graph that represents the connectivity of convex sets.

    Every node in this graph represents a convex set.
    The nodes must not intersect.
    Every edge represents an adjacency between two convex sets.
    """

    bounding_box: Box
    """
    The bounding box of the search space. Defaults to the entire three dimensional space.
    """

    def __init__(self, bounding_box: Optional[Box] = None):
        super().__init__()
        if bounding_box is None:
            bounding_box = Box(x=SimpleInterval(-np.inf, np.inf),
                               y=SimpleInterval(-np.inf, np.inf),
                               z= SimpleInterval(-np.inf, np.inf))

        self.bounding_box = bounding_box

    def calculate_connectivity(self, tolerance=.001):
        """
        Generate the edges for this graph.

        :param tolerance: The tolerance for the intersection.
        This value enlarges the boxes before the intersection to account for numeric imprecision.
        """

        number_of_nodes = len(self.nodes)

        # make the bounding boxes a bit larger
        for node in self.nodes:
            node.enlarge(tolerance, tolerance, tolerance, tolerance, tolerance, tolerance)

        # calculate the connectivity for each edge pair
        for n1, n2 in tqdm.tqdm(itertools.combinations(self.nodes, 2), desc="Calculating connectivity",
                                total=number_of_nodes * (number_of_nodes - 1) // 2):

            intersection = n1.intersection_with(n2)

            # if they are connected, save the intersection
            if intersection:
                intersection.enlarge(-tolerance, -tolerance, -tolerance, -tolerance, -tolerance, -tolerance)
                self.add_edge(n1, n2, intersection=intersection)

        # recreate the original bounding boxes
        for node in self.nodes:
            node.enlarge(-tolerance, -tolerance, -tolerance, -tolerance, -tolerance, -tolerance)

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
        occupied_space = ~free_space & self.bounding_box.simple_event.as_composite_set()
        return occupied_space.plot(color="red")

    def node_of_pose(self, x, y, z) -> Optional[Box]:
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

    def path_from_to(self, start: Pose, goal: Pose) -> Optional[List[Pose]]:
        """
        Calculate a connected path from a start pose to a goal pose.

        :param start: The start pose.
        :param goal: The goal pose.
        :return: The path as a sequence of points to navigate to or None if no path exists.
        """

        # get poses from params
        start_node = self.node_of_pose(*start.position_as_list())
        goal_node = self.node_of_pose(*goal.position_as_list())

        if self.bounding_box is not None:
            if not self.bounding_box.contains(*start.position_as_list()):
                raise PoseOccupiedError(start)
            if not self.bounding_box.contains(*goal.position_as_list()):
                raise PoseOccupiedError(goal)

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
            intersection: Box = self[source][target]["intersection"]
            x_target = intersection.x_interval.center()
            y_target = intersection.y_interval.center()
            z_target = intersection.z_interval.center()
            result.append(Pose([x_target, y_target, z_target], [0, 0, 0, 1]))

        result.append(goal)
        return result

    @classmethod
    def free_space_from_world(cls, world: World, tolerance=.001, bounding_box: Optional[Box] = None) -> Self:
        """
        Create a connectivity graph from the free space in the belief state of the robot.

        :param world: The belief state.
        :param tolerance: The tolerance for the intersection when calculating the connectivity.
        :param bounding_box: The search space for the connectivity graph.
        :return: The connectivity graph.
        """

        obstacles = None

        # create an event that describes the obstacles in the belief state of the robot
        for obj in world.objects:

            # don't take self-collision into account
            if obj.is_a_robot:
                continue

            # get the bounding box of every link in the object description
            for link in obj.link_name_to_id.keys():
                bb = obj.get_link_axis_aligned_bounding_box(link)
                bb_as_simple_event = bounding_box_as_random_event(bb)

                if obstacles is None:
                    obstacles = bb_as_simple_event.as_composite_set()
                else:
                    obstacles |= bb_as_simple_event.as_composite_set()

        # limit the search space
        bounding_box_of_world = bounding_box.simple_event.__deepcopy__().as_composite_set()
        obstacles &= bounding_box_of_world

        # calculate the free space
        free_space = ~obstacles & bounding_box_of_world

        # create a connectivity graph from the free space and calculate the edges
        result = cls(bounding_box=bounding_box)
        result.add_nodes_from(Box.from_event(free_space))
        result.calculate_connectivity(tolerance)

        return result

def plot_path_in_rviz(path: List[Pose]):
    """
    Plot a path in rviz.
    :param path: The path to plot.
    """
    ...
