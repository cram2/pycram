import itertools


import networkx as nx
import plotly.graph_objects as go
from sortedcontainers import SortedSet
from typing_extensions import Self, Optional, List

from random_events.interval import SimpleInterval, Bound
from random_events.product_algebra import SimpleEvent, Event
from random_events.variable import Continuous
import matplotlib.pyplot as plt
import tqdm

from pycram import World
from pycram.datastructures.dataclasses import BoundingBox


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

    def calculate_connectivity(self, tolerance=.001):
        """
        Generate the edges for this graph.

        :param tolerance: The tolerance for the intersection.
        This value enlarges the boxes before the intersection to account for numeric imprecision.
        """

        number_of_nodes = len(self.nodes)

        for node in self.nodes:
            node.enlarge(tolerance, tolerance, tolerance, tolerance, tolerance, tolerance)

        for n1, n2 in tqdm.tqdm(itertools.combinations(self.nodes, 2), desc="Calculating connectivity",
                                total=number_of_nodes * (number_of_nodes - 1) // 2):

            intersection = n1.intersection_with(n2)

            if intersection:
                intersection.enlarge(-tolerance, -tolerance, -tolerance, -tolerance, -tolerance, -tolerance)
                self.add_edge(n1, n2, intersection=intersection)

        for node in self.nodes:
            node.enlarge(-tolerance, -tolerance, -tolerance, -tolerance, -tolerance, -tolerance)

    def plot(self) -> List[go.Mesh3d]:
        return [trace for node in self.nodes for trace in node.simple_event.plot()]

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

    def calculate_path(self, start, goal):
        ...

    @classmethod
    def free_space_from_world(cls, world: World, tolerance=.001, search_space: Optional[Box] = None) -> Self:

        obstacles = None

        # create an event that describes the obstacles in the belief state of the robot
        for obj in world.objects:
            if obj.is_a_robot:
                continue

            for link in obj.link_name_to_id.keys():
                bb = obj.get_link_axis_aligned_bounding_box(link)
                bb_as_simple_event = bounding_box_as_random_event(bb)

                if obstacles is None:
                    obstacles = bb_as_simple_event.as_composite_set()
                else:
                    obstacles |= bb_as_simple_event.as_composite_set()

        bounding_box_of_world = search_space.simple_event.__deepcopy__().as_composite_set()
        obstacles &= bounding_box_of_world
        free_space = ~obstacles
        result = cls()
        result.add_nodes_from(Box.from_event(free_space))
        result.calculate_connectivity()
        return result
