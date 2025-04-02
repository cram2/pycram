from enum import Enum, auto
from functools import cached_property

import numpy as np
from std_msgs.msg import ColorRGBA
from .tf_transformations import quaternion_from_euler
from random_events.interval import closed_open
from typing_extensions import Optional, Type
from visualization_msgs.msg import Marker, MarkerArray

from .datastructures.world import World
from .costmaps import Costmap, OccupancyCostmap, VisibilityCostmap
import matplotlib.colorbar
from .datastructures.pose import PoseStamped
from .ros import  create_publisher
from .units import meter

from pint import Quantity
from probabilistic_model.probabilistic_circuit.nx.helper import uniform_measure_of_event
from probabilistic_model.probabilistic_circuit.nx.probabilistic_circuit import ProbabilisticCircuit
from random_events.product_algebra import Event, SimpleEvent
from random_events.variable import Continuous


class Filter(Enum):
    OCCUPANCY = auto()
    VISIBILITY = auto()


class ProbabilisticCostmap:
    """
    A Costmap that uses probability distributions for representation.
    """

    x: Continuous = Continuous("x")
    """
    The variable for the x-axis (height) in meters.
    """

    y: Continuous = Continuous("y")
    """
    The variable for the y-axis (width) in meters.
    """

    costmap: Costmap
    """
    The legacy costmap.
    """

    origin: PoseStamped
    """
    The origin of the costmap.
    """

    size: Quantity
    """
    The side length of the costmap. The costmap is a square.
    """

    distribution: Optional[ProbabilisticCircuit] = None
    """
    The distribution associated with the costmap.
    """

    def __init__(self, origin: PoseStamped,
                 size: Quantity = 2 * meter,
                 max_cells = 10000,
                 costmap_type: Type[Costmap] = OccupancyCostmap,
                 world: Optional[World] = None):

        self.world = world if world else World.current_world
        self.origin = origin
        self.size = size

        # calculate the number of cells per axis
        number_of_cells = int(np.sqrt(max_cells))
        resolution = self.size.to(meter) / number_of_cells

        if costmap_type == OccupancyCostmap:
            robot_bounding_box = self.world.robot.get_axis_aligned_bounding_box()
            distance_to_obstacle = max(robot_bounding_box.width, robot_bounding_box.depth) / 2
            self.costmap = OccupancyCostmap(
                origin=self.origin,
                distance_to_obstacle=distance_to_obstacle,
                size=number_of_cells,
                resolution=resolution.magnitude,
                from_ros=False,
                world = self.world)
        elif costmap_type == VisibilityCostmap:
            camera = list(self.world.robot_description.cameras.values())[0]
            self.costmap = VisibilityCostmap(
                min_height=camera.minimal_height, max_height=camera.maximal_height, size=number_of_cells,
                resolution=resolution.magnitude, origin=self.origin, world=self.world)
        else:
            raise NotImplementedError(f"Unknown costmap type {costmap_type}")
        self.create_distribution()

    @cached_property
    def publisher(self):
        return create_publisher("/pycram/viz_marker", MarkerArray, queue_size=10)

    def create_event_from_map(self) -> Event:
        """
        :return: The event that is encoded by the costmaps map.
        """
        area = Event()
        for rectangle in self.costmap.partitioning_rectangles():
            rectangle.translate(self.origin.position.x, self.origin.position.y)
            area.simple_sets.add(SimpleEvent({self.x: closed_open(rectangle.x_lower, rectangle.x_upper),
                                                    self.y: closed_open(rectangle.y_lower, rectangle.y_upper)}))
        return area

    def create_distribution(self):
        """
        Create a probabilistic circuit from the costmap.
        """
        # self.distribution = fully_factorized([self.x, self.y], {self.x: self.origin.position.x,
        #                                                     self.y: self.origin.position.y},
        #                                      {self.x: 1, self.y: 1})
        # self.distribution, _ = self.distribution.conditional(self.create_event_from_map())
        self.distribution = uniform_measure_of_event(self.create_event_from_map())

    def sample_to_pose(self, sample: np.ndarray) -> PoseStamped:
        """
        Convert a sample from the costmap to a pose.

        :param sample: The sample to convert
        :return: The pose corresponding to the sample
        """
        x = sample[0]
        y = sample[1]
        position = [x, y, self.origin.position.z]
        angle = np.arctan2(position[1] - self.origin.position.y, position[0] - self.origin.position.x) + np.pi
        orientation = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return PoseStamped(position, orientation, self.origin.frame_id)

    def visualize(self):
        """
        Visualize the costmap for rviz.
        """
        samples = self.distribution.sample(1000)
        likelihoods = self.distribution.likelihood(samples)
        likelihoods /= max(likelihoods)

        colorscale = matplotlib.cm.get_cmap("inferno")
        marker = Marker()
        marker.type = Marker.POINTS
        marker.id = 0
        marker.action = Marker.ADD
        marker.header.frame_id = self.origin.frame_id
        marker.pose = PoseStamped().pose
        # marker.lifetime = Duration(60)
        marker.scale.x = 0.05
        marker.scale.y = 0.05


        for index, (sample, likelihood) in enumerate(zip(samples, likelihoods)):
            position = self.sample_to_pose(sample).pose.position
            position.z = 0.1
            marker.points.append(position)
            marker.colors.append(ColorRGBA(**dict(zip(["r", "g", "b","a"], [*colorscale(likelihood)[:3], 1.0]))))

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.publisher.publish(marker_array)
