from enum import Enum, auto

import numpy as np
import tf
from tf.transformations import quaternion_from_matrix
from random_events.interval import closed, closed_open
from typing_extensions import Optional, Type

from . import World
from .costmaps import Costmap, OccupancyCostmap, VisibilityCostmap
from .datastructures.pose import Pose
from .units import meter, centimeter

from pint import Unit, Quantity
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

    origin: Pose

    size: Quantity
    resolution: Quantity

    distribution: Optional[ProbabilisticCircuit] = None

    def __init__(self, origin: Pose,
                 size: Quantity = 1 * meter,
                 resolution: Quantity = 20 * centimeter,
                 costmap_type: Type[Costmap] = OccupancyCostmap,
                 world: Optional[World] = None, **kwargs):
        self.world = world if world else World.current_world
        self.origin = origin
        self.size = size.to(centimeter)
        self.resolution = resolution.to(meter)

        if costmap_type == OccupancyCostmap:
            self.costmap = OccupancyCostmap(
                origin=self.origin,
                distance_to_obstacle=0.2,
                size=self.size.magnitude,
                resolution=self.resolution.magnitude,
                from_ros=False,
                world = self.world)
        else:
            raise NotImplementedError
        self.create_distribution()

    def create_distribution(self):
        """
        Create a probabilistic circuit from the costmap.
        """
        area = Event()
        for rectangle in self.costmap.partitioning_rectangles():
            rectangle.translate(self.origin.position.x, self.origin.position.y)
            area.simple_sets.add(SimpleEvent({self.x: closed_open(rectangle.x_lower, rectangle.x_upper),
                                                    self.y: closed_open(rectangle.y_lower, rectangle.y_upper)}))
        self.distribution = uniform_measure_of_event(area)

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