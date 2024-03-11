import dataclasses
import json
import time
from typing import Optional, List, Tuple

import portion
import sqlalchemy
import sqlalchemy.orm

from probabilistic_model.learning.jpt.jpt import JPT
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from random_events.events import Event
import numpy as np
import pybullet
import tf.transformations
from random_events.variables import Continuous
from sqlalchemy import select

import pycram.designators.location_designator
import pycram.task
from ...costmaps import OccupancyCostmap, plot_grid
from ...plan_failures import PlanFailure
from ...pose import Pose
from pycram.bullet_world import BulletWorld, Object

from ...orm.action_designator import PickUpAction
from ...orm.object_designator import Object
from ...orm.base import Position, RobotState, Pose as ORMPose, Quaternion
from ...orm.task import TaskTreeNode
from .database_location import Location, RequiresDatabase, Rectangle, AbstractCostmapLocation
from ...enums import TaskStatus

import pandas as pd

from probabilistic_model.probabilistic_circuit.probabilistic_circuit import DeterministicSumUnit, ProbabilisticCircuit, DecomposableProductUnit
from probabilistic_model.probabilistic_circuit.distributions import GaussianDistribution


class GaussianCostmapModel:
    """
    Class that generates a Gaussian Costmap around the center of an object. The costmap cuts out a square in the middle
    that has side lengths given by ``distance_to_center``.
    """

    distance_to_center: float
    """
    The side length of the cut out square.
    """

    variance: float
    """
    The variance of the distributions involved
    """

    relative_x = Continuous("relative_x")
    relative_y = Continuous("relative_y")

    def __init__(self, distance_to_center: float = 0., variance: float = 0.5):
        self.distance_to_center = distance_to_center
        self.variance = variance

    def create_model_with_center(self) -> ProbabilisticCircuit:
        """
        Create a fully factorized gaussian at the center of the map.
        """
        centered_model = DecomposableProductUnit()
        centered_model.add_subcircuit(GaussianDistribution(self.relative_x, 0., 0.5))
        centered_model.add_subcircuit(GaussianDistribution(self.relative_y, 0., 0.5))
        return centered_model.probabilistic_circuit

    def create_model(self) -> ProbabilisticCircuit:
        """
        Create a gaussian model that assumes mass everywhere besides the center square.

        :return: The probabilistic circuit
        """
        centered_model = self.create_model_with_center()

        model = DeterministicSumUnit()

        north_region = Event({self.relative_x: portion.closed(-self.distance_to_center, self.distance_to_center),
                              self.relative_y: portion.closed(self.distance_to_center, float("inf"))})
        south_region = Event({self.relative_x: portion.closed(-self.distance_to_center, self.distance_to_center),
                              self.relative_y: portion.closed(-float("inf"), -self.distance_to_center)})
        east_region = Event({self.relative_x: portion.closed(self.distance_to_center, float("inf")),
                             self.relative_y: portion.open(-float("inf"), float("inf"))})
        west_region = Event({self.relative_x: portion.closed(-float("inf"), -self.distance_to_center),
                             self.relative_y: portion.open(-float("inf"), float("inf"))})

        for region in [north_region, south_region, east_region, west_region]:
            conditional, probability = centered_model.conditional(region)
            model.add_subcircuit(conditional.root, probability)

        return model.probabilistic_circuit


class QueryBuilder(RequiresDatabase):

    def select_statement(self):
        return (select(PickUpAction.arm, PickUpAction.grasp, RobotState.torso_height, self.relative_x,
                       self.relative_y, Quaternion.x, Quaternion.y, Quaternion.z, TaskTreeNode.status)).distinct()


class JPTCostmapLocation(AbstractCostmapLocation):
    """
    Costmap Locations using Joint Probability Trees (JPTs).
    JPT costmaps are trained to model the dependency with a robot position relative to the object, the robots type,
    the objects type, the robot torso height, and the grasp parameters.
    Solutions to the problem definitions are chosen in such a way that the success probability is highest.
    """

    def __init__(self, target: Object, reachable_for=None, reachable_arm=None,
                 model: Optional[ProbabilisticCircuit] = None):
        """
        Create a JPT Costmap

        :param target: The target object
        :param reachable_for: The robot to grab the object with
        :param reachable_arm: The arm to use
        :param model: The JPT model as a loaded tree in memory, either model or path must be set
        """
        super().__init__(target, reachable_for, reachable_arm)
        self.model = model

        # initialize member for visualized objects
        self.visual_ids: List[int] = []

        # easy access to models variables
        (self.arm, self.grasp, self.relative_x, self.relative_y, self.status, self.torso_height, self.qx, self.qy,
         self.qz) = self.model.variables

    @classmethod
    def fit_from_database(cls, session: sqlalchemy.orm.session.Session, success_only: bool = False) -> (
            ProbabilisticCircuit):
        """
        Fit a JPT to become a location designator using the data from the database that is reachable via the session.

        :param session:
        :param success_only:
        :return:
        """
        query_builder = QueryBuilder(session)
        query = query_builder.create_query()
        if success_only:
            query = query.where(TaskTreeNode.status == TaskStatus.SUCCEEDED)
        samples = pd.read_sql(query, session.bind)
        samples = samples.rename(columns={"anon_1": "relative_x", "anon_2": "relative_y"})
        variables = infer_variables_from_dataframe(samples, scale_continuous_types=False)
        model = JPT(variables, min_samples_leaf=0.1)
        model.fit(samples)
        return model.probabilistic_circuit

    def events_from_occupancy_costmap(self) -> List[Event]:
        """
        Create a list of boxes that can be used as evidences for a jpt. The list of boxes describe areas where the
        robot can stand.

        :return: List of evidences describing the found boxes
        """

        events = []
        for rectangle in self.create_occupancy_rectangles():
            # get the occupancy costmap
            event = Event({self.relative_x: portion.closedopen(rectangle.x_lower, rectangle.x_upper),
                           self.relative_y: portion.closedopen(rectangle.y_lower, rectangle.y_upper)})
            events.append(event)

        return events

    def ground_model(self) -> DeterministicSumUnit:
        """
        Ground the model to the current evidence.
        """

        locations = self.events_from_occupancy_costmap()

        model = DeterministicSumUnit()

        for location in locations:
            conditional, probability = self.model.conditional(location)
            if probability > 0:
                model.add_subcircuit(conditional.root, probability)

        if len(model.subcircuits) == 0:
            raise PlanFailure("No possible locations found")

        model.normalize()
        return model

    def sample_to_location(self, sample: np.ndarray) -> Location:
        """
        Convert a numpy array sampled from the JPT to a costmap-location

        :param sample: The drawn sample
        :return: The usable costmap-location
        """
        sample_dict = {variable: value for variable, value in zip(self.model.variables, sample)}
        target_x, target_y, target_z = self.target.pose.position_as_list()
        pose = [target_x + sample_dict[self.relative_x], target_y + sample_dict[self.relative_y], 0]
        orientation = [sample_dict[self.qx], sample_dict[self.qy], sample_dict[self.qz], 1]
        torso_height = sample_dict[self.torso_height]
        result = Location(Pose(pose, orientation), sample_dict[self.arm], torso_height, sample_dict[self.grasp])
        return result

    def __iter__(self):
        model = self.ground_model()
        samples = model.sample(20)
        for sample in samples:
            yield self.sample_to_location(sample)

    def visualize(self):
        """
        Plot the possible areas to stand in the BulletWorld. The opacity is the probability of success.

        """
        raise NotImplementedError

    def close_visualization(self) -> None:
        """
        Close all plotted objects.

        """
        for id in self.visual_ids:
            pybullet.removeBody(id)
        self.visual_ids = []
