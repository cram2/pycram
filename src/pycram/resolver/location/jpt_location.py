from dataclasses import dataclass
from typing import Optional, List

import numpy as np
import pandas as pd
import portion
import pybullet
import sqlalchemy.orm
import tqdm
from probabilistic_model.learning.jpt.jpt import JPT
from probabilistic_model.learning.jpt.variables import infer_variables_from_dataframe
from probabilistic_model.probabilistic_circuit.distributions import GaussianDistribution, SymbolicDistribution
from probabilistic_model.probabilistic_circuit.probabilistic_circuit import DeterministicSumUnit, ProbabilisticCircuit, \
    DecomposableProductUnit
from random_events.events import Event
from random_events.variables import Continuous, Symbolic
from sqlalchemy import select
from tf import transformations

from .database_location import Location, RequiresDatabase, AbstractCostmapLocation
from ...bullet_world import BulletWorld
from ...costmaps import OccupancyCostmap, plot_grid, VisibilityCostmap
from ...designator import ObjectDesignatorDescription, ActionDesignatorDescription
from ...designators.actions.actions import PickUpActionPerformable, NavigateActionPerformable, ActionAbstract, \
    LookAtActionPerformable
from ...enums import TaskStatus, Grasp, Arms
from ...local_transformer import LocalTransformer
from ...orm.action_designator import PickUpAction as ORMPickUpAction, FaceAtAction as ORMFaceAtAction
from ...orm.base import RobotState, Quaternion
from ...orm.object_designator import Object
from ...orm.task import TaskTreeNode
from ...plan_failures import PlanFailure
from ...pose import Pose
from ...robot_descriptions import robot_description
from ...task import with_tree


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
    grasp = Symbolic("grasp", ["front", "left", "right"])
    arm = Symbolic("arm", ["left", "right"])

    def __init__(self, distance_to_center: float = 0., variance: float = 0.5):
        self.distance_to_center = distance_to_center
        self.variance = variance

    def create_model_with_center(self) -> ProbabilisticCircuit:
        """
        Create a fully factorized gaussian at the center of the map.
        """
        centered_model = DecomposableProductUnit()
        centered_model.add_subcircuit(GaussianDistribution(self.relative_x, 0., self.variance))
        centered_model.add_subcircuit(GaussianDistribution(self.relative_y, 0., self.variance))
        return centered_model.probabilistic_circuit

    def create_model(self) -> ProbabilisticCircuit:
        """
        Create a gaussian model that assumes mass everywhere besides the center square.

        :return: The probabilistic circuit
        """
        centered_model = self.create_model_with_center()

        region_model = DeterministicSumUnit()

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
            region_model.add_subcircuit(conditional.root, probability)

        result = DecomposableProductUnit()
        p_arms = SymbolicDistribution(self.arm, [1 / len(self.arm.domain) for _ in self.arm.domain])
        p_grasp = SymbolicDistribution(self.grasp, [1 / len(self.grasp.domain) for _ in self.grasp.domain])
        result.add_subcircuit(p_arms)
        result.add_subcircuit(p_grasp)
        result.add_subcircuit(region_model)

        return result.probabilistic_circuit


class QueryBuilder(RequiresDatabase):

    def select_statement(self):
        return (select(ORMPickUpAction.arm, ORMPickUpAction.grasp, RobotState.torso_height, self.relative_x,
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


@dataclass
class FaceAtPerformable(ActionAbstract):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose: Pose
    """
    The pose to face 
    """

    orm_class = ORMFaceAtAction

    @with_tree
    def perform(self) -> None:
        # get the robot position
        robot_position = BulletWorld.robot.pose

        # calculate orientation for robot to face the object
        angle = np.arctan2(robot_position.position.y - self.pose.position.y,
                           robot_position.position.x - self.pose.position.x) + np.pi
        orientation = list(transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))

        # create new robot pose
        new_robot_pose = Pose(robot_position.to_list()[0], orientation)

        # turn robot
        NavigateActionPerformable(new_robot_pose).perform()

        # look at target
        LookAtActionPerformable(self.pose).perform()


@dataclass
class FunkyPickUpActionPerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    arm: Arms
    """
    The arm to use
    """

    grasp: Grasp
    """
    The grasp to use
    """

    def perform(self):
        NavigateActionPerformable(self.standing_position).perform()
        FaceAtPerformable(self.object_designator.pose).perform()
        PickUpActionPerformable(self.object_designator, self.arm, self.grasp).perform()

class FunkyPickUpAction(ActionDesignatorDescription):
    """
    Cooler PickUpAction
    """

    sample_amount: int = 20

    @dataclass
    class Variables:
        arm: Symbolic
        grasp: Symbolic
        relative_x: Continuous
        relative_y: Continuous

    def __init__(self, object_designator: ObjectDesignatorDescription.Object, arms: List[str], grasps: List[str],
                 policy: Optional[ProbabilisticCircuit] = None):
        super().__init__(None)

        self.object_designator = object_designator
        self.arms = arms
        self.grasps = grasps

        if policy is None:
            policy = self.create_initial_policy()
        self.policy = policy

        self.variables = self.Variables(*self.policy.variables)

    @staticmethod
    def create_initial_policy() -> ProbabilisticCircuit:
        return GaussianCostmapModel(0.3, 0.5).create_model()

    def sample_to_action(self, sample: List) -> FunkyPickUpActionPerformable:
        arm, grasp, relative_x, relative_y = sample
        position = [relative_x, relative_y, 0.]
        pose = Pose(position, frame=self.object_designator.bullet_world_object.tf_frame)
        standing_position = LocalTransformer().transform_pose(pose, "map")
        standing_position.position.z = 0
        action = FunkyPickUpActionPerformable(standing_position, self.object_designator, arm, grasp)
        return action

    def events_from_occupancy_costmap(self) -> List[Event]:
        # create Occupancy costmap for the target object
        ocm = OccupancyCostmap(distance_to_obstacle=0.3, from_ros=False, size=200, resolution=0.02,
                               origin=self.object_designator.pose)
        vcm = VisibilityCostmap(min_height=1.27, max_height=1.69,
                                size=200, resolution=0.02, origin=self.object_designator.pose)
        mcm = ocm + vcm
        rectangles = AbstractCostmapLocation.create_occupancy_rectangles_from_map(mcm)

        events = []

        for rectangle in rectangles:
            event = Event({self.variables.relative_x: portion.open(rectangle.x_lower, rectangle.x_upper),
                           self.variables.relative_y: portion.open(rectangle.y_lower, rectangle.y_upper)})
            events.append(event)
        return events

    def ground_model(self, model: ProbabilisticCircuit, events: List[Event]) -> ProbabilisticCircuit:
        """
        Ground the model to the current evidence.
        """
        result = DeterministicSumUnit()

        for event in events:
            conditional, probability = model.conditional(event)
            if probability > 0:
                result.add_subcircuit(conditional.root, probability)

        if len(result.subcircuits) == 0:
            raise PlanFailure("No possible locations found")

        result.normalize()
        return result.probabilistic_circuit

    def iterate_without_occupancy_costmap(self):
        samples = self.policy.sample(self.sample_amount)
        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    def __iter__(self):
        model = self.ground_model(self.policy, self.events_from_occupancy_costmap())
        samples = model.sample(self.sample_amount)
        likelihoods = [model.likelihood(sample) for sample in samples]

        # sort samples by likelihood
        samples = [x for _, x in sorted(zip(likelihoods, samples), key=lambda pair: pair[0], reverse=True)]

        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    @staticmethod
    def query_for_database():
        query_builder = RequiresDatabase()
        query = select(ORMPickUpAction.arm, ORMPickUpAction.grasp, query_builder.relative_x,
                       query_builder.relative_y, ).distinct()
        query = query_builder.join_statement(query).where(TaskTreeNode.status == TaskStatus.SUCCEEDED)
        return query

    def try_out_policy(self):
        successful_tries = 0
        total_tries = 0

        pbar = tqdm.tqdm(self.iterate_without_occupancy_costmap(), total=self.sample_amount)

        for action in pbar:
            total_tries += 1
            try:
                action.perform()
                successful_tries += 1
            except PlanFailure as p:
                pass

            pbar.set_postfix({"Success Probability": successful_tries / total_tries})
            BulletWorld.current_bullet_world.reset_bullet_world()
