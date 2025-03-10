import json
from dataclasses import fields, dataclass
from enum import IntEnum

import random_events
import tqdm
from probabilistic_model.probabilistic_circuit.nx.helper import fully_factorized
from probabilistic_model.probabilistic_circuit.nx.probabilistic_circuit import ProbabilisticCircuit
from probabilistic_model.probabilistic_model import ProbabilisticModel
from random_events.interval import SimpleInterval
from random_events.product_algebra import Event, SimpleEvent
from random_events.set import SetElement, Set
from random_events.variable import Symbolic, Continuous, Variable
from sortedcontainers import SortedSet
from sqlalchemy import select
from typing_extensions import Optional, List, Iterator

from ...action_designator import MoveAndPickUpAction, ActionAbstract, MoveAndPlaceAction
from ....costmaps import OccupancyCostmap, VisibilityCostmap
from ....datastructures.enums import Arms as EArms, Grasp as EGrasp, TaskStatus
from ....datastructures.pose import Pose
from ....datastructures.world import World
from ....designator import ObjectDesignatorDescription, ActionDescription
from ....failures import ObjectUnreachable, PlanFailure
from ....local_transformer import LocalTransformer
from ....orm.views import PickUpWithContextView, PlaceWithContextView
from ....world_concepts.world_object import Object


class Grasp(IntEnum):
    FRONT = 0
    LEFT = 1
    RIGHT = 2
    TOP = 3


class Arms(IntEnum):
    LEFT = 0
    RIGHT = 1


class ProbabilisticAction:
    """
    Abstract class for performables that have a probabilistic parametrization.
    """

    @dataclass
    class Variables:
        """
        Variables for probabilistic performables.

        This inner class serves the purpose to define the variables that are used in a model and make them easily
        accessible for the user. The user can access the variables by using the dot notation, e.g. `self.variables.x`.

        The members of this class have to be ordered the same way as the variables in the policy.
        The order of the variables in the policy is most likely alphabetical by name.
        """
        ...

    policy: ProbabilisticCircuit
    """
    The policy that is used to determine the parameters.
    """

    variables: Variables
    """
    The variables of this action.
    """

    def __init__(self, policy: Optional[ProbabilisticCircuit] = None):
        self.variables = self.Variables()
        if policy is None:
            policy = self.default_policy()
        self.policy = policy

    def sample_to_action(self, sample: List) -> ActionAbstract:
        """
        Convert a sample from the policy to a performable action.

        :param sample: The sample.
        :return: The action.
        """
        raise NotImplementedError

    def all_variables(self) -> SortedSet:
        """
        :return: All variables of the action.
        """
        return SortedSet([getattr(self.variables, field.name) for field in fields(self.variables)
                          if issubclass(field.type, Variable)])

    def default_policy(self) -> ProbabilisticCircuit:
        """
        :return: The default policy for the action.
        """
        means = {v: 0 for v in self.all_variables() if v.is_numeric}
        variances = {v: 1 for v in self.all_variables() if v.is_numeric}
        model = fully_factorized(self.all_variables(), means, variances)
        return model

    def evidence_from_belief_state(self) -> Event:
        """
        :return: the evidence from the belief state that influences the distribution.
        """
        raise NotImplementedError


class MoveAndPickUp(ActionDescription, ProbabilisticAction):

    @dataclass
    class Variables:
        arm: Symbolic = Symbolic("arm", Set.from_iterable(Arms))
        grasp: Symbolic = Symbolic("grasp", Set.from_iterable(Grasp))
        relative_x: Continuous = Continuous("relative_x")
        relative_y: Continuous = Continuous("relative_y")

    variables: Variables

    sample_amount: int = 20
    """
    The amount of samples that should be drawn from the policy when iterating over it.
    """

    object_designator: Object
    """
    The object designator that should be picked up.
    """

    arms: List[Arms]
    """
    The arms that can be used for the pick up.
    """

    grasps: List[Grasp]
    """
    The grasps that can be used for the pick up.
    """

    def __init__(self, object_designator: Object, arms: List[Arms], grasps: List[Grasp],
                 policy: Optional[ProbabilisticCircuit] = None):
        ProbabilisticAction.__init__(self, policy)
        self.object_designator = object_designator
        self.arms = arms
        self.grasps = grasps

    def sample_to_action(self, sample: List) -> MoveAndPickUpAction:
        arm, grasp, relative_x, relative_y = sample
        position = [relative_x, relative_y, 0.]
        pose = Pose(position, frame=self.object_designator.tf_frame)
        standing_position = LocalTransformer().transform_pose(pose, "map")
        standing_position.position.z = 0
        action = MoveAndPickUpAction(standing_position, self.object_designator, EArms[Arms(int(arm)).name],
                                     EGrasp(int(grasp)), True, 0.03)
        return action

    def events_from_occupancy_and_visibility_costmap(self) -> Event:
        """
        Create events from the occupancy and visibility costmap.

        :return: The events that can be used as evidence for the model.
        """

        # create occupancy and visibility costmap for the target object
        ocm = OccupancyCostmap(distance_to_obstacle=0.4, from_ros=False, size=200, resolution=0.1,
                               origin=self.object_designator.pose)
        vcm = VisibilityCostmap(min_height=1.27, max_height=1.69,
                                size=200, resolution=0.1, origin=self.object_designator.pose)
        mcm = ocm + vcm

        # convert rectangles to events
        events = []
        for rectangle in mcm.partitioning_rectangles():
            event = SimpleEvent({self.variables.relative_x: random_events.interval.open(rectangle.x_lower, rectangle.x_upper),
                                 self.variables.relative_y: random_events.interval.open(rectangle.y_lower, rectangle.y_upper)})
            events.append(event)
        return Event(*events)

    def ground_model(self, model: Optional[ProbabilisticCircuit] = None,
                     event: Optional[Event] = None) -> ProbabilisticCircuit:
        """
        Ground the model to the current evidence.

        :param model: The model that should be grounded. If None, the policy is used.
        :param event: The events that should be used as evidence. If None, the occupancy costmap is used.
        :return: The grounded model
        """

        if model is None:
            model = self.policy
        if event is None:
            event = self.events_from_occupancy_and_visibility_costmap()

        result, probability = model.conditional(event)

        if probability == 0:
            raise ObjectUnreachable("No possible locations found")

        return result

    def iter_with_mode(self) -> Iterator[MoveAndPickUpAction]:
        """
        Generate performables by sampling from the mode of the policy conditioned on visibility and occupancy.
        """
        model = self.ground_model()

        modes, log_max = model.log_mode(check_determinism=False)
        model, _ = model.conditional(modes)
        samples = model.sample(self.sample_amount)

        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    def __iter__(self) -> Iterator[MoveAndPickUpAction]:
        """
        Generate performables by sampling from the policy conditioned on visibility and occupancy.
        """
        model = self.ground_model(self.policy, self.events_from_occupancy_and_visibility_costmap())
        samples = model.sample(self.sample_amount)
        likelihoods = model.likelihood(samples)

        # sort samples by likelihood
        samples = [x for _, x in sorted(zip(likelihoods, samples), key=lambda pair: pair[0], reverse=True)]

        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    def iterate_without_occupancy_costmap(self) -> Iterator[MoveAndPickUpAction]:
        """
        Generate performables by sampling from the policy without conditioning on visibility and occupancy.
        """
        samples = self.policy.sample(self.sample_amount)
        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    @staticmethod
    def query_for_database():
        view = PickUpWithContextView
        query = (select(view.arm, view.grasp, view.relative_x, view.relative_y)
                 .where(view.status == TaskStatus.SUCCEEDED))
        return query

    def batch_rollout(self):
        """
        Try the policy without conditioning on visibility and occupancy and count the successful tries.
        """

        # initialize statistics
        successful_tries = 0
        total_tries = 0

        # create the progress bar
        progress_bar = tqdm.tqdm(iter(self.iterate_without_occupancy_costmap()), total=self.sample_amount)

        for action in progress_bar:
            total_tries += 1
            try:
                action.perform()
                successful_tries += 1
            except PlanFailure as p:
                pass

            # update progress bar
            progress_bar.set_postfix({"Success Probability": successful_tries / total_tries})

            # reset world
            World.current_world.reset_world()


class MoveAndPlace(ActionDescription, ProbabilisticAction):

    @dataclass
    class Variables:
        relative_x: Continuous = Continuous("relative_x")
        relative_y: Continuous = Continuous("relative_y")

    variables: Variables

    sample_amount: int = 20
    """
    The amount of samples that should be drawn from the policy when iterating over it.
    """

    object_designator: Object
    """
    The object designator that should be picked up.
    """

    target_location: Pose
    """
    The position to place the object.
    """

    arms: List[Arms]
    """
    The arms that can be used for the pick up.
    """

    def __init__(self, object_designator: Object,
                 target_location: Pose, policy: Optional[ProbabilisticCircuit] = None):
        ProbabilisticAction.__init__(self, policy)
        self.object_designator = object_designator
        self.target_location = target_location

    def sample_to_action(self, sample: List) -> MoveAndPlaceAction:
        relative_x, relative_y = sample
        position = [relative_x + self.target_location.position.x, relative_y + self.target_location.position.y, 0.]
        pose = Pose(position, frame=self.target_location.frame)
        standing_position = LocalTransformer().transform_pose(pose, "map")
        standing_position.position.z = 0
        action = MoveAndPlaceAction(standing_position, self.object_designator,
                                    self.target_location, EArms.LEFT, True)
        return action

    def events_from_occupancy_and_visibility_costmap(self) -> Event:
        """
        Create events from the occupancy and visibility costmap.

        :return: The events that can be used as evidence for the model.
        """

        # create occupancy and visibility costmap for the target object
        ocm = OccupancyCostmap(distance_to_obstacle=0.4, from_ros=False, size=200, resolution=0.1,
                               origin=self.target_location)
        vcm = VisibilityCostmap(min_height=1.27, max_height=1.69,
                                size=200, resolution=0.1, origin=self.target_location)
        mcm = ocm + vcm

        # convert rectangles to events
        events = []
        for rectangle in mcm.partitioning_rectangles():
            event = SimpleEvent({self.variables.relative_x: random_events.interval.open(rectangle.x_lower, rectangle.x_upper),
                                 self.variables.relative_y: random_events.interval.open(rectangle.y_lower, rectangle.y_upper)})
            events.append(event)
        return Event(*events)

    def ground_model(self, model: Optional[ProbabilisticCircuit] = None,
                     event: Optional[Event] = None) -> ProbabilisticCircuit:
        """
        Ground the model to the current evidence.

        :param model: The model that should be grounded. If None, the policy is used.
        :param event: The events that should be used as evidence. If None, the occupancy costmap is used.
        :return: The grounded model
        """

        if model is None:
            model = self.policy
        if event is None:
            event = self.events_from_occupancy_and_visibility_costmap()

        result, probability = model.conditional(event)

        if probability == 0:
            raise ObjectUnreachable("No possible locations found")

        return result

    def iter_with_mode(self) -> Iterator[MoveAndPlaceAction]:
        """
        Generate performables by sampling from the mode of the policy conditioned on visibility and occupancy.
        """
        model = self.ground_model()
        modes, _ = model.log_mode(check_determinism=False)
        model = self.ground_model(model, modes)
        samples = model.sample(self.sample_amount)

        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    def __iter__(self) -> Iterator[MoveAndPickUpAction]:
        """
        Generate performables by sampling from the policy conditioned on visibility and occupancy.
        """
        model = self.ground_model(self.policy, self.events_from_occupancy_and_visibility_costmap())
        samples = model.sample(self.sample_amount)
        likelihoods = model.likelihood(samples)

        # sort samples by likelihood
        samples = [x for _, x in sorted(zip(likelihoods, samples), key=lambda pair: pair[0], reverse=True)]

        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    def iterate_without_occupancy_costmap(self) -> Iterator[MoveAndPickUpAction]:
        """
        Generate performables by sampling from the policy without conditioning on visibility and occupancy.
        """
        samples = self.policy.sample(self.sample_amount)
        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    @staticmethod
    def query_for_database():
        view = PlaceWithContextView
        query = (select(view.arm, view.relative_x, view.relative_y).where(view.status == TaskStatus.SUCCEEDED))
        return query

    def batch_rollout(self):

        # initialize statistics
        successful_tries = 0
        total_tries = 0

        # create the progress bar
        progress_bar = tqdm.tqdm(iter(self.iterate_without_occupancy_costmap()), total=self.sample_amount)

        for action in progress_bar:
            total_tries += 1
            try:
                action.perform()
                successful_tries += 1
            except PlanFailure as p:
                pass

            # update progress bar
            progress_bar.set_postfix({"Success Probability": successful_tries / total_tries})

            # reset world
            World.current_world.reset_world()
