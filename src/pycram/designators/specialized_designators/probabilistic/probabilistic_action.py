from dataclasses import dataclass
from sqlalchemy import select

import portion
from probabilistic_model.probabilistic_circuit.distributions import GaussianDistribution, SymbolicDistribution
from probabilistic_model.probabilistic_circuit.probabilistic_circuit import ProbabilisticCircuit, \
    DecomposableProductUnit
from random_events.events import Event, ComplexEvent
from random_events.variables import Symbolic, Continuous
import tqdm
from typing_extensions import Optional, List, Iterator
from ....datastructures.world import World
from ....costmaps import OccupancyCostmap, VisibilityCostmap
from ....designator import ActionDesignatorDescription, ObjectDesignatorDescription
from ...action_designator import MoveAndPickUpPerformable, ActionAbstract
from ....datastructures.enums import Arms, Grasp, TaskStatus
from ....local_transformer import LocalTransformer
from ....orm.views import PickUpWithContextView
from ....plan_failures import ObjectUnreachable, PlanFailure
from ....datastructures.pose import Pose


class ProbabilisticAction:
    """
    Abstract class for probabilistic performables.
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
        if policy is None:
            policy = self.default_policy()
        self.policy = policy
        self.variables = self.Variables(*self.policy.variables)

    def default_policy(self) -> ProbabilisticCircuit:
        """
        Create a default policy for the action.

        :return: The default policy for the action
        """
        raise NotImplementedError

    def sample_to_action(self, sample: List) -> ActionAbstract:
        """
        Convert a sample from the policy to a performable action.
        :param sample: The sample
        :return: The action
        """
        raise NotImplementedError


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

    def __init__(self, distance_to_center: float = 0.2, variance: float = 0.5):
        self.distance_to_center = distance_to_center
        self.variance = variance

    def center_event(self) -> Event:
        """
        Create an event that describes the center of the map.
        """
        return Event({self.relative_x: portion.open(-self.distance_to_center, self.distance_to_center),
                      self.relative_y: portion.open(-self.distance_to_center, self.distance_to_center)})

    def create_model_with_center(self) -> ProbabilisticCircuit:
        """
        Create a fully factorized gaussian at the center of the map.
        """
        centered_model = DecomposableProductUnit()
        centered_model.add_subcircuit(GaussianDistribution(self.relative_x, 0., self.variance))
        centered_model.add_subcircuit(GaussianDistribution(self.relative_y, 0., self.variance))
        centered_model.add_subcircuit(SymbolicDistribution(self.grasp,
                                                           [1/len(self.grasp.domain)] * len(self.grasp.domain)))
        centered_model.add_subcircuit(SymbolicDistribution(self.arm,
                                                           [1/len(self.arm.domain)] * len(self.arm.domain)))
        return centered_model.probabilistic_circuit

    def create_model(self) -> ProbabilisticCircuit:
        """
        Create a gaussian model that assumes mass everywhere besides the center square.

        :return: The probabilistic circuit
        """
        centered_model = self.create_model_with_center()
        outer_event = self.center_event().complement()
        limiting_event = Event({self.relative_x: portion.open(-2, 2),
                                self.relative_y: portion.open(-2, 2)})
        event = outer_event & limiting_event
        # go.Figure(event.plot()).show()
        result, _ = centered_model.conditional(event)
        return result


class MoveAndPickUp(ActionDesignatorDescription, ProbabilisticAction):

    @dataclass
    class Variables:
        arm: Symbolic = Symbolic("arm", ["left", "right"])
        grasp: Symbolic = Symbolic("grasp", ["front", "left", "right", "top"])
        relative_x: Continuous = Continuous("relative_x")
        relative_y: Continuous = Continuous("relative_y")

    variables: Variables  # Type hint variables

    sample_amount: int = 20
    """
    The amount of samples that should be drawn from the policy when iterating over it.
    """

    object_designator: ObjectDesignatorDescription.Object
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

    def __init__(self, object_designator: ObjectDesignatorDescription.Object, arms: List[str], grasps: List[str],
                 policy: Optional[ProbabilisticCircuit] = None):
        ActionDesignatorDescription.__init__(self)
        ProbabilisticAction.__init__(self, policy)
        self.object_designator = object_designator
        self.arms = arms
        self.grasps = grasps

    def default_policy(self) -> ProbabilisticCircuit:
        return GaussianCostmapModel().create_model()

    def sample_to_action(self, sample: List) -> MoveAndPickUpPerformable:
        """
        Convert a sample from the underlying distribution to a performable action.
        :param sample: The sample
        :return: The action
        """
        arm, grasp, relative_x, relative_y = sample
        position = [relative_x, relative_y, 0.]
        pose = Pose(position, frame=self.object_designator.world_object.tf_frame)
        standing_position = LocalTransformer().transform_pose(pose, "map")
        standing_position.position.z = 0
        action = MoveAndPickUpPerformable(standing_position, self.object_designator, arm, grasp)
        return action

    def events_from_occupancy_and_visibility_costmap(self) -> ComplexEvent:
        """
        Create events from the occupancy and visibility costmap.

        :return: The events that can be used as evidence for the model.
        """

        # create occupancy and visibility costmap for the target object
        ocm = OccupancyCostmap(distance_to_obstacle=0.3, from_ros=False, size=200, resolution=0.1,
                               origin=self.object_designator.pose)
        vcm = VisibilityCostmap(min_height=1.27, max_height=1.69,
                                size=200, resolution=0.1, origin=self.object_designator.pose)
        mcm = ocm + vcm

        # convert rectangles to events
        events = []
        for rectangle in mcm.partitioning_rectangles():
            event = Event({self.variables.relative_x: portion.open(rectangle.x_lower, rectangle.x_upper),
                           self.variables.relative_y: portion.open(rectangle.y_lower, rectangle.y_upper)})
            events.append(event)
        return ComplexEvent(events)

    def ground_model(self, model: Optional[ProbabilisticCircuit] = None,
                     event: Optional[ComplexEvent] = None) -> ProbabilisticCircuit:
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

    def iter_with_mode(self) -> Iterator[MoveAndPickUpPerformable]:
        """
        Generate performables by sampling from the mode of the policy conditioned on visibility and occupancy.
        """
        model = self.ground_model()
        modes, _ = model.mode()
        model = self.ground_model(model, modes)
        samples = model.sample(self.sample_amount)

        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    def __iter__(self) -> Iterator[MoveAndPickUpPerformable]:
        """
        Generate performables by sampling from the policy conditioned on visibility and occupancy.
        """
        model = self.ground_model(self.policy, self.events_from_occupancy_and_visibility_costmap())
        samples = model.sample(self.sample_amount)
        likelihoods = [model.likelihood(sample) for sample in samples]

        # sort samples by likelihood
        samples = [x for _, x in sorted(zip(likelihoods, samples), key=lambda pair: pair[0], reverse=True)]

        for sample in samples:
            action = self.sample_to_action(sample)
            yield action

    def iterate_without_occupancy_costmap(self) -> Iterator[MoveAndPickUpPerformable]:
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

        :amount: The amount of tries
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
            World.current_world.reset_current_world()
