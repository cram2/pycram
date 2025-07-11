import enum
from dataclasses import dataclass
from typing import Type

from probabilistic_model.distributions import SymbolicDistribution
from probabilistic_model.probabilistic_circuit.nx.helper import fully_factorized, leaf
from probabilistic_model.probabilistic_circuit.nx.probabilistic_circuit import ProbabilisticCircuit, SumUnit, \
    ProductUnit
from probabilistic_model.utils import MissingDict
from random_events.product_algebra import SimpleEvent
from random_events.set import Set
from random_events.variable import Symbolic, Continuous
from sqlalchemy import select
from typing_extensions import Optional, List

from ....robot_plans import MoveAndPickUpAction
from ....datastructures.dataclasses import BoundingBox
from ....datastructures.enums import Arms, Grasp, VerticalAlignment, ApproachDirection
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....datastructures.world import World
from ....parameterizer import collision_free_event
from ....utils import classproperty
from ....world_concepts.world_object import Object


class Variables(enum.Enum):
    """
    Abstract enum for variables in a parameterizer.
    """

    @classmethod
    def all(cls):
        return [v.value for v in cls]


class ProbabilisticAction:
    """
    Abstract class for performables that have a probabilistic parametrization.
    """

    policy: ProbabilisticCircuit
    """
    The policy that is used to determine the parameters.
    """

    variables: Type[Variables]

    def __init__(self, policy: Optional[ProbabilisticCircuit] = None):
        if policy is None:
            policy = self.default_policy()
        self.policy = policy

    def default_policy(self) -> ProbabilisticCircuit:
        """
        :return: The default policy for the action.
        """
        means = {v: 0 for v in self.variables.all() if v.is_numeric}
        variances = {v: 1 for v in self.variables.all() if v.is_numeric}
        model = fully_factorized(self.variables.all(), means, variances)
        return model


class MoveAndPickUpVariables(Variables):
    arm = Symbolic("arm", Set.from_iterable(Arms))
    keep_joint_states = Symbolic("keep_joint_states", Set.from_iterable([False, True]))

    rotate_gripper = Symbolic("rotate_gripper", Set.from_iterable([False, True]))

    x = Continuous("x")
    y = Continuous("y")

    approach_direction = Symbolic("approach_direction", Set.from_iterable(ApproachDirection))
    vertical_alignment = Symbolic("vertical_alignment", Set.from_iterable(VerticalAlignment))


@dataclass
class MoveAndPickUpParameterizer(ProbabilisticAction):
    """
    Action that moves the agent to an object and picks it up using probability tools to parameterize.
    """

    variables = MoveAndPickUpVariables

    partial: PartialDesignator[MoveAndPickUpAction]

    def collision_free_condition_for_object(self, obj: Object):
        search_space_size = 1.
        search_space = BoundingBox(min_x=obj.pose.position.x - search_space_size,
                                   min_y=obj.pose.position.y - search_space_size,
                                   min_z=obj.pose.position.z - search_space_size,
                                   max_x=obj.pose.position.x + search_space_size,
                                   max_y=obj.pose.position.y + search_space_size,
                                   max_z=obj.pose.position.z + search_space_size).as_collection()
        navigate_conditions = collision_free_event(obj.world, search_space)
        return navigate_conditions

    def accessing_distribution_for_object(self, obj: Object, object_variable: Symbolic) -> ProbabilisticCircuit:
        model = self.default_policy()

        # add object distribution her
        p_object = SymbolicDistribution(object_variable, MissingDict(float, {obj.id: 1.}))
        root = model.root
        new_root = ProductUnit(probabilistic_circuit=model)
        new_root.add_subcircuit(leaf(p_object, model))
        new_root.add_subcircuit(root)

        # move model to position of the object
        model.translate({self.variables.x.value: obj.pose.position.x, self.variables.y.value: obj.pose.position.y})

        # apply collision-free condition
        condition = self.collision_free_condition_for_object(obj)
        condition.fill_missing_variables(model.variables)

        # apply grasp conditions
        grasp_condition = SimpleEvent(
            {self.variables.approach_direction.value: [ApproachDirection.FRONT, ApproachDirection.BACK, ApproachDirection.LEFT, ApproachDirection.RIGHT],
                self.variables.vertical_alignment.value: [VerticalAlignment.TOP, VerticalAlignment.BOTTOM, VerticalAlignment.NoAlignment],}).as_composite_set()
        grasp_condition.fill_missing_variables(model.variables)
        condition &= grasp_condition

        # apply arm condition
        arm_condition = SimpleEvent({self.variables.arm.value: [Arms.LEFT, Arms.RIGHT], }).as_composite_set()
        arm_condition.fill_missing_variables(model.variables)
        condition &= arm_condition

        conditional, prob = model.truncated(condition)

        return conditional

    @classproperty
    def object_variable(cls) -> Symbolic:
        return Symbolic("object_designator", Set.from_iterable([obj.id for obj in World.current_world.objects]))

    def create_distribution(self):

        result = ProbabilisticCircuit()
        root = SumUnit(probabilistic_circuit=result)

        for obj in self.partial.kwargs["object_designator"]:
            model = self.accessing_distribution_for_object(obj, self.object_variable)
            if model is None:
                continue
            root.add_subcircuit(model.root, 0.)
        root.normalize()

        return result

    def create_actions(self, amount: int = 100) -> List[MoveAndPickUpAction]:
        model = self.create_distribution()
        samples = model.sample(amount)
        ll = model.log_likelihood(samples)
        sorted_indices = ll.argsort()
        samples = samples[sorted_indices][:amount]

        return [self.sample_to_action(sample, model) for sample in samples]

    def sample_to_action(self, sample: List, model: ProbabilisticCircuit) -> MoveAndPickUpAction:
        sample_dict = {variable: value for variable, value in zip(model.variables, sample)}
        obj = World.current_world.get_object_by_id(sample_dict[self.object_variable])

        standing_position = PoseStamped.from_list(
            [sample_dict[self.variables.x.value], sample_dict[self.variables.y.value], 0.])

        grasp_description = GraspDescription(
            approach_direction=list(ApproachDirection)[int(sample_dict[self.variables.approach_direction.value])],
            rotate_gripper=sample_dict[self.variables.rotate_gripper.value],
            vertical_alignment=list(VerticalAlignment)[int(sample_dict[self.variables.vertical_alignment.value])], )

        return MoveAndPickUpAction(standing_position=standing_position, object_designator=obj,
                                   arm=Arms(int(sample_dict[self.variables.arm.value])),
                                   grasp_description=grasp_description,
                                   keep_joint_states=sample_dict[self.variables.keep_joint_states.value])

    def create_action(self):
        return self.create_actions(100)[0]

    def query_for_database(self):
        select(

        )
