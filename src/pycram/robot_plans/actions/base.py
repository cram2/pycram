from __future__ import annotations

import abc
from dataclasses import dataclass, field

from jedi.inference.gradual.typing import Tuple
from typing_extensions import Type, Any, Optional, Callable

from ...datastructures.pose import PoseStamped
from semantic_world.world import World
from ...failures import PlanFailure
from ...has_parameters import HasParameters
from ...plan import PlanNode, Plan
from ...robot_description import RobotDescription
from pycrap.ontologies import Agent


def record_object_pre_perform(action):
    """
    Record the object before the action is performed.

    This should be appended to the pre performs of every action that interacts with an object.
    """
    # for every field in the action that is an object
    # write it to a dict mapping the OG field name to the frozen copy
    # action.object_at_execution = action.object_designator.frozen_copy()
    pass

@dataclass
class ActionDescription(HasParameters):
    robot_position: PoseStamped = field(init=False)
    robot_torso_height: float = field(init=False)
    robot_type: Type[Agent] = field(init=False)

    # Is assigned in the __post_init method of the ActionNode
    _plan_node: PlanNode = field(init=False, default=None)

    _pre_perform_callbacks = []
    _post_perform_callbacks = []

    @property
    def plan_node(self) -> PlanNode:
        return self._plan_node

    @plan_node.setter
    def plan_node(self, value: PlanNode):
        if not isinstance(value, PlanNode):
            raise TypeError("plan_node must be an instance of PlanNode")
        self._plan_node = value

    @property
    def plan_struct(self) -> Plan:
        return self.plan_node.plan

    @property
    def world(self) -> World:
        return self.plan_struct.world

    @property
    def context(self) -> Tuple[World, Plan]:
        return self.world, self.plan_struct

    def __post_init__(self):
        pass
        # self._pre_perform_callbacks.append(self._update_robot_params)

    def perform(self) -> Any:
        """
        Full execution: pre-check, plan, post-check
        """
        for pre_cb in self._pre_perform_callbacks:
            pre_cb(self)

        self.validate_precondition()

        result = None
        try:
            result = self.plan()
        except PlanFailure as e:
            raise e
        finally:
            for post_cb in self._post_perform_callbacks:
                post_cb(self)

            self.validate_postcondition(result)

        return result

    @abc.abstractmethod
    def plan(self) -> Any:
        """
        Symbolic plan. Should only call motions or sub-actions.
        """
        pass

    @abc.abstractmethod
    def validate_precondition(self):
        """
        Symbolic/world state precondition validation.
        """
        pass

    @abc.abstractmethod
    def validate_postcondition(self, result: Optional[Any] = None):
        """
        Symbolic/world state postcondition validation.
        """
        pass

    @classmethod
    def pre_perform(cls, func) -> Callable:
        cls._pre_perform_callbacks.append(func)
        return func

    @classmethod
    def post_perform(cls, func) -> Callable:
        cls._post_perform_callbacks.append(func)
        return func

    def _update_robot_params(self, action: ActionDescription):
        action.robot_position = World.robot.pose
        action.robot_torso_height = World.robot.get_joint_position(
            RobotDescription.current_robot_description.torso_joint)
        action.robot_type = World.robot.obj_type
