from __future__ import annotations

import abc
from dataclasses import dataclass, field

from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.world import World
from typing_extensions import Any, Optional, Callable

from ...datastructures.dataclasses import ExecutionData, Context
from ...failures import PlanFailure
from ...has_parameters import HasParameters
from ...plan import PlanNode, Plan


@dataclass
class ActionDescription(HasParameters):

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
    def context(self) -> Context:
        return Context(self.world, self.robot_view, self.plan_struct)

    @property
    def robot_view(self) -> AbstractRobot:
        return self.plan_struct.robot

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
