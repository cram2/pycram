from __future__ import annotations

import abc
from dataclasses import dataclass, field

from typing_extensions import Type, Any, Optional, Callable

from ...datastructures.pose import PoseStamped
from ...failures import PlanFailure
from ...has_parameters import HasParameters
from ...multirobot import RobotManager
from pycrap.ontologies import Agent


def record_object_pre_perform(action):
    """
    Record the object before the action is performed.

    This should be appended to the pre performs of every action that interacts with an object.
    """
    # for every field in the action that is an object
    # write it to a dict mapping the OG field name to the frozen copy
    action.object_at_execution = action.object_designator.frozen_copy()


@dataclass
class ActionDescription(HasParameters):
    robot_position: PoseStamped = field(init=False)
    robot_torso_height: float = field(init=False)
    robot_type: Type[Agent] = field(init=False)

    _pre_perform_callbacks = []
    _post_perform_callbacks = []

    def __post_init__(self):
        self._pre_perform_callbacks.append(self._update_robot_params)

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
        action.robot_position = RobotManager.get_active_robot().pose
        action.robot_torso_height = RobotManager.get_active_robot().get_joint_position(
            RobotManager.get_robot_description().torso_joint)
        action.robot_type = RobotManager.get_active_robot().obj_type
