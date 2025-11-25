from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

import numpy as np
from typing_extensions import Union, Optional, Type, Any, Iterable

from ..base import ActionDescription
from ...motions.navigation import MoveMotion, LookingMotion
from ....config.action_conf import ActionConfig
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....failures import LookAtGoalNotReached
from ....failures import NavigationGoalNotReachedError
from ....has_parameters import has_parameters
from ....language import SequentialPlan
from ....validation.error_checkers import PoseErrorChecker

@has_parameters
@dataclass
class NavigateAction(ActionDescription):
    """
    Navigates the Robot to a position.
    """

    target_location: PoseStamped
    """
    Location to which the robot should be navigated
    """

    keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    def execute(self) -> None:
        return SequentialPlan(self.context,  MoveMotion(self.target_location, self.keep_joint_states)).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        pose_validator = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_validator.is_error_acceptable(World.robot.pose, self.target_location):
            raise NavigationGoalNotReachedError(World.robot.pose, self.target_location)

    @classmethod
    def description(cls, target_location: Union[Iterable[PoseStamped], PoseStamped],
                    keep_joint_states: Union[Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> \
            PartialDesignator[Type[NavigateAction]]:
        return PartialDesignator(NavigateAction, target_location=target_location,
                                 keep_joint_states=keep_joint_states)


@has_parameters
@dataclass
class LookAtAction(ActionDescription):
    """
    Lets the robot look at a position.
    """

    target: PoseStamped
    """
    Position at which the robot should look, given as 6D pose
    """

    def execute(self) -> None:
        SequentialPlan(self.context,  LookingMotion(target=self.target)).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the robot is looking at the target location by spawning a virtual object at the target location and
        creating a ray from the camera and checking if it intersects with the object.
        """
        return

    @classmethod
    def description(cls, target: Union[Iterable[PoseStamped], PoseStamped]) -> PartialDesignator[Type[LookAtAction]]:
        return PartialDesignator(LookAtAction, target=target)






NavigateActionDescription = NavigateAction.description
LookAtActionDescription = LookAtAction.description



