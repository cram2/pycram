from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

import numpy as np
from typing_extensions import Union, Optional, Type, Any, Iterable

from ..core.navigation import LookAtActionDescription,NavigateActionDescription
from ....config.action_conf import ActionConfig
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....has_parameters import has_parameters
from ....language import SequentialPlan
from ....robot_plans.actions.base import ActionDescription
from ....tf_transformations import quaternion_from_euler


@has_parameters
@dataclass
class FaceAtAction(ActionDescription):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose: PoseStamped
    """
    The pose to face 
    """
    keep_joint_states: bool = ActionConfig.face_at_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    def plan(self) -> None:
        # get the robot position
        robot_position = World.robot.pose

        # calculate orientation for robot to face the object
        angle = np.arctan2(robot_position.position.y - self.pose.position.y,
                           robot_position.position.x - self.pose.position.x) + np.pi
        orientation = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))

        # create new robot pose
        new_robot_pose = PoseStamped.from_list(robot_position.position.to_list(), orientation)

        # turn robot
        SequentialPlan(self.context, NavigateActionDescription(new_robot_pose, self.keep_joint_states),
            # look at target
            LookAtActionDescription(self.pose)).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in the LookAtActionPerformable.perform() method so no need to validate here.
        pass

    @classmethod
    def description(cls, pose: Union[Iterable[PoseStamped], PoseStamped],
                    keep_joint_states: Union[Iterable[bool], bool] = ActionConfig.face_at_keep_joint_states) -> \
            PartialDesignator[Type[FaceAtAction]]:
        return PartialDesignator(FaceAtAction, pose=pose, keep_joint_states=keep_joint_states)

FaceAtActionDescription = FaceAtAction.description