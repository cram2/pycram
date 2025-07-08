from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

import numpy as np
from typing_extensions import Union, Optional, Type, Any, Iterable

from ..base import ActionDescription
from ... import LookAtActionDescription, DetectActionDescription, NavigateActionDescription
from ...motions.navigation import MoveMotion, LookingMotion
from ....config.action_conf import ActionConfig
from ....datastructures.enums import DetectionTechnique
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....datastructures.world import UseProspectionWorld
from ....datastructures.world import World
from ....designators.location_designator import CostmapLocation
from ....designators.object_designator import BelieveObject
from ....failure_handling import try_action
from ....failures import LookAtGoalNotReached
from ....failures import NavigationGoalNotReachedError, PerceptionObjectNotFound
from ....has_parameters import has_parameters
from ....language import SequentialPlan, TryInOrderPlan
from ....local_transformer import LocalTransformer
from ....plan import with_plan
from ....tf_transformations import quaternion_from_euler
from ....validation.error_checkers import PoseErrorChecker
from ....world_reasoning import move_away_all_objects_to_create_empty_space, generate_object_at_target, \
    cast_a_ray_from_camera
from pycrap.ontologies import PhysicalObject

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

    def plan(self) -> None:
        motion_action = MoveMotion(self.target_location, self.keep_joint_states)
        return try_action(motion_action, failure_type=NavigationGoalNotReachedError)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        pose_validator = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_validator.is_error_acceptable(World.robot.pose, self.target_location):
            raise NavigationGoalNotReachedError(World.robot.pose, self.target_location)

    @classmethod
    @with_plan
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

    def plan(self) -> None:
        LookingMotion(target=self.target).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the robot is looking at the target location by spawning a virtual object at the target location and
        creating a ray from the camera and checking if it intersects with the object.
        """
        return
        with UseProspectionWorld():
            move_away_all_objects_to_create_empty_space(exclude_objects=[World.robot.name, "floor"])
            # Create a virtual object at the target location, the current size is 40x40x40 cm which is very big in
            # my opinion, maybe this indicates that the looking at action is not accurate # TODO check this
            gen_obj = generate_object_at_target(self.target.position.to_list(), size=(0.4, 0.4, 0.4))
            ray_result = cast_a_ray_from_camera()
            gen_obj.remove()
            if not ray_result.intersected or ray_result.obj_id != gen_obj.id:
                raise LookAtGoalNotReached(World.robot, self.target)

    @classmethod
    @with_plan
    def description(cls, target: Union[Iterable[PoseStamped], PoseStamped]) -> PartialDesignator[Type[LookAtAction]]:
        return PartialDesignator(LookAtAction, target=target)






NavigateActionDescription = NavigateAction.description
LookAtActionDescription = LookAtAction.description



