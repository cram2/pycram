from pycram.robot_plans.actions.base import ActionDescription
from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

import numpy as np

from .misc import DetectActionDescription
from pycram.designators.object_designator import BelieveObject
from pycram.has_parameters import has_parameters
from pycram.language import SequentialPlan, TryInOrderPlan
from pycram.plan import with_plan

from pycram.datastructures.partial_designator import PartialDesignator

from pycram.tf_transformations import quaternion_from_euler
from typing_extensions import Union, Optional, Type, Any, Iterable

from pycrap.ontologies import PhysicalObject
from pycram.designators.location_designator import CostmapLocation
from pycram.designators.motion_designator import MoveMotion, \
    LookingMotion
from pycram.datastructures.world import UseProspectionWorld
from pycram.failure_handling import try_action
from pycram.failures import LookAtGoalNotReached
from pycram.local_transformer import LocalTransformer
from pycram.failures import NavigationGoalNotReachedError, PerceptionObjectNotFound
from pycram.config.action_conf import ActionConfig

from pycram.datastructures.enums import DetectionTechnique

from pycram.datastructures.pose import PoseStamped
from pycram.datastructures.world import World

from pycram.validation.error_checkers import PoseErrorChecker
from pycram.world_reasoning import move_away_all_objects_to_create_empty_space, generate_object_at_target, \
    cast_a_ray_from_camera


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
        NavigateAction(new_robot_pose, self.keep_joint_states).perform()

        # look at target
        LookAtAction(self.pose).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in the LookAtActionPerformable.perform() method so no need to validate here.
        pass

    @classmethod
    @with_plan
    def description(cls, pose: Union[Iterable[PoseStamped], PoseStamped],
                    keep_joint_states: Union[Iterable[bool], bool] = ActionConfig.face_at_keep_joint_states) -> \
            PartialDesignator[Type[FaceAtAction]]:
        return PartialDesignator(FaceAtAction, pose=pose, keep_joint_states=keep_joint_states)

@has_parameters
@dataclass
class SearchAction(ActionDescription):
    """
    Searches for a target object around the given location.
    """

    target_location: PoseStamped
    """
    Location around which to look for a target object.
    """

    object_type: Type[PhysicalObject]
    """
    Type of the object which is searched for.
    """

    def plan(self) -> None:
        NavigateActionDescription(
            CostmapLocation(target=self.target_location, visible_for=World.robot)).resolve().perform()

        lt = LocalTransformer()
        target_base = lt.transform_pose(self.target_location, World.robot.tf_frame)

        target_base_left = target_base.copy()
        target_base_left.pose.position.y -= 0.5

        target_base_right = target_base.copy()
        target_base_right.pose.position.y += 0.5

        plan = TryInOrderPlan(
            SequentialPlan(
                LookAtActionDescription(target_base_left),
                DetectActionDescription(DetectionTechnique.TYPES,
                                        object_designator=BelieveObject(types=[self.object_type]))),
            SequentialPlan(
                LookAtActionDescription(target_base_right),
                DetectActionDescription(DetectionTechnique.TYPES,
                                        object_designator=BelieveObject(types=[self.object_type]))),
            SequentialPlan(
                LookAtActionDescription(target_base),
                DetectActionDescription(DetectionTechnique.TYPES,
                                        object_designator=BelieveObject(types=[self.object_type]))))

        obj = plan.perform()
        if obj is not None:
            return obj
        raise PerceptionObjectNotFound(self.object_type, DetectionTechnique.TYPES, self.target_location)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        pass

    @classmethod
    @with_plan
    def description(cls, target_location: Union[Iterable[PoseStamped], PoseStamped],
                    object_type: Union[Iterable[PhysicalObject], PhysicalObject]) -> PartialDesignator[
        Type[SearchAction]]:
        return PartialDesignator(SearchAction, target_location=target_location, object_type=object_type)


NavigateActionDescription = NavigateAction.description

LookAtActionDescription = LookAtAction.description
FaceAtActionDescription = FaceAtAction.description

SearchActionDescription = SearchAction.description