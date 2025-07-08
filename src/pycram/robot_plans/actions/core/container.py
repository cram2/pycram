from __future__ import annotations

from dataclasses import dataclass
from datetime import timedelta

from typing_extensions import Union, Optional, Type, Any, Iterable

from .grasping import GraspingActionDescription
from ...motions.container import OpeningMotion, ClosingMotion
from ...motions.gripper import MoveGripperMotion
from ....config.action_conf import ActionConfig
from ....datastructures.enums import Arms, GripperState, ContainerManipulationType
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.world import World
from ....description import Joint, Link, ObjectDescription
from ....failures import ContainerManipulationError
from ....has_parameters import has_parameters
from ....plan import with_plan
from ....robot_plans.actions.base import ActionDescription


@has_parameters
@dataclass
class OpenAction(ActionDescription):
    """
    Opens a container like object
    """

    object_designator: ObjectDescription.Link
    """
    Object designator_description describing the object that should be opened
    """
    arm: Arms
    """
    Arm that should be used for opening the container
    """
    grasping_prepose_distance: float = ActionConfig.grasping_prepose_distance
    """
    The distance in meters the gripper should be at in the x-axis away from the handle.
    """

    def plan(self) -> None:
        GraspingActionDescription(self.object_designator, self.arm, self.grasping_prepose_distance).perform()
        OpeningMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the container is opened, this assumes that the container state can be read accurately from the
        real world.
        """
        validate_close_open(self.object_designator, self.arm, OpenAction)

    @classmethod
    @with_plan
    def description(cls, object_designator_description: Union[Iterable[ObjectDescription.Link], ObjectDescription.Link],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasping_prepose_distance: Union[
                        Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> \
            PartialDesignator[Type[OpenAction]]:
        return PartialDesignator(OpenAction, object_designator=object_designator_description,
                                 arm=arm,
                                 grasping_prepose_distance=grasping_prepose_distance)


@has_parameters
@dataclass
class CloseAction(ActionDescription):
    """
    Closes a container like object.
    """

    object_designator: ObjectDescription.Link
    """
    Object designator_description describing the object that should be closed
    """
    arm: Arms
    """
    Arm that should be used for closing
    """
    grasping_prepose_distance: float = ActionConfig.grasping_prepose_distance
    """
    The distance in meters between the gripper and the handle before approaching to grasp.
    """

    def plan(self) -> None:
        GraspingActionDescription(self.object_designator, self.arm, self.grasping_prepose_distance).perform()
        ClosingMotion(self.object_designator, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the container is closed, this assumes that the container state can be read accurately from the
        real world.
        """
        validate_close_open(self.object_designator, self.arm, CloseAction)

    @classmethod
    @with_plan
    def description(cls, object_designator_description: Union[Iterable[ObjectDescription.Link], ObjectDescription.Link],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasping_prepose_distance: Union[
                        Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> \
            PartialDesignator[Type[CloseAction]]:
        return PartialDesignator(CloseAction, object_designator=object_designator_description,
                                 arm=arm,
                                 grasping_prepose_distance=grasping_prepose_distance)


def validate_close_open(object_designator: ObjectDescription.Link, arm: Arms,
                        action_type: Union[Type[OpenAction], Type[CloseAction]]):
    """
    Validates if the container is opened or closed by checking the joint position of the container.

    :param object_designator: The object designator_description describing the object that should be opened or closed.
    :param arm: The arm that should be used for opening or closing the container.
    :param action_type: The type of the action that should be validated.
    """
    obj_part = object_designator
    obj = object_designator.parent_entity
    container_joint_name = obj.find_joint_above_link(object_designator.name)
    lower_limit, upper_limit = obj.get_joint_limits(container_joint_name)
    joint_obj: Joint = obj.joints[container_joint_name]
    if issubclass(action_type, CloseAction):
        check_closed(joint_obj, obj_part, arm, lower_limit)
    elif issubclass(action_type, OpenAction):
        check_opened(joint_obj, obj_part, arm, upper_limit)
    else:
        raise ValueError(f"Invalid action type: {action_type}")


def check_opened(joint_obj: Joint, obj_part: Link, arm: Arms, upper_limit: float):
    if joint_obj.position < upper_limit - joint_obj.acceptable_error:
        raise ContainerManipulationError(World.robot, [arm], obj_part, joint_obj,
                                         ContainerManipulationType.Opening)


def check_closed(joint_obj: Joint, obj_part: Link, arm: Arms, lower_limit: float):
    if joint_obj.position > lower_limit + joint_obj.acceptable_error:
        raise ContainerManipulationError(World.robot, [arm], obj_part, joint_obj,
                                         ContainerManipulationType.Closing)

OpenActionDescription = OpenAction.description
CloseActionDescription = CloseAction.description

