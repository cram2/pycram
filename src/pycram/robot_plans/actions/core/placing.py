from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta
from functools import cached_property

from typing_extensions import Union, Optional, Type, Any, Iterable

from ...motions.gripper import MoveTCPMotion, MoveGripperMotion
from ....datastructures.dataclasses import FrozenObject
from ....datastructures.enums import Arms, GripperState
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....datastructures.world import World
from ....description import Link
from ....failures import ObjectNotPlacedAtTargetLocation, ObjectStillInContact
from ....has_parameters import has_parameters
from ....local_transformer import LocalTransformer
from ....plan import with_plan
from ....robot_description import EndEffectorDescription
from ....robot_description import RobotDescription, KinematicChainDescription
from ....robot_plans.actions.base import ActionDescription, record_object_pre_perform
from ....validation.error_checkers import PoseErrorChecker
from ....world_concepts.world_object import Object


@has_parameters
@dataclass
class PlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    object_designator: Object
    """
    Object designator_description describing the object that should be place
    """
    target_location: PoseStamped
    """
    Pose in the world at which the object should be placed
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """
    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """
    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

        # Store the object's data copy at execution
        self.pre_perform(record_object_pre_perform)

    def plan(self) -> None:
        target_pose = self.object_designator.attachments[
            World.robot].get_child_link_target_pose_given_parent(self.target_location)
        pre_place_pose = target_pose.copy()
        pre_place_pose.position.z += 0.1
        MoveTCPMotion(pre_place_pose, self.arm).perform()

        MoveTCPMotion(target_pose, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        World.robot.detach(self.object_designator)

        retract_pose = LocalTransformer().translate_pose_along_local_axis(target_pose,
                                                                          self.end_effector.get_approach_axis(),
                                                                          -self.object_designator.get_approach_offset())
        MoveTCPMotion(retract_pose, self.arm).perform()

    @cached_property
    def gripper_link(self) -> Link:
        return World.robot.links[self.arm_chain.get_tool_frame()]

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    @cached_property
    def end_effector(self) -> EndEffectorDescription:
        return self.arm_chain.end_effector

    @cached_property
    def local_transformer(self) -> LocalTransformer:
        return LocalTransformer()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the object is placed at the target location.
        """
        self.validate_loss_of_contact()
        self.validate_placement_location()

    def validate_loss_of_contact(self):
        """
        Check if the object is still in contact with the robot after placing it.
        """
        contact_links = self.object_designator.get_contact_points_with_body(World.robot).get_all_bodies()
        if contact_links:
            raise ObjectStillInContact(self.object_designator, contact_links,
                                       self.target_location, World.robot, self.arm)

    def validate_placement_location(self):
        """
        Check if the object is placed at the target location.
        """
        pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_error_checker.is_error_acceptable(self.object_designator.pose, self.target_location):
            raise ObjectNotPlacedAtTargetLocation(self.object_designator, self.target_location, World.robot, self.arm)

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    target_location: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None) -> PartialDesignator[Type[PlaceAction]]:
        return PartialDesignator(PlaceAction, object_designator=object_designator,
                                 target_location=target_location,
                                 arm=arm)



PlaceActionDescription = PlaceAction.description