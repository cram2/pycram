from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta

from semantic_world.world_description.connections import Connection6DoF
from semantic_world.world_description.world_entity import Body
from typing_extensions import Union, Optional, Type, Any, Iterable

from config.action_conf import ActionConfig
from ...motions.gripper import MoveTCPMotion, MoveGripperMotion
from ....datastructures.dataclasses import FrozenObject
from ....datastructures.enums import Arms, GripperState
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....failures import ObjectNotPlacedAtTargetLocation, ObjectStillInContact
from ....has_parameters import has_parameters
from ....language import SequentialPlan
from ....robot_description import ViewManager
from ....robot_plans.actions.base import ActionDescription, record_object_pre_perform
from ....utils import translate_pose_along_local_axis
from ....validation.error_checkers import PoseErrorChecker


@has_parameters
@dataclass
class PlaceAction(ActionDescription):
    """
    Places an Object at a position using an arm.
    """

    object_designator: Body
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
        pre_place_pose = self.world.transform(self.target_location.to_spatial_type(), self.world.root)
        pre_place_pose = PoseStamped.from_spatial_type(pre_place_pose)
        pre_place_pose.position.z += 0.1
        SequentialPlan(self.context, self.robot_view,
                       MoveTCPMotion(pre_place_pose, self.arm),

                       MoveTCPMotion(self.target_location, self.arm),

                       MoveGripperMotion(GripperState.OPEN, self.arm)).perform()

        # Detaches the object from the robot
        world_root = self.world.root
        obj_transform = self.world.compute_forward_kinematics(world_root, self.object_designator)
        with self.world.modify_world():
            self.world.remove_connection(self.object_designator.parent_connection)
            connection = Connection6DoF(world_root, self.object_designator, _world=self.world)
            connection.origin = obj_transform
            self.world.add_connection(connection)

        ee_view = ViewManager().get_end_effector_view(self.arm, self.robot_view)

        retract_pose = translate_pose_along_local_axis(PoseStamped.from_spatial_type(self.object_designator.global_pose),
                                                       ee_view.front_facing_axis.to_np()[:3],
                                                       -ActionConfig.pick_up_prepose_distance)

        SequentialPlan(self.context, self.robot_view,  MoveTCPMotion(retract_pose, self.arm)).perform()

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
    def description(cls, object_designator: Union[Iterable[Body], Body],
                    target_location: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms]) -> PartialDesignator[Type[PlaceAction]]:
        return PartialDesignator(PlaceAction, object_designator=object_designator,
                                 target_location=target_location,
                                 arm=arm)


PlaceActionDescription = PlaceAction.description
