from pycram.robot_plans.actions.base import ActionDescription, record_object_pre_perform
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta

import numpy as np

from pycram.robot_plans.actions.core.grasping import PickUpActionDescription
from pycram.robot_plans.actions.core.navigation import NavigateActionDescription
from pycram.robot_plans.actions.core.placing import PlaceActionDescription
from pycram.robot_plans.actions.core.robot_body import ParkArmsActionDescription
from pycram.designators.object_designator import BelieveObject
from pycram.has_parameters import has_parameters
from pycram.plan import with_plan

from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.dataclasses import FrozenObject

from typing_extensions import Union, Optional, Type, Any, Iterable

from pycram.designators.location_designator import ProbabilisticCostmapLocation
from pycram.datastructures.grasp import GraspDescription
from pycram.failures import ObjectUnfetchable, ReachabilityFailure

from pycram.datastructures.enums import Arms

from pycram.datastructures.pose import PoseStamped

from pycram.robot_description import RobotDescription
from pycram.world_concepts.world_object import Object


@has_parameters
@dataclass
class TransportAction(ActionDescription):
    """
    Transports an object to a position using an arm
    """

    object_designator: Object = field(repr=False)
    """
    Object designator_description describing the object that should be transported.
    """
    target_location: PoseStamped
    """
    Target Location to which the object should be transported
    """
    arm: Optional[Arms]
    """
    Arm that should be used
    """
    place_rotation_agnostic: Optional[bool] = False
    """
    If True, the robot will place the object in the same orientation as it is itself, no matter how the object was grasped.
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
        robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
        ParkArmsActionDescription(Arms.BOTH).perform()
        pickup_loc = ProbabilisticCostmapLocation(target=self.object_designator,
                                                  reachable_for=robot_desig_resolved,
                                                  reachable_arm=self.arm)
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = pickup_loc.resolve()
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateActionDescription(pickup_pose, True).perform()
        PickUpActionDescription(self.object_designator, pickup_pose.arm,
                                grasp_description=pickup_pose.grasp_description).perform()
        ParkArmsActionDescription(Arms.BOTH).perform()
        try:
            place_loc = ProbabilisticCostmapLocation(
                target=self.target_location,
                reachable_for=robot_desig_resolved,
                reachable_arm=pickup_pose.arm,
                grasp_descriptions=[pickup_pose.grasp_description],
                object_in_hand=self.object_designator,
                rotation_agnostic=self.place_rotation_agnostic,
            ).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                self.object_designator, robot_desig_resolved, pickup_pose.arm, pickup_pose.grasp_description)
        NavigateActionDescription(place_loc, True).perform()

        if self.place_rotation_agnostic:
            # Placing rotation agnostic currently means that the robot will position its gripper in the same orientation
            # as it is itself, no matter how the object was grasped
            robot_rotation = robot_desig_resolved.get_pose().orientation
            self.target_location.orientation = robot_rotation
            approach_direction = GraspDescription(pickup_pose.grasp_description.approach_direction, None, False)
            side_grasp = np.array(robot_desig_resolved.robot_description.get_arm_chain(pickup_pose.arm).end_effector.grasps[approach_direction])
            # Inverting the quaternion for the used grasp to cancel it out during placing, since placing considers the
            # object orientation relative to the gripper )
            side_grasp *= np.array([-1, -1, -1, 1])
            self.target_location.rotate_by_quaternion(side_grasp.tolist())

        PlaceActionDescription(self.object_designator, self.target_location, pickup_pose.arm).perform()
        ParkArmsActionDescription(Arms.BOTH).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation of each core action is done in the action itself, so no more validation needed here.
        pass

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    target_location: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None, place_rotation_agnostic: Optional[bool] = False) -> PartialDesignator[Type[TransportAction]]:
        return PartialDesignator(TransportAction, object_designator=object_designator,
                                 target_location=target_location,
                                 arm=arm, place_rotation_agnostic=place_rotation_agnostic)

@has_parameters
@dataclass
class PickAndPlaceAction(ActionDescription):
    """
    Transports an object to a position using an arm without moving the base of the robot
    """

    object_designator: Object
    """
    Object designator_description describing the object that should be transported.
    """
    target_location: PoseStamped
    """
    Target Location to which the object should be transported
    """
    arm: Arms
    """
    Arm that should be used
    """
    grasp_description: GraspDescription
    """
    Description of the grasp to pick up the target
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
        ParkArmsActionDescription(Arms.BOTH).perform()
        PickUpActionDescription(self.object_designator, self.arm,
                     grasp_description=self.grasp_description).perform()
        ParkArmsActionDescription(Arms.BOTH).perform()
        PlaceActionDescription(self.object_designator, self.target_location, self.arm).perform()
        ParkArmsActionDescription(Arms.BOTH).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        if self.object_designator.pose.__eq__(self.target_location):
            pass
        else:
            raise ValueError("Object not moved to the target location")

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    target_location: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasp_description = GraspDescription) -> PartialDesignator[Type[PickAndPlaceAction]]:
        return PartialDesignator(PickAndPlaceAction, object_designator=object_designator,
                                 target_location=target_location,
                                 arm=arm,
                                 grasp_description=grasp_description)


PickAndPlaceActionDescription = PickAndPlaceAction.description
TransportActionDescription = TransportAction.description

