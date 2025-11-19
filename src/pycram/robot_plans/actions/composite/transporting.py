from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta

import numpy as np
from typing_extensions import Union, Optional, Type, Any, Iterable

from ....config.action_conf import ActionConfig
from .facing import FaceAtActionDescription
from ..core import ParkArmsActionDescription, NavigateActionDescription, PickUpActionDescription, PlaceActionDescription, \
    PlaceAction
from ....datastructures.dataclasses import FrozenObject
from ....datastructures.enums import Arms, Grasp, VerticalAlignment
from ....datastructures.grasp import GraspDescription
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import PoseStamped
from ....designators.location_designator import ProbabilisticCostmapLocation, CostmapLocation
from ....designators.object_designator import BelieveObject
from ....failures import ObjectUnfetchable, ReachabilityFailure, ConfigurationNotReached
from ....has_parameters import has_parameters
from ....plan import with_plan
from ....robot_description import RobotDescription
from ....robot_plans.actions.base import ActionDescription, record_object_pre_perform
from ....ros import loginfo
from ....world_concepts.world_object import Object
from ....datastructures.world import World


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
            approach_direction = GraspDescription(pickup_pose.grasp_description.approach_direction, VerticalAlignment.NoAlignment, False)
            side_grasp = np.array(
                robot_desig_resolved.robot_description.get_arm_chain(pickup_pose.arm).end_effector.grasps[
                    approach_direction])
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
                    arm: Union[Iterable[Arms], Arms] = None, place_rotation_agnostic: Optional[bool] = False) -> \
    PartialDesignator[Type[TransportAction]]:
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

@has_parameters
@dataclass
class MoveAndPlaceAction(ActionDescription):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: PoseStamped
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: Object
    """
    The object to pick up
    """

    target_location: PoseStamped
    """
    The location to place the object.
    """

    arm: Arms
    """
    The arm to use
    """

    keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    def plan(self):
        NavigateActionDescription(self.standing_position, self.keep_joint_states).perform()
        FaceAtActionDescription(self.target_location, self.keep_joint_states).perform()
        PlaceAction(self.object_designator, self.target_location, self.arm).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the core action perform methods so no need to validate here.
        pass

    @classmethod
    @with_plan
    def description(cls, standing_position: Union[Iterable[PoseStamped], PoseStamped],
                    object_designator: Union[Iterable[Object], Object],
                    target_location: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None,
                    keep_joint_states: Union[Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states, ) -> \
            PartialDesignator[Type[MoveAndPlaceAction]]:
        return PartialDesignator(MoveAndPlaceAction,
                                 standing_position=standing_position,
                                 object_designator=object_designator,
                                 target_location=target_location,
                                 arm=arm)



@has_parameters
@dataclass
class MoveAndPickUpAction(ActionDescription):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: PoseStamped
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: Object
    """
    The object to pick up
    """

    arm: Arms
    """
    The arm to use
    """

    grasp_description: GraspDescription
    """
    The grasp to use
    """

    keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
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

    def plan(self):
        NavigateActionDescription(self.standing_position, self.keep_joint_states).perform()
        FaceAtActionDescription(self.object_designator.pose, self.keep_joint_states).perform()
        PickUpActionDescription(self.object_designator, self.arm, self.grasp_description).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the core action perform methods so no need to validate here.
        pass

    @classmethod
    @with_plan
    def description(cls, standing_position: Union[Iterable[PoseStamped], PoseStamped],
                    object_designator: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasp_description: Union[Iterable[Grasp], Grasp] = None,
                    keep_joint_states: Union[Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states) -> \
            PartialDesignator[Type[MoveAndPickUpAction]]:
        return PartialDesignator(MoveAndPickUpAction,
                                 standing_position=standing_position,
                                 object_designator=object_designator,
                                 arm=arm,
                                 grasp_description=grasp_description,
                                 keep_joint_states=keep_joint_states)




@has_parameters
@dataclass
class EfficientTransportAction(ActionDescription):
    """
    To transport an object to a target location by choosing the closest
    available arm using simple Euclidean distance.
    """
    object_designator: Object
    target_location: PoseStamped

    def _choose_best_arm(self, robot: Object, obj: Object) -> Arms:
        """
        Function to find the closest available arm.
        """
        rd = RobotDescription.current_robot_description
        try:
            left_tool_frame = rd.get_arm_chain(Arms.LEFT).get_tool_frame()
            right_tool_frame = rd.get_arm_chain(Arms.RIGHT).get_tool_frame()
            left_tip = robot.get_link_position(left_tool_frame)
            right_tip = robot.get_link_position(right_tool_frame)
        except Exception as e:
            raise ConfigurationNotReached(f"Could not get tool frames or link positions for arms: {e}")

        # Calculating the distance from gripper to the object
        object_pos_vec = np.array([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
        left_dist = np.linalg.norm(np.array(left_tip) - object_pos_vec)
        right_dist = np.linalg.norm(np.array(right_tip) - object_pos_vec)

        # If the arms are free or not
        attached_links = robot._attached_objects.values() if hasattr(robot, '_attached_objects') else []
        left_free = left_tool_frame not in attached_links
        right_free = right_tool_frame not in attached_links

        # Decide which arm to use based on proximity and availability
        if left_free and (not right_free or left_dist <= right_dist):
            return Arms.LEFT
        elif right_free:
            return Arms.RIGHT
        else:
            raise ConfigurationNotReached("No free arm available to grasp the object.")

    def plan(self) -> None:
        """
        The main plan for the transport action, optimized for a stationary robot.
        """
        robot = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
        obj = self.object_designator

        if not obj or not obj.pose:
            raise ConfigurationNotReached(f"Couldn't resolve the pose for the object: {self.object_designator}")

        # Intelligently choose the best arm
        chosen_arm = self._choose_best_arm(robot, obj)
        loginfo(f"Chosen arm for transport: {chosen_arm.name}")

        ParkArmsActionDescription(Arms.BOTH).perform()

        PickUpActionDescription(
            object_designator=self.object_designator,
            arm=chosen_arm
        ).perform()

        ParkArmsActionDescription(Arms.BOTH).perform()

        # Attempting the placement.
        PlaceActionDescription(
            object_designator=self.object_designator,
            target_location=self.target_location,
            arm=chosen_arm
        ).perform()


        ParkArmsActionDescription(Arms.BOTH).perform()

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    target_location: Union[Iterable[PoseStamped], PoseStamped]) -> PartialDesignator[Type['EfficientTransportAction']]:
        return PartialDesignator(cls,
                                 object_designator=object_designator,
                                 target_location=target_location)





TransportActionDescription = TransportAction.description
PickAndPlaceActionDescription = PickAndPlaceAction.description
MoveAndPlaceActionDescription = MoveAndPlaceAction.description
MoveAndPickUpActionDescription = MoveAndPickUpAction.description
EfficientTransportActionDescription = EfficientTransportAction.description
