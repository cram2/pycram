# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import datetime
import inspect
import math
from dataclasses import dataclass, field
from datetime import timedelta
from functools import cached_property
from time import sleep

import numpy as np

from .object_designator import BelieveObject
from ..datastructures.world_entity import PhysicalBody
from ..has_parameters import has_parameters
from ..language import SequentialPlan, TryInOrderPlan
from ..plan import with_plan

from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.dataclasses import FrozenObject

from .. import utils
from ..tf_transformations import quaternion_from_euler
from typing_extensions import List, Union, Optional, Type, Dict, Any, Iterable

from pycrap.ontologies import Location, PhysicalObject
from .location_designator import CostmapLocation, ProbabilisticCostmapLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion
from ..datastructures.grasp import GraspDescription
from ..datastructures.world import World, UseProspectionWorld
from ..description import Joint, Link, ObjectDescription
from ..designator import ActionDescription, ObjectDesignatorDescription
from ..failure_handling import try_action
from ..failures import TorsoGoalNotReached, ConfigurationNotReached, ObjectNotInGraspingArea, \
    ObjectNotPlacedAtTargetLocation, ObjectStillInContact, LookAtGoalNotReached, \
    ContainerManipulationError
from ..local_transformer import LocalTransformer
from ..failures import ObjectUnfetchable, ReachabilityFailure, NavigationGoalNotReachedError, PerceptionObjectNotFound, \
    ObjectNotGraspedError
from ..robot_description import EndEffectorDescription
from ..ros import sleep
from ..config.action_conf import ActionConfig

from ..datastructures.enums import Arms, Grasp, GripperState, DetectionTechnique, DetectionState, MovementType, \
    TorsoState, StaticJointState, Frame, FindBodyInRegionMethod, ContainerManipulationType, AxisIdentifier, \
    VerticalAlignment

from ..datastructures.pose import PoseStamped, Vector3Stamped
from ..datastructures.world import World

from ..robot_description import RobotDescription, KinematicChainDescription
from ..ros import logwarn, loginfo
from ..validation.error_checkers import PoseErrorChecker
from ..validation.goal_validator import create_multiple_joint_goal_validator
from ..world_concepts.world_object import Object
from ..world_reasoning import move_away_all_objects_to_create_empty_space, generate_object_at_target, \
    cast_a_ray_from_camera, has_gripper_grasped_body, is_body_between_fingers


# ----------------------------------------------------------------------------
# ---------------- Performables ----------------------------------------------
# ----------------------------------------------------------------------------

def record_object_pre_perform(action):
    """
    Record the object before the action is performed.

    This should be appended to the pre performs of every action that interacts with an object.
    """
    # for every field in the action that is an object
    # write it to a dict mapping the OG field name to the frozen copy
    action.object_at_execution = action.object_designator.frozen_copy()


@has_parameters
@dataclass
class MoveTorsoAction(ActionDescription):
    """
    Move the torso of the robot up and down.
    """
    torso_state: TorsoState
    """
    The state of the torso that should be set
    """

    def plan(self) -> None:
        joint_positions: dict = RobotDescription.current_robot_description.get_static_joint_chain("torso",
                                                                                                  self.torso_state)
        MoveJointsMotion(list(joint_positions.keys()), list(joint_positions.values())).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: timedelta = timedelta(seconds=2)):
        """
        Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.
        """

        joint_positions: dict = RobotDescription.current_robot_description.get_static_joint_chain("torso",
                                                                                                  self.torso_state)
        validator = create_multiple_joint_goal_validator(World.current_world.robot, joint_positions)
        validator.wait_until_goal_is_achieved(max_wait_time=max_wait_time,
                                              time_per_read=timedelta(milliseconds=20))
        if not validator.goal_achieved:
            raise TorsoGoalNotReached(validator)

    @classmethod
    @with_plan
    def description(cls, torso_state: Union[Iterable[TorsoState], TorsoState]) -> PartialDesignator[
        Type[MoveTorsoAction]]:
        return PartialDesignator(MoveTorsoAction, torso_state=torso_state)


@has_parameters
@dataclass
class SetGripperAction(ActionDescription):
    """
    Set the gripper state of the robot.
    """

    gripper: Arms
    """
    The gripper that should be set 
    """
    motion: GripperState
    """
    The motion that should be set on the gripper
    """

    def plan(self) -> None:
        arm_chains = RobotDescription.current_robot_description.get_arm_chain(self.gripper)
        if type(arm_chains) is not list:
            MoveGripperMotion(gripper=arm_chains.arm_type, motion=self.motion).perform()
        else:
            for chain in arm_chains:
                MoveGripperMotion(gripper=chain.arm_type, motion=self.motion).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: timedelta = timedelta(seconds=2)):
        """
        Needs gripper state to be read or perceived.
        """
        pass

    @classmethod
    @with_plan
    def description(cls, gripper: Union[Iterable[Arms], Arms],
                    motion: Union[Iterable[GripperState], GripperState] = None) -> PartialDesignator[
        Type[SetGripperAction]]:
        return PartialDesignator(SetGripperAction, gripper=gripper, motion=motion)


@has_parameters
@dataclass
class ReleaseAction(ActionDescription):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """
    object_designator: Object
    """
    The object that should be released
    """

    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity
    """

    gripper: Arms
    """
    The gripper that should be used to release the object
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
        raise NotImplementedError

    @classmethod
    def description(cls, object_designator: ObjectDesignatorDescription,
                    gripper: Optional[Union[Iterable[Arms], Arms]] = None) -> PartialDesignator[
        Type[ReleaseAction]]:
        return PartialDesignator(ReleaseAction, object_designator=object_designator, gripper=gripper)


@has_parameters
@dataclass
class GripAction(ActionDescription):
    """
    Grip an object with the robot.

    Note: This action can not be used yet.
    """
    object_designator: Object
    """
    The object that should be gripped
    """

    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity
    """

    gripper: Arms
    """
    The gripper that should be used to grip the object
    """

    effort: float = None
    """
    The effort that should be used to grip the object
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
        raise NotImplementedError()

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    gripper: Union[Iterable[Arms], Arms] = None, effort: Union[Iterable[float], float] = None, ) -> \
            PartialDesignator[Type[GripAction]]:
        return PartialDesignator(GripAction, object_designator=object_designator,
                                 gripper=gripper, effort=effort)


@has_parameters
@dataclass
class ParkArmsAction(ActionDescription):
    """
    Park the arms of the robot.
    """

    arm: Arms
    """
    Entry from the enum for which arm should be parked.
    """

    def plan(self) -> None:
        joint_poses = self.get_joint_poses()

        MoveJointsMotion(names=list(joint_poses.keys()), positions=list(joint_poses.values())).perform()

    def get_joint_poses(self) -> Dict[str, float]:
        """
        :return: The joint positions that should be set for the arm to be in the park position.
        """
        joint_poses = {}
        arm_chains = RobotDescription.current_robot_description.get_arm_chain(self.arm)
        if type(arm_chains) is not list:
            joint_poses = arm_chains.get_static_joint_states(StaticJointState.Park)
        else:
            for arm_chain in RobotDescription.current_robot_description.get_arm_chain(self.arm):
                joint_poses.update(arm_chain.get_static_joint_states(StaticJointState.Park))
        return joint_poses

    def validate(self, result: Optional[Any] = None, max_wait_time: timedelta = timedelta(seconds=2)):
        """
        Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.
        """
        joint_poses = self.get_joint_poses()
        validator = create_multiple_joint_goal_validator(World.current_world.robot, joint_poses)
        validator.wait_until_goal_is_achieved(max_wait_time=max_wait_time,
                                              time_per_read=timedelta(milliseconds=20))
        if not validator.goal_achieved:
            raise ConfigurationNotReached(validator, configuration_type=StaticJointState.Park)

    @classmethod
    @with_plan
    def description(cls, arm: Union[Iterable[Arms], Arms]) -> PartialDesignator[Type[ParkArmsAction]]:
        return PartialDesignator(cls, arm=arm)


@has_parameters
@dataclass
class CarryAction(ActionDescription):
    """
    Parks the robot's arms. And align the arm with the given Axis of a frame.
    """

    arm: Arms
    """
    Entry from the enum for which arm should be parked.
    """

    align: Optional[bool] = False
    """
    If True, aligns the end-effector with a specified axis.
    """

    tip_link: Optional[str] = None
    """
    Name of the tip link to align with, e.g the object.
    """

    tip_axis: Optional[AxisIdentifier] = None
    """
    Tip axis of the tip link, that should be aligned.
    """

    root_link: Optional[str] = None
    """
    Base link of the robot; typically set to the torso.
    """

    root_axis: Optional[AxisIdentifier] = None
    """
    Goal axis of the root link, that should be used to align with.
    """

    def plan(self) -> None:
        joint_poses = self.get_joint_poses()
        tip_normal = self.axis_to_vector3_stamped(self.tip_axis, link=self.tip_link)
        root_normal = self.axis_to_vector3_stamped(self.root_axis, link=self.root_link)

        MoveJointsMotion(names=list(joint_poses.keys()), positions=list(joint_poses.values()),
                         align=self.align, tip_link=self.tip_link, tip_normal=tip_normal,
                         root_link=self.root_link, root_normal=root_normal).perform()

    def get_joint_poses(self) -> Dict[str, float]:
        """
        :return: The joint positions that should be set for the arm to be in the park position.
        """
        joint_poses = {}
        arm_chains = RobotDescription.current_robot_description.get_arm_chain(self.arm)
        if type(arm_chains) is not list:
            joint_poses = arm_chains.get_static_joint_states(StaticJointState.Park)
        else:
            for arm_chain in RobotDescription.current_robot_description.get_arm_chain(self.arm):
                joint_poses.update(arm_chain.get_static_joint_states(StaticJointState.Park))
        return joint_poses

    def axis_to_vector3_stamped(self, axis: AxisIdentifier, link: str = "base_link") -> Vector3Stamped:
        v = {
            AxisIdentifier.X: Vector3Stamped(x=1.0, y=0.0, z=0.0),
            AxisIdentifier.Y: Vector3Stamped(x=0.0, y=1.0, z=0.0),
            AxisIdentifier.Z: Vector3Stamped(x=0.0, y=0.0, z=1.0),
        }[axis]
        v.frame_id = link
        v.header.stamp = datetime.datetime.now()
        return v

    def validate(self, result: Optional[Any] = None, max_wait_time: timedelta = timedelta(seconds=2)):
        """
        Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.
        """
        joint_poses = self.get_joint_poses()
        validator = create_multiple_joint_goal_validator(World.current_world.robot, joint_poses)
        validator.wait_until_goal_is_achieved(max_wait_time=max_wait_time,
                                              time_per_read=timedelta(milliseconds=20))
        if not validator.goal_achieved:
            raise ConfigurationNotReached(validator, configuration_type=StaticJointState.Park)

    @classmethod
    @with_plan
    def description(cls, arm: Union[Iterable[Arms], Arms], align: Optional[bool] = False,
                    tip_link: Optional[str] = None, tip_axis: Optional[AxisIdentifier] = None,
                    root_link: Optional[str] = None, root_axis: Optional[AxisIdentifier] = None) \
            -> PartialDesignator[Type[CarryAction]]:
        return PartialDesignator(cls, arm=arm, align=align, tip_link=tip_link, tip_axis=tip_axis,
                                 root_link=root_link, root_axis=root_axis)

@has_parameters
@dataclass
class ReachToPickUpAction(ActionDescription):
    """
    Let the robot reach a specific pose.
    """

    object_designator: Object
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp_description: GraspDescription
    """
    The grasp description that should be used for picking up the object
    """

    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the world object is changed.
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

        target_pose = self.object_designator.get_grasp_pose(self.end_effector, self.grasp_description)
        target_pose.rotate_by_quaternion(self.end_effector.grasps[self.grasp_description])

        target_pre_pose = LocalTransformer().translate_pose_along_local_axis(target_pose,
                                                                             self.end_effector.get_approach_axis(),
                                                                             -self.object_designator.get_approach_offset())

        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        self.move_gripper_to_pose(target_pre_pose)

        self.move_gripper_to_pose(target_pose, MovementType.STRAIGHT_CARTESIAN)

        # Remove the vis axis from the world if it was added
        World.current_world.remove_vis_axis()

    def move_gripper_to_pose(self, pose: PoseStamped, movement_type: MovementType = MovementType.CARTESIAN,
                             add_vis_axis: bool = True):
        """
        Move the gripper to a specific pose.

        :param pose: The pose to go to.
        :param movement_type: The type of movement that should be performed.
        :param add_vis_axis: If a visual axis should be added to the world.
        """
        pose = self.local_transformer.transform_pose(pose, Frame.Map.value)
        if add_vis_axis:
            World.current_world.add_vis_axis(pose)
        MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=movement_type).perform()

    @cached_property
    def local_transformer(self) -> LocalTransformer:
        return LocalTransformer()

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    @cached_property
    def end_effector(self) -> EndEffectorDescription:
        return self.arm_chain.end_effector

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if object is contained in the gripper such that it can be grasped and picked up.
        """
        fingers_link_names = self.arm_chain.end_effector.fingers_link_names
        if fingers_link_names:
            if not is_body_between_fingers(self.object_designator, fingers_link_names,
                                           method=FindBodyInRegionMethod.MultiRay):
                raise ObjectNotInGraspingArea(self.object_designator, World.robot, self.arm, self.grasp_description)
        else:
            logwarn(f"Cannot validate reaching to pick up action for arm {self.arm} as no finger links are defined.")

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasp: Union[Iterable[Grasp], Grasp] = None) -> PartialDesignator[Type[ReachToPickUpAction]]:
        return PartialDesignator(ReachToPickUpAction, object_designator=object_designator,
                                 arm=arm,
                                 grasp=grasp)


@has_parameters
@dataclass
class PickUpAction(ActionDescription):
    """
    Let the robot pick up an object.
    """

    object_designator: Object
    """
    Object designator_description describing the object that should be picked up
    """

    arm: Arms
    """
    The arm that should be used for pick up
    """

    grasp_description: GraspDescription
    """
    The GraspDescription that should be used for picking up the object
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
        ReachToPickUpAction(self.object_designator, self.arm, self.grasp_description).perform()

        MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm).perform()

        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        World.robot.attach(self.object_designator, tool_frame)

        self.lift_object(distance=0.1)

        # Remove the vis axis from the world
        World.current_world.remove_vis_axis()

    def lift_object(self, distance: float = 0.1):
        lift_to_pose = self.gripper_pose()
        lift_to_pose.pose.position.z += distance
        MoveTCPMotion(lift_to_pose, self.arm, allow_gripper_collision=True).perform()

    def gripper_pose(self) -> PoseStamped:
        """
        Get the pose of the gripper.

        :return: The pose of the gripper.
        """
        gripper_link = self.arm_chain.get_tool_frame()
        return World.robot.links[gripper_link].pose

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if picked up object is in contact with the gripper.
        """
        if not has_gripper_grasped_body(self.arm, self.object_designator):
            raise ObjectNotGraspedError(self.object_designator, World.robot, self.arm, self.grasp_description)

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasp_description: Union[Iterable[GraspDescription], GraspDescription] = None) -> \
            PartialDesignator[Type[PickUpAction]]:
        return PartialDesignator(PickUpAction, object_designator=object_designator, arm=arm,
                                 grasp_description=grasp_description)


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
            side_grasp = np.array(robot_desig_resolved.robot_description.get_arm_chain(pickup_pose.arm).end_effector.grasps[approach_direction])
            # Inverting the quaternion for the used grasp to cancel it out during placing, since placing considers the
            # object orientation relative to the gripper )
            side_grasp *= np.array([-1, -1, -1, 1])
            self.target_location.rotate_by_quaternion(side_grasp.tolist())

        PlaceActionDescription(self.object_designator, self.target_location, pickup_pose.arm).perform()
        ParkArmsActionDescription(Arms.BOTH).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation of each atomic action is done in the action itself, so no more validation needed here.
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
class DetectAction(ActionDescription):
    """
    Detects an object that fits the object description and returns an object designator_description describing the object.

    If no object is found, an PerceptionObjectNotFound error is raised.
    """

    technique: DetectionTechnique
    """
    The technique that should be used for detection
    """
    state: Optional[DetectionState] = None
    """
    The state of the detection, e.g Start Stop for continues perception
    """
    object_designator: Optional[Object] = None
    """
    The type of the object that should be detected, only considered if technique is equal to Type
    """
    region: Optional[Location] = None
    """
    The region in which the object should be detected
    """

    # object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    # """
    # The object at the time this Action got created. It is used to be a static, information holding entity
    # """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    def __post_init__(self):
        super().__post_init__()

        # # Store the object's data copy at execution
        # self.pre_perform(record_object_pre_perform)

    def plan(self) -> None:
        return try_action(DetectingMotion(technique=self.technique, state=self.state,
                                          object_designator_description=self.object_designator,
                                          region=self.region), PerceptionObjectNotFound)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        return
        # if not result:
        #     raise PerceptionObjectNotFound(self.object_designator, self.technique, self.region)

    @classmethod
    @with_plan
    def description(cls, technique: Union[Iterable[DetectionTechnique], DetectionTechnique],
                    state: Union[Iterable[DetectionState], DetectionState] = None,
                    object_designator: Union[Iterable[Object], Object] = None,
                    region: Union[Iterable[Location], Location] = None) -> PartialDesignator[Type[DetectAction]]:
        return PartialDesignator(DetectAction, technique=technique,
                                 state=state,
                                 object_designator=object_designator,
                                 region=region)


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
        GraspingAction(self.object_designator, self.arm, self.grasping_prepose_distance).perform()
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
        GraspingAction(self.object_designator, self.arm, self.grasping_prepose_distance).perform()
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


@has_parameters
@dataclass
class GraspingAction(ActionDescription):
    """
    Grasps an object described by the given Object Designator description
    """
    object_designator: Object# Union[Object, ObjectDescription.Link]
    """
    Object Designator for the object that should be grasped
    """
    arm: Arms
    """
    The arm that should be used to grasp
    """
    prepose_distance: float = ActionConfig.grasping_prepose_distance
    """
    The distance in meters the gripper should be at before grasping the object
    """

    def plan(self) -> None:
        object_pose = self.object_designator.pose
        lt = LocalTransformer()
        gripper_name = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()

        object_pose_in_gripper = lt.transform_pose(object_pose,
                                                   World.robot.get_link_tf_frame(gripper_name))

        pre_grasp = object_pose_in_gripper.copy()
        pre_grasp.pose.position.x -= self.prepose_distance

        MoveTCPMotion(pre_grasp, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm).perform()

        MoveTCPMotion(object_pose, self.arm, allow_gripper_collision=True).perform()
        MoveGripperMotion(GripperState.CLOSE, self.arm, allow_gripper_collision=True).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        body = self.object_designator
        contact_links = body.get_contact_points_with_body(World.robot).get_all_bodies()
        arm_chain = RobotDescription.current_robot_description.get_arm_chain(self.arm)
        gripper_links = arm_chain.end_effector.links
        if not any([link.name in gripper_links for link in contact_links]):
            raise ObjectNotGraspedError(self.object_designator, World.robot, self.arm, None)

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    arm: Union[Iterable[Arms], Arms] = None,
                    prepose_distance: Union[Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> \
            PartialDesignator[Type[GraspingAction]]:
        return PartialDesignator(GraspingAction, object_designator=object_designator, arm=arm,
                                 prepose_distance=prepose_distance)


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
        NavigateAction(self.standing_position, self.keep_joint_states).perform()
        FaceAtAction(self.object_designator.pose, self.keep_joint_states).perform()
        PickUpAction(self.object_designator, self.arm, self.grasp_description).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
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
        NavigateAction(self.standing_position, self.keep_joint_states).perform()
        FaceAtAction(self.target_location, self.keep_joint_states).perform()
        PlaceAction(self.object_designator, self.target_location, self.arm).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
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

MoveTorsoActionDescription = MoveTorsoAction.description
SetGripperActionDescription = SetGripperAction.description
ParkArmsActionDescription = ParkArmsAction.description
ReachToPickUpActionDescription = ReachToPickUpAction.description
PickUpActionDescription = PickUpAction.description
PlaceActionDescription = PlaceAction.description
NavigateActionDescription = NavigateAction.description
TransportActionDescription = TransportAction.description
LookAtActionDescription = LookAtAction.description
DetectActionDescription = DetectAction.description
OpenActionDescription = OpenAction.description
CloseActionDescription = CloseAction.description
GraspingActionDescription = GraspingAction.description
FaceAtActionDescription = FaceAtAction.description
MoveAndPickUpActionDescription = MoveAndPickUpAction.description
MoveAndPlaceActionDescription = MoveAndPlaceAction.description
ReleaseActionDescription = ReleaseAction.description
GripActionDescription = GripAction.description
SearchActionDescription = SearchAction.description
PickAndPlaceActionDescription = PickAndPlaceAction.description
CarryActionDescription = CarryAction.description
