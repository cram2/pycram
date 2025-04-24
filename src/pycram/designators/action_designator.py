# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import inspect
from dataclasses import dataclass, field
from datetime import timedelta
from functools import cached_property
from time import sleep

import numpy as np
from ..plan import with_plan

from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.dataclasses import FrozenObject

from .. import utils
from ..tf_transformations import quaternion_from_euler
from typing_extensions import List, Union, Optional, Type, Dict, Any, Iterable, Self

from pycrap.ontologies import Location
from .location_designator import CostmapLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..datastructures.enums import Arms, Grasp, GripperState, DetectionTechnique, DetectionState, MovementType, \
    TorsoState, StaticJointState, Frame, FindBodyInRegionMethod, ContainerManipulationType, Frame, FindBodyInRegionMethod, ContainerManipulationType
from ..datastructures.pose import PoseStamped
from ..datastructures.grasp import GraspDescription
from ..datastructures.world import World, UseProspectionWorld
from ..description import Joint, Link, ObjectDescription
from ..designator import ActionDescription
from ..failure_handling import try_action
from ..failures import TorsoGoalNotReached, ConfigurationNotReached, ObjectNotInGraspingArea, \
    ObjectNotPlacedAtTargetLocation, ObjectStillInContact, LookAtGoalNotReached, \
    ContainerManipulationError
from ..local_transformer import LocalTransformer
from ..failures import ObjectUnfetchable, ReachabilityFailure, NavigationGoalNotReachedError, PerceptionObjectNotFound, \
    ObjectNotGraspedError
from ..config.action_conf import ActionConfig

from ..datastructures.enums import Arms, Grasp, GripperState, DetectionTechnique, DetectionState, MovementType, \
    TorsoState, StaticJointState

from ..datastructures.pose import PoseStamped
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
    def description(cls, torso_state: Union[Iterable[TorsoState], TorsoState]) -> PartialDesignator[Type[MoveTorsoAction]]:
        return PartialDesignator(MoveTorsoAction, torso_state=torso_state)


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
                    motion: Union[Iterable[GripperState], GripperState] = None) -> PartialDesignator[Type[SetGripperAction]]:
        return PartialDesignator(SetGripperAction, gripper=gripper, motion=motion)


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
                    gripper: Union[Iterable[Arms], Arms] = None, effort: Union[Iterable[float], float] = None, ) -> PartialDesignator[Type[GripAction]]:
        return PartialDesignator(GripAction, object_designator=object_designator,
                                 gripper=gripper, effort=effort)


@dataclass
class ParkArmsAction(ActionDescription):
    """
    Park the arms of the robot.
    """

    arm: Arms
    """
    Entry from the enum for which arm should be parked
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
        return PartialDesignator(ParkArmsAction, arm=arm)


SPECIAL_KNOWLEDGE = {
    'bigknife':
        [("top", [-0.08, 0, 0])],
    'whisk':
        [("top", [-0.08, 0, 0])],
    'bowl':
        [("front", [1.0, 2.0, 3.0]),
         ("key2", [4.0, 5.0, 6.0])]
}


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

    prepose_distance: float = ActionConfig.pick_up_prepose_distance
    """
    The distance in meters the gripper should be at before picking up the object
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
        adjusted_oTm = self.adjust_target_pose_to_grasp_type()

        pre_pose = self.calculate_pre_grasping_pose(adjusted_oTm)

        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        self.go_to_pose(pre_pose)

        self.go_to_pose(adjusted_oTm, MovementType.STRAIGHT_CARTESIAN, add_vis_axis=True)

        # Remove the vis axis from the world if it was added
        World.current_world.remove_vis_axis()

    def go_to_pose(self, pose: PoseStamped, movement_type: MovementType = MovementType.CARTESIAN,
                   add_vis_axis: bool = True):
        """
        Go to a specific pose.

        :param pose: The pose to go to.
        :param movement_type: The type of movement that should be performed.
        :param add_vis_axis: If a visual axis should be added to the world.
        """
        pose = self.local_transformer.transform_pose(pose, Frame.Map.value)
        if add_vis_axis:
            World.current_world.add_vis_axis(pose)
        MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=movement_type).perform()

    def adjust_target_pose_to_grasp_type(self) -> PoseStamped:
        """
        Adjust the target pose according to the grasp type by adjusting the orientation of the gripper.

        :return: The adjusted target pose.
        """
        # Get grasp orientation and target pose
        grasp = RobotDescription.current_robot_description.get_arm_chain(self.arm).end_effector.grasps[
            self.grasp_description]
        oTm = self.object_designator.get_pose()
        # Transform the object pose to the object frame, basically the origin of the object frame
        mTo = self.local_transformer.transform_to_object_frame(oTm, self.object_designator)
        # Adjust the pose according to the special knowledge of the object designator_description
        adjusted_pose = self.special_knowledge_adjustment_pose(grasp, mTo)
        # Transform the adjusted pose to the map frame
        adjusted_oTm = self.local_transformer.transform_pose(adjusted_pose, Frame.Map.value)
        # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper
        adjusted_oTm.rotate_by_quaternion(grasp)
        return adjusted_oTm

    def special_knowledge_adjustment_pose(self, grasp: Grasp, pose: PoseStamped) -> PoseStamped:
        """
        Get the adjusted target pose based on special knowledge for "grasp front".

        :param grasp: From which side the object should be grasped
        :param pose: Pose at which the object should be grasped, before adjustment
        :return: The adjusted grasp pose
        """
        lt = LocalTransformer()
        pose_in_object = lt.transform_pose(pose, self.object_designator.tf_frame)

        special_knowledge = []  # Initialize as an empty list
        if self.object_designator.obj_type in SPECIAL_KNOWLEDGE:
            special_knowledge = SPECIAL_KNOWLEDGE[self.object_designator.obj_type]

        for key, value in special_knowledge:
            if key == grasp:
                # Adjust target pose based on special knowledge
                pose_in_object.pose.position.x += value[0]
                pose_in_object.pose.position.y += value[1]
                pose_in_object.pose.position.z += value[2]
                loginfo(f"Adjusted target pose based on special knowledge for grasp: {grasp}")
                return pose_in_object
        return pose

    def calculate_pre_grasping_pose(self, obj_pose: PoseStamped) -> PoseStamped:
        """
        Calculate the pre grasping pose of the object depending on the gripper and the pre-pose distance.

        :return: The pre grasping pose of the object.
        """
        # pre-pose depending on the gripper.
        oTg = self.local_transformer.transform_pose(obj_pose, self.gripper_frame)
        oTg.pose.position.x -= self.prepose_distance  # in x since this is how the gripper is oriented
        return self.local_transformer.transform_pose(oTg, Frame.Map.value)

    @cached_property
    def local_transformer(self) -> LocalTransformer:
        return LocalTransformer()

    @cached_property
    def gripper_frame(self) -> str:
        return World.robot.get_link_tf_frame(self.arm_chain.get_tool_frame())

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

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
                    grasp: Union[Iterable[Grasp], Grasp] = None,
                    prepose_distance: Union[Iterable[float], float] = ActionConfig.pick_up_prepose_distance) -> \
    PartialDesignator[Type[ReachToPickUpAction]]:
        return PartialDesignator(ReachToPickUpAction, object_designator=object_designator,
                                 arm=arm,
                                 grasp=grasp,
                                 prepose_distance=prepose_distance)


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

    prepose_distance: float = ActionConfig.pick_up_prepose_distance
    """
    The distance in meters the gripper should be at before picking up the object
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
        ReachToPickUpAction(self.object_designator, self.arm, self.grasp_description,
                                       self.prepose_distance).perform()

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
                    grasp_description: Union[Iterable[GraspDescription], GraspDescription] = None,
                    prepose_distance: Union[Iterable[float], float] = ActionConfig.pick_up_prepose_distance) -> \
            PartialDesignator[Type[PickUpAction]]:
        return PartialDesignator(PickUpAction, object_designator=object_designator, arm=arm,
                                 grasp_description=grasp_description, prepose_distance=prepose_distance)


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
        MoveTCPMotion(target_pose, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        World.robot.detach(self.object_designator)

        retract_pose = self.calculate_retract_pose(target_pose, distance=0.05)
        MoveTCPMotion(retract_pose, self.arm).perform()

    def calculate_target_pose_of_gripper(self):
        """
        Calculate the target pose of the gripper given the target pose of the held object.
        wTg (world to gripper) = wTo (world to object target) * oTg (object to gripper, this is constant since object
        is attached to the gripper)
        """
        gripper_pose_in_object = self.local_transformer.transform_pose(self.gripper_link.pose,
                                                                       self.object_designator.tf_frame)
        object_to_gripper = gripper_pose_in_object.to_transform_stamped("object")
        world_to_object_target = self.target_location.to_transform_stamped("target")
        world_to_gripper_target = world_to_object_target * object_to_gripper
        return world_to_gripper_target.to_pose_stamped()

    def calculate_retract_pose(self, target_pose: PoseStamped, distance: float) -> PoseStamped:
        """
        Calculate the retract pose of the gripper.

        :param target_pose: The target pose of the gripper.
        :param distance: The distance the gripper should be retracted.
        """
        retract_pose = self.local_transformer.transform_pose(target_pose, self.gripper_link.tf_frame)
        retract_pose.position.x -= distance
        return retract_pose

    @cached_property
    def gripper_link(self) -> Link:
        return World.robot.links[self.arm_chain.get_tool_frame()]

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

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


@dataclass
class TransportAction(ActionDescription):
    """
    Transports an object to a position using an arm
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
    pickup_prepose_distance: float = ActionConfig.pick_up_prepose_distance
    """
    Distance between the object and the gripper in the x-axis before picking up the object.
    """
    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity
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
        ParkArmsAction(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=self.object_designator,
                                     reachable_for=robot_desig_resolved,
                                     reachable_arm=[self.arm],
                                     prepose_distance=self.pickup_prepose_distance)
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = pickup_loc.resolve()
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateAction(pickup_pose, True).perform()
        PickUpAction(self.object_designator, pickup_pose.arm,
                     grasp_description=pickup_pose.grasp_description,
                     prepose_distance=self.pickup_prepose_distance).perform()
        ParkArmsAction(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(
                target=self.target_location,
                reachable_for=robot_desig_resolved,
                reachable_arm=[pickup_pose.arm],
                grasp_descriptions=[pickup_pose.grasp_description],
                object_in_hand=self.object_designator
            ).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                f"No location found from where the robot can reach the target location: {self.target_location}")
        NavigateAction(place_loc, True).perform()
        PlaceAction(self.object_designator, self.target_location, self.arm).perform()
        ParkArmsAction(Arms.BOTH).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation of each atomic action is done in the action itself, so no more validation needed here.
        pass

    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    target_location: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None,
                    pickup_prepose_distance: Union[Iterable[float], float] = ActionConfig.pick_up_prepose_distance) -> \
            PartialDesignator[Type[TransportAction]]:
        return PartialDesignator(TransportAction, object_designator=object_designator,
                                 target_location=target_location,
                                 arm=arm,
                                 pickup_prepose_distance=pickup_prepose_distance)


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
    state: DetectionState = None
    """
    The state of the detection, e.g Start Stop for continues perception
    """
    object_designator: Optional[Object] = None
    """
    The type of the object that should be detected, only considered if technique is equal to Type
    """
    region: Location = None
    """
    The region in which the object should be detected
    """

    object_at_execution: Optional[FrozenObject] = field(init=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity
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
        return try_action(DetectingMotion(technique=self.technique, state=self.state,
                                          object_designator_description=self.object_designator,
                                          region=self.region), PerceptionObjectNotFound)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        if not result:
            raise PerceptionObjectNotFound(self.object_designator, self.technique, self.region)

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
                    grasping_prepose_distance: Union[Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> \
            PartialDesignator[Type[OpenAction]]:
        return PartialDesignator(OpenAction, object_designator=object_designator_description,
                                 arm=arm,
                                 grasping_prepose_distance=grasping_prepose_distance)


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
                    grasping_prepose_distance: Union[Iterable[float], float] = ActionConfig.grasping_prepose_distance) -> \
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


@dataclass
class GraspingAction(ActionDescription):
    """
    Grasps an object described by the given Object Designator description
    """
    object_designator: Union[Object, ObjectDescription.Link]
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
        return PartialDesignator(GraspingAction, object_designator=object_designator, arm=arm, prepose_distance=prepose_distance)


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

    grasp: Grasp
    """
    The grasp to use
    """

    keep_joint_states: bool = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    pick_up_prepose_distance: float = ActionConfig.pick_up_prepose_distance
    """
    The distance in meters the gripper should be at before picking up the object
    """

    def plan(self):
        if self.grasp == Grasp.TOP:
            grasp = GraspDescription(Grasp.FRONT, self.grasp, False)
        else:
            grasp = GraspDescription(self.grasp, None, False)
        NavigateAction(self.standing_position, self.keep_joint_states).perform()
        FaceAtAction(self.object_designator.pose, self.keep_joint_states).perform()
        PickUpAction(self.object_designator, self.arm, grasp,
                     self.pick_up_prepose_distance).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
        pass

    @classmethod
    @with_plan
    def description(cls, standing_position: Union[Iterable[PoseStamped], PoseStamped],
                    object_designator: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None,
                    grasp: Union[Iterable[Grasp], Grasp] = None,
                    keep_joint_states: Union[Iterable[bool], bool] = ActionConfig.navigate_keep_joint_states,
                    pick_up_prepose_distance: Union[Iterable[float], float] = ActionConfig.pick_up_prepose_distance) -> \
            PartialDesignator[Type[MoveAndPickUpAction]]:
        return PartialDesignator(MoveAndPickUpAction,
                                 standing_position=standing_position,
                                 object_designator=object_designator,
                                 arm=arm,
                                 grasp=grasp,
                                 pick_up_prepose_distance=pick_up_prepose_distance)


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


@dataclass
class PouringAction(ActionDescription):
    """
    Action class for the Pouring action.
    """

    object_: ObjectDesignatorDescription
    """
    The object to be poured into.
    """

    tool: ObjectDesignatorDescription
    """
    The tool used for pouring.
    """

    arm: Arms
    """
    The robot arm designated for the pouring task.
    """

    technique: Optional[str] = None
    """
    The technique used for pouring (default is None).
    """

    angle: Optional[float] = 90
    """
    The angle of the pouring action (default is 90).
    """

    def plan(self) -> None:
        lt = LocalTransformer()
        movement_type: MovementType = MovementType.CARTESIAN
        oTm = self.object_.pose
        grasp_rotation = RobotDescription.current_robot_description.get_arm_chain(self.arm).end_effector.get_grasp(
            Grasp.FRONT, None, False)
        oTbs = lt.transform_pose(oTm, World.robot.get_link_tf_frame("base_link"))
        oTbs.pose.position.x += 0.009
        oTbs.pose.position.z += 0.17
        oTbs.pose.position.y -= 0.125

        oTms = lt.transform_pose(oTbs, "map")
        World.current_world.add_vis_axis(oTms)
        oTog = lt.transform_pose(oTms, World.robot.get_link_tf_frame("base_link"))
        oTog.orientation = grasp_rotation
        oTgm = lt.transform_pose(oTog, "map")

        MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False, movement_type=movement_type).perform()

        World.current_world.add_vis_axis(oTgm)

        adjusted_oTgm = oTgm.copy()
        new_q = utils.axis_angle_to_quaternion([1, 0, 0], self.angle)
        new_x = new_q[0]
        new_y = new_q[1]
        new_z = new_q[2]
        new_w = new_q[3]
        adjusted_oTgm.rotate_by_quaternion([new_x, new_y, new_z, new_w])

        World.current_world.add_vis_axis(adjusted_oTgm)
        MoveTCPMotion(adjusted_oTgm, self.arm, allow_gripper_collision=False, movement_type=movement_type).perform()
        sleep(3)
        MoveTCPMotion(oTgm, self.arm, allow_gripper_collision=False, movement_type=movement_type).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
        pass

    @classmethod
    @with_plan
    def description(cls, object: Union[Iterable[Object], Object],
                    tool: Union[Iterable[Object], Object],
                    arm: Optional[Union[Iterable[Arms], Arms]] = None,
                    technique: Optional[Union[Iterable[str], str]] = None,
                    angle: Optional[Union[Iterable[float], float]] = 90) -> PartialDesignator[Type[PouringAction]]:
        return PartialDesignator(PouringAction, object=object, tool=tool, arm=arm, technique=technique, angle=angle)


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
PouringActionDescription = PouringAction.description
