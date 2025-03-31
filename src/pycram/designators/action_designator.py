# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import inspect
import itertools
from dataclasses import dataclass, field
from datetime import timedelta
from functools import cached_property

import numpy as np
from sqlalchemy.orm import Session

from .. import utils
from ..tf_transformations import quaternion_from_euler
from typing_extensions import List, Union, Optional, Type, Dict, Any

from pycrap.ontologies import Location
from .location_designator import CostmapLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..datastructures.enums import Arms, Grasp, GripperState, DetectionTechnique, DetectionState, MovementType, \
    TorsoState, StaticJointState, Frame, FindBodyInRegionMethod, ContainerManipulationType
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.pose import Pose
from ..datastructures.grasp import GraspDescription
from ..datastructures.property import GraspableProperty, ReachableProperty, GripperIsFreeProperty, SpaceIsFreeProperty
from ..datastructures.world import World, UseProspectionWorld
from ..description import Joint, Link
from ..designator import ActionDesignatorDescription
from ..failure_handling import try_action
from ..failures import ObjectUnfetchable, ReachabilityFailure, NavigationGoalNotReachedError, PerceptionObjectNotFound, \
    ObjectNotGraspedError, TorsoGoalNotReached, ConfigurationNotReached, ObjectNotInGraspingArea, \
    ObjectNotPlacedAtTargetLocation, ObjectStillInContact, LookAtGoalNotReached, \
    ContainerManipulationError
from ..knowledge.knowledge_engine import ReasoningInstance
from ..local_transformer import LocalTransformer
from ..failures import ObjectUnfetchable, ReachabilityFailure, NavigationGoalNotReachedError, PerceptionObjectNotFound, \
    ObjectNotGraspedError
from ..robot_description import RobotDescription
from ..ros import sleep
from ..tasktree import with_tree
from ..world_reasoning import contact

from owlready2 import Thing

from ..datastructures.enums import Arms, Grasp, GripperState, DetectionTechnique, DetectionState, MovementType, \
    TorsoState, StaticJointState

from ..designator import ActionDesignatorDescription
from ..datastructures.pose import Pose
from ..datastructures.world import World

from ..orm.action_designator import Action as ORMAction
from ..orm.action_designator import (ParkArmsAction as ORMParkArmsAction, NavigateAction as ORMNavigateAction,
                                     PickUpAction as ORMPickUpAction, PlaceAction as ORMPlaceAction,
                                     MoveTorsoAction as ORMMoveTorsoAction, SetGripperAction as ORMSetGripperAction,
                                     LookAtAction as ORMLookAtAction, DetectAction as ORMDetectAction,
                                     TransportAction as ORMTransportAction, OpenAction as ORMOpenAction,
                                     CloseAction as ORMCloseAction, GraspingAction as ORMGraspingAction,
                                     FaceAtAction as ORMFaceAtAction, ReachToPickUpAction as ORMReachToPickUpAction)
from ..orm.base import Pose as ORMPose
from ..orm.object_designator import Object as ORMObject
from ..robot_description import RobotDescription, KinematicChainDescription
from ..ros import logwarn
from ..tasktree import with_tree
from ..validation.error_checkers import PoseErrorChecker
from ..validation.goal_validator import create_multiple_joint_goal_validator
from ..world_concepts.world_object import Object
from ..world_reasoning import move_away_all_objects_to_create_empty_space, generate_object_at_target, \
    cast_a_ray_from_camera, has_gripper_grasped_body, is_body_between_fingers


# ----------------------------------------------------------------------------
# ---------------- Performables ----------------------------------------------
# ----------------------------------------------------------------------------


@dataclass
class ActionAbstract(ActionDesignatorDescription.Action, abc.ABC):
    """Base class for performable performables."""
    orm_class: Type[ORMAction] = field(init=False, default=None, repr=False)
    """
    The ORM class that is used to insert this action into the database. Must be overwritten by every action in order to
    be able to insert the action into the database.
    """

    @abc.abstractmethod
    def plan(self) -> None:
        """
        plan of the action.

        Will be overwritten by each action.
        """
        pass

    def to_sql(self) -> ORMAction:
        """
        Convert this action to its ORM equivalent.

        Needs to be overwritten by an action if it didn't overwrite the orm_class attribute with its ORM equivalent.

        :return: An instance of the ORM equivalent of the action with the parameters set
        """
        # get all class parameters
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # list of parameters that will be passed to the ORM class. If the name does not match the orm_class equivalent
        # or if it is a type that needs to be inserted into the session manually, it will not be added to the list
        parameters = [value for key, value in class_variables.items() if key in orm_class_variables
                      and not isinstance(value, (ObjectDesignatorDescription.Object, Pose))]

        return self.orm_class(*parameters)

    def insert(self, session: Session, **kwargs) -> ORMAction:
        """
        Insert this action into the database.

        Needs to be overwritten by an action if the action has attributes that do not exist in the orm class
        equivalent. In that case, the attributes need to be inserted into the session manually.

        :param session: Session with a database that is used to add and commit the objects
        :param kwargs: Possible extra keyword arguments
        :return: The completely instanced ORM action that was inserted into the database
        """

        action = super().insert(session)

        # get all class parameters
        class_variables = {key: value for key, value in vars(self).items()
                           if key in inspect.getfullargspec(self.__init__).args}

        # get all orm class parameters
        orm_class_variables = inspect.getfullargspec(self.orm_class.__init__).args

        # loop through all class parameters and insert them into the session unless they are already added by the ORM
        for key, value in class_variables.items():
            if key not in orm_class_variables:
                variable = value.insert(session)
                if isinstance(variable, ORMObject):
                    action.object = variable
                elif isinstance(variable, ORMPose):
                    action.pose = variable
        session.add(action)

        return action

    def __str__(self):
        # all fields that are not ORM classes
        fields = {}
        for key, value in vars(self).items():
            if key.startswith("orm_"):
                continue
            if isinstance(value, ObjectDesignatorDescription.Object):
                fields[key] = value.name
            elif isinstance(value, Pose):
                fields[key] = value.__str__()
        fields_str = "\n".join([f"{key}: {value}" for key, value in fields.items()])
        return f"{self.__class__.__name__.replace('Performable', '')}:\n{fields_str}"

    def __repr__(self):
        return self.__str__()


@dataclass
class MoveTorsoActionPerformable(ActionAbstract):
    """
    Move the torso of the robot up and down.
    """

    joint_positions: Dict[str, float]
    """
    The joint positions that should be set. The keys are the joint names and the values are the joint positions.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMMoveTorsoAction)

    @with_tree
    def plan(self) -> None:
        joints_positions = list(self.joint_positions.items())
        joints, positions = zip(*joints_positions)
        MoveJointsMotion(list(joints), list(positions)).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: timedelta = timedelta(seconds=2)):
        """
        Create a goal validator for the joint positions and wait until the goal is achieved or the timeout is reached.
        """
        validator = create_multiple_joint_goal_validator(World.current_world.robot, self.joint_positions)
        validator.wait_until_goal_is_achieved(max_wait_time=max_wait_time,
                                              time_per_read=timedelta(milliseconds=20))
        if not validator.goal_achieved:
            raise TorsoGoalNotReached(validator)


@dataclass
class SetGripperActionPerformable(ActionAbstract):
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
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMSetGripperAction)

    @with_tree
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


@dataclass
class ReleaseActionPerformable(ActionAbstract):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """

    gripper: Arms

    object_designator: ObjectDesignatorDescription.Object

    def plan(self) -> None:
        raise NotImplementedError


@dataclass
class GripActionPerformable(ActionAbstract):
    """
    Grip an object with the robot.

    Note: This action can not be used yet.
    """

    gripper: Arms
    object_designator: ObjectDesignatorDescription.Object
    effort: float

    @with_tree
    def plan(self) -> None:
        raise NotImplementedError()


@dataclass
class ParkArmsActionPerformable(ActionAbstract):
    """
    Park the arms of the robot.
    """

    arm: Arms
    """
    Entry from the enum for which arm should be parked
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMParkArmsAction)

    @with_tree
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


@dataclass
class ReachToPickUpActionPerformable(ActionAbstract):
    """
    Let the robot reach a specific pose.
    """

    object_designator: ObjectDesignatorDescription.Object
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

    orm_object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False, repr=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """

    prepose_distance: float
    """
    The distance in meters the gripper should be at before picking up the object
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMReachToPickUpAction)

    def __post_init__(self):
        super(ActionAbstract, self).__post_init__()
        # Store the object's data copy at execution
        self.orm_object_at_execution = self.object_designator.frozen_copy()

    @with_tree
    def plan(self) -> None:
        adjusted_oTm = self.adjust_target_pose_to_grasp_type()

        pre_pose = self.calculate_pre_grasping_pose(adjusted_oTm)

        MoveGripperMotion(motion=GripperState.OPEN, gripper=self.arm).perform()

        self.go_to_pose(pre_pose)

        self.approach_target_pose(adjusted_oTm)

        # Remove the vis axis from the world if it was added
        World.current_world.remove_vis_axis()

    def approach_target_pose(self, pose: Pose, add_vis_axis: bool = True):
        """
        Go to the target pose by moving in a straight line motion.

        :param pose: The pose to go to.
        :param add_vis_axis: If a visual axis should be added to the world.
        """
        self.go_to_pose(pose, MovementType.STRAIGHT_CARTESIAN, add_vis_axis)

    def go_to_pose(self, pose: Pose, movement_type: MovementType = MovementType.CARTESIAN,
                   add_vis_axis: bool = True):
        """
        Go to a specific pose.

        :param pose: The pose to go to.
        :param movement_type: The type of movement that should be performed.
        :param add_vis_axis: If a visual axis should be added to the world.
        """
        pose = self.transform_pose(pose, Frame.Map.value)
        if add_vis_axis:
            World.current_world.add_vis_axis(pose)
        MoveTCPMotion(pose, self.arm, allow_gripper_collision=False, movement_type=movement_type).perform()

    def adjust_target_pose_to_grasp_type(self) -> Pose:
        """
        Adjust the target pose according to the grasp type by adjusting the orientation of the gripper.

        :return: The adjusted target pose.
        """
        # Get grasp orientation and target pose
        grasp = RobotDescription.current_robot_description.get_arm_chain(self.arm).end_effector.grasps[
            self.grasp_description]
        oTm = self.world_object.get_pose()
        # Transform the object pose to the object frame, basically the origin of the object frame
        mTo = self.local_transformer.transform_to_object_frame(oTm, self.world_object)
        # Adjust the pose according to the special knowledge of the object designator_description
        adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(grasp, mTo)
        # Transform the adjusted pose to the map frame
        adjusted_oTm = self.transform_pose(adjusted_pose, Frame.Map.value)
        # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper
        adjusted_oTm.rotate_by_quaternion(grasp)
        return adjusted_oTm

    def calculate_pre_grasping_pose(self, obj_pose: Pose) -> Pose:
        """
        Calculate the pre grasping pose of the object depending on the gripper and the pre-pose distance.

        :return: The pre grasping pose of the object.
        """
        # pre-pose depending on the gripper.
        oTg = obj_pose.copy()
        oTg.pose.position.x -= self.prepose_distance  # in x since this is how the gripper is oriented
        return self.transform_pose(oTg, Frame.Map.value)

    def transform_to_gripper_frame(self, pose: Pose) -> Pose:
        """
        Transform a pose to the gripper frame.

        :param pose: The pose to transform.
        :return: The transformed pose.
        """
        return self.transform_pose(pose, self.gripper_frame)

    def transform_pose(self, pose: Pose, frame: str) -> Pose:
        """
        Transform a pose to a different frame.

        :param pose: The pose to transform.
        :param frame: The frame to transform the pose to.
        :return: The transformed pose.
        """
        return self.local_transformer.transform_pose(pose, frame)

    @cached_property
    def local_transformer(self) -> LocalTransformer:
        return self.world_object.local_transformer

    @cached_property
    def world_object(self) -> Object:
        return self.object_designator.world_object

    @cached_property
    def gripper_frame(self) -> str:
        return World.robot.get_link_tf_frame(self.arm_chain.get_tool_frame())

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    # TODO find a way to use orm_object_at_execution instead of object_designator in the automatic orm mapping in
    #  ActionAbstract
    def to_sql(self) -> ORMAction:
        return ORMReachToPickUpAction(arm=self.arm, prepose_distance=self.prepose_distance)

    def insert(self, session: Session, **kwargs) -> ORMAction:
        action = super(ActionAbstract, self).insert(session)
        action.object = self.orm_object_at_execution.insert(session)
        session.add(action)
        return action

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if object is contained in the gripper such that it can be grasped and picked up.
        """
        fingers_link_names = self.arm_chain.end_effector.fingers_link_names
        if fingers_link_names:
            if not is_body_between_fingers(self.world_object, fingers_link_names,
                                           method=FindBodyInRegionMethod.MultiRay):
                raise ObjectNotInGraspingArea(self.world_object, World.robot, self.arm, self.grasp_description)
        else:
            logwarn(f"Cannot validate reaching to pick up action for arm {self.arm} as no finger links are defined.")


@dataclass
class PickUpActionPerformable(ActionAbstract):
    """
    Let the robot pick up an object.
    """

    object_designator: ObjectDesignatorDescription.Object
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

    orm_object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False, repr=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """

    prepose_distance: float
    """
    The distance in meters the gripper should be at before picking up the object
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPickUpAction)

    def __post_init__(self):
        super(ActionAbstract, self).__post_init__()
        # Store the object's data copy at execution
        self.orm_object_at_execution = self.object_designator.frozen_copy()

    @with_tree
    def plan(self) -> None:
        ReachToPickUpActionPerformable(self.object_designator, self.arm, self.grasp_description,
                                       self.prepose_distance).perform()

        MoveGripperMotion(motion=GripperState.CLOSE, gripper=self.arm).perform()

        tool_frame = RobotDescription.current_robot_description.get_arm_chain(self.arm).get_tool_frame()
        World.robot.attach(self.world_object, tool_frame)

        self.lift_object(distance=0.1)

        # Remove the vis axis from the world
        World.current_world.remove_vis_axis()

    def lift_object(self, distance: float = 0.1):
        lift_to_pose = self.gripper_pose()
        lift_to_pose.pose.position.z += distance
        MoveTCPMotion(lift_to_pose, self.arm, allow_gripper_collision=True).perform()

    def gripper_pose(self) -> Pose:
        """
        Get the pose of the gripper.

        :return: The pose of the gripper.
        """
        gripper_link = self.arm_chain.get_tool_frame()
        return World.robot.links[gripper_link].pose

    # TODO find a way to use orm_object_at_execution instead of object_designator in the automatic orm mapping in
    #  ActionAbstract
    def to_sql(self) -> ORMAction:
        return ORMPickUpAction(arm=self.arm, prepose_distance=self.prepose_distance)

    def insert(self, session: Session, **kwargs) -> ORMAction:
        action = super(ActionAbstract, self).insert(session)
        action.object = self.orm_object_at_execution.insert(session)
        session.add(action)
        return action

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if picked up object is in contact with the gripper.
        """
        if not has_gripper_grasped_body(self.arm, self.world_object):
            raise ObjectNotGraspedError(self.world_object, World.robot, self.arm, self.grasp_description)

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    @cached_property
    def world_object(self) -> Object:
        return self.object_designator.world_object


@dataclass
class PlaceActionPerformable(ActionAbstract):
    """
    Places an Object at a position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be place
    """
    arm: Arms
    """
    Arm that is currently holding the object
    """
    target_location: Pose
    """
    Pose in the world at which the object should be placed
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPlaceAction)

    @with_tree
    def plan(self) -> None:
        target_pose = self.object_designator.world_object.attachments[
            World.robot].get_child_link_target_pose_given_parent(self.target_location)
        MoveTCPMotion(target_pose, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm).perform()
        World.robot.detach(self.object_designator.world_object)

        retract_pose = self.calculate_retract_pose(target_pose, distance=0.05)
        MoveTCPMotion(retract_pose, self.arm).perform()

    def calculate_target_pose_of_gripper(self):
        """
        Calculate the target pose of the gripper given the target pose of the held object.
        wTg (world to gripper) = wTo (world to object target) * oTg (object to gripper, this is constant since object
        is attached to the gripper)
        """
        gripper_pose_in_object = self.local_transformer.transform_pose(self.gripper_pose, self.world_object.tf_frame)
        object_to_gripper = gripper_pose_in_object.to_transform("object")
        world_to_object_target = self.target_location.to_transform("target")
        world_to_gripper_target = world_to_object_target * object_to_gripper
        return world_to_gripper_target.to_pose()

    def calculate_retract_pose(self, target_pose: Pose, distance: float) -> Pose:
        """
        Calculate the retract pose of the gripper.

        :param target_pose: The target pose of the gripper.
        :param distance: The distance the gripper should be retracted.
        """
        retract_pose = self.local_transformer.transform_pose(target_pose, self.gripper_tool_frame)
        retract_pose.position.x -= distance
        return retract_pose

    @cached_property
    def world_object(self) -> Object:
        return self.object_designator.world_object

    @property
    def gripper_pose(self) -> Pose:
        """
        Get the pose of the gripper.

        :return: The pose of the gripper.
        """
        return self.gripper_link.pose

    @cached_property
    def gripper_tool_frame(self) -> str:
        return self.gripper_link.tf_frame

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
        contact_links = self.world_object.get_contact_points_with_body(World.robot).get_all_bodies()
        if contact_links:
            raise ObjectStillInContact(self.world_object, contact_links,
                                       self.target_location, World.robot, self.arm)

    def validate_placement_location(self):
        """
        Check if the object is placed at the target location.
        """
        pose_error_checker = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_error_checker.is_error_acceptable(self.world_object.pose, self.target_location):
            raise ObjectNotPlacedAtTargetLocation(self.world_object, self.target_location, World.robot, self.arm)


@dataclass
class NavigateActionPerformable(ActionAbstract):
    """
    Navigates the Robot to a position.
    """

    target_location: Pose
    """
    Location to which the robot should be navigated
    """

    keep_joint_states: bool
    """
    Keep the joint states of the robot the same during the navigation.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMNavigateAction)

    @with_tree
    def plan(self) -> None:
        motion_action = MoveMotion(self.target_location, self.keep_joint_states)
        return try_action(motion_action, failure_type=NavigationGoalNotReachedError)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        pose_validator = PoseErrorChecker(World.conf.get_pose_tolerance())
        if not pose_validator.is_error_acceptable(World.robot.pose, self.target_location):
            raise NavigationGoalNotReachedError(World.robot, self.target_location)


@dataclass
class TransportActionPerformable(ActionAbstract):
    """
    Transports an object to a position using an arm
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be transported.
    """
    target_location: Pose
    """
    Target Location to which the object should be transported
    """
    arm: Arms
    """
    Arm that should be used
    """
    pickup_prepose_distance: float
    """
    Distance between the object and the gripper in the x-axis before picking up the object.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMTransportAction)

    @with_tree
    def plan(self) -> None:
        robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=self.object_designator,
                                     reachable_for=robot_desig_resolved,
                                     reachable_arms=[self.arm],
                                     prepose_distance=self.pickup_prepose_distance)
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = None
        for pose in pickup_loc:
            if self.arm == pose.reachable_arm:
                pickup_pose = pose
                break
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateActionPerformable(pickup_pose.pose, True).perform()
        PickUpActionPerformable(self.object_designator, pickup_pose.reachable_arm,
                                pickup_pose.grasp_description,
                                prepose_distance=self.pickup_prepose_distance).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(
                target=self.target_location,
                reachable_for=robot_desig_resolved,
                reachable_arms=[pickup_pose.reachable_arm],
                grasp_descriptions=[pickup_pose.grasp_description],
                object_in_hand=self.object_designator
            ).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                f"No location found from where the robot can reach the target location: {self.target_location}")
        NavigateActionPerformable(place_loc.pose, True).perform()
        PlaceActionPerformable(self.object_designator, place_loc.reachable_arm, self.target_location).perform()
        ParkArmsActionPerformable(Arms.BOTH).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation of each atomic action is done in the action itself, so no more validation needed here.
        pass


@dataclass
class LookAtActionPerformable(ActionAbstract):
    """
    Lets the robot look at a position.
    """

    target: Pose
    """
    Position at which the robot should look, given as 6D pose
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMLookAtAction)

    @with_tree
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
            gen_obj = generate_object_at_target(self.target.position_as_list(), size=(0.4, 0.4, 0.4))
            ray_result = cast_a_ray_from_camera()
            gen_obj.remove()
            if not ray_result.intersected or ray_result.obj_id != gen_obj.id:
                raise LookAtGoalNotReached(World.robot, self.target)


@dataclass
class DetectActionPerformable(ActionAbstract):
    """
    Detects an object that fits the object description and returns an object designator_description describing the object.

    If no object is found, an PerceptionObjectNotFound error is raised.
    """

    technique: DetectionTechnique
    """
    The technique that should be used for detection
    """
    state: DetectionState
    """
    The state of the detection, e.g Start Stop for continues perception
    """
    object_designator_description: Optional[ObjectDesignatorDescription] = None
    """
    The type of the object that should be detected, only considered if technique is equal to Type
    """
    region: Optional[Location] = None
    """
    The region in which the object should be detected
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMDetectAction)

    orm_object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)

    @with_tree
    def plan(self) -> None:
        return try_action(DetectingMotion(technique=self.technique, state=self.state,
                                          object_designator_description=self.object_designator_description,
                                          region=self.region), PerceptionObjectNotFound)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        if not result:
            raise PerceptionObjectNotFound(self.object_designator_description, self.technique, self.region)


@dataclass
class OpenActionPerformable(ActionAbstract):
    """
    Opens a container like object
    """

    object_designator: ObjectPart.Object
    """
    Object designator_description describing the object that should be opened
    """
    arm: Arms
    """
    Arm that should be used for opening the container
    """
    grasping_prepose_distance: float
    """
    The distance in meters the gripper should be at in the x-axis away from the handle.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMOpenAction)

    @with_tree
    def plan(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator, self.grasping_prepose_distance).perform()
        OpeningMotion(self.object_designator, self.arm).perform()

        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the container is opened, this assumes that the container state can be read accurately from the
        real world.
        """
        validate_close_open(self.object_designator, self.arm, OpenActionPerformable)


@dataclass
class CloseActionPerformable(ActionAbstract):
    """
    Closes a container like object.
    """

    object_designator: ObjectPart.Object
    """
    Object designator_description describing the object that should be closed
    """
    arm: Arms
    """
    Arm that should be used for closing
    """
    grasping_prepose_distance: float
    """
    The distance in meters between the gripper and the handle before approaching to grasp.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMCloseAction)

    @with_tree
    def plan(self) -> None:
        GraspingActionPerformable(self.arm, self.object_designator, self.grasping_prepose_distance).perform()
        ClosingMotion(self.object_designator, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the container is closed, this assumes that the container state can be read accurately from the
        real world.
        """
        validate_close_open(self.object_designator, self.arm, CloseActionPerformable)


def validate_close_open(object_designator: ObjectDesignatorDescription.Object, arm: Arms,
                        action_type: Union[OpenActionPerformable, CloseActionPerformable]):
    """
    Validates if the container is opened or closed by checking the joint position of the container.

    :param object_designator: The object designator_description describing the object that should be opened or closed.
    :param arm: The arm that should be used for opening or closing the container.
    :param action_type: The type of the action that should be validated.
    """
    obj_part = object_designator.world_object
    container_joint_name = obj_part.find_joint_above_link(object_designator.name)
    lower_limit, upper_limit = obj_part.get_joint_limits(container_joint_name)
    joint_obj: Joint = obj_part.joints[container_joint_name]
    if issubclass(action_type, CloseActionPerformable):
        check_closed(joint_obj, obj_part, arm, lower_limit)
    elif issubclass(action_type, OpenActionPerformable):
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
class GraspingActionPerformable(ActionAbstract):
    """
    Grasps an object described by the given Object Designator description
    """
    arm: Arms
    """
    The arm that should be used to grasp
    """
    object_desig: ObjectDesignatorDescription.Object
    """
    Object Designator for the object that should be grasped
    """
    prepose_distance: float
    """
    The distance in meters the gripper should be at before grasping the object
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMGraspingAction)

    @with_tree
    def plan(self) -> None:
        if isinstance(self.object_desig, ObjectPart.Object):
            object_pose = self.object_desig.part_pose
        else:
            object_pose = self.object_desig.world_object.get_pose()
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
        if isinstance(self.object_desig, ObjectPart.Object):
            body = self.object_desig.world_object.links[self.object_desig.name]
        else:
            body = self.object_desig.world_object
        contact_links = body.get_contact_points_with_body(World.robot).get_all_bodies()
        arm_chain = RobotDescription.current_robot_description.get_arm_chain(self.arm)
        gripper_links = arm_chain.end_effector.links
        if not any([link.name in gripper_links for link in contact_links]):
            raise ObjectNotGraspedError(self.object_desig.world_object, World.robot, self.arm, None)


@dataclass
class FaceAtPerformable(ActionAbstract):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose: Pose
    """
    The pose to face 
    """
    keep_joint_states: bool
    """
    Keep the joint states of the robot the same during the navigation.
    """

    orm_class = ORMFaceAtAction

    @with_tree
    def plan(self) -> None:
        # get the robot position
        robot_position = World.robot.pose

        # calculate orientation for robot to face the object
        angle = np.arctan2(robot_position.position.y - self.pose.position.y,
                           robot_position.position.x - self.pose.position.x) + np.pi
        orientation = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))

        # create new robot pose
        new_robot_pose = Pose(robot_position.position_as_list(), orientation)

        # turn robot
        NavigateActionPerformable(new_robot_pose, self.keep_joint_states).perform()

        # look at target
        LookAtActionPerformable(self.pose).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in the LookAtActionPerformable.perform() method so no need to validate here.
        pass


@dataclass
class MoveAndPickUpPerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
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

    keep_joint_states: bool
    """
    Keep the joint states of the robot the same during the navigation.
    """

    pick_up_prepose_distance: float
    """
    The distance in meters the gripper should be at before picking up the object
    """

    # @with_tree
    def plan(self):
        if self.grasp == Grasp.TOP:
            grasp = GraspDescription(Grasp.FRONT, self.grasp, False)
        else:
            grasp = GraspDescription(self.grasp, None, False)
        NavigateActionPerformable(self.standing_position, self.keep_joint_states).perform()
        FaceAtPerformable(self.object_designator.pose, self.keep_joint_states).perform()
        PickUpActionPerformable(self.object_designator, self.arm, grasp,
                                self.pick_up_prepose_distance).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
        pass


@dataclass
class MoveAndPlacePerformable(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position: Pose
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    target_location: Pose
    """
    The location to place the object.
    """

    arm: Arms
    """
    The arm to use
    """

    keep_joint_states: bool
    """
    Keep the joint states of the robot the same during the navigation.
    """

    @with_tree
    def plan(self):
        NavigateActionPerformable(self.standing_position, self.keep_joint_states).perform()
        FaceAtPerformable(self.target_location, self.keep_joint_states).perform()
        PlaceActionPerformable(self.object_designator, self.arm, self.target_location).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
        pass


@dataclass
class PouringPerformable(ActionAbstract):
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

    @with_tree
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
# ----------------------------------------------------------------------------
#               Action Designators Description
# ----------------------------------------------------------------------------


class MoveTorsoAction(ActionDesignatorDescription):
    """
    Action Designator for Moving the torso of the robot up and down
    """
    performable_class = MoveTorsoActionPerformable

    def __init__(self, torso_states: List[TorsoState]):
        """
        Create a designator_description description to move the torso of the robot up and down.

        :param torso_states: A list of possible states for the torso. The states are defined in the robot description.
        """
        super().__init__()
        self.torso_states: List[TorsoState] = torso_states

    def ground(self) -> MoveTorsoActionPerformable:
        """
        Creates a performable action designator_description with the first element from the list of possible torso heights.

        :return: A performable action designator_description
        """
        joint_positions: dict = RobotDescription.current_robot_description.get_static_joint_chain("torso",
                                                                                                  self.torso_states[0])
        return MoveTorsoActionPerformable(joint_positions)

    def __iter__(self):
        """
        Iterates over all possible values for this designator_description and returns a performable action designator_description with the value.

        :return: A performable action designator_description
        """
        for torso_state in self.torso_states:
            joint_positions: dict = RobotDescription.current_robot_description.get_static_joint_chain("torso",
                                                                                                      torso_state)
            yield MoveTorsoActionPerformable(joint_positions)


class SetGripperAction(ActionDesignatorDescription):
    """
    Set the gripper state of the robot
    """

    performable_class = SetGripperActionPerformable

    def __init__(self, grippers: List[Arms], motions: List[GripperState]):
        """
        Sets the gripper state, the desired state is given with the motion. Motion can either be 'open' or 'close'.

        :param grippers: A list of possible grippers
        :param motions: A list of possible motions
        """
        super().__init__()
        self.grippers: List[Arms] = grippers
        self.motions: List[GripperState] = motions

    def ground(self) -> SetGripperActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first element in the grippers and motions list.

        :return: A performable designator_description
        """
        return SetGripperActionPerformable(self.grippers[0], self.motions[0])

    def __iter__(self):
        """
        Iterates over all possible combinations of grippers and motions

        :return: A performable designator_description with a combination of gripper and motion
        """
        for parameter_combination in itertools.product(self.grippers, self.motions):
            yield SetGripperActionPerformable(*parameter_combination)


class ReleaseAction(ActionDesignatorDescription):
    """
    Releases an Object from the robot.

    Note: This action can not be used yet.
    """

    performable_class = ReleaseActionPerformable

    def __init__(self, object_designator_description: ObjectDesignatorDescription, grippers: List[Arms] = None):
        super().__init__()
        self.grippers: List[Arms] = grippers
        self.object_designator_description = object_designator_description

    def ground(self) -> ReleaseActionPerformable:
        return ReleaseActionPerformable(self.grippers[0], self.object_designator_description.ground())

    def __iter__(self):
        ri = ReasoningInstance(self,
                               PartialDesignator(ReleaseActionPerformable, self.grippers,
                                                 self.object_designator_description))
        for desig in ri:
            yield desig


class GripAction(ActionDesignatorDescription):
    """
    Grip an object with the robot.

    :ivar grippers: The grippers to consider
    :ivar object_designator_description: The description of objects to consider
    :ivar efforts: The efforts to consider

    Note: This action can not be used yet.
    """

    performable_class = GripActionPerformable

    def __init__(self, object_designator_description: ObjectDesignatorDescription, grippers: List[Arms] = None,
                 efforts: List[float] = None):
        super().__init__()
        self.grippers: List[Arms] = grippers
        self.object_designator_description: ObjectDesignatorDescription = object_designator_description
        self.efforts: List[float] = efforts

    def ground(self) -> GripActionPerformable:
        return GripActionPerformable(self.grippers[0], self.object_designator_description.ground(), self.efforts[0])

    def __iter__(self):
        ri = ReasoningInstance(self,
                               PartialDesignator(GripActionPerformable, self.grippers,
                                                 self.object_designator_description,
                                                 self.efforts))
        for desig in ri:
            yield desig


class ParkArmsAction(ActionDesignatorDescription):
    """
    Park the arms of the robot.
    """

    performable_class = ParkArmsActionPerformable

    def __init__(self, arms: List[Arms]):
        """
        Moves the arms in the pre-defined parking position. Arms are taken from pycram.enum.Arms

        :param arms: A list of possible arms, that could be used
        """
        super().__init__()
        self.arms: List[Arms] = arms

    def ground(self) -> ParkArmsActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first element of the list of possible arms

        :return: A performable designator_description
        """
        return ParkArmsActionPerformable(self.arms[0])

    def __iter__(self) -> ParkArmsActionPerformable:
        """
        Iterates over all possible solutions and returns a performable designator with the arm.

        :return: A performable designator_description
        """
        for arm in self.arms:
            yield ParkArmsActionPerformable(arm)


class PickUpAction(ActionDesignatorDescription):
    """
    Designator to let the robot pick up an object.
    """

    performable_class = PickUpActionPerformable

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 arms: List[Arms] = None, grasp_descriptions: List[GraspDescription] = None, prepose_distance: float = 0.03):
        """
        Lets the robot pick up an object. The description needs an object designator_description describing the object that should be
        picked up, an arm that should be used as well as the grasp from which side the object should be picked up.

        :param object_designator_description: List of possible object designator_description
        :param arms: List of possible arms that could be used
        :param grasps: List of possible grasps for the object
        :param prepose_distance: The distance between the object and the gripper in the x-axis before picking up the
        object.
        """
        super().__init__()
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.grasp_descriptions: List[GraspDescription] = grasp_descriptions
        object_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                        ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()
        self.prepose_distance: float = prepose_distance
        self.knowledge_condition = GraspableProperty(self.object_designator_description) & ReachableProperty(
            object_desig.pose)

    def __iter__(self) -> PickUpActionPerformable:
        grasp = self.grasp_descriptions[0] if self.grasp_descriptions else None
        object_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                        ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        yield PickUpActionPerformable(object_desig, self.arms[0], grasp, self.prepose_distance)


class PlaceAction(ActionDesignatorDescription):
    """
    Places an Object at a position using an arm.
    """

    performable_class = PlaceActionPerformable

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose],
                 arms: List[Arms] = None):
        """
        Create an Action Description to place an object

        :param object_designator_description: Description of object to place.
        :param target_locations: List of possible positions/orientations to place the object
        :param arms: List of possible arms to use
        """
        super().__init__()
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        object_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                        ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()
        self.target_locations: List[Pose] = target_locations
        self.arms: List[Arms] = arms
        self.knowledge_condition = ReachableProperty(object_desig.pose)

    def ground(self) -> PlaceActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entries from the list of possible entries.

        :return: A performable designator_description
        """
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()

        return PlaceActionPerformable(obj_desig, self.arms[0], self.target_locations[0])

    def __iter__(self) -> PlaceActionPerformable:
        obj_desig = self.object_designator_description if isinstance(self.object_designator_description,
                                                                     ObjectDesignatorDescription.Object) else self.object_designator_description.resolve()
        yield PlaceActionPerformable(obj_desig, self.arms[0], self.target_locations[0])


class NavigateAction(ActionDesignatorDescription):
    """
    Navigates the Robot to a position.
    """

    performable_class = NavigateActionPerformable

    def __init__(self, target_locations: List[Pose], keep_joint_states: bool = False):
        """
        Navigates the robot to a location.

        :param target_locations: A list of possible target locations for the navigation.
        :param keep_joint_states: If the joint states should be kept the same during the navigation.
        """
        super().__init__()
        self.target_locations: List[Pose] = target_locations
        if len(self.target_locations) == 1:
            self.knowledge_condition = SpaceIsFreeProperty(self.target_locations[0])
        else:
            root = SpaceIsFreeProperty(self.target_locations[0])
            for location in self.target_locations[1:]:
                root |= SpaceIsFreeProperty(location)
            self.knowledge_condition = root
        self.keep_joint_states: bool = keep_joint_states

    def ground(self) -> NavigateActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entry of possible target locations.

        :return: A performable designator_description
        """
        return NavigateActionPerformable(self.target_locations[0], self.keep_joint_states)

    def __iter__(self) -> NavigateActionPerformable:
        """
        Iterates over all possible target locations

        :return: A performable designator_description
        """
        for location in self.target_locations:
            yield NavigateActionPerformable(location, self.keep_joint_states)


class TransportAction(ActionDesignatorDescription):
    """
    Transports an object to a position using an arm
    """

    performable_class = TransportActionPerformable

    def __init__(self,
                 object_designator_description: Union[ObjectDesignatorDescription, ObjectDesignatorDescription.Object],
                 target_locations: List[Pose],
                 arms: List[Arms] = None,
                 pickup_prepose_distance: float = 0.03):
        """
        Designator representing a pick and place plan.

        :param object_designator_description: Object designator_description description or a specified Object designator_description that should be transported
        :param target_locations: A list of possible target locations for the object to be placed
        :param arms: A List of possible arms that could be used for transporting
        :param pickup_prepose_distance: The distance between the object and the gripper in the x-axis before
         picking up the object.
        """
        super().__init__()
        self.object_designator_description: Union[
            ObjectDesignatorDescription, ObjectDesignatorDescription.Object] = object_designator_description
        self.arms: List[Arms] = arms
        self.target_locations: List[Pose] = target_locations
        self.pickup_prepose_distance: float = pickup_prepose_distance

    def ground(self) -> TransportActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entries from the lists of possible parameter.

        :return: A performable designator_description
        """
        obj_desig = self.object_designator_description \
            if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object) \
            else self.object_designator_description.resolve()

        return TransportActionPerformable(obj_desig, self.target_locations[0], self.arms[0],
                                          self.pickup_prepose_distance)

    def __iter__(self) -> TransportActionPerformable:
        obj_desig = self.object_designator_description \
            if isinstance(self.object_designator_description, ObjectDesignatorDescription.Object) \
            else self.object_designator_description.resolve()
        ri = ReasoningInstance(self,
                               PartialDesignator(TransportActionPerformable, obj_desig, self.target_locations,
                                                 self.arms, self.pickup_prepose_distance))
        for desig in ri:
            yield desig


class LookAtAction(ActionDesignatorDescription):
    """
    Lets the robot look at a position.
    """

    performable_class = LookAtActionPerformable

    def __init__(self, targets: List[Pose]):
        """
        Moves the head of the robot such that it points towards the given target location.

        :param targets: A list of possible locations to look at
        """
        super().__init__()
        self.targets: List[Pose] = targets

    def ground(self) -> LookAtActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the first entry in the list of possible targets

        :return: A performable designator_description
        """
        return LookAtActionPerformable(self.targets[0])

    def __iter__(self) -> LookAtActionPerformable:
        """
        Iterates over all possible target locations

        :return: A performable designator_description
        """
        for target in self.targets:
            yield LookAtActionPerformable(target)


class DetectAction(ActionDesignatorDescription):
    """
    Detects an object that fits the object description and returns an object designator_description describing the object.
    """

    performable_class = DetectActionPerformable

    def __init__(self, technique: DetectionTechnique, state: Optional[DetectionState] = None,
                 object_designator_description: Optional[ObjectDesignatorDescription] = None,
                 region: Optional[Location] = None):
        """
        Tries to detect an object in the field of view (FOV) of the robot.

      """
        super().__init__()
        self.technique: DetectionTechnique = technique
        self.state: DetectionState = DetectionState.START if state is None else state
        self.object_designator_description: Optional[ObjectDesignatorDescription] = object_designator_description
        self.region: Optional[Location] = region
        # TODO: Implement knowledge condition
        # self.knowledge_condition = VisibleProperty(self.object_designator_description)

    def ground(self) -> DetectActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the executed object description.

        :return: A performable designator_description
        """
        return DetectActionPerformable(self.technique, self.state, self.object_designator_description, self.region)

    def __iter__(self) -> DetectActionPerformable:
        """
        Iterates over all possible values for this designator_description and returns a performable action designator_description with the value.

        :return: A performable action designator_description
        """
        yield DetectActionPerformable(self.technique, self.state, self.object_designator_description, self.region)


class OpenAction(ActionDesignatorDescription):
    """
    Opens a container like object

    Can currently not be used
    """

    performable_class = OpenActionPerformable

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms] = None,
                 grasping_prepose_distance: float = 0.03):
        """
        Moves the arm of the robot to open a container.

        :param object_designator_description: Object designator_description describing the handle that should be used to open
        :param arms: A list of possible arms that should be used
        :param grasping_prepose_distance: The distance in meters between gripper and handle before approaching to grasp.
        """
        super().__init__()
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms
        self.grasping_prepose_distance: float = grasping_prepose_distance
        self.knowledge_condition = GripperIsFreeProperty(self.arms)

    def ground(self) -> OpenActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the executed object description and the first entries
        from the lists of possible parameter.

        :return: A performable designator_description
        """
        return OpenActionPerformable(self.object_designator_description.resolve(), self.arms[0],
                                     grasping_prepose_distance=self.grasping_prepose_distance)

    def __iter__(self) -> OpenActionPerformable:
        """
        Iterates over all possible values for this designator_description and returns a performable action designator_description with the value.

        :return: A performable action designator_description
        """
        ri = ReasoningInstance(self,
                               PartialDesignator(OpenActionPerformable, self.object_designator_description, self.arms,
                                                 self.grasping_prepose_distance))
        for desig in ri:
            yield desig


class CloseAction(ActionDesignatorDescription):
    """
    Closes a container like object.

    Can currently not be used
    """

    performable_class = CloseActionPerformable

    def __init__(self, object_designator_description: ObjectPart, arms: List[Arms] = None,
                 grasping_prepose_distance: float = 0.03):
        """
        Attempts to close an open container

        :param object_designator_description: Object designator_description description of the handle that should be used
        :param arms: A list of possible arms to use
        :param grasping_prepose_distance: The distance in meters between the gripper and the handle before approaching
        to grasp.
        """
        super().__init__()
        self.object_designator_description: ObjectPart = object_designator_description
        self.arms: List[Arms] = arms
        self.grasping_prepose_distance: float = grasping_prepose_distance
        self.knowledge_condition = GripperIsFreeProperty(self.arms)

    def ground(self) -> CloseActionPerformable:
        """
        Default specialized_designators that returns a performable designator_description with the executed object designator_description and the first entry from
        the list of possible arms.

        :return: A performable designator_description
        """
        return CloseActionPerformable(self.object_designator_description.resolve(), self.arms[0],
                                      self.grasping_prepose_distance)

    def __iter__(self) -> CloseActionPerformable:
        """
        Iterates over all possible solutions for this designator_description and returns a performable action designator.

        :yield: A performable fully parametrized Action designator
        """
        ri = ReasoningInstance(self,
                               PartialDesignator(CloseActionPerformable, self.object_designator_description, self.arms,
                                                 self.grasping_prepose_distance))
        for desig in ri:
            yield desig


class GraspingAction(ActionDesignatorDescription):
    """
    Grasps an object described by the given Object Designator description
    """

    performable_class = GraspingActionPerformable

    def __init__(self, object_description: Union[ObjectDesignatorDescription, ObjectPart], arms: List[Arms] = None,
                 prepose_distance: float = 0.03):
        """
        Will try to grasp the object described by the given description. Grasping is done by moving into a pre grasp
        position 10 cm before the object, opening the gripper, moving to the object and then closing the gripper.

        :param arms: List of Arms that should be used for grasping
        :param object_description: Description of the object that should be grasped
        :param prepose_distance: The distance in meters between the gripper and the object before approaching to grasp.
        """
        super().__init__()
        self.arms: List[Arms] = arms
        self.object_description: ObjectDesignatorDescription = object_description
        self.prepose_distance: float = prepose_distance

    def ground(self) -> GraspingActionPerformable:
        """
        Default specialized_designators that takes the first element from the list of arms and the first solution for the object
        designator_description description ond returns it.

        :return: A performable action designator_description that contains specific arguments
        """
        return GraspingActionPerformable(self.arms[0], self.object_description.resolve(), self.prepose_distance)

    def __iter__(self) -> CloseActionPerformable:
        """
        Iterates over all possible solutions for this designator_description and returns a performable action
        designator.

        :yield: A fully parametrized Action designator
        """
        ri = ReasoningInstance(self,
                               PartialDesignator(GraspingActionPerformable, self.object_description, self.arms,
                                                 self.prepose_distance))
        for desig in ri:
            yield desig

class PouringAction(ActionDesignatorDescription):
    """
    Designator for pouring liquids from one container to another.
    """

    def __init__(self, object_: ObjectDesignatorDescription, tool: ObjectDesignatorDescription,
                 arms: List[Arms], technique: Optional[str] = None, angle: Optional[float] = 90):
        """
        Will try to move the arm to the object and pour the liquid from one container to another.

        :param object_: The object to be poured
        :param tool: The tool used for pouring
        :param arms: The robot arm designated for the pouring task
        :param technique: The technique used for pouring (default is None)
        :param angle: The angle of the pouring action (default is 90)
        """
        super(PouringAction, self).__init__()
        self.object_: ObjectDesignatorDescription = object_
        self.tool: ObjectDesignatorDescription = tool
        self.arms: List[Arms] = arms
        self.technique: Optional[str] = technique
        self.angle: Optional[float] = angle

    def ground(self) -> PouringPerformable:
        """
        Default resolver, returns a performable designator with the first entries from the lists of possible parameter.

        :return: A performable designator
        """
        return PouringPerformable(self.object_, self.tool, self.arms[0], self.technique, self.angle)

    def __iter__(self) -> PouringPerformable:
        """
        Iterates over all possible values for this designator_description and returns a performable action designator_description with the value.

        :return: A performable action designator_description
        """
        yield PouringPerformable(self.object_, self.tool, self.arms[0], self.technique, self.angle)
