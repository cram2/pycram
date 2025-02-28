# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import abc
import inspect
from dataclasses import dataclass, field
from datetime import timedelta
from functools import cached_property

import numpy as np
from sqlalchemy.orm import Session
from ..tf_transformations import quaternion_from_euler
from typing_extensions import List, Union, Optional, Type, Dict, Any

from pycrap.ontologies import Location
from .location_designator import CostmapLocation
from .motion_designator import MoveJointsMotion, MoveGripperMotion, MoveTCPMotion, MoveMotion, \
    LookingMotion, DetectingMotion, OpeningMotion, ClosingMotion
from .object_designator import ObjectDesignatorDescription, BelieveObject, ObjectPart
from ..datastructures.enums import Frame, FindBodyInRegionMethod, ContainerManipulationType
from ..datastructures.world import World, UseProspectionWorld
from ..description import Joint, Link
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
class ActionAbstract(ActionDescription, abc.ABC):
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


@dataclass
class MoveTorsoAction(ActionAbstract):
    """
    Move the torso of the robot up and down.
    """
    torso_state: Union[List[TorsoState], TorsoState]
    """
    The state of the torso that should be set
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMMoveTorsoAction)

    @with_tree
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


@dataclass
class SetGripperAction(ActionAbstract):
    """
    Set the gripper state of the robot.
    """

    gripper:  Union[List[Arms], Arms]
    """
    The gripper that should be set 
    """
    motion:  Union[List[GripperState], GripperState] = None
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
class ReleaseAction(ActionAbstract):
    """
    Releases an Object from the robot.

    Note: This action can not ve used yet.
    """
    object_designator: ObjectDesignatorDescription.Object

    gripper: Arms = None

    def plan(self) -> None:
        raise NotImplementedError


@dataclass
class GripAction(ActionAbstract):
    """
    Grip an object with the robot.

    Note: This action can not be used yet.
    """
    object_designator: ObjectDesignatorDescription.Object
    gripper:  Union[List[Arms], Arms] = None
    effort:  Union[List[float], float] = None

    @with_tree
    def plan(self) -> None:
        raise NotImplementedError()


@dataclass
class ParkArmsAction(ActionAbstract):
    """
    Park the arms of the robot.
    """

    arm:  Union[List[Arms], Arms]
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
class ReachToPickUpAction(ActionAbstract):
    """
    Let the robot reach a specific pose.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be picked up
    """

    arm:  Union[List[Arms], Arms] = None
    """
    The arm that should be used for pick up
    """

    grasp:  Union[List[Grasp], Grasp] = None
    """
    The grasp that should be used. For example, 'left' or 'right'
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False, repr=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """

    prepose_distance:  Union[List[float], float] = ActionConfig.pick_up_prepose_distance
    """
    The distance in meters the gripper should be at before picking up the object
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMReachToPickUpAction)

    def __post_init__(self):
        super(ActionAbstract, self).__post_init__()
        # Store the object's data copy at execution
        self.object_at_execution = self.object_designator.frozen_copy()

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
        grasp = RobotDescription.current_robot_description.grasps[self.grasp]
        # oTm = Object Pose in Frame map
        oTm = self.world_object.get_pose()
        # Transform the object pose to the object frame, basically the origin of the object frame
        mTo = self.local_transformer.transform_to_object_frame(oTm, self.world_object)
        # Adjust the pose according to the special knowledge of the object designator_description
        adjusted_pose = self.object_designator.special_knowledge_adjustment_pose(self.grasp, mTo)
        # Transform the adjusted pose to the map frame
        adjusted_oTm = self.transform_pose(adjusted_pose, Frame.Map.value)
        # multiplying the orientation therefore "rotating" it, to get the correct orientation of the gripper
        adjusted_oTm.multiply_quaternion(grasp)
        return adjusted_oTm

    def calculate_pre_grasping_pose(self, obj_pose: Pose) -> Pose:
        """
        Calculate the pre grasping pose of the object depending on the gripper and the pre-pose distance.

        :return: The pre grasping pose of the object.
        """
        # pre-pose depending on the gripper.
        oTg = self.transform_to_gripper_frame(obj_pose)
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

    # TODO find a way to use object_at_execution instead of object_designator in the automatic orm mapping in
    #  ActionAbstract
    def to_sql(self) -> ORMAction:
        return ORMReachToPickUpAction(arm=self.arm, grasp=self.grasp, prepose_distance=self.prepose_distance)

    def insert(self, session: Session, **kwargs) -> ORMAction:
        action = super(ActionAbstract, self).insert(session)
        action.object = self.object_at_execution.insert(session)
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
                raise ObjectNotInGraspingArea(self.world_object, World.robot, self.arm, self.grasp)
        else:
            logwarn(f"Cannot validate reaching to pick up action for arm {self.arm} as no finger links are defined.")


@dataclass
class PickUpAction(ActionAbstract):
    """
    Let the robot pick up an object.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be picked up
    """

    arm:  Union[List[Arms], Arms] = None
    """
    The arm that should be used for pick up
    """

    grasp:  Union[List[Grasp], Grasp] = None
    """
    The grasp that should be used. For example, 'left' or 'right'
    """

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False, repr=False)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """

    prepose_distance:  Union[List[float], float] = ActionConfig.pick_up_prepose_distance
    """
    The distance in meters the gripper should be at before picking up the object
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPickUpAction)

    def __post_init__(self):
        super(ActionAbstract, self).__post_init__()

        # Store the object's data copy at execution
        @PickUpAction.pre_perform
        def pre_perform(pick_up_action: PickUpAction):
            pick_up_action.object_at_execution = pick_up_action.object_designator.frozen_copy()

    @with_tree
    def plan(self) -> None:
        ReachToPickUpAction(self.object_designator, self.arm, self.grasp, self.prepose_distance).perform()

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

    # TODO find a way to use object_at_execution instead of object_designator in the automatic orm mapping in
    #  ActionAbstract
    def to_sql(self) -> ORMAction:
        return ORMPickUpAction(arm=self.arm, grasp=self.grasp, prepose_distance=self.prepose_distance)

    def insert(self, session: Session, **kwargs) -> ORMAction:
        action = super(ActionAbstract, self).insert(session)
        action.object = self.object_at_execution.insert(session)
        session.add(action)
        return action

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if picked up object is in contact with the gripper.
        """
        if not has_gripper_grasped_body(self.arm, self.world_object):
            raise ObjectNotGraspedError(self.world_object, World.robot, self.arm, self.grasp)

    @cached_property
    def arm_chain(self) -> KinematicChainDescription:
        return RobotDescription.current_robot_description.get_arm_chain(self.arm)

    @cached_property
    def world_object(self) -> Object:
        return self.object_designator.world_object


@dataclass
class PlaceAction(ActionAbstract):
    """
    Places an Object at a position using an arm.
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be place
    """
    target_location:  Union[List[Pose], Pose]
    """
    Pose in the world at which the object should be placed
    """
    arm:  Union[List[Arms], Arms] = None
    """
    Arm that is currently holding the object
    """

    orm_class: Type[ActionAbstract] = field(init=False, default=ORMPlaceAction)

    @with_tree
    def plan(self) -> None:
        target_pose = self.calculate_target_pose_of_gripper()
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
        contact_links = self.world_object.get_contact_points_with_body(World.robot).get_bodies_in_contact()
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
class NavigateAction(ActionAbstract):
    """
    Navigates the Robot to a position.
    """

    target_location:  Union[List[Pose], Pose]
    """
    Location to which the robot should be navigated
    """

    keep_joint_states:  Union[List[bool], bool] = ActionConfig.navigate_keep_joint_states
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
            raise NavigationGoalNotReachedError(World.robot.pose, self.target_location)


@dataclass
class TransportAction(ActionAbstract):
    """
    Transports an object to a position using an arm
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    Object designator_description describing the object that should be transported.
    """
    target_location:  Union[List[Pose], Pose]
    """
    Target Location to which the object should be transported
    """
    arm:  Union[List[Arms], Arms] = None
    """
    Arm that should be used
    """
    pickup_prepose_distance:  Union[List[float], float] = ActionConfig.pick_up_prepose_distance
    """
    Distance between the object and the gripper in the x-axis before picking up the object.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMTransportAction)

    @with_tree
    def plan(self) -> None:
        robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
        ParkArmsAction(Arms.BOTH).perform()
        pickup_loc = CostmapLocation(target=self.object_designator, reachable_for=robot_desig_resolved,
                                     reachable_arm=self.arm, prepose_distance=self.pickup_prepose_distance)
        # Tries to find a pick-up position for the robot that uses the given arm
        pickup_pose = None
        for pose in pickup_loc:
            if self.arm in pose.reachable_arms:
                pickup_pose = pose
                break
        if not pickup_pose:
            raise ObjectUnfetchable(
                f"Found no pose for the robot to grasp the object: {self.object_designator} with arm: {self.arm}")

        NavigateAction(pickup_pose.pose, True).perform()
        PickUpAction(self.object_designator, self.arm, Grasp.FRONT,
                     prepose_distance=self.pickup_prepose_distance).perform()
        ParkArmsAction(Arms.BOTH).perform()
        try:
            place_loc = CostmapLocation(target=self.target_location, reachable_for=robot_desig_resolved,
                                        reachable_arm=self.arm).resolve()
        except StopIteration:
            raise ReachabilityFailure(
                f"No location found from where the robot can reach the target location: {self.target_location}")
        NavigateAction(place_loc.pose, True).perform()
        PlaceAction(self.object_designator, self.target_location, self.arm).perform()
        ParkArmsAction(Arms.BOTH).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation of each atomic action is done in the action itself, so no more validation needed here.
        pass


@dataclass
class LookAtAction(ActionAbstract):
    """
    Lets the robot look at a position.
    """

    target:  Union[List[Pose], Pose]
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
class DetectAction(ActionAbstract):
    """
    Detects an object that fits the object description and returns an object designator_description describing the object.

    If no object is found, an PerceptionObjectNotFound error is raised.
    """

    technique:  Union[List[DetectionTechnique], DetectionTechnique]
    """
    The technique that should be used for detection
    """
    state:  Union[List[DetectionState], DetectionState] = None
    """
    The state of the detection, e.g Start Stop for continues perception
    """
    object_designator_description: Optional[ObjectDesignatorDescription.Object] = None
    """
    The type of the object that should be detected, only considered if technique is equal to Type
    """
    region:  Union[List[Location], Location] = None
    """
    The region in which the object should be detected
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMDetectAction)

    object_at_execution: Optional[ObjectDesignatorDescription.Object] = field(init=False)

    @with_tree
    def plan(self) -> None:
        return try_action(DetectingMotion(technique=self.technique, state=self.state,
                                          object_designator_description=self.object_designator_description,
                                          region=self.region), PerceptionObjectNotFound)

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        if not result:
            raise PerceptionObjectNotFound(self.object_designator_description, self.technique, self.region)


@dataclass
class OpenAction(ActionAbstract):
    """
    Opens a container like object
    """

    object_designator: ObjectPart.Object
    """
    Object designator_description describing the object that should be opened
    """
    arm:  Union[List[Arms], Arms] = None
    """
    Arm that should be used for opening the container
    """
    grasping_prepose_distance:  Union[List[float], float] = ActionConfig.grasping_prepose_distance
    """
    The distance in meters the gripper should be at in the x-axis away from the handle.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMOpenAction)

    @with_tree
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


@dataclass
class CloseAction(ActionAbstract):
    """
    Closes a container like object.
    """

    object_designator: ObjectPart.Object
    """
    Object designator_description describing the object that should be closed
    """
    arm:  Union[List[Arms], Arms] = None
    """
    Arm that should be used for closing
    """
    grasping_prepose_distance:  Union[List[float], float] = ActionConfig.grasping_prepose_distance
    """
    The distance in meters between the gripper and the handle before approaching to grasp.
    """
    orm_class: Type[ActionAbstract] = field(init=False, default=ORMCloseAction)

    @with_tree
    def plan(self) -> None:
        GraspingAction(self.object_designator, self.arm,  self.grasping_prepose_distance).perform()
        ClosingMotion(self.object_designator, self.arm).perform()
        MoveGripperMotion(GripperState.OPEN, self.arm, allow_gripper_collision=True).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        """
        Check if the container is closed, this assumes that the container state can be read accurately from the
        real world.
        """
        validate_close_open(self.object_designator, self.arm, CloseAction)


def validate_close_open(object_designator: ObjectDesignatorDescription.Object, arm: Arms,
                        action_type: Union[Type[OpenAction], Type[CloseAction]]):
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
class GraspingAction(ActionAbstract):
    """
    Grasps an object described by the given Object Designator description
    """
    object_desig: Union[ObjectDesignatorDescription.Object, ObjectPart.Object]
    """
    Object Designator for the object that should be grasped
    """
    arm:  Union[List[Arms], Arms] = None
    """
    The arm that should be used to grasp
    """
    prepose_distance:  Union[List[float], float] = ActionConfig.grasping_prepose_distance
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
        contact_links = body.get_contact_points_with_body(World.robot).get_bodies_in_contact()
        arm_chain = RobotDescription.current_robot_description.get_arm_chain(self.arm)
        gripper_links = arm_chain.end_effector.links
        if not any([link.name in gripper_links for link in contact_links]):
            raise ObjectNotGraspedError(self.object_desig.world_object, World.robot, self.arm, None)


@dataclass
class FaceAtAction(ActionAbstract):
    """
    Turn the robot chassis such that is faces the ``pose`` and after that perform a look at action.
    """

    pose:  Union[List[Pose], Pose]
    """
    The pose to face 
    """
    keep_joint_states:  Union[List[bool], bool] = ActionConfig.face_at_keep_joint_states
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
        NavigateAction(new_robot_pose, self.keep_joint_states).perform()

        # look at target
        LookAtAction(self.pose).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in the LookAtActionPerformable.perform() method so no need to validate here.
        pass


@dataclass
class MoveAndPickUpAction(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position:  Union[List[Pose], Pose]
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    arm:  Union[List[Arms], Arms] = None
    """
    The arm to use
    """

    grasp:  Union[List[Grasp], Grasp] = None
    """
    The grasp to use
    """

    keep_joint_states:  Union[List[bool], bool] = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    pick_up_prepose_distance:  Union[List[float], float] = ActionConfig.pick_up_prepose_distance
    """
    The distance in meters the gripper should be at before picking up the object
    """

    # @with_tree
    def plan(self):
        NavigateAction(self.standing_position, self.keep_joint_states).perform()
        FaceAtAction(self.object_designator.pose, self.keep_joint_states).perform()
        PickUpAction(self.object_designator, self.arm, self.grasp,
                     self.pick_up_prepose_distance).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
        pass


@dataclass
class MoveAndPlaceAction(ActionAbstract):
    """
    Navigate to `standing_position`, then turn towards the object and pick it up.
    """

    standing_position:  Union[List[Pose], Pose]
    """
    The pose to stand before trying to pick up the object
    """

    object_designator: ObjectDesignatorDescription.Object
    """
    The object to pick up
    """

    target_location:  Union[List[Pose], Pose]
    """
    The location to place the object.
    """

    arm:  Union[List[Arms], Arms] = None
    """
    The arm to use
    """

    keep_joint_states:  Union[List[bool], bool] = ActionConfig.navigate_keep_joint_states
    """
    Keep the joint states of the robot the same during the navigation.
    """

    @with_tree
    def plan(self):
        NavigateAction(self.standing_position, self.keep_joint_states).perform()
        FaceAtAction(self.target_location, self.keep_joint_states).perform()
        PlaceAction(self.object_designator,self.target_location, self.arm).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):
        # The validation will be done in each of the atomic action perform methods so no need to validate here.
        pass
