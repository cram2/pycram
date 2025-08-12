from __future__ import annotations

import datetime
from dataclasses import dataclass
from datetime import timedelta

from typing_extensions import Union, Optional, Type, Dict, Any, Iterable

from ....datastructures.enums import Arms, GripperState, TorsoState, StaticJointState, AxisIdentifier
from ....datastructures.partial_designator import PartialDesignator
from ....datastructures.pose import Vector3Stamped
from ....datastructures.world import World
from ....failures import TorsoGoalNotReached, ConfigurationNotReached
from ....has_parameters import has_parameters
from ....language import SequentialPlan
from ....robot_description import RobotDescription
from ....robot_plans.actions.base import ActionDescription
from ....robot_plans.motions.gripper import MoveGripperMotion
from ....robot_plans.motions.robot_body import MoveJointsMotion
from ....validation.goal_validator import create_multiple_joint_goal_validator


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
        SequentialPlan(self.context,
                       MoveJointsMotion(list(joint_positions.keys()), list(joint_positions.values()))).perform()

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
            SequentialPlan(self.context, MoveGripperMotion(gripper=arm_chains.arm_type, motion=self.motion)).perform()
        else:
            for chain in arm_chains:
                SequentialPlan(self.context, MoveGripperMotion(gripper=chain.arm_type, motion=self.motion)).perform()

    def validate(self, result: Optional[Any] = None, max_wait_time: timedelta = timedelta(seconds=2)):
        """
        Needs gripper state to be read or perceived.
        """
        pass

    @classmethod
    def description(cls, gripper: Union[Iterable[Arms], Arms],
                    motion: Union[Iterable[GripperState], GripperState] = None) -> PartialDesignator[
        Type[SetGripperAction]]:
        return PartialDesignator(SetGripperAction, gripper=gripper, motion=motion)


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

        SequentialPlan(self.context,
                       MoveJointsMotion(names=list(joint_poses.keys()), positions=list(joint_poses.values()))).perform()

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

        SequentialPlan(self.context,
                       MoveJointsMotion(names=list(joint_poses.keys()), positions=list(joint_poses.values()),
                                        align=self.align, tip_link=self.tip_link, tip_normal=tip_normal,
                                        root_link=self.root_link, root_normal=root_normal)).perform()

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
    def description(cls, arm: Union[Iterable[Arms], Arms], align: Optional[bool] = False,
                    tip_link: Optional[str] = None, tip_axis: Optional[AxisIdentifier] = None,
                    root_link: Optional[str] = None, root_axis: Optional[AxisIdentifier] = None) \
            -> PartialDesignator[Type[CarryAction]]:
        return PartialDesignator(cls, arm=arm, align=align, tip_link=tip_link, tip_axis=tip_axis,
                                 root_link=root_link, root_axis=root_axis)


MoveTorsoActionDescription = MoveTorsoAction.description
SetGripperActionDescription = SetGripperAction.description
ParkArmsActionDescription = ParkArmsAction.description
CarryActionDescription = CarryAction.description
