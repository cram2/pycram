from dataclasses import dataclass
from typing import Optional, Dict, List

from .base import BaseMotion
from ...datastructures.enums import Arms, GripperState, MovementType, WaypointsMovementType
from ...datastructures.pose import PoseStamped
from ...failure_handling import try_motion
from ...failures import ToolPoseNotReachedError
from ...plan import with_plan
from ...process_module import ProcessModuleManager


@with_plan
@dataclass
class MoveArmJointsMotion(BaseMotion):
    """
    Moves the joints of each arm into the given position
    """

    left_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the left arm joints
    """
    right_arm_poses: Optional[Dict[str, float]] = None
    """
    Target positions for the right arm joints
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.move_arm_joints().execute(self)


@with_plan
@dataclass
class MoveGripperMotion(BaseMotion):
    """
    Opens or closes the gripper
    """

    motion: GripperState
    """
    Motion that should be performed, either 'open' or 'close'
    """
    gripper: Arms
    """
    Name of the gripper that should be moved
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper is allowed to collide with something
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.move_gripper().execute(self)


@with_plan
@dataclass
class MoveTCPMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    target: PoseStamped
    """
    Target pose to which the TCP should be moved
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: Optional[MovementType] = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        try_motion(pm_manager.move_tcp(), self, ToolPoseNotReachedError)


@with_plan
@dataclass
class MoveTCPWaypointsMotion(BaseMotion):
    """
    Moves the Tool center point (TCP) of the robot
    """

    waypoints: List[PoseStamped]
    """
    Waypoints the TCP should move along 
    """
    arm: Arms
    """
    Arm with the TCP that should be moved to the target
    """
    allow_gripper_collision: Optional[bool] = None
    """
    If the gripper can collide with something
    """
    movement_type: WaypointsMovementType = WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT
    """
    The type of movement that should be performed.
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        pm_manager.move_tcp_waypoints().execute(self)
