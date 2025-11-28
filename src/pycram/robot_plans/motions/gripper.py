from dataclasses import dataclass
from typing import Optional, Dict, List

from semantic_digital_twin.world_description.world_entity import Body

from .base import BaseMotion
from ...datastructures.enums import (
    Arms,
    GripperState,
    MovementType,
    WaypointsMovementType,
)
from ...datastructures.grasp import GraspDescription
from ...datastructures.pose import PoseStamped
from ...failure_handling import try_motion
from ...failures import ToolPoseNotReachedError
from ...process_module import ProcessModuleManager
from ...robot_description import ViewManager
from ...utils import translate_pose_along_local_axis


@dataclass
class ReachMotion(BaseMotion):
    """ """

    object_designator: Body
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
    movement_type: MovementType = MovementType.CARTESIAN
    """
    The type of movement that should be performed.
    """

    def perform(self):
        end_effector = ViewManager.get_end_effector_view(self.arm, self.robot_view)

        target_pose = GraspDescription.get_grasp_pose(
            self.grasp_description, end_effector, self.object_designator
        )
        target_pose.rotate_by_quaternion(
            GraspDescription.calculate_grasp_orientation(
                self.grasp_description,
                end_effector.front_facing_orientation.to_np()[:3],
            )
        )
        target_pre_pose = translate_pose_along_local_axis(
            target_pose,
            end_effector.front_facing_axis.to_np(),
            -self.object_designator.get_approach_offset(),
        )

        pose = PoseStamped.from_spatial_type(
            self.world.transform(target_pre_pose.to_spatial_type(), self.world.root)
        )

        MoveTCPMotion(
            pose,
            self.arm,
            allow_gripper_collision=False,
            movement_type=self.movement_type,
        ).perform()


# @dataclass
# class MoveArmJointsMotion(BaseMotion):
#     """
#     Moves the joints of each arm into the given position
#     """
#
#     left_arm_poses: Optional[Dict[str, float]] = None
#     """
#     Target positions for the left arm joints
#     """
#     right_arm_poses: Optional[Dict[str, float]] = None
#     """
#     Target positions for the right arm joints
#     """
#
#     def perform(self):
#         pm_manager = ProcessModuleManager().get_manager(self.robot_view)
#         return pm_manager.move_arm_joints().execute(self)


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
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.move_gripper().execute(self)


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
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        try_motion(pm_manager.move_tcp(), self, ToolPoseNotReachedError)


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
    movement_type: WaypointsMovementType = (
        WaypointsMovementType.ENFORCE_ORIENTATION_FINAL_POINT
    )
    """
    The type of movement that should be performed.
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        pm_manager.move_tcp_waypoints().execute(self)
