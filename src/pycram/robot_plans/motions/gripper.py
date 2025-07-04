from dataclasses import dataclass
from pycrap.ontologies import PhysicalObject, Location
from pycram.designators.object_designator import ObjectDesignatorDescription, ObjectPart
from pycram.datastructures.enums import MovementType, WaypointsMovementType
from pycram.failure_handling import try_motion
from pycram.failures import PerceptionObjectNotFound, ToolPoseNotReachedError
from pycram.object_descriptors.urdf import LinkDescription, ObjectDescription
from pycram.plan import with_plan
from pycram.process_module import ProcessModuleManager
from pycram.datastructures.enums import ObjectType, Arms, GripperState, ExecutionType, DetectionTechnique, DetectionState

from typing_extensions import Dict, Optional, Type, List
from pycram.datastructures.pose import PoseStamped, Vector3Stamped
from pycram.designator import BaseMotion
from pycram.world_concepts.world_object import Object
from pycram.external_interfaces.robokudo import robokudo_found



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

    def __str__(self):
        return (f"MoveGripperMotion:\n"
                f"Motion: {self.motion}\n"
                f"Gripper: {self.gripper}\n"
                f"AllowGripperCollision: {self.allow_gripper_collision}")

    def __repr__(self):
        return self.__str__()


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

    def __str__(self):
        return (f"MoveTCPMotion:\n"
                f"Target: {self.target}\n"
                f"Arm: {self.arm}\n"
                f"AllowGripperCollision: {self.allow_gripper_collision}\n"
                f"MovementType: {self.movement_type}")

    def __repr__(self):
        return self.__str__()


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

    def __str__(self):
        return (f"MoveTCPWaypointsMotion:\n"
                f"Waypoints: {self.waypoints}\n"
                f"Arm: {self.arm}\n"
                f"AllowGripperCollision: {self.allow_gripper_collision}\n"
                f"MovementType: {self.movement_type}")

    def __repr__(self):
        return self.__str__()