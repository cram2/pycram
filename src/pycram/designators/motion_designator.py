from dataclasses import dataclass
from pycrap.ontologies import PhysicalObject, Location
from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..datastructures.enums import MovementType
from ..failure_handling import try_motion
from ..failures import PerceptionObjectNotFound, ToolPoseNotReachedError
from ..object_descriptors.urdf import LinkDescription, ObjectDescription
from ..plan import with_plan
from ..process_module import ProcessModuleManager
from ..datastructures.enums import ObjectType, Arms, GripperState, ExecutionType, DetectionTechnique, DetectionState

from typing_extensions import Dict, Optional, Type
from ..datastructures.pose import PoseStamped
from ..tasktree import with_tree
from ..designator import BaseMotion
from ..world_concepts.world_object import Object
from ..external_interfaces.robokudo import robokudo_found


@with_plan
@dataclass
class MoveMotion(BaseMotion):
    """
    Moves the robot to a designated location
    """

    target: PoseStamped
    """
    Location to which the robot should be moved
    """

    keep_joint_states: bool = False
    """
    Keep the joint states of the robot during/at the end of the motion
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.navigate().execute(self)

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

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
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
class LookingMotion(BaseMotion):
    """
    Lets the robot look at a point
    """
    target: PoseStamped

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.looking().execute(self)


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

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
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
class DetectingMotion(BaseMotion):
    """
    Tries to detect an object in the FOV of the robot

    returns: ObjectDesignatorDescription.Object or Error: PerceptionObjectNotFound
    """

    technique: DetectionTechnique
    """
    Detection technique that should be used
    """
    state: DetectionState
    """
    State of the detection
    """
    object_designator_description: Optional[Object] = None
    """
    Description of the object that should be detected
    """
    region: Optional[Location] = None
    """
    Region in which the object should be detected
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        obj_dict = pm_manager.detecting().execute(self)
        return obj_dict


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
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_arm_joints().execute(self)

@with_plan
@dataclass
class WorldStateDetectingMotion(BaseMotion):
    """
    Detects an object based on the world state.
    """

    object_type: ObjectType
    """
    Object type that should be detected
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.world_state_detecting().execute(self)

@with_plan
@dataclass
class MoveJointsMotion(BaseMotion):
    """
    Moves any joint on the robot
    """

    names: list
    """
    List of joint names that should be moved 
    """
    positions: list
    """
    Target positions of joints, should correspond to the list of names
    """

    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.move_joints().execute(self)

@with_plan
@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: ObjectDescription.Link
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.open().execute(self)

@with_plan
@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: ObjectDescription.Link
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.close().execute(self)

@with_plan
@dataclass
class TalkingMotion(BaseMotion):
    """
    Talking Motion, lets the robot say a sentence.
    """

    cmd: str
    """
    Talking Motion, let the robot say a sentence.
    """

    @with_tree
    def perform(self):
        pm_manager = ProcessModuleManager.get_manager()
        return pm_manager.talk().execute(self)