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
    align: Optional[bool] = False
    """
    If True, aligns the end-effector with a specified axis (optional).
    """
    tip_link: Optional[str] = None
    """
    Name of the tip link to align with, e.g the object (optional).
    """
    tip_normal: Optional[Vector3Stamped] = None
    """
    Normalized vector representing the current orientation axis of the end-effector (optional).
    """
    root_link: Optional[str] = None
    """
    Base link of the robot; typically set to the torso (optional).
    """
    root_normal: Optional[Vector3Stamped] = None
    """
    Normalized vector representing the desired orientation axis to align with (optional).
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.move_joints().execute(self)
