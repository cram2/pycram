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

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.navigate().execute(self)


@with_plan
@dataclass
class LookingMotion(BaseMotion):
    """
    Lets the robot look at a point
    """
    target: PoseStamped

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.looking().execute(self)
