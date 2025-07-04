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

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager()
        obj_dict = pm_manager.detecting().execute(self)
        return obj_dict


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
        pm_manager = ProcessModuleManager().get_manager()
        return pm_manager.world_state_detecting().execute(self)

