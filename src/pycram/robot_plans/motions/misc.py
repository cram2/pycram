from dataclasses import dataclass
from typing import Optional

from pycrap.ontologies import Location
from .base import BaseMotion
from ...datastructures.enums import DetectionTechnique, DetectionState, ObjectType
from ...process_module import ProcessModuleManager
from ...world_concepts.world_object import Object


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
