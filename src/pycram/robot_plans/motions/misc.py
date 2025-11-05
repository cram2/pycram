from dataclasses import dataclass
from typing import Optional

from semantic_digital_twin.world_description.world_entity import Body

from .base import BaseMotion
from ...datastructures.enums import DetectionTechnique, DetectionState, ObjectType
from ...process_module import ProcessModuleManager


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
    object_designator_description: Optional[Body] = None
    """
    Description of the object that should be detected
    """
    region: Optional['Location'] = None
    """
    Region in which the object should be detected
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        obj_dict = pm_manager.detecting().execute(self)
        return obj_dict

