from dataclasses import dataclass

from ...perception import PerceptionQuery
from .base import BaseMotion
from ...process_module import ProcessModuleManager


@dataclass
class DetectingMotion(BaseMotion):
    """
    Tries to detect an object in the FOV of the robot

    returns: ObjectDesignatorDescription.Object or Error: PerceptionObjectNotFound
    """

    query: PerceptionQuery
    """
    Query for the perception system that should be answered
    """

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        obj_dict = pm_manager.detecting().execute(self)
        return obj_dict

    def _motion_chart(self):
        pass

