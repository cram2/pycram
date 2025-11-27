from dataclasses import dataclass

from giskardpy.motion_statechart.goals.open_close import Open, Close
from semantic_digital_twin.world_description.world_entity import Body

from .base import BaseMotion
from ...datastructures.enums import Arms
from ...process_module import ProcessModuleManager
from ...robot_description import ViewManager


@dataclass
class OpeningMotion(BaseMotion):
    """
    Designator for opening container
    """

    object_part: Body
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        return
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.open().execute(self)

    @property
    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        return Open(tip_link=tip, environment_link=self.object_part)


@dataclass
class ClosingMotion(BaseMotion):
    """
    Designator for closing a container
    """

    object_part: Body
    """
    Object designator for the drawer handle
    """
    arm: Arms
    """
    Arm that should be used
    """

    def perform(self):
        return
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.close().execute(self)

    def _motion_chart(self):
        tip = ViewManager().get_end_effector_view(self.arm, self.robot_view).tool_frame
        return Close(tip_link=tip, environment_link=self.object_part)
