from dataclasses import dataclass

from .base import BaseMotion
from ...datastructures.pose import PoseStamped
from ...process_module import ProcessModuleManager


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
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.navigate().execute(self)


@dataclass
class LookingMotion(BaseMotion):
    """
    Lets the robot look at a point
    """

    target: PoseStamped

    def perform(self):
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.looking().execute(self)
