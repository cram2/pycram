from dataclasses import dataclass

from giskardpy.motion_statechart.tasks.cartesian_tasks import CartesianPose
from giskardpy.motion_statechart.tasks.pointing import Pointing

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
        return
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.navigate().execute(self)

    @property
    def _motion_chart(self):
        return CartesianPose(root_link=self.world.root, tip_link=self.robot_view.root, goal_pose=self.target.to_spatial_type())



