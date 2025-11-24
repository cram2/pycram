from dataclasses import dataclass
from typing import Optional

from .base import BaseMotion
from ...datastructures.pose import Vector3Stamped
from ...process_module import ProcessModuleManager
from giskardpy.motion_statechart.tasks.joint_tasks import JointPositionList, JointState


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
        pm_manager = ProcessModuleManager().get_manager(self.robot_view)
        return pm_manager.move_joints().execute(self)

    @property
    def _motion_chart(self):
        dofs = [self.world.get_connection_by_name(name) for name in self.names]
        return JointPositionList(goal_state=JointState(dict(zip(dofs, self.positions))))
