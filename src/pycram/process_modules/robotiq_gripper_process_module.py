from typing_extensions import Any

from .default_process_modules import *
from ..datastructures.enums import GripperState, ExecutionType
from ..datastructures.world import World
from pycram.robot_plans import MoveGripperMotion
from ..process_module import ProcessModuleManager
from ..robot_descriptions.ur5e_controlled_description import data as ur5e_data
from ..ros import create_publisher

try:
    from std_msgs.msg import Float64
except ImportError:
    loginfo ("Float64 message type not found, make sure to install std_msgs package")

class RobotiqMoveGripperReal(DefaultMoveGripperReal):
    """
    Opens or closes the gripper of the real Robotiq gripper, uses a topic for this instead of giskard
    """

    def _execute(self, designator: MoveGripperMotion) -> Any:
        value = ur5e_data.gripper_open_cmd_value if designator.motion == GripperState.OPEN \
            else ur5e_data.gripper_close_cmd_value
        publisher = create_publisher(ur5e_data.gripper_cmd_topic, Float64, queue_size=10, latch=True)
        World.current_world.step(func=lambda: publisher.publish(Float64(value)), step_seconds=0.1)
        return True


class RobotiqManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = ur5e_data.gripper_name

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return RobotiqMoveGripperReal(self._move_gripper_lock)

RobotiqManager()
