from threading import Lock

from std_msgs.msg import Float64
from typing_extensions import Any

from .default_process_modules import DefaultMoveGripper, DefaultManager
from ..datastructures.world import World
from ..datastructures.enums import GripperState, ExecutionType
from ..designators.motion_designator import MoveGripperMotion
from ..process_module import ProcessModule, ProcessModuleManager
from ..ros.publisher import create_publisher
from ..robot_descriptions.ur5e_controlled_description import GRIPPER_NAME, GRIPPER_CMD_TOPIC, OPEN_VALUE, CLOSE_VALUE


class RobotiqMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real Robotiq gripper, uses a topic for this instead of giskard
    """
    def _execute(self, designator: MoveGripperMotion) -> Any:
        value = OPEN_VALUE if designator.motion == GripperState.OPEN else CLOSE_VALUE
        publisher = create_publisher(GRIPPER_CMD_TOPIC, Float64, queue_size=10, latch=True)
        World.current_world.step(func=lambda: publisher.publish(Float64(value)), step_seconds=0.1)
        return True


class RobotiqManager(DefaultManager):

    def __init__(self):
        super().__init__()
        self.robot_name = GRIPPER_NAME

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return RobotiqMoveGripperReal(self._move_gripper_lock)
