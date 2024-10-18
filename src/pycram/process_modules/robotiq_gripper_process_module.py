from threading import Lock

import rospy
from std_msgs.msg import Float64
from typing_extensions import Any

from .default_process_modules import DefaultMoveGripper
from ..datastructures.world import World
from ..datastructures.enums import GripperState, ExecutionType
from ..designators.motion_designator import MoveGripperMotion
from ..process_module import ProcessModule, ProcessModuleManager


GRIPPER_NAME = "gripper-2F-85"
GRIPPER_CMD_TOPIC = "/gripper_command"
OPEN_VALUE = 0.0
CLOSE_VALUE = 255.0


class RobotiqMoveGripperReal(ProcessModule):
    """
    Opens or closes the gripper of the real Robotiq gripper, uses a topic for this instead of giskard
    """
    def _execute(self, designator: MoveGripperMotion) -> Any:
        value = OPEN_VALUE if designator.motion == GripperState.OPEN else CLOSE_VALUE
        publisher = rospy.Publisher(GRIPPER_CMD_TOPIC, Float64, queue_size=10, latch=True)
        World.current_world.step(func=lambda: publisher.publish(Float64(value)), step_seconds=0.1)
        return True


class RobotiqManager(ProcessModuleManager):

    def __init__(self):
        super().__init__(GRIPPER_NAME)
        self._move_gripper_lock = Lock()

    def move_gripper(self):
        if ProcessModuleManager.execution_type == ExecutionType.SIMULATED:
            return DefaultMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == ExecutionType.REAL:
            return RobotiqMoveGripperReal(self._move_gripper_lock)
