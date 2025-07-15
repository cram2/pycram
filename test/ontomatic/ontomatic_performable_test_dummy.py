from dataclasses import dataclass
from enum import Enum, auto
from pycram.robot_plans.actions.base import ActionDescription


class GripperState(Enum):
    """
    Enum for the different motions of the gripper.
    """
    OPEN = auto()
    CLOSE = auto()

class Arms(int, Enum):
    """
    Enum for Arms.
    """
    LEFT = 0
    RIGHT = 1
    BOTH = 2

@dataclass
class TestOntomaticPerformable(ActionDescription):
    """
    Set the gripper state of the robot.
    """

    gripper: Arms
    """
    The gripper that should be set
    """
    motion: GripperState
    """
    The motion that should be set on the gripper
    """
    the_truth: bool = True
    """
    The true meaning
    """