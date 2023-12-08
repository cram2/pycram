"""Module holding all enums of PyCRAM."""

from enum import Enum, auto


class Arms(Enum):
    """Enum for Arms."""
    LEFT = auto()
    RIGHT = auto()
    BOTH = auto()


class TaskStatus(Enum):
    """
    Enum for readable descriptions of a tasks' status.
    """
    CREATED = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED = 3


class JointType(Enum):
    """
    Enum for readable joint types.
    """
    REVOLUTE = 0
    PRISMATIC = 1
    SPHERICAL = 2
    PLANAR = 3
    FIXED = 4


class Grasp(Enum):
    """
    Enum for Grasp orientations.
    """
    FRONT = 0
    LEFT = 1
    RIGHT = 2
    TOP = 3


class ObjectType(Enum):
    """
    Enum for Object types to easier identify different objects
    """
    MILK = auto()
    SPOON = auto()
    BOWL = auto()
    BREAKFAST_CEREAL = auto()
    JEROEN_CUP = auto()
    ROBOT = auto()
    ENVIRONMENT = auto()
    GENERIC_OBJECT = auto()


class State(Enum):
    """
    Enumeration which describes the result of a language expression.
    """
    SUCCEEDED = 1
    FAILED = 0
    RUNNING = 2
    INTERRUPTED = 3

