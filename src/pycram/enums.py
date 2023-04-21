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
