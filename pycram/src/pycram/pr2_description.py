from enum import Enum, auto

class Arms(Enum):
    LEFT = auto()
    RIGHT = auto()
    BOTH = auto()

class ArmConfiguration(Enum):
    PARK = auto()
    CARRY = auto()

class Grasp(Enum):
    TOP = auto()