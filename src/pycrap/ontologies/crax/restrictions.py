from .dependencies import *
from .classes import *
from .individuals import *

Floor.is_a = [PhysicalObject]

Milk.is_a = [PhysicalObject]

Robot.is_a = [Agent]

Cereal.is_a = [PhysicalObject]

Kitchen.is_a = [Room]