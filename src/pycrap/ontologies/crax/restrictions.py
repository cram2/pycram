from .dependencies import *
from .classes import *
#from .individuals import *

Floor.is_a = [PhysicalObject]

Milk.is_a = [Food]

Robot.is_a = [Agent]

Cereal.is_a = [Food]

Kitchen.is_a = [Room, Location]

Food.is_a = [PhysicalObject]

Apartment.is_a = [Room, Location]

Cup.is_a = [Container, PhysicalObject]

Spoon.is_a = [PhysicalObject]

Bowl.is_a = [Container, PhysicalObject]
