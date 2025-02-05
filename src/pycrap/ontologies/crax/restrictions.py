from .dependencies import *
from .classes import *
#from .individuals import *
from .object_properties import *

Floor.is_a = [PhysicalObject]

Milk.is_a = [Food]

Robot.is_a = [Agent]

Cereal.is_a = [Food]

Kitchen.is_a = [Room, Location]

Food.is_a = [PhysicalObject]

Apartment.is_a = [Room, Location]

Cup.is_a = [Container]

Spoon.is_a = [PhysicalObject]

Bowl.is_a = [Container]

Container.is_a = [PhysicalObject]

ContinuousJoint.is_a = [HingeJoint]

HingeJoint.is_a = [MovableJoint]

FixedJoint.is_a = [Joint]

MovableJoint.is_a = [Joint]

PlanarJoint.is_a = [MovableJoint]

PrismaticJoint.is_a = [MovableJoint]

FloatingJoint.is_a = [MovableJoint]

RevoluteJoint.is_a = [HingeJoint]

Event.is_a = [Entity]

Entity.is_a = [Thing]

Action.is_a = [Event]

PhysicalTask.is_a = [Entity]


is_part_of.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty]
is_part_of.domain = [PhysicalObject]
is_part_of.range = [PhysicalObject]

has_part.is_a = [ObjectProperty, TransitiveProperty, ReflexiveProperty]
has_part.domain = [PhysicalObject]
has_part.range = [PhysicalObject]

has_parent_link.is_a = [ObjectProperty]
has_parent_link.domain = [Joint]
has_parent_link.range = [PhysicalObject]

has_child_link.is_a = [ObjectProperty]
has_child_link.domain = [Joint]
has_child_link.range = [PhysicalObject]

