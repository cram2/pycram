from owlready2 import Imp

from .dependencies import *
from .classes import *
from .individuals import *
from .object_properties import *

Floor.is_a = [PhysicalObject]

Milk.is_a = [Food]

Robot.is_a = [Agent]

Human.is_a = [Agent]

Cereal.is_a = [Food]

Kitchen.is_a = [Room]

Food.is_a = [PhysicalObject]

Apartment.is_a = [Room]

Container.is_a = [PhysicalObject]

Room.is_a = [Location, Container]

Cup.is_a = [Container]

Bowl.is_a = [Container]

DefaultPreferredGraspAlignment.is_a = [PreferredGraspAlignment, has_preferred_axis.some(NoAlignment),
                                       has_vertical_alignment.value(Falsy), has_rotated_gripper.value(Falsy),
                                       has_rim_grasp.value(Falsy)]

BowlPreferredGraspAlignment.is_a = [PreferredGraspAlignment, has_preferred_axis.value(NoAlignment),
                                    has_vertical_alignment.value(Truthy), has_rotated_gripper.value(Truthy),
                                    has_rim_grasp.value(Truthy)]

SpoonPreferredGraspAlignment.is_a = [PreferredGraspAlignment, has_preferred_axis.value(XAxis),
                                     has_vertical_alignment.value(Truthy), has_rotated_gripper.value(Falsy),
                                     has_rim_grasp.value(Falsy)]

CerealPreferredGraspAlignment.is_a = [PreferredGraspAlignment, has_preferred_axis.value(XAxis),
                                      has_vertical_alignment.value(Falsy), has_rotated_gripper.value(Falsy),
                                      has_rim_grasp.value(Falsy)]

Cabinet.is_a = [Container]

Drawer.is_a = [Container]

Spoon.is_a = [PhysicalObject]

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

Supporter.is_a = [PhysicalObject]

SupportedObject.is_a = [PhysicalObject]

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

contains.is_a = [ObjectProperty, TransitiveProperty]
contains.domain = [Base]
contains.range = [Base]
contains.inverse_property = is_contained_in

contains_object.is_a = [ObjectProperty, TransitiveProperty]
contains_object.domain = [Container]
contains_object.range = [PhysicalObject]
contains_object.inverse_property = is_physically_contained_in

is_contained_in.is_a = [ObjectProperty, TransitiveProperty]
is_contained_in.domain = [Base]
is_contained_in.range = [Base]

is_physically_contained_in.is_a = [ObjectProperty, TransitiveProperty]
is_physically_contained_in.domain = [PhysicalObject]
is_physically_contained_in.range = [Container]

supports.is_a = [ObjectProperty, TransitiveProperty]
supports.domain = [Supporter]
supports.range = [SupportedObject]
supports.inverse_property = is_supported_by

is_supported_by.is_a = [ObjectProperty, TransitiveProperty]
is_supported_by.domain = [SupportedObject]
is_supported_by.range = [Supporter]

for c in ontology.classes():
    c.has_preferred_alignment.append(DefaultPreferredGraspAlignment)

Bowl.has_preferred_alignment = [BowlPreferredGraspAlignment]
Spoon.has_preferred_alignment = [SpoonPreferredGraspAlignment]
Cereal.has_preferred_alignment = [CerealPreferredGraspAlignment]
