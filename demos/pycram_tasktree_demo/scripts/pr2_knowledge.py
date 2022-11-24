from enum import Enum, auto

from pycram.designators.object_designator import ObjectDesignator


OBJECT_HELD_LEFT = None
OBJECT_HELD_RIGHT = None

def free_arms():
    if not OBJECT_HELD_LEFT and not OBJECT_HELD_RIGHT:
        return [Arms.RIGHT, Arms.LEFT]
    elif not OBJECT_HELD_LEFT:
        return [Arms.LEFT]
    elif not OBJECT_HELD_RIGHT:
        return [Arms.RIGHT]
    else:
        return []

def reach_position_generator(target):
    if type(target) is ObjectDesignator:
        if target.prop_value('name') in ['sink_area_left_middle_drawer', 'sink_area_left_upper_drawer']:
            yield [0.3, 0.9, 0], [0,0,0,1]
            yield [0.4, 0.9, 0], [0,0,0,1]
            yield [0.5, 0.9, 0], [0,0,0,1]
            yield [0.6, 0.9, 0], [0,0,0,1]
        elif target.prop_value('name') is 'iai_fridge':
            yield [0.5, -0.4, 0], [0, 0, -0.258819, 0.9659258]
        else:
            yield [0.6, 0.9, 0], [0,0,0,1]
    else:
        yield [-1.8, 1, 0], [0,0,0,1]
        yield [-0.4, 1, 0], [0,0,1,0]

def container_opening_distance_generator(object_designator):
    if object_designator:
        if object_designator.prop_value('name') is 'iai_fridge':
            yield 1.0
        elif object_designator.prop_value('name') is 'sink_area_left_upper_drawer':
            yield 0.1
            yield 0.25
            yield 0.3
            yield 0.4
        else:
            yield 0.4
    else:
        yield 0.4

def object_fetching_location_generator(object_designator):
    object_type = object_designator.prop_value('type')
    if object_type is "spoon":
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_upper_drawer'), ('part-of', "kitchen")])
    elif object_type is "bowl":
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_middle_drawer'), ('part-of', "kitchen")])
    elif object_type is "milk":
        yield ObjectDesignator([('type', 'fridge'), ('name', 'iai_fridge'), ('part-of', "kitchen")])
        yield [1.3, 0.8, 0.95]  # Location on counter top
    elif object_type is "cereal":
        yield [1.3, 0.8, 0.95]  # Location on counter top
    else:
        # Otherwise just look everywhere
        yield [1.3, 0.8, 0.95]
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_upper_drawer'), ('part-of', "kitchen")])
        yield ObjectDesignator([('type', 'drawer'), ('name', 'sink_area_left_middle_drawer'), ('part-of', "kitchen")])
        yield ObjectDesignator([('type', 'fridge'), ('name', 'iai_fridge'), ('part-of', "kitchen")])

def object_placing_location_generator(object_designator, destination):
    if destination == "kitchen_island_countertop":
        object_type = object_designator.prop_value("type")
        if object_type == "milk":
            yield [-1.15, 1.2, 0.95]
            yield [-1.05, 1.2, 0.95]
        elif object_type == "cereal":
            yield [-1.15, 1.0, 0.95]
            yield [-1.05, 1.0, 0.95]
        elif object_type == "bowl":
            yield [-1.35, 1.1, 0.95]
            yield [-0.9, 1.1, 0.95]
        elif object_type == "spoon":
            yield [-1.35, 0.95, 0.95]
            yield [-0.9, 1.3, 0.95]
    else:
        raise NotImplementedError("This is just a hack for now.")

class Arms(Enum):
    LEFT = auto()
    RIGHT = auto()
    BOTH = auto()

class ArmConfiguration(Enum):
    PARK = auto()
    CARRY = auto()

class Grasp(Enum):
    TOP = auto()
