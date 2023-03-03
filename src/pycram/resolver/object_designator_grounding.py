from ..external_interfaces.knowrob import object_type, instances_of, object_pose
from ..designators.object_designator import *
from ..bullet_world import BulletWorld
from ..designator import ResolutionError, DesignatorError
from ..designators.object_designator import ObjectDesignator


def ground_located_object(description: LocatedObjectDesignatorDescription):
    def _ground_pose(desc):
        desc.pose = object_pose(desc.name, desc.reference_frame, desc.timestamp)

    if not description.type and not description.name:
        raise RuntimeError("Could not ground LocatedObjectDesignatorDescription: Either type or name must be given")
    if not description.type:
        description.type = object_type(description.name)
    elif not description.name:
        # Retrieve all objects of the given type, fetch their poses and yield the grounded description
        object_names = instances_of(description.type)
        for object_name in object_names:
            description.name = object_name
            _ground_pose(description)
            yield description.__dict__
    # Fetch the object pose and yield the grounded description
    _ground_pose(description)
    yield description.__dict__


def ground_object(description: ObjectDesignatorDescription):
    # If there already is an object reference fillout the rest of the Arguments
    if description.object:
        if not description.name:
            description.name = description.object.name
        if not descripition.type:
            description.type = descrption.object.type
        description.pose = description.object.get_position_and_orientation()
        return description.__dict__

    # If there is a name get every fitting object and if there is no type or Only
    # one object that fits the name return the object designator for it
    if description.name:
        objs_name = BulletWorld.current_bullet_world.get_objects_by_name(description.name)
        if len(objs_name) == 0:
            raise DesignatorError("No suitable object found")
        if len(objs_name) == 1 or not description.type:
            description.object = objs_name[0]
            description.type = objs_name[0].type
            description.pose = objs_name[0].get_position_and_orientation()
            return description.__dict__

    # Checks if the designator contains an objct type, if yes get every fitting
    # object (for this type). If there is only one object or no name return the
    # first fitting object
    if description.type:
        objs_type = BulletWorld.current_bullet_world.get_objects_by_type(description.type)
        if len(objs_type) == 0:
            raise DesignatorError("No suitable object found")
        if len(objs_type) == 1 or not description.name:
            description.object = objs_type[0]
            description.name = objs_type[0].name
            description.pose = objs_type[0].get_position_and_orientation()
            return description.__dict__

    # If there is a name and type in the object designator get every object that
    # fits both. If there are still more than one object that fits both properties
    # just return the first found.
    if describtion.name and describtion.type:
        intersection = set(objs_name).intersection(objs_type)
        if len(intersection) == 0:
            raise DesignatorError("No suitable object found")
        describtion.type = intersection[0].type
        describtion.name = intersection[0].name
        describtion.object = intersection[0]
        description.pose = intersection[0].get_position_and_orientation()

def ground_part_of(description):
    print(description.part_of)
    if type(description.part_of) == ObjectDesignator and not description.part_of._description.object:
        ref = description.part_of.reference()
        description.part_of = ref

    if description.part_of:
        return description.__dict__
    # Try to find the name as the name of a link in an object of the BulletWorld
    for obj in BulletWorld.current_bullet_world.objects:
        if description.name in obj.links.keys():
            description.part_of = obj
            return description.__dict__
    return description.__dict__




def call_ground(desig):
    type_to_function = {LocatedObjectDesignatorDescription: ground_located_object,
                        ObjectDesignatorDescription: ground_object,
                        ObjectPartDescription: ground_part_of}
    ground_function = type_to_function[type(desig._description)]
    return ground_function(desig._description)


ObjectDesignator.resolvers['grounding'] = call_ground
