from pycram.knowrob.reasoning import object_type, instances_of, object_pose
from pycram.object_designator import LocatedObjectDesignatorDescription, ObjectDesignator


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
    else:
        # Fetch the object pose and yield the grounded description
        _ground_pose(description)
        return description.__dict__


def call_ground(desig):
    type_to_function = {LocatedObjectDesignatorDescription: ground_located_object}
    ground_function = type_to_function[type(desig._description)]
    return ground_function(desig._description)


ObjectDesignator.resolvers['grounding'] = call_ground
