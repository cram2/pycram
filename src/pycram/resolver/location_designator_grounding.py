from pycram.designator import DesignatorError
from pycram.helper import transform
from pycram.location_designator import ObjectRelativeLocationDesignatorDescription, LocationDesignator, \
    LocationDesignatorDescription


def ground_object_relative_location(description: ObjectRelativeLocationDesignatorDescription):
    if description.pose is None:
        if description.relative_pose is None or description.reference_object is None:
            raise DesignatorError("Could not ground ObjectRelativeLocationDesignatorDescription: (Relative) pose and reference object must be given")
        # Fetch the object pose and yield the grounded description
        obj_grounded = description.reference_object.reference()
        obj_pose_world = obj_grounded["pose"]
        description.pose = transform(obj_pose_world, description.relative_pose, local_coords=False)

    return description.__dict__


def ground_location(description: LocationDesignatorDescription):
    if description.pose is None:
        raise DesignatorError("Could not ground LocationDesignatorDescription: Pose in world coordinates must be given")
    yield description.__dict__


def call_ground(desig):
    type_to_function = {ObjectRelativeLocationDesignatorDescription: ground_object_relative_location,
                        LocationDesignatorDescription: ground_location}
    ground_function = type_to_function[type(desig._description)]
    return ground_function(desig._description)


LocationDesignator.resolvers['grounding'] = call_ground
