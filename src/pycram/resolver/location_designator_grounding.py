from pycram.designator import DesignatorError
from pycram.helper import transform
from pycram.location_designator import ObjectRelativeLocationDesignatorDescription, LocationDesignator, \
    LocationDesignatorDescription
from pycram.costmaps import GaussianCostmap, OccupancyCostmap, VisibilityCostmap, plot_grid
from pycram.robot_description import InitializedRobotDescription as robot_description
from pycram.bullet_world import BulletWorld, Object
from pycram.pose_generator_and_validator import pose_generator, visibility_validator, reachability_validator


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


def gen_from_costmap(desig):
    min_height = list(robot_description.i.cameras.values())[0].min_height
    max_height = list(robot_description.i.cameras.values())[0].max_height
    # This ensures that the costmaps always get a position as their origin.
    if type(desig._description.target) == Object:
        target_pose = desig._description.target.get_position()
    else:
        target_pose = desig._description.target
    occupancy = OccupancyCostmap(0.2, False, 200, 0.02, target_pose, BulletWorld.current_bullet_world)
    final_map = occupancy

    if desig._description.reachable_for:
        gaussian = GaussianCostmap(200, 15, 0.02, target_pose)
        final_map += gaussian
    if desig._description.visible_for:
        visible = VisibilityCostmap(min_height, max_height, 200, 0.02, target_pose)
        final_map += visible
    test_world = BulletWorld.current_bullet_world.copy()
    robot_object = desig._description.visible_for if desig._description.visible_for else desig._description.reachable_for
    test_robot = test_world.get_objects_by_name(robot_object.name)[0]

    valid_poses = []
    for maybe_pose in pose_generator(final_map):
        res = True
        if desig._description.visible_for:
            res = res and visibility_validator(maybe_pose, test_robot, desig._description.target, test_world)
        if desig._description.reachable_for:
            res = res and reachability_validator(maybe_pose, test_robot, desig._description.target, test_world)

        if res:
            valid_poses.append(maybe_pose)
            # This number defines the total valid poses by this generator
            if len(valid_poses) == 15: break
            #yield {'position': maybe_pose[0], 'orientation': maybe_pose[1]}
    test_world.exit()
    for pose in valid_poses:
        yield {'position': pose[0], 'orientation': pose[1]}


LocationDesignator.resolvers['grounding'] = call_ground
LocationDesignator.resolvers['costmap'] = gen_from_costmap
