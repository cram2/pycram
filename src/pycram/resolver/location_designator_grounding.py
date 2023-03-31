from pycram.designator import DesignatorError
from pycram.helper import transform
from pycram.designators.location_designator import ObjectRelativeLocationDesignatorDescription, LocationDesignator, \
    LocationDesignatorDescription
from pycram.costmaps import GaussianCostmap, OccupancyCostmap, VisibilityCostmap, SemanticCostmap
from pycram.robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from pycram.bullet_world import BulletWorld, Object, Use_shadow_world
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
    """
    Generates positions for a given set of constrains from a costmap and returns
    them. The generation is based of a costmap which itself is the product of
    merging costmaps, each for a different purpose. In any case a occupancy costmap
    is used as the base, then according to the given constrains a visibility or
    gaussian costmap is also merged with this. Once the costmaps are merged,
    a generator generates pose candidates from the costmap. Each pose candidate
    is then validated against the constrains given by the designator if all validators
    pass the pose is considered valid and added to the list of valid poses.
    After a maximum of 15 valid poses are found the validation of poses stops
    and the found valid poses are yielded as soultions for the designator.
    :param desig: The location designator for which poses should be found.
    :yield: The found valid poses for the constrains given by the designator.
    """
    min_height = list(robot_description.i.cameras.values())[0].min_height
    max_height = list(robot_description.i.cameras.values())[0].max_height
    # This ensures that the costmaps always get a position as their origin.
    if type(desig._description.target) == Object:
        target_pose = desig._description.target.get_position_and_orientation()
    else:
        target_pose = desig._description.target


    ground_pose = [[target_pose[0][0], target_pose[0][1], 0], target_pose[1]]

    occupancy = OccupancyCostmap(0.4, False, 200, 0.02, [ground_pose[0], [0, 0, 0, 1]], BulletWorld.current_bullet_world)
    final_map = occupancy

    if desig._description.reachable_for:
        gaussian = GaussianCostmap(200, 15, 0.02, [ground_pose[0], [0, 0, 0, 1]])
        final_map += gaussian
    if desig._description.visible_for:
        visible = VisibilityCostmap(min_height, max_height, 200, 0.02, [target_pose[0], [0, 0, 0, 1]])
        final_map += visible
    #plot_grid(final_map.map)

    #test_world = BulletWorld.current_bullet_world.copy()
    robot_object = desig._description.visible_for if desig._description.visible_for else desig._description.reachable_for
    #test_robot = test_world.get_objects_by_name(robot_object.name)[0]

    with Use_shadow_world():
        test_robot = BulletWorld.current_bullet_world.get_shadow_object(robot_object)
        valid_poses = []
        for maybe_pose in pose_generator(final_map):
            res = True
            arms = None
            if desig._description.visible_for:
                res = res and visibility_validator(maybe_pose, test_robot, desig._description.target, BulletWorld.current_bullet_world)
            if desig._description.reachable_for:
                valid, arms = reachability_validator(maybe_pose, test_robot, desig._description.target, BulletWorld.current_bullet_world)
                if desig._description.reachable_arm:
                    res = res and valid and desig._description.reachable_arm in arms
                else:
                    res = res and valid

            if res:
                valid_poses.append([maybe_pose, arms])
                #print(f"Valid: {maybe_pose}")
                # This number defines the total valid poses by this generator
                if len(valid_poses) == 15: break
                #yield {'position': maybe_pose[0], 'orientation': maybe_pose[1]}
    #test_world.exit()
    for pose, arms in valid_poses:
        yield {'position': pose[0], 'orientation': pose[1], "arms": arms}

def gen_from_sem_costmap(desig):
    sem_costmap = SemanticCostmap(desig._description.part_of, desig._description.urdf_link_name)
    height_offset = 0
    if desig._description.for_object:
        min, max = desig._description.for_object.get_AABB()
        height_offset = (max[2] - min[2]) / 2
    for maybe_pose in pose_generator(sem_costmap):
        position = [maybe_pose[0][0], maybe_pose[0][1], maybe_pose[0][2] + height_offset]
        yield {'position': position, 'orientation': maybe_pose[1]}

LocationDesignator.resolvers['grounding'] = call_ground
LocationDesignator.resolvers['costmap'] = gen_from_costmap
LocationDesignator.resolvers['semantic-costmap'] = gen_from_sem_costmap
