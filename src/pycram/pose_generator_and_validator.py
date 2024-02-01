import tf
import numpy as np

from .world import Object, World
from .world_reasoning import contact
from .costmaps import Costmap
from .pose import Pose, Transform
from .robot_descriptions import robot_description
from .external_interfaces.ik import request_ik
from .plan_failures import IKError
from .helper import _apply_ik
from typing_extensions import Tuple, List, Union, Dict, Iterable


def pose_generator(costmap: Costmap, number_of_samples=100, orientation_generator=None) -> Iterable:
    """
    A generator that crates pose candidates from a given costmap. The generator
    selects the highest 100 values and returns the corresponding positions.
    Orientations are calculated such that the Robot faces the center of the costmap.

    :param costmap: The costmap from which poses should be sampled.
    :param number_of_samples: The number of samples from the costmap that should be returned at max
    :param orientation_generator: function that generates an orientation given a position and the origin of the costmap
    :Yield: A tuple of position and orientation
    """
    if not orientation_generator:
        orientation_generator = generate_orientation

    # Determines how many positions should be sampled from the costmap
    if number_of_samples == -1:
        number_of_samples = costmap.map.flatten().shape[0]
    indices = np.argpartition(costmap.map.flatten(), -number_of_samples)[-number_of_samples:]
    indices = np.dstack(np.unravel_index(indices, costmap.map.shape)).reshape(number_of_samples, 2)

    height = costmap.map.shape[0]
    width = costmap.map.shape[1]
    center = np.array([height // 2, width // 2])
    for ind in indices:
        if costmap.map[ind[0]][ind[1]] == 0:
            continue
        # The position is calculated by creating a vector from the 2D position in the costmap (given by x and y)
        # and the center of the costmap (since this is the origin). This vector is then turned into a transformation
        # and muiltiplied with the transformation of the origin.
        vector_to_origin = (center - ind) * costmap.resolution
        point_to_origin = Transform([*vector_to_origin, 0], frame="point", child_frame="origin")
        origin_to_map = costmap.origin.to_transform("origin").invert()
        point_to_map = point_to_origin * origin_to_map
        map_to_point = point_to_map.invert()

        orientation = orientation_generator(map_to_point.translation_as_list(), costmap.origin)
        yield Pose(map_to_point.translation_as_list(), orientation)


def height_generator() -> float:
    pass


def generate_orientation(position: List[float], origin: Pose) -> List[float]:
    """
    This method generates the orientation for a given position in a costmap. The
    orientation is calculated such that the robot faces the origin of the costmap.
    This generation is done by simply calculating the arctan between the position,
    in the costmap, and the origin of the costmap.

    :param position: The position in the costmap. This position is already converted
        to the world coordinate frame.
    :param origin: The origin of the costmap. This is also the point which the
        robot should face.
    :return: A quaternion of the calculated orientation
    """
    angle = np.arctan2(position[1]-origin.position.y, position[0]-origin.position.x) + np.pi
    quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
    return quaternion


def visibility_validator(pose: Pose,
                         robot: Object,
                         object_or_pose: Union[Object, Pose],
                         world: World) -> bool:
    """
    This method validates if the robot can see the target position from a given
    pose candidate. The target position can either be a position, in world coordinate
    system, or an object in the World. The validation is done by shooting a
    ray from the camera to the target position and checking that it does not collide
    with anything else.

    :param pose: The pose candidate that should be validated
    :param robot: The robot object for which this should be validated
    :param object_or_pose: The target position or object for which the pose
        candidate should be validated.
    :param world: The World instance in which this should be validated.
    :return: True if the target is visible for the robot, None in any other case.
    """
    robot_pose = robot.get_pose()
    if isinstance(object_or_pose, Object):
        robot.set_pose(pose)
        camera_pose = robot.links[robot_description.get_camera_frame()].pose
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose.get_position_as_list())
        res = ray == object_or_pose.id
    else:
        robot.set_pose(pose)
        camera_pose = robot.links[robot_description.get_camera_frame()].pose
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        # TODO: Check if this is correct
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose)
        res = ray == -1
    robot.set_pose(robot_pose)
    return res


def _in_contact(robot: Object, obj: Object, allowed_collision: Dict[Object, List[str]],
                allowed_robot_links: List[str]) -> bool:
    """
    This method checks if a given robot is in contact with a given object.
    :param robot: The robot object that should be checked for contact.
    :param obj: The object that should be checked for contact with the robot.
    :param allowed_collision: A dictionary that contains the allowed collisions for links of each object in the world.
    :param allowed_robot_links: A list of links of the robot that are allowed to be in contact with the object.
    :return: True if the robot is in contact with the object and False otherwise.
    """
    in_contact, contact_links = contact(robot, obj, return_links=True)
    allowed_links = allowed_collision[obj] if obj in allowed_collision.keys() else []

    if in_contact:
        for link in contact_links:
            if link[0].name in allowed_robot_links or link[1].name in allowed_links:
                in_contact = False
                # TODO: in_contact is never set to True after it was set to False is that correct?
                # TODO: If it is correct, then this loop should break after the first contact is found
    return in_contact


def reachability_validator(pose: Pose,
                           robot: Object,
                           target: Union[Object, Pose],
                           allowed_collision: Dict[Object, List] = None) -> Tuple[bool, List]:
    """
    This method validates if a target position is reachable for a pose candidate.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. if there is a solution
    the validator returns True and False in any other case.

    :param pose: The pose candidate for which the reachability should be validated
    :param robot: The robot object in the World for which the reachability
        should be validated.
    :param target: The target position or object instance which should be the
        target for reachability.
    :param allowed_collision:
    :return: True if the target is reachable for the robot and False in any other
        case.
    """
    if type(target) == Object:
        target = target.get_pose()

    robot.set_pose(pose)

    left_gripper = robot_description.get_tool_frame('left')
    right_gripper = robot_description.get_tool_frame('right')

    # left_joints = robot_description._safely_access_chains('left').joints
    left_joints = robot_description.chains['left'].joints
    # right_joints = robot_description._safely_access_chains('right').joints
    right_joints = robot_description.chains['right'].joints
    # TODO Make orientation adhere to grasping orientation
    res = False
    arms = []
    in_contact = False

    allowed_robot_links = []
    if robot in allowed_collision.keys():
        allowed_robot_links = allowed_collision[robot]

    joint_state_before_ik = robot._current_joints_positions
    try:
        # resp = request_ik(base_link, end_effector, target_diff, robot, left_joints)
        resp = request_ik(target, robot, left_joints, left_gripper)
        _apply_ik(robot, resp, left_joints)

        for obj in World.current_world.objects:
            if obj.name == "floor":
                continue
            in_contact = _in_contact(robot, obj, allowed_collision, allowed_robot_links)

        if not in_contact:
            arms.append("left")
            res = True
    except IKError:
        pass
    finally:
        robot.set_joint_positions(joint_state_before_ik)

    try:
        # resp = request_ik(base_link, end_effector, target_diff, robot, right_joints)
        resp = request_ik(target, robot, right_joints, right_gripper)
        _apply_ik(robot, resp, right_joints)

        for obj in World.current_world.objects:
            if obj.name == "floor":
                continue
            in_contact = _in_contact(robot, obj, allowed_collision, allowed_robot_links)

        if not in_contact:
            arms.append("right")
            res = True
    except IKError:
        pass
    finally:
        robot.set_joint_positions(joint_state_before_ik)

    return res, arms
