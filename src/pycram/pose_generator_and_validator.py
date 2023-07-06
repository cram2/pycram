import tf
import numpy as np
import rospy
import pybullet as p

from .bullet_world import Object, BulletWorld, Use_shadow_world
from .bullet_world_reasoning import contact
from .costmaps import Costmap
from .robot_descriptions import robot_description
from .external_interfaces.ik import _make_request_msg, request_ik
from .plan_failures import IKError
from .helper import _transform_to_torso, calculate_wrist_tool_offset, inverseTimes, _apply_ik
from moveit_msgs.srv import GetPositionIK
from typing import Type, Tuple, List, Union, Dict


def pose_generator(costmap: Type[Costmap], number_of_samples=100, orientation_generator=None) -> Tuple[List[float], List[float]]:
    """
    A generator that crates pose candidates from a given costmap. The generator
    selects the highest 100 values and returns the corresponding positions.
    Orientations are calculated such that the Robot faces the center of the costmap.

    :param costmap: The costmap from which poses should be sampled.
    :param number_of_samples: The number of samples from the costmap that should be returned at max
    :param orientation_generator: function that generates a orientation given a position and the origin of the costmap
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
        vector_to_origin = center - ind
        transform_to_origin = [[vector_to_origin[0] * costmap.resolution, vector_to_origin[1] * costmap.resolution, 0], [0, 0, 0, 1]]
        origin_to_map = p.invertTransform(costmap.origin[0], costmap.origin[1])
        point_to_map = p.multiplyTransforms(transform_to_origin[0], transform_to_origin[1], origin_to_map[0], origin_to_map[1])
        map_to_point = p.invertTransform(point_to_map[0], point_to_map[1])
        #world_pose = p.multiplyTransforms(costmap.origin[0], costmap.origin[1], transform_to_origin[0], transform_to_origin[1])

        #position = [(ind[0]- size/2) *-1 * costmap.resolution + costmap.origin[0][0], (ind[1] - size/2) * -1 * costmap.resolution + costmap.origin[0][1], 0]
        orientation = orientation_generator(map_to_point[0], costmap.origin)
        yield list(map_to_point[0]), orientation


def height_generator() -> float:
    pass


def generate_orientation(position: List[float], origin: List[float]) -> List[float]:
    """
    This method generates the orientation for a given position in a costmap. The
    orientation is calculated such that the robot faces the origin of the costmap.
    This generation is done by simply calculating the arctan between the position,
    in the costmap, and the origin of the costmap.

    :param position: The position in the costmap. This position is already converted
        to the world coordinate frame.
    :param origin: The origin of the costmap. This is also the point which the
        robot should face.
    :return: A quanternion of the calculated orientation
    """
    angle = np.arctan2(position[1]-origin[0][1], position[0]-origin[0][0]) + np.pi
    quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
    return quaternion


def visibility_validator(pose: Tuple[List[float], List[float]],
                         robot: Object,
                         object_or_pose: Union[Object, Tuple[List[float], List[float]]],
                         world: BulletWorld) -> bool:
    """
    This method validates if the robot can see the target position from a given
    pose candidate. The target position can either be a position, in world coordinate
    system, or an object in the BulletWorld. The validation is done by shooting a
    ray from the camera to the target position and checking that it does not collide
    with anything else.

    :param pose: The pose candidate that should be validated
    :param robot: The robot object for which this should be validated
    :param object_or_pose: The target position or object for which the pose
        candidate should be validated.
    :param world: The BulletWorld instance in which this should be validated.
    :return: True if the target is visible for the robot, None in any other case.
    """
    robot_pose = robot.get_position_and_orientation()
    if type(object_or_pose) == Object:
        robot.set_position_and_orientation(pose[0], pose[1])
        camera_pose = robot.get_link_position_and_orientation(robot_description.get_camera_frame())
        robot.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])
        ray = p.rayTest(camera_pose[0], object_or_pose.get_position(), physicsClientId=world.client_id)
        res = ray[0][0] == object_or_pose.id
        #res = visible(object_or_pose, camera_pose, world=world)
    else:
        robot.set_position_and_orientation(pose[0], pose[1])
        camera_pose = robot.get_link_position_and_orientation(robot_description.get_camera_frame())
        robot.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])
        ray = p.rayTest(camera_pose[0], object_or_pose, physicsClientId=world.client_id)
        res = ray[0][0] == -1
    robot.set_position_and_orientation(robot_pose[0], robot_pose[1])
    return res


def reachability_validator(pose: Tuple[List[float], List[float]],
                           robot: Object,
                           target: Union[Object, Tuple[List[float], List[float]]],
                           world: BulletWorld,
                           allowed_collision: Dict[Object, List] = None) -> Tuple[bool, List]:
    """
    This method validates if a target position is reachable for a pose candidate.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. if there is a solution
    the validator returns True and False in any other case.

    :param pose: The pose candidate for which the reachability should be validated
    :param robot: The robot object in the BulletWorld for which the reachability
        should be validated.
    :param target: The target position or object instance which should be the
        target for reachability.
    :param world: The BulletWorld instance in which the reachability should be
        validated.
    :param allowed_collision:
    :return: True if the target is reachable for the robot and False in any other
        case.
    """
    if type(target) == Object:
        target = target.get_position_and_orientation()

    # robot_pose = robot.get_position_and_orientation()
    robot.set_position_and_orientation(pose[0], pose[1])

    left_gripper = robot_description.get_tool_frame('left')
    right_gripper = robot_description.get_tool_frame('right')

    left_joints = robot_description._safely_access_chains('left').joints
    right_joints = robot_description._safely_access_chains('right').joints
    # TODO Make orientation adhere to grasping orientation
    target_torso = _transform_to_torso(target, robot)

    # Get Link before first joint in chain
    base_link = robot_description.get_parent(left_joints[0])
    # Get link after last joint in chain
    end_effector = robot_description.get_child(left_joints[-1])

    diff = calculate_wrist_tool_offset(end_effector, robot_description.get_tool_frame("left"), robot)
    target_diff = inverseTimes(target_torso, diff)

    # position = np.round(np.array(target_diff[0]), decimals=6)
    # target_round = (position, target_diff[1])

    res = False
    arms = []
    in_contact = False

    allowed_robot_links = []
    if robot in allowed_collision.keys():
        allowed_robot_links = allowed_collision[robot]

    try:
        resp = request_ik(base_link, end_effector, target_diff, robot, left_joints)

        _apply_ik(robot, resp, left_joints)

        for obj in BulletWorld.current_bullet_world.objects:
            if obj.name == "floor":
                continue
            in_contact, contact_links = contact(robot, obj, return_links=True)
            allowed_links = allowed_collision[obj] if obj in allowed_collision.keys() else []

            if in_contact:
                for link in contact_links:

                    if link[0] in allowed_robot_links or link[1] in allowed_links:
                        in_contact = False

        if not in_contact:
            arms.append("left")
            res = True
    except IKError:
        pass

    base_link = robot_description.get_parent(right_joints[0])
    # Get link after last joint in chain
    end_effector = robot_description.get_child(right_joints[-1])
    diff = calculate_wrist_tool_offset(end_effector, robot_description.get_tool_frame("right"), robot)
    target_diff = inverseTimes(target_torso, diff)

    # position = np.round(np.array(target_diff[0]), decimals=6)
    # target_round = (position, target_diff[1])
    try:
        resp = request_ik(base_link, end_effector, target_diff, robot, right_joints)

        _apply_ik(robot, resp, right_joints)

        for obj in BulletWorld.current_bullet_world.objects:
            if obj.name == "floor":
                continue
            in_contact, contact_links = contact(robot, obj, return_links=True)
            allowed_links = allowed_collision[obj] if obj in allowed_collision.keys() else []

            if in_contact:
                for link in contact_links:

                    if link[0] in allowed_robot_links or link[1] in allowed_links:
                        in_contact = False

        if not in_contact:
            arms.append("right")
            res = True
    except IKError:
        pass

    return res, arms
