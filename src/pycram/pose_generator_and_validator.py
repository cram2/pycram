import tf
import numpy as np
import rospy
import pybullet as p

from .bullet_world import Object
from .robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from .external_interfaces.ik import _make_request_msg
from .helper import _transform_to_torso
from moveit_msgs.srv import GetPositionIK


def pose_generator(costmap):
    """
    A generator that crates pose candidates from a given costmap. The generator
    selects the highest 100 values and returns the corresponding positions.
    Orientations are calculated such that the Robot faces the center of the costmap.
    :param costmap: The costmap from which poses should be sampled.
    :Yield: A tuple of position and orientation
    """
    # Determines how many positions should be sampled from the costmap
    number_of_samples = 100
    indices = np.argpartition(costmap.map.flatten(), -100)[-100:]
    #indices = np.argsort(costmap.map.flatten())[-number_of_samples:]
    indices = np.dstack(np.unravel_index(indices, costmap.map.shape)).reshape(number_of_samples, 2)
    size = costmap.map.shape[0]
    for ind in indices:
        if costmap.map[ind[0]][ind[1]] == 0:
            continue
        # The "-1" in the Formaula is needed since the logical coordinate system
        # is inverted against the one in the BulletWorld.
        # This is the logical assumption of the Costmap
        # ---------
        # |    x  |
        # |   --> |
        # |  |    |
        # |  | y  |
        # --------
        # And this is the coordinate system in BulletWorld. Both have the same rotation
        # ----------
        # |   | y  |
        # |   |    |
        # |<--     |
        # | x      |
        # ----------
        position = [(ind[0]- size/2) *-1 * costmap.resolution + costmap.origin[0], (ind[1] - size/2) * -1 * costmap.resolution + costmap.origin[1], 0]
        orientation = generate_orientation(position, costmap.origin)
        print(f"Indicies: {ind}, Position: {position}")
        yield (list(position), orientation)

def height_generator():
    pass


def generate_orientation(position, origin):
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
    angle = np.arctan2(position[1]-origin[1],position[0]-origin[0]) + np.pi
    quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
    return quaternion

def visibility_validator(pose, robot, object_or_pose, world):
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
        camera_pose = robot.get_link_position_and_orientation(robot_description.i.get_camera_frame())
        robot.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])
        ray = p.rayTest(camera_pose[0], object_or_pose.get_position(), physicsClientId=world.client_id)
        res = ray[0][0] == object_or_pose.id
        #res = visible(object_or_pose, camera_pose, world=world)
    else:
        robot.set_position_and_orientation(pose[0], pose[1])
        camera_pose = robot.get_link_position_and_orientation(robot_description.i.get_camera_frame())
        robot.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])
        ray = p.rayTest(camera_pose[0], object_or_pose, physicsClientId=world.client_id)
        res = ray[0][0] == -1
    robot.set_position_and_orientation(robot_pose[0], robot_pose[1])
    return res


def reachability_validator(pose, robot, target, world):
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
    :return: True if the target is reachable for the robot and False in any other
        case.
    """
    if type(target) == Object:
        target = target.get_position()

    robot_pose = robot.get_position_and_orientation()
    robot.set_position_and_orientation(pose[0], pose[1])

    left_gripper = robot_description.i.get_tool_frame('left')
    right_gripper = robot_description.i.get_tool_frame('right')
    left_joints = joints = robot_description.i._safely_access_chains('left').joints
    right_joints = joints = robot_description.i._safely_access_chains('right').joints
    # TODO Make orientation adhere to grasping orientation
    target_torso = _transform_to_torso([target, [0, 0, 0, 1]], robot)

    ik_msg_left = _make_request_msg(robot_description.i.base_frame, left_gripper, target_torso, robot, left_joints)

    rospy.wait_for_service('/kdl_ik_service/get_ik')
    ik = rospy.ServiceProxy('/kdl_ik_service/get_ik', GetPositionIK)
    resp = ik(ik_msg_left)

    if resp.error_code.val == -31:
        ik_msg_right = _make_request_msg(robot_description.i.base_frame, right_gripper, target_torso, robot, right_joints)
        resp = ik(ik_msg_right)
        #res = resp.error_code.val == 1
        res = False
    else:
        res = True
    robot.set_position_and_orientation(robot_pose[0], robot_pose[1])
    return res
