import tf
import numpy as np
import rospy
from .bullet_world_reasoning import visible, reachable
import pybullet as p
from .bullet_world import Object
from .robot_description import InitializedRobotDescription as robot_description
from .ik import _make_request_msg
from .helper import _transform_to_torso

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


def pose_generator(costmap):
    indices = np.argpartition(costmap.map.flatten(), -100)[-100:]
    indices = np.dstack(np.unravel_index(indices, costmap.map.shape)).reshape(100, 2)
    size = costmap.map.shape[0]
    for ind in indices:
        position = [(ind[0]- size/2) * costmap.resolution, (ind[1] - size/2) * costmap.resolution, 0]
        orientation = generate_orientation(position, costmap.origin)
        yield (list(position), orientation)

def height_generator():
    pass


def generate_orientation(position, origin):
    angle = np.arctan2(position[1]-origin[1],position[0]-origin[0]) + np.pi
    quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
    return quaternion

def visibility_validator(pose, robot, object_or_pose, world):
    robot_pose = robot.get_position_and_orientation()
    if type(object_or_pose) == Object:
        robot.set_position_and_orientation(pose[0], pose[1])
        camera_pose = robot.get_link_position_and_orientation(robot_description.i.get_camera_frame())
        res = visible(object_or_pose, camera_pose, world=world)
    else:
        robot.set_position_and_orientation(pose[0], pose[1])
        camera_pose = robot.get_link_position_and_orientation(robot_description.i.get_camera_frame())
        robot.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])
        ray = p.rayTest(camera_pose[0], object_or_pose, physicsClientId=world.client_id)
        res = ray[0][0] == -1
    robot.set_position_and_orientation(robot_pose[0], robot_pose[1])
    return res


def reachability_validator(pose, robot, target, world):
    robot_pose = robot.get_position_and_orientation()
    robot.set_position_and_orientation(pose[0], pose[1])

    left_gripper = robot_description.i.get_tool_frame('left')
    right_gripper = robot_description.i.get_tool_frame('right')
    left_joints = joints = robot_description.i._safely_access_chains('left').joints
    right_joints = joints = robot_description.i._safely_access_chains('right').joints
    target_torso = _transform_to_torso([target, [0, 0, 0, 1]], robot)

    ik_msg_left = _make_request_msg(robot_description.i.base_frame, left_gripper, target_torso, robot, left_joints)

    rospy.wait_for_service('/kdl_ik_service/get_ik')
    ik = rospy.ServiceProxy('/kdl_ik_service/get_ik', GetPositionIK)
    resp = ik(ik_msg_left)

    if resp.error_code.val == -31:
        ik_msg_right = _make_request_msg(robot_description.i.base_frame, right_gripper, target_torso, robot, right_joints)
        resp = ik(ik_msg_right)
        print(resp.error_code.val)
        res = resp.error_code.val == 1
    else:
        res = True
    print(pose)
    #print(res)
    robot.set_position_and_orientation(robot_pose[0], robot_pose[1])
    return res
