from typing import List

import pybullet as p
import rospy
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

from ..bullet_world import Object
from ..helper import calculate_wrist_tool_offset
from ..local_transformer import LocalTransformer
from ..pose import Pose, Transform
from ..robot_descriptions import robot_description
from ..plan_failures import IKError


def _get_position_for_joints(robot, joints):
    """
    Returns a list with all joint positions for the joint names specified in
    the joints parameter

    :param robot: The robot the joint states should be taken from
    :param joints: The list of joint names that should be in the output
    :return: A list of joint states according and in the same order as the joint
    names in the joints parameter
    """
    return list(
        map(lambda x: p.getJointState(robot.id, robot.get_joint_id(x), physicsClientId=robot.world.client_id)[0],
            joints))


def _make_request_msg(root_link: str, tip_link: str, target_pose: Pose, robot_object: Object,
                      joints: List[str]) -> PositionIKRequest:
    """
    Generates an ik request message for the kdl_ik_service. The message is
    of the type moveit_msgs/PositionIKRequest and contains all information
    contained in the parameter.

    :param root_link: The first link of the chain of joints to be altered
    :param tip_link: The last link of the chain of joints to be altered
    :param target_pose: Target pose for which the message should be created
    :param robot_object: The robot for which the ik should be generated
    :param joints: A list of joint names between the root_link and tip_link that should be altered.
    :return: A moveit_msgs/PositionIKRequest message containing all the information from the parameter
    """
    local_transformer = LocalTransformer()
    target_pose = local_transformer.transform_pose(target_pose, robot_object.get_link_tf_frame(root_link))

    robot_state = RobotState()
    joint_state = JointState()
    joint_state.name = joints
    joint_state.position = _get_position_for_joints(robot_object, joints)
    # joint_state.velocity = [0.0 for x in range(len(joints))]
    # joint_state.effort = [0.0 for x in range(len(joints))]
    robot_state.joint_state = joint_state

    msg_request = PositionIKRequest()
    # msg_request.group_name = "arm"
    msg_request.ik_link_name = tip_link
    msg_request.pose_stamped = target_pose
    msg_request.avoid_collisions = False
    msg_request.robot_state = robot_state
    msg_request.timeout = rospy.Duration(secs=1000)
    # msg_request.attempts = 1000

    return msg_request


def call_ik(root_link: str, tip_link: str, target_pose: Pose, robot_object: Object, joints: List[str]) -> List[float]:
    """
   Sends a request to the kdl_ik_service and returns the solution.
   Note that the robot in robot_object should be identical to the robot description
   uploaded to the parameter server. Furthermore, note that the root_link and
   tip_link are the links attached to the first and last joints in the joints list.

   :param root_link: The first link of the chain of joints to be altered
   :param tip_link: The last link in the chain of joints to be altered
   :param target_pose: The target pose in frame of root link
   second is the orientation as quaternion in world coordinate frame
   :param robot_object: The robot object for which the ik solution should be generated
   :param joints: A list of joint name that should be altered
   :return: The solution that was generated as a list of joint values corresponding to the order of joints given
   """
    if robot_description.name == "pr2":
        ik_service = "/pr2_right_arm_kinematics/get_ik" if "r_wrist" in tip_link else "/pr2_left_arm_kinematics/get_ik"
    else:
        ik_service = "/kdl_ik_service/get_ik"

    rospy.wait_for_service(ik_service)

    req = _make_request_msg(root_link, tip_link, target_pose, robot_object, joints)
    req.pose_stamped.header.frame_id = root_link
    ik = rospy.ServiceProxy(ik_service, GetPositionIK)
    try:
        resp = ik(req)
    except rospy.ServiceException as e:
        if robot_description.name == "pr2":
            raise IKError(target_pose, root_link)
        else:
            raise e

    if resp.error_code.val == -31:
        raise IKError(target_pose, root_link)

    return resp.solution.joint_state.position


def request_ik(target_pose: Pose, robot: Object, joints: List[str], gripper: str) -> List[float]:
    """
    Top-level method to request ik solution for a given pose. Before calling the ik service the links directly before
    and after the joint chain will be queried and the target_pose will be transformed into the frame of the root_link.
    Afterward, the offset between the tip_link and end effector will be calculated and taken into account. Lastly the
    ik service is called and the result returned

    :param target_pose: Pose for which an ik solution should be found
    :param robot: Robot object which should be used
    :param joints: List of joints that should be used in computation
    :param gripper: Name of the gripper which should grasp, this should be at the end of the given joint chain
    :return: A list of joint values
    """
    local_transformer = LocalTransformer()
    base_link = robot_description.get_parent(joints[0])
    # Get link after last joint in chain
    end_effector = robot_description.get_child(joints[-1])

    target_torso = local_transformer.transform_pose(target_pose, robot.get_link_tf_frame(base_link))
    # target_torso = _transform_to_torso(pose, shadow_robot)

    diff = calculate_wrist_tool_offset(end_effector, gripper, robot)
    target_diff = target_torso.to_transform("target").inverse_times(diff).to_pose()

    inv = call_ik(base_link, end_effector, target_diff, robot, joints)

    return inv

