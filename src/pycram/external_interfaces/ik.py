import rospy
import pybullet as p
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


class IKError(Exception):
    def __init__(self, pose):
        self.message = "Position {} is not reachable for end effector".format(pose)
        super(IKError, self).__init__(self.message)

def _get_position_for_joints(robot, joints):
    """
    Returns a list with all joint positions for the joint names specified in
    the joints parameter
    :param robot: The robot the joint states should be taken from
    :param joints: The list of joint names that should be in the output
    :return: A list of joint states according and in the same order as the joint
    names in the joints parameter
    """
    return list(map(lambda x: p.getJointState(robot.id, robot.get_joint_id(x))[0], joints))


def _make_request_msg(root_link, tip_link, target_pose, robot_object, joints):
    """
    This method generates an ik request message for the kdl_ik_service. The message is
     of the type moveit_msgs/PositionIKRequest and contains all information
     contained in the parameter.
    :param root_link: The first link of the chain of joints to be altered
    :param tip_link: The last link of the chain of joints to be altered
    :param target_pose: A list of two lists, the first is the pose in world coordinate frame
                        the second is the orientation as quanternion in world coordinate frame
    :param robot_object: The robot for which the ik should be generated
    :param joints: A list of joint names between the root_link and tip_link that should be altered.
    :return: A moveit_msgs/PositionIKRequest message containing all the information from the parameter
    """
    pose_sta = PoseStamped()
    pose_sta.header.frame_id = root_link
    pose_sta.header.stamp = rospy.Time.now()

    tar_pose = target_pose[0]
    tar_rotation = target_pose[1]

    pose = Pose()
    pose.position.x = tar_pose[0]
    pose.position.y = tar_pose[1]
    pose.position.z = tar_pose[2]
    pose.orientation.x = tar_rotation[0]
    pose.orientation.y = tar_rotation[1]
    pose.orientation.z = tar_rotation[2]
    pose.orientation.w = tar_rotation[3]
    pose_sta.pose = pose

    robot_state = RobotState()
    joint_state = JointState()
    joint_state.name = joints
    joint_state.position = _get_position_for_joints(robot_object, joints)
    joint_state.velocity = [0.0 for x in range(len(joints))]
    joint_state.effort = [0.0 for x in range(len(joints))]
    robot_state.joint_state = joint_state

    msg_request = PositionIKRequest()
    #msg_request.group_name = "arm"
    msg_request.ik_link_name = tip_link
    msg_request.pose_stamped = pose_sta
    msg_request.avoid_collisions = False
    msg_request.robot_state = robot_state
    #msg_request.timeout = rospy.Duration(secs=10000)
    #msg_request.attempts = 1000

    return msg_request

def request_ik(root_link, tip_link, target_pose_and_rotation, robot_object, joints):
    """
    This method sends a request to the kdl_ik_service and returns the solution.
    Note that the robot in robot_object should be identical to the robot description
    uploaded to the parameter server. Furthermore note that the root_link and
    tip_link are the first and last links of the joints that should be altered.
    These joints should also be specified in the joints parameter as a list of names.
    :param root_link: The first link of the chain of joints to be altered
    :param tip_link: The last link in the chain of joints to be altered
    :param target_pose_and_rotation: The target position and orientation as a list
    of two lists were the first is the position in world coordinate frame and the
    second is the orientation as quanternion in world coordinate fraem
    :param robot_object: The robot object for which the ik solution should be generated
    :param joints: A list of joint name that should be altered
    :return: The solution that was generated.
    """
    rospy.wait_for_service('/kdl_ik_service/get_ik')

    req = _make_request_msg(root_link, tip_link, target_pose_and_rotation, robot_object, joints)
    ik = rospy.ServiceProxy('/kdl_ik_service/get_ik', GetPositionIK)
    resp = ik(req)
    if resp.error_code.val == -31:
        raise IKError(target_pose_and_rotation)
    return(resp.solution.joint_state.position)
