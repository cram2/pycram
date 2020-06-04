import rospy
import pybullet as p
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose


def _get_position_for_joints(robot, joints):
    return list(map(lambda x: p.getJointState(robot.id, robot.get_joint_id(x))[0], joints))

def _make_request_msg(root_link, tip_link, target_pose, robot_object, joints):
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
    robot_state.joint_state = joint_state

    msg_request = PositionIKRequest()
    msg_request.group_name = "arm"
    msg_request.ik_link_name = tip_link
    msg_request.pose_stamped = pose_sta
    msg_request.avoid_collisions = True
    msg_request.timeout = rospy.Duration(secs=1000)
    msg_request.attempts = 100

    return msg_request

def request_ik(root_link, tip_link, target_pose_and_rotation, robot_object, joints):
    rospy.init_node('listener', anonymous=True)
    rospy.wait_for_service('/kdl_ik_service/get_ik')

    req = _make_request_msg(root_link, tip_link, target_pose_and_rotation, robot_object, joints)

    ik = rospy.ServiceProxy('/kdl_ik_service/get_ik', GetPositionIK)
    resp = ik(req)
    return(resp.solution.joint_state.position)
