from typing_extensions import List, Union

import rospy
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

from ..world_concepts.world_object import Object
from ..helper import calculate_wrist_tool_offset, _apply_ik
from ..local_transformer import LocalTransformer
from ..datastructures.pose import Pose
from ..robot_descriptions import robot_description
from ..plan_failures import IKError


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
    joint_state.position = [robot_object.get_joint_position(joint) for joint in joints]
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
   :param target_pose: The target pose in frame of root link second is the orientation as quaternion in world coordinate frame
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


def try_to_reach_with_grasp(pose_or_object: Union[Pose, Object],
                            prospection_robot: Object, gripper_name: str,
                            grasp: str) -> Union[Pose, None]:
    """
    Checks if the robot can reach a given position with a specific grasp orientation.
    To determine this the inverse kinematics are calculated and applied.

    :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
    :param prospection_robot: The robot that should reach for the position
    :param gripper_name: The name of the end effector
    :param grasp: The grasp type with which the object should be grasped
    """

    input_pose = pose_or_object.get_pose() if isinstance(pose_or_object, Object) else pose_or_object

    target_pose = apply_grasp_orientation_to_pose(grasp, input_pose)

    return try_to_reach(target_pose, prospection_robot, gripper_name)


def apply_grasp_orientation_to_pose(grasp: str, pose: Pose) -> Pose:
    """
    Applies the orientation of a grasp to a given pose. This is done by using the grasp orientation
    of the given grasp and applying it to the given pose.

    :param grasp: The name of the grasp
    :param pose: The pose to which the grasp orientation should be applied
    """
    local_transformer = LocalTransformer()
    target_map = local_transformer.transform_pose(pose, "map")
    grasp_orientation = robot_description.grasps.get_orientation_for_grasp(grasp)
    target_map.orientation.x = grasp_orientation[0]
    target_map.orientation.y = grasp_orientation[1]
    target_map.orientation.z = grasp_orientation[2]
    target_map.orientation.w = grasp_orientation[3]
    return target_map


def try_to_reach(pose_or_object: Union[Pose, Object], prospection_robot: Object,
                 gripper_name: str) -> Union[Pose, None]:
    input_pose = pose_or_object.get_pose() if isinstance(pose_or_object, Object) else pose_or_object

    arm = "left" if gripper_name == robot_description.get_tool_frame("left") else "right"
    joints = robot_description.chains[arm].joints

    try:
        inv = request_ik(input_pose, prospection_robot, joints, gripper_name)
    except IKError as e:
        rospy.logerr(f"Pose is not reachable: {e}")
        return None
    _apply_ik(prospection_robot, inv, joints)

    return input_pose


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

    diff = calculate_wrist_tool_offset(end_effector, gripper, robot)
    target_diff = target_torso.to_transform("target").inverse_times(diff).to_pose()

    inv = call_ik(base_link, end_effector, target_diff, robot, joints)

    return inv
