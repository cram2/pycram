import numpy as np

from .. import tf_transformations
from typing_extensions import List, Union, Tuple, Dict

from ..ros import get_node_names
from ..ros import  Duration, ServiceException
from ..ros import  loginfo_once, logerr
from ..ros import  get_service_proxy, wait_for_service
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

from ..datastructures.world import World, UseProspectionWorld
from ..world_concepts.world_object import Object
from ..utils import _apply_ik
from ..local_transformer import LocalTransformer
from ..datastructures.pose import PoseStamped
from ..robot_description import RobotDescription
from ..failures import IKError
from ..external_interfaces.giskard import projection_cartesian_goal, allow_gripper_collision
from .pinocchio_ik import compute_ik


def _make_request_msg(root_link: str, tip_link: str, target_pose: PoseStamped, robot_object: Object,
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
    msg_request.timeout = Duration(1000)
    # msg_request.attempts = 1000

    return msg_request


def call_ik(root_link: str, tip_link: str, target_pose: PoseStamped, robot_object: Object, joints: List[str]) -> List[float]:
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
    if RobotDescription.current_robot_description.name == "pr2":
        ik_service = "/pr2_right_arm_kinematics/get_ik" if "r_wrist" in tip_link else "/pr2_left_arm_kinematics/get_ik"
    else:
        ik_service = "/kdl_ik_service/get_ik"

    loginfo_once(f"Waiting for IK service: {ik_service}")
    wait_for_service(ik_service)

    req = _make_request_msg(root_link, tip_link, target_pose, robot_object, joints)
    req.pose_stamped.header.frame_id = root_link
    ik = get_service_proxy(ik_service, GetPositionIK)
    try:
        resp = ik(req)
    except ServiceException as e:
        if RobotDescription.current_robot_description.name == "pr2":
            raise IKError(target_pose, root_link, tip_link)
        else:
            raise e

    if resp.error_code.val == -31:
        raise IKError(target_pose, root_link, tip_link)

    return resp.solution.joint_state.position


def try_to_reach_with_grasp(pose_or_object: Union[PoseStamped, Object],
                            prospection_robot: Object, gripper_name: str,
                            grasp_quaternion: List[float]) -> Union[PoseStamped, None]:
    """
    Checks if the robot can reach a given position with a specific grasp orientation.
    To determine this the inverse kinematics are calculated and applied.

    :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
    :param prospection_robot: The robot that should reach for the position
    :param gripper_name: The name of the end effector
    :param grasp_quaternion: The orientation of the grasp
    """

    input_pose = pose_or_object.get_pose() if isinstance(pose_or_object, Object) else pose_or_object

    target_pose = apply_grasp_orientation_to_pose(grasp_quaternion, input_pose)

    return try_to_reach(target_pose, prospection_robot, gripper_name)


def apply_grasp_orientation_to_pose(grasp_orientation: List[float], pose: PoseStamped) -> PoseStamped:
    """
    Applies the orientation of a grasp to a given pose. This is done by using the grasp orientation
    of the given grasp and applying it to the given pose.

    :param grasp_orientation: The orientation of the grasp
    :param pose: The pose to which the grasp orientation should be applied
    """
    local_transformer = LocalTransformer()
    target_map = local_transformer.transform_pose(pose, "map")
    target_map.orientation.x = grasp_orientation[0]
    target_map.orientation.y = grasp_orientation[1]
    target_map.orientation.z = grasp_orientation[2]
    target_map.orientation.w = grasp_orientation[3]
    return target_map


def try_to_reach(pose_or_object: Union[PoseStamped, Object], prospection_robot: Object,
                 gripper_name: str) -> Union[PoseStamped, None]:
    """
    Tries to reach a given position with a given robot. This is done by calculating the inverse kinematics.

    :param pose_or_object: The position and rotation or Object for which reachability should be checked.
    :param prospection_robot: The robot that should be used to check for reachability, should be the one in the prospection world
    :param gripper_name: Name of the gripper tool frame
    :return: The pose at which the robot should stand or None if the target is not reachable
    """
    input_pose = pose_or_object.get_pose() if isinstance(pose_or_object, Object) else pose_or_object

    arm_chain = list(filter(lambda chain: chain.get_tool_frame() == gripper_name, RobotDescription.current_robot_description.get_manipulator_chains()))[0]

    joints = arm_chain.joints

    try:
        inv = request_ik(input_pose, prospection_robot, joints, gripper_name)
    except IKError as e:
        logerr(f"Pose is not reachable: {e}")
        return None
    _apply_ik(prospection_robot, inv)

    return input_pose


def request_ik(target_pose: PoseStamped, robot: Object, joints: List[str], gripper: str) -> Tuple[PoseStamped, Dict[str, float]]:
    """
    Top-level method to request ik solution for a given pose. This method will check if the giskard node is running
    and if so will call the giskard service. If the giskard node is not running the kdl_ik_service will be called.

    :param target_pose: Pose of the end-effector for which an ik solution should be found
    :param robot: The robot object which should be used
    :param joints: A list of joints that should be used in computation, this is only relevant for the kdl_ik_service
    :param gripper: Name of the tool frame which should grasp, this should be at the end of the given joint chain
    :return: A Pose at which the robt should stand as well as a dictionary of joint values
    """
    if "/giskard" not in get_node_names():
        return robot.pose, request_pinocchio_ik(target_pose, robot, gripper, joints)
        # return robot.pose, request_kdl_ik(target_pose, robot, joints, gripper)
    return request_giskard_ik(target_pose, robot, gripper)


def request_kdl_ik(target_pose: PoseStamped, robot: Object, joints: List[str], gripper: str) -> Dict[str, float]:
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
    base_link = RobotDescription.current_robot_description.get_parent(joints[0])
    # Get link after last joint in chain
    end_effector = RobotDescription.current_robot_description.get_child(joints[-1])

    target_torso = local_transformer.transform_pose(target_pose, robot.get_link_tf_frame(base_link))

    wrist_tool_frame_offset = robot.get_transform_between_links(end_effector, gripper)
    target_diff = target_torso.to_transform("target").inverse_times(wrist_tool_frame_offset).to_pose()

    inv = call_ik(base_link, end_effector, target_diff, robot, joints)

    return dict(zip(joints, inv))


def request_giskard_ik(target_pose: PoseStamped, robot: Object, gripper: str) -> Tuple[PoseStamped, Dict[str, float]]:
    """
    Calls giskard in projection mode and queries the ik solution for a full body ik solution. This method will
    try to drive the robot directly to a pose from which the target_pose is reachable for the end effector. If there
    are obstacles in the way this method will fail. In this case please use the GiskardLocation designator.

    :param target_pose: Pose at which the end effector should be moved.
    :param robot: Robot object which should be used.
    :param gripper: Name of the tool frame which should grasp, this should be at the end of the given joint chain.
    :return: A list of joint values.
    """
    loginfo_once(f"Using Giskard for full body IK")
    local_transformer = LocalTransformer()
    target_map = local_transformer.transform_pose(target_pose, "map")

    allow_gripper_collision("all")
    result = projection_cartesian_goal(target_map, gripper, "map")
    last_point = result.trajectory.points[-1]
    joint_names = result.trajectory.joint_names

    joint_states = dict(zip(joint_names, last_point.positions))
    prospection_robot = World.current_world.get_prospection_object_for_object(robot)

    orientation = list(tf_transformations.quaternion_from_euler(0, 0, joint_states["brumbrum_yaw"], axes="sxyz"))
    pose = PoseSteamped.from_list([joint_states["brumbrum_x"], joint_states["brumbrum_y"], 0], orientation)

    robot_joint_states = {}
    for joint_name, state in joint_states.items():
        if joint_name in robot.joints.keys():
            robot_joint_states[joint_name] = state

    with UseProspectionWorld():
        prospection_robot.set_multiple_joint_positions(robot_joint_states)
        prospection_robot.set_pose(pose)

        tip_pose = prospection_robot.get_link_pose(gripper)
        dist = tip_pose.dist(target_map)

        if dist > 0.01:
            raise IKError(target_pose, "map", gripper)
        return pose, robot_joint_states

def request_pinocchio_ik(target_pose: PoseStamped, robot: Object, target_link: str, joints: List[str]) -> Dict[str, float]:
    """
    Calls the pinocchio ik solver to calculate the ik solution for a given target link and pose.

    :param target_link: The target link for which the ik solution should be calculated
    :param target_pose: The target pose for which the ik solution should be calculated
    :param robot: The robot object for which the ik solution should be calculated
    :param joints: The joints that should be used in the calculation
    :return: A dictionary containing the joint names and joint values
    """
    lt = LocalTransformer()
    target_pose = lt.transform_pose(target_pose, robot.tf_frame)

    # Get link after last joint in chain
    wrist_link = RobotDescription.current_robot_description.get_child(joints[-1])

    # target_torso = lt.transform_pose(target_pose, robot.get_link_tf_frame(base_link))

    wrist_tool_frame_offset = robot.get_transform_between_links(wrist_link, target_link)
    target_diff = target_pose.to_transform("target").inverse_times(wrist_tool_frame_offset).to_pose()
    target_diff.round()

    res = compute_ik(wrist_link, target_diff, robot)

    return res