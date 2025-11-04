import numpy as np
from semantic_digital_twin.world_description.world_entity import Body

from .. import tf_transformations
from typing_extensions import List, Union, Tuple, Dict

from ..ros import get_node_names
from ..ros import  Duration, ServiceException
from ..logging import  loginfo_once, logerr
from ..utils import _apply_ik
from ..datastructures.pose import PoseStamped
from ..robot_description import RobotDescription
from ..failures import IKError
from ..external_interfaces.giskard import projection_cartesian_goal, allow_gripper_collision
from .pinocchio_ik import compute_ik
from ..datastructures.enums import Arms

try:
    from moveit_msgs.msg import PositionIKRequest
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionIK
    from sensor_msgs.msg import JointState
except ImportError:
    pass


def try_to_reach_with_grasp(pose_or_object: Union[PoseStamped, Body],
                            prospection_robot: Body, gripper_name: str,
                            grasp_quaternion: List[float]) -> Union[PoseStamped, None]:
    """
    Checks if the robot can reach a given position with a specific grasp orientation.
    To determine this the inverse kinematics are calculated and applied.

    :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
    :param prospection_robot: The robot that should reach for the position
    :param gripper_name: The name of the end effector
    :param grasp_quaternion: The orientation of the grasp
    """

    input_pose = pose_or_object.get_pose() if isinstance(pose_or_object, Body) else pose_or_object

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


def try_to_reach(pose_or_object: Union[PoseStamped, Body], prospection_robot: Body,
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


def request_ik(target_pose: PoseStamped, robot: Body, joints: List[str], gripper: str) -> Tuple[PoseStamped, Dict[str, float]]:
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

def request_giskard_ik(target_pose: PoseStamped, robot: Body, gripper: str) -> Tuple[PoseStamped, Dict[str, float]]:
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

    allow_gripper_collision(Arms.BOTH)
    result = projection_cartesian_goal(target_map, gripper, "map")
    last_point = result.trajectory.points[-1]
    joint_names = result.trajectory.joint_names

    joint_states = dict(zip(joint_names, last_point.positions))
    prospection_robot = World.current_world.get_prospection_object_for_object(robot)

    orientation = list(tf_transformations.quaternion_from_euler(0, 0, joint_states["brumbrum_yaw"], axes="sxyz"))
    pose = PoseStamped.from_list([joint_states["brumbrum_x"], joint_states["brumbrum_y"], 0], orientation)

    robot_joint_states = {}
    for joint_name, state in joint_states.items():
        if joint_name in robot.joints.keys():
            robot_joint_states[joint_name] = state

    with UseProspectionWorld():
        prospection_robot.set_multiple_joint_positions(robot_joint_states)
        prospection_robot.set_pose(pose)

        tip_pose = prospection_robot.get_link_pose(gripper)
        dist = tip_pose.position.euclidean_distance(target_map.position)

        if dist > 0.01:
            raise IKError(target_map, 'map', gripper, World.robot.get_pose(), robot.get_positions_of_all_joints())
        return pose, robot_joint_states

def request_pinocchio_ik(target_pose: PoseStamped, robot: Body, target_link: str, joints: List[str]) -> Dict[str, float]:
    """
    Calls the pinocchio ik solver to calculate the ik solution for a given target link and pose.

    :param target_link: The target link for which the ik solution should be calculated
    :param target_pose: The target pose for which the ik solution should be calculated
    :param robot: The robot object for which the ik solution should be calculated
    :param joints: The joints that should be used in the calculation
    :return: A dictionary containing the joint names and joint values
    """
    lt = LocalTransformer()
    target_pose_map = lt.transform_pose(target_pose, "map")
    target_pose = lt.transform_pose(target_pose_map, robot.tf_frame)

    # Get link after last joint in chain
    wrist_link = RobotDescription.current_robot_description.get_child(joints[-1])

    wrist_tool_frame_offset = robot.get_transform_between_links(wrist_link, target_link)
    target_diff = target_pose.to_transform_stamped("target").inverse_times(wrist_tool_frame_offset).to_pose_stamped()
    target_diff.round()

    try:
        res = compute_ik(wrist_link, target_diff, robot)
    except IKError:
        raise IKError(target_pose_map, 'map', target_link, robot.get_pose(), robot.get_positions_of_all_joints())

    return res