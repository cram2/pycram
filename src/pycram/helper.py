"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
from numbers import Number
from macropy.core.quotes import macros, ast_literal, q
import pybullet as p
from geometry_msgs.msg import Point, Quaternion, Pose, Transform, PoseStamped, TransformStamped, Vector3
from .robot_description import InitializedRobotDescription as robot_description


import rospy
from std_msgs.msg import Header
from time import time as current_time


class bcolors:
    """
    Color codes which can be used to highlight Text in the Terminal. For example,
    for warnings.
    Usage:
    Firstly import the class into the file.
    print(f'{bcolors.WARNING} Some Text {bcolors.ENDC}')
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def _apply_ik(robot, joint_poses, gripper):
    """
    Apllies a list of joint poses calculated by an inverse kinematics solver to a robot
    :param robot: The robot the joint poses should be applied on
    :param joint_poses: The joint poses to be applied
    :param gripper: specifies the gripper for which the ik solution should be applied
    :return: None
    """
    arm ="left" if gripper == robot_description.i.get_tool_frame("left") else "right"
    ik_joints = [robot_description.i.torso_joint] + robot_description.i._safely_access_chains(arm).joints
    #ik_joints = robot_description.i._safely_access_chains(arm).joints
    for i in range(0, len(ik_joints)):
        robot.set_joint_state(ik_joints[i], joint_poses[i])


def _transform_to_torso(pose_and_rotation, robot):
    #map_T_torso = robot.get_link_position_and_orientation("base_footprint")
    map_T_torso = robot.get_position_and_orientation()
    torso_T_map = p.invertTransform(map_T_torso[0], map_T_torso[1])
    map_T_target = pose_and_rotation
    torso_T_target = p.multiplyTransforms(torso_T_map[0], torso_T_map[1], map_T_target[0], map_T_target[1])
    return torso_T_target

def transform(pose,
              transformation):  # TODO: if pose is a list of position and orientation calculate new pose w/ orientation too
    res = [0, 0, 0]
    for i in range(0, 3):
        res[i] = pose[i] - transformation[i]
    return res


def pose_stamped2tuple(pose_stamped):
    if type(pose_stamped) is PoseStamped:
        p = pose_stamped.pose.position
        o = pose_stamped.pose.orientation
        return tuple(((p.x, p.y, p.z), (o.x, o.y, o.z, o.w)))


def is_list_position(list_pos):
    return len(list_pos) == 3 and all(isinstance(x, Number) for x in list_pos)


def is_list_pose(list_pose):
    if len(list_pose) == 2 and is_list_position(list_pose[0]):
        return len(list_pose[1]) == 4 and all(isinstance(x, Number) for x in list_pose[1])


def list2point_and_quaternion(pose_list):
    if len(pose_list) == 2 and len(pose_list[0]) == 3 and len(pose_list[1]) == 4:
        pos = pose_list[0]
        orient = pose_list[1]
        point = Point(pos[0], pos[1], pos[2])
        quaternion = Quaternion(orient[0], orient[1], orient[2], orient[3])
        return point, quaternion


def list2vector3_and_quaternion(pose_list):
    if len(pose_list) == 2 and len(pose_list[0]) == 3 and len(pose_list[1]) == 4:
        pos = pose_list[0]
        orient = pose_list[1]
        vector = Vector3(pos[0], pos[1], pos[2])
        quaternion = Quaternion(orient[0], orient[1], orient[2], orient[3])
        return vector, quaternion


def list2point(pos_list):
    if len(pos_list) == 3:
        return Point(pos_list[0], pos_list[1], pos_list[2])


def list2pose(pose_list):
    p, q = list2point_and_quaternion(pose_list)
    if p and q:
        return Pose(p, q)


def ensure_pose(pose):
    if type(pose) is Pose:
        return pose
    elif (type(pose) is list or type(pose) is tuple) and is_list_pose(pose):
        pose = list2pose(pose)
        return pose
    elif (type(pose) is list or type(pose) is tuple) and is_list_position(pose):
        point = list2point(pose)
        return Pose(point, Quaternion(0, 0, 0, 1))
    else:
        rospy.logerr("(helper) Cannot convert pose since it is no Pose object or valid list pose.")
        return None


def list2tf(pose_list):
    p, q = list2vector3_and_quaternion(pose_list)
    if p and q:
        return Transform(p, q)


def pose2tf(pose):
    if pose and pose.position and pose.orientation:
        p = pose.position
        return Transform(Vector3(p.x, p.y, p.z), pose.orientation)


def list2tfstamped(source_frame, target_frame, pose_list, time=None):
    tf = list2tf(pose_list)
    if tf:
        return tf2tfstamped(source_frame, target_frame, tf, time)


def pose2tfstamped(source_frame, target_frame, pose, time=None):
    tf = pose2tf(pose)
    if tf:
        return tf2tfstamped(source_frame, target_frame, tf, time)


def tf2tfstamped(source_frame, target_frame, tf, time=None):
    tf_time = time if time else rospy.Time(current_time())
    header = Header(0, tf_time, source_frame)
    return TransformStamped(header, target_frame, tf)


def _block(tree):
    """Wrap multiple statements into a single block and return it.

    If macros themselves are not a single statement, they can't always be nested (for example inside the par macro which executes each statement in an own thread).

    Arguments:
    tree -- the tree containing the statements.
    """
    with q as new_tree:
        # Wrapping the tree into an if block which itself is a statement that contains one or more statements.
        # The condition is just True and therefor makes sure that the wrapped statements get executed.
        if True:
            ast_literal[tree]

    return new_tree


class GeneratorList:
    """Implementation of generator list wrappers.

    Generator lists store the elements of a generator, so these can be fetched multiple times.

    Methods:
    get -- get the element at a specific index.
    has -- check if an element at a specific index exists.
    """

    def __init__(self, generator):
        """Create a new generator list.

        Arguments:
        generator -- the generator to use.
        """
        if isgeneratorfunction(generator):
            self._generator = generator()
        else:
            self._generator = generator

        self._generated = []

    def get(self, index=0):
        """Get the element at a specific index or raise StopIteration if it doesn't exist.

        Arguments:
        index -- the index to get the element of.
        """
        while len(self._generated) <= index:
            self._generated.append(next(self._generator))

        return self._generated[index]

    def has(self, index):
        """Check if an element at a specific index exists and return True or False.

        Arguments:
        index -- the index to check for.
        """
        try:
            self.get(index)
            return True
        except StopIteration:
            return False
