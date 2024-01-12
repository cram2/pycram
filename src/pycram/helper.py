"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
from typing import List
from typing import Tuple, Callable

import numpy as np
import pybullet as p
from pytransform3d.rotations import quaternion_wxyz_from_xyzw, quaternion_xyzw_from_wxyz
from pytransform3d.transformations import transform_from_pq, transform_from, pq_from_transform

from .bullet_world import Object as BulletWorldObject
from .local_transformer import LocalTransformer
from .pose import Transform, Pose
from .robot_descriptions import robot_description
import os
import math


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


def _apply_ik(robot: BulletWorldObject, joint_poses: List[float], joints: List[str]) -> None:
    """
    Apllies a list of joint poses calculated by an inverse kinematics solver to a robot

    :param robot: The robot the joint poses should be applied on
    :param joint_poses: The joint poses to be applied
    :param gripper: specifies the gripper for which the ik solution should be applied
    :return: None
    """
    # arm ="left" if gripper == robot_description.get_tool_frame("left") else "right"
    # ik_joints = [robot_description.torso_joint] + robot_description._safely_access_chains(arm).joints
    # ik_joints = robot_description._safely_access_chains(arm).joints
    robot.set_joint_states(dict(zip(joints, joint_poses)))
    # for i in range(0, len(joints)):
    #     robot.set_joint_state(joints[i], joint_poses[i])


def _transform_to_torso(pose_and_rotation: Tuple[List[float], List[float]], robot: BulletWorldObject) -> Tuple[
    List[float], List[float]]:
    # map_T_torso = robot.get_link_position_and_orientation("base_footprint")
    # map_T_torso = robot.get_position_and_orientation()
    map_T_torso = robot.get_link_pose(robot_description.torso_link).to_list()
    torso_T_map = p.invertTransform(map_T_torso[0], map_T_torso[1])
    map_T_target = pose_and_rotation
    torso_T_target = p.multiplyTransforms(torso_T_map[0], torso_T_map[1], map_T_target[0], map_T_target[1])
    return torso_T_target


def calculate_wrist_tool_offset(wrist_frame: str, tool_frame: str, robot: BulletWorldObject) -> Transform:
    local_transformer = LocalTransformer()
    tool_pose = robot.get_link_pose(tool_frame)
    wrist_to_tool = local_transformer.transform_pose(tool_pose, robot.get_link_tf_frame(wrist_frame))
    return wrist_to_tool.to_transform(robot.get_link_tf_frame(tool_frame))


def inverseTimes(transform1: Tuple[List[float], List[float]], transform2: Tuple[List[float], List[float]]) -> Tuple[
    List[float], List[float]]:
    """
    Like a Minus for Transforms, this subtracts the second transform from the first.
    """
    inv = p.invertTransform(transform2[0], transform2[1])
    return p.multiplyTransforms(transform1[0], transform1[1], inv[0], inv[1])


def transform(pose: List[float],
              transformation: List[float],
              local_coords=False):  # TODO: if pose is a list of position and orientation calculate new pose w/ orientation too
    input_has_rotation = len(pose) == 7
    transformation_has_rotation = len(transformation) == 7
    pose_tf = transform_from_pq(
        np.concatenate((pose[:3], quaternion_wxyz_from_xyzw(pose[3:])))) if input_has_rotation else transform_from(
        np.eye(3), pose)
    transformation_tf = transform_from_pq(np.concatenate((transformation[:3], quaternion_wxyz_from_xyzw(
        transformation[3:])))) if transformation_has_rotation else transform_from(np.eye(3), transformation)
    if local_coords:
        res = pose_tf @ transformation_tf
    else:
        res = transformation_tf @ pose_tf
    res = pq_from_transform(res)
    res[3:] = quaternion_xyzw_from_wxyz(res[3:])
    if not input_has_rotation:
        return res[:3].tolist()
    return res.tolist()


class GeneratorList:
    """Implementation of generator list wrappers.

    Generator lists store the elements of a generator, so these can be fetched multiple times.

    Methods:
    get -- get the element at a specific index.
    has -- check if an element at a specific index exists.
    """

    def __init__(self, generator: Callable):
        """Create a new generator list.

        Arguments:
        generator -- the generator to use.
        """
        if isgeneratorfunction(generator):
            self._generator = generator()
        else:
            self._generator = generator

        self._generated = []

    def get(self, index: int = 0):
        """Get the element at a specific index or raise StopIteration if it doesn't exist.

        Arguments:
        index -- the index to get the element of.
        """
        while len(self._generated) <= index:
            self._generated.append(next(self._generator))

        return self._generated[index]

    def has(self, index: int) -> bool:
        """Check if an element at a specific index exists and return True or False.

        Arguments:
        index -- the index to check for.
        """
        try:
            self.get(index)
            return True
        except StopIteration:
            return False


def axis_angle_to_quaternion(axis: List, angle: float) -> Tuple:
    """
    Convert axis-angle to quaternion.

    :param axis: (x, y, z) tuple representing rotation axis.
    :param angle: rotation angle in degree
    :return: The quaternion representing the axis angle
    """
    angle = math.radians(angle)
    axis_length = math.sqrt(sum([i ** 2 for i in axis]))
    normalized_axis = tuple(i / axis_length for i in axis)

    x = normalized_axis[0] * math.sin(angle / 2)
    y = normalized_axis[1] * math.sin(angle / 2)
    z = normalized_axis[2] * math.sin(angle / 2)
    w = math.cos(angle / 2)

    return (x, y, z, w)


def multiply_quaternions(q1: List, q2: List) -> List:
    """
    Multiply two quaternions using the robotics convention (x, y, z, w).

    :param q1: The first quaternion
    :param q2: The second quaternion
    :return: The quaternion resulting from the multiplication
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return (x, y, z, w)


def quaternion_rotate(q: List, v: List) -> List:
    """
    Rotate a vector v using quaternion q.

    :param q: A quaternion of how v should be rotated
    :param v: A vector that should be rotated by q
    :return: V rotated by Q as a quaternion
    """
    q_conj = (-q[0], -q[1], -q[2], q[3])  # Conjugate of the quaternion
    v_quat = (*v, 0)  # Represent the vector as a quaternion with w=0
    return multiply_quaternions(multiply_quaternions(q, v_quat), q_conj)[:3]


def multiply_poses(pose1: Pose, pose2: Pose) -> Tuple:
    """
    Multiply two poses.

    :param pose1: first Pose that should be multiplied
    :param pose2: Second Pose that should be multiplied
    :return: A Tuple of position and quaternion as result of the multiplication
    """
    pos1, quat1 = pose1.pose.position, pose1.pose.orientation
    pos2, quat2 = pose2.pose.position, pose2.pose.orientation
    # Multiply the orientations
    new_quat = multiply_quaternions(quat1, quat2)

    # Transform the position
    new_pos = np.add(pos1, quaternion_rotate(quat1, pos2))

    return new_pos, new_quat