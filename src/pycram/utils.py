"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
from typing_extensions import List, Tuple, Callable

import numpy as np

from .datastructures.pose import Pose
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


def _apply_ik(robot: 'pycram.world_concepts.WorldObject', joint_poses: List[float], joints: List[str]) -> None:
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
    robot.set_joint_positions(dict(zip(joints, joint_poses)))
    # for i in range(0, len(joints)):
    #     robot.set_joint_state(joints[i], joint_poses[i])


# def calculate_wrist_tool_offset(wrist_frame: str, tool_frame: str, robot: WorldObject) -> Transform:
#     return robot.get_transform_between_links(wrist_frame, tool_frame)


# def transform(pose: List[float],
#               transformation: List[float],
#               local_coords=False):  # TODO: if pose is a list of position and orientation calculate new pose w/ orientation too
#     input_has_rotation = len(pose) == 7
#     transformation_has_rotation = len(transformation) == 7
#     pose_tf = transform_from_pq(
#         np.concatenate((pose[:3], quaternion_wxyz_from_xyzw(pose[3:])))) if input_has_rotation else transform_from(
#         np.eye(3), pose)
#     transformation_tf = transform_from_pq(np.concatenate((transformation[:3], quaternion_wxyz_from_xyzw(
#         transformation[3:])))) if transformation_has_rotation else transform_from(np.eye(3), transformation)
#     if local_coords:
#         res = pose_tf @ transformation_tf
#     else:
#         res = transformation_tf @ pose_tf
#     res = pq_from_transform(res)
#     res[3:] = quaternion_xyzw_from_wxyz(res[3:])
#     if not input_has_rotation:
#         return res[:3].tolist()
#     return res.tolist()


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
