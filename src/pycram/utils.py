"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
import sys
from inspect import isgeneratorfunction

import numpy as np
from typing_extensions import List, Tuple, Callable

import os

from urdf_parser_py.urdf import URDF

from .datastructures.pose import Pose
import math

from typing_extensions import Dict
from scipy.spatial.transform import Rotation as R


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


def _apply_ik(robot: 'pycram.world_concepts.WorldObject', pose_and_joint_poses: Tuple[Pose, Dict[str, float]]) -> None:
    """
    Apllies a list of joint poses calculated by an inverse kinematics solver to a robot

    :param robot: The robot the joint poses should be applied on
    :param pose_and_joint_poses: The base pose and joint states as returned by the ik solver
    :return: None
    """
    pose, joint_states = pose_and_joint_poses
    robot.set_pose(pose)
    robot.set_joint_positions(joint_states)


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


def translate_relative_to_object(obj_pose, palm_axis, translation_value) -> Pose:
    """
    Applies the translation directly along the palm axis returned by get_palm_axis().

    Args:
        oTg: The current pose of the object relative to the gripper.
        palm_axis: A list [x, y, z] where one value is 1 or -1, and the others are 0.
        translation_value: The magnitude of the retreat in meters.
        gripper_pose: The current pose of the gripper.

    Returns:
        None: Modifies the oTg.pose in place.
    """
    object_pose = obj_pose.copy()
    local_retraction = np.array([palm_axis[0] * translation_value,
                                 palm_axis[1] * translation_value,
                                 palm_axis[2] * translation_value])

    quat = object_pose.orientation_as_list()

    rotation_matrix = R.from_quat(quat).as_matrix()

    retraction_world = rotation_matrix @ local_retraction

    object_pose.pose.position.x -= retraction_world[0]
    object_pose.pose.position.y -= retraction_world[1]
    object_pose.pose.position.z -= retraction_world[2]

    return object_pose


def on_error_do_nothing(message):
    pass


def load_urdf_silently(urdf_path_or_string: str, from_string_instead_of_file: bool = False) -> URDF:
    """
    Loads a URDF file or XML string with suppressed error messages.

    This function temporarily overrides the `on_error` function in the `urdf_parser_py.xml_reflection.core` module
    to suppress warnings and error messages during URDF parsing.

    Args:
        urdf_path_or_string (str): Path to the URDF file or XML string if `from_string_instead_of_file` is True.
        from_string_instead_of_file (bool, optional): If True, interprets `urdf_path_or_string` as an XML string.
            Defaults to False.

    Returns:
        URDF: Parsed URDF object.

    Raises:
        ImportError: If the `urdf_parser_py.xml_reflection.core` module is not found.
    """
    urdf_core_module = sys.modules.get('urdf_parser_py.xml_reflection.core')
    if urdf_core_module is None:
        raise ImportError(
            "Could not locate `urdf_parser_py.xml_reflection.core` module. Please check how the module changed since "
            "'https://github.com/ros/urdf_parser_py/blob/3bcb9051e3bc6ebb8bff0bf8dd2c2281522b05d9/src/urdf_parser_py/xml_reflection/core.py#L33'")

    original_on_error = getattr(urdf_core_module, 'on_error', None)
    urdf_core_module.on_error = on_error_do_nothing

    try:
        if from_string_instead_of_file:
            return URDF.from_xml_string(urdf_path_or_string)
        else:
            return URDF.from_xml_file(urdf_path_or_string)
    finally:
        if original_on_error is not None:
            urdf_core_module.on_error = original_on_error


class suppress_stdout_stderr(object):
    """
    A context manager for doing a "deep suppression" of stdout and stderr in
    Python, i.e. will suppress all prints, even if the print originates in a
    compiled C/Fortran sub-function.

    This will not suppress raised exceptions, since exceptions are printed
    to stderr just before a script exits, and after the context manager has
    exited (at least, I think that is why it lets exceptions through).
    Copied from https://stackoverflow.com/questions/11130156/suppress-stdout-stderr-print-from-python-functions
    """

    def __init__(self):
        # Open a pair of null files (one for stdout and one for stderr)
        self.null_fds = [os.open(os.devnull, os.O_RDWR) for _ in range(2)]
        # Save the actual stdout (1) and stderr (2) file descriptors
        self.save_fds = [os.dup(1), os.dup(2)]

    def __enter__(self):
        # Redirect stdout and stderr to null
        os.dup2(self.null_fds[0], 1)  # Redirect stdout
        os.dup2(self.null_fds[1], 2)  # Redirect stderr

    def __exit__(self, exc_type, exc_val, exc_tb):
        # Restore stdout and stderr
        os.dup2(self.save_fds[0], 1)  # Restore stdout
        os.dup2(self.save_fds[1], 2)  # Restore stderr

        # Close all file descriptors
        for fd in self.null_fds + self.save_fds:
            os.close(fd)


def hacky_urdf_parser_fix(urdf_str):
    # TODO this function is inefficient but the tested urdfs's aren't big enough for it to be a problem
    fixed_urdf = ''
    delete = False
    black_list = ['transmission', 'gazebo']
    black_open = ['<{}'.format(x) for x in black_list]
    black_close = ['</{}'.format(x) for x in black_list]
    for line in urdf_str.split('\n'):
        if len([x for x in black_open if x in line]) > 0:
            delete = True
        if len([x for x in black_close if x in line]) > 0:
            delete = False
            continue
        if not delete:
            fixed_urdf += line + '\n'
    return fixed_urdf
