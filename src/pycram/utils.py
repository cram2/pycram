"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
from typing_extensions import List, Tuple, Callable

import os

from .datastructures.pose import Pose
import math

from typing_extensions import Dict


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
        # Open a pair of null files
        self.null_fds = [os.open(os.devnull, os.O_RDWR) for x in range(2)]
        # Save the actual stdout (1) and stderr (2) file descriptors.
        self.save_fds = [os.dup(1), os.dup(2)]

    def __enter__(self):
        # Assign the null pointers to stdout and stderr.
        # This one is not needed for URDF parsing output
        # os.dup2(self.null_fds[0], 1)
        os.dup2(self.null_fds[1], 2)

    def __exit__(self, *_):
        # Re-assign the real stdout/stderr back to (1) and (2)
        # This one is not needed for URDF parsing output
        # os.dup2(self.save_fds[0], 1)
        os.dup2(self.save_fds[1], 2)
        # Close all file descriptors
        for fd in self.null_fds + self.save_fds:
            os.close(fd)
