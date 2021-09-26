"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction

from macropy.core.quotes import macros, ast_literal, q
import pybullet as p
from .robot_description import InitializedRobotDescription as robot_description


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
