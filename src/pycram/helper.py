"""Implementation of helper functions and classes for internal usage only.

Functions:
_block -- wrap multiple statements into a single block.

Classes:
GeneratorList -- implementation of generator list wrappers.
"""
from inspect import isgeneratorfunction
from threading import Lock

from macropy.core.quotes import ast_literal, q
import pybullet as p
from geometry_msgs.msg import Point, Quaternion, Pose, Transform, PoseStamped, TransformStamped, Vector3

import rospy
from std_msgs.msg import Header
from time import time as current_time


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


class SingletonMeta(type):
    """
    This is a thread-safe implementation of Singleton.
    """

    _instances = {}

    _lock: Lock = Lock()
    """
    We now have a lock object that will be used to synchronize threads during
    first access to the Singleton.
    """

    def __call__(cls, *args, **kwargs):
        """
        Possible changes to the value of the `__init__` argument do not affect
        the returned instance.
        """
        # Now, imagine that the program has just been launched. Since there's no
        # Singleton instance yet, multiple threads can simultaneously pass the
        # previous conditional and reach this point almost at the same time. The
        # first of them will acquire lock and will proceed further, while the
        # rest will wait here.
        with cls._lock:
            # The first thread to acquire the lock, reaches this conditional,
            # goes inside and creates the Singleton instance. Once it leaves the
            # lock block, a thread that might have been waiting for the lock
            # release may then enter this section. But since the Singleton field
            # is already initialized, the thread won't create a new object.
            if cls not in cls._instances:
                instance = super().__call__(*args, **kwargs)
                cls._instances[cls] = instance
        return cls._instances[cls]
