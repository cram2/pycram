from abc import ABC, abstractmethod
from collections.abc import Iterable

import numpy as np
from ..tf_transformations import quaternion_multiply, quaternion_inverse
from typing_extensions import List, Union, Optional, Any, Sized, Iterable as T_Iterable, TYPE_CHECKING, Tuple

from ..datastructures.enums import JointType
if TYPE_CHECKING:
    from ..datastructures.pose import PoseStamped


class ErrorChecker(ABC):
    """
    An abstract class that resembles an error checker. It has two main methods, one for calculating the error between
    two values and another for checking if the error is acceptable.
    """
    def __init__(self, acceptable_error: Union[float, T_Iterable[float]], is_iterable: Optional[bool] = False):
        """
        Initialize the error checker.

        :param acceptable_error: The acceptable error.
        :param is_iterable: Whether the error is iterable (i.e. list of errors).
        """
        self._acceptable_error: np.ndarray = np.array(acceptable_error)
        self.tiled_acceptable_error: Optional[np.ndarray] = None
        self.is_iterable = is_iterable

    def reset(self) -> None:
        """
        Reset the error checker.
        """
        self.tiled_acceptable_error = None

    @property
    def acceptable_error(self) -> np.ndarray:
        return self._acceptable_error

    @acceptable_error.setter
    def acceptable_error(self, new_acceptable_error: Union[float, T_Iterable[float]]) -> None:
        self._acceptable_error = np.array(new_acceptable_error)

    def update_acceptable_error(self, new_acceptable_error: Optional[T_Iterable[float]] = None,
                                tile_to_match: Optional[Sized] = None,) -> None:
        """
        Update the acceptable error with a new value, and tile it to match the length of the error if needed.

        :param new_acceptable_error: The new acceptable error.
        :param tile_to_match: The iterable to match the length of the error with.
        """
        if new_acceptable_error is not None:
            self.acceptable_error = new_acceptable_error
        if tile_to_match is not None and self.is_iterable:
            self.update_tiled_acceptable_error(tile_to_match)

    def update_tiled_acceptable_error(self, tile_to_match: Sized) -> None:
        """
        Tile the acceptable error to match the length of the error.

        :param tile_to_match: The object to match the length of the error.
        :return: The tiled acceptable error.
        """
        self.tiled_acceptable_error = np.tile(self.acceptable_error.flatten(),
                                              len(tile_to_match) // self.acceptable_error.size)

    @abstractmethod
    def _calculate_error(self, value_1: Any, value_2: Any) -> Union[float, List[float]]:
        """
        Calculate the error between two values.

        :param value_1: The first value.
        :param value_2: The second value.
        :return: The error between the two values.
        """
        pass

    def calculate_error(self, value_1: Any, value_2: Any) -> Union[float, List[float]]:
        """
        Calculate the error between two values.

        :param value_1: The first value.
        :param value_2: The second value.
        :return: The error between the two values.
        """
        if self.is_iterable:
            return [self._calculate_error(v1, v2) for v1, v2 in zip(value_1, value_2)]
        else:
            return self._calculate_error(value_1, value_2)

    def is_error_acceptable(self, value_1: Any, value_2: Any) -> bool:
        """
        Check if the error is acceptable.

        :param value_1: The first value.
        :param value_2: The second value.
        :return: Whether the error is acceptable.
        """
        error = self.calculate_error(value_1, value_2)
        if self.is_iterable:
            error = np.array(error).flatten()
            if self.tiled_acceptable_error is None or\
                    len(error) != len(self.tiled_acceptable_error):
                self.update_tiled_acceptable_error(error)
            return np.all(error <= self.tiled_acceptable_error)
        else:
            return is_error_acceptable(error, self.acceptable_error)


class PoseErrorChecker(ErrorChecker):

    def __init__(self, acceptable_error: Union[Tuple[float], T_Iterable[Tuple[float]]] = (1e-3, np.pi / 180),
                 is_iterable: Optional[bool] = False):
        """
        Initialize the pose error checker.

        :param acceptable_error: The acceptable pose error (position error, orientation error).
        :param is_iterable: Whether the error is iterable (i.e. list of errors).
        """
        super().__init__(acceptable_error, is_iterable)

    def _calculate_error(self, value_1: Any, value_2: Any) -> List[float]:
        """
        Calculate the error between two poses.

        :param value_1: The first pose.
        :param value_2: The second pose.
        """
        return calculate_pose_error(value_1, value_2)


class PositionErrorChecker(ErrorChecker):

    def __init__(self, acceptable_error: Optional[float] = 1e-3, is_iterable: Optional[bool] = False):
        """
        Initialize the position error checker.

        :param acceptable_error: The acceptable position error.
        :param is_iterable: Whether the error is iterable (i.e. list of errors).
        """
        super().__init__(acceptable_error, is_iterable)

    def _calculate_error(self, value_1: Any, value_2: Any) -> float:
        """
        Calculate the error between two positions.

        :param value_1: The first position.
        :param value_2: The second position.
        :return: The error between the two positions.
        """
        return calculate_position_error(value_1, value_2)


class OrientationErrorChecker(ErrorChecker):

    def __init__(self, acceptable_error: Optional[float] = np.pi / 180, is_iterable: Optional[bool] = False):
        """
        Initialize the orientation error checker.

        :param acceptable_error: The acceptable orientation error.
        :param is_iterable: Whether the error is iterable (i.e. list of errors).
        """
        super().__init__(acceptable_error, is_iterable)

    def _calculate_error(self, value_1: Any, value_2: Any) -> float:
        """
        Calculate the error between two quaternions.

        :param value_1: The first quaternion.
        :param value_2: The second quaternion.
        :return: The error between the two quaternions.
        """
        return calculate_orientation_error(value_1, value_2)


class SingleValueErrorChecker(ErrorChecker):

    def __init__(self, acceptable_error: Optional[float] = 1e-3, is_iterable: Optional[bool] = False):
        """
        Initialize the single value error checker.

        :param acceptable_error: The acceptable error between two values.
        :param is_iterable: Whether the error is iterable (i.e. list of errors).
        """
        super().__init__(acceptable_error, is_iterable)

    def _calculate_error(self, value_1: Any, value_2: Any) -> float:
        """
        Calculate the error between two values.

        :param value_1: The first value.
        :param value_2: The second value.
        :return: The error between the two values.
        """
        return abs(value_1 - value_2)


class RevoluteJointPositionErrorChecker(SingleValueErrorChecker):

    def __init__(self, acceptable_error: Optional[float] = np.pi / 180, is_iterable: Optional[bool] = False):
        """
        Initialize the revolute joint position error checker.

        :param acceptable_error: The acceptable revolute joint position error.
        :param is_iterable: Whether the error is iterable (i.e. list of errors).
        """
        super().__init__(acceptable_error, is_iterable)


class PrismaticJointPositionErrorChecker(SingleValueErrorChecker):

    def __init__(self, acceptable_error: Optional[float] = 1e-3, is_iterable: Optional[bool] = False):
        """
        Initialize the prismatic joint position error checker.

        :param acceptable_error: The acceptable prismatic joint position error.
        :param is_iterable: Whether the error is iterable (i.e. list of errors).
        """
        super().__init__(acceptable_error, is_iterable)


class IterableErrorChecker(ErrorChecker):

    def __init__(self, acceptable_error: Optional[T_Iterable[float]] = None):
        """
        Initialize the iterable error checker.

        :param acceptable_error: The acceptable error between two values.
        """
        super().__init__(acceptable_error, True)

    def _calculate_error(self, value_1: Any, value_2: Any) -> float:
        """
        Calculate the error between two values.

        :param value_1: The first value.
        :param value_2: The second value.
        :return: The error between the two values.
        """
        return abs(value_1 - value_2)


class MultiJointPositionErrorChecker(IterableErrorChecker):

    def __init__(self, joint_types: List[JointType], acceptable_error: Optional[T_Iterable[float]] = None):
        """
        Initialize the multi-joint position error checker.

        :param joint_types: The types of the joints.
        :param acceptable_error: The acceptable error between two joint positions.
        """
        self.joint_types = joint_types
        if acceptable_error is None:
            acceptable_error = [np.pi/180 if jt == JointType.REVOLUTE else 1e-3 for jt in joint_types]
        super().__init__(acceptable_error)

    def _calculate_error(self, value_1: Any, value_2: Any) -> float:
        """
        Calculate the error between two joint positions.

        :param value_1: The first joint position.
        :param value_2: The second joint position.
        :return: The error between the two joint positions.
        """
        return calculate_joint_position_error(value_1, value_2)


def calculate_pose_error(pose_1: 'PoseStamped', pose_2: 'PoseStamped') -> List[float]:
    """
    Calculate the error between two poses.

    :param pose_1: The first pose.
    :param pose_2: The second pose.
    :return: The error between the two poses.
    """
    return [calculate_position_error(pose_1.position.to_list(), pose_2.position.to_list()),
            calculate_orientation_error(pose_1.orientation.to_list(), pose_2.orientation.to_list())]


def calculate_position_error(position_1: List[float], position_2: List[float]) -> float:
    """
    Calculate the error between two positions.

    :param position_1: The first position.
    :param position_2: The second position.
    :return: The error between the two positions.
    """
    return np.linalg.norm(np.array(position_1) - np.array(position_2))


def calculate_orientation_error(quat_1: List[float], quat_2: List[float]) -> float:
    """
    Calculate the error between two quaternions.

    :param quat_1: The first quaternion.
    :param quat_2: The second quaternion.
    :return: The error between the two quaternions.
    """
    return calculate_angle_between_quaternions(quat_1, quat_2)


def calculate_joint_position_error(joint_position_1: float, joint_position_2: float) -> float:
    """
    Calculate the error between two joint positions.

    :param joint_position_1: The first joint position.
    :param joint_position_2: The second joint position.
    :return: The error between the two joint positions.
    """
    return abs(joint_position_1 - joint_position_2)


def is_error_acceptable(error: Union[float, T_Iterable[float]],
                        acceptable_error: Union[float, T_Iterable[float]]) -> bool:
    """
    Check if the error is acceptable.

    :param error: The error.
    :param acceptable_error: The acceptable error.
    :return: Whether the error is acceptable.
    """
    if isinstance(error, Iterable):
        return all([error_i <= acceptable_error_i for error_i, acceptable_error_i in zip(error, acceptable_error)])
    else:
        return error <= acceptable_error


def calculate_angle_between_quaternions(quat_1: List[float], quat_2: List[float]) -> float:
    """
    Calculates the angle between two quaternions.

    :param quat_1: The first quaternion.
    :param quat_2: The second quaternion.
    :return: A float value that represents the angle between the two quaternions.
    """
    quat_diff = calculate_quaternion_difference(quat_1, quat_2)
    quat_diff_angle = 2 * np.arctan2(np.linalg.norm(quat_diff[0:3]), quat_diff[3])
    if quat_diff_angle > np.pi:
        quat_diff_angle = 2 * np.pi - quat_diff_angle
    return quat_diff_angle


def calculate_quaternion_difference(quat_1: List[float], quat_2: List[float]) -> List[float]:
    """
    Calculates the quaternion difference.

    :param quat_1: The quaternion of the object at the first time step.
    :param quat_2: The quaternion of the object at the second time step.
    :return: A list of float values that represent the quaternion difference.
    """
    quat_diff = quaternion_multiply(quaternion_inverse(quat_1), quat_2)
    return quat_diff
