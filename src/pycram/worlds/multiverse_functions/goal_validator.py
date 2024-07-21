from abc import abstractmethod, ABC

import numpy as np
from typing_extensions import List, Union, Any, Callable

from pycram.datastructures.pose import Pose


class GoalValidator(ABC):
    """
    A class to validate the goal by tracking the goal achievement progress.
    """

    def __init__(self, initial_value: Any, goal_value: Any, current_value_getter: Callable[[], Any], name: str,
                 acceptable_percentage_of_goal_achieved: float = 0.1):
        self.initial_value: Any = initial_value
        self.current_value_getter: Callable[[], Any] = current_value_getter
        self.goal_value: Any = goal_value
        self.name: str = name
        self.acceptable_percentage_of_goal_achieved: float = acceptable_percentage_of_goal_achieved
        self.initial_error: float = self.calculate_initial_error()

    def calculate_initial_error(self) -> float:
        """
        Calculate the initial error.
        """
        return self.calculate_error(self.goal_value, self.initial_value)

    @property
    def current_value(self) -> Any:
        """
        Get the current value.
        """
        return self.current_value_getter()

    @property
    def current_error(self) -> float:
        """
        Calculate the current error.
        """
        return self.calculate_error(self.goal_value, self.current_value)

    @abstractmethod
    def calculate_error(self, value_1: Any, value_2: Any) -> float:
        """
        Calculate the error between two values.
        """
        pass

    @property
    def percentage_of_goal_achieved(self) -> float:
        """
        Calculate the percentage of goal achieved.
        """
        if self.initial_error > 1e-3:
            return 1 - self.current_error / self.initial_error
        else:
            return 1

    @property
    def goal_achieved(self) -> bool:
        """
        Check if the goal is achieved.
        return: Whether the goal is achieved.
        """
        return self.percentage_of_goal_achieved >= self.acceptable_percentage_of_goal_achieved


class SingleValueGoalValidator(GoalValidator):
    """
    A class to validate the goal by tracking the goal achievement progress for a single value.
    """

    def __init__(self, initial_value: Union[float, int], goal_value: Union[float, int],
                 current_value_getter: Callable[[], Union[float, int]], name: str,
                 acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)

    def calculate_error(self, value_1: Union[float, int], value_2: Union[float, int]) -> Union[float, int]:
        """
        Calculate the error between two values.
        return: The error between the two values.
        """
        return abs(value_1 - value_2)


class JointGoalValidator(SingleValueGoalValidator):
    def __init__(self, initial_value: Union[float, int], goal_value: Union[float, int],
                 current_value_getter: Callable[[], Union[float, int]], name: str,
                 acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)


class IterableGoalValidator(GoalValidator):
    """
    A class to validate the goal by tracking the goal achievement progress for an iterable goal.
    """

    def __init__(self, initial_value: List[Any], goal_value: List[Any],
                 current_value_getter: Callable[[], List[Any]], name: str,
                 acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)

    def calculate_error(self, iterable_1: List[Any], iterable_2: List[Any]) -> float:
        """
        Calculate the error between two iterables.
        return: The error between the two iterables.
        """
        return np.linalg.norm(np.array(iterable_1) - np.array(iterable_2))


def get_combined_goal_validator(goal_validators: List[GoalValidator], combined_name: str):
    """
    Get a combined goal validator.
    :param goal_validators: The goal validators to combine.
    :param combined_name: The name of the combined goal validator.
    return: The combined goal validator.
    """

    class CombinedGoalValidator(IterableGoalValidator):
        def __init__(self, initial_value: List[Any], goal_value: List[Any],
                     current_value_getter: Callable[[], List[Any]],
                     name: str, acceptable_percentage_of_goal_achieved: float = 0.1):
            super().__init__(initial_value, goal_value, current_value_getter, name,
                             acceptable_percentage_of_goal_achieved)

        def calculate_error(self, iterable_1: List[Any], iterable_2: List[Any]) -> float:
            """
            Calculate the error between two iterables.
            return: The error between the two iterables.
            """
            return (sum([goal_validator.calculate_error(value_1, value_2)
                        for goal_validator, value_1, value_2 in zip(goal_validators, iterable_1, iterable_2)]) /
                    len(goal_validators))

    return CombinedGoalValidator([goal_validator.initial_value for goal_validator in goal_validators],
                                 [goal_validator.goal_value for goal_validator in goal_validators],
                                 [goal_validator.current_value_getter for goal_validator in goal_validators],
                                 combined_name, 0.5)


class PositionGoalValidator(IterableGoalValidator):
    def __init__(self, initial_value: List[float], goal_value: List[float],
                 current_value_getter: Callable[[], List[float]],
                 name: str, acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)


class OrientationGoalValidator(IterableGoalValidator):
    def __init__(self, initial_value: List[float], goal_value: List[float],
                 current_value_getter: Callable[[], List[float]],
                 name: str, acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)


class MultiJointGoalValidator(IterableGoalValidator):
    def __init__(self, initial_value: List[float], goal_value: List[float],
                 current_value_getter: Callable[[], List[float]], name: str,
                 acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)


class PoseGoalValidator(GoalValidator):
    """
    A class to validate the goal by tracking the goal achievement progress for a pose goal.
    """

    def __init__(self, initial_value: Pose, goal_value: Pose, current_value_getter: Callable[[], Pose], name: str,
                 acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)

    def calculate_error(self, pose_1: Pose, pose_2: Pose) -> float:
        """
        Calculate the error between two poses.
        return: The error between the two poses.
        """
        return calculate_pose_error(pose_1, pose_2)


class MultiPoseGoalValidator(IterableGoalValidator):
    def __init__(self, initial_value: List[Pose], goal_value: List[Pose],
                 current_value_getter: Callable[[], List[Pose]], name: str,
                 acceptable_percentage_of_goal_achieved: float = 0.1):
        super().__init__(initial_value, goal_value, current_value_getter, name, acceptable_percentage_of_goal_achieved)

    def calculate_error(self, iterable_1: List[Pose], iterable_2: List[Pose]) -> float:
        """
        Calculate the error between two iterables.
        return: The error between the two iterables.
        """
        return (sum([calculate_pose_error(pose_1, pose_2) for pose_1, pose_2 in zip(iterable_1, iterable_2)]) /
                len(iterable_1))


def calculate_pose_error(pose_1: Pose, pose_2: Pose) -> float:
    """
    Calculate the error between two poses.
    return: The error between the two poses.
    """
    return (np.linalg.norm(np.array(pose_1.position_as_list()) - np.array(pose_2.position_as_list()))
            + np.linalg.norm(np.array(pose_1.orientation_as_list()) - np.array(pose_2.orientation_as_list())))
