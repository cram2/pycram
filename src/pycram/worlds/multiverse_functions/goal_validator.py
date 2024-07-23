import numpy as np
from typing_extensions import Any, Callable, Optional, Union, Iterable

from pycram.worlds.multiverse_functions.error_checkers import ErrorChecker


class GoalValidator:
    """
    A class to validate the goal by tracking the goal achievement progress.
    """

    def __init__(self, error_checker: ErrorChecker, current_value_getter: Callable[[], Any],
                 acceptable_percentage_of_goal_achieved: Optional[float] = 0.8):
        """
        Initialize the goal validator.
        :param error_checker: The error checker.
        :param current_value_getter: The current value getter.
        :param acceptable_percentage_of_goal_achieved: The acceptable percentage of goal achieved, if given, will be
        used to check if this percentage is achieved instead of the complete goal.
        """
        self.error_checker: ErrorChecker = error_checker
        self.current_value_getter: Callable[[], Any] = current_value_getter
        self.acceptable_percentage_of_goal_achieved: Optional[float] = acceptable_percentage_of_goal_achieved
        self.goal_value: Any = None
        self.initial_error: Optional[np.ndarray] = None

    @property
    def _acceptable_error(self) -> np.ndarray:
        """
        Get the acceptable error.
        """
        if self.error_checker.is_iterable:
            return self.tiled_acceptable_error
        else:
            return self.acceptable_error

    @property
    def acceptable_error(self) -> np.ndarray:
        """
        Get the acceptable error.
        """
        return self.error_checker.acceptable_error

    @property
    def tiled_acceptable_error(self) -> Optional[np.ndarray]:
        """
        Get the tiled acceptable error.
        """
        return self.error_checker.tiled_acceptable_error

    def register_goal_value(self, goal_value: Any, initial_value: Optional[Any] = None,
                            acceptable_error: Optional[Union[float, Iterable[float]]] = None):
        """
        Register the goal value.
        :param goal_value: The goal value.
        :param initial_value: The initial value.
        :param acceptable_error: The acceptable error.
        """
        self.goal_value = goal_value
        self.update_initial_error(goal_value, initial_value=initial_value)
        self.error_checker.update_acceptable_error(acceptable_error, self.initial_error)

    def update_initial_error(self, goal_value: Any, initial_value: Optional[Any] = None) -> None:
        """
        Calculate the initial error.
        """
        if initial_value is None:
            self.initial_error: np.ndarray = self.current_error
        else:
            self.initial_error: np.ndarray = self.calculate_error(goal_value, initial_value)

    @property
    def current_value(self) -> Any:
        """
        Get the current value.
        """
        return self.current_value_getter()

    @property
    def current_error(self) -> np.ndarray:
        """
        Calculate the current error.
        """
        return self.calculate_error(self.goal_value, self.current_value)

    def calculate_error(self, value_1: Any, value_2: Any) -> np.ndarray:
        """
        Calculate the error between two values.
        """
        return np.array(self.error_checker.calculate_error(value_1, value_2)).flatten()

    @property
    def percentage_of_goal_achieved(self) -> float:
        """
        Calculate the percentage of goal achieved.
        """
        percent_array = 1 - self.relative_current_error / self.relative_initial_error
        percent_array_filtered = percent_array[self.relative_initial_error > self._acceptable_error]
        if len(percent_array_filtered) == 0:
            return 1
        else:
            return np.mean(percent_array_filtered)

    @property
    def actual_percentage_of_goal_achieved(self) -> float:
        """
        Calculate the percentage of goal achieved.
        """
        percent_array = 1 - self.current_error / np.maximum(self.initial_error, 1e-3)
        percent_array_filtered = percent_array[self.initial_error > self._acceptable_error]
        if len(percent_array_filtered) == 0:
            return 1
        else:
            return np.mean(percent_array_filtered)

    @property
    def relative_current_error(self) -> np.ndarray:
        """
        Get the relative current error.
        """
        return self.get_relative_error(self.current_error, threshold=0)

    @property
    def relative_initial_error(self) -> np.ndarray:
        """
        Get the relative initial error.
        """
        return self.get_relative_error(self.initial_error)

    def get_relative_error(self, error: Any, threshold: Optional[float] = 1e-3) -> np.ndarray:
        """
        Get the relative error by comparing the error with the acceptable error and filtering out the errors that are
        less than the threshold.
        :param error: The error.
        :param threshold: The threshold.
        """
        return np.maximum(error-self._acceptable_error, threshold)

    @property
    def goal_achieved(self) -> bool:
        """
        Check if the goal is achieved.
        return: Whether the goal is achieved.
        """
        if self.acceptable_percentage_of_goal_achieved is None:
            return self.is_current_error_acceptable
        else:
            return self.percentage_of_goal_achieved >= self.acceptable_percentage_of_goal_achieved

    @property
    def is_current_error_acceptable(self) -> bool:
        """
        Check if the error is acceptable.
        return: Whether the error is acceptable.
        """
        return self.error_checker.is_error_acceptable(self.current_value, self.goal_value)
