from .designator import DesignatorDescription
from .plan_failures import PlanFailure
from threading import Lock
from typing import Union
from .language import Language, Monitor


class FailureHandling(Language):
    """
    Base class for failure handling mechanisms in automated systems or workflows.

    This class provides a structure for implementing different strategies to handle
    failures that may occur during the execution of a plan or process. It is designed
    to be extended by subclasses that implement specific failure handling behaviors.
    """

    def __init__(self, designator_description: Union[DesignatorDescription, Monitor]):
        """
        Initializes a new instance of the FailureHandling class.

        :param designator_description: The description or context of the task or process for which the failure handling is being set up.
        """
        self.designator_description = designator_description

    def perform(self):
        """
        Abstract method to perform the failure handling mechanism.

        This method should be overridden in subclasses to implement the specific
        behavior for handling failures.

        :raises NotImplementedError: If the method is not implemented in a subclass.
        """
        raise NotImplementedError()


class Retry(FailureHandling):
    """
    A subclass of FailureHandling that implements a retry mechanism.

    This class represents a specific failure handling strategy where the system
    attempts to retry a failed action a certain number of times before giving up.

    Attributes:
        max_tries (int): The maximum number of attempts to retry the action.

    Inherits:
        All attributes and methods from the FailureHandling class.

    Overrides:
        perform(): Implements the retry logic.
    """

    def __init__(self, designator_description: DesignatorDescription, max_tries: int = 3):
        """
        Initializes a new instance of the Retry class.

        :param designator_description: The description or context of the task or process for which the retry mechanism is being set up.
        :param max_tries: The maximum number of attempts to retry. Defaults to 3.
        """
        super().__init__(designator_description)
        self.max_tries = max_tries

    def perform(self):
        """
        Implementation of the retry mechanism.

        This method attempts to perform the action specified in the designator_description.
        If the action fails, it is retried up to max_tries times. If all attempts fail,
        the last exception is raised.

        :raises PlanFailure: If all retry attempts fail.
        """
        tries = 0
        for action in iter(self.designator_description):
            tries += 1
            try:
                action.perform()
                break
            except PlanFailure as e:
                if tries >= self.max_tries:
                    raise e


class RetryMonitor(FailureHandling):
    """
    A subclass of FailureHandling that implements a retry mechanism that works with a Monitor.

    This class represents a specific failure handling strategy that allows us to retry a demo that is
    being monitored, in case that monitoring condition is triggered.

    Attributes:
        max_tries (int): The maximum number of attempts to retry the action.
        recovery (dict): A dictionary that maps exception types to recovery actions

    Inherits:
        All attributes and methods from the FailureHandling class.

    Overrides:
        perform(): Implements the retry logic.
    """

    def __init__(self, designator_description: Monitor, max_tries: int = 3, recovery: dict = None):
        """
        Initializes a new instance of the Retry class.

        Args:
            designator_description (DesignatorDescription): The description or context
            of the task or process for which the retry mechanism is being set up.
            max_tries (int, optional): The maximum number of attempts to retry. Defaults to 3.
            recovery (dict, optional): A dictionary that maps exception types to recovery actions. Defaults to None.
        """
        super().__init__(designator_description)
        self.max_tries = max_tries
        self.lock = Lock()
        if recovery is None:
            self.recovery = {}
        else:
            if not isinstance(recovery, dict):
                raise ValueError(
                    "Recovery must be a dictionary with exception types as keys and Language instances as values.")
            for key, value in recovery.items():
                if not issubclass(key, BaseException):
                    raise TypeError("Keys in the recovery dictionary must be exception types.")
                if not isinstance(value, Language):
                    raise TypeError("Values in the recovery dictionary must be instances of the Language class.")
            self.recovery = recovery

    def perform(self):
        """
        Implementation of the retry mechanism.

        This method attempts to perform the Monitor + plan specified in the designator_description.
        If the action fails, it is retried up to max_tries times. If all attempts fail,
        the last exception is raised. In every loop, we need to clear the kill_event, and set all
        relevant 'interrupted' variables to False, to make sure the Monitor and plan are executed
        properly again

        Raises:
            PlanFailure: If all retry attempts fail.

        Returns:
            The state of the execution performed, as well as a flattened list of the results, in the correct order
        """

        def reset_interrupted(child):
            if hasattr(child, "interrupted"):
                child.interrupted = False
            for sub_child in getattr(child, "children", []):
                reset_interrupted(sub_child)

        def flatten(result):
            flattened_list = []
            for item in result:
                if isinstance(item, list):
                    flattened_list.extend(item)
                else:
                    flattened_list.append(item)
            return flattened_list

        status, res = None, None
        with self.lock:
            tries = 0
            while True:
                self.designator_description.kill_event.clear()
                self.designator_description.interrupted = False
                for child in self.designator_description.children:
                    reset_interrupted(child)
                try:
                    status, res = self.designator_description.perform()
                    break
                except PlanFailure as e:
                    tries += 1
                    if tries >= self.max_tries:
                        raise e
                    exception_type = type(e)
                    if exception_type in self.recovery:
                        self.recovery[exception_type].perform()
        return status, flatten(res)
