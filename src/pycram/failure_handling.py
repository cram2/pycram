import logging

from .datastructures.enums import TaskStatus
from .designator import DesignatorDescription, BaseMotion
from .failures import PlanFailure
from threading import Lock
from typing_extensions import Union, Tuple, Any, List, Optional, Type, Callable
from .language import LanguageMixin, MonitorNode, MonitorPlan
from .plan import Plan
from .process_module import ProcessModule


class FailureHandling(LanguageMixin):
    """
    Base class for failure handling mechanisms in automated systems or workflows.

    This class provides a structure for implementing different strategies to handle
    failures that may occur during the execution of a plan or process. It is designed
    to be extended by subclasses that implement specific failure handling behaviors.
    """

    def __init__(self, plan: Optional[Plan] = None):
        """
        Initializes a new instance of the FailureHandling class.

        :param Union[DesignatorDescription, MonitorNode] plan: The description or context of the task
        or process for which the failure handling is being set up.
        """
        self.plan = plan

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
    """
    max_tries: int
    """
    The maximum number of attempts to retry the action.
    """

    def __init__(self, plan: MonitorPlan, max_tries: int = 3):
        """
        Initializes a new instance of the Retry class.

        :param plan: The description or context of the task or process for which the retry mechanism is being set up.
        :param max_tries: The maximum number of attempts to retry. Defaults to 3.
        """
        super().__init__(plan)
        self.max_tries = max_tries

    def perform(self) -> List[Any]:
        """
        Implementation of the retry mechanism.

        This method attempts to perform the action specified in the designator_description.
        If the action fails, it is retried up to max_tries times. If all attempts fail,
        the last exception is raised.

        :raises PlanFailure: If all retry attempts fail.
        """
        tries = 0
        for action in iter(self.plan):
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
    """
    max_tries: int
    """
    The maximum number of attempts to retry the action.
    """
    recovery: dict
    """
    A dictionary that maps exception types to recovery actions
    """

    def __init__(self, monitor_plan: MonitorPlan, max_tries: int = 3, recovery: dict = None):
        """
        Initializes a new instance of the RetryMonitor class.
        :param MonitorNode monitor_plan: The Monitor instance to be used.
        :param int max_tries: The maximum number of attempts to retry. Defaults to 3.
        :param dict recovery: A dictionary that maps exception types to recovery actions. Defaults to None.
        """
        super().__init__(monitor_plan)
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
                if not isinstance(value, LanguageMixin):
                    raise TypeError("Values in the recovery dictionary must be instances of the Language class.")
            self.recovery = recovery

    def perform(self) -> List[Any]:
        """
        This method attempts to perform the Monitor + plan specified in the designator_description. If the action
        fails, it is retried up to max_tries times. If all attempts fail, the last exception is raised. In every
        loop, we need to clear the kill_event, and set all relevant 'interrupted' variables to False, to make sure
        the Monitor and plan are executed properly again.

        :raises PlanFailure: If all retry attempts fail.

        :return: The state of the execution performed, as well as a flattened list of the
        results, in the correct order
        """

        def reset_interrupted(child):
            # child.interrupted = False
            child.status = TaskStatus.CREATED
            try:
                for sub_child in child.children:
                    reset_interrupted(sub_child)
            except AttributeError:
                pass

        def flatten(result):
            flattened_list = []
            if result:
                for item in result:
                    if isinstance(item, list):
                        flattened_list.extend(item)
                    else:
                        flattened_list.append(item)
                return flattened_list
            return None

        status, res = None, None
        with self.lock:
            tries = 0
            while True:
                # self.plan.kill_event.clear()
                self.plan.interrupted = False
                for child in self.plan.root.children:
                    reset_interrupted(child)
                try:
                    res = self.plan.perform()
                    break
                except PlanFailure as e:
                    tries += 1
                    if tries >= self.max_tries:
                        raise e
                    exception_type = type(e)
                    if exception_type in self.recovery:
                        self.recovery[exception_type].perform()
        return flatten(res)


def try_action(action: Any, failure_type: Type[Exception], max_tries: int = 3):
    """
    A generic function to retry an action a certain number of times before giving up, with a specific failure type.

    :param action: The action to be performed, it must have a perform() method.
    :param failure_type: The type of exception to catch.
    :param max_tries: The maximum number of attempts to retry the action. Defaults to 3.
    """
    return try_method(method=lambda: action.perform(),
                      failure_type=failure_type,
                      max_tries=max_tries,
                      name="action")


def try_motion(motion: ProcessModule, motion_designator_instance: BaseMotion,
               failure_type: Type[Exception], max_tries: int = 3):
    """
    A generic function to retry a motion a certain number of times before giving up, with a specific exception.

    :param motion: The motion to be executed.
    :param motion_designator_instance: The instance of the motion designator that has the description of the motion.
    :param failure_type: The type of exception to catch.
    :param max_tries: The maximum number of attempts to retry the motion.
    """
    return try_method(method=lambda: motion.execute(motion_designator_instance),
                      failure_type=failure_type,
                      max_tries=max_tries,
                      name="motion")


def try_method(method: Callable, failure_type: Type[Exception], max_tries: int = 3, name: str = "method"):
    """
    A generic function to retry a method a certain number of times before giving up, with a specific exception.

    :param method: The method to be called.
    :param failure_type: The type of exception to catch.
    :param max_tries: The maximum number
    :param name: The name of the method to be called.
    """
    current_retry = 0
    result = None
    while current_retry < max_tries:
        try:
            result = method()
            break
        except failure_type as e:
            logging.debug(f"Caught exception {e} during {name} execution {method}. Retrying...")
            current_retry += 1
    if current_retry == max_tries:
        logging.error(f"Failed to execute {name} {method} after {max_tries} retries.")
    return result
