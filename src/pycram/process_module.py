"""Implementation of process modules.

Classes:
ProcessModule -- implementation of process modules.
"""
import time

from .fluent import Fluent
from .designator import Designator
from typing import Callable, List, Type, Any


class ProcessModule:
    """Implementation of process modules.

    Process modules are the part that communicate with the outer world to execute designators.

    Variables:
    resolvers -- list of all process module resolvers.

    Functions:
    perform -- automatically choose a process module and execute the given designator.

    Methods:
    execute -- execute the given designator.
    """

    resolvers: List[Callable] = []
    """List of all process module resolvers. Process module resolvers are functions which take a designator as argument and return a process module."""
    robot_type: str = ""
    """The type of the robot, either real or simulated. Is used to determine which Process Module is choosen for execution."""
    # Adds a delay of 0.5 seconds after executing a process module, to make the
    execution_delay = True

    @staticmethod
    def perform(designator: Type['MotionDesignatorDescription.Motion']) -> Any:
        """Automatically choose a process module and execute the given designator.

        Arguments:
        designator -- the designator to choose the process module for and to execute.
        """
        result = None
        for resolver in ProcessModule.resolvers:
            pm = resolver(designator)

            if pm is not None:
                result = pm.execute(designator)
        if ProcessModule.execution_delay:
            time.sleep(0.5)

        return result

    def __init__(self):
        """Create a new process module."""
        self._running: Fluent = Fluent(False)
        self._designators: List[Designator] = []

    def _execute(self, designator: Type[Designator]):
        """This is a helper method for internal usage only.

        This method is to be overwritten instead of the execute method.
        """
        pass

    def execute(self, designator: Type[Designator]) -> Any:
        """Execute the given designator. If the process module is already executing another designator, it queues the given designator and executes them in order.

        Arguments:
        designator -- the designator to execute.
        """
        self._designators.append(designator)
        # (self._running == False).wait_for()
        self._running.set_value(True)
        designator = self._designators[0]
        try:
            ret = self._execute(designator)
        finally:
            self._running.set_value(False)
        self._designators.remove(designator)
        self._running.set_value(False)
        return ret


class real_robot():
    def __init__(self):
        self.pre: str = ""

    def __enter__(self):
        self.pre = ProcessModule.robot_type
        ProcessModule.robot_type = "real"

    def __exit__(self, type, value, traceback):
        ProcessModule.robot_type = self.pre

    def __call__(self):
        return self


class simulated_robot():
    def __init__(self):
        self.pre: str = ""

    def __enter__(self):
        self.pre = ProcessModule.robot_type
        ProcessModule.robot_type = "simulated"

    def __exit__(self, type, value, traceback):
        ProcessModule.robot_type = self.pre

    def __call__(self):
        return self


def with_real_robot(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        pre = ProcessModule.robot_type
        ProcessModule.robot_type = "real"
        func(*args, **kwargs)
        ProcessModule.robot_type = pre
    return wrapper


def with_simulated_robot(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        pre = ProcessModule.robot_type
        ProcessModule.robot_type = "simulated"
        func(*args, **kwargs)
        ProcessModule.robot_type = pre
    return wrapper


# These are imported so they dont have to be initialized when executing with
simulated_robot = simulated_robot()
real_robot = real_robot()
