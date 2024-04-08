"""Implementation of process modules.

Classes:
ProcessModule -- implementation of process modules.
"""
# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations
import inspect
import threading
import time
from abc import ABC
from typing_extensions import Callable, Type, Any, Union

import rospy

from .language import Language
from .robot_descriptions import robot_description
from typing_extensions import TYPE_CHECKING

if TYPE_CHECKING:
    from .designators.motion_designator import BaseMotion


class ProcessModule:
    """
    Implementation of process modules. Process modules are the part that communicate with the outer world to execute
     designators.
    """
    execution_delay = True
    """
    Adds a delay of 0.5 seconds after executing a process module, to make the execution in simulation more realistic
    """
    block_list = []
    """
    List of thread ids for which no Process Modules should be executed. This is used as an interrupt mechanism for 
    Designators
    """

    def __init__(self, lock):
        """Create a new process module."""
        self._lock = lock

    def _execute(self, designator: BaseMotion) -> Any:
        """
        Helper method for internal usage only.
        This method is to be overwritten instead of the execute method.
        """
        pass

    def execute(self, designator: BaseMotion) -> Any:
        """
        Execute the given designator. If there is already another process module of the same kind the `self._lock` will
        lock this thread until the execution of that process module is finished. This implicitly queues the execution of
        process modules.

        :param designator: The designator to execute.
        :return: Return of the Process Module if there is any
        """
        if threading.get_ident() in Language.block_list:
            return None
        with self._lock:
            ret = self._execute(designator)
            if ProcessModule.execution_delay:
                time.sleep(0.5)

        return ret


class RealRobot:
    """
    Management class for executing designators on the real robot. This is intended to be used in a with environment.
    When importing this class an instance is imported instead.

    Example:

    .. code-block:: python

        with real_robot:
            some designators
    """

    def __init__(self):
        self.pre: str = ""
        self.pre_delay: bool = False

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
        sets it to 'real'
        """
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "real"
        self.pre_delay = ProcessModule.execution_delay
        ProcessModule.execution_delay = False

    def __exit__(self, _type, value, traceback):
        """
        Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
        used one.
        """
        ProcessModuleManager.execution_type = self.pre
        ProcessModule.execution_delay = self.pre_delay

    def __call__(self):
        return self


class SimulatedRobot:
    """
    Management class for executing designators on the simulated robot. This is intended to be used in
    a with environment. When importing this class an instance is imported instead.

    Example:

    .. code-block:: python

        with simulated_robot:
            some designators
    """

    def __init__(self):
        self.pre: str = ""

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
        sets it to 'simulated'
        """
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "simulated"

    def __exit__(self, _type, value, traceback):
        """
        Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
        used one.
        """
        ProcessModuleManager.execution_type = self.pre

    def __call__(self):
        return self


def with_real_robot(func: Callable) -> Callable:
    """
    Decorator to execute designators in the decorated class on the real robot.

    Example:

    .. code-block:: python

        @with_real_robot
        def plan():
            some designators

    :param func: Function this decorator is annotating
    :return: The decorated function wrapped into the decorator
    """

    def wrapper(*args, **kwargs):
        pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "real"
        ret = func(*args, **kwargs)
        ProcessModuleManager.execution_type = pre
        return ret

    return wrapper


def with_simulated_robot(func: Callable) -> Callable:
    """
    Decorator to execute designators in the decorated class on the simulated robot.

    Example:

    .. code-block:: python

        @with_simulated_robot
        def plan():
            some designators

    :param func: Function this decorator is annotating
    :return: The decorated function wrapped into the decorator
    """

    def wrapper(*args, **kwargs):
        pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "simulated"
        ret = func(*args, **kwargs)
        ProcessModuleManager.execution_type = pre
        return ret

    return wrapper


# These are imported, so they don't have to be initialized when executing with
simulated_robot = SimulatedRobot()
real_robot = RealRobot()


class ProcessModuleManager(ABC):
    """
    Base class for managing process modules, any new process modules have to implement this class to register the
    Process Modules
    """
    execution_type = None
    """
    Whether the robot for which the process module is intended for is real or a simulated one
    """
    available_pms = []
    """
    List of all available Process Module Managers
    """
    _instance = None
    """
    Singelton instance of this Process Module Manager
    """

    def __new__(cls, *args, **kwargs):
        """
        Creates a new instance if :py:attr:`~ProcessModuleManager._instance` is None, otherwise the instance
        in :py:attr:`~ProcessModuleManager._instance` is returned.
        :return: Singelton instance of this Process Module Manager
        """
        if not cls._instance:
            cls._instance = super(ProcessModuleManager, cls).__new__(cls)
            return cls._instance
        else:
            return cls._instance

    def __init__(self, robot_name):
        """
        Registers the Process modules for this robot. The name of the robot has to match the name given in the robot
        description.

        :param robot_name: Name of the robot for which these Process Modules are intended
        """
        self.robot_name = robot_name
        ProcessModuleManager.available_pms.append(self)

    @staticmethod
    def get_manager() -> Union[ProcessModuleManager, None]:
        """
        Returns the Process Module manager for the currently loaded robot or None if there is no Manager.

        :return: ProcessModuleManager instance of the current robot
        """
        manager = None
        _default_manager = None
        if not ProcessModuleManager.execution_type:
            rospy.logerr(
                f"No execution_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
            return

        for pm_manager in ProcessModuleManager.available_pms:
            if pm_manager.robot_name == robot_description.name:
                manager = pm_manager
            if pm_manager.robot_name == "default":
                _default_manager = pm_manager

        if manager:
            return manager
        elif _default_manager:
            rospy.logwarn_once(f"No Process Module Manager found for robot: '{robot_description.name}'"
                               f", using default process modules")
            return _default_manager
        else:
            rospy.logerr(f"No Process Module Manager found for robot: '{robot_description.name}'"
                         f", and no default process modules available")
            return None

    def navigate(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for navigating the robot with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for navigating
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def pick_up(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for picking up with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for picking up an object
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def place(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for placing with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for placing an Object
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def looking(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for looking at a point with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for looking at a specific point
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def detecting(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for detecting an object with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for detecting an object
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_tcp(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving the Tool Center Point with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the TCP
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_arm_joints(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving the joints of the robot arm
        with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the arm joints
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def world_state_detecting(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for detecting an object using the world state with respect to the
        :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for world state detecting
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_joints(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving any joint of the robot with respect to the
        :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving joints
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_gripper(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for moving the gripper with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the gripper
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def open(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for opening drawers with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for opening drawers
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def close(self) -> Type[ProcessModule]:
        """
        Returns the Process Module for closing drawers with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for closing drawers
        """
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")
