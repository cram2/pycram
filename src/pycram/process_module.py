"""Implementation of process modules.

Classes:
ProcessModule -- implementation of process modules.
"""
# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import glob
import importlib
from os.path import dirname, basename, isfile, join
from datetime import timedelta
from threading import Lock
import time
from abc import ABC, abstractmethod

from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.world import World
from typing_extensions import Callable, Any, Union, Optional, List

from .robot_description import RobotDescription, ViewManager
from typing_extensions import TYPE_CHECKING
from .datastructures.enums import ExecutionType
from .ros import logerr, logwarn_once
from .config.world_conf import WorldConfig

if TYPE_CHECKING:
    from pycram.robot_plans.motions import BaseMotion


class ProcessModule:
    """
    Implementation of process modules. Process modules are the part that communicate with the outer world to execute
     designators.
    """
    execution_delay: Optional[timedelta] = WorldConfig.execution_delay
    """
    Adds a delay after executing a process module, to make the execution in simulation more realistic
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
        Execute the given designator_description. If there is already another process module of the same kind the `self._lock` will
        lock this thread until the execution of that process module is finished. This implicitly queues the execution of
        process modules.

        :param designator: The designator_description to execute.
        :return: Return of the Process Module if there is any
        """
        with self._lock:
            ret = self._execute(designator)
            if ProcessModule.execution_delay:
                time.sleep(self.execution_delay.total_seconds())

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
        self.pre: ExecutionType = ExecutionType.REAL
        self.pre_delay: timedelta = WorldConfig.execution_delay

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
        sets it to 'real'
        """
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = ExecutionType.REAL
        self.pre_delay = ProcessModule.execution_delay
        ProcessModule.execution_delay = timedelta(seconds=0)

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
        self.pre: ExecutionType = ExecutionType.SIMULATED

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
        sets it to 'simulated'
        """
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = ExecutionType.SIMULATED

    def __exit__(self, _type, value, traceback):
        """
        Exit method for the 'with' scope, sets the :py:attr:`~ProcessModuleManager.execution_type` to the previously
        used one.
        """
        ProcessModuleManager.execution_type = self.pre

    def __call__(self):
        return self


class SemiRealRobot:
    """
    Management class for executing designators on the semi-real robot. This is intended to be used in a with environment.
    When importing this class an instance is imported instead.

    Example:

    .. code-block:: python

        with semi_real_robot:
            some designators
    """

    def __init__(self):
        self.pre: ExecutionType = ExecutionType.SEMI_REAL

    def __enter__(self):
        """
        Entering function for 'with' scope, saves the previously set :py:attr:`~ProcessModuleManager.execution_type` and
        sets it to 'semi_real'
        """
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = ExecutionType.SEMI_REAL

    def __exit__(self, type, value, traceback):
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
        ProcessModuleManager.execution_type = ExecutionType.REAL
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
        ProcessModuleManager.execution_type = ExecutionType.SIMULATED
        ret = func(*args, **kwargs)
        ProcessModuleManager.execution_type = pre
        return ret

    return wrapper


# These are imported, so they don't have to be initialized when executing with
simulated_robot = SimulatedRobot()
real_robot = RealRobot()
semi_real_robot = SemiRealRobot()


class ProcessModuleManager(ABC):
    """
    Base class for managing process modules, any new process modules have to implement this class to register the
    Process Modules
    """
    execution_type: ExecutionType = None
    """
    Whether the robot for which the process module is intended for is real or a simulated one
    """
    available_pms: List[ManagerBase] = []
    """
    List of all available Process Module Managers
    """
    _instance: ProcessModuleManager = None
    """
    Singelton instance of this Process Module Manager
    """
    _initialized: bool = False

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

    def __init__(self):
        """
        Registers the Process modules for this robot. The name of the robot has to match the name given in the robot
        description.

        :param robot_name: Name of the robot for which these Process Modules are intended
        """
        if self._initialized:
            return
        self.available_pms = []
        self.register_all_process_modules()
        self._initialized = True

    def register_manager(self, manager: ManagerBase):
        """
        Register a new Process Module Manager for the given robot name.

        :param manager: The Process Module Manager to register
        """
        if not isinstance(manager, ManagerBase):
            raise TypeError(f"Expected ProcessModuleManager, got {type(manager)}")
        self.available_pms.append(manager)


    def get_manager(self, robot: AbstractRobot) -> ManagerBase:
        """
        Returns the Process Module manager for the currently loaded robot or None if there is no Manager.

        :return: ProcessModuleManager instance of the current robot
        """
        self.register_all_process_modules()
        default_manager = None

        for pm_manager in self.available_pms:
            if robot.name.name in pm_manager.robot_name:
                return pm_manager
            if pm_manager.robot_name == "default":
                default_manager = pm_manager
        logwarn_once(f"No Process Module Manager found for robot: '{robot.name}' returning default process modules")
        return default_manager

        # manager = None
        # _default_manager = None
        # if not self.execution_type:
        #     raise RuntimeError(
        #         f"No execution_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
        #
        # robot_description = RobotDescription.current_robot_description
        # chains = robot_description.get_manipulator_chains()
        # gripper_name = [chain.end_effector.gripper_object_name for chain in chains
        #                 if chain.end_effector.gripper_object_name]
        # gripper_name = gripper_name[0] if len(gripper_name) > 0 else None
        #
        # for pm_manager in self.available_pms:
        #     if pm_manager.robot_name == robot_description.name or\
        #             ((pm_manager.robot_name == gripper_name) and gripper_name):
        #         manager = pm_manager
        #     if pm_manager.robot_name == "default":
        #         _default_manager = pm_manager
        #
        # if manager:
        #     return manager
        # elif _default_manager:
        #     logwarn_once(f"No Process Module Manager found for robot: '{RobotDescription.current_robot_description.name}'"
        #                        f", using default process modules")
        #     return _default_manager
        # else:
        #     logerr(f"No Process Module Manager found for robot: '{RobotDescription.current_robot_description.name}'"
        #                  f", and no default process modules available")
        #     return None

    @staticmethod
    def register_all_process_modules():
        modules = glob.glob(join(dirname(__file__) + "/process_modules", "*.py"))
        __all__ = [basename(f)[:-3] for f in modules if isfile(f) and not f.endswith('__init__.py')]

        for module_name in __all__:
            try:
                importlib.import_module(f".{module_name}", package="pycram.process_modules")
            except Exception as e:
                print(f"Error loading module {module_name}: {e}")


class ManagerBase(ABC):
    """
    Base class for all Process Module Managers. This class is used to register the Process Modules for the robot.
    It is intended to be used as a base class for all Process Module Managers.
    """

    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()
        self._move_tcp_waypoints_lock = Lock()
        ProcessModuleManager().register_manager(self)

    @abstractmethod
    def navigate(self) -> ProcessModule:
        """
        Get the Process Module for navigating the robot with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for navigating
        """
        pass

    @abstractmethod
    def looking(self) -> ProcessModule:
        """
        Get the Process Module for looking at a point with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`
        :return:
        """
        pass

    @abstractmethod
    def detecting(self) -> ProcessModule:
        """
        Get the Process Module for detecting an object with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for detecting an object
        """
        pass

    @abstractmethod
    def move_tcp(self) -> ProcessModule:
        """
        Get the Process Module for moving the Tool Center Point with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the TCP
        """
        pass

    @abstractmethod
    def move_arm_joints(self) -> ProcessModule:
        """
        Get the Process Module for moving the joints of the robot arm
        with respect to the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the arm joints
        """
        pass

    @abstractmethod
    def move_joints(self) -> ProcessModule:
        """
        Get the Process Module for moving any joint of the robot with respect to the
        :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving joints
        """
        pass

    @abstractmethod
    def move_gripper(self) -> ProcessModule:
        """
        Get the Process Module for moving the gripper with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the gripper
        """
        pass

    @abstractmethod
    def open(self) -> ProcessModule:
        """
        Get the Process Module for opening drawers with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for opening drawers
        """
        pass

    @abstractmethod
    def close(self) -> ProcessModule:
        """
        Get the Process Module for closing drawers with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for closing drawers
        """
        pass

    @abstractmethod
    def move_tcp_waypoints(self) -> ProcessModule:
        """
        Get the Process Module for moving the Tool Center Point along a list of waypoints with respect to
         the :py:attr:`~ProcessModuleManager.execution_type`

        :return: The Process Module for moving the TCP along a list of waypoints
        """
        pass