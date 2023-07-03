"""Implementation of process modules.

Classes:
ProcessModule -- implementation of process modules.
"""
import inspect
import time
from abc import ABC

import rospy

from .fluent import Fluent
from .designator import Designator
from typing import Callable, List, Type, Any

from .robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description


class ProcessModule:
    """
    Implementation of process modules.
    Process modules are the part that communicate with the outer world to execute designators.
    """
    execution_delay = True
    """
    Adds a delay of 0.5 seconds after executing a process module, to make the
    """

    @staticmethod
    def perform(designator: Type['MotionDesignatorDescription.Motion']) -> Any:
        """Automatically choose a process module and execute the given designator.

        :param designator: The designator to choose the process module for and to execute.
        :return: Result of the Process Module if there is any
        """
        result = None
        process_module_manager = None

        for pm_manager in ProcessModuleManager.available_pms:
            if pm_manager.robot_name == robot_description.i.name:
                process_module_manager = pm_manager

        if process_module_manager:
            if not ProcessModuleManager.execution_typ:
                rospy.logerr(
                    f"No execution_type is set, did you use the with_simulated_robot or with_real_robot decorator?")
            pm = pm_manager.__getattribute__(designator.cmd.replace("-", "_"))()

            result = pm.execute(designator)
            if ProcessModule.execution_delay:
                time.sleep(0.5)
        else:
            rospy.logerr(f"No Process Module found matching Motion Designator {designator}")

        return result

    def __init__(self):
        """Create a new process module."""
        self._running: Fluent = Fluent(False)
        self._designators: List[Designator] = []

    def _execute(self, designator: Type[Designator]) -> Any:
        """
        Helper method for internal usage only.
        This method is to be overwritten instead of the execute method.
        """
        pass

    def execute(self, designator: Type[Designator]) -> Any:
        """
        Execute the given designator. If the process module is already executing another designator, it queues the given designator and executes them in order.

        :param designator: the designator to execute.
        :return: Return of the Process Module if there is any
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
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "real"

    def __exit__(self, type, value, traceback):
        ProcessModuleManager.execution_type = self.pre

    def __call__(self):
        return self


class simulated_robot():
    def __init__(self):
        self.pre: str = ""

    def __enter__(self):
        self.pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "simulated"

    def __exit__(self, type, value, traceback):
        ProcessModuleManager.execution_type = self.pre

    def __call__(self):
        return self


def with_real_robot(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "real"
        func(*args, **kwargs)
        ProcessModuleManager.execution_type = pre

    return wrapper


def with_simulated_robot(func: Callable) -> Callable:
    def wrapper(*args, **kwargs):
        pre = ProcessModuleManager.execution_type
        ProcessModuleManager.execution_type = "simulated"
        func(*args, **kwargs)
        ProcessModuleManager.execution_type = pre

    return wrapper


# These are imported so they dont have to be initialized when executing with
simulated_robot = simulated_robot()
real_robot = real_robot()


class ProcessModuleManager(ABC):
    """
    Base class for managing process modules, any new process modules have to implement this class to register the
    Process Modules
    """
    execution_type = None
    """
    Whether the robot on which the motion designator should be executed is a real or a simulated one
    """
    available_pms = []
    """
    List of all available Process Modules
    """

    def __init__(self, robot_name):
        """
        Registers the Process modules for this robot. The name of the robot has to match the name given in the robot
        description.

        :param robot_name: Name of the robot for which these Process Modules are intended
        """
        self.robot_name = robot_name
        ProcessModuleManager.available_pms.append(self)

    def navigate(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def pick_up(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def place(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def looking(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def opening_gripper(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def closing_gripper(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def detecting(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_tcp(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_arm_joints(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def world_state_detecting(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_joints(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def move_gripper(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def open(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")

    def close(self) -> Type[ProcessModule]:
        raise NotImplementedError(
            f"There are no Process Modules for '{inspect.currentframe().f_code.co_name}' for robot '{self.robot_name}'")
