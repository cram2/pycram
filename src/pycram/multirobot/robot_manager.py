from __future__ import annotations
from abc import ABC
from typing import Optional

from typing_extensions import TYPE_CHECKING

from pycram.robot_description import RobotDescriptionManager, RobotDescription
from pycram.ros import logdebug

if TYPE_CHECKING:
    from pycram.world_concepts.world_object import Object


class RobotManager(ABC):
    """
    Singleton class for managing multiple robots simultaneously
    """

    _instance = None
    """
    Instance of this Robot Manager
    """
    active_robot = None
    """
    Currently active robot
    """
    available_robots = {}
    """
    List of all available robots
    """
    robot_description = None
    """
    Robot description of active robot
    """
    giskard_robot = None
    """
    Active robot using giskard to avoid conflicts when other robots use other services 
    """

    def __new__(cls, *args, **kwargs):
        """
        Creates a new instance if :py:attr:`~RobotManager._instance` is None, otherwise the instance
        in :py:attr:`~RobotManager._instance` is returned.
        :return: Singleton instance of this Robot Manager
        """
        if not cls._instance:
            cls._instance = super(RobotManager, cls).__new__(cls)
            return cls._instance
        else:
            return cls._instance

    def __init__(self):
        """
        Initialize RobotManager.
        Makes sure that the RobotDescriptionManager has already been initialized.
        """
        logdebug(f'Initialize RobotManager.')

    @staticmethod
    def add_robot(robot: Object):
        """
        Add another robot to the list of available robots

        :param robot: Robot object to add
        """
        RobotManager.available_robots[robot.name] = robot

    @staticmethod
    def set_active_robot(robot: Optional[Object] = None):
        """
        Set the currently active robot based on its name.

        :param robot: Robot object to use
        """
        rdm = RobotDescriptionManager()
        RobotManager.robot_description = rdm.load_description(name=robot.name)
        RobotManager.active_robot = RobotManager.available_robots[robot.name] if robot else None
        RobotManager.set_giskard_robot(robot) if robot else None
        logdebug(f'Setting active robot. Is now: {robot.name}')

    @staticmethod
    def get_active_robot(robot: Optional[Object] = None) -> Object:
        """
        Returns the active robot if the given variable is None.
        Robot can be None in single-robot and sequential multi-robot scenarios, since there
        the context blocks 'with_simulated_robot' and 'with_real_robot' provide the appropriate 'active robot'.
        We do this because we may want to be able to combine simulated and real robots in the same plan.
        However, this is not sufficient for concurrent multi-robot execution,
        for which the robot variable needs to be provided to the action directly.

        :param robot: Robot object
        :return: Currently active robot
        """
        robot = robot or RobotManager.active_robot

        return robot

    @staticmethod
    def get_robot_description(robot: Optional[Object] = None) -> RobotDescription:
        """
        Retrieve the description of the active robot.
        For concurrent robot actions, a robot can be given as parameter and its description is used instead.
        This is an equivalent behaviour to 'get_active_robot'.

        :param robot: Robot object to use. If None is given, use the first robot added instead.

        return: RobotDescription
        """
        if robot is None:
            return RobotDescription.current_robot_description

        if robot.name in RobotDescriptionManager().descriptions.keys():
            return RobotDescriptionManager().descriptions[robot.name]

        raise Exception(f"Robot {robot.name} could not be found in RobotDescriptionManager")

    @staticmethod
    def multiple_robots_active() -> bool:
        """
        State if multiple robots exist to differentiate between single-robot and multi-robot demos.

        return: True if multiple robots exist, False otherwise.
        """
        if len(list(RobotManager.available_robots.keys())) > 1:
            return True

        return False

    @staticmethod
    def set_giskard_robot(robot: Object):
        """
        Set the robot that is supposed to use giskard.

        :param robot: Robot object to set
        """
        RobotManager.giskard_robot = RobotManager.available_robots[robot.name]
