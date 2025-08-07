from abc import ABC

from pycram.robot_description import RobotDescriptionManager, RobotDescription
from pycram.ros import logdebug


class RobotManager(ABC):
    """
    Class for managing multiple robots simultaneously
    """

    _instance = None
    """
    Singelton instance of this Robot Manager
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
        :return: Singelton instance of this Robot Manager
        """
        if not cls._instance:
            cls._instance = super(RobotManager, cls).__new__(cls)
            return cls._instance
        else:
            return cls._instance

    def __init__(self):
        """
        Initialize RobotManager
        """
        rdm = RobotDescriptionManager()
        logdebug(f'Initialize RobotManager.')

    @staticmethod
    def add_robot(robot):
        """
        Add another robot to the list of available robots

        :param robot: Robot object to add
        """
        RobotManager.available_robots[robot.name] = robot

    @staticmethod
    def set_active_robot(robot_name: str = None):
        """
        Set the currently active robot based on its name.

        :param robot_name: Robot name
        """
        rdm = RobotDescriptionManager()
        RobotManager.robot_description = rdm.load_description(name=robot_name)
        RobotManager.active_robot = RobotManager.available_robots[robot_name] if robot_name else None
        RobotManager.set_giskard_robot(robot_name) if robot_name else None
        logdebug(f'Setting active robot. Is now: {robot_name}')

    @staticmethod
    def get_active_robot(robot=None):
        """
        Returns the active robot if the given variable is None.
        This differentiation is used for handling multiple robots in multithreaded Actions

        :param robot: Robot object
        :return: Currently active robot
        """
        robot = robot or RobotManager.active_robot

        return robot

    @staticmethod
    def get_robot_description(robot=None) -> RobotDescription:
        """
        Retrieve the description of the active robot.
        For concurrent robot actions, a robot can be given as parameter and its description is used instead.

        :param robot: Robot object to use. If None is given, use the first robot added instead.

        return: RobotDescription
        """
        if robot is None:
            return RobotDescription.current_robot_description

        if robot.name in RobotDescriptionManager().descriptions.keys():
            return RobotDescriptionManager().descriptions[robot.name]

        raise Exception(f"Robot {robot.name} could not be found in RobotDescriptionManager")

    @staticmethod
    def multiple_robots_active():
        """
        State if multiple robots exist to differentiate between single-robot and multi-robot demos.

        return: True if multiple robots exist, False otherwise.
        """
        if len(list(RobotManager.available_robots.keys())) > 1:
            return True

        return False

    @staticmethod
    def set_giskard_robot(robot_name: str):
        """
        Set the giskard robot with the given robot

        :param robot_name: Name of the robot
        """
        RobotManager.giskard_robot = RobotManager.available_robots[robot_name]
