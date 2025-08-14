"""Implementation of helper functions and classes for internal usage only.

Classes:
Singleton -- implementation of singleton metaclass
"""
import os
from typing_extensions import Dict, Optional, Tuple
import xml.etree.ElementTree as ET

from .datastructures.enums import DescriptionType
from .ros import  loginfo


class Singleton(type):
    """
    Metaclass for singletons
    """

    _instances = {}
    """
    Dictionary of singleton child classes inheriting from this metaclass, keyed by child class objects.
    """

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


def get_robot_urdf_and_mjcf_file_paths(robot_name: str, robot_relative_dir: str,
                                       multiverse_resources: Optional[str] = None)\
        -> Tuple[Optional[str], Optional[str]]:
    """
    Get the paths to the MJCF and URDF files of a robot from the Multiverse resources directory.

    :param robot_name: The name of the robot.
    :param robot_relative_dir: The relative directory of the robot in the Multiverse resources/robots directory.
    :param multiverse_resources: The path to the Multiverse resources directory.
    """
    multiverse_resources = find_multiverse_resources_path() if multiverse_resources is None else multiverse_resources

    urdf_filename: Optional[str] = None
    mjcf_filename: Optional[str] = None
    if multiverse_resources is not None:
        urdf_filename = get_robot_description_path(robot_relative_dir, robot_name,
                                                   description_type=URDFObject,
                                                   resources_dir=multiverse_resources)
        mjcf_filename = get_robot_description_path(robot_relative_dir, robot_name,
                                                   resources_dir=multiverse_resources)
    return urdf_filename, mjcf_filename


def parse_mjcf_actuators(file_path: str) -> Dict[str, str]:
    """
    Parse the actuator elements from an MJCF file.

    :param file_path: The path to the MJCF file.
    """
    tree = ET.parse(file_path)
    root = tree.getroot()

    joint_actuators = {}

    # Iterate through all actuator elements
    for actuator in root.findall(".//actuator/*"):
        name = actuator.get('name')
        joint = actuator.get('joint')
        if name and joint:
            joint_actuators[joint] = name

    return joint_actuators


def get_robot_description_path(robot_relative_dir: str, robot_name: str,
                               description_type: DescriptionType = DescriptionType.MJCF,
                               file_name: Optional[str] = None,
                               resources_dir: Optional[str] = None) -> Optional[str]:
    """
    Get the path to the description file of a robot.

    :param robot_relative_dir: The relative directory of the robot in the resources/robots directory.
    :param robot_name: The name of the robot.
    :param description_type: The type of the description (URDF, MJCF).
    :param file_name: The name of the XML file of the robot.
    :param resources_dir: The path to the Multiverse resources directory.
    :return: The path to the description file of the robot if it exists, otherwise None.
    """
    resources_dir = find_multiverse_resources_path() if resources_dir is None else resources_dir
    if resources_dir is None:
        loginfo("Could not find Multiverse resources path and no other resources were given.")
        return None
    file_name = file_name if file_name is not None else robot_name
    extension = description_type.get_file_extension()
    extension_folders = []
    if extension[1:] in ['xml', 'mjcf']:
        extension_folders.append('mjcf')
    else:
        extension_folders.append(extension[1:])
    if extension not in file_name:
        file_name = file_name + extension

    robot_folders = [os.path.join(resources_dir, 'robots', robot_relative_dir)]
    robot_folders.append(os.path.join(robot_folders[0], robot_name))
    for robot_folder in robot_folders:
        if resources_dir is not None and os.path.exists(robot_folder):
            list_dir = os.listdir(robot_folder)
            for extension_folder in extension_folders:
                if extension_folder in list_dir:
                    if file_name in os.listdir(robot_folder + f'/{extension_folder}'):
                        return os.path.join(robot_folder, extension_folder, file_name)
                elif file_name in os.listdir(robot_folder):
                    return os.path.join(robot_folder, file_name)
    logwarn(f"Robot {robot_name} not found in resources.")
    return None


def find_multiverse_resources_path() -> Optional[str]:
    """
    :return: The path to the Multiverse resources directory.
    """
    # Get the path to the Multiverse installation
    multiverse_path = find_multiverse_path()

    # Check if the path to the Multiverse installation was found
    if multiverse_path:
        # Construct the path to the resources directory
        resources_path = os.path.join(multiverse_path, 'resources')

        # Check if the resources directory exists
        if os.path.exists(resources_path):
            return resources_path

    return None


def find_multiverse_path() -> Optional[str]:
    """
    :return: the path to the Multiverse installation.
    """
    # Get the value of PYTHONPATH environment variable
    pythonpath = os.getenv('PYTHONPATH')
    multiverse_relative_path = "Multiverse/multiverse"

    # Check if PYTHONPATH is set
    if pythonpath:
        # Split the PYTHONPATH into individual paths using the platform-specific path separator
        paths = pythonpath.split(os.pathsep)

        # Iterate through each path and check if 'Multiverse' is in it
        for path in paths:
            if multiverse_relative_path in path:
                multiverse_path = path.split(multiverse_relative_path)[0]
                return multiverse_path + multiverse_relative_path


def perform(action_instance):
    """
    Executes the perform logic for a given action instance.

    :param action_instance: An instance of an action class.
    """
    return action_instance.perform()


def an(designator):
    """
    Resolve the first available action from the designator.

    :param designator: The designator description instance.
    :return: The first resolved action instance.
    """
    return designator.resolve()
