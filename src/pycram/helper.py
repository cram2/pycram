"""Implementation of helper functions and classes for internal usage only.

Classes:
Singleton -- implementation of singleton metaclass
"""
import os
from typing_extensions import Dict, Optional
import xml.etree.ElementTree as ET

from pycram.ros.logging import logwarn


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


def get_robot_mjcf_path(robot_relative_dir: str, robot_name: str, xml_name: Optional[str] = None) -> Optional[str]:
    """
    Get the path to the MJCF file of a robot.

    :param robot_relative_dir: The relative directory of the robot in the Multiverse resources/robots directory.
    :param robot_name: The name of the robot.
    :param xml_name: The name of the XML file of the robot.
    :return: The path to the MJCF file of the robot if it exists, otherwise None.
    """
    xml_name = xml_name if xml_name is not None else robot_name
    if '.xml' not in xml_name:
        xml_name = xml_name + '.xml'
    multiverse_resources = find_multiverse_resources_path()
    try:
        robot_folder = os.path.join(multiverse_resources, 'robots', robot_relative_dir, robot_name)
    except TypeError:
        logwarn("Multiverse resources path not found.")
        return None
    if multiverse_resources is not None:
        list_dir = os.listdir(robot_folder)
        if 'mjcf' in list_dir:
            if xml_name in os.listdir(robot_folder + '/mjcf'):
                return os.path.join(robot_folder, 'mjcf', xml_name)
        elif xml_name in os.listdir(robot_folder):
            return os.path.join(robot_folder, xml_name)
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


