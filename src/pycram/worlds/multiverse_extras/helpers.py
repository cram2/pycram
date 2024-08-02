import os

from typing_extensions import Optional, List, Dict
import xml.etree.ElementTree as ET


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


def get_robot_mjcf_path(company_name: str, robot_name: str, xml_name: Optional[str] = None) -> Optional[str]:
    """
    Get the path to the MJCF file of a robot.
    :param company_name: The name of the company that created the robot.
    :param robot_name: The name of the robot.
    :param xml_name: The name of the XML file of the robot.
    :return: The path to the MJCF file of the robot.
    """
    xml_name = xml_name if xml_name is not None else robot_name
    multiverse_resources = find_multiverse_resources_path()
    if multiverse_resources is not None:
        return os.path.join(multiverse_resources, 'robots', company_name, robot_name, 'mjcf', f'{robot_name}.xml')
    return None


def get_resource_paths(dirname: str) -> List[str]:
    resources_paths = ["../robots", "../worlds", "../objects"]
    resources_paths = [
        os.path.join(dirname, resources_path.replace('../', '')) if not os.path.isabs(
            resources_path) else resources_path
        for resources_path in resources_paths
    ]

    def add_directories(path: str) -> None:
        with os.scandir(path) as entries:
            for entry in entries:
                if entry.is_dir():
                    resources_paths.append(entry.path)
                    add_directories(entry.path)

    resources_path_copy = resources_paths.copy()
    for resources_path in resources_path_copy:
        add_directories(resources_path)

    return resources_paths


def find_multiverse_resources_path() -> Optional[str]:
    """
    Find the path to the Multiverse resources directory.
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
    Find the path to the Multiverse installation.
    """
    # Get the value of PYTHONPATH environment variable
    pythonpath = os.getenv('PYTHONPATH')

    # Check if PYTHONPATH is set
    if pythonpath:
        # Split the PYTHONPATH into individual paths using the platform-specific path separator
        paths = pythonpath.split(os.pathsep)

        # Iterate through each path and check if 'Multiverse' is in it
        for path in paths:
            if 'multiverse' in path:
                multiverse_path = path.split('multiverse')[0]
                return multiverse_path + 'multiverse'

    return None
