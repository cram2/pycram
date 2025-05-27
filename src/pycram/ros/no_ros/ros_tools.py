import os
from functools import lru_cache


@lru_cache(maxsize=None)
def get_ros_package_path(package_name: str) -> str:
    """
    Get the path of a ROS package. Using the os module to avoid importing rospkg.
    """
    home_dir = os.path.expanduser('~')
    # search recursively in the home directory
    for root, dirs, files in os.walk(home_dir):
        if package_name in dirs:
            return os.path.join(root, package_name)
    raise FileNotFoundError(f"Package '{package_name}' not found in the home directory.")

def sleep(duration: float):
    time.sleep(duration)

def get_node_names(namespace=None):
    """
    Get the names of all nodes in the ROS system.
    """
    # This is a placeholder implementation. In a real implementation, you would use
    # the appropriate ROS API to get the node names.
    return []

def create_ros_pack(ros_paths=None):
    """
    Creates a RosPack instance to search for resources of ros packages.
    """
    # This is a placeholder implementation. In a real implementation, you would use
    # the appropriate ROS API to create a RosPack instance.
    return None

class ResourceNotFound(Exception):
    """
    Exception raised when a resource is not found.
    """
    pass

def get_parameter(name: str):
    """
    Get a parameter from the ROS parameter server.
    """
    # This is a placeholder implementation. In a real implementation, you would use
    # the appropriate ROS API to get the parameter.
    return None

def wait_for_message(topic_name: str, msg_type):
    """
    Wait for a message on a topic.
    """
    # This is a placeholder implementation. In a real implementation, you would use
    # the appropriate ROS API to wait for the message.
    return None

def is_master_online():
    """
    Check if the ROS master is online.
    """
    # This is a placeholder implementation. In a real implementation, you would use
    # the appropriate ROS API to check if the master is online.
    return False


def get_time():
    """
    Get the current time from the ROS system.
    """
    # This is a placeholder implementation. In a real implementation, you would use
    # the appropriate ROS API to get the time.
    return 0.0