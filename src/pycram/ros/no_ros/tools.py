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