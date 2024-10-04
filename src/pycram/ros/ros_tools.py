import rosgraph
import rosnode
import rospy
# import rospkg

from rospkg import RosPack, ResourceNotFound
from typing_extensions import Any


def get_node_names(namespace=None):
    return rosnode.get_node_names(namespace)


def create_ros_pack(ros_paths: Any = None) -> RosPack:
    """
    Creates a RosPack instance to search for resources of ros packages.

    :param ros_paths: An ordered list of paths to search for resources.
    :return: An instance of RosPack
    """
    return RosPack(ros_paths)


def get_ros_package_path(package_name: str) -> str:
    rospack = create_ros_pack()
    return rospack.get_path(package_name)


def get_parameter(name: str) -> Any:
    return rospy.get_param(name)


def wait_for_message(topic_name: str):
    return rospy.wait_for_message(topic_name)


def is_master_online():
    return rosgraph.is_master_online()


def sleep(duration: float):
    rospy.sleep(duration)

def create_timer(duration: int, callback, oneshot=False):
    return rospy.Timer(rospy.Duration(duration), callback, oneshot=oneshot)