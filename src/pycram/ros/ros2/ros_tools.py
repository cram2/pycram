from . import node
from ...logging import logwarn
import rclpy
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory

def get_node_names(namespace=None):
    """
    Get the names of all nodes in the ROS system.

    :param namespace: The namespace to search for nodes.
    :return: A list of node names.
    """
    return node.get_node_names()

def create_ros_pack(ros_paths=None):
    logwarn("create_ros_pack is not implemented in ROS 2. Please use get_ros_package_path.")
    return None

def get_ros_package_path(package_name):
    return get_package_share_directory(package_name=package_name)

def get_parameter(name):
    try:
        node_names_and_namespaces = node.get_node_names_and_namespaces()
        for node_name, namespace in node_names_and_namespaces:
            client = node.create_client(GetParameters, f'{namespace}/{node_name}/get_parameters')
            request = GetParameters.Request()
            request.names = [name]

            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)
            client.destroy()
            if future.result() is not None and future.result().values:
                value = future.result().values[0].value
                return value
    except Exception as e:
        print(f"Failed to get parameter '{name}': {e}")

def wait_for_message(topic_name, msg_type):
    last_message = None
    def sub_callback(msg):
        global last_message
        last_message = msg
        sub.destroy()

    sub = node.create_subscription(msg_type, topic_name, sub_callback())
    return last_message

def is_master_online():
    return node is not None

def sleep(duration):
    """
    Sleep for a given duration.

    :param duration: The duration to sleep in seconds.
    """
    rate = node.create_rate(1 / duration)
    rate.sleep()

def get_time():
    return node.get_time()

def create_timer(duration, callback, oneshot=False):
    timer = node.create_timer(duration, callback, autostart=True)
    return timer

class ResourceNotFound(Exception):
    def __init__(self, *args, **kwargs):
        """Create a new plan failure."""
        Exception.__init__(self, *args, **kwargs)

class ServiceException(Exception):
    def __init__(self, *args, **kwargs):
        """Create a new plan failure."""
        Exception.__init__(self, *args, **kwargs)

