# Funky MonkeyPatching for ROS2 compatability
import atexit
import signal

import builtin_interfaces.msg
def to_sec(self):
    """
    Returns the time in seconds from a builtin_interfaces.msg.Time message.

    :return: The time in seconds.
    """
    return self.sec

builtin_interfaces.msg.Time.to_sec = to_sec

import rclpy
import threading
from rclpy.node import Node

rclpy.init()
node = Node('pycram')
atexit.register(rclpy.shutdown)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()



from .data_types import *
from .ros_tools import  *
from .logging import *
from .action_lib import *
from .service import *
from .publisher import *
from .subscriber import *
from .viz_marker_publisher import *