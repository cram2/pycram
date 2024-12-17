from .data_types import *
from .ros_tools import *

import rclpy
import threading
from rclpy.node import Node

rclpy.init()
node = Node('pycram')
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
