import os
from urllib.parse import urlparse

import roslibpy

from pycram.helper import SingletonMeta


class ROSBridge(metaclass=SingletonMeta):
    def __init__(self):
        ros_host, ros_port = self._get_ros_master_host_and_port()
        self.ros_client = roslibpy.Ros(ros_host, ros_port)

    @staticmethod
    def get_ros_master_host_and_port():
        uri = urlparse(os.environ["ROS_MASTER_URI"])
        return uri.hostname, 9090

    def __enter__(self):
        self.ros_client.run()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.ros_client.terminate()
