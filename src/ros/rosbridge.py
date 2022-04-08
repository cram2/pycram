import logging
import os
from urllib.parse import urlparse

import roslibpy

ros_host = urlparse(os.environ["ROS_MASTER_URI"]).hostname
rosbridge_port = 9090

ros_client = roslibpy.Ros(ros_host, rosbridge_port)
ros_client.run()
