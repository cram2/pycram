import os
if os.environ.get('ROS_VERSION') == '1':
    from .ros1 import *
elif os.environ.get('ROS_VERSION') == '2':
    from .ros2 import *
else:
    from .no_ros import *