import os
if os.environ.get('ROS_VERSION') == '1':
    import rospy
    from .ros1.ros_tools import *
    from .ros1.logging import *
    from .ros1.service import *
    from .ros1.action_lib import *
    from .ros1.data_types import *
    from .ros1.publisher import *
    from .ros1.subscriber import *
    from .ros1.viz_marker_publisher import *

    # Check is for sphinx autoAPI to be able to work in a CI workflow
    if is_master_online():
        rospy.init_node("pycram")
elif os.environ.get('ROS_VERSION') == '2':
    pass