import os
if os.environ.get('ROS_VERSION') == '1':
    # import rospy
    # from .ros import  *
    # from .ros import  *
    # from .ros import  *
    # from .ros import  *
    # from .ros import  *
    # from .ros import  *
    # from .ros import  *
    # from .ros import  *
    #
    # # Check is for sphinx autoAPI to be able to work in a CI workflow
    # if is_master_online():
    #     rospy.init_node("pycram")
    from .ros1 import  *
elif os.environ.get('ROS_VERSION') == '2':
    from .ros2 import  *