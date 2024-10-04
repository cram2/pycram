import rospy
from .ros_tools import is_master_online

# Check is for sphinx autoAPI to be able to work in a CI workflow
if is_master_online():
    rospy.init_node("pycram")


class viz_maker_publisher:

    def VizMarkerPublisher(self):
        raise DeprecationWarning("This function moved and can now be found in pycram.ros_utils.viz_marker_publisher")