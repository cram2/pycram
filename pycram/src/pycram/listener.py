import pycram.helper as helper

from time import time as current_time
from tf import TransformListener, TransformerROS
from rospy import Duration, Time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, TransformStamped

transformer = TransformListener()
# Wait "waiting_time" seconds before lookup of transform or transforming of a pose
waiting_time = 4
# Timeout in seconds for lookup of transform and transforming of a pose
timeout = 5


def get_global_transforms():
    tf_frames = transformer.getFrameStrings()
    looked_up = []
    tf_time = Time(current_time())
    ret = []
    for source_frame in tf_frames:
        for target_frame in tf_frames:
            if (source_frame is not target_frame) and not \
                    any(map((lambda l: source_frame in l and target_frame in l), looked_up)):
                if transformer.canTransform(source_frame, target_frame, tf_time):
                    pos, q = transformer.lookupTransform(source_frame, target_frame, tf_time)
                    tf_stamped = helper.list2tfstamped(source_frame, target_frame, [pos, q], time=tf_time)
                    ret.append([source_frame, target_frame, tf_stamped])
                    looked_up.append([source_frame, target_frame])
    return ret


