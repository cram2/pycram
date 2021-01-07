import pycram.helper as helper

from time import time as current_time
from tf import TransformListener
from rospy import Duration, Time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

transformer = TransformListener()
# Wait "waiting_time" seconds before lookup of transform or transforming of a pose
waiting_time = 3
# Timeout in seconds for lookup of transform and transforming of a pose
timeout = 5


def tf_pose_transform(source_frame, target_frame, pose, time=None):
    tf_pose = helper.ensure_pose(pose)
    tf_time = Time(time) if time else Time(current_time() + waiting_time)
    tf_pose_stamped = PoseStamped(Header(0, tf_time, source_frame), tf_pose)
    return tf_pose_stamped_transform(target_frame, tf_pose_stamped, time)


def tf_pose_stamped_transform(target_frame, pose_stamped, time=None):
    transformer.waitForTransform(Time(time) if time else pose_stamped.header.frame_id, target_frame,
                                 time=pose_stamped.header.stamp, timeout=Duration(timeout))
    return helper.pose_stamped2tuple(transformer.transformPose(target_frame, pose_stamped))


def tf_transform(source_frame, target_frame, time=None):
    tf_time = Time(time) if time else Time(current_time() + waiting_time)
    transformer.waitForTransform(source_frame, target_frame, time=tf_time, timeout=Duration(timeout))
    return transformer.lookupTransform(source_frame, target_frame, tf_time)
