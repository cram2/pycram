import pycram.helper as helper

from tf import TransformListener
from rospy import Duration

transformer = TransformListener()


def tf_pose_transform(target_frame, pose):
    tf_pose = helper.ensure_pose(pose)
    return helper.pose_stamped2tuple(transformer.transformPose(target_frame, tf_pose))


def tf_transform(source_frame, target_frame, time, timeout):
    transformer.waitForTransform(source_frame, target_frame, time=time, timeout=Duration(timeout))
    return transformer.lookupTransform(source_frame, target_frame, time)
