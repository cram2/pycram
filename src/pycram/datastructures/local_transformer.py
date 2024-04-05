import sys
import logging

if 'world' in sys.modules:
    logging.warning("(publisher) Make sure that you are not loading this module from pycram.world.")
import rospy

from tf import TransformerROS
from rospy import Duration

from geometry_msgs.msg import TransformStamped
from pycram.datastructures.pose import Pose, Transform
from typing_extensions import List, Optional, Union, Iterable


class LocalTransformer(TransformerROS):
    """
    This class allows to use the TF class TransformerROS without using the ROS
    network system or the topic /tf, where transforms are usually published to.
    Instead, a local transformer is saved and allows to publish local transforms,
    as well the use of TFs convenient lookup functions (see functions below).

    This class uses the robots (currently only one! supported) URDF file to
    initialize the tfs for the robot. Moreover, the function update_local_transformer_from_btr
    updates these tfs by copying the tfs state from the world.

    This class extends the TransformerRos, you can find documentation for TransformerROS here:
    `TFDoc <http://wiki.ros.org/tf/TfUsingPython>`_
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        super().__init__(interpolate=True, cache_time=Duration(10))

        # TF tf_stampeds and static_tf_stampeds of the Robot in the World:
        # These are initialized with the function init_transforms_from_urdf and are
        # used to update the local transformer with the function update_local_transformer_from_btr
        # TODO: Ask Jonas if this is still needed
        self.tf_stampeds: List[TransformStamped] = []
        self.static_tf_stampeds: List[TransformStamped] = []

        # Since this file can't import world.py this holds the reference to the current_world
        self.world = None
        # TODO: Ask Jonas if this is still needed
        self.prospection_world = None

        # If the singelton was already initialized
        self._initialized = True

    def transform_pose(self, pose: Pose, target_frame: str) -> Union[Pose, None]:
        """
        Transforms a given pose to the target frame after updating the transforms for all objects in the current world.

        :param pose: Pose that should be transformed
        :param target_frame: Name of the TF frame into which the Pose should be transformed
        :return: A transformed pose in the target frame
        """
        self.world.update_transforms_for_objects_in_current_world()
        copy_pose = pose.copy()
        copy_pose.header.stamp = rospy.Time(0)
        if not self.canTransform(target_frame, pose.frame, rospy.Time(0)):
            rospy.logerr(
                f"Can not transform pose: \n {pose}\n to frame: {target_frame}.\n Maybe try calling 'update_transforms_for_object'")
            return
        new_pose = super().transformPose(target_frame, copy_pose)

        copy_pose.pose = new_pose.pose
        copy_pose.header.frame_id = new_pose.header.frame_id
        copy_pose.header.stamp = rospy.Time.now()

        return Pose(*copy_pose.to_list(), frame=new_pose.header.frame_id)

    def lookup_transform_from_source_to_target_frame(self, source_frame: str, target_frame: str,
                                                     time: Optional[rospy.rostime.Time] = None) -> Transform:
        """
        Update the transforms for all world objects then Look up for the latest known transform that transforms a point
         from source frame to target frame. If no time is given the last common time between the two frames is used.
        :param time: Time at which the transform should be looked up
        """
        self.world.update_transforms_for_objects_in_current_world()
        tf_time = time if time else self.getLatestCommonTime(source_frame, target_frame)
        translation, rotation = self.lookupTransform(source_frame, target_frame, tf_time)
        return Transform(translation, rotation, source_frame, target_frame)

    def update_transforms(self, transforms: Iterable[Transform], time: rospy.Time = None) -> None:
        """
        Updates transforms by updating the time stamps of the header of each transform. If no time is given the current
        time is used.
        """
        time = time if time else rospy.Time.now()
        for transform in transforms:
            transform.header.stamp = time
            self.setTransform(transform)

    def get_all_frames(self) -> List[str]:
        """
        Returns all know coordinate frames as a list with human-readable entries.

        :return: A list of all know coordinate frames.
        """
        frames = self.allFramesAsString().split("\n")
        frames.remove("")
        return frames

    def transformPose(self, target_frame, ps) -> Pose:
        """
        Alias for :func:`~LocalTransformer.transform_pose_to_target_frame` to avoid confusion since a similar method
         exists in the super class.
        """
        return self.transform_pose(ps, target_frame)

