import sys
import logging

if 'bullet_world' in sys.modules:
    logging.warning("(publisher) Make sure that you are not loading this module from pycram.bullet_world.")
import rospkg
import rospy
import atexit

from threading import Thread, currentThread
from tf import TransformerROS, transformations
from rospy import Duration, logerr, Rate, is_shutdown
from urdf_parser_py.urdf import URDF

from geometry_msgs.msg import TransformStamped
from .pose import Pose
from .robot_descriptions import robot_description
from typing import List, Optional, Tuple, Union, Callable


class LocalTransformer(TransformerROS):
    """
    This class allows to use the TF class TransformerROS without using the ROS
    network system or the topic /tf, where transforms are usually published to.
    Instead, a local transformer is saved and allows to publish local transforms,
    as well the use of TFs convenient lookup functions (see functions below).

    This class uses the robots (currently only one! supported) URDF file to
    initialize the tfs for the robot. Moreover, the function update_local_transformer_from_btr
    updates these tfs by copying the tfs state from the pybullet world.

    This class extends the TransformerRos, you can find documentation for TransformerROS here:
    `TFDoc <http://wiki.ros.org/tf/TfUsingPython>`_
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(LocalTransformer, cls).__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        super().__init__(interpolate=True, cache_time=Duration(10))

        # TF tf_stampeds and static_tf_stampeds of the Robot in the BulletWorld:
        # These are initialized with the function init_transforms_from_urdf and are
        # used to update the local transformer with the function update_local_transformer_from_btr
        self.tf_stampeds: List[TransformStamped] = []
        self.static_tf_stampeds: List[TransformStamped] = []

        # Since this file can't import bullet_world.py this holds the reference to the current_bullet_world
        self.bullet_world = None

        # If the singelton was already initialized
        self._initialized = True

    def init_transforms_from_urdf(self) -> None:
        """
        This static function gets with the robot name in robot_description.py, the
        robots URDF file from the "resources" folder in the ROS "pycram" folder.
        All joints of the URDF are extracted and saved in a list of static and
        normal TFStamped.

        :returns: list of static_tf_stampeds, list of tf_stampeds
        """
        # Save joint translations and orientations in tf_stampeds
        # static_tf_stampeds saves only the static joint translations and orientations
        self.tf_stampeds = []
        self.static_tf_stampeds = []
        # Get URDF file path
        robot_name = robot_description.name
        rospack = rospkg.RosPack()
        filename = rospack.get_path('pycram') + '/resources/' + robot_name + '.urdf'
        # ... and open it
        with open(filename) as f:
            s = f.read()
            robot = URDF.from_xml_string(s)
            # Save for every joint in the robot the source_frame, target_frame,
            # aswell the joints origin, where the translation xyz and rotation rpy
            # are saved
            for joint in robot.joints:

                source_frame = self.projection_namespace + '/' + joint.parent
                target_frame = self.projection_namespace + '/' + joint.child

                if joint.origin:
                    if joint.origin.xyz:
                        translation = joint.origin.xyz
                    else:
                        translation = [0, 0, 0]
                    if joint.origin.rpy:
                        roll = joint.origin.rpy[0]
                        pitch = joint.origin.rpy[1]
                        yaw = joint.origin.rpy[2]
                        rotation = transformations.quaternion_from_euler(roll, pitch, yaw)
                    else:
                        rotation = [0, 0, 0, 1]

                if joint.name in robot_description.odom_joints:
                    # since pybullet wont update this joints, these are declared as static
                    translation = [0, 0, 0]
                    rotation = [0, 0, 0, 1]

                # Wrap the joint attributes in a TFStamped and append it to static_tf_stamped if the joint was fixed
                tf_stamped = pycram.helper_deprecated.list2tfstamped(source_frame, target_frame, [translation, rotation])
                if (joint.type and joint.type == 'fixed') or joint.name in robot_description.odom_joints:
                    self.static_tf_stampeds.append(tf_stamped)
                else:
                    self.tf_stampeds.append(tf_stamped)

    def update_objects(self) -> None:
        """
        Updates transformations for all objects that are currently in :py:attr:´~BulletWorld.current_bullet_world´
        """
        if self.bullet_world:
            for obj in list(self.bullet_world.current_bullet_world.objects):
                self.update_transforms_for_object(obj)

    def transform_pose(self, pose: Pose, target_frame: str) -> Union[Pose, None]:
        """
        Transforms a given pose to the target frame.

        :param pose: Pose that should be transformed
        :param target_frame: Name of the TF frame into which the Pose should be transformed
        :return: A transformed pose in the target frame
        """
        copy_pose = pose.copy()
        copy_pose.header.stamp = rospy.Time(0)
        if not self.canTransform(target_frame, pose.frame, rospy.Time(0)):
            rospy.logerr(
                f"Can not transform pose: \n {pose}\n to frame: {target_frame}.\n Maybe try calling 'update_transforms_for_object'")
            return
        new_pose = self.transformPose(target_frame, copy_pose)

        copy_pose.pose = new_pose.pose
        copy_pose.header.frame_id = new_pose.header.frame_id
        copy_pose.header.stamp = rospy.Time.now()

        return Pose(*copy_pose.to_list(), frame=new_pose.header.frame_id)

    def transform_to_object_frame(self, pose: Pose,
                                  bullet_object: 'bullet_world.Object', link_name: str = None) -> Union[Pose, None]:
        """
        Transforms the given pose to the coordinate frame of the given BulletWorld object. If no link name is given the
        base frame of the Object is used, otherwise the link frame is used as target for the transformation.

        :param pose: Pose that should be transformed
        :param bullet_object: BulletWorld Object in which frame the pose should be transformed
        :param link_name: A link of the BulletWorld Object which will be used as target coordinate frame instead
        :return: The new pose the in coordinate frame of the object
        """
        if link_name:
            target_frame = bullet_object.get_link_tf_frame(link_name)
        else:
            target_frame = bullet_object.tf_frame
        return self.transform_pose(pose, target_frame)

    def tf_transform(self, source_frame: str, target_frame: str,
                     time: Optional[rospy.rostime.Time] = None) -> TransformStamped:
        """
        Returns the latest known transform between the 'source_frame' and 'target_frame'. If no time is given the last
        common time between the two frames is used.

        :param source_frame: Source frame of the transform
        :param target_frame: Target frame of the transform
        :param time: Time at which the transform should be
        :return:
        """
        tf_time = time if time else self.getLatestCommonTime(source_frame, target_frame)
        return self.lookupTransform(source_frame, target_frame, tf_time)

    def update_transforms_for_object(self, bullet_object: 'bullet_world.Object') -> None:
        """
        Updates local transforms for a Bullet Object, this includes the base as well as all links

        :param bullet_object: Object for which the Transforms should be updated
        """
        self.setTransform(
            bullet_object.get_pose().to_transform(bullet_object.tf_frame))
        for link_name, id in bullet_object.links.items():
            if id == -1:
                continue
            tf_stamped = bullet_object.get_link_pose(link_name).to_transform(
                bullet_object.get_link_tf_frame(link_name))
            self.setTransform(tf_stamped)

    def get_all_frames(self) -> List[str]:
        """
        Returns all know coordinate frames as a list with human-readable entries.

        :return: A list of all know coordinate frames.
        """
        frames = self.allFramesAsString().split("\n")
        frames.remove("")
        return frames

