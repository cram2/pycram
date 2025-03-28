from __future__ import annotations
import sys
import logging
import numpy as np
from .ros import Time, Duration, logerr
from geometry_msgs.msg import TransformStamped, PoseStamped
from transforms3d.quaternions import quat2mat, mat2quat

if 'world' in sys.modules:
    logging.warning("(publisher) Make sure that you are not loading this module from pycram.world.")

class TransformerROS:
    pass

from .datastructures.pose import Pose, Transform
from typing_extensions import List, Optional, Union, Iterable, TYPE_CHECKING, Tuple

if TYPE_CHECKING:
    from .world_concepts.world_object import Object
    from .datastructures.world import World


from tf2_ros import Buffer

def _build_affine(
        rotation: Optional[Iterable] = None,
        translation: Optional[Iterable] = None) -> np.ndarray:
    """
    Build an affine matrix from a quaternion and a translation.

    :param rotation: The quaternion as [w, x, y, z]
    :param translation: The translation as [x, y, z]
    :returns: The quaternion and the translation array
    """
    affine = np.eye(4)
    if rotation is not None:
        affine[:3, :3] = quat2mat(np.asarray(rotation))
    if translation is not None:
        affine[:3, 3] = np.asarray(translation)
    return affine

def _transform_to_affine(transform: TransformStamped) -> np.ndarray:
    """
    Convert a `TransformStamped` to a affine matrix.

    :param transform: The transform that should be converted
    :returns: The affine transform
    """
    transform = transform.transform
    transform_rotation_matrix = [
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z
    ]
    transform_translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ]
    return _build_affine(transform_rotation_matrix, transform_translation)


def _decompose_affine(affine: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Decompose an affine transformation into a quaternion and the translation.

    :param affine: The affine transformation matrix
    :returns: The quaternion and the translation array
    """
    return mat2quat(affine[:3, :3]), affine[:3, 3]

# Pose
def do_transform_pose(
        pose: Pose,
        transform: TransformStamped) -> Pose:
    """
    Transform a `Pose` using a given `TransformStamped`.

    This method is used to share the tranformation done in
    `do_transform_pose_stamped()` and `do_transform_pose_with_covariance_stamped()`

    :param pose: The pose
    :param transform: The transform
    :returns: The transformed pose
    """
    quaternion, point = _decompose_affine(
        np.matmul(
            _transform_to_affine(transform),
            _build_affine(
                translation=[
                    pose.position.x,
                    pose.position.y,
                    pose.position.z
                ],
                rotation=[
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z])))
    res = Pose()
    res.position.x = point[0]
    res.position.y = point[1]
    res.position.z = point[2]
    res.orientation.w = quaternion[0]
    res.orientation.x = quaternion[1]
    res.orientation.y = quaternion[2]
    res.orientation.z = quaternion[3]
    return res


# PoseStamped
def do_transform_pose_stamped(
        pose: PoseStamped,
        transform: TransformStamped) -> PoseStamped:
    """
    Transform a `PoseStamped` using a given `TransformStamped`.

    :param pose: The stamped pose
    :param transform: The transform
    :returns: The transformed pose stamped
    """
    res = PoseStamped()
    res.pose = do_transform_pose(pose.pose, transform)
    res.header = transform.header
    return res



class LocalTransformer(Buffer):
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
    prospection_prefix: str = "prospection/"

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        # super().__init__(interpolate=True, cache_time=Duration(10))
        super().__init__(cache_time=Duration(10))
        # Since this file can't import world.py this holds the reference to the current_world
        self.world = None
        # TODO: Ask Jonas if this is still needed
        self.prospection_world = None

        # If the singelton was already initialized
        self._initialized = True
        self.registration.add(Pose, do_transform_pose_stamped)
        self.registration.add(PoseStamped, do_transform_pose_stamped)

    def transform_to_object_frame(self, pose: Pose,
                                  world_object: Object, link_name: str = None) -> Union[
        Pose, None]:
        """
        Transforms the given pose to the coordinate frame of the given World object. If no link name is given the
        base frame of the Object is used, otherwise the link frame is used as target for the transformation.

        :param pose: Pose that should be transformed
        :param world_object: BulletWorld Object in which frame the pose should be transformed
        :param link_name: A link of the BulletWorld Object which will be used as target coordinate frame instead
        :return: The new pose the in coordinate frame of the object
        """
        if link_name:
            target_frame = world_object.get_link_tf_frame(link_name)
        else:
            target_frame = world_object.tf_frame
        return self.transform_pose(pose, target_frame)

    def update_transforms_for_objects(self, objects: List[Object]) -> None:
        """
        Updates the transforms for objects affected by the transformation. The objects are identified by their names.

        :param objects: List of objects for which the transforms should be updated
        """
        [obj.update_link_transforms() for obj in objects]

    def transform_pose(self, pose: Pose, target_frame: str) -> Optional[Pose]:
        """
        Transforms a given pose to the target frame after updating the transforms for all objects in the current world.

        :param pose: Pose that should be transformed
        :param target_frame: Name of the TF frame into which the Pose should be transformed
        :return: A transformed pose in the target frame
        """
        objects = list(map(self.get_object_from_frame, [pose.frame, target_frame]))
        self.update_transforms_for_objects([obj for obj in objects if obj is not None])

        copy_pose = pose.copy()
        copy_pose.header.stamp = Time(0)
        if not self.can_transform(target_frame, pose.frame, Time(0)):
            logerr(
                f"Can not transform pose: \n {pose}\n to frame: {target_frame}."
                f"\n Maybe try calling 'update_transforms_for_object'")
            return
        # new_pose = super().transformPose(target_frame, copy_pose)
        new_pose = self.transform(copy_pose.to_pose_stamped(), target_frame)

        copy_pose.pose = new_pose.pose
        copy_pose.header.frame_id = new_pose.header.frame_id
        copy_pose.header.stamp = Time().now()

        return Pose(*copy_pose.to_list(), frame=new_pose.header.frame_id)

    def get_object_from_frame(self, frame: str) -> Optional[Object]:
        """
        Get the name of the object that is associated with the given frame.

        :param frame: The frame for which the object name should be returned
        :return: The name of the object associated with the frame
        """
        if frame == "map":
            return None
        world = self.get_world_from_frame(frame)
        found_objects = [obj for obj in world.objects if frame == obj.tf_frame]
        return found_objects[0] if len(found_objects) > 0 else self.get_object_from_link_frame(frame)

    def translate_pose_along_local_axis(self, pose: Pose, axis: List, distance: float) -> Pose:
        """
        Translate a pose along a given 3d vector (axis) by a given distance. The axis is given in the local coordinate
        frame of the pose. The axis is normalized and then scaled by the distance.

        :param pose: The pose that should be translated
        :param axis: The local axis along which the translation should be performed
        :param distance: The distance by which the pose should be translated

        :return: The translated pose
        """
        transform = pose.to_transform("self")
        normalized_translation_vector = np.array(axis) / np.linalg.norm(axis)
        scaled_translation_vector = (normalized_translation_vector * distance).tolist()
        translation = Transform(scaled_translation_vector, [0, 0, 0, 1], "self", "translation",
                                pose.header.stamp)
        return (transform * translation).to_pose()

    def get_object_from_link_frame(self, link_frame: str) -> Optional[Object]:
        """
        Get the name of the object that is associated with the given link frame.

        :param link_frame: The frame of the link for which the object name should be returned
        :return: The name of the object associated with the link frame
        """
        world = self.get_world_from_frame(link_frame)
        found_objects = [obj for obj in world.objects for link in obj.links.values()
                         if link_frame in (link.name, link.tf_frame)]
        return found_objects[0] if len(found_objects) > 0 else None

    def get_world_from_frame(self, frame: str) -> World:
        """
        Get the world that is associated with the given frame name.

        :param frame: The frame name.
        """
        return self.prospection_world if self.prospection_prefix in frame else self.world

    def lookup_transform_from_source_to_target_frame(self, source_frame: str, target_frame: str,
                                                     time: Optional[Time] = None) -> Transform:
        """
        Update the transforms for all world objects then Look up for the latest known transform that transforms a point
         from source frame to target frame. If no time is given the last common time between the two frames is used.

        :param source_frame: The frame in which the point is currently represented
        :param target_frame: The frame in which the point should be represented
        :param time: Time at which the transform should be looked up
        :return: The transform from source_frame to target_frame
        """
        objects = list(map(self.get_object_from_frame, [source_frame, target_frame]))
        self.update_transforms_for_objects([obj for obj in objects if obj is not None])

        tf_time = time if time else self.get_latest_common_time(source_frame, target_frame)
        transform_stamped = self.lookup_transform(source_frame, target_frame, tf_time)
        return Transform(transform_stamped.transform.translation,
                         transform_stamped.transform.rotation,
                         source_frame, target_frame)

    def update_transforms(self, transforms: Iterable[Transform], time: Time = None) -> None:
        """
        Updates transforms by updating the time stamps of the header of each transform. If no time is given the current
        time is used.
        """
        time = time if time else Time().now()
        for transform in transforms:
            transform.header.stamp = time
            self.set_transform(transform, "pycram/local_transformer")

    def get_all_frames(self) -> List[str]:
        """
        :return: A list of all known coordinate frames as a list with human-readable entries.
        """
        frames = self.all_frames_as_string().split("\n")
        frames.remove("")
        return frames