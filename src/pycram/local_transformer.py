from __future__ import annotations

import sys
import logging

import numpy as np
from pytransform3d.transform_manager import TransformManager
from pytransform3d.transformations import transform_from_pq
from pytransform3d.rotations import quaternion_from_matrix

from .ros import Time

from .tf_transformations import quaternion_matrix

if 'world' in sys.modules:
    logging.warning("(publisher) Make sure that you are not loading this module from pycram.world.")

from .datastructures.pose import PoseStamped, TransformStamped, Pose, Header, Transform
from typing_extensions import List, Optional, Union, Iterable, TYPE_CHECKING, Tuple, Dict, Hashable

if TYPE_CHECKING:
    from .world_concepts.world_object import Object
    from .datastructures.world import World

class LocalTransformer(TransformManager):
    _instance = None
    prospection_prefix: str = "prospection/"

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized: return
        super().__init__()
        self.world: World = None
        # TODO: Ask Jonas if this is still needed
        self.prospection_world: World = None
        self._initialized = True
        self.add_transform("map", "map", transform_from_pq((0, 0, 0, 1, 0, 0, 0)))

    def transform_to_object_frame(self, pose: PoseStamped,
                                  world_object: Object, link_name: str = None) -> Union[
        PoseStamped, None]:
        """
        Transforms the given PoseStamped to the object coordinate frame

        :param pose: The Pose that should be transformed
        :param world_object: The world object to which coordinate frame should be transformed
        :param link_name: An optional link name of the object
        :return: The transformed PoseStamped
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
        for obj in objects:
            obj.update_link_transforms()
            obj.update_transform()
            self.update_transforms([obj.pose.to_transform_stamped(obj.tf_frame)])

    def transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        Transforms the given PoseStamped to an arbitrary coordinate frame

        :param pose: The pose that should be transformed
        :param target_frame: The name of the target frame
        :return: The transformed PoseStamped in target frame
        """
        objects = list(filter(None, map(self.get_object_from_frame, [pose.frame_id, target_frame])))
        self.update_transforms_for_objects(objects)

        source_frame = pose.header.frame_id

        wxyz = self.xyzw_to_wxyz(pose.orientation)

        pose_matrix = transform_from_pq(np.hstack((np.array(pose.pose.position),
                                                   np.array(wxyz))))

        transform_matrix = self.get_transform(target_frame, source_frame)

        new_pose = transform_matrix @ pose_matrix

        return_pose = PoseStamped(pose=Pose.from_matrix(new_pose), header=Header(frame_id=target_frame))
        return return_pose



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

    @staticmethod
    def translate_pose_along_local_axis(pose: PoseStamped, axis: List, distance: float) -> PoseStamped:
        """
        Translate a pose along a given 3d vector (axis) by a given distance. The axis is given in the local coordinate
        frame of the pose. The axis is normalized and then scaled by the distance.

        :param pose: The pose that should be translated
        :param axis: The local axis along which the translation should be performed
        :param distance: The distance by which the pose should be translated

        :return: The translated pose
        """
        normalized_translation_vector = np.array(axis) / np.linalg.norm(axis)

        rot_matrix = quaternion_matrix(pose.orientation.to_list())[:3, :3]
        translation_in_world = rot_matrix @ normalized_translation_vector
        scaled_translation_vector = np.array(pose.position.to_list()) + translation_in_world * distance

        return PoseStamped.from_list(list(scaled_translation_vector), pose.orientation.to_list(), pose.frame_id)

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
                                                     time: Optional[Time] = None) -> TransformStamped:
        pass

    def update_transforms(self, transforms: Iterable[TransformStamped], time: Time = None) -> None:
        """
       Updates transforms by updating the time stamps of the header of each transform. If no time is given the current
       time is used.
       """
        for transform in transforms:
            wxyz = self.xyzw_to_wxyz(transform.rotation.to_list())
            pt_transform = transform_from_pq(np.hstack((np.array(transform.translation.to_list()),
                                                        np.array(wxyz))))
            self.add_transform(transform.header.frame_id, transform.child_frame_id, pt_transform)

    @staticmethod
    def xyzw_to_wxyz(xyzw: List[float]) -> List[float]:
        """
        Convert a quaternion from XYZW to WXYZ format.

        :param xyzw: The quaternion in XYZW format.
        """
        return [xyzw[3], *xyzw[:3]]

    def get_all_frames(self) -> Dict[Tuple[Hashable, Hashable], Transform]:
        return {key: Transform.from_matrix(value) for key, value in self.transforms.items()}
