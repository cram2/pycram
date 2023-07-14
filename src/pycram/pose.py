# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import copy

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf import transformations


class Pose(PoseStamped):

    def __init__(self, position=None, orientation=None, frame="map"):
        super().__init__()
        if position:
            self.pose.position.x = position[0]
            self.pose.position.y = position[1]
            self.pose.position.z = position[2]

        if orientation:
            self.pose.orientation.x = orientation[0]
            self.pose.orientation.y = orientation[1]
            self.pose.orientation.z = orientation[2]
            self.pose.orientation.w = orientation[3]
        else:
            self.pose.orientation.w = 1.0

        self.header.frame_id = frame
        self.header.stamp = rospy.Time.now()

        self.position = self.pose.position
        self.orientation = self.pose.orientation

        self.frame = frame

    @property
    def frame(self):
        return self.header.frame_id

    @frame.setter
    def frame(self, value):
        self.header.frame_id = value

    def to_list(self):
        return [[self.pose.position.x, self.pose.position.y, self.pose.position.z],
                [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]]

    def to_transform(self, child_frame: str):
        return Transform(self.position_as_list(), self.orientation_as_list(), self.frame, child_frame)

    def copy(self):
        return copy.deepcopy(self)

    def position_as_list(self):
        return [self.pose.position.x, self.pose.position.y, self.pose.position.z]

    def orientation_as_list(self):
        return [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]


class Transform(TransformStamped):

    def __init__(self, translation=None, rotation=None, frame="map", child_frame=""):
        super().__init__()
        if translation:
            self.transform.translation.x = translation[0]
            self.transform.translation.y = translation[1]
            self.transform.translation.z = translation[2]

        if rotation:
            self.transform.rotation.x = rotation[0]
            self.transform.rotation.y = rotation[1]
            self.transform.rotation.z = rotation[2]
            self.transform.rotation.w = rotation[3]
        else:
            self.transform.rotation.w = 1.0

        self.header.frame_id = frame
        self.child_frame_id = child_frame
        self.header.stamp = rospy.Time.now()

        self.translation = self.transform.translation
        self.rotation = self.transform.rotation

        self.frame = frame

    @property
    def frame(self):
        return self.header.frame_id

    @frame.setter
    def frame(self, value):
        self.header.frame_id = value

    def copy(self):
        return copy.deepcopy(self)

    def translation_as_list(self):
        return [self.transform.translation.x, self.transform.translation.y, self.transform.translation.z]

    def rotation_as_list(self):
        return [self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z,
                self.transform.rotation.w]

    def to_pose(self):
        return Pose(self.translation_as_list(), self.rotation_as_list(), self.frame)

    def invert(self):
        transform = transformations.concatenate_matrices(transformations.translation_matrix(self.translation_as_list()),
                                                         transformations.quaternion_matrix(self.rotation_as_list()))
        inverse_transform = transformations.inverse_matrix(transform)
        translation = transformations.translation_from_matrix(inverse_transform)
        quaternion = transformations.quaternion_from_matrix(inverse_transform)
        return Transform(list(translation), list(quaternion), self.child_frame_id, self.header.frame_id)

    def __mul__(self, other: Transform) -> Transform:
        if not type(other) == Transform:
            rospy.logerr(f"Can only multiply two Transforms")
            return
        self_trans = transformations.translation_matrix(self.translation_as_list())
        self_rot = transformations.quaternion_matrix(self.rotation_as_list())
        self_mat = np.dot(self_trans, self_rot)

        other_trans = transformations.translation_matrix(other.translation_as_list())
        other_rot = transformations.quaternion_matrix(other.rotation_as_list())
        other_mat = np.dot(other_trans, other_rot)

        new_mat = np.dot(self_mat, other_mat)
        new_trans = transformations.translation_from_matrix(new_mat)
        new_rot = transformations.quaternion_from_matrix(new_mat)
        return Transform(list(new_trans), list(new_rot), self.frame, other.child_frame_id)



