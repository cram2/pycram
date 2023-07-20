# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import copy
from typing import List, Union, Optional

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf import transformations


class Pose(PoseStamped):
    """
    Pose representation for PyCRAM, this class extends the PoseStamped ROS message from geometry_msgs. Thus making it
    compatible with every ROS service and message expecting a PoseStamped message.

    Naming convention for Poses:
        Pose: Instances of this class, representing a cartesian position and a quaternion for orientation

        Position: Only the cartesian position in xyz

        Orientation: Only the quaternion as xyzw
    """

    def __init__(self, position: Optional[List[float]] = None, orientation: Optional[List[float]] = None,
                 frame: str = "map"):
        """
        Poses can be initialized by a position and orientation given as lists, this is optional. By default, Poses are
        initialized with the position being [0, 0, 0], the orientation being [0, 0, 0, 1] and the frame being 'map'.

        :param position: An optional position of this Pose
        :param orientation: An optional orientation of this Pose
        :param frame: An optional frame in which this pose is
        """
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
    def frame(self) -> str:
        """
        Property for the frame_id such that it is easier accessible. Instead of Pose.header.frame_id it is Pose.frame

        :return: The TF frame of this Pose
        """
        return self.header.frame_id

    @frame.setter
    def frame(self, value: str) -> None:
        """
        Sets the TF frame of this pose to the given new frame

        :param value: The new TF frame
        """
        self.header.frame_id = value

    def to_list(self) -> List[List[float]]:
        """
        Returns the position and orientation of this pose as a list containing two list.

        :return: The position and orientation as lists
        """
        return [[self.pose.position.x, self.pose.position.y, self.pose.position.z],
                [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]]

    def to_transform(self, child_frame: str) -> Transform:
        """
        Converts this pose to a Transform from the TF frame of the pose to the given child_frame

        :param child_frame: Child frame id to which the Transform points
        :return: A new Transform
        """
        return Transform(self.position_as_list(), self.orientation_as_list(), self.frame, child_frame)

    def copy(self) -> Pose:
        """
        Creates a deep copy of this pose.

        :return: A copy of this pose
        """
        return copy.deepcopy(self)

    def position_as_list(self) -> List[float]:
        """
        Returns only the position as a list of xyz.

        :return: The position as a list
        """
        return [self.pose.position.x, self.pose.position.y, self.pose.position.z]

    def orientation_as_list(self) -> List[float]:
        """
        Returns only the orientation as a list of a quaternion

        :return: The orientation as a quaternion with xyzw
        """
        return [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]

    def dist(self, other_pose: Pose) -> float:
        """
        Calculates the euclidian distance between this Pose and the given one. For distance calculation only the
        position is used.

        :param other_pose: Pose to which the distance should be calculated
        :return: The distance between the Poses
        """
        self_position = self.position_as_list()
        other_position = other_pose.position_as_list()
        return np.linalg.norm(np.array(self_position) - np.array(other_position))

    def __eq__(self, other: Pose) -> bool:
        """
        Overloads the '==' operator to check for equality between two Poses. Only compares the position, orientation and
        frame. Timestamps of Poses are not takes into account.

        :param other: Other pose which should be compared
        :return: True if both Poses have the same position, orientation and frame. False otherwise
        """
        if not type(other) == Pose:
            return False
        self_position = self.position_as_list()
        other_position = other.position_as_list()

        self_orient = self.orientation_as_list()
        other_orient = other.orientation_as_list()

        return self_position == other_position and self_orient == other_orient and self.frame == other.frame


class Transform(TransformStamped):
    """
    Represents a Transformation from one TF frame to another in PyCRAM. Like with Poses this class inherits from the ROS
    message TransformStamped form geometry_msgs and is therefore compatible with ROS services and messages that require
    a TransformStamped message.

    Naming Convention for Transforms:
        Transform: Instances of this class, representing a translation and rotation from frame_id to child_frame_id

        Translation: A vector representing the conversion in cartesian space

        Rotation: A quaternion representing the conversion of rotation between both frames
    """
    def __init__(self, translation: Optional[List[float]] = None, rotation: Optional[List[float]] = None,
                 frame: str = "map", child_frame: str = ""):
        """
        Transforms take a translation, rotation, frame and child_frame as optional arguments. If nothing is given the
        Transform will be initialized with [0, 0, 0] for translation, [0, 0, 0, 1] for rotation, 'map' for frame and an
        empty string for child_frame

        :param translation: Optional translation from frame to child_frame in cartesian space
        :param rotation: Optional rotation from frame to child frame given as quaternion
        :param frame: Origin TF frame of this Transform
        :param child_frame: Target frame for this Transform
        """
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
    def frame(self) -> str:
        """
        Property for the frame_id such that it is easier accessible. Instead of Pose.header.frame_id it is Pose.frame

        :return: The TF frame of this Pose
        """
        return self.header.frame_id

    @frame.setter
    def frame(self, value: str) -> None:
        """
        Sets the TF frame of this pose to the given new frame

        :param value: The new TF frame
        """
        self.header.frame_id = value

    def copy(self) -> Transform:
        """
        Creates a deep copy of this pose.

        :return: A copy of this pose
        """
        return copy.deepcopy(self)

    def translation_as_list(self) -> List[float]:
        """
        Returns the translation of this Transform as a list.

        :return: The translation as a list of xyz
        """
        return [self.transform.translation.x, self.transform.translation.y, self.transform.translation.z]

    def rotation_as_list(self) -> List[float]:
        """
        Returns the rotation of this Transform as a list representing a quaternion.

        :return: The rotation of this Transform as a list with xyzw
        """
        return [self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z,
                self.transform.rotation.w]

    def to_pose(self) -> Pose:
        """
        Converts this Transform to a Pose, in this process the child_frame_id is lost.

        :return: A new pose with same translation as position and rotation as orientation
        """
        return Pose(self.translation_as_list(), self.rotation_as_list(), self.frame)

    def invert(self) -> Transform:
        """
        Inverts this Transform, the new Transform points from the child_frame_id to the frame_id

        :return: A new inverted Transform
        """
        transform = transformations.concatenate_matrices(transformations.translation_matrix(self.translation_as_list()),
                                                         transformations.quaternion_matrix(self.rotation_as_list()))
        inverse_transform = transformations.inverse_matrix(transform)
        translation = transformations.translation_from_matrix(inverse_transform)
        quaternion = transformations.quaternion_from_matrix(inverse_transform)
        return Transform(list(translation), list(quaternion), self.child_frame_id, self.header.frame_id)

    def __mul__(self, other: Transform) -> Union[Transform, None]:
        """
        Multiplies this Transform with another one. The resulting Transform points from the frame_id of this Transform
        to the child_frame_id of the other Transform.

        :param other: The Transform which should be multiplied with this one.
        :return: The resulting Transform from the multiplication
        """
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

    def inverse_times(self, other_transform: Transform) -> Transform:
        """
        Like a 'minus' for Transforms, subtracts the other_transform from this one.

        :param other_transform: Transform which should be subtracted from this one
        :return: The resulting Transform form the calculation
        """
        inv = other_transform.invert()
        return self * inv

    def __eq__(self, other: Transform) -> bool:
        """
        Overloads the '==' operator to check for equality between two Transforms. Only compares the translation,
        rotation, frame and child frame. Timestamps of Poses are not takes into account.

        :param other: Other pose which should be compared
        :return: True if both Transforms have the same translation, rotation, frame and child frame. False otherwise
        """
        if not type(other) == Transform:
            return False
        self_position = self.translation_as_list()
        other_position = other.translation_as_list()

        self_orient = self.rotation_as_list()
        other_orient = other.rotation_as_list()

        return self_position == other_position and self_orient == other_orient and \
            self.frame == other.frame and self.child_frame_id == other.child_frame_id
