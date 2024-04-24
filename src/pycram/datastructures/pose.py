# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import math
import datetime
from typing_extensions import List, Union, Optional

import numpy as np
import rospy
import sqlalchemy.orm
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Point
from geometry_msgs.msg import (Pose as GeoPose, Quaternion as GeoQuaternion)
from tf import transformations
from ..orm.base import Pose as ORMPose, Position, Quaternion, ProcessMetaData


def get_normalized_quaternion(quaternion: np.ndarray) -> GeoQuaternion:
    """
    Normalizes a given quaternion such that it has a magnitude of 1.

    :param quaternion: The quaternion that should be normalized
    :return: The normalized quaternion
    """
    mag = math.sqrt(sum(v**2 for v in quaternion))
    normed_rotation = quaternion / mag

    geo_quaternion = GeoQuaternion()
    geo_quaternion.x = normed_rotation[0]
    geo_quaternion.y = normed_rotation[1]
    geo_quaternion.z = normed_rotation[2]
    geo_quaternion.w = normed_rotation[3]

    return geo_quaternion


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
                 frame: str = "map", time: rospy.Time = None):
        """
        Poses can be initialized by a position and orientation given as lists, this is optional. By default, Poses are
        initialized with the position being [0, 0, 0], the orientation being [0, 0, 0, 1] and the frame being 'map'.

        :param position: An optional position of this Pose
        :param orientation: An optional orientation of this Pose
        :param frame: An optional frame in which this pose is
        :param time: The time at which this Pose is valid, as ROS time
        """
        super().__init__()
        if position is not None:
            self.position = position

        if orientation is not None:
            self.orientation = orientation
        else:
            self.pose.orientation.w = 1.0

        self.header.frame_id = frame

        self.header.stamp = time if time else rospy.Time.now()

        self.frame = frame

    @staticmethod
    def from_pose_stamped(pose_stamped: PoseStamped) -> Pose:
        """
        Converts a geometry_msgs/PoseStamped message to a Pose object. Should be used for compatability with ROS.

        :param pose_stamped: The pose stamped message which should be converted
        :return: A Pose object with the same information as the given message
        """
        p = Pose()
        p.header = pose_stamped.header
        p.pose = pose_stamped.pose
        return p

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

    @property
    def position(self) -> Point:
        """
        Property that points to the position of this pose
        """
        return self.pose.position

    @position.setter
    def position(self, value) -> None:
        """
        Sets the position for this Pose, the position can either be a list of xyz or a geometry_msgs/Pose message.

        :param value: List or geometry_msgs/Pose message for the position
        """
        if (not isinstance(value, list) and not isinstance(value, tuple) and not isinstance(value, GeoPose)
                and not isinstance(value, Point)):
            rospy.logerr("Position can only be a list or geometry_msgs/Pose")
            raise TypeError("Position can only be a list/tuple or geometry_msgs/Pose")
        if isinstance(value, list) or isinstance(value, tuple) and len(value) == 3:
            self.pose.position.x = value[0]
            self.pose.position.y = value[1]
            self.pose.position.z = value[2]
        else:
            # TODO: Check if this is correct or if it should be handled as an error
            self.pose.position = value

    @property
    def orientation(self) -> GeoQuaternion:
        """
        Property that points to the orientation of this pose
        """
        return self.pose.orientation

    @orientation.setter
    def orientation(self, value) -> None:
        """
        Sets the orientation of this Pose, the orientation can either be a list of xyzw or a geometry_msgs/Quaternion
        message

        :param value: New orientation, either a list or geometry_msgs/Quaternion
        """
        if not isinstance(value, list) and not isinstance(value, tuple) and not isinstance(value, GeoQuaternion):
            rospy.logwarn("Orientation can only be a list or geometry_msgs/Quaternion")
            return

        if isinstance(value, list) or isinstance(value, tuple) and len(value) == 4:
            orientation = np.array(value)
        else:
            orientation = np.array([value.x, value.y, value.z, value.w])
        # This is used instead of np.linalg.norm since numpy is too slow on small arrays
        self.pose.orientation = get_normalized_quaternion(orientation)

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
        return Transform(self.position_as_list(), self.orientation_as_list(), self.frame, child_frame,
                         self.header.stamp)

    def copy(self) -> Pose:
        """
        Creates a deep copy of this pose.

        :return: A copy of this pose
        """
        p = Pose(self.position_as_list(), self.orientation_as_list(), self.frame, self.header.stamp)
        p.header.frame_id = self.header.frame_id
        return p

    def position_as_list(self) -> List[float]:
        """
        Returns only the position as a list of xyz.

        :return: The position as a list
        """
        return [self.position.x, self.position.y, self.position.z]

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
        if not isinstance(other, Pose):
            return False
        self_position = self.position_as_list()
        other_position = other.position_as_list()

        self_orient = self.orientation_as_list()
        other_orient = other.orientation_as_list()

        return self_position == other_position and self_orient == other_orient and self.frame == other.frame

    def set_position(self, new_position: List[float]) -> None:
        """
        Sets the position of this Pose to the given position. Position has to be given as a vector in cartesian space.

        :param new_position: New position as a vector of xyz
        """
        self.position = new_position

    def set_orientation(self, new_orientation: List[float]) -> None:
        """
        Sets the orientation to the given quaternion. The new orientation has to be given as a quaternion.

        :param new_orientation: New orientation as a quaternion with xyzw
        """
        self.orientation = new_orientation

    def to_sql(self) -> ORMPose:
        return ORMPose(datetime.datetime.utcfromtimestamp(self.header.stamp.to_sec()), self.frame)

    def insert(self, session: sqlalchemy.orm.Session) -> ORMPose:

        metadata = ProcessMetaData().insert(session)

        position = Position(*self.position_as_list())
        position.process_metadata = metadata
        orientation = Quaternion(*self.orientation_as_list())
        orientation.process_metadata = metadata
        session.add(position)
        session.add(orientation)

        pose = self.to_sql()
        pose.process_metadata = metadata
        pose.orientation = orientation
        pose.position = position
        session.add(pose)

        return pose

    def multiply_quaternions(self, quaternion: List) -> None:
        """
        Multiply the quaternion of this Pose with the given quaternion, the result will be the new orientation of this
        Pose.

        :param quaternion: The quaternion by which the orientation of this Pose should be multiplied
        """
        x1, y1, z1, w1 = quaternion
        x2, y2, z2, w2 = self.orientation_as_list()

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        self.orientation = (x, y, z, w)

    def set_orientation_from_euler(self, axis: List, euler_angles: List[float]) -> None:
        """
        Convert axis-angle to quaternion.

        :param axis: (x, y, z) tuple representing rotation axis.
        :param angle: rotation angle in degree
        :return: The quaternion representing the axis angle
        """
        angle = math.radians(euler_angles)
        axis_length = math.sqrt(sum([i ** 2 for i in axis]))
        normalized_axis = tuple(i / axis_length for i in axis)

        x = normalized_axis[0] * math.sin(angle / 2)
        y = normalized_axis[1] * math.sin(angle / 2)
        z = normalized_axis[2] * math.sin(angle / 2)
        w = math.cos(angle / 2)

        return (x, y, z, w)


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
                 frame: Optional[str] = "map", child_frame: Optional[str] = "", time: rospy.Time = None):
        """
        Transforms take a translation, rotation, frame and child_frame as optional arguments. If nothing is given the
        Transform will be initialized with [0, 0, 0] for translation, [0, 0, 0, 1] for rotation, 'map' for frame and an
        empty string for child_frame

        :param translation: Optional translation from frame to child_frame in cartesian space
        :param rotation: Optional rotation from frame to child frame given as quaternion
        :param frame: Origin TF frame of this Transform
        :param child_frame: Target frame for this Transform
        :param time: The time at which this Transform is valid, as ROS time
        """
        super().__init__()
        if translation:
            self.translation = translation

        if rotation:
            self.rotation = rotation
        else:
            self.transform.rotation.w = 1.0

        self.header.frame_id = frame
        self.child_frame_id = child_frame
        self.header.stamp = time if time else rospy.Time.now()

        self.frame = frame

    @classmethod
    def from_pose_and_child_frame(cls, pose: Pose, child_frame_name: str) -> Transform:
        return cls(pose.position_as_list(), pose.orientation_as_list(), pose.frame, child_frame_name,
                   time=pose.header.stamp)

    @staticmethod
    def from_transform_stamped(transform_stamped: TransformStamped) -> Transform:
        """
        Creates a Transform instance from a geometry_msgs/TransformStamped message. Should be used for compatibility with
        ROS.

        :param transform_stamped: The transform stamped message that should be converted
        :return: An Transform with the same information as the transform stamped message
        """
        t = Transform()
        t.header = transform_stamped.header
        t.child_frame_id = transform_stamped.child_frame_id
        t.transform = transform_stamped.transform

        return t

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

    @property
    def translation(self) -> None:
        """
        Property that points to the translation of this Transform
        """
        return self.transform.translation

    @translation.setter
    def translation(self, value) -> None:
        """
        Setter for the translation of this Transform, the new value can either be of type list or a
        geometry_msgs/Vector message.

        :param value: The new value for the translation, either a list or geometry_msgs/Vector3
        """
        if not isinstance(value, list) and not isinstance(value, Vector3):
            rospy.logwarn("Value of a translation can only be a list of a geometry_msgs/Vector3")
            return
        if isinstance(value, list) and len(value) == 3:
            self.transform.translation.x = value[0]
            self.transform.translation.y = value[1]
            self.transform.translation.z = value[2]
        else:
            self.transform.translation = value

    @property
    def rotation(self) -> None:
        """
        Property that points to the rotation of this Transform
        """
        return self.transform.rotation

    @rotation.setter
    def rotation(self, value):
        """
        Setter for the rotation of this Transform, the new value can either be a list or a geometry_msgs/Quaternion
        message

        :param value: The new value for the rotation, either a list or geometry_msgs/Quaternion
        """
        if not isinstance(value, list) and not isinstance(value, GeoQuaternion):
            rospy.logwarn("Value of the rotation can only be a list or a geometry.msgs/Quaternion")
            return
        if isinstance(value, list) and len(value) == 4:
            rotation = np.array(value)

        else:
            rotation = np.array([value.x, value.y, value.z, value.w])
        # This is used instead of np.linalg.norm since numpy is too slow on small arrays
        self.transform.rotation = get_normalized_quaternion(rotation)

    def copy(self) -> Transform:
        """
        Creates a deep copy of this pose.

        :return: A copy of this pose
        """
        t = Transform(self.translation_as_list(), self.rotation_as_list(), self.frame, self.child_frame_id, self.header.stamp)
        t.header.frame_id = self.header.frame_id
        # t.header.stamp = self.header.stamp
        return t

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
        return Pose(self.translation_as_list(), self.rotation_as_list(), self.frame, self.header.stamp)

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
        return Transform(list(translation), list(quaternion), self.child_frame_id, self.header.frame_id, self.header.stamp)

    def __mul__(self, other: Transform) -> Union[Transform, None]:
        """
        Multiplies this Transform with another one. The resulting Transform points from the frame_id of this Transform
        to the child_frame_id of the other Transform.

        :param other: The Transform which should be multiplied with this one.
        :return: The resulting Transform from the multiplication
        """
        if not isinstance(other, Transform):
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
        if not isinstance(other, Transform):
            return False
        self_position = self.translation_as_list()
        other_position = other.translation_as_list()

        self_orient = self.rotation_as_list()
        other_orient = other.rotation_as_list()

        return self_position == other_position and self_orient == other_orient and \
            self.frame == other.frame and self.child_frame_id == other.child_frame_id

    def set_translation(self, new_translation: List[float]) -> None:
        """
        Sets the translation of this Transform to the newly given one. Translation has to be a vector in cartesian space

        :param new_translation: The new translation as a vector with xyz.
        """
        self.translation = new_translation

    def set_rotation(self, new_rotation: List[float]) -> None:
        """
        Sets the rotation of this Transform to the newly given one. Rotation has to be a quaternion.

        :param new_rotation: The new rotation as a quaternion with xyzw
        """
        self.rotation = new_rotation


