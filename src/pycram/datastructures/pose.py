# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import datetime
import math
from collections.abc import Sequence

import numpy as np
import sqlalchemy.orm
import typing_extensions
from geometry_msgs.msg import (Pose as GeoPose, Quaternion as GeoQuaternion)
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Point
from typing_extensions import List, Union, Optional, Self, Tuple

from .enums import AxisIdentifier, Grasp
from .grasp import PreferredGraspAlignment, GraspDescription
from ..orm.base import Pose as ORMPose, Position, Quaternion, ProcessMetaData
from ..ros import Time
from ..ros import logwarn, logerr
from ..tf_transformations import euler_from_quaternion, translation_matrix, quaternion_matrix, concatenate_matrices, \
    inverse_matrix, translation_from_matrix, quaternion_from_matrix, quaternion_multiply
from ..validation.error_checkers import calculate_pose_error
from scipy.spatial.transform import Rotation as R

if typing_extensions.TYPE_CHECKING:
    from ..world_concepts.world_object import Object


def get_normalized_quaternion(quaternion: np.ndarray) -> GeoQuaternion:
    """
    Normalizes a given quaternion such that it has a magnitude of 1.

    :param quaternion: The quaternion that should be normalized
    :return: The normalized quaternion
    """
    mag = math.sqrt(sum(v ** 2 for v in quaternion))
    normed_rotation = [f / mag for f in quaternion]

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

    def __init__(self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None,
                 frame: str = "map", time: Time = None):
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

        self.header.stamp = time if time else Time().now()

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

    def to_pose_stamped(self) -> PoseStamped:
        """
        Converts this Pose to a PoseStamped message. This is useful for compatibility with ROS.

        :return: A PoseStamped message with the same information as this Pose
        """
        return PoseStamped(
            header=self.header,
            pose=self.pose
        )

    def get_position_diff(self, target_pose: Self) -> Point:
        """
        Get the difference between the target and the current positions.

        :param target_pose: The target pose.
        :return: The difference between the two positions.
        """
        return Point(x=target_pose.position.x - self.position.x, y=target_pose.position.y - self.position.y,
                     z=target_pose.position.z - self.position.z)

    def get_z_angle_difference(self, target_pose: Self) -> float:
        """
        Get the difference between two z angles.

        :param target_pose: The target pose.
        :return: The difference between the two z angles.
        """
        return target_pose.z_angle - self.z_angle

    @property
    def z_angle(self) -> float:
        """
        The z angle of the orientation of this Pose in radians.
        """
        return euler_from_quaternion(self.orientation_as_list())[2]

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
    def position(self, value: Union[Sequence[float], GeoPose, Point]) -> None:
        """
        Sets the position for this Pose, the position can either be a sequence of xyz, a Point
        or a geometry_msgs/Pose message.

        :param value: Sequence or geometry_msgs/Pose message for the position
        """
        if not isinstance(value, (list, tuple, np.ndarray, GeoPose, Point)):
            err_msg = "Position can only be one of (list, tuple, np.ndarray, geometry_msgs/Pose, Point) not " + \
                      str(type(value))
            logerr(err_msg)
            raise TypeError(err_msg)
        if isinstance(value, (list, tuple, np.ndarray)) and len(value) == 3:
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
    def orientation(self, value: Union[Sequence[float], GeoQuaternion]) -> None:
        """
        Sets the orientation of this Pose, the orientation can either be a sequence of xyzw
        or a geometry_msgs/Quaternion message

        :param value: New orientation, either a list or geometry_msgs/Quaternion
        """
        if not isinstance(value, (list, tuple, np.ndarray, GeoQuaternion)):
            err_msg = (f"Orientation can only be a Sequence (list, tuple, ...etc.) or a geometry_msgs/Quaternion "
                       f"not {type(value)}")
            logerr(err_msg)
            raise TypeError(err_msg)

        if isinstance(value, (list, tuple, np.ndarray)) and len(value) == 4:
            orientation = np.array(value)
        else:
            orientation = np.array([value.x, value.y, value.z, value.w])
        # This is used instead of np.linalg.norm since numpy is too slow on small arrays
        self.pose.orientation = get_normalized_quaternion(orientation)

    def to_list(self) -> List[List[float]]:
        """
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
        :return: The position as a list of xyz values.
        """
        return [self.position.x, self.position.y, self.position.z]

    def orientation_as_list(self) -> List[float]:
        """
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

    def almost_equal(self, other: Pose, position_tolerance_in_meters: float = 1e-3,
                     orientation_tolerance_in_degrees: float = 1) -> bool:
        """
        Checks if the given Pose is almost equal to this Pose. The position and orientation can have a certain
        tolerance. The position tolerance is given in meters and the orientation tolerance in degrees. The position
        error is calculated as the euclidian distance between the positions and the orientation error as the angle
        between the quaternions.
        
        :param other: The other Pose which should be compared
        :param position_tolerance_in_meters: The tolerance for the position in meters
        :param orientation_tolerance_in_degrees: The tolerance for the orientation in degrees
        :return: True if the Poses are almost equal, False otherwise
        """
        error = calculate_pose_error(self, other)
        return error[0] <= position_tolerance_in_meters and error[1] <= orientation_tolerance_in_degrees * math.pi / 180

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

    def round(self, decimals: int = 4) -> None:
        """
        Rounds the position and orientation of this Pose to the given number of decimals.

        :param decimals: The number of decimals to which the position and orientation should be rounded
        """
        self.position = [round(v, decimals) for v in self.position_as_list()]
        self.orientation = [round(v, decimals) for v in self.orientation_as_list()]

    def to_sql(self) -> ORMPose:
        return ORMPose(datetime.datetime.utcfromtimestamp(self.header.stamp.to_sec()), self.frame)

    def insert(self, session: sqlalchemy.orm.Session) -> ORMPose:

        metadata = ProcessMetaData().insert(session)

        position = Position(*self.position_as_list())
        position.process_metadata = metadata
        orientation = Quaternion(**dict(zip(["x", "y", "z", "w"], self.orientation_as_list())))
        orientation.process_metadata = metadata
        session.add(position)
        session.add(orientation)

        pose = self.to_sql()
        pose.process_metadata = metadata
        pose.orientation = orientation
        pose.position = position
        session.add(pose)

        return pose

    def rotate_by_quaternion(self, quaternion: Tuple[float, float, float, float]) -> None:
        """
        Rotates this Pose by the given quaternion. The orientation of this Pose is multiplied by the given quaternion,
        according to the hamilton product. The quaternion has to be given as a tuple with xyzw.
        The orientation is normalized after the rotation.

        :param quaternion: The quaternion by which this Pose should be rotated
        """
        self.orientation = quaternion_multiply(self.orientation_as_list(), quaternion)

    def get_vector_to_pose(self, other_pose: Pose) -> np.ndarray:
        """
        Get the vector between two poses, by computing position of other_pose - position of self.

        :param other_pose: The other pose.

        :return: The vector between the two poses.
        """
        return np.array(other_pose.position_as_list()) - np.array(self.position_as_list())

    @staticmethod
    def calculate_closest_faces(pose_to_robot_vector: Tuple[float, float, float],
                                specified_grasp_axis: Optional[AxisIdentifier] = None) -> Tuple[
        GraspDescription, GraspDescription]:
        """
        Determines the faces of the object based on the input vector.

        If `specified_grasp_axis` is None, it calculates the primary and secondary faces based on the vector's magnitude
        determining which sides of the object are most aligned with the robot. This will either be the x, y plane for side faces
        or the z axis for top/bottom faces.
        If `specified_grasp_axis` is provided, it only considers the specified axis and calculates the faces aligned
        with that axis.

        :param pose_to_robot_vector: A 3D vector representing one of the robot's axes in the pose's frame, with
                              irrelevant components set to np.nan.
        :param specified_grasp_axis: Specifies a specific axis (e.g., X, Y, Z) to focus on.

        :return: A tuple of two Grasp enums representing the primary and secondary faces.
        """
        all_axes = [AxisIdentifier.X, AxisIdentifier.Y, AxisIdentifier.Z]

        if specified_grasp_axis:
            valid_axes = [specified_grasp_axis]
        else:
            valid_axes = [axis for axis in all_axes if not np.isnan(pose_to_robot_vector[axis.value.index(1)])]

        object_to_robot_vector = np.array(pose_to_robot_vector) + 1e-9
        sorted_axes = sorted(valid_axes, key=lambda axis: abs(object_to_robot_vector[axis.value.index(1)]),
                             reverse=True)

        primary_axis = sorted_axes[0]
        primary_sign = int(np.sign(object_to_robot_vector[primary_axis.value.index(1)]))
        primary_face = Grasp.from_axis_direction(primary_axis, primary_sign)

        if len(sorted_axes) > 1:
            secondary_axis = sorted_axes[1]
            secondary_sign = int(np.sign(object_to_robot_vector[secondary_axis.value.index(1)]))
            secondary_face = Grasp.from_axis_direction(secondary_axis, secondary_sign)
        else:
            secondary_sign = -primary_sign
            secondary_axis = primary_axis
            secondary_face = Grasp.from_axis_direction(secondary_axis, secondary_sign)

        return primary_face, secondary_face

    def calculate_grasp_descriptions(self, robot: Object, grasp_alignment: Optional[PreferredGraspAlignment] = None) -> List[GraspDescription]:
        """
        This method determines the possible grasp configurations (approach axis and vertical alignment) of the self,
        taking into account the self's orientation, position, and whether the gripper should be rotated by 90°.

        :param robot: The robot for which the grasp configurations are being calculated.

        :return: A sorted list of GraspDescription instances representing all grasp permutations.
        """
        objectTmap = self

        robot_pose = robot.get_pose()

        if grasp_alignment:
            side_axis = grasp_alignment.preferred_axis
            vertical = grasp_alignment.with_vertical_alignment
            rotated_gripper = grasp_alignment.with_rotated_gripper
        else:
            side_axis, vertical, rotated_gripper = None, False, False

        object_to_robot_vector_world = objectTmap.get_vector_to_pose(robot_pose)
        orientation = objectTmap.orientation_as_list()

        mapRobject = R.from_quat(orientation).as_matrix()
        objectRmap = mapRobject.T

        object_to_robot_vector_local = objectRmap.dot(object_to_robot_vector_world)
        vector_x, vector_y, vector_z = object_to_robot_vector_local

        vector_side = np.array([vector_x, vector_y, np.nan], dtype=float)
        side_faces = self.calculate_closest_faces(vector_side, side_axis)

        vector_vertical = np.array([np.nan, np.nan, vector_z], dtype=float)
        vertical_faces = self.calculate_closest_faces(vector_vertical) if vertical else [None]

        grasp_configs = [
            GraspDescription(approach_direction=side, vertical_alignment=top_face, rotate_gripper=rotated_gripper)
            for top_face in vertical_faces
            for side in side_faces
        ]

        return grasp_configs

    def __str__(self):
        return (f"Pose: {[round(v, 3) for v in self.position_as_list()]}, {[round(v, 3) for v in self.orientation_as_list()]}"
                f" in frame {self.frame}")

    def __repr__(self):
        return self.__str__()


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
                 frame: Optional[str] = "map", child_frame: Optional[str] = "", time: Time = None):
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
        self.header.stamp = time if time else Time().now()

        self.frame = frame

    def apply_transform_to_array_of_points(self, points: np.ndarray) -> np.ndarray:
        """
        Applies this Transform to an array of points. The points are given as a Nx3 matrix, where N is the number of
        points. The points are transformed from the child_frame_id to the frame_id of this Transform.

        :param points: The points that should be transformed, given as a Nx3 matrix.
        """
        homogeneous_transform = self.get_homogeneous_matrix()
        # add the homogeneous coordinate, by adding a column of ones to the position vectors, becoming 4xN matrix
        homogenous_points = np.concatenate((points, np.ones((points.shape[0], 1))), axis=1).T
        transformed_points = homogeneous_transform @ homogenous_points
        return transformed_points[:3, :].T

    def get_homogeneous_matrix(self) -> np.ndarray:
        """
        :return: The homogeneous matrix of this Transform
        """
        translation = translation_matrix(self.translation_as_list())
        rotation = quaternion_matrix(self.rotation_as_list())
        return np.dot(translation, rotation)

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
    def translation(self) -> Vector3:
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
            logwarn("Value of a translation can only be a list or a geometry_msgs/Vector3")
            return
        if isinstance(value, list) and len(value) == 3:
            self.transform.translation.x = value[0]
            self.transform.translation.y = value[1]
            self.transform.translation.z = value[2]
        else:
            self.transform.translation = value

    @property
    def rotation(self) -> Quaternion:
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
            logwarn("Value of the rotation can only be a list or a geometry.msgs/Quaternion")
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
        t = Transform(self.translation_as_list(), self.rotation_as_list(), self.frame, self.child_frame_id,
                      self.header.stamp)
        t.header.frame_id = self.header.frame_id
        # t.header.stamp = self.header.stamp
        return t

    def translation_as_list(self) -> List[float]:
        """
        :return: The translation as a list of xyz
        """
        return [self.transform.translation.x, self.transform.translation.y, self.transform.translation.z]

    def rotation_as_list(self) -> List[float]:
        """
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
        transform = concatenate_matrices(translation_matrix(self.translation_as_list()),
                                         quaternion_matrix(self.rotation_as_list()))
        inverse_transform = inverse_matrix(transform)
        translation = translation_from_matrix(inverse_transform)
        quaternion = quaternion_from_matrix(inverse_transform)
        return Transform(list(translation), list(quaternion), self.child_frame_id, self.header.frame_id,
                         self.header.stamp)

    def round(self, decimals: int = 4):
        """
        Rounds the translation and rotation of this Transform to the given number of decimals.

        :param decimals: The number of decimals to which the translation and rotation should be rounded
        """
        self.translation = [round(v, decimals) for v in self.translation_as_list()]
        self.rotation = [round(v, decimals) for v in self.rotation_as_list()]

    def __mul__(self, other: Transform) -> Union[Transform, None]:
        """
        Multiplies this Transform with another one. The resulting Transform points from the frame_id of this Transform
        to the child_frame_id of the other Transform.

        :param other: The Transform which should be multiplied with this one.
        :return: The resulting Transform from the multiplication
        """
        if not isinstance(other, Transform):
            logerr(f"Can only multiply two Transforms")
            return
        self_trans = translation_matrix(self.translation_as_list())
        self_rot = quaternion_matrix(self.rotation_as_list())
        self_mat = np.dot(self_trans, self_rot)

        other_trans = translation_matrix(other.translation_as_list())
        other_rot = quaternion_matrix(other.rotation_as_list())
        other_mat = np.dot(other_trans, other_rot)

        new_mat = np.dot(self_mat, other_mat)
        new_trans = translation_from_matrix(new_mat)
        new_rot = quaternion_from_matrix(new_mat)
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