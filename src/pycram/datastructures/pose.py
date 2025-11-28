from __future__ import annotations

import copy
import datetime
import math
from dataclasses import dataclass, field

import numpy as np
from semantic_digital_twin.spatial_types.spatial_types import (
    Vector3 as SpatialVector3,
    Quaternion as SpatialQuaternion,
    TransformationMatrix as SpatialTransformationMatrix,
)
from semantic_digital_twin.world_description.world_entity import Body
from typing_extensions import Self, Tuple, Optional, List, TYPE_CHECKING

from .enums import AxisIdentifier, Arms
from ..has_parameters import has_parameters, HasParameters
from ..ros import Time as ROSTime
from ..tf_transformations import (
    quaternion_multiply,
    translation_matrix,
    quaternion_matrix,
    inverse_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)

if TYPE_CHECKING:
    from .grasp import GraspDescription


@has_parameters
@dataclass
class Vector3(HasParameters):
    """
    A 3D vector with x, y and z coordinates.
    """

    x: float = 0
    y: float = 0
    z: float = 0

    def euclidean_distance(self, other: Self) -> float:
        """
        The euclidian distance between this vector and another vector.

        :param other: The other vector to calculate the distance to.
        :return: The euclidian distance
        """
        return (
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        ) ** 0.5

    def ros_message(self):
        """
        Convert the vector to a ROS message of type Vector3.

        :return: The ROS message.
        """
        from geometry_msgs.msg import Vector3 as ROSVector3

        return ROSVector3(x=float(self.x), y=float(self.y), z=float(self.z))

    def to_list(self) -> List[float]:
        """
        Convert the vector to a list.

        :return: A list containing the x, y and z coordinates.
        """
        return [self.x, self.y, self.z]

    def to_spatial_type(self, reference_frame: Body = None) -> SpatialVector3:
        return SpatialVector3(
            x_init=float(self.x),
            y_init=float(self.y),
            z_init=self.z,
            reference_frame=reference_frame,
        )

    def round(self, decimals: int = 4):
        """
        Rounds the coordinates of the vector to the specified number of decimal places.

        :param decimals: Number of decimal places to round to.
        """
        self.x = round(self.x, decimals)
        self.y = round(self.y, decimals)
        self.z = round(self.z, decimals)

    def almost_equal(self, other: Self, tolerance: float = 1e-6) -> bool:
        """
        Check if two vectors are almost equal within a given tolerance.

        :param other: The other vector to compare to.
        :param tolerance: The tolerance for the comparison as number of decimal places.
        :return: True if the vectors are almost equal, False otherwise.
        """
        return bool(
            np.isclose(
                np.array(self.to_list()), np.array(other.to_list()), atol=tolerance
            ).all()
        )

    def vector_to_position(self, other: Self) -> Vector3:
        """
        Calculates a vector from this vector to another vector.

        :param other: The vector to calculate the vector to.
        :return: A new vector between this vector and the other vector.
        """
        return other - self

    def to_numpy(self) -> np.ndarray:
        """
        Convert the vector to a numpy array.

        :return: A numpy array containing the x, y and z coordinates.
        """
        return np.array(self.to_list())

    def __add__(self, other: Self) -> Vector3:
        """
        Adds two vectors together.

        :param other: The other vector to add.
        :return: A new vector that is the sum of this vector and the other vector.
        """
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Self) -> Vector3:
        """
        Subtracts two vectors.

        :param other: The other vector to subtract.
        :return: A new vector that is the difference of this vector and the other vector.
        """
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: float) -> Vector3:
        """
        Multiplies the vector by a scalar.

        :param other: The scalar to multiply by.
        :return: A new vector that is the product of this vector and the scalar.
        """
        return Vector3(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other: float) -> Vector3:
        """
        Multiplies the vector by a scalar (right multiplication).

        :param other: The scalar to multiply by.
        :return: A new vector that is the product of this vector and the scalar.
        """
        return Vector3(self.x * other, self.y * other, self.z * other)

    @classmethod
    def from_list(cls, vector: List[float]) -> Self:
        """
        Factory to create a Vector3 from a list of coordinates.

        :param vector: The list of coordinates.
        :return: A new Vector3 object.
        """
        return cls(*vector)


@has_parameters
@dataclass
class Quaternion(HasParameters):
    """
    A quaternion with x, y, z and w components.
    """

    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 1

    def __post_init__(self):
        self.normalize()

    def normalize(self):
        """
        Normalize the quaternion in-place.
        """
        # TODO fix this
        # if the object is not fully constructed yet
        if not (
            hasattr(self, "x")
            and hasattr(self, "y")
            and hasattr(self, "z")
            and hasattr(self, "w")
        ):
            return

        norm = (self.x**2 + self.y**2 + self.z**2 + self.w**2) ** 0.5
        object.__setattr__(self, "x", self.x / norm)
        object.__setattr__(self, "y", self.y / norm)
        object.__setattr__(self, "z", self.z / norm)
        object.__setattr__(self, "w", self.w / norm)

    def ros_message(self):
        from geometry_msgs.msg import Quaternion as ROSQuaternion

        return ROSQuaternion(
            x=float(self.x), y=float(self.y), z=float(self.z), w=float(self.w)
        )

    def to_list(self) -> List[float]:
        """
        Convert the quaternion to a list.

        :return: A list containing the x, y, z and w components.
        """
        return [self.x, self.y, self.z, self.w]

    def to_numpy(self) -> np.ndarray:
        """
        Convert the quaternion to a numpy array.

        :return: A numpy array containing the x, y, z and w components.
        """
        return np.array(self.to_list())

    def to_spatial_type(self) -> SpatialQuaternion:
        """
        Convert the quaternion to a SpatialQuaternion.

        :return: A SpatialQuaternion object containing the x, y, z and w components.
        """
        return SpatialQuaternion(
            x_init=float(self.x),
            y_init=float(self.y),
            z_init=float(self.z),
            w_init=float(self.w),
        )

    def round(self, decimals: int = 4):
        """
        Rounds the components of the quaternion to the specified number of decimal places.

        :param decimals: The number of decimal places to round to.
        """
        self.x = round(self.x, decimals)
        self.y = round(self.y, decimals)
        self.z = round(self.z, decimals)
        self.w = round(self.w, decimals)

    def almost_equal(self, other: Self, tolerance: float = 1e-6) -> bool:
        """
        Check if two quaternions are almost equal within a given tolerance.

        :param other: The other quaternion to compare to.
        :param tolerance: The tolerance for the comparison as number of decimal places.
        :return: True if the quaternions are almost equal, False otherwise.
        """
        return bool(
            np.isclose(
                np.array(self.to_list()), np.array(other.to_list()), atol=tolerance
            ).all()
        )

    def __mul__(self, other: Self) -> Quaternion:
        """
        Multiplies two quaternions together.

        :param other: The other quaternion to multiply with.
        :return: A new quaternion that is the product of this quaternion and the other quaternion.
        """
        return Quaternion.from_list(
            quaternion_multiply(self.to_list(), other.to_list())
        )

    @classmethod
    def from_list(cls, quaternion: List[float]) -> Self:
        """
        Factory to create a Quaternion from a list of components.

        :param quaternion: A list of components [x, y, z, w].
        :return: A new Quaternion object.
        """
        return cls(*quaternion)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> Self:
        """
        Create a Quaternion from a 3x3 rotation matrix.

        :param matrix: A 3x3 rotation matrix as numpy array.
        :return: A Quaternion object created from the matrix.
        """
        return cls(*quaternion_from_matrix(matrix))

    def __eq__(self, other):
        return self.almost_equal(other)

    # # TODO fix this
    # def __setattr__(self, key, value):
    #      object.__setattr__(self, key, value)
    #      self.normalize()


@has_parameters
@dataclass
class Pose(HasParameters):
    """
    A pose in 3D space.
    """

    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)

    def __repr__(self):
        return (
            f"Pose: {[round(v, 3) for v in [self.position.x, self.position.y, self.position.z]]}, "
            f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]]}"
        )

    def ros_message(self):
        """
        Convert the pose to a ROS message of type Pose.

        :return: The ROS message.
        """
        from geometry_msgs.msg import Point as ROSPoint
        from geometry_msgs.msg import Pose as ROSPose

        point = ROSPoint(
            x=float(self.position.x), y=float(self.position.y), z=float(self.position.z)
        )
        return ROSPose(position=point, orientation=self.orientation.ros_message())

    def to_list(self):
        """
        Convert the pose to a list of [position, orientation].

        :return: A list containing the position and orientation of this pose.
        """
        return [self.position.to_list(), self.orientation.to_list()]

    def to_spatial_type(self) -> SpatialTransformationMatrix:
        return SpatialTransformationMatrix.from_xyz_quaternion(
            pos_x=self.position.x,
            pos_y=self.position.y,
            pos_z=self.position.z,
            quat_x=self.orientation.x,
            quat_y=self.orientation.y,
            quat_z=self.orientation.z,
            quat_w=self.orientation.w,
        )

    def round(self, decimals: int = 4):
        """
        Rounds the components of the pose (position and orientation) to the specified number of decimal places.

        :param decimals: The number of decimal places to round to.
        """
        self.position.round(decimals)
        self.orientation.round(decimals)

    def almost_equal(
        self,
        other: Pose,
        position_tolerance: float = 1e-6,
        orientation_tolerance: float = 1e-5,
    ) -> bool:
        """
        Check if two poses are almost equal within given tolerances for position and orientation.

        :param other: The other pose to compare to.
        :param position_tolerance: Tolerance for position comparison as number of decimal places.
        :param orientation_tolerance: Tolerance for orientation comparison as number of decimal places.
        :return:  True if the poses are almost equal, False otherwise.
        """
        return self.position.almost_equal(
            other.position, position_tolerance
        ) and self.orientation.almost_equal(other.orientation, orientation_tolerance)

    def __eq__(self, other: Self) -> bool:
        """
        Check if two poses are equal. Uses almost_equal with a tolerance of 1e-4 for both position and orientation.

        :param other: The other pose to compare to.
        :return: True if the poses are equal, False otherwise.
        """
        return self.almost_equal(
            other, position_tolerance=1e-4, orientation_tolerance=1e-4
        )

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> Self:
        """
        Create a Pose from a 4x4 transformation matrix.

        :param matrix: A 4x4 transformation matrix as numpy array.
        :return: A pose object created from the matrix.
        """
        translation = translation_from_matrix(matrix)
        rotation = quaternion_from_matrix(matrix)
        return cls.from_list(translation, rotation)

    @classmethod
    def from_list(cls, position: List[float], orientation: List[float]) -> Self:
        """
        Factory to create a Pose from a list of position and orientation.

        :param position: List of position [x, y, z].
        :param orientation: List of orientation [x, y, z, w].
        :return: A new Pose object.
        """
        return cls(
            Vector3(position[0], position[1], position[2]),
            Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]),
        )


@dataclass
class Header:
    """
    A header with a timestamp.
    """

    frame_id: Body = field(default=None)
    stamp: datetime.datetime = field(
        default_factory=datetime.datetime.now, compare=False
    )
    sequence: int = field(default=0, compare=False)

    def ros_message(self):
        """
        Convert the header to a ROS message of type Header.

        :return: The ROS message.
        """
        from std_msgs.msg import Header as ROSHeader

        split_time = str(self.stamp.timestamp()).split(".")
        stamp = ROSTime(int(split_time[0]), int(split_time[1]))
        return ROSHeader(frame_id=self.frame_id.name.name, stamp=stamp)

    def __deepcopy__(self, memo):
        return Header(frame_id=self.frame_id, stamp=self.stamp, sequence=self.sequence)


@has_parameters
@dataclass
class Vector3Stamped(Vector3):
    """
    A Vector3 with an attached ROS Header (timestamp and frame).
    Inherits all vector operations and adds frame/time metadata.
    """

    header: Header = field(default_factory=Header)

    @property
    def frame_id(self):
        return self.header.frame_id

    @frame_id.setter
    def frame_id(self, value: Body):
        self.header.frame_id = value

    def __repr__(self):
        return f"Vector3: {[round(v, 3) for v in [self.x, self.y, self.z]]} in frame_id {self.frame_id.name}"

    def ros_message(self):
        """
        Convert to a ROS Vector3Stamped message.

        :return: The ROS message.
        """
        from geometry_msgs.msg import Vector3Stamped as ROSVector3Stamped
        from geometry_msgs.msg import Vector3 as ROSVector3

        return ROSVector3Stamped(
            vector=ROSVector3(x=self.x, y=self.y, z=self.z),
            header=self.header.ros_message(),
        )

    @classmethod
    def from_ros_message(cls, message):
        """
        Create a Vector3Stamped from a ROS message.

        :param message: The Vector3Stamped ROS message.
        :return: A new Vector3Stamped object.
        """
        header = Header(frame_id=message.header.frame_id, stamp=message.header.stamp)
        return cls(
            x=message.vector.x, y=message.vector.y, z=message.vector.z, header=header
        )

    def to_spatial_type(self) -> SpatialVector3:
        return SpatialVector3(
            x_init=float(self.x),
            y_init=float(self.y),
            z_init=self.z,
            reference_frame=self.header.frame_id,
        )


@has_parameters
@dataclass
class PoseStamped(HasParameters):
    """
    A pose in 3D space with a timestamp.
    """

    pose: Pose = field(default_factory=Pose)
    header: Header = field(default_factory=Header)

    @property
    def position(self):
        return self.pose.position

    @position.setter
    def position(self, value: Vector3):
        self.pose.position = value

    @property
    def orientation(self):
        return self.pose.orientation

    @orientation.setter
    def orientation(self, value: Quaternion):
        self.pose.orientation = value

    @property
    def frame_id(self):
        return self.header.frame_id

    @frame_id.setter
    def frame_id(self, value: Body):
        self.header.frame_id = value

    def __repr__(self):
        return (
            f"Pose: {[round(v, 3) for v in [self.position.x, self.position.y, self.position.z]]}, "
            f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]]} "
            f"in frame_id {self.frame_id.name if self.frame_id is not None else None}"
        )

    def ros_message(self):
        """
        Convert the pose to a ROS message of type PoseStamped.

        :return: The ROS message.
        """
        from geometry_msgs.msg import PoseStamped as ROSPoseStamped

        return ROSPoseStamped(
            pose=self.pose.ros_message(), header=self.header.ros_message()
        )

    @classmethod
    def from_ros_message(cls, message: "ROSPoseStamped") -> Self:
        """
        Create a PoseStamped from a ROS message.

        :param message: The PoseStamped ROS message.
        :return: A new PoseStamped object created from the ROS message.
        """
        header = Header(frame_id=message.header.frame_id, stamp=message.header.stamp)
        position = Vector3(
            x=message.pose.position.x,
            y=message.pose.position.y,
            z=message.pose.position.z,
        )
        orientation = Quaternion(
            x=message.pose.orientation.x,
            y=message.pose.orientation.y,
            z=message.pose.orientation.z,
            w=message.pose.orientation.w,
        )
        return cls(pose=Pose(position=position, orientation=orientation), header=header)

    @classmethod
    def from_list(
        cls,
        position: Optional[List[float]] = None,
        orientation: Optional[List[float]] = None,
        frame: Optional[Body] = None,
    ) -> Self:
        """
        Factory to create a PoseStamped from a list of position and orientation.

        :param position: Position as a list of [x, y, z].
        :param orientation: Orientation as a list of [x, y, z, w].
        :param frame: Frame in which the pose is defined.
        :return: A new PoseStamped object.
        """
        assert type(position) is list or position is None
        assert (
            type(orientation) is list
            or orientation is None
            or type(orientation) is tuple
        )
        position = position or [0.0, 0.0, 0.0]
        orientation = orientation or [0.0, 0.0, 0.0, 1.0]
        return cls(
            pose=Pose.from_list(position, orientation),
            header=Header(frame_id=frame, stamp=datetime.datetime.now()),
        )

    @classmethod
    def from_matrix(cls, matrix: np.ndarray, frame: Body) -> Self:
        """
        Create a PoseStamped from a 4x4 transformation matrix and a frame.

        :param matrix: A 4x4 transformation matrix as numpy array.
        :param frame: The frame in which the pose is defined.
        :return: A PoseStamped object created from the matrix and frame.
        """
        pose = Pose.from_matrix(matrix)
        return cls(
            pose=pose, header=Header(frame_id=frame, stamp=datetime.datetime.now())
        )

    @classmethod
    def from_spatial_type(cls, spatial_type: SpatialTransformationMatrix) -> Self:
        """
        Create a PoseStamped from a SpatialTransformationMatrix and a frame.

        :param spatial_type: A SpatialTransformationMatrix object.
        :return: A PoseStamped object created from the spatial type and frame.
        """
        return PoseStamped.from_matrix(
            spatial_type.to_np(), spatial_type.reference_frame
        )

    def to_transform_stamped(self, child_link_id: Body) -> TransformStamped:
        """
        Converts the PoseStamped to a TransformStamped given a frame to which the transform is pointing.

        :param child_link_id: Frame to which the transform is pointing.
        :return: A TransformStamped object.
        """
        return TransformStamped(
            header=self.header,
            pose=Transform.from_list(
                self.position.to_list(), self.orientation.to_list()
            ),
            child_frame_id=child_link_id,
        )

    def to_spatial_type(self) -> SpatialTransformationMatrix:
        """
        Converts the PoseStamped to a SpatialTransformationMatrix.

        :return: A SpatialTransformationMatrix object representing the pose in 3D space.
        """
        return SpatialTransformationMatrix.from_xyz_quaternion(
            pos_x=self.position.x,
            pos_y=self.position.y,
            pos_z=self.position.z,
            quat_x=self.orientation.x,
            quat_y=self.orientation.y,
            quat_z=self.orientation.z,
            quat_w=self.orientation.w,
            reference_frame=self.header.frame_id,
        )

    def round(self, decimals: int = 4):
        """
        Rounds the components of the pose (position and orientation) to the specified number of decimal places.

        :param decimals: Number of decimal places to round to.
        """
        self.position.round(decimals)
        self.orientation.round(decimals)

    def to_list(self):
        """
        Convert the pose to a list of [position, orientation, frame_id].

        :return: A list of [pose, frame_id].
        """
        return [self.pose.to_list(), self.frame_id]

    def almost_equal(
        self,
        other: PoseStamped,
        position_tolerance: float = 1e-6,
        orientation_tolerance: float = 1e-5,
    ) -> bool:
        """
        Check if two PoseStamped objects are almost equal within given tolerances for position and orientation and if the
        frame_id is the same.

        :param other: The other PoseStamped object to compare to.
        :param position_tolerance: Tolerance for position comparison as number of decimal places.
        :param orientation_tolerance: Tolerance for orientation comparison as number of decimal places.
        :return: True if the PoseStamped objects are almost equal, False otherwise.
        """
        return (
            self.position.almost_equal(other.position, position_tolerance)
            and self.orientation.almost_equal(other.orientation, orientation_tolerance)
            and self.frame_id == other.frame_id
        )

    def rotate_by_quaternion(self, quaternion: List[float]):
        """
        Rotates the orientation of the pose by a given quaternion.

        :param quaternion: A list representing the quaternion [x, y, z, w].
        """
        self.orientation = self.orientation * Quaternion.from_list(quaternion)

    def is_facing_2d_axis(
        self,
        pose_b: PoseStamped,
        axis: Optional[AxisIdentifier] = AxisIdentifier.X,
        threshold_deg=82,
    ) -> Tuple[bool, float]:
        """
        Check if this pose is facing another pose along a specific axis (X or Y) within a given angular threshold.

        :param pose_b: The target pose to compare against.
        :param axis: The axis to check alignment with ('x' or 'y'). Defaults to 'x'.
        :param threshold_deg: The maximum angular difference in degrees to consider as 'facing'. Defaults to 82 degrees.
        :return: Tuple of (True/False if facing, signed angular difference in radians).
        """

        a_pos = np.array([self.position.x, self.position.y])
        b_pos = np.array([pose_b.position.x, pose_b.position.y])
        target_angle = math.atan2(*(b_pos - a_pos)[::-1])

        q = self.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        if axis == AxisIdentifier.Y:
            yaw += math.pi / 2
        diff = math.atan2(math.sin(target_angle - yaw), math.cos(target_angle - yaw))
        return abs(diff) < math.radians(threshold_deg), diff

    def is_facing_x_or_y(self, pose_b: PoseStamped) -> bool:
        """
        Check if this pose is facing another pose along either the X or Y axis within a default angular threshold.

        :param pose_b: The target pose to compare against.
        :return: True if this pose is facing the target along either X or Y axis, False otherwise.
        """

        result = {
            a: self.is_facing_2d_axis(pose_b, axis=a)
            for a in (AxisIdentifier.X, AxisIdentifier.Y)
        }
        return any(r[0] for r in result.values())


@dataclass
class Transform(Pose):
    @property
    def translation(self):
        return self.position

    @property
    def rotation(self):
        return self.orientation

    def to_matrix(self) -> np.ndarray:
        """
        Converts the transform to a 4x4 transformation matrix.

        :return: A numpy array representing the transformation matrix.
        """
        translation = translation_matrix(self.translation.to_list())
        rotation = quaternion_matrix(self.rotation.to_list())
        return np.dot(translation, rotation)

    def __invert__(self) -> Transform:
        """
        Inverts the transform, returning a new Transform object.

        :return: The inverted Transform object.
        """
        inv = inverse_matrix(self.to_matrix())
        translation = translation_from_matrix(inv)
        rotation = quaternion_from_matrix(inv)
        return Transform.from_list(translation, rotation)

    def __mul__(self, other: Transform) -> Transform:
        """
        Multiplies two transforms together, returning a new Transform object.

        :param other: The other Transform object to multiply with.
        :return: A new Transform object that is the product of this transform and the other transform.
        """
        self_matrix = self.to_matrix()
        other_matrix = other.to_matrix()
        multiplication = self_matrix @ other_matrix
        return Transform.from_matrix(multiplication)

    @classmethod
    def from_pose(cls, pose: Pose) -> Self:
        """
        Create a Transform from a Pose object.

        :param pose: The pose to convert to a Transform.
        :return: A new Transform object created from the Pose.
        """
        return cls(pose.position, pose.orientation)

    def ros_message(self):
        """
        Convert the transform to a ROS message of type Transform.

        :return: The ROS message.
        """
        from geometry_msgs.msg import Transform as ROSTransform

        return ROSTransform(
            translation=self.translation.ros_message(),
            rotation=self.rotation.ros_message(),
        )


@has_parameters
@dataclass
class TransformStamped(PoseStamped):
    child_frame_id: Body = field(default_factory=str)
    """
    Target frame id of the transform.
    """
    pose: Transform = field(default_factory=Pose)
    """
    The transform of the transform.
    """

    @property
    def transform(self) -> Transform:
        return self.pose

    @transform.setter
    def transform(self, value: Transform):
        self.pose = value

    @property
    def translation(self):
        return self.pose.position

    @translation.setter
    def translation(self, value: Vector3):
        self.pose.position = value

    @property
    def rotation(self):
        return self.pose.orientation

    @rotation.setter
    def rotation(self, value: Quaternion):
        self.pose.orientation = value

    def __invert__(self) -> Self:
        """
        Inverts the transform, returning a new TransformStamped object which points from child_frame_id to frame_id.

        :return: A new TransformStamped object that is the inverse of this transform.
        """
        result = copy.deepcopy(self)
        result.header.frame_id = self.child_frame_id
        result.child_frame_id = self.header.frame_id
        result.transform = ~self.transform
        return result

    def __mul__(self, other) -> Self:
        """
        Multiplies two TransformStamped objects together, returning a new TransformStamped object.

        :param other: The other TransformStamped object to multiply with.
        :return: A new TransformStamped object that is the product of this transform and the other transform.
        """
        result = copy.deepcopy(self)
        result.child_frame_id = other.child_frame_id
        result.transform = self.transform * other.transform
        return result

    def __deepcopy__(self, memo):
        return TransformStamped(
            copy.deepcopy(self.pose), copy.deepcopy(self.header), self.child_frame_id
        )

    @classmethod
    def from_list(
        cls,
        translation: List[float] = None,
        rotation: List[float] = None,
        frame: Body = None,
        child_frame_id: Body = None,
    ) -> Self:
        """
        Factory to create a TransformStamped from a list of position and orientation.

        :param translation: Translation as a list of [x, y, z].
        :param rotation: Rotation as a list of [x, y, z, w].
        :param frame: Original frame in which the transform is defined.
        :param child_frame_id: Target frame id of the transform.
        :return: A new TransformStamped object.
        """
        translation = translation or [0.0, 0.0, 0.0]
        rotation = rotation or [0.0, 0.0, 0.0, 1.0]
        return cls(
            pose=Transform.from_list(translation, rotation),
            header=Header(frame_id=frame, stamp=datetime.datetime.now()),
            child_frame_id=child_frame_id,
        )

    def ros_message(self):
        """
        Convert the TransformStamped to a ROS message of type TransformStamped.

        :return: The ROS message.
        """
        from geometry_msgs.msg import TransformStamped as ROSTransformStamped

        return ROSTransformStamped(
            transform=self.transform.ros_message(),
            header=self.header.ros_message(),
            child_frame_id=self.child_frame_id.name.name,
        )

    def to_pose_stamped(self) -> PoseStamped:
        """
        Converts the TransformStamped to a PoseStamped object.

        :return: A PoseStamped object created from the TransformStamped.
        """
        p = Pose(self.pose.position, self.pose.orientation)
        return PoseStamped(p, self.header)

    def to_spatial_type(self) -> SpatialTransformationMatrix:
        """
        Converts the TransformStamped to a SpatialTransformationMatrix.

        :return: A SpatialTransformationMatrix object representing the transform in 3D space.
        """
        return SpatialTransformationMatrix.from_xyz_quaternion(
            pos_x=self.translation.x,
            pos_y=self.translation.y,
            pos_z=self.translation.z,
            quat_x=self.rotation.x,
            quat_y=self.rotation.y,
            quat_z=self.rotation.z,
            quat_w=self.rotation.w,
            reference_frame=self.header.frame_id,
            child_frame=self.child_frame_id,
        )

    def inverse_times(self, other: TransformStamped) -> Self:
        """
        Multiply this TransformStamped by the inverse of another TransformStamped. Essentially, this is the same as
        "subtracting" another TransformStamped from this one.

        :param other: The other TransformStamped to subtract.
        :return: A new TransformStamped object that is the result of this transform minus the other transform.
        """
        return self * ~other


# @has_parameters
@dataclass
class GraspPose(PoseStamped):
    """
    A pose from which a grasp can be performed along with the respective arm and grasp description.
    """

    arm: Arms = None
    """
    Arm corresponding to the grasp pose.
    """
    grasp_description: GraspDescription = None
    """
    Grasp description corresponding to the grasp pose.
    """


Point = Vector3
