from __future__ import annotations

import copy
import datetime
from dataclasses import dataclass, field, fields

import numpy as np
from geometry_msgs.msg import (Vector3 as ROSVector3, Quaternion as ROSQuaternion, Point as ROSPoint, Pose as ROSPose,
                               PoseStamped as ROSPoseStamped, Transform as ROSTransform,
                               TransformStamped as ROSTransformStamped)
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header as ROSHeader
from typing_extensions import Self, Tuple, Optional, List, TYPE_CHECKING, Any

from .enums import AxisIdentifier, Arms, Grasp
from .grasp import GraspDescription, PreferredGraspAlignment
from ..has_parameters import has_parameters, HasParameters
from ..ros import Time as ROSTime
from ..tf_transformations import quaternion_multiply, translation_matrix, quaternion_matrix, inverse_matrix, \
    translation_from_matrix, quaternion_from_matrix

if TYPE_CHECKING:
    from ..world_concepts.world_object import Object


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
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    def ros_message(self) -> ROSVector3:
        return ROSVector3(x=self.x, y=self.y, z=self.z)

    def to_list(self):
        return [self.x, self.y, self.z]

    def round(self, decimals: int = 4):
        self.x = round(self.x, decimals)
        self.y = round(self.y, decimals)
        self.z = round(self.z, decimals)

    def almost_equal(self, other: Self, tolerance: float = 1e-6) -> bool:
        return bool(np.isclose(np.array(self.to_list()), np.array(other.to_list()), atol=tolerance).all())

    def vector_to_position(self, other: Self) -> Vector3:
        return other - self

    def to_numpy(self):
        return np.array(self.to_list())

    def __add__(self, other: Self) -> Vector3:
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Self) -> Vector3:
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other: float) -> Vector3:
        return Vector3(self.x * other, self.y * other, self.z * other)

    def __rmul__(self, other: float) -> Vector3:
        return Vector3(self.x * other, self.y * other, self.z * other)

    @classmethod
    def from_list(cls, vector: List[float]) -> Self:
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
        if not (hasattr(self, "x") and
                hasattr(self, "y") and
                hasattr(self, "z") and
                hasattr(self, "w")):
            return

        norm = (self.x ** 2 + self.y ** 2 + self.z ** 2 + self.w ** 2) ** 0.5
        object.__setattr__(self, "x", self.x / norm)
        object.__setattr__(self, "y", self.y / norm)
        object.__setattr__(self, "z", self.z / norm)
        object.__setattr__(self, "w", self.w / norm)

    def ros_message(self) -> ROSQuaternion:
        return ROSQuaternion(x=self.x, y=self.y, z=self.z, w=self.w)

    def to_list(self):
        return [self.x, self.y, self.z, self.w]

    def to_numpy(self):
        return np.array(self.to_list())

    def round(self, decimals: int = 4):
        self.x = round(self.x, decimals)
        self.y = round(self.y, decimals)
        self.z = round(self.z, decimals)
        self.w = round(self.w, decimals)

    def almost_equal(self, other: Self, tolerance: float = 1e-6) -> bool:
        return bool(np.isclose(np.array(self.to_list()), np.array(other.to_list()), atol=tolerance).all())

    def __mul__(self, other: Self) -> Quaternion:
        return quaternion_multiply(self.to_list(), other)

    @classmethod
    def from_list(cls, quaternion: List[float]) -> Self:
        return cls(*quaternion)

    # TODO fix this
    # def __setattr__(self, key, value):
    #     object.__setattr__(self, key, value)
    #     self.normalize()


@has_parameters
@dataclass
class Pose(HasParameters):
    """
    A pose in 3D space.
    """
    position: Vector3 = field(default_factory=Vector3)
    orientation: Quaternion = field(default_factory=Quaternion)

    def __post_init__(self):
        if not isinstance(self.position, Vector3):
            raise TypeError("Pose must have an instance of Vector3")
        if not isinstance(self.orientation, Quaternion):
            raise TypeError("Pose must have an instance of Quaternion")

    def __repr__(self):
        return (f"Pose: {[round(v, 3) for v in [self.position.x, self.position.y, self.position.z]]}, "
                f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]]}")

    def ros_message(self) -> ROSPose:
        point = ROSPoint(x=self.position.x, y=self.position.y, z=self.position.z)
        return ROSPose(position=point, orientation=self.orientation.ros_message())

    def to_list(self):
        return [self.position.to_list(), self.orientation.to_list()]

    def copy(self):
        return copy.copy(self)

    def round(self, decimals: int = 4):
        self.position.round(decimals)
        self.orientation.round(decimals)

    def almost_equal(self, other: Pose, position_tolerance: float = 1e-6, orientation_tolerance: float = 1e-5) -> bool:
        return self.position.almost_equal(other.position, position_tolerance) and self.orientation.almost_equal(
            other.orientation, orientation_tolerance)

    def __eq__(self, other) -> bool:
        return self.almost_equal(other, position_tolerance=1e-4, orientation_tolerance=1e-4)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray):
        translation = translation_from_matrix(matrix)
        rotation = quaternion_from_matrix(matrix)
        return cls.from_list(translation, rotation)

    @classmethod
    def from_list(cls, position: List[float], orientation: List[float]) -> Self:
        return cls(Vector3(position[0], position[1], position[2]),
                   Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]))

    def __setattr__(self, name: str, value: Any):
        # check if the settet type is correct
        field_of_name = [f for f in fields(self.__class__) if f.name == name]
        if field_of_name:
            field_of_name = field_of_name[0]
            field_type = globals()[field_of_name.type]
            assert isinstance(value, field_type)
        object.__setattr__(self, name, value)


@dataclass
class Header:
    """
    A header with a timestamp.
    """
    frame_id: str = "map"
    stamp: datetime.datetime = field(default_factory=datetime.datetime.now, compare=False)
    sequence: int = field(default=0, compare=False)

    def ros_message(self) -> ROSHeader:
        split_time = str(self.stamp.timestamp()).split(".")
        stamp = ROSTime(int(split_time[0]), int(split_time[1]))
        return ROSHeader(frame_id=self.frame_id, stamp=stamp, seq=self.sequence)


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
    def frame_id(self, value: str):
        self.header.frame_id = value

    def __repr__(self):
        return (f"Pose: {[round(v, 3) for v in [self.position.x, self.position.y, self.position.z]]}, "
                f"{[round(v, 3) for v in [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]]} "
                f"in frame_id {self.frame_id}")

    def ros_message(self) -> ROSPoseStamped:
        return ROSPoseStamped(pose=self.pose.ros_message(), header=self.header.ros_message())

    @classmethod
    def from_ros_message(cls, message: ROSPoseStamped):
        header = Header(frame_id=message.header.frame_id, stamp=message.header.stamp)
        position = Vector3(x=message.pose.position.x, y=message.pose.position.y, z=message.pose.position.z)
        orientation = Quaternion(x=message.pose.orientation.x, y=message.pose.orientation.y,
                                 z=message.pose.orientation.z, w=message.pose.orientation.w)
        return cls(pose=Pose(position=position, orientation=orientation), header=header)

    @classmethod
    def from_list(cls, position: Optional[List[float]] = None, orientation: Optional[List[float]] = None,
                  frame: Optional[str] = "map") -> Self:
        position = position or [0.0, 0.0, 0.0]
        orientation = orientation or [0.0, 0.0, 0.0, 1.0]
        return cls(pose=Pose.from_list(position, orientation),
                   header=Header(frame_id=frame, stamp=datetime.datetime.now()))

    def to_transform_stamped(self, child_link_id: str) -> TransformStamped:
        return TransformStamped(header=self.header, pose=Transform.from_pose(self.pose), child_frame_id=child_link_id)

    def round(self, decimals: int = 4):
        self.position.round(decimals)
        self.orientation.round(decimals)

    def copy(self) -> Self:
        return copy.deepcopy(self)

    def to_list(self):
        return self.pose.to_list()

    @staticmethod
    def calculate_closest_faces(pose_to_robot_vector: Vector3,
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
            valid_axes = [axis for axis in all_axes if
                          not np.isnan(pose_to_robot_vector.to_list()[axis.value.index(1)])]

        object_to_robot_vector = np.array(pose_to_robot_vector.to_list()) + 1e-9
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

    def calculate_grasp_descriptions(self, robot: Object, grasp_alignment: Optional[PreferredGraspAlignment] = None) -> \
    List[GraspDescription]:
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

        object_to_robot_vector_world = objectTmap.position.vector_to_position(robot_pose.position)
        orientation = objectTmap.orientation.to_list()

        mapRobject = R.from_quat(orientation).as_matrix()
        objectRmap = mapRobject.T

        object_to_robot_vector_local = objectRmap.dot(object_to_robot_vector_world.to_numpy())
        vector_x, vector_y, vector_z = object_to_robot_vector_local

        vector_side = Vector3(vector_x, vector_y, np.nan)
        side_faces = self.calculate_closest_faces(vector_side, side_axis)

        vector_vertical = Vector3(np.nan, np.nan, vector_z)
        vertical_faces = self.calculate_closest_faces(vector_vertical) if vertical else [None]

        grasp_configs = [
            GraspDescription(approach_direction=side, vertical_alignment=top_face, rotate_gripper=rotated_gripper)
            for top_face in vertical_faces
            for side in side_faces
        ]

        return grasp_configs

    def almost_equal(self, other: PoseStamped, position_tolerance: float = 1e-6,
                     orientation_tolerance: float = 1e-5) -> bool:
        return self.position.almost_equal(other.position, position_tolerance) and self.orientation.almost_equal(
            other.orientation, orientation_tolerance) and self.frame_id == other.frame_id

    def rotate_by_quaternion(self, quaternion: List[float]):
        self.orientation = Quaternion.from_list(self.orientation.to_list())


@dataclass
class Transform(Pose):
    @property
    def translation(self):
        return self.position

    @property
    def rotation(self):
        return self.orientation

    def to_matrix(self):
        translation = translation_matrix(self.translation.to_list())
        rotation = quaternion_matrix(self.rotation.to_list())
        return np.dot(translation, rotation)

    def __invert__(self):
        inv = inverse_matrix(self.to_matrix())
        translation = translation_from_matrix(inv)
        rotation = quaternion_from_matrix(inv)
        return Transform.from_list(translation, rotation)

    def __mul__(self, other: Transform):
        self_matrix = self.to_matrix()
        other_matrix = other.to_matrix()
        multiplication = self_matrix @ other_matrix
        return Transform.from_matrix(multiplication)

    @classmethod
    def from_pose(cls, pose: Pose):
        return cls(pose.position, pose.orientation)

    def ros_message(self) -> ROSTransform:
        return ROSTransform(translation=self.translation.ros_message(), rotation=self.rotation.ros_message())


@has_parameters
@dataclass
class TransformStamped(PoseStamped):
    child_frame_id: str = field(default_factory=str)

    pose: Transform = field(default_factory=Pose)

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

    def __invert__(self):
        result = copy.deepcopy(self)
        result.header.frame_id = self.child_frame_id
        result.child_frame_id = self.header.frame_id
        result.transform = ~self.transform
        return result

    def __mul__(self, other):
        result = copy.deepcopy(self)
        result.child_frame_id = other.child_frame_id
        result.transform = self.transform * other.transform
        return result

    @classmethod
    def from_list(cls, position: List[float] = None, orientation: List[float] = None, frame: str = "map",
                  child_frame_id="") -> Self:
        position = position or [0.0, 0.0, 0.0]
        orientation = orientation or [0.0, 0.0, 0.0, 1.0]
        return cls(pose=Transform.from_list(position, orientation),
                   header=Header(frame_id=frame, stamp=datetime.datetime.now()),
                   child_frame_id=child_frame_id)

    def ros_message(self) -> ROSTransformStamped:
        return ROSTransformStamped(transform=self.transform.ros_message(), header=self.header.ros_message(),
                                   child_frame_id=self.child_frame_id)

    def to_pose_stamped(self) -> PoseStamped:
        return PoseStamped(self.pose, self.header)

    def inverse_times(self, other: TransformStamped):
        return self * ~other


@has_parameters
@dataclass
class GraspPose(PoseStamped):
    arm: Arms = None
    grasp_description: GraspDescription = None


Point = Vector3
