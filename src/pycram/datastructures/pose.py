from __future__ import annotations

import copy
import datetime
import inspect
from dataclasses import dataclass, field, fields

import numpy as np
from typing_extensions import Self, Tuple, Optional, List, TYPE_CHECKING, Union, Any

from .enums import AxisIdentifier, Arms, Grasp
from .grasp import GraspDescription, PreferredGraspAlignment
from ..has_parameters import has_parameters, HasParameters
from ..tf_transformations import quaternion_multiply, translation_matrix, quaternion_matrix, inverse_matrix, \
    translation_from_matrix, rotation_from_matrix, quaternion_from_matrix
from scipy.spatial.transform import Rotation as R
from ..ros import Time as ROSTime
from geometry_msgs.msg import (Vector3 as ROSVector3, Quaternion as ROSQuaternion, Point as ROSPoint, Pose as ROSPose,
                                   PoseStamped as ROSPoseStamped, Transform as ROSTransform, TransformStamped as ROSTransformStamped)
from std_msgs.msg import Header as ROSHeader

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
        # self.x /= norm
        # self.y /= norm
        # self.z /= norm
        # self.w /= norm
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
        return self.position.almost_equal(other.position, position_tolerance) and self.orientation.almost_equal(other.orientation, orientation_tolerance)

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
        orientation = Quaternion(x=message.pose.orientation.x, y=message.pose.orientation.y, z=message.pose.orientation.z, w=message.pose.orientation.w)
        return cls(pose=Pose(position=position, orientation=orientation), header=header)

    @classmethod
    def from_list(cls, position: Optional[List[float]] = None, orientation: Optional[List[float]] = None, frame: Optional[str] = "map") -> Self:
        position = position or [0.0, 0.0, 0.0]
        orientation = orientation or [0.0, 0.0, 0.0, 1.0]
        return cls(pose=Pose.from_list(position, orientation), header=Header(frame_id=frame, stamp=datetime.datetime.now()))

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
            valid_axes = [axis for axis in all_axes if not np.isnan(pose_to_robot_vector.to_list()[axis.value.index(1)])]

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

    def almost_equal(self, other: PoseStamped, position_tolerance: float = 1e-6, orientation_tolerance: float = 1e-5) -> bool:
        return self.position.almost_equal(other.position, position_tolerance) and self.orientation.almost_equal(other.orientation, orientation_tolerance) and self.frame_id == other.frame_id

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
    def from_list(cls, position: List[float] = None, orientation: List[float] = None, frame: str = "map", child_frame_id = "") -> Self:
        position = position or [0.0, 0.0, 0.0]
        orientation = orientation or [0.0, 0.0, 0.0, 1.0]
        return cls(pose=Transform.from_list(position, orientation),
                   header=Header(frame_id=frame, stamp=datetime.datetime.now()),
                   child_frame_id=child_frame_id)

    def ros_message(self) -> ROSTransformStamped:
        return ROSTransformStamped(transform=self.transform.ros_message(), header=self.header.ros_message(), child_frame_id=self.child_frame_id)

    def invert(self):
        return ~self

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



# # used for delayed evaluation of typing until python 3.11 becomes mainstream
# from __future__ import annotations
#
# import datetime
# import math
# from collections.abc import Sequence
#
# import numpy as np
# import sqlalchemy.orm
# import typing_extensions
# from geometry_msgs.msg import (Pose as GeoPose, Quaternion as GeoQuaternion, PoseStamped as ROSPoseStamped)
# from geometry_msgs.msg import TransformStamped, Vector3, Point
# from typing_extensions import List, Union, Optional, Self, Tuple
#
# from .enums import AxisIdentifier, Grasp, Arms
# from .grasp import PreferredGraspAlignment, GraspDescription
# from ..orm.base import Pose as ORMPose, Position, Quaternion, ProcessMetaData
# from ..ros import Time
# from ..ros import logwarn, logerr
# from ..tf_transformations import euler_from_quaternion, translation_matrix, quaternion_matrix, concatenate_matrices, \
#     inverse_matrix, translation_from_matrix, quaternion_from_matrix, quaternion_multiply
# from ..validation.error_checkers import calculate_pose_error
# from scipy.spatial.transform import Rotation as R
#
# if typing_extensions.TYPE_CHECKING:
#     from ..world_concepts.world_object import Object
#
#
# def get_normalized_quaternion(quaternion: np.ndarray) -> GeoQuaternion:
#     """
#     Normalizes a given quaternion such that it has a magnitude of 1.
#
#     :param quaternion: The quaternion that should be normalized
#     :return: The normalized quaternion
#     """
#     mag = math.sqrt(sum(v ** 2 for v in quaternion))
#     normed_rotation = [f / mag for f in quaternion]
#
#     geo_quaternion = GeoQuaternion()
#     geo_quaternion.x = normed_rotation[0]
#     geo_quaternion.y = normed_rotation[1]
#     geo_quaternion.z = normed_rotation[2]
#     geo_quaternion.w = normed_rotation[3]
#
#     return geo_quaternion
#
#
# class PoseStamped(ROSPoseStamped):
#     """
#     Pose representation for PyCRAM, this class extends the PoseStamped ROS message from geometry_msgs. Thus making it
#     compatible with every ROS service and message expecting a PoseStamped message.
#
#     Naming convention for Poses:
#         Pose: Instances of this class, representing a cartesian position and a quaternion for orientation
#
#         Position: Only the cartesian position in xyz
#
#         Orientation: Only the quaternion as xyzw
#     """
#
#     def __init__(self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None,
#                  frame: str = "map", time: Time = None):
#         """
#         Poses can be initialized by a position and orientation given as lists, this is optional. By default, Poses are
#         initialized with the position being [0, 0, 0], the orientation being [0, 0, 0, 1] and the frame being 'map'.
#
#         :param position: An optional position of this Pose
#         :param orientation: An optional orientation of this Pose
#         :param frame: An optional frame in which this pose is
#         :param time: The time at which this Pose is valid, as ROS time
#         """
#         super().__init__()
#         if position is not None:
#             self.position = position
#
#         if orientation is not None:
#             self.orientation = orientation
#         else:
#             self.pose.orientation.w = 1.0
#
#         self.header.frame_id = frame
#
#         self.header.stamp = time if time else Time().now()
#
#         self.frame = frame
#
#     @staticmethod
#     def from_pose_stamped(pose_stamped: ROSPoseStamped) -> PoseStamped:
#         """
#         Converts a geometry_msgs/PoseStamped message to a Pose object. Should be used for compatability with ROS.
#
#         :param pose_stamped: The pose stamped message which should be converted
#         :return: A Pose object with the same information as the given message
#         """
#         p = PoseStamped()
#         p.header = pose_stamped.header
#         p.pose = pose_stamped.pose
#         return p
#
#     def to_pose_stamped(self) -> ROSPoseStamped:
#         """
#         Converts this Pose to a PoseStamped message. This is useful for compatibility with ROS.
#
#         :return: A PoseStamped message with the same information as this Pose
#         """
#         return ROSPoseStamped(
#             header=self.header,
#             pose=self.pose
#         )
#
#     def get_position_diff(self, target_pose: Self) -> Point:
#         """
#         Get the difference between the target and the current positions.
#
#         :param target_pose: The target pose.
#         :return: The difference between the two positions.
#         """
#         return Point(x=target_pose.position.x - self.position.x, y=target_pose.position.y - self.position.y,
#                      z=target_pose.position.z - self.position.z)
#
#     def get_z_angle_difference(self, target_pose: Self) -> float:
#         """
#         Get the difference between two z angles.
#
#         :param target_pose: The target pose.
#         :return: The difference between the two z angles.
#         """
#         return target_pose.z_angle - self.z_angle
#
#     @property
#     def z_angle(self) -> float:
#         """
#         The z angle of the orientation of this Pose in radians.
#         """
#         return euler_from_quaternion(self.orientation.to_list())[2]
#
#     @property
#     def frame(self) -> str:
#         """
#         Property for the frame_id such that it is easier accessible. Instead of Pose.header.frame_id it is Pose.frame
#
#         :return: The TF frame of this Pose
#         """
#         return self.header.frame_id
#
#     @frame.setter
#     def frame(self, value: str) -> None:
#         """
#         Sets the TF frame of this pose to the given new frame
#
#         :param value: The new TF frame
#         """
#         self.header.frame_id = value
#
#     @property
#     def position(self) -> Point:
#         """
#         Property that points to the position of this pose
#         """
#         return self.pose.position
#
#     @position.setter
#     def position(self, value: Union[Sequence[float], GeoPose, Point]) -> None:
#         """
#         Sets the position for this Pose, the position can either be a sequence of xyz, a Point
#         or a geometry_msgs/Pose message.
#
#         :param value: Sequence or geometry_msgs/Pose message for the position
#         """
#         if not isinstance(value, (list, tuple, np.ndarray, GeoPose, Point)):
#             err_msg = "Position can only be one of (list, tuple, np.ndarray, geometry_msgs/Pose, Point) not " + \
#                       str(type(value))
#             logerr(err_msg)
#             raise TypeError(err_msg)
#         if isinstance(value, (list, tuple, np.ndarray)) and len(value) == 3:
#             self.pose.position.x = value[0]
#             self.pose.position.y = value[1]
#             self.pose.position.z = value[2]
#         else:
#             # TODO: Check if this is correct or if it should be handled as an error
#             self.pose.position = value
#
#     @property
#     def orientation(self) -> GeoQuaternion:
#         """
#         Property that points to the orientation of this pose
#         """
#         return self.pose.orientation
#
#     @orientation.setter
#     def orientation(self, value: Union[Sequence[float], GeoQuaternion]) -> None:
#         """
#         Sets the orientation of this Pose, the orientation can either be a sequence of xyzw
#         or a geometry_msgs/Quaternion message
#
#         :param value: New orientation, either a list or geometry_msgs/Quaternion
#         """
#         if not isinstance(value, (list, tuple, np.ndarray, GeoQuaternion)):
#             err_msg = (f"Orientation can only be a Sequence (list, tuple, ...etc.) or a geometry_msgs/Quaternion "
#                        f"not {type(value)}")
#             logerr(err_msg)
#             raise TypeError(err_msg)
#
#         if isinstance(value, (list, tuple, np.ndarray)) and len(value) == 4:
#             orientation = np.array(value)
#         else:
#             orientation = np.array([value.x, value.y, value.z, value.w])
#         # This is used instead of np.linalg.norm since numpy is too slow on small arrays
#         self.pose.orientation = get_normalized_quaternion(orientation)
#
#     def to_list(self) -> List[List[float]]:
#         """
#         :return: The position and orientation as lists
#         """
#         return [[self.pose.position.x, self.pose.position.y, self.pose.position.z],
#                 [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]]
#
#     def to_transform(self, child_frame: str) -> Transform:
#         """
#         Converts this pose to a Transform from the TF frame of the pose to the given child_frame
#
#         :param child_frame: Child frame id to which the Transform points
#         :return: A new Transform
#         """
#         return Transform(self.position.to_list(), self.orientation.to_list(), self.frame, child_frame,
#                          self.header.stamp)
#
#     def copy(self) -> PoseStamped:
#         """
#         Creates a deep copy of this pose.
#
#         :return: A copy of this pose
#         """
#         p = PoseStamped(self.position.to_list(), self.orientation.to_list(), self.frame, self.header.stamp)
#         p.header.frame_id = self.header.frame_id
#         return p
#
#     def position.to_list()(self) -> List[float]:
#         """
#         :return: The position as a list of xyz values.
#         """
#         return [self.position.x, self.position.y, self.position.z]
#
#     def orientation.to_list()(self) -> List[float]:
#         """
#         :return: The orientation as a quaternion with xyzw
#         """
#         return [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
#
#     def dist(self, other_pose: PoseStamped) -> float:
#         """
#         Calculates the euclidian distance between this Pose and the given one. For distance calculation only the
#         position is used.
#
#         :param other_pose: Pose to which the distance should be calculated
#         :return: The distance between the Poses
#         """
#         self_position = self.position.to_list()
#         other_position = other_pose.position.to_list()
#         return np.linalg.norm(np.array(self_position) - np.array(other_position))
#
#     def __eq__(self, other: PoseStamped) -> bool:
#         """
#         Overloads the '==' operator to check for equality between two Poses. Only compares the position, orientation and
#         frame. Timestamps of Poses are not takes into account.
#
#         :param other: Other pose which should be compared
#         :return: True if both Poses have the same position, orientation and frame. False otherwise
#         """
#         if not isinstance(other, PoseStamped):
#             return False
#         self_position = self.position.to_list()
#         other_position = other.position.to_list()
#
#         self_orient = self.orientation.to_list()
#         other_orient = other.orientation.to_list()
#
#         return self_position == other_position and self_orient == other_orient and self.frame == other.frame
#
#     def almost_equal(self, other: PoseStamped, position_tolerance_in_meters: float = 1e-3,
#                      orientation_tolerance_in_degrees: float = 1) -> bool:
#         """
#         Checks if the given Pose is almost equal to this Pose. The position and orientation can have a certain
#         tolerance. The position tolerance is given in meters and the orientation tolerance in degrees. The position
#         error is calculated as the euclidian distance between the positions and the orientation error as the angle
#         between the quaternions.
#
#         :param other: The other Pose which should be compared
#         :param position_tolerance_in_meters: The tolerance for the position in meters
#         :param orientation_tolerance_in_degrees: The tolerance for the orientation in degrees
#         :return: True if the Poses are almost equal, False otherwise
#         """
#         error = calculate_pose_error(self, other)
#         return error[0] <= position_tolerance_in_meters and error[1] <= orientation_tolerance_in_degrees * math.pi / 180
#
#     def set_position(self, new_position: List[float]) -> None:
#         """
#         Sets the position of this Pose to the given position. Position has to be given as a vector in cartesian space.
#
#         :param new_position: New position as a vector of xyz
#         """
#         self.position = new_position
#
#     def set_orientation(self, new_orientation: List[float]) -> None:
#         """
#         Sets the orientation to the given quaternion. The new orientation has to be given as a quaternion.
#
#         :param new_orientation: New orientation as a quaternion with xyzw
#         """
#         self.orientation = new_orientation
#
#     def round(self, decimals: int = 4) -> None:
#         """
#         Rounds the position and orientation of this Pose to the given number of decimals.
#
#         :param decimals: The number of decimals to which the position and orientation should be rounded
#         """
#         self.position = [round(v, decimals) for v in self.position.to_list()]
#         self.orientation = [round(v, decimals) for v in self.orientation.to_list()]
#
#     def rotate_by_quaternion(self, quaternion: Tuple[float, float, float, float]) -> None:
#         """
#         Rotates this Pose by the given quaternion. The orientation of this Pose is multiplied by the given quaternion,
#         according to the hamilton product. The quaternion has to be given as a tuple with xyzw.
#         The orientation is normalized after the rotation.
#
#         :param quaternion: The quaternion by which this Pose should be rotated
#         """
#         self.orientation = quaternion_multiply(self.orientation.to_list(), quaternion)
#
#     def get_vector_to_pose(self, other_pose: PoseStamped) -> np.ndarray:
#         """
#         Get the vector between two poses, by computing position of other_pose - position of self.
#
#         :param other_pose: The other pose.
#
#         :return: The vector between the two poses.
#         """
#         return np.array(other_pose.position.to_list()) - np.array(self.position.to_list())
#
#     @staticmethod
#     def calculate_closest_faces(pose_to_robot_vector: Tuple[float, float, float],
#                                 specified_grasp_axis: Optional[AxisIdentifier] = None) -> Tuple[
#         GraspDescription, GraspDescription]:
#         """
#         Determines the faces of the object based on the input vector.
#
#         If `specified_grasp_axis` is None, it calculates the primary and secondary faces based on the vector's magnitude
#         determining which sides of the object are most aligned with the robot. This will either be the x, y plane for side faces
#         or the z axis for top/bottom faces.
#         If `specified_grasp_axis` is provided, it only considers the specified axis and calculates the faces aligned
#         with that axis.
#
#         :param pose_to_robot_vector: A 3D vector representing one of the robot's axes in the pose's frame, with
#                               irrelevant components set to np.nan.
#         :param specified_grasp_axis: Specifies a specific axis (e.g., X, Y, Z) to focus on.
#
#         :return: A tuple of two Grasp enums representing the primary and secondary faces.
#         """
#         all_axes = [AxisIdentifier.X, AxisIdentifier.Y, AxisIdentifier.Z]
#
#         if specified_grasp_axis:
#             valid_axes = [specified_grasp_axis]
#         else:
#             valid_axes = [axis for axis in all_axes if not np.isnan(pose_to_robot_vector[axis.value.index(1)])]
#
#         object_to_robot_vector = np.array(pose_to_robot_vector) + 1e-9
#         sorted_axes = sorted(valid_axes, key=lambda axis: abs(object_to_robot_vector[axis.value.index(1)]),
#                              reverse=True)
#
#         primary_axis = sorted_axes[0]
#         primary_sign = int(np.sign(object_to_robot_vector[primary_axis.value.index(1)]))
#         primary_face = Grasp.from_axis_direction(primary_axis, primary_sign)
#
#         if len(sorted_axes) > 1:
#             secondary_axis = sorted_axes[1]
#             secondary_sign = int(np.sign(object_to_robot_vector[secondary_axis.value.index(1)]))
#             secondary_face = Grasp.from_axis_direction(secondary_axis, secondary_sign)
#         else:
#             secondary_sign = -primary_sign
#             secondary_axis = primary_axis
#             secondary_face = Grasp.from_axis_direction(secondary_axis, secondary_sign)
#
#         return primary_face, secondary_face
#
#     def calculate_grasp_descriptions(self, robot: Object, grasp_alignment: Optional[PreferredGraspAlignment] = None) -> List[GraspDescription]:
#         """
#         This method determines the possible grasp configurations (approach axis and vertical alignment) of the self,
#         taking into account the self's orientation, position, and whether the gripper should be rotated by 90°.
#
#         :param robot: The robot for which the grasp configurations are being calculated.
#
#         :return: A sorted list of GraspDescription instances representing all grasp permutations.
#         """
#         objectTmap = self
#
#         robot_pose = robot.get_pose()
#
#         if grasp_alignment:
#             side_axis = grasp_alignment.preferred_axis
#             vertical = grasp_alignment.with_vertical_alignment
#             rotated_gripper = grasp_alignment.with_rotated_gripper
#         else:
#             side_axis, vertical, rotated_gripper = None, False, False
#
#         object_to_robot_vector_world = objectTmap.get_vector_to_pose(robot_pose)
#         orientation = objectTmap.orientation.to_list()
#
#         mapRobject = R.from_quat(orientation).as_matrix()
#         objectRmap = mapRobject.T
#
#         object_to_robot_vector_local = objectRmap.dot(object_to_robot_vector_world)
#         vector_x, vector_y, vector_z = object_to_robot_vector_local
#
#         vector_side = np.array([vector_x, vector_y, np.nan], dtype=float)
#         side_faces = self.calculate_closest_faces(vector_side, side_axis)
#
#         vector_vertical = np.array([np.nan, np.nan, vector_z], dtype=float)
#         vertical_faces = self.calculate_closest_faces(vector_vertical) if vertical else [None]
#
#         grasp_configs = [
#             GraspDescription(approach_direction=side, vertical_alignment=top_face, rotate_gripper=rotated_gripper)
#             for top_face in vertical_faces
#             for side in side_faces
#         ]
#
#         return grasp_configs
#
#     def __str__(self):
#         return (f"Pose: {[round(v, 3) for v in self.position.to_list()]}, {[round(v, 3) for v in self.orientation.to_list()]}"
#                 f" in frame {self.frame}")
#
#     def __repr__(self):
#         return self.__str__()
#
#
# class TransformStamped.from_list(TransformStamped):
#     """
#     Represents a Transformation from one TF frame to another in PyCRAM. Like with Poses this class inherits from the ROS
#     message TransformStamped form geometry_msgs and is therefore compatible with ROS services and messages that require
#     a TransformStamped message.
#
#     Naming Convention for Transforms:
#         Transform: Instances of this class, representing a translation and rotation from frame_id to child_frame_id
#
#         Translation: A vector representing the conversion in cartesian space
#
#         Rotation: A quaternion representing the conversion of rotation between both frames
#     """
#
#     def __init__(self, translation: Optional[List[float]] = None, rotation: Optional[List[float]] = None,
#                  frame: Optional[str] = "map", child_frame: Optional[str] = "", time: Time = None):
#         """
#         Transforms take a translation, rotation, frame and child_frame as optional arguments. If nothing is given the
#         Transform will be initialized with [0, 0, 0] for translation, [0, 0, 0, 1] for rotation, 'map' for frame and an
#         empty string for child_frame
#
#         :param translation: Optional translation from frame to child_frame in cartesian space
#         :param rotation: Optional rotation from frame to child frame given as quaternion
#         :param frame: Origin TF frame of this Transform
#         :param child_frame: Target frame for this Transform
#         :param time: The time at which this Transform is valid, as ROS time
#         """
#         super().__init__()
#         if translation:
#             self.translation = translation
#
#         if rotation:
#             self.rotation = rotation
#         else:
#             self.transform.rotation.w = 1.0
#
#         self.header.frame_id = frame
#         self.child_frame_id = child_frame
#         self.header.stamp = time if time else Time().now()
#
#         self.frame = frame
#
#     def apply_transform_to_array_of_points(self, points: np.ndarray) -> np.ndarray:
#         """
#         Applies this Transform to an array of points. The points are given as a Nx3 matrix, where N is the number of
#         points. The points are transformed from the child_frame_id to the frame_id of this Transform.
#
#         :param points: The points that should be transformed, given as a Nx3 matrix.
#         """
#         homogeneous_transform = self.get_homogeneous_matrix()
#         # add the homogeneous coordinate, by adding a column of ones to the position vectors, becoming 4xN matrix
#         homogenous_points = np.concatenate((points, np.ones((points.shape[0], 1))), axis=1).T
#         transformed_points = homogeneous_transform @ homogenous_points
#         return transformed_points[:3, :].T
#
#     def get_homogeneous_matrix(self) -> np.ndarray:
#         """
#         :return: The homogeneous matrix of this Transform
#         """
#         translation = translation_matrix(self.translation.to_list())
#         rotation = quaternion_matrix(self.rotation.to_list())
#         return np.dot(translation, rotation)
#
#     @classmethod
#     def from_pose_and_child_frame(cls, pose: PoseStamped, child_frame_name: str) -> TransformStamped:
#         return cls(pose.position.to_list(), pose.orientation.to_list(), pose.frame, child_frame_name,
#                    time=pose.header.stamp)
#
#     @staticmethod
#     def from_transform_stamped(transform_stamped: TransformStamped) -> TransformStamped:
#         """
#         Creates a Transform instance from a geometry_msgs/TransformStamped message. Should be used for compatibility with
#         ROS.
#
#         :param transform_stamped: The transform stamped message that should be converted
#         :return: An Transform with the same information as the transform stamped message
#         """
#         t = TransformStamped.from_list()
#         t.header = transform_stamped.header
#         t.child_frame_id = transform_stamped.child_frame_id
#         t.transform = transform_stamped.transform
#
#         return t
#
#     @property
#     def frame(self) -> str:
#         """
#         Property for the frame_id such that it is easier accessible. Instead of Pose.header.frame_id it is Pose.frame
#
#         :return: The TF frame of this Pose
#         """
#         return self.header.frame_id
#
#     @frame.setter
#     def frame(self, value: str) -> None:
#         """
#         Sets the TF frame of this pose to the given new frame
#
#         :param value: The new TF frame
#         """
#         self.header.frame_id = value
#
#     @property
#     def translation(self) -> Vector3:
#         """
#         Property that points to the translation of this Transform
#         """
#         return self.transform.translation
#
#     @translation.setter
#     def translation(self, value) -> None:
#         """
#         Setter for the translation of this Transform, the new value can either be of type list or a
#         geometry_msgs/Vector message.
#
#         :param value: The new value for the translation, either a list or geometry_msgs/Vector3
#         """
#         if not isinstance(value, list) and not isinstance(value, Vector3):
#             logwarn("Value of a translation can only be a list or a geometry_msgs/Vector3")
#             return
#         if isinstance(value, list) and len(value) == 3:
#             self.transform.translation.x = value[0]
#             self.transform.translation.y = value[1]
#             self.transform.translation.z = value[2]
#         else:
#             self.transform.translation = value
#
#     @property
#     def rotation(self) -> Quaternion:
#         """
#         Property that points to the rotation of this Transform
#         """
#         return self.transform.rotation
#
#     @rotation.setter
#     def rotation(self, value):
#         """
#         Setter for the rotation of this Transform, the new value can either be a list or a geometry_msgs/Quaternion
#         message
#
#         :param value: The new value for the rotation, either a list or geometry_msgs/Quaternion
#         """
#         if not isinstance(value, list) and not isinstance(value, GeoQuaternion):
#             logwarn("Value of the rotation can only be a list or a geometry.msgs/Quaternion")
#             return
#         if isinstance(value, list) and len(value) == 4:
#             rotation = np.array(value)
#
#         else:
#             rotation = np.array([value.x, value.y, value.z, value.w])
#         # This is used instead of np.linalg.norm since numpy is too slow on small arrays
#         self.transform.rotation = get_normalized_quaternion(rotation)
#
#     def copy(self) -> TransformStamped:
#         """
#         Creates a deep copy of this pose.
#
#         :return: A copy of this pose
#         """
#         t = TransformStamped.from_list(self.translation.to_list(), self.rotation.to_list(), self.frame, self.child_frame_id,
#                              self.header.stamp)
#         t.header.frame_id = self.header.frame_id
#         # t.header.stamp = self.header.stamp
#         return t
#
#     def translation.to_list(self) -> List[float]:
#         """
#         :return: The translation as a list of xyz
#         """
#         return [self.transform.translation.x, self.transform.translation.y, self.transform.translation.z]
#
#     def rotation.to_list(self) -> List[float]:
#         """
#         :return: The rotation of this Transform as a list with xyzw
#         """
#         return [self.transform.rotation.x, self.transform.rotation.y, self.transform.rotation.z,
#                 self.transform.rotation.w]
#
#     def to_pose(self) -> PoseStamped:
#         """
#         Converts this Transform to a Pose, in this process the child_frame_id is lost.
#
#         :return: A new pose with same translation as position and rotation as orientation
#         """
#         return PoseStamped(self.translation.to_list(), self.rotation.to_list(), self.frame, self.header.stamp)
#
#     def invert(self) -> TransformStamped:
#         """
#         Inverts this Transform, the new Transform points from the child_frame_id to the frame_id
#
#         :return: A new inverted Transform
#         """
#         transform = concatenate_matrices(translation_matrix(self.translation.to_list()),
#                                          quaternion_matrix(self.rotation.to_list()))
#         inverse_transform = inverse_matrix(transform)
#         translation = translation_from_matrix(inverse_transform)
#         quaternion = quaternion_from_matrix(inverse_transform)
#         return TransformStamped.from_list(list(translation), list(quaternion), self.child_frame_id, self.header.frame_id,
#                                 self.header.stamp)
#
#     def round(self, decimals: int = 4):
#         """
#         Rounds the translation and rotation of this Transform to the given number of decimals.
#
#         :param decimals: The number of decimals to which the translation and rotation should be rounded
#         """
#         self.translation = [round(v, decimals) for v in self.translation.to_list()]
#         self.rotation = [round(v, decimals) for v in self.rotation.to_list()]
#
#     def __mul__(self, other: TransformStamped) -> Union[TransformStamped, None]:
#         """
#         Multiplies this Transform with another one. The resulting Transform points from the frame_id of this Transform
#         to the child_frame_id of the other Transform.
#
#         :param other: The Transform which should be multiplied with this one.
#         :return: The resulting Transform from the multiplication
#         """
#         if not isinstance(other, TransformStamped):
#             logerr(f"Can only multiply two Transforms")
#             return
#         self_trans = translation_matrix(self.translation.to_list())
#         self_rot = quaternion_matrix(self.rotation.to_list())
#         self_mat = np.dot(self_trans, self_rot)
#
#         other_trans = translation_matrix(other.translation.to_list())
#         other_rot = quaternion_matrix(other.rotation.to_list())
#         other_mat = np.dot(other_trans, other_rot)
#
#         new_mat = np.dot(self_mat, other_mat)
#         new_trans = translation_from_matrix(new_mat)
#         new_rot = quaternion_from_matrix(new_mat)
#         return TransformStamped.from_list(list(new_trans), list(new_rot), self.frame, other.child_frame_id)
#
#     def inverse_times(self, other_transform: TransformStamped) -> TransformStamped:
#         """
#         Like a 'minus' for Transforms, subtracts the other_transform from this one.
#
#         :param other_transform: Transform which should be subtracted from this one
#         :return: The resulting Transform form the calculation
#         """
#         inv = other_transform.invert()
#         return self * inv
#
#     def __eq__(self, other: TransformStamped) -> bool:
#         """
#         Overloads the '==' operator to check for equality between two Transforms. Only compares the translation,
#         rotation, frame and child frame. Timestamps of Poses are not takes into account.
#
#         :param other: Other pose which should be compared
#         :return: True if both Transforms have the same translation, rotation, frame and child frame. False otherwise
#         """
#         if not isinstance(other, TransformStamped):
#             return False
#         self_position = self.translation.to_list()
#         other_position = other.translation.to_list()
#
#         self_orient = self.rotation.to_list()
#         other_orient = other.rotation.to_list()
#
#         return self_position == other_position and self_orient == other_orient and \
#             self.frame == other.frame and self.child_frame_id == other.child_frame_id
#
#     def set_translation(self, new_translation: List[float]) -> None:
#         """
#         Sets the translation of this Transform to the newly given one. Translation has to be a vector in cartesian space
#
#         :param new_translation: The new translation as a vector with xyz.
#         """
#         self.translation = new_translation
#
#     def set_rotation(self, new_rotation: List[float]) -> None:
#         """
#         Sets the rotation of this Transform to the newly given one. Rotation has to be a quaternion.
#
#         :param new_rotation: The new rotation as a quaternion with xyzw
#         """
#         self.rotation = new_rotation
# #
#
# class GraspPose(PoseStamped):
#
#     arm: Arms = None
#     """
#     The arm with which the grasp pose is attached to.
#     """
#     grasp_description: GraspDescription
#     """
#     The grasp description of the grasp.
#     """
#
#     def __init__(self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None,
#                  frame: str = "map", time: Time = None, arm: Arms = None, grasp_description: GraspDescription = None):
#         """
#         Poses can be initialized by a position and orientation given as lists, this is optional. By default, Poses are
#         initialized with the position being [0, 0, 0], the orientation being [0, 0, 0, 1] and the frame being 'map'.
#
#         :param position: An optional position of this Pose
#         :param orientation: An optional orientation of this Pose
#         :param frame: An optional frame in which this pose is
#         :param time: The time at which this Pose is valid, as ROS time
#         :param arm: The arm with which the grasp from this pose can be achieved
#         :param grasp_description: The grasp description which needs to be used
#         """
#         super().__init__()
#         if position is not None:
#             self.position = position
#
#         if orientation is not None:
#             self.orientation = orientation
#         else:
#             self.pose.orientation.w = 1.0
#
#         self.header.frame_id = frame
#
#         self.header.stamp = time if time else Time().now()
#
#         self.frame = frame
#
#         self.arm = arm
#
#         self.grasp_description = grasp_description