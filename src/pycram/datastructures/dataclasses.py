from __future__ import annotations

import itertools
import math
from abc import ABC, abstractmethod
from copy import deepcopy, copy
from dataclasses import dataclass, fields, field
from enum import Enum
from typing import Iterator

import numpy as np
import plotly.graph_objects as go
import trimesh
from matplotlib import pyplot as plt

from random_events.interval import closed, SimpleInterval, Bound
from random_events.product_algebra import SimpleEvent, Event
from random_events.variable import Continuous
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.world_modification import WorldModelModificationBlock
from typing_extensions import List, Optional, Tuple, Callable, Dict, Any, Union, TYPE_CHECKING, Sequence, Self, \
    deprecated, Type

from .enums import JointType, Shape, VirtualMobileBaseJointName, Grasp, ApproachDirection, VerticalAlignment
from .pose import PoseStamped, Point, TransformStamped
from ..ros import logwarn, logwarn_once
from ..validation.error_checkers import calculate_joint_position_error, is_error_acceptable


@dataclass
class ExecutionData:
    """
    A dataclass for storing the information of an execution that is used for creating a robot description for that
    execution. An execution is a Robot with a virtual mobile base that can be used to move the robot in the environment.
    """
    execution_start_pose: PoseStamped
    """
    Start of the robot at the start of execution of an action designator
    """

    execution_start_world_state: np.ndarray
    """
    The world state at the start of execution of an action designator
    """

    execution_end_pose: PoseStamped  = None
    """
    The pose of the robot at the end of executing an action designator
    """

    execution_end_world_state: np.ndarray = None
    """
    The world state at the end of executing an action designator
    """

    added_world_modifications: List[WorldModelModificationBlock] = None
    """
    A list of World modification blocks that were added during the execution of the action designator
    """

    manipulated_body_pose_start: PoseStamped = None
    """
    Start pose of the manipulated Body if there was one
    """

    manipulated_body_pose_end: PoseStamped = None
    """
    End pose of the manipulated Body if there was one
    """

    manipulated_body: Body = None
    """
    Reference to the manipulated body 
    """


@dataclass
class ManipulatorData:
    """
    A dataclass for storing the information of a manipulator that is used for creating a robot description for that
    manipulator. A manipulator is an Arm with an end-effector that can be used to interact with the environment.
    """
    name: str
    """
    Name of the Manipulator.
    """
    base_link: str
    """
    Manipulator's base link.
    """
    arm_end_link: str
    """
    Manipulator's arm end link.
    """
    joint_names: List[str]
    """
    List of joint names.
    """
    home_joint_values: List[float]
    """
    List of joint values for the home position. (default position)
    """
    gripper_name: str
    """
    Name of the gripper at the end of the arm.
    """
    gripper_tool_frame: str
    """
    Name of the frame of the gripper tool.
    """
    gripper_joint_names: List[str]
    """
    List of gripper joint names.
    """
    closed_joint_values: List[float]
    """
    List of joint values for the gripper in the closed position.
    """
    open_joint_values: List[float]
    """
    List of joint values for the gripper in the open position.
    """
    opening_distance: float
    """
    The opening distance of the gripper.
    """
    fingers_link_names: Optional[List[str]] = None
    """
    List of link names for the fingers of the gripper.
    """
    relative_dir: str = ''
    """
    Relative directory of the manipulator description file in the resources directory.
    """
    gripper_cmd_topic: Optional[str] = None
    """
    Gripper command topic in ROS if it has one.
    """
    gripper_open_cmd_value: Optional[float] = None
    """
    Grip open command value.
    """
    gripper_close_cmd_value: Optional[float] = None
    """
    Grip close command value.
    """
    gripper_relative_dir: Optional[str] = None
    """
    Relative directory of the gripper description file in the resources directory if it has one and is not part of the
     manipulator description file.
    """


def get_point_as_list(point: Point) -> List[float]:
    """
    Return the point as a list.

    :param point: The point.
    :return: The point as a list
    """
    return [point.x, point.y, point.z]


@dataclass
class Color:
    """
    Dataclass for storing rgba_color as an RGBA value.
    The values are stored as floats between 0 and 1.
    The default rgba_color is white. 'A' stands for the opacity.
    """
    R: float = 1
    G: float = 1
    B: float = 1
    A: float = 1

    @classmethod
    def from_list(cls, color: List[float]):
        """
        Set the rgba_color from a list of RGBA values.

        :param color: The list of RGBA values
        """
        if len(color) == 3:
            return cls.from_rgb(color)
        elif len(color) == 4:
            return cls.from_rgba(color)
        else:
            raise ValueError("Color list must have 3 or 4 elements")

    @classmethod
    def from_rgb(cls, rgb: List[float]):
        """
        Set the rgba_color from a list of RGB values.

        :param rgb: The list of RGB values
        """
        return cls(rgb[0], rgb[1], rgb[2], 1)

    @classmethod
    def from_rgba(cls, rgba: List[float]):
        """
        Set the rgba_color from a list of RGBA values.

        :param rgba: The list of RGBA values
        """
        return cls(rgba[0], rgba[1], rgba[2], rgba[3])

    def get_rgba(self) -> List[float]:
        """
        Return the rgba_color as a list of RGBA values.

        :return: The rgba_color as a list of RGBA values
        """
        return [self.R, self.G, self.B, self.A]

    def get_rgb(self) -> List[float]:
        """
        Return the rgba_color as a list of RGB values.

        :return: The rgba_color as a list of RGB values
        """
        return [self.R, self.G, self.B]


class Colors(Color, Enum):
    """
    Enum for easy access to some common colors.
    """
    PINK = (1, 0, 1, 1)
    BLACK = (0, 0, 0, 1)
    WHITE = (1, 1, 1, 1)
    RED = (1, 0, 0, 1)
    GREEN = (0, 1, 0, 1)
    BLUE = (0, 0, 1, 1)
    YELLOW = (1, 1, 0, 1)
    CYAN = (0, 1, 1, 1)
    MAGENTA = (1, 0, 1, 1)
    GREY = (0.5, 0.5, 0.5, 1)

    @classmethod
    def from_string(cls, color: str) -> Color:
        """
        Set the rgba_color from a string. If the string is not a valid color, it will return the color WHITE.

        :param color: The string of the color
        """
        try:
            return cls[color.upper()]
        except KeyError:
            return cls.WHITE


@dataclass
class CollisionCallbacks:
    """
    Dataclass for storing the collision callbacks which are callables that get called when there is a collision
    or when a collision is no longer there.
    """
    on_collision_cb: Callable
    no_collision_cb: Optional[Callable] = None

@dataclass
class LateralFriction:
    """
    Dataclass for storing the information of the lateral friction.
    """
    lateral_friction: float
    lateral_friction_direction: List[float]

@dataclass
class TextAnnotation:
    """
    Dataclass for storing text annotations that can be displayed in the simulation.
    """
    text: str
    position: List[float]
    id: int
    color: Color = field(default_factory=lambda: Color(0, 0, 0, 1))
    size: float = 0.1


@dataclass
class VirtualJoint:
    """
    A virtual (not real) joint that is most likely used for simulation purposes.
    """
    name: str
    type_: JointType
    axes: Optional[Point] = None

    @property
    def type(self):
        return self.type_

    @property
    def is_virtual(self):
        return True

    def __hash__(self):
        return hash(self.name)


class Rotations(Dict[Optional[Union[Grasp, bool]], List[float]]):
    """
    A dictionary that defines standard quaternions for different grasps and orientations. This is mainly used
    to automatically calculate all grasp descriptions of a robot gripper for the robot description.

    SIDE_ROTATIONS: The quaternions for the different approach directions (front, back, left, right)
    VERTICAL_ROTATIONS: The quaternions for the different vertical alignments, in case the object requires for
    example a top grasp
    HORIZONTAL_ROTATIONS: The quaternions for the different horizontal alignments, in case the gripper needs to roll
    90°
    """
    SIDE_ROTATIONS = {
        ApproachDirection.FRONT: [0, 0, 0, 1],
        ApproachDirection.BACK: [0, 0, 1, 0],
        ApproachDirection.LEFT: [0, 0, -math.sqrt(2) / 2, math.sqrt(2) / 2],
        ApproachDirection.RIGHT: [0, 0, math.sqrt(2) / 2, math.sqrt(2) / 2],
    }

    VERTICAL_ROTATIONS = {
        VerticalAlignment.NoAlignment: [0, 0, 0, 1],
        VerticalAlignment.TOP: [0, math.sqrt(2) / 2, 0, math.sqrt(2) / 2],
        VerticalAlignment.BOTTOM: [0, -math.sqrt(2) / 2, 0, math.sqrt(2) / 2],
    }

    HORIZONTAL_ROTATIONS = {
        False: [0, 0, 0, 1],
        True: [math.sqrt(2) / 2, 0, 0, math.sqrt(2) / 2],
    }


@dataclass
class VirtualMobileBaseJoints:
    """
    Dataclass for storing the names, types and axes of the virtual mobile base joints of a mobile robot.
    """

    translation_x: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.LINEAR_X.value,
                                                         JointType.PRISMATIC,
                                                         Point(x=1.0, y=0.0, z=0.0))
    translation_y: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.LINEAR_Y.value,
                                                         JointType.PRISMATIC,
                                                         Point(x=0.0, y=1.0, z=0.0))
    angular_z: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.ANGULAR_Z.value,
                                                     JointType.REVOLUTE,
                                                     Point(x=0.0, y=0.0, z=1.0))

    @property
    def names(self) -> List[str]:
        """
        Return the names of the virtual mobile base joints.
        """
        return [getattr(self, field.name).name for field in fields(self)]

    def get_types(self) -> Dict[str, JointType]:
        """
        Return the joint types of the virtual mobile base joints.
        """
        return {getattr(self, field.name).name: getattr(self, field.name).type_ for field in fields(self)}

    def get_axes(self) -> Dict[str, Point]:
        """
        Return the axes (i.e. The axis on which the joint moves) of the virtual mobile base joints.
        """
        return {getattr(self, field.name).name: getattr(self, field.name).axes for field in fields(self)}


@dataclass
class MultiverseMetaData:
    """Meta data for the Multiverse Client, the simulation_name should be non-empty and unique for each simulation"""
    world_name: str = "world"
    simulation_name: str = "cram"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


@dataclass
class RayResult:
    """
    A dataclass to store the ray result. The ray result contains the body name that the ray intersects with and the
    distance from the ray origin to the intersection point.
    """
    obj_id: int
    """
    The object id of the body that the ray intersects with.
    """
    link_id: int = -1
    """
    The link id of the body that the ray intersects with, -1 if root link or None.
    """
    _hit_fraction: Optional[float] = None  # TODO: Not sure of definition
    """
    The fraction of the ray length at which the intersection point is located a range in [0, 1].
    """
    hit_position: Optional[List[float]] = None
    """
    The intersection point in cartesian world coordinates.
    """
    hit_normal: Optional[List[float]] = None
    """
    The normal at the intersection point in cartesian world coordinates.
    """
    distance: Optional[float] = None
    """
    The distance from the ray origin to the intersection point.
    """

    @property
    def intersected(self) -> bool:
        """
        Check if the ray intersects with a body.
        return: Whether the ray intersects with a body.
        """
        #if not self.obj_id:
        #    logwarn("obj_id should be available to check if the ray intersects with a body,"
        #            "It appears that the ray result is not valid.")
        return self.obj_id != -1

    @property
    def hit_fraction(self) -> Optional[float]:
        if not self._hit_fraction and self.obj_id == -1:
            return 1.0
        return self._hit_fraction

    @hit_fraction.setter
    def hit_fraction(self, value: float):
        self._hit_fraction = value

    def update_distance(self, from_position: List[float], to_position: Optional[List[float]] = None) -> float:
        """
        The distance from the ray origin to the intersection point.
        """
        if self.hit_position:
            self.distance = float(np.linalg.norm(np.array(self.hit_position) - np.array(from_position)))
            if not self.hit_fraction:
                self.hit_fraction = self.distance / np.linalg.norm(np.array(to_position) - np.array(from_position))
            return self.distance
        elif not self.hit_fraction or not to_position:
            raise ValueError(f"Either hit_position or (to_position and hit_fraction)"
                             f" should be available to calculate distance,"
                             f" given hit_fraction: {self.hit_fraction}, to_position: {to_position}")
        return np.linalg.norm(np.array(to_position) - np.array(from_position)) * self.hit_fraction


@deprecated("Use RayResult instead")
@dataclass
class MultiverseRayResult:
    """
    A dataclass to store the ray result. The ray result contains the body name that the ray intersects with and the
    distance from the ray origin to the intersection point.
    """
    body_name: str
    distance: float

    def intersected(self) -> bool:
        """
        Check if the ray intersects with a body.
        return: Whether the ray intersects with a body.
        """
        return self.distance >= 0 and self.body_name != ""


@dataclass
class MultiverseContactPoint:
    """
    A dataclass to store all the contact data returned from Multiverse for a single object.
    """
    body_1: str
    body_2: str
    position: List[float]
    normal: List[float]


@dataclass
class ReasoningResult:
    """
    Result of a reasoning result of knowledge source
    """
    success: bool
    reasoned_parameter: Dict[str, Any] = field(default_factory=dict)



@dataclass
class FrozenObject:

    name: str
    """
    Name of this Object
    """
    concept: Type[PhysicalObject]
    """
    The Concept of the Object as the PyCRAP concept
    """
    path: Optional[str] = None
    """
    The path to the source file
    """
    description: Optional[ObjectDescription] = None
    """
    The description of the object, this is a combination of links and joints
    """
    pose: Optional[PoseStamped] = field(default_factory=PoseStamped)
    """
    The pose at which this object is placed
    """
    links: Optional[Dict[str, FrozenLink]] = None
    """
    A dictionary with the link name as key and the link object as value
    """
    joints: Optional[Dict[str, FrozenJoint]] = None
    """
    A dictionary of all joints, with the joint name as key and the joint object as value
    """


@dataclass(frozen=True)
class FrozenLink:
    name: str
    """
    Name of this FrozenLink
    """
    pose: PoseStamped
    """
    Pose of this Link in the world frame
    """
    geometry: Union[VisualShape, List[VisualShape]]
    """
    The geometry of this link
    """


@dataclass(frozen=True)
class FrozenJoint:
    name: str
    """
    Name of this FrozenJoints
    """
    type: JointType
    """
    The type of this joint
    """
    children: Sequence[str]
    """
    A sequence of the names of all children
    """
    parent: Optional[str]
    """
    The name of the parent joint or None if there is no parent joint
    """
    state: float
    """
    State of the joint
    """