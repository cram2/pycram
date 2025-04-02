from __future__ import annotations

import itertools
import math
from abc import ABC, abstractmethod
from copy import deepcopy, copy
from dataclasses import dataclass, fields, field
from enum import Enum

import numpy as np
import plotly.graph_objects as go
import sqlalchemy
import trimesh
from matplotlib import pyplot as plt
from std_msgs.msg import ColorRGBA

from random_events.interval import closed, SimpleInterval, Bound
from random_events.product_algebra import SimpleEvent, Event
from random_events.variable import Continuous
from typing_extensions import List, Optional, Tuple, Callable, Dict, Any, Union, TYPE_CHECKING, Sequence, Self, \
    deprecated, Type

from pycrap.ontologies import PhysicalObject
from .enums import JointType, Shape, VirtualMobileBaseJointName, Grasp, AxisIdentifier
from .pose import PoseStamped, Point, TransformStamped
from ..orm.base import ProcessMetaData
from ..ros import logwarn, logwarn_once
from ..utils import classproperty
from ..validation.error_checkers import calculate_joint_position_error, is_error_acceptable
from ..orm.object_designator import Object as ORMObject

if TYPE_CHECKING:
    from ..description import Link, ObjectDescription
    from ..world_concepts.world_object import Object
    from ..world_concepts.constraints import Attachment
    from .world_entity import PhysicalBody
    from .world import World


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
class BoundingBox:
    """
    Dataclass for storing an axis-aligned bounding box.

    An axis aligned bounding box is the cartesian product of the three closed intervals
    [min_x, max_x] x [min_y, max_y] x [min_z, max_z].

    Depth is the distance between the min_x and max_x, and should always be the long side (excluding height).
    Width is the distance between the min_y and max_y, and should always be the short side (excluding height).
    Height is the distance between the min_z and max_z, "up" is according to how the object would stand on a table.

    Set-Algebraic operations are possible by converting the bounding box to a random event.
    """

    min_x: float
    """
    The minimum x-coordinate of the bounding box.
    """

    min_y: float
    """
    The minimum y-coordinate of the bounding box.
    """

    min_z: float
    """
    The minimum z-coordinate of the bounding box.
    """

    max_x: float
    """
    The maximum x-coordinate of the bounding box.
    """

    max_y: float
    """
    The maximum y-coordinate of the bounding box.
    """

    max_z: float
    """
    The maximum z-coordinate of the bounding box.
    """

    x_variable = Continuous("x")
    """
    The x variable for the random-events interface.
    """

    y_variable = Continuous("y")
    """
    The y variable for the random-events interface.
    """

    z_variable = Continuous("z")
    """
    The z variable for the random-events interface.
    """

    def __hash__(self):
        # The hash should be this since comparing those via hash is checking if those are the same and not just equal
        return id(self)

    @property
    def x_interval(self) -> SimpleInterval:
        """
        :return: The x interval of the bounding box.
        """
        return SimpleInterval(self.min_x, self.max_x, Bound.CLOSED, Bound.CLOSED)

    @property
    def y_interval(self) -> SimpleInterval:
        """
        :return: The y interval of the bounding box.
        """
        return SimpleInterval(self.min_y, self.max_y, Bound.CLOSED, Bound.CLOSED)

    @property
    def z_interval(self) -> SimpleInterval:
        """
        :return: The z interval of the bounding box.
        """
        return SimpleInterval(self.min_z, self.max_z, Bound.CLOSED, Bound.CLOSED)

    @property
    def simple_event(self) -> SimpleEvent:
        """
        :return: The bounding box as a random event.
        """
        return SimpleEvent({self.x_variable: self.x_interval,
                            self.y_variable: self.y_interval,
                            self.z_variable: self.z_interval})

    @classmethod
    def from_simple_event(cls, simple_event: SimpleEvent):
        """
        Create a list of bounding boxes from a simple random event.

        :param simple_event: The random event.
        :return: The list of bounding boxes.
        """
        result = []
        for x, y, z in itertools.product(simple_event[cls.x_variable].simple_sets,
                                         simple_event[cls.y_variable].simple_sets,
                                         simple_event[cls.z_variable].simple_sets):
            result.append(cls(x.lower, y.lower, z.lower, x.upper, y.upper, z.upper))
        return result

    @classmethod
    def from_event(cls, event: Event) -> List[Self]:
        """
        Create a list of bounding boxes from a random event.

        :param event: The random event.
        :return: The list of bounding boxes.
        """
        return [box for simple_event in event.simple_sets for box in cls.from_simple_event(simple_event)]

    def intersection_with(self, other: BoundingBox) -> Optional[BoundingBox]:
        """
        Compute the intersection of two bounding boxes.

        :param other: The other bounding box.
        :return: The intersection of the two bounding boxes or None if they do not intersect.
        """
        result = self.simple_event.intersection_with(other.simple_event)
        if result.is_empty():
            return None
        return self.__class__.from_simple_event(result)[0]

    def contains(self, x: float, y: float, z: float):
        """
        Check if the bounding box contains a point.

        :param x: The x-coordinate of the point.
        :param y: The y-coordinate of the point.
        :param z: The z-coordinate of the point.
        :return: True if the bounding box contains the point, False otherwise.
        """
        return self.simple_event.contains((x, y, z))

    def contains_box(self, other: BoundingBox):
        """
        Check if the bounding box contains another bounding box.

        :param other: The other bounding box.
        :return: True if the bounding box contains the other bounding box, False otherwise.
        """
        return (other.simple_event.as_composite_set() - self.simple_event.as_composite_set()).is_empty()

    @classmethod
    def merge_multiple_bounding_boxes_into_mesh(cls, bounding_boxes: List[BoundingBox],
                                                save_mesh_to: Optional[str] = None,
                                                use_random_events: bool = True,
                                                plot: bool = False) -> trimesh.Trimesh:
        """
        Merge multiple axis-aligned bounding boxes into a single mesh.

        :param bounding_boxes: The list of axis-aligned bounding boxes.
        :param save_mesh_to: The file path to save the mesh to.
        :param use_random_events: If True, use random events to compute the new shape, otherwise use pyvista.
        :param plot: If True, plot the mesh.
        :return: The mesh of the merged bounding boxes.
        """
        if use_random_events:

            all_intervals = [(box.get_min(), box.get_max()) for box in bounding_boxes]
            event = None
            for min_point, max_point in all_intervals:
                new_event = SimpleEvent({cls.x_variable: closed(min_point[0], max_point[0]),
                                         cls.y_variable: closed(min_point[1], max_point[1]),
                                         cls.z_variable: closed(min_point[2], max_point[2])}).as_composite_set()
                # TODO fix this when random events is fixed.
                if event:
                    event = event.__deepcopy__().union_with(new_event)
                else:
                    event = new_event
            if plot:
                fig = go.Figure(event.plot(), event.plotly_layout())
                fig.update_layout(title="Merged Bounding Boxes")
                fig.show()
            mesh = BoundingBox.get_mesh_from_event(event)
        else:
            mesh = BoundingBox.get_mesh_from_boxes(bounding_boxes)
        if plot:
            mesh.show()
            BoundingBox.plot_3d_points([mesh.vertices])
        if save_mesh_to is not None:
            mesh.export(save_mesh_to)
        return mesh

    @property
    @abstractmethod
    def transform(self) -> TransformStamped:
        """
        Get the transformation of the bounding box.
        """
        pass

    def extents(self) -> np.ndarray:
        """
        :return: The size of the bounding box in each dimension.
        """
        return np.array([self.depth, self.width, self.height])

    @staticmethod
    def get_mesh_from_boxes(boxes: List[BoundingBox]) -> trimesh.Trimesh:
        """
        Get the mesh from the boxes

        :param boxes: The list of boxes
        :return: The mesh.
        """
        first_box_mesh = boxes[0].as_mesh
        if len(boxes) == 1:
            return first_box_mesh
        else:
            return first_box_mesh.union([box.as_mesh for box in boxes[1:]]).convex_hull

    @property
    def as_mesh(self) -> trimesh.Trimesh:
        """
        :return: The mesh of the bounding box.
        """
        return trimesh.primitives.Box(self.extents(), self.transform.transform.to_matrix())

    @staticmethod
    def get_mesh_from_event(event: Event) -> trimesh.Trimesh:
        """
        Get the mesh from the event.

        :param event: The event.
        :return: The mesh.
        """
        # form cartesian product of all intervals
        intervals = [value.simple_sets for simple_event in event.simple_sets for _, value in simple_event.items()]
        simple_events = list(itertools.product(*intervals))

        # for every atomic interval
        all_vertices = []
        all_faces = []
        for i, simple_event in enumerate(simple_events):
            x, y, z = 0, 1, 2
            for j in range(2):
                x, y, z = x + j * 3, y + j * 3, z + j * 3
                # Create a 3D mesh trace for the rectangle
                all_vertices.extend([[simple_event[x].lower, simple_event[y].lower, simple_event[z].lower],
                                     [simple_event[x].lower, simple_event[y].lower, simple_event[z].upper],
                                     [simple_event[x].lower, simple_event[y].upper, simple_event[z].lower],
                                     [simple_event[x].lower, simple_event[y].upper, simple_event[z].upper],
                                     [simple_event[x].upper, simple_event[y].lower, simple_event[z].lower],
                                     [simple_event[x].upper, simple_event[y].lower, simple_event[z].upper],
                                     [simple_event[x].upper, simple_event[y].upper, simple_event[z].lower],
                                     [simple_event[x].upper, simple_event[y].upper, simple_event[z].upper]])
                all_faces.extend((np.array(BoundingBox.get_box_faces()) + i * 16 + j * 8).tolist())
        return trimesh.Trimesh(np.array(all_vertices), np.array(all_faces))

    @staticmethod
    def get_box_faces() -> List[List[int]]:
        return [[0, 1, 2], [2, 3, 1], [4, 5, 6], [6, 7, 5],
                [0, 1, 4], [4, 5, 1], [2, 3, 6], [6, 7, 2],
                [0, 2, 4], [4, 6, 2], [1, 3, 5], [5, 7, 3]]

    @classmethod
    def from_min_max(cls, min_point: Sequence[float], max_point: Sequence[float]):
        """
        Set the axis-aligned bounding box from a minimum and maximum point.

        :param min_point: The minimum point
        :param max_point: The maximum point
        """
        return cls(min_point[0], min_point[1], min_point[2], max_point[0], max_point[1], max_point[2])

    @property
    def origin(self) -> List[float]:
        return [(self.min_x + self.max_x) / 2, (self.min_y + self.max_y) / 2, (self.min_z + self.max_z) / 2]

    @property
    def base_origin(self) -> List[float]:
        center = self.origin
        return [center[0], center[1], self.min_z]

    @property
    def origin_point(self) -> Point:
        return Point(**dict(zip(["x", "y", "z"], self.origin)))

    def get_points_list(self) -> List[List[float]]:
        """
        :return: The points of the bounding box as a list of lists of floats.
        """
        return [[point.x, point.y, point.z] for point in self.get_points()]

    def get_points(self) -> List[Point]:
        """
        :return: The points of the bounding box as a list of Point instances.
        """
        return [Point(x=self.min_x, y=self.min_y, z=self.min_z),
                Point(x=self.min_x, y=self.min_y, z=self.max_z),
                Point(x=self.min_x, y=self.max_y, z=self.min_z),
                Point(x=self.min_x, y=self.max_y, z=self.max_z),
                Point(x=self.max_x, y=self.min_y, z=self.min_z),
                Point(x=self.max_x, y=self.min_y, z=self.max_z),
                Point(x=self.max_x, y=self.max_y, z=self.min_z),
                Point(x=self.max_x, y=self.max_y, z=self.max_z)]

    def get_min_max_points(self) -> Tuple[Point, Point]:
        """
        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min_point(), self.get_max_point()

    def get_min_point(self) -> Point:
        """
        :return: The axis-aligned bounding box as a minimum point
        """
        return Point(x=self.min_x, y=self.min_y, z=self.min_z)

    def get_max_point(self) -> Point:
        """
        :return: The axis-aligned bounding box as a maximum point
        """
        return Point(x=self.max_x, y=self.max_y, z=self.max_z)

    def get_min_max(self) -> Tuple[List[float], List[float]]:
        """
        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min(), self.get_max()

    def get_min(self) -> List[float]:
        """
        :return: The minimum point of the axis-aligned bounding box
        """
        return [self.min_x, self.min_y, self.min_z]

    def get_max(self) -> List[float]:
        """
        :return: The maximum point of the axis-aligned bounding box
        """
        return [self.max_x, self.max_y, self.max_z]

    def enlarge(self, min_x: float = 0., min_y: float = 0, min_z: float = 0,
                max_x: float = 0., max_y: float = 0., max_z: float = 0.):
        """
        Enlarge the axis-aligned bounding box by a given amount in-place.
        :param min_x: The amount to enlarge the minimum x-coordinate
        :param min_y: The amount to enlarge the minimum y-coordinate
        :param min_z: The amount to enlarge the minimum z-coordinate
        :param max_x: The amount to enlarge the maximum x-coordinate
        :param max_y: The amount to enlarge the maximum y-coordinate
        :param max_z: The amount to enlarge the maximum z-coordinate
        """
        self.min_x -= min_x
        self.min_y -= min_y
        self.min_z -= min_z
        self.max_x += max_x
        self.max_y += max_y
        self.max_z += max_z

    def enlarge_all(self, amount: float):
        """
        Enlarge the axis-aligned bounding box in all dimensions by a given amount in-place.

        :param amount: The amount to enlarge the bounding box
        """
        self.enlarge(amount, amount, amount,
                     amount, amount, amount)

    @property
    def depth(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_z - self.min_z

    @property
    def width(self) -> float:
        return self.max_y - self.min_y

    @property
    def dimensions(self) -> List[float]:
        """
        According to the IAI conventions, found at https://ai.uni-bremen.de/wiki/3dmodeling/items
        1. z is height, according to how the object would stand on a table
        2. x is depth, representing the long side of the object
        3. y is width, representing the remaining dimension
        """
        if self.width > self.depth:
            logwarn_once("The width of the bounding box is greater than the depth. This means the object's"
                         "axis alignment is potentially going against IAI conventions.")
        return [self.depth, self.width, self.height]

    @staticmethod
    def plot_3d_points(list_of_points: List[np.ndarray]):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        for points in list_of_points:
            color = np.random.rand(3, )
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=color, marker='o')

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.xlim(0, 2)
        plt.ylim(0, 2)
        ax.set_zlim(0, 2)

        plt.show()


@dataclass
class AxisAlignedBoundingBox(BoundingBox):

    @property
    def transform(self) -> TransformStamped:
        return TransformStamped.from_list(self.origin)

    def get_rotated_box(self, transform: TransformStamped) -> RotatedBoundingBox:
        """
        Apply a transformation to the axis-aligned bounding box and return the transformed axis-aligned bounding box.

        :return: The transformed axis-aligned bounding box
        """
        return RotatedBoundingBox.from_min_max(self.get_min(), self.get_max(), transform)

    @classmethod
    def from_origin_and_half_extents(cls, origin: Point, half_extents: Point):
        """
        Set the axis-aligned bounding box from the origin of the body and half the size.

        :param origin: The origin point
        :param half_extents: The half size of the bounding box.
        """
        min_point = [origin.x - half_extents.x, origin.y - half_extents.y, origin.z - half_extents.z]
        max_point = [origin.x + half_extents.x, origin.y + half_extents.y, origin.z + half_extents.z]
        return cls.from_min_max(min_point, max_point)

    @classmethod
    def from_multiple_bounding_boxes(cls, bounding_boxes: List[AxisAlignedBoundingBox]) -> AxisAlignedBoundingBox:
        """
        Set the axis-aligned bounding box from multiple axis-aligned bounding boxes.

        :param bounding_boxes: The list of axis-aligned bounding boxes.
        """
        min_x = min([box.min_x for box in bounding_boxes])
        min_y = min([box.min_y for box in bounding_boxes])
        min_z = min([box.min_z for box in bounding_boxes])
        max_x = max([box.max_x for box in bounding_boxes])
        max_y = max([box.max_y for box in bounding_boxes])
        max_z = max([box.max_z for box in bounding_boxes])
        return cls(min_x, min_y, min_z, max_x, max_y, max_z)

    def shift_by(self, shift: Point) -> AxisAlignedBoundingBox:
        """
        Shift the axis-aligned bounding box by a given shift.

        :param shift: The shift to apply
        :return: The shifted axis-aligned bounding box
        """
        return AxisAlignedBoundingBox(self.min_x + shift.x, self.min_y + shift.y, self.min_z + shift.z,
                                      self.max_x + shift.x, self.max_y + shift.y, self.max_z + shift.z)


@dataclass
class RotatedBoundingBox(BoundingBox):
    """
    Dataclass for storing a rotated bounding box.
    """

    def __init__(self, min_x: float, min_y: float, min_z: float, max_x: float, max_y: float, max_z: float,
                 transform: Optional[TransformStamped] = None, points: Optional[List[Point]] = None):
        """
        Set the rotated bounding box from a minimum and maximum point.
        :param transform: The transformation
        :param points: The points of the rotated bounding box.
        """
        self._transform: Optional[TransformStamped] = transform
        super().__init__(min_x, min_y, min_z, max_x, max_y, max_z)
        self._points: Optional[List[Point]] = points

    @property
    def transform(self) -> TransformStamped:
        return self._transform

    @classmethod
    def from_min_max(cls, min_point: Sequence[float], max_point: Sequence[float],
                     transform: Optional[TransformStamped] = None):
        """
        Set the rotated bounding box from a minimum, maximum point, and a transformation.

        :param min_point: The minimum point
        :param max_point: The maximum point
        :param transform: The transformation
        """
        return cls(min_point[0], min_point[1], min_point[2], max_point[0], max_point[1], max_point[2], transform)

    def get_points(self) -> List[Point]:
        """
        :return: The points of the rotated bounding box.
        """
        points_array = np.array([[point.x, point.y, point.z] for point in super().get_points()])
        if self._points is None:
            transformed_points = self.transform.apply_transform_to_array_of_points(points_array).tolist()
            self._points = [Point(**dict(zip(["x", "y", "z"], point))) for point in transformed_points]
        return self._points


@dataclass
class CollisionCallbacks:
    """
    Dataclass for storing the collision callbacks which are callables that get called when there is a collision
    or when a collision is no longer there.
    """
    on_collision_cb: Callable
    no_collision_cb: Optional[Callable] = None


@dataclass
class MultiBody:
    """
    Dataclass for storing the information of a multibody which consists of a base and multiple links with joints.
    """
    base_visual_shape_index: int
    base_pose: PoseStamped
    link_visual_shape_indices: List[int]
    link_poses: List[PoseStamped]
    link_masses: List[float]
    link_inertial_frame_poses: List[PoseStamped]
    link_parent_indices: List[int]
    link_joint_types: List[JointType]
    link_joint_axis: List[Point]
    link_collision_shape_indices: List[int]


@dataclass
class VisualShape(ABC):
    """
    Abstract dataclass for storing the information of a visual shape.
    """
    rgba_color: Color
    visual_frame_position: List[float]

    @abstractmethod
    def shape_data(self) -> Dict[str, Any]:
        """
        :return: the shape data of the visual shape (e.g. half extents for a box, radius for a sphere) as a dictionary.
        """
        pass

    @property
    @abstractmethod
    def visual_geometry_type(self) -> Shape:
        """
        :return: The visual geometry type of the visual shape (e.g. box, sphere) as a Shape object.
        """
        pass

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis-aligned bounding box of the visual shape.
        """
        raise NotImplementedError


@dataclass
class BoxVisualShape(VisualShape):
    """
    Dataclass for storing the information of a box visual shape
    """
    half_extents: List[float]

    def shape_data(self) -> Dict[str, List[float]]:
        return {"halfExtents": self.half_extents}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.BOX

    @property
    def size(self) -> List[float]:
        return self.half_extents

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis-aligned bounding box of the box visual shape.
        """
        return AxisAlignedBoundingBox(-self.half_extents[0], -self.half_extents[1], -self.half_extents[2],
                                      self.half_extents[0], self.half_extents[1], self.half_extents[2])


@dataclass
class SphereVisualShape(VisualShape):
    """
    Dataclass for storing the information of a sphere visual shape
    """
    radius: float

    def shape_data(self) -> Dict[str, float]:
        return {"radius": self.radius}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.SPHERE

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis-aligned bounding box of the sphere visual shape.
        """
        return AxisAlignedBoundingBox(-self.radius, -self.radius, -self.radius, self.radius, self.radius, self.radius)


@dataclass
class CapsuleVisualShape(VisualShape):
    """
    Dataclass for storing the information of a capsule visual shape
    """
    radius: float
    length: float

    def shape_data(self) -> Dict[str, float]:
        return {"radius": self.radius, "length": self.length}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.CAPSULE

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis-aligned bounding box of the capsule visual shape.
        """
        return AxisAlignedBoundingBox(-self.radius, -self.radius, -self.length / 2,
                                      self.radius, self.radius, self.length / 2)


@dataclass
class CylinderVisualShape(CapsuleVisualShape):
    """
    Dataclass for storing the information of a cylinder visual shape
    """

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.CYLINDER


@dataclass
class MeshVisualShape(VisualShape):
    """
    Dataclass for storing the information of a mesh visual shape
    """
    scale: List[float]
    file_name: str

    def shape_data(self) -> Dict[str, Union[List[float], str]]:
        return {"meshScale": self.scale, "meshFileName": self.file_name}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.MESH

    def get_axis_aligned_bounding_box(self, file_path: Optional[str] = None) -> AxisAlignedBoundingBox:
        """
        :param file_path: An alternative file path.
        :return: The axis-aligned bounding box of the mesh visual shape.
        """
        mesh_file_path = file_path if file_path is not None else self.file_name
        mesh = trimesh.load(mesh_file_path)
        min_bound, max_bound = mesh.bounds
        return AxisAlignedBoundingBox.from_min_max(min_bound, max_bound)


@dataclass
class PlaneVisualShape(VisualShape):
    """
    Dataclass for storing the information of a plane visual shape
    """
    normal: List[float]

    def shape_data(self) -> Dict[str, List[float]]:
        return {"normal": self.normal}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.PLANE


VisualShapeUnion = Union[BoxVisualShape, SphereVisualShape, CapsuleVisualShape,
CylinderVisualShape, MeshVisualShape, PlaneVisualShape]


@dataclass
class State(ABC):
    """
    Abstract dataclass for storing the state of an entity (e.g. world, object, link, joint).
    """
    pass


@dataclass
class PhysicalBodyState(State):
    """
    Dataclass for storing the state of a physical body.
    """
    pose: PoseStamped
    is_translating: bool
    is_rotating: bool
    velocity: List[float]
    acceptable_pose_error: Tuple[float, float] = (0.001, 0.001)
    acceptable_velocity_error: Tuple[float, float] = (0.001, 0.001)

    def __eq__(self, other: PhysicalBodyState):
        return (self.pose_is_almost_equal(other)
                and self.is_translating == other.is_translating
                and self.is_rotating == other.is_rotating
                and self.velocity_is_almost_equal(other)
                )

    def pose_is_almost_equal(self, other: PhysicalBodyState) -> bool:
        """
        Check if the pose of the object is almost equal to the pose of another object.

        :param other: The state of the other object.
        :return: True if the poses are almost equal, False otherwise.
        """
        return self.pose.almost_equal(other.pose, other.acceptable_pose_error[0], other.acceptable_pose_error[1])

    def velocity_is_almost_equal(self, other: PhysicalBodyState) -> bool:
        """
        Check if the velocity of the object is almost equal to the velocity of another object.

        :param other: The state of the other object.
        :return: True if the velocities are almost equal, False otherwise.
        """
        if self.velocity is None or other.velocity is None:
            return self.velocity == other.velocity
        return (self.vector_is_almost_equal(self.velocity[:3], other.velocity[:3], self.acceptable_velocity_error[0])
                and self.vector_is_almost_equal(self.velocity[3:], other.velocity[3:],
                                                self.acceptable_velocity_error[1]))

    @staticmethod
    def vector_is_almost_equal(vector1: List[float], vector2: List[float], acceptable_error: float) -> bool:
        """
        Check if the vector is almost equal to another vector.

        :param vector1: The first vector.
        :param vector2: The second vector.
        :param acceptable_error: The acceptable error.
        :return: True if the vectors are almost equal, False otherwise.
        """
        return np.all(np.array(vector1) - np.array(vector2) <= acceptable_error)

    def __copy__(self):
        return PhysicalBodyState(pose=self.pose.copy(),
                                 is_translating=self.is_translating, is_rotating=self.is_rotating,
                                 velocity=self.velocity.copy(),
                                 acceptable_pose_error=deepcopy(self.acceptable_pose_error),
                                 acceptable_velocity_error=deepcopy(self.acceptable_velocity_error))


@dataclass
class LinkState(State):
    """
    Dataclass for storing the state of a link.
    """
    body_state: PhysicalBodyState
    constraint_ids: Dict[Link, int]

    def __eq__(self, other: LinkState):
        return (self.body_state == other.body_state
                and self.all_constraints_exist(other)
                and self.all_constraints_are_equal(other))

    def all_constraints_exist(self, other: LinkState) -> bool:
        """
        Check if all constraints exist in the other link state.

        :param other: The state of the other link.
        :return: True if all constraints exist, False otherwise.
        """
        return (all([cid_k in other.constraint_ids.keys() for cid_k in self.constraint_ids.keys()])
                and len(self.constraint_ids.keys()) == len(other.constraint_ids.keys()))

    def all_constraints_are_equal(self, other: LinkState) -> bool:
        """
        Check if all constraints are equal to the ones in the other link state.

        :param other: The state of the other link.
        :return: True if all constraints are equal, False otherwise.
        """
        return all([cid == other_cid for cid, other_cid in zip(self.constraint_ids.values(),
                                                               other.constraint_ids.values())])

    def __copy__(self):
        return LinkState(copy(self.body_state), constraint_ids=copy(self.constraint_ids))


@dataclass
class JointState(State):
    """
    Dataclass for storing the state of a joint.
    """
    position: float
    acceptable_error: float

    def __eq__(self, other: JointState):
        error = calculate_joint_position_error(self.position, other.position)
        return is_error_acceptable(error, other.acceptable_error)

    def __copy__(self):
        return JointState(position=self.position, acceptable_error=self.acceptable_error)


@dataclass
class ObjectState(State):
    """
    Dataclass for storing the state of an object.
    """
    body_state: PhysicalBodyState
    attachments: Dict[Object, Attachment]
    link_states: Dict[int, LinkState]
    joint_states: Dict[int, JointState]

    def __eq__(self, other: ObjectState):
        return (self.body_state == other.body_state
                and self.all_attachments_exist(other) and self.all_attachments_are_equal(other)
                and self.link_states == other.link_states
                and self.joint_states == other.joint_states)

    @property
    def pose(self) -> PoseStamped:
        return self.body_state.pose

    def all_attachments_exist(self, other: ObjectState) -> bool:
        """
        Check if all attachments exist in the other object state.

        :param other: The state of the other object.
        :return: True if all attachments exist, False otherwise.
        """
        return (all([att_k in other.attachments.keys() for att_k in self.attachments.keys()])
                and len(self.attachments.keys()) == len(other.attachments.keys()))

    def all_attachments_are_equal(self, other: ObjectState) -> bool:
        """
        Check if all attachments are equal to the ones in the other object state.

        :param other: The state of the other object.
        :return: True if all attachments are equal, False otherwise
        """
        return all([att == other_att for att, other_att in zip(self.attachments.values(), other.attachments.values())])

    def __copy__(self):
        return ObjectState(body_state=self.body_state.copy(), attachments=copy(self.attachments),
                           link_states=copy(self.link_states),
                           joint_states=copy(self.joint_states))


@dataclass
class WorldState(State):
    """
    Dataclass for storing the state of the world.
    """
    object_states: Dict[str, ObjectState]
    simulator_state_id: Optional[int] = None

    def __eq__(self, other: WorldState):
        return (self.simulator_state_is_equal(other) and self.all_objects_exist(other)
                and self.all_objects_states_are_equal(other))

    def simulator_state_is_equal(self, other: WorldState) -> bool:
        """
        Check if the simulator state is equal to the simulator state of another world state.

        :param other: The state of the other world.
        :return: True if the simulator states are equal, False otherwise.
        """
        return self.simulator_state_id == other.simulator_state_id

    def all_objects_exist(self, other: WorldState) -> bool:
        """
        Check if all objects exist in the other world state.

        :param other: The state of the other world.
        :return: True if all objects exist, False otherwise.
        """
        return (all([obj_name in other.object_states.keys() for obj_name in self.object_states.keys()])
                and len(self.object_states.keys()) == len(other.object_states.keys()))

    def all_objects_states_are_equal(self, other: WorldState) -> bool:
        """
        Check if all object states are equal to the ones in the other world state.

        :param other: The state of the other world.
        :return: True if all object states are equal, False otherwise.
        """
        return all([obj_state == other_obj_state
                    for obj_state, other_obj_state in zip(self.object_states.values(),
                                                          other.object_states.values())])

    def __copy__(self):
        return WorldState(object_states=deepcopy(self.object_states),
                          simulator_state_id=self.simulator_state_id
                          )


@dataclass
class LateralFriction:
    """
    Dataclass for storing the information of the lateral friction.
    """
    lateral_friction: float
    lateral_friction_direction: List[float]


@dataclass
class ContactPoint:
    """
    Dataclass for storing the information of a contact point between two bodies.
    """
    body_a: PhysicalBody
    body_b: PhysicalBody
    position_on_body_a: Optional[List[float]] = None
    position_on_body_b: Optional[List[float]] = None
    normal_on_body_b: Optional[List[float]] = None  # the contact normal vector on object b pointing towards object a
    distance: Optional[float] = None  # distance between the two objects (+ve for separation, -ve for penetration)
    normal_force: Optional[float] = None  # normal force applied during last step simulation
    lateral_friction_1: Optional[LateralFriction] = None
    lateral_friction_2: Optional[LateralFriction] = None

    @property
    def normal(self) -> List[float]:
        return self.normal_on_body_b

    @property
    def bodies(self) -> Tuple[PhysicalBody, PhysicalBody]:
        return self.body_a, self.body_b

    def __str__(self):
        return f"ContactPoint: {self.body_a.name} - {self.body_b.name}"

    def __repr__(self):
        return self.__str__()


ClosestPoint = ContactPoint
"""
The closest point between two objects which has the same structure as ContactPoint.
"""


class ContactPointsList(list):
    """
    A list of contact points.
    """

    def get_bodies_that_got_removed(self, previous_points: ContactPointsList) -> List[PhysicalBody]:
        """
        Return the bodies that are not in the current points list but were in the initial points list.

        :param previous_points: The initial points list.
        :return: A list of bodies that got removed.
        """
        initial_bodies_in_contact = previous_points.get_all_bodies()
        current_bodies_in_contact = self.get_all_bodies()
        return [body for body in initial_bodies_in_contact if body not in current_bodies_in_contact]

    def get_all_bodies(self, excluded: List[PhysicalBody] = None) -> List[PhysicalBody]:
        """
        :return: A list of all involved bodies in the points.
        """
        excluded = excluded if excluded is not None else []
        return list(set([body for point in self for body in point.bodies if body not in excluded]))

    def check_if_two_objects_are_in_contact(self, obj_a: Object, obj_b: Object) -> bool:
        """
        Check if two objects are in contact.

        :param obj_a: An instance of the Object class that represents the first object.
        :param obj_b: An instance of the Object class that represents the second object.
        :return: True if the objects are in contact, False otherwise.
        """
        return (any([self.is_body_in_object(point.body_b, obj_b)
                     and self.is_body_in_object(point.body_a, obj_a) for point in self]) or
                any([self.is_body_in_object(point.body_a, obj_b)
                     and self.is_body_in_object(point.body_b, obj_a) for point in self]))

    @staticmethod
    def is_body_in_object(body: PhysicalBody, obj: Object) -> bool:
        """
        Check if the body belongs to the object.

        :param body: The body.
        :param obj: The object.
        :return: True if the body belongs to the object, False otherwise.
        """
        return body in list(obj.links.values()) or body == obj

    def get_normals_of_object(self, obj: Object) -> List[List[float]]:
        """
        Get the normals of the object.

        :param obj: An instance of the Object class that represents the object.
        :return: A list of float vectors that represent the normals of the object.
        """
        return self.get_points_of_object(obj).get_normals()

    def get_normals(self) -> List[List[float]]:
        """
        Get the normals of the points.

        :return: A list of float vectors that represent the normals of the contact points.
        """
        return [point.normal_on_body_b for point in self]

    def get_links_in_contact_of_object(self, obj: Object) -> List[PhysicalBody]:
        """
        Get the links in contact of the object.

        :param obj: An instance of the Object class that represents the object.
        :return: A list of Link instances that represent the links in contact of the object.
        """
        return [point.body_b for point in self if point.body_b.parent_entity == obj]

    def get_points_of_object(self, obj: Object) -> ContactPointsList:
        """
        Get the points of the object.

        :param obj: An instance of the Object class that represents the object that the points are related to.
        :return: A ContactPointsList instance that represents the contact points of the object.
        """
        return ContactPointsList([point for point in self if self.is_body_in_object(point.body_b, obj)])

    def get_points_of_link(self, link: Link) -> ContactPointsList:
        """
        Get the points of the link.

        :param link: An instance of the Link class that represents the link that the points are related to.
        :return: A ContactPointsList instance that represents the contact points of the link.
        """
        return self.get_points_of_body(link)

    def get_points_of_body(self, body: PhysicalBody) -> ContactPointsList:
        """
        Get the points of the body.

        :param body: An instance of the PhysicalBody class that represents the body that the points are related to.
        :return: A ContactPointsList instance that represents the contact points of the body.
        """
        return ContactPointsList([point for point in self if body == point.body_b])

    def get_objects_that_got_removed(self, previous_points: ContactPointsList) -> List[Object]:
        """
        Return the object that is not in the current points list but was in the initial points list.

        :param previous_points: The initial points list.
        :return: A list of Object instances that represent the objects that got removed.
        """
        initial_objects_in_contact = previous_points.get_objects_that_have_points()
        current_objects_in_contact = self.get_objects_that_have_points()
        return [obj for obj in initial_objects_in_contact if obj not in current_objects_in_contact]

    def get_new_objects(self, previous_points: ContactPointsList) -> List[Object]:
        """
        Return the object that is not in the initial points list but is in the current points list.

        :param previous_points: The initial points list.
        :return: A list of Object instances that represent the new objects.
        """
        initial_objects_in_contact = previous_points.get_objects_that_have_points()
        current_objects_in_contact = self.get_objects_that_have_points()
        return [obj for obj in current_objects_in_contact if obj not in initial_objects_in_contact]

    def is_object_in_the_list(self, obj: Object) -> bool:
        """
        Check if the object is one of the objects that have points in the list.

        :param obj: An instance of the Object class that represents the object.
        :return: True if the object is in the list, False otherwise.
        """
        return obj in self.get_objects_that_have_points()

    def get_names_of_objects_that_have_points(self) -> List[str]:
        """
        Return the names of the objects that have points in the list.

        :return: A list of strings that represent the names of the objects that have points in the list.
        """
        return [obj.name for obj in self.get_objects_that_have_points()]

    def get_objects_that_have_points(self) -> List[Object]:
        """
        Return the objects that have points in the list.

        :return: A list of Object instances that represent the objects that have points in the list.
        """
        return list({point.body_b.parent_entity for point in self})

    def __str__(self):
        return f"ContactPointsList: {', '.join([point.__str__() for point in self])}"

    def __repr__(self):
        return self.__str__()


ClosestPointsList = ContactPointsList
"""
The list of closest points which has same structure as ContactPointsList.
"""


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
        Grasp.FRONT: [0, 0, 0, 1],
        Grasp.BACK: [0, 0, 1, 0],
        Grasp.LEFT: [0, 0, -math.sqrt(2) / 2, math.sqrt(2) / 2],
        Grasp.RIGHT: [0, 0, math.sqrt(2) / 2, math.sqrt(2) / 2],
    }

    VERTICAL_ROTATIONS = {
        None: [0, 0, 0, 1],
        Grasp.TOP: [0, math.sqrt(2) / 2, 0, math.sqrt(2) / 2],
        Grasp.BOTTOM: [0, -math.sqrt(2) / 2, 0, math.sqrt(2) / 2],
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
        if not self.obj_id:
            logwarn("obj_id should be available to check if the ray intersects with a body,"
                    "It appears that the ray result is not valid.")
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

    def to_sql(self):
        return ORMObject(obj_type=str(self.concept), name=self.name)

    def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObject:
        metadata = ProcessMetaData().insert(session)
        obj = self.to_sql()
        pose = self.pose.insert(session)
        obj.pose = pose
        obj.process_metadata = metadata
        session.add(obj)

        return obj

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