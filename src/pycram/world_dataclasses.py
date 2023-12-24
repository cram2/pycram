from dataclasses import dataclass
from typing import List, Optional, Tuple
from .enums import JointType


@dataclass
class Color:
    """
    Dataclass for storing color as an RGBA value.
    """
    R: float = 1
    G: float = 1
    B: float = 1
    A: float = 1

    @classmethod
    def from_rgba(cls, rgba: List[float]):
        """
        Sets the color from a list of RGBA values.

        :param rgba: The list of RGBA values
        """
        return cls(rgba[0], rgba[1], rgba[2], rgba[3])

    def get_rgba(self) -> List[float]:
        """
        Returns the color as a list of RGBA values.

        :return: The color as a list of RGBA values
        """
        return [self.R, self.G, self.B, self.A]


@dataclass
class Constraint:
    """
    Dataclass for storing a constraint between two objects.
    """
    parent_obj_id: int
    parent_link_name: str
    child_obj_id: int
    child_link_name: str
    joint_type: JointType
    joint_axis_in_child_link_frame: List[int]
    joint_frame_position_wrt_parent_origin: List[float]
    joint_frame_position_wrt_child_origin: List[float]
    joint_frame_orientation_wrt_parent_origin: Optional[List[float]] = None
    joint_frame_orientation_wrt_child_origin: Optional[List[float]] = None


@dataclass
class Point:
    x: float
    y: float
    z: float

    @classmethod
    def from_list(cls, point: List[float]):
        """
        Sets the point from a list of x, y, z values.

        :param point: The list of x, y, z values
        """
        return cls(point[0], point[1], point[2])

@dataclass
class AxisAlignedBoundingBox:
    """
    Dataclass for storing an axis-aligned bounding box.
    """
    min_x: float
    min_y: float
    min_z: float
    max_x: float
    max_y: float
    max_z: float

    @classmethod
    def from_min_max(cls, min_point: List[float], max_point: List[float]):
        """
        Sets the axis-aligned bounding box from a minimum and maximum point.

        :param min_point: The minimum point
        :param max_point: The maximum point
        """
        return cls(min_point[0], min_point[1], min_point[2], max_point[0], max_point[1], max_point[2])

    def get_min_max_points(self) -> Tuple[Point, Point]:
        """
        Returns the axis-aligned bounding box as a tuple of minimum and maximum points.

        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min_point(), self.get_max_point()

    def get_min_point(self) -> Point:
        """
        Returns the axis-aligned bounding box as a minimum point.

        :return: The axis-aligned bounding box as a minimum point
        """
        return Point(self.min_x, self.min_y, self.min_z)

    def get_max_point(self) -> Point:
        """
        Returns the axis-aligned bounding box as a maximum point.

        :return: The axis-aligned bounding box as a maximum point
        """
        return Point(self.max_x, self.max_y, self.max_z)

    def get_min_max(self) -> Tuple[List[float], List[float]]:
        """
        Returns the axis-aligned bounding box as a tuple of minimum and maximum points.

        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min(), self.get_max()

    def get_min(self) -> List[float]:
        """
        Returns the minimum point of the axis-aligned bounding box.

        :return: The minimum point of the axis-aligned bounding box
        """
        return [self.min_x, self.min_y, self.min_z]

    def get_max(self) -> List[float]:
        """
        Returns the maximum point of the axis-aligned bounding box.

        :return: The maximum point of the axis-aligned bounding box
        """
        return [self.max_x, self.max_y, self.max_z]
