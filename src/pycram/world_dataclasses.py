from dataclasses import dataclass
from typing_extensions import List, Optional, Tuple, Callable, Dict, Any, Union
from .enums import JointType, Shape
from .pose import Pose, Point
from abc import ABC, abstractmethod


def get_point_as_list(point: Point) -> List[float]:
    """
    Returns the point as a list.

    :param point: The point
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
        Sets the rgba_color from a list of RGBA values.

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
        Sets the rgba_color from a list of RGB values.

        :param rgb: The list of RGB values
        """
        return cls(rgb[0], rgb[1], rgb[2], 1)

    @classmethod
    def from_rgba(cls, rgba: List[float]):
        """
        Sets the rgba_color from a list of RGBA values.

        :param rgba: The list of RGBA values
        """
        return cls(rgba[0], rgba[1], rgba[2], rgba[3])

    def get_rgba(self) -> List[float]:
        """
        Returns the rgba_color as a list of RGBA values.

        :return: The rgba_color as a list of RGBA values
        """
        return [self.R, self.G, self.B, self.A]

    def get_rgb(self) -> List[float]:
        """
        Returns the rgba_color as a list of RGB values.

        :return: The rgba_color as a list of RGB values
        """
        return [self.R, self.G, self.B]


@dataclass
class Constraint:
    """
    Dataclass for storing a constraint between two objects.
    """
    parent_link: 'Link'
    child_link: 'Link'
    joint_type: JointType
    joint_axis_in_child_link_frame: Point
    joint_frame_pose_wrt_parent_origin: Pose
    joint_frame_pose_wrt_child_origin: Pose

    def get_parent_object_id(self) -> int:
        """
        Returns the id of the parent object of the constraint.

        :return: The id of the parent object of the constraint
        """
        return self.parent_link.object.id

    def get_child_object_id(self) -> int:
        """
        Returns the id of the child object of the constraint.

        :return: The id of the child object of the constraint
        """
        return self.child_link.object.id

    def get_parent_link_id(self) -> int:
        """
        Returns the id of the parent link of the constraint.

        :return: The id of the parent link of the constraint
        """
        return self.parent_link.id

    def get_child_link_id(self) -> int:
        """
        Returns the id of the child link of the constraint.

        :return: The id of the child link of the constraint
        """
        return self.child_link.id

    def get_joint_axis_as_list(self) -> List[float]:
        """
        Returns the joint axis of the constraint as a list.

        :return: The joint axis of the constraint as a list
        """
        return get_point_as_list(self.joint_axis_in_child_link_frame)
    
    def get_joint_position_wrt_parent_as_list(self) -> List[float]:
        """
        Returns the joint frame pose with respect to the parent origin as a list.

        :return: The joint frame pose with respect to the parent origin as a list
        """
        return self.joint_frame_pose_wrt_parent_origin.position_as_list()


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


@dataclass
class CollisionCallbacks:
    on_collision_cb: Callable
    no_collision_cb: Optional[Callable] = None


@dataclass
class MultiBody:
    base_visual_shape_index: int
    base_pose: Pose
    link_visual_shape_indices: List[int]
    link_poses: List[Pose]
    link_masses: List[float]
    link_inertial_frame_poses: List[Pose]
    link_parent_indices: List[int]
    link_joint_types: List[JointType]
    link_joint_axis: List[Point]
    link_collision_shape_indices: List[int]


@dataclass
class VisualShape(ABC):
    rgba_color: Color
    visual_frame_position: List[float]

    @abstractmethod
    def shape_data(self) -> Dict[str, Any]:
        """
        Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere).
        """
        pass

    @property
    @abstractmethod
    def visual_geometry_type(self) -> Shape:
        """
        Returns the visual geometry type of the visual shape (e.g. box, sphere).
        """
        pass


@dataclass
class BoxVisualShape(VisualShape):
    half_extents: List[float]

    def shape_data(self) -> Dict[str, List[float]]:
        return {"halfExtents": self.half_extents}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.BOX


@dataclass
class SphereVisualShape(VisualShape):
    radius: float

    def shape_data(self) -> Dict[str, float]:
        return {"radius": self.radius}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.SPHERE


@dataclass
class CapsuleVisualShape(VisualShape):
    radius: float
    length: float

    def shape_data(self) -> Dict[str, float]:
        return {"radius": self.radius, "length": self.length}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.CAPSULE


@dataclass
class CylinderVisualShape(CapsuleVisualShape):

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.CYLINDER


@dataclass
class MeshVisualShape(VisualShape):
    mesh_scale: List[float]
    mesh_file_name: str

    def shape_data(self) -> Dict[str, Union[List[float], str]]:
        return {"meshScale": self.mesh_scale, "meshFileName": self.mesh_file_name}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.MESH


@dataclass
class PlaneVisualShape(VisualShape):
    normal: List[float]

    def shape_data(self) -> Dict[str, List[float]]:
        return {"normal": self.normal}

    @property
    def visual_geometry_type(self) -> Shape:
        return Shape.PLANE


@dataclass
class State(ABC):
    pass


@dataclass
class WorldState(State):
    state_id: int


@dataclass
class ObjectState(State):
    state_id: int
    attachments: Dict['Object', 'Attachment']


@dataclass
class LinkState(State):
    constraint_ids: Dict['Link', int]


@dataclass
class JointState(State):
    position: float
