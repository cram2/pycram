from __future__ import annotations

from dataclasses import dataclass
from typing_extensions import List, Optional, Tuple, Callable, Dict, Any, Union, TYPE_CHECKING
from .enums import JointType, Shape
from .pose import Pose, Point
from abc import ABC, abstractmethod

if TYPE_CHECKING:
    from ..description import Link
    from ..world_concepts.world_object import Object
    from ..world_concepts.constraints import Attachment


def get_point_as_list(point: Point) -> List[float]:
    """
    Returns the point as a list.

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

    @property
    def size(self) -> List[float]:
        return self.half_extents


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
    scale: List[float]
    file_name: str

    def shape_data(self) -> Dict[str, Union[List[float], str]]:
        return {"meshScale": self.scale, "meshFileName": self.file_name}

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
class LinkState(State):
    constraint_ids: Dict[Link, int]


@dataclass
class JointState(State):
    position: float


@dataclass
class ObjectState(State):
    pose: Pose
    attachments: Dict[Object, Attachment]
    link_states: Dict[int, LinkState]
    joint_states: Dict[int, JointState]


@dataclass
class WorldState(State):
    simulator_state_id: int
    object_states: Dict[str, ObjectState]


@dataclass
class LateralFriction:
    lateral_friction: float
    lateral_friction_direction: List[float]


@dataclass
class ContactPoint:
    link_a: Link
    link_b: Link
    position_on_object_a: Optional[List[float]] = None
    position_on_object_b: Optional[List[float]] = None
    normal_on_b: Optional[List[float]] = None  # normal on object b pointing towards object a
    distance: Optional[float] = None
    normal_force: Optional[List[float]] = None  # normal force applied during last step simulation
    lateral_friction_1: Optional[LateralFriction] = None
    lateral_friction_2: Optional[LateralFriction] = None
    force_x_in_world_frame: Optional[float] = None
    force_y_in_world_frame: Optional[float] = None
    force_z_in_world_frame: Optional[float] = None

    def __str__(self):
        return f"ContactPoint: {self.link_a.object.name} - {self.link_b.object.name}"

    def __repr__(self):
        return self.__str__()


ClosestPoint = ContactPoint


class ContactPointsList(list):

    def __index__(self, index: int) -> ContactPoint:
        return super().__getitem__(index)

    def __getitem__(self, item) -> Union[ContactPoint, 'ContactPointsList']:
        if isinstance(item, slice):
            return ContactPointsList(super().__getitem__(item))
        return super().__getitem__(item)

    def get_normals_of_object(self, obj: Object) -> List[List[float]]:
        """
        Gets the normals of the object.
        :param obj: An instance of the Object class that represents the object.
        :return: A list of float vectors that represent the normals of the object.
        """
        return self.get_points_of_object(obj).get_normals()

    def get_normals(self) -> List[List[float]]:
        """
        Gets the normals of the points.
        :return: A list of float vectors that represent the normals of the contact points.
        """
        return [point.normal_on_b for point in self]

    def get_links_in_contact_of_object(self, obj: Object) -> List[Link]:
        """
        Gets the links in contact of the object.
        :param obj: An instance of the Object class that represents the object.
        :return: A list of Link instances that represent the links in contact of the object.
        """
        return [point.link_b for point in self if point.link_b.object == obj]

    def get_points_of_object(self, obj: Object) -> 'ContactPointsList':
        """
        Gets the points of the object.
        :param obj:
        :return:
        """
        return ContactPointsList([point for point in self if point.link_b.object == obj])

    def get_objects_that_got_removed(self, previous_points: 'ContactPointsList') -> List[Object]:
        """
        Returns the object that is not in the current points list but was in the initial points list.
        :param previous_points: The initial points list.
        :return: A list of Object instances that represent the objects that got removed.
        """
        initial_objects_in_contact = previous_points.get_objects_that_have_points()
        current_objects_in_contact = self.get_objects_that_have_points()
        return [obj for obj in initial_objects_in_contact if obj not in current_objects_in_contact]

    def get_new_objects(self, previous_points: 'ContactPointsList') -> List[Object]:
        """
        Returns the object that is not in the initial points list but is in the current points list.
        :param previous_points: The initial points list.
        :return: A list of Object instances that represent the new objects.
        """
        initial_objects_in_contact = previous_points.get_objects_that_have_points()
        current_objects_in_contact = self.get_objects_that_have_points()
        return [obj for obj in current_objects_in_contact if obj not in initial_objects_in_contact]

    def is_object_in_the_list(self, obj: Object) -> bool:
        """
        Checks if the object is one of the objects that have points in the list.
        :param obj: An instance of the Object class that represents the object.
        :return: True if the object is in the list, False otherwise.
        """
        return obj in self.get_objects_that_have_points()

    def get_objects_that_have_points(self) -> List[Object]:
        return [point.link_b.object for point in self]

    def get_names_of_objects_that_have_points(self) -> List[str]:
        return [point.link_b.object.name for point in self]

    def __str__(self):
        return f"ContactPointsList: {', '.join(self.get_names_of_objects_that_have_points())}"

    def __repr__(self):
        return self.__str__()


ClosestPointsList = ContactPointsList


@dataclass
class TextAnnotation:
    text: str
    position: List[float]
    color: Color
    id: int


@dataclass
class VirtualMoveBaseJoints:
    translation_x: str
    translation_y: str
    angular_z: str
