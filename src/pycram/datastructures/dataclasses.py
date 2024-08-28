from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from typing_extensions import List, Optional, Tuple, Callable, Dict, Any, Union, TYPE_CHECKING

from .enums import JointType, Shape, VirtualMoveBaseJointName
from .pose import Pose, Point
from ..validation.error_checkers import calculate_joint_position_error, is_error_acceptable
from ..config import world_conf as conf

if TYPE_CHECKING:
    from ..description import Link
    from ..world_concepts.world_object import Object
    from ..world_concepts.constraints import Attachment


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
        Set the axis-aligned bounding box from a minimum and maximum point.

        :param min_point: The minimum point
        :param max_point: The maximum point
        """
        return cls(min_point[0], min_point[1], min_point[2], max_point[0], max_point[1], max_point[2])

    def get_min_max_points(self) -> Tuple[Point, Point]:
        """
        Return the axis-aligned bounding box as a tuple of minimum and maximum points.

        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min_point(), self.get_max_point()

    def get_min_point(self) -> Point:
        """
        Return the axis-aligned bounding box as a minimum point.

        :return: The axis-aligned bounding box as a minimum point
        """
        return Point(self.min_x, self.min_y, self.min_z)

    def get_max_point(self) -> Point:
        """
        Return the axis-aligned bounding box as a maximum point.

        :return: The axis-aligned bounding box as a maximum point
        """
        return Point(self.max_x, self.max_y, self.max_z)

    def get_min_max(self) -> Tuple[List[float], List[float]]:
        """
        Return the axis-aligned bounding box as a tuple of minimum and maximum points.

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
    """
    Dataclass for storing the collision callbacks.
    """
    on_collision_cb: Callable
    no_collision_cb: Optional[Callable] = None


@dataclass
class MultiBody:
    """
    Dataclass for storing the information of a multibody which consists of a base and multiple links with joints.
    """
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
    """
    Abstract dataclass for storing the information of a visual shape.
    """
    rgba_color: Color
    visual_frame_position: List[float]

    @abstractmethod
    def shape_data(self) -> Dict[str, Any]:
        """
        Returns the shape data of the visual shape (e.g. half extents for a box, radius for a sphere) as a dictionary.
        """
        pass

    @property
    @abstractmethod
    def visual_geometry_type(self) -> Shape:
        """
        Returns the visual geometry type of the visual shape (e.g. box, sphere) as a Shape object.
        """
        pass


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


@dataclass
class State(ABC):
    """
    Abstract dataclass for storing the state of an entity (e.g. world, object, link, joint).
    """
    pass


@dataclass
class LinkState(State):
    """
    Dataclass for storing the state of a link.
    """
    constraint_ids: Dict[Link, int]

    def __eq__(self, other: 'LinkState'):
        return self.all_constraints_exist(other) and self.all_constraints_are_equal(other)

    def all_constraints_exist(self, other: 'LinkState'):
        """
        Check if all constraints exist in the other link state.

        :param other: The state of the other link.
        """
        return (all([cid_k in other.constraint_ids.keys() for cid_k in self.constraint_ids.keys()])
                and len(self.constraint_ids.keys()) == len(other.constraint_ids.keys()))

    def all_constraints_are_equal(self, other: 'LinkState'):
        """
        Check if all constraints are equal to the ones in the other link state.

        :param other: The state of the other link.
        """
        return all([cid == other_cid for cid, other_cid in zip(self.constraint_ids.values(),
                                                               other.constraint_ids.values())])


@dataclass
class JointState(State):
    """
    Dataclass for storing the state of a joint.
    """
    position: float
    acceptable_error: float

    def __eq__(self, other: 'JointState'):
        error = calculate_joint_position_error(self.position, other.position)
        return is_error_acceptable(error, other.acceptable_error)


@dataclass
class ObjectState(State):
    """
    Dataclass for storing the state of an object.
    """
    pose: Pose
    attachments: Dict[Object, Attachment]
    link_states: Dict[int, LinkState]
    joint_states: Dict[int, JointState]
    acceptable_pose_error: Tuple[float, float]

    def __eq__(self, other: 'ObjectState'):
        return (self.pose_is_almost_equal(other)
                and self.all_attachments_exist(other) and self.all_attachments_are_equal(other)
                and self.link_states == other.link_states
                and self.joint_states == other.joint_states)

    def pose_is_almost_equal(self, other: 'ObjectState'):
        """
        Check if the pose of the object is almost equal to the pose of another object.

        :param other: The state of the other object.
        """
        return self.pose.almost_equal(other.pose, other.acceptable_pose_error[0], other.acceptable_pose_error[1])

    def all_attachments_exist(self, other: 'ObjectState'):
        """
        Check if all attachments exist in the other object state.

        :param other: The state of the other object.
        """
        return (all([att_k in other.attachments.keys() for att_k in self.attachments.keys()])
                and len(self.attachments.keys()) == len(other.attachments.keys()))

    def all_attachments_are_equal(self, other: 'ObjectState'):
        """
        Check if all attachments are equal to the ones in the other object state.

        :param other: The state of the other object.
        """
        return all([att == other_att for att, other_att in zip(self.attachments.values(), other.attachments.values())])


@dataclass
class WorldState(State):
    """
    Dataclass for storing the state of the world.
    """
    simulator_state_id: int
    object_states: Dict[str, ObjectState]

    def __eq__(self, other: 'WorldState'):
        return (self.simulator_state_is_equal(other) and self.all_objects_exist(other)
                and self.all_objects_states_are_equal(other))

    def simulator_state_is_equal(self, other: 'WorldState'):
        """
        Check if the simulator state is equal to the simulator state of another world state.

        :param other: The state of the other world.
        """
        return self.simulator_state_id == other.simulator_state_id

    def all_objects_exist(self, other: 'WorldState'):
        """
        Check if all objects exist in the other world state.

        :param other: The state of the other world.
        """
        return (all([obj_name in other.object_states.keys() for obj_name in self.object_states.keys()])
                and len(self.object_states.keys()) == len(other.object_states.keys()))

    def all_objects_states_are_equal(self, other: 'WorldState'):
        """
        Check if all object states are equal to the ones in the other world state.

        :param other: The state of the other world.
        """
        return all([obj_state == other_obj_state
                    for obj_state, other_obj_state in zip(self.object_states.values(),
                                                          other.object_states.values())])


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
    Dataclass for storing the information of a contact point between two objects.
    """
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
"""
The closest point between two objects which has the same structure as ContactPoint.
"""


class ContactPointsList(list):
    """
    A list of contact points.
    """

    def __index__(self, index: int) -> ContactPoint:
        return super().__getitem__(index)

    def __getitem__(self, item) -> Union[ContactPoint, 'ContactPointsList']:
        if isinstance(item, slice):
            return ContactPointsList(super().__getitem__(item))
        return super().__getitem__(item)

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
        return [point.normal_on_b for point in self]

    def get_links_in_contact_of_object(self, obj: Object) -> List[Link]:
        """
        Get the links in contact of the object.

        :param obj: An instance of the Object class that represents the object.
        :return: A list of Link instances that represent the links in contact of the object.
        """
        return [point.link_b for point in self if point.link_b.object == obj]

    def get_points_of_object(self, obj: Object) -> 'ContactPointsList':
        """
        Get the points of the object.

        :param obj: An instance of the Object class that represents the object that the points are related to.
        :return: A ContactPointsList instance that represents the contact points of the object.
        """
        return ContactPointsList([point for point in self if point.link_b.object == obj])

    def get_objects_that_got_removed(self, previous_points: 'ContactPointsList') -> List[Object]:
        """
        Return the object that is not in the current points list but was in the initial points list.

        :param previous_points: The initial points list.
        :return: A list of Object instances that represent the objects that got removed.
        """
        initial_objects_in_contact = previous_points.get_objects_that_have_points()
        current_objects_in_contact = self.get_objects_that_have_points()
        return [obj for obj in initial_objects_in_contact if obj not in current_objects_in_contact]

    def get_new_objects(self, previous_points: 'ContactPointsList') -> List[Object]:
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

    def get_objects_that_have_points(self) -> List[Object]:
        """
        Return the objects that have points in the list.

        :return: A list of Object instances that represent the objects that have points in the list.
        """
        return [point.link_b.object for point in self]

    def get_names_of_objects_that_have_points(self) -> List[str]:
        """
        Return the names of the objects that have points in the list.

        :return: A list of strings that represent the names of the objects that have points in the list.
        """
        return [point.link_b.object.name for point in self]

    def __str__(self):
        return f"ContactPointsList: {', '.join(self.get_names_of_objects_that_have_points())}"

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
    color: Color
    id: int


@dataclass
class VirtualMoveBaseJoints:
    """
    Dataclass for storing the names, types and axes of the virtual move base joints of a mobile robot.
    """
    translation_x: Optional[str] = VirtualMoveBaseJointName.LINEAR_X.value
    translation_y: Optional[str] = VirtualMoveBaseJointName.LINEAR_Y.value
    angular_z: Optional[str] = VirtualMoveBaseJointName.ANGULAR_Z.value

    @property
    def names(self) -> List[str]:
        """
        Return the names of the virtual move base joints.
        """
        return [self.translation_x, self.translation_y, self.angular_z]

    def get_types(self) -> Dict[str, JointType]:
        """
        Return the joint types of the virtual move base joints.
        """
        return {self.translation_x: JointType.PRISMATIC,
                self.translation_y: JointType.PRISMATIC,
                self.angular_z: JointType.REVOLUTE}

    def get_axes(self) -> Dict[str, Point]:
        """
        Return the axes (i.e. The axis on which the joint moves) of the virtual move base joints.
        """
        return {self.translation_x: Point(1, 0, 0),
                self.translation_y: Point(0, 1, 0),
                self.angular_z: Point(0, 0, 1)}
