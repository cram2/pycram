from __future__ import annotations

import os
from abc import ABC, abstractmethod
from copy import deepcopy, copy
from dataclasses import dataclass, fields, field

import numpy as np
import trimesh
from typing_extensions import List, Optional, Tuple, Callable, Dict, Any, Union, TYPE_CHECKING, Sequence

from .enums import JointType, Shape, VirtualMobileBaseJointName
from .pose import Pose, Point, Transform
from ..validation.error_checkers import calculate_joint_position_error, is_error_acceptable

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
class BoundingBox:
    """
    Dataclass for storing an axis-aligned bounding box.
    """
    min_x: float
    min_y: float
    min_z: float
    max_x: float
    max_y: float
    max_z: float

    def get_points_list(self) -> List[List[float]]:
        """
        :return: The points of the bounding box as a list of lists of floats.
        """
        return list(filter(get_point_as_list, self.get_points()))

    def get_points(self) -> List[Point]:
        """
        :return: The points of the bounding box as a list of Point instances.
        """
        return [Point(self.min_x, self.min_y, self.min_z),
                Point(self.min_x, self.min_y, self.max_z),
                Point(self.min_x, self.max_y, self.min_z),
                Point(self.min_x, self.max_y, self.max_z),
                Point(self.max_x, self.min_y, self.min_z),
                Point(self.max_x, self.min_y, self.max_z),
                Point(self.max_x, self.max_y, self.min_z),
                Point(self.max_x, self.max_y, self.max_z)]

    def get_min_max_points(self) -> Tuple[Point, Point]:
        """
        :return: The axis-aligned bounding box as a tuple of minimum and maximum points
        """
        return self.get_min_point(), self.get_max_point()

    def get_min_point(self) -> Point:
        """
        :return: The axis-aligned bounding box as a minimum point
        """
        return Point(self.min_x, self.min_y, self.min_z)

    def get_max_point(self) -> Point:
        """
        :return: The axis-aligned bounding box as a maximum point
        """
        return Point(self.max_x, self.max_y, self.max_z)

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

    @property
    def width(self) -> float:
        return self.max_x - self.min_x

    @property
    def height(self) -> float:
        return self.max_z - self.min_z

    @property
    def depth(self) -> float:
        return self.max_y - self.min_y


@dataclass
class AxisAlignedBoundingBox(BoundingBox):

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
    def from_multiple_bounding_boxes(cls, bounding_boxes: List[AxisAlignedBoundingBox]) -> 'AxisAlignedBoundingBox':
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

    def get_transformed_box(self, transform: Transform) -> 'AxisAlignedBoundingBox':
        """
        Apply a transformation to the axis-aligned bounding box and return the transformed axis-aligned bounding box.

        :param transform: The transformation to apply
        :return: The transformed axis-aligned bounding box
        """
        transformed_points = transform.apply_transform_to_array_of_points(np.array(self.get_min_max()))
        min_p = [min(transformed_points[:, i]) for i in range(3)]
        max_p = [max(transformed_points[:, i]) for i in range(3)]
        return AxisAlignedBoundingBox.from_min_max(min_p, max_p)

    @classmethod
    def from_min_max(cls, min_point: Sequence[float], max_point: Sequence[float]):
        """
        Set the axis-aligned bounding box from a minimum and maximum point.

        :param min_point: The minimum point
        :param max_point: The maximum point
        """
        return cls(min_point[0], min_point[1], min_point[2], max_point[0], max_point[1], max_point[2])


@dataclass
class RotatedBoundingBox(BoundingBox):
    """
    Dataclass for storing a rotated bounding box.
    """

    def __init__(self, min_x: float, min_y: float, min_z: float, max_x: float, max_y: float, max_z: float,
                 transform: Transform, points: Optional[List[Point]] = None):
        self.min_x, self.min_y, self.min_z = min_x, min_y, min_z
        self.max_x, self.max_y, self.max_z = max_x, max_y, max_z
        self.transform: Transform = transform
        self._points: Optional[List[Point]] = points

    @classmethod
    def from_min_max(cls, min_point: Sequence[float], max_point: Sequence[float], transform: Transform):
        """
        Set the rotated bounding box from a minimum, maximum point, and a transformation.

        :param min_point: The minimum point
        :param max_point: The maximum point
        :param transform: The transformation
        """
        return cls(min_point[0], min_point[1], min_point[2], max_point[0], max_point[1], max_point[2], transform)

    @classmethod
    def from_axis_aligned_bounding_box(cls, axis_aligned_bounding_box: AxisAlignedBoundingBox,
                                       transform: Transform) -> 'RotatedBoundingBox':
        """
        Set the rotated bounding box from an axis-aligned bounding box and a transformation.

        :param axis_aligned_bounding_box: The axis-aligned bounding box.
        :param transform: The transformation.
        """
        return cls(axis_aligned_bounding_box.min_x, axis_aligned_bounding_box.min_y, axis_aligned_bounding_box.min_z,
                   axis_aligned_bounding_box.max_x, axis_aligned_bounding_box.max_y, axis_aligned_bounding_box.max_z,
                   transform)

    def get_points_list(self) -> List[List[float]]:
        """
        :return: The points of the rotated bounding box as a list of lists of floats.
        """
        return [[point.x, point.y, point.z] for point in self.get_points()]

    def get_points(self, transform: Optional[Transform] = None) -> List[Point]:
        """
        :param transform: The transformation to apply to the points, if None the stored transformation is used.
        :return: The points of the rotated bounding box.
        """
        if (self._points is None) or (transform is not None):
            if transform is not None:
                self.transform = transform
            points_array = np.array([[self.min_x, self.min_y, self.min_z],
                                     [self.min_x, self.min_y, self.max_z],
                                     [self.min_x, self.max_y, self.min_z],
                                     [self.min_x, self.max_y, self.max_z],
                                     [self.max_x, self.min_y, self.min_z],
                                     [self.max_x, self.min_y, self.max_z],
                                     [self.max_x, self.max_y, self.min_z],
                                     [self.max_x, self.max_y, self.max_z]])
            transformed_points = self.transform.apply_transform_to_array_of_points(points_array).tolist()
            self._points = [Point(*point) for point in transformed_points]
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
class LinkState(State):
    """
    Dataclass for storing the state of a link.
    """
    constraint_ids: Dict[Link, int]

    def __eq__(self, other: 'LinkState'):
        return self.all_constraints_exist(other) and self.all_constraints_are_equal(other)

    def all_constraints_exist(self, other: 'LinkState') -> bool:
        """
        Check if all constraints exist in the other link state.

        :param other: The state of the other link.
        :return: True if all constraints exist, False otherwise.
        """
        return (all([cid_k in other.constraint_ids.keys() for cid_k in self.constraint_ids.keys()])
                and len(self.constraint_ids.keys()) == len(other.constraint_ids.keys()))

    def all_constraints_are_equal(self, other: 'LinkState') -> bool:
        """
        Check if all constraints are equal to the ones in the other link state.

        :param other: The state of the other link.
        :return: True if all constraints are equal, False otherwise.
        """
        return all([cid == other_cid for cid, other_cid in zip(self.constraint_ids.values(),
                                                               other.constraint_ids.values())])

    def __copy__(self):
        return LinkState(constraint_ids=copy(self.constraint_ids))


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

    def __copy__(self):
        return JointState(position=self.position, acceptable_error=self.acceptable_error)


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

    def pose_is_almost_equal(self, other: 'ObjectState') -> bool:
        """
        Check if the pose of the object is almost equal to the pose of another object.

        :param other: The state of the other object.
        :return: True if the poses are almost equal, False otherwise.
        """
        return self.pose.almost_equal(other.pose, other.acceptable_pose_error[0], other.acceptable_pose_error[1])

    def all_attachments_exist(self, other: 'ObjectState') -> bool:
        """
        Check if all attachments exist in the other object state.

        :param other: The state of the other object.
        :return: True if all attachments exist, False otherwise.
        """
        return (all([att_k in other.attachments.keys() for att_k in self.attachments.keys()])
                and len(self.attachments.keys()) == len(other.attachments.keys()))

    def all_attachments_are_equal(self, other: 'ObjectState') -> bool:
        """
        Check if all attachments are equal to the ones in the other object state.

        :param other: The state of the other object.
        :return: True if all attachments are equal, False otherwise
        """
        return all([att == other_att for att, other_att in zip(self.attachments.values(), other.attachments.values())])

    def __copy__(self):
        return ObjectState(pose=self.pose.copy(), attachments=copy(self.attachments),
                           link_states=copy(self.link_states),
                           joint_states=copy(self.joint_states),
                           acceptable_pose_error=deepcopy(self.acceptable_pose_error))


@dataclass
class WorldState(State):
    """
    Dataclass for storing the state of the world.
    """
    object_states: Dict[str, ObjectState]
    simulator_state_id: Optional[int] = None

    def __eq__(self, other: 'WorldState'):
        return (self.simulator_state_is_equal(other) and self.all_objects_exist(other)
                and self.all_objects_states_are_equal(other))

    def simulator_state_is_equal(self, other: 'WorldState') -> bool:
        """
        Check if the simulator state is equal to the simulator state of another world state.

        :param other: The state of the other world.
        :return: True if the simulator states are equal, False otherwise.
        """
        return self.simulator_state_id == other.simulator_state_id

    def all_objects_exist(self, other: 'WorldState') -> bool:
        """
        Check if all objects exist in the other world state.

        :param other: The state of the other world.
        :return: True if all objects exist, False otherwise.
        """
        return (all([obj_name in other.object_states.keys() for obj_name in self.object_states.keys()])
                and len(self.object_states.keys()) == len(other.object_states.keys()))

    def all_objects_states_are_equal(self, other: 'WorldState') -> bool:
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
    Dataclass for storing the information of a contact point between two objects.
    """
    link_a: Link
    link_b: Link
    position_on_object_a: Optional[List[float]] = None
    position_on_object_b: Optional[List[float]] = None
    normal_on_b: Optional[List[float]] = None  # the contact normal vector on object b pointing towards object a
    distance: Optional[float] = None  # distance between the two objects (+ve for separation, -ve for penetration)
    normal_force: Optional[List[float]] = None  # normal force applied during last step simulation
    lateral_friction_1: Optional[LateralFriction] = None
    lateral_friction_2: Optional[LateralFriction] = None

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

    def get_links_that_got_removed(self, previous_points: 'ContactPointsList') -> List[Link]:
        """
        Return the links that are not in the current points list but were in the initial points list.

        :param previous_points: The initial points list.
        :return: A list of Link instances that represent the links that got removed.
        """
        initial_links_in_contact = previous_points.get_links_in_contact()
        current_links_in_contact = self.get_links_in_contact()
        return [link for link in initial_links_in_contact if link not in current_links_in_contact]

    def get_links_in_contact(self) -> List[Link]:
        """
        Get the links in contact.

        :return: A list of Link instances that represent the links in contact.
        """
        return [point.link_b for point in self]

    def check_if_two_objects_are_in_contact(self, obj_a: Object, obj_b: Object) -> bool:
        """
        Check if two objects are in contact.

        :param obj_a: An instance of the Object class that represents the first object.
        :param obj_b: An instance of the Object class that represents the second object.
        :return: True if the objects are in contact, False otherwise.
        """
        return (any([point.link_b.object == obj_b and point.link_a.object == obj_a for point in self]) or
                any([point.link_a.object == obj_b and point.link_b.object == obj_a for point in self]))

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
        return list({point.link_b.object for point in self})

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
    color: Color = Color(0, 0, 0, 1)
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


@dataclass
class VirtualMobileBaseJoints:
    """
    Dataclass for storing the names, types and axes of the virtual mobile base joints of a mobile robot.
    """
    translation_x: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.LINEAR_X.value,
                                                         JointType.PRISMATIC,
                                                         Point(1, 0, 0))
    translation_y: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.LINEAR_Y.value,
                                                         JointType.PRISMATIC,
                                                         Point(0, 1, 0))
    angular_z: Optional[VirtualJoint] = VirtualJoint(VirtualMobileBaseJointName.ANGULAR_Z.value,
                                                     JointType.REVOLUTE,
                                                     Point(0, 0, 1))

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
