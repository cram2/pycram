from __future__ import annotations

import logging
import os
import pathlib
from abc import ABC, abstractmethod

from .ros.data_types import Time
import trimesh
from geometry_msgs.msg import Point, Quaternion
from typing_extensions import Tuple, Union, Any, List, Optional, Dict, TYPE_CHECKING, Self, deprecated

from .datastructures.dataclasses import JointState, AxisAlignedBoundingBox, Color, LinkState, VisualShape
from .datastructures.enums import JointType
from .datastructures.pose import Pose, Transform
from .datastructures.world_entity import WorldEntity
from .failures import ObjectDescriptionNotFound
from .local_transformer import LocalTransformer

if TYPE_CHECKING:
    from .world_concepts.world_object import Object


class EntityDescription(ABC):
    """
    A description of an entity. This can be a link, joint or object description.
    """

    @property
    @abstractmethod
    def origin(self) -> Pose:
        """
        :return: the origin of this entity.
        """
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """
        :return: the name of this entity.
        """
        pass


class LinkDescription(EntityDescription):
    """
    A link description of an object.
    """

    def __init__(self, parsed_link_description: Any):
        self.parsed_description = parsed_link_description

    @property
    @abstractmethod
    def geometry(self) -> Union[VisualShape, None]:
        """
        The geometry type of the collision element of this link.
        """
        pass


class JointDescription(EntityDescription):
    """
    A class that represents the description of a joint.
    """

    def __init__(self, parsed_joint_description: Optional[Any] = None, is_virtual: bool = False):
        """
        :param parsed_joint_description: The parsed description of the joint (e.g. from urdf or mjcf file).
        :param is_virtual: True if the joint is virtual (i.e. not a physically existing joint), False otherwise.
        """
        self.parsed_description = parsed_joint_description
        self.is_virtual: Optional[bool] = is_virtual

    @property
    @abstractmethod
    def type(self) -> JointType:
        """
        :return: The type of this joint.
        """
        pass

    @property
    @abstractmethod
    def axis(self) -> Point:
        """
        :return: The axis of this joint, for example the rotation axis for a revolute joint.
        """
        pass

    @property
    @abstractmethod
    def has_limits(self) -> bool:
        """
        :return: True if the joint has limits, False otherwise.
        """
        pass

    @property
    def limits(self) -> Tuple[float, float]:
        """
        :return: The lower and upper limits of this joint.
        """
        lower, upper = self.lower_limit, self.upper_limit
        if lower > upper:
            lower, upper = upper, lower
        return lower, upper

    @property
    @abstractmethod
    def lower_limit(self) -> Union[float, None]:
        """
        :return: The lower limit of this joint, or None if the joint has no limits.
        """
        pass

    @property
    @abstractmethod
    def upper_limit(self) -> Union[float, None]:
        """
        :return: The upper limit of this joint, or None if the joint has no limits.
        """
        pass

    @property
    @abstractmethod
    def parent(self) -> str:
        """
        :return: The name of the parent link of this joint.
        """
        pass

    @property
    @abstractmethod
    def child(self) -> str:
        """
        :return: The name of the child link of this joint.
        """
        pass

    @property
    def damping(self) -> float:
        """
        :return: The damping of this joint.
        """
        raise NotImplementedError

    @property
    def friction(self) -> float:
        """
        :return: The friction of this joint.
        """
        raise NotImplementedError


class ObjectEntity(WorldEntity):
    """
    An abstract base class that represents a physical part/entity of an Object.
    This can be a link or a joint of an Object.
    """

    def __init__(self, _id: int, obj: Object):
        WorldEntity.__init__(self, _id, obj.world)
        self.object: Object = obj

    @property
    def object_name(self) -> str:
        """
        The name of the object to which this joint belongs.
        """
        return self.object.name

    @property
    @abstractmethod
    def pose(self) -> Pose:
        """
        :return: The pose of this entity relative to the world frame.
        """
        pass

    @property
    def transform(self) -> Transform:
        """
        The transform of this entity.

        :return: The transform of this entity.
        """
        return self.pose.to_transform(self.tf_frame)

    @property
    @abstractmethod
    def tf_frame(self) -> str:
        """
        The tf frame of this entity.

        :return: The tf frame of this entity.
        """
        pass

    @property
    def object_id(self) -> int:
        """
        :return: the id of the object to which this entity belongs.
        """
        return self.object.id


class Link(ObjectEntity, LinkDescription, ABC):
    """
    A link of an Object in the World.
    """

    def __init__(self, _id: int, link_description: LinkDescription, obj: Object):
        ObjectEntity.__init__(self, _id, obj)
        LinkDescription.__init__(self, link_description.parsed_description)
        self.local_transformer: LocalTransformer = LocalTransformer()
        self.constraint_ids: Dict[Link, int] = {}
        self._current_pose: Optional[Pose] = None
        self.update_pose()

    def set_pose(self, pose: Pose) -> None:
        """
        Set the pose of this link to the given pose.
        NOTE: This will move the entire object such that the link is at the given pose, it will not consider any joints
        that can allow the link to be at the given pose.

        :param pose: The target pose for this link.
        """
        self.object.set_pose(self.get_object_pose_given_link_pose(pose))

    def get_object_pose_given_link_pose(self, pose):
        """
        Get the object pose given the link pose, which could be a hypothetical link pose to see what would be the object
        pose in that case (assuming that the object itself moved not the joints).

        :param pose: The link pose.
        """
        return (pose.to_transform(self.tf_frame) * self.get_transform_to_root_link()).to_pose()

    def get_pose_given_object_pose(self, pose):
        """
        Get the link pose given the object pose, which could be a hypothetical object pose to see what would be the link
        pose in that case (assuming that the object itself moved not the joints).

        :param pose: The object pose.
        """
        return (pose.to_transform(self.object.tf_frame) * self.get_transform_from_root_link()).to_pose()

    def get_transform_from_root_link(self) -> Transform:
        """
        Return the transformation from the root link of the object to this link.
        """
        return self.get_transform_from_link(self.object.root_link)

    def get_transform_to_root_link(self) -> Transform:
        """
        Return the transformation from this link to the root link of the object.
        """
        return self.get_transform_to_link(self.object.root_link)

    @property
    def current_state(self) -> LinkState:
        return LinkState(self.constraint_ids.copy())

    @current_state.setter
    def current_state(self, link_state: LinkState) -> None:
        if self.current_state != link_state:
            self.constraint_ids = link_state.constraint_ids

    def add_fixed_constraint_with_link(self, child_link: Self,
                                       child_to_parent_transform: Optional[Transform] = None) -> int:
        """
        Add a fixed constraint between this link and the given link, to create attachments for example.

        :param child_link: The child link to which a fixed constraint should be added.
        :param child_to_parent_transform: The transformation between the two links.
        :return: The unique id of the constraint.
        """
        if child_to_parent_transform is None:
            child_to_parent_transform = child_link.get_transform_to_link(self)
        constraint_id = self.world.add_fixed_constraint(self, child_link, child_to_parent_transform)
        self.constraint_ids[child_link] = constraint_id
        child_link.constraint_ids[self] = constraint_id
        return constraint_id

    def remove_constraint_with_link(self, child_link: 'Link') -> None:
        """
        Remove the constraint between this link and the given link.

        :param child_link: The child link of the constraint that should be removed.
        """
        self.world.remove_constraint(self.constraint_ids[child_link])
        del self.constraint_ids[child_link]
        if self in child_link.constraint_ids.keys():
            del child_link.constraint_ids[self]

    @property
    def is_only_link(self) -> bool:
        """
        :return: True if this link is the only link, False otherwise.
        """
        return self.object.has_one_link

    @property
    def is_root(self) -> bool:
        """
        :return: True if this link is the root link, False otherwise.
        """
        return self.object.get_root_link_id() == self.id

    def update_transform(self, transform_time: Optional[Time] = None) -> None:
        """
        Update the transformation of this link at the given time.

        :param transform_time: The time at which the transformation should be updated.
        """
        self.local_transformer.update_transforms([self.transform], transform_time)

    def get_transform_to_link(self, link: 'Link') -> Transform:
        """
        :param link: The link to which the transformation should be returned.
        :return: A Transform object with the transformation from this link to the given link.
        """
        return link.get_transform_from_link(self)

    def get_transform_from_link(self, link: 'Link') -> Transform:
        """
        :param link: The link from which the transformation should be returned.
        :return: A Transform object with the transformation from the given link to this link.
        """
        return self.get_pose_wrt_link(link).to_transform(self.tf_frame)

    def get_pose_wrt_link(self, link: 'Link') -> Pose:
        """
        :param link: The link with respect to which the pose should be returned.
        :return: A Pose object with the pose of this link with respect to the given link.
        """
        return self.local_transformer.transform_pose(self.pose, link.tf_frame)

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: An AxisAlignedBoundingBox object with the axis aligned bounding box of this link.
        """
        return self.world.get_link_axis_aligned_bounding_box(self)

    @property
    def position(self) -> Point:
        """
        :return: A Point object containing the position of the link relative to the world frame.
        """
        return self.pose.position

    @property
    def position_as_list(self) -> List[float]:
        """
        :return: A list containing the position of the link relative to the world frame.
        """
        return self.pose.position_as_list()

    @property
    def orientation(self) -> Quaternion:
        """
        :return: A Quaternion object containing the orientation of the link relative to the world frame.
        """
        return self.pose.orientation

    @property
    def orientation_as_list(self) -> List[float]:
        """
        :return: A list containing the orientation of the link relative to the world frame.
        """
        return self.pose.orientation_as_list()

    def update_pose(self) -> None:
        """
        Update the current pose of this link from the world.
        """
        self._current_pose = self.world.get_link_pose(self)

    @property
    def pose(self) -> Pose:
        """
        :return: A Pose object containing the pose of the link relative to the world frame.
        """
        if self.world.conf.update_poses_from_sim_on_get:
            self.update_pose()
        return self._current_pose

    @property
    def pose_as_list(self) -> List[List[float]]:
        """
        :return: A list containing the position and orientation of the link relative to the world frame.
        """
        return self.pose.to_list()

    def get_origin_transform(self) -> Transform:
        """
        :return: the transformation between the link frame and the origin frame of this link.
        """
        return self.origin.to_transform(self.tf_frame)

    @property
    def color(self) -> Color:
        """
        :return: A Color object containing the rgba_color of this link.
        """
        return self.world.get_link_color(self)

    @deprecated("Use color property setter instead")
    def set_color(self, color: Color) -> None:
        """
        Set the color of this link, could be rgb or rgba.

        :param color: The color as a list of floats, either rgb or rgba.
        """
        self.color = color

    @color.setter
    def color(self, color: Color) -> None:
        """
        Set the color of this link, could be rgb or rgba.

        :param color: The color as a list of floats, either rgb or rgba.
        """
        self.world.set_link_color(self, color)

    @property
    def origin_transform(self) -> Transform:
        """
        :return: The transform from world to origin of entity.
        """
        return self.origin.to_transform(self.tf_frame)

    @property
    def tf_frame(self) -> str:
        """
        The name of the tf frame of this link.
        """
        return f"{self.object.tf_frame}/{self.name}"

    def __eq__(self, other):
        return self.id == other.id and self.object == other.object and self.name == other.name

    def __copy__(self):
        return Link(self.id, self, self.object)

    def __hash__(self):
        return hash((self.id, self.object, self.name))


class RootLink(Link, ABC):
    """
    The root link of an Object in the World.
    This differs from the normal AbstractLink class in that the pose and the tf_frame is the same as that of the object.
    """

    def __init__(self, obj: Object):
        super().__init__(obj.get_root_link_id(), obj.get_root_link_description(), obj)

    @property
    def tf_frame(self) -> str:
        """
        :return: the tf frame of the root link, which is the same as the tf frame of the object.
        """
        return self.object.tf_frame

    def update_pose(self) -> None:
        self._current_pose = self.world.get_object_pose(self.object)

    def __copy__(self):
        return RootLink(self.object)


class Joint(ObjectEntity, JointDescription, ABC):
    """
    Represent a joint of an Object in the World.
    """

    def __init__(self, _id: int,
                 joint_description: JointDescription,
                 obj: Object, is_virtual: Optional[bool] = False):
        ObjectEntity.__init__(self, _id, obj)
        JointDescription.__init__(self, joint_description.parsed_description, is_virtual)
        self.acceptable_error = (self.world.conf.revolute_joint_position_tolerance if self.type == JointType.REVOLUTE
                                 else self.world.conf.prismatic_joint_position_tolerance)
        self._update_position()

    @property
    def tf_frame(self) -> str:
        """
        The tf frame of a joint is the tf frame of the child link.
        """
        return self.child_link.tf_frame

    @property
    def pose(self) -> Pose:
        """
        :return: The pose of this joint. The pose is the pose of the child link of this joint.
        """
        return self.child_link.pose

    def _update_position(self) -> None:
        """
        Update the current position of the joint from the physics simulator.
        """
        self._current_position = self.world.get_joint_position(self)

    @property
    def parent_link(self) -> Link:
        """
        :return: The parent link as a AbstractLink object.
        """
        return self.object.get_link(self.parent)

    @property
    def child_link(self) -> Link:
        """
        :return: The child link as a AbstractLink object.
        """
        return self.object.get_link(self.child)

    @property
    def position(self) -> float:
        if self.world.conf.update_poses_from_sim_on_get:
            self._update_position()
        return self._current_position

    def reset_position(self, position: float) -> None:
        self.world.reset_joint_position(self, position)
        self._update_position()

    def get_object_id(self) -> int:
        """
        :return: The integer id of the object to which this joint belongs.
        """
        return self.object.id

    @position.setter
    def position(self, joint_position: float) -> None:
        """
        Set the position of the given joint to the given joint pose. If the pose is outside the joint limits,
         issue a warning. However, set the joint either way.

        :param joint_position: The target pose for this joint
        """
        # TODO Limits for rotational (infinite) joints are 0 and 1, they should be considered separately
        if self.has_limits:
            low_lim, up_lim = self.limits
            if not low_lim <= joint_position <= up_lim:
                logging.warning(
                    f"The joint position has to be within the limits of the joint. The joint limits for {self.name}"
                    f" are {low_lim} and {up_lim}")
                logging.warning(f"The given joint position was: {joint_position}")
                # Temporarily disabled because kdl outputs values exciting joint limits
                # return
        self.reset_position(joint_position)

    def enable_force_torque_sensor(self) -> None:
        self.world.enable_joint_force_torque_sensor(self.object, self.id)

    def disable_force_torque_sensor(self) -> None:
        self.world.disable_joint_force_torque_sensor(self.object, self.id)

    def get_reaction_force_torque(self) -> List[float]:
        return self.world.get_joint_reaction_force_torque(self.object, self.id)

    def get_applied_motor_torque(self) -> float:
        return self.world.get_applied_joint_motor_torque(self.object, self.id)

    @property
    def current_state(self) -> JointState:
        return JointState(self.position, self.acceptable_error)

    @current_state.setter
    def current_state(self, joint_state: JointState) -> None:
        """
        Update the current state of this joint from the given joint state if the position is different.

        :param joint_state: The joint state to update from.
        """
        if self.current_state != joint_state:
            self.position = joint_state.position

    def __copy__(self):
        return Joint(self.id, self, self.object)

    def __eq__(self, other):
        return self.id == other.id and self.object == other.object and self.name == other.name

    def __hash__(self):
        return hash((self.id, self.object, self.name))


class ObjectDescription(EntityDescription):
    """
    A class that represents the description of an object.
    """

    mesh_extensions: Tuple[str] = (".obj", ".stl", ".dae", ".ply")
    """
    The file extensions of the mesh files that can be used to generate a description file.
    """

    class Link(Link, ABC):
        ...

    class RootLink(RootLink, ABC):
        ...

    class Joint(Joint, ABC):
        ...

    def __init__(self, path: Optional[str] = None):
        """
        :param path: The path of the file to update the description data from.
        """

        self._links: Optional[List[LinkDescription]] = None
        self._joints: Optional[List[JointDescription]] = None
        self._link_map: Optional[Dict[str, Any]] = None
        self._joint_map: Optional[Dict[str, Any]] = None

        if path:
            self.update_description_from_file(path)
        else:
            self._parsed_description = None

        self.virtual_joint_names: List[str] = []

    @property
    @abstractmethod
    def child_map(self) -> Dict[str, List[Tuple[str, str]]]:
        """
        :return: A dictionary mapping the name of a link to its children which are represented as a tuple of the child
            joint name and the link name.
        """
        pass

    @property
    @abstractmethod
    def parent_map(self) -> Dict[str, Tuple[str, str]]:
        """
        :return: A dictionary mapping the name of a link to its parent joint and link as a tuple.
        """
        pass

    @property
    @abstractmethod
    def link_map(self) -> Dict[str, LinkDescription]:
        """
        :return: A dictionary mapping the name of a link to its description.
        """
        pass

    @property
    @abstractmethod
    def joint_map(self) -> Dict[str, JointDescription]:
        """
        :return: A dictionary mapping the name of a joint to its description.
        """
        pass

    def is_joint_virtual(self, name: str) -> bool:
        """
        :param name: The name of the joint.
        :return: True if the joint is virtual, False otherwise.
        """
        return name in self.virtual_joint_names

    @abstractmethod
    def add_joint(self, name: str, child: str, joint_type: JointType,
                  axis: Point, parent: Optional[str] = None, origin: Optional[Pose] = None,
                  lower_limit: Optional[float] = None, upper_limit: Optional[float] = None,
                  is_virtual: Optional[bool] = False) -> None:
        """
        Add a joint to this object.

        :param name: The name of the joint.
        :param child: The name of the child link.
        :param joint_type: The type of the joint.
        :param axis: The axis of the joint.
        :param parent: The name of the parent link.
        :param origin: The origin of the joint.
        :param lower_limit: The lower limit of the joint.
        :param upper_limit: The upper limit of the joint.
        :param is_virtual: True if the joint is virtual, False otherwise.
        """
        pass

    def update_description_from_file(self, path: str) -> None:
        """
        Update the description of this object from the file at the given path.

        :param path: The path of the file to update from.
        """
        self._parsed_description = self.load_description(path)

    def update_description_from_string(self, description_string: str) -> None:
        """
        Update the description of this object from the given description string.

        :param description_string: The description string to update from.
        """
        self._parsed_description = self.load_description_from_string(description_string)

    def load_description_from_string(self, description_string: str) -> Any:
        """
        Load the description from the given string.

        :param description_string: The description string to load from.
        """
        raise NotImplementedError

    @property
    def parsed_description(self) -> Any:
        """
        :return: The object parsed from the description file.
        """
        return self._parsed_description

    @parsed_description.setter
    def parsed_description(self, parsed_description: Any):
        """
        :param parsed_description: The parsed description object (depends on the description file type).
        """
        self._parsed_description = parsed_description

    @abstractmethod
    def load_description(self, path: str) -> Any:
        """
        Load the description from the file at the given path.

        :param path: The path to the source file, if only a filename is provided then the resources directories will be
         searched.
        """
        pass

    def generate_description_from_file(self, path: str, name: str, extension: str, save_path: str,
                                       scale_mesh: Optional[float] = None) -> None:
        """
        Generate and preprocess the description from the file at the given path and save the preprocessed
        description. The generated description will be saved at the given save path.

        :param path: The path of the file to preprocess.
        :param name: The name of the object.
        :param extension: The file extension of the file to preprocess.
        :param save_path: The path to save the generated description file.
        :param scale_mesh: The scale of the mesh.
        :raises ObjectDescriptionNotFound: If the description file could not be found/read.
        """

        if extension in self.mesh_extensions:
            if extension == ".ply":
                mesh = trimesh.load(path)
                path = path.replace(extension, ".obj")
                if scale_mesh is not None:
                    mesh.apply_scale(scale_mesh)
                mesh.export(path)
            self.generate_from_mesh_file(path, name, save_path=save_path)
        elif extension == self.get_file_extension():
            self.generate_from_description_file(path, save_path=save_path)
        else:
            try:
                # Using the description from the parameter server
                self.generate_from_parameter_server(path, save_path=save_path)
            except KeyError:
                logging.warning(f"Couldn't find file data in the ROS parameter server")

        if not self.check_description_file_exists_and_can_be_read(save_path):
            raise ObjectDescriptionNotFound(name, path, extension)

    @staticmethod
    def check_description_file_exists_and_can_be_read(path: str) -> bool:
        """
        Check if the description file exists at the given path.

        :param path: The path to the description file.
        :return: True if the file exists, False otherwise.
        """
        exists = os.path.exists(path)
        if exists:
            with open(path, "r") as file:
                exists = bool(file.read())
        return exists

    @staticmethod
    def write_description_to_file(description_string: str, save_path: str) -> None:
        """
        Write the description string to the file at the given path.

        :param description_string: The description string to write.
        :param save_path: The path of the file to write to.
        """
        with open(save_path, "w") as file:
            file.write(description_string)

    def get_file_name(self, path_object: pathlib.Path, extension: str, object_name: str) -> str:
        """
        :param path_object: The path object of the description file or the mesh file.
        :param extension: The file extension of the description file or the mesh file.
        :param object_name: The name of the object.
        :return: The file name of the description file.
        """
        if extension in self.mesh_extensions:
            file_name = path_object.stem + self.get_file_extension()
        elif extension == self.get_file_extension():
            file_name = path_object.name
        else:
            file_name = object_name + self.get_file_extension()

        return file_name

    @classmethod
    @abstractmethod
    def generate_from_mesh_file(cls, path: str, name: str, save_path: str) -> None:
        """
        Generate a description file from one of the mesh types defined in the mesh_extensions and
        return the path of the generated file. The generated file will be saved at the given save_path.

        :param path: The path to the .obj file.
        :param name: The name of the object.
        :param save_path: The path to save the generated description file.
        """
        pass

    @classmethod
    @abstractmethod
    def generate_from_description_file(cls, path: str, save_path: str, make_mesh_paths_absolute: bool = True) -> None:
        """
        Preprocess the given file and return the preprocessed description string. The preprocessed description will be
        saved at the given save_path.

        :param path: The path of the file to preprocess.
        :param save_path: The path to save the preprocessed description file.
        :param make_mesh_paths_absolute: Whether to make the mesh paths absolute.
        """
        pass

    @classmethod
    @abstractmethod
    def generate_from_parameter_server(cls, name: str, save_path: str) -> None:
        """
        Preprocess the description from the ROS parameter server and return the preprocessed description string.
        The preprocessed description will be saved at the given save_path.

        :param name: The name of the description on the parameter server.
        :param save_path: The path to save the preprocessed description file.
        """
        pass

    @property
    @abstractmethod
    def links(self) -> List[LinkDescription]:
        """
        :return: A list of links descriptions of this object.
        """
        pass

    def get_link_by_name(self, link_name: str) -> LinkDescription:
        """
        :return: The link description with the given name.
        """
        return self.link_map[link_name]

    @property
    @abstractmethod
    def joints(self) -> List[JointDescription]:
        """
        :return: A list of joints descriptions of this object.
        """
        pass

    def get_joint_by_name(self, joint_name: str) -> JointDescription:
        """
        :return: The joint description with the given name.
        """
        return self.joint_map[joint_name]

    @abstractmethod
    def get_root(self) -> str:
        """
        :return: the name of the root link of this object.
        """
        pass

    def get_tip(self) -> str:
        """
        :return: the name of the tip link of this object.
        """
        raise NotImplementedError

    @abstractmethod
    def get_chain(self, start_link_name: str, end_link_name: str, joints: Optional[bool] = True,
                  links: Optional[bool] = True, fixed: Optional[bool] = True) -> List[str]:
        """
        :return: the chain of links from 'start_link_name' to 'end_link_name'.
        """
        pass

    @staticmethod
    @abstractmethod
    def get_file_extension() -> str:
        """
        :return: The file extension of the description file.
        """
        pass
