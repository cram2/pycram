from __future__ import annotations

import logging
import pathlib
from abc import ABC, abstractmethod

import rospy
from geometry_msgs.msg import Point, Quaternion
from typing_extensions import Tuple, Union, Any, List, Optional, Dict, TYPE_CHECKING

from .datastructures.enums import JointType
from .local_transformer import LocalTransformer
from .datastructures.pose import Pose, Transform
from .datastructures.world import WorldEntity
from .datastructures.dataclasses import JointState, AxisAlignedBoundingBox, Color, LinkState, VisualShape

if TYPE_CHECKING:
    from .world_concepts.world_object import Object


class EntityDescription(ABC):

    """
    A class that represents a description of an entity. This can be a link, joint or object description.
    """

    @property
    @abstractmethod
    def origin(self) -> Pose:
        """
        Returns the origin of this entity.
        """
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """
        Returns the name of this entity.
        """
        pass


class LinkDescription(EntityDescription):
    """
    A class that represents a link description of an object.
    """

    def __init__(self, parsed_link_description: Any):
        self.parsed_description = parsed_link_description

    @property
    @abstractmethod
    def geometry(self) -> Union[VisualShape, None]:
        """
        Returns the geometry type of the collision element of this link.
        """
        pass


class JointDescription(EntityDescription):
    """
    A class that represents the description of a joint.
    """

    def __init__(self, parsed_joint_description: Any):
        self.parsed_description = parsed_joint_description

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
        Checks if this joint has limits.

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
    def parent_link_name(self) -> str:
        """
        :return: The name of the parent link of this joint.
        """
        pass

    @property
    @abstractmethod
    def child_link_name(self) -> str:
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
    @abstractmethod
    def pose(self) -> Pose:
        """
        :return: The pose of this entity relative to the world frame.
        """
        pass

    @property
    def transform(self) -> Transform:
        """
        Returns the transform of this entity.

        :return: The transform of this entity.
        """
        return self.pose.to_transform(self.tf_frame)

    @property
    @abstractmethod
    def tf_frame(self) -> str:
        """
        Returns the tf frame of this entity.

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
    Represents a link of an Object in the World.
    """

    def __init__(self, _id: int, link_description: LinkDescription, obj: Object):
        ObjectEntity.__init__(self, _id, obj)
        LinkDescription.__init__(self, link_description.parsed_description)
        self.local_transformer: LocalTransformer = LocalTransformer()
        self.constraint_ids: Dict[Link, int] = {}
        self._update_pose()

    @property
    def current_state(self) -> LinkState:
        return LinkState(self.constraint_ids.copy())

    @current_state.setter
    def current_state(self, link_state: LinkState) -> None:
        self.constraint_ids = link_state.constraint_ids

    def add_fixed_constraint_with_link(self, child_link: 'Link') -> int:
        """
        Adds a fixed constraint between this link and the given link, used to create attachments for example.

        :param child_link: The child link to which a fixed constraint should be added.
        :return: The unique id of the constraint.
        """
        constraint_id = self.world.add_fixed_constraint(self,
                                                        child_link,
                                                        child_link.get_transform_from_link(self))
        self.constraint_ids[child_link] = constraint_id
        child_link.constraint_ids[self] = constraint_id
        return constraint_id

    def remove_constraint_with_link(self, child_link: 'Link') -> None:
        """
        Removes the constraint between this link and the given link.

        :param child_link: The child link of the constraint that should be removed.
        """
        self.world.remove_constraint(self.constraint_ids[child_link])
        del self.constraint_ids[child_link]
        if self in child_link.constraint_ids.keys():
            del child_link.constraint_ids[self]

    @property
    def is_root(self) -> bool:
        """
        Returns whether this link is the root link of the object.

        :return: True if this link is the root link, False otherwise.
        """
        return self.object.get_root_link_id() == self.id

    def update_transform(self, transform_time: Optional[rospy.Time] = None) -> None:
        """
        Updates the transformation of this link at the given time.

        :param transform_time: The time at which the transformation should be updated.
        """
        self.local_transformer.update_transforms([self.transform], transform_time)

    def get_transform_to_link(self, link: 'Link') -> Transform:
        """
        Returns the transformation from this link to the given link.

        :param link: The link to which the transformation should be returned.
        :return: A Transform object with the transformation from this link to the given link.
        """
        return link.get_transform_from_link(self)

    def get_transform_from_link(self, link: 'Link') -> Transform:
        """
        Returns the transformation from the given link to this link.

        :param link: The link from which the transformation should be returned.
        :return: A Transform object with the transformation from the given link to this link.
        """
        return self.get_pose_wrt_link(link).to_transform(self.tf_frame)

    def get_pose_wrt_link(self, link: 'Link') -> Pose:
        """
        Returns the pose of this link with respect to the given link.

        :param link: The link with respect to which the pose should be returned.
        :return: A Pose object with the pose of this link with respect to the given link.
        """
        return self.local_transformer.transform_pose(self.pose, link.tf_frame)

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of this link.

        :return: An AxisAlignedBoundingBox object with the axis aligned bounding box of this link.
        """
        return self.world.get_link_axis_aligned_bounding_box(self)

    @property
    def position(self) -> Point:
        """
        The getter for the position of the link relative to the world frame.

        :return: A Point object containing the position of the link relative to the world frame.
        """
        return self.pose.position

    @property
    def position_as_list(self) -> List[float]:
        """
        The getter for the position of the link relative to the world frame as a list.

        :return: A list containing the position of the link relative to the world frame.
        """
        return self.pose.position_as_list()

    @property
    def orientation(self) -> Quaternion:
        """
        The getter for the orientation of the link relative to the world frame.

        :return: A Quaternion object containing the orientation of the link relative to the world frame.
        """
        return self.pose.orientation

    @property
    def orientation_as_list(self) -> List[float]:
        """
        The getter for the orientation of the link relative to the world frame as a list.

        :return: A list containing the orientation of the link relative to the world frame.
        """
        return self.pose.orientation_as_list()

    def _update_pose(self) -> None:
        """
        Updates the current pose of this link from the world.
        """
        self._current_pose = self.world.get_link_pose(self)

    @property
    def pose(self) -> Pose:
        """
        The pose of the link relative to the world frame.

        :return: A Pose object containing the pose of the link relative to the world frame.
        """
        return self._current_pose

    @property
    def pose_as_list(self) -> List[List[float]]:
        """
        The pose of the link relative to the world frame as a list.

        :return: A list containing the position and orientation of the link relative to the world frame.
        """
        return self.pose.to_list()

    def get_origin_transform(self) -> Transform:
        """
        Returns the transformation between the link frame and the origin frame of this link.
        """
        return self.origin.to_transform(self.tf_frame)

    @property
    def color(self) -> Color:
        """
        The getter for the rgba_color of this link.

        :return: A Color object containing the rgba_color of this link.
        """
        return self.world.get_link_color(self)

    @color.setter
    def color(self, color: Color) -> None:
        """
        The setter for the color of this link, could be rgb or rgba.

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
    Represents the root link of an Object in the World.
    It differs from the normal AbstractLink class in that the pose ande the tf_frame is the same as that of the object.
    """

    def __init__(self, obj: Object):
        super().__init__(obj.get_root_link_id(), obj.get_root_link_description(), obj)

    @property
    def tf_frame(self) -> str:
        """
        Returns the tf frame of the root link, which is the same as the tf frame of the object.
        """
        return self.object.tf_frame

    def _update_pose(self) -> None:
        self._current_pose = self.object.get_pose()

    def __copy__(self):
        return RootLink(self.object)


class Joint(ObjectEntity, JointDescription, ABC):
    """
    Represents a joint of an Object in the World.
    """

    def __init__(self, _id: int,
                 joint_description: JointDescription,
                 obj: Object):
        ObjectEntity.__init__(self, _id, obj)
        JointDescription.__init__(self, joint_description.parsed_description)
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
        Returns the pose of this joint. The pose is the pose of the child link of this joint.

        :return: The pose of this joint.
        """
        return self.child_link.pose

    def _update_position(self) -> None:
        """
        Updates the current position of the joint from the physics simulator.
        """
        self._current_position = self.world.get_joint_position(self)

    @property
    def parent_link(self) -> Link:
        """
        Returns the parent link of this joint.

        :return: The parent link as a AbstractLink object.
        """
        return self.object.get_link(self.parent_link_name)

    @property
    def child_link(self) -> Link:
        """
        Returns the child link of this joint.

        :return: The child link as a AbstractLink object.
        """
        return self.object.get_link(self.child_link_name)

    @property
    def position(self) -> float:
        return self._current_position

    def reset_position(self, position: float) -> None:
        self.world.reset_joint_position(self, position)
        self._update_position()

    def get_object_id(self) -> int:
        """
        Returns the id of the object to which this joint belongs.

        :return: The integer id of the object to which this joint belongs.
        """
        return self.object.id

    @position.setter
    def position(self, joint_position: float) -> None:
        """
        Sets the position of the given joint to the given joint pose. If the pose is outside the joint limits,
         an error will be printed. However, the joint will be set either way.

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
        return JointState(self.position)

    @current_state.setter
    def current_state(self, joint_state: JointState) -> None:
        """
        Updates the current state of this joint from the given joint state if the position is different.

        :param joint_state: The joint state to update from.
        """
        if self._current_position != joint_state.position:
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

    mesh_extensions: Tuple[str] = (".obj", ".stl", ".dae")
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
        if path:
            self.update_description_from_file(path)
        else:
            self._parsed_description = None

    def update_description_from_file(self, path: str) -> None:
        """
        Updates the description of this object from the file at the given path.

        :param path: The path of the file to update from.
        """
        self._parsed_description = self.load_description(path)

    @property
    def parsed_description(self) -> Any:
        """
        Return the object parsed from the description file.
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
        Loads the description from the file at the given path.

        :param path: The path to the source file, if only a filename is provided then the resources directories will be
         searched.
        """
        pass

    def generate_description_from_file(self, path: str, name: str, extension: str) -> str:
        """
        Generates and preprocesses the description from the file at the given path and returns the preprocessed
        description as a string.

        :param path: The path of the file to preprocess.
        :param name: The name of the object.
        :param extension: The file extension of the file to preprocess.
        :return: The processed description string.
        """
        description_string = None

        if extension in self.mesh_extensions:
            description_string = self.generate_from_mesh_file(path, name)
        elif extension == self.get_file_extension():
            description_string = self.generate_from_description_file(path)
        else:
            try:
                # Using the description from the parameter server
                description_string = self.generate_from_parameter_server(path)
            except KeyError:
                logging.warning(f"Couldn't find dile data in the ROS parameter server")
        if description_string is None:
            logging.error(f"Could not find file with path {path} in the resources directory nor"
                          f" in the ros parameter server.")
            raise FileNotFoundError

        return description_string

    def get_file_name(self, path_object: pathlib.Path, extension: str, object_name: str) -> str:
        """
        Returns the file name of the description file.

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
    def generate_from_mesh_file(cls, path: str, name: str) -> str:
        """
        Generates a description file from one of the mesh types defined in the mesh_extensions and
        returns the path of the generated file.

        :param path: The path to the .obj file.
        :param name: The name of the object.
        :return: The path of the generated description file.
        """
        pass

    @classmethod
    @abstractmethod
    def generate_from_description_file(cls, path: str) -> str:
        """
        Preprocesses the given file and returns the preprocessed description string.

        :param path: The path of the file to preprocess.
        :return: The preprocessed description string.
        """
        pass

    @classmethod
    @abstractmethod
    def generate_from_parameter_server(cls, name: str) -> str:
        """
        Preprocesses the description from the ROS parameter server and returns the preprocessed description string.

        :param name: The name of the description on the parameter server.
        :return: The preprocessed description string.
        """
        pass

    @property
    @abstractmethod
    def links(self) -> List[LinkDescription]:
        """
        :return: A list of links descriptions of this object.
        """
        pass

    @abstractmethod
    def get_link_by_name(self, link_name: str) -> LinkDescription:
        """
        :return: The link description with the given name.
        """
        pass

    @property
    @abstractmethod
    def joints(self) -> List[JointDescription]:
        """
        :return: A list of joints descriptions of this object.
        """
        pass

    @abstractmethod
    def get_joint_by_name(self, joint_name: str) -> JointDescription:
        """
        :return: The joint description with the given name.
        """
        pass

    @abstractmethod
    def get_root(self) -> str:
        """
        :return: the name of the root link of this object.
        """
        pass

    @abstractmethod
    def get_chain(self, start_link_name: str, end_link_name: str) -> List[str]:
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
