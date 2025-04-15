from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from deprecated import deprecated
from geometry_msgs.msg import Point, Quaternion
from trimesh.parent import Geometry3D
from typing_extensions import Type, Optional, Dict, Tuple, List, Union

from ..datastructures.dataclasses import (Color, ObjectState, LinkState, JointState,
                                          AxisAlignedBoundingBox, VisualShape, ClosestPointsList,
                                          ContactPointsList, RotatedBoundingBox, VirtualJoint, FrozenObject, FrozenLink, FrozenJoint)
from ..datastructures.enums import ObjectType, JointType
from ..datastructures.pose import Pose, Transform
from ..datastructures.world import World
from ..datastructures.world_entity import PhysicalBody
from ..description import ObjectDescription, LinkDescription, Joint
from ..failures import ObjectAlreadyExists, WorldMismatchErrorBetweenAttachedObjects, UnsupportedFileExtension, \
    ObjectDescriptionUndefined
from ..local_transformer import LocalTransformer
from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription
from ..object_descriptors.urdf import ObjectDescription as URDF
from ..ros import logwarn, logerr, Time

try:
    from ..object_descriptors.mjcf import ObjectDescription as MJCF
except ImportError:
    MJCF = None
from ..robot_description import RobotDescriptionManager, RobotDescription
from ..world_concepts.constraints import Attachment
from pycrap.ontologies import PhysicalObject, Joint, \
    Robot, Floor, Location, Bowl, Spoon, Cereal

Link = ObjectDescription.Link


class Object(PhysicalBody):
    """
    Represents a spawned Object in the World.
    """

    tf_prospection_world_prefix: str = "prospection/"
    """
    The prefix for the tf frame of objects in the prospection world.
    """

    extension_to_description_type: Dict[str, Type[ObjectDescription]] = {URDF.get_file_extension(): URDF}
    """
    A dictionary that maps the file extension to the corresponding ObjectDescription type.
    """

    ontology_concept: Type[PhysicalObject] = PhysicalObject

    def __init__(self, name: str, concept: Type[PhysicalObject], path: Optional[str] = None,
                 description: Optional[ObjectDescription] = None,
                 pose: Optional[Pose] = None,
                 world: Optional[World] = None,
                 color: Optional[Color] = None,
                 ignore_cached_files: bool = False,
                 scale_mesh: Optional[float] = None,
                 mesh_transform: Optional[Transform] = None):
        """
        The constructor loads the description file into the given World, if no World is specified the
        :py:attr:`~World.current_world` will be used. It is also possible to load .obj and .stl file into the World.
        The rgba_color parameter is only used when loading .stl or .obj files,
        for URDFs :func:`~Object.set_color` can be used.

        :param name: The name of the object
        :param concept: The type of the object as ontological concept from PyCRAP
        :param path: The path to the source file, if only a filename is provided then the resources directories will be
         searched, it could be None in some cases when for example it is a generic object.
        :param description: The ObjectDescription of the object, this contains the joints and links of the object.
        :param pose: The pose at which the Object should be spawned
        :param world: The World in which the object should be spawned, if no world is specified the
         :py:attr:`~World.current_world` will be used.
        :param color: The rgba_color with which the object should be spawned.
        :param ignore_cached_files: If true the file will be spawned while ignoring cached files.
        :param scale_mesh: The scale of the mesh.
        """

        self.world = world if world is not None else World.current_world
        self.name: str = name
        super().__init__(-1, self.world, concept=concept)

        pose = Pose() if pose is None else pose

        self.path: Optional[str] = path

        self._resolve_description(path, description)
        self.cache_manager = self.world.cache_manager

        self.local_transformer = LocalTransformer()
        self.original_pose = self.local_transformer.transform_pose(pose, "map")
        self._current_pose = self.original_pose
        self.scale_mesh = scale_mesh if scale_mesh is not None else 1.0
        color = Color() if color is None else color

        if path is not None:
            self.path = self.world.preprocess_object_file_and_get_its_cache_path(path, ignore_cached_files,
                                                                                 self.description, self.name,
                                                                                 scale_mesh=self.scale_mesh,
                                                                                 mesh_transform=mesh_transform,
                                                                                 color=color)

            self.description.update_description_from_file(self.path)
        # if the object is an agent in the belief state
        if self.is_a_robot and not self.world.is_prospection_world:
            self._update_world_robot_and_description()

        self.id = self._spawn_object_and_get_id()

        self._init_joint_name_and_id_map()
        self._init_link_name_and_id_map()

        self._init_links_and_update_transforms()

        if color is not None:
            self.color: Color = color

        self._init_joints()

        self.attachments: Dict[Object, Attachment] = {}

        self.world.add_object(self)

    @property
    def parts(self) -> Dict[str, PhysicalBody]:
        return self.links

    @property
    def tf_frame(self) -> str:
        """
        The tf frame of the object.
        """
        return (self.tf_prospection_world_prefix if self.world.is_prospection_world else "") + self.name

    @property
    def color(self) -> Union[Color, Dict[str, Color]]:
        """
        Return the rgba_color of this object. The return is either:

            1. A Color object with RGBA values, this is the case if the object only has one link (this
                happens for example if the object is spawned from a .obj or .stl file)
            2. A dict with the link name as key and the rgba_color as value. The rgba_color is given as a Color Object.
                Please keep in mind that not every link may have a rgba_color. This is dependent on the URDF from which
                 the object is spawned.

        :return: The rgba_color as Color object with RGBA values between 0 and 1 or a dict with the link name as key and
         the rgba_color as value.
        """
        link_to_color_dict = self.links_colors

        if len(link_to_color_dict) == 1:
            return list(link_to_color_dict.values())[0]
        else:
            return link_to_color_dict

    @color.setter
    def color(self, rgba_color: Union[Color, Dict[str, Color]]) -> None:
        """
        Change the color of this object.

        :param rgba_color: The color as Color object with RGBA values between 0 and 1
        """
        # Check if there is only one link, this is the case for primitive
        # forms or if loaded from an .stl or .obj file
        if self.has_one_link:
            self.root_link.color = rgba_color
        else:
            if isinstance(rgba_color, Color):
                for link in self.links.values():
                    link.color = rgba_color
            else:
                for link_name, color in rgba_color.items():
                    self.links[link_name].color = color

    def get_mesh_path(self) -> List[str]:
        """
        Get the path to the mesh file of the object.

        :return: The path(s) to the mesh file(s).
        """
        if self.has_one_link:
            return self.root_link.get_mesh_path()
        else:
            raise ValueError("The object has more than one link, therefore the mesh path cannot be determined.")

    def _resolve_description(self, path: Optional[str] = None, description: Optional[ObjectDescription] = None) -> None:
        """
        Find the correct description type of the object and initialize it and set the description of this object to it.

        :param path: The path to the source file.
        :param description: The ObjectDescription of the object.
        """
        if description is not None:
            self.description = description
            return
        if path is None:
            raise ObjectDescriptionUndefined(self.name)
        extension = Path(path).suffix
        if extension in self.extension_to_description_type:
            self.description = self.extension_to_description_type[extension]()
        elif extension in ObjectDescription.mesh_extensions:
            self.description = self.world.conf.default_description_type()
        else:
            raise UnsupportedFileExtension(self.name, path)

    def set_mobile_robot_pose(self, pose: Pose) -> None:
        """
        Set the goal for the mobile base joints of a mobile robot to reach a target pose. This is used for example when
        the simulator does not support setting the pose of the robot directly (e.g. MuJoCo).

        :param pose: The target pose.
        """
        goal = self.get_mobile_base_joint_goal(pose)
        goal = {vj.name: pos for vj, pos in goal.items()}
        self.set_multiple_joint_positions(goal)

    def get_mobile_base_joint_goal(self, pose: Pose) -> Dict[VirtualJoint, float]:
        """
        Get the goal for the mobile base joints of a mobile robot to reach a target pose.

        :param pose: The target pose.
        :return: The goal for the mobile base joints.
        """
        # target_translation, target_angle = self.get_mobile_base_pose_difference(pose)
        # Get the joints of the base link
        mobile_base_joints = self.world.get_robot_mobile_base_joints()
        return {mobile_base_joints.translation_x: pose.position.x,
                mobile_base_joints.translation_y: pose.position.y,
                mobile_base_joints.angular_z: pose.z_angle}

    def get_mobile_base_pose_difference(self, pose: Pose) -> Tuple[Point, float]:
        """
        Get the difference between the current and the target pose of the mobile base.

        :param pose: The target pose.
        :return: The difference between the current and the target pose of the mobile base.
        """
        return self.original_pose.get_position_diff(pose), self.original_pose.get_z_angle_difference(pose)

    @property
    def joint_actuators(self) -> Optional[Dict[str, str]]:
        """
        The joint actuators of the robot.
        """
        if self.obj_type == ObjectType.ROBOT:
            return self.robot_description.joint_actuators
        return None

    @property
    def has_actuators(self) -> bool:
        """
        True if the object has actuators, otherwise False.
        """
        return self.robot_description.has_actuators

    @property
    def robot_description(self) -> RobotDescription:
        """
        The current robot description.
        """
        return self.world.robot_description

    def get_actuator_for_joint(self, joint: Joint) -> Optional[str]:
        """
        Get the actuator name for a joint.

        :param joint: The joint object for which to get the actuator.
        :return: The name of the actuator.
        """
        return self.robot_description.get_actuator_for_joint(joint.name)

    def get_multiple_link_positions(self, links: List[Link]) -> Dict[str, List[float]]:
        """
        Get the positions of multiple links of the object.

        :param links: The link objects of which to get the positions.
        :return: The positions of the links.
        """
        return self.world.get_multiple_link_positions(links)

    def get_multiple_link_orientations(self, links: List[Link]) -> Dict[str, List[float]]:
        """
        Get the orientations of multiple links of the object.

        :param links: The link objects of which to get the orientations.
        :return: The orientations of the links.
        """
        return self.world.get_multiple_link_orientations(links)

    def get_multiple_link_poses(self, links: List[Link]) -> Dict[str, Pose]:
        """
        Get the poses of multiple links of the object.

        :param links: The link objects of which to get the poses.
        :return: The poses of the links.
        """
        return self.world.get_multiple_link_poses(links)

    def get_poses_of_attached_objects(self) -> Dict[Object, Pose]:
        """
        Get the poses of the attached objects.

        :return: The poses of the attached objects
        """
        return {child_object: attachment.get_child_object_pose()
                for child_object, attachment in self.attachments.items() if not attachment.loose}

    def get_target_poses_of_attached_objects_given_parent(self, pose: Pose) -> Dict[Object, Pose]:
        """
        Get the target poses of the attached objects of an object. Given the pose of the parent object. (i.e. the poses
         to which the attached objects will move when the parent object is at the given pose)

        :param pose: The pose of the parent object.
        :return: The target poses of the attached objects
        """
        return {child_object: attachment.get_child_object_pose_given_parent(pose) for child_object, attachment
                in self.attachments.items() if not attachment.loose}

    @property
    def name(self):
        """
        The name of the object.
        """
        return self._name

    @name.setter
    def name(self, name: str):
        """
        Set the name of the object.
        """
        self._name = name
        if name in [obj.name for obj in self.world.objects]:
            raise ObjectAlreadyExists(self)

    @property
    def pose(self):
        """
        The current pose of the object.
        """
        return self.world.get_object_pose(self)

    @pose.setter
    def pose(self, pose: Pose):
        """
        Set the pose of the object.
        """
        self.set_pose(pose)

    @property
    def transform(self):
        """
        The current transform of the object.
        """
        return self.get_pose().to_transform(self.tf_frame)

    @property
    def obj_type(self) -> Type[PhysicalObject]:
        return self.ontology_concept

    def _spawn_object_and_get_id(self) -> int:
        """
        Loads an object to the given World with the given position and orientation. The rgba_color will only be
        used when an .obj or .stl file is given.
        If a .obj or .stl file is given, before spawning, an urdf file with the .obj or .stl as mesh will be created
        and this URDf file will be loaded instead.
        When spawning a URDf file a new file will be created in the cache directory, if there exists none.
        This new file will have resolved mesh file paths, meaning there will be no references
        to ROS packages instead there will be absolute file paths.

        :return: The unique id of the object and the path of the file that was loaded.
        """
        if isinstance(self.description, GenericObjectDescription):
            return self.world.load_generic_object_and_get_id(self.description, pose=self._current_pose)

        path = self.path if self.world.conf.let_pycram_handle_spawning else self.name

        try:
            obj_id = self.world.load_object_and_get_id(path, self._current_pose, self.obj_type)
            return obj_id

        except Exception as e:
            logerr(
                f"The caught error: {e}, The File could not be loaded. Please note that the path has to be either"
                f" a URDF, stl or obj file or the name of an URDF string on the parameter server.")
            os.remove(path)
            raise e

    def _update_world_robot_and_description(self):
        """
        Initialize the robot description of the object, load the description from the RobotDescriptionManager and set
        the robot as the current robot in the World. Also add the virtual mobile base joints to the robot.
        """
        rdm = RobotDescriptionManager()
        rdm.load_description(self.description.name)
        World.robot = self
        self._add_virtual_move_base_joints()

    def _add_virtual_move_base_joints(self):
        """
        Add the virtual mobile base joints to the robot description.
        """
        virtual_joints = self.robot_description.virtual_mobile_base_joints
        if virtual_joints is None:
            return
        child_link = self.description.get_root()
        axes = virtual_joints.get_axes()
        for joint_name, joint_type in virtual_joints.get_types().items():
            self.description.add_joint(joint_name, child_link, joint_type, axes[joint_name], is_virtual=True)

    def _init_joint_name_and_id_map(self) -> None:
        """
        Create a dictionary which maps the joint names to their unique ids and vice versa.
        """
        n_joints = len(self.joint_names)
        self.joint_name_to_id = dict(zip(self.joint_names, range(n_joints)))
        self.joint_id_to_name = dict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))

    def _init_link_name_and_id_map(self) -> None:
        """
        Create a dictionary which maps the link names to their unique ids and vice versa.
        """
        n_links = len(self.link_names)
        self.link_name_to_id: Dict[str, int] = dict(zip(self.link_names, range(n_links)))
        self.link_name_to_id[self.description.get_root()] = -1
        self.link_id_to_name: Dict[int, str] = dict(zip(self.link_name_to_id.values(), self.link_name_to_id.keys()))

    def _init_links_and_update_transforms(self) -> None:
        """
        Initialize the link objects from the URDF file and creates a dictionary which maps the link names to the
        corresponding link objects.
        """
        self.links = {}
        for link_name, link_id in self.link_name_to_id.items():
            link_description = self.description.get_link_by_name(link_name)
            if link_name == self.description.get_root():
                self.links[link_name] = self.description.RootLink(self)
            else:
                self.links[link_name] = self.description.Link(link_id, link_description, self)

        self.update_link_transforms()

    def _init_joints(self):
        """
        Initialize the joint objects from the URDF file and creates a dictionary which mas the joint names to the
        corresponding joint objects
        """
        self.joints = {}
        for joint_name, joint_id in self.joint_name_to_id.items():
            parsed_joint_description = self.description.get_joint_by_name(joint_name)
            is_virtual = self.is_joint_virtual(joint_name)
            self.joints[joint_name] = self.description.Joint(joint_id, parsed_joint_description, self,
                                                             is_virtual=is_virtual)

    def is_joint_virtual(self, name: str):
        """
        Check if a joint is virtual.
        """
        return self.description.is_joint_virtual(name)

    @property
    def virtual_joint_names(self):
        """
        The names of the virtual joints.
        """
        return self.description.virtual_joint_names

    @property
    def virtual_joints(self):
        """
        The virtual joints as a list.
        """
        return [joint for joint in self.joints.values() if joint.is_virtual]

    @property
    def has_one_link(self) -> bool:
        """
        True if the object has only one link, otherwise False.
        """
        return len(self.links) == 1

    @property
    def link_names(self) -> List[str]:
        """
        The names of the links as a list.
        """
        return self.world.get_object_link_names(self)

    @property
    def joint_names(self) -> List[str]:
        """
        The names of the joints as a list.
        """
        joint_names = self.world.get_object_joint_names(self)
        return joint_names

    def get_link(self, link_name: str) -> ObjectDescription.Link:
        """
        Return the link object with the given name.

        :param link_name: The name of the link.
        :return: The link object.
        """
        return self.links[link_name]

    def get_link_pose(self, link_name: str) -> Pose:
        """
        Return the pose of the link with the given name.

        :param link_name: The name of the link.
        :return: The pose of the link.
        """
        return self.links[link_name].pose

    def get_link_position(self, link_name: str) -> Point:
        """
        Return the position of the link with the given name.

        :param link_name: The name of the link.
        :return: The position of the link.
        """
        return self.links[link_name].position

    def get_link_position_as_list(self, link_name: str) -> List[float]:
        """
        Return the position of the link with the given name.

        :param link_name: The name of the link.
        :return: The position of the link.
        """
        return self.links[link_name].position_as_list

    def get_link_orientation(self, link_name: str) -> Quaternion:
        """
        Return the orientation of the link with the given name.

        :param link_name: The name of the link.
        :return: The orientation of the link.
        """
        return self.links[link_name].orientation

    def get_link_orientation_as_list(self, link_name: str) -> List[float]:
        """
        Return the orientation of the link with the given name.

        :param link_name: The name of the link.
        :return: The orientation of the link.
        """
        return self.links[link_name].orientation_as_list

    def get_link_tf_frame(self, link_name: str) -> str:
        """
        Return the tf frame of the link with the given name.

        :param link_name: The name of the link.
        :return: The tf frame of the link.
        """
        return self.links[link_name].tf_frame

    def get_link_axis_aligned_bounding_box(self, link_name: str,
                                           transform_to_link_pose: bool = True) -> AxisAlignedBoundingBox:
        """
        Return the axis aligned bounding box of the link with the given name.

        :param link_name: The name of the link.
        :param transform_to_link_pose: If True, the bounding box will be transformed to fit link pose.
        :return: The axis aligned bounding box of the link.
        """
        return self.links[link_name].get_axis_aligned_bounding_box(transform_to_link_pose)

    def get_transform_between_links(self, from_link: str, to_link: str) -> Transform:
        """
        Return the transform between two links.

        :param from_link: The name of the link from which the transform should be calculated.
        :param to_link: The name of the link to which the transform should be calculated.
        """
        return self.links[from_link].get_transform_to_link(self.links[to_link])

    def get_link_color(self, link_name: str) -> Color:
        """
        Return the color of the link with the given name.

        :param link_name: The name of the link.
        :return: The color of the link.
        """
        return self.links[link_name].color

    def set_link_color(self, link_name: str, color: List[float]) -> None:
        """
        Set the color of the link with the given name.

        :param link_name: The name of the link.
        :param color: The new color of the link.
        """
        self.links[link_name].color = Color.from_list(color)

    def get_link_geometry(self, link_name: str) -> List[VisualShape]:
        """
        Return the collision geometry of the link with the given name.

        :param link_name: The name of the link.
        :return: List of the collision geometry of the link.
        """
        return self.links[link_name].geometry

    def get_link_visual_geometry(self, link_name: str) -> List[VisualShape]:
        """
        Return the visual geometry of the link with the given name.

        :param link_name: The name of the link.
        :return: The visual geometry of the link.
        """
        return self.links[link_name].visual_geometry

    def get_link_transform(self, link_name: str) -> Transform:
        """
        Return the transform of the link with the given name.

        :param link_name: The name of the link.
        :return: The transform of the link.
        """
        return self.links[link_name].transform

    def get_link_origin(self, link_name: str) -> Pose:
        """
        Return the origin of the link with the given name.

        :param link_name: The name of the link.
        :return: The origin of the link as a 'Pose'.
        """
        return self.links[link_name].origin

    def get_link_origin_transform(self, link_name: str) -> Transform:
        """
        Return the origin transform of the link with the given name.

        :param link_name: The name of the link.
        :return: The origin transform of the link.
        """
        return self.links[link_name].origin_transform

    @property
    def base_origin_shift(self) -> np.ndarray:
        """
        The shift between the base of the object and the origin of the object.

        :return: A numpy array with the shift between the base of the object and the origin of the object.
        """
        return np.array(self.get_position_as_list()) - np.array(self.get_base_position_as_list())

    def __repr__(self):
        return self.__class__.__qualname__ + (f"(name={self.name}, object_type={self.obj_type.name},"
                                              f" file_path={self.path}, pose={self.pose}, world={self.world})")

    def remove(self) -> None:
        """
        Remove this object from the World it currently resides in.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. Then remove this Object from the simulation/world.
        """
        self.world.remove_object(self)

    def reset(self, remove_saved_states=False) -> None:
        """
        Reset the Object to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and the object will be set to the position and
        orientation in which it was spawned.

        :param remove_saved_states: If True the saved states will be removed.
        """
        self.reset_concepts()
        self.detach_all()
        self.reset_all_links()
        self.reset_all_joints()
        self.set_pose(self.original_pose)
        if remove_saved_states:
            self.remove_saved_states()

    def reset_all_joints(self) -> None:
        """
        Reset all joints of the object.
        """
        for joint in self.joints.values():
            joint.reset_concepts()
        self.reset_all_joints_positions()

    def reset_all_links(self) -> None:
        """
        Reset all links of the object.
        """
        for link in self.links.values():
            link.reset()

    def merge(self, other: Object, name: Optional[str] = None, pose: Optional[Pose] = None,
              new_description_file: Optional[str] = None) -> Object:
        """
        Merge the object with another object. This is done by merging the descriptions of the objects,
        removing the original objects creating a new merged object.

        :param other: The object to merge with.
        :param name: The name of the merged object.
        :param pose: The pose of the merged object.
        :param new_description_file: The new description file of the merged object.
        :return: The merged object.
        """
        pose = self.pose if pose is None else pose
        child_pose = self.local_transformer.transform_pose(other.pose, self.tf_frame)
        description = self.description.merge_description(other.description, child_pose_wrt_parent=child_pose,
                                                         new_description_file=new_description_file)
        name = self.name if name is None else name
        color = self.color if isinstance(self.color, Color) else self.color[self.root_link.name]
        other.remove()
        self.remove()
        return Object(name, self.obj_type, description.xml_path, description=description, pose=pose, world=self.world,
                      color=color)

    def attach(self,
               child_object: Object,
               parent_link: Optional[str] = None,
               child_link: Optional[str] = None,
               bidirectional: bool = True,
               coincide_the_objects: bool = False,
               parent_to_child_transform: Optional[Transform] = None) -> None:
        """
        Attach another object to this object. This is done by
        saving the transformation between the given link, if there is one, and
        the base pose of the other object. Additionally, the name of the link, to
        which the object is attached, will be saved.
        Furthermore, a simulator constraint will be created so the attachment
        also works while simulation.
        Loose attachments means that the attachment will only be one-directional. For example, if this object moves the
        other, attached, object will also move but not the other way around.

        :param child_object: The other object that should be attached.
        :param parent_link: The link name of this object.
        :param child_link: The link name of the other object.
        :param bidirectional: If the attachment should be a loose attachment.
        :param coincide_the_objects: If True the object frames will be coincided.
        :param parent_to_child_transform: The transform from the parent to the child object.
        """
        parent_link = self.links[parent_link] if parent_link else self.root_link
        child_link = child_object.links[child_link] if child_link else child_object.root_link

        if coincide_the_objects and parent_to_child_transform is None:
            parent_to_child_transform = Transform()
        attachment = Attachment(parent_link, child_link, bidirectional, parent_to_child_transform)

        self.attachments[child_object] = attachment
        child_object.attachments[self] = attachment.get_inverse()

        self.world.attachment_event(self, [self, child_object])

    def detach(self, child_object: Object) -> None:
        """
        Detache another object from this object. This is done by
        deleting the attachment from the attachments dictionary of both objects
        and deleting the constraint of the simulator.
        Afterward the detachment event of the corresponding World will be fired.

        :param child_object: The object which should be detached
        """
        del self.attachments[child_object]
        del child_object.attachments[self]

        self.world.detachment_event(self, [self, child_object])

    def detach_all(self) -> None:
        """
        Detach all objects attached to this object.
        """
        attachments = self.attachments.copy()
        for att in attachments.keys():
            self.detach(att)

    def update_attachment_with_object(self, child_object: Object):
        self.attachments[child_object].update_transform_and_constraint()

    def get_position(self) -> Point:
        """
        Return the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_pose().position

    def get_orientation(self) -> Pose.orientation:
        """
        Return the orientation of this object as a list of xyzw, representing a quaternion.

        :return: A list of xyzw
        """
        return self.get_pose().orientation

    def get_position_as_list(self) -> List[float]:
        """
        Return the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_pose().position_as_list()

    def get_base_position_as_list(self) -> List[float]:
        """
        Return the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_base_origin().position_as_list()

    def get_orientation_as_list(self) -> List[float]:
        """
        Return the orientation of this object as a list of xyzw, representing a quaternion.

        :return: A list of xyzw
        """
        return self.get_pose().orientation_as_list()

    def get_pose(self) -> Pose:
        """
        Return the position of this object as a list of xyz. Alias for :func:`~Object.get_position`.

        :return: The current pose of this object
        """
        return self.pose

    def set_pose(self, pose: Pose, base: bool = False, set_attachments: bool = True) -> None:
        """
        Set the Pose of the object.

        :param pose: New Pose for the object
        :param base: If True places the object base instead of origin at the specified position and orientation
        :param set_attachments: Whether to set the poses of the attached objects to this object or not.
        """
        pose_in_map = self.local_transformer.transform_pose(pose, "map")
        if base:
            pose_in_map.position = (np.array(pose_in_map.position_as_list()) + self.base_origin_shift).tolist()

        self.reset_base_pose(pose_in_map)

        if set_attachments:
            self._set_attached_objects_poses()

    def reset_base_pose(self, pose: Pose) -> bool:
        return self.world.reset_object_base_pose(self, pose)

    def move_base_to_origin_pose(self) -> None:
        """
        Move the object such that its base will be at the current origin position.
        This is useful when placing objects on surfaces where you want the object base in contact with the surface.
        """
        self.set_pose(self.get_pose(), base=True)

    def save_state(self, state_id: int, save_dir: Optional[str] = None) -> None:
        """
        Save the state of this object by saving the state of all links and attachments.

        :param state_id: The unique id of the state.
        :param save_dir: The directory in which to save the state.
        """
        self.save_links_states(state_id)
        self.save_joints_states(state_id)
        super().save_state(state_id, save_dir)

    def save_links_states(self, state_id: int) -> None:
        """
        Save the state of all links of this object.

        :param state_id: The unique id of the state.
        """
        for link in self.links.values():
            link.save_state(state_id)

    def save_joints_states(self, state_id: int) -> None:
        """
        Save the state of all joints of this object.

        :param state_id: The unique id of the state.
        """
        for joint in self.joints.values():
            joint.save_state(state_id)

    @property
    def current_state(self) -> ObjectState:
        """
        The current state of this object as an ObjectState.
        """
        return ObjectState(self.body_state, self.attachments.copy(), self.link_states.copy(),
                           self.joint_states.copy())

    @current_state.setter
    def current_state(self, state: ObjectState) -> None:
        """
        Set the current state of this object to the given state.
        """
        if self.current_state != state:
            self.body_state = state.body_state
            self.set_attachments(state.attachments)
            self.link_states = state.link_states
            self.joint_states = state.joint_states

    def set_attachments(self, attachments: Dict[Object, Attachment]) -> None:
        """
        Set the attachments of this object to the given attachments.

        :param attachments: A dictionary with the object as key and the attachment as value.
        """
        self.detach_objects_not_in_attachments(attachments)
        self.attach_objects_in_attachments(attachments)

    def detach_objects_not_in_attachments(self, attachments: Dict[Object, Attachment]) -> None:
        """
        Detach objects that are not in the attachments list and are in the current attachments list.

        :param attachments: A dictionary with the object as key and the attachment as value.
        """
        copy_of_attachments = self.attachments.copy()
        for obj, attachment in copy_of_attachments.items():
            original_obj = obj
            if self.world.is_prospection_world and len(attachments) > 0 \
                    and not list(attachments.keys())[0].world.is_prospection_world:
                obj = self.world.get_object_for_prospection_object(obj)
            if obj not in attachments:
                if attachment.is_inverse:
                    original_obj.detach(self)
                else:
                    self.detach(original_obj)

    def attach_objects_in_attachments(self, attachments: Dict[Object, Attachment]) -> None:
        """
        Attach objects that are in the given attachments list but not in the current attachments list.

        :param attachments: A dictionary with the object as key and the attachment as value.
        """
        for obj, attachment in attachments.items():
            is_prospection = self.world.is_prospection_world and not obj.world.is_prospection_world
            if is_prospection:
                obj = self.world.get_prospection_object_for_object(obj)
            if obj in self.attachments:
                if self.attachments[obj] != attachment:
                    if attachment.is_inverse:
                        obj.detach(self)
                    else:
                        self.detach(obj)
                else:
                    continue
            self.mimic_attachment_with_object(attachment, obj)

    def mimic_attachment_with_object(self, attachment: Attachment, child_object: Object) -> None:
        """
        Mimic the given attachment for this and the given child objects.

        :param attachment: The attachment to mimic.
        :param child_object: The child object.
        """
        att_transform = self.get_attachment_transform_with_object(attachment, child_object)
        if attachment.is_inverse:
            child_object.attach(self, attachment.child_link.name, attachment.parent_link.name,
                                attachment.bidirectional,
                                parent_to_child_transform=att_transform.invert())
        else:
            self.attach(child_object, attachment.parent_link.name, attachment.child_link.name,
                        attachment.bidirectional, parent_to_child_transform=att_transform)

    def get_attachment_transform_with_object(self, attachment: Attachment, child_object: Object) -> Transform:
        """
        Return the attachment transform for the given parent and child objects, taking into account the prospection
        world.

        :param attachment: The attachment.
        :param child_object: The child object.
        :return: The attachment transform.
        """
        if self.world != child_object.world:
            raise WorldMismatchErrorBetweenAttachedObjects(self, child_object)
        att_transform = attachment.parent_to_child_transform.copy()
        if self.world.is_prospection_world and not attachment.parent_object.world.is_prospection_world:
            att_transform.frame = self.tf_prospection_world_prefix + att_transform.frame
            att_transform.child_frame_id = self.tf_prospection_world_prefix + att_transform.child_frame_id
        return att_transform

    @property
    def link_states(self) -> Dict[int, LinkState]:
        """
        The current state of all links of this object.

        :return: A dictionary with the link id as key and the current state of the link as value.
        """
        return {link.id: link.current_state for link in self.links.values()}

    @link_states.setter
    def link_states(self, link_states: Dict[int, LinkState]) -> None:
        """
        Set the current state of all links of this object.

        :param link_states: A dictionary with the link id as key and the current state of the link as value.
        """
        for link in self.links.values():
            link.current_state = link_states[link.id]

    @property
    def joint_states(self) -> Dict[int, JointState]:
        """
        The current state of all joints of this object.

        :return: A dictionary with the joint id as key and the current state of the joint as value.
        """
        return {joint.id: joint.current_state for joint in self.joints.values()}

    @joint_states.setter
    def joint_states(self, joint_states: Dict[int, JointState]) -> None:
        """
        Set the current state of all joints of this object.

        :param joint_states: A dictionary with the joint id as key and the current state of the joint as value.
        """
        for joint in self.joints.values():
            if joint.name not in self.robot_virtual_move_base_joints_names():
                joint.current_state = joint_states[joint.id]

    def robot_virtual_move_base_joints_names(self):
        if self.robot_description.virtual_mobile_base_joints is None:
            return []
        return self.robot_description.virtual_mobile_base_joints.names

    def remove_saved_states(self) -> None:
        """
        Remove all saved states of this object.
        """
        super().remove_saved_states()
        self.remove_links_saved_states()
        self.remove_joints_saved_states()

    def remove_links_saved_states(self) -> None:
        """
        Remove all saved states of the links of this object.
        """
        for link in self.links.values():
            link.remove_saved_states()

    def remove_joints_saved_states(self) -> None:
        """
        Remove all saved states of the joints of this object.
        """
        for joint in self.joints.values():
            joint.remove_saved_states()

    def _set_attached_objects_poses(self, already_moved_objects: Optional[List[Object]] = None) -> None:
        """
        Update the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this call _set_attached_objects method for all attached objects.

        :param already_moved_objects: A list of Objects that were already moved, these will be excluded to prevent loops
         in the update.
        """
        if not self.world.conf.let_pycram_move_attached_objects:
            return

        if already_moved_objects is None:
            already_moved_objects = []

        for child in self.attachments:

            if child in already_moved_objects:
                continue

            attachment = self.attachments[child]
            if attachment.loose:
                self.update_attachment_with_object(child)
                child.update_attachment_with_object(self)

            else:
                child.set_pose(attachment.get_child_link_target_pose(), set_attachments=False)
                child._set_attached_objects_poses(already_moved_objects + [self])

    def set_position(self, position: Union[Pose, Point, List], base=False) -> None:
        """
        Set this Object to the given position, if base is true, place the bottom of the Object at the position
        instead of the origin in the center of the Object. The given position can either be a Pose,
        in this case only the position is used or a geometry_msgs.msg/Point which is the position part of a Pose.

        :param position: Target position as xyz.
        :param base: If the bottom of the Object should be placed or the origin in the center.
        """
        pose = Pose()
        if isinstance(position, Pose):
            target_position = position.position
            pose.frame = position.frame
        elif isinstance(position, Point):
            target_position = position
        elif isinstance(position, (List, np.ndarray, tuple)):
            if len(position) == 3:
                target_position = Point(**dict(zip(["x", "y", "z"], position)))
            else:
                raise ValueError("The given position has to be a sequence of 3 values.")
        else:
            raise TypeError("The given position has to be a Pose, Point or a sequence of xyz values.")

        pose.position = target_position
        pose.orientation = self.get_orientation()
        self.set_pose(pose, base=base)

    def set_orientation(self, orientation: Union[Pose, Quaternion, List, Tuple, np.ndarray]) -> None:
        """
        Set the orientation of the Object to the given orientation. Orientation can either be a Pose, in this case only
        the orientation of this pose is used or a geometry_msgs.msg/Quaternion which is the orientation of a Pose.

        :param orientation: Target orientation given as a list of xyzw.
        """
        pose = Pose()
        if isinstance(orientation, Pose):
            target_orientation = orientation.orientation
            pose.frame = orientation.frame
        elif isinstance(orientation, Quaternion):
            target_orientation = orientation
        elif (isinstance(orientation, list) or isinstance(orientation, np.ndarray) or isinstance(orientation, tuple)) \
                and len(orientation) == 4:
            target_orientation = Quaternion(**dict(zip(["x", "y", "z", "w"], orientation)))
        else:
            raise TypeError("The given orientation has to be a Pose, Quaternion or one of list/tuple/ndarray of xyzw.")

        pose.pose.position = self.get_position()
        pose.pose.orientation = target_orientation
        self.set_pose(pose)

    def get_joint_id(self, name: str) -> int:
        """
        Return the unique id for a joint name. As used by the world/simulator.

        :param name: The joint name
        :return: The unique id
        """
        return self.joint_name_to_id[name]

    def get_root_link_description(self) -> LinkDescription:
        """
        Return the root link of the URDF of this object.

        :return: The root link as defined in the URDF of this object.
        """
        for link_description in self.description.links:
            if link_description.name == self.description.get_root():
                return link_description

    @property
    def root_link(self) -> ObjectDescription.Link:
        """
        The root link of this object.

        :return: The root link of this object.
        """
        return self.links[self.description.get_root()]

    @property
    def tip_link(self) -> ObjectDescription.Link:
        """
        The tip link of this object.

        :return: The tip link of this object.
        """
        return self.links[self.description.get_tip()]

    def get_root_link_id(self) -> int:
        """
        Return the unique id of the root link of this object.

        :return: The unique id of the root link of this object.
        """
        return self.get_link_id(self.description.get_root())

    def get_link_id(self, link_name: str) -> int:
        """
        Return a unique id for a link name.

        :param link_name: The name of the link.
        :return: The unique id of the link.
        """
        return self.link_name_to_id[link_name]

    def get_link_by_id(self, link_id: int) -> ObjectDescription.Link:
        """
        Return the link for a given unique link id

        :param link_id: The unique id of the link.
        :return: The link object.
        """
        return self.links[self.link_id_to_name[link_id]]

    def reset_all_joints_positions(self) -> None:
        """
        Set the current position of all joints to 0. This is useful if the joints should be reset to their default
        """
        joint_names = [joint.name for joint in self.joints.values()]
        if len(joint_names) == 0:
            return
        joint_positions = [0] * len(joint_names)
        self.set_multiple_joint_positions(dict(zip(joint_names, joint_positions)))

    def set_joint_position(self, joint_name: str, joint_position: float) -> None:
        """
        Set the position of the given joint to the given joint pose and updates the poses of all attached objects.

        :param joint_name: The name of the joint
        :param joint_position: The target pose for this joint
        """
        self.clip_joint_positions_to_limits({joint_name: joint_position})
        if self.world.reset_joint_position(self.joints[joint_name], joint_position):
            self._set_attached_objects_poses()

    @deprecated("Use set_multiple_joint_positions instead")
    def set_joint_positions(self, joint_positions: Dict[str, float]) -> None:
        self.set_multiple_joint_positions(joint_positions)

    def set_multiple_joint_positions(self, joint_positions: Dict[str, float]) -> None:
        """
        Set the current position of multiple joints at once, this method should be preferred when setting
        multiple joints at once instead of running :func:`~Object.set_joint_position` in a loop.

        :param joint_positions: A dictionary with the joint names as keys and the target positions as values.
        """
        joint_positions = {self.joints[joint_name]: joint_position
                           for joint_name, joint_position in joint_positions.items()}
        if self.world.set_multiple_joint_positions(joint_positions):
            self._set_attached_objects_poses()

    def clip_joint_positions_to_limits(self, joint_positions: Dict[str, float]) -> Dict[str, float]:
        """
        Clip the given joint positions to the joint limits.

        :param joint_positions: A dictionary with the joint names as keys and the target positions as values.
        :return: A dictionary with the joint names as keys and the clipped positions as values.
        """
        return {joint_name: np.clip(joint_position, self.joints[joint_name].lower_limit,
                                    self.joints[joint_name].upper_limit)
        if self.joints[joint_name].has_limits else joint_position
                for joint_name, joint_position in joint_positions.items()}

    def get_joint_position(self, joint_name: str) -> float:
        """
        Return the current position of the given joint.

        :param joint_name: The name of the joint
        :return: The current position of the given joint
        """
        return self.joints[joint_name].position

    def get_joint_damping(self, joint_name: str) -> float:
        """
        Return the damping of the given joint (friction).

        :param joint_name: The name of the joint
        :return: The damping of the given joint
        """
        return self.joints[joint_name].damping

    def get_joint_upper_limit(self, joint_name: str) -> float:
        """
        Return the upper limit of the given joint.

        :param joint_name: The name of the joint
        :return: The upper limit of the given joint
        """
        return self.joints[joint_name].upper_limit

    def get_joint_lower_limit(self, joint_name: str) -> float:
        """
        Return the lower limit of the given joint.

        :param joint_name: The name of the joint
        :return: The lower limit of the given joint
        """
        return self.joints[joint_name].lower_limit

    def get_joint_axis(self, joint_name: str) -> Point:
        """
        Return the axis of the given joint.

        :param joint_name: The name of the joint
        :return: The axis of the given joint
        """
        return self.joints[joint_name].axis

    def get_joint_type(self, joint_name: str) -> JointType:
        """
        Return the type of the given joint.

        :param joint_name: The name of the joint
        :return: The type of the given joint
        """
        return self.joints[joint_name].type

    def get_joint_limits(self, joint_name: str) -> Tuple[float, float]:
        """
        Return the lower and upper limits of the given joint.

        :param joint_name: The name of the joint
        :return: The lower and upper limits of the given joint
        """
        return self.joints[joint_name].limits

    def get_joint_child_link(self, joint_name: str) -> ObjectDescription.Link:
        """
        Return the child link of the given joint.

        :param joint_name: The name of the joint
        :return: The child link of the given joint
        """
        return self.joints[joint_name].child_link

    def get_joint_parent_link(self, joint_name: str) -> ObjectDescription.Link:
        """
        Return the parent link of the given joint.

        :param joint_name: The name of the joint
        :return: The parent link of the given joint
        """
        return self.joints[joint_name].parent_link

    def find_joint_above_link(self, link_name: str, joint_type: Optional[JointType] = None) -> Optional[str]:
        """
        Traverses the chain from 'link' to the URDF origin and returns the first joint that is of type 'joint_type'.
        If no joint type is given, the first joint that is not FIXED is returned.

        :param link_name: AbstractLink name above which the joint should be found
        :param joint_type: Joint type that should be searched for
        :return: Name of the first joint which has the given type
        """
        chain = self.description.get_chain(self.description.get_root(), link_name)
        reversed_chain = reversed(chain)
        for element in reversed_chain:
            if element not in self.joint_name_to_id:
                continue

            element_joint_type = self.get_joint_type(element)
            if joint_type is not None and element_joint_type == joint_type:
                return element

            if joint_type is None and element_joint_type != JointType.FIXED:
                return element

        logwarn(f"No joint of type {joint_type} found above link {link_name}")
        return None

    def get_multiple_joint_positions(self, joint_names: List[str]) -> Dict[str, float]:
        """
        Return the positions of multiple joints at once.

        :param joint_names: A list of joint names.
        :return: A dictionary with the joint names as keys and the joint positions as values.
        """
        return self.world.get_multiple_joint_positions([self.joints[joint_name] for joint_name in joint_names])

    def get_positions_of_controllable_joints(self) -> Dict[str, float]:
        """
        Return a list of all controllable joints of this object.

        :return: A list of all controllable joints.
        """
        return {j.name: j.position for j in self.joints.values()
                if j.type != JointType.FIXED and not j.is_virtual}

    def get_positions_of_all_joints(self) -> Dict[str, float]:
        """
        Return the positions of all joints of the object as a dictionary of joint names and joint positions.

        :return: A dictionary with all joints positions'.
        """
        return {j.name: j.position for j in self.joints.values()}

    def update_link_transforms(self, transform_time: Optional[Time] = None) -> None:
        """
        Update the transforms of all links of this object using time 'transform_time' or the current ros time.

        :param transform_time: The time to use for the transform update.
        """
        for link in self.links.values():
            link.update_transform(transform_time)

    @property
    def contact_points(self) -> ContactPointsList:
        """
        Return a list of contact points of this Object with other Objects.

        :return: A list of all contact points with other objects
        """
        return self.world.get_object_contact_points(self)

    def contact_points_simulated(self) -> ContactPointsList:
        """
        Return a list of all contact points between this Object and other Objects after stepping the simulation once.

        :return: A list of contact points between this Object and other Objects
        """
        state_id = self.world.save_state()
        self.world.step()
        contact_points = self.contact_points
        self.world.restore_state(state_id)
        return contact_points

    def closest_points(self, max_distance: float) -> ClosestPointsList:
        """
        Return a list of closest points between this Object and other Objects.

        :param max_distance: The maximum distance between the closest points
        :return: A list of closest points between this Object and other Objects
        """
        return self.world.get_body_closest_points(self, max_distance)

    def closest_points_with_obj(self, other_object: Object, max_distance: float) -> ClosestPointsList:
        """
        Return a list of closest points between this Object and another Object.

        :param other_object: The other object
        :param max_distance: The maximum distance between the closest points
        :return: A list of closest points between this Object and the other Object
        """
        return self.world.get_closest_points_between_two_bodies(self, other_object, max_distance)

    def set_color(self, rgba_color: Color) -> None:
        self.color = rgba_color

    def get_color(self) -> Union[Color, Dict[str, Color]]:
        return self.color

    @property
    def links_colors(self) -> Dict[str, Color]:
        """
        The color of each link as a dictionary with link names as keys and RGBA colors as values.
        """
        return self.world.get_colors_of_object_links(self)

    def get_axis_aligned_bounding_box(self, shift_to_object_position: bool = True) -> AxisAlignedBoundingBox:
        """
        Return the axis aligned bounding box of this object.

        :param shift_to_object_position: If True, the bounding box will be shifted to the object position.
        :return: The axis aligned bounding box of this object.
        """
        if self.has_one_link:
            return self.root_link.get_axis_aligned_bounding_box(shift_to_object_position)
        else:
            return self.world.get_object_axis_aligned_bounding_box(self)

    def get_rotated_bounding_box(self) -> RotatedBoundingBox:
        """
        Return the rotated bounding box of this object.

        :return: The rotated bounding box of this object.
        """
        if self.has_one_link:
            return self.root_link.get_rotated_bounding_box()
        else:
            return self.world.get_object_rotated_bounding_box(self)

    def get_convex_hull(self) -> Geometry3D:
        """
        Return the convex hull of this object.

        :return: The convex hull of this object.
        """
        if self.has_one_link:
            return self.root_link.get_convex_hull()
        else:
            return self.world.get_body_convex_hull(self)

    def get_base_origin(self) -> Pose:
        """
        Return the origin of the base/bottom of this object.

        :return: the origin of the base/bottom of this object.
        """
        aabb = self.get_axis_aligned_bounding_box()
        base_width = np.absolute(aabb.min_x - aabb.max_x)
        base_length = np.absolute(aabb.min_y - aabb.max_y)
        return Pose([aabb.min_x + base_width / 2, aabb.min_y + base_length / 2, aabb.min_z],
                    self.get_orientation_as_list())

    def get_joint_by_id(self, joint_id: int) -> Joint:
        """
        Return the joint object with the given id.

        :param joint_id: The unique id of the joint.
        :return: The joint object.
        """
        return dict([(joint.id, joint) for joint in self.joints.values()])[joint_id]

    def get_link_for_attached_objects(self) -> Dict[Object, ObjectDescription.Link]:
        """
        Return a dictionary which maps attached object to the link of this object to which the given object is attached.

        :return: The link of this object to which the given object is attached.
        """
        return {obj: attachment.parent_link for obj, attachment in self.attachments.items()}

    def copy_to_prospection(self) -> Object:
        """
        Copy this object to the prospection world.

        :return: The copied object in the prospection world.
        """
        return self.copy_to_world(self.world.prospection_world)

    def copy_to_world(self, world: World) -> Object:
        """
        Copy this object to the given world.

        :param world: The world to which the object should be copied.
        :return: The copied object in the given world.
        """
        obj = Object(self.name, self.obj_type, self.path, self.description, self.original_pose,
                     world, self.color)
        return obj

    def parent_entity(self) -> World:
        """
        :return: The parent of this object which is the world.
        """
        return self.world

    def frozen_copy(self) -> FrozenObject:
        """
        Creates a copied version of this object which contains the information of this object but can not be interacted
        with.

        :return FrozenObject: The copied forzen object.
        """
        frozen_links = {l_name: FrozenLink(l.name, l.pose, l.geometry) for l_name, l in self.links.items()}
        frozen_joints = {j_name: FrozenJoint(j.name, j.type, [j.child], j.parent, j.current_state.position) for j_name, j in self.joints.items()}

        return FrozenObject(self.name, self.obj_type, self.path, self.description, self.pose,
                            frozen_links, frozen_joints)

    def insert(self, session):
        pass

