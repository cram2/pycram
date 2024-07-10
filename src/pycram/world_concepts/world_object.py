from __future__ import annotations

import logging
import os

import numpy as np
import rospy
from geometry_msgs.msg import Point, Quaternion
from typing_extensions import Type, Optional, Dict, Tuple, List, Union

from ..description import ObjectDescription, LinkDescription, Joint
from ..object_descriptors.urdf import ObjectDescription as URDFObject
from ..robot_descriptions import robot_description
from ..datastructures.world import WorldEntity, World
from ..world_concepts.constraints import Attachment
from ..datastructures.dataclasses import (Color, ObjectState, LinkState, JointState,
                                               AxisAlignedBoundingBox, VisualShape)
from ..datastructures.enums import ObjectType, JointType
from ..local_transformer import LocalTransformer
from ..datastructures.pose import Pose, Transform
from ..robot_description import RobotDescriptionManager

Link = ObjectDescription.Link


class Object(WorldEntity):
    """
    Represents a spawned Object in the World.
    """

    prospection_world_prefix: str = "prospection/"
    """
    The ObjectDescription of the object, this contains the name and type of the object as well as the path to the source 
    file.
    """

    def __init__(self, name: str, obj_type: ObjectType, path: str,
                 description: Optional[Type[ObjectDescription]] = URDFObject,
                 pose: Optional[Pose] = None,
                 world: Optional[World] = None,
                 color: Optional[Color] = Color(),
                 ignore_cached_files: Optional[bool] = False):
        """
        The constructor loads the description file into the given World, if no World is specified the
        :py:attr:`~World.current_world` will be used. It is also possible to load .obj and .stl file into the World.
        The rgba_color parameter is only used when loading .stl or .obj files,
        for URDFs :func:`~Object.set_color` can be used.

        :param name: The name of the object
        :param obj_type: The type of the object as an ObjectType enum.
        :param path: The path to the source file, if only a filename is provided then the resources directories will be searched.
        :param description: The ObjectDescription of the object, this contains the joints and links of the object.
        :param pose: The pose at which the Object should be spawned
        :param world: The World in which the object should be spawned, if no world is specified the :py:attr:`~World.current_world` will be used.
        :param color: The rgba_color with which the object should be spawned.
        :param ignore_cached_files: If true the file will be spawned while ignoring cached files.
        """

        super().__init__(-1, world)

        if pose is None:
            pose = Pose()
        if name in [obj.name for obj in self.world.objects]:
            rospy.logerr(f"An object with the name {name} already exists in the world.")
            return None
        self.name: str = name
        self.obj_type: ObjectType = obj_type
        self.color: Color = color
        self.description = description()
        self.cache_manager = self.world.cache_manager

        self.local_transformer = LocalTransformer()
        self.original_pose = self.local_transformer.transform_pose(pose, "map")
        self._current_pose = self.original_pose

        self.id, self.path = self._load_object_and_get_id(path, ignore_cached_files)

        self.description.update_description_from_file(self.path)

        self.tf_frame = ((self.prospection_world_prefix if self.world.is_prospection_world else "")
                         + f"{self.name}")

        self._init_joint_name_and_id_map()
        self._init_link_name_and_id_map()

        self._init_links_and_update_transforms()
        self._init_joints()

        self.attachments: Dict[Object, Attachment] = {}

        if not self.world.is_prospection_world:
            self._add_to_world_sync_obj_queue()

        self.world.objects.append(self)

        if self.obj_type == ObjectType.ROBOT and not self.world.is_prospection_world:
            rdm = RobotDescriptionManager()
            rdm.load_description(self.name)
            World.robot = self

    @property
    def pose(self):
        return self.get_pose()

    @pose.setter
    def pose(self, pose: Pose):
        self.set_pose(pose)

    def _load_object_and_get_id(self, path: Optional[str] = None,
                                ignore_cached_files: Optional[bool] = False) -> Tuple[int, Union[str, None]]:
        """
        Loads an object to the given World with the given position and orientation. The rgba_color will only be
        used when an .obj or .stl file is given.
        If a .obj or .stl file is given, before spawning, an urdf file with the .obj or .stl as mesh will be created
        and this URDf file will be loaded instead.
        When spawning a URDf file a new file will be created in the cache directory, if there exists none.
        This new file will have resolved mesh file paths, meaning there will be no references
        to ROS packges instead there will be absolute file paths.

        :param path: The path to the description file, if None then no file will be loaded, this is useful when the PyCRAM is not responsible for loading the file but another system is.
        :param ignore_cached_files: Whether to ignore files in the cache directory.
        :return: The unique id of the object and the path of the file that was loaded.
        """
        if path is not None:
            try:
                path = self.world.update_cache_dir_with_object(path, ignore_cached_files, self)
            except FileNotFoundError as e:
                logging.error("Could not generate description from file.")
                raise e

        try:
            simulator_object_path = path
            if simulator_object_path is None:
                # This is useful when the object is already loaded in the simulator so it would use its name instead of
                #  its path
                simulator_object_path = self.name
            obj_id = self.world.load_object_and_get_id(simulator_object_path, Pose(self.get_position_as_list(),
                                                                                   self.get_orientation_as_list()))
            return obj_id, path

        except Exception as e:
            logging.error(
                "The File could not be loaded. Please note that the path has to be either a URDF, stl or obj file or"
                " the name of an URDF string on the parameter server.")
            os.remove(path)
            raise e

    def _init_joint_name_and_id_map(self) -> None:
        """
        Creates a dictionary which maps the joint names to their unique ids and vice versa.
        """
        n_joints = len(self.joint_names)
        self.joint_name_to_id = dict(zip(self.joint_names, range(n_joints)))
        self.joint_id_to_name = dict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))

    def _init_link_name_and_id_map(self) -> None:
        """
        Creates a dictionary which maps the link names to their unique ids and vice versa.
        """
        n_links = len(self.link_names)
        self.link_name_to_id: Dict[str, int] = dict(zip(self.link_names, range(n_links)))
        self.link_name_to_id[self.description.get_root()] = -1
        self.link_id_to_name: Dict[int, str] = dict(zip(self.link_name_to_id.values(), self.link_name_to_id.keys()))

    def _init_links_and_update_transforms(self) -> None:
        """
        Initializes the link objects from the URDF file and creates a dictionary which maps the link names to the
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
            joint_description = self.description.get_joint_by_name(joint_name)
            self.joints[joint_name] = self.description.Joint(joint_id, joint_description, self)

    def _add_to_world_sync_obj_queue(self) -> None:
        """
        Adds this object to the objects queue of the WorldSync object of the World.
        """
        self.world.world_sync.add_obj_queue.put(self)

    @property
    def link_names(self) -> List[str]:
        """
        :return: The name of each link as a list.
        """
        return self.world.get_object_link_names(self)

    @property
    def joint_names(self) -> List[str]:
        """
        :return: The name of each joint as a list.
        """
        return self.world.get_object_joint_names(self)

    def get_link(self, link_name: str) -> ObjectDescription.Link:
        """
        Returns the link object with the given name.

        :param link_name: The name of the link.
        :return: The link object.
        """
        return self.links[link_name]

    def get_link_pose(self, link_name: str) -> Pose:
        """
        Returns the pose of the link with the given name.

        :param link_name: The name of the link.
        :return: The pose of the link.
        """
        return self.links[link_name].pose

    def get_link_position(self, link_name: str) -> Point:
        """
        Returns the position of the link with the given name.

        :param link_name: The name of the link.
        :return: The position of the link.
        """
        return self.links[link_name].position

    def get_link_position_as_list(self, link_name: str) -> List[float]:
        """
        Returns the position of the link with the given name.

        :param link_name: The name of the link.
        :return: The position of the link.
        """
        return self.links[link_name].position_as_list

    def get_link_orientation(self, link_name: str) -> Quaternion:
        """
        Returns the orientation of the link with the given name.

        :param link_name: The name of the link.
        :return: The orientation of the link.
        """
        return self.links[link_name].orientation

    def get_link_orientation_as_list(self, link_name: str) -> List[float]:
        """
        Returns the orientation of the link with the given name.

        :param link_name: The name of the link.
        :return: The orientation of the link.
        """
        return self.links[link_name].orientation_as_list

    def get_link_tf_frame(self, link_name: str) -> str:
        """
        Returns the tf frame of the link with the given name.

        :param link_name: The name of the link.
        :return: The tf frame of the link.
        """
        return self.links[link_name].tf_frame

    def get_link_axis_aligned_bounding_box(self, link_name: str) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of the link with the given name.

        :param link_name: The name of the link.
        :return: The axis aligned bounding box of the link.
        """
        return self.links[link_name].get_axis_aligned_bounding_box()

    def get_transform_between_links(self, from_link: str, to_link: str) -> Transform:
        """
        Returns the transform between two links.

        :param from_link: The name of the link from which the transform should be calculated.
        :param to_link: The name of the link to which the transform should be calculated.
        """
        return self.links[from_link].get_transform_to_link(self.links[to_link])

    def get_link_color(self, link_name: str) -> Color:
        """
        Returns the color of the link with the given name.

        :param link_name: The name of the link.
        :return: The color of the link.
        """
        return self.links[link_name].color

    def set_link_color(self, link_name: str, color: List[float]) -> None:
        """
        Sets the color of the link with the given name.

        :param link_name: The name of the link.
        :param color: The new color of the link.
        """
        self.links[link_name].color = Color.from_list(color)

    def get_link_geometry(self, link_name: str) -> Union[VisualShape, None]:
        """
        Returns the geometry of the link with the given name.

        :param link_name: The name of the link.
        :return: The geometry of the link.
        """
        return self.links[link_name].geometry

    def get_link_transform(self, link_name: str) -> Transform:
        """
        Returns the transform of the link with the given name.

        :param link_name: The name of the link.
        :return: The transform of the link.
        """
        return self.links[link_name].transform

    def get_link_origin(self, link_name: str) -> Pose:
        """
        Returns the origin of the link with the given name.

        :param link_name: The name of the link.
        :return: The origin of the link as a 'Pose'.
        """
        return self.links[link_name].origin

    def get_link_origin_transform(self, link_name: str) -> Transform:
        """
        Returns the origin transform of the link with the given name.

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
        skip_attr = ["links", "joints", "description", "attachments"]
        return self.__class__.__qualname__ + f"(" + ', \n'.join(
            [f"{key}={value}" if key not in skip_attr else f"{key}: ..." for key, value in self.__dict__.items()]) + ")"

    def remove(self) -> None:
        """
        Removes this object from the World it currently resides in.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to world remove object is done
        to remove this Object from the simulation/world.
        """
        self.world.remove_object(self)

    def reset(self, remove_saved_states=True) -> None:
        """
        Resets the Object to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and the object will be set to the position and
        orientation in which it was spawned.

        :param remove_saved_states: If True the saved states will be removed.
        """
        self.detach_all()
        self.reset_all_joints_positions()
        self.set_pose(self.original_pose)
        if remove_saved_states:
            self.remove_saved_states()

    def attach(self,
               child_object: Object,
               parent_link: Optional[str] = None,
               child_link: Optional[str] = None,
               bidirectional: Optional[bool] = True) -> None:
        """
        Attaches another object to this object. This is done by
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
        """
        parent_link = self.links[parent_link] if parent_link else self.root_link
        child_link = child_object.links[child_link] if child_link else child_object.root_link

        attachment = Attachment(parent_link, child_link, bidirectional)

        self.attachments[child_object] = attachment
        child_object.attachments[self] = attachment.get_inverse()

        self.world.attachment_event(self, [self, child_object])

    def detach(self, child_object: Object) -> None:
        """
        Detaches another object from this object. This is done by
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
        Returns the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_pose().position

    def get_orientation(self) -> Pose.orientation:
        """
        Returns the orientation of this object as a list of xyzw, representing a quaternion.

        :return: A list of xyzw
        """
        return self.get_pose().orientation

    def get_position_as_list(self) -> List[float]:
        """
        Returns the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_pose().position_as_list()

    def get_base_position_as_list(self) -> List[float]:
        """
        Returns the position of this Object as a list of xyz.

        :return: The current position of this object
        """
        return self.get_base_origin().position_as_list()

    def get_orientation_as_list(self) -> List[float]:
        """
        Returns the orientation of this object as a list of xyzw, representing a quaternion.

        :return: A list of xyzw
        """
        return self.get_pose().orientation_as_list()

    def get_pose(self) -> Pose:
        """
        Returns the position of this object as a list of xyz. Alias for :func:`~Object.get_position`.

        :return: The current pose of this object
        """
        return self._current_pose

    def set_pose(self, pose: Pose, base: Optional[bool] = False, set_attachments: Optional[bool] = True) -> None:
        """
        Sets the Pose of the object.

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

    def reset_base_pose(self, pose: Pose):
        self.world.reset_object_base_pose(self, pose)
        self.update_pose()

    def update_pose(self):
        """
        Updates the current pose of this object from the world, and updates the poses of all links.
        """
        self._current_pose = self.world.get_object_pose(self)
        self._update_all_links_poses()
        self.update_link_transforms()

    def _update_all_links_poses(self):
        """
        Updates the poses of all links by getting them from the simulator.
        """
        for link in self.links.values():
            link._update_pose()

    def move_base_to_origin_pose(self) -> None:
        """
        Move the object such that its base will be at the current origin position.
        This is useful when placing objects on surfaces where you want the object base in contact with the surface.
        """
        self.set_pose(self.get_pose(), base=True)

    def save_state(self, state_id) -> None:
        """
        Saves the state of this object by saving the state of all links and attachments.

        :param state_id: The unique id of the state.
        """
        self.save_links_states(state_id)
        self.save_joints_states(state_id)
        super().save_state(state_id)

    def save_links_states(self, state_id: int) -> None:
        """
        Saves the state of all links of this object.

        :param state_id: The unique id of the state.
        """
        for link in self.links.values():
            link.save_state(state_id)

    def save_joints_states(self, state_id: int) -> None:
        """
        Saves the state of all joints of this object.

        :param state_id: The unique id of the state.
        """
        for joint in self.joints.values():
            joint.save_state(state_id)

    @property
    def current_state(self) -> ObjectState:
        return ObjectState(self.get_pose().copy(), self.attachments.copy(), self.link_states.copy(), self.joint_states.copy())

    @current_state.setter
    def current_state(self, state: ObjectState) -> None:
        if self.get_pose().dist(state.pose) != 0.0:
            self.set_pose(state.pose, base=False, set_attachments=False)

        self.set_attachments(state.attachments)
        self.link_states = state.link_states
        self.joint_states = state.joint_states

    def set_attachments(self, attachments: Dict[Object, Attachment]) -> None:
        """
        Sets the attachments of this object to the given attachments.

        :param attachments: A dictionary with the object as key and the attachment as value.
        """
        for obj, attachment in attachments.items():
            if self.world.is_prospection_world and not obj.world.is_prospection_world:
                # In case this object is in the prospection world and the other object is not, the attachment will no
                # be set.
                continue
            if obj in self.attachments:
                if self.attachments[obj] != attachment:
                    self.detach(obj)
                else:
                    continue
            self.attach(obj, attachment.parent_link.name, attachment.child_link.name,
                        attachment.bidirectional)

    @property
    def link_states(self) -> Dict[int, LinkState]:
        """
        Returns the current state of all links of this object.

        :return: A dictionary with the link id as key and the current state of the link as value.
        """
        return {link.id: link.current_state for link in self.links.values()}

    @link_states.setter
    def link_states(self, link_states: Dict[int, LinkState]) -> None:
        """
        Sets the current state of all links of this object.

        :param link_states: A dictionary with the link id as key and the current state of the link as value.
        """
        for link in self.links.values():
            link.current_state = link_states[link.id]

    @property
    def joint_states(self) -> Dict[int, JointState]:
        """
        Returns the current state of all joints of this object.

        :return: A dictionary with the joint id as key and the current state of the joint as value.
        """
        return {joint.id: joint.current_state for joint in self.joints.values()}

    @joint_states.setter
    def joint_states(self, joint_states: Dict[int, JointState]) -> None:
        """
        Sets the current state of all joints of this object.

        :param joint_states: A dictionary with the joint id as key and the current state of the joint as value.
        """
        for joint in self.joints.values():
            joint.current_state = joint_states[joint.id]

    def remove_saved_states(self) -> None:
        """
        Removes all saved states of this object.
        """
        super().remove_saved_states()
        self.remove_links_saved_states()
        self.remove_joints_saved_states()

    def remove_links_saved_states(self) -> None:
        """
        Removes all saved states of the links of this object.
        """
        for link in self.links.values():
            link.remove_saved_states()

    def remove_joints_saved_states(self) -> None:
        """
        Removes all saved states of the joints of this object.
        """
        for joint in self.joints.values():
            joint.remove_saved_states()

    def _set_attached_objects_poses(self, already_moved_objects: Optional[List[Object]] = None) -> None:
        """
        Updates the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this the _set_attached_objects method of all attached objects
        will be called.

        :param already_moved_objects: A list of Objects that were already moved, these will be excluded to prevent loops in the update.
        """

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
                link_to_object = attachment.parent_to_child_transform
                child.set_pose(link_to_object.to_pose(), set_attachments=False)
                child._set_attached_objects_poses(already_moved_objects + [self])

    def set_position(self, position: Union[Pose, Point, List], base=False) -> None:
        """
        Sets this Object to the given position, if base is true the bottom of the Object will be placed at the position
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
        elif isinstance(position, list):
            target_position = position
        else:
            raise TypeError("The given position has to be a Pose, Point or a list of xyz.")

        pose.position = target_position
        pose.orientation = self.get_orientation()
        self.set_pose(pose, base=base)

    def set_orientation(self, orientation: Union[Pose, Quaternion, List, Tuple, np.ndarray]) -> None:
        """
        Sets the orientation of the Object to the given orientation. Orientation can either be a Pose, in this case only
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
            target_orientation = Quaternion(*orientation)
        else:
            raise TypeError("The given orientation has to be a Pose, Quaternion or one of list/tuple/ndarray of xyzw.")

        pose.pose.position = self.get_position()
        pose.pose.orientation = target_orientation
        self.set_pose(pose)

    def get_joint_id(self, name: str) -> int:
        """
        Returns the unique id for a joint name. As used by the world/simulator.

        :param name: The joint name
        :return: The unique id
        """
        return self.joint_name_to_id[name]

    def get_root_link_description(self) -> LinkDescription:
        """
        Returns the root link of the URDF of this object.

        :return: The root link as defined in the URDF of this object.
        """
        for link_description in self.description.links:
            if link_description.name == self.root_link_name:
                return link_description

    @property
    def root_link(self) -> ObjectDescription.Link:
        """
        Returns the root link of this object.

        :return: The root link of this object.
        """
        return self.links[self.description.get_root()]

    @property
    def root_link_name(self) -> str:
        """
        Returns the name of the root link of this object.

        :return: The name of the root link of this object.
        """
        return self.description.get_root()

    def get_root_link_id(self) -> int:
        """
        Returns the unique id of the root link of this object.

        :return: The unique id of the root link of this object.
        """
        return self.get_link_id(self.description.get_root())

    def get_link_id(self, link_name: str) -> int:
        """
        Returns a unique id for a link name.

        :param link_name: The name of the link.
        :return: The unique id of the link.
        """
        return self.link_name_to_id[link_name]

    def get_link_by_id(self, link_id: int) -> ObjectDescription.Link:
        """
        Returns the link for a given unique link id

        :param link_id: The unique id of the link.
        :return: The link object.
        """
        return self.links[self.link_id_to_name[link_id]]

    def reset_all_joints_positions(self) -> None:
        """
        Sets the current position of all joints to 0. This is useful if the joints should be reset to their default
        """
        joint_names = list(self.joint_name_to_id.keys())
        joint_positions = [0] * len(joint_names)
        self.set_joint_positions(dict(zip(joint_names, joint_positions)))

    def set_joint_positions(self, joint_poses: dict) -> None:
        """
        Sets the current position of multiple joints at once, this method should be preferred when setting
        multiple joints at once instead of running :func:`~Object.set_joint_position` in a loop.

        :param joint_poses:
        """
        for joint_name, joint_position in joint_poses.items():
            self.joints[joint_name].position = joint_position
        # self.update_pose()
        self._update_all_links_poses()
        self.update_link_transforms()
        self._set_attached_objects_poses()

    def set_joint_position(self, joint_name: str, joint_position: float) -> None:
        """
        Sets the position of the given joint to the given joint pose and updates the poses of all attached objects.

        :param joint_name: The name of the joint
        :param joint_position: The target pose for this joint
        """
        self.joints[joint_name].position = joint_position
        self._update_all_links_poses()
        self.update_link_transforms()
        self._set_attached_objects_poses()

    def get_joint_position(self, joint_name: str) -> float:
        """
        :param joint_name: The name of the joint
        :return: The current position of the given joint
        """
        return self.joints[joint_name].position

    def get_joint_damping(self, joint_name: str) -> float:
        """
        :param joint_name: The name of the joint
        :return: The damping of the given joint
        """
        return self.joints[joint_name].damping

    def get_joint_upper_limit(self, joint_name: str) -> float:
        """
        :param joint_name: The name of the joint
        :return: The upper limit of the given joint
        """
        return self.joints[joint_name].upper_limit

    def get_joint_lower_limit(self, joint_name: str) -> float:
        """
        :param joint_name: The name of the joint
        :return: The lower limit of the given joint
        """
        return self.joints[joint_name].lower_limit

    def get_joint_axis(self, joint_name: str) -> Point:
        """
        :param joint_name: The name of the joint
        :return: The axis of the given joint
        """
        return self.joints[joint_name].axis

    def get_joint_type(self, joint_name: str) -> JointType:
        """
        :param joint_name: The name of the joint
        :return: The type of the given joint
        """
        return self.joints[joint_name].type

    def get_joint_limits(self, joint_name: str) -> Tuple[float, float]:
        """
        :param joint_name: The name of the joint
        :return: The lower and upper limits of the given joint
        """
        return self.joints[joint_name].limits

    def get_joint_child_link(self, joint_name: str) -> ObjectDescription.Link:
        """
        :param joint_name: The name of the joint
        :return: The child link of the given joint
        """
        return self.joints[joint_name].child_link

    def get_joint_parent_link(self, joint_name: str) -> ObjectDescription.Link:
        """
        :param joint_name: The name of the joint
        :return: The parent link of the given joint
        """
        return self.joints[joint_name].parent_link

    def find_joint_above_link(self, link_name: str, joint_type: JointType) -> str:
        """
        Traverses the chain from 'link' to the URDF origin and returns the first joint that is of type 'joint_type'.

        :param link_name: AbstractLink name above which the joint should be found
        :param joint_type: Joint type that should be searched for
        :return: Name of the first joint which has the given type
        """
        chain = self.description.get_chain(self.description.get_root(), link_name)
        reversed_chain = reversed(chain)
        container_joint = None
        for element in reversed_chain:
            if element in self.joint_name_to_id and self.get_joint_type(element) == joint_type:
                container_joint = element
                break
        if not container_joint:
            rospy.logwarn(f"No joint of type {joint_type} found above link {link_name}")
        return container_joint

    def get_positions_of_all_joints(self) -> Dict[str, float]:
        """
        Returns the positions of all joints of the object as a dictionary of joint names and joint positions.

        :return: A dictionary with all joints positions'.
        """
        return {j.name: j.position for j in self.joints.values()}

    def update_link_transforms(self, transform_time: Optional[rospy.Time] = None) -> None:
        """
        Updates the transforms of all links of this object using time 'transform_time' or the current ros time.
        """
        for link in self.links.values():
            link.update_transform(transform_time)

    def contact_points(self) -> List:
        """
        Returns a list of contact points of this Object with other Objects.

        :return: A list of all contact points with other objects
        """
        return self.world.get_object_contact_points(self)

    def contact_points_simulated(self) -> List:
        """
        Returns a list of all contact points between this Object and other Objects after stepping the simulation once.

        :return: A list of contact points between this Object and other Objects
        """
        state_id = self.world.save_state()
        self.world.step()
        contact_points = self.contact_points()
        self.world.restore_state(state_id)
        return contact_points

    def set_color(self, rgba_color: Color) -> None:
        """
        Changes the color of this object, the color has to be given as a list
        of RGBA values.

        :param rgba_color: The color as Color object with RGBA values between 0 and 1
        """
        # Check if there is only one link, this is the case for primitive
        # forms or if loaded from an .stl or .obj file
        if self.links != {}:
            for link in self.links.values():
                link.color = rgba_color
        else:
            self.root_link.color = rgba_color

    def get_color(self) -> Union[Color, Dict[str, Color]]:
        """
        This method returns the rgba_color of this object. The return is either:

            1. A Color object with RGBA values, this is the case if the object only has one link (this
                happens for example if the object is spawned from a .obj or .stl file)
            2. A dict with the link name as key and the rgba_color as value. The rgba_color is given as a Color Object.
                Please keep in mind that not every link may have a rgba_color. This is dependent on the URDF from which
                 the object is spawned.

        :return: The rgba_color as Color object with RGBA values between 0 and 1 or a dict with the link name as key and the rgba_color as value.
        """
        link_to_color_dict = self.links_colors

        if len(link_to_color_dict) == 1:
            return list(link_to_color_dict.values())[0]
        else:
            return link_to_color_dict

    @property
    def links_colors(self) -> Dict[str, Color]:
        """
        The color of each link as a dictionary with link names as keys and RGBA colors as values.
        """
        return self.world.get_colors_of_object_links(self)

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis aligned bounding box of this object.
        """
        return self.world.get_object_axis_aligned_bounding_box(self)

    def get_base_origin(self) -> Pose:
        """
        :return: the origin of the base/bottom of this object.
        """
        aabb = self.get_axis_aligned_bounding_box()
        base_width = np.absolute(aabb.min_x - aabb.max_x)
        base_length = np.absolute(aabb.min_y - aabb.max_y)
        return Pose([aabb.min_x + base_width / 2, aabb.min_y + base_length / 2, aabb.min_z],
                    self.get_orientation_as_list())

    def get_joint_by_id(self, joint_id: int) -> Joint:
        """
        Returns the joint object with the given id.

        :param joint_id: The unique id of the joint.
        :return: The joint object.
        """
        return dict([(joint.id, joint) for joint in self.joints.values()])[joint_id]

    def copy_to_prospection(self) -> Object:
        """
        Copies this object to the prospection world.

        :return: The copied object in the prospection world.
        """
        obj = Object(self.name, self.obj_type, self.path, type(self.description), self.get_pose(),
                     self.world.prospection_world, self.color)
        obj.current_state = self.current_state
        return obj

    def __copy__(self) -> Object:
        """
        Returns a copy of this object. The copy will have the same name, type, path, description, pose, world and color.

        :return: A copy of this object.
        """
        obj = Object(self.name, self.obj_type, self.path, type(self.description), self.get_pose(),
                     self.world.prospection_world, self.color)
        obj.current_state = self.current_state
        return obj

    def __eq__(self, other):
        if not isinstance(other, Object):
            return False
        return (self.id == other.id and self.world == other.world and self.name == other.name
                and self.obj_type == other.obj_type)

    def __hash__(self):
        return hash((self.name, self.obj_type, self.id, self.world.id))
