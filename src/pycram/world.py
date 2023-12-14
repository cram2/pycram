# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import logging
import os
import pathlib
import re
import threading
import time
import xml.etree.ElementTree
from queue import Queue
import tf
from typing import List, Optional, Dict, Tuple, Callable
from typing import Union

import numpy as np
import rospkg
import rospy

import urdf_parser_py.urdf
from geometry_msgs.msg import Quaternion, Point
from urdf_parser_py.urdf import URDF, Collision, GeometricType

from .event import Event
from .robot_descriptions import robot_description
from .enums import JointType, ObjectType
from .local_transformer import LocalTransformer
from sensor_msgs.msg import JointState

from .pose import Pose, Transform

from abc import ABC, abstractmethod
from dataclasses import dataclass
from .world_dataclasses import Color, Constraint, AxisAlignedBoundingBox


@dataclass
class WorldState:
    state_id: int
    attachments: Dict[Object, Dict[Object, Attachment]]
    constraint_ids: Dict[Object, Dict[Object, int]]


class World(ABC):
    """
    The World Class represents the physics Simulation and belief state.
    """

    current_world: World = None
    """
        Global reference to the currently used World, usually this is the
        graphical one. However, if you are inside a Use_shadow_world() environment the current_world points to the
        shadow world. In this way you can comfortably use the current_world, which should point towards the World
        used at the moment.
    """

    robot: Object = None
    """
    Global reference to the spawned Object that represents the robot. The robot is identified by checking the name in the 
    URDF with the name of the URDF on the parameter server. 
    """

    data_directory: List[str] = [os.path.dirname(__file__) + "/../../resources"]
    """
    Global reference for the data directories, this is used to search for the URDF files of the robot and the objects.
    """

    simulation_time_step: float = None
    """
    Global reference for the simulation time step, this is used to calculate the frequency of the simulation,
    and also for calculating the equivalent real time for the simulation.
    """

    def __init__(self, mode: str, is_prospection_world: bool, simulation_time_step: float):
        """
       Creates a new simulation, the mode decides if the simulation should be a rendered window or just run in the
       background. There can only be one rendered simulation.
       The World object also initializes the Events for attachment, detachment and for manipulating the world.

       :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default parameter is "GUI"
       :param is_prospection_world: For internal usage, decides if this World should be used as a prospection world.
        """

        self._init_current_world_and_simulation_time_step(simulation_time_step)
        # init the current world and simulation time step which are class variables.

        self.objects: List[Object] = []
        self.client_id: int = -1
        self.mode: str = mode

        self._init_events()
        self.coll_callbacks: Dict[Tuple[Object, Object], Tuple[Callable, Callable]] = {}
        self.local_transformer = LocalTransformer()

        self.is_prospection_world: bool = is_prospection_world
        if is_prospection_world:  # then no need to add another prospection world
            self.prospection_world = None
            self.world_sync = None
        else:  # a normal world should have a synced prospection world
            self._init_and_sync_prospection_world()
            self._update_local_transformer_worlds()

        self._saved_states: Dict[int, WorldState] = {}
        # Different states of the world indexed by int state id.

    def _init_current_world_and_simulation_time_step(self, simulation_time_step):
        World.current_world = self if World.current_world is None else World.current_world
        World.simulation_time_step = simulation_time_step

    def _init_events(self):
        self.detachment_event: Event = Event()
        self.attachment_event: Event = Event()
        self.manipulation_event: Event = Event()

    def _init_and_sync_prospection_world(self):
        self._init_prospection_world()
        self._sync_prospection_world()

    def _update_local_transformer_worlds(self):
        self.local_transformer.world = self
        self.local_transformer.prospection_world = self.prospection_world

    @classmethod
    def _init_prospection_world(cls):
        cls.prospection_world: World = cls("DIRECT", True, cls.simulation_time_step)

    def _sync_prospection_world(self):
        self.world_sync: WorldSync = WorldSync(self, self.prospection_world)
        self.world_sync.start()

    @property
    def simulation_frequency(self):
        return int(1/World.simulation_time_step)

    @abstractmethod
    def load_urdf_at_pose_and_get_object_id(self, path: str, pose: Pose) -> int:
        pass

    def get_objects_by_name(self, name: str) -> List[Object]:
        """
        Returns a list of all Objects in this World with the same name as the given one.

        :param name: The name of the returned Objects.
        :return: A list of all Objects with the name 'name'.
        """
        return list(filter(lambda obj: obj.name == name, self.objects))

    def get_objects_by_type(self, obj_type: str) -> List[Object]:
        """
        Returns a list of all Objects which have the type 'obj_type'.

        :param obj_type: The type of the returned Objects.
        :return: A list of all Objects that have the type 'obj_type'.
        """
        return list(filter(lambda obj: obj.obj_type == obj_type, self.objects))

    def get_object_by_id(self, id: int) -> Object:
        """
        Returns the single Object that has the unique id.

        :param id: The unique id for which the Object should be returned.
        :return: The Object with the id 'id'.
        """
        return list(filter(lambda obj: obj.id == id, self.objects))[0]

    @abstractmethod
    def remove_object(self, obj_id: int) -> None:
        """
        Remove an object by its ID.

        :param obj_id: The unique id of the object to be removed.
        """
        pass

    def _set_attached_objects(self, parent, prev_object: List[Object]) -> None:
        """
        Updates the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this the _set_attached_objects method of all attached objects
        will be called.

        :param prev_object: A list of Objects that were already moved,
         these will be excluded to prevent loops in the update.
        """
        for child in parent.attachments:
            if child in prev_object:
                continue
            if not parent.attachments[child].bidirectional:
                parent.attachments[child].update_attachment()
                child.attachments[parent].update_attachment()
            else:
                link_to_object = parent.attachments[child].child_to_parent_transform

                world_to_object = parent.local_transformer.transform_pose_to_target_frame(link_to_object.to_pose(), "map")
                self.reset_object_base_pose(child,
                                            world_to_object.position_as_list(),
                                            world_to_object.orientation_as_list())
                child._current_pose = world_to_object
                self._set_attached_objects(child, prev_object + [parent])

    def attach_objects(self,
                       parent_object: Object,
                       child_object: Object,
                       parent_link_id: Optional[int] = -1,
                       child_link_id: Optional[int] = -1,
                       bidirectional: Optional[bool] = True) -> None:
        """
        Attaches two objects together by saving the current transformation between the links coordinate frames.
        Furthermore, a constraint will be created so the attachment also works while in simulation.

        :param bidirectional: If the parent should also follow the child.
        """

        # Add the attachment to the attachment dictionary of both objects
        attachment = Attachment(parent_object, child_object, parent_link_id, child_link_id, bidirectional)
        self.update_objects_attachments_collection(parent_object, child_object, attachment)
        self.attachment_event(parent_object, [parent_object, child_object])

    def update_objects_attachments_collection(self,
                                              parent_object: Object,
                                              child_object: Object,
                                              attachment: Attachment) -> None:
        parent_object.attachments[child_object] = attachment
        child_object.attachments[parent_object] = attachment.get_inverse()

    def add_fixed_constraint(self,
                             parent_object_id: int,
                             child_object_id: int,
                             child_to_parent_transform: Transform,
                             parent_link_id: Optional[int] = -1,
                             child_link_id: Optional[int] = -1) -> int:
        """
        Creates a fixed joint constraint between the given parent and child links,
        the joint frame will be at the origin of the child link frame, and would have the same orientation
        as the child link frame. if no link is given, the base link will be used (id = -1).

        returns the constraint id
        """
        # -1 in link id means use the base link of the object
        constraint = Constraint(parent_obj_id=parent_object_id,
                                parent_link_id=parent_link_id,
                                child_obj_id=child_object_id,
                                child_link_id=child_link_id,
                                joint_type=JointType.FIXED,
                                joint_axis_in_child_link_frame=[0, 0, 0],
                                joint_frame_position_wrt_parent_origin=child_to_parent_transform.translation_as_list(),
                                joint_frame_position_wrt_child_origin=[0, 0, 0],
                                joint_frame_orientation_wrt_parent_origin=child_to_parent_transform.rotation_as_list(),
                                )
        constraint_id = self.add_constraint(constraint)
        return constraint_id

    @abstractmethod
    def add_constraint(self, constraint: Constraint) -> int:
        """
        Add a constraint between two objects so that they become attached
        """
        pass

    @staticmethod
    def detach_objects(obj1: Object, obj2: Object) -> None:
        """
        Detaches obj2 from obj1. This is done by
        deleting the attachment from the attachments dictionary of both objects
        and deleting the constraint of pybullet.
        Afterward the detachment event of the corresponding BulletWorld will be fired.

        :param obj1: The object from which an object should be detached.
        :param obj2: The object which should be detached.
        """
        del obj1.attachments[obj2]
        del obj2.attachments[obj1]
        del obj1.cids[obj2]
        del obj2.cids[obj1]

        obj1.world.detachment_event(obj1, [obj1, obj2])

    @abstractmethod
    def remove_constraint(self, constraint_id):
        pass

    def get_object_joint_limits(self, obj: Object, joint_name: str) -> Tuple[float, float]:
        """
        Get the joint limits of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: A tuple containing the upper and the lower limits of the joint.
        """
        return self.get_object_joint_upper_limit(obj, joint_name), self.get_object_joint_lower_limit(obj, joint_name)

    @abstractmethod
    def get_object_joint_upper_limit(self, obj: Object, joint_name: str) -> float:
        """
        Get the joint upper limit of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: The joint upper limit as a float.
        """
        pass

    @abstractmethod
    def get_object_joint_lower_limit(self, obj: Object, joint_name: str) -> float:
        """
        Get the joint lower limit of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: The joint lower limit as a float.
        """
        pass

    @abstractmethod
    def get_object_joint_axis(self, obj: Object, joint_name: str) -> Tuple[float]:
        """
        Returns the axis along which a joint is moving. The given joint_name has to be part of this object.

        :param obj: The object
        :param joint_name: Name of the joint for which the axis should be returned.
        :return: The axis a vector of xyz
        """
        pass

    @abstractmethod
    def get_object_joint_type(self, obj: Object, joint_name: str) -> JointType:
        """
        Returns the type of the joint as element of the Enum :mod:`~pycram.enums.JointType`.

        :param obj: The object
        :param joint_name: Joint name for which the type should be returned
        :return: The type of  the joint
        """
        pass

    @abstractmethod
    def get_object_joint_position(self, obj: Object, joint_name: str) -> float:
        """
        Get the state of a joint of an articulated object

        :param obj: The object
        :param joint_name: The name of the joint
        """
        pass

    @abstractmethod
    def get_object_link_pose(self, obj_id: int, link_id: int) -> Pose:
        """
        Get the pose of a link of an articulated object with respect to the world frame.
        The pose is given as a tuple of position and orientation.
        """
        pass

    def simulate(self, seconds: float, real_time: Optional[float] = False) -> None:
        """
        Simulates Physics in the World for a given amount of seconds. Usually this simulation is faster than real
        time. By setting the 'real_time' parameter this simulation is slowed down such that the simulated time is equal
         to real time.

        :param seconds: The amount of seconds that should be simulated.
        :param real_time: If the simulation should happen in real time or faster.
        """
        for i in range(0, int(seconds * self.simulation_frequency)):
            self.step()
            for objects, callback in self.coll_callbacks.items():
                contact_points = self.get_contact_points_between_two_objects(objects[0], objects[1])
                if contact_points != ():
                    callback[0]()
                elif callback[1] is not None:  # Call no collision callback
                    callback[1]()
            if real_time:
                # Simulation runs at 240 Hz
                time.sleep(self.simulation_time_step)

    @abstractmethod
    def get_object_contact_points(self, obj: Object) -> List:
        """
        Returns a list of contact points of this Object with other Objects.

        :param obj: The object.
        :return: A list of all contact points with other objects
        """
        pass

    @abstractmethod
    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> List:
        """
        Returns a list of contact points between obj1 and obj2.

        :param obj1: The first object.
        :param obj2: The second object.
        :return: A list of all contact points between the two objects.
        """
        pass

    @abstractmethod
    def get_object_joint_names(self, obj_id: int) -> List[str]:
        """
        Get the names of all joints of an articulated object.
        """
        pass

    @abstractmethod
    def get_object_link_names(self, obj_id: int) -> List[str]:
        """
        Get the names of all links of an articulated object.
        """
        pass

    def get_object_link_id(self, obj: Object, link_idx: int) -> str:
        """
        Get the name of a link in an articulated object.

        :param obj: The object
        :param link_idx: The index of the link (would indicate order).
        """
        pass

    @abstractmethod
    def get_object_number_of_joints(self, obj_id: int) -> int:
        """
        Get the number of joints of an articulated object
        """
        pass

    def get_object_number_of_links(self, obj_id: int) -> int:
        """
        Get the number of links of an articulated object
        """
        pass

    @abstractmethod
    def reset_object_joint_position(self, obj: Object, joint_name: str, joint_pose: float) -> None:
        """
        Reset the joint position instantly without physics simulation

        :param obj: The object
        :param joint_name: The name of the joint
        :param joint_pose: The new joint pose
        """
        pass

    @abstractmethod
    def reset_object_base_pose(self, obj: Object, position: List[float], orientation: List[float]):
        """
        Reset the world position and orientation of the base of the object instantaneously,
        not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

        :param obj: The object
        :param position: The new position of the object as a vector of x,y,z
        :param orientation: The new orientation of the object as a quaternion of x,y,z,w
        """
        pass

    @abstractmethod
    def step(self):
        """
        Step the world simulation using forward dynamics
        """
        pass

    def set_object_color(self, obj: Object, color: Color, link: Optional[str] = ""):
        """
        Changes the color of this object, the color has to be given as a list
        of RGBA values. Optionally a link name can can be provided, if no link
        name is provided all links of this object will be colored.

        :param obj: The object which should be colored
        :param color: The color as Color object with RGBA values between 0 and 1
        :param link: The link name of the link which should be colored
        """
        if link == "":
            # Check if there is only one link, this is the case for primitive
            # forms or if loaded from an .stl or .obj file
            if obj.link_name_to_id != {}:
                for link_id in obj.link_name_to_id.values():
                    self.set_object_link_color(obj, link_id, color)
            else:
                self.set_object_link_color(obj, -1, color)
        else:
            self.set_object_link_color(obj, obj.link_name_to_id[link], color)

    @abstractmethod
    def set_object_link_color(self, obj: Object, link_id: int, rgba_color: Color):
        """
        Changes the color of a link of this object, the color has to be given as Color object.

        :param obj: The object which should be colored
        :param link_id: The link id of the link which should be colored
        :param rgba_color: The color as Color object with RGBA values between 0 and 1
        """
        pass

    def get_object_color(self,
                         obj: Object,
                         link: Optional[str] = None) -> Union[Color, Dict[str, Color], None]:
        """
        This method returns the color of this object or a link of this object. If no link is given then the
        return is either:

            1. A Color object with RGBA values, this is the case if the object only has one link (this
                happens for example if the object is spawned from a .obj or .stl file)
            2. A dict with the link name as key and the color as value. The color is given as a Color Object.
                Please keep in mind that not every link may have a color. This is dependent on the URDF from which the
                object is spawned.

        If a link is specified then the return is a list with RGBA values representing the color of this link.
        It may be that this link has no color, in this case the return is None as well as an error message.

        :param obj: The object for which the color should be returned.
        :param link: the link name for which the color should be returned.
        :return: The color of the object or link, or a dictionary containing every colored link with its color
        """
        link_to_color_dict = self.get_object_colors(obj)

        if link:
            if link in link_to_color_dict.keys():
                return link_to_color_dict[link]
            elif link not in obj.link_name_to_id.keys():
                rospy.logerr(f"The link '{link}' is not part of this obejct")
                return None
            else:
                rospy.logerr(f"The link '{link}' has no color")
                return None

        if len(link_to_color_dict) == 1:
            return list(link_to_color_dict.values())[0]
        else:
            return link_to_color_dict

    @abstractmethod
    def get_object_colors(self, obj: Object) -> Dict[str, Color]:
        """
        Get the RGBA colors of each link in the object as a dictionary from link name to color.

        :param obj: The object
        :return: A dictionary with link names as keys and a Color object for each link as value.
        """
        pass

    @abstractmethod
    def get_object_aabb(self, obj: Object) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of this object. The return of this method are two points in
        world coordinate frame which define a bounding box.

        :param obj: The object for which the bounding box should be returned.
        :return: AxisAlignedBoundingBox object containing the min and max points of the bounding box.
        """
        pass

    @abstractmethod
    def get_object_link_aabb(self, obj_id: int, link_id: int) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of the link. The return of this method are two points in
        world coordinate frame which define a bounding box.
        """
        pass

    def get_attachment_event(self) -> Event:
        """
        Returns the event reference that is fired if an attachment occurs.

        :return: The reference to the attachment event
        """
        return self.attachment_event

    def get_detachment_event(self) -> Event:
        """
        Returns the event reference that is fired if a detachment occurs.

        :return: The event reference for the detachment event.
        """
        return self.detachment_event

    def get_manipulation_event(self) -> Event:
        """
        Returns the event reference that is fired if any manipulation occurs.

        :return: The event reference for the manipulation event.
        """
        return self.manipulation_event

    @abstractmethod
    def set_realtime(self, real_time: bool) -> None:
        """
        Enables the real time simulation of Physic in the BulletWorld. By default, this is disabled and Physics is only
        simulated to reason about it.

        :param real_time: Whether the World should simulate Physics in real time.
        """
        pass

    @abstractmethod
    def set_gravity(self, gravity_vector: List[float]) -> None:
        """
        Sets the gravity that is used in the World. By default, it is set to the gravity on earth ([0, 0, -9.8]).
         Gravity is given as a vector in x,y,z. Gravity is only applied while simulating Physic.

        :param gravity_vector: The gravity vector that should be used in the World.
        """
        pass

    def set_robot_if_not_set(self, robot: Object) -> None:
        """
        Sets the robot if it is not set yet.

        :param robot: The Object reference to the Object representing the robot.
        """
        if not self.robot_is_set():
            self.set_robot(robot)

    @staticmethod
    def set_robot(robot: Union[Object, None]) -> None:
        """
        Sets the global variable for the robot Object. This should be set on spawning the robot.

        :param robot: The Object reference to the Object representing the robot.
        """
        World.robot = robot

    @staticmethod
    def robot_is_set() -> bool:
        """
        Returns whether the robot has been set or not.

        :return: True if the robot has been set, False otherwise.
        """
        return World.robot is not None

    def exit(self, wait_time_before_exit_in_secs: Optional[float] = None) -> None:
        """
        Closes the World as well as the shadow world, also collects any other thread that is running.
        """
        if wait_time_before_exit_in_secs is not None:
            time.sleep(wait_time_before_exit_in_secs)
        self.exit_prospection_world_if_exists()
        self.disconnect_from_physics_server()
        self.reset_current_world()
        self.reset_robot()
        self.join_threads()

    def exit_prospection_world_if_exists(self):
        if self.prospection_world:
            self.terminate_world_sync()
            self.prospection_world.exit()

    @abstractmethod
    def disconnect_from_physics_server(self):
        """
        Disconnects the world from the physics server.
        """
        pass

    def reset_current_world(self):
        if World.current_world == self:
            World.current_world = None

    def reset_robot(self):
        self.set_robot(None)

    @abstractmethod
    def join_threads(self):
        """
        Join any running threads. Useful for example when exiting the world.
        """
        pass

    def terminate_world_sync(self):
        self.world_sync.terminate = True
        self.world_sync.join()

    def save_state(self) -> int:
        """
        Returns the id of the saved state of the World. The saved state contains the position, orientation and joint
        position of every Object in the World, the objects attachments and the constraint ids.

        :return: A unique id of the state
        """
        state_id = self.save_physics_simulator_state()
        self._saved_states[state_id] = WorldState(state_id,
                                                  self.get_objects_attachments(),
                                                  self.get_objects_constraint_ids())
        return state_id

    def restore_state(self, state_id) -> None:
        """
        Restores the state of the World according to the given state using the unique state id. This includes position,
         orientation, and joint states. However, restore can not respawn objects if there are objects that were deleted
          between creation of the state and restoring, they will be skipped.

        :param state_id: The unique id representing the state, as returned by :func:`~save_state`
        """
        self.restore_physics_simulator_state(state_id)
        self.restore_attachments_and_constraints_from_saved_world_state(state_id)

    @abstractmethod
    def save_physics_simulator_state(self) -> int:
        """
        Saves the state of the physics simulator and returns the unique id of the state.
        """
        pass

    def get_objects_attachments(self) -> Dict[Object, Dict[Object, Attachment]]:
        """
        Get The attachments collections that is stored in each object.
        """
        attachments = {}
        for o in self.objects:
            attachments[o] = o.attachments.copy()
        return attachments

    def get_objects_constraint_ids(self) -> Dict[Object, Dict[Object, int]]:
        """
        Get the constraint ids collection that is stored in each object.
        """
        constraint_ids = {}
        for o in self.objects:
            constraint_ids[o] = o.cids.copy()
        return constraint_ids

    @abstractmethod
    def restore_physics_simulator_state(self, state_id):
        """
        Restores the objects and environment state in the physics simulator according to
         the given state using the unique state id.
        """
        pass

    def restore_attachments_and_constraints_from_saved_world_state(self, state_id: int):
        """
        Restores the attachments and constraints of the objects in the World. This is done by setting the attachments,
        and the cids attributes of each object in the World to the given attachments and constraint_ids.
        """
        self.restore_attachments_from_saved_world_state(state_id)
        self.restore_constraints_from_saved_world_state(state_id)

    def restore_attachments_from_saved_world_state(self, state_id: int):
        attachments = self._saved_states[state_id].attachments
        for obj in self.objects:
            try:
                obj.attachments = attachments[obj]
            except KeyError:
                continue

    def restore_constraints_from_saved_world_state(self, state_id: int):
        constraint_ids = self._saved_states[state_id].constraint_ids
        for obj in self.objects:
            try:
                obj.cids = constraint_ids[obj]
            except KeyError:
                continue

    def _copy(self) -> World:
        """
        Copies this World into another and returns it. The other World
        will be in Direct mode. The prospection world should always be preferred instead of creating a new World.
        This method should only be used if necessary since there can be unforeseen problems.

        :return: The reference to the new World
        """
        world = World("DIRECT", False, World.simulation_time_step)
        for obj in self.objects:
            o = Object(obj.name, obj.obj_type, obj.path, Pose(obj.get_position(), obj.get_orientation()), world,
                       obj.color)
            for joint in obj.joint_name_to_id:
                o.set_joint_position(joint, obj.get_joint_position(joint))
        return world

    def register_two_objects_collision_callbacks(self, object_a: Object, object_b: Object,
                                                 on_collision_callback: Callable,
                                                 on_collision_removal_callback: Optional[Callable] = None) -> None:
        """
        Registers callback methods for contact between two Objects. There can be a callback for when the two Objects
        get in contact and, optionally, for when they are not in contact anymore.

        :param object_a: An object in the World
        :param object_b: Another object in the World
        :param on_collision_callback: A function that should be called if the objects are in contact
        :param on_collision_removal_callback: A function that should be called if the objects are not in contact
        """
        self.coll_callbacks[(object_a, object_b)] = (on_collision_callback, on_collision_removal_callback)

    @classmethod
    def add_resource_path(cls, path: str) -> None:
        """
        Adds a resource path in which the BulletWorld will search for files. This resource directory is searched if an
        Object is spawned only with a filename.

        :param path: A path in the filesystem in which to search for files.
        """
        cls.data_directory.append(path)

    def get_prospection_object_from_object(self, obj: Object) -> Object:
        """
        Returns the corresponding object from the prospection world for a given object in the main world.
         If the given Object is already in the prospection world, it is returned.

        :param obj: The object for which the corresponding object in the prospection World should be found.
        :return: The corresponding object in the prospection world.
        """
        try:
            return self.world_sync.object_mapping[obj]
        except KeyError:
            prospection_world = self if self.is_prospection_world else self.prospection_world
            if obj in prospection_world.objects:
                return obj
            else:
                raise ValueError(
                    f"There is no prospection object for the given object: {obj}, this could be the case if"
                    f" the object isn't anymore in the main (graphical) World"
                    f" or if the given object is already a prospection object. ")

    def get_object_from_prospection_object(self, prospection_object: Object) -> Object:
        """
        Returns the corresponding object from the main World for a given
        object in the shadow world. If the  given object is not in the shadow
        world an error will be raised.

        :param prospection_object: The object for which the corresponding object in the main World should be found.
        :return: The object in the main World.
        """
        object_map = self.world_sync.object_mapping
        try:
            return list(object_map.keys())[list(object_map.values()).index(prospection_object)]
        except ValueError:
            raise ValueError("The given object is not in the shadow world.")

    def reset_world(self) -> None:
        """
        Resets the World to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they were spawned.
        """
        for obj in self.objects:
            obj.detach_all()
            obj.reset_all_joints_positions()
            obj.set_pose(obj.original_pose)

    def update_transforms_for_objects_in_current_world(self) -> None:
        """
        Updates transformations for all objects that are currently in :py:attr:`~pycram.world.World.current_world`.
        """
        curr_time = rospy.Time.now()
        for obj in list(self.current_world.objects):
            obj.update_link_transforms(curr_time)


class UseProspectionWorld:
    """
    An environment for using the prospection world, while in this environment the :py:attr:`~World.current_world`
    variable will point to the prospection world.

    Example:
        with UseProspectionWorld():
            NavigateAction.Action([[1, 0, 0], [0, 0, 0, 1]]).perform()
    """

    def __init__(self):
        self.prev_world: World = None

    def __enter__(self):
        if not World.current_world.is_prospection_world:
            time.sleep(20 * World.simulation_time_step)
            # blocks until the adding queue is ready
            World.current_world.world_sync.add_obj_queue.join()

            self.prev_world = World.current_world
            World.current_world.world_sync.pause_sync = True
            World.current_world = World.current_world.prospection_world

    def __exit__(self, *args):
        if self.prev_world is not None:
            World.current_world = self.prev_world
            World.current_world.world_sync.pause_sync = False


class WorldSync(threading.Thread):
    """
    Synchronizes the state between the World and its prospection world.
    Meaning the cartesian and joint position of everything in the prospection world will be
    synchronized with the main World.
    Adding and removing objects is done via queues, such that loading times of objects
    in the prospection world does not affect the World.
    The class provides the possibility to pause the synchronization, this can be used
    if reasoning should be done in the prospection world.
    """

    def __init__(self, world: World, prospection_world: World):
        threading.Thread.__init__(self)
        self.world: World = world
        self.prospection_world: World = prospection_world
        self.prospection_world.world_sync = self

        self.terminate: bool = False
        self.add_obj_queue: Queue = Queue()
        self.remove_obj_queue: Queue = Queue()
        self.pause_sync: bool = False
        # Maps bullet to prospection world objects
        self.object_mapping: Dict[Object, Object] = {}
        self.equal_states = False

    def run(self):
        """
        Main method of the synchronization, this thread runs in a loop until the
        terminate flag is set.
        While this loop runs it continuously checks the cartesian and joint position of
        every object in the World and updates the corresponding object in the
        prospection world. When there are entries in the adding or removing queue the corresponding objects will be added
        or removed in the same iteration.
        """
        while not self.terminate:
            self.check_for_pause()
            # self.equal_states = False
            for i in range(self.add_obj_queue.qsize()):
                obj = self.add_obj_queue.get()
                # [name, type, path, position, orientation, self.world.prospection_world, color, world object]
                o = Object(obj[0], obj[1], obj[2], Pose(obj[3], obj[4]), obj[5], obj[6])
                # Maps the World object to the prospection world object
                self.object_mapping[obj[7]] = o
                self.add_obj_queue.task_done()
            for i in range(self.remove_obj_queue.qsize()):
                obj = self.remove_obj_queue.get()
                # Get prospection world object reference from object mapping
                prospection_obj = self.object_mapping[obj]
                prospection_obj.remove()
                del self.object_mapping[obj]
                self.remove_obj_queue.task_done()

            for world_obj, prospection_obj in self.object_mapping.items():
                b_pose = world_obj.get_pose()
                s_pose = prospection_obj.get_pose()
                if b_pose.dist(s_pose) != 0.0:
                    prospection_obj.set_pose(world_obj.get_pose())

                # Manage joint positions
                if len(world_obj.joint_name_to_id) > 2:
                    for joint_name in world_obj.joint_name_to_id.keys():
                        if prospection_obj.get_joint_position(joint_name) != world_obj.get_joint_position(joint_name):
                            prospection_obj.set_positions_of_all_joints(world_obj.get_positions_of_all_joints())
                            break

            self.check_for_pause()
            # self.check_for_equal()
            time.sleep(1 / 240)

        self.add_obj_queue.join()
        self.remove_obj_queue.join()

    def check_for_pause(self) -> None:
        """
        Checks if :py:attr:`~self.pause_sync` is true and sleeps this thread until it isn't anymore.
        """
        while self.pause_sync:
            time.sleep(0.1)

    def check_for_equal(self) -> None:
        """
        Checks if both Worlds have the same state, meaning all objects are in the same position.
        This is currently not used, but might be used in the future if synchronization issues worsen.
        """
        eql = True
        for obj, prospection_obj in self.object_mapping.items():
            eql = eql and obj.get_pose() == prospection_obj.get_pose()
        self.equal_states = eql


class Link:

    def __init__(self,
                 _id: int,
                 obj_id: int,
                 obj_tf_frame: str,
                 urdf_link: urdf_parser_py.urdf.Link,
                 world: World,
                 is_root: Optional[bool] = False):
        self.id = _id
        self.obj_id = obj_id
        self.tf_frame = f"{obj_tf_frame}/{self.name}" if not is_root else obj_tf_frame
        self.urdf_link = urdf_link
        self.world = world
        self.object = world.get_object_by_id(obj_id)
        self.local_transformer = self.object.local_transformer

    @property
    def transform(self) -> Transform:
        """
        The transformation from the world frame to this link frame.
        """
        return self.pose.to_transform(self.tf_frame)

    def get_transform_to_other_link(self, other_link: Link):
        new_pose = self.local_transformer.transform_pose_to_target_frame(other_link.pose, self.tf_frame)
        return new_pose.to_transform(other_link.tf_frame)

    def get_pose_wrt_other_link(self, other_link: Link):
        return self.local_transformer.transform_pose_to_target_frame(self.pose, other_link.tf_frame)

    def get_aabb(self) -> AxisAlignedBoundingBox:
        return self.world.get_object_link_aabb(self.obj_id, self.id)

    @property
    def position(self) -> Point:
        return self.pose.position

    @property
    def orientation(self):
        return self.pose.orientation

    @property
    def pose(self):
        """
        The pose of the link relative to the world frame.
        """
        return self.world.get_object_link_pose(self.obj_id, self.id)

    @property
    def name(self):
        return self.urdf_link.name

    def get_geometry(self) -> GeometricType:
        return None if not self.collision else self.collision.geometry

    @property
    def collision(self):
        return self.urdf_link.collision






class Object:
    """
    Represents a spawned Object in the World.
    """

    def __init__(self, name: str, obj_type: Union[str, ObjectType], path: str,
                 pose: Pose = None,
                 world: World = None,
                 color: Optional[Color] = Color(),
                 ignore_cached_files: Optional[bool] = False):
        """
        The constructor loads the urdf file into the given World, if no World is specified the
        :py:attr:`~World.current_world` will be used. It is also possible to load .obj and .stl file into the World.
        The color parameter is only used when loading .stl or .obj files, for URDFs :func:`~Object.set_color` can be used.

        :param name: The name of the object
        :param obj_type: The type of the object
        :param path: The path to the source file, if only a filename is provided then the resourcer directories will be searched
        :param pose: The pose at which the Object should be spawned
        :param world: The World in which the object should be spawned, if no world is specified the :py:attr:`~World.current_world` will be used
        :param color: The color with which the object should be spawned.
        :param ignore_cached_files: If true the file will be spawned while ignoring cached files.
        """
        if pose is None:
            pose = Pose()
        self.world: World = world if world is not None else World.current_world
        self.local_transformer = LocalTransformer()
        self.name: str = name
        self.obj_type: Union[str, ObjectType] = obj_type
        self.color: Color = color
        pose_in_map = self.local_transformer.transform_pose_to_target_frame(pose, "map")
        position, orientation = pose_in_map.to_list()
        self.id, self.path = _load_object(name, path, position, orientation, self.world, color, ignore_cached_files)
        self.joint_name_to_id: Dict[str, int] = self._get_joint_name_to_id_map()
        self.link_name_to_id: Dict[str, int] = self._get_link_name_to_id_map()
        self.attachments: Dict[Object, Attachment] = {}
        self.cids: Dict[Object, int] = {}
        self.original_pose = pose_in_map

        self.tf_frame = ("prospection/" if self.world.is_prospection_world else "") + self.name + "_" + str(self.id)

        if not self.world.is_prospection_world:
            self.world.world_sync.add_obj_queue.put(
                [name, obj_type, path, position, orientation, self.world.prospection_world, color, self])

        with open(self.path) as f:
            self.urdf_object = URDF.from_xml_string(f.read())
            if self.urdf_object.name == robot_description.name:
                self.world.set_robot_if_not_set(self)

        self.link_name_to_id[self.urdf_object.get_root()] = -1

        self._current_pose = pose_in_map
        self._current_link_poses: Dict[str, Pose] = {}
        self._current_link_transforms: Dict[str, Transform] = {}
        self._current_joints_positions = {}
        self._init_current_positions_of_joint()
        self._update_current_link_poses_and_transforms()

        self.base_origin_shift = np.array(position) - np.array(self.get_base_origin().position_as_list())
        self.update_link_transforms()
        self.link_to_geometry = self.get_geometry_for_link()

        self.world.objects.append(self)

    def _init_current_positions_of_joint(self) -> None:
        """
        Initialize the cached joint position for each joint.
        """
        for joint_name in self.joint_name_to_id.keys():
            self._current_joints_positions[joint_name] = self.world.get_object_joint_position(self, joint_name)

    def __repr__(self):
        skip_attr = ["links", "joints", "urdf_object", "attachments", "cids", "_current_link_poses",
                     "_current_link_transforms", "link_to_geometry"]
        return self.__class__.__qualname__ + f"(" + ', \n'.join(
            [f"{key}={value}" if key not in skip_attr else f"{key}: ..." for key, value in self.__dict__.items()]) + ")"

    def remove(self) -> None:
        """
        Removes this object from the World it currently resides in.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to world remove object is done
        to remove this Object from the simulation/world.
        """
        self.detach_all()

        self.world.objects.remove(self)

        # This means the current world of the object is not the prospection world, since it
        # has a reference to the prospection world
        if self.world.prospection_world is not None:
            self.world.world_sync.remove_obj_queue.put(self)
            self.world.world_sync.remove_obj_queue.join()

        self.world.remove_object(self.id)

        if World.robot == self:
            World.robot = None

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
        Furthermore, a constraint of pybullet will be created so the attachment
        also works while simulation.
        Loose attachments means that the attachment will only be one-directional. For example, if this object moves the
        other, attached, object will also move but not the other way around.

        :param child_object: The other object that should be attached.
        :param parent_link: The link name of this object.
        :param child_link: The link name of the other object.
        :param bidirectional: If the attachment should be a loose attachment.
        """
        self.world.attach_objects(self,
                                  child_object,
                                  self.get_link_id(parent_link),
                                  child_object.get_link_id(child_link),
                                  bidirectional)

    def detach(self, child_object: Object) -> None:
        """
        Detaches another object from this object. This is done by
        deleting the attachment from the attachments dictionary of both objects
        and deleting the constraint of pybullet.
        Afterward the detachment event of the corresponding World will be fired.

        :param child_object: The object which should be detached
        """
        self.world.detach_objects(self, child_object)

    def detach_all(self) -> None:
        """
        Detach all objects attached to this object.
        """
        attachments = self.attachments.copy()
        for att in attachments.keys():
            self.world.detach_objects(self, att)

    def get_position(self) -> Pose.position:
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

    def get_pose(self) -> Pose:
        """
        Returns the position of this object as a list of xyz. Alias for :func:`~Object.get_position`.

        :return: The current pose of this object
        """
        return self._current_pose

    def set_pose(self, pose: Pose, base: bool = False) -> None:
        """
        Sets the Pose of the object.

        :param pose: New Pose for the object
        :param base: If True places the object base instead of origin at the specified position and orientation
        """
        pose_in_map = self.local_transformer.transform_pose_to_target_frame(pose, "map")
        position, orientation = pose_in_map.to_list()
        if base:
            position = np.array(position) + self.base_origin_shift
        self.world.reset_object_base_pose(self, position, orientation)
        self._current_pose = pose_in_map
        self._update_current_link_poses_and_transforms()
        self.world._set_attached_objects(self, [self])

    @property
    def pose(self) -> Pose:
        """
        Property that returns the current position of this Object.

        :return: The position as a list of xyz
        """
        return self.get_pose()

    @pose.setter
    def pose(self, value: Pose) -> None:
        """
        Sets the Pose of the Object to the given value. Function for attribute use.

        :param value: New Pose of the Object
        """
        self.set_pose(value)

    def move_base_to_origin_pos(self) -> None:
        """
        Move the object such that its base will be at the current origin position.
        This is useful when placing objects on surfaces where you want the object base in contact with the surface.
        """
        self.set_pose(self.get_pose(), base=True)

    def _set_attached_objects(self, prev_object: List[Object]) -> None:
        """
        Updates the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this the _set_attached_objects method of all attached objects
        will be called.

        :param prev_object: A list of Objects that were already moved, these will be excluded to prevent loops in the update.
        """
        self.world._set_attached_objects(self, prev_object)

    def _calculate_transform_from_other_object_base_to_this_object_base(self, other_object: Object):
        pose_wrt_this_object_base = self.local_transformer.transform_pose_to_target_frame(other_object.pose,
                                                                                          self.tf_frame)
        return Transform.from_pose_and_child_frame(pose_wrt_this_object_base, other_object.tf_frame)

    def transform_pose_to_link_frame(self, pose: Pose, link_name: str) -> Union[Pose, None]:
        """
        :return: The new pose transformed to be relative to the link coordinate frame.
        """
        target_frame = self.get_link_tf_frame(link_name)
        return self.local_transformer.transform_pose_to_target_frame(pose, target_frame)

    def set_position(self, position: Union[Pose, Point], base=False) -> None:
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
        else:
            target_position = position

        pose.pose.position = target_position
        pose.pose.orientation = self.get_orientation()
        self.set_pose(pose, base=base)

    def set_orientation(self, orientation: Union[Pose, Quaternion]) -> None:
        """
        Sets the orientation of the Object to the given orientation. Orientation can either be a Pose, in this case only
        the orientation of this pose is used or a geometry_msgs.msg/Quaternion which is the orientation of a Pose.

        :param orientation: Target orientation given as a list of xyzw.
        """
        pose = Pose()
        if isinstance(orientation, Pose):
            target_orientation = orientation.orientation
            pose.frame = orientation.frame
        else:
            target_orientation = orientation

        pose.pose.position = self.get_position()
        pose.pose.orientation = target_orientation
        self.set_pose(pose)

    def _get_joint_name_to_id_map(self) -> Dict[str, int]:
        """
        Creates a dictionary which maps the joint names to their unique ids.
        """
        joint_names = self.world.get_object_joint_names(self.id)
        n_joints = len(joint_names)
        return dict(zip(joint_names, range(n_joints)))

    def _get_link_name_to_id_map(self) -> Dict[str, int]:
        """
        Creates a dictionary which maps the link names to their unique ids.
        """
        link_names = self.world.get_object_link_names(self.id)
        n_links = len(link_names)
        return dict(zip(link_names, range(n_links)))

    def get_joint_id(self, name: str) -> int:
        """
        Returns the unique id for a joint name. As used by PyBullet.

        :param name: The joint name
        :return: The unique id
        """
        return self.joint_name_to_id[name]

    def get_link_id(self, name: str) -> int:
        """
        Returns a unique id for a link name. If the name is None return -1.

        :param name: The link name
        :return: The unique id
        """
        if name is None:
            return -1
        return self.link_name_to_id[name]

    def get_link_by_id(self, id: int) -> str:
        """
        Returns the name of a link for a given unique link id

        :param id: id for link
        :return: The link name
        """
        return dict(zip(self.link_name_to_id.values(), self.link_name_to_id.keys()))[id]

    def get_joint_by_id(self, joint_id: int) -> str:
        """
        Returns the joint name for a unique PyBullet id

        :param joint_id: The Pybullet id of for joint
        :return: The joint name
        """
        return dict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))[joint_id]

    def get_other_object_link_pose_relative_to_my_link(self,
                                                       my_link_id: int,
                                                       other_link_id: int,
                                                       other_object: Object) -> Pose:
        """
        Calculates the pose of a link (child_link) in the coordinate frame of another link (parent_link).
        :return: The pose of the source frame in the target frame
        """

        child_link_pose = other_object.get_link_pose(other_object.get_link_by_id(other_link_id))
        parent_link_frame = self.get_link_tf_frame(self.get_link_by_id(my_link_id))
        return self.local_transformer.transform_pose_to_target_frame(child_link_pose, parent_link_frame)

    def get_transform_from_my_link_to_other_object_link(self,
                                                        my_link_id: int,
                                                        other_link_id: int,
                                                        other_object: Object) -> Transform:
        pose = self.get_other_object_link_pose_relative_to_my_link(my_link_id, other_link_id, other_object)
        return pose.to_transform(other_object.get_link_tf_frame_by_id(other_link_id))

    def get_link_position(self, name: str) -> Pose.position:
        """
        Returns the position of a link of this Object. Position is returned as a list of xyz.

        :param name: The link name
        :return: The link position as xyz
        """
        return self.get_link_pose(name).position

    def get_link_orientation(self, name: str) -> Quaternion:
        """
        Returns the orientation of a link of this Object. Orientation is returned as a quaternion.

        :param name: The name of the link
        :return: The orientation of the link as a quaternion
        """
        return self.get_link_pose(name).orientation

    def get_link_pose(self, name: str) -> Pose:
        """
        Returns a Pose of the link corresponding to the given name. The returned Pose will be in world coordinate frame.

        :param name: Link name for which a Pose should be returned
        :return: The pose of the link
        """
        if name in self.link_name_to_id.keys() and self.link_name_to_id[name] == -1:
            return self.get_pose()
        return self._current_link_poses[name]

    def get_link_pose_by_id(self, link_id: int) -> Pose:
        return self.get_link_pose(self.get_link_by_id(link_id))

    def reset_all_joints_positions(self) -> None:
        """
        Sets the current position of all joints to 0. This is useful if the joints should be reset to their default
        """
        joint_names = list(self.joint_name_to_id.keys())
        joint_positions = [0] * len(joint_names)
        self.set_positions_of_all_joints(dict(zip(joint_names, joint_positions)))

    def set_positions_of_all_joints(self, joint_poses: dict) -> None:
        """
        Sets the current position of multiple joints at once, this method should be preferred when setting
         multiple joints at once instead of running :func:`~Object.set_joint_position` in a loop.

        :param joint_poses:
        :return:
        """
        for joint_name, joint_position in joint_poses.items():
            self.world.reset_object_joint_position(self, joint_name, joint_position)
            self._current_joints_positions[joint_name] = joint_position
        self._update_current_link_poses_and_transforms()
        self.world._set_attached_objects(self, [self])

    def set_joint_position(self, joint_name: str, joint_position: float) -> None:
        """
        Sets the position of the given joint to the given joint pose. If the pose is outside the joint limits, as stated
        in the URDF, an error will be printed. However, the joint will be set either way.

        :param joint_name: The name of the joint
        :param joint_position: The target pose for this joint
        """
        # TODO Limits for rotational (infinitie) joints are 0 and 1, they should be considered seperatly
        up_lim, low_lim = self.world.get_object_joint_limits(self, joint_name)
        if low_lim > up_lim:
            low_lim, up_lim = up_lim, low_lim
        if not low_lim <= joint_position <= up_lim:
            logging.error(
                f"The joint position has to be within the limits of the joint. The joint limits for {joint_name}"
                f" are {low_lim} and {up_lim}")
            logging.error(f"The given joint position was: {joint_position}")
            # Temporarily disabled because kdl outputs values exciting joint limits
            # return
        self.world.reset_object_joint_position(self, joint_name, joint_position)
        self._current_joints_positions[joint_name] = joint_position
        self._update_current_link_poses_and_transforms()
        self.world._set_attached_objects(self, [self])

    def get_joint_position(self, joint_name: str) -> float:
        """
        Returns the joint position for the given joint name.

        :param joint_name: The name of the joint
        :return: The current pose of the joint
        """
        return self._current_joints_positions[joint_name]

    def contact_points(self) -> List:
        """
        Returns a list of contact points of this Object with other Objects. For a more detailed explanation of the returned
        list please look at `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_

        :return: A list of all contact points with other objects
        """
        return self.world.get_object_contact_points(self)

    def contact_points_simulated(self) -> List:
        """
        Returns a list of all contact points between this Object and other Objects after stepping the simulation once.
        For a more detailed explanation of the returned
        list please look at `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_

        :return: A list of contact points between this Object and other Objects
        """
        state_id = self.world.save_state()
        self.world.step()
        contact_points = self.contact_points()
        self.world.restore_state(state_id)
        return contact_points

    def update_joints_from_topic(self, topic_name: str) -> None:
        """
        Updates the joints of this object with positions obtained from a topic with the message type JointState.
        Joint names on the topic have to correspond to the joints of this object otherwise an error message will be logged.

        :param topic_name: Name of the topic with the joint states
        """
        msg = rospy.wait_for_message(topic_name, JointState)
        joint_names = msg.name
        joint_positions = msg.position
        if set(joint_names).issubset(self.joint_name_to_id.keys()):
            for i in range(len(joint_names)):
                self.set_joint_position(joint_names[i], joint_positions[i])
        else:
            add_joints = set(joint_names) - set(self.joint_name_to_id.keys())
            rospy.logerr(f"There are joints in the published joint state which are not in this model: /n \
                    The following joint{'s' if len(add_joints) != 1 else ''}: {add_joints}")

    def update_pose_from_tf(self, frame: str) -> None:
        """
        Updates the pose of this object from a TF message.

        :param frame: Name of the TF frame from which the position should be taken
        """
        tf_listener = tf.TransformListener()
        time.sleep(0.5)
        position, orientation = tf_listener.lookupTransform(frame, "map", rospy.Time(0))
        position = [position[0][0] * -1,
                    position[0][1] * -1,
                    position[0][2]]
        self.set_position(Pose(position, orientation))

    def set_color(self, color: Color, link: Optional[str] = "") -> None:
        """
        Changes the color of this object, the color has to be given as a list
        of RGBA values. Optionally a link name can can be provided, if no link
        name is provided all links of this object will be colored.

        :param color: The color as RGBA values between 0 and 1
        :param link: The link name of the link which should be colored
        """
        self.world.set_object_color(self, color, link)

    def get_color(self, link: Optional[str] = None) -> Union[Color, Dict[str, Color], None]:
        return self.world.get_object_color(self, link)

    def get_aabb(self) -> AxisAlignedBoundingBox:
        return self.world.get_object_aabb(self)

    def get_link_aabb(self, link_id: int) -> AxisAlignedBoundingBox:
        return self.world.get_object_link_aabb(self.id, link_id)

    def get_base_origin(self) -> Pose:
        """
        Returns the origin of the base/bottom of an object

        :return: The position of the bottom of this Object
        """
        aabb = self.get_link_aabb(-1)
        base_width = np.absolute(aabb.min_x - aabb.max_x)
        base_length = np.absolute(aabb.min_y - aabb.max_y)
        return Pose([aabb.min_x + base_width / 2, aabb.min_y + base_length / 2, aabb.min_z],
                    self.get_pose().orientation_as_list())

    def get_joint_limits(self, joint: str) -> Tuple[float, float]:
        """
        Returns the lower and upper limit of a joint, if the lower limit is higher
        than the upper they are swapped to ensure the lower limit is always the smaller one.

        :param joint: The name of the joint for which the limits should be found.
        :return: The lower and upper limit of the joint.
        """
        if joint not in self.joint_name_to_id.keys():
            raise KeyError(f"The given Joint: {joint} is not part of this object")
        lower, upper = self.world.get_object_joint_limits(self, joint)
        if lower > upper:
            lower, upper = upper, lower
        return lower, upper

    def get_joint_axis(self, joint_name: str) -> Tuple[float]:
        """
        Returns the axis along which a joint is moving. The given joint_name has to be part of this object.

        :param joint_name: Name of the joint for which the axis should be returned.
        :return: The axis a vector of xyz
        """
        return self.world.get_object_joint_axis(self, joint_name)

    def get_joint_type(self, joint_name: str) -> JointType:
        """
        Returns the type of the joint as element of the Enum :mod:`~pycram.enums.JointType`.

        :param joint_name: Joint name for which the type should be returned
        :return: The type of  the joint
        """
        return self.world.get_object_joint_type(self, joint_name)

    def find_joint_above(self, link_name: str, joint_type: JointType) -> str:
        """
        Traverses the chain from 'link_name' to the URDF origin and returns the first joint that is of type 'joint_type'.

        :param link_name: Link name above which the joint should be found
        :param joint_type: Joint type that should be searched for
        :return: Name of the first joint which has the given type
        """
        chain = self.urdf_object.get_chain(self.urdf_object.get_root(), link_name)
        reversed_chain = reversed(chain)
        container_joint = None
        for element in reversed_chain:
            if element in self.joint_name_to_id and self.get_joint_type(element) == joint_type:
                container_joint = element
                break
        if not container_joint:
            rospy.logwarn(f"No joint of type {joint_type} found above link {link_name}")
        return container_joint

    def get_positions_of_all_joints(self) -> Dict[str: float]:
        """
        Returns the positions of all joints of the object as a dictionary of joint names and joint positions.

        :return: A dictionary with all joints positions'.
        """
        return self._current_joints_positions

    def get_link_tf_frame(self, link_name: str) -> str:
        """
        Returns the name of the tf frame for the given link name. This method does not check if the given name is
        actually a link of this object.

        :param link_name: Name of a link for which the tf frame should be returned
        :return: A TF frame name for a specific link
        """
        if link_name == self.urdf_object.get_root():
            return self.tf_frame
        return self.tf_frame + "/" + link_name

    def get_link_tf_frame_by_id(self, link_id: int) -> str:
        return self.get_link_tf_frame(self.get_link_by_id(link_id))

    def get_geometry_for_link(self) -> Dict[str, urdf_parser_py.urdf.GeometricType]:
        """
        Extracts the geometry information for each collision of each link and links them to the respective link.

        :return: A dictionary with link name as key and geometry information as value
        """
        link_to_geometry = {}
        for link in self.link_name_to_id.keys():
            link_obj = self.urdf_object.link_map[link]
            if not link_obj.collision:
                link_to_geometry[link] = None
            else:
                link_to_geometry[link] = link_obj.collision.geometry
        return link_to_geometry

    def update_link_transforms(self, transform_time: Optional[rospy.Time] = None):
        self.local_transformer.update_transforms(self._current_link_transforms.values(), transform_time)

    def _update_current_link_poses_and_transforms(self) -> None:
        """
        Updates the cached poses and transforms for each link of this Object
        """
        for link_name in self.link_name_to_id.keys():
            if link_name == self.urdf_object.get_root():
                self._update_root_link_pose_and_transform()
            else:
                self._update_link_pose_and_transform(link_name)

    def _update_root_link_pose_and_transform(self) -> None:
        link_name = self.urdf_object.get_root()
        self._current_link_poses[link_name] = self._current_pose
        self._current_link_transforms[link_name] = self._current_pose.to_transform(self.tf_frame)

    def _update_link_pose_and_transform(self, link_name: str) -> None:
        self._current_link_poses[link_name] = self.world.get_object_link_pose(self.id, self.get_link_id(link_name))
        self._current_link_transforms[link_name] = self._current_link_poses[link_name].to_transform(
            self.get_link_tf_frame(link_name))


def filter_contact_points(contact_points, exclude_ids) -> List:
    """
    Returns a list of contact points where Objects that are in the 'exclude_ids' list are removed.

    :param contact_points: A list of contact points
    :param exclude_ids: A list of unique ids of Objects that should be removed from the list
    :return: A list containing 'contact_points' without Objects that are in 'exclude_ids'
    """
    return list(filter(lambda cp: cp[2] not in exclude_ids, contact_points))


def get_path_from_data_dir(file_name: str, data_directory: str) -> str:
    """
    Returns the full path for a given file name in the given directory. If there is no file with the given filename
    this method returns None.

    :param file_name: The filename of the searched file.
    :param data_directory: The directory in which to search for the file.
    :return: The full path in the filesystem or None if there is no file with the filename in the directory
    """
    for file in os.listdir(data_directory):
        if file == file_name:
            return data_directory + f"/{file_name}"


def _get_robot_name_from_urdf(urdf_string: str) -> str:
    """
    Extracts the robot name from the 'robot_name' tag of a URDF.

    :param urdf_string: The URDF as string.
    :return: The name of the robot described by the URDF.
    """
    res = re.findall(r"robot\ *name\ *=\ *\"\ *[a-zA-Z_0-9]*\ *\"", urdf_string)
    if len(res) == 1:
        begin = res[0].find("\"")
        end = res[0][begin + 1:].find("\"")
        robot = res[0][begin + 1:begin + 1 + end].lower()
        return robot
    raise Exception("Robot Name not Found")


class Attachment:
    def __init__(self,
                 parent_object: Object,
                 child_object: Object,
                 parent_link_id: Optional[int] = -1,  # -1 means base link
                 child_link_id: Optional[int] = -1,
                 bidirectional: Optional[bool] = False,
                 child_to_parent_transform: Optional[Transform] = None,
                 constraint_id: Optional[int] = None):
        """
        Creates an attachment between the parent object and the child object.
        """
        self.parent_object = parent_object
        self.child_object = child_object
        self.parent_link_id = parent_link_id
        self.child_link_id = child_link_id
        self.bidirectional = bidirectional
        self._loose = False and not self.bidirectional

        self.child_to_parent_transform = child_to_parent_transform
        if self.child_to_parent_transform is None:
            self.update_transform()

        self.constraint_id = constraint_id
        if self.constraint_id is None:
            self.add_constraint_and_update_objects_constraints_collection()

    def update_attachment(self):
        self.update_transform()
        self.update_constraint()

    def update_transform(self):
        self.child_to_parent_transform = self.calculate_transform()

    def update_constraint(self):
        self.remove_constraint_if_exists()
        self.add_constraint_and_update_objects_constraints_collection()

    def calculate_transform(self):
        return self.parent_object.get_transform_from_my_link_to_other_object_link(self.parent_link_id,
                                                                                  self.child_link_id,
                                                                                  self.child_object)

    def remove_constraint_if_exists(self):
        if self.constraint_id is not None:
            self.parent_object.world.remove_constraint(self.constraint_id)

    def add_constraint_and_update_objects_constraints_collection(self):
        self.constraint_id = self.add_fixed_constraint()
        self.update_objects_constraints_collection()

    def add_fixed_constraint(self):
        constraint_id = self.parent_object.world.add_fixed_constraint(self.parent_object.id,
                                                                      self.child_object.id,
                                                                      self.child_to_parent_transform,
                                                                      self.parent_link_id,
                                                                      self.child_link_id)
        return constraint_id

    def update_objects_constraints_collection(self):
        self.parent_object.cids[self.child_object] = self.constraint_id
        self.child_object.cids[self.parent_object] = self.constraint_id

    def get_inverse(self):
        attachment = Attachment(self.child_object, self.parent_object, self.child_link_id,
                                self.parent_link_id, self.bidirectional, self.child_to_parent_transform.invert(),
                                self.constraint_id)
        attachment.loose = False if self.loose else True
        return attachment

    @property
    def loose(self) -> bool:
        """
        If true, then the child object will not move when parent moves.
        """
        return self._loose

    @loose.setter
    def loose(self, loose: bool):
        self._loose = loose and not self.bidirectional

    @property
    def is_reversed(self) -> bool:
        """
        If true means that when child moves, parent moves not the other way around.
        """
        return self.loose

    def __del__(self):
        self.remove_constraint_if_exists()


def _load_object(name: str,
                 path: str,
                 position: List[float],
                 orientation: List[float],
                 world: World,
                 color: Color,
                 ignore_cached_files: bool) -> Tuple[int, str]:
    """
    Loads an object to the given World with the given position and orientation. The color will only be
    used when an .obj or .stl file is given.
    If a .obj or .stl file is given, before spawning, an urdf file with the .obj or .stl as mesh will be created
    and this URDf file will be loaded instead.
    When spawning a URDf file a new file will be created in the cache directory, if there exists none.
    This new file will have resolved mesh file paths, meaning there will be no references
    to ROS packges instead there will be absolute file paths.

    :param name: The name of the object which should be spawned
    :param path: The path to the source file or the name on the ROS parameter server
    :param position: The position in which the object should be spawned
    :param orientation: The orientation in which the object should be spawned
    :param world: The World to which the Object should be spawned
    :param color: The color of the object, only used when .obj or .stl file is given
    :param ignore_cached_files: Whether to ignore files in the cache directory.
    :return: The unique id of the object and the path to the file used for spawning
    """
    pa = pathlib.Path(path)
    extension = pa.suffix
    world, world_id = _world_and_id(world)
    if re.match("[a-zA-Z_0-9].[a-zA-Z0-9]", path):
        for dir in world.data_directory:
            path = get_path_from_data_dir(path, dir)
            if path: break

    if not path:
        raise FileNotFoundError(
            f"File {pa.name} could not be found in the resource directory {world.data_directory}")
    # rospack = rospkg.RosPack()
    # cach_dir = rospack.get_path('pycram') + '/resources/cached/'
    cach_dir = world.data_directory[0] + '/cached/'
    if not pathlib.Path(cach_dir).exists():
        os.mkdir(cach_dir)

    # if file is not yet cached corrcet the urdf and save if in the cache directory
    if not _is_cached(path, name, cach_dir) or ignore_cached_files:
        if extension == ".obj" or extension == ".stl":
            path = _generate_urdf_file(name, path, color, cach_dir)
        elif extension == ".urdf":
            with open(path, mode="r") as f:
                urdf_string = fix_missing_inertial(f.read())
                urdf_string = remove_error_tags(urdf_string)
                urdf_string = fix_link_attributes(urdf_string)
                try:
                    urdf_string = _correct_urdf_string(urdf_string)
                except rospkg.ResourceNotFound as e:
                    rospy.logerr(f"Could not find resource package linked in this URDF")
                    raise e
            path = cach_dir + pa.name
            with open(path, mode="w") as f:
                f.write(urdf_string)
        else:  # Using the urdf from the parameter server
            urdf_string = rospy.get_param(path)
            path = cach_dir + name + ".urdf"
            with open(path, mode="w") as f:
                f.write(_correct_urdf_string(urdf_string))
    # save correct path in case the file is already in the cache directory
    elif extension == ".obj" or extension == ".stl":
        path = cach_dir + pa.stem + ".urdf"
    elif extension == ".urdf":
        path = cach_dir + pa.name
    else:
        path = cach_dir + name + ".urdf"

    try:
        obj = world.load_urdf_at_pose_and_get_object_id(path, Pose(position, orientation))
        return obj, path
    except Exception as e:
        logging.error(
            "The File could not be loaded. Plese note that the path has to be either a URDF, stl or obj file or the name of an URDF string on the parameter server.")
        os.remove(path)
        raise (e)


def _is_cached(path: str, name: str, cach_dir: str) -> bool:
    """
    Checks if the file in the given path is already cached or if
    there is already a cached file with the given name, this is the case if a .stl, .obj file or a description from
    the parameter server is used.

    :param path: The path given by the user to the source file.
    :param name: The name for this object.
    :param cach_dir: The absolute path the cach directory in the pycram package.
    :return: True if there already exists a chached file, False in any other case.
    """
    file_name = pathlib.Path(path).name
    full_path = pathlib.Path(cach_dir + file_name)
    if full_path.exists():
        return True
    # Returns filename without the filetype, e.g. returns "test" for "test.txt"
    file_stem = pathlib.Path(path).stem
    full_path = pathlib.Path(cach_dir + file_stem + ".urdf")
    if full_path.exists():
        return True
    return False


def _correct_urdf_string(urdf_string: str) -> str:
    """
    Changes paths for files in the URDF from ROS paths to paths in the file system. Since PyBullet can't deal with ROS
    package paths.

    :param urdf_string: The name of the URDf on the parameter server
    :return: The URDF string with paths in the filesystem instead of ROS packages
    """
    r = rospkg.RosPack()
    new_urdf_string = ""
    for line in urdf_string.split('\n'):
        if "package://" in line:
            s = line.split('//')
            s1 = s[1].split('/')
            path = r.get_path(s1[0])
            line = line.replace("package://" + s1[0], path)
        new_urdf_string += line + '\n'

    return fix_missing_inertial(new_urdf_string)


def fix_missing_inertial(urdf_string: str) -> str:
    """
    Insert inertial tags for every URDF link that has no inertia.
    This is used to prevent PyBullet from dumping warnings in the terminal

    :param urdf_string: The URDF description as string
    :returns: The new, corrected URDF description as string.
    """

    inertia_tree = xml.etree.ElementTree.ElementTree(xml.etree.ElementTree.Element("inertial"))
    inertia_tree.getroot().append(xml.etree.ElementTree.Element("mass", {"value": "0.1"}))
    inertia_tree.getroot().append(xml.etree.ElementTree.Element("origin", {"rpy": "0 0 0", "xyz": "0 0 0"}))
    inertia_tree.getroot().append(xml.etree.ElementTree.Element("inertia", {"ixx": "0.01",
                                                                            "ixy": "0",
                                                                            "ixz": "0",
                                                                            "iyy": "0.01",
                                                                            "iyz": "0",
                                                                            "izz": "0.01"}))

    # create tree from string
    tree = xml.etree.ElementTree.ElementTree(xml.etree.ElementTree.fromstring(urdf_string))

    for link_element in tree.iter("link"):
        inertial = [*link_element.iter("inertial")]
        if len(inertial) == 0:
            link_element.append(inertia_tree.getroot())

    return xml.etree.ElementTree.tostring(tree.getroot(), encoding='unicode')


def remove_error_tags(urdf_string: str) -> str:
    """
    Removes all tags in the removing_tags list from the URDF since these tags are known to cause errors with the
    URDF_parser

    :param urdf_string: String of the URDF from which the tags should be removed
    :return: The URDF string with the tags removed
    """
    tree = xml.etree.ElementTree.ElementTree(xml.etree.ElementTree.fromstring(urdf_string))
    removing_tags = ["gazebo", "transmission"]
    for tag_name in removing_tags:
        all_tags = tree.findall(tag_name)
        for tag in all_tags:
            tree.getroot().remove(tag)

    return xml.etree.ElementTree.tostring(tree.getroot(), encoding='unicode')


def fix_link_attributes(urdf_string: str) -> str:
    """
    Removes the attribute 'type' from links since this is not parsable by the URDF parser.

    :param urdf_string: The string of the URDF from which the attributes should be removed
    :return: The URDF string with the attributes removed
    """
    tree = xml.etree.ElementTree.ElementTree(xml.etree.ElementTree.fromstring(urdf_string))

    for link in tree.iter("link"):
        if "type" in link.attrib.keys():
            del link.attrib["type"]

    return xml.etree.ElementTree.tostring(tree.getroot(), encoding='unicode')


def _generate_urdf_file(name: str, path: str, color: Color, cach_dir: str) -> str:
    """
    Generates an URDf file with the given .obj or .stl file as mesh. In addition, the given color will be
    used to crate a material tag in the URDF. The resulting file will then be saved in the cach_dir path with the name
    as filename.

    :param name: The name of the object
    :param path: The path to the .obj or .stl file
    :param color: The color which should be used for the material tag
    :param cach_dir The absolute file path to the cach directory in the pycram package
    :return: The absolute path of the created file
    """
    urdf_template = '<?xml version="0.0" ?> \n \
                    <robot name="~a_object"> \n \
                     <link name="~a_main"> \n \
                        <visual> \n \
                            <geometry>\n \
                                <mesh filename="~b" scale="1 1 1"/> \n \
                            </geometry>\n \
                            <material name="white">\n \
                                <color rgba="~c"/>\n \
                            </material>\n \
                      </visual> \n \
                    <collision> \n \
                    <geometry>\n \
                        <mesh filename="~b" scale="1 1 1"/>\n \
                    </geometry>\n \
                    </collision>\n \
                    </link> \n \
                    </robot>'
    urdf_template = fix_missing_inertial(urdf_template)
    rgb = " ".join(list(map(str, color.get_rgba())))
    pathlib_obj = pathlib.Path(path)
    path = str(pathlib_obj.resolve())
    content = urdf_template.replace("~a", name).replace("~b", path).replace("~c", rgb)
    with open(cach_dir + pathlib_obj.stem + ".urdf", "w", encoding="utf-8") as file:
        file.write(content)
    return cach_dir + pathlib_obj.stem + ".urdf"


def _world_and_id(world: World) -> Tuple[World, int]:
    """
    Selects the world to be used. If the given world is None the 'current_world' is used.

    :param world: The world which should be used or None if 'current_world' should be used
    :return: The World object and the id of this World
    """
    world = world if world is not None else World.current_world
    client_id = world.client_id if world is not None else World.current_world.client_id
    return world, client_id
