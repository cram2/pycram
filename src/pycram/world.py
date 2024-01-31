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
from tf.transformations import quaternion_from_euler
from typing_extensions import List, Optional, Dict, Tuple, Callable
from typing_extensions import Union

import numpy as np
import rospkg
import rospy

import urdf_parser_py.urdf
from geometry_msgs.msg import Quaternion, Point
from urdf_parser_py.urdf import URDF, Collision, GeometricType

from .event import Event
from .robot_descriptions import robot_description
from .enums import JointType, ObjectType, WorldMode
from .local_transformer import LocalTransformer
from sensor_msgs.msg import JointState

from .pose import Pose, Transform

from abc import ABC, abstractmethod
from .world_dataclasses import (Color, Constraint, AxisAlignedBoundingBox, CollisionCallbacks,
                                MultiBody, VisualShape, BoxVisualShape, CylinderVisualShape, SphereVisualShape,
                                CapsuleVisualShape, PlaneVisualShape, MeshVisualShape, LinkState, ObjectState)


class World(ABC):
    """
    The World Class represents the physics Simulation and belief state, it is the main interface for reasoning about
    the World. This is implemented as a singleton, the current World can be accessed via the static variable
     current_world which is managed by the World class itself.
    """

    simulation_frequency: float
    """
    Global reference for the simulation frequency (Hz), used in calculating the equivalent real time in the simulation.
    """

    current_world: Optional[World] = None
    """
        Global reference to the currently used World, usually this is the
        graphical one. However, if you are inside a UseProspectionWorld() environment the current_world points to the
        prospection world. In this way you can comfortably use the current_world, which should point towards the World
        used at the moment.
    """

    robot: Optional[Object] = None
    """
    Global reference to the spawned Object that represents the robot. The robot is identified by checking the name in
     the URDF with the name of the URDF on the parameter server. 
    """

    data_directory: List[str] = [os.path.join(os.path.dirname(__file__), '..', '..', 'resources')]
    """
    Global reference for the data directories, this is used to search for the URDF files of the robot and the objects.
    """

    cach_dir = data_directory[0] + '/cached/'
    """
    Global reference for the cache directory, this is used to cache the URDF files of the robot and the objects.
    """

    def __init__(self, mode: WorldMode, is_prospection_world: bool, simulation_frequency: float):
        """
       Creates a new simulation, the mode decides if the simulation should be a rendered window or just run in the
       background. There can only be one rendered simulation.
       The World object also initializes the Events for attachment, detachment and for manipulating the world.

       :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default parameter is "GUI"
       :param is_prospection_world: For internal usage, decides if this World should be used as a prospection world.
        """

        if World.current_world is None:
            World.current_world = self
        World.simulation_frequency = simulation_frequency

        self.is_prospection_world: bool = is_prospection_world
        self._init_and_sync_prospection_world()

        self.local_transformer = LocalTransformer()
        self._update_local_transformer_worlds()

        self.objects: List[Object] = []
        # List of all Objects in the World

        self.client_id: int = -1
        # This is used to connect to the physics server (allows multiple clients)

        self.mode: WorldMode = mode
        # The mode of the simulation, can be "GUI" or "DIRECT"

        self.coll_callbacks: Dict[Tuple[Object, Object], CollisionCallbacks] = {}

        self._init_events()

        self.saved_states: List[int] = []

    def _init_events(self):
        """
        Initializes dynamic events that can be used to react to changes in the World.
        """
        self.detachment_event: Event = Event()
        self.attachment_event: Event = Event()
        self.manipulation_event: Event = Event()

    def _init_and_sync_prospection_world(self):
        """
        Initializes the prospection world and the synchronization between the main and the prospection world.
        """
        self._init_prospection_world()
        self._sync_prospection_world()

    def _update_local_transformer_worlds(self):
        """
        Updates the local transformer worlds with the current world and prospection world.
        """
        self.local_transformer.world = self
        self.local_transformer.prospection_world = self.prospection_world

    def _init_prospection_world(self):
        """
        Initializes the prospection world, if this is a prospection world itself it will not create another prospection,
        world, but instead set the prospection world to None, else it will create a prospection world.
        """
        if self.is_prospection_world:  # then no need to add another prospection world
            self.prospection_world = None
        else:
            self.prospection_world: World = self.__class__(WorldMode.DIRECT,
                                                           True,
                                                           World.simulation_frequency)

    def _sync_prospection_world(self):
        """
        Synchronizes the prospection world with the main world, this means that every object in the main world will be
        added to the prospection world and vice versa.
        """
        if self.is_prospection_world:  # then no need to add another prospection world
            self.world_sync = None
        else:
            self.world_sync: WorldSync = WorldSync(self, self.prospection_world)
            self.world_sync.start()

    @property
    def simulation_time_step(self):
        """
        The time step of the simulation in seconds.
        """
        return 1/World.simulation_frequency

    @abstractmethod
    def load_urdf_and_get_object_id(self, path: str, pose: Pose) -> int:
        """
        Loads a URDF file at the given pose and returns the id of the loaded object.

        :param path: The path to the URDF file.
        :param pose: The pose at which the object should be loaded.
        :return: The id of the loaded object.
        """
        pass

    # TODO: This is not used anywhere, should it be removed?
    def get_objects_by_name(self, name: str) -> List[Object]:
        """
        Returns a list of all Objects in this World with the same name as the given one.

        :param name: The name of the returned Objects.
        :return: A list of all Objects with the name 'name'.
        """
        return list(filter(lambda obj: obj.name == name, self.objects))

    def get_objects_by_type(self, obj_type: ObjectType) -> List[Object]:
        """
        Returns a list of all Objects which have the type 'obj_type'.

        :param obj_type: The type of the returned Objects.
        :return: A list of all Objects that have the type 'obj_type'.
        """
        return list(filter(lambda obj: obj.obj_type == obj_type, self.objects))

    def get_object_by_id(self, obj_id: int) -> Object:
        """
        Returns the single Object that has the unique id.

        :param obj_id: The unique id for which the Object should be returned.
        :return: The Object with the id 'id'.
        """
        return list(filter(lambda obj: obj.id == obj_id, self.objects))[0]

    @abstractmethod
    def remove_object_from_simulator(self, obj: Object) -> None:
        """
        Removes an object from the physics simulator.
        :param obj: The object to be removed.
        """
        pass

    def remove_object(self, obj: Object) -> None:
        """
        Removes this object from the current world.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to world remove object is done
        to remove this Object from the simulation/world.

        :param obj: The object to be removed.
        """
        obj.detach_all()

        self.objects.remove(obj)

        # This means the current world of the object is not the prospection world, since it
        # has a reference to the prospection world
        if self.prospection_world is not None:
            self.world_sync.remove_obj_queue.put(self)
            self.world_sync.remove_obj_queue.join()

        self.remove_object_from_simulator(obj)

        if World.robot == self:
            World.robot = None

    def add_fixed_constraint(self, parent_link: Link, child_link: Link, child_to_parent_transform: Transform) -> int:
        """
        Creates a fixed joint constraint between the given parent and child links,
        the joint frame will be at the origin of the child link frame, and would have the same orientation
        as the child link frame.

        :param parent_link: The constrained link of the parent object.
        :param child_link: The constrained link of the child object.
        :param child_to_parent_transform: The transform from the child link frame to the parent link frame.
        :return: The unique id of the created constraint.
        """

        constraint = Constraint(parent_obj_id=parent_link.get_object_id(),
                                parent_link_name=parent_link.name,
                                child_obj_id=child_link.get_object_id(),
                                child_link_name=child_link.name,
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
        Add a constraint between two objects links so that they become attached for example.

        :param constraint: The constraint data used to create the constraint.
        """
        pass

    @abstractmethod
    def remove_constraint(self, constraint_id) -> None:
        """
        Remove a constraint by its ID.

        :param constraint_id: The unique id of the constraint to be removed.
        """
        pass

    def get_object_joint_limits(self, obj: Object, joint_name: str) -> Tuple[float, float]:
        """
        Get the joint limits of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: A tuple containing the upper and the lower limits of the joint respectively.
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
        Returns the axis along/around which a joint is moving. The given joint_name has to be part of this object.

        :param obj: The object.
        :param joint_name: Name of the joint for which the axis should be returned.
        :return: The axis which is a 3D vector of xyz values.
        """
        pass

    @abstractmethod
    def get_object_joint_type(self, obj: Object, joint_name: str) -> JointType:
        """
        Returns the type of the joint as element of the Enum :mod:`~pycram.enums.JointType`.

        :param obj: The object.
        :param joint_name: Joint name for which the type should be returned.
        :return: The type of  the joint as element of the Enum :mod:`~pycram.enums.JointType`.
        """
        pass

    @abstractmethod
    def get_object_joint_position(self, obj: Object, joint_name: str) -> float:
        """
        Get the position of a joint of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: The joint position as a float.
        """
        pass

    @abstractmethod
    def get_link_pose(self, link: Link) -> Pose:
        """
        Get the pose of a link of an articulated object with respect to the world frame.

        :param link: The link as a Link object.
        :return: The pose of the link as a Pose object.
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
            curr_time = rospy.Time.now()
            self.step()
            for objects, callbacks in self.coll_callbacks.items():
                contact_points = self.get_contact_points_between_two_objects(objects[0], objects[1])
                if contact_points != ():
                    callbacks.on_collision_cb()
                elif callbacks.no_collision_cb is not None:
                    callbacks.no_collision_cb()
            if real_time:
                loop_time = rospy.Time.now() - curr_time
                time.sleep(max(0, self.simulation_time_step - loop_time.to_sec()))

    @abstractmethod
    def perform_collision_detection(self) -> None:
        """
        Checks for collisions between all objects in the World and updates the contact points.
        """
        pass

    @abstractmethod
    def get_object_contact_points(self, obj: Object) -> List:
        """
        Returns a list of contact points of this Object with all other Objects.

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

    def get_joint_rest_pose(self, obj: Object, joint_name: str) -> float:
        """
        Get the rest pose of a joint of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: The rest pose of the joint.
        """
        pass

    def get_joint_damping(self, obj: Object, joint_name: str) -> float:
        """
        Get the damping of a joint of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: The damping of the joint.
        """
        pass

    @abstractmethod
    def get_joint_names(self, obj: Object) -> List[str]:
        """
        Get the names of all joints of an articulated object.
        :param obj: The object.
        :return: A list of all joint names of the object.
        """
        pass

    @abstractmethod
    def get_link_names(self, obj: Object) -> List[str]:
        """
        Get the names of all links of an articulated object.
        :param obj: The object.
        :return: A list of all link names of the object.
        """
        pass

    def get_number_of_joints(self, obj: Object) -> int:
        """
        Get the number of joints of an articulated object
        :param obj: The object.
        :return: The number of joints of the object.
        """
        pass

    def get_number_of_links(self, obj: Object) -> int:
        """
        Get the number of links of an articulated object
        :param obj: The object.
        :return: The number of links of the object.
        """
        pass

    @abstractmethod
    def reset_joint_position(self, obj: Object, joint_name: str, joint_pose: float) -> None:
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

    def set_object_color(self, obj: Object, rgba_color: Color):
        """
        Changes the color of this object, the color has to be given as a list
        of RGBA values.

        :param obj: The object which should be colored
        :param rgba_color: The color as Color object with RGBA values between 0 and 1
        """
        # Check if there is only one link, this is the case for primitive
        # forms or if loaded from an .stl or .obj file
        if obj.links != {}:
            for link in obj.links.values():
                self.set_link_color(link, rgba_color)
        else:
            self.set_link_color(obj.get_root_link(), rgba_color)

    @abstractmethod
    def set_link_color(self, link: Link, rgba_color: Color):
        """
        Changes the rgba_color of a link of this object, the rgba_color has to be given as Color object.

        :param link: The link which should be colored.
        :param rgba_color: The rgba_color as Color object with RGBA values between 0 and 1.
        """
        pass

    @abstractmethod
    def get_link_color(self, link: Link) -> Color:
        """
        This method returns the rgba_color of this link.
        :param link: The link for which the rgba_color should be returned.
        :return: The rgba_color as Color object with RGBA values between 0 and 1.
        """
        pass

    def get_object_color(self, obj: Object) -> Union[Color, Dict[str, Color]]:
        """
        This method returns the rgba_color of this object. The return is either:

            1. A Color object with RGBA values, this is the case if the object only has one link (this
                happens for example if the object is spawned from a .obj or .stl file)
            2. A dict with the link name as key and the rgba_color as value. The rgba_color is given as a Color Object.
                Please keep in mind that not every link may have a rgba_color. This is dependent on the URDF from which
                 the object is spawned.

        :param obj: The object for which the rgba_color should be returned.
        :return: The rgba_color as Color object with RGBA values between 0 and 1 or a dict with the link name as key and
            the rgba_color as value.
        """
        link_to_color_dict = self.get_colors_of_object_links(obj)

        if len(link_to_color_dict) == 1:
            return list(link_to_color_dict.values())[0]
        else:
            return link_to_color_dict

    @abstractmethod
    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        """
        Get the RGBA colors of each link in the object as a dictionary from link name to rgba_color.

        :param obj: The object
        :return: A dictionary with link names as keys and a Color object for each link as value.
        """
        pass

    @abstractmethod
    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of this object. The return of this method are two points in
        world coordinate frame which define a bounding box.

        :param obj: The object for which the bounding box should be returned.
        :return: AxisAlignedBoundingBox object containing the min and max points of the bounding box.
        """
        pass

    @abstractmethod
    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
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
        Enables the real time simulation of Physics in the World. By default, this is disabled and Physics is only
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
        Closes the World as well as the prospection world, also collects any other thread that is running.
        :param wait_time_before_exit_in_secs: The time to wait before exiting the world in seconds.
        """
        if wait_time_before_exit_in_secs is not None:
            time.sleep(wait_time_before_exit_in_secs)
        self.exit_prospection_world_if_exists()
        self.disconnect_from_physics_server()
        self.reset_current_world()
        self.reset_robot()
        self.join_threads()

    def exit_prospection_world_if_exists(self) -> None:
        """
        Exits the prospection world if it exists.
        """
        if self.prospection_world:
            self.terminate_world_sync()
            self.prospection_world.exit()

    @abstractmethod
    def disconnect_from_physics_server(self) -> None:
        """
        Disconnects the world from the physics server.
        """
        pass

    def reset_current_world(self) -> None:
        """
        Resets the current world to None if this is the current world.
        """
        if World.current_world == self:
            World.current_world = None

    def reset_robot(self) -> None:
        """
        Sets the robot class variable to None.
        """
        self.set_robot(None)

    @abstractmethod
    def join_threads(self) -> None:
        """
        Join any running threads. Useful for example when exiting the world.
        """
        pass

    def terminate_world_sync(self) -> None:
        """
        Terminates the world sync thread.
        """
        self.world_sync.terminate = True
        self.world_sync.join()

    def save_state(self) -> int:
        """
        Returns the id of the saved state of the World. The saved state contains the states of all the objects and
        the state of the physics simulator.

        :return: A unique id of the state
        """
        state_id = self.save_physics_simulator_state()
        self.save_objects_state(state_id)
        self.saved_states.append(state_id)
        return state_id

    def save_objects_state(self, state_id: int) -> None:
        """
        Saves the state of all objects in the World according to the given state using the unique state id.
        :param state_id: The unique id representing the state.
        """
        for obj in self.objects:
            obj.save_state(state_id)

    def restore_state(self, state_id) -> None:
        """
        Restores the state of the World according to the given state using the unique state id. This includes the state
        of the physics simulator and the state of all objects.
        However, restore can not respawn objects if there are objects that were deleted between creation of the state
        and restoring, they will be skipped.

        :param state_id: The unique id representing the state.
        """
        self.restore_physics_simulator_state(state_id)
        self.restore_objects_states(state_id)

    @abstractmethod
    def save_physics_simulator_state(self) -> int:
        """
        Saves the state of the physics simulator and returns the unique id of the state.
        :return: The unique id representing the state.
        """
        pass

    @abstractmethod
    def remove_physics_simulator_state(self, state_id: int) -> None:
        """
        Removes the state of the physics simulator with the given id.
        :param state_id: The unique id representing the state.
        """
        pass

    @abstractmethod
    def restore_physics_simulator_state(self, state_id: int) -> None:
        """
        Restores the objects and environment state in the physics simulator according to
         the given state using the unique state id.
        :param state_id: The unique id representing the state.
        """
        pass

    def restore_objects_states(self, state_id: int) -> None:
        """
        Restores the state of all objects in the World according to the given state using the unique state id.
        :param state_id: The unique id representing the state.
        """
        for obj in self.objects:
            obj.restore_state(state_id)

    def get_images_for_target(self,
                              target_pose: Pose,
                              cam_pose: Pose,
                              size: Optional[int] = 256) -> List[np.ndarray]:
        """
        Calculates the view and projection Matrix and returns 3 images:

        1. An RGB image
        2. A depth image
        3. A segmentation Mask, the segmentation mask indicates for every pixel the visible Object.

        :param target_pose: The pose to which the camera should point.
        :param cam_pose: The pose of the camera.
        :param size: The height and width of the images in pixels.
        :return: A list containing an RGB and depth image as well as a segmentation mask, in this order.
        """
        pass

    def register_two_objects_collision_callbacks(self,
                                                 object_a: Object,
                                                 object_b: Object,
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
        self.coll_callbacks[(object_a, object_b)] = CollisionCallbacks(on_collision_callback,
                                                                       on_collision_removal_callback)

    @classmethod
    def add_resource_path(cls, path: str) -> None:
        """
        Adds a resource path in which the World will search for files. This resource directory is searched if an
        Object is spawned only with a filename.

        :param path: A path in the filesystem in which to search for files.
        """
        cls.data_directory.append(path)

    def get_prospection_object_for_object(self, obj: Object) -> Object:
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

    def get_object_for_prospection_object(self, prospection_object: Object) -> Object:
        """
        Returns the corresponding object from the main World for a given
        object in the prospection world. If the  given object is not in the prospection
        world an error will be raised.

        :param prospection_object: The object for which the corresponding object in the main World should be found.
        :return: The object in the main World.
        """
        object_map = self.world_sync.object_mapping
        try:
            return list(object_map.keys())[list(object_map.values()).index(prospection_object)]
        except ValueError:
            raise ValueError("The given object is not in the prospection world.")

    def reset_world(self, remove_saved_states=True) -> None:
        """
        Resets the World to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they were spawned.
        :param remove_saved_states: If the saved states should be removed.
        """

        if remove_saved_states:
            self.remove_saved_states()

        for obj in self.objects:
            obj.reset(remove_saved_states)

    def remove_saved_states(self) -> None:
        """
        Removes all saved states of the World.
        """
        for state_id in self.saved_states:
            self.remove_physics_simulator_state(state_id)
        self.saved_states = []

    def update_transforms_for_objects_in_current_world(self) -> None:
        """
        Updates transformations for all objects that are currently in :py:attr:`~pycram.world.World.current_world`.
        """
        curr_time = rospy.Time.now()
        for obj in list(self.current_world.objects):
            obj.update_link_transforms(curr_time)

    @abstractmethod
    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        """ Cast a ray and return the first object hit, if any.

        :param from_position: The starting position of the ray in Cartesian world coordinates.
        :param to_position: The ending position of the ray in Cartesian world coordinates.
        :return: The object id of the first object hit, or -1 if no object was hit.
        """
        pass

    @abstractmethod
    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        """ Cast a batch of rays and return the result for each of the rays (first object hit, if any. or -1)
         Takes optional argument num_threads to specify the number of threads to use
           to compute the ray intersections for the batch. Specify 0 to let simulator decide, 1 (default) for single
            core execution, 2 or more to select the number of threads to use.

        :param from_positions: The starting positions of the rays in Cartesian world coordinates.
        :param to_positions: The ending positions of the rays in Cartesian world coordinates.
        :param num_threads: The number of threads to use to compute the ray intersections for the batch.
        """
        pass

    def create_visual_shape(self, visual_shape: VisualShape) -> int:
        """
        Creates a visual shape in the physics simulator and returns the unique id of the created shape.
        :param visual_shape: The visual shape to be created, uses the VisualShape dataclass defined in world_dataclasses
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_multi_body_from_visual_shapes(self, visual_shape_ids: List[int], pose: Pose) -> int:
        """
        Creates a multi body from visual shapes in the physics simulator and returns the unique id of the created
        multi body.
        :param visual_shape_ids: The ids of the visual shapes that should be used to create the multi body.
        :param pose: The pose of the origin of the multi body relative to the world frame.
        :return: The unique id of the created multi body.
        """
        # Dummy paramater since these are needed to spawn visual shapes as a
        # multibody.
        num_of_shapes = len(visual_shape_ids)
        link_poses = [[0, 0, 0] for _ in range(num_of_shapes)]
        link_orientations = [[0, 0, 0, 1] for _ in range(num_of_shapes)]
        link_masses = [1.0 for _ in range(num_of_shapes)]
        link_parent = [0 for _ in range(num_of_shapes)]
        link_joints = [JointType.FIXED for _ in range(num_of_shapes)]
        link_collision = [-1 for _ in range(num_of_shapes)]
        link_joint_axis = [[1, 0, 0] for _ in range(num_of_shapes)]

        multi_body = MultiBody(base_visual_shape_index=-1, base_position=pose.position_as_list(),
                               base_orientation=pose.orientation_as_list(),
                               link_visual_shape_indices=visual_shape_ids, link_positions=link_poses,
                               link_orientations=link_orientations, link_masses=link_masses,
                               link_inertial_frame_positions=link_poses,
                               link_inertial_frame_orientations=link_orientations,
                               link_parent_indices=link_parent, link_joint_types=link_joints,
                               link_joint_axis=link_joint_axis,
                               link_collision_shape_indices=link_collision)
        return self.create_multi_body(multi_body)

    def create_multi_body(self, multi_body: MultiBody) -> int:
        """
        Creates a multi body in the physics simulator and returns the unique id of the created multi body. The multibody
        is created by joining multiple links/shapes together with joints.
        :param multi_body: The multi body to be created, uses the MultiBody dataclass defined in world_dataclasses.
        :return: The unique id of the created multi body.
        """
        raise NotImplementedError

    def create_box_visual_shape(self, shape_data: BoxVisualShape) -> int:
        """
        Creates a box visual shape in the physics simulator and returns the unique id of the created shape.
        :param shape_data: The parameters that define the box visual shape to be created,
         uses the BoxVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_cylinder_visual_shape(self, shape_data: CylinderVisualShape) -> int:
        """
        Creates a cylinder visual shape in the physics simulator and returns the unique id of the created shape.
        :param shape_data: The parameters that define the cylinder visual shape to be created,
         uses the CylinderVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_sphere_visual_shape(self, shape_data: SphereVisualShape) -> int:
        """
        Creates a sphere visual shape in the physics simulator and returns the unique id of the created shape.
        :param shape_data: The parameters that define the sphere visual shape to be created,
         uses the SphereVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_capsule_visual_shape(self, shape_data: CapsuleVisualShape) -> int:
        """
        Creates a capsule visual shape in the physics simulator and returns the unique id of the created shape.
        :param shape_data: The parameters that define the capsule visual shape to be created,
         uses the CapsuleVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_plane_visual_shape(self, shape_data: PlaneVisualShape) -> int:
        """
        Creates a plane visual shape in the physics simulator and returns the unique id of the created shape.
        :param shape_data: The parameters that define the plane visual shape to be created,
         uses the PlaneVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def create_mesh_visual_shape(self, shape_data: MeshVisualShape) -> int:
        """
        Creates a mesh visual shape in the physics simulator and returns the unique id of the created shape.
        :param shape_data: The parameters that define the mesh visual shape to be created,
         uses the MeshVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        raise NotImplementedError

    def add_text(self, text: str, position: List[float], orientation: Optional[List[float]] = None, size: float = 0.1,
                 color: Optional[Color] = Color(), life_time: Optional[float] = 0,
                 parent_object_id: Optional[int] = None) -> None:
        """
        Adds text to the world.
        :param text: The text to be added.
        :param position: The position of the text in the world.
        :param orientation: By default, debug text will always face the camera,
        automatically rotation. By specifying a text orientation (quaternion), the orientation will be fixed in
        world space or local space (when parent is specified).
        :param size: The size of the text.
        :param color: The color of the text.
        :param life_time: The lifetime in seconds of the text to remain in the world, if 0 the text will remain
         in the world until it is removed manually.
        :param parent_object_id: The id of the object to which the text should be attached.
        """
        raise NotImplementedError

    def remove_text(self, text_id: Optional[int] = None) -> None:
        """
        Removes text from the world using the given id. if no id is given all text will be removed.
        :param text_id: The id of the text to be removed.
        """
        raise NotImplementedError

    def enable_joint_force_torque_sensor(self, obj: Object, fts_joint_idx: int) -> None:
        """
        You can enable a joint force/torque sensor in each joint. Once enabled, if you perform
        a simulation step, the get_joint_force_torque will report the joint reaction forces in the fixed degrees of
        freedom: a fixed joint will measure all 6DOF joint forces/torques. A revolute/hinge joint
        force/torque sensor will measure 5DOF reaction forces along all axis except the hinge axis. The
        applied force by a joint motor is available through get_applied_joint_motor_torque.
        :param obj: The object in which the joint is located.
        :param fts_joint_idx: The index of the joint for which the force torque sensor should be enabled.
        """
        raise NotImplementedError

    def disable_joint_force_torque_sensor(self, obj: Object, joint_id: int) -> None:
        """
        Disables the force torque sensor of a joint.
        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the force torque sensor should be disabled.
        """
        raise NotImplementedError

    def get_joint_force_torque(self, obj: Object, joint_id: int) -> List[float]:
        """
        Returns the joint reaction forces and torques of the specified joint.
        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the force torque should be returned.
        :return: The joint reaction forces and torques of the specified joint.
        """
        raise NotImplementedError

    def get_applied_joint_motor_torque(self, obj: Object, joint_id: int) -> float:
        """
        Returns the applied torque by a joint motor.
        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the applied motor torque should be returned.
        :return: The applied torque by a joint motor.
        """
        raise NotImplementedError


class UseProspectionWorld:
    """
    An environment for using the prospection world, while in this environment the :py:attr:`~World.current_world`
    variable will point to the prospection world.

    Example:
        with UseProspectionWorld():
            NavigateAction.Action([[1, 0, 0], [0, 0, 0, 1]]).perform()
    """

    def __init__(self):
        self.prev_world: Optional[World] = None
        # The previous world is saved to restore it after the with block is exited.

    def __enter__(self):
        """
        This method is called when entering the with block, it will set the current world to the prospection world
        """
        if not World.current_world.is_prospection_world:
            time.sleep(20 * World.current_world.simulation_time_step)
            # blocks until the adding queue is ready
            World.current_world.world_sync.add_obj_queue.join()

            self.prev_world = World.current_world
            World.current_world.world_sync.pause_sync = True
            World.current_world = World.current_world.prospection_world

    def __exit__(self, *args):
        """
        This method is called when exiting the with block, it will restore the previous world to be the current world.
        """
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
        # Maps world to prospection world objects
        self.object_mapping: Dict[Object, Object] = {}
        self.equal_states = False

    def run(self):
        """
        Main method of the synchronization, this thread runs in a loop until the
        terminate flag is set.
        While this loop runs it continuously checks the cartesian and joint position of
        every object in the World and updates the corresponding object in the
        prospection world. When there are entries in the adding or removing queue the corresponding objects will
         be added or removed in the same iteration.
        """
        while not self.terminate:
            self.check_for_pause()
            # self.equal_states = False
            for i in range(self.add_obj_queue.qsize()):
                obj = self.add_obj_queue.get()
                # [name, type, path, position, orientation, self.world.prospection_world, rgba_color, world object]
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
    """
    Represents a link of an Object in the World.
    """
    def __init__(self,
                 _id: int,
                 urdf_link: urdf_parser_py.urdf.Link,
                 obj: Object):
        self.id: int = _id
        self.urdf_link: urdf_parser_py.urdf.Link = urdf_link
        self.object: Object = obj
        self.world: World = obj.world
        self.local_transformer: LocalTransformer = LocalTransformer()
        self.constraint_ids: Dict[Link, int] = {}
        self.saved_states: Dict[int, LinkState] = {}

    def save_state(self, state_id: int) -> None:
        """
        Saves the state of this link.
        :param state_id: The unique id of the state.
        """
        self.saved_states[state_id] = self.get_current_state()

    def restore_state(self, state_id: int) -> None:
        """
        Restores the state of this link.
        :param state_id: The unique id of the state.
        """
        self.constraint_ids = self.saved_states[state_id].constraint_ids

    def get_current_state(self) -> LinkState:
        """
        :return: The current state of this link as a LinkState object.
        """
        return LinkState(self.constraint_ids.copy())

    def add_fixed_constraint_with_link(self, child_link: Link) -> int:
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

    def remove_constraint_with_link(self, child_link: Link) -> None:
        """
        Removes the constraint between this link and the given link.
        :param child_link: The child link of the constraint that should be removed.
        """
        self.world.remove_constraint(self.constraint_ids[child_link])
        del self.constraint_ids[child_link]
        del child_link.constraint_ids[self]

    def get_object_id(self) -> int:
        """
        Returns the id of the object to which this link belongs.
        :return: The integer id of the object to which this link belongs.
        """
        return self.object.id

    @property
    def tf_frame(self) -> str:
        """
        Returns the tf frame of this link.
        :return: The tf frame of this link as a string.
        """
        return f"{self.object.tf_frame}/{self.urdf_link.name}"

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

    @property
    def transform(self) -> Transform:
        """
        The transformation from the world frame to this link frame.
        :return: A Transform object with the transformation from the world frame to this link frame.
        """
        return self.pose.to_transform(self.tf_frame)

    def get_transform_to_link(self, link: Link) -> Transform:
        """
        Returns the transformation from this link to the given link.
        :param link: The link to which the transformation should be returned.
        :return: A Transform object with the transformation from this link to the given link.
        """
        return link.get_transform_from_link(self)

    def get_transform_from_link(self, link: Link) -> Transform:
        """
        Returns the transformation from the given link to this link.
        :param link: The link from which the transformation should be returned.
        :return: A Transform object with the transformation from the given link to this link.
        """
        return self.get_pose_wrt_link(link).to_transform(self.tf_frame)

    def get_pose_wrt_link(self, link: Link) -> Pose:
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

    @property
    def pose(self) -> Pose:
        """
        The pose of the link relative to the world frame.
        :return: A Pose object containing the pose of the link relative to the world frame.
        """
        return self.world.get_link_pose(self)

    @property
    def pose_as_list(self) -> List[List[float]]:
        """
        The pose of the link relative to the world frame as a list.
        :return: A list containing the position and orientation of the link relative to the world frame.
        """
        return self.pose.to_list()

    @property
    def name(self) -> str:
        """
        The name of the link as defined in the URDF.
        :return: The name of the link as a string.
        """
        return self.urdf_link.name

    def get_geometry(self) -> GeometricType:
        """
        Returns the geometry type of the URDF collision element of this link.
        """
        return None if not self.collision else self.collision.geometry

    def get_origin_transform(self) -> Transform:
        """
        Returns the transformation between the link frame and the origin frame of this link.
        """
        return Transform(self.origin.xyz, list(quaternion_from_euler(*self.origin.rpy)))

    @property
    def origin(self) -> urdf_parser_py.urdf.Pose:
        """
        The URDF origin pose of this link.
        :return: A '~urdf_parser_py.urdf.Pose' object containing the origin pose of this link.
        """
        return self.collision.origin

    @property
    def collision(self) -> Collision:
        """
        The URDF collision element of this link which has a geometry, and origin.
        :return: A '~urdf_parser_py.urdf.Collision' object containing the collision element of this link.
        """
        return self.urdf_link.collision

    @property
    def color(self) -> Color:
        """
        The getter for the rgba_color of this link.
        :return: A Color object containing the rgba_color of this link.
        """
        return self.world.get_link_color(self)

    @color.setter
    def color(self, color: List[float]) -> None:
        """
        The setter for the color of this link, could be rgb or rgba.
        :param color: The color as a list of floats, either rgb or rgba.
        """
        self.world.set_link_color(self, Color.from_list(color))


class RootLink(Link):
    """
    Represents the root link of an Object in the World.
    It differs from the normal Link class in that the pose ande the tf_frame is the same as that of the object.
    """
    def __init__(self, obj: Object):
        super().__init__(obj.get_root_link_id(), obj.get_root_urdf_link(), obj)

    @property
    def tf_frame(self) -> str:
        """
        Returns the tf frame of the root link, which is the same as the tf frame of the object.
        :return: A string containing the tf frame of the root link.
        """
        return self.object.tf_frame

    @property
    def pose(self) -> Pose:
        """
        Returns the pose of the root link, which is the same as the pose of the object.
        :return: A Pose object containing the pose of the root link.
        """
        return self.object.get_pose()


class Object:
    """
    Represents a spawned Object in the World.
    """

    def __init__(self, name: str, obj_type: ObjectType, path: str,
                 pose: Optional[Pose] = None,
                 world: Optional[World] = None,
                 color: Optional[Color] = Color(),
                 ignore_cached_files: Optional[bool] = False):
        """
        The constructor loads the urdf file into the given World, if no World is specified the
        :py:attr:`~World.current_world` will be used. It is also possible to load .obj and .stl file into the World.
        The rgba_color parameter is only used when loading .stl or .obj files,
         for URDFs :func:`~Object.set_color` can be used.

        :param name: The name of the object
        :param obj_type: The type of the object as an ObjectType enum.
        :param path: The path to the source file, if only a filename is provided then the resourcer directories will be
         searched
        :param pose: The pose at which the Object should be spawned
        :param world: The World in which the object should be spawned,
         if no world is specified the :py:attr:`~World.current_world` will be used.
        :param color: The rgba_color with which the object should be spawned.
        :param ignore_cached_files: If true the file will be spawned while ignoring cached files.
        """

        if pose is None:
            pose = Pose()

        self.world: World = world if world is not None else World.current_world

        self.name: str = name
        self.obj_type: ObjectType = obj_type
        self.color: Color = color

        self.local_transformer = LocalTransformer()
        self.original_pose = self.local_transformer.transform_pose(pose, "map")
        self._current_pose = self.original_pose

        self.id, self.path = self._load_object_and_get_id(path, ignore_cached_files)

        self.tf_frame = ("prospection/" if self.world.is_prospection_world else "") + self.name + "_" + str(self.id)

        self._init_urdf_object()

        if self.urdf_object.name == robot_description.name:
            self.world.set_robot_if_not_set(self)

        self._init_joint_name_and_id_map()

        self._init_link_name_and_id_map()

        self._init_links()
        self.update_link_transforms()

        self.attachments: Dict[Object, Attachment] = {}
        self.saved_states: Dict[int, ObjectState] = {}
        # takes the state id as key and returns the attachments of the object at that state

        if not self.world.is_prospection_world:
            self._add_to_world_sync_obj_queue(path)

        self._init_current_positions_of_joint()

        self.world.objects.append(self)

    def _load_object_and_get_id(self, path, ignore_cached_files: bool) -> Tuple[int, str]:
        """
        Loads an object to the given World with the given position and orientation. The rgba_color will only be
        used when an .obj or .stl file is given.
        If a .obj or .stl file is given, before spawning, an urdf file with the .obj or .stl as mesh will be created
        and this URDf file will be loaded instead.
        When spawning a URDf file a new file will be created in the cache directory, if there exists none.
        This new file will have resolved mesh file paths, meaning there will be no references
        to ROS packges instead there will be absolute file paths.

        :param ignore_cached_files: Whether to ignore files in the cache directory.
        :return: The unique id of the object and the path of the file that was loaded.
        """
        pa = pathlib.Path(path)
        extension = pa.suffix
        if re.match("[a-zA-Z_0-9].[a-zA-Z0-9]", path):
            for data_dir in World.data_directory:
                path = get_path_from_data_dir(path, data_dir)
                if path:
                    break

        if not path:
            raise FileNotFoundError(
                f"File {pa.name} could not be found in the resource directory {World.data_directory}")
        if not pathlib.Path(World.cach_dir).exists():
            os.mkdir(World.cach_dir)

        # if file is not yet cached corrcet the urdf and save if in the cache directory
        if not _is_cached(path) or ignore_cached_files:
            if extension == ".obj" or extension == ".stl":
                path = self._generate_urdf_file(path)
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
                path = World.cach_dir + pa.name
                with open(path, mode="w") as f:
                    f.write(urdf_string)
            else:  # Using the urdf from the parameter server
                urdf_string = rospy.get_param(path)
                path = World.cach_dir + self.name + ".urdf"
                with open(path, mode="w") as f:
                    f.write(_correct_urdf_string(urdf_string))
        # save correct path in case the file is already in the cache directory
        elif extension == ".obj" or extension == ".stl":
            path = World.cach_dir + pa.stem + ".urdf"
        elif extension == ".urdf":
            path = World.cach_dir + pa.name
        else:
            path = World.cach_dir + self.name + ".urdf"

        try:
            obj_id = self.world.load_urdf_and_get_object_id(path, Pose(self.get_position_as_list(),
                                                                       self.get_orientation_as_list()))
            return obj_id, path
        except Exception as e:
            logging.error(
                "The File could not be loaded. Please note that the path has to be either a URDF, stl or obj file or"
                " the name of an URDF string on the parameter server.")
            os.remove(path)
            raise (e)

    def _generate_urdf_file(self, path) -> str:
        """
        Generates an URDf file with the given .obj or .stl file as mesh. In addition, the given rgba_color will be
        used to crate a material tag in the URDF. The resulting file will then be saved in the cach_dir path with
         the name as filename.

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
                                    <rgba_color rgba="~c"/>\n \
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
        rgb = " ".join(list(map(str, self.color.get_rgba())))
        pathlib_obj = pathlib.Path(path)
        path = str(pathlib_obj.resolve())
        content = urdf_template.replace("~a", self.name).replace("~b", path).replace("~c", rgb)
        with open(World.cach_dir + pathlib_obj.stem + ".urdf", "w", encoding="utf-8") as file:
            file.write(content)
        return World.cach_dir + pathlib_obj.stem + ".urdf"

    def _init_urdf_object(self) -> None:
        """
        Initializes the URDF object from the URDF file.
        """
        with open(self.path) as f:
            self.urdf_object = URDF.from_xml_string(f.read())

    def _init_joint_name_and_id_map(self) -> None:
        """
        Creates a dictionary which maps the joint names to their unique ids and vice versa.
        """
        joint_names = self.world.get_joint_names(self)
        n_joints = len(joint_names)
        self.joint_name_to_id = dict(zip(joint_names, range(n_joints)))
        self.joint_id_to_name = dict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))

    def _init_link_name_and_id_map(self) -> None:
        """
        Creates a dictionary which maps the link names to their unique ids and vice versa.
        """
        link_names = self.world.get_link_names(self)
        n_links = len(link_names)
        self.link_name_to_id: Dict[str, int] = dict(zip(link_names, range(n_links)))
        self.link_id_to_name: Dict[int, str] = dict(zip(self.link_name_to_id.values(), self.link_name_to_id.keys()))
        self.link_name_to_id[self.urdf_object.get_root()] = -1
        self.link_id_to_name[-1] = self.urdf_object.get_root()

    def _init_links(self) -> None:
        """
        Initializes the link objects from the URDF file and creates a dictionary which maps the link names to the
        corresponding link objects.
        """
        links = {}
        for urdf_link in self.urdf_object.links:
            link_name = urdf_link.name
            link_id = self.link_name_to_id[link_name]
            if link_name == self.urdf_object.get_root():
                links[link_name] = RootLink(self)
            else:
                links[link_name] = Link(link_id, urdf_link, self)
        self.links = links

    def _add_to_world_sync_obj_queue(self, path: str) -> None:
        """
        Adds this object to the objects queue of the WorldSync object of the World.
        :param path: The path of the URDF file of this object.
        """
        self.world.world_sync.add_obj_queue.put(
            [self.name, self.obj_type, path, self.get_position_as_list(), self.get_orientation_as_list(),
             self.world.prospection_world, self.color, self])

    def _init_current_positions_of_joint(self) -> None:
        """
        Initialize the cached joint position for each joint.
        """
        self._current_joints_positions = {}
        for joint_name in self.joint_name_to_id.keys():
            self._current_joints_positions[joint_name] = self.world.get_object_joint_position(self, joint_name)

    @property
    def base_origin_shift(self) -> np.ndarray:
        """
        The shift between the base of the object and the origin of the object.
        :return: A numpy array with the shift between the base of the object and the origin of the object.
        """
        return np.array(self.get_pose().position_as_list()) - np.array(self.get_base_origin().position_as_list())

    def __repr__(self):
        skip_attr = ["links", "joints", "urdf_object", "attachments"]
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
            self.saved_states = {}

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
        parent_link = self.links[parent_link] if parent_link else self.get_root_link()
        child_link = child_object.links[child_link] if child_link else child_object.get_root_link()

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

        position, orientation = pose_in_map.to_list()
        if base:
            position = np.array(position) + self.base_origin_shift

        self.world.reset_object_base_pose(self, position, orientation)
        self._current_pose = pose_in_map

        if set_attachments:
            self._set_attached_objects_poses()

    def move_base_to_origin_pos(self) -> None:
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
        self.saved_states[state_id] = ObjectState(state_id, self.attachments.copy())

    def save_links_states(self, state_id: int) -> None:
        """
        Saves the state of all links of this object.
        :param state_id: The unique id of the state.
        """
        for link in self.links.values():
            link.save_state(state_id)

    def restore_state(self, state_id: int) -> None:
        """
        Restores the state of this object by restoring the state of all links and attachments.
        :param state_id: The unique id of the state.
        """
        self.restore_links_states(state_id)
        self.restore_attachments(state_id)

    def restore_attachments(self, state_id) -> None:
        """
        Restores the attachments of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        self.attachments = self.saved_states[state_id].attachments

    def restore_links_states(self, state_id) -> None:
        """
        Restores the states of all links of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        for link in self.links.values():
            link.restore_state(state_id)

    def remove_saved_states(self) -> None:
        """
        Removes all saved states of this object.
        """
        self.saved_states = {}
        for link in self.links.values():
            link.saved_states = {}

    def _set_attached_objects_poses(self, already_moved_objects: Optional[List[Object]] = None) -> None:
        """
        Updates the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this the _set_attached_objects method of all attached objects
        will be called.

        :param already_moved_objects: A list of Objects that were already moved, these will be excluded to prevent
         loops in the update.
        """

        if already_moved_objects is None:
            already_moved_objects = []

        for child in self.attachments:

            if child in already_moved_objects:
                continue

            attachment = self.attachments[child]
            if not attachment.bidirectional:
                self.update_attachment_with_object(child)
                child.update_attachment_with_object(self)

            else:
                link_to_object = attachment.parent_to_child_transform
                child.set_pose(link_to_object.to_pose(), set_attachments=False)
                child._set_attached_objects_poses(already_moved_objects + [self])

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
        elif isinstance(position, Point):
            target_position = position
        elif isinstance(position, list):
            target_position = position
        else:
            raise TypeError("The given position has to be a Pose, Point or a list of xyz.")

        pose.position = target_position
        pose.orientation = self.get_orientation()
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

    def get_joint_id(self, name: str) -> int:
        """
        Returns the unique id for a joint name. As used by the world/simulator.

        :param name: The joint name
        :return: The unique id
        """
        return self.joint_name_to_id[name]

    def get_root_urdf_link(self) -> urdf_parser_py.urdf.Link:
        """
        Returns the root link of the URDF of this object.
        :return: The root link as defined in the URDF of this object.
        """
        link_name = self.urdf_object.get_root()
        for link in self.urdf_object.links:
            if link.name == link_name:
                return link

    def get_root_link(self) -> Link:
        """
        Returns the root link of this object.
        :return: The root link of this object.
        """
        return self.links[self.urdf_object.get_root()]

    def get_root_link_id(self) -> int:
        """
        Returns the unique id of the root link of this object.
        :return: The unique id of the root link of this object.
        """
        return self.get_link_id(self.urdf_object.get_root())

    def get_link_id(self, link_name: str) -> int:
        """
        Returns a unique id for a link name.
        :param link_name: The name of the link.
        :return: The unique id of the link.
        """
        assert link_name is not None
        return self.link_name_to_id[link_name]

    def get_link_by_id(self, link_id: int) -> Link:
        """
        Returns the link for a given unique link id
        :param link_id: The unique id of the link.
        :return: The link object.
        """
        return self.links[self.link_id_to_name[link_id]]

    def get_joint_by_id(self, joint_id: int) -> str:
        """
        Returns the joint name for a unique world id

        :param joint_id: The world id of for joint
        :return: The joint name
        """
        return dict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))[joint_id]

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
        """
        for joint_name, joint_position in joint_poses.items():
            self.world.reset_joint_position(self, joint_name, joint_position)
            self._current_joints_positions[joint_name] = joint_position
        self._set_attached_objects_poses()

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
        self.world.reset_joint_position(self, joint_name, joint_position)
        self._current_joints_positions[joint_name] = joint_position
        self._set_attached_objects_poses()

    def get_joint_position(self, joint_name: str) -> float:
        """
        Returns the joint position for the given joint name.

        :param joint_name: The name of the joint
        :return: The current pose of the joint
        """
        return self._current_joints_positions[joint_name]

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

    def update_joints_from_topic(self, topic_name: str) -> None:
        """
        Updates the joints of this object with positions obtained from a topic with the message type JointState.
        Joint names on the topic have to correspond to the joints of this object otherwise an error message will be
         logged.

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

    def set_color(self, color: Color) -> None:
        """
        Changes the rgba_color of this object, the rgba_color has to be given as a list
        of RGBA values. All links of this object will be colored.

        :param color: The rgba_color as RGBA values between 0 and 1
        """
        self.world.set_object_color(self, color)

    def get_color(self) -> Union[Color, Dict[str, Color]]:
        """
        :return: The rgba_color of this object or a dictionary of link names and their colors.
        """
        return self.world.get_object_color(self)

    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis aligned bounding box of this object.
        """
        return self.world.get_object_axis_aligned_bounding_box(self)

    def get_base_origin(self) -> Pose:
        """
        :return: the origin of the base/bottom of this object.
        """
        aabb = self.get_link_by_id(-1).get_axis_aligned_bounding_box()
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
        Traverses the chain from 'link' to the URDF origin and returns the first joint that is of type 'joint_type'.

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

    def update_link_transforms(self, transform_time: Optional[rospy.Time] = None) -> None:
        """
        Updates the transforms of all links of this object using time 'transform_time' or the current ros time.
        """
        for link in self.links.values():
            link.update_transform(transform_time)


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
                 parent_link: Link,
                 child_link: Link,
                 bidirectional: Optional[bool] = False,
                 parent_to_child_transform: Optional[Transform] = None,
                 constraint_id: Optional[int] = None):
        """
        Creates an attachment between the parent object link and the child object link.
        This could be a bidirectional attachment, meaning that both objects will move when one moves.
        :param parent_link: The parent object link.
        :param child_link: The child object link.
        :param bidirectional: If true, both objects will move when one moves.
        :param parent_to_child_transform: The transform from the parent link to the child object link.
        :param constraint_id: The id of the constraint in the simulator.
        """
        self.parent_link: Link = parent_link
        self.child_link: Link = child_link
        self.bidirectional: bool = bidirectional
        self._loose: bool = False and not bidirectional

        self.parent_to_child_transform: Transform = parent_to_child_transform
        if self.parent_to_child_transform is None:
            self.update_transform()

        self.constraint_id: int = constraint_id
        if self.constraint_id is None:
            self.add_fixed_constraint()

    def update_transform_and_constraint(self) -> None:
        """
        Updates the transform and constraint of this attachment.
        """
        self.update_transform()
        self.update_constraint()

    def update_transform(self) -> None:
        """
        Updates the transform of this attachment by calculating the transform from the parent link to the child link.
        """
        self.parent_to_child_transform = self.calculate_transform()

    def update_constraint(self) -> None:
        """
        Updates the constraint of this attachment by removing the old constraint if one exists and adding a new one.
        """
        self.remove_constraint_if_exists()
        self.add_fixed_constraint()

    def add_fixed_constraint(self) -> None:
        """
        Adds a fixed constraint between the parent link and the child link.
        """
        cid = self.parent_link.add_fixed_constraint_with_link(self.child_link)
        self.constraint_id = cid

    def calculate_transform(self) -> Transform:
        """
        Calculates the transform from the parent link to the child link.
        """
        return self.parent_link.get_transform_to_link(self.child_link)

    def remove_constraint_if_exists(self) -> None:
        """
        Removes the constraint between the parent and the child links if one exists.
        """
        if self.child_link in self.parent_link.constraint_ids:
            self.parent_link.remove_constraint_with_link(self.child_link)

    def get_inverse(self) -> Attachment:
        """
        :return: A new Attachment object with the parent and child links swapped.
        """
        attachment = Attachment(self.child_link, self.parent_link, self.bidirectional,
                                constraint_id=self.constraint_id)
        attachment.loose = False if self.loose else True
        return attachment

    @property
    def loose(self) -> bool:
        """
        If true, then the child object will not move when parent moves.
        """
        return self._loose

    @loose.setter
    def loose(self, loose: bool) -> None:
        """
        Sets the loose property of this attachment.
        :param loose: If true, then the child object will not move when parent moves.
        """
        self._loose = loose and not self.bidirectional

    @property
    def is_reversed(self) -> bool:
        """
        :return: True if the parent and child links are swapped.
        """
        return self.loose

    def __del__(self) -> None:
        """
        Removes the constraint between the parent and the child links if one exists when the attachment is deleted.
        """
        self.remove_constraint_if_exists()


def _correct_urdf_string(urdf_string: str) -> str:
    """
    Changes paths for files in the URDF from ROS paths to paths in the file system. Since World (PyBullet legac)
     can't deal with ROS package paths.

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
    This is used to prevent Legacy(PyBullet) from dumping warnings in the terminal

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


def _is_cached(path) -> bool:
    """
    Checks if the file in the given path is already cached or if
    there is already a cached file with the given name, this is the case if a .stl, .obj file or a description from
    the parameter server is used.

    :return: True if there already exists a cached file, False in any other case.
    """
    file_name = pathlib.Path(path).name
    full_path = pathlib.Path(World.cach_dir + file_name)
    if full_path.exists():
        return True
    # Returns filename without the filetype, e.g. returns "test" for "test.txt"
    file_stem = pathlib.Path(path).stem
    full_path = pathlib.Path(World.cach_dir + file_stem + ".urdf")
    if full_path.exists():
        return True
    return False


def _world_and_id(world: World) -> Tuple[World, int]:
    """
    Selects the world to be used. If the given world is None the 'current_world' is used.

    :param world: The world which should be used or None if 'current_world' should be used
    :return: The World object and the id of this World
    """
    world = world if world is not None else World.current_world
    client_id = world.client_id if world is not None else World.current_world.client_id
    return world, client_id
