# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import logging
import os
import pathlib
import re
import threading
import time
from abc import ABC, abstractmethod
from queue import Queue

import numpy as np
import rospy
from geometry_msgs.msg import Quaternion, Point
from typing_extensions import List, Optional, Dict, Tuple, Callable, Any, Type
from typing_extensions import Union

from .enums import JointType, ObjectType, WorldMode, Shape
from .event import Event
from .local_transformer import LocalTransformer
from .pose import Pose, Transform
from .robot_descriptions import robot_description
from .world_dataclasses import (Color, AxisAlignedBoundingBox, CollisionCallbacks,
                                MultiBody, VisualShape, BoxVisualShape, CylinderVisualShape, SphereVisualShape,
                                CapsuleVisualShape, PlaneVisualShape, MeshVisualShape,
                                LinkState, ObjectState, JointState, State, WorldState)


class StateEntity:
    """
    The StateEntity class is used to store the state of an object or the physics simulator. This is used to save and
    restore the state of the World.
    """

    def __init__(self):
        self._saved_states: Dict[int, State] = {}

    @property
    def saved_states(self) -> Dict[int, State]:
        """
        Returns the saved states of this entity.
        """
        return self._saved_states

    def save_state(self, state_id: int) -> int:
        """
        Saves the state of this entity with the given state id.

        :param state_id: The unique id of the state.
        """
        self._saved_states[state_id] = self.current_state
        return state_id

    @property
    @abstractmethod
    def current_state(self) -> State:
        """
        Returns the current state of this entity.

        :return: The current state of this entity.
        """
        pass

    @current_state.setter
    @abstractmethod
    def current_state(self, state: State) -> None:
        """
        Sets the current state of this entity.

        :param state: The new state of this entity.
        """
        pass

    def restore_state(self, state_id: int) -> None:
        """
        Restores the state of this entity from a saved state using the given state id.

        :param state_id: The unique id of the state.
        """
        self.current_state = self.saved_states[state_id]

    def remove_saved_states(self) -> None:
        """
        Removes all saved states of this entity.
        """
        self._saved_states = {}


class World(StateEntity, ABC):
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

    cache_dir = data_directory[0] + '/cached/'
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

        StateEntity.__init__(self)

        if World.current_world is None:
            World.current_world = self
        World.simulation_frequency = simulation_frequency

        self.cache_manager = CacheManager()

        self.is_prospection_world: bool = is_prospection_world
        self._init_and_sync_prospection_world()

        self.local_transformer = LocalTransformer()
        self._update_local_transformer_worlds()

        self.objects: List[Object] = []
        # List of all Objects in the World

        self.id: int = -1
        # This is used to connect to the physics server (allows multiple clients)

        self.mode: WorldMode = mode
        # The mode of the simulation, can be "GUI" or "DIRECT"

        self.coll_callbacks: Dict[Tuple[Object, Object], CollisionCallbacks] = {}

        self._init_events()

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

    def update_cache_dir_with_object(self, path: str, ignore_cached_files: bool,
                                     obj: Object) -> str:
        """
        Updates the cache directory with the given object.

        :param path: The path to the object.
        :param ignore_cached_files: If the cached files should be ignored.
        :param obj: The object to be added to the cache directory.
        """
        return self.cache_manager.update_cache_dir_with_object(path, ignore_cached_files, obj.description, obj.name)

    @property
    def simulation_time_step(self):
        """
        The time step of the simulation in seconds.
        """
        return 1 / World.simulation_frequency

    @abstractmethod
    def load_description_and_get_object_id(self, path: str, pose: Pose) -> int:
        """
        Loads a description file (e.g. URDF) at the given pose and returns the id of the loaded object.

        :param path: The path to the description file.
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
            self.world_sync.remove_obj_queue.put(obj)
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

        constraint = Constraint(parent_link=parent_link,
                                child_link=child_link,
                                _type=JointType.FIXED,
                                axis_in_child_frame=Point(0, 0, 0),
                                constraint_to_parent=child_to_parent_transform,
                                child_to_constraint=Transform(frame=child_link.tf_frame)
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

    @abstractmethod
    def get_joint_position(self, obj: Object, joint_name: str) -> float:
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

        :param link: The link as a AbstractLink object.
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
        self.update_all_objects_poses()

    def update_all_objects_poses(self) -> None:
        """
        Updates the positions of all objects in the world.
        """
        for obj in self.objects:
            obj.update_pose()

    @abstractmethod
    def get_object_pose(self, obj: Object) -> Pose:
        """
        Get the pose of an object in the world frame from the current object pose in the simulator.
        """
        pass

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
    def reset_object_base_pose(self, obj: Object, pose: Pose):
        """
        Reset the world position and orientation of the base of the object instantaneously,
        not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

        :param obj: The object.
        :param pose: The new pose as a Pose object.
        """
        pass

    @abstractmethod
    def step(self):
        """
        Step the world simulation using forward dynamics
        """
        pass

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

    def exit(self) -> None:
        """
        Closes the World as well as the prospection world, also collects any other thread that is running.
        """
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

    def save_state(self, state_id: Optional[int] = None) -> int:
        """
        Returns the id of the saved state of the World. The saved state contains the states of all the objects and
        the state of the physics simulator.

        :return: A unique id of the state
        """
        state_id = self.save_physics_simulator_state()
        self.save_objects_state(state_id)
        self._current_state = WorldState(state_id, self.object_states)
        return super().save_state(state_id)

    @property
    def current_state(self) -> WorldState:
        return self._current_state

    @current_state.setter
    def current_state(self, state: WorldState) -> None:
        self.restore_physics_simulator_state(state.simulator_state_id)
        self.object_states = state.object_states

    @property
    def object_states(self) -> Dict[int, ObjectState]:
        """
        Returns the states of all objects in the World.
        :return: A dictionary with the object id as key and the object state as value.
        """
        return {obj.id: obj.current_state for obj in self.objects}

    @object_states.setter
    def object_states(self, states: Dict[int, ObjectState]) -> None:
        """
        Sets the states of all objects in the World.
        """
        for obj_id, obj_state in states.items():
            self.get_object_by_id(obj_id).current_state = obj_state

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
        super().remove_saved_states()

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
        # Dummy parameter since these are needed to spawn visual shapes as a
        # multibody.
        num_of_shapes = len(visual_shape_ids)
        link_poses = [Pose() for _ in range(num_of_shapes)]
        link_masses = [1.0 for _ in range(num_of_shapes)]
        link_parent = [0 for _ in range(num_of_shapes)]
        link_joints = [JointType.FIXED for _ in range(num_of_shapes)]
        link_collision = [-1 for _ in range(num_of_shapes)]
        link_joint_axis = [Point(1, 0, 0) for _ in range(num_of_shapes)]

        multi_body = MultiBody(base_visual_shape_index=-1, base_pose=pose,
                               link_visual_shape_indices=visual_shape_ids, link_poses=link_poses,
                               link_masses=link_masses,
                               link_inertial_frame_poses=link_poses,
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
                 parent_object_id: Optional[int] = None, parent_link_id: Optional[int] = None) -> int:
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
        :param parent_link_id: The id of the link to which the text should be attached.
        :return: The id of the added text.
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
        a simulation step, the get_joint_reaction_force_torque will report the joint reaction forces in
        the fixed degrees of freedom: a fixed joint will measure all 6DOF joint forces/torques.
        A revolute/hinge joint force/torque sensor will measure 5DOF reaction forces along all axis except
        the hinge axis. The applied force by a joint motor is available through get_applied_joint_motor_torque.
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

    def get_joint_reaction_force_torque(self, obj: Object, joint_id: int) -> List[float]:
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

    def run(self, wait_time_as_n_simulation_steps: Optional[int] = 1):
        """
        Main method of the synchronization, this thread runs in a loop until the
        terminate flag is set.
        While this loop runs it continuously checks the cartesian and joint position of
        every object in the World and updates the corresponding object in the
        prospection world. When there are entries in the adding or removing queue the corresponding objects will
         be added or removed in the same iteration.
        :param wait_time_as_n_simulation_steps: The time in simulation steps to wait between each iteration of the
         syncing loop.
        """
        while not self.terminate:
            self.check_for_pause()
            # self.equal_states = False
            for i in range(self.add_obj_queue.qsize()):
                obj = self.add_obj_queue.get()
                # [0:name, 1:type, 2:path, 3:description, 4:position, 5:orientation,
                # 6:self.world.prospection_world, 7:rgba_color, 8:world object]
                o = Object(obj[0], obj[1], obj[2], obj[3], Pose(obj[4], obj[5]), obj[6], obj[7])
                # Maps the World object to the prospection world object
                self.object_mapping[obj[8]] = o
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
                            prospection_obj.set_joint_positions(world_obj.get_positions_of_all_joints())
                            break

            self.check_for_pause()
            # self.check_for_equal()
            time.sleep(wait_time_as_n_simulation_steps * self.world.simulation_time_step)

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


class WorldEntity(StateEntity, ABC):
    """
    A data class that represents an entity of the world, such as an object or a link.
    """

    def __init__(self, _id: int, world: Optional[World] = None):
        StateEntity.__init__(self)
        self.id = _id
        self.world: World = world if world is not None else World.current_world


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
                 description: Type[ObjectDescription],
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
        :param path: The path to the source file, if only a filename is provided then the resources directories will be
         searched.
        :param description: The ObjectDescription of the object, this contains the joints and links of the object.
        :param pose: The pose at which the Object should be spawned
        :param world: The World in which the object should be spawned,
         if no world is specified the :py:attr:`~World.current_world` will be used.
        :param color: The rgba_color with which the object should be spawned.
        :param ignore_cached_files: If true the file will be spawned while ignoring cached files.
        """

        super().__init__(-1, world)

        if pose is None:
            pose = Pose()

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
                         + f"{self.name}_{self.id}")

        self._update_object_description_from_file(self.path)

        if self.description.name == robot_description.name:
            self.world.set_robot_if_not_set(self)

        self._init_joint_name_and_id_map()
        self._init_link_name_and_id_map()

        self._init_links_and_update_transforms()
        self._init_joints()

        self.attachments: Dict[Object, Attachment] = {}

        if not self.world.is_prospection_world:
            self._add_to_world_sync_obj_queue(self.path)

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

        path = self.world.update_cache_dir_with_object(path, ignore_cached_files, self)

        try:
            obj_id = self.world.load_description_and_get_object_id(path, Pose(self.get_position_as_list(),
                                                                              self.get_orientation_as_list()))
            return obj_id, path
        except Exception as e:
            logging.error(
                "The File could not be loaded. Please note that the path has to be either a URDF, stl or obj file or"
                " the name of an URDF string on the parameter server.")
            os.remove(path)
            raise (e)

    def _update_object_description_from_file(self, path: str) -> None:
        """
        Updates the object description from the given file path.
        :param path: The path to the file from which the object description should be updated.
        """
        self.description.update_description_from_file(path)

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
        n_links = len(self.link_names_without_root)
        self.link_name_to_id: Dict[str, int] = dict(zip(self.link_names_without_root, range(n_links)))
        self.link_name_to_id[self.description.get_root()] = -1
        self.link_id_to_name: Dict[int, str] = dict(zip(self.link_name_to_id.values(), self.link_name_to_id.keys()))

    def _init_links_and_update_transforms(self) -> None:
        """
        Initializes the link objects from the URDF file and creates a dictionary which maps the link names to the
        corresponding link objects.
        """
        self.links = {}
        for link_description in self.description.links:
            link_name = link_description.name
            link_id = self.link_name_to_id[link_name]
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
        for joint_description in self.description.joints:
            joint_name = joint_description.name
            joint_id = self.joint_name_to_id[joint_name]
            self.joints[joint_name] = self.description.Joint(joint_id, joint_description, self)

    def _add_to_world_sync_obj_queue(self, path: str) -> None:
        """
        Adds this object to the objects queue of the WorldSync object of the World.
        :param path: The path of the URDF file of this object.
        """
        self.world.world_sync.add_obj_queue.put(
            [self.name, self.obj_type, path, type(self.description), self.get_position_as_list(),
             self.get_orientation_as_list(),
             self.world.prospection_world, self.color, self])

    @property
    def link_names_without_root(self):
        """
        :return: The name of each link except the root link as a list.
        """
        return list(filter(lambda x: x != self.root_link_name, self.link_names))

    @property
    def link_names(self) -> List[str]:
        """
        :return: The name of each link as a list.
        """
        return [link.name for link in self.description.links]

    @property
    def number_of_links(self) -> int:
        """
        :return: The number of links of this object.
        """
        return len(self.description.links)

    @property
    def joint_names(self) -> List[str]:
        """
        :return: The name of each joint as a list.
        """
        return [joint.name for joint in self.description.joints]

    @property
    def number_of_joints(self) -> int:
        """
        :return: The number of joints of this object.
        """
        return len(self.description.joints)

    @property
    def base_origin_shift(self) -> np.ndarray:
        """
        The shift between the base of the object and the origin of the object.
        :return: A numpy array with the shift between the base of the object and the origin of the object.
        """
        return np.array(self.get_pose().position_as_list()) - np.array(self.get_base_origin().position_as_list())

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
            pose_in_map.position = np.array(pose_in_map.position) + self.base_origin_shift

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

    def _update_all_joints_positions(self):
        """
        Updates the posisitons of all joints by getting them from the simulator.
        """
        for joint in self.joints.values():
            joint._update_position()

    def _update_all_links_poses(self):
        """
        Updates the poses of all links by getting them from the simulator.
        """
        for link in self.links.values():
            link._update_pose()

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
        return ObjectState(self.attachments.copy(), self.link_states, self.joint_states)

    @current_state.setter
    def current_state(self, state: ObjectState) -> None:
        self.attachments = state.attachments
        self.link_states = state.link_states
        self.joint_states = state.joint_states

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

    def restore_state(self, state_id: int) -> None:
        """
        Restores the state of this object by restoring the state of all links and attachments.
        :param state_id: The unique id of the state.
        """
        self.restore_attachments(state_id)
        self.restore_links_states(state_id)
        self.restore_joints_states(state_id)

    def restore_attachments(self, state_id: int) -> None:
        """
        Restores the attachments of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        self.attachments = self.saved_states[state_id].attachments

    def restore_links_states(self, state_id: int) -> None:
        """
        Restores the states of all links of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        for link in self.links.values():
            link.restore_state(state_id)

    def restore_joints_states(self, state_id: int) -> None:
        """
        Restores the states of all joints of this object from a saved state using the given state id.
        :param state_id: The unique id of the state.
        """
        for joint in self.joints.values():
            joint.restore_state(state_id)

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

    def get_root_link_description(self) -> LinkDescription:
        """
        Returns the root link of the URDF of this object.
        :return: The root link as defined in the URDF of this object.
        """
        root_link_name = self.description.get_root()
        for link_description in self.description.links:
            if link_description.name == root_link_name:
                return link_description

    @property
    def root_link(self) -> Link:
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
        return self.joints[joint_name].position

    def get_joint_damping(self, joint_name: str) -> float:
        return self.joints[joint_name].damping

    def get_joint_upper_limit(self, joint_name: str) -> float:
        return self.joints[joint_name].upper_limit

    def get_joint_lower_limit(self, joint_name: str) -> float:
        return self.joints[joint_name].lower_limit

    def get_joint_axis(self, joint_name: str) -> Point:
        return self.joints[joint_name].axis

    def get_joint_type(self, joint_name: str) -> JointType:
        return self.joints[joint_name].type

    def get_joint_limits(self, joint_name: str) -> Tuple[float, float]:
        return self.joints[joint_name].limits

    def find_joint_above(self, link_name: str, joint_type: JointType) -> str:
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

    def get_positions_of_all_joints(self) -> Dict[str: float]:
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

        :return: The rgba_color as Color object with RGBA values between 0 and 1 or a dict with the link name as key and
            the rgba_color as value.
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
        aabb = self.get_link_by_id(-1).get_axis_aligned_bounding_box()
        base_width = np.absolute(aabb.min_x - aabb.max_x)
        base_length = np.absolute(aabb.min_y - aabb.max_y)
        return Pose([aabb.min_x + base_width / 2, aabb.min_y + base_length / 2, aabb.min_z],
                    self.get_pose().orientation_as_list())


def filter_contact_points(contact_points, exclude_ids) -> List:
    """
    Returns a list of contact points where Objects that are in the 'exclude_ids' list are removed.

    :param contact_points: A list of contact points
    :param exclude_ids: A list of unique ids of Objects that should be removed from the list
    :return: A list containing 'contact_points' without Objects that are in 'exclude_ids'
    """
    return list(filter(lambda cp: cp[2] not in exclude_ids, contact_points))


class EntityDescription(ABC):

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
    def geometry(self) -> Shape:
        """
        Returns the geometry type of the URDF collision element of this link.
        """
        pass


class JointDescription(EntityDescription):
    """
    A class that represents a joint description of a URDF joint.
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

    def __repr__(self):
        return self.__class__.__qualname__ + f"(" + ', \n'.join(
            [f"{key}={value}" for key, value in self.__dict__.items()]) + ")"

    def __str__(self):
        return self.__repr__()


class AbstractConstraint:
    """
    Represents an abstract constraint concept, this could be used to create joints for example or any kind of constraint
    between two links in the world.
    """

    def __init__(self,
                 parent_link: Link,
                 child_link: Link,
                 _type: JointType,
                 parent_to_constraint: Transform,
                 child_to_constraint: Transform):
        self.parent_link: Link = parent_link
        self.child_link: Link = child_link
        self.type: JointType = _type
        self.parent_to_constraint = parent_to_constraint
        self.child_to_constraint = child_to_constraint
        self._parent_to_child = None

    @property
    def parent_to_child_transform(self) -> Union[Transform, None]:
        if self._parent_to_child is None:
            if self.parent_to_constraint is not None and self.child_to_constraint is not None:
                self._parent_to_child = self.parent_to_constraint * self.child_to_constraint.invert()
        return self._parent_to_child

    @parent_to_child_transform.setter
    def parent_to_child_transform(self, transform: Transform) -> None:
        self._parent_to_child = transform

    @property
    def parent_object_id(self) -> int:
        """
        Returns the id of the parent object of the constraint.

        :return: The id of the parent object of the constraint
        """
        return self.parent_link.object_id

    @property
    def child_object_id(self) -> int:
        """
        Returns the id of the child object of the constraint.

        :return: The id of the child object of the constraint
        """
        return self.child_link.object_id

    @property
    def parent_link_id(self) -> int:
        """
        Returns the id of the parent link of the constraint.

        :return: The id of the parent link of the constraint
        """
        return self.parent_link.id

    @property
    def child_link_id(self) -> int:
        """
        Returns the id of the child link of the constraint.

        :return: The id of the child link of the constraint
        """
        return self.child_link.id

    @property
    def position_wrt_parent_as_list(self) -> List[float]:
        """
        Returns the constraint frame pose with respect to the parent origin as a list.

        :return: The constraint frame pose with respect to the parent origin as a list
        """
        return self.pose_wrt_parent.position_as_list()

    @property
    def orientation_wrt_parent_as_list(self) -> List[float]:
        """
        Returns the constraint frame orientation with respect to the parent origin as a list.

        :return: The constraint frame orientation with respect to the parent origin as a list
        """
        return self.pose_wrt_parent.orientation_as_list()

    @property
    def pose_wrt_parent(self) -> Pose:
        """
        Returns the joint frame pose with respect to the parent origin.

        :return: The joint frame pose with respect to the parent origin
        """
        return self.parent_to_constraint.to_pose()

    @property
    def position_wrt_child_as_list(self) -> List[float]:
        """
        Returns the constraint frame pose with respect to the child origin as a list.

        :return: The constraint frame pose with respect to the child origin as a list
        """
        return self.pose_wrt_child.position_as_list()

    @property
    def orientation_wrt_child_as_list(self) -> List[float]:
        """
        Returns the constraint frame orientation with respect to the child origin as a list.

        :return: The constraint frame orientation with respect to the child origin as a list
        """
        return self.pose_wrt_child.orientation_as_list()

    @property
    def pose_wrt_child(self) -> Pose:
        """
        Returns the joint frame pose with respect to the child origin.

        :return: The joint frame pose with respect to the child origin
        """
        return self.child_to_constraint.to_pose()


class Constraint(AbstractConstraint):
    """
    Represents a constraint between two links in the World.
    """

    def __init__(self,
                 parent_link: Link,
                 child_link: Link,
                 _type: JointType,
                 axis_in_child_frame: Point,
                 constraint_to_parent: Transform,
                 child_to_constraint: Transform):
        parent_to_constraint = constraint_to_parent.invert()
        AbstractConstraint.__init__(self, parent_link, child_link, _type, parent_to_constraint, child_to_constraint)
        self.axis: Point = axis_in_child_frame

    @property
    def axis_as_list(self) -> List[float]:
        """
        Returns the axis of this constraint as a list.

        :return: The axis of this constraint as a list of xyz
        """
        return [self.axis.x, self.axis.y, self.axis.z]


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
        self._current_position = self.world.get_joint_position(self.object, self.name)

    @property
    def parent_link(self) -> Link:
        """
        Returns the parent link of this joint.
        :return: The parent link as a AbstractLink object.
        """
        return self.object.links[self.parent_link_name]

    @property
    def child_link(self) -> Link:
        """
        Returns the child link of this joint.
        :return: The child link as a AbstractLink object.
        """
        return self.object.links[self.child_link_name]

    @property
    def position(self) -> float:
        return self._current_position

    def reset_position(self, position: float) -> None:
        self.world.reset_joint_position(self.object, self.name, position)
        self._current_position = position
        self._update_position()
        # if self.name == "r_upper_arm_roll_joint":
        #     if position == -1.463:
        #         print("here")
        #     print(self.object.name)
        #     print(self.object.world.id)
        #     print(self.name)
        #     print("required", position)
        #     print("actual", self._current_position)
        #     print(position == self._current_position)
        #     print("===================================")

    def get_object_id(self) -> int:
        """
        Returns the id of the object to which this joint belongs.
        :return: The integer id of the object to which this joint belongs.
        """
        return self.object.id

    @position.setter
    def position(self, joint_position: float) -> None:
        """
        Sets the position of the given joint to the given joint pose. If the pose is outside the joint limits, as stated
        in the URDF, an error will be printed. However, the joint will be set either way.

        :param joint_position: The target pose for this joint
        """
        # TODO Limits for rotational (infinitie) joints are 0 and 1, they should be considered seperatly
        if self.has_limits:
            low_lim, up_lim = self.limits
            if not low_lim <= joint_position <= up_lim:
                logging.error(
                    f"The joint position has to be within the limits of the joint. The joint limits for {self.name}"
                    f" are {low_lim} and {up_lim}")
                logging.error(f"The given joint position was: {joint_position}")
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

    def restore_state(self, state_id: int) -> None:
        self.current_state = self.saved_states[state_id]

    @property
    def current_state(self) -> JointState:
        return JointState(self.position)

    @current_state.setter
    def current_state(self, joint_state: JointState) -> None:
        self.position = joint_state.position


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
    def color(self, color: List[float]) -> None:
        """
        The setter for the color of this link, could be rgb or rgba.
        :param color: The color as a list of floats, either rgb or rgba.
        """
        self.world.set_link_color(self, Color.from_list(color))

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


class ObjectDescription(EntityDescription):
    MESH_EXTENSIONS: Tuple[str] = (".obj", ".stl")

    class Link(Link, ABC):
        ...

    class RootLink(RootLink, ABC):
        ...

    class Joint(Joint, ABC):
        ...

    def __init__(self, path: Optional[str] = None):
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
        Return the object parsed from the description file.
        :param parsed_description: The parsed description object.
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

    def generate_description_from_file(self, path: str, extension: str) -> str:
        """
        Generates and preprocesses the description from the file at the given path and returns the preprocessed
        description as a string.
        :param path: The path of the file to preprocess.
        :param extension: The file extension of the file to preprocess.
        :return: The processed description string.
        """

        if extension in self.MESH_EXTENSIONS:
            description_string = self.generate_from_mesh_file(path)
        elif extension == self.get_file_extension():
            description_string = self.generate_from_description_file(path)
        else:
            # Using the description from the parameter server
            description_string = self.generate_from_parameter_server(path)

        return description_string

    def get_file_name(self, path_object: pathlib.Path, extension: str, object_name: str) -> str:
        """
        Returns the file name of the description file.
        :param path_object: The path object of the description file or the mesh file.
        :param extension: The file extension of the description file or the mesh file.
        :param object_name: The name of the object.
        :return: The file name of the description file.
        """
        if extension in self.MESH_EXTENSIONS:
            file_name = path_object.stem + self.get_file_extension()
        elif extension == self.get_file_extension():
            file_name = path_object.name
        else:
            file_name = object_name + self.get_file_extension()

        return file_name

    @classmethod
    @abstractmethod
    def generate_from_mesh_file(cls, path: str) -> str:
        """
        Generates a description file from one of the mesh types defined in the MESH_EXTENSIONS and
        returns the path of the generated file.
        :param path: The path to the .obj file.
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

    @property
    @abstractmethod
    def joints(self) -> List[JointDescription]:
        """
        :return: A list of joints descriptions of this object.
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


class Attachment(AbstractConstraint):
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
        super().__init__(parent_link, child_link, JointType.FIXED, parent_to_child_transform,
                         Transform(frame=child_link.tf_frame))
        self.id = constraint_id
        self.bidirectional: bool = bidirectional
        self._loose: bool = False

        if self.parent_to_child_transform is None:
            self.update_transform()

        if self.id is None:
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
        self.id = self.parent_link.add_fixed_constraint_with_link(self.child_link)

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
                                constraint_id=self.id)
        attachment.loose = not self._loose
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


class CacheManager:
    cache_dir: str = World.cache_dir
    """
    The directory where the cached files are stored.
    """

    mesh_extensions: List[str] = [".obj", ".stl"]
    """
    The file extensions of mesh files.
    """

    def update_cache_dir_with_object(self, path: str, ignore_cached_files: bool,
                                     object_description: ObjectDescription, object_name: str) -> str:
        """
        Checks if the file is already in the cache directory, if not it will be preprocessed and saved in the cache.
        """
        path_object = pathlib.Path(path)
        extension = path_object.suffix

        path = self.look_for_file_in_data_dir(path, path_object)

        self.create_cache_dir_if_not_exists()

        # save correct path in case the file is already in the cache directory
        cache_path = self.cache_dir + object_description.get_file_name(path_object, extension, object_name)

        # if file is not yet cached preprocess the description file and save it in the cache directory.
        if not self.is_cached(path, object_description) or ignore_cached_files:
            self.generate_description_and_write_to_cache(path, extension, cache_path, object_description)

        return cache_path

    def generate_description_and_write_to_cache(self, path: str, extension: str, cache_path: str,
                                                object_description: ObjectDescription) -> None:
        """
        Generates the description from the file at the given path and writes it to the cache directory.
        :param path: The path of the file to preprocess.
        :param extension: The file extension of the file to preprocess.
        :param cache_path: The path of the file in the cache directory.
        :param object_description: The object description of the file.
        """
        description_string = object_description.generate_description_from_file(path, extension)
        self.write_to_cache(description_string, cache_path)

    @staticmethod
    def write_to_cache(description_string: str, cache_path: str) -> None:
        """
        Writes the description string to the cache directory.
        :param description_string: The description string to write to the cache directory.
        :param cache_path: The path of the file in the cache directory.
        """
        with open(cache_path, "w") as file:
            file.write(description_string)

    @staticmethod
    def look_for_file_in_data_dir(path: str, path_object: pathlib.Path) -> str:
        """
        Looks for a file in the data directory of the World. If the file is not found in the data directory, this method
        raises a FileNotFoundError.
        :param path: The path of the file to look for.
        :param path_object: The pathlib object of the file to look for.
        """
        if re.match("[a-zA-Z_0-9].[a-zA-Z0-9]", path):
            for data_dir in World.data_directory:
                for file in os.listdir(data_dir):
                    if file == path:
                        return data_dir + f"/{path}"
                if path:
                    break

        if not path:
            raise FileNotFoundError(
                f"File {path_object.name} could not be found in the resource directory {World.data_directory}")

        return path

    def create_cache_dir_if_not_exists(self):
        """
        Creates the cache directory if it does not exist.
        """
        if not pathlib.Path(self.cache_dir).exists():
            os.mkdir(self.cache_dir)

    def is_cached(self, path: str, object_description: ObjectDescription) -> bool:
        """
        Checks if the file in the given path is already cached or if
        there is already a cached file with the given name, this is the case if a .stl, .obj file or a description from
        the parameter server is used.

        :param path: The path of the file to check.
        :param object_description: The object description of the file.
        :return: True if there already exists a cached file, False in any other case.
        """
        return True if self.check_with_extension(path) else self.check_without_extension(path, object_description)

    def check_with_extension(self, path: str) -> bool:
        """
        Checks if the file in the given ath exists in the cache directory including file extension.
        :param path: The path of the file to check.
        """
        file_name = pathlib.Path(path).name
        full_path = pathlib.Path(self.cache_dir + file_name)
        return full_path.exists()

    def check_without_extension(self, path: str, object_description: ObjectDescription) -> bool:
        """
        Checks if the file in the given path exists in the cache directory without file extension,
        the extension is added after the file name manually in this case.
        :param path: The path of the file to check.
        :param object_description: The object description of the file.
        """
        file_stem = pathlib.Path(path).stem
        full_path = pathlib.Path(self.cache_dir + file_stem + object_description.get_file_extension())
        return full_path.exists()
