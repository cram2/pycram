# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import os
import threading
import time
import signal
from abc import ABC, abstractmethod
from copy import copy

import numpy as np
from trimesh import Trimesh
from typing_extensions import List, Optional, Dict, Tuple, Callable, TYPE_CHECKING, Union, Type, deprecated

import pycrap
from pycrap.ontologies import PhysicalObject, Robot, Floor, Apartment
from pycrap.ontologies.crax.rules import HierarchicalContainment, CRAXRule
from pycrap.ontology_wrapper import OntologyWrapper
from ..cache_manager import CacheManager
from ..config.world_conf import WorldConfig
from ..datastructures.dataclasses import (Color, AxisAlignedBoundingBox, CollisionCallbacks,
                                          MultiBody, VisualShape, BoxVisualShape, CylinderVisualShape,
                                          SphereVisualShape,
                                          CapsuleVisualShape, PlaneVisualShape, MeshVisualShape,
                                          ObjectState, WorldState, ClosestPointsList,
                                          ContactPointsList, VirtualMobileBaseJoints, RotatedBoundingBox, RayResult)
from ..datastructures.enums import JointType, WorldMode, Arms, AdjacentBodyMethod as ABM
from ..datastructures.pose import PoseStamped, TransformStamped, Point
from ..datastructures.world_entity import PhysicalBody, WorldEntity
from ..failures import ProspectionObjectNotFound, ObjectNotFound
from ..local_transformer import LocalTransformer
from ..robot_description import RobotDescription
from ..ros import Time
from ..ros import logwarn
from ..validation.goal_validator import (GoalValidator,
                                         validate_joint_position, validate_multiple_joint_positions,
                                         validate_object_pose, validate_multiple_object_poses)
from ..world_concepts.constraints import Constraint
from ..world_concepts.event import Event

if TYPE_CHECKING:
    from ..world_concepts.world_object import Object
    from ..description import Link, Joint, ObjectDescription
    from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription


class World(WorldEntity, ABC):
    """
    The World Class represents the physics Simulation and belief state, it is the main interface for reasoning about
    the World. This is implemented as a singleton, the current World can be accessed via the static variable
    current_world which is managed by the World class itself.
    """

    conf: Type[WorldConfig] = WorldConfig
    """
    The configurations of the world, the default configurations are defined in world_conf.py in the config folder.
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

    environment: Optional[Object] = None
    """
    Global reference to the spawned Object that represents the environment. The environment is identified by checking the name in
     the URDF with the name of the URDF on the parameter server. 
    """

    cache_manager: CacheManager = CacheManager(conf.cache_dir, [conf.resources_path], False)
    """
    Global reference for the cache manager, this is used to cache the description files of the robot and the objects.
    """

    ontology: Optional[OntologyWrapper] = None
    """
    The ontology of this world.
    """

    def __init__(self, mode: WorldMode = WorldMode.DIRECT, is_prospection: bool = False, clear_cache: bool = False,
                 id_: int = -1):
        """
        Create a new simulation, the mode decides if the simulation should be a rendered window or just run in the
        background. There can only be one rendered simulation.
        The World object also initializes the Events for attachment, detachment and for manipulating the world.

        :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default parameter is
         "GUI"
        :param is_prospection: For internal usage, decides if this World should be used as a prospection world.
        :param clear_cache: Whether to clear the cache directory.
        :param id_: The unique id of the world.
        """
        self.is_prospection_world: bool = is_prospection
        if not is_prospection:
            self.ontology = OntologyWrapper()
        else:
            self.ontology = None
        WorldEntity.__init__(self, id_, self, concept=pycrap.ontologies.World)

        self.latest_state_id: Optional[int] = None

        if clear_cache or (self.conf.clear_cache_at_start and not self.cache_manager.cache_cleared):
            self.cache_manager.clear_cache()

        GoalValidator.raise_error = self.conf.raise_goal_validator_error

        if World.current_world is None:
            World.current_world = self

        self.object_lock: threading.Lock = threading.Lock()

        self._init_world(mode)

        self.objects: List[Object] = []
        # List of all Objects in the World

        self._init_and_sync_prospection_world()

        self.local_transformer = LocalTransformer()
        self._update_local_transformer_worlds()

        self.mode: WorldMode = mode

        self.coll_callbacks: Dict[Tuple[Object, Object], CollisionCallbacks] = {}

        self._init_events()

        self._current_state: Optional[WorldState] = None

        self.original_state_id = self.save_state()

        self.on_add_object_callbacks: List[Callable[[Object], None]] = []

        self._set_world_rules()

        signal.signal(signal.SIGINT, self.signal_handler)

    def _set_world_rules(self):
        """
        Create the rules for the world.
        """
        if self.is_prospection_world:
            return
        HierarchicalContainment()

    @property
    def rules(self) -> List[CRAXRule]:
        """
        Return the rules of the world.
        """
        return list(CRAXRule.all_rules[self.ontology].values())

    @staticmethod
    def update_containment_for(bodies: List[PhysicalBody],
                               candidate_selection_method: ABM = ABM.ClosestPoints) \
            -> List[PhysicalBody]:
        """
        Update the containment for the given bodies by checking if they are contained in other bodies.

        :param bodies: The bodies to update the containment for.
        :param candidate_selection_method: The method to select the candidate bodies for containment update.
        :return: The updated bodies.
        """
        checked_bodies: List[PhysicalBody] = []
        for body in bodies:
            body.update_containment(excluded_bodies=checked_bodies,
                                    candidate_selection_method=candidate_selection_method)
            checked_bodies.append(body)
        return checked_bodies

    @property
    def parent_entity(self) -> Optional[WorldEntity]:
        """
        Return the parent entity of this entity, in this case it is None as the World is the top level entity.
        """
        return None

    @property
    def name(self) -> str:
        """
        Return the name of the world, which is the name of the implementation class (e.g. BulletWorld).
        """
        return self.__class__.__name__

    def get_body_convex_hull(self, body: PhysicalBody) -> Trimesh:
        """
        :param body: The body object.
        :return: The convex hull of the body as a Geometry3D object.
        """
        raise NotImplementedError

    def add_callback_on_add_object(self, callback: Callable[[Object], None]) -> None:
        """
        Add a callback that is called when an object is added to the world.

        :param callback: The callback.
        """
        self.on_add_object_callbacks.append(callback)

    def remove_callback_on_add_object(self, callback: Callable[[Object], None]) -> None:
        """
        Remove a callback that is called when an object is added to the world.

        :param callback: The callback.
        """
        self.on_add_object_callbacks.remove(callback)

    @classmethod
    def get_cache_dir(cls) -> str:
        """
        Return the cache directory.
        """
        return cls.cache_manager.cache_dir

    def add_object(self, obj: Object) -> None:
        """
        Add an object to the world.

        :param obj: The object to be added.
        """
        self.object_lock.acquire()
        self.objects.append(obj)
        self.add_object_to_original_state(obj)
        self.object_lock.release()
        self.invoke_on_add_object_callbacks(obj)

    def invoke_on_add_object_callbacks(self, obj: Object) -> None:
        """
        Invoke the object added callbacks.

        :param obj: The object.
        """
        for callback in self.on_add_object_callbacks:
            callback(obj)

    @property
    def robot_description(self) -> RobotDescription:
        """
        Return the current robot description.
        """
        return RobotDescription.current_robot_description

    @property
    def robot_has_actuators(self) -> bool:
        """
        Return whether the robot has actuators.
        """
        return self.robot_description.has_actuators

    def get_actuator_for_joint(self, joint: Joint) -> str:
        """
        Get the actuator name for a given joint.
        """
        return self.robot_joint_actuators[joint.name]

    def joint_has_actuator(self, joint: Joint) -> bool:
        """
        Return whether the joint has an actuator.
        """
        return joint.name in self.robot_joint_actuators

    @property
    def robot_joint_actuators(self) -> Dict[str, str]:
        """
        Return the joint actuators of the robot.
        """
        return self.robot_description.joint_actuators

    def check_object_exists(self, obj: Object) -> bool:
        """
        Check if the object exists in the simulator.

        :param obj: The object to check.
        :return: True if the object is in the world, False otherwise.
        """
        raise NotImplementedError

    @abstractmethod
    def _init_world(self, mode: WorldMode):
        """
        Initialize the physics simulation.
        """
        raise NotImplementedError

    def _init_events(self):
        """
        Initialize dynamic events that can be used to react to changes in the World.
        """
        self.detachment_event: Event = Event()
        self.attachment_event: Event = Event()
        self.manipulation_event: Event = Event()

    def _init_and_sync_prospection_world(self):
        """
        Initialize the prospection world and the synchronization between the main and the prospection world.
        """
        self._init_prospection_world()
        self._sync_prospection_world()

    def _update_local_transformer_worlds(self):
        """
        Update the local transformer worlds with the current world and prospection world.
        """
        self.local_transformer.world = self
        self.local_transformer.prospection_world = self.prospection_world

    def _init_prospection_world(self):
        """
        Initialize the prospection world, if this is a prospection world itself it will not create another prospection,
        world, but instead set the prospection world to None, else it will create a prospection world.
        """
        if self.is_prospection_world:  # then no need to add another prospection world
            self.prospection_world = None
        else:
            self.prospection_world: World = self.__class__(is_prospection=True)

    def _sync_prospection_world(self):
        """
        Synchronize the prospection world with the main world, this means that every object in the main world will be
        added to the prospection world and vice versa.
        """
        if self.is_prospection_world:  # then no need to add another prospection world
            self.world_sync = None
        else:
            self.world_sync: WorldSync = WorldSync(self, self.prospection_world)
            self.world_sync.start()

    def preprocess_object_file_and_get_its_cache_path(self, path: str, ignore_cached_files: bool,
                                                      description: ObjectDescription, name: str,
                                                      scale_mesh: Optional[float] = None,
                                                      mesh_transform: Optional[TransformStamped] = None,
                                                      color: Optional[Color] = None) -> str:
        """
        Update the cache directory with the given object.

        :param path: The path to the object.
        :param ignore_cached_files: If the cached files should be ignored.
        :param description: The object description.
        :param name: The name of the object.
        :param scale_mesh: The scale of the mesh.
        :param mesh_transform: The mesh transform to apply to the mesh.
        :param color: The color of the object.
        :return: The path of the cached object.
        """
        return self.cache_manager.update_cache_dir_with_object(path, ignore_cached_files, description, name,
                                                               scale_mesh, mesh_transform, color)

    @property
    def simulation_time_step(self):
        """
        The time step of the simulation in seconds.
        """
        return 1 / self.__class__.conf.simulation_frequency

    @abstractmethod
    def load_object_and_get_id(self, path: Optional[str] = None, pose: Optional[PoseStamped] = None,
                               obj_type: Optional[Type[PhysicalObject]] = None) -> int:
        """
        Load a description file (e.g. URDF) at the given pose and returns the id of the loaded object.

        :param path: The path to the description file, if None the description file is assumed to be already loaded.
        :param pose: The pose at which the object should be loaded.
        :param obj_type: The type of the object.
        :return: The id of the loaded object.
        """
        pass

    def load_generic_object_and_get_id(self, description: GenericObjectDescription,
                                       pose: Optional[PoseStamped] = None) -> int:
        """
        Create a visual and collision box in the simulation and returns the id of the loaded object.

        :param description: The object description.
        :param pose: The pose at which the object should be loaded.
        """
        raise NotImplementedError

    def get_object_names(self) -> List[str]:
        """
        Return the names of all objects in the World.

        :return: A list of object names.
        """
        return [obj.name for obj in self.objects]

    def get_object_by_name(self, name: str) -> Optional[Object]:
        """
        Return the object with the given name. If there is no object with the given name, None is returned.

        :param name: The name of the returned Objects.
        :return: The object with the given name, if there is one.
        """

        matching_objects = list(filter(lambda obj: obj.name == name, self.objects))
        return matching_objects[0] if len(matching_objects) > 0 else None

    def get_object_by_type(self, obj_type: PhysicalObject) -> List[Object]:
        """
        Return a list of all Objects which have the type 'obj_type'.

        :param obj_type: The type of the returned Objects.
        :return: A list of all Objects that have the type 'obj_type'.
        """
        return list(filter(lambda obj: obj.obj_type == obj_type, self.objects))

    def get_object_by_id(self, obj_id: int) -> Object:
        """
        Return the single Object that has the unique id.

        :param obj_id: The unique id for which the Object should be returned.
        :return: The Object with the id 'id'.
        """
        return list(filter(lambda obj: obj.id == obj_id, self.objects))[0]

    def get_scene_objects(self) -> List[Object]:
        """
        :return: A list of all objects in the world except the robot, floor, and apartment.
        """
        return [obj for obj in self.objects if obj.obj_type not in {Robot, Floor, Apartment}]

    def remove_visual_object(self, obj_id: int) -> bool:
        """
        Remove the object with the given id from the world, and saves a new original state for the world.

        :param obj_id: The unique id of the object to be removed.
        :return: Whether the object was removed successfully.
        """

        removed = self._remove_visual_object(obj_id)
        if removed:
            self.update_simulator_state_id_in_original_state()
        else:
            logwarn(f"Object with id {obj_id} could not be removed.")
        return removed

    @abstractmethod
    def _remove_visual_object(self, obj_id: int) -> bool:
        """
        Remove the visual object with the given id from the world, and update the simulator state in the original state.

        :param obj_id: The unique id of the visual object to be removed.
        :return: Whether the object was removed successfully.
        """
        pass

    @abstractmethod
    def remove_object_from_simulator(self, obj: Object) -> bool:
        """
        Remove an object from the physics simulator.

        :param obj: The object to be removed.
        :return: Whether the object was removed successfully.
        """
        pass

    def remove_object(self, obj: Object) -> None:
        """
        Remove this object from the current world.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to world remove object is done
        to remove this Object from the simulation/world.

        :param obj: The object to be removed.
        """
        self.object_lock.acquire()

        obj.detach_all()

        if self.remove_object_from_simulator(obj):
            self.objects.remove(obj)
            self.remove_object_from_original_state(obj)

            if World.robot == obj and not self.is_prospection_world:
                World.robot = None
        else:
            logwarn(f"Object {obj.name} could not be removed from the simulator, but all attachments were removed")

        self.object_lock.release()

    def remove_object_from_original_state(self, obj: Object) -> None:
        """
        Remove an object from the original state of the world.

        :param obj: The object to be removed.
        """
        self.original_state.object_states.pop(obj.name)
        self.update_simulator_state_id_in_original_state(use_same_id=True)

    def add_object_to_original_state(self, obj: Object) -> None:
        """
        Add an object to the original state of the world.

        :param obj: The object to be added.
        """
        self.original_state.object_states[obj.name] = obj.current_state
        self.update_simulator_state_id_in_original_state()

    def add_fixed_constraint(self, parent_link: Link, child_link: Link,
                             child_to_parent_transform: TransformStamped) -> int:
        """
        Create a fixed joint constraint between the given parent and child links,
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
                                axis_in_child_frame=Point(x=0, y=0, z=0),
                                constraint_to_parent=child_to_parent_transform,
                                child_to_constraint=TransformStamped.from_list(frame=child_link.tf_frame)
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

    def get_joint_position(self, joint: Joint) -> float:
        """
        Wrapper for :meth:`_get_joint_position` that return 0.0 for a joint if it is in the ignore joints list.
        """
        if joint.object.is_a_robot and joint.name in self.robot_description.ignore_joints:
            return 0.0
        return self._get_joint_position(joint)

    @abstractmethod
    def _get_joint_position(self, joint: Joint) -> float:
        """
        Get the position of a joint of an articulated object

        :param joint: The joint to get the position for.
        :return: The joint position as a float.
        """
        pass

    @abstractmethod
    def get_object_joint_names(self, obj: Object) -> List[str]:
        """
        Return the names of all joints of this object.

        :param obj: The object.
        :return: A list of joint names.
        """
        pass

    @abstractmethod
    def get_link_pose(self, link: Link) -> PoseStamped:
        """
        Get the pose of a link of an articulated object with respect to the world frame.

        :param link: The link as a AbstractLink object.
        :return: The pose of the link as a Pose object.
        """
        pass

    @abstractmethod
    def get_multiple_link_poses(self, links: List[Link]) -> Dict[str, PoseStamped]:
        """
        Get the poses of multiple links of an articulated object with respect to the world frame.

        :param links: The links as a list of AbstractLink objects.
        :return: A dictionary with link names as keys and Pose objects as values.
        """
        pass

    @abstractmethod
    def get_link_position(self, link: Link) -> List[float]:
        """
        Get the position of a link of an articulated object with respect to the world frame.

        :param link: The link as a AbstractLink object.
        :return: The position of the link as a list of floats.
        """
        pass

    @abstractmethod
    def get_link_orientation(self, link: Link) -> List[float]:
        """
        Get the orientation of a link of an articulated object with respect to the world frame.

        :param link: The link as a AbstractLink object.
        :return: The orientation of the link as a list of floats.
        """
        pass

    @abstractmethod
    def get_multiple_link_positions(self, links: List[Link]) -> Dict[str, List[float]]:
        """
        Get the positions of multiple links of an articulated object with respect to the world frame.

        :param links: The links as a list of AbstractLink objects.
        :return: A dictionary with link names as keys and lists of floats as values.
        """
        pass

    @abstractmethod
    def get_multiple_link_orientations(self, links: List[Link]) -> Dict[str, List[float]]:
        """
        Get the orientations of multiple links of an articulated object with respect to the world frame.

        :param links: The links as a list of AbstractLink objects.
        :return: A dictionary with link names as keys and lists of floats as values.
        """
        pass

    @abstractmethod
    def get_object_link_names(self, obj: Object) -> List[str]:
        """
        Return the names of all links of this object.

        :param obj: The object.
        :return: A list of link names.
        """
        pass

    def simulate(self, seconds: float, real_time: Optional[bool] = False,
                 func: Optional[Callable[[], None]] = None) -> None:
        """
        Simulate Physics in the World for a given amount of seconds. Usually this simulation is faster than real
        time. By setting the 'real_time' parameter this simulation is slowed down such that the simulated time is equal
        to real time.

        :param seconds: The amount of seconds that should be simulated.
        :param real_time: If the simulation should happen in real time or faster.
        :param func: A function that should be called during the simulation
        """
        self.set_realtime(real_time)
        for i in range(0, int(seconds * self.conf.simulation_frequency)):
            curr_time = Time().now()
            self.step(func)
            for objects, callbacks in self.coll_callbacks.items():
                contact_points = self.get_contact_points_between_two_bodies(objects[0], objects[1])
                if len(contact_points) > 0:
                    callbacks.on_collision_cb()
                elif callbacks.no_collision_cb is not None:
                    callbacks.no_collision_cb()
            if real_time:
                loop_time = Time().now() - curr_time
                time_diff = self.simulation_time_step - loop_time.to_sec()
                time.sleep(max(0, time_diff))

    @abstractmethod
    def get_object_pose(self, obj: Object) -> PoseStamped:
        """
        Get the pose of an object in the world frame from the current object pose in the simulator.

        :param obj: The object.
        """
        pass

    @abstractmethod
    def get_multiple_object_poses(self, objects: List[Object]) -> Dict[str, PoseStamped]:
        """
        Get the poses of multiple objects in the world frame from the current object poses in the simulator.

        :param objects: The objects.
        """
        pass

    @abstractmethod
    def get_multiple_object_positions(self, objects: List[Object]) -> Dict[str, List[float]]:
        """
        Get the positions of multiple objects in the world frame from the current object poses in the simulator.

        :param objects: The objects.
        """
        pass

    @abstractmethod
    def get_object_position(self, obj: Object) -> List[float]:
        """
        Get the position of an object in the world frame from the current object pose in the simulator.

        :param obj: The object.
        """
        pass

    @abstractmethod
    def get_multiple_object_orientations(self, objects: List[Object]) -> Dict[str, List[float]]:
        """
        Get the orientations of multiple objects in the world frame from the current object poses in the simulator.

        :param objects: The objects.
        """
        pass

    @abstractmethod
    def get_object_orientation(self, obj: Object) -> List[float]:
        """
        Get the orientation of an object in the world frame from the current object pose in the simulator.

        :param obj: The object.
        """
        pass

    @property
    def robot_virtual_joints(self) -> List[Joint]:
        """
        The virtual joints of the robot.
        """
        return [self.robot.joints[name] for name in self.robot_virtual_joints_names]

    @property
    def robot_virtual_joints_names(self) -> List[str]:
        """
        The names of the virtual joints of the robot.
        """
        return self.robot_description.virtual_mobile_base_joints.names

    def get_robot_mobile_base_joints(self) -> VirtualMobileBaseJoints:
        """
        Get the mobile base joints of the robot.

        :return: The mobile base joints.
        """
        return self.robot_description.virtual_mobile_base_joints

    @abstractmethod
    def perform_collision_detection(self) -> None:
        """
        Check for collisions between all objects in the World and updates the contact points.
        """
        pass

    @deprecated("Use get_body_contact_points instead")
    def get_object_contact_points(self, obj: Object) -> ContactPointsList:
        """
        Same as :meth:`get_body_contact_points` but with objects instead of any type of bodies.
        """
        return self.get_body_contact_points(obj)

    @abstractmethod
    def get_body_contact_points(self, body: PhysicalBody) -> ContactPointsList:
        """
        Return the contact points of a body with all other bodies in the world.

        :param body: The body.
        """
        pass

    @deprecated("Use get_contact_points_between_two_bodies instead")
    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> ContactPointsList:
        """
        Same as :meth:`get_contact_points_between_two_bodies` but with objects instead of any type of bodies.
        """
        return self.get_contact_points_between_two_bodies(obj1, obj2)

    @abstractmethod
    def get_contact_points_between_two_bodies(self, body_1: PhysicalBody, body_2: PhysicalBody) -> ContactPointsList:
        """
        Return a list of contact points between two bodies.

        :param body_1: The first body.
        :param body_2: The second body.
        :return: A list of all contact points between the two bodies.
        """
        pass

    @deprecated("Use get_contact_points_between_two_bodies instead")
    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> ContactPointsList:
        """
        Same as :meth:`get_contact_points_between_two_bodies` but with objects instead of any type of bodies.
        """
        return self.get_contact_points_between_two_bodies(obj1, obj2)

    def get_body_closest_points(self, body: PhysicalBody, max_distance: float) -> ClosestPointsList:
        """
        Return the closest points of this body with all other bodies in the world.

        :param body: The body.
        :param max_distance: The maximum allowed distance between the points.
        :return: A list of the closest points.
        """
        all_obj_closest_points = [self.get_closest_points_between_two_bodies(body, other_body, max_distance)
                                  for other_body in self.objects if other_body != body]
        return ClosestPointsList([point for closest_points in all_obj_closest_points for point in closest_points])

    def get_closest_points_between_two_bodies(self, body_a: PhysicalBody, body_b: PhysicalBody, max_distance: float) \
            -> ClosestPointsList:
        """
        Return the closest points between two objects.

        :param body_a: The first body.
        :param body_b: The second body.
        :param max_distance: The maximum distance between the points.
        :return: A list of the closest points.
        """
        raise NotImplementedError

    def reset_joint_position(self, joint: Joint, joint_position: float) -> bool:
        """
        Wrapper around :meth:`_reset_joint_position` that checks if the joint should be ignored.
        """
        if joint.object.is_a_robot and self.robot_description.ignore_joints:
            if joint.name in self.robot_description.ignore_joints:
                return True
        return self._reset_joint_position(joint, joint_position)

    @validate_joint_position
    @abstractmethod
    def _reset_joint_position(self, joint: Joint, joint_position: float) -> bool:
        """
        Reset the joint position instantly without physics simulation

        .. note::
           It is recommended to use the validate_joint_position decorator to validate the joint position for
           the implementation of this method.

        :param joint: The joint to reset the position for.
        :param joint_position: The new joint pose.
        :return: True if the reset was successful, False otherwise
        """
        pass

    def set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> bool:
        """
        Wrapper around :meth:`_set_multiple_joint_positions` that checks if any of the joints should be ignored.
        """
        filtered_joint_positions = copy(joint_positions)
        for joint, position in joint_positions.items():
            if joint.name in self.robot_description.ignore_joints:
                filtered_joint_positions.pop(joint)
        return self._set_multiple_joint_positions(filtered_joint_positions)

    @validate_multiple_joint_positions
    @abstractmethod
    def _set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> bool:
        """
        Set the positions of multiple joints of an articulated object.

        .. note::
           It is recommended to use the validate_multiple_joint_positions decorator to validate the
           joint positions for the implementation of this method.

        :param joint_positions: A dictionary with joint objects as keys and joint positions as values.
        :return: True if the set was successful, False otherwise.
        """
        pass

    def get_multiple_joint_positions(self, joints: List[Joint]) -> Dict[str, float]:
        """
        Wrapper around :meth:`_get_multiple_joint_positions` that checks if any of the joints should be ignored.
        """
        filtered_joints = [joint for joint in joints if not joint.object.is_a_robot or
                           joint.name not in self.robot_description.ignore_joints]
        joint_positions = self._get_multiple_joint_positions(filtered_joints)
        joint_positions.update({joint.name: 0.0 for joint in joints if joint not in filtered_joints})
        return joint_positions

    @abstractmethod
    def _get_multiple_joint_positions(self, joints: List[Joint]) -> Dict[str, float]:
        """
        Get the positions of multiple joints of an articulated object.

        :param joints: The joints as a list of Joint objects.
        """
        pass

    @validate_object_pose
    @abstractmethod
    def reset_object_base_pose(self, obj: Object, pose: PoseStamped) -> bool:
        """
        Reset the world position and orientation of the base of the object instantaneously,
        not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

        .. note::
           It is recommended to use the validate_object_pose decorator to validate the object pose for the
           implementation of this method.

        :param obj: The object.
        :param pose: The new pose as a Pose object.
        :return: True if the reset was successful, False otherwise.
        """
        pass

    @validate_multiple_object_poses
    @abstractmethod
    def reset_multiple_objects_base_poses(self, objects: Dict[Object, PoseStamped]) -> bool:
        """
        Reset the world position and orientation of the base of multiple objects instantaneously,
        not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

        :param objects: A dictionary with objects as keys and poses as values.
        :return: True if the reset was successful, False otherwise.
        """
        pass

    @abstractmethod
    def step(self, func: Optional[Callable[[], None]] = None, step_seconds: Optional[float] = None) -> None:
        """
        Step the world simulation using forward dynamics.

        :param func: An optional function to be called during the step.
        :param step_seconds: The amount of seconds to step the simulation if None the simulation is stepped by the
        simulation time step.
        """
        pass

    def get_arm_tool_frame_link(self, arm: Arms) -> Link:
        """
        Get the tool frame link of the arm of the robot.

        :param arm: The arm for which the tool frame link should be returned.
        :return: The tool frame link of the arm.
        """
        ee_link_name = self.robot_description.get_arm_tool_frame(arm)
        return self.robot.get_link(ee_link_name)

    @abstractmethod
    def set_link_color(self, link: Link, rgba_color: Color):
        """
        Change the rgba_color of a link of this object, the rgba_color has to be given as Color object.

        :param link: The link which should be colored.
        :param rgba_color: The rgba_color as Color object with RGBA values between 0 and 1.
        """
        pass

    @abstractmethod
    def get_link_color(self, link: Link) -> Color:
        """
        :param link: The link for which the rgba_color should be returned.
        :return: The rgba_color as Color object with RGBA values between 0 and 1.
        """
        pass

    @abstractmethod
    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        """
        :param obj: The object
        :return: The RGBA colors of each link in the object as a dictionary from link name to rgba_color.
        """
        pass

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        """
        :param obj: The object for which the bounding box should be returned.
        :return: the axis aligned bounding box of this object. The return of this method are two points in
        world coordinate frame which define a bounding box.
        """
        raise NotImplementedError()

    def get_object_rotated_bounding_box(self, obj: Object) -> RotatedBoundingBox:
        """
        :param obj: The object for which the bounding box should be returned.
        :return: the rotated bounding box of this object. The return of this method are two points in
        world coordinate frame which define a bounding box.
        """
        raise NotImplementedError()

    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        """
        :param link: The link for which the bounding box should be returned.
        :return: The axis aligned bounding box of the link. The return of this method are two points in
        world coordinate frame which define a bounding box.
        """
        raise NotImplementedError()

    def get_link_rotated_bounding_box(self, link: Link) -> RotatedBoundingBox:
        """
        :param link: The link for which the bounding box should be returned.
        :return: The rotated bounding box of the link. The return of this method are two points in
        world coordinate frame which define a bounding box.
        """
        raise NotImplementedError()

    @abstractmethod
    def set_realtime(self, real_time: bool) -> None:
        """
        Enable the real time simulation of Physics in the World. By default, this is disabled and Physics is only
        simulated to reason about it.

        :param real_time: Whether the World should simulate Physics in real time.
        """
        pass

    @abstractmethod
    def set_gravity(self, gravity_vector: List[float]) -> None:
        """
        Set the gravity that is used in the World. By default, it is set to the gravity on earth ([0, 0, -9.8]).
         Gravity is given as a vector in x,y,z. Gravity is only applied while simulating Physic.

        :param gravity_vector: The gravity vector that should be used in the World.
        """
        pass

    def set_robot_if_not_set(self, robot: Object) -> None:
        """
        Set the robot if it is not set yet.

        :param robot: The Object reference to the Object representing the robot.
        """
        if not self.robot_is_set():
            self.set_robot(robot)

    @staticmethod
    def set_robot(robot: Union[Object, None]) -> None:
        """
        Set the global variable for the robot Object This should be set on spawning the robot.

        :param robot: The Object reference to the Object representing the robot.
        """
        World.robot = robot

    @staticmethod
    def robot_is_set() -> bool:
        """
        Return whether the robot has been set or not.

        :return: True if the robot has been set, False otherwise.
        """
        return World.robot is not None

    def exit(self, remove_saved_states: bool = True) -> None:
        """
        Close the World as well as the prospection world, also collects any other thread that is running.

        :param remove_saved_states: Whether to remove the saved states.
        """
        self.reset_world(remove_saved_states)
        self.remove_all_objects()
        self.exit_prospection_world_if_exists()
        self.disconnect_from_physics_server()
        self.reset_robot()
        self.join_threads()
        if self.ontology:
            self.ontology.destroy_individuals()
        if World.current_world == self:
            World.current_world = None

    def exit_prospection_world_if_exists(self) -> None:
        """
        Exit the prospection world if it exists.
        """
        if self.prospection_world:
            self.terminate_world_sync()
            self.prospection_world.exit()

    def signal_handler(self, sig, frame) -> None:
        """
        Signal handler for graceful exit of the world when a signal is received (e.g., SIGINT).
        """
        self.exit()
        print("Exiting World ...")
        exit(0)

    @abstractmethod
    def disconnect_from_physics_server(self) -> None:
        """
        Disconnect the world from the physics server.
        """
        pass

    def reset_current_world(self) -> None:
        """
        Reset the pose of every object in the World to the pose it was spawned in and sets every joint to 0.
        """
        for obj in self.objects:
            obj.set_pose(obj.original_pose)
            obj.set_multiple_joint_positions(dict(zip(list(obj.joint_names), [0] * len(obj.joint_names))))

    def reset_robot(self) -> None:
        """
        Set the robot class variable to None.
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
        Terminate the world sync thread.
        """
        self.world_sync.terminate = True
        self.world_sync.join()

    def save_state(self, state_id: Optional[int] = None, use_same_id: bool = False, to_file: bool = False) -> int:
        """
        Return the id of the saved state of the World. The saved state contains the states of all the objects and
        the state of the physics simulator.

        :param state_id: The id of the saved state.
        :param use_same_id: Whether to use the same current state id for the new saved state.
        :param to_file: Whether to save the state to a file.
        :return: A unique id of the state
        """

        sim_state_id = self.save_physics_simulator_state(state_id=state_id, use_same_id=use_same_id)

        if state_id is None:
            if self.latest_state_id is None:
                self.latest_state_id = 0
            else:
                self.latest_state_id += 0 if use_same_id else 1
            state_id = self.latest_state_id

        self.save_objects_state(state_id)

        self._current_state = WorldState(self.object_states, sim_state_id)

        return super().save_state(state_id, self.get_cache_dir() if to_file else None)

    @property
    def current_state(self) -> WorldState:
        if self._current_state is None:
            simulator_state_id = None if not self.conf.use_physics_simulator_state else (
                self.save_physics_simulator_state(use_same_id=True))
            self._current_state = WorldState(self.object_states, simulator_state_id)
        return WorldState(self.object_states, self._current_state.simulator_state_id)

    @current_state.setter
    def current_state(self, state: WorldState) -> None:
        if self.current_state != state:
            if self.conf.use_physics_simulator_state:
                self.restore_physics_simulator_state(state.simulator_state_id)
                self.set_object_states_without_poses(state.object_states)
            else:
                for obj in self.objects:
                    self.get_object_by_name(obj.name).current_state = state.object_states[obj.name]

    def set_object_states_without_poses(self, states: Dict[str, ObjectState]) -> None:
        """
        Set the states of all objects in the World except the poses.

        :param states: A dictionary with the object id as key and the object state as value.
        """
        for obj_name, obj_state in states.items():
            obj = self.get_object_by_name(obj_name)
            obj.set_attachments(obj_state.attachments)
            obj.link_states = obj_state.link_states
            obj.joint_states = obj_state.joint_states
            if obj.name == self.robot.name and len(self.robot_virtual_joints) > 0:
                obj.set_mobile_robot_pose(obj_state.pose)

    @property
    def object_states(self) -> Dict[str, ObjectState]:
        """
        Return the states of all objects in the World.

        :return: A dictionary with the object id as key and the object state as value.
        """
        return {obj.name: obj.current_state for obj in self.objects}

    @object_states.setter
    def object_states(self, states: Dict[str, ObjectState]) -> None:
        """
        Set the states of all objects in the World.
        """
        for obj_name, obj_state in states.items():
            self.get_object_by_name(obj_name).current_state = obj_state

    def save_objects_state(self, state_id: int) -> None:
        """
        Save the state of all objects in the World according to the given state using the unique state id.

        :param state_id: The unique id representing the state.
        """
        for obj in self.objects:
            obj.save_state(state_id)

    @abstractmethod
    def save_physics_simulator_state(self, state_id: Optional[int] = None, use_same_id: bool = False) -> int:
        """
        Save the state of the physics simulator and returns the unique id of the state.

        :param state_id: The used specified unique id representing the state.
        :param use_same_id: If the same id should be used for the state.
        :return: The unique id representing the state.
        """
        pass

    @abstractmethod
    def remove_physics_simulator_state(self, state_id: int) -> None:
        """
        Remove the state of the physics simulator with the given id.

        :param state_id: The unique id representing the state.
        """
        pass

    @abstractmethod
    def restore_physics_simulator_state(self, state_id: int) -> None:
        """
        Restore the objects and environment state in the physics simulator according to
         the given state using the unique state id.

        :param state_id: The unique id representing the state.
        """
        pass

    def get_images_for_target(self,
                              target_pose: PoseStamped,
                              cam_pose: PoseStamped,
                              size: Optional[int] = 256) -> List[np.ndarray]:
        """
        Calculate the view and projection Matrix and returns 3 images:

        1. An RGB image
        2. A depth image
        3. A segmentation Mask, the segmentation mask indicates for every pixel the visible Object

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
        Register callback methods for contact between two Objects. There can be a callback for when the two Objects
        get in contact and, optionally, for when they are not in contact anymore.

        :param object_a: An object in the World
        :param object_b: Another object in the World
        :param on_collision_callback: A function that should be called if the objects are in contact
        :param on_collision_removal_callback: A function that should be called if the objects are not in contact
        """
        self.coll_callbacks[(object_a, object_b)] = CollisionCallbacks(on_collision_callback,
                                                                       on_collision_removal_callback)

    @classmethod
    def get_data_directories(cls) -> List[str]:
        """
        The resources directories where the objects, robots, and environments are stored.
        """
        return cls.cache_manager.data_directories

    @classmethod
    def add_resource_path(cls, path: str, prepend: bool = False) -> None:
        """
        Add a resource path in which the World will search for files. This resource directory is searched if an
        Object is spawned only with a filename.

        :param path: A path in the filesystem in which to search for files.
        :param prepend: Put the new path at the beginning of the list such that it is searched first.
        """
        if prepend:
            cls.cache_manager.data_directories = [path] + cls.cache_manager.data_directories
        else:
            cls.cache_manager.data_directories.append(path)

    @classmethod
    def remove_resource_path(cls, path: str) -> None:
        """
        Remove the given path from the data_directories list.

        :param path: The path to remove.
        """
        cls.cache_manager.data_directories.remove(path)

    @classmethod
    def change_cache_dir_path(cls, path: str) -> None:
        """
        Change the cache directory to the given path

        :param path: The new path for the cache directory.
        """
        cls.cache_manager.cache_dir = os.path.join(path, cls.conf.cache_dir_name)

    def get_prospection_object_for_object(self, obj: Object) -> Object:
        """
        Return the corresponding object from the prospection world for a given object in the main world.
         If the given Object is already in the prospection world, it is returned.

        :param obj: The object for which the corresponding object in the prospection World should be found.
        :return: The corresponding object in the prospection world.
        """
        with UseProspectionWorld():
            return self.world_sync.get_prospection_object(obj)

    def get_object_for_prospection_object(self, prospection_object: Object) -> Object:
        """
        Return the corresponding object from the main World for a given
        object in the prospection world. If the  given object is not in the prospection
        world an error will be raised.

        :param prospection_object: The object for which the corresponding object in the main World should be found.
        :return: The object in the main World.
        """
        with UseProspectionWorld():
            return self.world_sync.get_world_object(prospection_object)

    def remove_all_objects(self, exclude_objects: Optional[List[Object]] = None) -> None:
        """
        Remove all objects from the World.

        :param exclude_objects: A list of objects that should not be removed.
        """
        objs_copy = [obj for obj in self.objects]
        exclude_objects = [] if exclude_objects is None else exclude_objects
        [self.remove_object(obj) for obj in objs_copy if obj not in exclude_objects]

    def reset_world(self, remove_saved_states=False) -> None:
        """
        Reset the World to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they were spawned.

        :param remove_saved_states: If the saved states should be removed.
        """
        self.restore_state(self.original_state_id)
        if remove_saved_states:
            self.remove_saved_states()
            self.original_state_id = self.save_state(use_same_id=True)

    def reset_concepts(self):
        """
        Reset the concepts of the World.
        """
        super().reset_concepts()
        [obj.reset_concepts() for obj in self.objects]

    def remove_saved_states(self) -> None:
        """
        Remove all saved states of the World.
        """
        if self.conf.use_physics_simulator_state:
            simulator_state_ids = set([state.simulator_state_id for state in self.saved_states.values()])
            for ssid in simulator_state_ids:
                self.remove_physics_simulator_state(ssid)
        else:
            self.remove_objects_saved_states()
        super().remove_saved_states()
        self.latest_state_id = None
        self.original_state_id = None

    def remove_objects_saved_states(self) -> None:
        """
        Remove all saved states of the objects in the World.
        """
        for obj in self.objects:
            obj.remove_saved_states()

    def update_transforms_for_objects_in_current_world(self) -> None:
        """
        Updates transformations for all objects that are currently in :py:attr:`~pycram.world.World.current_world`.
        """
        curr_time = Time().now()
        for obj in list(self.current_world.objects):
            obj.update_link_transforms(curr_time)

    def ray_test(self, from_position: List[float], to_position: List[float], calculate_distance: bool = False) \
            -> RayResult:
        """
        A wrapper around the :py:meth:`~pycram.world.World._ray_test` method that also calculates the distance
         of the ray if the calculate_distance parameter is set to True.

        :param from_position: The starting position of the ray in Cartesian world coordinates.
        :param to_position: The ending position of the ray in Cartesian world coordinates.
        :param calculate_distance: Whether to calculate the distance of the ray.
        :return: A RayResult object.
        """
        result = self._ray_test(from_position, to_position)
        if calculate_distance and not result.distance:
            result.update_distance(from_position, to_position)
        return result

    @abstractmethod
    def _ray_test(self, from_position: List[float], to_position: List[float]) -> RayResult:
        """ Cast a ray and return the first object hit, if any.

        :param from_position: The starting position of the ray in Cartesian world coordinates.
        :param to_position: The ending position of the ray in Cartesian world coordinates.
        :return: A RayResult object.
        """
        pass

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1, calculate_distances: bool = False) -> List[RayResult]:
        """
        A wrapper around the :py:meth:`~pycram.world.World._ray_test_batch` method that also calculates the distances
        of the rays if the calculate_distances parameter is set to True.

        :param from_positions: The starting positions of the rays in Cartesian world coordinates.
        :param to_positions: The ending positions of the rays in Cartesian world coordinates.
        :param num_threads: The number of threads to use to compute the ray intersections for the batch.
        :param calculate_distances: Whether to calculate the distances of the rays.
        :return: A list of RayResult objects.
        """
        results = self._ray_test_batch(from_positions, to_positions, num_threads)
        if calculate_distances:
            _ = [result.update_distance(from_positions[i], to_positions[i]) for i, result in enumerate(results)
                 if not result.distance]
        return results

    @abstractmethod
    def _ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                        num_threads: int = 1) -> List[RayResult]:
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
        return self._simulator_object_creator(self._create_visual_shape, visual_shape)

    def _create_visual_shape(self, visual_shape: VisualShape) -> int:
        """
        See :py:meth:`~pycram.world.World.create_visual_shape`
        """
        raise NotImplementedError

    def create_multi_body_from_visual_shapes(self, visual_shape_ids: List[int], pose: PoseStamped) -> int:
        """
        Creates a multi body from visual shapes in the physics simulator and returns the unique id of the created
        multi body.

        :param visual_shape_ids: The ids of the visual shapes that should be used to create the multi body.
        :param pose: The pose of the origin of the multi body relative to the world frame.
        :return: The unique id of the created multi body.
        """
        # Dummy parameter since these are needed to spawn visual shapes as a multibody.
        num_of_shapes = len(visual_shape_ids)
        link_poses = [PoseStamped() for _ in range(num_of_shapes)]
        link_masses = [1.0 for _ in range(num_of_shapes)]
        link_parent = [0 for _ in range(num_of_shapes)]
        link_joints = [JointType.FIXED.value for _ in range(num_of_shapes)]
        link_collision = [-1 for _ in range(num_of_shapes)]
        link_joint_axis = [Point(x=1, y=0, z=0) for _ in range(num_of_shapes)]

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
        return self._simulator_object_creator(self._create_multi_body, multi_body)

    def _create_multi_body(self, multi_body: MultiBody) -> int:
        """
        See :py:meth:`~pycram.world.World.create_multi_body`
        """
        raise NotImplementedError

    def create_box_visual_shape(self, shape_data: BoxVisualShape) -> int:
        """
        Creates a box visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the box visual shape to be created, uses the BoxVisualShape
         dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        return self._simulator_object_creator(self._create_box_visual_shape, shape_data)

    def _create_box_visual_shape(self, shape_data: BoxVisualShape) -> int:
        """
        See :py:meth:`~pycram.world.World.create_box_visual_shape`
        """
        raise NotImplementedError

    def create_cylinder_visual_shape(self, shape_data: CylinderVisualShape) -> int:
        """
        Creates a cylinder visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the cylinder visual shape to be created, uses the
         CylinderVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        return self._simulator_object_creator(self._create_cylinder_visual_shape, shape_data)

    def _create_cylinder_visual_shape(self, shape_data: CylinderVisualShape) -> int:
        """
        See :py:meth:`~pycram.world.World.create_cylinder_visual_shape`
        """
        raise NotImplementedError

    def create_sphere_visual_shape(self, shape_data: SphereVisualShape) -> int:
        """
        Creates a sphere visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the sphere visual shape to be created, uses the SphereVisualShape
         dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        return self._simulator_object_creator(self._create_sphere_visual_shape, shape_data)

    def _create_sphere_visual_shape(self, shape_data: SphereVisualShape) -> int:
        """
        See :py:meth:`~pycram.world.World.create_sphere_visual_shape`
        """
        raise NotImplementedError

    def create_capsule_visual_shape(self, shape_data: CapsuleVisualShape) -> int:
        """
        Creates a capsule visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the capsule visual shape to be created, uses the
         CapsuleVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        return self._simulator_object_creator(self._create_capsule_visual_shape, shape_data)

    def _create_capsule_visual_shape(self, shape_data: CapsuleVisualShape) -> int:
        """
        See :py:meth:`~pycram.world.World.create_capsule_visual_shape`
        """
        raise NotImplementedError

    def create_plane_visual_shape(self, shape_data: PlaneVisualShape) -> int:
        """
        Creates a plane visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the plane visual shape to be created, uses the PlaneVisualShape
         dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        return self._simulator_object_creator(self._create_plane_visual_shape, shape_data)

    def _create_plane_visual_shape(self, shape_data: PlaneVisualShape) -> int:
        """
        See :py:meth:`~pycram.world.World.create_plane_visual_shape`
        """
        raise NotImplementedError

    def create_mesh_visual_shape(self, shape_data: MeshVisualShape) -> int:
        """
        Creates a mesh visual shape in the physics simulator and returns the unique id of the created shape.

        :param shape_data: The parameters that define the mesh visual shape to be created,
        uses the MeshVisualShape dataclass defined in world_dataclasses.
        :return: The unique id of the created shape.
        """
        return self._simulator_object_creator(self._create_mesh_visual_shape, shape_data)

    def _create_mesh_visual_shape(self, shape_data: MeshVisualShape) -> int:
        """
        See :py:meth:`~pycram.world.World.create_mesh_visual_shape`
        """
        raise NotImplementedError

    def add_text(self, text: str, position: List[float], orientation: Optional[List[float]] = None, size: float = 0.1,
                 color: Optional[Color] = Color(), life_time: Optional[float] = 0,
                 parent_object_id: Optional[int] = None, parent_link_id: Optional[int] = None) -> int:
        """
        Adds text to the world.

        :param text: The text to be added.
        :param position: The position of the text in the world.
        :param orientation: By default, debug text will always face the camera, automatically rotation. By specifying a
         text orientation (quaternion), the orientation will be fixed in world space or local space
          (when parent is specified).
        :param size: The size of the text.
        :param color: The color of the text.
        :param life_time: The lifetime in seconds of the text to remain in the world, if 0 the text will remain in the
         world until it is removed manually.
        :param parent_object_id: The id of the object to which the text should be attached.
        :param parent_link_id: The id of the link to which the text should be attached.
        :return: The id of the added text.
        """
        return self._simulator_object_creator(self._add_text, text, position, orientation, size, color, life_time,
                                              parent_object_id, parent_link_id)

    def _add_text(self, text: str, position: List[float], orientation: Optional[List[float]] = None, size: float = 0.1,
                  color: Optional[Color] = Color(), life_time: Optional[float] = 0,
                  parent_object_id: Optional[int] = None, parent_link_id: Optional[int] = None) -> int:
        """
        See :py:meth:`~pycram.world.World.add_text`
        """
        raise NotImplementedError

    def remove_text(self, text_id: Optional[int] = None) -> None:
        """
        Removes text from the world using the given id. if no id is given all text will be removed.

        :param text_id: The id of the text to be removed.
        """
        self._simulator_object_remover(self._remove_text, text_id)

    def _remove_text(self, text_id: Optional[int] = None) -> None:
        """
        See :py:meth:`~pycram.world.World.remove_text`
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
        Get the joint reaction forces and torques of the specified joint.

        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the force torque should be returned.
        :return: The joint reaction forces and torques of the specified joint.
        """
        raise NotImplementedError

    def get_applied_joint_motor_torque(self, obj: Object, joint_id: int) -> float:
        """
        Get the applied torque by a joint motor.

        :param obj: The object in which the joint is located.
        :param joint_id: The id of the joint for which the applied motor torque should be returned.
        :return: The applied torque by a joint motor.
        """
        raise NotImplementedError

    def pause_world_sync(self) -> None:
        """
        Pause the world synchronization.
        """
        self.world_sync.sync_lock.acquire()

    def resume_world_sync(self) -> None:
        """
        Resume the world synchronization.
        """
        if self.world_sync.sync_lock.locked():
            self.world_sync.sync_lock.release()

    def add_vis_axis(self, pose: PoseStamped) -> int:
        """
        Add a visual axis to the world.

        :param pose: The pose of the visual axis.
        :return: The id of the added visual axis.
        """
        return self._simulator_object_creator(self._add_vis_axis, pose)

    def _add_vis_axis(self, pose: PoseStamped) -> None:
        """
        See :py:meth:`~pycram.world.World.add_vis_axis`
        """
        logwarn(f"Visual axis is not supported in {self.__class__.__name__}")

    def remove_vis_axis(self) -> None:
        """
        Remove the visual axis from the world.
        """
        self._simulator_object_remover(self._remove_vis_axis)

    def _remove_vis_axis(self) -> None:
        """
        See :py:meth:`~pycram.world.World.remove_vis_axis`
        """
        logwarn(f"Visual axis is not supported in {self.__class__.__name__}")

    def _simulator_object_creator(self, creator_func: Callable, *args, **kwargs) -> int:
        """
        Create an object in the physics simulator and returns the created object id.

        :param creator_func: The function that creates the object in the physics simulator.
        :param args: The arguments for the creator function.
        :param kwargs: The keyword arguments for the creator function.
        :return: The created object id.
        """
        obj_id = creator_func(*args, **kwargs)
        self.update_simulator_state_id_in_original_state()
        return obj_id

    def _simulator_object_remover(self, remover_func: Callable, *args, **kwargs) -> None:
        """
        Remove an object from the physics simulator.

        :param remover_func: The function that removes the object from the physics simulator.
        :param args: The arguments for the remover function.
        :param kwargs: The keyword arguments for the remover function.
        """
        remover_func(*args, **kwargs)
        self.update_simulator_state_id_in_original_state()

    def update_original_state(self) -> int:
        """
        Update the original state of the world.
        :return: The id of the updated original state.
        """
        return self.save_state(self.original_state_id, use_same_id=True)

    def update_simulator_state_id_in_original_state(self, use_same_id: bool = False) -> None:
        """
        Update the simulator state id in the original state if use_physics_simulator_state is True in the configuration.

        :param use_same_id: If the same id should be used for the state.
        """
        if self.conf.use_physics_simulator_state:
            self.original_state.simulator_state_id = self.save_physics_simulator_state(use_same_id=use_same_id)

    @property
    def original_state(self) -> WorldState:
        """
        The saved original state of the world.
        """
        return self.saved_states[self.original_state_id]

    def __eq__(self, other: World):
        if not isinstance(other, self.__class__):
            return False
        return (self.is_prospection_world == other.is_prospection_world
                and self.id == other.id)

    def __hash__(self):
        return hash((self.__class__.__name__, self.is_prospection_world, self.id))


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
        # Please do not edit this function, it works as it is now!
        if not World.current_world.is_prospection_world:
            self.prev_world = World.current_world
            World.current_world = World.current_world.prospection_world
            # This is also a join statement since it is called from the main thread.
            World.current_world.world_sync.sync_worlds()

    def __exit__(self, *args):
        """
        This method is called when exiting the with block, it will restore the previous world to be the current world.
        """
        if self.prev_world is not None:
            World.current_world = self.prev_world


class WorldSync(threading.Thread):
    """
    Synchronizes the state between the World and its prospection world.
    Meaning the cartesian and joint position of everything in the prospection world will be
    synchronized with the main World.
    The class provides the possibility to pause the synchronization, this can be used
    if reasoning should be done in the prospection world.
    """

    WAIT_TIME_AS_N_SIMULATION_STEPS = 20
    """
    The time in simulation steps to wait between each iteration of the syncing loop.
    """

    def __init__(self, world: World, prospection_world: World):
        threading.Thread.__init__(self)
        self.world: World = world
        self.prospection_world: World = prospection_world
        self.prospection_world.world_sync = self

        self.terminate: bool = False
        self.pause_sync: bool = False
        # Maps world to prospection world objects
        self.object_to_prospection_object_map: Dict[Object, Object] = {}
        self.prospection_object_to_object_map: Dict[Object, Object] = {}
        self.equal_states = False
        self.sync_lock: threading.Lock = threading.Lock()

    def run(self):
        """
        Main method of the synchronization, this thread runs in a loop until the
        terminate flag is set.
        While this loop runs it continuously checks the cartesian and joint position of
        every object in the World and updates the corresponding object in the
        prospection world.
        """
        while not self.terminate:
            self.sync_lock.acquire()
            time.sleep(WorldSync.WAIT_TIME_AS_N_SIMULATION_STEPS * self.world.simulation_time_step)
            self.sync_lock.release()

    def get_world_object(self, prospection_object: Object) -> Object:
        """
        Get the corresponding object from the main World for a given object in the prospection world.

        :param prospection_object: The object for which the corresponding object in the main World should be found.
        :return: The object in the main World.
        """
        try:
            return self.prospection_object_to_object_map[prospection_object]
        except KeyError:
            if prospection_object in self.world.objects:
                return prospection_object
            raise ObjectNotFound(prospection_object)

    def get_prospection_object(self, obj: Object) -> Object:
        """
        Get the corresponding object from the prospection world for a given object in the main world.

        :param obj: The object for which the corresponding object in the prospection World should be found.
        :return: The corresponding object in the prospection world.
        """
        try:
            return self.object_to_prospection_object_map[obj]
        except KeyError:
            if obj in self.prospection_world.objects:
                return obj
            raise ProspectionObjectNotFound(obj)

    def sync_worlds(self):
        """
        Syncs the prospection world with the main world by adding and removing objects and synchronizing their states.
        """
        self.remove_objects_not_in_world()
        self.add_objects_not_in_prospection_world()
        self.prospection_object_to_object_map = {prospection_obj: obj for obj, prospection_obj in
                                                 self.object_to_prospection_object_map.items()}
        self.sync_objects_states()

    def remove_objects_not_in_world(self):
        """
        Removes all objects that are not in the main world from the prospection world.
        """
        obj_map_copy = copy(self.object_to_prospection_object_map)
        [self.remove_object(obj) for obj in obj_map_copy.keys() if obj not in self.world.objects]

    def add_objects_not_in_prospection_world(self):
        """
        Adds all objects that are in the main world but not in the prospection world to the prospection world.
        """
        obj_map_copy = copy(self.object_to_prospection_object_map)
        [self.add_object(obj) for obj in self.world.objects if obj not in obj_map_copy.keys()]

    def add_object(self, obj: Object) -> None:
        """
        Adds an object to the prospection world.

        :param obj: The object to be added.
        """
        self.object_to_prospection_object_map[obj] = obj.copy_to_prospection()

    def remove_object(self, obj: Object) -> None:
        """
        Removes an object from the prospection world.

        :param obj: The object to be removed.
        """
        prospection_obj = self.object_to_prospection_object_map[obj]
        prospection_obj.remove()
        del self.object_to_prospection_object_map[obj]

    def sync_objects_states(self) -> None:
        """
        Synchronizes the state of all objects in the World with the prospection world.
        """
        # Set the pose of the prospection objects to the pose of the world objects
        obj_pose_dict = {prospection_obj: obj.pose
                         for obj, prospection_obj in self.object_to_prospection_object_map.items()}
        for obj, prospection_obj in self.object_to_prospection_object_map.items():
            prospection_obj.set_attachments(obj.attachments)
            prospection_obj.joint_states = obj.joint_states
        self.world.prospection_world.reset_multiple_objects_base_poses(obj_pose_dict)

    def check_for_equal(self) -> bool:
        """
        Checks if both Worlds have the same state, meaning all objects are in the same position.
        This is currently not used, but might be used in the future if synchronization issues worsen.

        :return: True if both Worlds have the same state, False otherwise.
        """
        eql = True
        prospection_names = self.prospection_world.get_object_names()
        eql = eql and [name in prospection_names for name in self.world.get_object_names()]
        eql = eql and len(prospection_names) == len(self.world.get_object_names())
        if not eql:
            return False
        for obj, prospection_obj in self.object_to_prospection_object_map.items():
            eql = eql and obj.get_pose().position.euclidean_distance(prospection_obj.get_pose().position) < 0.001
        self.equal_states = eql
        return eql
