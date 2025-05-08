from __future__ import annotations

import os
import pickle
from abc import ABC, abstractmethod
from copy import copy
from threading import RLock

from trimesh.parent import Geometry3D
from typing_extensions import TYPE_CHECKING, Dict, Optional, List, deprecated, Union, Type, Tuple

from pycrap.ontologies import PhysicalObject, Room, Location, Robot, Floor
from .dataclasses import State, ContactPointsList, ClosestPointsList, Color, PhysicalBodyState, \
    AxisAlignedBoundingBox, RotatedBoundingBox, RayResult
from .enums import AdjacentBodyMethod, AxisIdentifier, Arms, Grasp
from .mixins import HasConcept
from ..local_transformer import LocalTransformer
from ..ros import Time, logdebug
from .pose import GraspDescription, Vector3, PoseStamped
from .grasp import PreferredGraspAlignment

if TYPE_CHECKING:
    from ..datastructures.world import World
    from ..world_concepts.world_object import Object
    from .pose import PoseStamped, GeoQuaternion as Quaternion, TransformStamped, Point


class StateEntity:
    """
    The StateEntity class is used to store the state of an object or the physics simulator. This is used to save and
    restore the state of the World.
    """

    SAVED_FILE_PREFIX = 'pycram_saved_state_'

    def __init__(self):
        self._saved_states: Dict[int, State] = {}

    @property
    def saved_states(self) -> Dict[int, State]:
        """
        :return: the saved states of this entity.
        """
        return self._saved_states

    def save_state(self, state_id: int, save_dir: Optional[str] = None) -> int:
        """
        Saves the state of this entity with the given state id.

        :param state_id: The unique id of the state.
        :param save_dir: The directory where the file should be saved.
        """
        self._saved_states[state_id] = self.current_state
        if save_dir is not None:
            self.save_state_to_file(self.get_saved_file_path_of_state(save_dir, state_id), state_id)
        return state_id

    def save_state_to_file(self, file_path: str, state_id: Optional[int] = None):
        """
        Saves the state of this entity to a file.

        :param file_path: The path to the file.
        :param state_id: The unique id of the state if saving a specific state not the current state.
        """
        state_to_save = self.current_state if state_id is None else self.saved_states[state_id]
        with open(file_path, 'wb') as file:
            pickle.dump(state_to_save, file)

    def restore_state_from_file(self, save_dir: str, state_id: int) -> None:
        """
        Restores the state of this entity from a file.

        :param save_dir: The directory where the file is saved.
        :param state_id: The unique id of the state.
        """
        with open(self.get_saved_file_path_of_state(save_dir, state_id), 'rb') as file:
            self.current_state = pickle.load(file)

    def get_saved_file_path_of_state(self, save_dir: str, state_id: int) -> str:
        """
        Gets the path of the file that stores the state with the given id.

        :param save_dir: The directory where the file should be saved.
        :param state_id: The unique id of the state.
        :return: The name of the file that stores the state with the given id.
        """
        return os.path.join(save_dir, f'{self.SAVED_FILE_PREFIX}{state_id}.pkl')

    @property
    @abstractmethod
    def current_state(self) -> State:
        """
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

    def restore_state(self, state_id: int, saved_file_dir: Optional[str] = None) -> None:
        """
        Restores the state of this entity from a saved state using the given state id.

        :param state_id: The unique id of the state.
        :param saved_file_dir: The directory where the file is saved.
        """
        if saved_file_dir is not None:
            self.restore_state_from_file(saved_file_dir, state_id)
        else:
            self.current_state = self.saved_states[state_id]

    def remove_saved_states(self) -> None:
        """
        Removes all saved states of this entity.
        """
        self._saved_states = {}


class WorldEntity(StateEntity, HasConcept):
    """
    A class that represents an entity of the world, such as an object or a link.
    """

    def __init__(self, _id: int, world: World, concept: Type[PhysicalObject] = PhysicalObject, parse_name: bool = True):
        StateEntity.__init__(self)
        self.id = _id
        self.world: World = world
        HasConcept.__init__(self, name=self.name, world=self.world, concept=concept, parse_name=parse_name)

    def reset_concepts(self):
        """
        Reset the concepts of this entity.
        """
        if not self.world.is_prospection_world and hasattr(self.ontology_individual, 'reload'):
            self.ontology_individual.reload()

    @property
    @abstractmethod
    def name(self) -> str:
        """
        :return: The name of this body.
        """
        ...

    @property
    @abstractmethod
    def parent_entity(self) -> Optional[WorldEntity]:
        """
        :return: The parent entity of this entity, if it has one.
        """
        pass

    def __eq__(self, other: WorldEntity) -> bool:
        """
        Check if this body is equal to another body.
        """
        if not isinstance(other, WorldEntity):
            return False
        return self.id == other.id and self.name == other.name and self.parent_entity == other.parent_entity

    def __hash__(self) -> int:
        return hash((self.id, self.name, self.parent_entity))


class PhysicalBody(WorldEntity):
    """
    A class that represents a physical body in the world that has some related physical properties.
    """

    def __init__(self, body_id: int, world: World, concept: Type[PhysicalObject] = PhysicalObject,
                 parse_name: bool = True):
        WorldEntity.__init__(self, body_id, world, concept, parse_name)

        self.local_transformer = LocalTransformer()
        self._is_translating: Optional[bool] = None
        self._is_rotating: Optional[bool] = None
        self._velocity: Optional[List[float]] = None
        self.ontology_lock: RLock = RLock()
        self.updated_containment_of_parts: bool = False
        self.latest_known_parts: Dict[str, PhysicalBody] = {}

    def reset_concepts(self):
        super().reset_concepts()
        self.contained_bodies = []
        for part in self.parts.values():
            part.reset_concepts()

    @property
    def is_an_environment(self) -> bool:
        """
        Check if the object is of type environment.

        :return: True if the object is of type environment, False otherwise.
        """
        return ((isinstance(self.parent_entity, PhysicalBody) and self.parent_entity.is_an_environment) or
                (issubclass(self.ontology_concept, Location) or issubclass(self.ontology_concept, Floor)))

    @property
    def is_a_robot(self) -> bool:
        """
        Check if the object is a robot.
        TODO: Check if this is a the correct filter
        :return: True if the object is a robot, False otherwise.
        """
        return (issubclass(self.ontology_concept, Robot) or
                (isinstance(self.parent_entity, PhysicalBody) and self.parent_entity.is_a_robot))

    @property
    @abstractmethod
    def parts(self) -> Dict[str, PhysicalBody]:
        """
        :return: The parts of this body as a dictionary mapping the part name to the part.
        """
        ...

    def update_containment(self, excluded_bodies: Optional[List[PhysicalBody]] = None,
                           candidate_selection_method: AdjacentBodyMethod = AdjacentBodyMethod.ClosestPoints,
                           max_distance: float = 0.5) -> None:
        """
        Update the containment of the object by checking if it is contained in other bodies,
         excluding the given excluded bodies.

        :param excluded_bodies: The bodies that should be excluded from the containment check.
        :param candidate_selection_method: The method to select the candidates for the containment check.
        :param max_distance: The maximum distance from this body to other bodies to consider a body as a candidate.
        """
        excluded_bodies = [] if excluded_bodies is None else excluded_bodies
        excluded_bodies.append(self)
        if candidate_selection_method == AdjacentBodyMethod.ClosestPoints:
            bodies = self.get_adjacent_bodies_using_closest_points(max_distance)
        else:
            bodies = self.get_adjacent_bodies_using_rays(max_distance)
        for body in bodies:
            if body in excluded_bodies:
                continue
            if body.get_axis_aligned_bounding_box().contains_box(self.get_axis_aligned_bounding_box()):
                logdebug(f"{body.name} contains {self.name}")
                body.contained_bodies = [self]

    def get_adjacent_bodies_using_closest_points(self, max_distance: float = 0.5) -> List[PhysicalBody]:
        """
        Get the adjacent bodies of the body using the closest points between the bodies.

        :param max_distance: The maximum distance to consider a body as adjacent.
        :return: The bodies that are adjacent to this body if any.
        """
        closest_points = self.closest_points(max_distance)
        return closest_points.get_all_bodies(excluded=[self])

    def get_adjacent_bodies_using_rays(self, max_distance: float = 0.5) -> List[PhysicalBody]:
        """
        Cast rays in the 6 rays in all directions (+x, -x, +y, -y, +z,  and -z directions) to find the adjacent bodies
         of the body.

        :param max_distance: The max distance that the rays can travel.
        :return: The bodies that are adjacent to this body if any.
        """
        if max_distance <= 0:
            raise ValueError("The distance should be greater than zero.")

        rays_start, rays_end = self.cast_rays_in_all_directions(max_distance)
        rays_results = self.world.ray_test_batch(rays_start, rays_end)

        bodies_ids = set([(rr.obj_id, rr.link_id) for rr in rays_results if rr.intersected])

        bodies = [self.world.get_object_by_id(obj_id).get_link_by_id(link_id) for obj_id, link_id in bodies_ids]

        return bodies

    def cast_rays_in_all_directions(self, max_distance: float = 0.5) -> List[RayResult]:
        """
        Get the rays in all 6 directions (+x, -x, +y, -y, +z,  and -z directions).

        :param max_distance: The maximum distance the rays can travel.
        :return: The rays in all 6 directions.
        """
        bb = self.get_axis_aligned_bounding_box()
        origin = bb.origin
        min_, max_ = bb.get_min_max()

        rays_start, rays_end = [], []
        for i in range(3):
            i_rays_start, i_rays_end = self.get_axis_rays(origin, min_[i], max_[i], i, max_distance)
            rays_start.extend(i_rays_start)
            rays_end.extend(i_rays_end)

        return self.world.ray_test_batch(rays_start, rays_end)

    @staticmethod
    def get_axis_rays(origin: List[float], min_val: float, max_val: float, idx: int, max_distance: float = 0.5)\
            -> Tuple[List[List[float]], List[List[float]]]:
        """
        Get the rays in the given axis.

        :param origin: The origin of the rays.
        :param min_val: The start in the -ve direction of the axis.
        :param max_val: The start in the +ve direction of the axis.
        :param idx: The index of the direction/axis.
        :param max_distance: The maximum distance the rays can travel.
        """
        ray_left_start = [min_val if i == idx else origin[i] for i in range(3)]
        ray_left_end = [min_val - max_distance if i == idx else origin[i] for i in range(3)]
        ray_right_start = [max_val if i == idx else origin[i] for i in range(3)]
        ray_right_end = [max_val + max_distance if i == idx else origin[i] for i in range(3)]
        return [ray_left_start, ray_right_start], [ray_left_end, ray_right_end]

    def contains_body(self, body: PhysicalBody) -> bool:
        """
        Check if this body contains another body.

        :param body: The physical body to check if it is contained by this body.
        :return: True if the body contains the other body, otherwise False.
        """
        with self.ontology_lock:
            contained_bodies = self.contained_bodies
            return body in contained_bodies or (body.parent_entity and body.parent_entity in contained_bodies)

    @property
    def contained_bodies(self) -> List[PhysicalBody]:
        """
        :return: True if the object contains the other object, otherwise False.
        """
        with self.ontology_lock:
            self.world.ontology.reason()
            return [self.world.ontology.python_objects[phys_obj]
                    for phys_obj in self.ontology_individual.contains_object]

    @contained_bodies.setter
    def contained_bodies(self, bodies: List[PhysicalBody]) -> None:
        """
        Set the bodies that are contained by this body.

        :param bodies: The bodies that are contained in this body.
        """
        with self.ontology_lock:
            self.ontology_individual.contains_object = [body.ontology_individual for body in bodies]

    @abstractmethod
    def get_axis_aligned_bounding_box(self) -> AxisAlignedBoundingBox:
        """
        :return: The axis-aligned bounding box of this body.
        """
        ...

    @abstractmethod
    def get_rotated_bounding_box(self) -> RotatedBoundingBox:
        """
        :return: The rotated bounding box of this body.
        """
        ...

    def _plot_convex_hull(self):
        """
        Plot the convex hull of the geometry.
        """
        self.get_convex_hull().show()

    @abstractmethod
    def get_convex_hull(self) -> Geometry3D:
        """
        :return: The convex hull of this body.
        """
        ...

    @property
    def body_state(self) -> PhysicalBodyState:
        return PhysicalBodyState(self.pose.copy(), self.is_translating, self.is_rotating, copy(self.velocity)
                                 , self.world.conf.get_pose_tolerance())

    @body_state.setter
    def body_state(self, state: PhysicalBodyState) -> None:
        if self.body_state != state:
            self.pose = state.pose
            self.is_translating = state.is_translating
            self.is_rotating = state.is_rotating
            self.velocity = state.velocity

    @property
    def velocity(self) -> Optional[List[float]]:
        return self._velocity

    @velocity.setter
    def velocity(self, velocity: List[float]) -> None:
        self._velocity = velocity

    @property
    def is_translating(self) -> Optional[bool]:
        return self._is_translating

    @is_translating.setter
    def is_translating(self, is_translating: bool) -> None:
        self._is_translating = is_translating

    @property
    def is_rotating(self) -> Optional[bool]:
        return self._is_rotating

    @is_rotating.setter
    def is_rotating(self, is_rotating: bool) -> None:
        self._is_rotating = is_rotating

    @property
    def position(self) -> Point:
        """
        :return: A Point object containing the position of the link relative to the world frame.
        """
        return self.pose.position


    @property
    def orientation(self) -> Quaternion:
        """
        :return: A Quaternion object containing the orientation of the link relative to the world frame.
        """
        return self.pose.orientation

    @property
    def pose_as_list(self) -> List[List[float]]:
        """
        :return: A list containing the position and orientation of the link relative to the world frame.
        """
        return self.pose.to_list()

    @property
    def transform(self) -> TransformStamped:
        """
        The transform of this entity.

        :return: The transform of this entity.
        """
        return self.pose.to_transform_stamped(self.tf_frame)

    def update_transform(self, transform_time: Optional[Time] = None) -> None:
        """
        Update the transformation of this link at the given time.

        :param transform_time: The time at which the transformation should be updated.
        """
        self.local_transformer.update_transforms([self.transform], transform_time)

    @property
    @abstractmethod
    def pose(self) -> PoseStamped:
        """
        :return: A Pose object containing the pose of the link relative to the world frame.
        """
        ...

    @pose.setter
    @abstractmethod
    def pose(self, pose: PoseStamped) -> None:
        """
        Set the pose of this body.

        :param pose: The pose of this body.
        """
        ...

    @property
    @abstractmethod
    def tf_frame(self) -> str:
        """
        The tf frame of this entity.

        :return: The tf frame of this entity.
        """
        pass

    @property
    def contact_points(self) -> ContactPointsList:
        """
        :return: The contact points of this body with other physical bodies.
        """
        return self.world.get_body_contact_points(self)

    def get_contact_points_with_body(self, body: PhysicalBody) -> ContactPointsList:
        """
        :param body: The body to get the contact points with.
        :return: The contact points of this body with the given body.
        """
        return self.world.get_contact_points_between_two_bodies(self, body)

    def closest_points(self, max_distance: float) -> ClosestPointsList:
        """
        :param max_distance: The maximum distance to consider a body as close, only points closer than or equal to this
         distance will be returned.
        :return: The closest points of this body with other physical bodies within the given maximum distance.
        """
        return self.world.get_body_closest_points(self, max_distance)

    def get_closest_points_with_body(self, body: PhysicalBody, max_distance: float) -> ClosestPointsList:
        """
        :param body: The body to get the points with.
        :param max_distance: The maximum distance to consider a body as close, only points closer than or equal to this
         distance will be returned.
        :return: The closest points of this body with the given body within the given maximum distance.
        """
        return self.world.get_closest_points_between_two_bodies(self, body, max_distance)

    @property
    def is_moving(self) -> Optional[bool]:
        """
        :return: True if this body is moving, False if not, and None if not known.
        """
        if self.is_translating is not None or self.is_rotating is not None:
            return self.is_translating or self.is_rotating
        else:
            return None

    @property
    def is_stationary(self) -> Optional[bool]:
        """
        :return: True if this body is stationary, False otherwise.
        """
        return None if self.is_moving is None else not self.is_moving

    @property
    @abstractmethod
    def color(self) -> Union[Color, Dict[str, Color]]:
        """
        :return: A Color object containing the rgba_color of this body or a dictionary if the body is articulated.
        """
        ...

    @deprecated("Use color property setter instead")
    def set_color(self, color: Color) -> None:
        """
        Set the color of this body, could be rgb or rgba.

        :param color: The color as a list of floats, either rgb or rgba.
        """
        self.color = color

    @color.setter
    @abstractmethod
    def color(self, color: Color) -> None:
        """
        Set the color of this body, could be rgb or rgba.

        :param color: The color as a list of floats, either rgb or rgba.
        """
        ...

    def get_preferred_grasp_alignment(self) -> PreferredGraspAlignment:
        """
        Determines the preferred grasp alignment for an object based on its type.
        This includes the preferred axis, whether grasping from the top is preferred,
        and whether horizontal alignment (90° rotation around X) is preferred.

        :return: The preferred grasp alignment for the object.
        """
        object_type = self.ontology_concept

        try:
            pref_axis = object_type.has_preferred_alignment[0].has_preferred_axis[0].value
            sidegrasp_axis = AxisIdentifier.from_tuple(pref_axis)
            grasp_top = object_type.has_preferred_alignment[0].has_vertical_alignment[0].value
            grasp_horizontal = object_type.has_preferred_alignment[0].has_rotated_gripper[0].value
        except IndexError:
            sidegrasp_axis, grasp_top, grasp_horizontal = (None, False, False)

        preferred_alignment = PreferredGraspAlignment(sidegrasp_axis, grasp_top, grasp_horizontal)

        return preferred_alignment

    def calculate_grasp_descriptions(self, robot: Object) -> List[GraspDescription]:
        """
        Calculates the grasp configurations of an object relative to the robot based on orientation and position.

        This method determines the possible grasp configurations (side and top/bottom faces) of the object,
        taking into account the object's orientation, position, and whether horizontal alignment is preferred.

        :param robot: The robot for which the grasp configurations are being calculated.

        :return: A sorted list of GraspDescription instances representing all grasp permutations.
        """
        objectTmap = self.pose

        preferred_grasp_alignment = self.get_preferred_grasp_alignment()

        grasp_configs = objectTmap.calculate_grasp_descriptions(robot, preferred_grasp_alignment)

        return grasp_configs

    def get_grasp_pose(self, end_effector, grasp: GraspDescription) -> Pose:
        """
        Translates the grasp pose of the object using the desired grasp description and object knowledge.
        Leaves the orientation untouched.
        Returns the translated grasp pose.

        :param end_effector: The end effector that will be used to grasp the object.
        :param grasp: The desired grasp description.

        :return: The grasp pose of the object.
        """
        grasp_pose = self.pose.copy()

        if self.ontology_concept.has_preferred_alignment[0].has_rim_grasp[0].value:
            approach_axis = end_effector.get_approach_axis()
            approach_direction = grasp.approach_direction
            rim_direction = GraspDescription(approach_direction, None, False)
            rim_direction_index = approach_direction.value[0].value.index(1)
            rim_offset = self.get_rotated_bounding_box().dimensions[rim_direction_index] / 2
            grasp_pose.rotate_by_quaternion(end_effector.grasps[rim_direction])
            grasp_pose = LocalTransformer().translate_pose_along_local_axis(grasp_pose, approach_axis, -rim_offset)
            grasp_pose = PoseStamped.from_list(grasp_pose.position.to_list(), self.orientation.to_list())

        return grasp_pose

    def get_approach_offset(self) -> float:
        """
        :return: The pre-grasp offset of the object. It is the largest dimension of the object divided by 2.
        """
        max_object_dimension = max(self.get_rotated_bounding_box().dimensions)
        return max_object_dimension / 2
