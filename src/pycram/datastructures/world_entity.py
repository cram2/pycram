from __future__ import annotations

import os
import pickle
from abc import ABC, abstractmethod
from copy import copy

from trimesh.parent import Geometry3D
from typing_extensions import TYPE_CHECKING, Dict, Optional, List, deprecated, Union, Type

from pycrap.ontologies import PhysicalObject
from .dataclasses import State, ContactPointsList, ClosestPointsList, Color, PhysicalBodyState, \
    AxisAlignedBoundingBox, RotatedBoundingBox
from .mixins import HasConcept
from ..local_transformer import LocalTransformer
from ..ros.data_types import Time

if TYPE_CHECKING:
    from ..datastructures.world import World
    from .pose import Pose, Point, GeoQuaternion as Quaternion, Transform


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


class WorldEntity(StateEntity, ABC):
    """
    A class that represents an entity of the world, such as an object or a link.
    """

    def __init__(self, _id: int, world: World):
        StateEntity.__init__(self)
        self.id = _id
        self.world: World = world

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


class PhysicalBody(WorldEntity, HasConcept, ABC):
    """
    A class that represents a physical body in the world that has some related physical properties.
    """

    ontology_concept: Type[PhysicalObject] = PhysicalObject
    """
    The ontology concept of this entity.
    """

    def __init__(self, body_id: int, world: World, concept: Type[PhysicalObject] = PhysicalObject):
        WorldEntity.__init__(self, body_id, world)
        HasConcept.__init__(self)

        # set ontology related information
        self.ontology_concept = concept
        if not self.world.is_prospection_world:
            self.ontology_individual = self.ontology_concept(namespace=self.world.ontology.ontology)

        self.local_transformer = LocalTransformer()
        self._is_translating: Optional[bool] = None
        self._is_rotating: Optional[bool] = None
        self._velocity: Optional[List[float]] = None

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

    @property
    def pose_as_list(self) -> List[List[float]]:
        """
        :return: A list containing the position and orientation of the link relative to the world frame.
        """
        return self.pose.to_list()

    @property
    def transform(self) -> Transform:
        """
        The transform of this entity.

        :return: The transform of this entity.
        """
        return self.pose.to_transform(self.tf_frame)

    def update_transform(self, transform_time: Optional[Time] = None) -> None:
        """
        Update the transformation of this link at the given time.

        :param transform_time: The time at which the transformation should be updated.
        """
        self.local_transformer.update_transforms([self.transform], transform_time)

    @property
    @abstractmethod
    def pose(self) -> Pose:
        """
        :return: A Pose object containing the pose of the link relative to the world frame.
        """
        ...

    @pose.setter
    @abstractmethod
    def pose(self, pose: Pose) -> None:
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
