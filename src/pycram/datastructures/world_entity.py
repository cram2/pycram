from __future__ import annotations

import os
import pickle
from abc import ABC, abstractmethod

from typing_extensions import TYPE_CHECKING, Dict, Optional, List

from .dataclasses import State, ContactPointsList, ClosestPointsList, Color, VisualShape, PhysicalBodyState

if TYPE_CHECKING:
    from ..datastructures.world import World
    from .pose import Pose, Point, GeoQuaternion as Quaternion


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


class PhysicalBody(WorldEntity, ABC):
    """
    A class that represents a physical body in the world that has some related physical properties.
    """

    def __init__(self, body_id: int, world: World):
        WorldEntity.__init__(self, body_id, world)
        self._is_translating: Optional[bool] = None
        self._is_rotating: Optional[bool] = None
        self._velocity: Optional[List[float]] = None

    @property
    def current_state(self) -> PhysicalBodyState:
        return PhysicalBodyState(self.pose, self.is_translating, self.is_rotating, self.velocity, self.contact_points)

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

    @abstractmethod
    @property
    def color(self) -> Color:
        """
        :return: The color of this body.
        """
        ...

    @abstractmethod
    @property
    def shape(self) -> VisualShape:
        """
        :return: The shape of this body.
        """
        ...

    @abstractmethod
    @property
    def pose(self) -> Pose:
        """
        :return: The pose of this entity.
        """
        ...

    @abstractmethod
    @property
    def contact_points(self) -> ContactPointsList:
        """
        :return: The contact points of this body with other physical bodies.
        """
        ...

    @abstractmethod
    def get_contact_points_with_body(self, body: 'PhysicalBody') -> ContactPointsList:
        """
        :param body: The body to get the contact points with.
        :return: The contact points of this body with the given body.
        """
        ...

    @abstractmethod
    @property
    def distances(self) -> Dict['PhysicalBody', float]:
        """
        :return: The closest distances of this body to other physical bodies.
        """
        ...

    @abstractmethod
    def get_distance_with_body(self, body: 'PhysicalBody') -> float:
        """
        :param body: The body to get the distance with.
        :return: The closest distance of this body to the given body.
        """
        ...

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

