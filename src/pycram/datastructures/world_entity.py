import os
import pickle
from abc import ABC, abstractmethod

from typing_extensions import TYPE_CHECKING, Dict, Optional

from .dataclasses import State

if TYPE_CHECKING:
    from ..datastructures.world import World


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
    A data class that represents an entity of the world, such as an object or a link.
    """

    def __init__(self, _id: int, world: 'World'):
        StateEntity.__init__(self)
        self.id = _id
        self.world: 'World' = world
