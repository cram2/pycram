from abc import ABC, abstractmethod

from typing_extensions import TYPE_CHECKING, Dict

from .dataclasses import State

if TYPE_CHECKING:
    from ..datastructures.world import World


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
        :return: the saved states of this entity.
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


class WorldEntity(StateEntity, ABC):
    """
    A data class that represents an entity of the world, such as an object or a link.
    """

    def __init__(self, _id: int, world: 'World'):
        StateEntity.__init__(self)
        self.id = _id
        self.world: 'World' = world
