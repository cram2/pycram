from typing import Dict, List, Optional
from pycram.datastructures.enums import WorldMode
from pycram.datastructures.world import World


class WorldFactory:
    """
    A factory for creating and managing multiple World instances.

    This factory provides a method to create new world instances with configurable parameters,
    including a type identifier ("belief" or "prospection"). It also maintains a registry of created
    worlds for easier management and lookup.
    """

    # Registry to hold created worlds keyed by their unique id.
    _registry: Dict[int, World] = {}
    # Internal counter for assigning unique ids.
    _id_counter: int = 0

    @classmethod
    def create_world(cls, world_type: str, mode: WorldMode = WorldMode.DIRECT, clear_cache: bool = False,
                     **kwargs) -> World:
        """
        Create a new world instance based on the specified type.

        :param world_type: A string identifier for the world type ("belief" or "prospection").
        :param mode: The simulation mode (e.g., WorldMode.DIRECT or WorldMode.GUI).
        :param clear_cache: Whether to clear the cache when creating the world.
        :param kwargs: Additional keyword arguments to pass to the World constructor.
        :return: The newly created World instance.
        """
        # Assign a unique id to the new world.
        world_id = cls._id_counter
        cls._id_counter += 1

        if world_type.lower() == "belief":
            # Create the main belief world (set_as_main=True ensures it becomes the global current world).
            world = World(
                mode=mode,
                is_prospection=False,
                clear_cache=clear_cache,
                id_=world_id,
                set_as_main=True,  # This sets the global current_world for belief state.
                world_type=world_type,
                **kwargs
            )
        elif world_type.lower() == "prospection":
            # Create a prospection world (not to override the main world).
            world = World(
                mode=mode,
                is_prospection=True,
                clear_cache=clear_cache,
                id_=world_id,
                set_as_main=False,  # Prospection worlds will not become the global main world.
                world_type=world_type,
                **kwargs
            )
        else:
            raise ValueError(f"Unknown world type: {world_type}")

        # Register the new world in the factory's registry.
        cls._registry[world_id] = world
        return world

    @classmethod
    def get_world(cls, world_id: int) -> Optional[World]:
        """
        Retrieve a world by its unique identifier.

        :param world_id: The unique id of the world.
        :return: The World instance if found, else None.
        """
        return cls._registry.get(world_id)

    @classmethod
    def list_worlds(cls) -> List[World]:
        """
        List all registered worlds.

        :return: A list of all created World instances.
        """
        return list(cls._registry.values())

    @classmethod
    def unregister_world(cls, world_id: int) -> None:
        """
        Unregister a world from the factory.

        :param world_id: The unique id of the world to remove.
        """
        if world_id in cls._registry:
            # Optionally, call world.exit() here.
            del cls._registry[world_id]

    @classmethod
    def clear_registry(cls) -> None:
        """
        Clear all registered worlds and reset the internal counter.
        This method can be used for testing or to reset the factory.
        """
        for world in cls._registry.values():
            # Optionally, exit or clean up the world if necessary.
            world.exit(remove_saved_states=True)
        cls._registry.clear()
        cls._id_counter = 0