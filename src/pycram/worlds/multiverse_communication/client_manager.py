from typing_extensions import Optional, Type, Union, List, Dict

from pycram.worlds.multiverse_communication.clients import MultiverseWriter, MultiverseAPI, MultiverseClient, \
    MultiverseReader


class MultiverseClientManager:
    BASE_PORT: int = 9000
    """
    The base port of the Multiverse client.
    """
    clients: Optional[Dict[str, MultiverseClient]] = {}
    """
    The list of Multiverse clients.
    """
    last_used_port: int = BASE_PORT

    def create_reader(self, is_prospection_world: Optional[bool] = False) -> 'MultiverseReader':
        """
        Create a Multiverse reader client.
        param max_wait_time_for_data: The maximum wait time for the data in seconds.
        param is_prospection_world: Whether the reader is connected to the prospection world.
        """
        return self.create_client(MultiverseReader, "reader", is_prospection_world)

    def create_writer(self, simulation: str, is_prospection_world: Optional[bool] = False) -> MultiverseWriter:
        """
        Create a Multiverse writer client.
        param simulation: The name of the simulation that the writer is connected to
         (usually the name defined in the .muv file).
        param is_prospection_world: Whether the writer is connected to the prospection world.
        """
        return self.create_client(MultiverseWriter, "writer", is_prospection_world,
                                  simulation=simulation)

    def create_api_requester(self, simulation: str, is_prospection_world: Optional[bool] = False) -> MultiverseAPI:
        """
        Create a Multiverse API client.
        param simulation: The name of the simulation that the API is connected to
         (usually the name defined in the .muv file).
        param is_prospection_world: Whether the API is connected to the prospection world.
        """
        return self.create_client(MultiverseAPI, "api_requester", is_prospection_world, simulation=simulation)

    def create_client(self,
                      client_type: Type[MultiverseClient],
                      name: Optional[str] = None,
                      is_prospection_world: Optional[bool] = False,
                      **kwargs) -> Union[MultiverseClient, MultiverseAPI,
                                         MultiverseReader, MultiverseWriter]:
        """
        Create a Multiverse client.
        param kwargs: The keyword arguments.
        """
        MultiverseClientManager.last_used_port += 1
        name = (name or client_type.__name__) + f"_{self.last_used_port}"
        client = client_type(name, self.last_used_port, is_prospection_world=is_prospection_world, **kwargs)
        self.clients[name] = client
        return client

    @classmethod
    def stop_all_clients(cls):
        """
        Stop all clients.
        """
        for client in cls.clients:
            if isinstance(client, MultiverseReader):
                client.stop_thread = True
                client.join()
            elif isinstance(client, MultiverseClient):
                client.stop()
        cls.clients = {}
