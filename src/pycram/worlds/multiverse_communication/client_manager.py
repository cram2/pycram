from typing_extensions import Optional, Type, Union, Dict

from ...worlds.multiverse_communication.clients import MultiverseWriter, MultiverseAPI, MultiverseClient, \
    MultiverseReader, MultiverseController

from ...config.multiverse_conf import MultiverseConfig as Conf


class MultiverseClientManager:
    BASE_PORT: int = Conf.BASE_CLIENT_PORT
    """
    The base port of the Multiverse client.
    """
    clients: Optional[Dict[str, MultiverseClient]] = {}
    """
    The list of Multiverse clients.
    """
    last_used_port: int = BASE_PORT

    def __init__(self, simulation_wait_time_factor: Optional[float] = 1.0):
        """
        Initialize the Multiverse client manager.

        :param simulation_wait_time_factor: The simulation wait time factor.
        """
        self.simulation_wait_time_factor = simulation_wait_time_factor

    def create_reader(self, is_prospection_world: Optional[bool] = False) -> MultiverseReader:
        """
        Create a Multiverse reader client.

        :param is_prospection_world: Whether the reader is connected to the prospection world.
        """
        return self.create_client(MultiverseReader, "reader", is_prospection_world)

    def create_writer(self, simulation: str, is_prospection_world: Optional[bool] = False) -> MultiverseWriter:
        """
        Create a Multiverse writer client.

        :param simulation: The name of the simulation that the writer is connected to
         (usually the name defined in the .muv file).
        :param is_prospection_world: Whether the writer is connected to the prospection world.
        """
        return self.create_client(MultiverseWriter, "writer", is_prospection_world,
                                  simulation=simulation)

    def create_controller(self, is_prospection_world: Optional[bool] = False) -> MultiverseController:
        """
        Create a Multiverse controller client.

        :param is_prospection_world: Whether the controller is connected to the prospection world.
        """
        return self.create_client(MultiverseController, "controller", is_prospection_world)

    def create_api_requester(self, simulation: str, is_prospection_world: Optional[bool] = False) -> MultiverseAPI:
        """
        Create a Multiverse API client.

        :param simulation: The name of the simulation that the API is connected to
         (usually the name defined in the .muv file).
        :param is_prospection_world: Whether the API is connected to the prospection world.
        """
        return self.create_client(MultiverseAPI, "api_requester", is_prospection_world, simulation=simulation)

    def create_client(self,
                      client_type: Type[MultiverseClient],
                      name: Optional[str] = None,
                      is_prospection_world: Optional[bool] = False,
                      **kwargs) -> Union[MultiverseClient, MultiverseAPI,
                                         MultiverseReader, MultiverseWriter, MultiverseController]:
        """
        Create a Multiverse client.

        :param client_type: The type of the client to create.
        :param name: The name of the client.
        :param is_prospection_world: Whether the client is connected to the prospection world.
        :param kwargs: Any other keyword arguments that should be passed to the client constructor.
        """
        MultiverseClientManager.last_used_port += 1
        name = (name or client_type.__name__) + f"_{self.last_used_port}"
        client = client_type(name, self.last_used_port, is_prospection_world=is_prospection_world,
                             simulation_wait_time_factor=self.simulation_wait_time_factor, **kwargs)
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
