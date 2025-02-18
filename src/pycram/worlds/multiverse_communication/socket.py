#!/usr/bin/env python3

"""Multiverse Client base class."""

from multiverse_client_pybind import MultiverseClientPybind  # noqa
from typing_extensions import Optional, List, Dict, Callable, TypeVar

from ...datastructures.dataclasses import MultiverseMetaData
from ...config.multiverse_conf import MultiverseConfig as Conf
from ...ros import loginfo, logwarn

T = TypeVar("T")


class MultiverseSocket:

    def __init__(
            self,
            port: str,
            host: str = Conf.HOST,
            meta_data: MultiverseMetaData = MultiverseMetaData(),
    ) -> None:
        """
        Initialize the MultiverseSocket, connect to the Multiverse Server and start the communication.

        :param port: The port of the client.
        :param host: The host of the client.
        :param meta_data: The metadata for the Multiverse Client as MultiverseMetaData.
        """
        if not isinstance(port, str) or port == "":
            raise ValueError(f"Must specify client port for {self.__class__.__name__}")
        self._send_data = None
        self.port = port
        self.host = host
        self._meta_data = meta_data
        self.client_name = self._meta_data.simulation_name
        self._multiverse_socket = MultiverseClientPybind(
            f"{Conf.SERVER_HOST}:{Conf.SERVER_PORT}"
        )
        self.request_meta_data = {
            "meta_data": self._meta_data.__dict__,
            "send": {},
            "receive": {},
        }
        self._api_callbacks: Optional[Dict] = None

        self._start_time = 0.0

    def run(self) -> None:
        """Run the client."""
        self.log_info("Start")
        self._run()

    def _run(self) -> None:
        """Run the client, should call the _connect_and_start() method. It's left to the user to implement this method
        in threaded or non-threaded fashion.
        """
        self._connect_and_start()

    def stop(self) -> None:
        """Stop the client."""
        self._disconnect()

    @property
    def request_meta_data(self) -> Dict:
        """The request_meta_data which is sent to the server.
        """
        return self._request_meta_data

    @request_meta_data.setter
    def request_meta_data(self, request_meta_data: Dict) -> None:
        """Set the request_meta_data, make sure to clear the `send` and `receive` field before setting the request
        """
        self._request_meta_data = request_meta_data
        self._multiverse_socket.set_request_meta_data(self._request_meta_data)

    @property
    def response_meta_data(self) -> Dict:
        """Get the response_meta_data.

        :return: The response_meta_data as a dictionary.
        """
        response_meta_data = self._multiverse_socket.get_response_meta_data()
        assert isinstance(response_meta_data, dict)
        if response_meta_data == {}:
            message = f"[Client {self.port}] Receive empty response meta data."
            self.log_warn(message)
        return response_meta_data

    def send_and_receive_meta_data(self):
        """
        Send and receive the metadata, this should be called before sending and receiving data.
        """
        self._communicate(True)

    def send_and_receive_data(self):
        """
        Send and receive the data, this should be called after sending and receiving the metadata.
        """
        self._communicate(False)

    @property
    def send_data(self) -> List[float]:
        """Get the send_data."""
        return self._send_data

    @send_data.setter
    def send_data(self, send_data: List[float]) -> None:
        """Set the send_data, the first element should be the current simulation time,
        the rest should be the data to send with the following order:
        double -> uint8_t -> uint16_t

        :param send_data: The data to send.
        """
        assert isinstance(send_data, list)
        self._send_data = send_data
        self._multiverse_socket.set_send_data(self._send_data)

    @property
    def receive_data(self) -> List[float]:
        """Get the receive_data, the first element should be the current simulation time,
        the rest should be the received data with the following order:
        double -> uint8_t -> uint16_t

        :return: The received data.
        """
        receive_data = self._multiverse_socket.get_receive_data()
        assert isinstance(receive_data, list)
        return receive_data

    @property
    def api_callbacks(self) -> Dict[str, Callable[[List[str]], List[str]]]:
        """Get the api_callbacks.

        :return: The api_callbacks as a dictionary of function names and their respective callbacks.
        """
        return self._api_callbacks

    @api_callbacks.setter
    def api_callbacks(self, api_callbacks: Dict[str, Callable[[List[str]], List[str]]]) -> None:
        """Set the api_callbacks.

        :param api_callbacks: The api_callbacks as a dictionary of function names and their respective callbacks.
        """
        self._multiverse_socket.set_api_callbacks(api_callbacks)
        self._api_callbacks = api_callbacks

    def _bind_request_meta_data(self, request_meta_data: T) -> T:
        """Bind the request_meta_data before sending it to the server.

        :param request_meta_data: The request_meta_data to bind.
        :return: The bound request_meta_data.
        """
        pass

    def _bind_response_meta_data(self, response_meta_data: T) -> T:
        """Bind the response_meta_data after receiving it from the server.

        :param response_meta_data: The response_meta_data to bind.
        :return: The bound response_meta_data.
        """
        pass

    def _bind_send_data(self, send_data: T) -> T:
        """Bind the send_data before sending it to the server.

        :param send_data: The send_data to bind.
        :return: The bound send_data.
        """
        pass

    def _bind_receive_data(self, receive_data: T) -> T:
        """Bind the receive_data after receiving it from the server.

        :param receive_data: The receive_data to bind.
        :return: The bound receive_data.
        """
        pass

    def _connect_and_start(self) -> None:
        """Connect to the server and start the client.
        """
        self._multiverse_socket.connect(self.host, self.port)
        self._multiverse_socket.start()
        self._start_time = self._multiverse_socket.get_time_now()

    def _disconnect(self) -> None:
        """Disconnect from the server.
        """
        self._multiverse_socket.disconnect()

    def _communicate(self, resend_request_meta_data: bool = False) -> bool:
        """Communicate with the server.

        :param resend_request_meta_data: Resend the request metadata.
        :return: True if the communication was successful, False otherwise.
        """
        return self._multiverse_socket.communicate(resend_request_meta_data)

    def _restart(self) -> None:
        """Restart the client.
        """
        self._disconnect()
        self._connect_and_start()

    def log_info(self, message: str) -> None:
        """Log information.

        :param message: The message to log.
        """
        loginfo(self._message_template(message))

    def log_warn(self, message: str) -> None:
        """Warn the user.

        :param message: The message to warn about.
        """
        logwarn(self._message_template(message))

    def _message_template(self, message: str) -> str:
        return (f"[{self.__class__.__name__}:{self.port}]: {message} : sim time {self.sim_time},"
                f" world time {self.world_time}")

    @property
    def world_time(self) -> float:
        """Get the world time from the server.

        :return: The world time.
        """
        return self._multiverse_socket.get_world_time()

    @property
    def sim_time(self) -> float:
        """Get the current simulation time.

        :return: The current simulation time.
        """
        return self._multiverse_socket.get_time_now() - self._start_time
