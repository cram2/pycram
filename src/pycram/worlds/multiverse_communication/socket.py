#!/usr/bin/env python3

import dataclasses
from typing import List, Dict, TypeVar
import logging

from multiverse_client_pybind import MultiverseClientPybind  # noqa
from ...config import multiverse_conf as conf

T = TypeVar("T")


@dataclasses.dataclass
class MultiverseMetaData:
    world_name: str = "world"
    simulation_name: str = "cram"
    length_unit: str = "m"
    angle_unit: str = "rad"
    mass_unit: str = "kg"
    time_unit: str = "s"
    handedness: str = "rhs"


class MultiverseSocket:

    def __init__(
            self,
            port: str,
            host: str = conf.HOST,
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
            f"{conf.SERVER_HOST}:{conf.SERVER_PORT}"
        )
        self.request_meta_data = {
            "meta_data": self._meta_data.__dict__,
            "send": {},
            "receive": {},
        }

    def run(self) -> None:
        message = f"[Client {self.port}] Start {self.__class__.__name__}{self.port}"
        logging.info(message)
        self._connect_and_start()

    def stop(self) -> None:
        self._disconnect()

    def send_and_receive_meta_data(self):
        self._communicate(True)

    def send_and_receive_data(self):
        self._communicate(False)

    @property
    def request_meta_data(self) -> Dict:
        return self._request_meta_data

    @request_meta_data.setter
    def request_meta_data(self, request_meta_data: Dict) -> None:
        self._request_meta_data = request_meta_data
        self._multiverse_socket.set_request_meta_data(self._request_meta_data)

    @property
    def response_meta_data(self) -> Dict:
        response_meta_data = self._multiverse_socket.get_response_meta_data()
        if not response_meta_data:
            message = f"[Client {self.port}] Receive empty response meta data."
            logging.warning(message)
        return response_meta_data

    @property
    def send_data(self) -> List[float]:
        return self._send_data

    @send_data.setter
    def send_data(self, send_data: List[float]) -> None:
        self._send_data = send_data
        self._multiverse_socket.set_send_data(self._send_data)

    @property
    def receive_data(self) -> List[float]:
        receive_data = self._multiverse_socket.get_receive_data()
        if not receive_data:
            message = f"[Client {self.port}] Receive empty data."
            logging.warning(message)
        return receive_data

    def _connect_and_start(self) -> None:
        self._multiverse_socket.connect(self.host, self.port)
        self._multiverse_socket.start()

    def _disconnect(self) -> None:
        self._multiverse_socket.disconnect()

    def _communicate(self, resend_request_meta_data: bool = False) -> None:
        self._multiverse_socket.communicate(resend_request_meta_data)

    def _restart(self) -> None:
        self._disconnect()
        self._connect_and_start()
