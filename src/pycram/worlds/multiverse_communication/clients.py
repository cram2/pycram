import datetime
import logging
import os
import threading
from threading import RLock

from time import time, sleep

from geometry_msgs.msg import Point
from typing_extensions import List, Dict, Tuple, Optional, Callable, Union

from .socket import MultiverseSocket, MultiverseMetaData
from ...config.multiverse_conf import MultiverseConfig as Conf
from ...datastructures.dataclasses import MultiverseRayResult, MultiverseContactPoint, \
    AxisAlignedBoundingBox
from ...datastructures.enums import (MultiverseAPIName as API, MultiverseBodyProperty as BodyProperty,
                                     MultiverseProperty as Property)
from ...datastructures.pose import PoseStamped
from ...failures import MultiverseFailedAPIResponse
from ...utils import wxyz_to_xyzw
from ...world_concepts.constraints import Constraint
from ...world_concepts.world_object import Object, Link


class MultiverseClient(MultiverseSocket):

    def __init__(self, name: str, port: int, is_prospection_world: bool = False,
                 simulation_wait_time_factor: float = 1.0, **kwargs):
        """
        Initialize the Multiverse client, which connects to the Multiverse server.

        :param name: The name of the client.
        :param port: The port of the client.
        :param is_prospection_world: Whether the client is connected to the prospection world.
        :param simulation_wait_time_factor: The simulation wait time factor (default is 1.0), which can be used to
            increase or decrease the wait time for the simulation.
        """
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = ((Conf.prospection_world_prefix if is_prospection_world else "belief_state")
                                     + "_" + name)
        meta_data.world_name = Conf.prospection_world_prefix if is_prospection_world else "belief_state"
        self.is_prospection_world = is_prospection_world
        super().__init__(port=str(port), meta_data=meta_data)
        self.simulation_wait_time_factor = simulation_wait_time_factor
        self.run()


class MultiverseReader(MultiverseClient):
    MAX_WAIT_TIME_FOR_DATA: datetime.timedelta = Conf.READER_MAX_WAIT_TIME_FOR_DATA
    """
    The maximum wait time for the data in seconds.
    """

    def __init__(self, name: str, port: int, is_prospection_world: bool = False,
                 simulation_wait_time_factor: float = 1.0, **kwargs):
        """
        Initialize the Multiverse reader, which reads the data from the Multiverse server in a separate thread.
        This class provides methods to get data (e.g., position, orientation) from the Multiverse server.

        :param port: The port of the Multiverse reader client.
        :param is_prospection_world: Whether the reader is connected to the prospection world.
        :param simulation_wait_time_factor: The simulation wait time factor.
        """
        super().__init__(name, port, is_prospection_world, simulation_wait_time_factor=simulation_wait_time_factor)

        self.request_meta_data["receive"][""] = [""]

        self.data_lock = threading.Lock()
        self.thread = threading.Thread(target=self.receive_all_data_from_server)
        self.stop_thread = False

        self.thread.start()

    def get_body_pose(self, name: str, wait: bool = False) -> Optional[Dict[str, List[float]]]:
        """
        Get the body pose from the multiverse server.

        :param name: The name of the body.
        :param wait: Whether to wait for the data.
        :return: The position and orientation of the body.
        """
        return self.get_body_data(name, [BodyProperty.POSITION, BodyProperty.ORIENTATION], wait=wait)

    def get_multiple_body_poses(self, body_names: List[str], wait: bool = False) -> Optional[Dict[str, PoseStamped]]:
        """
        Get the body poses from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param wait: Whether to wait for the data.
        :return: The positions and orientations of the bodies as a dictionary.
        """
        data = self.get_multiple_body_data(body_names,
                                           {name: [BodyProperty.POSITION, BodyProperty.ORIENTATION]
                                            for name in body_names
                                            },
                                           wait=wait)
        if data is not None:
            return {name: PoseStamped(data[name][BodyProperty.POSITION.value],
                                      wxyz_to_xyzw(data[name][BodyProperty.ORIENTATION.value]))
                    for name in body_names}

    def get_body_position(self, name: str, wait: bool = False) -> Optional[List[float]]:
        """
        Get the body position from the multiverse server.

        :param name: The name of the body.
        :param wait: Whether to wait for the data.
        :return: The position of the body.
        """
        return self.get_body_property(name, BodyProperty.POSITION, wait=wait)

    def get_multiple_body_positions(self, body_names: List[str],
                                    wait: bool = False) -> Optional[Dict[str, List[float]]]:
        """
        Get the body positions from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param wait: Whether to wait for the data.
        :return: The positions of the bodies as a dictionary.
        """
        return self.get_multiple_body_properties(body_names, [BodyProperty.POSITION], wait=wait)

    def get_body_orientation(self, name: str, wait: bool = False) -> Optional[List[float]]:
        """
        Get the body orientation from the multiverse server.

        :param name: The name of the body.
        :param wait: Whether to wait for the data.
        :return: The orientation of the body.
        """
        return self.get_body_property(name, BodyProperty.ORIENTATION, wait=wait)

    def get_multiple_body_orientations(self, body_names: List[str],
                                       wait: bool = False) -> Optional[Dict[str, List[float]]]:
        """
        Get the body orientations from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param wait: Whether to wait for the data.
        :return: The orientations of the bodies as a dictionary.
        """
        data = self.get_multiple_body_properties(body_names, [BodyProperty.ORIENTATION], wait=wait)
        if data is not None:
            return {name: wxyz_to_xyzw(data[name][BodyProperty.ORIENTATION.value]) for name in body_names}

    def get_body_property(self, name: str, property_: Property, wait: bool = False) -> Optional[List[float]]:
        """
        Get the body property from the multiverse server.

        :param name: The name of the body.
        :param property_: The property of the body as a Property.
        :param wait: Whether to wait for the data.
        :return: The property of the body.
        """
        data = self.get_body_data(name, [property_], wait=wait)
        if data is not None:
            return data[property_.value]

    def get_multiple_body_properties(self, body_names: List[str], properties: List[Property],
                                     wait: bool = False) -> Optional[Dict[str, Dict[str, List[float]]]]:
        """
        Get the body properties from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param properties: The properties of the bodies.
        :param wait: Whether to wait for the data.
        :return: The properties of the bodies as a dictionary.
        """
        return self.get_multiple_body_data(body_names, {name: properties for name in body_names}, wait=wait)

    def get_body_data(self, name: str,
                      properties: Optional[List[Property]] = None,
                      wait: bool = False) -> Optional[Dict]:
        """
        Get the body data from the multiverse server.

        :param name: The name of the body.
        :param properties: The properties of the body.
        :param wait: Whether to wait for the data.
        :return: The body data as a dictionary.
        """
        if wait:
            return self.wait_for_body_data(name, properties)

        data = self.get_received_data()
        if self.check_for_body_data(name, data, properties):
            return data[name]

    def get_multiple_body_data(self, body_names: List[str],
                               properties: Optional[Dict[str, List[Property]]] = None,
                               wait: bool = False) -> Optional[Dict]:
        """
        Get the body data from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param properties: The properties of the bodies.
        :param wait: Whether to wait for the data.
        :return: The body data as a dictionary.
        """

        if wait:
            return self.wait_for_multiple_body_data(body_names, properties)

        data = self.get_received_data()
        if self.check_multiple_body_data(body_names, data, properties):
            return {name: data[name] for name in body_names}

    def wait_for_body_data(self, name: str, properties: Optional[List[Property]] = None) -> Dict:
        """
        Wait for the body data from the multiverse server.

        :param name: The name of the body.
        :param properties: The properties of the body.
        :return: The body data as a dictionary.
        """
        return self._wait_for_body_data_template(name, self.check_for_body_data, properties)[name]

    def wait_for_multiple_body_data(self, body_names: List[str],
                                    properties: Optional[Dict[str, List[Property]]] = None) -> Dict:
        """
        Wait for the body data from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param properties: The properties of the bodies.
        :return: The body data as a dictionary.
        """
        return self._wait_for_body_data_template(body_names, self.check_multiple_body_data, properties)

    def _wait_for_body_data_template(self, body_names: Union[str, List[str]],
                                     check_func: Callable[[Union[str, List[str]], Dict, Union[Dict, List]], bool],
                                     properties: Optional[Union[Dict, List]] = None) -> Dict:
        """
        Wait for the body data from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param properties: The properties of the bodies.
        :param check_func: The function to check if the data is received.
        :return: The body data as a dictionary.
        """
        start = time()
        data_received_flag = False
        while time() - start < self.MAX_WAIT_TIME_FOR_DATA.total_seconds():
            received_data = self.get_received_data()
            data_received_flag = check_func(body_names, received_data, properties)
            if data_received_flag:
                return received_data
        if not data_received_flag:
            properties_str = "Data" if properties is None else f"Properties {properties}"
            msg = f"{properties_str} for {body_names} not received within {self.MAX_WAIT_TIME_FOR_DATA} seconds"
            logging.error(msg)
            raise ValueError(msg)

    def check_multiple_body_data(self, body_names: List[str], data: Dict,
                                 properties: Optional[Dict[str, List[Property]]] = None) -> bool:
        """
        Check if the body data is received from the multiverse server for multiple bodies.

        :param body_names: The names of the bodies.
        :param data: The data received from the multiverse server.
        :param properties: The properties of the bodies.
        :return: Whether the body data is received.
        """
        if properties is None:
            return all([self.check_for_body_data(name, data) for name in body_names])
        else:
            return all([self.check_for_body_data(name, data, properties[name]) for name in body_names])

    @staticmethod
    def check_for_body_data(name: str, data: Dict, properties: Optional[List[Property]] = None) -> bool:
        """
        Check if the body data is received from the multiverse server.

        :param name: The name of the body.
        :param data: The data received from the multiverse server.
        :param properties: The properties of the body.
        :return: Whether the body data is received.
        """
        if properties is None:
            return name in data
        else:
            return name in data and all([prop.value in data[name] and None not in data[name][prop.value]
                                         for prop in properties])

    def get_received_data(self):
        """
        Get the latest received data from the multiverse server.
        """
        self.data_lock.acquire()
        data = self.response_meta_data["receive"]
        self.data_lock.release()
        return data

    def receive_all_data_from_server(self):
        """
        Get all data from the multiverse server.
        """
        while not self.stop_thread:
            self.request_meta_data["receive"][""] = [""]
            self.data_lock.acquire()
            self.send_and_receive_meta_data()
            self.data_lock.release()
            sleep(0.01)
        self.stop()

    def join(self):
        self.thread.join()


class MultiverseWriter(MultiverseClient):

    def __init__(self, name: str, port: int, simulation: Optional[str] = None,
                 is_prospection_world: bool = False,
                 simulation_wait_time_factor: float = 1.0, **kwargs):
        """
        Initialize the Multiverse writer, which writes the data to the Multiverse server.
        This class provides methods to send data (e.g., position, orientation) to the Multiverse server.

        :param port: The port of the Multiverse writer client.
        :param simulation: The name of the simulation that the writer is connected to
         (usually the name defined in the .muv file).
        :param is_prospection_world: Whether the writer is connected to the prospection world.
        :param simulation_wait_time_factor: The wait time factor for the simulation (default is 1.0), which can be used
         to increase or decrease the wait time for the simulation.
        """
        super().__init__(name, port, is_prospection_world, simulation_wait_time_factor=simulation_wait_time_factor)
        self.simulation = simulation
        self.lock = RLock()

    def spawn_robot_with_actuators(self, robot_name: str,
                                   actuator_joint_commands: Optional[Dict[str, List[str]]] = None) -> None:
        """
        Spawn the robot with controlled actuators in the simulation.

        :param robot_name: The name of the robot.
        :param actuator_joint_commands: A dictionary mapping actuator names to joint command names.
        """
        send_meta_data = {robot_name: [BodyProperty.POSITION.value, BodyProperty.ORIENTATION.value,
                                       BodyProperty.RELATIVE_VELOCITY.value]}
        relative_velocity = [0.0] * 6
        data = [self.sim_time] + [0.0] * 3 + [1, 0, 0, 0] + relative_velocity
        self.send_data_to_server(data, send_meta_data=send_meta_data, receive_meta_data=actuator_joint_commands)

    def _reset_request_meta_data(self, set_simulation_name: bool = True):
        """
        Reset the request metadata.

        :param set_simulation_name: Whether to set the simulation name to the value of self.simulation_name.
        """
        self.request_meta_data = {
            "meta_data": self._meta_data.__dict__.copy(),
            "send": {},
            "receive": {},
        }
        if self.simulation is not None and set_simulation_name:
            self.request_meta_data["meta_data"]["simulation_name"] = self.simulation

    def set_body_pose(self, body_name: str, position: List[float], orientation: List[float]) -> None:
        """
        Set the body pose in the simulation.

        :param body_name: The name of the body.
        :param position: The position of the body.
        :param orientation: The orientation of the body.
        """
        self.send_body_data_to_server(body_name,
                                      {BodyProperty.POSITION: position,
                                       BodyProperty.ORIENTATION: orientation,
                                       BodyProperty.RELATIVE_VELOCITY: [0.0] * 6})

    def set_multiple_body_poses(self, body_data: Dict[str, Dict[BodyProperty, List[float]]]) -> None:
        """
        Set the body poses in the simulation for multiple bodies.

        :param body_data: The data to be sent for multiple bodies.
        """
        self.send_multiple_body_data_to_server(body_data)

    def set_body_position(self, body_name: str, position: List[float]) -> None:
        """
        Set the body position in the simulation.

        :param body_name: The name of the body.
        :param position: The position of the body.
        """
        self.set_body_property(body_name, BodyProperty.POSITION, position)

    def set_body_orientation(self, body_name: str, orientation: List[float]) -> None:
        """
        Set the body orientation in the simulation.

        :param body_name: The name of the body.
        :param orientation: The orientation of the body.
        """
        self.set_body_property(body_name, BodyProperty.ORIENTATION, orientation)

    def set_body_property(self, body_name: str, property_: Property, value: List[float]) -> None:
        """
        Set the body property in the simulation.

        :param body_name: The name of the body.
        :param property_: The property of the body.
        :param value: The value of the property.
        """
        self.send_body_data_to_server(body_name, {property_: value})

    def remove_body(self, body_name: str) -> None:
        """
        Remove the body from the simulation.

        :param body_name: The name of the body.
        """
        self.send_data_to_server([self.sim_time],
                                 send_meta_data={body_name: []},
                                 receive_meta_data={body_name: []})

    def reset_world(self) -> None:
        """
        Reset the world in the simulation.
        """
        self.send_data_to_server([0], set_simulation_name=False)

    def send_body_data_to_server(self, body_name: str, body_data: Dict[Property, List[float]]) -> Dict:
        """
        Send data to the multiverse server.

        :param body_name: The name of the body.
        :param body_data: The data to be sent.
        :return: The response from the server.
        """
        send_meta_data = {body_name: list(map(str, body_data.keys()))}
        flattened_data = [value for data in body_data.values() for value in data]
        return self.send_data_to_server([self.sim_time, *flattened_data], send_meta_data=send_meta_data)

    def send_multiple_body_data_to_server(self, body_data: Dict[str, Dict[Property, List[float]]]) -> Dict:
        """
        Send data to the multiverse server for multiple bodies.

        :param body_data: The data to be sent for multiple bodies.
        :return: The response from the server.
        """
        send_meta_data = {body_name: list(map(str, data.keys())) for body_name, data in body_data.items()}
        response_meta_data = self.send_meta_data_and_get_response(send_meta_data)
        body_names = list(response_meta_data["send"].keys())
        flattened_data = [value for body_name in body_names for data in body_data[body_name].values()
                          for value in data]
        self.lock.acquire()
        self.send_data = [self.sim_time, *flattened_data]
        self.send_and_receive_data()
        response_meta_data = self.response_meta_data
        self.lock.release()
        return response_meta_data

    def send_meta_data_and_get_response(self, send_meta_data: Dict) -> Dict:
        """
        Send metadata to the multiverse server and get the response.

        :param send_meta_data: The metadata to be sent.
        :return: The response from the server.
        """
        self.lock.acquire()
        self._reset_request_meta_data()
        self.request_meta_data["send"] = send_meta_data
        self.send_and_receive_meta_data()
        response_meta_data = self.response_meta_data
        self.lock.release()
        return response_meta_data

    def send_data_to_server(self, data: List,
                            send_meta_data: Optional[Dict] = None,
                            receive_meta_data: Optional[Dict] = None,
                            set_simulation_name: bool = True) -> Dict:
        """
        Send data to the multiverse server.

        :param data: The data to be sent.
        :param send_meta_data: The metadata to be sent.
        :param receive_meta_data: The metadata to be received.
        :param set_simulation_name: Whether to set the simulation name to the value of self.simulation.
        :return: The response from the server.
        """
        self.lock.acquire()
        self._reset_request_meta_data(set_simulation_name=set_simulation_name)
        if send_meta_data:
            self.request_meta_data["send"] = send_meta_data
        if receive_meta_data:
            self.request_meta_data["receive"] = receive_meta_data
        self.send_and_receive_meta_data()
        self.send_data = data
        self.send_and_receive_data()
        response_meta_data = self.response_meta_data
        self.lock.release()
        return response_meta_data


class MultiverseController(MultiverseWriter):

    def __init__(self, name: str, port: int, is_prospection_world: bool = False, **kwargs):
        """
        Initialize the Multiverse controller, which controls the robot in the simulation.
        This class provides methods to send controller data to the Multiverse server.

        :param port: The port of the Multiverse controller client.
        :param is_prospection_world: Whether the controller is connected to the prospection world.
        """
        super().__init__(name, port, is_prospection_world=is_prospection_world)

    def init_controller(self, actuator_joint_commands: Dict[str, List[str]]) -> None:
        """
        Initialize the controller by sending the controller data to the multiverse server.

        :param actuator_joint_commands: A dictionary mapping actuator names to joint command names.
        """
        self.send_data_to_server([self.sim_time] + [0.0] * len(actuator_joint_commands),
                                 send_meta_data=actuator_joint_commands)


class MultiverseAPI(MultiverseClient):
    API_REQUEST_WAIT_TIME: datetime.timedelta = datetime.timedelta(milliseconds=200)
    """
    The wait time for the API request in seconds.
    """
    APIs_THAT_NEED_WAIT_TIME: List[API] = [API.ATTACH, API.DETACH]

    def __init__(self, name: str, port: int, simulation: str, is_prospection_world: bool = False,
                 simulation_wait_time_factor: float = 1.0):
        """
        Initialize the Multiverse API, which sends API requests to the Multiverse server.
        This class provides methods like attach and detach objects, get contact points, and other API requests.

        :param port: The port of the Multiverse API client.
        :param simulation: The name of the simulation that the API is connected to
         (usually the name defined in the .muv file).
        :param is_prospection_world: Whether the API is connected to the prospection world.
        :param simulation_wait_time_factor: The simulation wait time factor, which can be used to increase or decrease
            the wait time for the simulation.
        """
        super().__init__(name, port, is_prospection_world, simulation_wait_time_factor=simulation_wait_time_factor)
        self.simulation = simulation
        self.wait: bool = False  # Whether to wait after sending the API request.

    def get_body_bounding_box(self, body_name: str,
                              with_children: bool = False) -> Union[AxisAlignedBoundingBox, List[AxisAlignedBoundingBox]]:
        """
        Get the body bounding box from the multiverse server, they are with respect to the body's frame.
        """
        bounding_boxes_data = self._get_bounding_box(body_name, with_children)
        bounding_boxes = []
        for bounding_box in bounding_boxes_data:
            origin = Point(x=bounding_box[0], y=bounding_box[1], z=bounding_box[2])
            size = Point(x=bounding_box[3], y=bounding_box[4], z=bounding_box[5])
            bounding_boxes.append(AxisAlignedBoundingBox.from_origin_and_half_extents(origin, size))
        if with_children:
            return bounding_boxes
        return bounding_boxes[0]

    def _get_bounding_box(self, body_name: str, with_children: bool = False) -> List[List[float]]:
        """
        Get the body bounding box from the multiverse server.
        """
        params = [body_name]
        if with_children:
            params.append("with_children")
        response = self._request_single_api_callback(API.GET_BOUNDING_BOX, *params)[0]
        return [list(map(float, bounding_box.split())) for bounding_box in response]

    def save(self, save_name: str, save_directory: Optional[str] = None) -> str:
        """
        Save the current state of the simulation.

        :param save_name: The name of the save.
        :param save_directory: The path to save the simulation, can be relative or absolute. If the path is relative,
         it will be saved in the saved folder in multiverse.
        :return: The save path.
        """
        response = self._request_single_api_callback(API.SAVE, self.get_save_path(save_name, save_directory))
        return response[0]

    def load(self, save_name: str, save_directory: Optional[str] = None) -> None:
        """
        Load the saved state of the simulation.

        :param save_name: The name of the save.
        :param save_directory: The path to load the simulation, can be relative or absolute. If the path is relative,
         it will be loaded from the saved folder in multiverse.
        """
        self._request_single_api_callback(API.LOAD, self.get_save_path(save_name, save_directory))

    @staticmethod
    def get_save_path(save_name: str, save_directory: Optional[str] = None) -> str:
        """
        Get the save path.

        :param save_name: The save name.
        :param save_directory: The save directory.
        :return: The save path.
        """
        return save_name if not save_directory else os.path.join(save_directory, save_name)

    def attach(self, constraint: Constraint) -> None:
        """
        Request to attach the child link to the parent link.

        :param constraint: The constraint.
        """
        self.wait = True
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        attachment_pose = self._get_attachment_pose_as_string(constraint)
        self._attach(child_link_name, parent_link_name, attachment_pose)

    def _attach(self, child_link_name: str, parent_link_name: str, attachment_pose: str) -> None:
        """
        Attach the child link to the parent link.

        :param child_link_name: The name of the child link.
        :param parent_link_name: The name of the parent link.
        :param attachment_pose: The attachment pose.
        """
        self._request_single_api_callback(API.ATTACH, child_link_name, parent_link_name,
                                          attachment_pose)

    def get_constraint_link_names(self, constraint: Constraint) -> Tuple[str, str]:
        """
        Get the link names of the constraint.

        :param constraint: The constraint.
        :return: The link names of the constraint.
        """
        return self.get_parent_link_name(constraint), self.get_constraint_child_link_name(constraint)

    def get_parent_link_name(self, constraint: Constraint) -> str:
        """
        Get the parent link name of the constraint.

        :param constraint: The constraint.
        :return: The parent link name of the constraint.
        """
        return self.get_link_name_for_constraint(constraint.parent_link)

    def get_constraint_child_link_name(self, constraint: Constraint) -> str:
        """
        Get the child link name of the constraint.

        :param constraint: The constraint.
        :return: The child link name of the constraint.
        """
        return self.get_link_name_for_constraint(constraint.child_link)

    @staticmethod
    def get_link_name_for_constraint(link: Link) -> str:
        """
        Get the link name from link object, if the link belongs to a one link object, return the object name.

        :param link: The link.
        :return: The link name.
        """
        return link.name if not link.is_only_link else link.object.name

    def detach(self, constraint: Constraint) -> None:
        """
        Request to detach the child link from the parent link.

        :param constraint: The constraint.
        """
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        self._detach(child_link_name, parent_link_name)

    def _detach(self, child_link_name: str, parent_link_name: str) -> None:
        """
        Detach the child link from the parent link.

        :param child_link_name: The name of the child link.
        :param parent_link_name: The name of the parent link.
        """
        self._request_single_api_callback(API.DETACH, child_link_name, parent_link_name)

    def _get_attachment_pose_as_string(self, constraint: Constraint) -> str:
        """
        Get the attachment pose as a string.

        :param constraint: The constraint.
        :return: The attachment pose as a string.
        """
        pose = constraint.parent_to_child_transform.to_pose()
        return self._pose_to_string(pose)

    @staticmethod
    def _pose_to_string(pose: PoseStamped) -> str:
        """
        Convert the pose to a string.

        :param pose: The pose.
        :return: The pose as a string.
        """
        return f"{pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.w} {pose.orientation.x} " \
               f"{pose.orientation.y} {pose.orientation.z}"

    def check_object_exists(self, obj: Object) -> bool:
        """
        Check if the object exists in the simulation.

        :param obj: The object.
        :return: Whether the object exists in the simulation.
        """
        return self._request_single_api_callback(API.EXIST, obj.name)[0] == 'yes'

    def get_resultant_force_and_torque_on_object(self, obj: Object) -> Tuple[List[float], List[float]]:
        """
        Get the resultant force and torque on the object.

        :param obj: The object.
        :return: The resultant force and torque on the object.
        """
        effort = self._request_single_api_callback(API.GET_CONSTRAINT_EFFORT, obj.name)
        return self._parse_constraint_effort(effort)

    def get_contact_points_between_bodies(self, body_1_name: str, body_2_name: str) -> List[MultiverseContactPoint]:
        """
        Request the contact points between two bodies.

        :param body_1_name: The name of the first body.
        :param body_2_name: The name of the second body.
        :return: The contact points between the bodies as a list of MultiverseContactPoint.
        """
        points = self._request_single_api_callback(API.GET_CONTACT_POINTS, body_1_name, body_2_name)
        return self._parse_contact_points(body_1_name, body_2_name, points)

    def get_objects_intersected_with_rays(self, from_positions: List[List[float]],
                                          to_positions: List[List[float]]) -> List[MultiverseRayResult]:
        """
        Get the rays intersections with the objects from the from_positions to the to_positions.

        :param from_positions: The starting positions of the rays.
        :param to_positions: The ending positions of the rays.
        :return: The rays intersections with the objects as a list of MultiverseRayResult.
        """
        get_rays_response = self._get_rays(from_positions, to_positions)
        return self._parse_get_rays_response(get_rays_response)

    def _get_rays(self, from_positions: List[List[float]],
                  to_positions: List[List[float]]) -> List[str]:
        """
        Get the rays intersections with the objects from the from_positions to the to_positions.

        :param from_positions: The starting positions of the rays.
        :param to_positions: The ending positions of the rays.
        :return: The rays intersections with the objects as a dictionary.
        """
        from_positions = self.list_of_positions_to_string(from_positions)
        to_positions = self.list_of_positions_to_string(to_positions)
        return self._request_single_api_callback(API.GET_RAYS, from_positions, to_positions)

    @staticmethod
    def _parse_get_rays_response(response: List[str]) -> List[MultiverseRayResult]:
        """
        Parse the response of the get rays API.

        :param response: The response of the get rays API as a list of strings.
        :return: The rays as a list of lists of floats.
        """
        get_rays_results = []
        for ray_response in response:
            if ray_response == "None":
                get_rays_results.append(MultiverseRayResult("", -1))
            else:
                result = ray_response.split()
                result[1] = float(result[1])
                get_rays_results.append(MultiverseRayResult(*result))
        return get_rays_results

    @staticmethod
    def list_of_positions_to_string(positions: List[List[float]]) -> str:
        """
        Convert the list of positions to a string.

        :param positions: The list of positions.
        :return: The list of positions as a string.
        """
        return " ".join([f"{position[0]} {position[1]} {position[2]}" for position in positions])

    @staticmethod
    def _parse_constraint_effort(contact_effort: List[str]) -> Tuple[List[float], List[float]]:
        """
        Parse the contact effort of an object.

        :param contact_effort: The contact effort of the object as a list of strings.
        :return: The contact effort of the object as a list of floats.
        """
        contact_effort = contact_effort[0].split()
        contact_effort = list(map(float, contact_effort))
        forces, torques = contact_effort[:3], contact_effort[3:]
        return forces, torques

    @staticmethod
    def _parse_contact_points(body_1, body_2, contact_points: List[str]) -> List[MultiverseContactPoint]:
        """
        Parse the contact points of an object.

        :param body_1: The name of the first body.
        :param body_2: The name of the second body.
        :param contact_points: The contact points of the object as a list of strings.
        :return: The contact positions, and normal vectors as a list of MultiverseContactPoint.
        """
        contact_point_data = [list(map(float, contact_point.split())) for contact_point in contact_points]
        return [MultiverseContactPoint(body_1, body_2, point[:3], point[3:]) for point in contact_point_data]

    def get_contact_points(self, body_name: str) -> List[MultiverseContactPoint]:
        """
        Get the contact points of a body from the multiverse server.

        :param body_name: The name of the body.
        """
        contact_data = self._request_single_api_callback(API.GET_CONTACT_BODIES_AND_POINTS,
                                                         body_name, "with_children")
        contact_points = []
        for contact_point in contact_data:
            contact_point_data = list(contact_point.split())
            position_and_normal = list(map(float, contact_point_data[2:]))
            for i in range(0, len(position_and_normal), 6):
                contact_points.append(MultiverseContactPoint(contact_point_data[0],
                                                             contact_point_data[1],
                                                             position_and_normal[i:i + 3],
                                                             position_and_normal[i + 3:i + 6]))
        return contact_points

    def pause_simulation(self) -> None:
        """
        Pause the simulation.
        """
        self._request_single_api_callback(API.PAUSE)

    def unpause_simulation(self) -> None:
        """
        Unpause the simulation.
        """
        self._request_single_api_callback(API.UNPAUSE)

    def _request_single_api_callback(self, api_name: API, *params) -> List[str]:
        """
        Request a single API callback from the server.

        :param api_data: The API data to request the callback.
        :return: The API response as a list of strings.
        """
        response = self._request_apis_callbacks({api_name: list(params)})
        return response[api_name]

    def _request_apis_callbacks(self, api_data: Dict[API, List]) -> Dict[API, List[str]]:
        """
        Request the API callbacks from the server.

        :param api_data: The API data to add to the request metadata.
        :return: The API response as a list of strings.
        """
        self._reset_api_callback()
        for api_name, params in api_data.items():
            self._add_api_request(api_name.value, *params)
        self._send_api_request()
        if self.wait:
            sleep(self.API_REQUEST_WAIT_TIME.total_seconds() * self.simulation_wait_time_factor)
            self.wait = False
        responses = self._get_all_apis_responses()
        self.validate_apis_response(api_data, responses)
        return responses

    @staticmethod
    def validate_apis_response(api_data: Dict[API, List], responses: Dict[API, List[str]]):
        """
        Validate the responses from the multiverse server and raise error if an api request failed.

        :param api_data: The data of the api request which has the api name and the arguments.
        :param responses: The responses of the given api requests.
        :raises FailedAPIResponse: when one of the responses reports that the request failed.
        """
        for api_name, response in responses.items():
            for val in response:
                if 'failed' in val:
                    raise MultiverseFailedAPIResponse(response, api_name, api_data[api_name])

    def _get_all_apis_responses(self) -> Dict[API, List[str]]:
        """
        Get all the API responses from the server.

        :return: The API responses as a list of APIData.
        """
        list_of_api_responses = self.response_meta_data["api_callbacks_response"][self.simulation]
        return {API[api_name.upper()]: response for api_response in list_of_api_responses
                for api_name, response in api_response.items()}

    def _add_api_request(self, api_name: str, *params):
        """
        Add an API request to the request metadata.

        :param api_name: The name of the API.
        :param params: The parameters of the API.
        """
        self.request_meta_data["api_callbacks"][self.simulation].append({api_name: list(params)})

    def _send_api_request(self):
        """
        Send the API request to the server.
        """
        if "api_callbacks" not in self.request_meta_data:
            logging.error("No API request to send")
            raise ValueError
        self.send_and_receive_meta_data()
        self.request_meta_data.pop("api_callbacks")

    def _reset_api_callback(self):
        """
        Initialize the API callback in the request metadata.
        """
        self.request_meta_data["api_callbacks"] = {self.simulation: []}
