import logging
import threading
from time import time, sleep

from typing_extensions import List, Dict, Tuple, Optional

from .socket import MultiverseSocket, MultiverseMetaData, SocketAddress
from ..multiverse_datastructures.enums import MultiverseAPIName as API
from ..multiverse_datastructures.dataclasses import RayResult, MultiverseContactPoint
from ...datastructures.pose import Pose
from ...datastructures.world import World
from ...world_concepts.constraints import Constraint
from ...world_concepts.world_object import Object, Link


class MultiverseClient(MultiverseSocket):

    def __init__(self, name: str, port: int, is_prospection_world: Optional[bool] = False,
                 simulation_wait_time_factor: Optional[float] = 1.0, **kwargs):
        """
        Initialize the Multiverse client, which connects to the Multiverse server.
        param name: The name of the client.
        param port: The port of the client.
        param is_prospection_world: Whether the client is connected to the prospection world.
        param simulation_wait_time_factor: The simulation wait time factor (default is 1.0), which can be used to
            increase or decrease the wait time for the simulation.
        """
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = (World.prospection_world_prefix if is_prospection_world else "") + name
        meta_data.world_name = (World.prospection_world_prefix if is_prospection_world else "") + meta_data.world_name
        super().__init__(SocketAddress(port=str(port)), meta_data)
        self.simulation_wait_time_factor = simulation_wait_time_factor
        self.run()


class MultiverseReader(MultiverseClient):

    MAX_WAIT_TIME_FOR_DATA: Optional[float] = 2
    """
    The maximum wait time for the data in seconds.
    """

    def __init__(self, name: str, port: int, is_prospection_world: Optional[bool] = False,
                 simulation_wait_time_factor: Optional[float] = 1.0, **kwargs):
        """
        Initialize the Multiverse reader, which reads the data from the Multiverse server in a separate thread.
        This class provides methods to get data (e.g., position, orientation) from the Multiverse server.
        param port: The port of the Multiverse reader client.
        param is_prospection_world: Whether the reader is connected to the prospection world.
        param simulation_wait_time_factor: The simulation wait time factor.
        """
        super().__init__(name, port, is_prospection_world, simulation_wait_time_factor=simulation_wait_time_factor)

        self.request_meta_data["receive"][""] = [""]

        self.data_lock = threading.Lock()
        self.thread = threading.Thread(target=self.receive_all_data_from_server)
        self.stop_thread = False

        self.thread.start()

    def get_body_pose(self, name: str, wait: Optional[bool] = True) -> Optional[Pose]:
        """
        Get the body pose from the multiverse server.
        param name: The name of the body.
        param wait: Whether to wait for the data.
        return: The position and orientation of the body.
        """
        data = self.get_multiple_body_poses([name], wait=wait)
        if data is not None:
            return data[name]

    def get_multiple_body_poses(self, body_names: List[str], wait: Optional[bool] = True) -> Optional[Dict[str, Pose]]:
        """
        Get the body poses from the multiverse server for multiple bodies.
        param body_names: The names of the bodies.
        param wait: Whether to wait for the data.
        return: The positions and orientations of the bodies as a dictionary.
        """
        data = self.get_multiple_body_data(body_names, {name: ["position", "quaternion"] for name in body_names},
                                           wait=wait)
        if data is not None:
            return {name: Pose(data[name]["position"], self.wxyz_to_xyzw(data[name]["quaternion"]))
                    for name in body_names}

    def get_body_position(self, name: str, wait: Optional[bool] = True) -> Optional[List[float]]:
        """
        Get the body position from the multiverse server.
        param name: The name of the body.
        param wait: Whether to wait for the data.
        return: The position of the body.
        """
        return self.get_multiple_body_positions([name], wait=wait)[name]

    def get_multiple_body_positions(self, body_names: List[str],
                                    wait: Optional[bool] = True) -> Optional[Dict[str, List[float]]]:
        """
        Get the body positions from the multiverse server for multiple bodies.
        param body_names: The names of the bodies.
        param wait: Whether to wait for the data.
        return: The positions of the bodies as a dictionary.
        """
        return self.get_multiple_body_properties(body_names, ["position"], wait=wait)

    def get_body_orientation(self, name: str, wait: Optional[bool] = True) -> Optional[List[float]]:
        """
        Get the body orientation from the multiverse server.
        param name: The name of the body.
        param wait: Whether to wait for the data.
        return: The orientation of the body.
        """
        return self.get_multiple_body_orientations([name], wait=wait)[name]

    def get_multiple_body_orientations(self, body_names: List[str],
                                       wait: Optional[bool] = True) -> Optional[Dict[str, List[float]]]:
        """
        Get the body orientations from the multiverse server for multiple bodies.
        param body_names: The names of the bodies.
        param wait: Whether to wait for the data.
        return: The orientations of the bodies as a dictionary.
        """
        data = self.get_multiple_body_properties(body_names, ["quaternion"], wait=wait)
        if data is not None:
            return {name: self.wxyz_to_xyzw(data[name]["quaternion"]) for name in body_names}

    def get_body_property(self, name: str, property_name: str, wait: Optional[bool] = True) -> Optional[List[float]]:
        """
        Get the body property from the multiverse server.
        param name: The name of the body.
        param property_name: The name of the property.
        param wait: Whether to wait for the data.
        return: The property of the body.
        """
        data = self.get_multiple_body_properties([name], [property_name], wait=wait)
        if data is not None:
            return data[name][property_name]

    def get_multiple_body_properties(self, body_names: List[str], properties: List[str],
                                     wait: Optional[bool] = True) -> Dict[str, Dict[str, List[float]]]:
        """
        Get the body properties from the multiverse server for multiple bodies.
        param body_names: The names of the bodies.
        param properties: The properties of the bodies.
        param wait: Whether to wait for the data.
        return: The properties of the bodies as a dictionary.
        """
        return self.get_multiple_body_data(body_names, {name: properties for name in body_names}, wait=wait)

    @staticmethod
    def wxyz_to_xyzw(quaternion: List[float]) -> List[float]:
        """
        Convert the quaternion from wxyz to xyzw.
        param quaternion: The quaternion as a list of floats.
        return: The quaternion as a list of floats.
        """
        return quaternion[1:] + [quaternion[0]]

    def get_body_data(self, name: str,
                      properties: Optional[List[str]] = None,
                      wait: Optional[bool] = True) -> Optional[Dict]:
        """
        Get the body data from the multiverse server.
        param name: The name of the body.
        param properties: The properties of the body.
        param wait: Whether to wait for the data.
        return: The body data as a dictionary.
        """
        properties = {name: properties} if properties is not None else None
        return self.get_multiple_body_data([name], properties, wait=wait)[name]

    def get_multiple_body_data(self, body_names: List[str], properties: Optional[Dict[str, List[str]]] = None,
                               wait: Optional[bool] = True) -> Dict:
        """
        Get the body data from the multiverse server for multiple bodies.
        param body_names: The names of the bodies.
        param properties: The properties of the bodies.
        param wait: Whether to wait for the data.
        return: The body data as a dictionary.
        """

        if wait:
            return self.wait_for_multiple_body_data(body_names, properties)

        data = self.get_received_data()
        if self.check_multiple_body_data(body_names, data, properties):
            return {name: data[name] for name in body_names}

    def wait_for_multiple_body_data(self, body_names: List[str],
                                    properties: Optional[Dict[str, List[str]]] = None) -> Dict:
        """
        Wait for the body data from the multiverse server for multiple bodies.
        param body_names: The names of the bodies.
        param properties: The properties of the bodies.
        return: The body data as a dictionary.
        """
        start = time()
        data_received_flag = False
        while time() - start < self.MAX_WAIT_TIME_FOR_DATA:
            received_data = self.get_received_data()
            data_received_flag = self.check_multiple_body_data(body_names, received_data, properties)
            if data_received_flag:
                return {name: received_data[name] for name in body_names}
        if not data_received_flag:
            properties_str = "Data" if properties is None else f"Properties {properties}"
            msg = f"{properties_str} for bodies {body_names} not received within {self.MAX_WAIT_TIME_FOR_DATA} seconds"
            logging.error(msg)
            raise ValueError(msg)

    def check_multiple_body_data(self, body_names: List[str], data: Dict,
                                 properties: Optional[Dict[str, List[str]]] = None) -> bool:
        """
        Check if the body data is received from the multiverse server for multiple bodies.
        param body_names: The names of the bodies.
        param data: The data received from the multiverse server.
        param properties: The properties of the bodies.
        return: Whether the body data is received.
        """
        if properties is None:
            return all([self.check_for_body_data(name, data) for name in body_names])
        else:
            return all([self.check_for_body_data(name, data, properties[name]) for name in body_names])

    @staticmethod
    def check_for_body_data(name: str, data: Dict, properties: Optional[List[str]] = None) -> bool:
        """
        Check if the body data is received from the multiverse server.
        param name: The name of the body.
        param data: The data received from the multiverse server.
        param properties: The properties of the body.
        return: Whether the body data is received.
        """
        if properties is None:
            return name in data
        else:
            return name in data and all([prop in data[name] and None not in data[name][prop] for prop in properties])

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

    time_for_sim_update: Optional[float] = 0.0  # 0.02
    """
    Wait time for the sent data to be applied in the simulator.
    """
    time_for_setting_body_data: Optional[float] = 0.0  # 0.01
    """
    Wait time for setting body data.
    """

    def __init__(self, name: str, port: int, simulation: str, is_prospection_world: Optional[bool] = False,
                 simulation_wait_time_factor: Optional[float] = 1.0, **kwargs):
        """
        Initialize the Multiverse writer, which writes the data to the Multiverse server.
        This class provides methods to send data (e.g., position, orientation) to the Multiverse server.
        param port: The port of the Multiverse writer client.
        param simulation: The name of the simulation that the writer is connected to
         (usually the name defined in the .muv file).
        param is_prospection_world: Whether the writer is connected to the prospection world.
        param simulation_wait_time_factor: The wait time factor for the simulation (default is 1.0), which can be used
         to increase or decrease the wait time for the simulation.
        """
        super().__init__(name, port, is_prospection_world, simulation_wait_time_factor=simulation_wait_time_factor)
        self.simulation = simulation
        self.time_start = time()

    def _reset_request_meta_data(self):
        """
        Reset the request metadata.
        """
        self.request_meta_data = {
            "meta_data": self._meta_data.__dict__.copy(),
            "send": {},
            "receive": {},
        }
        self.request_meta_data["meta_data"]["simulation_name"] = self.simulation

    def set_body_pose(self, body_name: str, position: List[float], orientation: List[float]) -> None:
        """
        Set the body pose in the simulation.
        param body_name: The name of the body.
        param position: The position of the body.
        param orientation: The orientation of the body.
        """
        self.send_body_data_to_server(body_name,
                                      {"position": position,
                                       "quaternion": orientation}
                                      )

    def set_body_position(self, body_name: str, position: List[float]) -> None:
        """
        Set the body position in the simulation.
        param body_name: The name of the body.
        param position: The position of the body.
        """
        self.send_body_data_to_server(body_name, {"position": position})

    def set_body_orientation(self, body_name: str, orientation: List[float]) -> None:
        """
        Set the body orientation in the simulation.
        param body_name: The name of the body.
        param orientation: The orientation of the body.
        """
        self.send_body_data_to_server(body_name, {"quaternion": orientation})

    def remove_body(self, body_name: str) -> None:
        """
        Remove the body from the simulation.
        param body_name: The name of the body.
        """
        self.send_data_to_server([time() - self.time_start],
                                 send_meta_data={body_name: []},
                                 receive_meta_data={body_name: []})

    def reset_world(self) -> None:
        """
        Reset the world in the simulation.
        """
        self.send_data_to_server([0])

    def send_body_data_to_server(self, body_name: str, data: Dict[str, List[float]]) -> Dict:
        """
        Send data to the multiverse server.
        param body_name: The name of the body.
        param data: The data to be sent.
        return: The response from the server.
        """
        return self.send_multiple_body_data_to_server({body_name: data})

    def send_multiple_body_data_to_server(self, body_data: Dict[str, Dict[str, List[float]]]) -> Dict:
        """
        Send data to the multiverse server for multiple bodies.
        param body_data: The data to be sent for multiple bodies.
        return: The response from the server.
        """
        send_meta_data = {body_name: list(data.keys()) for body_name, data in body_data.items()}
        response_meta_data = self.send_meta_data_and_get_response(send_meta_data)
        body_names = list(response_meta_data["send"].keys())
        flattened_data = [value for body_name in body_names for data in body_data[body_name].values()
                          for value in data]
        self.send_data = [time() - self.time_start, *flattened_data]
        self.send_and_receive_data()
        sleep(self.time_for_setting_body_data * self.simulation_wait_time_factor)
        return self.response_meta_data

    def send_meta_data_and_get_response(self, send_meta_data: Dict) -> Dict:
        """
        Send metadata to the multiverse server and get the response.
        param send_meta_data: The metadata to be sent.
        return: The response from the server.
        """
        self._reset_request_meta_data()
        self.request_meta_data["send"] = send_meta_data
        self.send_and_receive_meta_data()
        return self.response_meta_data

    def send_data_to_server(self, data: List,
                            send_meta_data: Optional[Dict] = None,
                            receive_meta_data: Optional[Dict] = None) -> Dict:
        """
        Send data to the multiverse server.
        param data: The data to be sent.
        param send_meta_data: The metadata to be sent.
        param receive_meta_data: The metadata to be received.
        return: The response from the server.
        """
        self._reset_request_meta_data()
        if send_meta_data:
            self.request_meta_data["send"] = send_meta_data
        if receive_meta_data:
            self.request_meta_data["receive"] = receive_meta_data
        self.send_and_receive_meta_data()
        self.send_data = data
        self.send_and_receive_data()
        sleep(self.time_for_sim_update * self.simulation_wait_time_factor)
        return self.response_meta_data


class MultiverseAPI(MultiverseClient):

    BASE_NAME: str = "api_requester"
    """
    The base name of the Multiverse reader.
    """
    API_REQUEST_WAIT_TIME: float = 0.2
    """
    The wait time for the API request in seconds.
    """

    def __init__(self, name: str, port: int, simulation: str, is_prospection_world: Optional[bool] = False,
                 simulation_wait_time_factor: Optional[float] = 1.0):
        """
        Initialize the Multiverse API, which sends API requests to the Multiverse server.
        This class provides methods like attach and detach objects, get contact points, and other API requests.
        param port: The port of the Multiverse API client.
        param simulation: The name of the simulation that the API is connected to
         (usually the name defined in the .muv file).
        param is_prospection_world: Whether the API is connected to the prospection world.
        param simulation_wait_time_factor: The simulation wait time factor, which can be used to increase or decrease
            the wait time for the simulation.
        """
        super().__init__(name, port, is_prospection_world, simulation_wait_time_factor=simulation_wait_time_factor)
        self.simulation = simulation

    def attach(self, constraint: Constraint) -> None:
        """
        Request to attach the child link to the parent link.
        param constraint: The constraint.
        """
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        attachment_pose = self._get_attachment_pose_as_string(constraint)
        self._attach(child_link_name, parent_link_name, attachment_pose)

    def _attach(self, child_link_name: str, parent_link_name: str, attachment_pose: str) -> None:
        """
        Attach the child link to the parent link.
        param child_link_name: The name of the child link.
        param parent_link_name: The name of the parent link.
        param attachment_pose: The attachment pose.
        """
        self._request_single_api_callback(API.ATTACH, child_link_name, parent_link_name,
                                          attachment_pose)

    def get_constraint_link_names(self, constraint: Constraint) -> Tuple[str, str]:
        """
        Get the link names of the constraint.
        param constraint: The constraint.
        return: The link names of the constraint.
        """
        return self.get_parent_link_name(constraint), self.get_constraint_child_link_name(constraint)

    def get_parent_link_name(self, constraint: Constraint) -> str:
        """
        Get the parent link name of the constraint.
        param constraint: The constraint.
        return: The parent link name of the constraint.
        """
        return self.get_link_name_for_constraint(constraint.parent_link)

    def get_constraint_child_link_name(self, constraint: Constraint) -> str:
        """
        Get the child link name of the constraint.
        param constraint: The constraint.
        return: The child link name of the constraint.
        """
        return self.get_link_name_for_constraint(constraint.child_link)

    @staticmethod
    def get_link_name_for_constraint(link: Link) -> str:
        """
        Get the link name from link object, if the link belongs to a one link object, return the object name.
        param link: The link.
        return: The link name.
        """
        return link.name if not link.is_only_link else link.object.name

    def detach(self, constraint: Constraint) -> None:
        """
        Request to detach the child link from the parent link.
        param constraint: The constraint.
        """
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        self._detach(child_link_name, parent_link_name)

    def _detach(self, child_link_name: str, parent_link_name: str) -> None:
        """
        Detach the child link from the parent link.
        param child_link_name: The name of the child link.
        param parent_link_name: The name of the parent link.
        """
        self._request_single_api_callback(API.DETACH, child_link_name, parent_link_name)

    def _get_attachment_pose_as_string(self, constraint: Constraint) -> str:
        """
        Get the attachment pose as a string.
        param constraint: The constraint.
        return: The attachment pose as a string.
        """
        pose = constraint.parent_to_child_transform.to_pose()
        return self._pose_to_string(pose)

    @staticmethod
    def _pose_to_string(pose: Pose) -> str:
        """
        Convert the pose to a string.
        param pose: The pose.
        return: The pose as a string.
        """
        return f"{pose.position.x} {pose.position.y} {pose.position.z} {pose.orientation.w} {pose.orientation.x} " \
               f"{pose.orientation.y} {pose.orientation.z}"

    def check_object_exists(self, obj: Object) -> bool:
        """
        Check if the object exists in the simulation.
        param obj: The object.
        return: Whether the object exists in the simulation.
        """
        return self._object_exists(obj.name)

    def _object_exists(self, object_name: str) -> bool:
        """
        Check if the object exists in the simulation.
        param object_name: The name of the object.
        return: Whether the object exists in the simulation.
        """
        return self._request_single_api_callback(API.EXIST, object_name)[0] == 'yes'

    def get_contact_points(self, obj: Object) -> List[MultiverseContactPoint]:
        """
        Request the contact points of an object, this includes the object names and the contact forces and torques.
        param obj: The object.
        return: The contact points of the object as a list of MultiverseContactPoint.
        """
        api_response_data = self._get_contact_points(obj.name)
        body_names = api_response_data[API.GET_CONTACT_BODIES]
        contact_efforts = self._parse_constraint_effort(api_response_data[API.GET_CONSTRAINT_EFFORT])
        return [MultiverseContactPoint(body_names[i], contact_efforts[:3], contact_efforts[3:])
                for i in range(len(body_names))]

    def get_objects_intersected_with_rays(self, from_positions: List[List[float]],
                                          to_positions: List[List[float]]) -> List[RayResult]:
        """
        Get the rays intersections with the objects from the from_positions to the to_positions.
        param from_positions: The starting positions of the rays.
        param to_positions: The ending positions of the rays.
        return: The rays intersections with the objects as a list of RayResult.
        """
        get_rays_response = self._get_rays(from_positions, to_positions)
        return self._parse_get_rays_response(get_rays_response)

    def _get_rays(self, from_positions: List[List[float]],
                  to_positions: List[List[float]]) -> List[str]:
        """
        Get the rays intersections with the objects from the from_positions to the to_positions.
        param from_positions: The starting positions of the rays.
        param to_positions: The ending positions of the rays.
        return: The rays intersections with the objects as a dictionary.
        """
        from_positions = self.list_of_positions_to_string(from_positions)
        to_positions = self.list_of_positions_to_string(to_positions)
        return self._request_single_api_callback(API.GET_RAYS, from_positions, to_positions)

    @staticmethod
    def _parse_get_rays_response(response: List[str]) -> List[RayResult]:
        """
        Parse the response of the get rays API.
        param response: The response of the get rays API as a list of strings.
        return: The rays as a list of lists of floats.
        """
        get_rays_results = []
        for ray_response in response:
            if ray_response == "None":
                get_rays_results.append(RayResult("", -1))
            else:
                result = ray_response.split()
                result[1] = float(result[1])
                get_rays_results.append(RayResult(*result))
        return get_rays_results

    @staticmethod
    def list_of_positions_to_string(positions: List[List[float]]) -> str:
        """
        Convert the list of positions to a string.
        param positions: The list of positions.
        return: The list of positions as a string.
        """
        return " ".join([f"{position[0]} {position[1]} {position[2]}" for position in positions])

    @staticmethod
    def _parse_constraint_effort(contact_effort: List[str]) -> List[float]:
        """
        Parse the contact effort of an object.
        param contact_effort: The contact effort of the object as a list of strings.
        return: The contact effort of the object as a list of floats.
        """
        return list(map(float, contact_effort[0].split()))

    def _get_contact_points(self, object_name) -> Dict[API, List]:
        """
        Request the contact points of an object.
        param object_name: The name of the object.
        return: The contact points api response as a dictionary.
        """
        return self._request_apis_callbacks({API.GET_CONTACT_BODIES: [object_name],
                                             API.GET_CONSTRAINT_EFFORT: [object_name]
                                             })

    def _request_single_api_callback(self, api_name: API, *params) -> List[str]:
        """
        Request a single API callback from the server.
        param api_data: The API data to request the callback.
        return: The API response as a list of strings.
        """
        response = self._request_apis_callbacks({api_name: list(params)})
        return response[api_name]

    def _request_apis_callbacks(self, api_data: Dict[API, List]) -> Dict[API, List[str]]:
        """
        Request the API callbacks from the server.
        param api_data_request: The API data to add to the request metadata.
        return: The API response as a list of strings.
        """
        self._reset_api_callback()
        for api_name, params in api_data.items():
            self._add_api_request(api_name.value, *params)
        self._send_api_request()
        responses = self._get_all_apis_responses()
        sleep(self.API_REQUEST_WAIT_TIME * self.simulation_wait_time_factor)
        return responses

    def _get_all_apis_responses(self) -> Dict[API, List[str]]:
        """
        Get all the API responses from the server.
        return: The API responses as a list of APIData.
        """
        list_of_api_responses = self.response_meta_data["api_callbacks_response"][self.simulation]
        return {API[api_name.upper()]: response for api_response in list_of_api_responses
                for api_name, response in api_response.items()}

    def _add_api_request(self, api_name: str, *params):
        """
        Add an API request to the request metadata.
        param api_name: The name of the API.
        param params: The parameters of the API.
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
