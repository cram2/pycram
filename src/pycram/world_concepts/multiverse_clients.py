import logging
import threading
from dataclasses import dataclass
from time import time, sleep

from typing_extensions import List, Dict, Tuple, Optional

from .constraints import Constraint
from .multiverse_socket import MultiverseSocket, MultiverseMetaData, SocketAddress
from .world_object import Object, Link
from ..datastructures.world import World
from ..datastructures.pose import Pose


@dataclass
class RayResult:
    """
    A dataclass to store the ray result. The ray result contains the body name that the ray intersects with and the
    distance from the ray origin to the intersection point.
    """
    body_name: str
    distance: float

    def intersected(self) -> bool:
        """
        Check if the ray intersects with a body.
        return: Whether the ray intersects with a body.
        """
        return self.distance >= 0 and self.body_name != ""


@dataclass
class APIData:
    """
    A dataclass to store the API data.
    """
    api_name: str
    params: List[str]

    @property
    def as_dict(self) -> Dict:
        return {self.api_name: self.params}


class APIDataDict(dict):
    """
    A dictionary to store the API data, where the key is the API name and the value is the parameters.
    """

    @classmethod
    def from_list(cls, api_data_list: List[APIData]) -> "APIDataDict":
        data = {api_data.api_name: api_data.params for api_data in api_data_list}
        return cls(data)


@dataclass
class MultiverseContactPoint:
    """
    A dataclass to store the contact point returned from Multiverse.
    """
    body_name: str
    contact_force: List[float]
    contact_torque: List[float]


class MultiverseReader(MultiverseSocket):

    PORT: int = 9000
    """
    The port of the Multiverse reader client.
    """
    PROSPECTION_PORT: int = PORT + 3
    """
    The port of the Multiverse reader client for the prospection world."""

    def __init__(self, max_wait_time_for_data: Optional[float] = 1, is_prospection_world: Optional[bool] = False):
        """
        Initialize the Multiverse reader, which reads the data from the Multiverse server in a separate thread.
        This class provides methods to get data (e.g., position, orientation) from the Multiverse server.
        param max_wait_time_for_data: The maximum wait time for the data in seconds.
        param is_prospection_world: Whether the reader is connected to the prospection world.
        """
        self.max_wait_time_for_data = max_wait_time_for_data
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = (World.prospection_world_prefix if is_prospection_world else "") + "reader"
        meta_data.world_name = (World.prospection_world_prefix if is_prospection_world else "") + meta_data.world_name
        port = self.PROSPECTION_PORT if is_prospection_world else self.PORT
        super().__init__(SocketAddress(port=str(port)), meta_data)
        self.run()
        self.data_lock = threading.Lock()
        self.thread = threading.Thread(target=self.receive_all_data_from_server)
        self.stop_thread = False
        self.request_meta_data["receive"][""] = [""]
        self.thread.start()

    def get_body_pose(self, name: str, wait: Optional[bool] = True) -> Optional[Pose]:
        """
        Get the body pose from the multiverse server.
        param name: The name of the body.
        param wait: Whether to wait for the data.
        return: The position and orientation of the body.
        """
        data = self.get_body_data(name, ["position", "quaternion"], wait=wait)
        if data is not None:
            return Pose(data["position"], self.wxyz_to_xyzw(data["quaternion"]))

    def get_body_position(self, name: str, wait: Optional[bool] = True) -> Optional[List[float]]:
        """
        Get the body position from the multiverse server.
        param name: The name of the body.
        param wait: Whether to wait for the data.
        return: The position of the body.
        """
        return self.get_body_data(name, ["position"], wait=wait)

    def get_body_orientation(self, name: str, wait: Optional[bool] = True) -> Optional[List[float]]:
        """
        Get the body orientation from the multiverse server.
        param name: The name of the body.
        param wait: Whether to wait for the data.
        return: The orientation of the body.
        """
        data = self.get_body_property(name, "quaternion", wait=wait)
        if data is not None:
            return self.wxyz_to_xyzw(data)

    def get_body_property(self, name: str, property_name: str, wait: Optional[bool] = True) -> Optional[List[float]]:
        """
        Get the body property from the multiverse server.
        param name: The name of the body.
        param property_name: The name of the property.
        param wait: Whether to wait for the data.
        return: The property of the body.
        """
        data = self.get_body_data(name, [property_name], wait=wait)
        if data is not None:
            return data[property_name]

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
        if wait:
            return self.wait_for_body_data(name, properties)
        elif self.check_for_body_data(name, self.get_received_data(), properties):
            return self.get_received_data()[name]

    def wait_for_body_data(self, name: str, properties: Optional[List[str]] = None) -> Dict:
        """
        Wait for the body data from the multiverse server.
        param name: The name of the body.
        param properties: The properties of the body.
        return: The body data as a dictionary.
        """
        start = time()
        while time() - start < self.max_wait_time_for_data:
            received_data = self.get_received_data()
            data_received_flag = self.check_for_body_data(name, received_data, properties)
            if data_received_flag:
                return received_data[name]

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


class MultiverseWriter(MultiverseSocket):

    time_for_sim_update: Optional[float] = 0.02
    """
    Wait time for the sent data to be applied in the simulator.
    """
    time_for_setting_body_data: Optional[float] = 0.01
    """
    Wait time for setting body data.
    """

    PORT: int = MultiverseReader.PORT + 1
    """
    The port of the Multiverse writer client.
    """
    PROSPECTION_PORT: int = PORT + 3
    """
    The port of the Multiverse writer client for the prospection world.
    """

    def __init__(self, simulation: str, is_prospection_world: Optional[bool] = False):
        """
        Initialize the Multiverse writer, which writes the data to the Multiverse server.
        This class provides methods to send data (e.g., position, orientation) to the Multiverse server.
        param simulation: The name of the simulation that the writer is connected to
         (usually the name defined in the .muv file).
        param is_prospection_world: Whether the writer is connected to the prospection world.
        """
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = (World.prospection_world_prefix if is_prospection_world else "") + "writer"
        meta_data.world_name = (World.prospection_world_prefix if is_prospection_world else "") + meta_data.world_name
        port = self.PROSPECTION_PORT if is_prospection_world else self.PORT
        super().__init__(SocketAddress(port=str(port)), meta_data)
        self.simulation = simulation
        self.time_start = time()
        self.run()

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
                                       "quaternion": orientation})

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
        sleep(self.time_for_setting_body_data)
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
        sleep(self.time_for_sim_update)
        return self.response_meta_data


class MultiverseAPI(MultiverseSocket):

    GET_CONTACT_BODIES_API_NAME = "get_contact_bodies"
    GET_CONSTRAINT_EFFORT_API_NAME = "get_constraint_effort"
    ATTACH_API_NAME = "attach"
    DETACH_API_NAME = "detach"
    GET_RAYS_API_NAME = "get_rays"
    EXIST_API_NAME = "exist"
    """
    The API names for the API callbacks to the Multiverse server.
    """

    PORT: int = MultiverseWriter.PORT + 1
    """
    The port of the Multiverse API client.
    """
    PROSPECTION_PORT: int = PORT + 3
    """
    The port of the Multiverse API client for the prospection world.
    """
    API_REQUEST_WAIT_TIME: float = 0.2
    """
    The wait time for the API request in seconds.
    """

    def __init__(self, simulation: str, is_prospection_world: Optional[bool] = False):
        """
        Initialize the Multiverse API, which sends API requests to the Multiverse server.
        This class provides methods like attach and detach objects, get contact points, and other API requests.
        param simulation: The name of the simulation that the API is connected to
         (usually the name defined in the .muv file).
        param is_prospection_world: Whether the API is connected to the prospection world.
        """
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = (World.prospection_world_prefix if is_prospection_world else "") + "api_requester"
        meta_data.world_name = (World.prospection_world_prefix if is_prospection_world else "") + meta_data.world_name
        port = self.PROSPECTION_PORT if is_prospection_world else self.PORT
        super().__init__(SocketAddress(port=str(port)), meta_data)
        self.simulation = simulation
        self.run()

    def attach(self, constraint: Constraint) -> None:
        """
        Request to attach the child link to the parent link.
        param constraint: The constraint.
        """
        self._request_single_api_callback(self._get_attach_api_data(constraint))

    def _get_attach_api_data(self, constraint: Constraint) -> APIData:
        """
        Get the attach API data to be added to the api callback request metadata.
        param constraint: The constraint.
        return: The attach API data as an APIData.
        """
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        return APIData(self.ATTACH_API_NAME, [child_link_name,
                                              parent_link_name,
                                              self._get_attachment_pose_as_string(constraint)])

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
        self._request_single_api_callback(self._get_detach_api_data(constraint))

    def _get_detach_api_data(self, constraint: Constraint) -> APIData:
        """
        Get the detach API data to be added to the api callback request metadata.
        param constraint: The constraint.
        return: The detach API data as an APIData.
        """
        parent_link_name, child_link_name = self.get_constraint_link_names(constraint)
        return APIData(self.DETACH_API_NAME, [child_link_name, parent_link_name])

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

    def check_object_exists(self, object_name: str) -> bool:
        """
        Check if the object exists in the simulation.
        param object_name: The name of the object.
        return: Whether the object exists in the simulation.
        """
        api_data = self._get_object_exists_api_data(object_name)
        return self._request_single_api_callback(api_data)[0] == 'yes'

    def _get_object_exists_api_data(self, object_name: str) -> APIData:
        """
        Get the object exists API data to be added to the api callback request metadata.
        param object_name: The name of the object.
        return: The object exists API data as an APIData.
        """
        return APIData(self.EXIST_API_NAME, [object_name])

    def get_contact_points(self, obj: Object) -> List[MultiverseContactPoint]:
        """
        Request the contact points of an object, this includes the object names and the contact forces and torques.
        param obj: The object.
        return: The contact points of the object as a list of MultiverseContactPoint.
        """
        api_response_data = self._request_apis_callbacks(self._get_contact_points_api_data(obj))
        body_names = api_response_data[self.GET_CONTACT_BODIES_API_NAME]
        contact_efforts = self._parse_constraint_effort(api_response_data[self.GET_CONSTRAINT_EFFORT_API_NAME])
        return [MultiverseContactPoint(body_names[i], contact_efforts[:3], contact_efforts[3:])
                for i in range(len(body_names))]

    def get_objects_intersected_with_rays(self, from_positions: List[List[float]],
                                          to_positions: List[List[float]]) -> List[RayResult]:
        """
        Get the rays from the from_positions to the to_positions.
        param from_positions: The from positions of the rays.
        param to_positions: The to positions of the rays.
        return: The rays as a list of RayResult.
        """
        api_data = self._get_rays_api_data(from_positions, to_positions)
        get_rays_response = self._request_single_api_callback(api_data)
        return self._parse_get_rays_response(get_rays_response)

    def _get_rays_api_data(self, from_positions: List[List[float]], to_positions: List[List[float]]) -> APIData:
        """
        Get the rays API data to be added to the api callback request metadata.
        param from_positions: The from positions of the rays.
        param to_positions: The to positions of the rays.
        return: The rays API data as an APIData.
        """
        return APIData(self.GET_RAYS_API_NAME, [self.list_of_positions_to_string(from_positions),
                                                self.list_of_positions_to_string(to_positions)]
                       )

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

    def _request_objects_in_contact(self, obj: Object) -> List[str]:
        """
        Request the objects in contact with an object.
        param obj: The object.
        return: The objects in contact as a list of strings for object names.
        """
        return self._request_single_api_callback(self._get_contact_bodies_api_data(obj))

    def _request_contact_effort(self, obj: Object) -> List[str]:
        """
        Request the contact effort of an object.
        param obj: The object.
        return: The contact effort of the object as a list of strings that represent the contact forces and torques.
        """
        return self._request_single_api_callback(self._get_constraint_effort_api_data(obj))

    def _get_contact_points_api_data(self, obj: Object) -> APIDataDict:
        """
        Get the contact points API data to be added to the api callback request metadata, this includes the
         contact bodies and the contact effort api data.
        param obj: The object.
        return: The contact points API data as an APIDataDict.
        """
        api_data_list = [self._get_contact_bodies_api_data(obj), self._get_constraint_effort_api_data(obj)]
        return APIDataDict.from_list(api_data_list)

    @staticmethod
    def _get_contact_bodies_api_data(obj: Object):
        """
        Get the contact bodies API data to be added to the api callback request metadata.
        param obj: The object.
        """
        return APIData("get_contact_bodies", [obj.name])

    @staticmethod
    def _get_constraint_effort_api_data(obj: Object) -> APIData:
        """
        Get the constraint effort API data to be added to the api callback request metadata.
        param obj: The object.
        """
        return APIData("get_constraint_effort", [obj.name])

    def _request_single_api_callback(self, api_data: APIData) -> List[str]:
        """
        Request a single API callback from the server.
        param api_data: The API data to request the callback.
        return: The API response as a list of strings.
        """
        api_data_dict = APIDataDict(api_data.as_dict)
        response = self._request_apis_callbacks(api_data_dict)
        return response[api_data.api_name]

    def _request_apis_callbacks(self, api_data: APIDataDict) -> APIDataDict:
        """
        Request the API callbacks from the server.
        param api_data_request: The API data to add to the request metadata.
        return: The API response as a list of strings.
        """
        self._reset_api_callback()
        for api_name, params in api_data.items():
            self._add_api_request(api_name, *params)
        self._send_api_request()
        responses = self._get_all_apis_responses()
        sleep(self.API_REQUEST_WAIT_TIME)
        return responses

    def _get_all_apis_responses(self) -> APIDataDict:
        """
        Get all the API responses from the server.
        return: The API responses as a list of APIData.
        """
        list_of_api_responses = self.response_meta_data["api_callbacks_response"][self.simulation]
        dict_of_api_responses = APIDataDict({api_name: response for api_response in list_of_api_responses
                                             for api_name, response in api_response.items()})
        return dict_of_api_responses

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
