import logging
import threading
from dataclasses import dataclass
from time import time

from typing_extensions import List, Dict, Tuple, Optional

from .constraints import Constraint
from .multiverse_socket import MultiverseSocket, MultiverseMetaData, SocketAddress
from .world_object import Object, Link
from ..datastructures.pose import Pose


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
    def __init__(self):
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = "reader"
        super().__init__(SocketAddress(port="8000"), meta_data)
        self.run()
        self.data_lock = threading.Lock()
        self.thread = threading.Thread(target=self.get_all_data_from_server)
        self.stop_thread = False
        self.request_meta_data["receive"][""] = [""]
        self.thread.start()

    def get_all_data_from_server(self):
        """
        Get all data from the multiverse server.
        """
        while not self.stop_thread:
            self.data_lock.acquire()
            self.send_and_receive_meta_data()
            self.data_lock.release()
        self.stop()

    def join(self):
        self.thread.join()


class MultiverseWriter(MultiverseSocket):
    def __init__(self, simulation: str):
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = "writer"
        super().__init__(SocketAddress(port="8001"), meta_data)
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

    def set_body_pose(self, body_name: str, position: List[float], orientation: List[float]):
        self.send_body_data_to_server(body_name,
                                      {"position": position,
                                       "orientation": orientation})

    def set_body_position(self, body_name: str, position: List[float]):
        self.send_body_data_to_server(body_name, {"position": position})

    def set_body_orientation(self, body_name: str, orientation: List[float]):
        self.send_body_data_to_server(body_name, {"orientation": orientation})

    def remove_body(self, body_name: str):
        self.send_data_to_server([time() - self.time_start],
                                 send_meta_data={body_name: []},
                                 receive_meta_data={body_name: []})

    def reset_world(self):
        self.send_data_to_server([0])

    def send_body_data_to_server(self, body_name: str, data: Dict[str, List[float]]) -> Dict:
        """
        Send data to the multiverse server.
        param body_name: The name of the body.
        param data: The data to be sent.
        """
        send_meta_data = {body_name: list(data.keys())}
        send_data = [time() - self.time_start, *data.values()]
        return self.send_data_to_server(send_data, send_meta_data=send_meta_data)

    def send_data_to_server(self, data: List,
                            send_meta_data: Optional[Dict] = None,
                            receive_meta_data: Optional[Dict] = None) -> Dict:
        """
        Send data to the multiverse server.
        param data: The data to be sent.
        param send_meta_data: The metadata to be sent.
        """
        self._reset_request_meta_data()
        if send_meta_data:
            self.request_meta_data["send"] = send_meta_data
        if receive_meta_data:
            self.request_meta_data["receive"] = receive_meta_data
        self.send_and_receive_meta_data()
        self.send_data = data
        self.send_and_receive_data()
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

    def __init__(self, simulation: str):
        meta_data = MultiverseMetaData()
        meta_data.simulation_name = "api_requester"
        super().__init__(SocketAddress(port="8002"), meta_data)
        self.simulation = simulation

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
        pose = constraint.child_link.get_pose_wrt_link(constraint.parent_link)
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

    def check_object_exists(self, object_name: str) -> List[str]:
        api_data = self._get_object_exists_api_data(object_name)
        return self._request_single_api_callback(api_data)

    def _get_object_exists_api_data(self, object_name: str) -> APIData:
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
        return self._get_all_apis_responses()

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
