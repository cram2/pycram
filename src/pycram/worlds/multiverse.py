import logging
import os
from dataclasses import dataclass
from time import time, sleep

from typing_extensions import List, Dict, Optional, Tuple
from tf.transformations import quaternion_matrix
import numpy as np

from ..datastructures.dataclasses import AxisAlignedBoundingBox, Color, ContactPointsList, ContactPoint
from ..datastructures.enums import WorldMode, JointType, ObjectType
from ..datastructures.pose import Pose
from ..datastructures.world import World
from ..description import Link, Joint
from ..world_concepts.constraints import Constraint
from ..world_concepts.multiverse_socket import MultiverseSocket, SocketAddress
from ..world_concepts.world_object import Object


def get_resource_paths(dirname: str) -> List[str]:
    resources_paths = ["../robots", "../worlds", "../objects"]
    resources_paths = [
        os.path.join(dirname, resources_path.replace('../', '')) if not os.path.isabs(
            resources_path) else resources_path
        for resources_path in resources_paths
    ]

    def add_directories(path: str) -> None:
        with os.scandir(path) as entries:
            for entry in entries:
                if entry.is_dir():
                    resources_paths.append(entry.path)
                    add_directories(entry.path)

    resources_path_copy = resources_paths.copy()
    for resources_path in resources_path_copy:
        add_directories(resources_path)

    return resources_paths


def find_multiverse_resources_path() -> Optional[str]:
    """
    Find the path to the Multiverse resources directory.
    """
    # Get the path to the Multiverse installation
    multiverse_path = find_multiverse_path()

    # Check if the path to the Multiverse installation was found
    if multiverse_path:
        # Construct the path to the resources directory
        resources_path = os.path.join(multiverse_path, 'resources')

        # Check if the resources directory exists
        if os.path.exists(resources_path):
            return resources_path

    return None


def find_multiverse_path() -> Optional[str]:
    """
    Find the path to the Multiverse installation.
    """
    # Get the value of PYTHONPATH environment variable
    pythonpath = os.getenv('PYTHONPATH')

    # Check if PYTHONPATH is set
    if pythonpath:
        # Split the PYTHONPATH into individual paths using the platform-specific path separator
        paths = pythonpath.split(os.pathsep)

        # Iterate through each path and check if 'Multiverse' is in it
        for path in paths:
            if 'multiverse' in path:
                multiverse_path = path.split('multiverse')[0]
                return multiverse_path + 'multiverse'

    return None


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


class Multiverse(MultiverseSocket, World):
    """
    This class implements an interface between Multiverse and PyCRAM.
    """

    _joint_type_to_position_name: Dict[JointType, str] = {
        JointType.REVOLUTE: "joint_rvalue",
        JointType.PRISMATIC: "joint_tvalue",
    }
    """
    A dictionary to map JointType to the corresponding multiverse attribute name.
    """

    added_multiverse_resources: bool = False
    """
    A flag to check if the multiverse resources have been added.
    """

    GET_CONTACT_BODIES_API_NAME = "get_contact_bodies"
    GET_CONSTRAINT_EFFORT_API_NAME = "get_constraint_effort"
    ATTACH_API_NAME = "attach"
    DETACH_API_NAME = "detach"
    GET_RAYS_API_NAME = "get_rays"
    EXIST_API_NAME = "exist"
    """
    The API names for the API callbacks to the Multiverse server.
    """

    def __init__(self, simulation: str, mode: Optional[WorldMode] = WorldMode.DIRECT,
                 is_prospection: Optional[bool] = False,
                 simulation_frequency: Optional[float] = 60.0,
                 client_addr: Optional[SocketAddress] = SocketAddress(port="7000")):
        """
        Initialize the Multiverse Socket and the PyCram World.
        param mode: The mode of the world (DIRECT or GUI).
        param is_prospection: Whether the world is prospection or not.
        param simulation_frequency: The frequency of the simulation.
        param client_addr: The address of the multiverse client.
        """
        MultiverseSocket.__init__(self, client_addr)
        World.__init__(self, mode, is_prospection, simulation_frequency)
        self.simulation: str = simulation
        self._make_sure_multiverse_resources_are_added()
        self.set_attached_objects_poses = False
        self.handle_spawning = False
        self.update_poses_on_get = True
        self.last_object_id: int = -1
        self.last_constraint_id: int = -1
        self.constraints: Dict[int, Constraint] = {}
        self.object_name_to_id: Dict[str, int] = {}
        self.object_id_to_name: Dict[int, str] = {}
        self.time_start = time()
        self.run()
        self.floor = self._spawn_floor()

    def _init_world(self, mode: WorldMode):
        pass

    def _make_sure_multiverse_resources_are_added(self):
        """
        Add the multiverse resources to the pycram world resources.
        """
        if not self.added_multiverse_resources:
            dirname = find_multiverse_resources_path()
            resources_paths = get_resource_paths(dirname)
            for resource_path in resources_paths:
                self.add_resource_path(resource_path)
            self.added_multiverse_resources = True

    def _spawn_floor(self):
        """
        Spawn the plane in the simulator.
        """
        floor = Object("floor", ObjectType.ENVIRONMENT, "plane.urdf",
                       world=self)
        # sleep(0.5)
        return floor

    def get_joint_position_name(self, joint: Joint) -> str:
        if joint.type not in self._joint_type_to_position_name:
            logging.warning(f"Invalid joint type: {joint.type}")
            return "joint_rvalue"
        return self._joint_type_to_position_name[joint.type]

    def load_object_and_get_id(self, name: Optional[str] = None,
                               pose: Optional[Pose] = None) -> int:
        """
        Spawn the object in the simulator and return the object id. Object name has to be unique and has to be same as
        the name of the object in the description file.
        param name: The name of the object to be loaded.
        param pose: The pose of the object.
        """
        if pose is None:
            pose = Pose()

        self._reset_body_pose(name, pose)

        self.last_object_id += 1

        self.object_name_to_id[name] = self.last_object_id
        self.object_id_to_name[self.last_object_id] = name

        return self.last_object_id

    def _init_spawn_object(self, name: str) -> None:
        """
        Initialize the object spawning process.
        """
        self._init_setter()
        self.request_meta_data["send"][name] = ["position", "quaternion"]
        self.send_and_receive_meta_data()

    def get_object_joint_names(self, obj: Object) -> List[str]:

        return [joint.name for joint in obj.description.joints
                if joint.type in [JointType.REVOLUTE, JointType.PRISMATIC]]

    def get_object_link_names(self, obj: Object) -> List[str]:
        return [link.name for link in obj.description.links]

    def _init_getter(self):
        self._reset_request_meta_data()

    def get_joint_position(self, joint: Joint) -> float:
        self.check_object_exists_and_issue_warning_if_not(joint.object)
        self._init_getter()
        attribute = self.get_joint_position_name(joint)
        self.request_meta_data["receive"][joint.name] = [attribute]
        self.send_and_receive_meta_data()
        receive_data = self.response_meta_data["receive"][joint.name][attribute]
        if len(receive_data) != 1:
            logging.error(f"Invalid joint position data: {receive_data}")
            raise ValueError
        return receive_data[0]

    def _init_setter(self):
        self._reset_request_meta_data()
        self._set_simulation_in_request_meta_data()

    def reset_joint_position(self, joint: Joint, joint_position: float) -> None:
        self._init_setter()
        attribute = self.get_joint_position_name(joint)
        self.request_meta_data["send"][joint.name] = [attribute]
        self.send_and_receive_meta_data()
        self.send_data = [time() - self.time_start, joint_position]
        self.send_and_receive_data()

    def get_link_pose(self, link: Link) -> Pose:
        self.check_object_exists_and_issue_warning_if_not(link.object)
        return self._get_body_pose(link.name)

    def get_object_pose(self, obj: Object) -> Pose:
        self.check_object_exists_and_issue_warning_if_not(obj)
        return self._get_body_pose(obj.name)

    def _get_body_pose(self, body_name: str) -> Pose:
        self._init_getter()
        self.request_meta_data["receive"][body_name] = ["position", "quaternion"]
        self.send_and_receive_meta_data()
        self.send_data = [time() - self.time_start]
        self.send_and_receive_data()
        if len(self.receive_data) != 8:
            logging.error(f"Invalid body pose data: {self.receive_data}")
            raise ValueError
        wxyz = self.receive_data[4:]
        xyzw = [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]
        return Pose(self.receive_data[1:4], xyzw)

    def reset_object_base_pose(self, obj: Object, pose: Pose):
        self.check_object_exists_and_issue_warning_if_not(obj)
        self._reset_body_pose(obj.name, pose)

    def multiverse_reset_world(self):
        self._init_setter()
        self.send_and_receive_meta_data()
        self.send_data = [0]
        self.send_and_receive_data()

    def _reset_body_pose(self, body_name: str, pose: Pose) -> None:
        """
        Reset the pose of a body in the simulator.
        param body_name: The name of the body.
        param pose: The pose of the body.
        """
        self._init_setter()
        self.request_meta_data["send"][body_name] = ["position", "quaternion"]
        self.send_and_receive_meta_data()
        xyzw = pose.orientation_as_list()
        wxyz = [xyzw[3], *xyzw[:3]]
        self.send_data = [time() - self.time_start, *pose.position_as_list(), *wxyz]
        self.send_and_receive_data()

    def get_all_objects_data_from_server(self) -> Dict[str, Dict]:
        """
        Get all objects data from the multiverse server.
        """
        self._init_getter()
        self.request_meta_data["receive"][""] = [""]
        self.send_and_receive_meta_data()
        return self.response_meta_data["receive"]

    def disconnect_from_physics_server(self) -> None:
        self._reset_request_meta_data()
        self.stop()

    def join_threads(self) -> None:
        pass

    def remove_object_from_simulator(self, obj: Object) -> None:
        self._multiverse_remove_object(obj.name)

    def _multiverse_remove_object(self, object_name: str):
        """
        Remove the object from the simulator.
        """
        self._set_simulation_in_request_meta_data()
        self.request_meta_data["send"][object_name] = []
        self.request_meta_data["receive"][object_name] = []
        self.send_and_receive_meta_data()
        self.send_data = [time() - self.time_start]
        self.send_and_receive_data()

    def add_constraint(self, constraint: Constraint) -> int:

        if constraint.type != JointType.FIXED:
            logging.error("Only fixed constraints are supported in Multiverse")
            raise ValueError

        self._request_attach(constraint)

        self.last_constraint_id += 1

        self.constraints[self.last_constraint_id] = constraint

        return self.last_constraint_id

    def _request_attach(self, constraint: Constraint) -> None:
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

    def remove_constraint(self, constraint_id) -> None:
        constraint = self.constraints.pop(constraint_id)
        self._request_detach(constraint)

    def _request_detach(self, constraint: Constraint) -> None:
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

    @staticmethod
    def get_link_name_for_constraint(link: Link) -> str:
        """
        Get the link name from link object, if the link belongs to a one link object, return the object name.
        param link: The link.
        return: The link name.
        """
        return link.name if not link.is_only_link else link.object.name

    def _get_attachment_pose_as_string(self, constraint: Constraint) -> str:
        """
        Get the attachment pose as a string.
        param constraint: The constraint.
        return: The attachment pose as a string.
        """
        self.check_object_exists_and_issue_warning_if_not(constraint.parent_link.object)
        self.check_object_exists_and_issue_warning_if_not(constraint.child_link.object)
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

    def perform_collision_detection(self) -> None:
        logging.warning("perform_collision_detection is not implemented in Multiverse")

    def get_object_contact_points(self, obj: Object) -> ContactPointsList:
        self.check_object_exists_and_issue_warning_if_not(obj)
        multiverse_contact_points = self._request_contact_points(obj)
        contact_points = ContactPointsList([])
        body_link = None
        for point in multiverse_contact_points:
            if point.body_name == "world":
                continue
            body_object = self.get_object_by_name(point.body_name)
            if body_object is None:
                for obj in self.objects:
                    for link in obj.links.values():
                        if link.name == point.body_name:
                            body_link = link
                            break
            else:
                body_link = body_object.root_link
            if body_link is None:
                logging.error(f"Body link not found: {point.body_name}")
                raise ValueError
            contact_points.append(ContactPoint(obj.root_link, body_link))
            normal_force = self._get_normal_force_on_object_from_contact_force(obj, point.contact_force)
            contact_points[-1].normal_on_b = normal_force
            contact_points[-1].normal_force = normal_force
        return contact_points

    @staticmethod
    def _get_normal_force_on_object_from_contact_force(obj: Object, contact_force: List[float]) -> float:
        """
        Get the normal force on an object from the contact force exerted by another object that is expressed in the
        world frame. Thus transforming the contact force to the object frame is necessary.
        """
        obj_quat = obj.get_orientation_as_list()
        obj_rot_matrix = quaternion_matrix(obj_quat)[:3, :3]
        contact_force_array = obj_rot_matrix @ np.array(contact_force).reshape(3, 1)
        return contact_force_array.flatten().tolist()[2]

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> ContactPointsList:
        self.check_object_exists_and_issue_warning_if_not(obj1)
        self.check_object_exists_and_issue_warning_if_not(obj2)
        logging.warning("get_contact_points_between_two_objects is not implemented in Multiverse")
        return ContactPointsList([])

    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        logging.error("ray_test is not implemented in Multiverse")
        raise NotImplementedError

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        logging.error("ray_test_batch is not implemented in Multiverse")
        raise NotImplementedError

    def step(self):
        logging.warning("step is not implemented in Multiverse")

    def save_physics_simulator_state(self) -> int:
        logging.warning("save_physics_simulator_state is not implemented in Multiverse")
        return 0

    def remove_physics_simulator_state(self, state_id: int) -> None:
        logging.warning("remove_physics_simulator_state is not implemented in Multiverse")

    def restore_physics_simulator_state(self, state_id: int) -> None:
        logging.error("restore_physics_simulator_state is not implemented in Multiverse")
        raise NotImplementedError

    def set_link_color(self, link: Link, rgba_color: Color):
        self.check_object_exists_and_issue_warning_if_not(link.object)
        logging.warning("set_link_color is not implemented in Multiverse")

    def get_link_color(self, link: Link) -> Color:
        self.check_object_exists_and_issue_warning_if_not(link.object)
        logging.warning("get_link_color is not implemented in Multiverse")
        return Color()

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        self.check_object_exists_and_issue_warning_if_not(obj)
        logging.warning("get_colors_of_object_links is not implemented in Multiverse")
        return {}

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        self.check_object_exists_and_issue_warning_if_not(obj)
        logging.error("get_object_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        self.check_object_exists_and_issue_warning_if_not(link.object)
        logging.error("get_link_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def set_realtime(self, real_time: bool) -> None:
        logging.warning("set_realtime is not implemented in Multiverse")

    def set_gravity(self, gravity_vector: List[float]) -> None:
        logging.warning("set_gravity is not implemented in Multiverse")

    def check_object_exists_and_issue_warning_if_not(self, obj: Object):
        if obj not in self.objects:
            logging.warning(f"Object {obj.name} does not exist in the simulator")

    def check_object_exists_in_multiverse(self, object_name: str) -> bool:
        result = self._request_check_object_exists(object_name)[0]
        return result == "yes"

    def _request_check_object_exists(self, object_name: str) -> List[str]:
        api_data = self._get_object_exists_api_data(object_name)
        return self._request_single_api_callback(api_data)

    def _get_object_exists_api_data(self, object_name: str) -> APIData:
        return APIData(self.EXIST_API_NAME, [object_name])

    def _request_contact_points(self, obj: Object) -> List[MultiverseContactPoint]:
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
        self._init_api_callback()
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

    def _init_api_callback(self):
        """
        Initialize the API callback in the request metadata.
        """
        self._reset_request_meta_data()
        self._reset_api_callback()

    def _reset_api_callback(self):
        """
        Reset the API callback in the request metadata.
        """
        self.request_meta_data["api_callbacks"] = {self.simulation: []}

    def _set_simulation_in_request_meta_data(self):
        """
        Set the simulation name in the request metadata. (for e.g. name of simulator in the muv file)
        """
        self.request_meta_data["meta_data"]["simulation_name"] = self.simulation

    def _reset_request_meta_data(self):
        """
        Reset the request metadata.
        """
        self.request_meta_data = {
            "meta_data": self._meta_data.__dict__,
            "send": {},
            "receive": {},
        }