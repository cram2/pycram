import logging
import os

import numpy as np
from tf.transformations import quaternion_matrix
from typing_extensions import List, Dict, Optional

from .multiverse_communication.client_manager import MultiverseClientManager
from .multiverse_datastructures.enums import MultiverseJointProperty, MultiverseBodyProperty
from .multiverse_functions.goal_validator import validate_object_pose, validate_multiple_joint_positions
from ..datastructures.dataclasses import AxisAlignedBoundingBox, Color, ContactPointsList, ContactPoint
from ..datastructures.enums import WorldMode, JointType, ObjectType
from ..datastructures.pose import Pose
from ..datastructures.world import World
from ..description import Link, Joint
from ..robot_description import RobotDescription
from ..world_concepts.constraints import Constraint
from ..world_concepts.world_object import Object


class Multiverse(World):
    """
    This class implements an interface between Multiverse and PyCRAM.
    """

    _joint_type_to_position_name: Dict[JointType, MultiverseJointProperty] = {
        JointType.REVOLUTE: MultiverseJointProperty.REVOLUTE_JOINT_POSITION,
        JointType.PRISMATIC: MultiverseJointProperty.PRISMATIC_JOINT_POSITION,
    }
    """
    A dictionary to map JointType to the corresponding multiverse attribute name.
    """

    added_multiverse_resources: bool = False
    """
    A flag to check if the multiverse resources have been added.
    """

    simulation: Optional[str] = None
    """
    The simulation name to be used in the Multiverse world (this is the name defined in
     the multiverse configuration file).
    """

    try:
        simulation_wait_time_factor = float(os.environ['Multiverse_Simulation_Wait_Time_Factor'])
    except KeyError:
        simulation_wait_time_factor = 1.0
    """
    The factor to multiply the simulation wait time with, this is used to adjust the simulation wait time to account for
    the time taken by the simulation to process the request, this depends on the computational power of the machine
    running the simulation.
    TODO: This should be replaced by a feedback mechanism that waits until a certain condition is met, e.g. the action
    is completed.
    """

    position_tol: float = 2e-3
    """
    The tolerance for position comparison. (e.g. for checking if the object has reached the desired position)
    """

    orientation_tol: float = 2e-3
    """
    The tolerance for orientation comparison. (e.g. for checking if the object has reached the desired orientation)
    """

    def __init__(self, mode: Optional[WorldMode] = WorldMode.DIRECT,
                 is_prospection: Optional[bool] = False,
                 simulation_frequency: Optional[float] = 60.0,
                 simulation: Optional[str] = None):
        """
        Initialize the Multiverse Socket and the PyCram World.
        param mode: The mode of the world (DIRECT or GUI).
        param is_prospection: Whether the world is prospection or not.
        param simulation_frequency: The frequency of the simulation.
        param client_addr: The address of the multiverse client.
        param simulation: The name of the simulation.
        """

        self._make_sure_multiverse_resources_are_added()

        if Multiverse.simulation is None:
            if simulation is None:
                logging.error("Simulation name not provided")
                raise ValueError("Simulation name not provided")
            Multiverse.simulation = simulation

        self.simulation = (self.prospection_world_prefix if is_prospection else "") + Multiverse.simulation

        World.__init__(self, mode, is_prospection, simulation_frequency)

        self._init_clients()

        self._set_world_job_flags()

        self._init_constraint_and_object_id_name_map_collections()

        if not self.is_prospection_world:
            self._spawn_floor()

    def _init_clients(self):
        client_manager = MultiverseClientManager(self.simulation_wait_time_factor)
        self.reader = client_manager.create_reader(is_prospection_world=self.is_prospection_world)
        self.writer = client_manager.create_writer(self.simulation, is_prospection_world=self.is_prospection_world)
        self.api_requester = client_manager.create_api_requester(self.simulation,
                                                                 is_prospection_world=self.is_prospection_world)

    def _set_world_job_flags(self):
        self.let_pycram_move_attached_objects = False
        self.let_pycram_handle_spawning = False
        self.update_poses_from_sim_on_get = True

    def _init_constraint_and_object_id_name_map_collections(self):
        self.last_object_id: int = -1
        self.last_constraint_id: int = -1
        self.constraints: Dict[int, Constraint] = {}
        self.object_name_to_id: Dict[str, int] = {}
        self.object_id_to_name: Dict[int, str] = {}

    def _init_world(self, mode: WorldMode):
        pass

    def _make_sure_multiverse_resources_are_added(self):
        """
        Add the multiverse resources to the pycram world resources.
        """
        if not self.added_multiverse_resources:
            World.cache_manager.clear_cache()
            dirname = find_multiverse_resources_path()
            World.data_directory = [dirname] + self.data_directory
            World.cache_manager.data_directory = World.data_directory
            self.added_multiverse_resources = True

    def _spawn_floor(self):
        """
        Spawn the plane in the simulator.
        """
        self.floor = Object("floor", ObjectType.ENVIRONMENT, "plane.urdf",
                            world=self)

    def get_joint_position_name(self, joint: Joint) -> MultiverseJointProperty:
        if joint.type not in self._joint_type_to_position_name:
            raise ValueError(f"Joint type {joint.type} is not supported in Multiverse")
        return self._joint_type_to_position_name[joint.type]

    def load_object_and_get_id(self, name: Optional[str] = None,
                               pose: Optional[Pose] = None,
                               obj_type: Optional[ObjectType] = None) -> int:
        """
        Spawn the object in the simulator and return the object id. Object name has to be unique and has to be same as
        the name of the object in the description file.
        param name: The name of the object to be loaded.
        param pose: The pose of the object.
        param obj_type: The type of the object.
        """
        if pose is None:
            pose = Pose()

        if not obj_type == ObjectType.ENVIRONMENT:
            self._set_body_pose(name, pose)

        return self._update_object_id_name_maps_and_get_latest_id(name)

    def _update_object_id_name_maps_and_get_latest_id(self, name: str) -> int:
        """
        Update the object id name maps and return the latest object id.
        param name: The name of the object.
        return: The latest object id.
        """
        self.last_object_id += 1
        self.object_name_to_id[name] = self.last_object_id
        self.object_id_to_name[self.last_object_id] = name
        return self.last_object_id

    def get_object_joint_names(self, obj: Object) -> List[str]:
        return [joint.name for joint in obj.description.joints
                if joint.type in self._joint_type_to_position_name.keys()]

    def get_object_link_names(self, obj: Object) -> List[str]:
        return [link.name for link in obj.description.links]

    def get_link_position(self, link: Link) -> List[float]:
        return self.reader.get_body_position(link.name)

    def get_link_orientation(self, link: Link) -> List[float]:
        return self.reader.get_body_orientation(link.name)

    def get_multiple_link_positions(self, links: List[Link]) -> Dict[str, List[float]]:
        return self.reader.get_multiple_body_positions([link.name for link in links])

    def get_multiple_link_orientations(self, links: List[Link]) -> Dict[str, List[float]]:
        return self.reader.get_multiple_body_orientations([link.name for link in links])

    @validate_multiple_joint_positions
    def set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> bool:
        data = {joint.name: {self.get_joint_position_name(joint): [position]}
                for joint, position in joint_positions.items()}
        self.writer.send_multiple_body_data_to_server(data)
        return True

    def get_joint_position(self, joint: Joint) -> Optional[float]:
        data = self.get_multiple_joint_positions([joint])
        if data is not None:
            return data[joint.name]

    def get_multiple_joint_positions(self, joints: List[Joint]) -> Optional[Dict[str, float]]:
        joint_names = [joint.name for joint in joints]
        data = self.reader.get_multiple_body_data(joint_names, {joint.name: [self.get_joint_position_name(joint)]
                                                                for joint in joints})
        if data is not None:
            return {name: list(value.values())[0][0] for name, value in data.items()}

    def get_link_pose(self, link: Link) -> Optional[Pose]:
        return self._get_body_pose(link.name)

    def get_multiple_link_poses(self, links: List[Link]) -> Dict[str, Pose]:
        return self._get_multiple_body_poses([link.name for link in links])

    def get_object_pose(self, obj: Object) -> Pose:
        if obj.obj_type == ObjectType.ENVIRONMENT:
            return Pose()
        return self._get_body_pose(obj.name)

    def get_multiple_object_poses(self, objects: List[Object]) -> Dict[str, Pose]:
        return self._get_multiple_body_poses([obj.name for obj in objects])

    @validate_object_pose
    def reset_object_base_pose(self, obj: Object, pose: Pose) -> bool:
        if obj.has_type_environment():
            return False

        if (obj.obj_type == ObjectType.ROBOT and
                RobotDescription.current_robot_description.virtual_move_base_joints is not None):
            self.set_mobile_robot_pose(pose)
        else:
            self._set_body_pose(obj.name, pose)

        return True

    def is_object_a_child_in_a_fixed_joint_constraint(self, obj: Object) -> bool:
        """
        Check if the object is a child in a fixed joint constraint. This means that the object is not free to move.
        It should be moved according to the parent object.
        :param obj: The object to check.
        :return: True if the object is a child in a fixed joint constraint, False otherwise.
        """
        constraints = list(self.constraints.values())
        for c in constraints:
            if c.child_link.object == obj and c.type == JointType.FIXED:
                return True
        return False

    def reset_multiple_objects_base_poses(self, objects: Dict[Object, Pose]) -> None:
        """
        Reset the poses of multiple objects in the simulator.
        param objects: The dictionary of objects and poses.
        """
        self._set_multiple_body_poses({obj.name: pose for obj, pose in objects.items()})

    def _set_body_pose(self, body_name: str, pose: Pose) -> None:
        """
        Reset the pose of a body (object, link, or joint) in the simulator.
        param body_name: The name of the body.
        param pose: The pose of the body.
        """
        self._set_multiple_body_poses({body_name: pose})

    def _set_multiple_body_poses(self, body_poses: Dict[str, Pose]) -> None:
        """
        Reset the poses of multiple bodies in the simulator.
        param body_poses: The dictionary of body names and poses.
        """
        self.writer.set_multiple_body_poses({name: {MultiverseBodyProperty.POSITION: pose.position_as_list(),
                                                    MultiverseBodyProperty.ORIENTATION:
                                                        self.xyzw_to_wxyz(pose.orientation_as_list())}
                                             for name, pose in body_poses.items()})

    def _get_body_pose(self, body_name: str, wait: Optional[bool] = True) -> Optional[Pose]:
        """
        Get the pose of a body in the simulator.
        param body_name: The name of the body.
        :param wait: Whether to wait until the pose is received.
        return: The pose of the body.
        """
        return self.reader.get_body_pose(body_name, wait)

    def _get_multiple_body_poses(self, body_names: List[str]) -> Dict[str, Pose]:
        return self.reader.get_multiple_body_poses(body_names)

    def get_multiple_object_positions(self, objects: List[Object]) -> Dict[str, List[float]]:
        return self.reader.get_multiple_body_positions([obj.name for obj in objects])

    def get_object_position(self, obj: Object) -> List[float]:
        return self.reader.get_body_position(obj.name)

    def get_multiple_object_orientations(self, objects: List[Object]) -> Dict[str, List[float]]:
        return self.reader.get_multiple_body_orientations([obj.name for obj in objects])

    def get_object_orientation(self, obj: Object) -> List[float]:
        return self.reader.get_body_orientation(obj.name)

    def multiverse_reset_world(self):
        self.writer.reset_world()

    @staticmethod
    def xyzw_to_wxyz(xyzw: List[float]) -> List[float]:
        return [xyzw[3], *xyzw[:3]]

    def disconnect_from_physics_server(self) -> None:
        MultiverseClientManager.stop_all_clients()

    def join_threads(self) -> None:
        self.reader.stop_thread = True
        self.reader.join()

    def remove_object_by_id(self, obj_id: int) -> None:
        obj = self.get_object_by_id(obj_id)
        self.remove_object_from_simulator(obj)

    def remove_object_from_simulator(self, obj: Object) -> None:
        if obj.obj_type != ObjectType.ENVIRONMENT:
            self.writer.remove_body(obj.name)

    def add_constraint(self, constraint: Constraint) -> int:

        if constraint.type != JointType.FIXED:
            logging.error("Only fixed constraints are supported in Multiverse")
            raise ValueError

        if not self.let_pycram_move_attached_objects:
            self.api_requester.attach(constraint)

        return self._update_constraint_collection_and_get_latest_id(constraint)

    def _update_constraint_collection_and_get_latest_id(self, constraint: Constraint) -> int:
        """
        Update the constraint collection and return the latest constraint id.
        param constraint: The constraint to be added.
        return: The latest constraint id.
        """
        self.last_constraint_id += 1
        self.constraints[self.last_constraint_id] = constraint
        return self.last_constraint_id

    def remove_constraint(self, constraint_id) -> None:
        constraint = self.constraints.pop(constraint_id)
        self.api_requester.detach(constraint)

    def perform_collision_detection(self) -> None:
        logging.warning("perform_collision_detection is not implemented in Multiverse")

    def get_object_contact_points(self, obj: Object) -> ContactPointsList:
        """
        Note: Currently Multiverse only gets one contact point per contact objects.
        """
        multiverse_contact_points = self.api_requester.get_contact_points(obj)
        contact_points = ContactPointsList([])
        body_link = None
        for point in multiverse_contact_points:
            if point.body_name == "world":
                point.body_name = "floor"
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
                raise ValueError(f"Body link not found: {point.body_name}")
            contact_points.append(ContactPoint(obj.root_link, body_link))
            # b_obj = body_link.object
            # normal_force_in_b_frame = self._get_normal_force_on_object_from_contact_force(b_obj, point.contact_force)
            contact_points[-1].force_x_in_world_frame = point.contact_force[0]
            contact_points[-1].force_y_in_world_frame = point.contact_force[1]
            contact_points[-1].force_z_in_world_frame = point.contact_force[2]
            contact_points[-1].normal_on_b = point.contact_force[2]
            contact_points[-1].normal_force = point.contact_force[2]
        return contact_points

    @staticmethod
    def _get_normal_force_on_object_from_contact_force(obj: Object, contact_force: List[float]) -> float:
        """
        Get the normal force on an object from the contact force exerted by another object that is expressed in the
        world frame. Thus transforming the contact force to the object frame is necessary.
        """
        obj_quat = obj.get_orientation_as_list()
        obj_rot_matrix = quaternion_matrix(obj_quat)[:3, :3]
        # invert the rotation matrix to get the transformation from world to object frame
        obj_rot_matrix = np.linalg.inv(obj_rot_matrix)
        contact_force_array = obj_rot_matrix @ np.array(contact_force).reshape(3, 1)
        return contact_force_array.flatten().tolist()[2]

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> ContactPointsList:
        obj1_contact_points = self.get_object_contact_points(obj1)
        return obj1_contact_points.get_points_of_object(obj2)

    def ray_test(self, from_position: List[float], to_position: List[float]) -> Optional[int]:
        ray_test_result = self.ray_test_batch([from_position], [to_position])
        return ray_test_result[0] if ray_test_result[0] != -1 else None

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        """
        Note: Currently, num_threads is not used in Multiverse.
        """
        ray_results = self.api_requester.get_objects_intersected_with_rays(from_positions, to_positions)
        results = []
        for ray_result in ray_results:
            if ray_result.intersected():
                results.append(self.floor.id if ray_result.body_name == "world" else
                               self.object_name_to_id[ray_result.body_name])
            else:
                results.append(-1)
        return results

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
        logging.warning("set_link_color is not implemented in Multiverse")

    def get_link_color(self, link: Link) -> Color:
        logging.warning("get_link_color is not implemented in Multiverse")
        return Color()

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        logging.warning("get_colors_of_object_links is not implemented in Multiverse")
        return {}

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        logging.error("get_object_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        logging.error("get_link_axis_aligned_bounding_box is not implemented in Multiverse")
        raise NotImplementedError

    def set_realtime(self, real_time: bool) -> None:
        logging.warning("set_realtime is not implemented in Multiverse")

    def set_gravity(self, gravity_vector: List[float]) -> None:
        logging.warning("set_gravity is not implemented in Multiverse")

    def check_object_exists_in_multiverse(self, obj: Object) -> bool:
        return self.api_requester.check_object_exists(obj)


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
