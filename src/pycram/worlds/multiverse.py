import logging
import os
from time import sleep, time

import numpy as np
from tf.transformations import quaternion_matrix
from typing_extensions import List, Dict, Optional

from .multiverse_communication.client_manager import MultiverseClientManager
from .multiverse_functions.error_checkers import PoseErrorChecker, MultiJointPositionErrorChecker, \
    RevoluteJointPositionErrorChecker, PrismaticJointPositionErrorChecker
from .multiverse_functions.goal_validator import GoalValidator
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

    def get_joint_position_name(self, joint: Joint) -> str:
        if joint.type not in self._joint_type_to_position_name:
            logging.warning(f"Invalid joint type: {joint.type}")
            return "joint_rvalue"
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

    def reset_joint_position(self, joint: Joint, joint_position: float) -> None:
        self.set_multiple_joint_positions({joint: joint_position})

    def set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> None:
        initial_joint_positions = {joint: self.get_joint_position(joint) for joint in joint_positions.keys()}
        data = {joint.name: {self.get_joint_position_name(joint): [position]}
                for joint, position in joint_positions.items()}
        self.writer.send_multiple_body_data_to_server(data)
        self._wait_until_multiple_joint_goals_are_achieved(joint_positions, initial_joint_positions)

    def _wait_until_multiple_joint_goals_are_achieved(self, joint_positions: Dict[Joint, float],
                                                      initial_joint_positions: Dict[Joint, float]) -> None:
        goal_validator = self._get_multi_joint_goal_validator(joint_positions, initial_joint_positions)
        self._wait_until_goal_is_achieved(goal_validator)

    def _get_multi_joint_goal_validator(self, joint_positions: Dict[Joint, float],
                                        initial_joint_positions: Dict[Joint, float]) -> GoalValidator:
        joints = list(joint_positions.keys())
        joint_types = [joint.type for joint in joints]
        target_joint_positions = list(joint_positions.values())
        initial_joint_positions = list(initial_joint_positions.values())
        goal_validator = GoalValidator(target_joint_positions,
                                       lambda: list(self.get_multiple_joint_positions(joints).values()),
                                       MultiJointPositionErrorChecker(joint_types),
                                       initial_value=initial_joint_positions)
        return goal_validator

    def get_joint_position(self, joint: Joint) -> float:
        data = self.get_multiple_joint_positions([joint])
        if data is not None:
            return data[joint.name]

    def get_multiple_joint_positions(self, joints: List[Joint]) -> Optional[Dict[str, float]]:
        self.check_object_exists_and_issue_warning_if_not(joints[0].object)
        joint_names = [joint.name for joint in joints]
        data = self.reader.get_multiple_body_data(joint_names, {joint.name: [self.get_joint_position_name(joint)]
                                                                for joint in joints})
        if data is not None:
            return {joint_name: list(data[joint_name].values())[0][0] for joint_name in joint_names}

    def _wait_until_joint_goal_is_achieved(self, joint: Joint, joint_position: float,
                                           initial_joint_position: float) -> None:
        error_checker = RevoluteJointPositionErrorChecker() if joint.type == JointType.REVOLUTE else \
            PrismaticJointPositionErrorChecker()
        goal_validator = GoalValidator(joint_position, lambda: self.get_joint_position(joint),
                                       error_checker, initial_value=initial_joint_position)
        self._wait_until_goal_is_achieved(goal_validator)

    def get_link_pose(self, link: Link) -> Pose:
        self.check_object_exists_and_issue_warning_if_not(link.object)
        return self._get_body_pose(link.name)

    def get_object_pose(self, obj: Object) -> Pose:
        self.check_object_exists_and_issue_warning_if_not(obj)
        if obj.obj_type == ObjectType.ENVIRONMENT:
            return Pose()
        return self._get_body_pose(obj.name)

    def reset_object_base_pose(self, obj: Object, pose: Pose):
        self.check_object_exists_and_issue_warning_if_not(obj)
        if obj.obj_type == ObjectType.ENVIRONMENT:
            return
        if (obj.obj_type == ObjectType.ROBOT and
                RobotDescription.current_robot_description.virtual_move_base_joints is not None):
            self.set_mobile_robot_pose(pose)
        else:
            initial_attached_objects_poses = list(obj.get_poses_of_attached_objects().values())
            self._set_body_pose(obj.name, pose)
            if len(initial_attached_objects_poses) > 0:
                self._wait_until_all_attached_objects_poses_are_set(obj, initial_attached_objects_poses)

    def _wait_until_all_attached_objects_poses_are_set(self, obj: Object, initial_poses: List[Pose]) -> None:
        """
        Wait until all attached objects are set to the desired poses.
        param obj: The object to which the attached objects belong.
        param initial_poses: The list of initial poses of the attached objects.
        """
        target_poses = obj.get_target_poses_of_attached_objects()
        body_names = [obj.name for obj in target_poses.keys()]
        self._wait_until_all_pose_goals_are_achieved(body_names, list(target_poses.values()),
                                                     initial_poses)

    def _wait_until_all_pose_goals_are_achieved(self, body_names: List[str], poses: List[Pose],
                                                initial_poses: List[Pose]) -> None:
        """
        Wait until all poses are set to the desired poses.
        param poses: The dictionary of the desired poses
        param initial_poses: The dictionary of the initial poses
        """
        goal_validator = GoalValidator(poses, lambda: list(self._get_multiple_body_poses(body_names).values()),
                                       PoseErrorChecker(), initial_value=initial_poses)
        self._wait_until_goal_is_achieved(goal_validator)

    def _get_body_pose(self, body_name: str, wait: Optional[bool] = True) -> Optional[Pose]:
        """
        Get the pose of a body in the simulator.
        param body_name: The name of the body.
        :param wait: Whether to wait until the pose is received.
        return: The pose of the body.
        """
        return self.reader.get_body_pose(body_name, wait)

    def get_multiple_object_poses(self, objects: List[Object]) -> Dict[str, Pose]:
        return self._get_multiple_body_poses([obj.name for obj in objects])

    def _get_multiple_body_poses(self, body_names: List[str]) -> Dict[str, Pose]:
        return self.reader.get_multiple_body_poses(body_names)

    def multiverse_reset_world(self):
        self.writer.reset_world()

    def _set_body_pose(self, body_name: str, pose: Pose) -> None:
        """
        Reset the pose of a body (object, link, or joint) in the simulator.
        param body_name: The name of the body.
        param pose: The pose of the body.
        """
        xyzw = pose.orientation_as_list()
        wxyz = [xyzw[3], *xyzw[:3]]
        initial_pose = self._get_body_pose(body_name, wait=False)
        self.writer.set_body_pose(body_name, pose.position_as_list(), wxyz)
        if initial_pose is not None:
            self._wait_until_pose_goal_is_achieved(body_name, pose, initial_pose)

    def _wait_until_pose_goal_is_achieved(self, body_name: str, target_pose: Pose, initial_pose: Pose):
        """
        Wait until the pose of a body is set.
        param body_name: The name of the body.
        param target_pose: The target pose of the body.
        param initial_pose: The initial pose of the body.
        """
        goal_validator = GoalValidator(target_pose, lambda: self._get_body_pose(body_name),
                                       PoseErrorChecker(), initial_value=initial_pose)
        self._wait_until_goal_is_achieved(goal_validator)

    def _wait_until_goal_is_achieved(self, goal_validator: GoalValidator) -> None:
        """
        Wait until the target is reached.
        param goal_validator: The goal validator object to validate and keep track of the goal achievement progress.
        """
        start_time = time()
        current = goal_validator.current_value
        while not goal_validator.goal_achieved:
            sleep(0.01)
            if time() - start_time > self.reader.MAX_WAIT_TIME_FOR_DATA:
                msg = f"Failed to achieve goal from initial error {goal_validator.initial_error} with" \
                      f" goal {goal_validator.goal_value} within {self.reader.MAX_WAIT_TIME_FOR_DATA}" \
                      f" seconds, the current value is {current}, error is {goal_validator.current_error}, percentage" \
                      f" of goal achieved is {goal_validator.percentage_of_goal_achieved}"
                logging.error(msg)
                raise TimeoutError(msg)
            current = goal_validator.current_value

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

        self.check_object_exists_and_issue_warning_if_not(constraint.parent_link.object)
        self.check_object_exists_and_issue_warning_if_not(constraint.child_link.object)

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
        self.check_object_exists_and_issue_warning_if_not(obj)
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
        self.check_object_exists_and_issue_warning_if_not(obj1)
        self.check_object_exists_and_issue_warning_if_not(obj2)
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
            msg = f"Object {obj} does not exist in the simulator"
            logging.warning(msg)

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
