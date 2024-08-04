import logging
import os
from time import sleep

import numpy as np
from tf.transformations import quaternion_matrix
from typing_extensions import List, Dict, Optional

from .multiverse_communication.client_manager import MultiverseClientManager
from .multiverse_communication.clients import MultiverseController, MultiverseReader, MultiverseWriter, MultiverseAPI
from .multiverse_datastructures.enums import MultiverseBodyProperty, MultiverseJointPosition, \
    MultiverseJointCMD
from .multiverse_extras.helpers import find_multiverse_resources_path
from ..datastructures.dataclasses import AxisAlignedBoundingBox, Color, ContactPointsList, ContactPoint
from ..datastructures.enums import WorldMode, JointType, ObjectType
from ..datastructures.pose import Pose
from ..datastructures.world import World
from ..description import Link, Joint
from ..robot_description import RobotDescription
from ..validation.goal_validator import validate_object_pose, validate_multiple_joint_positions, \
    validate_joint_position
from ..world_concepts.constraints import Constraint
from ..world_concepts.world_object import Object
from ..config import multiverse_conf as conf


class Multiverse(World):
    """
    This class implements an interface between Multiverse and PyCRAM.
    """

    supported_joint_types = (JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC)
    """
    A Tuple for the supported pycram joint types in Multiverse.
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

    use_bullet_mode: bool = conf.use_bullet_mode
    """
    If True, the simulation will always be in paused state unless the simulate() function is called, this behaves 
    similar to bullet_world which uses the bullet physics engine.
    """

    use_controller: bool = conf.use_controller and not use_bullet_mode
    """
    Whether to use the controller for the robot joints or not.
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

    def __init__(self, mode: Optional[WorldMode] = WorldMode.DIRECT,
                 is_prospection: Optional[bool] = False,
                 simulation_frequency: float = conf.simulation_frequency,
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

        World.__init__(self, mode, is_prospection, simulation_frequency, **conf.job_handling.as_dict(),
                       **conf.error_tolerance.as_dict())

        self.client_manager = MultiverseClientManager(self.simulation_wait_time_factor)
        self._init_clients()

        self._init_constraint_and_object_id_name_map_collections()

        if not self.is_prospection_world:
            self._spawn_floor()

        if self.use_bullet_mode:
            self.api_requester.pause_simulation()

    def _init_clients(self):
        self.reader: MultiverseReader = self.client_manager.create_reader(
            is_prospection_world=self.is_prospection_world)
        self.writer: MultiverseWriter = self.client_manager.create_writer(
            self.simulation,
            is_prospection_world=self.is_prospection_world)
        self.api_requester: MultiverseAPI = self.client_manager.create_api_requester(
            self.simulation,
            is_prospection_world=self.is_prospection_world)
        if self.use_controller:
            self.joint_controller: MultiverseController = self.client_manager.create_controller(
                is_prospection_world=self.is_prospection_world)

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

    @staticmethod
    def get_joint_position_name(joint: Joint) -> MultiverseJointPosition:
        return MultiverseJointPosition.from_pycram_joint_type(joint.type)

    def spawn_robot_with_controller(self, name: str, pose: Pose) -> None:
        """
        Spawn the robot in the simulator.
        param robot_description: The robot description.
        param pose: The pose of the robot.
        return: The object of the robot.
        """
        actuator_joint_commands = {
            actuator_name: [self.get_joint_cmd_name(self.robot_description.joint_types[joint_name]).value]
            for joint_name, actuator_name in self.robot_joint_actuators.items()
        }
        self.joint_controller.init_controller(actuator_joint_commands)
        self.writer.spawn_robot_with_actuators(name, pose.position_as_list(),
                                               self.xyzw_to_wxyz(pose.orientation_as_list()),
                                               actuator_joint_commands)

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

        # Do not spawn objects with type environment as they should be already present in the simulator through the
        # multiverse description file (.muv file).
        if not obj_type == ObjectType.ENVIRONMENT:
            self.spawn_object(name, obj_type, pose)

        return self._update_object_id_name_maps_and_get_latest_id(name)

    def spawn_object(self, name: str, object_type: ObjectType, pose: Pose) -> None:
        """
        Spawn the object in the simulator and return the object id.
        param obj: The object to be spawned.
        param pose: The pose of the object.
        return: The object id.
        """
        if object_type == ObjectType.ROBOT and self.use_controller:
            self.spawn_robot_with_controller(name, pose)
        else:
            self._set_body_pose(name, pose)

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
        return [joint.name for joint in obj.description.joints if joint.type in self.supported_joint_types]

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

    @validate_joint_position
    def reset_joint_position(self, joint: Joint, joint_position: float) -> bool:
        if self.use_controller and self.joint_has_actuator(joint):
            self._reset_joint_position_using_controller(joint, joint_position)
        else:
            self._set_multiple_joint_positions_without_controller({joint: joint_position})
        return True

    def _reset_joint_position_using_controller(self, joint: Joint, joint_position: float) -> bool:
        self.joint_controller.set_body_property(self.get_actuator_for_joint(joint),
                                                self.get_joint_cmd_name(joint.type),
                                                [joint_position])
        return True

    @validate_multiple_joint_positions
    def set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> bool:

        if self.use_controller:
            controlled_joints = self.get_controlled_joints(list(joint_positions.keys()))
            if len(controlled_joints) > 0:
                controlled_joint_positions = {joint: joint_positions[joint] for joint in controlled_joints}
                self._set_multiple_joint_positions_using_controller(controlled_joint_positions)
                joint_positions = {joint: joint_positions[joint] for joint in joint_positions.keys()
                                   if joint not in controlled_joints}
        if len(joint_positions) > 0:
            self._set_multiple_joint_positions_without_controller(joint_positions)

        return True

    def get_controlled_joints(self, joints: Optional[List[Joint]] = None) -> List[Joint]:
        joints = self.robot.joints if joints is None else joints
        return [joint for joint in joints if self.joint_has_actuator(joint)]

    def _set_multiple_joint_positions_without_controller(self, joint_positions: Dict[Joint, float]) -> None:
        joints_data = {joint.name: {self.get_joint_position_name(joint): [position]}
                       for joint, position in joint_positions.items()}
        self.writer.send_multiple_body_data_to_server(joints_data)

    def _set_multiple_joint_positions_using_controller(self, joint_positions: Dict[Joint, float]) -> bool:
        controlled_joints_data = {self.get_actuator_for_joint(joint):
                                  {self.get_joint_cmd_name(joint.type): [position]}
                                  for joint, position in joint_positions.items()}
        self.joint_controller.send_multiple_body_data_to_server(controlled_joints_data)
        return True

    def get_joint_position(self, joint: Joint) -> Optional[float]:
        joint_position_name = self.get_joint_position_name(joint)
        data = self.reader.get_body_data(joint.name, [joint_position_name])
        if data is not None:
            return data[joint_position_name.value][0]

    def get_multiple_joint_positions(self, joints: List[Joint]) -> Optional[Dict[str, float]]:
        joint_names = [joint.name for joint in joints]
        data = self.reader.get_multiple_body_data(joint_names, {joint.name: [self.get_joint_position_name(joint)]
                                                                for joint in joints})
        if data is not None:
            return {name: list(value.values())[0][0] for name, value in data.items()}

    @staticmethod
    def get_joint_cmd_name(joint_type: JointType) -> MultiverseJointCMD:
        return MultiverseJointCMD.from_pycram_joint_type(joint_type)

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
            self.set_mobile_robot_pose(obj, pose)
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
                                                        self.xyzw_to_wxyz(pose.orientation_as_list()),
                                                    MultiverseBodyProperty.RELATIVE_VELOCITY: [0.0]*6}
                                             for name, pose in body_poses.items()})

    def _get_body_pose(self, body_name: str, wait: Optional[bool] = True) -> Optional[Pose]:
        """
        Get the pose of a body in the simulator.
        param body_name: The name of the body.
        :param wait: Whether to wait until the pose is received.
        return: The pose of the body.
        """
        data = self.reader.get_body_pose(body_name, wait)
        return Pose(data[MultiverseBodyProperty.POSITION.value],
                    self.wxyz_to_xyzw(data[MultiverseBodyProperty.ORIENTATION.value]))

    @staticmethod
    def wxyz_to_xyzw(wxyz: List[float]) -> List[float]:
        """
        Convert a quaternion from WXYZ to XYZW format.
        """
        return [wxyz[1], wxyz[2], wxyz[3], wxyz[0]]

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
        self.step()

    def get_object_contact_points(self, obj: Object) -> ContactPointsList:
        """
        Note: Currently Multiverse only gets one contact point per contact objects.
        """
        if self.use_bullet_mode:
            # self.simulate(0.01)
            pass
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
        ray_test_result = self.ray_test_batch([from_position], [to_position])[0]
        return ray_test_result[0] if ray_test_result[0] != -1 else None

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[List[int]]:
        """
        Note: Currently, num_threads is not used in Multiverse.
        """
        ray_results = self.api_requester.get_objects_intersected_with_rays(from_positions, to_positions)
        results = []
        for ray_result in ray_results:
            results.append([])
            if ray_result.intersected():
                body_name = ray_result.body_name
                if body_name == "world":
                    results[-1].append(self.floor.id)
                elif body_name in self.object_name_to_id.keys():
                    results[-1].append(self.object_name_to_id[body_name])
                else:
                    for obj in self.objects:
                        if body_name in obj.links.keys():
                            results[-1].append(obj.id)
                            break
            else:
                results[-1].append(-1)
        return results

    def step(self):
        """
        Perform a simulation step in the simulator, this is useful when use_bullet_mode is True.
        """
        if self.use_bullet_mode:
            # self.api_requester.unpause_simulation()
            # sleep(self.simulation_time_step)
            # self.api_requester.pause_simulation()
            pass

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
        logging.warning("set_realtime is not implemented as an API in Multiverse, it is configured in the"
                        "multiverse configuration file (.muv file) as rtf_required where a value of 1 means real-time")

    def set_gravity(self, gravity_vector: List[float]) -> None:
        logging.warning("set_gravity is not implemented in Multiverse")

    def check_object_exists_in_multiverse(self, obj: Object) -> bool:
        return self.api_requester.check_object_exists(obj)
