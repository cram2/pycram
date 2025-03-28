import logging
import os
import shutil
from pathlib import Path
from time import sleep

import numpy as np
from ..tf_transformations import quaternion_matrix
from typing_extensions import List, Dict, Optional, Union, Tuple, Callable, Type

import pycrap
from pycrap import PhysicalObject
from .multiverse_communication.client_manager import MultiverseClientManager
from .multiverse_communication.clients import MultiverseController, MultiverseReader, MultiverseWriter, MultiverseAPI
from ..config.multiverse_conf import MultiverseConfig
from ..datastructures.dataclasses import Color, ContactPointsList, ContactPoint, RayResult
from ..datastructures.enums import WorldMode, JointType, MultiverseBodyProperty, MultiverseJointPosition, \
    MultiverseJointCMD
from ..datastructures.pose import PoseStamped
from ..datastructures.world import World
from ..datastructures.world_entity import PhysicalBody
from ..description import Link, Joint
from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription
from ..object_descriptors.mjcf import ObjectDescription as MJCF, PrimitiveObjectFactory
from ..robot_description import RobotDescription
from ..ros import  logwarn
from ..utils import RayTestUtils, wxyz_to_xyzw, xyzw_to_wxyz, adjust_camera_pose_based_on_target
from ..validation.goal_validator import validate_object_pose, validate_multiple_joint_positions, \
    validate_joint_position, validate_multiple_object_poses
from ..world_concepts.constraints import Constraint
from ..world_concepts.world_object import Object


class Multiverse(World):
    """
    This class implements an interface between Multiverse and PyCRAM.
    """

    conf: MultiverseConfig = MultiverseConfig
    """
    The Multiverse configuration.
    """

    supported_joint_types = (JointType.REVOLUTE, JointType.CONTINUOUS, JointType.PRISMATIC)
    """
    A Tuple for the supported pycram joint types in Multiverse.
    """

    added_multiverse_resources: bool = False
    """
    A flag to check if the multiverse resources have been added.
    """

    Object.extension_to_description_type[MJCF.get_file_extension()] = MJCF
    """
    Add the MJCF description extension to the extension to description type mapping for the objects.
    """

    def __init__(self, is_prospection: Optional[bool] = False,
                 clear_cache: bool = False):
        """
        Initialize the Multiverse Socket and the PyCram World.

        :param is_prospection: Whether the world is prospection or not.
        :param clear_cache: Whether to clear the cache or not.
        """

        self.latest_save_id: Optional[int] = None
        self.saved_simulator_states: Dict = {}
        self.make_sure_multiverse_resources_are_added(clear_cache=clear_cache)

        self.simulation = self.conf.prospection_world_prefix if is_prospection else "belief_state"
        self.client_manager = MultiverseClientManager(self.conf.simulation_wait_time_factor)
        self._init_clients(is_prospection=is_prospection)

        World.__init__(self, mode=WorldMode.DIRECT, is_prospection=is_prospection)

        self._init_constraint_and_object_id_name_map_collections()

        self.ray_test_utils = RayTestUtils(self.ray_test_batch, self.object_id_to_name)

        self.is_paused: bool = False

        if not self.is_prospection_world:
            self._spawn_floor()

        if self.conf.use_static_mode:
            self.pause_simulation()

    def _init_clients(self, is_prospection: bool = False):
        """
        Initialize the Multiverse clients that will be used to communicate with the Multiverse server.
        Each client is responsible for a specific task, e.g. reading data from the server, writing data to the serve,
         calling the API, or controlling the robot joints.

        :param is_prospection: Whether the world is prospection or not.
        """
        self.reader: MultiverseReader = self.client_manager.create_reader(
            is_prospection_world=is_prospection)
        self.writer: MultiverseWriter = self.client_manager.create_writer(
            self.simulation,
            is_prospection_world=is_prospection)
        self.api_requester: MultiverseAPI = self.client_manager.create_api_requester(
            self.simulation,
            is_prospection_world=is_prospection)
        if self.conf.use_controller:
            self.joint_controller: MultiverseController = self.client_manager.create_controller(
                is_prospection_world=is_prospection)

    def _init_constraint_and_object_id_name_map_collections(self):
        self.last_object_id: int = -1
        self.last_constraint_id: int = -1
        self.constraints: Dict[int, Constraint] = {}
        self.object_name_to_id: Dict[str, int] = {}
        self.object_id_to_name: Dict[int, str] = {}

    def _init_world(self, mode: WorldMode):
        pass

    @classmethod
    def make_sure_multiverse_resources_are_added(cls, clear_cache: bool = False) -> None:
        """
        Add the multiverse resources to the pycram world resources, and change the data directory and cache manager.

        :param clear_cache: Whether to clear the cache or not.
        """
        if not cls.added_multiverse_resources:
            if clear_cache:
                World.cache_manager.clear_cache()
            World.add_resource_path(cls.conf.resources_path, prepend=True)
            World.change_cache_dir_path(cls.conf.resources_path)
            cls.added_multiverse_resources = True

    def remove_multiverse_resources(self):
        """
        Remove the multiverse resources from the pycram world resources.
        """
        if self.added_multiverse_resources:
            World.remove_resource_path(self.conf.resources_path)
            World.change_cache_dir_path(self.conf.cache_dir)
            self.added_multiverse_resources = False

    def _spawn_floor(self):
        """
        Spawn the plane in the simulator.
        """
        self.floor = Object("floor", pycrap.Floor, "plane.urdf",
                            world=self)

    def pause_simulation(self) -> None:
        """
        Pause the simulation.
        """
        self.api_requester.pause_simulation()
        self.is_paused = True

    def unpause_simulation(self) -> None:
        """
        Unpause the simulation.
        """
        self.api_requester.unpause_simulation()
        self.is_paused = False

    def load_generic_object_and_get_id(self, description: GenericObjectDescription,
                                       pose: Optional[PoseStamped] = None) -> int:
        save_path = os.path.join(self.cache_manager.cache_dir, description.name + ".xml")
        object_factory = PrimitiveObjectFactory(description.name, description.links[0].geometry, save_path)
        object_factory.build_shape()
        object_factory.export_to_mjcf(save_path)
        return self.load_object_and_get_id(description.name, pose, pycrap.PhysicalObject)

    def get_images_for_target(self, target_pose: PoseStamped,
                              cam_pose: PoseStamped,
                              size: int = 256,
                              camera_min_distance: float = 0.1,
                              camera_max_distance: int = 3,
                              plot: bool = False) -> List[np.ndarray]:
        """
        Uses ray test to get the images for the target object. (target_pose is currently not used)
        """
        camera_description = RobotDescription.current_robot_description.get_default_camera()
        camera_frame = RobotDescription.current_robot_description.get_camera_frame(World.robot.name)
        adjusted_cam_pose = adjust_camera_pose_based_on_target(cam_pose, target_pose, camera_description)
        return self.ray_test_utils.get_images_for_target(adjusted_cam_pose, camera_description, camera_frame,
                                                         size, camera_min_distance, camera_max_distance, plot)

    @staticmethod
    def get_joint_position_name(joint: Joint) -> MultiverseJointPosition:
        """
        Get the attribute name of the joint position in the Multiverse from the pycram joint type.

        :param joint: The joint.
        """
        return MultiverseJointPosition.from_pycram_joint_type(joint.type)

    def spawn_robot_with_controller(self, name: str, pose: PoseStamped) -> None:
        """
        Spawn the robot in the simulator.

        :param name: The name of the robot.
        :param pose: The pose of the robot.
        """
        actuator_joint_commands = {
            actuator_name: [self.get_joint_cmd_name(self.robot_description.joint_types[joint_name]).value]
            for joint_name, actuator_name in self.robot_joint_actuators.items()
        }
        self.joint_controller.init_controller(actuator_joint_commands)
        self.writer.spawn_robot_with_actuators(name, actuator_joint_commands)
        if not pose.almost_equal(PoseStamped()):
            goal = self.robot.get_mobile_base_joint_goal(pose)
            self.set_multiple_joint_positions(goal)

    def load_object_and_get_id(self, name: Optional[str] = None,
                               pose: Optional[PoseStamped] = None,
                               obj_type: Optional[Type[PhysicalObject]] = None) -> int:
        """
        Spawn the object in the simulator and return the object id. Object name has to be unique and has to be same as
        the name of the object in the description file.

        :param name: The name of the object to be loaded.
        :param pose: The pose of the object.
        :param obj_type: The type of the object.
        """
        if pose is None:
            pose = PoseStamped()

        # Do not spawn objects with type environment as they should be already present in the simulator through the
        # multiverse description file (.muv file).
        obj_type = pycrap.PhysicalObject if obj_type is None else obj_type
        if not (issubclass(obj_type, pycrap.Location) or issubclass(obj_type, pycrap.Floor)):
            self.spawn_object(name, obj_type, pose)

        return self._update_object_id_name_maps_and_get_latest_id(name)

    def spawn_object(self, name: str, object_type: Type[PhysicalObject], pose: PoseStamped) -> None:
        """
        Spawn the object in the simulator.

        :param name: The name of the object.
        :param object_type: The type of the object.
        :param pose: The pose of the object.
        """
        if issubclass(object_type, pycrap.Robot):
            if self.conf.use_controller:
                self.spawn_robot_with_controller(name, pose)
            else:
                self.spawn_robot(name, pose)
        else:
            self._set_body_pose(name, pose)

    def spawn_robot(self, name: str, pose: PoseStamped) -> None:
        """
        Spawn the robot in the simulator.

        :param name: The name of the robot.
        :param pose: The pose of the robot.
        """
        self._set_body_pose(name, PoseStamped())
        self.robot.set_mobile_robot_pose(pose)

    def _update_object_id_name_maps_and_get_latest_id(self, name: str) -> int:
        """
        Update the object id name maps and return the latest object id.

        :param name: The name of the object.
        :return: The latest object id.
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
    def _reset_joint_position(self, joint: Joint, joint_position: float) -> bool:
        if not self.is_paused and self.conf.use_controller and self.joint_has_actuator(joint):
            self._reset_joint_position_using_controller(joint, joint_position)
        else:
            self._set_multiple_joint_positions_without_controller({joint: joint_position})
        return True

    def _reset_joint_position_using_controller(self, joint: Joint, joint_position: float) -> bool:
        """
        Reset the position of a joint in the simulator using the controller.

        :param joint: The joint.
        :param joint_position: The position of the joint.
        :return: True if the joint position is reset successfully.
        """
        self.joint_controller.set_body_property(self.get_actuator_for_joint(joint),
                                                self.get_joint_cmd_name(joint.type),
                                                [joint_position])
        return True

    @validate_multiple_joint_positions
    def _set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> bool:
        """
        Set the positions of multiple joints in the simulator. Also check if the joint is controlled by an actuator
        and use the controller to set the joint position if the joint is controlled.

        :param joint_positions: The dictionary of joints and positions.
        :return: True if the joint positions are set successfully (this means that the joint positions are set without
         errors, but not necessarily that the joint positions are set to the specified values).
        """

        if not self.is_paused and self.conf.use_controller:
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
        """
        Get the joints that are controlled by an actuator from the list of joints.

        :param joints: The list of joints to check.
        :return: The list of controlled joints.
        """
        joints = self.robot.joints if joints is None else joints
        return [joint for joint in joints if self.joint_has_actuator(joint)]

    def _set_multiple_joint_positions_without_controller(self, joint_positions: Dict[Joint, float]) -> None:
        """
        Set the positions of multiple joints in the simulator without using the controller.

        :param joint_positions: The dictionary of joints and positions.
        """
        joints_data = {joint.name: {self.get_joint_position_name(joint): [position]}
                       for joint, position in joint_positions.items()}
        self.writer.send_multiple_body_data_to_server(joints_data)

    def _set_multiple_joint_positions_using_controller(self, joint_positions: Dict[Joint, float]) -> bool:
        """
        Set the positions of multiple joints in the simulator using the controller.

        :param joint_positions: The dictionary of joints and positions.
        """
        controlled_joints_data = {self.get_actuator_for_joint(joint):
                                  {self.get_joint_cmd_name(joint.type): [position]}
                                  for joint, position in joint_positions.items()}
        self.joint_controller.send_multiple_body_data_to_server(controlled_joints_data)
        return True

    def _get_joint_position(self, joint: Joint) -> Optional[float]:
        joint_position_name = self.get_joint_position_name(joint)
        data = self.reader.get_body_data(joint.name, [joint_position_name])
        if data is not None:
            return data[joint_position_name.value][0]

    def _get_multiple_joint_positions(self, joints: List[Joint]) -> Optional[Dict[str, float]]:
        joint_names = [joint.name for joint in joints]
        data = self.reader.get_multiple_body_data(joint_names, {joint.name: [self.get_joint_position_name(joint)]
                                                                for joint in joints})
        if data is not None:
            return {name: list(value.values())[0][0] for name, value in data.items()}

    @staticmethod
    def get_joint_cmd_name(joint_type: JointType) -> MultiverseJointCMD:
        """
        Get the attribute name of the joint command in the Multiverse from the pycram joint type.

        :param joint_type: The pycram joint type.
        """
        return MultiverseJointCMD.from_pycram_joint_type(joint_type)

    def get_link_pose(self, link: Link) -> Optional[PoseStamped]:
        return self._get_body_pose(link.name)

    def get_multiple_link_poses(self, links: List[Link]) -> Dict[str, PoseStamped]:
        return self._get_multiple_body_poses([link.name for link in links])

    def get_object_pose(self, obj: Object) -> PoseStamped:
        if obj.is_an_environment:
            return PoseStamped()
        return self._get_body_pose(obj.name)

    def get_multiple_object_poses(self, objects: List[Object]) -> Dict[str, PoseStamped]:
        """
        Set the poses of multiple objects in the simulator. If the object is of type environment, the pose will be
        the default pose.

        :param objects: The list of objects.
        :return: The dictionary of object names and poses.
        """
        non_env_objects = [obj for obj in objects if not obj.is_an_environment]
        all_poses = self._get_multiple_body_poses([obj.name for obj in non_env_objects])
        all_poses.update({obj.name: PoseStamped() for obj in objects if obj.is_an_environment})
        return all_poses

    @validate_object_pose
    def reset_object_base_pose(self, obj: Object, pose: PoseStamped) -> bool:
        if obj.is_an_environment:
            return False

        if (obj.is_a_robot and
                RobotDescription.current_robot_description.virtual_mobile_base_joints is not None):
            obj.set_mobile_robot_pose(pose)
        else:
            self._set_body_pose(obj.name, pose)

        return True

    @validate_multiple_object_poses
    def reset_multiple_objects_base_poses(self, objects: Dict[Object, PoseStamped]) -> None:
        """
        Reset the poses of multiple objects in the simulator.

        :param objects: The dictionary of objects and poses.
        """
        for obj in objects.keys():
            if (obj.is_a_robot and
                    RobotDescription.current_robot_description.virtual_mobile_base_joints is not None):
                obj.set_mobile_robot_pose(objects[obj])
        objects = {obj: pose for obj, pose in objects.items()
                   if not obj.is_a_robot and not obj.is_an_environment}
        if len(objects) > 0:
            self._set_multiple_body_poses({obj.name: pose for obj, pose in objects.items()})

    def _set_body_pose(self, body_name: str, pose: PoseStamped) -> None:
        """
        Reset the pose of a body (object, link, or joint) in the simulator.

        :param body_name: The name of the body.
        :param pose: The pose of the body.
        """
        self._set_multiple_body_poses({body_name: pose})

    def _set_multiple_body_poses(self, body_poses: Dict[str, PoseStamped]) -> None:
        """
        Reset the poses of multiple bodies in the simulator.

        :param body_poses: The dictionary of body names and poses.
        """
        self.writer.set_multiple_body_poses({name: {MultiverseBodyProperty.POSITION: pose.position.to_list(),
                                                    MultiverseBodyProperty.ORIENTATION:
                                                        xyzw_to_wxyz(pose.orientation.to_list()()),
                                                    MultiverseBodyProperty.RELATIVE_VELOCITY: [0.0] * 6}
                                             for name, pose in body_poses.items()})

    def _get_body_pose(self, body_name: str, wait: Optional[bool] = True) -> Optional[PoseStamped]:
        """
        Get the pose of a body in the simulator.

        :param body_name: The name of the body.
        :param wait: Whether to wait until the pose is received.
        :return: The pose of the body.
        """
        data = self.reader.get_body_pose(body_name, wait)
        return PoseStamped(data[MultiverseBodyProperty.POSITION.value],
                           wxyz_to_xyzw(data[MultiverseBodyProperty.ORIENTATION.value]))

    def _get_multiple_body_poses(self, body_names: List[str]) -> Dict[str, PoseStamped]:
        """
        Get the poses of multiple bodies in the simulator.

        :param body_names: The list of body names.
        """
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
        """
        Reset the world using the Multiverse API.
        """
        self.writer.reset_world()

    def disconnect_from_physics_server(self) -> None:
        MultiverseClientManager.stop_all_clients()

    def join_threads(self) -> None:
        self.reader.stop_thread = True
        self.reader.join()

    def _remove_visual_object(self, obj_id: int) -> bool:
        logwarn("Currently multiverse does not create visual objects")
        return False

    def remove_object_from_simulator(self, obj: Object) -> bool:
        if not obj.is_an_environment:
            self.writer.remove_body(obj.name)
            return True
        logwarn("Cannot remove environment objects")
        return False

    def add_constraint(self, constraint: Constraint) -> int:

        if constraint.type != JointType.FIXED:
            logging.error("Only fixed constraints are supported in Multiverse")
            raise ValueError

        if not self.conf.let_pycram_move_attached_objects:
            self.api_requester.attach(constraint)

        return self._update_constraint_collection_and_get_latest_id(constraint)

    def _update_constraint_collection_and_get_latest_id(self, constraint: Constraint) -> int:
        """
        Update the constraint collection and return the latest constraint id.

        :param constraint: The constraint to be added.
        :return: The latest constraint id.
        """
        self.last_constraint_id += 1
        self.constraints[self.last_constraint_id] = constraint
        return self.last_constraint_id

    def remove_constraint(self, constraint_id) -> None:
        constraint = self.constraints.pop(constraint_id)
        self.api_requester.detach(constraint)

    def perform_collision_detection(self) -> None:
        ...

    def get_body_contact_points(self, body: PhysicalBody) -> ContactPointsList:
        if isinstance(body, Object):
            return self.get_object_contact_points(body)
        else:
            multiverse_contact_points = self.api_requester.get_contact_points(body.name)
            contact_points = ContactPointsList([])
            for mcp in multiverse_contact_points:
                body_object, body_link = self.get_object_with_body_name(mcp.body_2)
                obj, obj_link = self.get_object_with_body_name(mcp.body_1)
                contact_points.append(ContactPoint(obj_link, body_link))
                contact_points[-1].normal_on_body_b = mcp.normal
                contact_points[-1].position_on_b = mcp.position
            return contact_points

    def get_object_contact_points(self, obj: Object, ignore_attached_objects: bool = True) -> ContactPointsList:
        """
        Note: Currently Multiverse only gets one contact point per contact objects.

        :param obj: The object.
        :param ignore_attached_objects: Whether to ignore the attached objects or not.
        :return: The contact points of the object.
        """
        multiverse_contact_points = self.api_requester.get_contact_points(obj.name)
        if ignore_attached_objects and len(obj.attachments) > 0:
            attached_objects_link_names = []
            for att_obj in obj.attachments.keys():
                attached_objects_link_names.extend(att_obj.link_names)
            multiverse_contact_points = [mcp for mcp in multiverse_contact_points if
                                         mcp.body_1 not in attached_objects_link_names]
        contact_points = ContactPointsList([])
        for mcp in multiverse_contact_points:
            body_object, body_link = self.get_object_with_body_name(mcp.body_2)
            obj_link = obj.links[mcp.body_1] if mcp.body_1 in obj.links.keys() else obj.root_link
            contact_points.append(ContactPoint(obj_link, body_link))
            contact_points[-1].normal_on_body_b = mcp.normal
            contact_points[-1].position_on_b = mcp.position
        return contact_points

    def get_object_with_body_name(self, body_name: str) -> Tuple[Optional[Object], Optional[Link]]:
        """
        Get the object with the body name in the simulator, the body name can be the name of the object or the link.

        :param body_name: The name of the body.
        :return: The object if found otherwise None, and the link if the body name is a link.
        """
        if body_name == "world":
            body_name = "floor"
        body_object = self.get_object_by_name(body_name)
        body_link = None
        if body_object is None:
            for obj in self.objects:
                for link in obj.links.values():
                    if link.name == body_name:
                        body_link = link
                        body_object = obj
                        break
        else:
            body_link = body_object.root_link
        return body_object, body_link

    @staticmethod
    def _get_normal_force_on_object_from_contact_force(obj: Object, contact_force: List[float]) -> float:
        """
        Get the normal force on an object from the contact force exerted by another object that is expressed in the
        world frame. Thus transforming the contact force to the object frame is necessary.

        :param obj: The object.
        :param contact_force: The contact force.
        :return: The normal force on the object.
        """
        obj_quat = obj.get_orientation_as_list()
        obj_rot_matrix = quaternion_matrix(obj_quat)[:3, :3]
        # invert the rotation matrix to get the transformation from world to object frame
        obj_rot_matrix = np.linalg.inv(obj_rot_matrix)
        contact_force_array = obj_rot_matrix @ np.array(contact_force).reshape(3, 1)
        return contact_force_array.flatten().tolist()[2]

    def get_contact_points_between_two_bodies(self, obj1: Object, obj2: Object) -> ContactPointsList:
        obj1_contact_points = self.get_object_contact_points(obj1)
        return obj1_contact_points.get_points_of_object(obj2)

    def _ray_test(self, from_position: List[float], to_position: List[float]) -> RayResult:
        return self.ray_test_batch([from_position], [to_position])[0]

    def _ray_test_batch(self, from_positions: List[List[float]],
                        to_positions: List[List[float]],
                        num_threads: int = 1,
                        return_distance: bool = False) -> List[RayResult]:
        """
        Note: Currently, num_threads is not used in Multiverse.
        """
        ray_results = self.api_requester.get_objects_intersected_with_rays(from_positions, to_positions)
        results: List[RayResult] = []
        for ray_result in ray_results:
            results.append(RayResult(-1))
            if ray_result.intersected:
                body_name = ray_result.body_name
                if body_name == "world":
                    results[-1].obj_id = 0  # The floor id, which is always 0 since the floor is spawned first.
                elif body_name in self.object_name_to_id.keys():
                    results[-1].obj_id = self.object_name_to_id[body_name]
                else:
                    for obj in self.objects:
                        if body_name in obj.links.keys():
                            results[-1].obj_id = obj.id
                            break
            if return_distance:
                results[-1].distance = ray_result.distance
        return results

    def step(self, func: Optional[Callable[[], None]] = None, step_seconds: Optional[float] = None) -> None:
        """
        Perform a simulation step in the simulator, this is useful when use_static_mode is True.

        :param func: A function to be called after the simulation step.
        :param step_seconds: The number of seconds to step the simulation.
        """
        if self.conf.use_static_mode:
            self.unpause_simulation()
            if func is not None:
                func()
            sleep(self.simulation_time_step if step_seconds is None else step_seconds)
            self.pause_simulation()

    def save_physics_simulator_state(self, state_id: Optional[int] = None, use_same_id: bool = False) -> int:
        if state_id is None:
            self.latest_save_id = 0 if self.latest_save_id is None else self.latest_save_id + int(not use_same_id)
            state_id = self.latest_save_id
        save_name = f"save_{state_id}"
        self.saved_simulator_states[state_id] = self.api_requester.save(save_name)
        return state_id

    def remove_physics_simulator_state(self, state_id: int) -> None:
        save_path = self.saved_simulator_states.pop(state_id)
        shutil.rmtree(Path(save_path).parent)

    def restore_physics_simulator_state(self, state_id: int) -> None:
        self.api_requester.load(self.saved_simulator_states[state_id])

    def set_link_color(self, link: Link, rgba_color: Color):
        logwarn("set_link_color is not implemented in Multiverse")

    def get_link_color(self, link: Link) -> Color:
        logwarn("get_link_color is not implemented in Multiverse")
        return Color()

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        logwarn("get_colors_of_object_links is not implemented in Multiverse")
        return {}

    def set_realtime(self, real_time: bool) -> None:
        logwarn("set_realtime is not implemented as an API in Multiverse, it is configured in the"
                "multiverse configuration file (.muv file) as rtf_required where a value of 1 means real-time")

    def set_gravity(self, gravity_vector: List[float]) -> None:
        logwarn("set_gravity is not implemented in Multiverse")

    def check_object_exists(self, obj: Object) -> bool:
        """
        Check if the object exists in the Multiverse world.

        :param obj: The object.
        :return: True if the object exists, False otherwise.
        """
        return self.api_requester.check_object_exists(obj)
