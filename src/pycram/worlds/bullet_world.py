# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import os
import threading
import time

import numpy as np
import pycram_bullet as p
import yaml
from typing_extensions import List, Optional, Dict, Any, Callable

from pycrap.ontologies import Floor
from ..datastructures.dataclasses import Color, AxisAlignedBoundingBox, MultiBody, VisualShape, BoxVisualShape, \
    ClosestPoint, LateralFriction, ContactPoint, ContactPointsList, ClosestPointsList, RayResult
from ..datastructures.enums import ObjectType, WorldMode, JointType
from ..datastructures.pose import PoseStamped, Point
from ..datastructures.world import World
from ..datastructures.world_entity import PhysicalBody
from ..object_descriptors.generic import ObjectDescription as GenericObjectDescription
from ..object_descriptors.urdf import ObjectDescription
from ..ros import  logwarn, loginfo
from ..validation.goal_validator import (validate_multiple_joint_positions, validate_joint_position,
                                         validate_object_pose, validate_multiple_object_poses)
from ..world_concepts.constraints import Constraint
from ..world_concepts.world_object import Object
from ..config.world_conf import WorldConfig

Link = ObjectDescription.Link
RootLink = ObjectDescription.RootLink
Joint = ObjectDescription.Joint


class BulletWorld(World):
    """
    This class represents a BulletWorld, which is a simulation environment that uses the Bullet Physics Engine. This
    class is the main interface to the Bullet Physics Engine and should be used to spawn Objects, simulate Physic and
    manipulate the Bullet World.
    """

    def __init__(self, mode: WorldMode = WorldMode.DIRECT, is_prospection: bool = False,
                 use_multiverse_for_real_world_simulation: bool = False):
        """
        Creates a new simulation, the type decides of the simulation should be a rendered window or just run in the
        background. There can only be one rendered simulation.
        The BulletWorld object also initializes the Events for attachment, detachment and for manipulating the world.

        :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default is "GUI"
        :param is_prospection: For internal usage, decides if this BulletWorld should be used as a shadow world.
        :param use_multiverse_for_real_world_simulation: Whether to use the Multiverse for real world simulation.
        """
        super().__init__(mode=mode, is_prospection=is_prospection)

        if use_multiverse_for_real_world_simulation:
            self.add_multiverse_resources()

        # This disables file caching from PyBullet, since this would also cache
        # files that can not be loaded
        p.setPhysicsEngineParameter(enableFileCaching=0)

        # Needed to let the other thread start the simulation, before Objects are spawned.
        time.sleep(0.1)
        self.vis_axis: List[int] = []

        # Some default settings
        self.set_gravity([0, 0, -9.8])

        if not is_prospection:
            _ = Object("floor", Floor, "plane.urdf",
                       world=self)

    def add_multiverse_resources(self):
        """
        Add the Multiverse resources to the start of the data directories of the BulletWorld such they are searched
        first for the resources because the pycram objects need to have the same description as the Multiverse objects.
        """
        try:
            from .multiverse import Multiverse
            Multiverse.make_sure_multiverse_resources_are_added(self.conf.clear_cache_at_start)
        except ImportError:
            logwarn("Multiverse is not installed, please install it to use the Multiverse.")

    def _init_world(self, mode: WorldMode):
        self._gui_thread: Gui = Gui(self, mode)
        self._gui_thread.start()
        time.sleep(0.1)

    def load_generic_object_and_get_id(self, description: GenericObjectDescription,
                                       pose: Optional[PoseStamped] = None) -> int:
        """
        Creates a visual and collision box in the simulation.
        """
        # Create visual shape
        vis_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=description.shape_data,
                                        rgbaColor=description.color.get_rgba(), physicsClientId=self.id)

        # Create collision shape
        col_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=description.shape_data, physicsClientId=self.id)

        # Create MultiBody with both visual and collision shapes
        obj_id = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=col_shape, baseVisualShapeIndex=vis_shape,
                                   basePosition=description.origin.position.to_list(),
                                   baseOrientation=description.origin.orientation.to_list(), physicsClientId=self.id)

        if pose is not None:
            self._set_object_pose_by_id(obj_id, pose)
        # Assuming you have a list to keep track of created objects
        return obj_id

    def load_object_and_get_id(self, path: Optional[str] = None, pose: Optional[PoseStamped] = None,
                               obj_type: Optional[ObjectType] = None) -> int:
        if pose is None:
            pose = PoseStamped()
        return self._load_object_and_get_id(path, pose)

    def _load_object_and_get_id(self, path: str, pose: PoseStamped) -> int:
        if path is None:
            raise ValueError("Path to the object file is required.")
        return p.loadURDF(path,
                          basePosition=pose.position.to_list(),
                          baseOrientation=pose.orientation.to_list(), physicsClientId=self.id)

    def _remove_visual_object(self, obj_id: int) -> bool:
        self._remove_body(obj_id)
        return True

    def remove_object_from_simulator(self, obj: Object) -> bool:
        self._remove_body(obj.id)
        return True

    def _remove_body(self, body_id: int) -> Any:
        """
        Remove a body from PyBullet using the body id.

        :param body_id: The id of the body.
        """
        return p.removeBody(body_id, self.id)

    def add_constraint(self, constraint: Constraint) -> int:

        constraint_id = p.createConstraint(constraint.parent_object_id,
                                           constraint.parent_link_id,
                                           constraint.child_object_id,
                                           constraint.child_link_id,
                                           constraint.type.value,
                                           constraint.axis_as_list,
                                           constraint.position_wrt_parent_as_list,
                                           constraint.position_wrt_child_as_list,
                                           constraint.orientation_wrt_parent_as_list,
                                           constraint.orientation_wrt_child_as_list,
                                           physicsClientId=self.id)
        return constraint_id

    def remove_constraint(self, constraint_id):
        p.removeConstraint(constraint_id, physicsClientId=self.id)

    def _get_joint_position(self, joint: ObjectDescription.Joint) -> float:
        return p.getJointState(joint.object_id, joint.id, physicsClientId=self.id)[0]

    def get_object_joint_names(self, obj: Object) -> List[str]:
        return [p.getJointInfo(obj.id, i, physicsClientId=self.id)[1].decode('utf-8')
                for i in range(self.get_object_number_of_joints(obj))]

    def get_multiple_link_poses(self, links: List[Link]) -> Dict[str, PoseStamped]:
        return {link.name: self.get_link_pose(link) for link in links}

    def get_multiple_link_positions(self, links: List[Link]) -> Dict[str, List[float]]:
        return {link.name: self.get_link_position(link) for link in links}

    def get_multiple_link_orientations(self, links: List[Link]) -> Dict[str, List[float]]:
        return {link.name: self.get_link_orientation(link) for link in links}

    def get_link_position(self, link: Link) -> List[float]:
        return self.get_link_pose(link).position.to_list()

    def get_link_orientation(self, link: Link) -> List[float]:
        return self.get_link_pose(link).orientation.to_list()

    def get_link_pose(self, link: ObjectDescription.Link) -> PoseStamped:
        bullet_link_state = p.getLinkState(link.object_id, link.id, physicsClientId=self.id)
        return PoseStamped.from_list(*bullet_link_state[4:6])

    def get_object_link_names(self, obj: Object) -> List[str]:
        num_links = self.get_object_number_of_links(obj)
        return [p.getJointInfo(obj.id, i, physicsClientId=self.id)[12].decode('utf-8')
                for i in range(num_links)]

    def get_object_number_of_links(self, obj: Object) -> int:
        return p.getNumJoints(obj.id, physicsClientId=self.id)

    get_object_number_of_joints = get_object_number_of_links

    def perform_collision_detection(self) -> None:
        p.performCollisionDetection(physicsClientId=self.id)

    def get_body_contact_points(self, body: PhysicalBody) -> ContactPointsList:
        self.perform_collision_detection()
        body_data = self.get_body_and_link_id(body, index='A')
        points_list = p.getContactPoints(**body_data, physicsClientId=self.id)
        return ContactPointsList([ContactPoint(**self.parse_points_list_to_args(point)) for point in points_list
                                  if len(point) > 0])

    def get_contact_points_between_two_bodies(self, obj_a: PhysicalBody, obj_b: PhysicalBody) -> ContactPointsList:
        self.perform_collision_detection()
        body_a_data = self.get_body_and_link_id(obj_a, index='A')
        body_b_data = self.get_body_and_link_id(obj_b, index='B')
        points_list = p.getContactPoints(**body_a_data, **body_b_data, physicsClientId=self.id)
        return ContactPointsList([ContactPoint(**self.parse_points_list_to_args(point)) for point in points_list
                                  if len(point) > 0])

    def get_body_closest_points(self, body: PhysicalBody, max_distance: float) -> ClosestPointsList:
        all_obj_closest_points = [self.get_closest_points_between_two_bodies(body, other_body, max_distance)
                                  for other_body in self.objects if other_body != body]
        return ClosestPointsList([point for closest_points in all_obj_closest_points for point in closest_points])

    def get_closest_points_between_two_bodies(self, obj_a: PhysicalBody, obj_b: PhysicalBody,
                                              max_distance: float) -> ClosestPointsList:
        body_a_data = self.get_body_and_link_id(obj_a, index='A')
        body_b_data = self.get_body_and_link_id(obj_b, index='B')
        points_list = p.getClosestPoints(**body_a_data, **body_b_data, distance=max_distance, physicsClientId=self.id)
        return ClosestPointsList([ClosestPoint(**self.parse_points_list_to_args(point)) for point in points_list
                                  if len(point) > 0])

    @staticmethod
    def get_body_and_link_id(body: PhysicalBody, index='') -> Dict[str, int]:
        body_id, link_id = (body.object_id, body.id) if isinstance(body, Link) else (body.id, None)
        values_dict = {f"body{index}": body_id}
        if link_id is not None:
            values_dict.update({f"linkIndex{index}": link_id})
        return values_dict

    def parse_points_list_to_args(self, point: List) -> Dict:
        """
        Parses the list of points to a list of dictionaries with the keys as the names of the arguments of the
        ContactPoint class.

        :param point: The list of points.
        """
        return {"body_a": self.get_object_by_id(point[1]).get_link_by_id(point[3]),
                "body_b": self.get_object_by_id(point[2]).get_link_by_id(point[4]),
                "position_on_body_a": point[5],
                "position_on_body_b": point[6],
                "normal_on_body_b": point[7],
                "distance": point[8],
                "normal_force": point[9],
                "lateral_friction_1": LateralFriction(point[10], point[11]),
                "lateral_friction_2": LateralFriction(point[12], point[13])}

    @validate_multiple_joint_positions
    def _set_multiple_joint_positions(self, joint_positions: Dict[Joint, float]) -> bool:
        for joint, joint_position in joint_positions.items():
            self.reset_joint_position(joint, joint_position)
        return True

    @validate_joint_position
    def _reset_joint_position(self, joint: Joint, joint_position: float) -> bool:
        p.resetJointState(joint.object_id, joint.id, joint_position, physicsClientId=self.id)
        return True

    def _get_multiple_joint_positions(self, joints: List[Joint]) -> Dict[str, float]:
        return {joint.name: self.get_joint_position(joint) for joint in joints}

    @validate_multiple_object_poses
    def reset_multiple_objects_base_poses(self, objects: Dict[Object, PoseStamped]) -> bool:
        for obj, pose in objects.items():
            self.reset_object_base_pose(obj, pose)
        return True

    @validate_object_pose
    def reset_object_base_pose(self, obj: Object, pose: PoseStamped) -> bool:
        return self._set_object_pose_by_id(obj.id, pose)

    def _set_object_pose_by_id(self, obj_id: int, pose: PoseStamped) -> bool:
        p.resetBasePositionAndOrientation(obj_id, pose.position.to_list(), pose.orientation.to_list(),
                                          physicsClientId=self.id)
        return True

    def step(self, func: Optional[Callable[[], None]] = None, step_seconds: Optional[float] = None) -> None:
        if func is not None:
            logwarn("The BulletWorld step function does not support a function argument.")
        if step_seconds is not None:
            logwarn("The BulletWorld step function does not support a step_seconds argument.")
        p.stepSimulation(physicsClientId=self.id)

    def get_multiple_object_poses(self, objects: List[Object]) -> Dict[str, PoseStamped]:
        return {obj.name: self.get_object_pose(obj) for obj in objects}

    def get_multiple_object_positions(self, objects: List[Object]) -> Dict[str, List[float]]:
        return {obj.name: self.get_object_pose(obj).position.to_list() for obj in objects}

    def get_multiple_object_orientations(self, objects: List[Object]) -> Dict[str, List[float]]:
        return {obj.name: self.get_object_pose(obj).orientation.to_list() for obj in objects}

    def get_object_position(self, obj: Object) -> List[float]:
        return self.get_object_pose(obj).position.to_list()

    def get_object_orientation(self, obj: Object) -> List[float]:
        return self.get_object_pose(obj).orientation.to_list()

    def get_object_pose(self, obj: Object) -> PoseStamped:
        return PoseStamped.from_list(*p.getBasePositionAndOrientation(obj.id, physicsClientId=self.id), frame="map")

    def set_link_color(self, link: ObjectDescription.Link, rgba_color: Color):
        p.changeVisualShape(link.object_id, link.id, rgbaColor=rgba_color.get_rgba(), physicsClientId=self.id)

    def get_link_color(self, link: ObjectDescription.Link) -> Color:
        return self.get_colors_of_object_links(link.object)[link.name]

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        visual_data = p.getVisualShapeData(obj.id, physicsClientId=self.id)
        link_id_to_name = {v: k for k, v in obj.link_name_to_id.items()}
        links = list(map(lambda x: link_id_to_name[x[1]], visual_data))
        colors = list(map(lambda x: Color.from_rgba(x[7]), visual_data))
        link_to_color = dict(zip(links, colors))
        return link_to_color

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        return AxisAlignedBoundingBox.from_min_max(*p.getAABB(obj.id, physicsClientId=self.id))

    def get_link_axis_aligned_bounding_box(self, link: ObjectDescription.Link) -> AxisAlignedBoundingBox:
        return AxisAlignedBoundingBox.from_min_max(*p.getAABB(link.object_id, link.id, physicsClientId=self.id))

    def set_realtime(self, real_time: bool) -> None:
        p.setRealTimeSimulation(1 if real_time else 0, physicsClientId=self.id)

    def set_gravity(self, gravity_vector: List[float]) -> None:
        p.setGravity(gravity_vector[0], gravity_vector[1], gravity_vector[2], physicsClientId=self.id)

    def disconnect_from_physics_server(self):
        p.disconnect(physicsClientId=self.id)

    def join_threads(self):
        """
        Joins the GUI thread if it exists.
        """
        self.join_gui_thread_if_exists()

    def join_gui_thread_if_exists(self):
        if self._gui_thread:
            self._gui_thread.join()

    def save_physics_simulator_state(self, state_id: Optional[int] = None, use_same_id: bool = False) -> int:
        return p.saveState(physicsClientId=self.id)

    def restore_physics_simulator_state(self, state_id):
        p.restoreState(state_id, physicsClientId=self.id)

    def remove_physics_simulator_state(self, state_id: int):
        p.removeState(state_id, physicsClientId=self.id)

    def _add_vis_axis(self, pose: PoseStamped,
                      length: Optional[float] = 0.2) -> int:
        """
        Creates a Visual object which represents the coordinate frame at the given
        position and orientation. There can be an unlimited amount of vis axis objects.

        :param pose: The pose at which the axis should be spawned
        :param length: Optional parameter to configure the length of the axes
        :return: The id of the spawned object
        """

        pose_in_map = self.local_transformer.transform_pose(pose, "map")

        box_vis_shape = BoxVisualShape(Color(1, 0, 0, 0.8), [length, 0.01, 0.01], [length, 0.01, 0.01])
        vis_x = self.create_visual_shape(box_vis_shape)

        box_vis_shape = BoxVisualShape(Color(0, 1, 0, 0.8), [0.01, length, 0.01], [0.01, length, 0.01])
        vis_y = self.create_visual_shape(box_vis_shape)

        box_vis_shape = BoxVisualShape(Color(0, 0, 1, 0.8), [0.01, 0.01, length], [0.01, 0.01, length])
        vis_z = self.create_visual_shape(box_vis_shape)

        multibody = MultiBody(base_visual_shape_index=-1, base_pose=pose_in_map,
                              link_visual_shape_indices=[vis_x, vis_y, vis_z],
                              link_poses=[PoseStamped(), PoseStamped(), PoseStamped()], link_masses=[1.0, 1.0, 1.0],
                              link_inertial_frame_poses=[PoseStamped(), PoseStamped(), PoseStamped()], link_parent_indices=[0, 0, 0],
                              link_joint_types=[JointType.FIXED.value, JointType.FIXED.value, JointType.FIXED.value],
                              link_joint_axis=[Point(x=1.0, y=0.0, z=0.0), Point(x=0.0, y=1.0, z=0.0), Point(x=0.0, y=0.0, z=1.0)],
                              link_collision_shape_indices=[-1, -1, -1])

        body_id = self._create_multi_body(multibody)
        self.vis_axis.append(body_id)
        return body_id

    def _remove_vis_axis(self) -> None:
        """
        Removes all spawned vis axis objects that are currently in this BulletWorld.
        """
        for vis_id in self.vis_axis:
            p.removeBody(vis_id, physicsClientId=self.id)
        self.vis_axis = []

    def _ray_test(self, from_position: List[float], to_position: List[float]) -> RayResult:
        res = p.rayTest(from_position, to_position, physicsClientId=self.id)
        return RayResult(*res[0])

    def _ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                        num_threads: int = 1) -> List[RayResult]:
        result = p.rayTestBatch(from_positions, to_positions, numThreads=num_threads,
                                physicsClientId=self.id)
        return [RayResult(*r) for r in result] if result else None

    def _create_visual_shape(self, visual_shape: VisualShape) -> int:
        return p.createVisualShape(visual_shape.visual_geometry_type.value,
                                   rgbaColor=visual_shape.rgba_color.get_rgba(),
                                   visualFramePosition=visual_shape.visual_frame_position,
                                   physicsClientId=self.id, **visual_shape.shape_data())

    def _create_multi_body(self, multi_body: MultiBody) -> int:
        return p.createMultiBody(baseVisualShapeIndex=-multi_body.base_visual_shape_index,
                                 linkVisualShapeIndices=multi_body.link_visual_shape_indices,
                                 basePosition=multi_body.base_pose.position.to_list(),
                                 baseOrientation=multi_body.base_pose.orientation.to_list(),
                                 linkPositions=[pose.position.to_list() for pose in multi_body.link_poses],
                                 linkMasses=multi_body.link_masses,
                                 linkOrientations=[pose.orientation.to_list() for pose in multi_body.link_poses],
                                 linkInertialFramePositions=[pose.position.to_list()
                                                             for pose in multi_body.link_inertial_frame_poses],
                                 linkInertialFrameOrientations=[pose.orientation.to_list()
                                                                for pose in multi_body.link_inertial_frame_poses],
                                 linkParentIndices=multi_body.link_parent_indices,
                                 linkJointTypes=multi_body.link_joint_types,
                                 linkJointAxis=[[point.x, point.y, point.z] for point in multi_body.link_joint_axis],
                                 linkCollisionShapeIndices=multi_body.link_collision_shape_indices)

    def get_images_for_target(self,
                              target_pose: PoseStamped,
                              cam_pose: PoseStamped,
                              size: Optional[int] = 256) -> List[np.ndarray]:
        # TODO: Might depend on robot cameras, if so please add these camera parameters to RobotDescription object
        # TODO: of your robot with a CameraDescription object.
        fov = 90
        aspect = size / size
        near = 0.2
        far = 100

        view_matrix = p.computeViewMatrix(cam_pose.position.to_list(), target_pose.position.to_list(), [0, 0, 1])
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        return list(p.getCameraImage(size, size, view_matrix, projection_matrix,
                                     physicsClientId=self.id))[2:5]

    def _add_text(self, text: str, position: List[float], orientation: Optional[List[float]] = None,
                  size: Optional[float] = None, color: Optional[Color] = Color(), life_time: Optional[float] = 0,
                  parent_object_id: Optional[int] = None, parent_link_id: Optional[int] = None) -> int:
        args = {}
        if orientation:
            args["textOrientation"] = orientation
        if size:
            args["textSize"] = size
        if life_time:
            args["lifeTime"] = life_time
        if parent_object_id:
            args["parentObjectUniqueId"] = parent_object_id
        if parent_link_id:
            args["parentLinkIndex"] = parent_link_id
        return p.addUserDebugText(text, position, color.get_rgb(), physicsClientId=self.id, **args)

    def _remove_text(self, text_id: Optional[int] = None) -> None:
        if text_id is not None:
            p.removeUserDebugItem(text_id, physicsClientId=self.id)
        else:
            p.removeAllUserDebugItems(physicsClientId=self.id)

    def enable_joint_force_torque_sensor(self, obj: Object, fts_joint_idx: int) -> None:
        p.enableJointForceTorqueSensor(obj.id, fts_joint_idx, enableSensor=1, physicsClientId=self.id)

    def disable_joint_force_torque_sensor(self, obj: Object, joint_id: int) -> None:
        p.enableJointForceTorqueSensor(obj.id, joint_id, enableSensor=0, physicsClientId=self.id)

    def get_joint_reaction_force_torque(self, obj: Object, joint_id: int) -> List[float]:
        return p.getJointState(obj.id, joint_id, physicsClientId=self.id)[2]

    def get_applied_joint_motor_torque(self, obj: Object, joint_id: int) -> float:
        return p.getJointState(obj.id, joint_id, physicsClientId=self.id)[3]


class Gui(threading.Thread):
    """
    For internal use only. Creates a new thread for the physics simulation that is active until closed by
    :func:`~World.exit`
    Also contains the code for controlling the camera.
    """

    def __init__(self, world: World, mode: WorldMode):
        threading.Thread.__init__(self)
        self.world = world
        self.mode: WorldMode = mode
        self.camera_button_id = -1

        # Checks if there is a display connected to the system. If not, the simulation will be run in direct mode.
        if "DISPLAY" not in os.environ:
            loginfo("No display detected. Running the simulation in direct mode.")
            self.mode = WorldMode.DIRECT

    def run(self):
        """
        Initializes the new simulation and checks in an endless loop
        if it is still active. If it is the thread will be suspended for 1/80 seconds, if it is not the method and
        thus the thread terminates. The loop also checks for mouse and keyboard inputs to control the camera.
        """
        if self.mode == WorldMode.DIRECT:
            self.world.id = p.connect(p.DIRECT)
        else:
            self.world.id = p.connect(p.GUI)
            self.camera_button_id = p.addUserDebugParameter("Save as Default Camera", 1, 0, 1, physicsClientId=self.world.id)

            # DISCLAIMER
            # This camera control only works if the WorldMode.GUI BulletWorld is the first one to be created. This is
            # due to a bug in the function pybullet.getDebugVisualizerCamera() which only returns the information of
            # the first created simulation.

            # Disable the side windows of the GUI
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.world.id)
            # Change the init camera pose
            default_camera_config = WorldConfig.default_camera_config
            p.resetDebugVisualizerCamera(cameraDistance=default_camera_config["dist"],
                                         cameraYaw=default_camera_config["yaw"],
                                         cameraPitch=default_camera_config["pitch"],
                                         cameraTargetPosition=default_camera_config["target_position"],
                                         physicsClientId=self.world.id)

            # Get the initial camera target location
            camera_target_position = p.getDebugVisualizerCamera(physicsClientId=self.world.id)[11]

            sphere_visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1],
                                                   physicsClientId=self.world.id)

            # Create a sphere with a radius of 0.05 and a mass of 0
            sphere_uid = p.createMultiBody(baseMass=0.0,
                                           baseInertialFramePosition=[0, 0, 0],
                                           baseVisualShapeIndex=sphere_visual_id,
                                           basePosition=camera_target_position,
                                           physicsClientId=self.world.id)

            # Define the maxSpeed, used in calculations
            max_speed = 16

            # Set initial Camera Rotation
            camera_yaw = default_camera_config["yaw"]
            camera_pitch = default_camera_config["pitch"]

            # Keep track of the mouse state
            mouse_state = [0, 0, 0]
            old_mouse_x, old_mouse_y = 0, 0

            # Determines if the sphere at cameraTargetPosition is visible
            visible = 1

            # Initial value for the camera button
            last_button_value = 1

            # Loop to update the camera position based on keyboard events
            while p.isConnected(self.world.id):
                # Check if Camera button was pressed
                camera_button_value = p.readUserDebugParameter(self.camera_button_id)
                if camera_button_value != last_button_value:
                    last_button_value = camera_button_value

                    current_camera_config = p.getDebugVisualizerCamera()[8:]
                    v = dict(zip(["yaw", "pitch", "dist", "target_position"], current_camera_config))
                    v["target_position"] = list(v["target_position"])
                    yaml_path = os.path.join(os.path.dirname(__file__), "..", "config", 'camera.yaml')
                    with open(yaml_path, "w") as f:
                        yaml.dump(v, f)


                # Monitor user input
                keys = p.getKeyboardEvents(self.world.id)
                mouse = p.getMouseEvents(self.world.id)

                # Get infos about the camera
                width, height, dist = (p.getDebugVisualizerCamera()[0],
                                       p.getDebugVisualizerCamera()[1],
                                       p.getDebugVisualizerCamera()[10])
                # print("width: ", width, "height: ", height, "dist: ", dist)
                camera_target_position = p.getDebugVisualizerCamera(self.world.id)[11]

                # Get vectors used for movement on x,y,z Vector
                x_vec = [p.getDebugVisualizerCamera(self.world.id)[2][i] for i in [0, 4, 8]]
                y_vec = [p.getDebugVisualizerCamera(self.world.id)[2][i] for i in [2, 6, 10]]
                z_vec = (0, 0, 1)  # [p.getDebugVisualizerCamera()[2][i] for i in [1, 5, 9]]

                # Check the mouse state
                if mouse:
                    for m in mouse:

                        mouse_x = m[2]
                        mouse_y = m[1]

                        # update mouseState
                        # Left Mouse button
                        if m[0] == 2 and m[3] == 0:
                            mouse_state[0] = m[4]
                        # Middle mouse button (scroll wheel)
                        if m[0] == 2 and m[3] == 1:
                            mouse_state[1] = m[4]
                        # right mouse button
                        if m[0] == 2 and m[3] == 2:
                            mouse_state[2] = m[4]

                        # change visibility by clicking the mousewheel
                        # if m[4] == 6 and m[3] == 1 and visible == 1:
                        #     visible = 0
                        # elif m[4] == 6 and visible == 0:
                        #     visible = 1

                        # camera movement when the left mouse button is pressed
                        if mouse_state[0] == 3:
                            speed_x = abs(old_mouse_x - mouse_x) if (abs(old_mouse_x - mouse_x)) < max_speed \
                                else max_speed
                            speed_y = abs(old_mouse_y - mouse_y) if (abs(old_mouse_y - mouse_y)) < max_speed \
                                else max_speed

                            # max angle of 89.5 and -89.5 to make sure the camera does not flip (is annoying)
                            if mouse_x < old_mouse_x:
                                if (camera_pitch + speed_x) < 89.5:
                                    camera_pitch += (speed_x / 4) + 1
                            elif mouse_x > old_mouse_x:
                                if (camera_pitch - speed_x) > -89.5:
                                    camera_pitch -= (speed_x / 4) + 1

                            if mouse_y < old_mouse_y:
                                camera_yaw += (speed_y / 4) + 1
                            elif mouse_y > old_mouse_y:
                                camera_yaw -= (speed_y / 4) + 1

                        # Camera movement when the middle mouse button is pressed
                        if mouse_state[1] == 3:
                            speed_x = abs(old_mouse_x - mouse_x)
                            factor = 0.05

                            if mouse_x < old_mouse_x:
                                dist = dist - speed_x * factor
                            elif mouse_x > old_mouse_x:
                                dist = dist + speed_x * factor
                            dist = max(dist, 0.1)

                        # camera movement when the right mouse button is pressed
                        if mouse_state[2] == 3:
                            speed_x = abs(old_mouse_x - mouse_x) if (abs(old_mouse_x - mouse_x)) < 5 else 5
                            speed_y = abs(old_mouse_y - mouse_y) if (abs(old_mouse_y - mouse_y)) < 5 else 5
                            factor = 0.05

                            if mouse_x < old_mouse_x:
                                camera_target_position = np.subtract(camera_target_position,
                                                                     np.multiply(np.multiply(z_vec, factor), speed_x))
                            elif mouse_x > old_mouse_x:
                                camera_target_position = np.add(camera_target_position,
                                                                np.multiply(np.multiply(z_vec, factor), speed_x))

                            if mouse_y < old_mouse_y:
                                camera_target_position = np.add(camera_target_position,
                                                                np.multiply(np.multiply(x_vec, factor), speed_y))
                            elif mouse_y > old_mouse_y:
                                camera_target_position = np.subtract(camera_target_position,
                                                                     np.multiply(np.multiply(x_vec, factor), speed_y))
                        # update oldMouse values
                        old_mouse_y, old_mouse_x = mouse_y, mouse_x

                # check the keyboard state
                if keys:
                    # if shift is pressed, double the speed
                    if p.B3G_SHIFT in keys:
                        speed_mult = 5
                    else:
                        speed_mult = 2.5

                    # if control is pressed, the movements caused by the arrowkeys, the '+' as well as the '-' key
                    # change
                    if p.B3G_CONTROL in keys:

                        # the up and down arrowkeys cause the targetPos to move along the z axis of the map
                        if p.B3G_DOWN_ARROW in keys:
                            camera_target_position = np.subtract(camera_target_position,
                                                                 np.multiply(np.multiply(z_vec, 0.03), speed_mult))
                        elif p.B3G_UP_ARROW in keys:
                            camera_target_position = np.add(camera_target_position,
                                                            np.multiply(np.multiply(z_vec, 0.03), speed_mult))

                        # left and right arrowkeys cause the targetPos to move horizontally relative to the camera
                        if p.B3G_LEFT_ARROW in keys:
                            camera_target_position = np.subtract(camera_target_position,
                                                                 np.multiply(np.multiply(x_vec, 0.03), speed_mult))
                        elif p.B3G_RIGHT_ARROW in keys:
                            camera_target_position = np.add(camera_target_position,
                                                            np.multiply(np.multiply(x_vec, 0.03), speed_mult))

                        # the '+' and '-' keys cause the targetpos to move forwards and backwards relative to the camera
                        # while the camera stays at a constant distance. SHIFT + '=' is for US layout
                        if ord("+") in keys or p.B3G_SHIFT in keys and ord("=") in keys:
                            camera_target_position = np.subtract(camera_target_position,
                                                                 np.multiply(np.multiply(y_vec, 0.03), speed_mult))
                        elif ord("-") in keys:
                            camera_target_position = np.add(camera_target_position,
                                                            np.multiply(np.multiply(y_vec, 0.03), speed_mult))

                    # standard bindings for thearrowkeys, the '+' as well as the '-' key
                    else:

                        # left and right arrowkeys cause the camera to rotate around the yaw axis
                        if p.B3G_RIGHT_ARROW in keys:
                            camera_yaw += (360 / width) * speed_mult
                        elif p.B3G_LEFT_ARROW in keys:
                            camera_yaw -= (360 / width) * speed_mult

                        # the up and down arrowkeys cause the camera to rotate around the pitch axis
                        if p.B3G_DOWN_ARROW in keys:
                            if (camera_pitch + (360 / height) * speed_mult) < 89.5:
                                camera_pitch += (360 / height) * speed_mult
                        elif p.B3G_UP_ARROW in keys:
                            if (camera_pitch - (360 / height) * speed_mult) > -89.5:
                                camera_pitch -= (360 / height) * speed_mult

                        # the '+' and '-' keys cause the camera to zoom towards and away from the targetPos without
                        # moving it. SHIFT + '=' is for US layout since the events can't handle shift plus something
                        if ord("+") in keys or p.B3G_SHIFT in keys and ord("=") in keys:
                            if (dist - (dist * 0.02) * speed_mult) > 0.1:
                                dist -= dist * 0.02 * speed_mult
                        elif ord("-") in keys:
                            dist += dist * 0.02 * speed_mult
                # print("dist: ", dist)
                # print("camera_yaw: ", camera_yaw)
                # print("camera_pitch: ", camera_pitch)
                # print("camera_target_position: ", camera_target_position)

                p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=camera_yaw, cameraPitch=camera_pitch,
                                             cameraTargetPosition=camera_target_position, physicsClientId=self.world.id)
                if visible == 0:
                    camera_target_position = (0.0, -50, 50)
                p.resetBasePositionAndOrientation(sphere_uid, camera_target_position, [0, 0, 0, 1],
                                                  physicsClientId=self.world.id)
                time.sleep(1. / 80.)
