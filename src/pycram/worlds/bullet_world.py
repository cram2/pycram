# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import threading
import time

import numpy as np
import pybullet as p
import rosgraph
import rospy
from geometry_msgs.msg import Point
from typing_extensions import List, Optional, Dict

from ..datastructures.enums import ObjectType, WorldMode, JointType
from ..datastructures.pose import Pose
from ..object_descriptors.urdf import ObjectDescription
from ..datastructures.world import World
from ..world_concepts.constraints import Constraint
from ..datastructures.dataclasses import Color, AxisAlignedBoundingBox, MultiBody, VisualShape, BoxVisualShape
from ..world_concepts.world_object import Object

Link = ObjectDescription.Link
RootLink = ObjectDescription.RootLink
Joint = ObjectDescription.Joint


class BulletWorld(World):
    """
    This class represents a BulletWorld, which is a simulation environment that uses the Bullet Physics Engine. This
    class is the main interface to the Bullet Physics Engine and should be used to spawn Objects, simulate Physic and
    manipulate the Bullet World.
    """

    extension: str = ObjectDescription.get_file_extension()

    # Check is for sphinx autoAPI to be able to work in a CI workflow
    if rosgraph.is_master_online():  # and "/pycram" not in rosnode.get_node_names():
        rospy.init_node('pycram')

    def __init__(self, mode: WorldMode = WorldMode.DIRECT, is_prospection_world: bool = False, sim_frequency=240):
        """
        Creates a new simulation, the type decides of the simulation should be a rendered window or just run in the
        background. There can only be one rendered simulation.
        The BulletWorld object also initializes the Events for attachment, detachment and for manipulating the world.

        :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default is "GUI"
        :param is_prospection_world: For internal usage, decides if this BulletWorld should be used as a shadow world.
        """
        super().__init__(mode=mode, is_prospection_world=is_prospection_world, simulation_frequency=sim_frequency)

        # This disables file caching from PyBullet, since this would also cache
        # files that can not be loaded
        p.setPhysicsEngineParameter(enableFileCaching=0)

        # Needed to let the other thread start the simulation, before Objects are spawned.
        time.sleep(0.1)
        self.vis_axis: List[int] = []

        # Some default settings
        self.set_gravity([0, 0, -9.8])

        if not is_prospection_world:
            _ = Object("floor", ObjectType.ENVIRONMENT, "plane" + self.extension,
                       world=self)

    def _init_world(self, mode: WorldMode):
        self._gui_thread: Gui = Gui(self, mode)
        self._gui_thread.start()
        time.sleep(0.1)

    def load_object_and_get_id(self, path: Optional[str] = None, pose: Optional[Pose] = None) -> int:
        if pose is None:
            pose = Pose()
        return self._load_object_and_get_id(path, pose)

    def _load_object_and_get_id(self, path: str, pose: Pose) -> int:
        if path is None:
            raise ValueError("Path to the object file is required.")
        return p.loadURDF(path,
                          basePosition=pose.position_as_list(),
                          baseOrientation=pose.orientation_as_list(), physicsClientId=self.id)

    def remove_object_from_simulator(self, obj: Object) -> None:
        p.removeBody(obj.id, self.id)

    def remove_object_by_id(self, obj_id: int) -> None:
        p.removeBody(obj_id, self.id)

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

    def get_joint_position(self, joint: ObjectDescription.Joint) -> float:
        return p.getJointState(joint.object_id, joint.id, physicsClientId=self.id)[0]

    def get_object_joint_names(self, obj: Object) -> List[str]:
        return [p.getJointInfo(obj.id, i, physicsClientId=self.id)[1].decode('utf-8')
                for i in range(self.get_object_number_of_joints(obj))]

    def get_link_pose(self, link: ObjectDescription.Link) -> Pose:
        bullet_link_state = p.getLinkState(link.object_id, link.id, physicsClientId=self.id)
        return Pose(*bullet_link_state[4:6])

    def get_object_link_names(self, obj: Object) -> List[str]:
        num_links = self.get_object_number_of_links(obj)
        return [p.getJointInfo(obj.id, i, physicsClientId=self.id)[12].decode('utf-8')
                for i in range(num_links)]

    def get_object_number_of_links(self, obj: Object) -> int:
        return p.getNumJoints(obj.id, physicsClientId=self.id)

    get_object_number_of_joints = get_object_number_of_links

    def perform_collision_detection(self) -> None:
        p.performCollisionDetection(physicsClientId=self.id)

    def get_object_contact_points(self, obj: Object) -> List:
        """
        For a more detailed explanation of the
         returned list please look at:
         `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_
        """
        self.perform_collision_detection()
        return p.getContactPoints(obj.id, physicsClientId=self.id)

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> List:
        self.perform_collision_detection()
        return p.getContactPoints(obj1.id, obj2.id, physicsClientId=self.id)

    def reset_joint_position(self, joint: ObjectDescription.Joint, joint_position: str) -> None:
        p.resetJointState(joint.object_id, joint.id, joint_position, physicsClientId=self.id)

    def reset_object_base_pose(self, obj: Object, pose: Pose) -> None:
        p.resetBasePositionAndOrientation(obj.id, pose.position_as_list(), pose.orientation_as_list(),
                                          physicsClientId=self.id)

    def step(self):
        p.stepSimulation(physicsClientId=self.id)

    def get_object_pose(self, obj: Object) -> Pose:
        return Pose(*p.getBasePositionAndOrientation(obj.id, physicsClientId=self.id))

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

    def save_physics_simulator_state(self) -> int:
        return p.saveState(physicsClientId=self.id)

    def restore_physics_simulator_state(self, state_id):
        p.restoreState(state_id, physicsClientId=self.id)

    def remove_physics_simulator_state(self, state_id: int):
        p.removeState(state_id, physicsClientId=self.id)

    def add_vis_axis(self, pose: Pose,
                     length: Optional[float] = 0.2) -> None:
        """
        Creates a Visual object which represents the coordinate frame at the given
        position and orientation. There can be an unlimited amount of vis axis objects.

        :param pose: The pose at which the axis should be spawned
        :param length: Optional parameter to configure the length of the axes
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
                              link_poses=[Pose(), Pose(), Pose()], link_masses=[1.0, 1.0, 1.0],
                              link_inertial_frame_poses=[Pose(), Pose(), Pose()], link_parent_indices=[0, 0, 0],
                              link_joint_types=[JointType.FIXED.value, JointType.FIXED.value, JointType.FIXED.value],
                              link_joint_axis=[Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1)],
                              link_collision_shape_indices=[-1, -1, -1])

        self.vis_axis.append(self.create_multi_body(multibody))

    def remove_vis_axis(self) -> None:
        """
        Removes all spawned vis axis objects that are currently in this BulletWorld.
        """
        for vis_id in self.vis_axis:
            p.removeBody(vis_id, physicsClientId=self.id)
        self.vis_axis = []

    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        res = p.rayTest(from_position, to_position, physicsClientId=self.id)
        return res[0][0]

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        return p.rayTestBatch(from_positions, to_positions, numThreads=num_threads,
                              physicsClientId=self.id)

    def create_visual_shape(self, visual_shape: VisualShape) -> int:
        return p.createVisualShape(visual_shape.visual_geometry_type.value,
                                   rgbaColor=visual_shape.rgba_color.get_rgba(),
                                   visualFramePosition=visual_shape.visual_frame_position,
                                   physicsClientId=self.id, **visual_shape.shape_data())

    def create_multi_body(self, multi_body: MultiBody) -> int:
        return p.createMultiBody(baseVisualShapeIndex=-multi_body.base_visual_shape_index,
                                 linkVisualShapeIndices=multi_body.link_visual_shape_indices,
                                 basePosition=multi_body.base_pose.position_as_list(),
                                 baseOrientation=multi_body.base_pose.orientation_as_list(),
                                 linkPositions=[pose.position_as_list() for pose in multi_body.link_poses],
                                 linkMasses=multi_body.link_masses,
                                 linkOrientations=[pose.orientation_as_list() for pose in multi_body.link_poses],
                                 linkInertialFramePositions=[pose.position_as_list()
                                                             for pose in multi_body.link_inertial_frame_poses],
                                 linkInertialFrameOrientations=[pose.orientation_as_list()
                                                                for pose in multi_body.link_inertial_frame_poses],
                                 linkParentIndices=multi_body.link_parent_indices,
                                 linkJointTypes=multi_body.link_joint_types,
                                 linkJointAxis=[[point.x, point.y, point.z] for point in multi_body.link_joint_axis],
                                 linkCollisionShapeIndices=multi_body.link_collision_shape_indices)

    def get_images_for_target(self,
                              target_pose: Pose,
                              cam_pose: Pose,
                              size: Optional[int] = 256) -> List[np.ndarray]:
        # TODO: Might depend on robot cameras, if so please add these camera parameters to RobotDescription object
        # TODO: of your robot with a CameraDescription object.
        fov = 90
        aspect = size / size
        near = 0.2
        far = 100

        view_matrix = p.computeViewMatrix(cam_pose.position_as_list(), target_pose.position_as_list(), [0, 0, 1])
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        return list(p.getCameraImage(size, size, view_matrix, projection_matrix,
                                     physicsClientId=self.id))[2:5]

    def add_text(self, text: str, position: List[float], orientation: Optional[List[float]] = None,
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

    def remove_text(self, text_id: Optional[int] = None) -> None:
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

            # DISCLAIMER
            # This camera control only works if the WorldMooe.GUI BulletWorld is the first one to be created. This is
            # due to a bug in the function pybullet.getDebugVisualizerCamera() which only returns the information of
            # the first created simulation.

            # Disable the side windows of the GUI
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.world.id)
            # Change the init camera pose
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=270.0, cameraPitch=-50,
                                         cameraTargetPosition=[-2, 0, 1], physicsClientId=self.world.id)

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
            camera_yaw = 50
            camera_pitch = -35

            # Keep track of the mouse state
            mouse_state = [0, 0, 0]
            old_mouse_x, old_mouse_y = 0, 0

            # Determines if the sphere at cameraTargetPosition is visible
            visible = 1

            # Loop to update the camera position based on keyboard events
            while p.isConnected(self.world.id):
                # Monitor user input
                keys = p.getKeyboardEvents(self.world.id)
                mouse = p.getMouseEvents(self.world.id)

                # Get infos about the camera
                width, height, dist = (p.getDebugVisualizerCamera()[0],
                                       p.getDebugVisualizerCamera()[1],
                                       p.getDebugVisualizerCamera()[10])
                #print("width: ", width, "height: ", height, "dist: ", dist)
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
                p.resetBasePositionAndOrientation(sphere_uid, camera_target_position, [0, 0, 0, 1], physicsClientId=self.world.id)
                time.sleep(1. / 80.)
