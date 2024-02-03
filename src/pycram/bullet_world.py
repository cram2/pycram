# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import threading
import time
from typing_extensions import List, Optional, Dict, Tuple

import numpy as np
import pybullet as p
import rosgraph
import rospy

from .enums import JointType, ObjectType, WorldMode
from .pose import Pose
from .world import World, Object, Link
from .world_dataclasses import Color, Constraint, AxisAlignedBoundingBox, MultiBody, VisualShape, BoxVisualShape
from dataclasses import asdict


class BulletWorld(World):
    """
    This class represents a BulletWorld, which is a simulation environment that uses the Bullet Physics Engine. This
    class is the main interface to the Bullet Physics Engine and should be used to spawn Objects, simulate Physic and
    manipulate the Bullet World.
    """

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

        self._gui_thread: Gui = Gui(self, mode)
        self._gui_thread.start()

        # This disables file caching from PyBullet, since this would also cache
        # files that can not be loaded
        p.setPhysicsEngineParameter(enableFileCaching=0)

        # Needed to let the other thread start the simulation, before Objects are spawned.
        time.sleep(0.1)
        self.vis_axis: List[Object] = []

        # Some default settings
        self.set_gravity([0, 0, -9.8])

        if not is_prospection_world:
            plane = Object("floor", ObjectType.ENVIRONMENT, "plane.urdf", world=self)

    def load_urdf_and_get_object_id(self, path: str, pose: Pose) -> int:
        return p.loadURDF(path,
                          basePosition=pose.position_as_list(),
                          baseOrientation=pose.orientation_as_list(), physicsClientId=self.client_id)

    def remove_object_from_simulator(self, obj: Object) -> None:
        p.removeBody(obj.id, self.client_id)

    def add_constraint(self, constraint: Constraint) -> int:
        joint_axis_in_child_link_frame = [constraint.joint_axis_in_child_link_frame.x,
                                          constraint.joint_axis_in_child_link_frame.y,
                                          constraint.joint_axis_in_child_link_frame.z]

        constraint_id = p.createConstraint(constraint.parent_link.get_object_id(),
                                           constraint.parent_link.id,
                                           constraint.child_link.get_object_id(),
                                           constraint.child_link.id,
                                           constraint.joint_type.value,
                                           joint_axis_in_child_link_frame,
                                           constraint.joint_frame_pose_wrt_parent_origin.position_as_list(),
                                           constraint.joint_frame_pose_wrt_child_origin.position_as_list(),
                                           constraint.joint_frame_pose_wrt_parent_origin.orientation_as_list(),
                                           constraint.joint_frame_pose_wrt_child_origin.orientation_as_list(),
                                           physicsClientId=self.client_id)
        return constraint_id

    def remove_constraint(self, constraint_id):
        p.removeConstraint(constraint_id, physicsClientId=self.client_id)

    def get_joint_rest_position(self, obj: Object, joint_name: str) -> float:
        return p.getJointState(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[0]

    def get_joint_damping(self, obj: Object, joint_name: str) -> float:
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[6]

    def get_joint_upper_limit(self, obj: Object, joint_name: str) -> float:
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[8]

    def get_joint_lower_limit(self, obj: Object, joint_name: str) -> float:
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[9]

    def get_joint_axis(self, obj: Object, joint_name: str) -> Tuple[float]:
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[13]

    def get_joint_type(self, obj: Object, joint_name: str) -> JointType:
        joint_type = p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[2]
        return JointType(joint_type)

    def get_joint_position(self, obj: Object, joint_name: str) -> float:
        return p.getJointState(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[0]

    def get_link_pose(self, link: Link) -> Pose:
        return Pose(*p.getLinkState(link.get_object_id(), link.id, physicsClientId=self.client_id)[4:6])

    def perform_collision_detection(self) -> None:
        p.performCollisionDetection(physicsClientId=self.client_id)

    def get_object_contact_points(self, obj: Object) -> List:
        """
        For a more detailed explanation of the
         returned list please look at:
         `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_
        """
        self.perform_collision_detection()
        return p.getContactPoints(obj.id, physicsClientId=self.client_id)

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> List:
        self.perform_collision_detection()
        return p.getContactPoints(obj1.id, obj2.id, physicsClientId=self.client_id)

    def get_joint_names(self, obj: Object) -> List[str]:
        num_joints = self.get_number_of_joints(obj)
        return [p.getJointInfo(obj.id, i, physicsClientId=self.client_id)[1].decode('utf-8') for i in range(num_joints)]

    def get_link_names(self, obj: Object) -> List[str]:
        num_links = self.get_number_of_links(obj)
        return [p.getJointInfo(obj.id, i, physicsClientId=self.client_id)[12].decode('utf-8') for i in range(num_links)]

    def get_number_of_links(self, obj: Object) -> int:
        return self.get_number_of_joints(obj)

    def get_number_of_joints(self, obj: Object) -> int:
        return p.getNumJoints(obj.id, physicsClientId=self.client_id)

    def reset_joint_position(self, obj: Object, joint_name: str, joint_pose: float) -> None:
        p.resetJointState(obj.id, obj.joint_name_to_id[joint_name], joint_pose, physicsClientId=self.client_id)

    def reset_object_base_pose(self, obj: Object, pose: Pose) -> None:
        p.resetBasePositionAndOrientation(obj.id, pose.position_as_list(), pose.orientation_as_list(),
                                          physicsClientId=self.client_id)

    def step(self):
        p.stepSimulation(physicsClientId=self.client_id)

    def get_object_pose(self, obj: Object) -> Pose:
        return Pose(*p.getBasePositionAndOrientation(obj.id, physicsClientId=self.client_id))

    def set_link_color(self, link: Link, rgba_color: Color):
        p.changeVisualShape(link.get_object_id(), link.id, rgbaColor=rgba_color, physicsClientId=self.client_id)

    def get_link_color(self, link: Link) -> Color:
        return self.get_colors_of_object_links(link.object)[link.name]

    def get_colors_of_object_links(self, obj: Object) -> Dict[str, Color]:
        visual_data = p.getVisualShapeData(obj.id, physicsClientId=self.client_id)
        link_id_to_name = {v: k for k, v in obj.link_name_to_id.items()}
        links = list(map(lambda x: link_id_to_name[x[1]] if x[1] != -1 else "base", visual_data))
        colors = list(map(lambda x: Color.from_rgba(x[7]), visual_data))
        link_to_color = dict(zip(links, colors))
        return link_to_color

    def get_object_axis_aligned_bounding_box(self, obj: Object) -> AxisAlignedBoundingBox:
        return AxisAlignedBoundingBox.from_min_max(*p.getAABB(obj.id, physicsClientId=self.client_id))

    def get_link_axis_aligned_bounding_box(self, link: Link) -> AxisAlignedBoundingBox:
        return AxisAlignedBoundingBox.from_min_max(*p.getAABB(link.get_object_id(), link.id, physicsClientId=self.client_id))

    def set_realtime(self, real_time: bool) -> None:
        p.setRealTimeSimulation(1 if real_time else 0, physicsClientId=self.client_id)

    def set_gravity(self, gravity_vector: List[float]) -> None:
        p.setGravity(gravity_vector[0], gravity_vector[1], gravity_vector[2], physicsClientId=self.client_id)

    def disconnect_from_physics_server(self):
        p.disconnect(physicsClientId=self.client_id)

    def join_threads(self):
        """
        Joins the GUI thread if it exists.
        """
        self.join_gui_thread_if_exists()

    def join_gui_thread_if_exists(self):
        if self._gui_thread:
            self._gui_thread.join()

    def save_physics_simulator_state(self) -> int:
        return p.saveState(physicsClientId=self.client_id)

    def restore_physics_simulator_state(self, state_id):
        p.restoreState(state_id, physicsClientId=self.client_id)

    def remove_physics_simulator_state(self, state_id: int):
        p.removeState(state_id, physicsClientId=self.client_id)

    def add_vis_axis(self, pose: Pose,
                     length: Optional[float] = 0.2) -> None:
        """
        Creates a Visual object which represents the coordinate frame at the given
        position and orientation. There can be an unlimited amount of vis axis objects.

        :param pose: The pose at which the axis should be spawned
        :param length: Optional parameter to configure the length of the axes
        """

        pose_in_map = self.local_transformer.transform_pose(pose, "map")

        position, orientation = pose_in_map.to_list()

        box_vis_shape = BoxVisualShape(Color(1, 0, 0, 0.8), [length, 0.01, 0.01], [length, 0.01, 0.01])
        vis_x = self.create_visual_shape(box_vis_shape)

        box_vis_shape = BoxVisualShape(Color(0, 1, 0, 0.8), [0.01, length, 0.01], [0.01, length, 0.01])
        vis_y = self.create_visual_shape(box_vis_shape)

        box_vis_shape = BoxVisualShape(Color(0, 0, 1, 0.8), [0.01, 0.01, length], [0.01, 0.01, length])
        vis_z = self.create_visual_shape(box_vis_shape)

        obj = p.createMultiBody(baseVisualShapeIndex=-1, linkVisualShapeIndices=[vis_x, vis_y, vis_z],
                                basePosition=position, baseOrientation=orientation,
                                linkPositions=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                linkMasses=[1.0, 1.0, 1.0], linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
                                linkInertialFramePositions=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
                                linkParentIndices=[0, 0, 0],
                                linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED],
                                linkJointAxis=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                linkCollisionShapeIndices=[-1, -1, -1],
                                physicsClientId=self.client_id)

        self.vis_axis.append(obj)

    def remove_vis_axis(self) -> None:
        """
        Removes all spawned vis axis objects that are currently in this BulletWorld.
        """
        for vis_id in self.vis_axis:
            p.removeBody(vis_id, physicsClientId=self.client_id)
        self.vis_axis = []

    def ray_test(self, from_position: List[float], to_position: List[float]) -> int:
        res = p.rayTest(from_position, to_position, physicsClientId=self.client_id)
        return res[0][0]

    def ray_test_batch(self, from_positions: List[List[float]], to_positions: List[List[float]],
                       num_threads: int = 1) -> List[int]:
        return p.rayTestBatch(from_positions, to_positions, numThreads=num_threads,
                              physicsClientId=self.client_id)

    def create_visual_shape(self, visual_shape: VisualShape) -> int:
        return p.createVisualShape(visual_shape.visual_geometry_type.value, rgbaColor=visual_shape.rgba_color,
                                   visualFramePosition=visual_shape.visual_frame_position,
                                   physicsClientId=self.client_id, **visual_shape.shape_data())

    def create_multi_body(self, multi_body: MultiBody) -> int:
        return p.createMultiBody(baseVisualShapeIndex=-MultiBody.base_visual_shape_index,
                                 linkVisualShapeIndices=MultiBody.link_visual_shape_indices,
                                 basePosition=MultiBody.base_pose.position_as_list(),
                                 baseOrientation=MultiBody.base_pose.orientation_as_list(),
                                 linkPositions=[pose.position_as_list() for pose in MultiBody.link_poses],
                                 linkMasses=MultiBody.link_masses,
                                 linkOrientations=[pose.orientation_as_list() for pose in MultiBody.link_poses],
                                 linkInertialFramePositions=[pose.position_as_list()
                                                             for pose in MultiBody.link_inertial_frame_poses],
                                 linkInertialFrameOrientations=[pose.orientation_as_list()
                                                                for pose in MultiBody.link_inertial_frame_poses],
                                 linkParentIndices=MultiBody.link_parent_indices,
                                 linkJointTypes=MultiBody.link_joint_types,
                                 linkJointAxis=[[point.x, point.y, point.z] for point in MultiBody.link_joint_axis],
                                 linkCollisionShapeIndices=MultiBody.link_collision_shape_indices)

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

        view_matrix = p.computeViewMatrix(cam_pose.position_as_list(), target_pose.position_as_list(), [0, 0, 1],
                                          physicsClientId=self.client_id)
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far, physicsClientId=self.client_id)
        return list(p.getCameraImage(size, size, view_matrix, projection_matrix,
                                     physicsClientId=self.client_id))[2:5]

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
        return p.addUserDebugText(text, position, color.get_rgb(), physicsClientId=self.client_id, **args)

    def remove_text(self, text_id: Optional[int] = None) -> None:
        if text_id:
            p.removeUserDebugItem(text_id, physicsClientId=self.client_id)
        else:
            p.removeAllUserDebugItems(physicsClientId=self.client_id)

    def enable_joint_force_torque_sensor(self, obj: Object, fts_joint_idx: int) -> None:
        p.enableJointForceTorqueSensor(obj.id, fts_joint_idx, enableSensor=1, physicsClientId=self.client_id)

    def disable_joint_force_torque_sensor(self, obj: Object, joint_id: int) -> None:
        p.enableJointForceTorqueSensor(obj.id, joint_id, enableSensor=0, physicsClientId=self.client_id)

    def get_joint_reaction_force_torque(self, obj: Object, joint_id: int) -> List[float]:
        return p.getJointState(obj.id, joint_id, physicsClientId=self.client_id)[2]

    def get_applied_joint_motor_torque(self, obj: Object, joint_id: int) -> float:
        return p.getJointState(obj.id, joint_id, physicsClientId=self.client_id)[3]


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
            self.world.client_id = p.connect(p.DIRECT)
        else:
            self.world.client_id = p.connect(p.GUI)

            # Disable the side windows of the GUI
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.world.client_id)
            # Change the init camera pose
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=270.0, cameraPitch=-50,
                                         cameraTargetPosition=[-2, 0, 1], physicsClientId=self.world.client_id)

            # Get the initial camera target location
            cameraTargetPosition = p.getDebugVisualizerCamera(physicsClientId=self.world.client_id)[11]

            sphereVisualId = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1],
                                                 physicsClientId=self.world.client_id)

            # Create a sphere with a radius of 0.05 and a mass of 0
            sphereUid = p.createMultiBody(baseMass=0.0,
                                          baseInertialFramePosition=[0, 0, 0],
                                          baseVisualShapeIndex=sphereVisualId,
                                          basePosition=cameraTargetPosition,
                                          physicsClientId=self.world.client_id)

            # Define the maxSpeed, used in calculations
            maxSpeed = 16

            # Set initial Camera Rotation
            cameraYaw = 50
            cameraPitch = -35

            # Keep track of the mouse state
            mouseState = [0, 0, 0]
            oldMouseX, oldMouseY = 0, 0

            # Determines if the sphere at cameraTargetPosition is visible
            visible = 1

            # Loop to update the camera position based on keyboard events
            while p.isConnected(self.world.client_id):
                # Monitor user input
                keys = p.getKeyboardEvents()
                mouse = p.getMouseEvents()

                # Get infos about the camera
                width, height, dist = (p.getDebugVisualizerCamera()[0],
                                       p.getDebugVisualizerCamera()[1],
                                       p.getDebugVisualizerCamera()[10])
                cameraTargetPosition = p.getDebugVisualizerCamera()[11]

                # Get vectors used for movement on x,y,z Vector
                xVec = [p.getDebugVisualizerCamera()[2][i] for i in [0, 4, 8]]
                yVec = [p.getDebugVisualizerCamera()[2][i] for i in [2, 6, 10]]
                zVec = (0, 0, 1)  # [p.getDebugVisualizerCamera()[2][i] for i in [1, 5, 9]]

                # Check the mouse state
                if mouse:
                    for m in mouse:

                        mouseX = m[2]
                        mouseY = m[1]

                        # update mouseState
                        # Left Mouse button
                        if m[0] == 2 and m[3] == 0:
                            mouseState[0] = m[4]
                        # Middle mouse butto (scroll wheel)
                        if m[0] == 2 and m[3] == 1:
                            mouseState[1] = m[4]
                        # right mouse button
                        if m[0] == 2 and m[3] == 2:
                            mouseState[2] = m[4]

                        # change visibility by clicking the mousewheel
                        if m[4] == 6 and m[3] == 1 and visible == 1:
                            visible = 0
                        elif m[4] == 6 and visible == 0:
                            visible = 1

                        # camera movement when the left mouse button is pressed
                        if mouseState[0] == 3:
                            speedX = abs(oldMouseX - mouseX) if (abs(oldMouseX - mouseX)) < maxSpeed else maxSpeed
                            speedY = abs(oldMouseY - mouseY) if (abs(oldMouseY - mouseY)) < maxSpeed else maxSpeed

                            # max angle of 89.5 and -89.5 to make sure the camera does not flip (is annoying)
                            if mouseX < oldMouseX:
                                if (cameraPitch + speedX) < 89.5:
                                    cameraPitch += (speedX / 4) + 1
                            elif mouseX > oldMouseX:
                                if (cameraPitch - speedX) > -89.5:
                                    cameraPitch -= (speedX / 4) + 1

                            if mouseY < oldMouseY:
                                cameraYaw += (speedY / 4) + 1
                            elif mouseY > oldMouseY:
                                cameraYaw -= (speedY / 4) + 1

                        if mouseState[1] == 3:
                            speedX = abs(oldMouseX - mouseX)
                            factor = 0.05

                            if mouseX < oldMouseX:
                                dist = dist - speedX * factor
                            elif mouseX > oldMouseX:
                                dist = dist + speedX * factor
                            dist = max(dist, 0.1)

                        # camera movement when the right mouse button is pressed
                        if mouseState[2] == 3:
                            speedX = abs(oldMouseX - mouseX) if (abs(oldMouseX - mouseX)) < 5 else 5
                            speedY = abs(oldMouseY - mouseY) if (abs(oldMouseY - mouseY)) < 5 else 5
                            factor = 0.05

                            if mouseX < oldMouseX:
                                cameraTargetPosition = np.subtract(cameraTargetPosition,
                                                                   np.multiply(np.multiply(zVec, factor), speedX))
                            elif mouseX > oldMouseX:
                                cameraTargetPosition = np.add(cameraTargetPosition,
                                                              np.multiply(np.multiply(zVec, factor), speedX))

                            if mouseY < oldMouseY:
                                cameraTargetPosition = np.add(cameraTargetPosition,
                                                              np.multiply(np.multiply(xVec, factor), speedY))
                            elif mouseY > oldMouseY:
                                cameraTargetPosition = np.subtract(cameraTargetPosition,
                                                                   np.multiply(np.multiply(xVec, factor), speedY))
                        # update oldMouse values
                        oldMouseY, oldMouseX = mouseY, mouseX

                # check the keyboard state
                if keys:
                    # if shift is pressed, double the speed
                    if p.B3G_SHIFT in keys:
                        speedMult = 5
                    else:
                        speedMult = 2.5

                    # if control is pressed, the movements caused by the arrowkeys, the '+' as well as the '-' key
                    # change
                    if p.B3G_CONTROL in keys:

                        # the up and down arrowkeys cause the targetPos to move along the z axis of the map
                        if p.B3G_DOWN_ARROW in keys:
                            cameraTargetPosition = np.subtract(cameraTargetPosition,
                                                               np.multiply(np.multiply(zVec, 0.03), speedMult))
                        elif p.B3G_UP_ARROW in keys:
                            cameraTargetPosition = np.add(cameraTargetPosition,
                                                          np.multiply(np.multiply(zVec, 0.03), speedMult))

                        # left and right arrowkeys cause the targetPos to move horizontally relative to the camera
                        if p.B3G_LEFT_ARROW in keys:
                            cameraTargetPosition = np.subtract(cameraTargetPosition,
                                                               np.multiply(np.multiply(xVec, 0.03), speedMult))
                        elif p.B3G_RIGHT_ARROW in keys:
                            cameraTargetPosition = np.add(cameraTargetPosition,
                                                          np.multiply(np.multiply(xVec, 0.03), speedMult))

                        # the '+' and '-' keys cause the targetpos to move forwards and backwards relative to the camera
                        # while the camera stays at a constant distance. SHIFT + '=' is for US layout
                        if ord("+") in keys or p.B3G_SHIFT in keys and ord("=") in keys:
                            cameraTargetPosition = np.subtract(cameraTargetPosition,
                                                               np.multiply(np.multiply(yVec, 0.03), speedMult))
                        elif ord("-") in keys:
                            cameraTargetPosition = np.add(cameraTargetPosition,
                                                          np.multiply(np.multiply(yVec, 0.03), speedMult))

                    # standard bindings for thearrowkeys, the '+' as well as the '-' key
                    else:

                        # left and right arrowkeys cause the camera to rotate around the yaw axis
                        if p.B3G_RIGHT_ARROW in keys:
                            cameraYaw += (360 / width) * speedMult
                        elif p.B3G_LEFT_ARROW in keys:
                            cameraYaw -= (360 / width) * speedMult

                        # the up and down arrowkeys cause the camera to rotate around the pitch axis
                        if p.B3G_DOWN_ARROW in keys:
                            if (cameraPitch + (360 / height) * speedMult) < 89.5:
                                cameraPitch += (360 / height) * speedMult
                        elif p.B3G_UP_ARROW in keys:
                            if (cameraPitch - (360 / height) * speedMult) > -89.5:
                                cameraPitch -= (360 / height) * speedMult

                        # the '+' and '-' keys cause the camera to zoom towards and away from the targetPos without
                        # moving it. SHIFT + '=' is for US layout since the events can't handle shift plus something
                        if ord("+") in keys or p.B3G_SHIFT in keys and ord("=") in keys:
                            if (dist - (dist * 0.02) * speedMult) > 0.1:
                                dist -= dist * 0.02 * speedMult
                        elif ord("-") in keys:
                            dist += dist * 0.02 * speedMult

                p.resetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=cameraYaw, cameraPitch=cameraPitch,
                                             cameraTargetPosition=cameraTargetPosition)
                if visible == 0:
                    cameraTargetPosition = (0.0, -50, 50)
                p.resetBasePositionAndOrientation(sphereUid, cameraTargetPosition, [0, 0, 0, 1])
                time.sleep(1. / 80.)
