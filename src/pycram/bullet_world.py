# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import threading
import time
from typing import List, Optional, Dict, Tuple

import numpy as np
import pybullet as p
import rosgraph
import rospy

from .enums import JointType, ObjectType
from .pose import Pose
from .world import World, Object
from .world_dataclasses import Color, Constraint, AxisAlignedBoundingBox, MultiBody


class BulletWorld(World):
    """
    This class represents a BulletWorld, which is a simulation environment that uses the Bullet Physics Engine. This
    class is the main interface to the Bullet Physics Engine and should be used to spawn Objects, simulate Physic and
    manipulate the Bullet World.
    """

    # Check is for sphinx autoAPI to be able to work in a CI workflow
    if rosgraph.is_master_online():  # and "/pycram" not in rosnode.get_node_names():
        rospy.init_node('pycram')

    def __init__(self, mode: str = "GUI", is_prospection_world: bool = False, sim_time_step=0.004167):  # 240 Hz
        """
        Creates a new simulation, the type decides of the simulation should be a rendered window or just run in the
        background. There can only be one rendered simulation.
        The BulletWorld object also initializes the Events for attachment, detachment and for manipulating the world.

        :param mode: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default parameter is "GUI"
        :param is_prospection_world: For internal usage, decides if this BulletWorld should be used as a shadow world.
        """
        super().__init__(mode=mode, is_prospection_world=is_prospection_world, simulation_time_step=sim_time_step)

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

    def load_urdf_at_pose_and_get_object_id(self, path: str, pose: Pose) -> int:
        return p.loadURDF(path,
                          basePosition=pose.position_as_list(),
                          baseOrientation=pose.orientation_as_list(), physicsClientId=self.client_id)

    def remove_object(self, obj_id: int) -> None:
        """
        Remove an object by its ID.

        :param obj_id: The unique id of the object to be removed.
        """

        p.removeBody(obj_id, self.client_id)

    def add_constraint(self, constraint: Constraint) -> int:
        """
        Add a constraint between two objects so that attachment they become attached
        """

        parent_obj = self.get_object_by_id(constraint.parent_obj_id)
        parent_link_id = parent_obj.link_name_to_id[constraint.parent_link_name]
        child_obj = self.get_object_by_id(constraint.child_obj_id)
        child_link_id = child_obj.link_name_to_id[constraint.child_link_name]

        constraint_id = p.createConstraint(constraint.parent_obj_id,
                                           parent_link_id,
                                           constraint.child_obj_id,
                                           child_link_id,
                                           constraint.joint_type.as_int(),
                                           constraint.joint_axis_in_child_link_frame,
                                           constraint.joint_frame_position_wrt_parent_origin,
                                           constraint.joint_frame_position_wrt_child_origin,
                                           constraint.joint_frame_orientation_wrt_parent_origin,
                                           constraint.joint_frame_orientation_wrt_child_origin,
                                           physicsClientId=self.client_id)
        return constraint_id

    def remove_constraint(self, constraint_id):
        print("Removing constraint with id: ", constraint_id)
        p.removeConstraint(constraint_id, physicsClientId=self.client_id)

    def get_object_joint_rest_pose(self, obj: Object, joint_name: str) -> float:
        """
        Get the joint rest pose of an articulated object

        :param obj: The object
        :param joint_name: The name of the joint
        """
        return p.getJointState(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[0]

    def get_object_joint_damping(self, obj: Object, joint_name: str) -> float:
        """
        Get the joint damping of an articulated object

        :param obj: The object
        :param joint_name: The name of the joint
        """
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[6]

    def get_object_joint_upper_limit(self, obj: Object, joint_name: str) -> float:
        """
        Get the joint upper limit of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: The joint upper limit as a float.
        """
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[8]

    def get_object_joint_lower_limit(self, obj: Object, joint_name: str) -> float:
        """
        Get the joint lower limit of an articulated object

        :param obj: The object.
        :param joint_name: The name of the joint.
        :return: The joint lower limit as a float.
        """
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[9]

    def get_object_joint_axis(self, obj: Object, joint_name: str) -> Tuple[float]:
        """
        Returns the axis along which a joint is moving. The given joint_name has to be part of this object.

        :param obj: The object
        :param joint_name: Name of the joint for which the axis should be returned.
        :return: The axis a vector of xyz
        """
        return p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[13]

    def get_object_joint_type(self, obj: Object, joint_name: str) -> JointType:
        """
        Returns the type of the joint as element of the Enum :mod:`~pycram.enums.JointType`.

        :param obj: The object
        :param joint_name: Joint name for which the type should be returned
        :return: The type of  the joint
        """
        joint_type = p.getJointInfo(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[2]
        return JointType(joint_type)

    def get_object_joint_position(self, obj: Object, joint_name: str) -> float:
        """
        Get the state of a joint of an articulated object

        :param obj: The object
        :param joint_name: The name of the joint
        """
        return p.getJointState(obj.id, obj.joint_name_to_id[joint_name], physicsClientId=self.client_id)[0]

    def get_object_link_pose(self, obj_id: int, link_id: int) -> Pose:
        """
        Get the pose of a link of an articulated object with respect to the world frame.
        """
        return Pose(*p.getLinkState(obj_id, link_id, physicsClientId=self.client_id)[4:6])

    def perform_collision_detection(self) -> None:
        """
        Performs collision detection and updates the contact points.
        """
        p.performCollisionDetection(physicsClientId=self.client_id)

    def get_object_contact_points(self, obj: Object) -> List:
        """l.update_transforms_for_object(self.milk)
        Returns a list of contact points of this Object with other Objects. For a more detailed explanation of the returned
        list please look at `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_

        :param obj: The object.
        :return: A list of all contact points with other objects
        """
        return p.getContactPoints(obj.id, physicsClientId=self.client_id)

    def get_contact_points_between_two_objects(self, obj1: Object, obj2: Object) -> List:
        """
        Returns a list of contact points between obj1 and obj2.

        :param obj1: The first object.
        :param obj2: The second object.
        :return: A list of all contact points between the two objects.
        """
        return p.getContactPoints(obj1.id, obj2.id, physicsClientId=self.client_id)

    def get_object_joint_names(self, obj_id: int) -> List[str]:
        """
        Get the names of all joints of an articulated object.
        """
        num_joints = self.get_object_number_of_joints(obj_id)
        return [p.getJointInfo(obj_id, i, physicsClientId=self.client_id)[1].decode('utf-8') for i in range(num_joints)]

    def get_object_link_names(self, obj_id: int) -> List[str]:
        """
        Get the names of all joints of an articulated object.
        """
        num_links = self.get_object_number_of_links(obj_id)
        return [p.getJointInfo(obj_id, i, physicsClientId=self.client_id)[12].decode('utf-8') for i in range(num_links)]

    def get_object_number_of_links(self, obj_id: int) -> int:
        """
        Get the number of links of an articulated object

        :param obj_id: The object
        """
        return self.get_object_number_of_joints(obj_id)

    def get_object_number_of_joints(self, obj_id: int) -> int:
        """
        Get the number of joints of an articulated object

        :param obj_id: The object
        """
        return p.getNumJoints(obj_id, physicsClientId=self.client_id)

    def reset_object_joint_position(self, obj: Object, joint_name: str, joint_pose: float) -> None:
        """
        Reset the joint position instantly without physics simulation

        :param obj: The object
        :param joint_name: The name of the joint
        :param joint_pose: The new joint pose
        """
        p.resetJointState(obj.id, obj.joint_name_to_id[joint_name], joint_pose, physicsClientId=self.client_id)

    def reset_object_base_pose(self, obj: Object, position: List[float], orientation: List[float]):
        """
        Reset the world position and orientation of the base of the object instantaneously,
        not through physics simulation. (x,y,z) position vector and (x,y,z,w) quaternion orientation.

        :param obj: The object
        :param position: The new position of the object as a vector of x,y,z
        :param orientation: The new orientation of the object as a quaternion of x,y,z,w
        """
        p.resetBasePositionAndOrientation(obj.id, position, orientation, physicsClientId=self.client_id)

    def step(self):
        """
        Step the world simulation using forward dynamics
        """
        p.stepSimulation(physicsClientId=self.client_id)

    def set_object_link_color(self, obj: Object, link_id: int, rgba_color: Color):
        """
        Changes the color of a link of this object, the color has to be given as a 4 element list
        of RGBA values.

        :param obj: The object which should be colored
        :param link_id: The link id of the link which should be colored
        :param rgba_color: The color as RGBA values between 0 and 1
        """
        p.changeVisualShape(obj.id, link_id, rgbaColor=rgba_color, physicsClientId=self.client_id)

    def get_color_of_object_link(self, obj: Object, link_name: str) -> Color:
        return self.get_colors_of_all_links_of_object(obj)[link_name]

    def get_colors_of_all_links_of_object(self, obj: Object) -> Dict[str, Color]:
        """
        Get the RGBA colors of each link in the object as a dictionary from link name to color.

        :param obj: The object
        :return: A dictionary with link names as keys and 4 element list with the RGBA values for each link as value.
        """
        visual_data = p.getVisualShapeData(obj.id, physicsClientId=self.client_id)
        link_id_to_name = {v: k for k, v in obj.link_name_to_id.items()}
        links = list(map(lambda x: link_id_to_name[x[1]] if x[1] != -1 else "base", visual_data))
        colors = list(map(lambda x: Color.from_rgba(x[7]), visual_data))
        link_to_color = dict(zip(links, colors))
        return link_to_color

    def get_object_aabb(self, obj: Object) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of this object. The return of this method are two points in
        world coordinate frame which define a bounding box.

        :param obj: The object for which the bounding box should be returned.
        :return: AxisAlignedBoundingBox object with min and max box points.
        """
        return AxisAlignedBoundingBox.from_min_max(*p.getAABB(obj.id, physicsClientId=self.client_id))

    def get_object_link_aabb(self, obj_id: int, link_id: int) -> AxisAlignedBoundingBox:
        """
        Returns the axis aligned bounding box of the link. The return of this method are two points in
        world coordinate frame which define a bounding box.
        """
        return AxisAlignedBoundingBox.from_min_max(*p.getAABB(obj_id, link_id, physicsClientId=self.client_id))

    def set_realtime(self, real_time: bool) -> None:
        """
        Enables the real time simulation of Physic in the BulletWorld. By default, this is disabled and Physics is only
        simulated to reason about it.

        :param real_time: Whether the World should simulate Physics in real time.
        """
        p.setRealTimeSimulation(1 if real_time else 0, physicsClientId=self.client_id)

    def set_gravity(self, gravity_vector: List[float]) -> None:
        """
        Sets the gravity that is used in the World. By default, it is set to the gravity on earth ([0, 0, -9.8]).
         Gravity is given as a vector in x,y,z. Gravity is only applied while simulating Physic.

        :param gravity_vector: The gravity vector that should be used in the World.
        """
        p.setGravity(gravity_vector[0], gravity_vector[1], gravity_vector[2], physicsClientId=self.client_id)

    def exit(self, wait_time_before_exit_in_secs: Optional[float] = 0.1) -> None:
        """
        Closes the BulletWorld as well as the shadow world, also collects any other thread that is running. This is the
        preferred method to close the BulletWorld.
        """
        super().exit(wait_time_before_exit_in_secs)

    def disconnect_from_physics_server(self):
        """
        Disconnects the world from the physics server.
        """
        p.disconnect(physicsClientId=self.client_id)

    def join_threads(self):
        """
        Join any running threads. Useful for example when exiting the world.
        """
        self.join_gui_thread_if_exists()

    def join_gui_thread_if_exists(self):
        if self._gui_thread:
            self._gui_thread.join()

    def save_physics_simulator_state(self) -> int:
        """
        Saves the state of the physics simulator and returns the unique id of the state.
        """
        return p.saveState(physicsClientId=self.client_id)

    def restore_physics_simulator_state(self, state_id):
        """
        Restores the objects and environment state in the physics simulator according to
         the given state using the unique state id.
        """
        p.restoreState(state_id, physicsClientId=self.client_id)

    def remove_physics_simulator_state(self, state_id: int):
        """
        Removes the physics simulator state with the given unique state id.
        """
        p.removeState(state_id, physicsClientId=self.client_id)

    def add_vis_axis(self, pose: Pose,
                     length: Optional[float] = 0.2) -> None:
        """
        Creates a Visual object which represents the coordinate frame at the given
        position and orientation. There can be an unlimited amount of vis axis objects.

        :param pose: The pose at which the axis should be spawned
        :param length: Optional parameter to configure the length of the axes
        """

        position, orientation = pose.to_list()

        vis_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[length, 0.01, 0.01],
                                    rgbaColor=[1, 0, 0, 0.8], visualFramePosition=[length, 0.01, 0.01],
                                    physicsClientId=self.client_id)
        vis_y = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, length, 0.01],
                                    rgbaColor=[0, 1, 0, 0.8], visualFramePosition=[0.01, length, 0.01],
                                    physicsClientId=self.client_id)
        vis_z = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, length],
                                    rgbaColor=[0, 0, 1, 0.8], visualFramePosition=[0.01, 0.01, length],
                                    physicsClientId=self.client_id)

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
        for id in self.vis_axis:
            p.removeBody(id, physicsClientId=self.client_id)
        self.vis_axis = []

    def create_multi_body(self, multi_body: MultiBody) -> int:
        return p.createMultiBody(baseVisualShapeIndex=-MultiBody.base_visual_shape_index,
                                 linkVisualShapeIndices=MultiBody.link_visual_shape_indices,
                                 basePosition=MultiBody.base_position, baseOrientation=MultiBody.base_orientation,
                                 linkPositions=MultiBody.link_positions, linkMasses=MultiBody.link_masses,
                                 linkOrientations=MultiBody.link_orientations,
                                 linkInertialFramePositions=MultiBody.link_inertial_frame_positions,
                                 linkInertialFrameOrientations=MultiBody.link_inertial_frame_orientations,
                                 linkParentIndices=MultiBody.link_parent_indices,
                                 linkJointTypes=MultiBody.link_joint_types,
                                 linkJointAxis=MultiBody.link_joint_axis,
                                 linkCollisionShapeIndices=MultiBody.link_collision_shape_indices)

    def get_images_for_target(self,
                              target_pose: Pose,
                              cam_pose: Pose,
                              size: Optional[int] = 256) -> List[np.ndarray]:
        """
        Calculates the view and projection Matrix and returns 3 images:

        1. An RGB image
        2. A depth image
        3. A segmentation Mask, the segmentation mask indicates for every pixel the visible Object.

        From the given target_pose and cam_pose only the position is used.

        :param cam_pose: The pose of the camera
        :param target_pose: The pose to which the camera should point to
        :param size: The height and width of the images in pixel
        :return: A list containing an RGB and depth image as well as a segmentation mask, in this order.
        """
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


class Gui(threading.Thread):
    """
    For internal use only. Creates a new thread for the physics simulation that is active until closed by :func:`~World.exit`
    Also contains the code for controlling the camera.
    """

    def __init__(self, world: World, mode: str):
        threading.Thread.__init__(self)
        self.world = world
        self.mode: str = mode

    def run(self):
        """
        Initializes the new simulation and checks in an endless loop
        if it is still active. If it is the thread will be suspended for 1/80 seconds, if it is not the method and
        thus the thread terminates. The loop also checks for mouse and keyboard inputs to control the camera.
        """
        if self.mode != "GUI":
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
                keys = p.getKeyboardEvents(physicalClientId=self.world.client_id)
                mouse = p.getMouseEvents(physicalClientId=self.world.client_id)

                # Get infos about the camera
                width, height, dist = (p.getDebugVisualizerCamera(physicalClientId=self.world.client_id)[0],
                                       p.getDebugVisualizerCamera(physicalClientId=self.world.client_id)[1],
                                       p.getDebugVisualizerCamera(physicalClientId=self.world.client_id)[10])
                cameraTargetPosition = p.getDebugVisualizerCamera(physicalClientId=self.world.client_id)[11]

                # Get vectors used for movement on x,y,z Vector
                xVec = [p.getDebugVisualizerCamera(physicalClientId=self.world.client_id)[2][i] for i in [0, 4, 8]]
                yVec = [p.getDebugVisualizerCamera(physicalClientId=self.world.client_id)[2][i] for i in [2, 6, 10]]
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
                                             cameraTargetPosition=cameraTargetPosition,
                                             physicsClientId=self.world.client_id)
                if visible == 0:
                    cameraTargetPosition = (0.0, -50, 50)
                p.resetBasePositionAndOrientation(sphereUid, cameraTargetPosition, [0, 0, 0, 1],
                                                  physicsClientId=self.world.client_id)
                time.sleep(1. / 80.)
