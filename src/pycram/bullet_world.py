# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import logging
import os
import pathlib
import re
import threading
import time
import xml.etree.ElementTree
from queue import Queue
import tf
from typing import List, Optional, Dict, Tuple, Callable
from typing import Union

import numpy as np
import pybullet as p
import rospkg
import rospy
import rosgraph

from .event import Event
from .robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from sensor_msgs.msg import JointState



class BulletWorld:
    """
    The BulletWorld Class represents the physics Simulation and belief state.
    """

    current_bullet_world: BulletWorld = None
    """
    Global reference to the currently used BulletWorld, usually this is the
    graphical one. However, if you are inside a Use_shadow_world() environment the current_bullet_world points to the
    shadow world. In this way you can comfortably use the current_bullet_world, which should point towards the BulletWorld
    used at the moment.
    """

    robot: Object = None
    """
    Global reference to the spawned Object that represents the robot. The robot is identified by checking the name in the 
    URDF with the name of the URDF on the parameter server. 
    """

    # Check is for sphinx autoAPI to be able to work in a CI workflow
    if rosgraph.is_master_online():
        rospy.init_node('pycram')

    def __init__(self, type: str = "GUI", is_shadow_world: bool = False):
        """
        Creates a new simulation, the type decides of the simulation should be a rendered window or just run in the
        background. There can only be one rendered simulation.
        The BulletWorld object also initializes the Events for attachment, detachment and for manipulating the world.

        :param type: Can either be "GUI" for rendered window or "DIRECT" for non-rendered. The default parameter is "GUI"
        :param is_shadow_world: For internal usage, decides if this BulletWorld should be used as a shadow world.
        """
        self.objects: List[Object] = []
        self.client_id: int = -1
        self.detachment_event: Event = Event()
        self.attachment_event: Event = Event()
        self.manipulation_event: Event = Event()
        self.type: str = type
        self._gui_thread: Gui = Gui(self, type)
        self._gui_thread.start()
        # This disables file caching from PyBullet, since this would also cache
        # files that can not be loaded
        p.setPhysicsEngineParameter(enableFileCaching=0)
        # Needed to let the other thread start the simulation, before Objects are spawned.
        time.sleep(0.1)
        if BulletWorld.current_bullet_world == None:
            BulletWorld.current_bullet_world = self
        self.vis_axis: Object = []
        self.coll_callbacks: Dict[Tuple[Object, Object], Tuple[Callable, Callable]] = {}
        self.data_directory: List[str] = [os.path.dirname(__file__) + "/../../resources"]
        self.shadow_world: BulletWorld = BulletWorld("DIRECT", True) if not is_shadow_world else None
        self.world_sync: World_Sync = World_Sync(self, self.shadow_world) if not is_shadow_world else None
        self.is_shadow_world: bool = is_shadow_world
        if not is_shadow_world:
            self.world_sync.start()

        # Some default settings
        self.set_gravity([0, 0, -9.8])
        if not is_shadow_world:
            plane = Object("floor", "environment", "plane.urdf", world=self)

    def get_objects_by_name(self, name: str) -> List[Object]:
        """
        Returns a list of all Objects in this BulletWorld with the same name as the given one.

        :param name: The name of the returned Objects.
        :return: A list of all Objects with the name 'name'.
        """
        return list(filter(lambda obj: obj.name == name, self.objects))

    def get_objects_by_type(self, obj_type: str) -> List[Object]:
        """
        Returns a list of all Objects which have the type 'obj_type'.

        :param obj_type: The type of the returned Objects.
        :return: A list of all Objects that have the type 'obj_type'.
        """
        return list(filter(lambda obj: obj.type == obj_type, self.objects))

    def get_object_by_id(self, id: int) -> Object:
        """
        Returns the single Object that has the unique id.

        :param id: The unique id for which the Object should be returned.
        :return: The Object with the id 'id'.
        """
        return list(filter(lambda obj: obj.id == id, self.objects))[0]

    def get_attachment_event(self) -> Event:
        """
        Returns the event reference that is fired if an attachment occurs.

        :return: The reference to the attachment event
        """
        return self.attachment_event

    def get_detachment_event(self) -> Event:
        """
        Returns the event reference that is fired if a detachment occurs.

        :return: The event reference for the detachment event.
        """
        return self.detachment_event

    def get_manipulation_event(self) -> Event:
        """
        Returns the event reference that is fired if any manipulation occurs.

        :return: The event reference for the manipulation event.
        """
        return self.manipulation_event

    def set_realtime(self, real_time: bool) -> None:
        """
        Enables the real time simulation of Physic in the BulletWorld. By default this is disabled and Physic is only
        simulated to reason about it.

        :param real_time: Whether the BulletWorld should simulate Physic in real time.
        """
        p.setRealTimeSimulation(1 if real_time else 0, self.client_id)

    def set_gravity(self, velocity: List[float]) -> None:
        """
        Sets the gravity that is used in the BullteWorld, by default the is the gravity on earth ([0, 0, -9.8]). Gravity
        is given as a vector in x,y,z. Gravity is only applied while simulating Physic.

        :param velocity: The gravity vector that should be used in the BulletWorld.
        """
        p.setGravity(velocity[0], velocity[1], velocity[2], physicsClientId=self.client_id)

    def set_robot(self, robot: Object) -> None:
        """
        Sets the global variable for the robot Object. This should be set on spawning the robot.

        :param robot: The Object reference to the Object representing the robot.
        """
        BulletWorld.robot = robot

    def simulate(self, seconds: float, real_time: Optional[float] = False) -> None:
        """
        Simulates Physic in the BulletWorld for a given amount of seconds. Usually this simulation is faster than real
        time, meaning you can simulate for example 10 seconds of Physic in the BulletWorld in 1 second real time. By
        setting the 'real_time' parameter this simulation is slowed down such that the simulated time is equal to real
        time.

        :param seconds: The amount of seconds that should be simulated.
        :param real_time: If the simulation should happen in real time or faster.
        """
        for i in range(0, int(seconds * 240)):
            p.stepSimulation(self.client_id)
            for objects, callback in self.coll_callbacks.items():
                contact_points = p.getContactPoints(objects[0].id, objects[1].id, physicsClientId=self.client_id)
                # contact_points = p.getClosestPoints(objects[0].id, objects[1].id, 0.02)
                # print(contact_points[0][5])
                if contact_points != ():
                    callback[0]()
                elif callback[1] != None:  # Call no collision callback
                    callback[1]()
            if real_time:
                # Simulation runs at 240 Hz
                time.sleep(0.004167)

    def exit(self) -> None:
        """
        Closes the BulletWorld as well as the shadow world, also collects any other thread that is running. This is the
        preferred method to close the BulletWorld.
        """
        # True if this is NOT the shadow world since it has a reference to the
        # Shadow world
        time.sleep(0.1)
        if self.shadow_world:
            self.world_sync.terminate = True
            self.world_sync.join()
            self.shadow_world.exit()
        p.disconnect(self.client_id)
        if self._gui_thread:
            self._gui_thread.join()
        if BulletWorld.current_bullet_world == self:
            BulletWorld.current_bullet_world = None
        BulletWorld.robot = None

    def reset_bullet_world(self) -> None:
        """
        Resets the BulletWorld to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they were spawned.
        """
        for obj in self.objects:
            if obj.attachments:
                attached_objects = list(obj.attachments.keys())
                for att_obj in attached_objects:
                    obj.detach(att_obj)
            for joint_name in obj.joints.keys():
                obj.set_joint_state(joint_name, 0)
            obj.set_position_and_orientation(obj.original_pose[0], obj.original_pose[1])

    def save_state(self) -> int:
        """
        Returns the id of the saved state of the BulletWorld. The saved state contains the position, orientation and joint
        position of every Object in the BulletWorld.

        :return: A unique id of the state
        """
        objects2attached = {}
        # ToDo find out what this is for and where it is used
        for o in self.objects:
            objects2attached[o] = (o.attachments.copy(), o.cids.copy())
        return p.saveState(self.client_id), objects2attached

    def restore_state(self, state, objects2attached: Dict = {}) -> None:
        """
        Restores the state of the BulletWorld according to the given state id. This includes position, orientation and
        joint states. However, restore can not respawn objects if there are objects that were deleted between creation of
        the state and restoring they will be skiped.

        :param state: The unique id representing the state, as returned by :func:`~save_state`
        :param objects2attached: A dictionary of attachments, as saved in :py:attr:`~bullet_world.Object.attachments`
        """
        p.restoreState(state, physicsClientId=self.client_id)
        for obj in self.objects:
            try:
                obj.attachments, obj.cids = objects2attached[obj]
            except KeyError:
                continue

    def copy(self) -> BulletWorld:
        """
        Copies this Bullet World into another and returns it. The other BulletWorld
        will be in Direct mode. The shadow world should always be preferred instead of creating a new BulletWorld.
        This method should only be used if necessary since there can be unforeseen problems.

        :return: The reference to the new BulletWorld
        """
        world = BulletWorld("DIRECT")
        for obj in self.objects:
            o = Object(obj.name, obj.type, obj.path, obj.get_position(), obj.get_orientation(),
                       world, obj.color)
            for joint in obj.joints:
                o.set_joint_state(joint, obj.get_joint_state(joint))
        return world

    def add_vis_axis(self, position_and_orientation: Tuple[List[float], List[float]],
                     length: Optional[float] = 0.2) -> None:
        """
        Creates a Visual object which represents the coordinate frame at the given
        position and orientation. There can be an unlimited amount of vis axis objects.

        :param position_and_orientation: The position as vector of x,y,z and the orientation as a quaternion
        :param length: Optional parameter to configure the length of the axes
        """

        position = position_and_orientation[0]
        orientation = position_and_orientation[1]

        vis_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[length, 0.01, 0.01],
                                    rgbaColor=[1, 0, 0, 0.8], visualFramePosition=[length, 0.01, 0.01])
        vis_y = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, length, 0.01],
                                    rgbaColor=[0, 1, 0, 0.8], visualFramePosition=[0.01, length, 0.01])
        vis_z = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, length],
                                    rgbaColor=[0, 0, 1, 0.8], visualFramePosition=[0.01, 0.01, length])

        obj = p.createMultiBody(baseVisualShapeIndex=-1, linkVisualShapeIndices=[vis_x, vis_y, vis_z],
                                basePosition=position, baseOrientation=orientation,
                                linkPositions=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                linkMasses=[1.0, 1.0, 1.0], linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
                                linkInertialFramePositions=[[0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 1]],
                                linkParentIndices=[0, 0, 0],
                                linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED],
                                linkJointAxis=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                linkCollisionShapeIndices=[-1, -1, -1])

        self.vis_axis.append(obj)

    def remove_vis_axis(self) -> None:
        """
        Removes all spawned vis axis objects that are currently in this BulletWorld.
        """
        for id in self.vis_axis:
            p.removeBody(id)

    def register_collision_callback(self, objectA: Object, objectB: Object,
                                    callback_collision: Callable,
                                    callback_no_collision: Optional[Callable] = None) -> None:
        """
        Registers callback methods for contact between two Objects. There can be a callback for when the two Objects
        get in contact and, optionally, for when they are not in contact anymore.

        :param objectA: An object in the BulletWorld
        :param objectB: Another object in the BulletWorld
        :param callback_collision: A function that should be called if the objects are in contact
        :param callback_no_collision: A function that should be called if the objects are not in contact
        """
        self.coll_callbacks[(objectA, objectB)] = (callback_collision, callback_no_collision)

    def add_additional_resource_path(self, path: str) -> None:
        """
        Adds a resource path in which the BulletWorld will search for files. This resource directory is searched if an
        Object is spawned only with a filename.

        :param path: A path in the filesystem in which to search for files.
        """
        self.data_directory.append(path)

    def get_shadow_object(self, object: Object) -> Object:
        """
        Returns the corresponding object from the shadow world for the given object. If the given Object is already in
        the shadow world it is returned.

        :param object: The object for which the shadow worlds object should be returned.
        :return: The corresponding object in the shadow world.
        """
        try:
            return self.world_sync.object_mapping[object]
        except KeyError:
            shadow_world = self if self.is_shadow_world else self.shadow_world
            if object in shadow_world.objects:
                return object
            else:
                raise ValueError(
                    f"There is no shadow object for the given object: {object}, this could be the case if the object isn't anymore in the main (graphical) BulletWorld or if the given object is already a shadow object. ")

    def get_bullet_object_for_shadow(self, object: Object) -> Object:
        """
        Returns the corresponding object from the main Bullet World for a given
        object in the shadow world. If the  given object is not in the shadow
        world an error will be raised.

        :param object: The object for which the corresponding object in the main Bullet World should be found
        :return: The object in the main Bullet World
        """
        map = self.world_sync.object_mapping
        try:
            return list(map.keys())[list(map.values()).index(object)]
        except ValueError:
            raise ValueError("The given object is not in the shadow world.")

    def reset_bullet_world(self) -> None:
        """
        Resets the BulletWorld to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they were spawned.
        """
        for obj in self.objects:
            if obj.attachments:
                attached_objects = list(obj.attachments.keys())
                for att_obj in attached_objects:
                    obj.detach(att_obj)
            for joint_name in obj.joints.keys():
                obj.set_joint_state(joint_name, 0)
            obj.set_position_and_orientation(obj.original_pose[0], obj.original_pose[1])


class Use_shadow_world():
    """
    An environment for using the shadow world, while in this environment the :py:attr:`~BulletWorld.current_bullet_world`
    variable will point to the shadow world.

    Example:
        with Use_shadow_world():
            NavigateAction.Action([[1, 0, 0], [0, 0, 0, 1]]).perform()
    """

    def __init__(self):
        self.prev_world: BulletWorld = None

    def __enter__(self):
        if not BulletWorld.current_bullet_world.is_shadow_world:
            time.sleep(3 / 240)
            # blocks until the adding queue is ready
            BulletWorld.current_bullet_world.world_sync.add_obj_queue.join()

            self.prev_world = BulletWorld.current_bullet_world
            BulletWorld.current_bullet_world.world_sync.pause_sync = True
            BulletWorld.current_bullet_world = BulletWorld.current_bullet_world.shadow_world

    def __exit__(self, *args):
        if not self.prev_world == None:
            BulletWorld.current_bullet_world = self.prev_world
            BulletWorld.current_bullet_world.world_sync.pause_sync = False


class World_Sync(threading.Thread):
    """
    Synchronizes the state between the BulletWorld and its shadow world.
    Meaning the cartesian and joint position of everything in the shadow world will be
    synchronized with the main BulletWorld.
    Adding and removing objects is done via queues, such that loading times of objects
    in the shadow world does not affect the BulletWorld.
    The class provides the possibility to pause the synchronization, this can be used
    if reasoning should be done in the shadow world.
    """

    def __init__(self, world: BulletWorld, shadow_world: BulletWorld):
        threading.Thread.__init__(self)
        self.world: BulletWorld = world
        self.shadow_world: BulletWorld = shadow_world
        self.shadow_world.world_sync: World_Sync = self

        self.terminate: bool = False
        self.add_obj_queue: Queue = Queue()
        self.remove_obj_queue: Queue = Queue()
        self.pause_sync: bool = False
        # Maps bullet to shadow world objects
        self.object_mapping: Dict[Object, Object] = {}

    def run(self):
        """
        Main method of the synchronization, this thread runs in a loop until the
        terminate flag is set.
        While this loop runs it continuously checks the cartesian and joint position of
        every object in the BulletWorld and updates the corresponding object in the
        shadow world. When there are entries in the adding or removing queue the corresponding objects will be added
        or removed in the same iteration.
        """
        while not self.terminate:
            self.check_for_pause()
            for i in range(self.add_obj_queue.qsize()):
                obj = self.add_obj_queue.get()
                # [name, type, path, position, orientation, self.world.shadow_world, color, bulletworld object]
                o = Object(obj[0], obj[1], obj[2], obj[3], obj[4], obj[5], obj[6])
                # Maps the BulletWorld object to the shadow world object
                self.object_mapping[obj[7]] = o
                self.add_obj_queue.task_done()
            for i in range(self.remove_obj_queue.qsize()):
                obj = self.remove_obj_queue.get()
                # Get shadow world object reference from object mapping
                shadow_obj = self.object_mapping[obj]
                shadow_obj.remove()
                del self.object_mapping[obj]
                self.remove_obj_queue.task_done()

            for bulletworld_obj, shadow_obj in self.object_mapping.items():
                shadow_obj.set_position(bulletworld_obj.get_position())
                shadow_obj.set_orientation(bulletworld_obj.get_orientation())

                # Manage joint positions
                if len(bulletworld_obj.joints) > 2:
                    for joint_name in bulletworld_obj.joints.keys():
                        shadow_obj.set_joint_state(joint_name, bulletworld_obj.get_joint_state(joint_name))

            self.check_for_pause()
            time.sleep(1 / 240)

        self.add_obj_queue.join()
        self.remove_obj_queue.join()

    def check_for_pause(self) -> None:
        """
        Checks if :py:attr:`~self.pause_sync` is true and sleeps this thread until it isn't anymore.
        """
        while self.pause_sync:
            time.sleep(0.1)


class Gui(threading.Thread):
    """
    For internal use only. Creates a new thread for the physics simulation that is active until closed by :func:`~BulletWorld.exit`
    Also contains the code for controlling the camera.
    """

    def __init__(self, world, type):
        threading.Thread.__init__(self)
        self.world: BulletWorld = world
        self.type: str = type

    def run(self):
        """
        Initializes the new simulation and checks in an endless loop
        if it is still active. If it is the thread will be suspended for 1/80 seconds, if it is not the method and
        thus the thread terminates. The loop also checks for mouse and keyboard inputs to control the camera.
        """
        if self.type != "GUI":
            self.world.client_id = p.connect(p.DIRECT)
        else:
            self.world.client_id = p.connect(p.GUI)

            # Get the initial camera target location
            cameraTargetPosition = p.getDebugVisualizerCamera()[11]

            sphereVisualId = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 0, 0, 1])

            # Create a sphere with a radius of 0.05 and a mass of 0
            sphereUid = p.createMultiBody(baseMass=0.0,
                                          baseInertialFramePosition=[0, 0, 0],
                                          baseVisualShapeIndex=sphereVisualId,
                                          basePosition=cameraTargetPosition)

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
                width, height, dist = p.getDebugVisualizerCamera()[0], p.getDebugVisualizerCamera()[1], \
                    p.getDebugVisualizerCamera()[10]
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


class Object:
    """
    Represents a spawned Object in the BulletWorld.
    """

    def __init__(self, name: str, type: str, path: str,
                 position: Optional[List[float]] = [0, 0, 0],
                 orientation: Optional[List[float]] = [0, 0, 0, 1],
                 world: BulletWorld = None,
                 color: Optional[List[float]] = [1, 1, 1, 1],
                 ignoreCachedFiles: Optional[bool] = False):
        """
        The constructor loads the urdf file into the given BulletWorld, if no BulletWorld is specified the
        :py:attr:`~BulletWorld.current_bullet_world` will be used. It is also possible to load .obj and .stl file into the BulletWorld.
        The color parameter is only used when loading .stl or .obj files, for URDFs :func:`~Object.set_color` can be used.

        :param name: The name of the object
        :param type: The type of the object
        :param path: The path to the source file, if only a filename is provided then the resourcer directories will be searched
        :param position: The position in which the object should be spawned, as xyz
        :param orientation: The orientation with which the object should be spawned, as quaternion
        :param world: The BulletWorld in which the object should be spawned, if no world is specified the :py:attr:`~BulletWorld.current_bullet_world` will be used
        :param color: The color with which the object should be spawned.
        :param ignoreCachedFiles: If true the file will be spawned while ignoring cached files.
        """
        self.world: BulletWorld = world if world is not None else BulletWorld.current_bullet_world
        self.name: str = name
        self.type: str = type
        self.color: List[float] = color
        self.id, self.path = _load_object(name, path, position, orientation, self.world, color, ignoreCachedFiles)
        self.joints: Dict[str, int] = self._joint_or_link_name_to_id("joint")
        self.links: Dict[str, int] = self._joint_or_link_name_to_id("link")
        self.attachments: Dict[Object, List] = {}
        self.cids: Dict[Object, int] = {}
        self.world.objects.append(self)
        self.original_pose = [position, orientation]
        self.base_origin_shift = np.array(position) - np.array(self.get_base_origin())

        # This means "world" is not the shadow world since it has a reference to a shadow world
        if self.world.shadow_world != None:
            self.world.world_sync.add_obj_queue.put(
                [name, type, path, position, orientation, self.world.shadow_world, color, self])

        if re.search("[a-zA-Z0-9].urdf", self.path):
            with open(self.path, mode="r") as f:

                urdf_string = f.read()
            urdf_string = urdf_string
            robot_name = _get_robot_name_from_urdf(urdf_string)
            if robot_name == robot_description.i.name and BulletWorld.robot == None:
                BulletWorld.robot = self
        self.original_pose = [position, orientation]

    def __repr__(self):
        return self.__class__.__qualname__ + f"(" + ', '.join(
            [f"{key}={value}" for key, value in self.__dict__.items()]) + ")"

    def remove(self) -> None:
        """
        Removes this object from the BulletWorld it currently resides in.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to PyBullet is done
        to remove this Object from the simulation.
        """
        for obj in self.attachments.keys():
            self.detach(obj)
        self.world.objects.remove(self)
        # This means the current world of the object is not the shaow world, since it
        # has a reference to the shadow world
        if self.world.shadow_world != None:
            self.world.world_sync.remove_obj_queue.put(self)
        p.removeBody(self.id, physicsClientId=self.world.client_id)

    def attach(self, object: Object, link: Optional[str] = None, loose: Optional[bool] = False) -> None:
        """
        Attaches another object to this object. This is done by
        saving the transformation between the given link, if there is one, and
        the base pose of the other object. Additionally, the name of the link, to
        which the object is attached, will be saved.
        Furthermore, a constraint of pybullet will be created so the attachment
        also works while simulation.
        Loose attachments means that the attachment will only be one-directional. For example, if this object moves the
        other, attached, object will also move but not the other way around.

        :param object: The other object that should be attached
        :param link: The link of this object to which the other object should be
        :param loose: If the attachment should be a loose attachment.
        """
        link_id = self.get_link_id(link) if link else -1
        link_T_object = self._calculate_transform(object, link)
        self.attachments[object] = [link_T_object, link, loose]
        object.attachments[self] = [p.invertTransform(link_T_object[0], link_T_object[1]), None, False]

        cid = p.createConstraint(self.id, link_id, object.id, -1, p.JOINT_FIXED,
                                 [0, 1, 0], link_T_object[0], [0, 0, 0], link_T_object[1],
                                 physicsClientId=self.world.client_id)
        self.cids[object] = cid
        object.cids[self] = cid
        self.world.attachment_event(self, [self, object])

    def detach(self, object: Object) -> None:
        """
        Detaches another object from this object. This is done by
        deleting the attachment from the attachments dictionary of both objects
        and deleting the constraint of pybullet.
        Afterward the detachment event of the corresponding BulletWorld will be fired.

        :param object: The object which should be detached
        """
        del self.attachments[object]
        del object.attachments[self]

        p.removeConstraint(self.cids[object], physicsClientId=self.world.client_id)

        del self.cids[object]
        del object.cids[self]
        self.world.detachment_event(self, [self, object])

    def detach_all(self) -> None:
        """
        Detach all objects attached to this object.
        """
        attachments = self.attachments.copy()
        for att in attachments.keys():
            self.detach(att)

    def get_position(self) -> List[float]:
        """
        Returns the position of this Object as a list of xyz.
        """
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.world.client_id)[0]

    def get_pose(self) -> List[float]:
        """
        Returns the position of this object as a list of xyz. Alias for :func:`~Object.get_position`.
        """
        return self.get_position()

    def get_orientation(self) -> List[float]:
        """
        Returns the orientation of this object as a list of xyzw, representing a quaternion.

        :return: A list of xyzw
        """
        return p.getBasePositionAndOrientation(self.id, self.world.client_id)[1]

    def get_position_and_orientation(self) -> Tuple[List[float], List[float]]:
        """
        Returns the position and quaternion of this Object. The position is a list of xyz and the orientation is a list
        of xyzw representing a quaternion.

        :return: A list of position and quaternion
        """
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.world.client_id)[:2]

    def set_position_and_orientation(self, position, orientation, base=False) -> None:
        """
        Set the position and the orientation of the object in the bullet world

        :param position: The xyz values to place the object at.
        :param orientation: The xyzw values to orient the object to.
        :param base: If True places the object base instead of origin at the specified position and orientation
        """
        if base:
            position = np.array(position) + self.base_origin_shift
        p.resetBasePositionAndOrientation(self.id, position, orientation, self.world.client_id)
        self._set_attached_objects([self])

    @property
    def pose(self) -> List[float]:
        """
        Property that returns the current position of this Object.

        :return: The position as a list of xyz
        """
        return self.get_pose()

    def move_base_to_origin_pos(self) -> None:
        """
        Move the object such that its base will be at the current origin position.
        This is useful when placing objects on surfaces where you want the object base in contact with the surface.
        """
        self.set_position_and_orientation(*self.get_position_and_orientation(), base=True)

    def _set_attached_objects(self, prev_object: List[Object]) -> None:
        """
        Updates the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this the _set_attached_objects method of all attached objects
        will be called.

        :param prev_object: A list of Objects that were already moved, this will be excluded to prevent recursion in the update.
        """
        for obj in self.attachments:
            if obj in prev_object:
                continue
            if self.attachments[obj][2]:
                # Updates the attachment transformation and contraint if the
                # attachment is loos, instead of updating the position of all attached objects
                link_T_object = self._calculate_transform(obj, self.attachments[obj][1])
                link_id = self.get_link_id(self.attachments[obj][1]) if self.attachments[obj][1] else -1
                self.attachments[obj][0] = link_T_object
                obj.attachments[self][0] = p.invertTransform(link_T_object[0], link_T_object[1])
                p.removeConstraint(self.cids[obj])
                cid = p.createConstraint(self.id, link_id, obj.id, -1, p.JOINT_FIXED, [0, 0, 0], link_T_object[0],
                                         [0, 0, 0], link_T_object[1])
                self.cids[obj] = cid
                obj.cids[self] = cid
            else:
                # Updates the position of all attached objects
                link_T_object = self.attachments[obj][0]
                link_name = self.attachments[obj][1]
                world_T_link = self.get_link_position_and_orientation(
                    link_name) if link_name else self.get_position_and_orientation()
                world_T_object = p.multiplyTransforms(world_T_link[0], world_T_link[1], link_T_object[0],
                                                      link_T_object[1])
                p.resetBasePositionAndOrientation(obj.id, world_T_object[0], world_T_object[1])
                obj._set_attached_objects(prev_object + [self])

    def _calculate_transform(self, obj: Object, link: str) -> Tuple[List[float], List[float]]:
        """
        Calculates the transformation between another object and the given
        link of this object. If no link is provided then the base position will be used.

        :param obj: The other object for which the transformation should be calculated
        :param link: The optional link name
        :return: The transformation from the link (or base position) to the other objects base position
        """
        link_id = self.get_link_id(link) if link else -1
        world_T_link = self.get_link_position_and_orientation(link) if link else self.get_position_and_orientation()
        link_T_world = p.invertTransform(world_T_link[0], world_T_link[1])
        world_T_object = obj.get_position_and_orientation()
        link_T_object = p.multiplyTransforms(link_T_world[0], link_T_world[1],
                                             world_T_object[0], world_T_object[1], self.world.client_id)
        return link_T_object

    def set_position(self, position: List[float], base=False) -> None:
        """
        Sets this Object to the given position, if base is true the bottom of the Object will be placed at the position
        instead of the origin in the center of the Object.

        :param position: Target position as xyz.
        :param base: If the bottom of the Object should be placed or the origin in the center.
        """
        self.set_position_and_orientation(position, self.get_orientation(), base=base)

    def set_orientation(self, orientation: List[float]) -> None:
        """
        Sets the orientation of the Object to the given orientation, orientation needs to be a quaternion.

        :param orientation: Target orientation given as a list of xyzw.
        """
        self.set_position_and_orientation(self.get_position(), orientation)

    def set_pose(self, position: List[float]) -> None:
        """
        Sets the position of this Object to the given position.

        :param position: Target position as a list of xyz.
        """
        self.set_position(position)

    def _joint_or_link_name_to_id(self, type: str) -> Dict[str, int]:
        """
        Creates a dictionary which maps the link or joint name to the unique ids used by pybullet.

        :param type: Determines if the dictionary should be for joints or links
        :return: A dictionary that maps joint or link names to unique ids
        """
        nJoints = p.getNumJoints(self.id, self.world.client_id)
        joint_name_to_id = {}
        info = 1 if type == "joint" else 12
        for i in range(0, nJoints):
            joint_info = p.getJointInfo(self.id, i, self.world.client_id)
            joint_name_to_id[joint_info[info].decode('utf-8')] = joint_info[0]
        return joint_name_to_id

    def get_joint_id(self, name: str) -> int:
        """
        Returns the unique id for a joint name. As used by PyBullet.

        :param name: The joint name
        :return: The unique id
        """
        return self.joints[name]

    def get_link_id(self, name: str) -> int:
        """
        Returns a unique id for a link name. As used by PyBullet.

        :param name: The link name
        :return: The unique id
        """
        return self.links[name]

    def get_link_relative_to_other_link(self, source_frame: str, target_frame: str) -> Tuple[List[float], List[float]]:
        """
        Calculates the position of a link in the coordinate frame of another link.

        :param source_frame: The name of the source frame
        :param target_frame: The name of the target frame
        :return: The position and orientation of the source frame in the target frame
        """

        # Get pose of source_frame in map (although pose is returned we use the transform style for clarity)
        map_T_source_trans, map_T_source_rot = self.get_link_position_and_orientation(source_frame)

        # Get pose of target_frame in map (although pose is returned we use the transform style for clarity)
        map_T_target_trans, map_T_target_rot = self.get_link_position_and_orientation(target_frame)

        # Calculate Pose of target frame relatively to source_frame
        source_T_map_trans, source_T_map_rot = p.invertTransform(map_T_source_trans, map_T_source_rot)
        source_T_target_trans, source_T_target_rot = p.multiplyTransforms(source_T_map_trans, source_T_map_rot,
                                                                          map_T_target_trans, map_T_target_rot)
        return source_T_target_trans, source_T_target_rot

    def get_link_position_and_orientation(self, name: str) -> List[float]:
        """
        Returns the position and orientation of a link of this Object. Position is returned as xyz and orientation as a
        quaternion.

        :param name: The link name
        :return: The position and orientation
        """
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[4:6]

    def get_link_position(self, name: str) -> List[float]:
        """
        Returns the position of a link of this Object. Position is returned as a list of xyz.

        :param name: The link name
        :return: The link position as xyz
        """
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[4]

    def get_link_orientation(self, name: str) -> List[float]:
        """
        Returns the orientation of a link of this Object. Orientation is returned as a quaternion.

        :param name: The name of the link
        :return: The orientation of the link as a quaternion
        """
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[5]

    def set_joint_state(self, joint_name: str, joint_pose: float) -> None:
        """
        Sets the state of the given joint to the given joint pose. If the pose is outside the joint limits, as stated
        in the URDF, an error will be printed. However, the joint will be set either way.

        :param joint_name: The name of the joint
        :param joint_pose: The target pose for this joint
        """
        # TODO Limits for rotational (infinitie) joints are 0 and 1, they should be considered seperatly
        up_lim, low_lim = p.getJointInfo(self.id, self.joints[joint_name], physicsClientId=self.world.client_id)[8:10]
        if low_lim > up_lim:
            low_lim, up_lim = up_lim, low_lim
        if not low_lim <= joint_pose <= up_lim:
            logging.error(
                f"The joint position has to be within the limits of the joint. The joint limits for {joint_name} are {low_lim} and {up_lim}")
            logging.error(f"The given joint position was: {joint_pose}")
            # Temporarily disabled because kdl outputs values exciting joint limits
            # return
        p.resetJointState(self.id, self.joints[joint_name], joint_pose, physicsClientId=self.world.client_id)
        self._set_attached_objects([self])

    def get_joint_state(self, joint_name: str) -> float:
        """
        Returns the joint state for the given joint name.

        :param joint_name: The name of the joint
        :return: The current pose of the joint
        """
        return p.getJointState(self.id, self.joints[joint_name], physicsClientId=self.world.client_id)[0]

    def contact_points(self) -> List:
        """
        Returns a list of contact points of this Object with other Objects. For a more detailed explanation of the returned
        list please look at `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_

        :return: A list of all contact points with other objects
        """
        return p.getContactPoints(self.id)

    def contact_points_simulated(self) -> List:
        """
        Returns a list of all contact points between this Object and other Objects after stepping the simulation once.
        For a more detailed explanation of the returned
        list please look at `PyBullet Doc <https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#>`_

        :return: A list of contact points between this Object and other Objects
        """
        s = self.world.save_state()
        p.stepSimulation(self.world.client_id)
        contact_points = self.contact_points()
        self.world.restore_state(*s)
        return contact_points

    def update_joints_from_topic(self, topic_name: str) -> None:
        """
        Updates the joints of this object with positions obtained from a topic with the message type JointState.
        Joint names on the topic have to correspond to the joints of this object otherwise an error message will be logged.

        :param topic_name: Name of the topic with the joint states
        """
        msg = rospy.wait_for_message(topic_name, JointState)
        joint_names = msg.name
        joint_positions = msg.position
        if set(joint_names).issubset(self.joints.keys()):
            for i in range(len(joint_names)):
                self.set_joint_state(joint_names[i], joint_positions[i])
        else:
            add_joints = set(joint_names) - set(self.joints.keys())
            rospy.logerr(f"There are joints in the published joint state which are not in this model: /n \
                        The following joint{'s' if len(add_joints) != 1 else ''}: {add_joints}")

    def update_position_from_tf(self, frame: str) -> None:
        """
        Updates the position of this object from a TF message.

        :param frame: Name of the TF frame from which the position should be taken
        """
        tf_listener = tf.TransformListener()
        time.sleep(0.5)
        position = tf_listener.lookupTransform(frame, "map", rospy.Time(0))
        self.set_position([position[0][0] * -1,
                           position[0][1] * -1,
                           position[0][2],
                           position[1]])

    def set_color(self, color: List[float], link: Optional[str] = "") -> None:
        """
        Changes the color of this object, the color has to be given as a list
        of RGBA values. Optionally a link name can can be provided, if no link
        name is provided all links of this object will be colored.

        :param color: The color as RGBA values between 0 and 1
        :param link: The link name of the link which should be colored
        """
        if link == "":
            # Check if there is only one link, this is the case for primitive
            # forms or if loaded from an .stl or .obj file
            if self.links != {}:
                for link_id in self.links.values():
                    p.changeVisualShape(self.id, link_id, rgbaColor=color, physicsClientId=self.world.client_id)
            else:
                p.changeVisualShape(self.id, -1, rgbaColor=color, physicsClientId=self.world.client_id)
        else:
            p.changeVisualShape(self.id, self.links[link], rgbaColor=color, physicsClientId=self.world.client_id)

    def get_color(self, link: Optional[str] = None) -> Union[List[float], Dict[str, List[float]], None]:
        """
        This method returns the color of this object or a link of this object. If no link is given then the
        return is either:
            1. A list with the color as RGBA values, this is the case if the object only has one link (this
                happens for example if the object is spawned from a .obj or .stl file)
            2. A dict with the link name as key and the color as value. The color is given as RGBA values.
                Please keep in mind that not every link may have a color. This is dependent on the URDF from which the
                object is spawned.
        If a link is specified then the return is a list with RGBA values representing the color of this link.
        It may be that this link has no color, in this case the return is None as well as an error message.

        :param link: the link name for which the color should be returned.
        :return: The color of the object or link, or a dictionary containing every colored link with its color
        """
        visual_data = p.getVisualShapeData(self.id, physicsClientId=self.world.client_id)
        swap = {v: k for k, v in self.links.items()}
        links = list(map(lambda x: swap[x[1]] if x[1] != -1 else "base", visual_data))
        colors = list(map(lambda x: x[7], visual_data))
        link_to_color = dict(zip(links, colors))

        if link:
            if link in link_to_color.keys():
                return link_to_color[link]
            elif link not in self.links.keys():
                rospy.logerr(f"The link '{link}' is not part of this obejct")
                return None
            else:
                rospy.logerr(f"The link '{link}' has no color")
                return None

        if len(visual_data) == 1:
            return list(visual_data[0][7])
        else:
            return link_to_color

    def get_AABB(self, link_name: Optional[str] = None) -> Tuple[List[float], List[float]]:
        """
        Returns the axis aligned bounding box of this object, optionally a link name can be provided in this case
        the axis aligned bounding box of the link will be returned. The return of this method are two points in
        world coordinate frame which define a bounding box.

        :param link_name: The Optional name of a link of this object.
        :return: Two lists of x,y,z which define the bounding box.
        """
        if link_name:
            return p.getAABB(self.id, self.links[link_name], self.world.client_id)
        else:
            return p.getAABB(self.id, physicsClientId=self.world.client_id)

    def get_base_origin(self, link_name: Optional[str] = None) -> List[float]:
        """
        Returns the origin of the base/bottom of an object/link

        :param link_name: The link name for which the bottom position should be returned
        :return: The position of the bottom of this Object or link
        """
        aabb = self.get_AABB(link_name=link_name)
        base_width = np.absolute(aabb[0][0] - aabb[1][0])
        base_length = np.absolute(aabb[0][1] - aabb[1][1])
        return [aabb[0][0] + base_width / 2, aabb[0][1] + base_length / 2, aabb[0][2]]

    def get_joint_limits(self, joint: str) -> Tuple[float, float]:
        """
        Returns the lower and upper limit of a joint, if the lower limit is higher
        than the upper they are swapped to ensure the lower limit is always the smaller one.

        :param joint: The name of the joint for which the limits should be found.
        :return: The lower and upper limit of the joint.
        """
        if joint not in self.joints.keys():
            raise KeyError(f"The given Joint: {joint} is not part of this object")
        lower, upper = p.getJointInfo(self.id, self.joints[joint], self.world.client_id)[8: 10]
        if lower > upper:
            lower, upper = upper, lower
        return lower, upper


def filter_contact_points(contact_points, exclude_ids) -> List:
    """
    Returns a list of contact points where Objects that are in the 'exclude_ids' list are removed.

    :param contact_points: A list of contact points
    :param exclude_ids: A list of unique ids of Objects that should be removed from the list
    :return: A list containing 'contact_points' without Objects that are in 'exclude_ids'
    """
    return list(filter(lambda cp: cp[2] not in exclude_ids, contact_points))


def get_path_from_data_dir(file_name: str, data_directory: str) -> str:
    """
    Returns the full path for a given file name in the given directory. If there is no file with the given filename
    this method returns None.

    :param file_name: The filename of the searched file.
    :param data_directory: The directory in which to search for the file.
    :return: The full path in the filesystem or None if there is no file with the filename in the directory
    """
    dir = pathlib.Path(data_directory)
    for file in os.listdir(data_directory):
        if file == file_name:
            return data_directory + f"/{file_name}"


def _get_robot_name_from_urdf(urdf_string: str) -> str:
    """
    Extracts the robot name from the 'robot_name' tag of a URDF.

    :param urdf_string: The URDF as string.
    :return: The name of the robot described by the URDF.
    """
    res = re.findall(r"robot\ *name\ *=\ *\"\ *[a-zA-Z_0-9]*\ *\"", urdf_string)
    if len(res) == 1:
        begin = res[0].find("\"")
        end = res[0][begin + 1:].find("\"")
        robot = res[0][begin + 1:begin + 1 + end].lower()
    return robot


def _load_object(name: str,
                 path: str,
                 position: List[float],
                 orientation: List[float],
                 world: BulletWorld,
                 color: List[float],
                 ignoreCachedFiles: bool) -> Tuple[int, str]:
    """
    Loads an object to the given BulletWorld with the given position and orientation. The color will only be
    used when an .obj or .stl file is given.
    If a .obj or .stl file is given, before spawning, an urdf file with the .obj or .stl as mesh will be created
    and this URDf file will be loaded instead.
    When spawning a URDf file a new file will be created in the cache directory, if there exists none.
    This new file will have resolved mesh file paths, meaning there will be no references
    to ROS packges instead there will be absolute file paths.

    :param name: The name of the object which should be spawned
    :param path: The path to the source file or the name on the ROS parameter server
    :param position: The position in which the object should be spawned
    :param orientation: The orientation in which the object should be spawned
    :param world: The BulletWorld to which the Object should be spawned
    :param color: The color of the object, only used when .obj or .stl file is given
    :param ignoreCachedFiles: Whether to ignore files in the cache directory.
    :return: The unique id of the object and the path to the file used for spawning
    """
    pa = pathlib.Path(path)
    extension = pa.suffix
    world, world_id = _world_and_id(world)
    if re.match("[a-zA-Z_0-9].[a-zA-Z0-9]", path):
        for dir in world.data_directory:
            path = get_path_from_data_dir(path, dir)
            if path: break

    if not path:
        raise FileNotFoundError(f"File {pa.name} could not be found in the resource directory {world.data_directory}")
    # rospack = rospkg.RosPack()
    # cach_dir = rospack.get_path('pycram') + '/resources/cached/'
    cach_dir = world.data_directory[0] + '/cached/'
    if not pathlib.Path(cach_dir).exists():
        os.mkdir(cach_dir)

    # if file is not yet cached corrcet the urdf and save if in the cache directory
    if not _is_cached(path, name, cach_dir) or ignoreCachedFiles:
        if extension == ".obj" or extension == ".stl":
            path = _generate_urdf_file(name, path, color, cach_dir)
        elif extension == ".urdf":
            with open(path, mode="r") as f:
                urdf_string = fix_missing_inertial(f.read())
            path = cach_dir + pa.name
            with open(path, mode="w") as f:
                try:
                    f.write(_correct_urdf_string(urdf_string))
                except rospkg.ResourceNotFound as e:
                    os.remove(path)
                    raise e
        else:  # Using the urdf from the parameter server
            urdf_string = rospy.get_param(path)
            path = cach_dir + name + ".urdf"
            with open(path, mode="w") as f:
                f.write(_correct_urdf_string(urdf_string))
    # save correct path in case the file is already in the cache directory
    elif extension == ".obj" or extension == ".stl":
        path = cach_dir + pa.stem + ".urdf"
    elif extension == ".urdf":
        path = cach_dir + pa.name
    else:
        path = cach_dir + name + ".urdf"

    try:
        obj = p.loadURDF(path, basePosition=position, baseOrientation=orientation, physicsClientId=world_id)
        return obj, path
    except p.error as e:
        logging.error(
            "The File could not be loaded. Plese note that the path has to be either a URDF, stl or obj file or the name of an URDF string on the parameter server.")
        os.remove(path)
        raise (e)
        # print(f"{bcolors.BOLD}{bcolors.WARNING}The path has to be either a path to a URDf file, stl file, obj file or the name of a URDF on the parameter server.{bcolors.ENDC}")


def _is_cached(path: str, name: str, cach_dir: str) -> bool:
    """
    Checks if the file in the given path is already cached or if
    there is already a cached file with the given name, this is the case if a .stl, .obj file or a description from
    the parameter server is used.

    :param path: The path given by the user to the source file.
    :param name: The name for this object.
    :param cach_dir: The absolute path the cach directory in the pycram package.
    :return: True if there already exists a chached file, False in any other case.
    """
    file_name = pathlib.Path(path).name
    p = pathlib.Path(cach_dir + file_name)
    if p.exists():
        return True
    # Returns filename without the filetype, e.g. returns "test" for "test.txt"
    file_stem = pathlib.Path(path).stem
    p = pathlib.Path(cach_dir + file_stem + ".urdf")
    if p.exists():
        return True
    return False


def _correct_urdf_string(urdf_string: str) -> str:
    """
    Changes paths for files in the URDF from ROS paths to paths in the file system. Since PyBullet can't deal with ROS
    package paths.

    :param urdf_name: The name of the URDf on the parameter server
    :return: The URDF string with paths in the filesystem instead of ROS packages
    """
    r = rospkg.RosPack()
    new_urdf_string = ""
    for line in urdf_string.split('\n'):
        if "package://" in line:
            s = line.split('//')
            s1 = s[1].split('/')
            path = r.get_path(s1[0])
            line = line.replace("package://" + s1[0], path)
        new_urdf_string += line + '\n'

    return fix_missing_inertial(new_urdf_string)


def fix_missing_inertial(urdf_string: str) -> str:
    """
    Insert inertial tags for every URDF link that has no inertia.
    This is used to prevent PyBullet from dumping warnings in the terminal

    :param urdf_string: The URDF description as string
    :returns: The new, corrected URDF description as string.
    """

    inertia_tree = xml.etree.ElementTree.ElementTree(xml.etree.ElementTree.Element("inertial"))
    inertia_tree.getroot().append(xml.etree.ElementTree.Element("mass", {"value": "0.1"}))
    inertia_tree.getroot().append(xml.etree.ElementTree.Element("origin", {"rpy": "0 0 0", "xyz": "0 0 0"}))
    inertia_tree.getroot().append(xml.etree.ElementTree.Element("inertia", {"ixx": "0.01",
                                                                            "ixy": "0",
                                                                            "ixz": "0",
                                                                            "iyy": "0.01",
                                                                            "iyz": "0",
                                                                            "izz": "0.01"}))

    # create tree from string
    tree = xml.etree.ElementTree.ElementTree(xml.etree.ElementTree.fromstring(urdf_string))

    for link_element in tree.iter("link"):
        inertial = [*link_element.iter("inertial")]
        if len(inertial) == 0:
            link_element.append(inertia_tree.getroot())

    return xml.etree.ElementTree.tostring(tree.getroot(), encoding='unicode')


def _generate_urdf_file(name: str, path: str, color: List[float], cach_dir: str) -> str:
    """
    Generates an URDf file with the given .obj or .stl file as mesh. In addition, the given color will be
    used to crate a material tag in the URDF. The resulting file will then be saved in the cach_dir path with the name
    as filename.

    :param name: The name of the object
    :param path: The path to the .obj or .stl file
    :param color: The color which should be used for the material tag
    :param cach_dir The absolute file path to the cach directory in the pycram package
    :return: The absolute path of the created file
    """
    urdf_template = '<?xml version="0.0" ?> \n \
                        <robot name="~a_object"> \n \
                         <link name="~a_main"> \n \
                            <visual> \n \
                                <geometry>\n \
                                    <mesh filename="~b" scale="1 1 1"/> \n \
                                </geometry>\n \
                                <material name="white">\n \
                                    <color rgba="~c"/>\n \
                                </material>\n \
                          </visual> \n \
                        <collision> \n \
                        <geometry>\n \
                            <mesh filename="~b" scale="1 1 1"/>\n \
                        </geometry>\n \
                        </collision>\n \
                        </link> \n \
                        </robot>'
    urdf_template = fix_missing_inertial(urdf_template)
    rgb = " ".join(list(map(str, color)))
    pathlib_obj = pathlib.Path(path)
    path = str(pathlib_obj.resolve())
    content = urdf_template.replace("~a", name).replace("~b", path).replace("~c", rgb)
    with open(cach_dir + pathlib_obj.stem + ".urdf", "w", encoding="utf-8") as file:
        file.write(content)
    return cach_dir + pathlib_obj.stem + ".urdf"


def _world_and_id(world: BulletWorld) -> Tuple[BulletWorld, int]:
    """
    Selects the world to be used. If the given world is None the 'current_bullet_world' is used.
    
    :param world: The world which should be used or None if 'current_bullet_world' should be used
    :return: The BulletWorld object and the id of this BulletWorld
    """
    world = world if world is not None else BulletWorld.current_bullet_world
    id = world.client_id if world is not None else BulletWorld.current_bullet_world.client_id
    return world, id
