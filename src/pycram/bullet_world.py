"""Implementation of the BulletWorld

Classes:
BulletWorld -- The Representation of the physics simulation
Gui -- Starts a new thread to keep the gui persistent
Object -- Representation of an object in the BulletWorld
"""
# used for delayed evaluation of typing until python 3.11 becomes mainstream
from __future__ import annotations

import pybullet as p
import os
import threading
import time
import pathlib
import logging
import rospkg
import re
from queue import Queue

import rospy
from typing import List, Optional, Union, Dict
from .event import Event
from .robot_descriptions.robot_description_handler import InitializedRobotDescription as robot_description
from typing import List, Optional, Dict, Tuple, Callable, Type


class BulletWorld:
    """
    The BulletWorld Class represents the physics Simulation.
    The Class variable 'current_bullet_world' always points to the currently used BulletWorld, usually this is the
    graphical one. However, if you are inside a Use_shadow_world() environment the current_bullet_world points to the
    shadow world. In this way you can comfortably use the current_bullet_world which should point towards the BulletWorld
    used at the moment.
    """

    current_bullet_world: BulletWorld = None
    robot: Object = None
    rospy.init_node('pycram')

    def __init__(self, type: str = "GUI", is_shadow_world: bool = False):
        """
        The constructor initializes a new simulation. The parameter decides if the Simulation should be graphical or
        non-graphical. It can only exist one graphical simulation at the time, but an arbitrary amount of non-graphical.
        The BulletWorld object also initializes the Events for attachment, detachment and for manipulating the world.

        :param type: Can either be "GUI" for graphical or "DIRECT" for non-graphical. The default parameter is "GUI"
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
        time.sleep(1) # 0.1
        #self.last_bullet_world = BulletWorld.current_bullet_world
        if BulletWorld.current_bullet_world == None:
            BulletWorld.current_bullet_world = self
        self.vis_axis: Object = None
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

        :param name: The name of the Objects that should be returned.
        :return: A list of all Objects with the name 'name'.
        """
        return list(filter(lambda obj: obj.name == name, self.objects))

    def get_objects_by_type(self, obj_type: str) -> List[Object]:
        """
        Returns a list of all Objects which have the type 'obj_type'.

        :param obj_type: The type of Objects that should be returned.
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
        return self.attachment_event

    def get_detachment_event(self) -> Event:
        return self.detachment_event

    def get_manipulation_event(self) -> Event:
        return self.manipulation_event

    def set_realtime(self, real_time: bool) -> None:
        """
        Enables the real time simulation of Physic in the BulletWorld. By default this is disabled and Physic is only
        simulated to reason about it.

        :param real_time: Whether the BulletWorld should simulate Physic in real time.
        :return: None
        """
        p.setRealTimeSimulation(1 if real_time else 0, self.client_id)

    def set_gravity(self, velocity: List[float]) -> None:
        """
        Sets the gravity that is used in the BullteWorld, by default the is the gravity on earth ([0, 0, -9.8]). Gravity
        is given as a vector in x,y,z. Gravity is only applied while simulating Physic.

        :param velocity: The gravity vector that should be used in the BulletWorld.
        :return: None
        """
        p.setGravity(velocity[0], velocity[1], velocity[2], physicsClientId=self.client_id)

    def set_robot(self, robot: Object) -> None:
        BulletWorld.robot = robot

    def simulate(self, seconds: float, real_time: Optional[float] = False) -> None:
        """
        Simulates Physic in the BulletWorld for a given amount of seconds. Usually this simulation is faster than real
        time, meaning you can simulate for example 10 seconds of Physic in the BulletWorld in 1 second real time. By
        setting the 'real_time' parameter this simulation is slowed down such that the simulated time is equal to real
        time.

        :param seconds: The amount of seconds that should be simulated.
        :param real_time: If the simulation should happen in real time or faster.
        :return: None
        """
        for i in range(0, int(seconds * 240)):
            p.stepSimulation(self.client_id)
            for objects, callback in self.coll_callbacks.items():
                contact_points = p.getContactPoints(objects[0].id, objects[1].id, physicsClientId=self.client_id)
                #contact_points = p.getClosestPoints(objects[0].id, objects[1].id, 0.02)
                #print(contact_points[0][5])
                if contact_points != ():
                    callback[0]()
                elif callback[1] != None: # Call no collision callback
                    callback[1]()
            if real_time:
                # Simulation runs at 240 Hz
                time.sleep(0.004167)

    def exit(self) -> None:
        # True if this is NOT the shadow world since it has a reference to the
        # Shadow world
        if self.shadow_world:
            self.world_sync.terminate = True
            self.world_sync.join()
            self.shadow_world.exit()
        p.disconnect(self.client_id)
        if self._gui_thread:
            self._gui_thread.join()
        if BulletWorld.current_bullet_world == self:
            BulletWorld.current_bullet_world = None

    def reset_bullet_world(self):
        """
        This function resets the BulletWorld to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they where spawned.
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
        Returns the id of the saved state of the BulletWorld
        """
        objects2attached = {}
        # ToDo find out what this is for and where it is used
        for o in self.objects:
            objects2attached[o] = (o.attachments.copy(), o.cids.copy())
        return p.saveState(self.client_id), objects2attached

    def restore_state(self, state, objects2attached: Dict={}) -> None:
        """
        Restores the state of the BulletWorld according to the given state id
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
        will be in Direct mode.

        :return: The reference to the new BulletWorld
        """
        world = BulletWorld("DIRECT")
        for obj in self.objects:
            o = Object(obj.name, obj.type, obj.path, obj.get_position(), obj.get_orientation(),
                            world, obj.color)
            for joint in obj.joints:
                o.set_joint_state(joint, obj.get_joint_state(joint))
        return world

    def add_vis_axis(self, position_and_orientation: Tuple[List[float], List[float]], length: Optional[float] = 0.2) -> None:
        """
        Creates a Visual object which represents the coordinate frame at the given
        position and orientation. There can only be one vis axis at a time. If this
        method is called again the previous visualization will be deleted.

        :param position_and_orientation: The position as vector of x,y,z and the
        orientation as a quanternion
        :param length: Optional parameter to configure the length of the axes
        """
        if self.vis_axis:
            p.removeBody(self.vis_axis)

        position = position_and_orientation[0]
        orientation = position_and_orientation[1]

        vis_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[length, 0.01, 0.01],
                            rgbaColor=[1, 0, 0, 0.8], visualFramePosition=[length, 0.01, 0.01])
        vis_y = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, length, 0.01],
                            rgbaColor=[0, 1, 0, 0.8], visualFramePosition=[0.01, length, 0.01])
        vis_z = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, length],
                            rgbaColor=[0, 0, 1, 0.8], visualFramePosition=[0.01, 0.01, length])

        obj = p.createMultiBody(baseVisualShapeIndex=-1, linkVisualShapeIndices=[vis_x, vis_y, vis_z],
            basePosition=position, baseOrientation=orientation, linkPositions=[[0,0,0], [0,0,0], [0,0,0]],
            linkMasses=[1.0,1.0,1.0], linkOrientations=[[0,0,0,1],[0,0,0,1],[0,0,0,1]],
            linkInertialFramePositions=[[0,0,0], [0,0,0], [0,0,0]],
            linkInertialFrameOrientations=[[0,0,0,1],[0,0,0,1],[0,0,0,1]],linkParentIndices=[0, 0, 0],
            linkJointTypes=[p.JOINT_FIXED, p.JOINT_FIXED, p.JOINT_FIXED], linkJointAxis=[[1,0,0], [0,1,0], [0,0,1]],
            linkCollisionShapeIndices=[-1, -1, -1])

        self.vis_axis = obj

    def remove_vis_axis(self):
        """
        Checks if there is a vis_axis objects in the BulletWorld and removes it,
        if there is one.
        """
        if self.vis_axis:
            p.removeBody(self.vis_axis)

    def register_collision_callback(self, objectA: Object, objectB: Object,
                                    callback_collision: Callable, callback_no_collision: Optional[Callable] = None) -> None:
        """
        This function regsiters can register two callbacks, one if objectA and objectB are in contact
        and another if they are not in contact.

        :param A: An object in the BulletWorld
        :param B: Another object in the BulletWorld
        :param callback_collision: A function that should be called if the obejcts are in contact
        :param callback_no_collision: A funtion that should be called if the objects are not in contact
        """
        self.coll_callbacks[(objectA, objectB)] = (callback_collision, callback_no_collision)

    def add_additional_resource_path(self, path: str):
        self.data_directory.append(path)

    def get_shadow_object(self, object: Object) -> Object:
        """
        Returns the corresponding object from the shadow world for the given object.

        :param object: The object for which the shadow worlds object should be returned.
        :return: The corresponding object in the shadow world.
        """
        try:
            return self.world_sync.object_mapping[object]
        except KeyError:
            raise ValueError(f"There is no shadow object for the given object: {object}, \
                    this could be the case if the object isn't anymore in the main (graphical) BulletWorld \
                    or if the given object is already a shadow object. ")

    def get_bullet_object_for_shadow(self, object: Object) -> Object:
        """
        Returns the corresponding object from the main Bullet World for a given
        object in the shadow world. If the  given object is not in the shadow
        world an error will be logged.

        :param object: The object for which the corresponding object in the
            main Bullet World should be found
        :return: The object in the main Bullet World
        """
        map = self.world_sync.object_mapping
        try:
            return list(map.keys())[list(map.values()).index(object)]
        except ValueError:
            raise ValueError("The given object is not in the shadow world.")

    def reset_bullet_world(self) -> None:
        """
        This function resets the BulletWorld to the state it was first spawned in.
        All attached objects will be detached, all joints will be set to the
        default position of 0 and all objects will be set to the position and
        orientation in which they where spawned.
        """
        for obj in self.objects:
            if obj.attachments:
                attached_objects = list(obj.attachments.keys())
                for att_obj in attached_objects:
                    obj.detach(att_obj)
            for joint_name in obj.joints.keys():
                obj.set_joint_state(joint_name, 0)
            obj.set_position_and_orientation(obj.original_pose[0], obj.original_pose[1])


#current_bullet_world = BulletWorld.current_bullet_world

class Use_shadow_world():

    def __init__(self):
        self.prev_world: BulletWorld = None

    def __enter__(self):
        if not BulletWorld.current_bullet_world.is_shadow_world:
            self.prev_world = BulletWorld.current_bullet_world
            BulletWorld.current_bullet_world.world_sync.pause_sync = True
            BulletWorld.current_bullet_world = BulletWorld.current_bullet_world.shadow_world

    def __exit__(self, *args):
        if not self.prev_world == None:
            BulletWorld.current_bullet_world = self.prev_world
            BulletWorld.current_bullet_world.world_sync.pause_sync = False


class World_Sync(threading.Thread):
    """
    This class synchronizes the state between the BulletWorld and its shadow world.
    Meaning the cartesian and joint position of everything the shadow world will be
    synchronized with the BulletWorld.
    Addding and removing objects is done via queues, such that loading times of objects
    in the shadow world does not affect the BulletWorld.
    The class provides the possibility to pause the synchronization, this can be used
    if reasoning should be done in the shadow world to guarantee a consistant state.
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
        The main method of the synchronization, this thread runs in a loop until the
        terminate flag is set.
        While this loop runs it contilously checks the cartesian and joint position of
        every object in the BulletWorld and updates the corresponding object in the
        shadow world.
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
            time.sleep(0.1)

        self.add_obj_queue.join()
        self.remove_obj_queue.join()

    def check_for_pause(self):
        while self.pause_sync:
            time.sleep(0.1)


class Gui(threading.Thread):
    """
    This class is for internal use only. It initializes the physics simulation
    in a new thread an holds it active.
    """
    def __init__(self, world, type):
        threading.Thread.__init__(self)
        self.world: BulletWorld = world
        self.type: str = type

    def run(self):
        """
        This method initializes the new simulation and checks in an endless loop
        if it is still active. If it is the
        thread will be suspended for 10 seconds, if it is not the method and
        thus the thread terminates.
        """
        if self.type == "GUI":
            self.world.client_id = p.connect(p.GUI)
        else:
            self.world.client_id = p.connect(p.DIRECT)

        while p.isConnected(self.world.client_id):
            time.sleep(0.5)


class Object:
    """
    This class represents an object in the BulletWorld.
    """

    def __init__(self, name: str, type: str, path: str,
                 position: Optional[List[float]]=[0, 0, 0],
                 orientation: Optional[List[float]]=[0, 0, 0, 1],
                 world: BulletWorld=None,
                 color: Optional[List[float]]=[1, 1, 1, 1],
                 ignoreCachedFiles: Optional[bool]=False):
        """
        The constructor loads the urdf file into the given BulletWorld, if no BulletWorld is specified the
        'current_bullet_world' will be used. It is also possible to load .obj and .stl file into the BulletWorld.
        The color parameter takes the color as rgba. This is only used when spawning .obj or .stl files and will be
        ignored for .urdf files.

        :param name: The name of the object
        :param type; The type of the object
        :param path: The path to the source file, it can be either .urdf, .obj or .stl
        :param position: The position in which the object should be spawned
        :param orientation: The orientation with which the object should be spawned
        :param world: The BulletWorld in which the object should be spawned, if no world is specified the 'current_bullet_world' will be used
        :param color: The color with which the object should be spawned.
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
        # This means "world" is not the shadow world since it has a reference to a shadow world
        if self.world.shadow_world != None:
            self.world.world_sync.add_obj_queue.put([name, type, path, position, orientation, self.world.shadow_world, color, self])

        if re.search("[a-zA-Z0-9].urdf", self.path):
            with open(self.path, mode="r") as f:
                urdf_string = f.read()
            robot_name = _get_robot_name_from_urdf(urdf_string)
            if robot_name == robot_description.i.name and BulletWorld.robot == None:
                BulletWorld.robot = self
        self.original_pose = [position, orientation]

    def remove(self) -> None:
        """
        This method removes this object from the BulletWorld it currently
        resides in.
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
        This method attaches an other object to this object. This is done by
        saving the transformation between the given link, if there is one, and
        the base pose of the other object. Additional the name of the link, to
        which the obejct is attached, will be saved.
        Furthermore, a constraint of pybullet will be created so the attachment
        also works in the simulation.
        Loose attachments means that the attachment will only be one-directional

        :param object: The other object that should be attached
        :param link: The link of this obejct to which the other object should be
        :param loose: If the attachment should be a loose attachment.
        attached.
        """
        link_id = self.get_link_id(link) if link else -1
        link_T_object = self._calculate_transform(object, link)
        self.attachments[object] = [link_T_object, link, loose]
        object.attachments[self] = [p.invertTransform(link_T_object[0], link_T_object[1]), None, False]

        cid = p.createConstraint(self.id, link_id, object.id, -1, p.JOINT_FIXED,
                            [0, 1, 0], link_T_object[0], [0, 0, 0], link_T_object[1], physicsClientId=self.world.client_id)
        self.cids[object] = cid
        object.cids[self] = cid
        self.world.attachment_event(self, [self, object])


    def detach(self, object: Object) -> None:
        """
        This method detaches an other object from this object. This is done by
        deleting the attachment from the attachments dictionaty of both obejects
        and deleting the constraint of pybullet.
        After the detachment the detachment event of the
        corresponding BulletWorld w:ill be fired.

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

        :return:
        """
        attachments = self.attachments.copy()
        for att in attachments.keys():
            self.detach(att)

    def get_position(self) -> List[float]:
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.world.client_id)[0]

    def get_pose(self) -> List[float]:
        return self.get_position()

    def get_orientation(self) -> List[float]:
        return p.getBasePositionAndOrientation(self.id, self.world.client_id)[1]

    def get_position_and_orientation(self) -> Tuple[List[float], List[float]]:
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.world.client_id)[:2]

    def set_position_and_orientation(self, position, orientation) -> None:
        p.resetBasePositionAndOrientation(self.id, position, orientation, self.world.client_id)
        self._set_attached_objects([self])

    def _set_attached_objects(self, prev_object: List[Object]) -> None:
        """
        This method updates the positions of all attached objects. This is done
        by calculating the new pose in world coordinate frame and setting the
        base pose of the attached objects to this new pose.
        After this the _set_attached_objects method of all attached objects
        will be called.

        :param prev_object: The object that called this method, this will be
        excluded to prevent recursion in the update.
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
                cid = p.createConstraint(self.id, link_id, obj.id, -1, p.JOINT_FIXED, [0, 0, 0], link_T_object[0], [0, 0, 0], link_T_object[1])
                self.cids[obj] = cid
                obj.cids[self] = cid
            else:
                # Updates the position of all attached objects
                link_T_object = self.attachments[obj][0]
                link_name = self.attachments[obj][1]
                world_T_link = self.get_link_position_and_orientation(link_name) if link_name else self.get_position_and_orientation()
                world_T_object = p.multiplyTransforms(world_T_link[0], world_T_link[1], link_T_object[0], link_T_object[1])
                p.resetBasePositionAndOrientation(obj.id, world_T_object[0], world_T_object[1])
                obj._set_attached_objects(prev_object + [self])

    def _calculate_transform(self, obj: Object, link: str) -> Tuple[List[float], List[float]]:
        """
        This method calculates the transformation between another object and the given
        link of this object. If no link is provided then the base position will be used.

        :param obj: The other object for which the transformation should be calculated
        :param link: The optional link name
        :return: The transformation from the link (or base position) to the other objects
            base position
        """
        link_id = self.get_link_id(link) if link else -1
        world_T_link =  self.get_link_position_and_orientation(link) if link else self.get_position_and_orientation()
        link_T_world = p.invertTransform(world_T_link[0], world_T_link[1])
        world_T_object = obj.get_position_and_orientation()
        link_T_object = p.multiplyTransforms(link_T_world[0], link_T_world[1],
                                              world_T_object[0], world_T_object [1], self.world.client_id)
        return link_T_object

    def set_position(self, position: List[float]) -> None:
        self.set_position_and_orientation(position, self.get_orientation())

    def set_orientation(self, orientation: List[float]) -> None:
        self.set_position_and_orientation(self.get_position(), orientation)

    def set_pose(self, position: List[float]) -> None:
        self.set_position(position)

    def _joint_or_link_name_to_id(self, type: str) -> Dict[str, int]:
        nJoints = p.getNumJoints(self.id, self.world.client_id)
        joint_name_to_id = {}
        info = 1 if type == "joint" else 12
        for i in range(0, nJoints):
            joint_info = p.getJointInfo(self.id, i, self.world.client_id)
            joint_name_to_id[joint_info[info].decode('utf-8')] = joint_info[0]
        return joint_name_to_id

    def get_joint_id(self, name: str) -> int:
        return self.joints[name]

    def get_link_id(self, name: str) -> int:
        return self.links[name]

    def get_link_relative_to_other_link(self, source_frame: str, target_frame: str) -> Tuple[List[float], List[float]]:

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
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[4:6]

    def get_link_position(self, name: str) -> List[float]:
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[4]

    def get_link_orientation(self, name: str) -> List[float]:
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[5]

    def set_joint_state(self, joint_name: str, joint_pose: float) -> None:
        up_lim, low_lim = p.getJointInfo(self.id, self.joints[joint_name], physicsClientId=self.world.client_id)[8:10]
        if low_lim > up_lim:
            low_lim, up_lim = up_lim, low_lim
        if not low_lim <= joint_pose <= up_lim:
            logging.error(f"The joint position has to be within the limits of the joint. The joint limits for {joint_name} are {low_lim} and {up_lim}")
            logging.error(f"The given joint position was: {joint_pose}")
            # Temporarily disabled because kdl outputs values exciting joint limits
            #return
        p.resetJointState(self.id, self.joints[joint_name], joint_pose, physicsClientId=self.world.client_id)
        self._set_attached_objects([self])

    def get_joint_state(self, joint_name: str) -> float:
        return p.getJointState(self.id, self.joints[joint_name], physicsClientId=self.world.client_id)[0]

    def contact_points(self) -> List:
        return p.getContactPoints(self.id)

    def contact_points_simulated(self) -> List:
        s = self.world.save_state()
        p.stepSimulation(self.world.client_id)
        contact_points = self.contact_points()
        self.world.restore_state(*s)
        return contact_points

    def set_color(self, color:  List[float], link: Optional[str] = "") -> None:
        """
        Changes the color of this object, the color has to be given as a list
        of RGBA values. Optionaly a link name can can be provided, if no link
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
        This method returns the color of this object or a link of this obejct. If no link is given then the
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
        :return: Two lists of x,y,z which define the bouning box.
        """
        if link_name:
            return p.getAABB(self.id, self.links[link_name], self.world.client_id)
        else:
            return p.getAABB(self.id, physicsClientId=self.world.client_id)

    def get_joint_limits(self, joint: str) -> Tuple[float, float]:
        """
        Returns the lower and upper limit of a joint, if the lower limit is higher
        than the upper they are swaped to ensure the lower limit is always the smaller one.

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
    This Method extracts the robot name from the 'robot_name' tag of a URDF.

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
    This method loads an object to the given BulletWorld with the given position and orientation. The color will only be
    used when an .obj or .stl file is given.
    If a .obj or .stl file is given then, before spawning, a urdf file with the .obj or .stl as mesh will be created
    and this .urdf file will be loaded instead.
    When spawning a .urdf file a new file will be created in the cache directory, if there exists none.
    This new file will have resolved mesh file paths, meaning there will be no resferences
    to ROS packges instead there will be absolute file paths.

    :param name: The name of the object which should be spawned
    :param path: The path to the source file or the name on the ROS parameter server
    :param position: The position in which the object should be spawned
    :param orientation: The orientation in which the object should be spawned
    :param world: The BulletWorld to which the Object should be spawned
    :param color: The color of the object, only used when .obj or .stl file is given
    :param ignoreCachedFiles: Whether to ignore files in the cache directory.
    :return: The unique id of the object
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
    #rospack = rospkg.RosPack()
    #cach_dir = rospack.get_path('pycram') + '/resources/cached/'
    cach_dir = world.data_directory[0] + '/cached/'
    if not pathlib.Path(cach_dir).exists():
        os.mkdir(cach_dir)

    # if file is not yet cached corrcet the urdf and save if in the cache directory
    if not _is_cached(path, name, cach_dir) or ignoreCachedFiles:
        if extension == ".obj" or extension == ".stl":
            path = _generate_urdf_file(name, path, color, cach_dir)
        elif extension == ".urdf":
            with open(path, mode="r") as f:
                urdf_string = f.read()
            path = cach_dir + pa.name
            with open(path, mode="w") as f:
                try:
                    f.write(_correct_urdf_string(urdf_string))
                except rospkg.ResourceNotFound as e:
                    os.remove(path)
                    raise e
        else: # Using the urdf from the parameter server
            urdf_string = rospy.get_param(path)
            path = cach_dir + name + ".urdf"
            with open(path, mode="w") as f:
                f.write(_correct_urdf_string(urdf_string))
    # save correct path in case the file is already in the cache directory
    elif extension == ".obj" or extension == ".stl":
        path = cach_dir + pa.stem  + ".urdf"
    elif extension == ".urdf":
        path = cach_dir + pa.name
    else:
        path = cach_dir + name + ".urdf"

    try:
        obj = p.loadURDF(path, basePosition=position, baseOrientation=orientation, physicsClientId=world_id)
        return obj, path
    except p.error as e:
        logging.error("The File could not be loaded. Plese note that the path has to be either a URDF, stl or obj file or the name of an URDF string on the parameter server.")
        os.remove(path)
        raise(e)
        #print(f"{bcolors.BOLD}{bcolors.WARNING}The path has to be either a path to a URDf file, stl file, obj file or the name of a URDF on the parameter server.{bcolors.ENDC}")


def _is_cached(path: str, name: str, cach_dir: str) -> bool:
    """
    This method checks if the file in the given path is already cached or if
    there is already a cached file with the given name, this is the case if a .stl,
    .obj file or a descriotion from the parameter server is used.

    :param path The path given by the user to the source file.
    :param name The name for this object.
    :param cach_dir The absolute path the the cach directory in the pycram package.
    :return True if there already exists a chached file, False in any other case.
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
    This method gets the name of a urdf description and feteches it from the ROS
    parameter server. Afterwards the URDF will be traversed and references to ROS packages
    will be replaced with the absolute path in the filesystem.

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

    return new_urdf_string


def _generate_urdf_file(name: str, path: str, color: List[float], cach_dir: str) -> str:
    """
    This method generates an .urdf file with the given .obj or .stl file as mesh. In addition the given color will be
    used to crate a material tag in the URDF. The resulting file will then be saved in the cach_dir path with the name
    as filename.

    :param name: The name of the object
    :param path: The path to the .obj or .stl file
    :param color: The color which should be used for the material tag
    :param cach_dir The absolute file path to the cach directory in the pycram package
    :return: The name of the generated .urdf file
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
    rgb = " ".join(list(map(str, color)))
    pathlib_obj = pathlib.Path(path)
    path = str(pathlib_obj.resolve())
    content = urdf_template.replace("~a", name).replace("~b", path).replace("~c", rgb)
    with open(cach_dir + pathlib_obj.stem + ".urdf", "w", encoding="utf-8") as file:
        file.write(content)
    return cach_dir + pathlib_obj.stem + ".urdf"


def _world_and_id(world: BulletWorld) -> Tuple[BulletWorld, int]:
    """
    This method selects the world to be used. If the given world is None the 'current_bullet_world' is used.
    
    :param world: The world which should be used or None if 'current_bullet_world' should be used
    :return: The BulletWorld object and the id of this BulletWorld
    """
    world = world if world is not None else BulletWorld.current_bullet_world
    id = world.client_id if world is not None else BulletWorld.current_bullet_world.client_id
    return world, id
