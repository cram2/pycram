"""Implementation of the BulletWorld

Classes:
BulletWorld -- The Representation of the physics simulation
Gui -- Starts a new thread to keep the gui persistent
Object -- Representation of an object in the BulletWorld
"""

import pybullet as p
import threading
import time
import pathlib
from .event import Event
from .helper import transform


class BulletWorld:
    """
    The BulletWorld Class represents the physics Simulation.
    The Class variable 'current_bullet_world' always points to the latest initialized BulletWorld.
    Every BulletWorld object holds the reference to the previous 'current_bullet_world' and if the 'exit' method
    of this BulletWorld will be called the previous 'current_bullet_world' becomes the new 'current_bullet_world'.
    """

    current_bullet_world = None
    robot = None

    def __init__(self, type="GUI"):
        """
        The constructor initializes a new simulation. The parameter decides if the Simulation should be graphical or
        non-graphical. It can only exist one graphical simulation at the time, but an arbitrary amount of non-graphical.

        The BulletWorld object also initializes the Events for attachment, detachment and for manipulating the world.
        :param type: Can either be "GUI" for graphical or "DIRECT" for non-graphical. The default parameter is "GUI"
        """
        self.objects = []
        self.client_id = -1
        self.detachment_event = Event()
        self.attachment_event = Event()
        self.manipulation_event = Event()
        self._gui_thread = Gui(self, type)
        self._gui_thread.start()
        time.sleep(0.1)
        self.last_bullet_world = BulletWorld.current_bullet_world
        BulletWorld.current_bullet_world = self

    def get_objects_by_name(self, name):
        return list(filter(lambda obj: obj.name == name, self.objects))

    def get_objects_by_type(self, obj_type):
        return list(filter(lambda obj: obj.type == obj_type, self.objects))

    def get_object_by_id(self, id):
        return list(filter(lambda obj: obj.id == id, self.objects))[0]

    def get_attachment_event(self):
        return self.attachment_event

    def get_detachment_event(self):
        return self.detachment_event

    def get_manipulation_event(self):
        return self.manipulation_event

    def set_realtime(self, real_time):
        p.setRealTimeSimulation(1 if real_time else 0, self.client_id)

    def set_gravity(self, velocity):
        p.setGravity(velocity[0], velocity[1], velocity[2], physicsClientId=self.client_id)

    def set_robot(self, robot):
        BulletWorld.robot = robot

    def simulate(self, seconds):
        for i in range(0, int(seconds * 240)):
            p.stepSimulation(self.client_id)

    def exit(self):
        BulletWorld.current_bullet_world = self.last_bullet_world
        p.disconnect(self.client_id)
        self._gui_thread.join()


current_bullet_world = BulletWorld.current_bullet_world


class Gui(threading.Thread):
    """
    This class is for internal use only. It initializes the physics simulation in a new thread an holds it active.
    """
    def __init__(self, world, type):
        threading.Thread.__init__(self)
        self.world = world
        self.type = type

    def run(self):
        """
        This method initializes the new simulation and checks in an endless loop if it is still active. If it is the
        thread will be suspended for 10 seconds, if it is not the method and thus the thread terminates.
        """
        if self.type == "GUI":
            self.world.client_id = p.connect(p.GUI)
        else:
            self.world.client_id = p.connect(p.DIRECT)

        while p.isConnected(self.world.client_id):
            time.sleep(10)


class Object:
    """
    This class represents an object in the BulletWorld.
    """

    def __init__(self, name, type, path, position=[0, 0, 0], orientation=[0, 0, 0, 1], world=None, color=[1, 1, 1, 1]):
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
        self.world = world if world is not None else BulletWorld.current_bullet_world
        self.name = name
        self.type = type
        self.path = path
        self.id = _load_object(name, path, position, orientation, world, color)
        self.joints = self._joint_or_link_name_to_id("joint")
        self.links = self._joint_or_link_name_to_id("link")
        self.attachments = {}
        self.world.objects.append(self)

    def attach(self, object, parent_link=None, child_link=None):
        """
        This method attaches two objects. This will be done by creating a virtual fixed joint between the two objects.
        After the attachment the attachment event of the BulletWorld will be fired.
        It can only exist one attachment between two objects.
        :param object: The object which should be attached to this object
        :param parent_link: The Name of the link of this object to which the other object should be attached or -1
                                for the base position
        :param child_link: The link Name of the other object to which this object should be attached or -1 for the base
        """
        if object in self.attachments:
            return
        parent_link_id = -1 if parent_link is None else self.links[parent_link]
        child_link_id = -1 if child_link is None else object.links[child_link]
        world_gripper = p.getLinkState(self.id, parent_link_id)[4] if parent_link_id != -1 else self.get_position()
        world_object = object.get_position()
        gripper_object = p.multiplyTransforms(p.invertTransform(world_gripper, [0, 0, 0, 1])[0], [0, 0, 0, 1],
                                              world_object, [0, 0, 0, 1], self.world.client_id)[0]
        cid = p.createConstraint(self.id, parent_link_id,
                                 object.id, child_link_id,
                                 p.JOINT_FIXED,
                                 [0, 1, 0], gripper_object, [0, 0, 0],
                                 physicsClientId=self.world.client_id)
        p.changeConstraint(cid, maxForce=30)
        self.attachments[object] = cid, parent_link_id
        object.attachments[self] = cid, parent_link_id
        self.world.attachment_event(self, [self, object])

    def detach(self, object):
        """
        This method detaches an other object from this object. After the detachment the detachment event of the
        corresponding BulletWorld w:ill be fired.
        :param object: The object which should be detached
        """
        if object not in self.attachments:
            return
        p.removeConstraint(self.attachments[object][0])
        del self.attachments[object]
        del object.attachments[self]
        self.world.detachment_event(self, [self, object])

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)[0]

    def get_pose(self):
        return self.get_position()

    def get_orientation(self):
        return p.getBasePositionAndOrientation(self.id)[1]

    def set_position_and_orientation(self, position, orientation):
        p.resetBasePositionAndOrientation(self.id, position, orientation, self.world.client_id)
        for at in self.attachments:
            lid = self.attachments[at][1]
            new_p = p.getLinkState(BulletWorld.robot.id, lid)[0]
            p.resetBasePositionAndOrientation(at.id, new_p, at.get_orientation(), self.world.client_id)
        self.world.simulate(1)

    def set_position(self, position):
        self.set_position_and_orientation(position, self.get_orientation())

    def set_orientation(self, orientation):
        p.resetBasePositionAndOrientation(self.id, self.get_position(), orientation, self.world.client_id)

    def set_pose(self, position):
        self.set_position(position)

    def set_joint(self, joint, pose):
        p.resetJointState(self.id, self.joints[joint], pose)

    def _joint_or_link_name_to_id(self, type):
        nJoints = p.getNumJoints(self.id)
        joint_name_to_id = {}
        info = 1 if type == "joint" else 12
        for i in range(0, nJoints):
            joint_info = p.getJointInfo(self.id, i)
            joint_name_to_id[joint_info[info].decode('utf-8')] = joint_info[0]
        return joint_name_to_id

    def get_joint_id(self, name):
        return self.joints[name]

    def get_link_id(self, name):
        return self.links[name]

    def get_link_position_and_orientation(self, name):
        return p.getLinkState(self.id, self.links[name])[:2]

    def get_link_position(self, name):
        return p.getLinkState(self.id, self.links[name])[0]

    def get_link_orientation(self, name):
        return p.getLinkState(self.id, self.links[name])[1]

    def set_joint_state(self, joint_name, joint_pose):
        p.resetJointState(self.id, self.get_joint_id(joint_name), joint_pose)


def _load_object(name, path, position, orientation, world, color):
    """
    This method loads an object to the given BulletWorld with the given position and orientation. The color will only be
    used when an .obj or .stl file is given.
    If a .obj or .stl file is given then, before spawning, a urdf file with the .obj or .stl as mesh will be created
    and this .urdf file will be loaded instead.
    :param name: The name of the object which should be spawned
    :param path: The path to the source file
    :param position: The position in which the object should be spawned
    :param orientation: The orientation in which the object should be spawned
    :param world: The BulletWorld to which the Object should be spawned
    :param color: The color of the object, only used when .obj or .stl file is given
    :return: The unique id of the object
    """
    extension = pathlib.Path(path).suffix
    world, world_id = _world_and_id(world)
    if extension == ".obj" or extension == ".stl":
        path = _generate_urdf_file(name, path, color)
    return p.loadURDF(path, basePosition=position, baseOrientation=orientation, physicsClientId=world_id)


def _generate_urdf_file(name, path, color):
    """
    This method generates an .urdf file with the given .obj or .stl file as mesh. In addition the given color will be
    used to crate a material tag in the URDF,
    :param name: The name of the object
    :param path: The path to the .obj or .stl file
    :param color: The color which should be used for the material tag
    :return: The name of the generated .urdf file
    """
    urdf_template = '<?xml version="0.0" ?> \n \
                        <robot name="~a"> \n \
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
    content = urdf_template.replace("~a", name).replace("~b", path).replace("~c", rgb)
    with open(name + ".urdf", "w", encoding="utf-8") as file:
        file.write(content)
    return name + ".urdf"


def _world_and_id(world):
    """
    This method selects the world to be used. If the given world is None the 'current_bullet_world' is used.
    :param world: The world which should be used or None if 'current_bullet_world' should be used
    :return: The BulletWorld object and the id of this BulletWorld
    """
    world = world if world is not None else BulletWorld.current_bullet_world
    id = world.client_id if world is not None else BulletWorld.current_bullet_world.client_id
    return world, id
