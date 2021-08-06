"""Implementation of the BulletWorld

Classes:
BulletWorld -- The Representation of the physics simulation
Gui -- Starts a new thread to keep the gui persistent
Object -- Representation of an object in the BulletWorld
"""

import pybullet as p
import os
import threading
import time
import pathlib
import rospy
import rospkg
from .event import Event
from .helper import bcolors

class BulletWorld:
    """
    The BulletWorld Class represents the physics Simulation.
    The Class variable 'current_bullet_world' always points to the latest initialized BulletWorld.
    Every BulletWorld object holds the reference to the previous 'current_bullet_world' and if the 'exit' method
    of this BulletWorld will be called the previous 'current_bullet_world' becomes the new 'current_bullet_world'.
    """

    current_bullet_world = None
    robot = None
    node = rospy.init_node('pycram')

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
        time.sleep(1) # 0.1
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

    def save_state(self):
        """
        Returns the id of the saved state of the BulletWorld
        """
        objects2attached = {}
        for o in self.objects:
            objects2attached[o] = (o.attachments.copy(), o.cids.copy())
        return p.saveState(self.client_id), objects2attached

    def restore_state(self, state, objects2attached={}):
        """
        Restores the state of the BulletWorld according to the given state id
        """
        p.restoreState(state, physicsClientId=self.client_id)
        for obj in self.objects:
            try:
                obj.attachments, obj.cids = objects2attached[obj]
            except KeyError:
                continue

    def copy(self):
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



current_bullet_world = BulletWorld.current_bullet_world


class Gui(threading.Thread):
    """
    This class is for internal use only. It initializes the physics simulation
    in a new thread an holds it active.
    """
    def __init__(self, world, type):
        threading.Thread.__init__(self)
        self.world = world
        self.type = type

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

    def __init__(self, name, type, path, position=[0, 0, 0], orientation=[0, 0, 0, 1], world=None, color=[1, 1, 1, 1], ignoreCachedFiles=False):
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
        self.color = color
        self.id = _load_object(name, path, position, orientation, world, color, ignoreCachedFiles)
        self.joints = self._joint_or_link_name_to_id("joint")
        self.links = self._joint_or_link_name_to_id("link")
        self.attachments = {}
        self.cids = {}
        self.world.objects.append(self)

    def remove(self):
        """
        This method removes this object from the BulletWorld it currently
        resides in.
        For the object to be removed it has to be detached from all objects it
        is currently attached to. After this is done a call to PyBullet is done
        to remove this Object from the simulation.
        """
        for obj in self.attachments.keys():
            self.detach(obj)
        p.removeBody(self.id, physicsClientId=self.world.client_id)

    def attach(self, object, link=None, loose=False):
        """
        This method attaches an other object to this object. This is done by
        saving the transformation between the given link, if there is one, and
        the base pose of the other object. Additional the name of the link, to
        which the obejct is attached, will be saved.
        Furthermore, a constraint of pybullet will be created so the attachment
        also works in the simulation.
        :param object: The other object that should be attached
        :param link: The link of this obejct to which the other object should be
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


    def detach(self, object):
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

    def detach_all(self):
        """
        Detach all objects attached to this object.
        :return:
        """
        attachments = self.attachments.copy()
        for att in attachments.keys():
            self.detach(att)

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.world.client_id)[0]

    def get_pose(self):
        return self.get_position()

    def get_orientation(self):
        return p.getBasePositionAndOrientation(self.id, self.world.client_id)[1]

    def get_position_and_orientation(self):
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.world.client_id)[:2]

    def set_position_and_orientation(self, position, orientation):
        p.resetBasePositionAndOrientation(self.id, position, orientation, self.world.client_id)
        self._set_attached_objects([self])

    def _set_attached_objects(self, prev_object):
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

    def _calculate_transform(self, obj, link):
        link_id = self.get_link_id(link) if link else -1
        world_T_link =  self.get_link_position_and_orientation(link) if link else self.get_position_and_orientation()
        link_T_world = p.invertTransform(world_T_link[0], world_T_link[1])
        world_T_object = obj.get_position_and_orientation()
        link_T_object = p.multiplyTransforms(link_T_world[0], link_T_world[1],
                                              world_T_object[0], world_T_object [1], self.world.client_id)
        return link_T_object

    def set_position(self, position):
        self.set_position_and_orientation(position, self.get_orientation())

    def set_orientation(self, orientation):
        self.set_position_and_orientation(self.get_position(), orientation)

    def set_pose(self, position):
        self.set_position(position)

    def _joint_or_link_name_to_id(self, type):
        nJoints = p.getNumJoints(self.id, self.world.client_id)
        joint_name_to_id = {}
        info = 1 if type == "joint" else 12
        for i in range(0, nJoints):
            joint_info = p.getJointInfo(self.id, i, self.world.client_id)
            joint_name_to_id[joint_info[info].decode('utf-8')] = joint_info[0]
        return joint_name_to_id

    def get_joint_id(self, name):
        return self.joints[name]

    def get_link_id(self, name):
        return self.links[name]

    def get_link_relative_to_other_link(self, source_frame, target_frame):

        # Get pose of source_frame in map (although pose is returned we use the transform style for clarity)
        map_T_source_trans, map_T_source_rot = self.get_link_position_and_orientation(source_frame)

        # Get pose of target_frame in map (although pose is returned we use the transform style for clarity)
        map_T_target_trans, map_T_target_rot = self.get_link_position_and_orientation(target_frame)

        # Calculate Pose of target frame relatively to source_frame
        source_T_map_trans, source_T_map_rot = p.invertTransform(map_T_source_trans, map_T_source_rot)
        source_T_target_trans, source_T_target_rot = p.multiplyTransforms(source_T_map_trans, source_T_map_rot,
                                                                          map_T_target_trans, map_T_target_rot)
        return source_T_target_trans, source_T_target_rot

    def get_link_position_and_orientation(self, name):
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[4:6]

    def get_link_position(self, name):
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[4]

    def get_link_orientation(self, name):
        return p.getLinkState(self.id, self.links[name], physicsClientId=self.world.client_id)[5]

    def set_joint_state(self, joint_name, joint_pose):
        up_lim, low_lim = p.getJointInfo(self.id, self.joints[joint_name], physicsClientId=self.world.client_id)[8:10]
        if low_lim > up_lim:
            low_lim, up_lim = up_lim, low_lim
        if not low_lim <= joint_pose <= up_lim:
            rospy.logerr(f"The joint position has to be within the limits of the joint. The joint limits for {joint_name} are {low_lim} and {up_lim}")
            rospy.logerr(f"The given joint position was: {joint_pose}")
            # Temporarily disabled because kdl outputs values exciting joint limits
            #return
        p.resetJointState(self.id, self.joints[joint_name], joint_pose, physicsClientId=self.world.client_id)
        self._set_attached_objects([self])

    def get_joint_state(self, joint_name):
        return p.getJointState(self.id, self.joints[joint_name], physicsClientId=self.world.client_id)[0]

    def contact_points(self):
        return p.getContactPoints(self.id)

    def contact_points_simulated(self):
        s = self.world.save_state()
        p.stepSimulation(self.world.client_id)
        contact_points = self.contact_points()
        self.world.restore_state(*s)
        return contact_points

def filter_contact_points(contact_points, exclude_ids):
    return list(filter(lambda cp: cp[2] not in exclude_ids, contact_points))

def _load_object(name, path, position, orientation, world, color, ignoreCachedFiles):
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
    :return: The unique id of the object
    """
    pa = pathlib.Path(path)
    extension = pa.suffix
    world, world_id = _world_and_id(world)
    rospack = rospkg.RosPack()
    cach_dir = rospack.get_path('pycram') + '/resources/cached/'
    if not pathlib.Path(cach_dir).exists():
        os.mkdir(cach_dir)

    # if file is not yet cached corrcet the urdf and save if in the cache directory
    if not _is_cached(path, name, cach_dir) or ignoreCachedFiles:
        if extension == ".obj" or extension == ".stl":
            path = _generate_urdf_file(name, path, color, cach_dir)
        elif extension == ".urdf":
            with open(path, mode="r") as f:
                urdf_string = f.read()
            path = cach_dir +  pa.name
            with open(path, mode="w") as f:
                f.write(_correct_urdf_string(urdf_string))
        else: # Using the urdf from the parameter server
            urdf_string = rospy.get_param(urdf_name)
            path = cach_dir +  name + ".urdf"
            with open(path, mode="w") as f:
                f.write(_correct_urdf_string(urdf_string))
    # save correct path in case the file is already in the cache directory
    elif extension == ".obj" or extension == ".stl":
        path = cach_dir + name  + ".urdf"
    elif extension == ".urdf":
        path = cach_dir + pa.name
    else:
        path = cach_dir + name + ".urdf"

    try:
        obj = p.loadURDF(path, basePosition=position, baseOrientation=orientation, physicsClientId=world_id)
        return obj
    except p.error as e:
        print(e)
        rospy.logerr("The File could not be loaded. Plese note that the path has to be either a URDF, stl or obj file or the name of an URDF string on the parameter server.")
        #print(f"{bcolors.BOLD}{bcolors.WARNING}The path has to be either a path to a URDf file, stl file, obj file or the name of a URDF on the parameter server.{bcolors.ENDC}")


def _is_cached(path, name, cach_dir):
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
    p = pathlib.Path(cach_dir + name + ".urdf")
    if p.exists():
        return True
    return False


def _correct_urdf_string(urdf_string):
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

def _generate_urdf_file(name, path, color, cach_dir):
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
    path = str(pathlib.Path(path).resolve())
    content = urdf_template.replace("~a", name).replace("~b", path).replace("~c", rgb)
    with open(cach_dir + name + ".urdf", "w", encoding="utf-8") as file:
        file.write(content)
    return cach_dir + name + ".urdf"


def _world_and_id(world):
    """
    This method selects the world to be used. If the given world is None the 'current_bullet_world' is used.
    :param world: The world which should be used or None if 'current_bullet_world' should be used
    :return: The BulletWorld object and the id of this BulletWorld
    """
    world = world if world is not None else BulletWorld.current_bullet_world
    id = world.client_id if world is not None else BulletWorld.current_bullet_world.client_id
    return world, id
