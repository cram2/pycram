import pybullet as p
import threading
import time
import pathlib
from .helper import _client_id
from .event import Event

current_bullet_world = None


class BulletWorld:

    def __init__(self, type="GUI"):
        self.objects = []
        self.client_id = -1
        self.dummy = Event()
        self.detachment_event = Event()
        self.attachment_event = Event()
        self.manipulation_event = Event()
        gui_thread = Gui(self, type)
        gui_thread.start()
        time.sleep(0.1)
        global current_bullet_world
        current_bullet_world = self

    def get_objects_by_name(self, name):
        return list(filter(lambda x: x.name == name, self.objects))

    def get_object_by_id(self, id):
        return list(filter(lambda x: x.id == id, self.objects))[0]

    def get_attachment_event(self):
        print(self.detachment_event)
        return self.attachment_event

    def set_realtime(self, real_time):
        p.setRealTimeSimulation(1 if real_time else 0, self.client_id)

    def set_gravity(self, velocity):
        p.setGravity(velocity[0], velocity[1], velocity[2])

    def simulate(self, seconds):
        for i in range(0, int(seconds * 240)):
            p.stepSimulation(self.client_id)

    def exit(self):
        p.disconnect(self.client_id)


class Gui(threading.Thread):
    def __init__(self, world, type):
        threading.Thread.__init__(self)
        self.world = world
        self.type = type

    def run(self):
        if self.type == "GUI":
            self.world.client_id = p.connect(p.GUI)
        else:
            self.world.client_id = p.connect(p.DIRECT)

        while p.isConnected(self.world.client_id):
            time.sleep(10)


class Object:

    def __init__(self, name, path, position=[0, 0, 0], world=None, color=[1, 1, 1, 1]):
        global current_bullet_world
        self.world = world if world is not None else current_bullet_world
        self.name = name
        self.path = path
        self.id = _load_object(name, path, position, world, color)
        self.joints = self._joint_or_link_name_to_id("joint")
        self.links = self._joint_or_link_name_to_id("link")
        self.attachments = {}
        self.world.objects.append(self)
        self.event = self.world.attachment_event

    def attach(self, object, parent_link_id, child_link_id):
        world_gripper = p.getLinkState(self.id, parent_link_id)[4] if parent_link_id != -1 else self.get_position()
        world_object = object.get_position()
        gripper_object = p.multiplyTransforms(p.invertTransform(world_gripper, [0, 0, 0, 1])[0], [0, 0, 0, 1],
                                              world_object, [0, 0, 0, 1], self.world.client_id)[0]
        cid = p.createConstraint(self.id, parent_link_id,
                                 object.id, child_link_id,
                                 p.JOINT_FIXED,
                                 [0, 1, 0], gripper_object, [0, 0, 0],
                                 physicsClientId=self.world.client_id)
        self.attachments[object] = cid
        self.world.attachment_event(self, [self, object])

    def detach(self, object):
        p.removeConstraint(self.attachments[object])
        self.attachments[object] = None
        self.world.detachment_event(self, [self, object])

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)[0]

    def get_pose(self):
        return self.get_position()

    def get_orientation(self):
        return p.getBasePositionAndOrientation(self.id)[1]

    def set_position_and_orientation(self, position, orientation):
        p.resetBasePositionAndOrientation(self.id, position, orientation, self.world.client_id)

    def set_position(self, position):
        self.set_position_and_orientation(position, [0, 0, 0, 1])

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
        return p.getLinkState(self.id, self.get_link_id(name))[:2]

    def get_link_position(self, name):
        return p.getLinkState(self.id, self.get_link_id(name))[0]

    def get_link_orientation(self, name):
        return p.getLinkState(self.id, self.get_link_id(name))[1]


def _load_object(name, path, position, world, color):
    extension = pathlib.Path(path).suffix
    world_id = _client_id(world)
    if extension == ".obj" or extension == ".stl":
        path = _generate_urdf_file(name, path, color)
    return p.loadURDF(path, basePosition=position, physicsClientId=world_id)


def _generate_urdf_file(name, path, color):
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
    file = open(name + ".urdf", "w", encoding="utf-8")
    file.write(content)
    file.close()
    return name + ".urdf"
