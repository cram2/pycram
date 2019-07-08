import pybullet as p
import threading
import time
import itertools


current_bullet_world = None


class BulletWorld:

    def __init__(self):
        self.objects = []
        self.client_id = -1
        gui_thread = Gui(self)
        gui_thread.start()
        time.sleep(0.1)
        p.setGravity(0, 0, -9)
        global current_bullet_world
        current_bullet_world = self

    def _add_object(self, object):
        self.objects.append(object)

    def get_objects_by_name(self, name):
        res = []
        for o in self.objects:
            if o.name == name:
                res.append(o)
        return res

    def get_object_by_id(self, id):
        for o in self.objects:
            if o.id == id:
                return o

    def set_realtime(self):
        p.setRealTimeSimulation(1, self.client_id)

    def simulate(self, seconds):
        for i in range(0, seconds * 240):
            p.stepSimulation(self.client_id)

    def exit(self):
        p.disconnect(self.client_id)


class Gui(threading.Thread):
    def __init__(self, world):
        threading.Thread.__init__(self)
        self.world = world

    def run(self):
        self.world.client_id = p.connect(p.GUI)
        while p.isConnected(self.world.client_id):
            time.sleep(10)


class Object:
    def __init__(self, name, path, position=[0, 0, 0], world=None):
        global current_bullet_world
        self.world = world if world != None else current_bullet_world
        self.name = name
        self.path = path
        self.id = p.loadURDF(path, basePosition=position, physicsClientId=self.world.client_id)
        self.joints = self._joint_or_link_name_to_id("joint")
        self.links = self._joint_or_link_name_to_id("link")
        self.attachments = {}
        self.world.objects.append(self)

    def attach(self, object, parent_link_id, child_link_id):
        own_pos = self.get_position() if parent_link_id == -1 else p.getLinkState(self.id, parent_link_id, physicsClientId=self.world.client_id)[2]
        object_pos = object.get_position() if child_link_id == -1 else p.getLinkState(object.id, child_link_id, physicsClientId=self.world.client_id)[2]

        cid = p.createConstraint(self.id, parent_link_id,
                                 object.id, child_link_id,
                                 p.JOINT_FIXED,
                                 [0, 0, 1], own_pos, object_pos,
                                 physicsClientId=self.world.client_id)
        self.attachments[object] = cid

    def detach(self, object):
        p.removeConstraint(self.attachments[object])
        self.attachments[object] = None

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)[0]

    def get_pose(self):
        return self.get_position()

    def get_orientation(self):
        return p.getBasePositionAndOrientation(self.id)[1]

    def set_position_and_orientation(self, position, orientation):
        p.resetBasePositionAndOrientation(self.id, position, orientation, self.world.client_id)

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

