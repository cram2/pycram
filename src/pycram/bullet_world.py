import pybullet as p
import threading
import time
import itertools
from pycram.helper import _client_id

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
        p.setGravity(0, 0, -9.8, self.client_id)
        p.setRealTimeSimulation(1, self.client_id)

    def simulate(self, seconds):
        for i in range(0, seconds * 240):
            p.stepSimulation(self.client_id)


class Gui(threading.Thread):
    def __init__(self, world):
        threading.Thread.__init__(self)
        self.world = world

    def run(self):
        self.world.client_id = p.connect(p.GUI)
        while p.isConnected(self.world.client_id):
            time.sleep(100)


class Object:
    def __init__(self, name, path, position=[0, 0, 0], world=None):
        self.name = name
        self.path = path
        self.world = _client_id(world)
        self.id = p.loadURDF(path, basePosition=position, physicsClientId=self.world)
        world._add_object(self)

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)[0]

    def get_orientation(self):
        return p.getBasePositionAndOrientation(self.id)[1]

    def set_position_and_orientation(self, position, orientation):
        world_id = _client_id(self.world)
        p.resetBasePositionAndOrientation(self.id, position, orientation, world_id)


class Robot(Object):
    #TODO: Maybe Implement
    def __init__(self):
        return None


def stable(object, world):
    world_id = _client_id(world)
    object = object.id
    coords_prev = p.getBasePositionAndOrientation(object, physicsClientId=world_id)[0]
    state = p.saveState(clientServerId=world_id)
    p.setGravity(0, 0, -9.8)

    # one Step is approximately 1/240 seconds
    for i in range(0, 10 * 240):
        p.stepSimulation(physicsClientId=world_id)
    coords_past = p.getBasePositionAndOrientation(object, physicsClientId=world_id)[0]

    p.restoreState(state, clientServerId=world_id)
    coords_prev = list(map(lambda n: round(n, 3), coords_prev))
    coords_past = list(map(lambda n: round(n, 3), coords_past))
    return coords_past == coords_prev


def contact(object1, object2, world):
    world_id = _client_id(world)
    con_points = p.getContactPoints(object1.id, object2.id, physicsClientId=world_id)
    return con_points is not ()


def visible(object, world):
    world_id = _client_id(world)
    state = p.saveState()
    #matrix = p.computeViewMatrix() TODO
    for obj in world.objects:
        if obj.id is not object.id:
            #p.removeBody(object.id, physicsClientId=world_id)
            # Hot fix until I come up with something better
            p.resetBasePositionAndOrientation(obj.id, [100, 100, 100], [0, 0, 0, 1], world_id)

    seg_mask = p.getCameraImage(256, 256, shadow=0, physicsClientId=world_id)[4]
    flat_list = list(itertools.chain.from_iterable(seg_mask))
    max_pixel = sum(list(map(lambda x: 1 if x == object.id else 0, flat_list)))
    p.restoreState(state)

    seg_mask = p.getCameraImage(256, 256, shadow=0, physicsClientId=world_id)[4]
    flat_list = list(itertools.chain.from_iterable(seg_mask))
    real_pixel = sum(list(map(lambda x: 1 if x == object.id else 0, flat_list)))

    return real_pixel >= max_pixel > 0


def occluding(object, world):
    world_id = _client_id(world)
    state = p.saveState()
    #matrix = p.computeViewMatrix() TODO
    for obj in world.objects:
        if obj.id is not object.id:
            #p.removeBody(object.id, physicsClientId=world_id)
            # Hot fix until I come up with something better
            p.resetBasePositionAndOrientation(obj.id, [100, 100, 100], [0, 0, 0, 1], world_id)

    seg_mask = p.getCameraImage(256, 256, shadow=0, physicsClientId=world_id)[4]
    pixels = []
    for i in range(0, 256):
        for j in range(0, 256):
            if seg_mask[i][j] == object.id:
                pixels.append((i, j))
    p.restoreState(state)

    occluding = []
    seg_mask = p.getCameraImage(256, 256, shadow=0, physicsClientId=world_id)[4]
    for c in pixels:
        if not seg_mask[c[0]][c[1]] == object.id:
            occluding.append(seg_mask[c[0]][c[1]])

    return list(set(map(lambda x: world.get_object_by_id(x), occluding)))


def reachable(object, world):
    world_id = _client_id(world)


