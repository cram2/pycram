import pybullet as p
import threading
import time
from pycram.helper import _client_id



class BulletWorld:

    def __init__(self):
        self.objects = []
        self.client_id = -1
        gui_thread = Gui(self)
        gui_thread.start()
        time.sleep(0.1)

    def _add_object(self, object):
        self.objects.append(object)

    def get_object_by_name(self, name):
        return map(lambda obj: obj.name == name, self.objects)


class Gui(threading.Thread):
    def __init__(self, world):
        threading.Thread.__init__(self)
        self.world = world

    def run(self):
        self.world.client_id = p.connect(p.GUI)
        while 1:
            a=1

def stable(object, world=None):
    world = _client_id(world)
    coords_prev = p.getBasePositionAndOrientation(object)
    state = p.saveState()
    p.setGravity(0, 0, -9.8)

    for i in range(0, 1000):
        p.stepSimulation()
    coords_past = p.getBasePositionAndOrientation(object)

    p.restoreState(state)
    return coords_past == coords_prev

def contact(object1, object2, world=None):
    return p.getOverlappingObjects() is not None



class Object:
    def __init__(self, name, path, world=None):
        self.name = name
        self.path = path
        self.world = _client_id(world)
        self.id = p.loadURDF(path, physicsClientId=self.world)
        world._add_object(self)

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)

