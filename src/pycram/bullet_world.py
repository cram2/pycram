import pybullet as p
import threading
import time


current_bullet_world = None

class BulletWorld:
    def __init__(self):
        self.objects = []
        self.client_id = -1
        gui_thread = gui(self)
        gui_thread.start()
        time.sleep(0.1)

    def _add_object(self, object):
        self.objects.append(object)

    def get_object_by_name(self, name):
        return map(lambda obj: obj.name == name, self.objects)


class gui(threading.Thread):
    def __init__(self, world):
        threading.Thread.__init__(self)
        self.world = world

    def run(self):
        self.world.client_id = p.connect(p.GUI)
        print(self.world)
        self.infinity()


    def infinity(self):
        while 1:
            a = 1

def stable(object, world=None):
    if world is None:
        world = 0
    else:
        world = world.id
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
        if world is None:
            self.world = 1
        else:
            self.world = world.id
        self.id = p.loadURDF(path, self.world)
        world._add_object(self)

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)

