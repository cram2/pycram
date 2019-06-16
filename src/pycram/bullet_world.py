import pybullet as p
from pycram.projection import gui
from pycran.fluent import Fluent

current_bullet_world = None

class bullet_world:

    def __init__(self):
        self.objects = []
        self.id = gui(Fluent(name="client_id"))
        current_bullet_world = self

    def _add_object(self, object):
        self.objects.append(object)

    def get_object_by_name(self, name):
        return map(lambda obj: obj.name == name, self.objects)

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

    def __init__(self, name, path, world):
        self.name = name
        self.path = path
        self.world = world
        self.id = p.loadURDF(path, world.id)
        world._add_object(self)

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)

