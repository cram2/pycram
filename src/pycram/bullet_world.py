import pybullet as p


def stable(object):
    coords_prev = p.getBasePositionAndOrientation(object)
    state = p.saveState()
    p.setGravity(0, 0, -9.8)

    for i in range(0, 1000):
        p.stepSimulation()
    coords_past = p.getBasePositionAndOrientation(object)

    p.restoreState(state)
    return coords_past == coords_prev


class Object:
    name = ""
    id = None
    path = ""

    def __init__(self, name, path):
        self.name = name
        self.path = path
        self.id = p.loadURDF(path)

    def get_id(self):
        return self.id

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)

    def get_name(self):
        return self.name
