import pybullet as p


def spawn_object(name, path, position=None):
    id = p.loadURDF(path)
    if position is not None:
        p.resetBasePositionAndOrientation(position)

    return id


def open_debug_window():
    return p.connect(p.GUI)


