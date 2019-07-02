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

    def _compute_relative(self, object):

        return None

    def detach(self, object):
        p.removeConstraint(self.attachments[object])
        self.attachments[object] = None

    def get_position(self):
        return p.getBasePositionAndOrientation(self.id)[0]

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


def _get_seg_mask_for_target(cam_position, target_position):
    fov = 300
    aspect = 256 / 256
    near = 0.2
    far = 10

    view_matrix = p.computeViewMatrix(cam_position, target_position, [-1, 0, -1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    return p.getCameraImage(256, 256, view_matrix, projection_matrix)[4]


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


def visible(object, camera_position,  world):
    world_id = _client_id(world)
    state = p.saveState()
    for obj in world.objects:
        if obj.id is not object.id:
            # p.removeBody(object.id, physicsClientId=world_id)
            # Hot fix until I come up with something better
            p.resetBasePositionAndOrientation(obj.id, [100, 100, 100], [0, 0, 0, 1], world_id)

    seg_mask = _get_seg_mask_for_target(camera_position, object.get_position())
    flat_list = list(itertools.chain.from_iterable(seg_mask))
    max_pixel = sum(list(map(lambda x: 1 if x == object.id else 0, flat_list)))
    p.restoreState(state)

    seg_mask = _get_seg_mask_for_target(camera_position, object.get_position())
    flat_list = list(itertools.chain.from_iterable(seg_mask))
    real_pixel = sum(list(map(lambda x: 1 if x == object.id else 0, flat_list)))

    return real_pixel >= max_pixel > 0


def occluding(object, camera_position, world):
    world_id = _client_id(world)
    state = p.saveState()
    for obj in world.objects:
        if obj.id is not object.id:
            # p.removeBody(object.id, physicsClientId=world_id)
            # Hot fix until I come up with something better
            p.resetBasePositionAndOrientation(obj.id, [100, 100, 100], [0, 0, 0, 1], world_id)

    seg_mask = _get_seg_mask_for_target(camera_position, object.get_position())
    pixels = []
    for i in range(0, 256):
        for j in range(0, 256):
            if seg_mask[i][j] == object.id:
                pixels.append((i, j))
    p.restoreState(state)

    occluding = []
    seg_mask = _get_seg_mask_for_target(camera_position, object.get_position())
    for c in pixels:
        if not seg_mask[c[0]][c[1]] == object.id:
            occluding.append(seg_mask[c[0]][c[1]])

    return list(set(map(lambda x: world.get_object_by_id(x), occluding)))


def reachable(object, robot, gripper_name, world):
    world_id = _client_id(world)
    inv = p.calculateInverseKinematics(robot.id, robot.get_link_id(gripper_name), object.get_position(), physicsClientId=world_id)
    return inv is not ()


def blocking(object, robot, gripper_name, world):
    world_id = _client_id(world)

    return None


def _client_id(world):
    if world is not None:
        return world.client_id
    else:
        return 0
