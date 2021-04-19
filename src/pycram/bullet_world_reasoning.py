import pybullet as p
import itertools
import numpy as np
import time
from .bullet_world import _world_and_id, Object
from .ik import request_ik
from .robot_description import InitializedRobotDescription as robot_description
from .helper import _transform_to_torso, _apply_ik


class ReasoningError(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)


class CollisionError(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)


def _get_joint_names(robot, tip_link):
    res = []
    for i in range(p.getNumJoints(robot.id)):
        info = p.getJointInfo(robot.id, i)
        if info[2] != p.JOINT_FIXED:
            res.append(info[1])
    return res


def _get_seg_mask_for_target(target_position, cam_position, world=None):
    """
    Calculates the view and projection Matrix and returns the Segmentation mask
    The segmentation mask indicates for every pixel the visible Object.
    :param cam_position: The position of the Camera as a list of x,y,z and orientation as quaternion
    :param target_position: The position to which the camera should point as a list of x,y,z and orientation as quaternion
    :return: The Segmentation mask from the camera position
    """
    world, world_id = _world_and_id(world)
    # TODO: Might depend on robot cameras, if so please add these camera parameters to RobotDescription object
    # TODO: of your robot with a CameraDescription object.
    fov = 300
    aspect = 256 / 256
    near = 0.2
    far = 10

    view_matrix = p.computeViewMatrix(cam_position[0], target_position[0], [0, 0, -1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    return p.getCameraImage(256, 256, view_matrix, projection_matrix, physicsClientId=world_id)[4]


def _get_joint_ranges(robot):
    """
    Calculates the lower and upper limits, the joint ranges and the joint damping. For a given multibody.
    The rest poses are the current poses of the joints.
    Fixed joints will be skipped because they don't have limits or ranges.
    :param robot: The robot for whom the values should be calculated
    :return: The lists for the upper and lower limits, joint ranges, rest poses and joint damping
    """
    ll, ul, jr, rp, jd = [], [], [], [], []

    for i in range(0, p.getNumJoints(robot.id)):
        info = p.getJointInfo(robot.id, i)
        if info[3] > -1:
            ll.append(info[8])
            ul.append(info[9])
            jr.append(info[9] - info[8])
            rp.append(p.getJointState(robot.id, i)[0])
            jd.append(info[6])

    return ll, ul, jr, rp, jd


def stable(object, world=None):
    """
    This reasoning query checks if an object is stable in the world. This will be done by simulating the world for 10 seconds
    and compare the previous coordinates with the coordinates after the simulation.
    :param object: The object which should be checked
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the given object is stable in the world False else
    """
    world, world_id = _world_and_id(world)
    coords_prev = p.getBasePositionAndOrientation(object.id, physicsClientId=world_id)[0]
    state = p.saveState(physicsClientId=world_id)
    p.setGravity(0, 0, -9.8, world_id)

    # one Step is approximately 1/240 seconds
    world.simulate(2)
    coords_past = p.getBasePositionAndOrientation(object.id, physicsClientId=world_id)[0]

    p.restoreState(state, physicsClientId=world_id)
    coords_prev = list(map(lambda n: round(n, 3), coords_prev))
    coords_past = list(map(lambda n: round(n, 3), coords_past))
    return coords_past == coords_prev


def contact(object1, object2, world=None):
    """
    This reasoning query checks if two objects are in contact or not.
    :param object1: The first object
    :param object2: The second object
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the two objects are in contact False else
    """
    world, world_id = _world_and_id(world)
    p.stepSimulation(world_id)
    con_points = p.getContactPoints(object1.id, object2.id, physicsClientId=world_id)

    return con_points != ()


def visible(object, camera_position_and_orientation, front_facing_axis=None, threshold=0.8, world=None):
    """
    This reasoning query checks if an object is visible from a given position. This will be achieved by rendering the object
    alone and counting the visible pixel, then rendering the complete scene and compare the visible pixels with the
    absolut count of pixels.
    :param object: The object for which the visibility should be checked
    :param camera_position_and_orientation: The position and orientation of the
    camera in world coordinate fram
    :front_facing_axis: The axis, of the camera frame, which faces to the front of
    the robot. Given as list of x, y, z
    :threshold: The minimum percentage of the object that needs to be visibile
    for this method to return true
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the object is visible from the camera_position False if not
    """
    world, world_id = _world_and_id(world)
    front_facing_axis = robot_description.i.front_facing_axis if front_facing_axis == None else front_facing_axis
    det_world = world.copy()
    state = p.saveState(physicsClientId=det_world.client_id)
    for obj in det_world.objects:
        if obj.name == world.robot.name:
            continue
        elif object.get_position_and_orientation() == obj.get_position_and_orientation():
            object = obj
        else:
            p.resetBasePositionAndOrientation(obj.id, [100, 100, 100], [0, 0, 0, 1], det_world.client_id)

    world_T_cam = camera_position_and_orientation
    cam_T_point = list(np.multiply(front_facing_axis, 2))
    target_point = p.multiplyTransforms(world_T_cam[0], world_T_cam[1], cam_T_point, [0, 0, 0, 1])

    seg_mask = _get_seg_mask_for_target(target_point, world_T_cam, det_world)
    flat_list = list(itertools.chain.from_iterable(seg_mask))
    max_pixel = sum(list(map(lambda x: 1 if x == object.id else 0, flat_list)))
    p.restoreState(state, physicsClientId=det_world.client_id)

    if max_pixel == 0:
        # Object is not visible
        return False

    seg_mask = _get_seg_mask_for_target(target_point, world_T_cam, world)
    flat_list = list(itertools.chain.from_iterable(seg_mask))
    real_pixel = sum(list(map(lambda x: 1 if x == object.id else 0, flat_list)))

    det_world.exit()
    return real_pixel / max_pixel > threshold > 0


def occluding(object, camera_position_and_orientation, front_facing_axis, world=None):
    """
    This reasoning query lists the objects which are occluding a given object. This works similar to 'visible'.
    First the object alone will be rendered and the position of the pixels of the object in the picture will be saved.
    After that the complete scene will be rendered and the previous saved pixel positions will be compared to the
    actual pixels, if in one pixel an other object is visible ot will be saved as occluding.
    :param object: The object for which occluding should be checked
    :param camera_position_and_orientation: The position and orientation of the
    camera in world coordinate frame
    :front_facing_axis: The axis, of the camera frame, which faces to the front of
    the robot. Given as list of x, y, z
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: A list of occluding objects
    """
    world, world_id = _world_and_id(world)
    occ_world = world.copy()
    state = p.saveState(physicsClientId=occ_world.client_id)
    for obj in occ_world.objects:
        if obj.name == world.robot.name:
            continue
        elif object.get_position_and_orientation() == obj.get_position_and_orientation():
            object = obj
        else:
            p.resetBasePositionAndOrientation(obj.id, [100, 100, 100], [0, 0, 0, 1], occ_world.client_id)

    world_T_cam = camera_position_and_orientation
    cam_T_point = list(np.multiply(front_facing_axis, 2))
    target_point = p.multiplyTransforms(world_T_cam[0], world_T_cam[1], cam_T_point, [0, 0, 0, 1])

    seg_mask = _get_seg_mask_for_target(target_point, world_T_cam, occ_world)
    pixels = []
    for i in range(0, 256):
        for j in range(0, 256):
            if seg_mask[i][j] == object.id:
                pixels.append((i, j))
    p.restoreState(state, physicsClientId=occ_world.client_id)

    occluding = []
    seg_mask = _get_seg_mask_for_target(target_point, world_T_cam, occ_world)
    for c in pixels:
        if not seg_mask[c[0]][c[1]] == object.id:
            occluding.append(seg_mask[c[0]][c[1]])

    occ_objects = list(set(map(lambda x: occ_world.get_object_by_id(x), occluding)))
    occ_world.exit()

    return occ_objects


def reachable(pose, robot, gripper_name, world=None, threshold=0.01):
    """
    This reasoning query checks if the robot can reach a given position. To determine this the inverse kinematics are
    calculated and applied. Afterwards the distance between the position and the given end effector is calculated, if
    it is smaller than the threshold the reasoning query returns True, if not it returns False.
    :param pose: The position or Object for which reachability should be checked.
    :param robot: The robot that should reach for the position
    :param gripper_name: The name of the end effector
    :param world: The BulletWorld in which the reasoning query should opperate
    :param threshold: The threshold between the end effector and the position.
    :return: True if the end effector is closer than the threshold True if the end effector is closer than the threshold
    to the target position, False in every other case to the target position, False in every other case
    """
    if type(pose) == Object:
        pose = pose.get_position()
    world, world_id = _world_and_id(world)
    state = p.saveState(physicsClientId=world_id)
    arm = "left" if gripper_name == robot_description.i.get_tool_frame("left") else "right"
    joints = robot_description.i._safely_access_chains(arm).joints
    target = _transform_to_torso([pose, [0, 0, 0, 1]], robot)
    target = (target[0], [0, 0, 0, 1])
    inv = request_ik(robot_description.i.base_frame, gripper_name, target, robot, joints)

    _apply_ik(robot, inv, gripper_name)

    newp = p.getLinkState(robot.id, robot.get_link_id(gripper_name), physicsClientId=world_id)[4]
    diff = [pose[0] - newp[0], pose[1] - newp[1],  pose[2] - newp[2]]
    p.restoreState(state, physicsClientId=world_id)
    return np.sqrt(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2) < threshold


def blocking(object, robot, gripper_name, world=None):
    """
    This reasoning query checks if any objects are blocking an other object when an robot tries to pick it. This works
    similar to the reachable predicate. First the inverse kinematics between the robot and the object will be calculated
    and applied. Then it will be checked if the robot is in contact with any object except the given one.
    :param object: The object for which blocking objects should be found
    :param robot: The robot who reaches for the object
    :param gripper_name: The name of the end effector of the robot
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: A list of objects the robot is in collision with when reaching for the specified object
    """
    world, world_id = _world_and_id(world)
    state = p.saveState(physicsClientId=world_id)

    arm = "left" if gripper_name == robot_description.i.get_tool_frame("left") else "right"
    joints = robot_description.i._safely_access_chains(arm).joints
    target = _transform_to_torso(object.get_position_and_orientation(), robot)
    inv = request_ik(robot_description.i.base_frame, gripper_name, target, robot, joints)

    _apply_ik(robot, inv, gripper_name)

    block = []
    for obj in world.objects:
        if obj == object:
            continue
        if contact(robot, obj, world):
            block.append(obj)
    p.restoreState(state, physicsClientId=world_id)
    return block


def supporting(object1, object2, world=None):
    """
    This reasoning query checks if one object is supporting an other obkect. An object supports an other object if they are in
    contact and the second object is above the first one. (e.g. a Bottle will be supported by a table)
    :param object1: The first object
    :param object2: The second object
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the second object is in contact with the first one and the second one ist above the first False else
    """
    world, world_id = _world_and_id(world)
    return contact(object1, object2, world) and object2.getposition()[2] > object1.get_position()[2]
