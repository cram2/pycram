import pybullet as p
import itertools
import numpy as np

from .bullet_world import _world_and_id, Object, Use_shadow_world, BulletWorld
from .external_interfaces.ik import request_ik
from .robot_descriptions import robot_description
from .helper import _transform_to_torso, _apply_ik, calculate_wrist_tool_offset, inverseTimes
from typing import List, Tuple, Optional, Union, Dict


class ReasoningError(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)


class CollisionError(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)


def _get_joint_names(robot: Object, tip_link: str) -> List[str]:
    res = []
    for i in range(p.getNumJoints(robot.id)):
        info = p.getJointInfo(robot.id, i)
        if info[2] != p.JOINT_FIXED:
            res.append(info[1])
    return res


def _get_images_for_target(target_position: Tuple[List[float], List[float]],
                           cam_position: Tuple[List[float], List[float]],
                           world: Optional[BulletWorld] = None,
                           size: Optional[int] = 256) -> List[np.ndarray]:
    """
    Calculates the view and projection Matrix and returns 3 images:

    1. An RGB image
    2. A depth image
    3. A segmentation Mask, the segmentation mask indicates for every pixel the visible Object.

    :param cam_position: The position of the Camera as a list of x,y,z and orientation as quaternion
    :param target_position: The position to which the camera should point as a list of x,y,z and orientation as quaternion
    :param size: The height and width of the images in pixel
    :return: A list containing an RGB and depth image as well as a segmentation mask, in this order.
    """
    world, world_id = _world_and_id(world)
    # TODO: Might depend on robot cameras, if so please add these camera parameters to RobotDescription object
    # TODO: of your robot with a CameraDescription object.
    fov = 90
    aspect = size / size
    near = 0.2
    far = 100

    view_matrix = p.computeViewMatrix(cam_position[0], target_position[0], [0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    return list(p.getCameraImage(size, size, view_matrix, projection_matrix, physicsClientId=world_id))[2:5]


def _get_joint_ranges(robot: Object) -> Tuple[List, List, List, List, List]:
    """
    Calculates the lower and upper limits, the joint ranges and the joint damping. For a given robot Object.
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


def stable(object: Object,
           world: Optional[BulletWorld] = None) -> bool:
    """
    Checks if an object is stable in the world. Stable meaning that it's position will not change after simulating physics
    in the BulletWorld. This will be done by simulating the world for 10 seconds and compare the previous coordinates
    with the coordinates after the simulation.

    :param object: The object which should be checked
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the given object is stable in the world False else
    """
    world, world_id = _world_and_id(world)
    shadow_obj = BulletWorld.current_bullet_world.get_shadow_object(object)
    with Use_shadow_world():
        # coords_prev = p.getBasePositionAndOrientation(object.id, physicsClientId=BulletWorld.current_bullet_world.client_id)[0]
        coords_prev = shadow_obj.get_position()
        state = p.saveState(physicsClientId=BulletWorld.current_bullet_world.client_id)
        p.setGravity(0, 0, -9.8, BulletWorld.current_bullet_world.client_id)

        # one Step is approximately 1/240 seconds
        BulletWorld.current_bullet_world.simulate(2)
        # coords_past = p.getBasePositionAndOrientation(object.id, physicsClientId=world_id)[0]
        coords_past = shadow_obj.get_position()

        # p.restoreState(state, physicsClientId=BulletWorld.current_bullet_world.client_id)
        coords_prev = list(map(lambda n: round(n, 3), coords_prev))
        coords_past = list(map(lambda n: round(n, 3), coords_past))
        return coords_past == coords_prev


def contact(object1: Object,
            object2: Object,
            return_links: bool = False) -> Union[bool, Tuple[bool, List]]:
    """
    Checks if two objects are in contact or not. If the links should be returned then the output will also contain a
    list of tuples where the first element is the link name of 'object1' and the second element is the link name of
    'object2'.

    :param object1: The first object
    :param object2: The second object
    :param return_links: If the respective links on the objects that are in contact should be returned.
    :return: True if the two objects are in contact False else. If links should be returned a list of links in contact
    """

    if BulletWorld.current_bullet_world.is_shadow_world:
        shadow_obj1 = object1
        shadow_obj2 = object2
    else:
        shadow_obj1 = BulletWorld.current_bullet_world.get_shadow_object(object1)
        shadow_obj2 = BulletWorld.current_bullet_world.get_shadow_object(object2)
    with Use_shadow_world():
        # p.stepSimulation(BulletWorld.current_bullet_world.client_id)
        p.performCollisionDetection(BulletWorld.current_bullet_world.client_id)
        con_points = p.getContactPoints(shadow_obj1.id, shadow_obj2.id,
                                        physicsClientId=BulletWorld.current_bullet_world.client_id)

        if return_links:
            contact_links = []
            for point in con_points:
                # l = [BulletWorld.current_bullet_world.get_object_by_id(point[1]).name,
                #      BulletWorld.current_bullet_world.get_object_by_id(point[2]).name, shadow_obj1.get_link_by_id(point[3]),
                #      shadow_obj2.get_link_by_id(point[4])]
                contact_links.append((shadow_obj1.get_link_by_id(point[3]), shadow_obj2.get_link_by_id(point[4])))
            return con_points != (), contact_links

        else:
            return con_points != ()


def visible(object: Object,
            camera_position_and_orientation: Tuple[List[float], List[float]],
            front_facing_axis: Optional[List[float]] = None,
            threshold: float = 0.8,
            world: Optional[BulletWorld] = None) -> bool:
    """
    Checks if an object is visible from a given position. This will be achieved by rendering the object
    alone and counting the visible pixel, then rendering the complete scene and compare the visible pixels with the
    absolut count of pixels.

    :param object: The object for which the visibility should be checked
    :param camera_position_and_orientation: The position and orientation of the camera in world coordinate fram
    :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
    :param threshold: The minimum percentage of the object that needs to be visible for this method to return true
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the object is visible from the camera_position False if not
    """
    front_facing_axis = robot_description.front_facing_axis if front_facing_axis == None else front_facing_axis
    with Use_shadow_world():
        shadow_obj = BulletWorld.current_bullet_world.get_shadow_object(object)
        if BulletWorld.robot:
            shadow_robot = BulletWorld.current_bullet_world.get_shadow_object(BulletWorld.robot)
        state = p.saveState(physicsClientId=BulletWorld.current_bullet_world.client_id)
        for obj in BulletWorld.current_bullet_world.objects:
            if obj == shadow_obj or BulletWorld.robot and obj == shadow_robot:
                continue
            else:
                obj.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])

        world_T_cam = camera_position_and_orientation
        cam_T_point = list(np.multiply(front_facing_axis, 2))
        target_point = p.multiplyTransforms(world_T_cam[0], world_T_cam[1], cam_T_point, [0, 0, 0, 1])

        seg_mask = _get_images_for_target(target_point, world_T_cam, BulletWorld.current_bullet_world)[2]
        flat_list = list(itertools.chain.from_iterable(seg_mask))
        max_pixel = sum(list(map(lambda x: 1 if x == shadow_obj.id else 0, flat_list)))
        p.restoreState(state, physicsClientId=BulletWorld.current_bullet_world.client_id)
        if max_pixel == 0:
            # Object is not visible
            return False

        seg_mask = _get_images_for_target(target_point, world_T_cam, BulletWorld.current_bullet_world)[2]
        flat_list = list(itertools.chain.from_iterable(seg_mask))
        real_pixel = sum(list(map(lambda x: 1 if x == shadow_obj.id else 0, flat_list)))

        return real_pixel / max_pixel > threshold > 0


def occluding(object: Object,
              camera_position_and_orientation: Tuple[List[float], List[float]],
              front_facing_axis: Tuple[List[float], List[float]],
              world: Optional[BulletWorld] = None) -> bool:
    """
    Lists all objects which are occluding the given object. This works similar to 'visible'.
    First the object alone will be rendered and the position of the pixels of the object in the picture will be saved.
    After that the complete scene will be rendered and the previous saved pixel positions will be compared to the
    actual pixels, if in one pixel another object is visible ot will be saved as occluding.

    :param object: The object for which occlusion should be checked
    :param camera_position_and_orientation: The position and orientation of the camera in world coordinate frame
    :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: A list of occluding objects
    """
    world, world_id = _world_and_id(world)
    # occ_world = world.copy()
    # state = p.saveState(physicsClientId=occ_world.client_id)
    with Use_shadow_world():
        state = p.saveState(physicsClientId=BulletWorld.current_bullet_world.client_id)
        for obj in BulletWorld.current_bullet_world.objects:
            if obj.name == BulletWorld.robot.name:
                continue
            elif object.get_position_and_orientation() == obj.get_position_and_orientation():
                object = obj
            else:
                obj.set_position_and_orientation([100, 100, 0], [0, 0, 0, 1])
                # p.resetBasePositionAndOrientation(obj.id, [100, 100, 100], [0, 0, 0, 1], occ_world.client_id)

        world_T_cam = camera_position_and_orientation
        cam_T_point = list(np.multiply(front_facing_axis, 2))
        target_point = p.multiplyTransforms(world_T_cam[0], world_T_cam[1], cam_T_point, [0, 0, 0, 1])

        seg_mask = _get_images_for_target(target_point, world_T_cam, BulletWorld.current_bullet_world)[2]

        # All indices where the object that could be occluded is in the image
        # [0] at the end is to reduce by one dimension because dstack adds an unnecessary dimension
        pix = np.dstack((seg_mask == object.id).nonzero())[0]

        p.restoreState(state, physicsClientId=BulletWorld.current_bullet_world.client_id)

        occluding = []
        seg_mask = _get_images_for_target(target_point, world_T_cam, BulletWorld.current_bullet_world)[2]
        for c in pix:
            if not seg_mask[c[0]][c[1]] == object.id:
                occluding.append(seg_mask[c[0]][c[1]])

        occ_objects = list(set(map(BulletWorld.current_bullet_world.get_object_by_id, occluding)))
        occ_objects = list(map(world.get_bullet_object_for_shadow, occ_objects))

        return occ_objects


def reachable(pose: Union[Object, Tuple[List[float], List[float]]],
              robot: Object,
              gripper_name: str,
              threshold: float = 0.01) -> bool:
    """
    Checks if the robot can reach a given position. To determine this the inverse kinematics are
    calculated and applied. Afterward the distance between the position and the given end effector is calculated, if
    it is smaller than the threshold the reasoning query returns True, if not it returns False.

    :param pose: The position and rotation or Object for which reachability should be checked or an Object
    :param robot: The robot that should reach for the position
    :param gripper_name: The name of the end effector
    :param threshold: The threshold between the end effector and the position.
    :return: True if the end effector is closer than the threshold to the target position, False in every other case
    """
    if type(pose) == Object:
        pose = pose.get_position_and_orientation()
    # world, world_id = _world_and_id(world)
    # state = p.saveState(physicsClientId=world_id)
    shadow_robot = BulletWorld.current_bullet_world.get_shadow_object(robot)
    with Use_shadow_world():
        arm = "left" if gripper_name == robot_description.get_tool_frame("left") else "right"
        joints = robot_description._safely_access_chains(arm).joints
        target_torso = _transform_to_torso(pose, shadow_robot)

        # Get Link before first joint in chain
        base_link = robot_description.get_parent(joints[0])
        # Get link after last joint in chain
        end_effector = robot_description.get_child(joints[-1])

        diff = calculate_wrist_tool_offset(end_effector, robot_description.get_tool_frame(arm), shadow_robot)
        target_diff = inverseTimes(target_torso, diff)

        inv = request_ik(base_link, end_effector, target_diff, shadow_robot, joints)

        _apply_ik(shadow_robot, inv, joints)

        #        newp = p.getLinkState(robot.id, robot.get_link_id(gripper_name), physicsClientId=world_id)[4]
        newp = shadow_robot.get_link_position(gripper_name)
        diff = [pose[0][0] - newp[0], pose[0][1] - newp[1], pose[0][2] - newp[2]]
        # p.restoreState(state, physicsClientId=world_id)
    return np.sqrt(diff[0] ** 2 + diff[1] ** 2 + diff[2] ** 2) < threshold


def blocking(pose_or_object: Object,
             robot: Object,
             gripper_name: str,
             grasp: str = None) -> bool:
    """
    Checks if any objects are blocking another object when a robot tries to pick it. This works
    similar to the reachable predicate. First the inverse kinematics between the robot and the object will be calculated
    and applied. Then it will be checked if the robot is in contact with any object except the given one.

    :param pose_or_object: The object or position for which blocking objects should be found
    :param robot: The robot Object who reaches for the object
    :param gripper_name: The name of the end effector of the robot
    :param grasp: The grasp type with which the object should be grasped
    :return: A list of objects the robot is in collision with when reaching for the specified object
    """
    # world, world_id = _world_and_id(world)
    if type(pose_or_object) == Object:
        input_pose = object.get_position_and_orientation()
    else:
        input_pose = pose_or_object

    shadow_robot = BulletWorld.current_bullet_world.get_shadow_object(robot)
    with Use_shadow_world():
        arm = "left" if gripper_name == robot_description.get_tool_frame("left") else "right"
        joints = robot_description._safely_access_chains(arm).joints

        if grasp:
            target_torso = _transform_to_torso(
                [input_pose[0], robot_description.grasps.get_orientation_for_grasp(grasp)], shadow_robot)
        else:
            target_torso = _transform_to_torso(input_pose, shadow_robot)

        # Get Link before first joint in chain
        base_link = robot_description.get_parent(joints[0])
        # Get link after last joint in chain
        end_effector = robot_description.get_child(joints[-1])

        diff = calculate_wrist_tool_offset(end_effector, robot_description.get_tool_frame(arm), shadow_robot)
        target_diff = inverseTimes(target_torso, diff)

        inv = request_ik(base_link, end_effector, target_diff, shadow_robot, joints)

        _apply_ik(shadow_robot, inv, joints)

        block = []
        for obj in BulletWorld.current_bullet_world.objects:
            if contact(shadow_robot, obj):
                block.append(BulletWorld.current_bullet_world.get_bullet_object_for_shadow(obj))

    return block


def supporting(object1: Object,
               object2: Object,
               world: Optional[BulletWorld] = None) -> bool:
    """
    Checks if one object is supporting another object. An object supports another object if they are in
    contact and the second object is above the first one. (e.g. a Bottle will be supported by a table)

    :param object1: The first object
    :param object2: The second object
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the second object is in contact with the first one and the second one ist above the first False else
    """
    world, world_id = _world_and_id(world)
    return contact(object1, object2, world) and object2.get_position()[2] > object1.get_position()[2]


def link_pose_for_joint_config(object: Object, joint_config: Dict[str, float], link_name: str) -> Tuple[
    List[float], List[float]]:
    """
    Returns the pose a link would be in if the given joint configuration would be applied to the object. This is done
    by using the respective object in the shadow world and applying the joint configuration to this one. After applying
    the joint configuration the link position is taken from there.

    :param object: Object of which the link is a part
    :param joint_config: Dict with the goal joint configuration
    :param link_name: Name of the link for which the pose should be returned
    :return: The pose of the link after applying the joint configuration
    """
    shadow_object = BulletWorld.current_bullet_world.get_shadow_object(object)
    with Use_shadow_world():
        for joint, pose in joint_config.items():
            shadow_object.set_joint_state(joint, pose)
        return shadow_object.get_link_position_and_orientation(link_name)
