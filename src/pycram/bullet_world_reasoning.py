import pybullet as p
import itertools
import numpy as np
import rospy

from .bullet_world import _world_and_id, Object, Use_shadow_world, BulletWorld
from .external_interfaces.ik import request_ik
from .local_transformer import LocalTransformer
from .plan_failures import IKError
from .robot_descriptions import robot_description
from .helper import _transform_to_torso, _apply_ik, calculate_wrist_tool_offset, inverseTimes
from .pose import Pose, Transform
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


def _get_images_for_target(target_pose: Pose,
                           cam_pose: Pose,
                           world: Optional[BulletWorld] = None,
                           size: Optional[int] = 256) -> List[np.ndarray]:
    """
    Calculates the view and projection Matrix and returns 3 images:

    1. An RGB image
    2. A depth image
    3. A segmentation Mask, the segmentation mask indicates for every pixel the visible Object.

    From the given target_pose and cam_pose only the position is used.

    :param cam_pose: The pose of the camera
    :param target_pose: The pose to which the camera should point to
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

    view_matrix = p.computeViewMatrix(cam_pose.position_as_list(), target_pose.position_as_list(), [0, 0, 1])
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
        coords_prev = shadow_obj.pose.position_as_list()
        state = p.saveState(physicsClientId=BulletWorld.current_bullet_world.client_id)
        p.setGravity(0, 0, -9.8, BulletWorld.current_bullet_world.client_id)

        # one Step is approximately 1/240 seconds
        BulletWorld.current_bullet_world.simulate(2)
        # coords_past = p.getBasePositionAndOrientation(object.id, physicsClientId=world_id)[0]
        coords_past = shadow_obj.pose.position_as_list()

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

    with Use_shadow_world():
        shadow_obj1 = BulletWorld.current_bullet_world.get_shadow_object(object1)
        shadow_obj2 = BulletWorld.current_bullet_world.get_shadow_object(object2)
        p.performCollisionDetection(BulletWorld.current_bullet_world.client_id)
        con_points = p.getContactPoints(shadow_obj1.id, shadow_obj2.id,
                                        physicsClientId=BulletWorld.current_bullet_world.client_id)

        if return_links:
            contact_links = []
            for point in con_points:
                contact_links.append((shadow_obj1.get_link_by_id(point[3]), shadow_obj2.get_link_by_id(point[4])))
            return con_points != (), contact_links

        else:
            return con_points != ()


def visible(object: Object,
            camera_pose: Pose,
            front_facing_axis: Optional[List[float]] = None,
            threshold: float = 0.8,
            world: Optional[BulletWorld] = None) -> bool:
    """
    Checks if an object is visible from a given position. This will be achieved by rendering the object
    alone and counting the visible pixel, then rendering the complete scene and compare the visible pixels with the
    absolut count of pixels.

    :param object: The object for which the visibility should be checked
    :param camera_pose: The pose of the camera in map frame
    :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
    :param threshold: The minimum percentage of the object that needs to be visible for this method to return true
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: True if the object is visible from the camera_position False if not
    """
    front_facing_axis = robot_description.front_facing_axis if not front_facing_axis else front_facing_axis
    with Use_shadow_world():
        shadow_obj = BulletWorld.current_bullet_world.get_shadow_object(object)
        if BulletWorld.robot:
            shadow_robot = BulletWorld.current_bullet_world.get_shadow_object(BulletWorld.robot)
        state = p.saveState(physicsClientId=BulletWorld.current_bullet_world.client_id)
        for obj in BulletWorld.current_bullet_world.objects:
            if obj == shadow_obj or BulletWorld.robot and obj == shadow_robot:
                continue
            else:
                obj.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))

        world_to_cam = camera_pose.to_transform("camera")
        cam_to_point = Transform(list(np.multiply(front_facing_axis, 2)), [0, 0, 0, 1], "camera", "point")
        target_point = (world_to_cam * cam_to_point).to_pose()
        # target_point = p.multiplyTransforms(world_to_cam.translation_as_list(), world_to_cam.rotation_as_list(), cam_to_point.translation_as_list(), [0, 0, 0, 1])
        # print(target_point)

        seg_mask = _get_images_for_target(target_point, world_to_cam.to_pose(), BulletWorld.current_bullet_world)[2]
        flat_list = list(itertools.chain.from_iterable(seg_mask))
        max_pixel = sum(list(map(lambda x: 1 if x == shadow_obj.id else 0, flat_list)))
        p.restoreState(state, physicsClientId=BulletWorld.current_bullet_world.client_id)
        if max_pixel == 0:
            # Object is not visible
            return False

        seg_mask = _get_images_for_target(target_point, world_to_cam.to_pose(), BulletWorld.current_bullet_world)[2]
        flat_list = list(itertools.chain.from_iterable(seg_mask))
        real_pixel = sum(list(map(lambda x: 1 if x == shadow_obj.id else 0, flat_list)))

        return real_pixel / max_pixel > threshold > 0


def occluding(object: Object,
              camera_pose: Pose,
              front_facing_axis: Optional[List[float]] = None,
              world: Optional[BulletWorld] = None) -> List[Object]:
    """
    Lists all objects which are occluding the given object. This works similar to 'visible'.
    First the object alone will be rendered and the position of the pixels of the object in the picture will be saved.
    After that the complete scene will be rendered and the previous saved pixel positions will be compared to the
    actual pixels, if in one pixel another object is visible ot will be saved as occluding.

    :param object: The object for which occlusion should be checked
    :param camera_pose: The pose of the camera in world coordinate frame
    :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
    :param world: The BulletWorld if more than one BulletWorld is active
    :return: A list of occluding objects
    """
    front_facing_axis = robot_description.front_facing_axis if not front_facing_axis else front_facing_axis
    world, world_id = _world_and_id(world)
    # occ_world = world.copy()
    # state = p.saveState(physicsClientId=occ_world.client_id)
    with Use_shadow_world():
        state = p.saveState(physicsClientId=BulletWorld.current_bullet_world.client_id)
        for obj in BulletWorld.current_bullet_world.objects:
            if obj.name == BulletWorld.robot.name:
                continue
            elif object.get_pose() == obj.get_pose():
                object = obj
            else:
                obj.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))

        world_to_cam = camera_pose.to_transform("camera")
        cam_to_point = Transform(list(np.multiply(front_facing_axis, 2)), [0, 0, 0, 1], "camera", "point")
        target_point = (world_to_cam * cam_to_point).to_pose()

        seg_mask = _get_images_for_target(target_point, world_to_cam.to_pose(), BulletWorld.current_bullet_world)[2]

        # All indices where the object that could be occluded is in the image
        # [0] at the end is to reduce by one dimension because dstack adds an unnecessary dimension
        pix = np.dstack((seg_mask == object.id).nonzero())[0]

        p.restoreState(state, physicsClientId=BulletWorld.current_bullet_world.client_id)

        occluding = []
        seg_mask = _get_images_for_target(target_point, world_to_cam.to_pose(), BulletWorld.current_bullet_world)[2]
        for c in pix:
            if not seg_mask[c[0]][c[1]] == object.id:
                occluding.append(seg_mask[c[0]][c[1]])

        occ_objects = list(set(map(BulletWorld.current_bullet_world.get_object_by_id, occluding)))
        occ_objects = list(map(world.get_bullet_object_for_shadow, occ_objects))

        return occ_objects


def reachable(pose: Union[Object, Pose],
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
        pose = pose.get_pose()

    shadow_robot = BulletWorld.current_bullet_world.get_shadow_object(robot)
    with Use_shadow_world():
        arm = "left" if gripper_name == robot_description.get_tool_frame("left") else "right"
        joints = robot_description.chains[arm].joints
        try:
            inv = request_ik(pose, shadow_robot, joints, gripper_name)
        except IKError as e:
            return False

        _apply_ik(shadow_robot, inv, joints)

        diff = pose.dist(shadow_robot.get_link_pose(gripper_name))

    return diff < threshold


def blocking(pose_or_object: Union[Object, Pose],
             robot: Object,
             gripper_name: str,
             grasp: str = None) -> Union[List[Object], None]:
    """
    Checks if any objects are blocking another object when a robot tries to pick it. This works
    similar to the reachable predicate. First the inverse kinematics between the robot and the object will be calculated
    and applied. Then it will be checked if the robot is in contact with any object except the given one. If the given
    pose or Object is not reachable None will be returned

    :param pose_or_object: The object or pose for which blocking objects should be found
    :param robot: The robot Object who reaches for the object
    :param gripper_name: The name of the end effector of the robot
    :param grasp: The grasp type with which the object should be grasped
    :return: A list of objects the robot is in collision with when reaching for the specified object or None if the pose
    or object is not reachable.
    """
    if type(pose_or_object) == Object:
        input_pose = pose_or_object.get_pose()
    else:
        input_pose = pose_or_object

    shadow_robot = BulletWorld.current_bullet_world.get_shadow_object(robot)
    with Use_shadow_world():
        arm = "left" if gripper_name == robot_description.get_tool_frame("left") else "right"
        joints = robot_description.chains[arm].joints
        local_transformer = LocalTransformer()

        target_map = local_transformer.transform_pose(input_pose, "map")
        if grasp:
            grasp_orientation = robot_description.grasps.get_orientation_for_grasp(grasp)
            target_map.orientation.x = grasp_orientation[0]
            target_map.orientation.y = grasp_orientation[1]
            target_map.orientation.z = grasp_orientation[2]
            target_map.orientation.w = grasp_orientation[3]

        try:
            inv = request_ik(target_map, shadow_robot, joints, gripper_name)
        except IKError as e:
            rospy.logerr(f"Pose is not reachable: {e}")
            return None
        _apply_ik(shadow_robot, inv, joints)

        block = []
        for obj in BulletWorld.current_bullet_world.objects:
            if contact(shadow_robot, obj):
                block.append(BulletWorld.current_bullet_world.get_bullet_object_for_shadow(obj))
    return block


def supporting(object1: Object,
               object2: Object) -> bool:
    """
    Checks if one object is supporting another object. An object supports another object if they are in
    contact and the second object is above the first one. (e.g. a Bottle will be supported by a table)

    :param object1: Object that is supported
    :param object2: Object that supports the first object
    :return: True if the second object is in contact with the first one and the second one ist above the first False else
    """
    return contact(object1, object2) and object2.get_position().z > object1.get_position().z


def link_pose_for_joint_config(object: Object, joint_config: Dict[str, float], link_name: str) -> Pose:
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
        return shadow_object.get_link_pose(link_name)
