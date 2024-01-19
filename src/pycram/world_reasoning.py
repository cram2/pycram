import pybullet as p
import itertools
import numpy as np
import rospy

from .world import _world_and_id, Object, UseProspectionWorld
from .world import World
from .external_interfaces.ik import request_ik
from .local_transformer import LocalTransformer
from .plan_failures import IKError
from .robot_descriptions import robot_description
from .helper import _apply_ik
from .pose import Pose, Transform
from typing import List, Tuple, Optional, Union, Dict


class ReasoningError(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)


class CollisionError(Exception):
    def __init__(self, *args, **kwargs):
        Exception.__init__(self, *args, **kwargs)


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


def _try_to_reach_or_grasp(pose_or_object: Union[Pose, Object], prospection_robot: Object, gripper_name: str,
                           grasp: Optional[str] = None) -> Union[Pose, None]:
    """
    Checks if the robot can reach a given position and optionally also grasp an object.
    To determine this the inverse kinematics are calculated and applied.

    :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
    :param prospection_robot: The robot that should reach for the position
    :param gripper_name: The name of the end effector
    :param grasp: The grasp type with which the object should be grasped
    """
    if isinstance(pose_or_object, Object):
        input_pose = pose_or_object.get_pose()
    else:
        input_pose = pose_or_object

    arm = "left" if gripper_name == robot_description.get_tool_frame("left") else "right"
    joints = robot_description.chains[arm].joints
    target_pose = input_pose
    if grasp:
        local_transformer = LocalTransformer()
        target_map = local_transformer.transform_pose_to_target_frame(input_pose, "map")
        grasp_orientation = robot_description.grasps.get_orientation_for_grasp(grasp)
        target_map.orientation.x = grasp_orientation[0]
        target_map.orientation.y = grasp_orientation[1]
        target_map.orientation.z = grasp_orientation[2]
        target_map.orientation.w = grasp_orientation[3]
        target_pose = target_map

    try:
        inv = request_ik(target_pose, prospection_robot, joints, gripper_name)
    except IKError as e:
        rospy.logerr(f"Pose is not reachable: {e}")
        return None
    _apply_ik(prospection_robot, inv, joints)

    return target_pose


class WorldReasoning:

    def __init__(self, world: World):
        self.world = world

    def stable(self, obj: Object) -> bool:
        """
        Checks if an object is stable in the world. Stable meaning that it's position will not change after simulating physics
        in the BulletWorld. This will be done by simulating the world for 10 seconds and compare the previous coordinates
        with the coordinates after the simulation.

        :param obj: The object which should be checked
        :return: True if the given object is stable in the world False else
        """
        prospection_obj = self.world.get_prospection_object_from_object(obj)
        with UseProspectionWorld():
            coords_prev = prospection_obj.get_position_as_list()
            self.world.set_gravity([0, 0, -9.8])

            self.world.simulate(2)
            coords_past = prospection_obj.get_position_as_list()

            coords_prev = list(map(lambda n: round(n, 3), coords_prev))
            coords_past = list(map(lambda n: round(n, 3), coords_past))
            return coords_past == coords_prev

    def contact(self,
                object1: Object,
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

        with UseProspectionWorld():
            prospection_obj1 = self.world.get_prospection_object_from_object(object1)
            prospection_obj2 = self.world.get_prospection_object_from_object(object2)

            self.world.perform_collision_detection()
            con_points = self.world.get_contact_points_between_two_objects(prospection_obj1, prospection_obj2)

            if return_links:
                contact_links = []
                for point in con_points:
                    contact_links.append((prospection_obj1.get_link_by_id(point[3]), prospection_obj2.get_link_by_id(point[4])))
                return con_points != (), contact_links

            else:
                return con_points != ()

    def get_visible_objects(self, camera_pose: Pose,
                            front_facing_axis: Optional[List[float]] = None) -> Tuple[np.ndarray, Pose]:

        front_facing_axis = robot_description.front_facing_axis if not front_facing_axis else front_facing_axis

        world_to_cam = camera_pose.to_transform("camera")

        cam_to_point = Transform(list(np.multiply(front_facing_axis, 2)), [0, 0, 0, 1], "camera",
                                 "point")
        target_point = (world_to_cam * cam_to_point).to_pose()

        seg_mask = self.world.get_images_for_target(target_point, world_to_cam.to_pose())[2]

        return seg_mask, target_point

    def visible(self,
                obj: Object,
                camera_pose: Pose,
                front_facing_axis: Optional[List[float]] = None,
                threshold: float = 0.8) -> bool:
        """
        Checks if an object is visible from a given position. This will be achieved by rendering the object
        alone and counting the visible pixel, then rendering the complete scene and compare the visible pixels with the
        absolut count of pixels.

        :param obj: The object for which the visibility should be checked
        :param camera_pose: The pose of the camera in map frame
        :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
        :param threshold: The minimum percentage of the object that needs to be visible for this method to return true.
        :return: True if the object is visible from the camera_position False if not
        """
        with UseProspectionWorld():
            prospection_obj = self.world.get_prospection_object_from_object(obj)
            if World.robot:
                prospection_robot = self.world.get_prospection_object_from_object(World.robot)

            state_id = self.world.save_state()
            for obj in self.world.objects:
                if obj == prospection_obj or World.robot and obj == prospection_robot:
                    continue
                else:
                    obj.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))

            seg_mask, target_point = self.get_visible_objects(camera_pose, front_facing_axis)
            flat_list = list(itertools.chain.from_iterable(seg_mask))
            max_pixel = sum(list(map(lambda x: 1 if x == prospection_obj.id else 0, flat_list)))
            self.world.restore_state(state_id)

            if max_pixel == 0:
                # Object is not visible
                return False

            seg_mask = self.world.get_images_for_target(target_point, camera_pose)[2]
            flat_list = list(itertools.chain.from_iterable(seg_mask))
            real_pixel = sum(list(map(lambda x: 1 if x == prospection_obj.id else 0, flat_list)))

            return real_pixel / max_pixel > threshold > 0

    def occluding(self,
                  obj: Object,
                  camera_pose: Pose,
                  front_facing_axis: Optional[List[float]] = None) -> List[Object]:
        """
        Lists all objects which are occluding the given object. This works similar to 'visible'.
        First the object alone will be rendered and the position of the pixels of the object in the picture will be saved.
        After that the complete scene will be rendered and the previous saved pixel positions will be compared to the
        actual pixels, if in one pixel another object is visible ot will be saved as occluding.

        :param obj: The object for which occlusion should be checked
        :param camera_pose: The pose of the camera in world coordinate frame
        :param front_facing_axis: The axis, of the camera frame, which faces to the front of the robot. Given as list of xyz
        :return: A list of occluding objects
        """

        with UseProspectionWorld():
            state_id = self.world.save_state()
            for other_obj in self.world.objects:
                if other_obj.name == self.world.robot.name:
                    continue
                elif obj.get_pose() == other_obj.get_pose():
                    obj = other_obj
                else:
                    obj.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))

            seg_mask, target_point = self.get_visible_objects(camera_pose, front_facing_axis)

            # All indices where the object that could be occluded is in the image
            # [0] at the end is to reduce by one dimension because dstack adds an unnecessary dimension
            pix = np.dstack((seg_mask == obj.id).nonzero())[0]

            self.world.restore_state(state_id)

            occluding = []
            seg_mask = self.world.get_images_for_target(target_point, camera_pose)[2]
            for c in pix:
                if not seg_mask[c[0]][c[1]] == obj.id:
                    occluding.append(seg_mask[c[0]][c[1]])

            occ_objects = list(set(map(self.world.get_object_by_id, occluding)))
            occ_objects = list(map(self.world.get_object_from_prospection_object, occ_objects))

            return occ_objects

    def reachable(self,
                  pose_or_object: Union[Object, Pose],
                  robot: Object,
                  gripper_name: str,
                  threshold: float = 0.01) -> bool:
        """
        Checks if the robot can reach a given position. To determine this the inverse kinematics are
        calculated and applied. Afterward the distance between the position and the given end effector is calculated, if
        it is smaller than the threshold the reasoning query returns True, if not it returns False.

        :param pose_or_object: The position and rotation or Object for which reachability should be checked or an Object
        :param robot: The robot that should reach for the position
        :param gripper_name: The name of the end effector
        :param threshold: The threshold between the end effector and the position.
        :return: True if the end effector is closer than the threshold to the target position, False in every other case
        """

        prospection_robot = self.world.get_prospection_object_from_object(robot)
        with UseProspectionWorld():
            target_pose = _try_to_reach_or_grasp(pose_or_object, robot, gripper_name)

            if not target_pose:
                return False

            diff = target_pose.dist(prospection_robot.links[gripper_name].pose)

        return diff < threshold

    def blocking(self,
                 pose_or_object: Union[Object, Pose],
                 robot: Object,
                 gripper_name: str,
                 grasp: str = None) -> Union[List[Object], None]:
        """
        Checks if any objects are blocking another object when a robot tries to pick it. This works
        similar to the reachable predicate. First the inverse kinematics between the robot and the object will be
        calculated and applied. Then it will be checked if the robot is in contact with any object except the given one.
        If the given pose or Object is not reachable None will be returned

        :param pose_or_object: The object or pose for which blocking objects should be found
        :param robot: The robot Object who reaches for the object
        :param gripper_name: The name of the end effector of the robot
        :param grasp: The grasp type with which the object should be grasped
        :return: A list of objects the robot is in collision with when reaching for the specified object or None if the pose
        or object is not reachable.
        """
        prospection_robot = self.world.get_prospection_object_from_object(robot)
        with UseProspectionWorld():
            _try_to_reach_or_grasp(pose_or_object, robot, gripper_name, grasp)

            block = []
            for obj in self.world.objects:
                if self.contact(prospection_robot, obj):
                    block.append(self.world.get_object_from_prospection_object(obj))
        return block

    def supporting(self,
                   object1: Object,
                   object2: Object) -> bool:
        """
        Checks if one object is supporting another object. An object supports another object if they are in
        contact and the second object is above the first one. (e.g. a Bottle will be supported by a table)

        :param object1: Object that is supported
        :param object2: Object that supports the first object
        :return: True if the second object is in contact with the first one and the second one ist above the first False else
        """
        return self.contact(object1, object2) and object2.get_position().z > object1.get_position().z

    def link_pose_for_joint_config(self, obj: Object, joint_config: Dict[str, float], link_name: str) -> Pose:
        """
        Returns the pose a link would be in if the given joint configuration would be applied to the object.
        This is done by using the respective object in the prospection world and applying the joint configuration
        to this one. After applying the joint configuration the link position is taken from there.

        :param obj: Object of which the link is a part
        :param joint_config: Dict with the goal joint configuration
        :param link_name: Name of the link for which the pose should be returned
        :return: The pose of the link after applying the joint configuration
        """
        prospection_object = self.world.get_prospection_object_from_object(obj)
        with UseProspectionWorld():
            for joint, pose in joint_config.items():
                prospection_object.set_joint_position(joint, pose)
            return prospection_object.links[link_name].pose
