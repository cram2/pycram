import tf
import numpy as np
import rospy
import pybullet as p

from .bullet_world import Object, BulletWorld, Use_shadow_world
from .bullet_world_reasoning import contact
from .costmaps import Costmap
from .pose import Pose, Transform
from .robot_description import ManipulatorDescription
from .robot_descriptions import robot_description
from .external_interfaces.ik import request_ik
from .plan_failures import IKError
from .helper import _apply_ik
from typing import Type, Tuple, List, Union, Dict, Iterable


class PoseGenerator:
    current_orientation_generator = None

    def __init__(self, costmap: Costmap, number_of_samples=100, orientation_generator=None):
        """
       :param costmap: The costmap from which poses should be sampled.
       :param number_of_samples: The number of samples from the costmap that should be returned at max
       :param orientation_generator: function that generates an orientation given a position and the origin of the costmap
        """

        if not PoseGenerator.current_orientation_generator:
            PoseGenerator.current_orientation_generator = PoseGenerator.generate_orientation

        self.costmap = costmap
        self.number_of_samples = number_of_samples
        self.orientation_generator = orientation_generator if orientation_generator else PoseGenerator.current_orientation_generator

    def __iter__(self) -> Iterable:
        """
        A generator that crates pose candidates from a given costmap. The generator
        selects the highest 100 values and returns the corresponding positions.
        Orientations are calculated such that the Robot faces the center of the costmap.

        :Yield: A tuple of position and orientation
        """

        # Determines how many positions should be sampled from the costmap
        if self.number_of_samples == -1:
            self.number_of_samples = self.costmap.map.flatten().shape[0]
        indices = np.argpartition(self.costmap.map.flatten(), -self.number_of_samples)[-self.number_of_samples:]
        indices = np.dstack(np.unravel_index(indices, self.costmap.map.shape)).reshape(self.number_of_samples, 2)

        height = self.costmap.map.shape[0]
        width = self.costmap.map.shape[1]
        center = np.array([height // 2, width // 2])
        for ind in indices:
            if self.costmap.map[ind[0]][ind[1]] == 0:
                continue
            # The position is calculated by creating a vector from the 2D position in the costmap (given by x and y)
            # and the center of the costmap (since this is the origin). This vector is then turned into a transformation
            # and muiltiplied with the transformation of the origin.
            vector_to_origin = (center - ind) * self.costmap.resolution
            point_to_origin = Transform([*vector_to_origin, 0], frame="point", child_frame="origin")
            origin_to_map = self.costmap.origin.to_transform("origin").invert()
            point_to_map = point_to_origin * origin_to_map
            map_to_point = point_to_map.invert()

            orientation = self.orientation_generator(map_to_point.translation_as_list(), self.costmap.origin)
            yield Pose(map_to_point.translation_as_list(), orientation)

    @staticmethod
    def height_generator() -> float:
        pass

    @staticmethod
    def generate_orientation(position: List[float], origin: Pose) -> List[float]:
        """
        This method generates the orientation for a given position in a costmap. The
        orientation is calculated such that the robot faces the origin of the costmap.
        This generation is done by simply calculating the arctan between the position,
        in the costmap, and the origin of the costmap.

        :param position: The position in the costmap. This position is already converted
            to the world coordinate frame.
        :param origin: The origin of the costmap. This is also the point which the
            robot should face.
        :return: A quaternion of the calculated orientation
        """
        angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) + np.pi
        quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return quaternion


def visibility_validator(pose: Pose,
                         robot: Object,
                         object_or_pose: Union[Object, Pose],
                         world: BulletWorld) -> bool:
    """
    This method validates if the robot can see the target position from a given
    pose candidate. The target position can either be a position, in world coordinate
    system, or an object in the BulletWorld. The validation is done by shooting a
    ray from the camera to the target position and checking that it does not collide
    with anything else.

    :param pose: The pose candidate that should be validated
    :param robot: The robot object for which this should be validated
    :param object_or_pose: The target position or object for which the pose
        candidate should be validated.
    :param world: The BulletWorld instance in which this should be validated.
    :return: True if the target is visible for the robot, None in any other case.
    """
    robot_pose = robot.get_pose()
    if type(object_or_pose) == Object:
        robot.set_pose(pose)
        camera_pose = robot.get_link_pose(robot_description.get_camera_frame())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        ray = p.rayTest(camera_pose.position_as_list(), object_or_pose.get_pose().position_as_list(),
                        physicsClientId=world.client_id)
        res = ray[0][0] == object_or_pose.id
    else:
        robot.set_pose(pose)
        camera_pose = robot.get_link_pose(robot_description.get_camera_frame())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        ray = p.rayTest(camera_pose.position_as_list(), object_or_pose, physicsClientId=world.client_id)
        res = ray[0][0] == -1
    robot.set_pose(robot_pose)
    return res


def reachability_validator(pose: Pose,
                           robot: Object,
                           target: Union[Object, Pose],
                           allowed_collision: Dict[Object, List] = None) -> Tuple[bool, List]:
    """
    This method validates if a target position is reachable for a pose candidate.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. if there is a solution
    the validator returns True and False in any other case.

    :param pose: The pose candidate for which the reachability should be validated
    :param robot: The robot object in the BulletWorld for which the reachability
        should be validated.
    :param target: The target position or object instance which should be the
        target for reachability.
    :param allowed_collision:
    :return: True if the target is reachable for the robot and False in any other
        case.
    """
    if type(target) == Object:
        target = target.get_pose()

    robot.set_pose(pose)
    manipulator_descs = list(
        filter(lambda chain: isinstance(chain[1], ManipulatorDescription), robot_description.chains.items()))

    # TODO Make orientation adhere to grasping orientation
    res = False
    arms = []
    in_contact = False

    allowed_robot_links = []
    if robot in allowed_collision.keys():
        allowed_robot_links = allowed_collision[robot]

    for chain_name, chain in manipulator_descs:
        joint_state_before_ik = robot._current_joint_states
        try:
            resp = request_ik(target, robot, chain.joints, chain.tool_frame)

            _apply_ik(robot, resp, chain.joints)

            for obj in BulletWorld.current_bullet_world.objects:
                if obj.name == "floor":
                    continue
                in_contact, contact_links = contact(robot, obj, return_links=True)
                allowed_links = allowed_collision[obj] if obj in allowed_collision.keys() else []

                if in_contact:
                    for link in contact_links:

                        if link[0] in allowed_robot_links or link[1] in allowed_links:
                            in_contact = False

            if not in_contact:
                arms.append(chain_name)
                res = True
        except IKError:
            pass
        finally:
            robot.set_joint_states(joint_state_before_ik)
    return res, arms
