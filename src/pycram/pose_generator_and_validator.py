import functools
import random

import numpy as np

from pycrap.ontologies import PhysicalObject
from .object_descriptors.generic import ObjectDescription
from .tf_transformations import quaternion_from_euler
from typing_extensions import Tuple, List, Union, Dict, Iterable, Optional, Iterator

from .datastructures.enums import Arms
from .costmaps import Costmap, SemanticCostmap
from .datastructures.pose import PoseStamped, TransformStamped, GraspDescription
from .datastructures.world import World
from .external_interfaces.ik import request_ik
from .failures import IKError, RobotInCollision
from .local_transformer import LocalTransformer
from .multirobot import RobotManager
from .ros import logdebug
from .world_concepts.world_object import Object
from .world_reasoning import contact


class OrientationGenerator:
    """
    Provides methods to generate orientations for pose candidates.
    """

    @staticmethod
    def generate_origin_orientation(position: List[float], origin: PoseStamped) -> List[float]:
        """
        Generates an orientation such that the robot faces the origin of the costmap.

        :param position: The position in the costmap, already converted to the world coordinate frame.
        :param origin: The origin of the costmap, the point which the robot should face.
        :return: A quaternion of the calculated orientation.
        """
        angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) + np.pi
        quaternion = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return quaternion

    @staticmethod
    def generate_random_orientation(*_, rng: random.Random = random.Random(42)) -> List[float]:
        """
        Generates a random orientation rotated around the z-axis (yaw).
        A random angle is sampled using a provided RNG instance to ensure reproducibility.

        :param _: Ignored parameters to maintain compatibility with other orientation generators.
        :param rng: Random number generator instance for reproducible sampling.

        :return: A quaternion of the randomly generated orientation.
        """
        random_yaw = rng.uniform(0, 2 * np.pi)
        quaternion = list(quaternion_from_euler(0, 0, random_yaw, axes="sxyz"))
        return quaternion

class PoseGenerator(Iterable[PoseStamped]):
    """
    Creates pose candidates from a given costmap.
    The generator selects the highest values, amount is given by number_of_sample, and returns the corresponding
    positions.
    Orientations are calculated such that the Robot faces the center of the costmap.
    """

    current_orientation_generator = None
    """
    If no orientation generator is given, this generator is used to generate the orientation of the robot.
    """
    override_orientation_generator = None
    """
    Override the orientation generator with a custom generator, which will be used regardless of the current_orientation_generator.
    """

    def __init__(self, costmap: Costmap, number_of_samples=100, orientation_generator=None, randomize=False):
        """
        :param costmap: The costmap from which poses should be sampled.
        :param number_of_samples: The number of samples from the costmap that should be returned at max
        :param orientation_generator: function that generates an orientation given a position and the origin of the costmap
        """

        if not PoseGenerator.current_orientation_generator:
            PoseGenerator.current_orientation_generator = OrientationGenerator.generate_origin_orientation

        self.randomize = randomize
        self.costmap = costmap
        self.number_of_samples = number_of_samples
        self.orientation_generator = orientation_generator if orientation_generator else PoseGenerator.current_orientation_generator
        if PoseGenerator.override_orientation_generator:
            self.orientation_generator = PoseGenerator.override_orientation_generator

    def __iter__(self) -> Iterator[PoseStamped]:
        """
        A generator that crates pose candidates from a given costmap. The generator
        selects the highest 100 values and returns the corresponding positions.
        Orientations are calculated such that the Robot faces the center of the costmap.

        :Yield: A tuple of position and orientation
        """

        # Determines how many positions should be sampled from the costmap
        if self.number_of_samples == -1 or self.number_of_samples > self.costmap.map.flatten().shape[0]:
            self.number_of_samples = self.costmap.map.flatten().shape[0]
        if self.randomize:
            indices = np.random.choice(self.costmap.map.size, self.number_of_samples, replace=False)
        else:
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
            point_to_origin = TransformStamped.from_list([*vector_to_origin, 0], frame="point", child_frame_id="origin")
            origin_to_map = ~self.costmap.origin.to_transform_stamped("origin")
            point_to_map = point_to_origin * origin_to_map
            map_to_point = ~point_to_map

            orientation = self.orientation_generator(map_to_point.translation.to_list(), self.costmap.origin)
            yield PoseStamped.from_list(map_to_point.translation.to_list(), orientation)

    @staticmethod
    def height_generator() -> float:
        pass

    @staticmethod
    def generate_orientation(position: List[float], origin: PoseStamped) -> List[float]:
        """
        This method generates the orientation for a given position in a costmap. The
        orientation is calculated such that the robot faces the origin of the costmap.
        This generation is done by simply calculating the arctan between the position,
        in the costmap, and the origin of the costmap.

        :param position: The position in the costmap. This position is already converted to the world coordinate frame.
        :param origin: The origin of the costmap. This is also the point which the robot should face.
        :return: A quaternion of the calculated orientation
        """
        angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) + np.pi
        quaternion = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return quaternion


def visibility_validator(robot: Object,
                         object_or_pose: Union[Object, PoseStamped]) -> bool:
    """
    This method validates if the robot can see the target position from a given
    pose candidate. The target position can either be a position, in world coordinate
    system, or an object in the World. The validation is done by shooting a
    ray from the camera to the target position and checking that it does not collide
    with anything else.

    :param robot: The robot object for which this should be validated
    :param object_or_pose: The target position or object for which the pose candidate should be validated.
    :return: True if the target is visible for the robot, None in any other case.
    """
    world = robot.world
    robot_pose = robot.get_pose()

    if isinstance(object_or_pose, PoseStamped):
        gen_obj_desc = ObjectDescription("viz_object", [0, 0, 0], [0.05, 0.05, 0.05])
        obj = Object("viz_object", PhysicalObject, pose=object_or_pose, description=gen_obj_desc)
    else:
        obj = object_or_pose

    obj_id = obj.id

    camera_pose = robot.get_link_pose(RobotManager.get_robot_description().get_camera_link())
    robot.set_pose(PoseStamped.from_list([100, 100, 0], [0, 0, 0, 1]))
    ray = world.ray_test(camera_pose.position.to_list(), obj.get_pose().position.to_list())
    robot.set_pose(robot_pose)
    if isinstance(object_or_pose, PoseStamped):
        world.remove_object(obj)

    return ray.obj_id == obj_id


def reachability_validator(robot: Object,
                           target_pose: PoseStamped,
                           arm: Arms,
                           allowed_collision: Dict[Object, List] = None
                           ) -> Optional[Dict[str, float]]:
    """
    This method validates if a target position is reachable for the robot.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. if there is a solution
    the validator returns True and False in any other case.

    :param robot: The robot object in the World for which the reachability should be validated.
    :param target_pose: The target pose for which the reachability should be validated.
    :param arm: The arm that should be checked for reachability.
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates
     to a list of links of which this object consists

    :return: The final joint states of the robot if the target pose is reachable without collision, None in any other case.
    """

    allowed_collision = allowed_collision or {}
    arm_chain = RobotManager.get_robot_description().get_arm_chain(arm)
    allowed_collision[robot] = [link for link in arm_chain.end_effector.links]

    joint_state_before_ik = robot.get_positions_of_all_joints()
    try:
        pose, joint_states = request_ik(target_pose, robot, arm_chain.joints, arm_chain.end_effector.tool_frame)
        logdebug(f"Robot {arm.name} can reach target pose")
        robot.set_pose(pose)
        robot.set_multiple_joint_positions(joint_states)
        collision_check(robot, allowed_collision)
        logdebug(f"Robot is not in contact at target pose")

        robot.set_multiple_joint_positions(joint_state_before_ik)
        return joint_states

    except (IKError, RobotInCollision):
        logdebug(f"Robot {arm.name} cannot reach pose without collision")
        return None
    finally:
        robot.set_multiple_joint_positions(joint_state_before_ik)


def pose_sequence_reachability_validator(robot: Object,
                                         target_sequence: List[PoseStamped],
                                         arm: Arms,
                                         allowed_collision: Dict[Object, List] = None
                                         ) -> bool:
    """
    This method validates if a target sequence, if traversed in order, is reachable for the robot.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. If there is a solution
    the validator returns The arm that can reach the target position and None in any other case.

    :param robot: The robot object in the World for which the reachability should be validated.
    :param target_sequence: The target sequence of poses for which the reachability should be validated.
    :param arm: The arm that should be checked for reachability.
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates
     to a list of links of which this object consists

    :return: The arm that can reach the target position and None in any other case.
    """
    joint_state_before_ik = robot.get_positions_of_all_joints()
    for target_pose in target_sequence:
        joint_states = reachability_validator(robot, target_pose, arm, allowed_collision)
        if joint_states is None:
            robot.set_multiple_joint_positions(joint_state_before_ik)
            return False
        robot.set_multiple_joint_positions(joint_states)

    robot.set_multiple_joint_positions(joint_state_before_ik)
    return True


def collision_check(robot: Object, allowed_collision: Dict[Object, List]):
    """
    This method checks if a given robot collides with any object within the world
    which it is not allowed to collide with.
    This is done checking iterating over every object within the world and checking
    if the robot collides with it. Careful the floor will be ignored.
    If there is a collision with an object that was not within the allowed collision
    list the function will raise a RobotInCollision exception.

    :param robot: The robot object in the (Bullet)World where it should be checked if it collides with something
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates to a list of links of which this object consists

    :raises: RobotInCollision if the robot collides with an object it is not allowed to collide with.
    """
    allowed_robot_links = allowed_collision.get(robot, [])
    for obj in World.current_world.objects:
        if obj.name == "floor":
            continue
        in_contact, contact_links = contact(robot, obj, return_links=True)
        if not in_contact:
            continue

        allowed_links = allowed_collision.get(obj, [])
        if not all(link[0].name in allowed_robot_links or link[1].name in allowed_links for link in contact_links):
            raise RobotInCollision(f"Collision detected between {robot} and {obj}.")
