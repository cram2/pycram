import functools
import random
from copy import deepcopy

import numpy as np
from semantic_digital_twin.collision_checking.collision_detector import (
    CollisionCheck,
    Collision,
    CollisionDetector,
)
from semantic_digital_twin.collision_checking.trimesh_collision_detector import (
    TrimeshCollisionDetector,
)
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Box, Scale
from semantic_digital_twin.spatial_computations.ik_solver import (
    MaxIterationsException,
    UnreachableException,
)
from semantic_digital_twin.spatial_computations.raytracer import RayTracer
from semantic_digital_twin.robots.abstract_robot import AbstractRobot
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.world_entity import Body, KinematicStructureEntity
from semantic_digital_twin.world import World

from .tf_transformations import quaternion_from_euler
from typing_extensions import Tuple, List, Union, Dict, Iterable, Optional, Iterator

from .costmaps import Costmap, SemanticCostmap
from .datastructures.pose import PoseStamped, TransformStamped
from .failures import IKError, RobotInCollision
from .robot_description import RobotDescription
from .ros import logdebug
from .world_reasoning import contact


class OrientationGenerator:
    """
    Provides methods to generate orientations for pose candidates.
    """

    @staticmethod
    def generate_origin_orientation(
        position: List[float], origin: PoseStamped
    ) -> List[float]:
        """
        Generates an orientation such that the robot faces the origin of the costmap.

        :param position: The position in the costmap, already converted to the world coordinate frame.
        :param origin: The origin of the costmap, the point which the robot should face.
        :return: A quaternion of the calculated orientation.
        """
        angle = (
            np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x)
            + np.pi
        )
        quaternion = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return quaternion

    @staticmethod
    def generate_random_orientation(
        *_, rng: random.Random = random.Random(42)
    ) -> List[float]:
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

    def __init__(
        self,
        costmap: Costmap,
        number_of_samples=100,
        orientation_generator=None,
        randomize=False,
    ):
        """
        :param costmap: The costmap from which poses should be sampled.
        :param number_of_samples: The number of samples from the costmap that should be returned at max
        :param orientation_generator: function that generates an orientation given a position and the origin of the costmap
        """

        if not PoseGenerator.current_orientation_generator:
            PoseGenerator.current_orientation_generator = (
                OrientationGenerator.generate_origin_orientation
            )

        self.randomize = randomize
        self.costmap = costmap
        self.number_of_samples = number_of_samples
        self.orientation_generator = (
            orientation_generator
            if orientation_generator
            else PoseGenerator.current_orientation_generator
        )
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
        if (
            self.number_of_samples == -1
            or self.number_of_samples > self.costmap.map.flatten().shape[0]
        ):
            self.number_of_samples = self.costmap.map.flatten().shape[0]
        if self.randomize:
            indices = np.random.choice(
                self.costmap.map.size, self.number_of_samples, replace=False
            )
        else:
            indices = np.argpartition(
                self.costmap.map.flatten(), -self.number_of_samples
            )[-self.number_of_samples :]

        indices = np.dstack(np.unravel_index(indices, self.costmap.map.shape)).reshape(
            self.number_of_samples, 2
        )

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
            point_to_origin = TransformStamped.from_list(
                [*vector_to_origin, 0], frame="point", child_frame_id="origin"
            )
            origin_to_map = ~self.costmap.origin.to_transform_stamped("origin")
            point_to_map = point_to_origin * origin_to_map
            map_to_point = ~point_to_map

            orientation = self.orientation_generator(
                map_to_point.translation.to_list(), self.costmap.origin
            )
            yield PoseStamped.from_list(
                 map_to_point.translation.to_list(), orientation, self.costmap.world.root
            )

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
        angle = (
            np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x)
            + np.pi
        )
        quaternion = list(quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return quaternion


def visibility_validator(
    robot: AbstractRobot, object_or_pose: Union[Body, PoseStamped], world: World
) -> bool:
    """
    This method validates if the robot can see the target position from a given
    pose candidate. The target position can either be a position, in world coordinate
    system, or an object in the World. The validation is done by shooting a
    ray from the camera to the target position and checking that it does not collide
    with anything else.

    :param robot: The robot object for which this should be validated
    :param object_or_pose: The target position or object for which the pose candidate should be validated.
    :param world: The world in which the visibility should be validated.
    :return: True if the target is visible for the robot, None in any other case.
    """
    if isinstance(object_or_pose, PoseStamped):
        gen_body = Body(
            name=PrefixedName("vist_test_obj", "pycram"),
            collision=ShapeCollection([Box(scale=Scale(0.1, 0.1, 0.1))]),
        )
        with world.modify_world():
            world.add_connection(Connection6DoF(world.root, gen_body, _world=world))
        gen_body.parent_connection.origin = object_or_pose.to_spatial_type()
    else:
        gen_body = object_or_pose
    r_t = world.ray_tracer
    camera = list(robot.neck.sensors)[0]
    ray = r_t.ray_test(
        camera.bodies[0].global_pose.to_position().to_np()[:3],
        gen_body.global_pose.to_position().to_np()[:3],
        multiple_hits=True,
    )

    hit_bodies = [b for b in ray[2] if not b in robot.bodies]

    if isinstance(object_or_pose, PoseStamped):
        with world.modify_world():
            world.remove_connection(gen_body.parent_connection)
            world.remove_kinematic_structure_entity(gen_body)

    return hit_bodies[0] == gen_body if len(hit_bodies) > 0 else False


def reachability_validator(
    root: KinematicStructureEntity,
    tip: KinematicStructureEntity,
    target_pose: PoseStamped,
    world: World,
    allowed_collision: List[CollisionCheck] = None,
) -> Optional[Dict[str, float]]:
    """
    This method validates if a target position is reachable for the robot.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. if there is a solution
    the validator returns True and False in any other case.

    :param root: The body which is the root of the kinematic chain to be used for ik.
    :param tip: The body which is the tip of the kinematic chain to be used for ik.
    :param target_pose: The target pose for which the reachability should be validated.
    :param arm: The arm that should be checked for reachability.
    :param world: The world in which the robot is located.
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates
     to a list of links of which this object consists

    :return: The final joint states of the robot if the target pose is reachable without collision, None in any other case.
    """
    old_state = deepcopy(world.state.data)
    try:
        joint_states = world.compute_inverse_kinematics(
            root, tip, target_pose.to_spatial_type(), max_iterations=600
        )
        for dof, value in joint_states.items():
            world.state[dof.name].position = value
        world.notify_state_change()

        # collision_check(robot, allowed_collision)
        logdebug(f"Robot is not in contact at target pose")

        return joint_states

    except (IKError, RobotInCollision, MaxIterationsException, UnreachableException):
        return None
    finally:
        world.state.data = old_state
        world.notify_state_change()


def pose_sequence_reachability_validator(
    root: KinematicStructureEntity,
    tip: KinematicStructureEntity,
    target_sequence: List[PoseStamped],
    world: World,
    allowed_collision: List[CollisionCheck] = None,
) -> bool:
    """
    This method validates if a target sequence, if traversed in order, is reachable for the robot.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. If there is a solution
    the validator returns The arm that can reach the target position and None in any other case.

    :param root: The body which is the root of the kinematic chain to be used for ik.
    :param tip: The body which is the tip of the kinematic chain to be used
    :param target_sequence: The target sequence of poses for which the reachability should be validated.
    :param world: The world in which the robot is located.
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates
     to a list of links of which this object consists

    :return: The arm that can reach the target position and None in any other case.
    """
    old_state = world.state.data
    for target_pose in target_sequence:
        joint_states = reachability_validator(
            root, tip, target_pose, world, allowed_collision
        )
        if joint_states is None:
            world.state.data = old_state
            world.notify_state_change()
            return False
        world.state.data = old_state
        world.notify_state_change()

    world.state.data = old_state
    world.notify_state_change()
    return True


def collision_check(
    robot: AbstractRobot, allowed_collision: List[Body], world: World
) -> List[Collision]:
    """
    This method checks if a given robot collides with any object within the world
    which it is not allowed to collide with.
    This is done checking iterating over every object within the world and checking
    if the robot collides with it. Careful the floor will be ignored.
    If there is a collision with an object that was not within the allowed collision
    list the function will raise a RobotInCollision exception.

    :param robot: The robot object in the (Bullet)World where it should be checked if it collides with something
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates to a list of links of which this object consists
    :param world: The world in which collision should be checked
    :raises: RobotInCollision if the robot collides with an object it is not allowed to collide with.
    """
    collision_matrix = create_collision_matrix(allowed_collision, world, robot)

    return world.collision_detector.check_collisions(collision_matrix)


def create_collision_matrix(
    ignore_collision_with: List[Body], world: World, robot: AbstractRobot
) -> List[CollisionCheck]:
    """
    CCreates a list of collision checks that should be performed

    :param ignore_collision_with: List of objects for which collision should be ignored
    :param world: The world in which the collision check should be performed
    :param robot: The robot for which the collision check should be performed
    :return: A list of collision checks that should be performed
    """
    collision_checks = []
    attached_bodies = set(robot.bodies) - set(
        world.get_kinematic_structure_entities_of_branch(robot.root)
    )
    allowed_collision_with = ignore_collision_with + list(attached_bodies)

    for robot_body in robot.bodies_with_enabled_collision:
        for world_body in world.bodies_with_enabled_collision:
            if (
                world_body in allowed_collision_with
                or robot_body in allowed_collision_with
                or world_body in robot.bodies
            ):
                continue
            collision_checks.append(CollisionCheck(robot_body, world_body, 0.01, world))

    return collision_checks
