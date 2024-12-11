import numpy as np
import tf
from typing_extensions import Tuple, List, Union, Dict, Iterable, Optional

from .datastructures.enums import Arms, Grasp
from .costmaps import Costmap
from .datastructures.pose import Pose, Transform
from .datastructures.world import World
from .external_interfaces.ik import request_ik
from .failures import IKError
from .local_transformer import LocalTransformer
from .robot_description import RobotDescription
from .ros.logging import logdebug
from .world_concepts.world_object import Object
from .world_reasoning import contact


class PoseGenerator:
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
        if PoseGenerator.override_orientation_generator:
            self.orientation_generator = PoseGenerator.override_orientation_generator

    def __iter__(self) -> Iterable:
        """
        A generator that crates pose candidates from a given costmap. The generator
        selects the highest 100 values and returns the corresponding positions.
        Orientations are calculated such that the Robot faces the center of the costmap.

        :Yield: A tuple of position and orientation
        """

        # Determines how many positions should be sampled from the costmap
        if self.number_of_samples == -1 or self.number_of_samples > self.costmap.map.flatten().shape[0]:
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

        :param position: The position in the costmap. This position is already converted to the world coordinate frame.
        :param origin: The origin of the costmap. This is also the point which the robot should face.
        :return: A quaternion of the calculated orientation
        """
        angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) + np.pi
        quaternion = list(tf.transformations.quaternion_from_euler(0, 0, angle, axes="sxyz"))
        return quaternion


def visibility_validator(pose: Pose,
                         robot: Object,
                         object_or_pose: Union[Object, Pose],
                         world: World) -> bool:
    """
    This method validates if the robot can see the target position from a given
    pose candidate. The target position can either be a position, in world coordinate
    system, or an object in the World. The validation is done by shooting a
    ray from the camera to the target position and checking that it does not collide
    with anything else.

    :param pose: The pose candidate that should be validated
    :param robot: The robot object for which this should be validated
    :param object_or_pose: The target position or object for which the pose candidate should be validated.
    :param world: The World instance in which this should be validated.
    :return: True if the target is visible for the robot, None in any other case.
    """
    robot_pose = robot.get_pose()
    if isinstance(object_or_pose, Object):
        robot.set_pose(pose)
        camera_pose = robot.get_link_pose(RobotDescription.current_robot_description.get_camera_link())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose.get_position_as_list())
        res = ray == object_or_pose.id
    else:
        robot.set_pose(pose)
        camera_pose = robot.get_link_pose(RobotDescription.current_robot_description.get_camera_link())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        # TODO: Check if this is correct
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose)
        res = ray == -1
    robot.set_pose(robot_pose)
    return res


def _in_contact(robot: Object, obj: Object, allowed_collision: Dict[Object, List[str]],
                allowed_robot_links: List[str]) -> bool:
    """
    This method checks if a given robot is in contact with a given object.

    :param robot: The robot object that should be checked for contact.
    :param obj: The object that should be checked for contact with the robot.
    :param allowed_collision: A dictionary that contains the allowed collisions for links of each object in the world.
    :param allowed_robot_links: A list of links of the robot that are allowed to be in contact with the object.
    :return: True if the robot is in contact with the object and False otherwise.
    """
    in_contact, contact_links = contact(robot, obj, return_links=True)
    allowed_links = allowed_collision[obj] if obj in allowed_collision.keys() else []

    if in_contact:
        if all(link[0].name in allowed_robot_links or link[1].name in allowed_links for link in contact_links):
            in_contact = False
    return in_contact


def reachability_validator(pose: Pose,
                           robot: Object,
                           target: Union[Object, Pose],
                           prepose_distance: float = 0.03,
                           allowed_collision: Dict[Object, List] = None,
                           arm: Optional[Arms] = None) -> Tuple[bool, List[Arms]]:
    """
    This method validates if a target position is reachable for a pose candidate.
    This is done by asking the ik solver if there is a valid solution if the
    robot stands at the position of the pose candidate. if there is a solution
    the validator returns True and False in any other case.

    :param pose: The pose candidate for which the reachability should be validated
    :param robot: The robot object in the World for which the reachability should be validated.
    :param target: The target position or object instance which should be the target for reachability.
    :param prepose_distance: The distance the robot should retract from the target position after/before reaching it.
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates
     to a list of links of which this object consists
    :param arm: The arm that should be used for the reachability check. If None all arms are checked.
    :return: True if the target is reachable for the robot and False in any other case.
    """
    if type(target) == Object:
        target = target.get_pose()

    robot.set_pose(pose)

    if arm is not None:
        manipulator_descs = [RobotDescription.current_robot_description.get_arm_chain(arm)]
    else:
        manipulator_descs = RobotDescription.current_robot_description.get_manipulator_chains()

    # TODO Make orientation adhere to grasping orientation
    res = False
    arms = []
    for description in manipulator_descs:
        retract_target_pose = LocalTransformer().transform_pose(target, robot.get_link_tf_frame(
            description.end_effector.tool_frame))
        retract_target_pose.position.x -= prepose_distance  # Care hard coded value copied from PlaceAction class

        # retract_pose needs to be in world frame?
        retract_target_pose = LocalTransformer().transform_pose(retract_target_pose, "map")

        joints = description.joints
        tool_frame = description.end_effector.tool_frame

        # TODO Make orientation adhere to grasping orientation

        joint_state_before_ik = robot.get_positions_of_all_joints()
        try:
            # test the possible solution and apply it to the robot
            pose, joint_states = request_ik(target, robot, joints, tool_frame)
            logdebug(f"Robot {description.name} can reach the the target pose")
            robot.set_pose(pose)
            robot.set_multiple_joint_positions(joint_states)

            in_contact = collision_check(robot, allowed_collision)
            if not in_contact:  # only check for retract pose if pose worked
                logdebug("Robot is not in contact at target pose")
                pose, joint_states = request_ik(retract_target_pose, robot, joints, tool_frame)
                logdebug(f"Robot {description.name} can reach retract pose")
                robot.set_pose(pose)
                robot.set_multiple_joint_positions(joint_states)
                in_contact = collision_check(robot, allowed_collision)
            if not in_contact:
                logdebug("Robot is not in contact at retract pose")
                arms.append(description.arm_type)
        except IKError:
            logdebug(f"Robot {description.name} cannot reach pose")
            pass
        finally:
            robot.set_multiple_joint_positions(joint_state_before_ik)
    if arms:
        res = True
    return res, arms


def collision_check(robot: Object, allowed_collision: Dict[Object, List]):
    """
    This method checks if a given robot collides with any object within the world
    which it is not allowed to collide with.
    This is done checking iterating over every object within the world and checking
    if the robot collides with it. Careful the floor will be ignored.
    If there is a collision with an object that was not within the allowed collision
    list the function returns True else it will return False

    :param robot: The robot object in the (Bullet)World where it should be checked if it collides with something
    :param allowed_collision: dict of objects with which the robot is allowed to collide each object correlates to a list of links of which this object consists
    :return: True if the target is reachable for the robot and False in any other case.
    """
    in_contact = False
    allowed_robot_links = []
    if robot in allowed_collision.keys():
        allowed_robot_links = allowed_collision[robot]

    for obj in World.current_world.objects:
        if obj.name == "floor":
            continue
        in_contact = _in_contact(robot, obj, allowed_collision, allowed_robot_links)
        if in_contact:
            break
    return in_contact
