import tf
import numpy as np

from .datastructures.enums import Grasp
from .datastructures.world import World
from .ros.viz_marker_publisher import AxisMarkerPublisher
from .world_concepts.world_object import Object
from .world_reasoning import contact
from .costmaps import Costmap
from .local_transformer import LocalTransformer
from .datastructures.pose import Pose, Transform
from .robot_description import RobotDescription
from .external_interfaces.ik import request_ik
from .plan_failures import IKError
from .utils import _apply_ik, translate_relative_to_object
from typing_extensions import Tuple, List, Union, Dict, Iterable


class PoseGenerator:
    """
    Crates pose candidates from a given costmap. The generator
    selects the highest values, amount is given by number_of_sample, and returns the corresponding positions.
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
        Initializes a PoseGenerator for sampling poses from a given costmap.

        This class generates sampled poses within the specified costmap, using a defined number of samples
        and an optional orientation generator. If no orientation generator is provided, a default generator
        is used.

        Args:
            costmap (Costmap): The costmap from which poses should be sampled.
            number_of_samples (int, optional): Maximum number of samples to be returned from the costmap. Defaults to 100.
            orientation_generator (callable, optional): Function that generates an orientation given a position and the origin of the costmap.

        Returns:
            None
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
        Generates pose candidates from a costmap, selecting the highest 100 values and
        returning the corresponding positions. The orientations are calculated such that
        the robot faces the center of the costmap.

        Yields:
            Pose: A Pose object containing position and orientation.
        """

        np.random.seed(42)

        if self.number_of_samples == -1:
            self.number_of_samples = self.costmap.map.flatten().shape[0]
        number_of_samples = min(self.number_of_samples, self.costmap.map.size)

        height, width = self.costmap.map.shape
        center = np.array([height // 2, width // 2])

        flat_values = self.costmap.map.flatten()

        # Filter non-zero weights and adjust number_of_samples accordingly
        non_zero_indices = np.nonzero(flat_values)[0]
        non_zero_weights = flat_values[non_zero_indices]
        number_of_samples = min(number_of_samples, len(non_zero_indices))

        if non_zero_weights.sum() > 0:
            sampled_indices = np.random.choice(non_zero_indices, size=number_of_samples, replace=False,
                                               p=non_zero_weights / non_zero_weights.sum())
        else:
            sampled_indices = []

        indices = np.dstack(np.unravel_index(sampled_indices, self.costmap.map.shape)).reshape(number_of_samples, 2)

        sampled_weights = flat_values[sampled_indices]
        sorted_indices = indices[np.argsort(-sampled_weights)]

        for ind in sorted_indices:
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
        if RobotDescription.current_robot_description.name == "iai_donbot":
            angle = np.arctan2(position[1] - origin.position.y, position[0] - origin.position.x) - np.pi/2
        else:
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
        camera_pose = robot.get_link_pose(RobotDescription.current_robot_description.get_camera_frame())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose.get_position_as_list())
        res = ray == object_or_pose.id
    else:
        robot.set_pose(pose)
        camera_pose = robot.get_link_pose(RobotDescription.current_robot_description.get_camera_frame())
        robot.set_pose(Pose([100, 100, 0], [0, 0, 0, 1]))
        # TODO: Check if this is correct
        ray = world.ray_test(camera_pose.position_as_list(), object_or_pose)
        res = ray == -1
    robot.set_pose(robot_pose)
    return res


def _in_contact(robot: Object, obj: Object, allowed_collision: Dict[Object, List[str]],
                allowed_robot_links: List[str]) -> bool:
    """
    Checks if the specified robot is in contact with a given object, while accounting for
    allowed collisions on specific links.

    Args:
        robot (Object): The robot to check for contact.
        obj (Object): The object to check for contact with the robot.
        allowed_collision (Dict[Object, List[str]]): A dictionary of objects with lists of
                                                     link names allowed to collide.
        allowed_robot_links (List[str]): A list of robot link names that are allowed to be in
                                         contact with the object.

    Returns:
        bool: True if the robot is in contact with the object on non-allowed links, False otherwise.
    """
    in_contact, contact_links = contact(robot, obj, return_links=True)
    if not in_contact:
        return False

    allowed_links = allowed_collision.get(obj, [])

    for link in contact_links:
        if link[0].name not in allowed_robot_links and link[1].name not in allowed_links:
            return True

    return False



def reachability_validator(pose: Pose,
                           robot: Object,
                           target: Union[Object, Pose],
                           allowed_collision: Dict[Object, List] = None,
                           translation_value: float = 0.1) -> Tuple[bool, List]:
    """
    Validates if a target position is reachable for a given pose candidate.

    This method uses an IK solver to determine if a valid solution exists for the robot
    standing at the specified pose. If a solution is found, the validator returns `True`;
    otherwise, it returns `False`.

    Args:
        pose (Pose): The pose candidate for which reachability should be validated.
        robot (Object): The robot object in the world for which reachability is being validated.
        target (Union[Object, Pose]): The target position or object that should be reachable.
        allowed_collision (Dict[Object, List], optional): A dictionary of objects with which
            the robot is allowed to collide, where each object maps to a list of its constituent links.

    Returns:
        Tuple[bool, List]: A tuple where the first element is `True` if the target is reachable
        and `False` otherwise. The second element is a list of details about the solution or issues
        encountered during validation.
    """
    if type(target) == Object:
        target = target.get_pose()

    robot.set_pose(pose)
    # manipulator_descs = list(
    #    filter(lambda chain: isinstance(chain[1], ManipulatorDescription), robot_description.chains.items()))
    manipulator_descs = RobotDescription.current_robot_description.get_manipulator_chains()

    # TODO Make orientation adhere to grasping orientation
    res = False
    arms = []
    for description in manipulator_descs:

        joints = description.joints
        tool_frame = description.end_effector.tool_frame

        # TODO Make orientation adhere to grasping orientation
        in_contact = False

        joint_state_before_ik = robot.get_positions_of_all_joints()
        try:
            # test the possible solution and apply it to the robot
            pose, joint_states = request_ik(target, robot, joints, tool_frame)
            robot.set_pose(pose)
            robot.set_joint_positions(joint_states)
            # _apply_ik(robot, resp, joints)

            in_contact = collision_check(robot, allowed_collision)

            if not in_contact:  # only check for retract pose if pose worked

                palm_axis = RobotDescription.current_robot_description.get_palm_axis()

                retract_target_pose = translate_relative_to_object(target, palm_axis, translation_value)

                retract_target_pose = LocalTransformer().transform_pose(retract_target_pose, "map")

                marker = AxisMarkerPublisher()
                marker.publish([pose, retract_target_pose], length=0.3)

                pose, joint_states = request_ik(retract_target_pose, robot, joints, tool_frame)
                robot.set_pose(pose)
                robot.set_joint_positions(joint_states)
                # _apply_ik(robot, resp, joints)
                in_contact = collision_check(robot, allowed_collision)
            if not in_contact:
                arms.append(description.arm_type)
        except IKError:
            pass
        finally:
            robot.set_joint_positions(joint_state_before_ik)
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
        in_contact= _in_contact(robot, obj, allowed_collision, allowed_robot_links)

    return in_contact
