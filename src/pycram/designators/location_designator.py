import dataclasses
import time
from enum import Enum

from typing_extensions import List, Union, Iterable, Optional, Callable, Tuple

from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..datastructures.world import World, UseProspectionWorld
from ..helper import adjust_grasp_for_object_rotation, calculate_object_faces, calculate_grasp_offset
from ..local_transformer import LocalTransformer
from ..ros.viz_marker_publisher import AxisMarkerPublisher
from ..utils import translate_relative_to_object
from ..world_reasoning import link_pose_for_joint_config
from ..designator import DesignatorError, LocationDesignatorDescription
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap, \
    DirectionalCostmap
from ..datastructures.enums import JointType, Arms, Grasp
from ..pose_generator_and_validator import PoseGenerator, visibility_validator, reachability_validator
from ..robot_description import RobotDescription
from ..datastructures.pose import Pose


class Location(LocationDesignatorDescription):
    """
    Default location designator which only wraps a pose.
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, pose: Pose, resolver=None):
        """
        Basic location designator that represents a single pose.

        :param pose: The pose that should be represented by this location designator
        :param resolver: An alternative specialized_designators that returns a resolved location
        """
        super().__init__(resolver)
        self.pose: Pose = pose

    def ground(self) -> Location:
        """
        Default specialized_designators which returns a resolved designator which contains the pose given in init.

        :return: A resolved designator
        """
        return self.Location(self.pose)


# TODO Maybe delete this
class ObjectRelativeLocation(LocationDesignatorDescription):
    """
    Location relative to an object
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        relative_pose: Pose
        """
        Pose relative to the object
        """
        reference_object: ObjectDesignatorDescription.Object
        """
        Object to which the pose is relative
        """

    def __init__(self, relative_pose: Pose = None, reference_object: ObjectDesignatorDescription = None,
                 resolver=None):
        """
        Location designator representing a location relative to a given object.

        :param relative_pose: Pose that should be relative, in world coordinate frame
        :param reference_object: Object to which the pose should be relative
        :param resolver: An alternative specialized_designators that returns a resolved location for the input parameter
        """
        super().__init__(resolver)
        self.relative_pose: Pose = relative_pose
        self.reference_object: ObjectDesignatorDescription = reference_object

    def ground(self) -> Location:
        """
        Default specialized_designators which returns a resolved location for description input. Resolved location is the first result
        of the iteration of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self) -> Iterable[Location]:
        """
        Iterates over all possible solutions for a resolved location that is relative to the given object.

        :yield: An instance of ObjectRelativeLocation.Location with the relative pose
        """
        if self.relative_pose is None or self.reference_object is None:
            raise DesignatorError(
                "Could not ground ObjectRelativeLocation: (Relative) pose and reference object must be given")
        # Fetch the object pose and yield the grounded description
        obj_grounded = self.reference_object.resolve()

        lt = LocalTransformer()
        pose = lt.transform_to_object_frame(self.relative_pose, obj_grounded)

        yield self.Location(self.relative_pose, pose, self.reference_object)


class CostmapLocation(LocationDesignatorDescription):
    """
    Uses costmaps to create locations based on complex constraints, such as reachability and visibility.
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        reachable_arms: List[Arms]
        """
        List of arms with which the pose can be reached; only relevant when the `reachable_for` parameter is specified.
        """

    def __init__(self, target: Union[Pose, ObjectDesignatorDescription.Object],
                 reachable_for: Optional[ObjectDesignatorDescription.Object] = None,
                 visible_for: Optional[ObjectDesignatorDescription.Object] = None,
                 reachable_arm: Optional[Arms] = None, resolver: Optional[Callable] = None,
                 used_grasps: Optional[List[Enum]] = None,
                 object_in_hand: Optional[ObjectDesignatorDescription.Object] = None):
        """
        Initializes a location designator that uses costmaps to calculate locations based on complex constraints,
        such as reachability and visibility.

        Args:
            target (Union[Pose, ObjectDesignatorDescription.Object]): The location for which visibility or reachability
                should be calculated.
            reachable_for (Optional[ObjectDesignatorDescription.Object]): The object (usually a robot) for which
                reachability is calculated.
            visible_for (Optional[ObjectDesignatorDescription.Object]): The object (usually a robot) for which
                visibility is calculated.
            reachable_arm (Optional[Arms]): An optional arm with which the target should be reached.
            resolver (Optional[Callable]): An alternative function that resolves a location for the input of this description.
            used_grasps (Optional[List[Enum]]): A list of grasps to use to reach the target.
            object_in_hand (Optional[ObjectDesignatorDescription.Object]): The object currently held by the robot, if any.
        """
        super().__init__(resolver)
        self.target: Union[Pose, ObjectDesignatorDescription.Object] = target
        self.reachable_for: ObjectDesignatorDescription.Object = reachable_for
        self.visible_for: ObjectDesignatorDescription.Object = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.used_grasps: Optional[List[Enum]] = used_grasps
        self.object_in_hand: Optional[ObjectDesignatorDescription.Object] = object_in_hand

    def ground(self) -> Location:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self):
        """
        Generates valid positions based on constraints derived from a costmap, yielding each
        that satisfies the given requirements.

        This method merges multiple costmaps, each with a unique purpose (e.g., occupancy,
        visibility, reachability) to form a comprehensive costmap. From this combined map,
        candidate poses are generated and validated against constraints defined in the designator.
        Each pose that meets the specified constraints is yielded as a valid location.

        Yields:
            CostmapLocation.Location: A location instance with a valid position that satisfies
            the constraints, including reachability and visibility as applicable.
        """
        min_height = RobotDescription.current_robot_description.get_default_camera().minimal_height
        max_height = RobotDescription.current_robot_description.get_default_camera().maximal_height
        # This ensures that the costmaps always get a position as their origin.
        if isinstance(self.target, ObjectDesignatorDescription.Object):
            target_pose = self.target.world_object.get_pose()
        else:
            target_pose = self.target.copy()

        ground_pose = Pose(target_pose.position_as_list())
        ground_pose.position.z = 0
        distance_to_obstacle = RobotDescription.current_robot_description.get_costmap_offset()

        max_reach = RobotDescription.current_robot_description.get_max_reach()
        map_size: int = int(max_reach * 100 * 3)
        map_resolution = 0.15

        occupancy = OccupancyCostmap(distance_to_obstacle, False, map_size * 2, map_resolution, ground_pose)
        final_map = occupancy
        if self.reachable_for:
            distance = (distance_to_obstacle + max_reach) / 2
            gaussian = GaussianCostmap(map_size, 1.5, map_resolution, ground_pose, True, distance)
            final_map += gaussian
        if self.visible_for:
            visible = VisibilityCostmap(min_height, max_height, map_size, map_resolution,
                                        Pose(target_pose.position_as_list()))
            final_map += visible

        directional = DirectionalCostmap(map_size * 3, self.used_grasps[0], map_resolution, target_pose,
                                         self.object_in_hand is not None)
        final_map *= directional

        if final_map.world.allow_publish_debug_poses:
            directional.publish()
            time.sleep(1)
            final_map.publish(weighted=True)

        if self.visible_for or self.reachable_for:
            robot_object = self.visible_for.world_object if self.visible_for else self.reachable_for.world_object
            test_robot = World.current_world.get_prospection_object_for_object(robot_object)

        if self.object_in_hand:
            local_transformer = LocalTransformer()
            object_pose = self.object_in_hand.world_object.get_pose()
            target_pose = self.target.copy()
            tcp_to_object = local_transformer.transform_pose(object_pose,
                                                             World.robot.get_link_tf_frame(
                                                                 RobotDescription.current_robot_description.get_arm_chain(
                                                                     self.reachable_arm).get_tool_frame()))

            target_pose = target_pose.to_transform("target").inverse_times(
                tcp_to_object.to_transform("object")).to_pose()
        else:
            adjusted_grasp = adjust_grasp_for_object_rotation(target_pose, self.used_grasps[0], self.reachable_arm)
            target_pose = target_pose.copy()
            target_pose.set_orientation(adjusted_grasp)
            grasp_offset = calculate_grasp_offset(self.target.world_object.get_object_dimensions(), self.reachable_arm,
                                                  self.used_grasps[0])

            palm_axis = RobotDescription.current_robot_description.get_palm_axis()

            target_pose = translate_relative_to_object(target_pose, palm_axis, grasp_offset)

        with UseProspectionWorld():
            for maybe_pose in PoseGenerator(final_map, number_of_samples=600):
                if final_map.world.allow_publish_debug_poses:
                    gripper_pose = World.robot.get_link_pose(
                        RobotDescription.current_robot_description.get_arm_chain(self.reachable_arm).get_tool_frame())
                    marker = AxisMarkerPublisher()
                    marker.publish([maybe_pose, target_pose, gripper_pose], length=0.3)
                res = True
                arms = None

                if self.visible_for:
                    res = res and visibility_validator(maybe_pose, test_robot, target_pose,
                                                       World.current_world)
                if self.reachable_for:
                    hand_links = [
                        link
                        for description in RobotDescription.current_robot_description.get_manipulator_chains()
                        for link in description.end_effector.links
                    ]
                    valid, arms = reachability_validator(maybe_pose, test_robot, target_pose,
                                                         allowed_collision={test_robot: hand_links})
                    if self.reachable_arm:
                        res = res and valid and self.reachable_arm in arms
                    else:
                        res = res and valid
                if res:
                    yield self.Location(maybe_pose, arms)


class AccessingLocation(LocationDesignatorDescription):
    """
    Designates a location for accessing and opening drawers.

    This designator provides the robot with a pose for interacting with drawer handles.
    Calculating this position before the drawer is opened is recommended to avoid
    potential issues with pose estimation.
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        arms: List[Arms]
        """
        List of arms that can be used for accessing from this pose.
        """

    def __init__(self, handle_desig: ObjectPart.Object, robot_desig: ObjectDesignatorDescription.Object, resolver=None):
        """
        Initializes a location designator for accessing a drawer handle.

        Args:
            handle_desig (ObjectPart.Object): The designator for the drawer handle to be accessed.
            robot_desig (ObjectDesignatorDescription.Object): The designator for the robot that will perform the action.
            resolver (Optional[Callable]): An optional custom resolver function for location creation.

        """
        super().__init__(resolver)
        self.handle: ObjectPart.Object = handle_desig
        self.robot: ObjectDesignatorDescription.Object = robot_desig.world_object

    def ground(self) -> Location:
        """
        Default specialized_designators for this location designator, just returns the first element from the iteration

        :return: A location designator for a pose from which the drawer can be opened
        """
        return next(iter(self))

    def __iter__(self) -> Tuple[Location, Location]:
        """
        Generates poses for the robot to access and open a drawer specified by the handle
        designator. Poses are validated to ensure the robot can grasp the handle while the drawer
        is closed and that the handle can still be grasped when the drawer is open.

        The process involves generating candidate poses using a merged costmap that incorporates
        occupancy, Gaussian, and directional costmaps. For each candidate pose, reachability
        validation checks are performed for both the initial and goal (fully opened) positions
        of the drawer. Only poses that satisfy all constraints are yielded.

        Yields:
            Tuple[AccessingLocation.Location, AccessingLocation.Location]: A tuple containing
            two location designators, one for the initial and one for the goal pose, with
            the arms that can be used for each.
        """
        ground_pose = Pose(self.handle.part_pose.position_as_list())
        ground_pose.position.z = 0
        test_robot = World.current_world.get_prospection_object_for_object(self.robot)

        # Find a Joint of type prismatic which is above the handle in the URDF tree
        if self.handle.name == "handle_cab3_door_top":
            container_joint = self.handle.world_object.find_joint_above_link(self.handle.name, JointType.REVOLUTE)
        else:
            container_joint = self.handle.world_object.find_joint_above_link(self.handle.name, JointType.PRISMATIC)

        init_pose = link_pose_for_joint_config(self.handle.world_object, {
            container_joint: self.handle.world_object.get_joint_limits(container_joint)[0]},
                                               self.handle.name)

        # Calculate the pose the handle would be in if the drawer was to be fully opened
        goal_pose = link_pose_for_joint_config(self.handle.world_object, {
            container_joint: self.handle.world_object.get_joint_limits(container_joint)[1] - 0.05},
                                               self.handle.name)

        # Handle position for calculating rotation of the final pose
        half_pose = link_pose_for_joint_config(self.handle.world_object, {
            container_joint: self.handle.world_object.get_joint_limits(container_joint)[1] / 1.5},
                                               self.handle.name)

        grasp = calculate_object_faces(self.handle)[0]
        grasp = Grasp.FRONT
        original_init_pose = init_pose.copy()
        init_pose, half_pose, goal_pose = init_pose.copy(), half_pose.copy(), goal_pose.copy()

        adjusted_init_pose_grasp = adjust_grasp_for_object_rotation(init_pose, grasp, Arms.LEFT)
        adjusted_half_pose_grasp = adjust_grasp_for_object_rotation(half_pose, grasp, Arms.LEFT)
        adjusted_goal_pose_grasp = adjust_grasp_for_object_rotation(goal_pose, grasp, Arms.LEFT)

        init_pose.set_orientation(adjusted_init_pose_grasp)
        half_pose.set_orientation(adjusted_half_pose_grasp)
        goal_pose.set_orientation(adjusted_goal_pose_grasp)

        distance_to_obstacle = RobotDescription.current_robot_description.get_costmap_offset()
        max_reach = RobotDescription.current_robot_description.get_max_reach()
        map_size: int = int(max_reach * 100 * 2.5)
        map_resolution = 0.15

        # TODO: find better strategy for distance_to_obstacle
        occupancy = OccupancyCostmap(distance_to_obstacle, False, map_size * 2, map_resolution, ground_pose)
        distance = (distance_to_obstacle + max_reach) / 2
        gaussian = GaussianCostmap(map_size, 1.5, map_resolution, ground_pose, True, distance)
        final_map = occupancy + gaussian

        directional = DirectionalCostmap(map_size*3, Grasp.FRONT, map_resolution, original_init_pose)
        final_map *= directional

        if final_map.world.allow_publish_debug_poses:
            final_map.publish(weighted=True)

        with (UseProspectionWorld()):
            for init_maybe_pose in PoseGenerator(final_map, number_of_samples=600,
                                                 orientation_generator=lambda p, o: PoseGenerator.generate_orientation(
                                                     p,
                                                     half_pose)):
                if final_map.world.allow_publish_debug_poses:
                    marker = AxisMarkerPublisher()
                    marker.publish([init_pose, half_pose, goal_pose, init_maybe_pose], length=0.5)

                hand_links = [
                    link
                    for description in RobotDescription.current_robot_description.get_manipulator_chains()
                    for link in description.links
                ]

                valid_init, arms_init = reachability_validator(init_maybe_pose, test_robot, init_pose,
                                                               allowed_collision={test_robot: hand_links})
                if valid_init:
                    valid_goal, arms_goal = reachability_validator(init_maybe_pose, test_robot, goal_pose,
                                                                   allowed_collision={test_robot: hand_links})
                    goal_maybe_pose = init_maybe_pose.copy()

                    if not valid_goal:
                        mapThandle = init_pose.to_transform("init_handle")
                        mapTmaybe = init_maybe_pose.to_transform("init_maybe")
                        handleTmaybe = mapThandle.invert() * mapTmaybe
                        goal_maybe_pose = (goal_pose.to_transform("goal_handle") * handleTmaybe).to_pose()

                        if final_map.world.allow_publish_debug_poses:
                            marker = AxisMarkerPublisher()
                            marker.publish([goal_maybe_pose], length=0.5)

                        valid_goal, arms_goal = reachability_validator(goal_maybe_pose, test_robot, goal_pose,
                                                                       allowed_collision={test_robot: hand_links})

                if valid_init and valid_goal and not set(arms_init).isdisjoint(set(arms_goal)):
                    yield self.Location(init_maybe_pose, list(set(arms_init).intersection(set(arms_goal)))), \
                          self.Location(goal_maybe_pose, list(set(arms_init).intersection(set(arms_goal))))


class SemanticCostmapLocation(LocationDesignatorDescription):
    """
    Locations over semantic entities, like a table surface
    """

    @dataclasses.dataclass
    class Location(LocationDesignatorDescription.Location):
        pass

    def __init__(self, urdf_link_name, part_of, for_object=None, resolver=None):
        """
        Creates a distribution over a urdf link to sample poses which are on this link. Can be used, for example, to find
        poses that are on a table. Optionally an object can be given for which poses should be calculated, in that case
        the poses are calculated such that the bottom of the object is on the link.

        :param urdf_link_name: Name of the urdf link for which a distribution should be calculated
        :param part_of: Object of which the urdf link is a part
        :param for_object: Optional object that should be placed at the found location
        :param resolver: An alternative specialized_designators that creates a resolved location for the input parameter of this description
        """
        super().__init__(resolver)
        self.urdf_link_name: str = urdf_link_name
        self.part_of: ObjectDesignatorDescription.Object = part_of
        self.for_object: Optional[ObjectDesignatorDescription.Object] = for_object

    def ground(self) -> Location:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self):
        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: An instance of SemanticCostmapLocation.Location with the found valid position of the Costmap.
        """
        sem_costmap = SemanticCostmap(self.part_of.world_object, self.urdf_link_name)
        height_offset = 0
        if self.for_object:
            min_p, max_p = self.for_object.world_object.get_axis_aligned_bounding_box().get_min_max_points()
            height_offset = (max_p.z - min_p.z) / 2
        for maybe_pose in PoseGenerator(sem_costmap):
            maybe_pose.position.z += height_offset
            yield self.Location(maybe_pose)
