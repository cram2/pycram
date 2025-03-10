import dataclasses

import numpy as np
from typing_extensions import List, Union, Iterable, Optional, Callable, Iterator

from .object_designator import ObjectDesignatorDescription, ObjectPart
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap, Costmap
from ..datastructures.enums import JointType, Arms, Grasp
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.pose import Pose
from ..datastructures.world import World, UseProspectionWorld
from ..designator import DesignatorError, LocationDesignatorDescription
from ..local_transformer import LocalTransformer
from ..object_descriptors.urdf import ObjectDescription
from ..pose_generator_and_validator import PoseGenerator, visibility_validator, reachability_validator
from ..robot_description import RobotDescription
from ..ros import logdebug
from ..world_concepts.world_object import Object, Link
from ..world_reasoning import link_pose_for_joint_config, contact, is_held_object, prospect_robot_contact
from ..config.action_conf import ActionConfig


class Location(LocationDesignatorDescription):
    """
    Default location designator which only wraps a pose.
    """

    def __init__(self, pose: Pose):
        """
        Basic location designator that represents a single pose.

        :param pose: The pose that should be represented by this location designator
        """
        super().__init__()
        self.pose: Pose = pose

    def ground(self) -> Pose:
        """
        Default specialized_designators which returns a resolved designator which contains the pose given in init.

        :return: A resolved designator
        """
        return self.pose


class CostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains
    """

    def __init__(self, target: Union[Pose, Object],
                 reachable_for: Optional[Object] = None,
                 visible_for: Optional[Object] = None,
                 reachable_arm: Optional[Arms] = None,
                 prepose_distance: float = ActionConfig.pick_up_prepose_distance,
                 check_collision_at_start: bool = True,
                 ignore_collision_with: Optional[List[Object]] = None,
                 grasps: Optional[List[Grasp]] = None):
        """
        Location designator that uses costmaps as base to calculate locations for complex constrains like reachable or
        visible. In case of reachable the resolved location contains a list of arms with which the location is reachable.

        :param target: Location for which visibility or reachability should be calculated
        :param reachable_for: Object for which the reachability should be calculated, usually a robot
        :param visible_for: Object for which the visibility should be calculated, usually a robot
        :param reachable_arm: An optional arm with which the target should be reached
        :param prepose_distance: Distance to the target pose where the robot should be checked for reachability.
        :param check_collision_at_start: If True, the designator will check for collisions at the start pose.
        :param ignore_collision_with: List of objects that should be ignored for collision checking.
        :param grasps: List of grasps that should be tried to reach the target pose
        """
        super().__init__()
        PartialDesignator.__init__(self, CostmapLocation, target=target, reachable_for=reachable_for,
                                   visible_for=visible_for,
                                   reachable_arm=reachable_arm, prepose_distance=prepose_distance,
                                   check_collision_at_start=check_collision_at_start,
                                   ignore_collision_with=ignore_collision_with, grasps=grasps)
        self.target: Union[Pose, Object] = target
        self.reachable_for: Object = reachable_for
        self.visible_for: Object = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.prepose_distance = prepose_distance
        self.check_collision_at_start = check_collision_at_start
        self.ignore_collision_with = ignore_collision_with if ignore_collision_with is not None else []
        self.grasps: List[Optional[Grasp]] = grasps if grasps is not None else [None]

    def ground(self) -> Pose:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[Pose]:
        """
           Generates positions for a given set of constrains from a costmap and returns
           them. The generation is based of a costmap which itself is the product of
           merging costmaps, each for a different purpose. In any case an occupancy costmap
           is used as the base, then according to the given constrains a visibility or
           gaussian costmap is also merged with this. Once the costmaps are merged,
           a generator generates pose candidates from the costmap. Each pose candidate
           is then validated against the constraints given by the designator if all validators
           pass the pose is considered valid and yielded.

           :yield: An instance of CostmapLocation.Location with a valid position that satisfies the given constraints
           """

        for params in self.generate_permutations():
            target = params["target"]
            reachable_for = params["reachable_for"]
            visible_for = params["visible_for"]
            reachable_arm = params["reachable_arm"]
            prepose_distance = params["prepose_distance"]
            check_collision_at_start = params["check_collision_at_start"]
            ignore_collision_with = params["ignore_collision_with"]
            grasps = params["grasps"]
            # This ensures that the costmaps always get a position as their origin.
            if isinstance(target, Object):
                target_pose = target.get_pose()
                target_object = target
            else:
                target_pose = target.copy()
                target_object = None

            # ground_pose = [[target_pose[0][0], target_pose[0][1], 0], target_pose[1]]
            ground_pose = Pose(target_pose.position_as_list())
            ground_pose.position.z = 0

            occupancy = OccupancyCostmap(0.32, False, 200, 0.02, ground_pose)
            final_map = occupancy

            test_robot = None
            if visible_for or reachable_for:
                robot_object = visible_for if visible_for else reachable_for
                test_robot = World.current_world.get_prospection_object_for_object(robot_object)

            if reachable_for:
                gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
                final_map += gaussian
            if visible_for:
                min_height = RobotDescription.current_robot_description.get_default_camera().minimal_height
                max_height = RobotDescription.current_robot_description.get_default_camera().maximal_height
                visible = VisibilityCostmap(min_height, max_height, 200, 0.02,
                                            Pose(target_pose.position_as_list()), target_object=target_object,
                                            robot=test_robot)
                final_map += visible

            ignore_collision_with = [World.current_world.get_prospection_object_for_object(o) for o in
                                     ignore_collision_with]
            with UseProspectionWorld():
                for maybe_pose in PoseGenerator(final_map, number_of_samples=600):
                    if check_collision_at_start and (test_robot is not None):
                        if prospect_robot_contact(test_robot, maybe_pose, ignore_collision_with):
                            continue
                    res = True
                    arms = None
                    found_grasps = []
                    if visible_for:
                        visible_prospection_object = World.current_world.get_prospection_object_for_object(
                            target)
                        res = res and visibility_validator(maybe_pose, test_robot, visible_prospection_object,
                                                           World.current_world)
                    if reachable_for:
                        hand_links = []

                        if reachable_arm:
                            arm_chain = RobotDescription.current_robot_description.get_arm_chain(reachable_arm)
                            hand_links += arm_chain.end_effector.links
                        else:
                            for chain in RobotDescription.current_robot_description.get_manipulator_chains():
                                hand_links += chain.end_effector.links
                        valid, arms = reachability_validator(maybe_pose, test_robot, target_pose,
                                                             allowed_collision={test_robot: hand_links})
                        if reachable_arm:
                            res = res and valid and reachable_arm in arms

                        else:
                            for description in RobotDescription.current_robot_description.get_manipulator_chains():
                                hand_links += description.end_effector.links
                        allowed_collision = {test_robot: hand_links}
                        allowed_collision.update({o: o.link_names for o in ignore_collision_with})
                        for grasp in grasps:
                            target_pose_oriented = target_pose.copy()
                            if grasp is not None:
                                grasp_quaternion = RobotDescription.current_robot_description.grasps[grasp]
                                target_pose_oriented.multiply_quaternion(grasp_quaternion)
                            valid, arms = reachability_validator(maybe_pose, test_robot, target_pose_oriented,
                                                                 allowed_collision=allowed_collision,
                                                                 arm=reachable_arm,
                                                                 prepose_distance=prepose_distance)
                            if reachable_arm:
                                res = res and valid and reachable_arm in arms
                            else:
                                res = res and valid
                            if res:
                                found_grasps.append(grasp)
                                break
                    if res:
                        yield maybe_pose


class AccessingLocation(LocationDesignatorDescription):
    """
    Location designator which describes poses used for opening drawers
    """

    def __init__(self, handle_desig: ObjectDescription.Link,
                 robot_desig: Object,
                 prepose_distance: float = ActionConfig.grasping_prepose_distance):
        """
        Describes a position from where a drawer can be opened. For now this position should be calculated before the
        drawer will be opened. Calculating the pose while the drawer is open could lead to problems.

        :param handle_desig: ObjectPart designator for handle of the drawer
        :param robot_desig: Object designator for the robot which should open the drawer
        :param prepose_distance: Distance to the target pose where the robot should be checked for reachability.
        """
        super().__init__()
        PartialDesignator.__init__(self, AccessingLocation, handle_desig=handle_desig, robot_desig=robot_desig,
                                   prepose_distance=prepose_distance)
        self.handle: ObjectDescription.Link = handle_desig
        self.robot: Object = robot_desig
        self.prepose_distance = prepose_distance

    def ground(self) -> Pose:
        """
        Default specialized_designators for this location designator, just returns the first element from the iteration

        :return: A location designator for a pose from which the drawer can be opened
        """
        return next(iter(self))

    @staticmethod
    def adjust_map_for_drawer_opening(cost_map: Costmap, init_pose: Pose, goal_pose: Pose,
                                      width: float = 0.2):
        """
        Adjust the cost map for opening a drawer. This is done by removing all locations between the initial and final
        pose of the drawer/container.

        :param cost_map: Costmap that should be adjusted.
        :param init_pose: Pose of the drawer/container when it is fully closed.
        :param goal_pose: Pose of the drawer/container when it is fully opened.
        :param width: Width of the drawer/container.
        """
        motion_vector = [goal_pose.position.x - init_pose.position.x, goal_pose.position.y - init_pose.position.y,
                         goal_pose.position.z - init_pose.position.z]
        # remove locations between the initial and final pose
        motion_vector_length = np.linalg.norm(motion_vector)
        unit_motion_vector = np.array(motion_vector) / motion_vector_length
        orthogonal_vector = np.array([unit_motion_vector[1], -unit_motion_vector[0], 0])
        orthogonal_vector /= np.linalg.norm(orthogonal_vector)
        orthogonal_size = width
        map_origin_idx = cost_map.map.shape[0] // 2, cost_map.map.shape[1] // 2
        for i in range(int(motion_vector_length / cost_map.resolution)):
            for j in range(int(orthogonal_size / cost_map.resolution)):
                idx = (int(map_origin_idx[0] + i * unit_motion_vector[0] + j * orthogonal_vector[0]),
                       int(map_origin_idx[1] + i * unit_motion_vector[1] + j * orthogonal_vector[1]))
                cost_map.map[idx] = 0
                idx = (int(map_origin_idx[0] + i * unit_motion_vector[0] - j * orthogonal_vector[0]),
                       int(map_origin_idx[1] + i * unit_motion_vector[1] - j * orthogonal_vector[1]))
                cost_map.map[idx] = 0

    def __iter__(self) -> Iterator[Pose]:
        """
        Creates poses from which the robot can open the drawer specified by the ObjectPart designator describing the
        handle. Poses are validated by checking if the robot can grasp the handle while the drawer is closed and if
        the handle can be grasped if the drawer is open.

        :yield: A location designator containing the pose and the arms that can be used.
        """
        for params in self.generate_permutations():
            handle = params["handle"]

            # Find a Joint that moves with the handle in the URDF tree
            container_joint = handle.parent_entity.find_joint_above_link(handle.name)

            init_pose = link_pose_for_joint_config(handle.parent_entity, {
                container_joint: handle.parent_entity.get_joint_limits(container_joint)[0]},
                                                   handle.name)

            # Calculate the pose the handle would be in if the drawer was to be fully opened
            goal_pose = link_pose_for_joint_config(handle.parent_entity, {
                container_joint: handle.parent_entity.get_joint_limits(container_joint)[1] - 0.05},
                                                   handle.name)

            # Handle position for calculating rotation of the final pose
            half_pose = link_pose_for_joint_config(handle.parent_entity, {
                container_joint: handle.parent_entity.get_joint_limits(container_joint)[1] / 1.5},
                                                   handle.name)

            ground_pose = Pose(handle.pose.position_as_list())
            ground_pose.position.z = 0
            occupancy = OccupancyCostmap(distance_to_obstacle=0.25, from_ros=False, size=200, resolution=0.02,
                                         origin=ground_pose)
            gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)

            final_map = occupancy + gaussian
            joint_type = handle.parent_entity.joints[container_joint].type
            if joint_type == JointType.PRISMATIC:
                self.adjust_map_for_drawer_opening(final_map, init_pose, goal_pose)

            test_robot = World.current_world.get_prospection_object_for_object(self.robot)

            with UseProspectionWorld():
                orientation_generator = lambda p, o: PoseGenerator.generate_orientation(p, half_pose)
                for maybe_pose in PoseGenerator(final_map, number_of_samples=600,
                                                orientation_generator=orientation_generator):
                    if prospect_robot_contact(test_robot, maybe_pose):
                        logdebug("Robot is initially in collision, skipping that pose")
                        continue
                    logdebug("Robot is initially in a valid pose")
                    hand_links = []
                    for description in RobotDescription.current_robot_description.get_manipulator_chains():
                        hand_links += description.end_effector.links

                    valid_init, arms_init = reachability_validator(maybe_pose, test_robot, init_pose,
                                                                   allowed_collision={test_robot: hand_links},
                                                                   prepose_distance=self.prepose_distance)

                    if valid_init:
                        logdebug(f"Found a valid init pose for accessing {handle.name} with arms {arms_init}")
                        valid_goal, arms_goal = reachability_validator(maybe_pose, test_robot, goal_pose,
                                                                       allowed_collision={test_robot: hand_links},
                                                                       prepose_distance=self.prepose_distance)

                        arms_list = list(set(arms_init).intersection(set(arms_goal)))

                        if valid_goal and len(arms_list) > 0:
                            logdebug(f"Found a valid goal pose for accessing {handle.name} with arms {arms_list}")
                            # yield self.Location(maybe_pose, arms_list)
                            yield maybe_pose


class SemanticCostmapLocation(LocationDesignatorDescription):
    """
    Locations over semantic entities, like a table surface
    """

    def __init__(self, link_name, part_of, for_object=None, edges_only: bool = False,
                 horizontal_edges_only: bool = False, edge_size_in_meters: float = 0.06):
        """
        Creates a distribution over a link to sample poses which are on this link. Can be used, for example, to find
        poses that are on a table. Optionally an object can be given for which poses should be calculated, in that case
        the poses are calculated such that the bottom of the object is on the link.

        :param link_name: Name of the link for which a distribution should be calculated
        :param part_of: Object of which the urdf link is a part
        :param for_object: Optional object that should be placed at the found location
        :param edges_only: If True, only the edges of the link are considered
        :param horizontal_edges_only: If True, only the horizontal edges of the link are considered
        :param edge_size_in_meters: Size of the edges in meters.
        """
        super().__init__()
        PartialDesignator.__init__(self, SemanticCostmapLocation, link_name=link_name, part_of=part_of,
                                   for_object=for_object, edges_only=edges_only,
                                   horizontal_edges_only=horizontal_edges_only, edge_size_in_meters=edge_size_in_meters)
        self.link_name: str = link_name
        self.part_of: Object = part_of
        self.for_object: Optional[Object] = for_object
        self.edges_only: bool = edges_only
        self.horizontal_edges_only: bool = horizontal_edges_only
        self.edge_size_in_meters: float = edge_size_in_meters
        self.sem_costmap: Optional[SemanticCostmap] = None

    def ground(self) -> Pose:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[Pose]:
        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: An instance of SemanticCostmapLocation.Location with the found valid position of the Costmap.
        """
        for params in self.generate_permutations():
            part_of = params["part_of"]
            link_name = params["link_name"]
            edges_only = params["edges_only"]
            horizontal_edges_only = params["horizontal_edges_only"]
            edge_size_in_meters = params["edge_size_in_meters"]
            for_object = params["for_object"]

            self.sem_costmap = SemanticCostmap(part_of, link_name)
            if edges_only or horizontal_edges_only:
                self.sem_costmap = self.sem_costmap.get_edges_map(edge_size_in_meters,
                                                                  horizontal_only=horizontal_edges_only)
            height_offset = 0
            if for_object:
                min_p, max_p = for_object.get_axis_aligned_bounding_box().get_min_max_points()
                height_offset = (max_p.z - min_p.z) / 2
            for maybe_pose in PoseGenerator(self.sem_costmap):
                maybe_pose.position.z += height_offset
                yield maybe_pose
