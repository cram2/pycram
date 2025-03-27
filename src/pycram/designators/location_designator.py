import dataclasses

import numpy as np
from box import Box
from typing_extensions import List, Union, Iterable, Optional, Callable, Iterator, Sequence, Dict

from .object_designator import ObjectPart
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap, Costmap
from ..datastructures.enums import JointType, Arms, Grasp
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.pose import Pose, GraspDescription, GraspPose
from ..datastructures.world import World, UseProspectionWorld
from ..designator import DesignatorError, LocationDesignatorDescription, ObjectDesignatorDescription
from ..failures import RobotInCollision
from ..local_transformer import LocalTransformer
from ..object_descriptors.urdf import ObjectDescription
from ..pose_generator_and_validator import PoseGenerator, visibility_validator, pose_sequence_reachability_validator, \
    collision_check
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


@dataclasses.dataclass
class CostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains
    """

    def __init__(self, target: Union[Pose, Object],
                 reachable_for: Optional[Union[Iterable[Object], Object]] = None,
                 visible_for: Optional[Union[Iterable[Object], Object]] = None,
                 reachable_arm: Optional[Union[Iterable[Arms], Arms]] = None,
                 prepose_distance: Union[Union[Iterable[float], float]] = ActionConfig.pick_up_prepose_distance,
                 ignore_collision_with: Optional[Union[Iterable[Object], Object]] = None,
                 grasp_descriptions: Optional[Union[Iterable[GraspDescription], GraspDescription]] = None,
                 object_in_hand: Optional[Union[Iterable[Object], Object]] = None):
        """
        Location designator that uses costmaps as base to calculate locations for complex constrains like reachable or
        visible. In case of reachable the resolved location contains a list of arms with which the location is reachable.

        :param target: Location for which visibility or reachability should be calculated
        :param reachable_for: Object for which the reachability should be calculated, usually a robot
        :param visible_for: Object for which the visibility should be calculated, usually a robot
        :param reachable_arm: An optional arm with which the target should be reached
        :param prepose_distance: Distance to the target pose where the robot should be checked for reachability.
        :param ignore_collision_with: List of objects that should be ignored for collision checking.
        :param grasp_descriptions: List of grasps that should be tried to reach the target pose
        :param object_in_hand: Object that is currently in the hand of the robot
        """
        super().__init__()
        PartialDesignator.__init__(self, CostmapLocation, target=target, reachable_for=reachable_for,
                                   visible_for=visible_for,
                                   reachable_arm=reachable_arm if reachable_arm is not None else [
                                       Arms.LEFT, Arms.RIGHT],
                                   prepose_distance=prepose_distance,
                                   ignore_collision_with=ignore_collision_with if ignore_collision_with is not None else [
                                       []],
                                   grasp_descriptions=grasp_descriptions if grasp_descriptions is not None else [None],
                                   object_in_hand=object_in_hand)
        self.target: Union[Pose, Object] = target
        self.reachable_for: Object = reachable_for
        self.visible_for: Object = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.prepose_distance = prepose_distance
        self.ignore_collision_with = ignore_collision_with if ignore_collision_with is not None else [[]]
        self.grasps: List[Optional[Grasp]] = grasp_descriptions if grasp_descriptions is not None else [None]

    def ground(self) -> Pose:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    @staticmethod
    def setup_costmaps(target: Pose, robot: Object, visible_for, reachable_for) -> Costmap:
        """
        Sets up the costmaps for the given target and robot. The costmaps are merged and stored in the final_map


        """
        target_pose = Pose([target.position.x, target.position.y, target.position.z],
                           [target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
        ground_pose = Pose(target_pose.position_as_list())
        ground_pose.position.z = 0

        occupancy = OccupancyCostmap(0.32, False, 200, 0.02, ground_pose)
        final_map = occupancy

        if visible_for:
            camera = robot.robot_description.get_default_camera()
            visible = VisibilityCostmap(camera.minimal_height, camera.maximal_height, 200, 0.02,
                                        Pose(target_pose.position_as_list()), target_object=target, robot=robot)
            final_map += visible

        if reachable_for:
            gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
            final_map += gaussian

        return final_map

    @staticmethod
    def create_target_sequence(grasp_description: GraspDescription, target: Union[Pose, Object], robot: Object,
                               prepose_distance: float, object_in_hand: Object, reachable_arm: Arms) -> \
            List[Pose]:
        """
        Creates a sequence of poses which need to be reachable in this order

        :param grasp_description: Grasp description to be used for grasping
        :param target: The target of reachability, either a pose or an object
        :param robot: The robot that should be checked for reachability
        :param prepose_distance: Distance between the first pose in the sequence and the second
        :param object_in_hand: An object that is held if any
        :param reachable_arm: The arm which should be checked for reachability
        :return: A list of poses that need to be reachable in this order
        """
        end_effector = robot.robot_description.get_arm_chain(reachable_arm).end_effector
        if object_in_hand:
            current_target_pose = object_in_hand.attachments[
                World.robot].get_child_link_target_pose_given_parent(target)
        else:
            current_target_pose = target.copy() if isinstance(target,
                                                              Pose) else target.get_pose().copy()
            current_target_pose.rotate_by_quaternion(end_effector.grasps[grasp_description])

        lift_pose = current_target_pose.copy()
        lift_pose.position.z += 0.1

        retract_pose = current_target_pose.copy()
        retract_pose.position.x -= prepose_distance
        retract_pose = LocalTransformer().transform_pose(retract_pose, "map")

        target_sequence = ([lift_pose, current_target_pose, retract_pose] if object_in_hand
                           else [retract_pose, current_target_pose, lift_pose])

        return target_sequence

    @staticmethod
    def create_allowed_collisions(ignore_collision_with: List[Object], object_in_hand: Object) -> Dict[Object, str]:
        """
        Creates a dict of object which are allowed to collide with the robot without impacting reachability

        :param ignore_collision_with: List of objects for which collision should be ignored
        :param object_in_hand: An object that the robot might hold
        :return: A dict of objects that link to their names
        """
        ignore_collision_with = [World.current_world.get_prospection_object_for_object(obj)
                                 for obj in ignore_collision_with]

        allowed_collision = {object: object.link_id_to_name[-1] for object in ignore_collision_with}
        if object_in_hand:
            prospection_object = World.current_world.get_prospection_object_for_object(object_in_hand)
            allowed_collision.update({prospection_object: prospection_object.link_id_to_name[-1]})
        return allowed_collision

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
            params_box = Box(params)
            # Target is either a pose or an object since the object is later needed for the visibility validator
            target = params_box.target.copy() if isinstance(params_box.target, Pose) else params_box.target

            if params_box.visible_for or params_box.reachable_for:
                robot_object = params_box.visible_for if params_box.visible_for else params_box.reachable_for
                test_robot = World.current_world.get_prospection_object_for_object(robot_object)
            else:
                test_robot = World.current_world.robot

            allowed_collision = self.create_allowed_collisions(params_box.ignore_collision_with,
                                                               params_box.object_in_hand)

            final_map = self.setup_costmaps(target, test_robot, params_box.visible_for, params_box.reachable_for)

            with UseProspectionWorld():
                for pose_candidate in PoseGenerator(final_map, number_of_samples=600):
                    pose_candidate.position.z = 0
                    test_robot.set_pose(pose_candidate)
                    try:
                        collision_check(test_robot, allowed_collision)
                    except RobotInCollision:
                        continue

                    if not (params_box.reachable_for or params_box.visible_for):
                        yield pose_candidate
                        continue

                    if params_box.visible_for and not visibility_validator(test_robot, target):
                        continue

                    if not params_box.reachable_for:
                        yield pose_candidate
                        continue

                    grasp_descriptions = [
                        params_box.grasp_descriptions] if params_box.grasp_descriptions else target.calculate_grasp_descriptions(
                        test_robot)

                    for grasp_desc in grasp_descriptions:

                        target_sequence = self.create_target_sequence(grasp_desc, target, test_robot,
                                                                      params_box.prepose_distance,
                                                                      params_box.object_in_hand,
                                                                      params_box.reachable_arm)

                        is_reachable = pose_sequence_reachability_validator(test_robot, target_sequence,
                                                                            arm=params_box.reachable_arm,
                                                                            allowed_collision=allowed_collision)
                        if is_reachable:
                            yield GraspPose(pose_candidate.position_as_list(), pose_candidate.orientation_as_list(),
                                            arm=params_box.reachable_arm, grasp_description=grasp_desc)


class AccessingLocation(LocationDesignatorDescription):
    """
    Location designator which describes poses used for opening drawers
    """

    def __init__(self, handle_desig: Union[ObjectDescription.Link, Iterable[ObjectDescription.Link]],
                 robot_desig: Union[Object, Iterable[Object]],
                 arm: Union[List[Arms], Arms] = None,
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
                                   arm=arm if arm is not None else [Arms.LEFT, Arms.RIGHT],
                                   prepose_distance=prepose_distance)
        self.handle: ObjectDescription.Link = handle_desig
        self.robot: Object = robot_desig
        self.prepose_distance = prepose_distance
        self.arm = arm if arm is not None else [Arms.LEFT, Arms.RIGHT]

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

    def setup_costmaps(self, handle: Link) -> Costmap:
        """
        Sets up the costmaps for the given handle and robot. The costmaps are merged and stored in the final_map.
        """
        ground_pose = Pose(handle.pose.position_as_list())
        ground_pose.position.z = 0
        occupancy = OccupancyCostmap(distance_to_obstacle=0.25, from_ros=False, size=200, resolution=0.02,
                                     origin=ground_pose)
        final_map = occupancy

        gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
        final_map += gaussian

        return final_map

    def create_target_sequence(self, params_box: Box, final_map: Costmap) -> List[Pose]:
        """
        Creates the sequence of target poses

        :param params_box:
        :param final_map:
        :return:
        """
        # Find a Joint that moves with the handle in the URDF tree
        container_joint = params_box.handle.parent_entity.find_joint_above_link(params_box.handle.name)

        init_pose = link_pose_for_joint_config(params_box.handle.parent_entity, {
            container_joint: params_box.handle.parent_entity.get_joint_limits(container_joint)[0]},
                                               params_box.handle.name)

        # Calculate the pose the handle would be in if the drawer was to be fully opened
        goal_pose = link_pose_for_joint_config(params_box.handle.parent_entity, {
            container_joint: params_box.handle.parent_entity.get_joint_limits(container_joint)[1] - 0.05},
                                               params_box.handle.name)

        # Handle position for calculating rotation of the final pose
        half_pose = link_pose_for_joint_config(params_box.handle.parent_entity, {
            container_joint: params_box.handle.parent_entity.get_joint_limits(container_joint)[1] / 1.5},
                                               params_box.handle.name)

        joint_type = params_box.handle.parent_entity.joints[container_joint].type

        if joint_type == JointType.PRISMATIC:
            self.adjust_map_for_drawer_opening(final_map, init_pose, goal_pose)

        target_sequence = [init_pose, half_pose, goal_pose]
        return target_sequence

    def __iter__(self) -> Iterator[Pose]:
        """
        Creates poses from which the robot can open the drawer specified by the ObjectPart designator describing the
        handle. Poses are validated by checking if the robot can grasp the handle while the drawer is closed and if
        the handle can be grasped if the drawer is open.

        :yield: A location designator containing the pose and the arms that can be used.
        """
        for params in self.generate_permutations():
            params_box = Box(params)

            final_map = self.setup_costmaps(params_box.handle)

            target_sequence = self.create_target_sequence(params_box, final_map)
            half_pose = target_sequence[1]

            test_robot = World.current_world.get_prospection_object_for_object(params_box.robot)

            with UseProspectionWorld():
                orientation_generator = lambda p, o: PoseGenerator.generate_orientation(p, half_pose)
                for pose_candidate in PoseGenerator(final_map, number_of_samples=600,
                                                    orientation_generator=orientation_generator):
                    pose_candidate.position.z = 0
                    test_robot.set_pose(pose_candidate)
                    try:
                        collision_check(test_robot, {})
                    except RobotInCollision:
                        continue

                    for arm_chain in test_robot.robot_description.get_manipulator_chains():
                        grasp = arm_chain.end_effector.grasps[GraspDescription(Grasp.FRONT, None, False)]
                        current_target_sequence = [pose.copy() for pose in target_sequence]
                        [pose.rotate_by_quaternion(grasp) for pose in current_target_sequence]

                        is_reachable = pose_sequence_reachability_validator(test_robot, current_target_sequence,
                                                                            arm=arm_chain.arm_type)
                        if is_reachable:
                            yield pose_candidate


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
            params_box = Box(params)

            self.sem_costmap = SemanticCostmap(params_box.part_of, params_box.link_name)
            if params_box.edges_only or params_box.horizontal_edges_only:
                self.sem_costmap = self.sem_costmap.get_edges_map(params_box.edge_size_in_meters,
                                                                  horizontal_only=params_box.horizontal_edges_only)
            height_offset = 0
            if params_box.for_object:
                min_p, max_p = params_box.for_object.get_axis_aligned_bounding_box().get_min_max_points()
                height_offset = (max_p.z - min_p.z) / 2
            for maybe_pose in PoseGenerator(self.sem_costmap):
                maybe_pose.position.z += height_offset
                yield maybe_pose
