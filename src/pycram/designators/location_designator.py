import dataclasses
from functools import reduce

import plotly.graph_objects as go
import networkx as nx
import numpy as np
import tqdm
from box import Box
from probabilistic_model.distributions import DiracDeltaDistribution, GaussianDistribution
from probabilistic_model.distributions.helper import make_dirac
from probabilistic_model.probabilistic_circuit.nx.helper import uniform_measure_of_event, leaf
from probabilistic_model.probabilistic_circuit.nx.probabilistic_circuit import SumUnit, ProbabilisticCircuit, \
    ProductUnit
from random_events.interval import closed
from random_events.polytope import Polytope
from random_events.product_algebra import Event, SimpleEvent
from random_events.set import Set
from random_events.variable import Continuous, Symbolic
from scipy.spatial import ConvexHull
from sortedcontainers import SortedSet
from typing_extensions import List, Union, Iterable, Optional, Iterator, Dict

from ..config.action_conf import ActionConfig
from ..costmaps import OccupancyCostmap, VisibilityCostmap, SemanticCostmap, GaussianCostmap, Costmap
from ..datastructures.dataclasses import BoundingBox, AxisAlignedBoundingBox, Color
from ..datastructures.enums import JointType, Arms, Grasp
from ..datastructures.partial_designator import PartialDesignator
from ..datastructures.pose import PoseStamped, GraspDescription, GraspPose, Vector3
from ..datastructures.world import World, UseProspectionWorld
from ..designator import LocationDesignatorDescription
from ..failures import RobotInCollision
from ..graph_of_convex_sets import GraphOfConvexSets, plot_bounding_boxes_in_rviz
from ..local_transformer import LocalTransformer
from ..object_descriptors.urdf import ObjectDescription
from ..pose_generator_and_validator import PoseGenerator, visibility_validator, pose_sequence_reachability_validator, \
    collision_check, OrientationGenerator
from ..ros import logwarn
from ..ros_utils.viz_marker_publisher import plot_axis_in_rviz
from ..world_concepts.world_object import Object, Link
from ..world_reasoning import link_pose_for_joint_config


class Location(LocationDesignatorDescription):
    """
    Default location designator which only wraps a pose.
    """

    def __init__(self, pose: PoseStamped):
        """
        Basic location designator that represents a single pose.

        :param pose: The pose that should be represented by this location designator
        """
        super().__init__()
        self.pose: PoseStamped = pose

    def ground(self) -> PoseStamped:
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

    def __init__(self, target: Union[PoseStamped, Object],
                 reachable_for: Optional[Union[Iterable[Object], Object]] = None,
                 visible_for: Optional[Union[Iterable[Object], Object]] = None,
                 reachable_arm: Optional[Union[Iterable[Arms], Arms]] = None,
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
        :param ignore_collision_with: List of objects that should be ignored for collision checking.
        :param grasp_descriptions: List of grasps that should be tried to reach the target pose
        :param object_in_hand: Object that is currently in the hand of the robot
        """
        super().__init__()
        PartialDesignator.__init__(self, CostmapLocation, target=target, reachable_for=reachable_for,
                                   visible_for=visible_for,
                                   reachable_arm=reachable_arm if reachable_arm is not None else [
                                       Arms.LEFT, Arms.RIGHT],
                                   ignore_collision_with=ignore_collision_with if ignore_collision_with is not None else [
                                       []],
                                   grasp_descriptions=grasp_descriptions if grasp_descriptions is not None else [None],
                                   object_in_hand=object_in_hand)
        self.target: Union[PoseStamped, Object] = target
        self.reachable_for: Object = reachable_for
        self.visible_for: Object = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.ignore_collision_with = ignore_collision_with if ignore_collision_with is not None else [[]]
        self.grasps: List[Optional[Grasp]] = grasp_descriptions if grasp_descriptions is not None else [None]

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators which returns the first result from the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    @staticmethod
    def setup_costmaps(target: PoseStamped, robot: Object, visible_for, reachable_for) -> Costmap:
        """
        Sets up the costmaps for the given target and robot. The costmaps are merged and stored in the final_map


        """
        target_pose = PoseStamped.from_list([target.position.x, target.position.y, target.position.z],
                                  [target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w])
        ground_pose = PoseStamped.from_list(target_pose.position.to_list())
        ground_pose.position.z = 0

        occupancy = OccupancyCostmap(0.32, False, 200, 0.02, ground_pose)
        final_map = occupancy

        if visible_for:
            camera = robot.robot_description.get_default_camera()
            visible = VisibilityCostmap(camera.minimal_height, camera.maximal_height, 200, 0.02,
                                        PoseStamped.from_list(target_pose.position.to_list()), target_object=target, robot=robot)
            final_map += visible

        if reachable_for:
            gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
            final_map += gaussian

        return final_map

    @staticmethod
    def create_target_sequence(grasp_description: GraspDescription, target: Union[PoseStamped, Object], robot: Object,
                               object_in_hand: Object, reachable_arm: Arms) -> List[PoseStamped]:
        """
        Creates a sequence of poses which need to be reachable in this order

        :param grasp_description: Grasp description to be used for grasping
        :param target: The target of reachability, either a pose or an object
        :param robot: The robot that should be checked for reachability
        :param object_in_hand: An object that is held if any
        :param reachable_arm: The arm which should be checked for reachability
        :return: A list of poses that need to be reachable in this order
        """
        end_effector = robot.robot_description.get_arm_chain(reachable_arm).end_effector
        grasp_quaternion = end_effector.grasps[grasp_description]
        approach_axis = end_effector.get_approach_axis()
        if object_in_hand:
            current_target_pose = object_in_hand.attachments[
                World.robot].get_child_link_target_pose_given_parent(target)
            approach_offset_cm = object_in_hand.get_approach_offset()
        else:
            current_target_pose = target.copy() if isinstance(target, PoseStamped) else \
                target.get_grasp_pose(end_effector, grasp_description)
            current_target_pose.rotate_by_quaternion(grasp_quaternion)
            approach_offset_cm = 0.1 if isinstance(target, PoseStamped) else target.get_approach_offset()

        lift_pose = current_target_pose.copy()
        lift_pose.position.z += 0.1

        retract_pose = LocalTransformer().translate_pose_along_local_axis(current_target_pose,
                                                                          approach_axis,
                                                                          -approach_offset_cm)

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

    def __iter__(self) -> Iterator[PoseStamped]:
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
            target = params_box.target.copy() if isinstance(params_box.target, PoseStamped) else params_box.target

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
                                                                      params_box.object_in_hand,
                                                                      params_box.reachable_arm)

                        is_reachable = pose_sequence_reachability_validator(test_robot, target_sequence,
                                                                            arm=params_box.reachable_arm,
                                                                            allowed_collision=allowed_collision)
                        if is_reachable:
                            yield GraspPose(pose_candidate.pose, pose_candidate.header,
                                            arm=params_box.reachable_arm, grasp_description=grasp_desc)


class AccessingLocation(LocationDesignatorDescription):
    """
    Location designator which describes poses used for opening drawers
    """
    def __init__(self, handle: Union[ObjectDescription.Link, Iterable[ObjectDescription.Link]],
                 robot_desig: Union[Object, Iterable[Object]],
                 arm: Union[List[Arms], Arms] = None,
                 prepose_distance: float = ActionConfig.grasping_prepose_distance):
        """
        Describes a position from where a drawer can be opened. For now this position should be calculated before the
        drawer will be opened. Calculating the pose while the drawer is open could lead to problems.

        :param handle: ObjectPart designator for handle of the drawer
        :param robot_desig: Object designator for the robot which should open the drawer
        """
        super().__init__()
        PartialDesignator.__init__(self, AccessingLocation, handle=handle, robot_desig=robot_desig,
                                   arm=arm if arm is not None else [Arms.LEFT, Arms.RIGHT],
                                   prepose_distance=prepose_distance)
        self.handle: ObjectDescription.Link = handle
        self.robot: Object = robot_desig
        self.prepose_distance = prepose_distance
        self.arm = arm if arm is not None else [Arms.LEFT, Arms.RIGHT]

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators for this location designator, just returns the first element from the iteration

        :return: A location designator for a pose from which the drawer can be opened
        """
        return next(iter(self))

    @staticmethod
    def adjust_map_for_drawer_opening(cost_map: Costmap, init_pose: PoseStamped, goal_pose: PoseStamped,
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
        ground_pose = PoseStamped.from_list(handle.pose.position.to_list())
        ground_pose.position.z = 0
        occupancy = OccupancyCostmap(distance_to_obstacle=0.25, from_ros=False, size=200, resolution=0.02,
                                     origin=ground_pose)
        final_map = occupancy

        gaussian = GaussianCostmap(200, 15, 0.02, ground_pose)
        final_map += gaussian

        return final_map

    def create_target_sequence(self, params_box: Box, final_map: Costmap) -> List[PoseStamped]:
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

    def __iter__(self) -> Iterator[PoseStamped]:
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

            test_robot = World.current_world.get_prospection_object_for_object(params_box.robot_desig)

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
                 horizontal_edges_only: bool = False, edge_size_in_meters: float = 0.06, height_offset: float = 0.0):
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
                                   horizontal_edges_only=horizontal_edges_only, edge_size_in_meters=edge_size_in_meters,
                                   height_offset=height_offset)
        self.link_name: str = link_name
        self.part_of: Object = part_of
        self.for_object: Optional[Object] = for_object
        self.edges_only: bool = edges_only
        self.horizontal_edges_only: bool = horizontal_edges_only
        self.edge_size_in_meters: float = edge_size_in_meters
        self.sem_costmap: Optional[SemanticCostmap] = None

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[PoseStamped]:
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
            height_offset = params_box.height_offset
            if params_box.for_object:
                min_p, max_p = params_box.for_object.get_axis_aligned_bounding_box().get_min_max_points()
                height_offset = (max_p.z - min_p.z) / 2
            for maybe_pose in PoseGenerator(self.sem_costmap):
                maybe_pose.position.z += height_offset
                yield maybe_pose

class ProbabilisticSemanticLocation(LocationDesignatorDescription):
    """
    Location designator which samples poses from a semantic costmap with a probability distribution.
    """

    def __init__(self, link_names, part_of, for_object=None, edges_only: bool = False,
                 horizontal_edges_only: bool = False, edge_size_in_meters: float = 0.06, height_offset: float = 0.0,
                 link_is_center_link: bool = False, with_2d_obstacles: bool = False):
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
        PartialDesignator.__init__(self, ProbabilisticSemanticLocation, link_names=[link_names], part_of=part_of,
                                   for_object=for_object, edges_only=edges_only,
                                   horizontal_edges_only=horizontal_edges_only, edge_size_in_meters=edge_size_in_meters,
                                   height_offset=height_offset)
        self.link_names: list = link_names
        self.link_is_center_link: bool = link_is_center_link
        self.part_of: Object = part_of
        self.for_object: Optional[Object] = for_object
        self.edges_only: bool = edges_only
        self.horizontal_edges_only: bool = horizontal_edges_only
        self.edge_size_in_meters: float = edge_size_in_meters
        self.with_2d_obstacles: bool = with_2d_obstacles

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

    def __iter__(self) -> Iterator[PoseStamped]:

        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: An instance of ProbabilisticSemanticLocation.Location with the found valid position of the Costmap.
        """
        for params in self.generate_permutations():

            params_box = Box(params)
            test_robot = World.current_world.get_prospection_object_for_object(World.current_world.robot)

            links: List[Link] = [params_box.part_of.links[link_name] for link_name in params_box.link_names]
            link_bbs = [params_box.part_of.get_link_bounding_box_collection(link.name) for link in links]

            old_colors = [params_box.part_of.get_link_color(link).get_rgb() for link in params_box.link_names]
            [params_box.part_of.set_link_color(link, [1., 0., 1.]) for link in params_box.link_names]

            # merge bb collections
            merged_bb = reduce(lambda a, b: a.merge(b) or a, link_bbs)

            link_ids = [link.id for link in links]
            world = World.current_world

            link_id_symbol = Symbolic("link", Set.from_iterable(link_ids))
            target_x = Continuous('target_x')
            target_y = Continuous('target_y')

            final_discribution = SumUnit(ProbabilisticCircuit())

            for link in links:

                variables = [BoundingBox.x_variable, BoundingBox.y_variable]

                link_searchspace = link.get_axis_aligned_bounding_box().bloat(1, 1, 1).as_collection()

                link_width = link.get_axis_aligned_bounding_box().width
                link_depth = link.get_axis_aligned_bounding_box().depth

                link_min_dim = link_width if link_width < link_depth else link_depth

                wall_bloat = 0.3 if link_min_dim > 0.65 else link_min_dim / 2.5

                if self.with_2d_obstacles:
                    free_space = GraphOfConvexSets.navigation_map_from_world(world, search_space=link_searchspace)
                else:
                    free_space = GraphOfConvexSets.free_space_from_world(world, search_space=link_searchspace, bloat_obstacles=0.02, bloat_walls=wall_bloat)

                stand_on_ground = SimpleEvent({BoundingBox.z_variable: (0, 0.05)}).as_composite_set()
                stand_on_ground.fill_missing_variables(variables)

                event = params_box.part_of.get_link_bounding_box_collection(link.name).event

                target_node = free_space.node_of_pose(link.pose.position.x, link.pose.position.y,
                                                      link.pose.position.z + (
                                                                  link.get_axis_aligned_bounding_box().height / 2) + 0.1)
                sub_gcs: GraphOfConvexSets = nx.subgraph(free_space, nx.shortest_path(free_space, target_node))
                condition = Event(*[node.simple_event for node in sub_gcs.nodes])

                condition.fill_missing_variables([target_x, target_y])
                walls = GraphOfConvexSets.get_doors_and_walls_of_world(world, search_space=link_searchspace, bloat_walls=wall_bloat)

                if walls is not None:
                    # fig = go.Figure(walls.plot(color="red"))
                    # fig.add_traces(event.plot(color="blue"))
                    # fig.add_traces((event - walls).plot(color="green"))
                    # fig.show()
                    event = ~walls & event

                if event.is_empty():
                    continue

                event = event.marginal(variables)
                condition &= stand_on_ground
                condition = condition.marginal(variables)

                condition = ~event & condition

                p_target = uniform_measure_of_event(event)

                surface_samples = p_target.sample(100)
                p_target.log_likelihood(surface_samples)

                prob_circuit = ProbabilisticCircuit()
                circuit_root = SumUnit(prob_circuit)
                scale = 1.

                for surface_sample in surface_samples[:100]:
                    p_point_root = ProductUnit(prob_circuit)
                    circuit_root.add_subcircuit(p_point_root, 1.)
                    target_x_p = make_dirac(target_x, surface_sample[0])
                    target_y_p = make_dirac(target_y, surface_sample[1])
                    target_link_id_p = make_dirac(link_id_symbol, link.id)

                    x_p = GaussianDistribution(BoundingBox.x_variable, surface_sample[0], scale)
                    y_p = GaussianDistribution(BoundingBox.y_variable, surface_sample[1], scale)

                    p_point_root.add_subcircuit(leaf(target_x_p, prob_circuit))
                    p_point_root.add_subcircuit(leaf(target_y_p, prob_circuit))
                    p_point_root.add_subcircuit(leaf(target_link_id_p, prob_circuit))
                    p_point_root.add_subcircuit(leaf(x_p, prob_circuit))
                    p_point_root.add_subcircuit(leaf(y_p, prob_circuit))

                prob_circuit.normalize()

                event.fill_missing_variables(prob_circuit.variables)

                prob_circuit_conditioned, p_condition = prob_circuit.truncated(condition)

                if prob_circuit_conditioned is None:
                    continue

                # final_discribution.add_subcircuit(prob_circuit_conditioned.root, np.log(p_condition))
                # alles gleich
                final_discribution.add_subcircuit(prob_circuit_conditioned.root, np.log(1.))

            final_discribution.probabilistic_circuit.normalize()

            # go.Figure(final_discribution.probabilistic_circuit.marginal(variables).plot()).show()

            samples = final_discribution.probabilistic_circuit.sample(1005)
            np.random.shuffle(samples)
            # samples = samples[np.argsort(final_discribution.probabilistic_circuit.log_likelihood(samples))[::-1]]

            [params_box.part_of.set_link_color(link, old_color) for link, old_color in zip(params_box.link_names, old_colors)]

            # with UseProspectionWorld():
            # TODO: Use Prospection World here in the future, but currently there is annoying context management, causing
            #       subsequent actions to be executed in the Prospection World, causing problems with detecting etc
            for sample in samples:
                sample_dict = {var.name: val for var, val in zip(final_discribution.probabilistic_circuit.variables, sample)}
                link_id, target_x, target_y, nav_x, nav_y = sample

                ray_start = [target_x, target_y, 2]
                ray_end = [target_x, target_y, - 2.0]
                result = test_robot.world.ray_test(ray_start, ray_end)
                hit_body = result.link_id
                hit_pos = result.hit_position[2]

                if hit_body != link_id:
                    continue

                target_quat = OrientationGenerator.generate_random_orientation()
                target_pose = PoseStamped.from_list([target_x, target_y, hit_pos], target_quat, 'map')

                nav_quat = OrientationGenerator.generate_origin_orientation([nav_x, nav_y], target_pose)
                nav_pose = PoseStamped.from_list([nav_x, nav_y, 0], nav_quat, 'map')
                # print('Semantic Costmap Location')
                # plot_axis_in_rviz([target_pose, nav_pose], duration=300)

                test_robot.set_pose(nav_pose)
                try:
                    collision_check(test_robot, {})
                except RobotInCollision:
                    continue

                if params_box.for_object:
                    min_p, max_p = params_box.for_object.get_axis_aligned_bounding_box().get_min_max_points()
                    final_height_offset = (max_p.z - min_p.z) / 2
                    target_pose.position.z += final_height_offset

                yield target_pose

@dataclasses.dataclass
class ProbabilisticCostmapLocation(LocationDesignatorDescription):
    """
    Uses Costmaps to create locations for complex constrains
    """

    def __init__(self, target: Union[PoseStamped, Object],
                 reachable_for: Optional[Union[Iterable[Object], Object]] = None,
                 visible_for: Optional[Union[Iterable[Object], Object]] = None,
                 reachable_arm: Optional[Union[Iterable[Arms], Arms]] = None,
                 ignore_collision_with: Optional[Union[Iterable[Object], Object]] = None,
                 grasp_descriptions: Optional[Union[Iterable[GraspDescription], GraspDescription]] = None,
                 object_in_hand: Optional[Union[Iterable[Object], Object]] = None,
                 rotation_agnostic: bool = False):
        """
        Location designator that uses costmaps as base to calculate locations for complex constrains like reachable or
        visible. In case of reachable the resolved location contains a list of arms with which the location is reachable.

        :param target: Location for which visibility or reachability should be calculated
        :param reachable_for: Object for which the reachability should be calculated, usually a robot
        :param visible_for: Object for which the visibility should be calculated, usually a robot
        :param reachable_arm: An optional arm with which the target should be reached
        :param ignore_collision_with: List of objects that should be ignored for collision checking.
        :param grasp_descriptions: List of grasps that should be tried to reach the target pose
        :param object_in_hand: Object that is currently in the hand of the robot
        :param rotation_agnostic: If True, the target pose is adjusted so that it is pointing to the robot
        """
        super().__init__()
        PartialDesignator.__init__(self, ProbabilisticCostmapLocation, target=target, reachable_for=reachable_for,
                                   visible_for=visible_for,
                                   reachable_arm=reachable_arm if reachable_arm is not None else [
                                       Arms.LEFT, Arms.RIGHT],
                                   ignore_collision_with=ignore_collision_with if ignore_collision_with is not None else [
                                       []],
                                   grasp_descriptions=grasp_descriptions if grasp_descriptions is not None else [None],
                                   object_in_hand=object_in_hand, rotation_agnostic=rotation_agnostic)
        self.target: Union[PoseStamped, Object] = target
        self.reachable_for: Object = reachable_for
        self.visible_for: Object = visible_for
        self.reachable_arm: Optional[Arms] = reachable_arm
        self.ignore_collision_with = ignore_collision_with if ignore_collision_with is not None else [[]]
        self.grasps: List[Optional[Grasp]] = grasp_descriptions if grasp_descriptions is not None else [None]

    def ground(self) -> PoseStamped:
        """
        Default specialized_designators which returns the first element of the iterator of this instance.

        :return: A resolved location
        """
        return next(iter(self))

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

    @staticmethod
    def create_target_sequence(grasp_description: GraspDescription, target: Union[PoseStamped, Object], robot: Object,
                               object_in_hand: Object, reachable_arm: Arms, rotation_agnostic: bool) -> List[PoseStamped]:
        """
        Creates a sequence of poses which need to be reachable in this order

        :param grasp_description: Grasp description to be used for grasping
        :param target: The target of reachability, either a pose or an object
        :param robot: The robot that should be checked for reachability
        :param object_in_hand: An object that is held if any
        :param reachable_arm: The arm which should be checked for reachability
        :return: A list of poses that need to be reachable in this order
        """
        end_effector = robot.robot_description.get_arm_chain(reachable_arm).end_effector
        grasp_quaternion = end_effector.grasps[grasp_description]
        approach_axis = end_effector.get_approach_axis()

        target_pose = target.copy() if isinstance(target, PoseStamped) else \
            target.get_grasp_pose(end_effector, grasp_description)



        if object_in_hand:
            if rotation_agnostic:
                robot_rotation = robot.get_pose().orientation
                target_pose.orientation = robot_rotation
                approach_direction = GraspDescription(grasp_description.approach_direction, None, False)
                side_grasp = robot.robot_description.get_arm_chain(reachable_arm).end_effector.grasps[
                    approach_direction]
                side_grasp = [(-x, -y, -z, w) for x, y, z, w in zip(*[iter(side_grasp)]*4)]
                side_grasp = [elem for quat in side_grasp for elem in quat]
                target_pose.rotate_by_quaternion(side_grasp)

            target_pose = object_in_hand.attachments[
                World.robot].get_child_link_target_pose_given_parent(target_pose)
            approach_offset_cm = object_in_hand.get_approach_offset()
        else:

            target_pose.rotate_by_quaternion(grasp_quaternion)
            approach_offset_cm = 0.1 if isinstance(target, PoseStamped) else target.get_approach_offset()

        lift_pose = target_pose.copy()
        lift_pose.position.z += 0.1

        retract_pose = LocalTransformer().translate_pose_along_local_axis(target_pose,
                                                                          approach_axis,
                                                                          -approach_offset_cm)

        target_sequence = ([lift_pose, target_pose, retract_pose] if object_in_hand
                           else [retract_pose, target_pose, lift_pose])

        return target_sequence

    def __iter__(self) -> Iterator[PoseStamped]:

        """
        Creates a costmap on top of a link of an Object and creates positions from it. If there is a specific Object for
        which the position should be found, a height offset will be calculated which ensures that the bottom of the Object
        is at the position in the Costmap and not the origin of the Object which is usually in the centre of the Object.

        :yield: An instance of ProbabilisticSemanticLocation.Location with the found valid position of the Costmap.
        """
        for params in self.generate_permutations():
            params_box = Box(params)

            target = params_box.target.copy() if isinstance(params_box.target, PoseStamped) else params_box.target

            if params_box.visible_for or params_box.reachable_for:
                robot_object = params_box.visible_for if params_box.visible_for else params_box.reachable_for
                test_robot = World.current_world.get_prospection_object_for_object(robot_object)
            else:
                test_robot = World.current_world.robot

            # links: List[Link] = [params_box.part_of.links[link_name] for link_name in params_box.link_names]
            # link_ids = [link.id for link in links]
            world = World.current_world

            allowed_collision = self.create_allowed_collisions(params_box.ignore_collision_with,
                                                               params_box.object_in_hand)

            target_pose: PoseStamped = target.copy() if isinstance(target, PoseStamped) else target.get_pose()

            target_pos: Vector3 = target_pose.position

            search_distance = 1.5
            link_searchspace = AxisAlignedBoundingBox(target_pos.x - search_distance, target_pos.y - search_distance, 0,
                                                      target_pos.x + search_distance, target_pos.y + search_distance, target_pos.z + 0.35).as_collection()

            free_space_nav = GraphOfConvexSets.navigation_map_from_world(world, search_space=link_searchspace, bloat_obstacles=0.3)

            free_space = GraphOfConvexSets.free_space_from_world(world, search_space=link_searchspace, bloat_obstacles=0.01, bloat_walls=.2)

            target_node = free_space.node_of_pose(target_pose.position.x, target_pose.position.y, target_pose.position.z + 0.2)
            sub_gcs: GraphOfConvexSets = nx.subgraph(free_space, nx.shortest_path(free_space, target_node))

            condition = Event(*[node.simple_event for node in sub_gcs.nodes])
            condition2 = Event(*[node.simple_event for node in free_space_nav.nodes])

            variables = [BoundingBox.x_variable, BoundingBox.y_variable]

            stand_on_ground = SimpleEvent({BoundingBox.z_variable: (0, 0.05)}).as_composite_set()
            stand_on_ground.fill_missing_variables(variables)

            condition &= stand_on_ground

            rays_end = [[node.origin[0], node.origin[1], target_pos.z + 0.2] for node in sub_gcs.nodes]
            rays_start = [[target_pos.x, target_pos.y, target_pos.z + 0.2]] * len(rays_end)

            robot = World.robot
            robot_pose = robot.get_pose()
            robot.set_pose(PoseStamped.from_list([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z+100]))
            test = world.ray_test_batch(rays_start, rays_end)
            robot.set_pose(robot_pose)

            hit_positions = [[result.hit_position[0], result.hit_position[1]] for result in test]
            successful_hits = [
                [hx, hy] if (hx, hy) != (0.0, 0.0) else [rx, ry]
                for (hx, hy), (rx, ry, _) in zip(hit_positions, rays_end)
            ]
            hull = ConvexHull(successful_hits)
            A = hull.equations[:, :-1]
            b = -hull.equations[:, -1]

            # reason for this is the error
            # "E0000 00:00:1750253132.517928 1146896 linear_solver.cc:2026] No solution exists. MPSolverInterface::result_status_ = MPSOLVER_ABNORMAL"
            # this is not a great way to handle this i think, so we should find out in which situations this occurs @TODO
            try:
                room_event = Polytope(A, b).inner_box_approximation(minimum_volume=0.1)
            except RuntimeError as e:
                print('Polytope: ', Polytope(A, b))
                room_event = Polytope(A, b).to_simple_event().as_composite_set()
            room_event = room_event.update_variables({Continuous("x_0"): BoundingBox.x_variable,
                                                      Continuous("x_1"): BoundingBox.y_variable})
            room_event.fill_missing_variables(SortedSet([BoundingBox.z_variable]))

            condition &= room_event

            condition &= condition2

            target_x = Continuous('target_x')
            target_y = Continuous('target_y')

            condition.fill_missing_variables([target_x, target_y])

            prob_circuit = ProbabilisticCircuit()
            circuit_root = SumUnit(prob_circuit)

            scale = 1.

            p_point_root = ProductUnit(prob_circuit)
            circuit_root.add_subcircuit(p_point_root, 1.)
            target_x_p = DiracDeltaDistribution(target_x, target_pos.x, 1.)
            target_y_p = DiracDeltaDistribution(target_y, target_pos.y, 1.)

            x_p = GaussianDistribution(BoundingBox.x_variable, target_pos.x, scale)
            y_p = GaussianDistribution(BoundingBox.y_variable, target_pos.y, scale)

            p_point_root.add_subcircuit(leaf(target_x_p, prob_circuit))
            p_point_root.add_subcircuit(leaf(target_y_p, prob_circuit))
            p_point_root.add_subcircuit(leaf(x_p, prob_circuit))
            p_point_root.add_subcircuit(leaf(y_p, prob_circuit))

            prob_circuit.normalize()

            prob_circuit_conditioned, p_condition = prob_circuit.truncated(condition)

            if prob_circuit_conditioned is None:
                continue

            #dump to json
            # with open('prob_circuit_conditioned.json', 'w') as f:
            #     json.dump(prob_circuit_conditioned.to_json(), f, indent=4)

            # go.Figure(prob_circuit_conditioned.marginal(variables).plot()).show()

            with UseProspectionWorld():
                samples = prob_circuit_conditioned.sample(300)

                event: Optional[SimpleEvent] = None
                samples = samples[np.argsort(prob_circuit_conditioned.log_likelihood(samples))[::-1]]

                while len(samples) > 0:
                    _, _, nav_x, nav_y = samples[0]
                    if event:
                        samples = np.array([s for s in samples if not event.contains((s[2], s[3]))])

                    nav_quat = OrientationGenerator.generate_origin_orientation([nav_x, nav_y], target_pose)
                    pose_candidate = PoseStamped.from_list([nav_x, nav_y, 0], nav_quat, 'map')

                    test_robot.set_pose(pose_candidate)
                    # print('Costmap Location')
                    # plot_axis_in_rviz([pose_candidate, target_pose], duration=300)

                    event = SimpleEvent({BoundingBox.x_variable: closed(nav_x - 0.1, nav_x + 0.1),
                                         BoundingBox.y_variable: closed(nav_y - 0.1, nav_y + 0.1)})
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
                                                                      params_box.object_in_hand,
                                                                      params_box.reachable_arm,
                                                                      params_box.rotation_agnostic)
                        is_reachable = pose_sequence_reachability_validator(test_robot, target_sequence,
                                                                            arm=params_box.reachable_arm,
                                                                            allowed_collision=allowed_collision)
                        if is_reachable:
                            # print(f'Accepted target sequence: {target_sequence}')
                            yield GraspPose(pose_candidate.pose, pose_candidate.header,
                                            arm=params_box.reachable_arm, grasp_description=grasp_desc)